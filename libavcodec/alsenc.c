/**
 * MPEG-4 ALS audio encoder
 * Copyright (c) 2008 Justin Ruggles
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file alsenc.c
 * @author Justin Ruggles
 * MPEG-4 Audio Lossless Coding (mp4als) encoder.
 * All specification references are to ISO/IEC 14496-3 3rd edition, Amd. 2
 */

#include "avcodec.h"
#include "dsputil.h"
#include "put_bits.h"
#include "flacenc.h"
#include "golomb.h"
#include "mpeg4audio.h"
#include "alsdata.h"

#define LPC_USE_DOUBLE
#include "lpc.h"

#define MAX_CH         UINT16_MAX
#define MIN_BLOCKSIZE  2
#define MAX_BLOCKSIZE  UINT16_MAX
#define MAX_SAMPLERATE 16777215
#define MAX_EC_BLOCKS  8

#define ALS_SPECIFIC_CFG_SIZE  30
#define ALS_EXTRADATA_MAX_SIZE (6 + ALS_SPECIFIC_CFG_SIZE)

enum {
    ADAPT_METHOD_NONE,
    ADAPT_METHOD_LOG_SEARCH,
    ADAPT_METHOD_FULL_SEARCH,
};

typedef struct EntropyContext {
    int ec_blocks;                      ///< number of entropy coding blocks
    int rice_params[MAX_EC_BLOCKS];     ///< rice parameter for each ec block
} EntropyContext;

typedef struct AlsBlock {
    int constant_block;                     ///< indicates constant block
    int js_block;                           ///< indicates difference signal
    int lpc_order;                          ///< prediction order
    int q_parcor_coeffs[MAX_LPC_ORDER+1];   ///< quantized 7-bit parcor coeffs
    int parcor_coeffs[MAX_LPC_ORDER+1];     ///< reconstructed 20-bit parcor coeffs
    int lpc_coeffs[MAX_LPC_ORDER+1];        ///< 20-bit lpc coeffs
    EntropyContext ec;                      ///< entropy partitioning and parameters
    int32_t *sample_buf;                    ///< sample buffer, including previous samples
    int32_t *samples;                       ///< original sample values (pointer to start of current frame in sample_buf)
    int32_t *residual;                      ///< residual signal after prediction
} AlsBlock;

typedef struct AlsFrame {
    int frame_length;   ///< frame length, in samples
    AlsBlock *blocks;   ///< 1 block for each channel
} AlsFrame;

typedef struct AlsEncodeContext {
    int sample_rate;
    uint32_t sample_count;
    int channels;
    int bps;
    int random_access;
    int full_frame_length;
    int adapt_lpc_order;
    int order_bits;
    int parcor_table_number;
    int max_lpc_order;
    int ec_part;
    int joint_stereo;

    AlsFrame frame;

    PutBitContext pb;
    AVCodecContext *avctx;
    DSPContext dsp;
} AlsEncodeContext;

static int write_extradata(AlsEncodeContext *s)
{
    PutBitContext pb;
    uint8_t header[ALS_EXTRADATA_MAX_SIZE+FF_INPUT_BUFFER_PADDING_SIZE];
    int i, length;

    memset(header, 0, sizeof(header));
    init_put_bits(&pb, header, ALS_EXTRADATA_MAX_SIZE);

    /* AudioSpecificConfig */

    /* object type */
    put_bits(&pb, 5, 0x1F);
    put_bits(&pb, 6, AOT_ALS-32);

    /* sample rate */
    /* reference encoder always encodes sample rate explicitly */
    /*for (i = 0; i < 12; i++)
        if (s->sample_rate == ff_mpeg4audio_sample_rates[i])
            break;
    if (i < 12) {
        put_bits(&pb, 4, i);
    } else {*/
        put_bits(&pb, 4, 0xF);
        put_bits(&pb, 24, s->sample_rate);
    //}

    /* channel config */
    put_bits(&pb, 4, 0); // 0 = defined in DecoderSpecificConfig

    /* ALSSpecificConfig */
    flush_put_bits(&pb);
    put_bits(&pb, 32, MKBETAG('A','L','S','\0'));   // file id
    put_bits(&pb, 32, s->sample_rate);
    put_bits(&pb, 32, s->sample_count);
    put_bits(&pb, 16, s->channels-1);
    put_bits(&pb,  3, 1);                  // orig. file type (1=WAVE)
    put_bits(&pb,  3, (s->bps/8)-1);
    put_bits(&pb,  1, 0);                  // sample type (0=integer)
    put_bits(&pb,  1, 0);                  // msb-first (arbitrary since it's unknown)
    put_bits(&pb, 16, s->full_frame_length-1);
    put_bits(&pb,  8, s->random_access);   // indicate use of random access
    put_bits(&pb,  2, s->random_access);   // write rau sizes in frames
    put_bits(&pb,  1, !!s->adapt_lpc_order);
    put_bits(&pb,  2, s->parcor_table_number);
    put_bits(&pb,  1, 0);                  // ltp (0=off)
    put_bits(&pb, 10, s->max_lpc_order);
    put_bits(&pb,  2, 0);                  // block switching (0=off)
    put_bits(&pb,  1, 0);                  // entropy mode (0=Rice)
    put_bits(&pb,  1, s->ec_part);         // entropy partitioning (0=no partitioning)
    put_bits(&pb,  1, s->joint_stereo);
    put_bits(&pb,  1, 0);                  // multi-channel coding (0=off)
    put_bits(&pb,  1, 0);                  // channel config (0=not present)
    put_bits(&pb,  1, 0);                  // channel sorting (0=not present)
    put_bits(&pb,  1, 0);                  // crc (0=not present)
    put_bits(&pb,  1, 0);                  // RLS-LMS prediction (0=off)
    put_bits(&pb,  5, 0);                  // (reserved)
    put_bits(&pb,  1, 0);                  // aux data (0=not present)
    flush_put_bits(&pb);
    put_bits(&pb, 32, 0);                  // original header size
    put_bits(&pb, 32, 0);                  // original trailer size

    /* allocate and set extradata */
    s->avctx->extradata_size = 0;
    if (s->avctx->extradata) {
        av_freep(&s->avctx->extradata);
    }
    length = put_bits_count(&pb) / 8;
    s->avctx->extradata = av_mallocz(length + FF_INPUT_BUFFER_PADDING_SIZE);
    if (!s->avctx->extradata)
        return AVERROR(EIO);
    memcpy(s->avctx->extradata, header, length);
    s->avctx->extradata_size = length;

    return 0;
}

static av_cold int als_encode_init(AVCodecContext *avctx)
{
    AlsEncodeContext *s = avctx->priv_data;
    int ch;

    s->avctx = avctx;
    dsputil_init(&s->dsp, avctx);

    /* sample format (only signed 16-bit, for now) */
    if (avctx->sample_fmt != SAMPLE_FMT_S16) {
        av_log(avctx, AV_LOG_ERROR, "ALS encoder only supports 16-bit samples\n");
        return -1;
    }
    s->bps = 16;

    /* channels */
    if (avctx->channels < 1 || avctx->channels > MAX_CH) {
        av_log(avctx, AV_LOG_ERROR, "invalid number of channels: %d\n", avctx->channels);
        return -1;
    }
    s->channels = avctx->channels;

    /* sample rate */
    if (avctx->sample_rate < 1 || avctx->sample_rate > MAX_SAMPLERATE) {
        av_log(avctx, AV_LOG_ERROR, "invalid sample rate: %d\n", avctx->sample_rate);
        return -1;
    }
    s->sample_rate = avctx->sample_rate;

    /* frame length (fixed, for now) */
    s->avctx->frame_size = s->full_frame_length = 1024;
    s->frame.frame_length = s->full_frame_length;
    s->sample_count = 0xFFFFFFFF; // unknown

    /* TODO: user selectable */
    s->random_access = 0;

    /* LPC prediction settings */
    s->max_lpc_order = 12;
    s->adapt_lpc_order = ADAPT_METHOD_FULL_SEARCH;
    s->parcor_table_number = 0;
    if (s->adapt_lpc_order) {
        int log_order = av_log2((s->max_lpc_order     << 1) - 1);
        int log_n     = av_log2((s->full_frame_length >> 2) - 1);
        s->order_bits = av_clip(log_n, 1, log_order);
    }

    s->ec_part = 1;
    s->joint_stereo = 0;

    /* allocate extradata and write MP4 AudioSpecificConfig */
    if (write_extradata(s)) {
        av_log(avctx, AV_LOG_ERROR, "error allocating extradata\n");
        return -1;
    }
    s->sample_count = 0;

    /* allocate blocks */
    s->frame.blocks = av_mallocz(sizeof(AlsBlock)*s->channels);
    for (ch = 0; ch < s->channels; ch++) {
        s->frame.blocks[ch].sample_buf = av_mallocz((MAX_LPC_ORDER+s->full_frame_length)*sizeof(int32_t));
        s->frame.blocks[ch].samples = &s->frame.blocks[ch].sample_buf[MAX_LPC_ORDER];
        s->frame.blocks[ch].residual= av_mallocz((s->full_frame_length+1)*sizeof(int32_t));
    }

    avctx->coded_frame = avcodec_alloc_frame();
    avctx->coded_frame->key_frame = 1;

    return 0;
}

static int compute_rice_params(AlsBlock *blk, int n, int ec_part)
{
    FlacRiceContext rc;
    int i, bits=0;

    blk->ec.ec_blocks = 1;
    if (!ec_part || (n == 4) || (n & 3)) {
        bits = ff_flac_calc_rice_params(&rc, 0, 0, blk->residual, n, 0);
    } else {
        bits = ff_flac_calc_rice_params(&rc, 2, 2, blk->residual, n, 0);
        if (rc.params[0] == rc.params[1] && rc.params[1] == rc.params[2] &&
            rc.params[2] == rc.params[3]) {
            rc.porder = 0;
        }
        if (rc.porder == 2)
            blk->ec.ec_blocks = 4;
    }
    for (i = 0; i < blk->ec.ec_blocks; i++) {
        blk->ec.rice_params[i] = rc.params[i];
    }

    return bits;
}

/**
 * Calculate the parcor coefficients from autocorrelation data.
 */
static void compute_parcor_coeffs(const double *autoc, int max_order,
                                  double *parcor)
{
    int i;
    double lpc[MAX_LPC_ORDER][MAX_LPC_ORDER];

    compute_lpc_coefs(autoc, max_order, (double *)lpc, MAX_LPC_ORDER, 0, 1);
    for (i = 0; i < max_order; i++)
        parcor[i] = lpc[i][i];
}

/**
 * Quantize PARCOR coefficients.
 * The specification defines how PARCOR quantization and reconstruction should
 * be done. Some liberties can be taken with the quantization, but the
 * reconstruction must bit-exact since it is mirrored in the decoder.
 */
static void quantize_parcor_coeffs(const double *parcor, int order,
                                   int *q_parcor, int *r_parcor)
{
    int i;

    q_parcor[1] = lrint(64.0 * (M_SQRT2 * sqrt(parcor[0]+1.0) - 1.0));
    q_parcor[1] = av_clip(q_parcor[1], -64, 63);
    r_parcor[1] = 32 + ((q_parcor[1]+64)*(q_parcor[1]+65) << 7) - (1 << 20);
    if (order > 1) {
        q_parcor[2] = lrint(64.0 * (M_SQRT2 * sqrt(1.0-parcor[1]) - 1.0));
        q_parcor[2] = av_clip(q_parcor[2], -64, 63);
        r_parcor[2] = -32 - ((q_parcor[2]+64)*(q_parcor[2]+65) << 7) + (1 << 20);
    }
    for (i = 3; i <= order; i++) {
        q_parcor[i] = lrint(64.0 * parcor[i-1]);
        q_parcor[i] = av_clip(q_parcor[i], -64, 63);
        r_parcor[i] = (q_parcor[i] << 14) + (1 << 13);
    }
}

#define LPC_PREDICT_SAMPLE(lpc, smp_ptr, res_ptr, order)\
{\
    int64_t y = 1 << 19;\
    for (j = 1; j <= order; j++)\
        y += (int64_t)lpc[j] * (smp_ptr)[-j];\
    y = *(smp_ptr) + (y >> 20);\
    if (y < INT32_MIN || y > INT32_MAX)\
        return -1;\
    *(res_ptr) = y;\
}

/**
 * Calculate residual signal
 */
static int encode_residual(AlsBlock *blk, int n, int ra)
{
    int64_t p;
    int i, j, order, start;

    memset(blk->lpc_coeffs, 0, sizeof(blk->lpc_coeffs));
    order = ra ? FFMIN(n, blk->lpc_order) : blk->lpc_order;
    start = ra ? order : 0;

    for (i = 1; i <= order; i++) {
        if (ra) {
            LPC_PREDICT_SAMPLE(blk->lpc_coeffs, &blk->samples[i-1],
                               &blk->residual[i-1], i-1);
        }
        /* convert parcor coeffs to lpc coeffs */
        p = blk->parcor_coeffs[i];
        for (j = 1; j <= (i >> 1); j++) {
            int tmp = blk->lpc_coeffs[j] +  (((p * blk->lpc_coeffs[i-j]) + (1 << 19)) >> 20);
            blk->lpc_coeffs[i-j]         += (((p * blk->lpc_coeffs[j  ]) + (1 << 19)) >> 20);
            blk->lpc_coeffs[j] = tmp;
        }
        blk->lpc_coeffs[i] = blk->parcor_coeffs[i];
    }
    for (i = start; i < n; i++) {
        LPC_PREDICT_SAMPLE(blk->lpc_coeffs, &blk->samples[i],
                           &blk->residual[i], blk->lpc_order);
    }

    return 0;
}

static void encode_joint_stereo(AlsEncodeContext *s)
{
    s->frame.blocks[0].js_block = 0;
    s->frame.blocks[1].js_block = 0;
#if 0
    int32_t *left, *right;
    int i, mode, n, max_k;

    left  = s->frame.blocks[0].samples;
    right = s->frame.blocks[1].samples;
    n = s->frame.frame_length;
    max_k = s->bps <= 16 ? 15 : 31;

    mode = FLAC_CHMODE_INDEPENDENT;

    s->frame.blocks[0].js_block = (mode == FLAC_CHMODE_RIGHT_SIDE);
    s->frame.blocks[1].js_block = (mode == FLAC_CHMODE_LEFT_SIDE);

    if (mode != FLAC_CHMODE_INDEPENDENT) {
        int32_t *dest = (mode == FLAC_CHMODE_LEFT_SIDE) ? right : left;
        for (i = -MAX_LPC_ORDER; i < n; i++)
            dest[i] = right[i] - left[i];
    }
#endif
}

static int encode_residual_single_order(AlsBlock *blk, int n, int ra,
                                        int ec_part)
{
    double parcor[MAX_LPC_ORDER];

    if (encode_residual(blk, n, ra)) {
        /* prediction overflow, recalculate without prediction */
        memset(parcor, 0, sizeof(parcor));
        quantize_parcor_coeffs(parcor, blk->lpc_order, blk->q_parcor_coeffs,
                               blk->parcor_coeffs);
        if (encode_residual(blk, n, ra))
            return -1;
    }
    return compute_rice_params(blk, n, ec_part);
}

static int encode_residual_adaptive_lpc(AlsBlock *blk, int max_order, int n,
                                        int ra, int ec_part, int adapt)
{
    int i, opt_order;
    int bits[MAX_LPC_ORDER+1];

    if (adapt == ADAPT_METHOD_NONE) {
        blk->lpc_order = max_order;
        if (encode_residual_single_order(blk, n, ra, ec_part) < 0)
            return -1;
        return 0;
    }

    opt_order = 0;
    if (adapt == ADAPT_METHOD_LOG_SEARCH) {
        int step;
        for (i = 0; i < MAX_LPC_ORDER+1; i++)
            bits[i] = INT32_MAX;
        opt_order = max_order - (1 << av_log2(max_order>>1)) - 1;
        for (step = 16 ; step; step>>=1) {
            int last= opt_order;
            for (i = last-step; i <= last+step; i += step) {
                if (i < 0 || i >= max_order || bits[i] < INT32_MAX)
                    continue;
                blk->lpc_order = i+1;
                bits[i] = encode_residual_single_order(blk, n, ra, ec_part);
                if (bits[i] < 0)
                    return -1;
                bits[i] += blk->lpc_order * 11 / 2; // estimate parcor coeff bits
                if (bits[i] < bits[opt_order])
                    opt_order= i;
            }
        }
        opt_order++;
    } else if (adapt == ADAPT_METHOD_FULL_SEARCH) {
        opt_order = 0;
        bits[0] = INT32_MAX;
        for (i = 0; i <= max_order; i++) {
            blk->lpc_order = i;
            bits[i] = encode_residual_single_order(blk, n, ra, ec_part);
            if (bits[i] < 0)
                return -1;
            bits[i] += blk->lpc_order * 11 / 2; // estimate parcor coeff bits
            if (bits[i] < bits[opt_order])
                opt_order = i;
        }
    }

    if(blk->lpc_order != opt_order) {
        blk->lpc_order = opt_order;
        encode_residual_single_order(blk, n, ra, ec_part);
    }

    return 0;
}

static int encode_frame(AlsEncodeContext *s)
{
    double autoc[MAX_LPC_ORDER+1];
    double parcor[MAX_LPC_ORDER];
    AlsBlock *blk;
    int ch, n, i;
    int v;

    n = s->frame.frame_length;

    if (s->channels == 2 && s->joint_stereo) {
        encode_joint_stereo(s);
    } else {
        for (ch = 0; ch < s->channels; ch++)
            s->frame.blocks[ch].js_block = 0;
    }

    for (ch = 0; ch < s->channels; ch++) {
        blk = &s->frame.blocks[ch];

        /* check for constant and zero modes */
        blk->constant_block = 1;
        v = blk->samples[0];
        for (i = 1; i < n; i++) {
            if (blk->samples[i] != v)
                break;
        }
        if (i < n || v < INT16_MIN || v > INT16_MAX)
            blk->constant_block = 0;

        /* forward-adaptive (LPC) prediction */
        if (blk->constant_block) {
            blk->residual[0] = v;
        } else if (s->max_lpc_order) {
            if (!s->random_access && n < s->max_lpc_order) {
                s->dsp.flac_compute_autocorr(blk->samples-s->max_lpc_order,
                                             n+s->max_lpc_order,
                                             s->max_lpc_order, autoc);
            } else {
                s->dsp.flac_compute_autocorr(blk->samples, n, s->max_lpc_order,
                                             autoc);
            }
            compute_parcor_coeffs(autoc, s->max_lpc_order, parcor);
            quantize_parcor_coeffs(parcor, s->max_lpc_order,
                                   blk->q_parcor_coeffs, blk->parcor_coeffs);
            if (encode_residual_adaptive_lpc(blk, s->max_lpc_order, n,
                    s->random_access, s->ec_part, s->adapt_lpc_order)) {
                return -1;
            }
        } else {
            /* order=0, no prediction. just copy samples verbatim */
            memcpy(blk->residual, blk->samples, n*sizeof(int32_t));
            compute_rice_params(blk, n, s->ec_part);
        }
    }

    return 0;
}

/* shift samples to start of buffer for prediction in next frame */
static void shift_sample_buffer(AlsEncodeContext *s)
{
    AlsBlock *blk;
    int32_t *left, *right;
    int ch;
    int n = s->frame.frame_length;

    if (s->random_access || !s->max_lpc_order)
        return;

    left  = s->frame.blocks[0].samples;
    right = s->frame.blocks[1].samples;

    for (ch = 0; ch < s->channels; ch++) {
        blk = &s->frame.blocks[ch];

        /* reverse joint stereo coding */
        if (blk->js_block) {
            int i;
            if (ch == 0) {
                for (i = n-MAX_LPC_ORDER; i < n; i++)
                    left[i] = right[i] - left[i];
            } else if (ch == 1) {
                for (i = n-MAX_LPC_ORDER; i < n; i++)
                    right[i] += left[i];
            }
        }
        memmove(blk->sample_buf, blk->sample_buf+n, MAX_LPC_ORDER*sizeof(int32_t));
    }
}

static inline int put_bits_check_overflow(PutBitContext *pb)
{
    if (put_bits_ptr(pb) > pb->buf_end-4)
        return 1;
    return 0;
}

static inline void put_bits_safe(PutBitContext *pb, int n, unsigned int value)
{
    if (put_bits_check_overflow(pb))
        return;
    put_bits(pb, n, value);
}

static inline void set_sr_golomb_als(PutBitContext *pb, int v, int k)
{
    int q;

    /* remap to unsigned */
    v = -2*v-1;
    v ^= (v>>31);

    /* write quotient in zero-terminated unary */
    q = (v >> k) + 1;
    while (q > 31) {
        put_bits_safe(pb, 31, 0x7FFFFFFF);
        q -= 31;
    }
    put_bits_safe(pb, q, ((1<<q)-1)^1);

    /* write remainder using k bits */
    if (k)
        put_bits_safe(pb, k, (v >> 1) - (((v >> k)-(!(v&1))) << (k-1)));
}

static void output_block(AlsEncodeContext *s, int ch)
{
    PutBitContext *pb = &s->pb;
    AlsBlock *blk = &s->frame.blocks[ch];
    int i, k, order;
    int n = s->frame.frame_length;

    if (put_bits_check_overflow(&s->pb))
        return;

    put_bits(pb, 1, 1); // block type (1=normal)
    put_bits(pb, 1, blk->js_block); // joint stereo block (difference signal)

    /* write the rice code for the block */
    if (s->ec_part)
        put_bits(pb, 1, blk->ec.ec_blocks>1);
    put_bits(pb, 4+(s->bps>16), blk->ec.rice_params[0]);
    for (i = 1; i < blk->ec.ec_blocks; i++) {
        set_sr_golomb_als(pb, blk->ec.rice_params[i]-blk->ec.rice_params[i-1], 0);
    }

    put_bits_safe(pb, 1, 0); // shift lsb's

    /* quantized, rice encoded parcor coeffs */
    if (s->adapt_lpc_order) {
        put_bits_safe(pb, s->order_bits, blk->lpc_order);
    }
    if (s->parcor_table_number == 3) {
        for (order = 1; order <= blk->lpc_order; order++)
            put_bits_safe(pb, 7, blk->q_parcor_coeffs[order]+64);
    } else {
        const int8_t *offset = parcor_rice_table[s->parcor_table_number][0];
        const int8_t *rp  = parcor_rice_table[s->parcor_table_number][1];
        for (order = 0; order < blk->lpc_order; order++) {
            i = (order <= 19) ? order : 20 + (order & 1) + (order > 126);
            set_sr_golomb_als(pb, blk->q_parcor_coeffs[order+1]-offset[i], rp[i]);
        }
    }

    /* entropy coded residual */
    k = blk->ec.rice_params[0];
    i = 0;
    if (s->random_access && blk->lpc_order) {
        set_sr_golomb_als(pb, blk->residual[i++], s->bps-4);
        if (blk->lpc_order > 1 && n > 1) {
            set_sr_golomb_als(pb, blk->residual[i++], k+3);
        }
        if (blk->lpc_order > 2 && n > 2) {
            set_sr_golomb_als(pb, blk->residual[i++], k+1);
        }
    }
    if (blk->ec.ec_blocks > 1) {
        int eb;
        n >>= 2;
        for (eb = 0; eb < blk->ec.ec_blocks; eb++) {
            k = blk->ec.rice_params[eb];
            while (i < (eb+1)*n)
                set_sr_golomb_als(pb, blk->residual[i++], k);
        }
    } else {
        for (; i < n; i++) {
            set_sr_golomb_als(pb, blk->residual[i], k);
        }
    }

    if (put_bits_check_overflow(&s->pb))
        return;

    flush_put_bits(&s->pb);
}

static void output_block_constant(AlsEncodeContext *s, int ch)
{
    PutBitContext *pb = &s->pb;
    AlsBlock *blk = &s->frame.blocks[ch];
    int nonzero;

    if (put_bits_check_overflow(&s->pb))
        return;

    put_bits(pb, 1, 0); // block type (0=constant)

    nonzero = !!blk->residual[0];
    put_bits(pb, 1, nonzero);   // constant value (non-zero)
    put_bits(pb, 1, blk->js_block); // joint stereo block (difference signal)

    put_bits(pb, 5, 0);         // reserved
    if (nonzero) {
        put_bits(pb, 16, blk->residual[0] & 0xFFFF);
    }

    if (put_bits_check_overflow(&s->pb))
        return;

    flush_put_bits(&s->pb);
}

static void output_frame(AlsEncodeContext *s)
{
    int ch;

    /* random access unit size (to be filled-in later) */
    if (s->random_access) {
        put_bits(&s->pb, 32, 0);
    }

    for (ch = 0; ch < s->channels; ch++) {
        if (s->frame.blocks[ch].constant_block)
            output_block_constant(s, ch);
        else
            output_block(s, ch);
    }
}

/**
 * Copy channel-interleaved input samples into separate blocks
 */
static void copy_samples(AlsEncodeContext *s, int16_t *samples)
{
    int i, j, ch;
    AlsFrame *f;

    f = &s->frame;
    for (i = 0, j = 0; i < f->frame_length; i++)
        for (ch = 0; ch < s->channels; ch++, j++)
            f->blocks[ch].samples[i] = samples[j];
}

static int als_encode_frame(AVCodecContext *avctx, uint8_t *frame,
                            int buf_size, void *data)
{
    AlsEncodeContext *s;
    int16_t *samples = data;
    int out_bytes;

    s = avctx->priv_data;

    if (avctx->frame_size < 1) {
        av_log(avctx, AV_LOG_ERROR, "invalid frame size: %d\n", avctx->frame_size);
        return -1;
    }

    /* re-write extradata after encoding is finished */
    if (!data) {
        if (write_extradata(s)) {
            av_log(avctx, AV_LOG_ERROR, "error allocating extradata\n");
            return -1;
        }
        return 0;
    }

    /* check frame size */
    if (avctx->frame_size > s->full_frame_length) {
        av_log(avctx, AV_LOG_ERROR, "frame size cannot be larger than specified in header\n");
        return -1;
    }
    /* TODO: don't allow for furthur encoding after a smaller frame */
    s->frame.frame_length = avctx->frame_size;

    copy_samples(s, samples);

    if (encode_frame(s)) {
        av_log(avctx, AV_LOG_ERROR, "error: LPC prediction overflow\n");
        return -1;
    }

    init_put_bits(&s->pb, frame, buf_size);
    output_frame(s);

    /* check for truncated frame */
    if (put_bits_check_overflow(&s->pb)) {
        av_log(avctx, AV_LOG_ERROR, "error: frame size too large. increase buffer size.\n");
        return -1;
    }

    /* copy samples to start of buffer for prediction in next frame */
    shift_sample_buffer(s);

    /* write frame (rau) size at start of frame */
    out_bytes = put_bits_count(&s->pb) / 8;
    if (s->random_access) {
        AV_WB32(frame, out_bytes);
    }

    s->sample_count += s->frame.frame_length;
    return out_bytes;
}

static av_cold int als_encode_close(AVCodecContext *avctx)
{
    AlsEncodeContext *s = avctx->priv_data;
    if (s) {
        int ch;
        for (ch = 0; ch < s->channels; ch++) {
            av_freep(&s->frame.blocks[ch].sample_buf);
            av_freep(&s->frame.blocks[ch].residual);
        }
        av_freep(&s->frame.blocks);
    }
    av_freep(&avctx->extradata);
    avctx->extradata_size = 0;
    av_freep(&avctx->coded_frame);
    return 0;
}

AVCodec als_encoder = {
    "als",
    CODEC_TYPE_AUDIO,
    CODEC_ID_MP4ALS,
    sizeof(AlsEncodeContext),
    als_encode_init,
    als_encode_frame,
    als_encode_close,
    NULL,
    .capabilities = CODEC_CAP_DELAY | CODEC_CAP_SMALL_LAST_FRAME,
    .long_name = NULL_IF_CONFIG_SMALL("MPEG-4 ALS (Audio Lossless Coding)"),
};
