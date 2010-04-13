/*
 * MPEG-4 ALS decoder
 * Copyright (c) 2009 Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
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
 * @file libavcodec/alsdec.c
 * MPEG-4 ALS decoder
 * @author Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
 */


#define DEBUG


#include "als.h"
#include "als_data.h"

#include "avcodec.h"
#include "get_bits.h"
#include "unary.h"
#include "mpeg4audio.h"
#include "bytestream.h"
#include "bgmc.h"
#include "libavutil/crc.h"

#include <stdint.h>

/** Inter-channel weighting factors for multi-channel correlation.
 *  To be indexed by the Rice coded indices.
 */
static const int16_t mcc_weightings[] = {
    204,  192,  179,  166,  153,  140,  128,  115,
    102,   89,   76,   64,   51,   38,   25,   12,
      0,  -12,  -25,  -38,  -51,  -64,  -76,  -89,
   -102, -115, -128, -140, -153, -166, -179, -192
};


typedef struct {
    int stop_flag;
    int master_channel;
    int time_diff_flag;
    int time_diff_sign;
    int time_diff_index;
    int weighting[6];
} ALSChannelData;


typedef struct {
    AVCodecContext *avctx;
    ALSSpecificConfig sconf;
    GetBitContext gb;
    const AVCRC *crc_table;
    uint32_t crc_org;               ///< CRC value of the original input data
    uint32_t crc;                   ///< CRC value calculated from decoded data
    unsigned int cur_frame_length;  ///< length of the current frame to decode
    unsigned int frame_id;          ///< the frame ID / number of the current frame
    unsigned int js_switch;         ///< if true, joint-stereo decoding is enforced
    unsigned int num_blocks;        ///< number of blocks used in the current frame
    unsigned int s_max;             ///< maximum Rice parameter allowed in entropy coding
    uint8_t *bgmc_lut;              ///< pointer at lookup tables used for BGMC
    unsigned int *bgmc_lut_status;  ///< pointer at lookup table status flags used for BGMC
    int ltp_lag_length;             ///< number of bits used for ltp lag value
    int *use_ltp;                   ///< contains use_ltp flags for all channels
    int *ltp_lag;                   ///< contains ltp lag values for all channels
    int **ltp_gain;                 ///< gain values for ltp 5-tap filter for a channel
    int *ltp_gain_buffer;           ///< contains all gain values for ltp 5-tap filter
    int32_t **quant_cof;            ///< quantized parcor coefficients for a channel
    int32_t *quant_cof_buffer;      ///< contains all quantized parcor coefficients
    int32_t **lpc_cof;              ///< coefficients of the direct form prediction filter for a channel
    int32_t *lpc_cof_buffer;        ///< contains all coefficients of the direct form prediction filter
    int32_t *lpc_cof_reversed_buffer; ///< temporary buffer to set up a reversed versio of lpc_cof_buffer
    ALSChannelData **chan_data;     ///< channel data for multi-channel correlation
    ALSChannelData *chan_data_buffer; ///< contains channel data for all channels
    int *reverted_channels;         ///< stores a flag for each reverted channel
    int32_t *prev_raw_samples;      ///< contains unshifted raw samples from the previous block
    int32_t **raw_samples;          ///< decoded raw samples for each channel
    int32_t *raw_buffer;            ///< contains all decoded raw samples including carryover samples
    uint8_t *crc_buffer;            ///< buffer of byte order corrected samples used for CRC check
} ALSDecContext;


typedef struct {
    unsigned int block_length;      ///< number of samples within the block
    unsigned int ra_block;          ///< if true, this is a random access block
    int          const_block;       ///< if true, this is a constant value block
    int32_t      const_val;         ///< the sample value of a constant block
    int          js_blocks;         ///< true if this block contains a difference signal
    unsigned int shift_lsbs;        ///< shift of values for this block
    unsigned int opt_order;         ///< prediction order of this block
    int          store_prev_samples;///< if true, carryover samples have to be stored
    int          *use_ltp;          ///< if true, long-term prediction is used
    int          *ltp_lag;          ///< lag value for long-term prediction
    int          *ltp_gain;         ///< gain values for ltp 5-tap filter
    int32_t      *quant_cof;        ///< quantized parcor coefficients
    int32_t      *lpc_cof;          ///< coefficients of the direct form prediction
    int32_t      *raw_samples;      ///< decoded raw samples / residuals for this block
    int32_t      *prev_raw_samples; ///< contains unshifted raw samples from the previous block
    int32_t      *raw_other;        ///< decoded raw samples of the other channel of a channel pair
} ALSBlockData;


/** Reads an ALSSpecificConfig from a buffer into the output struct.
 */
static av_cold int read_specific_config(ALSDecContext *ctx)
{
    GetBitContext gb;
    uint64_t ht_size;
    int i, config_offset;
    MPEG4AudioConfig m4ac;
    ALSSpecificConfig *sconf = &ctx->sconf;
    AVCodecContext *avctx    = ctx->avctx;
    uint32_t als_id, header_size, trailer_size;

    init_get_bits(&gb, avctx->extradata, avctx->extradata_size * 8);

    config_offset = ff_mpeg4audio_get_config(&m4ac, avctx->extradata,
                                             avctx->extradata_size);

    if (config_offset < 0)
        return -1;

    skip_bits_long(&gb, config_offset);

    if (get_bits_left(&gb) < (30 << 3))
        return -1;

    // read the fixed items
    als_id                      = get_bits_long(&gb, 32);
    avctx->sample_rate          = m4ac.sample_rate;
    skip_bits_long(&gb, 32); // sample rate already known
    sconf->samples              = get_bits_long(&gb, 32);
    avctx->channels             = m4ac.channels;
    skip_bits(&gb, 16);      // number of channels already knwon
    skip_bits(&gb, 3);       // skip file_type
    sconf->resolution           = get_bits(&gb, 3);
    sconf->floating             = get_bits1(&gb);
    sconf->msb_first            = get_bits1(&gb);
    sconf->frame_length         = get_bits(&gb, 16) + 1;
    sconf->ra_distance          = get_bits(&gb, 8);
    sconf->ra_flag              = get_bits(&gb, 2);
    sconf->adapt_order          = get_bits1(&gb);
    sconf->coef_table           = get_bits(&gb, 2);
    sconf->long_term_prediction = get_bits1(&gb);
    sconf->max_order            = get_bits(&gb, 10);
    sconf->block_switching      = get_bits(&gb, 2);
    sconf->bgmc                 = get_bits1(&gb);
    sconf->sb_part              = get_bits1(&gb);
    sconf->joint_stereo         = get_bits1(&gb);
    sconf->mc_coding            = get_bits1(&gb);
    sconf->chan_config          = get_bits1(&gb);
    sconf->chan_sort            = get_bits1(&gb);
    sconf->crc_enabled          = get_bits1(&gb);
    sconf->rlslms               = get_bits1(&gb);
    skip_bits(&gb, 5);       // skip 5 reserved bits
    skip_bits1(&gb);         // skip aux_data_enabled


    // check for ALSSpecificConfig struct
    if (als_id != MKBETAG('A','L','S','\0'))
        return -1;

    ctx->cur_frame_length = sconf->frame_length;

    // read channel config
    if (sconf->chan_config)
        sconf->chan_config_info = get_bits(&gb, 16);
    // TODO: use this to set avctx->channel_layout


    // read channel sorting
    if (sconf->chan_sort && avctx->channels > 1) {
        int chan_pos_bits = av_ceil_log2(avctx->channels);
        int bits_needed  = avctx->channels * chan_pos_bits + 7;
        if (get_bits_left(&gb) < bits_needed)
            return -1;

        if (!(sconf->chan_pos = av_malloc(avctx->channels * sizeof(*sconf->chan_pos))))
            return AVERROR(ENOMEM);

        for (i = 0; i < avctx->channels; i++)
            sconf->chan_pos[i] = get_bits(&gb, chan_pos_bits);

        align_get_bits(&gb);
        // TODO: use this to actually do channel sorting
    } else {
        sconf->chan_sort = 0;
    }


    // read fixed header and trailer sizes,
    // if size = 0xFFFFFFFF then there is no data field!
    if (get_bits_left(&gb) < 64)
        return -1;

    header_size  = get_bits_long(&gb, 32);
    trailer_size = get_bits_long(&gb, 32);
    if (header_size  == 0xFFFFFFFF)
        header_size  = 0;
    if (trailer_size == 0xFFFFFFFF)
        trailer_size = 0;

    ht_size = ((int64_t)(header_size) + (int64_t)(trailer_size)) << 3;


    // skip the header and trailer data
    if (get_bits_left(&gb) < ht_size)
        return -1;

    if (ht_size > INT32_MAX)
        return -1;

    skip_bits_long(&gb, ht_size);


    // initialize CRC calculation
    if (sconf->crc_enabled) {
        if (get_bits_left(&gb) < 32)
            return -1;

        if (avctx->error_recognition >= FF_ER_CAREFUL) {
            ctx->crc_table = av_crc_get_table(AV_CRC_32_IEEE_LE);
            ctx->crc       = 0xFFFFFFFF;
            ctx->crc_org   = ~get_bits_long(&gb, 32);
        } else
            skip_bits_long(&gb, 32);
    }


    // no need to read the rest of ALSSpecificConfig (ra_unit_size & aux data)

    ff_als_dprint_specific_config(avctx, sconf);

    return 0;
}


/** Checks the ALSSpecificConfig for unsupported features.
 */
static int check_specific_config(ALSDecContext *ctx)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    int error = 0;

    // report unsupported feature and set error value
    #define MISSING_ERR(cond, str, errval)              \
    {                                                   \
        if (cond) {                                     \
            av_log_missing_feature(ctx->avctx, str, 0); \
            error = errval;                             \
        }                                               \
    }

    MISSING_ERR(sconf->floating,             "Floating point decoding",     -1);
    MISSING_ERR(sconf->rlslms,               "Adaptive RLS-LMS prediction", -1);
    MISSING_ERR(sconf->chan_sort,            "Channel sorting",              0);

    return error;
}


/** Reads and decodes a Rice codeword.
 */
static int32_t decode_rice(GetBitContext *gb, unsigned int k)
{
    int max = get_bits_left(gb) - k;
    int q   = get_unary(gb, 0, max);
    int r   = k ? get_bits1(gb) : !(q & 1);

    if (k > 1) {
        q <<= (k - 1);
        q  += get_bits_long(gb, k - 1);
    } else if (!k) {
        q >>= 1;
    }
    return r ? q : ~q;
}


/** Reads block switching field if necessary and sets actual block sizes.
 *  Also assures that the block sizes of the last frame correspond to the
 *  actual number of samples.
 */
static void get_block_sizes(ALSDecContext *ctx, unsigned int *div_blocks,
                            uint32_t *bs_info)
{
    ALSSpecificConfig *sconf     = &ctx->sconf;
    GetBitContext *gb            = &ctx->gb;
    unsigned int *ptr_div_blocks = div_blocks;
    unsigned int b;

    if (sconf->block_switching) {
        unsigned int bs_info_len = 1 << (sconf->block_switching + 2);
        *bs_info = get_bits_long(gb, bs_info_len);
        *bs_info <<= (32 - bs_info_len);
    }

    ctx->num_blocks = 0;
    ff_als_parse_bs_info(*bs_info, 0, 0, &ptr_div_blocks, &ctx->num_blocks);

    // The last frame may have an overdetermined block structure given in
    // the bitstream. In that case the defined block structure would need
    // more samples than available to be consistent.
    // The block structure is actually used but the block sizes are adapted
    // to fit the actual number of available samples.
    // Example: 5 samples, 2nd level block sizes: 2 2 2 2.
    // This results in the actual block sizes:    2 2 1 0.
    // This is not specified in 14496-3 but actually done by the reference
    // codec RM22 revision 2.
    // This appears to happen in case of an odd number of samples in the last
    // frame which is actually not allowed by the block length switching part
    // of 14496-3.
    // The ALS conformance files feature an odd number of samples in the last
    // frame.

    for (b = 0; b < ctx->num_blocks; b++)
        div_blocks[b] = ctx->sconf.frame_length >> div_blocks[b];

    if (ctx->cur_frame_length != ctx->sconf.frame_length) {
        unsigned int remaining = ctx->cur_frame_length;

        for (b = 0; b < ctx->num_blocks; b++) {
            if (remaining < div_blocks[b]) {
                div_blocks[b] = remaining;
                ctx->num_blocks = b + 1;
                break;
            }

            remaining -= div_blocks[b];
        }
    }
}


/** Reads the block data for a constant block
 */
static void read_const_block_data(ALSDecContext *ctx, ALSBlockData *bd)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    AVCodecContext *avctx    = ctx->avctx;
    GetBitContext *gb        = &ctx->gb;

    bd->const_val    = 0;
    bd->const_block  = get_bits1(gb);    // 1 = constant value, 0 = zero block (silence)
    bd->js_blocks    = get_bits1(gb);

    // skip 5 reserved bits
    skip_bits(gb, 5);

    if (bd->const_block) {
        unsigned int const_val_bits = sconf->floating ? 24 : avctx->bits_per_raw_sample;
        bd->const_val = get_sbits_long(gb, const_val_bits);
    }

    // ensure constant block decoding by reusing this field
    bd->const_block = 1;
}


/** Decodes the block data for a constant block
 */
static void decode_const_block_data(ALSDecContext *ctx, ALSBlockData *bd)
{
    int      smp = bd->block_length;
    int32_t  val = bd->const_val;
    int32_t *dst = bd->raw_samples;

    // write raw samples into buffer
    for (; smp; smp--)
        *dst++ = val;
}


/** Reads the block data for a non-constant block
 */
static int read_var_block_data(ALSDecContext *ctx, ALSBlockData *bd)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    AVCodecContext *avctx    = ctx->avctx;
    GetBitContext *gb        = &ctx->gb;
    unsigned int k;
    unsigned int s[8];
    unsigned int sx[8];
    unsigned int sub_blocks, log2_sub_blocks, sb_length;
    unsigned int start      = 0;
    unsigned int opt_order;
    int          sb;
    int32_t      *quant_cof = bd->quant_cof;
    int32_t      *current_res;


    // ensure variable block decoding by reusing this field
    bd->const_block = 0;

    bd->opt_order   = 1;
    bd->js_blocks   = get_bits1(gb);

    opt_order       = bd->opt_order;

    // determine the number of subblocks for entropy decoding
    if (!sconf->bgmc && !sconf->sb_part) {
        log2_sub_blocks = 0;
    } else {
        if (sconf->bgmc && sconf->sb_part)
            log2_sub_blocks = get_bits(gb, 2);
        else
            log2_sub_blocks = 2 * get_bits1(gb);
    }

    sub_blocks = 1 << log2_sub_blocks;

    // do not continue in case of a damaged stream since
    // block_length must be evenly divisible by sub_blocks
    if (bd->block_length & (sub_blocks - 1)) {
        av_log(avctx, AV_LOG_WARNING,
               "Block length is not evenly divisible by the number of subblocks.\n");
        return -1;
    }

    sb_length = bd->block_length >> log2_sub_blocks;

    if (sconf->bgmc) {
        s[0] = get_bits(gb, 8 + (sconf->resolution > 1));
        for (k = 1; k < sub_blocks; k++)
            s[k] = s[k - 1] + decode_rice(gb, 2);

        for (k = 0; k < sub_blocks; k++) {
            sx[k]   = s[k] & 0x0F;
            s [k] >>= 4;
        }
    } else {
        s[0] = get_bits(gb, 4 + (sconf->resolution > 1));
        for (k = 1; k < sub_blocks; k++) {
            s[k] = s[k - 1] + decode_rice(gb, 0);}
    }

    if (get_bits1(gb))
        bd->shift_lsbs = get_bits(gb, 4) + 1;

    bd->store_prev_samples = (bd->js_blocks && bd->raw_other) || bd->shift_lsbs;


    if (!sconf->rlslms) {
        if (sconf->adapt_order) {
            int opt_order_length = av_ceil_log2(av_clip((bd->block_length >> 3) - 1,
                                                2, sconf->max_order + 1));
            bd->opt_order        = get_bits(gb, opt_order_length);
        } else {
            bd->opt_order = sconf->max_order;
        }

        opt_order = bd->opt_order;

        if (opt_order) {
            int add_base;

            if (sconf->coef_table == 3) {
                add_base = 0x7F;

                // read coefficient 0
                quant_cof[0] = 32 * ff_als_parcor_scaled_values[get_bits(gb, 7)];

                // read coefficient 1
                if (opt_order > 1)
                    quant_cof[1] = -32 * ff_als_parcor_scaled_values[get_bits(gb, 7)];

                // read coefficients 2 to opt_order
                for (k = 2; k < opt_order; k++)
                    quant_cof[k] = get_bits(gb, 7);
            } else {
                int k_max;
                add_base = 1;

                // read coefficient 0 to 19
                k_max = FFMIN(opt_order, 20);
                for (k = 0; k < k_max; k++) {
                    int rice_param = ff_als_parcor_rice_table[sconf->coef_table][k][1];
                    int offset     = ff_als_parcor_rice_table[sconf->coef_table][k][0];
                    quant_cof[k] = decode_rice(gb, rice_param) + offset;
                }

                // read coefficients 20 to 126
                k_max = FFMIN(opt_order, 127);
                for (; k < k_max; k++)
                    quant_cof[k] = decode_rice(gb, 2) + (k & 1);

                // read coefficients 127 to opt_order
                for (; k < opt_order; k++)
                    quant_cof[k] = decode_rice(gb, 1);

                quant_cof[0] = 32 * ff_als_parcor_scaled_values[quant_cof[0] + 64];

                if (opt_order > 1)
                    quant_cof[1] = -32 * ff_als_parcor_scaled_values[quant_cof[1] + 64];
            }

            for (k = 2; k < opt_order; k++)
                quant_cof[k] = (quant_cof[k] << 14) + (add_base << 13);
        }
    }

    // read LTP gain and lag values
    if (sconf->long_term_prediction) {
        *bd->use_ltp = get_bits1(gb);

        if (*bd->use_ltp) {
            int r, c;

            bd->ltp_gain[0]   = decode_rice(gb, 1) << 3;
            bd->ltp_gain[1]   = decode_rice(gb, 2) << 3;

            r                 = get_unary(gb, 0, 4);
            c                 = get_bits(gb, 2);
            bd->ltp_gain[2]   = ff_als_ltp_gain_values[r][c];

            bd->ltp_gain[3]   = decode_rice(gb, 2) << 3;
            bd->ltp_gain[4]   = decode_rice(gb, 1) << 3;

            *bd->ltp_lag      = get_bits(gb, ctx->ltp_lag_length);
            *bd->ltp_lag     += FFMAX(4, opt_order + 1);
        }
    }

    // read first value and residuals in case of a random access block
    if (bd->ra_block) {
        if (opt_order)
            bd->raw_samples[0] = decode_rice(gb, avctx->bits_per_raw_sample - 4);
        if (opt_order > 1)
            bd->raw_samples[1] = decode_rice(gb, FFMIN(s[0] + 3, ctx->s_max));
        if (opt_order > 2)
            bd->raw_samples[2] = decode_rice(gb, FFMIN(s[0] + 1, ctx->s_max));

        start = FFMIN(opt_order, 3);
    }

    // read all residuals
    if (sconf->bgmc) {
        unsigned int delta[sub_blocks];
        unsigned int k    [sub_blocks];
        unsigned int b = av_clip((av_ceil_log2(bd->block_length) - 3) >> 1, 0, 5);
        unsigned int i = start;

        // read most significant bits
        unsigned int high;
        unsigned int low;
        unsigned int value;

        ff_bgmc_decode_init(gb, &high, &low, &value);

        current_res = bd->raw_samples + start;

        for (sb = 0; sb < sub_blocks; sb++, i = 0) {
            k    [sb] = s[sb] > b ? s[sb] - b : 0;
            delta[sb] = 5 - s[sb] + k[sb];

            ff_bgmc_decode(gb, sb_length, current_res,
                        delta[sb], sx[sb], &high, &low, &value, ctx->bgmc_lut, ctx->bgmc_lut_status);

            current_res += sb_length;
        }

        ff_bgmc_decode_end(gb);


        // read least significant bits and tails
        i = start;
        current_res = bd->raw_samples + start;

        for (sb = 0; sb < sub_blocks; sb++, i = 0) {
            unsigned int cur_tail_code = ff_bgmc_tail_code[sx[sb]][delta[sb]];
            unsigned int cur_k         = k[sb];
            unsigned int cur_s         = s[sb];

            for (; i < sb_length; i++) {
                int32_t res = *current_res;

                if (res == cur_tail_code) {
                    unsigned int max_msb =   (2 + (sx[sb] > 2) + (sx[sb] > 10))
                                          << (5 - delta[sb]);

                    res = decode_rice(gb, cur_s);

                    if (res >= 0) {
                        res += (max_msb    ) << cur_k;
                    } else {
                        res -= (max_msb - 1) << cur_k;
                    }
                } else {
                    if (res > cur_tail_code)
                        res--;

                    if (res & 1)
                        res = -res;

                    res >>= 1;

                    if (cur_k) {
                        res <<= cur_k;
                        res  |= get_bits_long(gb, cur_k);
                    }
                }

                *current_res++ = res;
            }
        }
    } else {
        current_res = bd->raw_samples + start;

        for (sb = 0; sb < sub_blocks; sb++, start = 0) {
            for (; start < sb_length; start++)
                *current_res++ = decode_rice(gb, s[sb]);}
     }

    if (!sconf->mc_coding || ctx->js_switch)
        align_get_bits(gb);

    return 0;
}


/** Decodes the block data for a non-constant block
 */
static int decode_var_block_data(ALSDecContext *ctx, ALSBlockData *bd)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int block_length = bd->block_length;
    unsigned int smp = 0;
    unsigned int k;
    int opt_order             = bd->opt_order;
    int sb;
    int64_t y;
    int32_t *quant_cof        = bd->quant_cof;
    int32_t *lpc_cof          = bd->lpc_cof;
    int32_t *raw_samples      = bd->raw_samples;
    int32_t *raw_samples_end  = bd->raw_samples + bd->block_length;
    int32_t *lpc_cof_reversed = ctx->lpc_cof_reversed_buffer;

    // reverse long-term prediction
    if (*bd->use_ltp) {
        int ltp_smp;

        for (ltp_smp = FFMAX(*bd->ltp_lag - 2, 0); ltp_smp < block_length; ltp_smp++) {
            int center = ltp_smp - *bd->ltp_lag;
            int begin  = FFMAX(0, center - 2);
            int end    = center + 3;
            int tab    = 5 - (end - begin);
            int base;

            y = 1 << 6;

            for (base = begin; base < end; base++, tab++) {
                y += MUL64(bd->ltp_gain[tab], raw_samples[base]);
            }

            raw_samples[ltp_smp] += y >> 7;
        }
    }

    // reconstruct all samples from residuals
    if (bd->ra_block) {
        for (smp = 0; smp < opt_order; smp++) {
            y = 1 << 19;

            for (sb = 0; sb < smp; sb++)
                y += MUL64(lpc_cof[sb], raw_samples[-(sb + 1)]);

            *raw_samples++ -= y >> 20;
            ff_als_parcor_to_lpc(smp, quant_cof, lpc_cof);
        }
    } else {
        for (k = 0; k < opt_order; k++)
            ff_als_parcor_to_lpc(k, quant_cof, lpc_cof);

        // store previous samples in case that they have to be altered
        if (bd->store_prev_samples)
            memcpy(bd->prev_raw_samples, raw_samples - sconf->max_order,
                   sizeof(*bd->prev_raw_samples) * sconf->max_order);

        // reconstruct difference signal for prediction (joint-stereo)
        if (bd->js_blocks && bd->raw_other) {
            int32_t *left, *right;

            if (bd->raw_other > raw_samples) {  // D = R - L
                left  = raw_samples;
                right = bd->raw_other;
            } else {                                // D = R - L
                left  = bd->raw_other;
                right = raw_samples;
            }

            for (sb = -1; sb >= -sconf->max_order; sb--)
                raw_samples[sb] = right[sb] - left[sb];
        }

        // reconstruct shifted signal
        if (bd->shift_lsbs)
            for (sb = -1; sb >= -sconf->max_order; sb--)
                raw_samples[sb] >>= bd->shift_lsbs;
    }

    // reverse linear prediction coefficients for efficiency
    lpc_cof = lpc_cof + opt_order;

    for (sb = 0; sb < opt_order; sb++)
        lpc_cof_reversed[sb] = lpc_cof[-(sb + 1)];

    // reconstruct raw samples
    raw_samples = bd->raw_samples + smp;
    lpc_cof     = lpc_cof_reversed + opt_order;

    for (; raw_samples < raw_samples_end; raw_samples++) {
        y = 1 << 19;

        for (sb = -opt_order; sb < 0; sb++)
            y += MUL64(lpc_cof[sb], raw_samples[sb]);

        *raw_samples -= y >> 20;
    }

    raw_samples = bd->raw_samples;

    // restore previous samples in case that they have been altered
    if (bd->store_prev_samples)
        memcpy(raw_samples - sconf->max_order, bd->prev_raw_samples,
               sizeof(*raw_samples) * sconf->max_order);

    return 0;
}


/** Reads the block data.
 */
static int read_block(ALSDecContext *ctx, ALSBlockData *bd)
{
    GetBitContext *gb        = &ctx->gb;

    // read block type flag and read the samples accordingly
    if (get_bits1(gb)) {
        if (read_var_block_data(ctx, bd))
            return -1;
    } else {
        read_const_block_data(ctx, bd);
    }

    return 0;
}


/** Decodes the block data.
 */
static int decode_block(ALSDecContext *ctx, ALSBlockData *bd)
{
    unsigned int smp;

    // read block type flag and read the samples accordingly
    if (bd->const_block)
        decode_const_block_data(ctx, bd);
    else  if (decode_var_block_data(ctx, bd))
        return -1;


    // TODO: read RLSLMS extension data

    if (bd->shift_lsbs)
        for (smp = 0; smp < bd->block_length; smp++)
            bd->raw_samples[smp] <<= bd->shift_lsbs;

    return 0;
}


/** Reads and decodes block data successively.
 */
static int read_decode_block(ALSDecContext *ctx, ALSBlockData *bd)
{
    int ret;

    ret = read_block(ctx, bd);

    if (ret)
        return ret;

    ret = decode_block(ctx, bd);

    return ret;
}


/** Computes the number of samples left to decode for the current frame and
 *  sets these samples to zero.
 */
static void zero_remaining(unsigned int b, unsigned int b_max,
                           const unsigned int *div_blocks, int32_t *buf)
{
    unsigned int count = 0;

    while (b < b_max)
        count += div_blocks[b];

    if (count)
        memset(buf, 0, sizeof(*buf) * count);
}


/** Decodes blocks independently.
 */
static int decode_blocks_ind(ALSDecContext *ctx, unsigned int ra_frame,
                             unsigned int c, const unsigned int *div_blocks,
                             unsigned int *js_blocks)
{
    unsigned int b;
    ALSBlockData bd;

    memset(&bd, 0, sizeof(ALSBlockData));

    bd.ra_block         = ra_frame;
    bd.use_ltp          = ctx->use_ltp;
    bd.ltp_lag          = ctx->ltp_lag;
    bd.ltp_gain         = ctx->ltp_gain[0];
    bd.quant_cof        = ctx->quant_cof[0];
    bd.lpc_cof          = ctx->lpc_cof[0];
    bd.prev_raw_samples = ctx->prev_raw_samples;
    bd.raw_samples      = ctx->raw_samples[c];


    for (b = 0; b < ctx->num_blocks; b++) {
        bd.shift_lsbs       = 0;
        bd.block_length     = div_blocks[b];

        if (read_decode_block(ctx, &bd)) {
            // damaged block, write zero for the rest of the frame
            zero_remaining(b, ctx->num_blocks, div_blocks, bd.raw_samples);
            return -1;
        }
        bd.raw_samples += div_blocks[b];
        bd.ra_block     = 0;
    }

    return 0;
}


/** Decodes blocks dependently.
 */
static int decode_blocks(ALSDecContext *ctx, unsigned int ra_frame,
                         unsigned int c, const unsigned int *div_blocks,
                         unsigned int *js_blocks)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int offset = 0;
    unsigned int b;
    ALSBlockData bd[2];

    memset(bd, 0, 2 * sizeof(ALSBlockData));

    bd[0].ra_block         = ra_frame;
    bd[0].use_ltp          = ctx->use_ltp;
    bd[0].ltp_lag          = ctx->ltp_lag;
    bd[0].ltp_gain         = ctx->ltp_gain[0];
    bd[0].quant_cof        = ctx->quant_cof[0];
    bd[0].lpc_cof          = ctx->lpc_cof[0];
    bd[0].prev_raw_samples = ctx->prev_raw_samples;
    bd[0].js_blocks        = *js_blocks;

    bd[1].ra_block         = ra_frame;
    bd[1].use_ltp          = ctx->use_ltp;
    bd[1].ltp_lag          = ctx->ltp_lag;
    bd[1].ltp_gain         = ctx->ltp_gain[0];
    bd[1].quant_cof        = ctx->quant_cof[0];
    bd[1].lpc_cof          = ctx->lpc_cof[0];
    bd[1].prev_raw_samples = ctx->prev_raw_samples;
    bd[1].js_blocks        = *(js_blocks + 1);

    // decode all blocks
    for (b = 0; b < ctx->num_blocks; b++) {
        unsigned int s;

        bd[0].shift_lsbs   = 0;
        bd[1].shift_lsbs   = 0;

        bd[0].block_length = div_blocks[b];
        bd[1].block_length = div_blocks[b];

        bd[0].raw_samples  = ctx->raw_samples[c    ] + offset;
        bd[1].raw_samples  = ctx->raw_samples[c + 1] + offset;

        bd[0].raw_other    = bd[1].raw_samples;
        bd[1].raw_other    = bd[0].raw_samples;

        if(read_decode_block(ctx, &bd[0]) || read_decode_block(ctx, &bd[1])) {
            // damaged block, write zero for the rest of the frame
            zero_remaining(b, ctx->num_blocks, div_blocks, bd[0].raw_samples);
            zero_remaining(b, ctx->num_blocks, div_blocks, bd[1].raw_samples);
            return -1;
        }

        // reconstruct joint-stereo blocks
        if (bd[0].js_blocks) {
            if (bd[1].js_blocks)
                av_log(ctx->avctx, AV_LOG_WARNING, "Invalid channel pair!\n");

            for (s = 0; s < div_blocks[b]; s++)
                bd[0].raw_samples[s] = bd[1].raw_samples[s] - bd[0].raw_samples[s];
        } else if (bd[1].js_blocks) {
            for (s = 0; s < div_blocks[b]; s++)
                bd[1].raw_samples[s] = bd[1].raw_samples[s] + bd[0].raw_samples[s];
        }

        offset  += div_blocks[b];
        bd[0].ra_block = 0;
        bd[1].ra_block = 0;
    }

    // store carryover raw samples,
    // the others channel raw samples are stored by the calling function.
    memmove(ctx->raw_samples[c] - sconf->max_order,
            ctx->raw_samples[c] - sconf->max_order + sconf->frame_length,
            sizeof(*ctx->raw_samples[c]) * sconf->max_order);

    return 0;
}


/** Reads the channel data.
  */
static int read_channel_data(ALSDecContext *ctx, ALSChannelData *cd, int c)
{
    GetBitContext *gb       = &ctx->gb;
    ALSChannelData *current = cd;
    unsigned int channels   = ctx->avctx->channels;
    int entries             = 0;

    while (entries < channels && !(current->stop_flag = get_bits1(gb))) {
        current->master_channel = get_bits_long(gb, av_ceil_log2(channels));

        if (current->master_channel >= channels) {
            av_log(ctx->avctx, AV_LOG_ERROR, "Invalid master channel!\n");
            return -1;
        }

        if (current->master_channel != c) {
            current->time_diff_flag = get_bits1(gb);
            current->weighting[0]   = mcc_weightings[av_clip(decode_rice(gb, 1) + 16, 0, 32)];
            current->weighting[1]   = mcc_weightings[av_clip(decode_rice(gb, 2) + 14, 0, 32)];
            current->weighting[2]   = mcc_weightings[av_clip(decode_rice(gb, 1) + 16, 0, 32)];

            if (current->time_diff_flag) {
                current->weighting[3] = mcc_weightings[av_clip(decode_rice(gb, 1) + 16, 0, 32)];
                current->weighting[4] = mcc_weightings[av_clip(decode_rice(gb, 1) + 16, 0, 32)];
                current->weighting[5] = mcc_weightings[av_clip(decode_rice(gb, 1) + 16, 0, 32)];

                current->time_diff_sign  = get_bits1(gb);
                current->time_diff_index = get_bits(gb, ctx->ltp_lag_length - 3) + 3;
            }
        }

        current++;
        entries++;
    }

    if (entries == channels) {
        av_log(ctx->avctx, AV_LOG_ERROR, "Damaged channel data!\n");
        return -1;
    }

    align_get_bits(gb);
    return 0;
}


/** Recursively reverts the inter-channel correlation for a block.
 */
static int revert_channel_correlation(ALSDecContext *ctx, ALSBlockData *bd,
                                       ALSChannelData **cd, int *reverted,
                                       unsigned int offset, int c)
{
    ALSChannelData *ch = cd[c];
    unsigned int   dep = 0;
    unsigned int channels = ctx->avctx->channels;

    if (reverted[c])
        return 0;

    reverted[c] = 1;

    while (dep < channels && !ch[dep].stop_flag) {
        revert_channel_correlation(ctx, bd, cd, reverted, offset,
                                   ch[dep].master_channel);

        dep++;
    }

    if (dep == channels) {
        av_log(ctx->avctx, AV_LOG_WARNING, "Invalid channel correlation!\n");
        return -1;
    }

    bd->use_ltp     = ctx->use_ltp + c;
    bd->ltp_lag     = ctx->ltp_lag + c;
    bd->ltp_gain    = ctx->ltp_gain[c];
    bd->lpc_cof     = ctx->lpc_cof[c];
    bd->quant_cof   = ctx->quant_cof[c];
    bd->raw_samples = ctx->raw_samples[c] + offset;

    dep = 0;
    while (!ch[dep].stop_flag) {
        unsigned int smp;
        unsigned int begin = 1;
        unsigned int end   = bd->block_length - 1;
        int64_t y;
        int32_t *master = ctx->raw_samples[ch[dep].master_channel] + offset;

        if (ch[dep].time_diff_flag) {
            int t = ch[dep].time_diff_index;

            if (ch[dep].time_diff_sign) {
                t      = -t;
                begin -= t;
            } else {
                end   -= t;
            }

            for (smp = begin; smp < end; smp++) {
                y  = (1 << 6) +
                     MUL64(ch[dep].weighting[0], master[smp - 1    ]) +
                     MUL64(ch[dep].weighting[1], master[smp        ]) +
                     MUL64(ch[dep].weighting[2], master[smp + 1    ]) +
                     MUL64(ch[dep].weighting[3], master[smp - 1 + t]) +
                     MUL64(ch[dep].weighting[4], master[smp     + t]) +
                     MUL64(ch[dep].weighting[5], master[smp + 1 + t]);

                bd->raw_samples[smp] += y >> 7;
            }
        } else {
            for (smp = begin; smp < end; smp++) {
                y  = (1 << 6) +
                     MUL64(ch[dep].weighting[0], master[smp - 1]) +
                     MUL64(ch[dep].weighting[1], master[smp    ]) +
                     MUL64(ch[dep].weighting[2], master[smp + 1]);

                bd->raw_samples[smp] += y >> 7;
            }
        }

        dep++;
    }

    return 0;
}


/** Reads the frame data.
 */
static int read_frame_data(ALSDecContext *ctx, unsigned int ra_frame)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    AVCodecContext *avctx    = ctx->avctx;
    GetBitContext *gb = &ctx->gb;
    unsigned int div_blocks[32];                ///< block sizes.
    unsigned int c;
    unsigned int js_blocks[2];

    uint32_t bs_info = 0;

    // skip the size of the ra unit if present in the frame
    if (sconf->ra_flag == RA_FLAG_FRAMES && ra_frame)
        skip_bits_long(gb, 32);

    if (sconf->mc_coding && sconf->joint_stereo) {
        ctx->js_switch = get_bits1(gb);
        align_get_bits(gb);
    }

    if (!sconf->mc_coding || ctx->js_switch) {
        int independent_bs = !sconf->joint_stereo;

        for (c = 0; c < avctx->channels; c++) {
            js_blocks[0] = 0;
            js_blocks[1] = 0;

            get_block_sizes(ctx, div_blocks, &bs_info);

            // if joint_stereo and block_switching is set, independent decoding
            // is signaled via the first bit of bs_info
            if (sconf->joint_stereo && sconf->block_switching)
                if (bs_info >> 31)
                    independent_bs = 2;

            // if this is the last channel, it has to be decoded independently
            if (c == avctx->channels - 1)
                independent_bs = 1;

            if (independent_bs) {
                if (decode_blocks_ind(ctx, ra_frame, c, div_blocks, js_blocks))
                    return -1;

                independent_bs--;
            } else {
                if (decode_blocks(ctx, ra_frame, c, div_blocks, js_blocks))
                    return -1;

                c++;
            }

            // store carryover raw samples
            memmove(ctx->raw_samples[c] - sconf->max_order,
                    ctx->raw_samples[c] - sconf->max_order + sconf->frame_length,
                    sizeof(*ctx->raw_samples[c]) * sconf->max_order);
        }
    } else { // multi-channel coding
        ALSBlockData   bd;
        int            b;
        int            *reverted_channels = ctx->reverted_channels;
        unsigned int   offset             = 0;

        for (c = 0; c < avctx->channels; c++)
            if (ctx->chan_data[c] < ctx->chan_data_buffer) {
                av_log(ctx->avctx, AV_LOG_ERROR, "Invalid channel data!\n");
                return -1;
            }

        memset(&bd,               0, sizeof(ALSBlockData));
        memset(reverted_channels, 0, sizeof(*reverted_channels) * avctx->channels);

        bd.ra_block         = ra_frame;
        bd.prev_raw_samples = ctx->prev_raw_samples;

        get_block_sizes(ctx, div_blocks, &bs_info);

        for (b = 0; b < ctx->num_blocks; b++) {
            bd.shift_lsbs   = 0;
            bd.block_length = div_blocks[b];

            for (c = 0; c < avctx->channels; c++) {
                bd.use_ltp     = ctx->use_ltp + c;
                bd.ltp_lag     = ctx->ltp_lag + c;
                bd.ltp_gain    = ctx->ltp_gain[c];
                bd.lpc_cof     = ctx->lpc_cof[c];
                bd.quant_cof   = ctx->quant_cof[c];
                bd.raw_samples = ctx->raw_samples[c] + offset;
                bd.raw_other   = NULL;

                read_block(ctx, &bd);
                if (read_channel_data(ctx, ctx->chan_data[c], c))
                    return -1;
            }

            for (c = 0; c < avctx->channels; c++)
                if (revert_channel_correlation(ctx, &bd, ctx->chan_data,
                                               reverted_channels, offset, c))
                    return -1;

            for (c = 0; c < avctx->channels; c++) {
                bd.use_ltp     = ctx->use_ltp + c;
                bd.ltp_lag     = ctx->ltp_lag + c;
                bd.ltp_gain    = ctx->ltp_gain[c];
                bd.lpc_cof     = ctx->lpc_cof[c];
                bd.quant_cof   = ctx->quant_cof[c];
                bd.raw_samples = ctx->raw_samples[c] + offset;
                decode_block(ctx, &bd);
            }

            memset(reverted_channels, 0, avctx->channels * sizeof(*reverted_channels));
            offset      += div_blocks[b];
            bd.ra_block  = 0;
        }

        // store carryover raw samples
        for (c = 0; c < avctx->channels; c++)
            memmove(ctx->raw_samples[c] - sconf->max_order,
                    ctx->raw_samples[c] - sconf->max_order + sconf->frame_length,
                    sizeof(*ctx->raw_samples[c]) * sconf->max_order);
    }

    // TODO: read_diff_float_data

    return 0;
}


/** Decodes an ALS frame.
 */
static int decode_frame(AVCodecContext *avctx,
                        void *data, int *data_size,
                        AVPacket *avpkt)
{
    ALSDecContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    const uint8_t *buffer    = avpkt->data;
    int buffer_size          = avpkt->size;
    int invalid_frame, size;
    unsigned int c, sample, ra_frame, bytes_read, shift;

    init_get_bits(&ctx->gb, buffer, buffer_size * 8);

    // In the case that the distance between random access frames is set to zero
    // (sconf->ra_distance == 0) no frame is treated as a random access frame.
    // For the first frame, if prediction is used, all samples used from the
    // previous frame are assumed to be zero.
    ra_frame = sconf->ra_distance && !(ctx->frame_id % sconf->ra_distance);

    // the last frame to decode might have a different length
    if (sconf->samples != 0xFFFFFFFF)
        ctx->cur_frame_length = FFMIN(sconf->samples - ctx->frame_id * (uint64_t) sconf->frame_length,
                                      sconf->frame_length);
    else
        ctx->cur_frame_length = sconf->frame_length;

    // decode the frame data
    if ((invalid_frame = read_frame_data(ctx, ra_frame) < 0))
        av_log(ctx->avctx, AV_LOG_WARNING,
               "Reading frame data failed. Skipping RA unit.\n");

    ctx->frame_id++;

    // check for size of decoded data
    size = ctx->cur_frame_length * avctx->channels *
           (av_get_bits_per_sample_format(avctx->sample_fmt) >> 3);

    if (size > *data_size) {
        av_log(avctx, AV_LOG_ERROR, "Decoded data exceeds buffer size.\n");
        return -1;
    }

    *data_size = size;

    // transform decoded frame into output format
    #define INTERLEAVE_OUTPUT(bps)                                 \
    {                                                              \
        int##bps##_t *dest = (int##bps##_t*) data;                 \
        shift = bps - ctx->avctx->bits_per_raw_sample;             \
        for (sample = 0; sample < ctx->cur_frame_length; sample++) \
            for (c = 0; c < avctx->channels; c++)                  \
                *dest++ = ctx->raw_samples[c][sample] << shift;    \
    }

    if (ctx->avctx->bits_per_raw_sample <= 16) {
        INTERLEAVE_OUTPUT(16)
    } else {
        INTERLEAVE_OUTPUT(32)
    }

    // update CRC
    if (sconf->crc_enabled && avctx->error_recognition >= FF_ER_CAREFUL) {
        uint8_t *crc_source;

        #define CONVERT_OUTPUT(bps, fun)                            \
        {                                                           \
            int##bps##_t *src  = (int##bps##_t*) data;              \
            int##bps##_t *dest = (int##bps##_t*) ctx->crc_buffer;   \
            for (sample = 0;                                        \
                 sample < ctx->cur_frame_length * avctx->channels;  \
                 sample++)                                          \
                    *dest++ = fun##_##bps (src[sample]);            \
        }

        #define COND_CONVERT(cond, func)                   \
        {                                                  \
            if (cond) {                                    \
                if (ctx->avctx->bits_per_raw_sample <= 16) \
                    CONVERT_OUTPUT(16, func)               \
                else                                       \
                    CONVERT_OUTPUT(32, func)               \
                crc_source = ctx->crc_buffer;              \
            } else {                                       \
                crc_source = data;                         \
            }                                              \
        }

#if HAVE_BIGENDIAN
        COND_CONVERT(!sconf->msb_first, le2me);
#else
        COND_CONVERT( sconf->msb_first, be2me);
#endif

        ctx->crc = av_crc(ctx->crc_table, ctx->crc, crc_source, size);

        // check CRC sums if this is the last frame
        if (ctx->cur_frame_length != sconf->frame_length &&
            ctx->crc_org != ctx->crc) {
            av_log(avctx, AV_LOG_ERROR, "CRC error.\n");
        }
    }


    bytes_read = invalid_frame ? buffer_size :
                                 (get_bits_count(&ctx->gb) + 7) >> 3;

    return bytes_read;
}


/** Uninitializes the ALS decoder.
 */
static av_cold int decode_end(AVCodecContext *avctx)
{
    ALSDecContext *ctx = avctx->priv_data;

    av_freep(&ctx->sconf.chan_pos);

    ff_bgmc_end(&ctx->bgmc_lut, &ctx->bgmc_lut_status);

    av_freep(&ctx->use_ltp);
    av_freep(&ctx->ltp_lag);
    av_freep(&ctx->ltp_gain);
    av_freep(&ctx->ltp_gain_buffer);
    av_freep(&ctx->quant_cof);
    av_freep(&ctx->lpc_cof);
    av_freep(&ctx->quant_cof_buffer);
    av_freep(&ctx->lpc_cof_buffer);
    av_freep(&ctx->lpc_cof_reversed_buffer);
    av_freep(&ctx->prev_raw_samples);
    av_freep(&ctx->raw_samples);
    av_freep(&ctx->raw_buffer);
    av_freep(&ctx->chan_data);
    av_freep(&ctx->chan_data_buffer);
    av_freep(&ctx->reverted_channels);

    return 0;
}


/** Initializes the ALS decoder.
 */
static av_cold int decode_init(AVCodecContext *avctx)
{
    unsigned int c;
    unsigned int channel_size;
    int num_buffers;
    ALSDecContext *ctx = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    ctx->avctx = avctx;

    if (!avctx->extradata) {
        av_log(avctx, AV_LOG_ERROR, "Missing required ALS extradata.\n");
        return -1;
    }

    if (read_specific_config(ctx)) {
        av_log(avctx, AV_LOG_ERROR, "Reading ALSSpecificConfig failed.\n");
        decode_end(avctx);
        return -1;
    }

    if (check_specific_config(ctx)) {
        decode_end(avctx);
        return -1;
    }

    if (sconf->bgmc)
        ff_bgmc_init(avctx, &ctx->bgmc_lut, &ctx->bgmc_lut_status);

    if (sconf->floating) {
        avctx->sample_fmt          = SAMPLE_FMT_FLT;
        avctx->bits_per_raw_sample = 32;
    } else {
        avctx->sample_fmt          = sconf->resolution > 1
                                     ? SAMPLE_FMT_S32 : SAMPLE_FMT_S16;
        avctx->bits_per_raw_sample = (sconf->resolution + 1) * 8;
    }

    // set maximum Rice parameter for progressive decoding based on resolution
    // This is not specified in 14496-3 but actually done by the reference
    // codec RM22 revision 2.
    ctx->s_max = sconf->resolution > 1 ? 31 : 15;

    // set lag value for long-term prediction
    ctx->ltp_lag_length = 8 + (avctx->sample_rate >=  96000) +
                              (avctx->sample_rate >= 192000);

    // allocate quantized parcor coefficient buffer
    num_buffers = sconf->mc_coding ? avctx->channels : 1;

    ctx->quant_cof        = av_malloc(sizeof(*ctx->quant_cof) * num_buffers);
    ctx->lpc_cof          = av_malloc(sizeof(*ctx->lpc_cof)   * num_buffers);
    ctx->quant_cof_buffer = av_malloc(sizeof(*ctx->quant_cof_buffer) *
                                      num_buffers * sconf->max_order);
    ctx->lpc_cof_buffer   = av_malloc(sizeof(*ctx->lpc_cof_buffer) *
                                      num_buffers * sconf->max_order);
    ctx->lpc_cof_reversed_buffer = av_malloc(sizeof(*ctx->lpc_cof_buffer) *
                                             sconf->max_order);

    if (!ctx->quant_cof              || !ctx->lpc_cof        ||
        !ctx->quant_cof_buffer       || !ctx->lpc_cof_buffer ||
        !ctx->lpc_cof_reversed_buffer) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        return AVERROR(ENOMEM);
    }

    // assign quantized parcor coefficient buffers
    for (c = 0; c < num_buffers; c++) {
        ctx->quant_cof[c] = ctx->quant_cof_buffer + c * sconf->max_order;
        ctx->lpc_cof[c]   = ctx->lpc_cof_buffer   + c * sconf->max_order;
    }

    // allocate and assign lag and gain data buffer for ltp mode
    ctx->use_ltp         = av_mallocz(sizeof(*ctx->use_ltp)  * num_buffers);
    ctx->ltp_lag         = av_malloc (sizeof(*ctx->ltp_lag)  * num_buffers);
    ctx->ltp_gain        = av_malloc (sizeof(*ctx->ltp_gain) * num_buffers);
    ctx->ltp_gain_buffer = av_malloc (sizeof(*ctx->ltp_gain_buffer) *
                                      num_buffers * 5);

    if (!ctx->use_ltp  || !ctx->ltp_lag ||
        !ctx->ltp_gain || !ctx->ltp_gain_buffer) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        decode_end(avctx);
        return AVERROR(ENOMEM);
    }

    for (c = 0; c < num_buffers; c++)
        ctx->ltp_gain[c] = ctx->ltp_gain_buffer + c * 5;

    // allocate and assign channel data buffer for mcc mode
    if (sconf->mc_coding) {
        ctx->chan_data_buffer  = av_malloc(sizeof(*ctx->chan_data_buffer) *
                                           num_buffers * num_buffers);
        ctx->chan_data         = av_malloc(sizeof(*ctx->chan_data) *
                                           num_buffers);
        ctx->reverted_channels = av_malloc(sizeof(*ctx->reverted_channels) *
                                           num_buffers);

        if (!ctx->chan_data_buffer || !ctx->chan_data || !ctx->reverted_channels) {
            av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
            decode_end(avctx);
            return AVERROR(ENOMEM);
        }

        for (c = 0; c < num_buffers; c++)
            ctx->chan_data[c] = ctx->chan_data_buffer + c * num_buffers;
    } else {
        ctx->chan_data         = NULL;
        ctx->chan_data_buffer  = NULL;
        ctx->reverted_channels = NULL;
    }

    avctx->frame_size = sconf->frame_length;
    channel_size      = sconf->frame_length + sconf->max_order;

    ctx->prev_raw_samples = av_malloc (sizeof(*ctx->prev_raw_samples) * sconf->max_order);
    ctx->raw_buffer       = av_mallocz(sizeof(*ctx->     raw_buffer)  * avctx->channels * channel_size);
    ctx->raw_samples      = av_malloc (sizeof(*ctx->     raw_samples) * avctx->channels);

    // allocate previous raw sample buffer
    if (!ctx->prev_raw_samples || !ctx->raw_buffer|| !ctx->raw_samples) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        decode_end(avctx);
        return AVERROR(ENOMEM);
    }

    // assign raw samples buffers
    ctx->raw_samples[0] = ctx->raw_buffer + sconf->max_order;
    for (c = 1; c < avctx->channels; c++)
        ctx->raw_samples[c] = ctx->raw_samples[c - 1] + channel_size;

    // allocate crc buffer
    #define COND_MALLOC(cond)                                       \
    {                                                               \
        if ((cond) &&  sconf->crc_enabled &&                        \
            avctx->error_recognition >= FF_ER_CAREFUL) {            \
            ctx->crc_buffer = av_malloc(sizeof(*ctx->crc_buffer) *  \
                                        ctx->cur_frame_length *     \
                                        avctx->channels *           \
                                        (av_get_bits_per_sample_format(avctx->sample_fmt) >> 3)); \
            if (!ctx->crc_buffer) {                                 \
                av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n"); \
                decode_end(avctx);                                  \
                return AVERROR(ENOMEM);                             \
            }                                                       \
        }                                                           \
    }

#if HAVE_BIGENDIAN
    COND_MALLOC(!sconf->msb_first);
#else
    COND_MALLOC( sconf->msb_first);
#endif

    return 0;
}


/** Flushes (resets) the frame ID after seeking.
 */
static av_cold void flush(AVCodecContext *avctx)
{
    ALSDecContext *ctx = avctx->priv_data;

    ctx->frame_id = 0;
}


AVCodec als_decoder = {
    "als",
    CODEC_TYPE_AUDIO,
    CODEC_ID_MP4ALS,
    sizeof(ALSDecContext),
    decode_init,
    NULL,
    decode_end,
    decode_frame,
    .flush = flush,
    .capabilities = CODEC_CAP_SUBFRAMES,
    .long_name = NULL_IF_CONFIG_SMALL("MPEG-4 Audio Lossless Coding (ALS)"),
};

