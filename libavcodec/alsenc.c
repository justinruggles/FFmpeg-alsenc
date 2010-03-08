/*
 * MPEG-4 ALS encoder
 * Copyright (c) 2010 Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
 * Copyright (c) 2010 Justin Ruggles <justin.ruggles@gmail.com>
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
 * @file libavcodec/alsenc.c
 * MPEG-4 ALS encoder
 * @author Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
 * @author Justin Ruggles <justin.ruggles@gmail.com>
 */


#define DEBUG


#include "als.h"
#include "als_data.h"

#include "avcodec.h"
#include "dsputil.h"
#include "put_bits.h"
#define LPC_USE_DOUBLE
#include "lpc.h"
#include "mpeg4audio.h"
#include "audioconvert.h"


/** Total size of fixed-size fields in ALSSpecificConfig */
#define ALS_SPECIFIC_CFG_SIZE  30


/** Estimates Rice parameters using sum of unsigned residual samples */
#define RICE_PARAM_ALGORITHM_ESTIMATE   0
/** Calculates Rice parameters using a search algorithm based on exact bit count */
#define RICE_PARAM_ALGORITHM_EXACT      1

/** Estimates Rice sub-block partitioning using sum of unsigned residual samples */
#define RICE_BIT_COUNT_ALGORITHM_ESTIMATE   0
/** Calculates Rice sub-block partitioning using an exact bit count */
#define RICE_BIT_COUNT_ALGORITHM_EXACT      1

/** Find optimal block partitioning using Full-Search */
#define BS_ALGORITHM_FULL_SEARCH    1
/** Find optimal block partitioning using Bottom-Up */
#define BS_ALGORITHM_BOTTOM_UP      0

/** Use writing into a temporary buffer to determine exact block size including overhead */
#define BS_DETERMINE_SIZE_BY_WRITE  1
/** Use bit counting using RICE_BIT_COUNT_ALGORITHM_* and overhead estimation */
#define BS_DETERMINE_SIZE_BY_COUNT  0


// probably mergeable or the very same as ALSBlockData from the decoder
typedef struct {
    int constant;                   ///< indicates constant block values
    int32_t constant_value;         ///< if constant, this is the value
    unsigned int length;            ///< length of the block in # of samples
    unsigned int sub_blocks;        ///< number of entropy coding sub-blocks in this block
    unsigned int rice_param[4];     ///< rice parameters to encode the residuals
                                    ///< of this block in case of not bgmc
    unsigned int opt_order;         ///< prediction order for this block
    int32_t *q_parcor_coeff;        ///< 7-bit quantized PARCOR coefficients
    unsigned int js_block;          ///< indicates actual usage of joint-stereo coding
    unsigned int shift_lsbs;        ///< number of bits the samples have been right shifted
    int32_t *res_ptr;               ///< points to the first residual for this block
    int32_t *smp_ptr;               ///< points to the first raw sample for this block
} ALSBlock;


typedef struct {
    AVCodecContext *avctx;
    ALSSpecificConfig sconf;
    PutBitContext pb;
    DSPContext dsp;
    unsigned int cur_frame_length;   ///< length of the current frame, in samples
    int js_switch;                   ///< force joint-stereo in case of MCC
    int *independent_bs;             ///< array containing independent_bs flag for each channel
    int32_t *raw_buffer;             ///< buffer containing all raw samples of the frame plus max_order samples from the previous frame (or zeroes) for all channels
    int32_t **raw_samples;           ///< pointer to the beginning of the current frame's samples in the buffer for each channel
    int32_t *res_buffer;             ///< buffer containing all residual samples of the frame plus max_order samples from the previous frame (or zeroes) for all channels
    int32_t **res_samples;           ///< pointer to the beginning of the current frame's samples in the buffer for each channel
    uint32_t *bs_info;               ///< block-partitioning used for the current frame
    int *num_blocks;                 ///< number of blocks used for the block partitioning
    unsigned int *bs_sizes_buffer;   ///< buffer containing all block sizes for all channels
    unsigned int **bs_sizes;         ///< pointer to the beginning of the channel's block sizes for each channel
#if BS_DETERMINE_SIZE_BY_WRITE
    int32_t *bs_tmp_buffer;          ///< buffer for temporarily writing a block to determine the real block size including block overhead
#endif
    ALSBlock *block_buffer;          ///< buffer containing all ALSBlocks for each channel
    ALSBlock **blocks;               ///< array of 32 ALSBlock pointers per channel pointing into the block_buffer
    int32_t *q_parcor_coeff_buffer;  ///< buffer containing 7-bit PARCOR coefficients for all blocks in all channels
    int32_t *r_parcor_coeff;         ///< scaled 21-bit quantized PARCOR coefficients for the current block
    int32_t *lpc_coeff;              ///< LPC coefficients for the current block
    unsigned int max_rice_param;     ///< maximum Rice param, depends on sample depth
    double *acf_window;              ///< pre-calculated autocorrelation window
} ALSEncContext;


static int write_specific_config(AVCodecContext *avctx);
static void gen_sizes(ALSEncContext *ctx, unsigned int channel, int stage);


/** Converts an array of channel-interleaved samples into
 *  multiple arrays of samples per channel
 */
static void deinterleave_raw_samples(ALSEncContext *ctx, void *data)
{
    unsigned int sample, c, shift;

    // transform input into internal format
    #define DEINTERLEAVE_INPUT(bps)                                \
    {                                                              \
        int##bps##_t *src = (int##bps##_t*) data;                  \
        shift = bps - ctx->avctx->bits_per_raw_sample;             \
        for (sample = 0; sample < ctx->cur_frame_length; sample++) \
            for (c = 0; c < ctx->avctx->channels; c++)             \
                ctx->raw_samples[c][sample] = (*src++) << shift;   \
    }

    if (ctx->avctx->bits_per_raw_sample <= 16) {
        DEINTERLEAVE_INPUT(16)
    } else {
        DEINTERLEAVE_INPUT(32)
    }
}


/** Chooses the appropriate method for difference channel coding
 *  for the current frame
 */
static void select_difference_coding_mode(ALSEncContext *ctx)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int c;

    // to be implemented
    // depends on sconf->joint_stereo and sconf->mc_coding
    // selects either joint_stereo or mcc mode
    // sets js_switch (js && mcc) and/or independent_bs
    // both parameters to be added to the context...
    // until mcc mode is implemented, syntax could be tested
    // by using js_switch all the time if mcc is enabled globally
    //
    // while not implemented, output most simple mode:
    //  -> 0 if !mcc and while js is not implemented
    //  -> 1 if  mcc (would require js to be implemented and
    //                correct output of mcc frames/block)

    ctx->js_switch = sconf->mc_coding;

    if (!sconf->mc_coding || ctx->js_switch)
        for (c = 0; c < avctx->channels; c++)
            ctx->independent_bs[c] = 1;
}


/** Recursively parses a given block partitioning and
 *  sum up all block sizes used to get the overall bit count
 */
static void parse_bs_size(const uint32_t bs_info, unsigned int n,
                          unsigned int *bs_sizes, unsigned int *bit_count)
{
    if (n < 31 && ((bs_info << n) & 0x40000000)) {
        // if the level is valid and the investigated bit n is set
        // then recursively check both children at bits (2n+1) and (2n+2)
        n   *= 2;
        parse_bs_size(bs_info, n + 1, bs_sizes, bit_count);
        parse_bs_size(bs_info, n + 2, bs_sizes, bit_count);
    } else {
        // else the bit is not set or the last level has been reached
        // (bit implicitly not set)
        (*bit_count) += bs_sizes[n];
    }
}


/** Recursively parses a given block partitioning and
 *  set all nodes to zero
 */
static void parse_bs_zero(uint32_t *bs_info, unsigned int n)
{
    if (n < 31) {
        // if the level is valid set this bit and
        // all children to zero
        *bs_info &= ~(1 << (30 - n));
        n        *= 2;
        parse_bs_zero(bs_info, n + 1);
        parse_bs_zero(bs_info, n + 2);
    }
}


/** Successively merging the subblocks of the frame until
 *  the minimal bit count for the frame is found.
 *  Using Full-Search strategy.
 */
static void merge_bs_fullsearch(ALSEncContext *ctx, unsigned int n,
                                 unsigned int c1, unsigned int c2)
{
#define GET_BIT(pos) ((*bs_info & (1 << (30 - pos))) > 0)
    uint32_t *bs_info = &ctx->bs_info[c1];

    if (n < 31 && ((*bs_info << n) & 0x40000000)) {
        // if the level is valid and the investigated bit n is set
        // then recursively check both children at bits (2n+1) and (2n+2)
        unsigned int *sizes_c1 = ctx->bs_sizes[c1];
        unsigned int *sizes_c2 = ctx->bs_sizes[c2];
        unsigned int a         = 2 * n + 1;
        unsigned int b         =     a + 1;
        unsigned int sum_a     = 0;
        unsigned int sum_b     = 0;
        unsigned int sum_n     = sizes_c1[n];

        if (GET_BIT(a)) {
            merge_bs_fullsearch(ctx, a, c1, c2);
        }

        if (GET_BIT(b)) {
            merge_bs_fullsearch(ctx, b, c1, c2);
        }


        parse_bs_size(*bs_info, a, sizes_c1, &sum_a);
        parse_bs_size(*bs_info, b, sizes_c1, &sum_b);

        if (c1 != c2) {
            sum_n += sizes_c2[n];
            parse_bs_size(*bs_info, a, sizes_c2, &sum_a);
            parse_bs_size(*bs_info, b, sizes_c2, &sum_b);
        }

        if (sum_a + sum_b > sum_n) {
            parse_bs_zero(bs_info, n);
            if (c1 != c2) {
                ctx->bs_info[c2] = *bs_info;
            }
        }

        if (GET_BIT(a) && GET_BIT(b)) {
            merge_bs_fullsearch(ctx, a, c1, c2);
            merge_bs_fullsearch(ctx, b, c1, c2);
        }
    }
}


/** Successively merging the subblocks of the frame until
 *  the minimal bit count for the frame is found.
 *  Using Bottom-Up strategy.
 */
static void merge_bs_bottomup(ALSEncContext *ctx, unsigned int n,
                                 unsigned int c1, unsigned int c2)
{
    uint32_t *bs_info = &ctx->bs_info[c1];

    if (n < 31 && ((*bs_info << n) & 0x40000000)) {
        // if the level is valid and the investigated bit n is set
        // then recursively check both children at bits (2n+1) and (2n+2)
        unsigned int *sizes_c1 = ctx->bs_sizes[c1];
        unsigned int *sizes_c2 = ctx->bs_sizes[c2];
        unsigned int a         = 2 * n + 1;
        unsigned int b         =     a + 1;
        unsigned int sum_a     = 0;
        unsigned int sum_b     = 0;
        unsigned int sum_n     = sizes_c1[n];

        if (GET_BIT(a) && GET_BIT(b)) {
            merge_bs_bottomup(ctx, a, c1, c2);
            merge_bs_bottomup(ctx, b, c1, c2);
        }

        if (!GET_BIT(a) && !GET_BIT(b)) {
            sum_a += sizes_c1[a];
            sum_b += sizes_c1[b];

            if (c1 != c2) {
                sum_n += sizes_c2[n];
                sum_a += sizes_c2[a];
                sum_b += sizes_c2[b];
            }

            if (sum_a + sum_b > sum_n) {
                parse_bs_zero(bs_info, n);
                if (c1 != c2) {
                    ctx->bs_info[c2] = *bs_info;
                }
            }
        }
    }
#undef GET_BIT(pos)
}


/** Reads block switching field if necessary and sets actual block sizes.
 *  Also assures that the block sizes of the last frame correspond to the
 *  actual number of samples.
 */
static void get_block_sizes(ALSEncContext *ctx,
                            uint32_t *bs_info,
                            unsigned int c1, unsigned int c2)
{
    ALSSpecificConfig *sconf     = &ctx->sconf;
    unsigned int div_blocks[32];
    unsigned int *ptr_div_blocks = div_blocks;
    unsigned int b;
    int32_t *res_ptr = ctx->res_samples[c1];
    int32_t *smp_ptr = ctx->raw_samples[c1];

    ALSBlock *block = ctx->blocks[c1];

    ctx->num_blocks[c1] = 0;

    ff_als_parse_bs_info(*bs_info, 0, 0, &ptr_div_blocks, &ctx->num_blocks[c1]);

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

    for (b = 0; b < ctx->num_blocks[c1]; b++) {
        div_blocks[b]  = ctx->sconf.frame_length >> div_blocks[b];
        block->length  = div_blocks[b];
        block->res_ptr = res_ptr;
        block->smp_ptr = smp_ptr;
        res_ptr += div_blocks[b];
        smp_ptr += div_blocks[b];
        block++;
    }

    if (ctx->cur_frame_length != sconf->frame_length) {
        unsigned int remaining = ctx->cur_frame_length;

        for (b = 0; b < ctx->num_blocks[c1]; b++) {
            if (remaining <= div_blocks[b]) {
                ctx->blocks[c1][b].length = remaining;
                ctx->num_blocks[c1] = b + 1;
                break;
            }
            remaining -= ctx->blocks[c1][b].length;
        }
    }

    if (c1 != c2) {
        res_ptr = ctx->res_samples[c2];
        smp_ptr = ctx->raw_samples[c2];
        ctx->num_blocks[c2] = ctx->num_blocks[c1];
        block = ctx->blocks[c2];

        for (b = 0; b < ctx->num_blocks[c1]; b++) {
            block->length  = div_blocks[b];
            block->res_ptr = res_ptr;
            block->smp_ptr = smp_ptr;
            res_ptr += div_blocks[b];
            smp_ptr += div_blocks[b];
            block++;
        }
    }
}


/** Selects the best block-partitioning for the current frame
 *  depending on the chosen algorithm and sets the block sizes
 *  accordingly
 */
static void get_partition(ALSEncContext *ctx, unsigned int c1, unsigned int c2)
{
    if(BS_ALGORITHM_BOTTOM_UP) {
        merge_bs_bottomup(ctx, 0, c1, c2);
    } else {
        merge_bs_fullsearch(ctx, 0, c1, c2);
    }

    get_block_sizes(ctx, &ctx->bs_info[c1], c1, c2);
}



/** Subdivide the frame into smaller blocks
 */
static void block_partitioning(ALSEncContext *ctx)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int c;

    if (sconf->block_switching && !(ctx->cur_frame_length & 1)) { // even number of samples only by now

        // generate all block sizes for this frame
        for (c = 0; c < avctx->channels; c++) {
            gen_sizes(ctx, c, 0);
        }

        // find the best partitioning for each channel
        if (!sconf->mc_coding || ctx->js_switch) {
            for (c = 0; c < avctx->channels; c++) {
                if (ctx->independent_bs[c]) {
                    get_partition(ctx, c, c);
                } else {
                    get_partition(ctx, c, c + 1);
                    c++;
                }
            }
        } else {

            // MCC: to be implemented

        }

    } else {

        // generate result for no block-switching,
        // one block per channel as long as the frame

        for (c = 0; c < avctx->channels; c++) {
            ctx->num_blocks[c]        = 1;
            ctx->blocks[c][0].length  = ctx->cur_frame_length;
            ctx->blocks[c][0].res_ptr = ctx->res_samples[c];
            ctx->blocks[c][0].smp_ptr = ctx->raw_samples[c];
        }
    }
}


static inline int rice_count(int v, int k)
{
    unsigned int v0;

    if (v < -INT16_MIN || v > INT16_MAX) {
        v0 = (unsigned int)((2LL*v) ^ (int64_t)(v>>31));
    } else {
        v0 = (2 * v) ^ (v >> 31);
    }

    return (v0 >> k) + 1 + k;
}


static inline int set_sr_golomb_als(PutBitContext *pb, int v, int k)
{
    int q;
    unsigned int v0;

    /* remap to unsigned */
    if (v < -INT16_MIN || v > INT16_MAX) {
        v0 = (unsigned int)((2LL*v) ^ (int64_t)(v>>31));
    } else {
        v0 = (2 * v) ^ (v >> 31);
    }

    /* write quotient in zero-terminated unary */
    q = (v0 >> k) + 1;

    /* protect from buffer overwrite */
    if (put_bits_count(pb) + q + k > pb->size_in_bits) {
        return -1;
    }

    while (q > 31) {
        put_bits(pb, 31, 0x7FFFFFFF);
        q -= 31;
    }
    put_bits(pb, q, ((1<<q)-1)^1);

    /* write remainder using k bits */
    if (k)
        put_bits(pb, k, (v0 >> 1) - (((v0 >> k)-(!(v0&1))) << (k-1)));

    return 0;
}


/** Write a given block of a given channel
 */
static int write_block(ALSEncContext *ctx, ALSBlock *block,
                       unsigned int c, unsigned int b)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    PutBitContext *pb        = &ctx->pb;
    unsigned int i;
    int start;


    // block_type
    put_bits(pb, 1, !block->constant);


    if (block->constant) {
        // const_block
        put_bits(pb, 1, block->constant_value != 0);


        // js_block
        put_bits(pb, 1, block->js_block);
        // reserved
        put_bits(pb, 5, 0);


        if (block->constant_value) {
            int const_val_bits = sconf->floating ? 24 : avctx->bits_per_raw_sample;
            if (const_val_bits == 32)
                put_bits32(pb, block->constant_value);
            else
                put_sbits(pb, const_val_bits, block->constant_value);
        }
    } else {
        int32_t *res_ptr;
        int sb, sb_length;

        // js_block
        put_bits(pb, 1, block->js_block);


        // ec_sub
        if (sconf->sb_part) {
            if (sconf->bgmc)
                put_bits(pb, 2, av_log2(block->sub_blocks));
            else
                put_bits(pb, 1, block->sub_blocks > 1);
        }


        // s[k], sx[k]
        if (sconf->bgmc) {

             // to be implemented

        } else {
            put_bits(pb, 4 + (avctx->bits_per_raw_sample > 16), block->rice_param[0]);

            for (sb = 1; sb < block->sub_blocks; sb++) {
                if (set_sr_golomb_als(pb, block->rice_param[sb] - block->rice_param[sb-1], 0))
                    return -1;
            }
        }


        // shift_lsbs && shift_pos
        put_bits(pb, 1, block->shift_lsbs > 0);

        if (block->shift_lsbs)
            put_bits(pb, 4, block->shift_lsbs);


        // opt_order && quant_cof
        if (!sconf->rlslms) {
            // opt_order
            if (sconf->adapt_order) {
                int opt_order_length = av_ceil_log2(av_clip((block->length >> 3) - 1,
                                                    2, sconf->max_order + 1));
                put_bits(pb, opt_order_length, block->opt_order);
            }


            // for each quant_cof, put(quant_cof) in rice code
            if (sconf->coef_table == 3) {
                for (i = 0; i < block->opt_order; i++)
                    put_bits(pb, 7, 64 + block->q_parcor_coeff[i]);
            } else {
                // write coefficient 0 to 19
                int next_max_order = FFMIN(block->opt_order, 20);
                for (i = 0; i < next_max_order; i++) {
                    int rice_param = ff_als_parcor_rice_table[sconf->coef_table][i][1];
                    int offset     = ff_als_parcor_rice_table[sconf->coef_table][i][0];
                    if (set_sr_golomb_als(pb, block->q_parcor_coeff[i] - offset, rice_param))
                        return -1;
                }

                // write coefficients 20 to 126
                next_max_order = FFMIN(block->opt_order, 127);
                for (; i < next_max_order; i++)
                    set_sr_golomb_als(pb, block->q_parcor_coeff[i] - (i & 1), 2);

                // write coefficients 127 to opt_order
                for (; i < block->opt_order; i++)
                    set_sr_golomb_als(pb, block->q_parcor_coeff[i], 1);
            }
        }


        // LPTenable && LTPgain && LTPlag
        if (sconf->long_term_prediction) {
            // to be implemented
            //
            // until then, disable it in the block
            // TODO: add ltp-flag into block struct when
            //       LTP becomes reality

            put_bits(pb, 1, 0);
        }


        // write residuals
        // for now, all frames are RA frames, so use progressive prediction for
        // the first 3 residual samples, up to opt_order

        res_ptr = block->res_ptr;
        sb_length = block->length / block->sub_blocks;

        for (sb = 0; sb < block->sub_blocks; sb++) {
            start = 0;
            if (!b && !sb) { // should be: if (!ra_block) or: if (!b && ra_frame) as soon as non-ra frames are supported
                int len = FFMIN(block->opt_order, sb_length);
                if (len > 0) {
                    if (set_sr_golomb_als(pb, *res_ptr++, avctx->bits_per_raw_sample-4))
                        return -1;
                    start++;
                    if (len > 1) {
                        if (set_sr_golomb_als(pb, *res_ptr++, FFMIN(block->rice_param[sb]+3, ctx->max_rice_param)))
                            return -1;
                        start++;
                        if (len > 2) {
                            if (set_sr_golomb_als(pb, *res_ptr++, FFMIN(block->rice_param[sb]+1, ctx->max_rice_param)))
                                return -1;
                            start++;
                        }
                    }
                }
            }
            for (i = start; i < sb_length; i++) {
                if (set_sr_golomb_als(pb, *res_ptr++, block->rice_param[sb]))
                    return -1;
            }
        }
    }


    if (!sconf->mc_coding || ctx->js_switch)
        align_put_bits(pb);


    return 0;
}


/** Write the frame
 */
static int write_frame(ALSEncContext *ctx, uint8_t *frame, int buf_size)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int c, b;
    int ret;

    init_put_bits(&ctx->pb, frame, buf_size);


    // make space for ra_unit_size
    if (sconf->ra_flag == RA_FLAG_FRAMES && sconf->ra_distance == 1) {
        // TODO: maybe keep frame count and allow other RA distances if API will allow
        put_bits32(&ctx->pb, 0);
    }


    // js_switch
    if (ctx->js_switch) {
        // to be implemented
        // not yet supported anyway
    }


    // write blocks
    if (!sconf->mc_coding || ctx->js_switch) {
        for (c = 0; c < avctx->channels; c++) {
            if (sconf->block_switching) {
                unsigned int bs_info_len = 1 << (sconf->block_switching + 2);
                uint32_t bs_info         = ctx->bs_info[c];

                if (bs_info_len == 32)
                    put_bits32(&ctx->pb, bs_info);
                else
                    put_bits(&ctx->pb, bs_info_len, bs_info >> (32 - bs_info_len));
            }

            for (b= 0; b < ctx->num_blocks[c]; b++) {
                if (ctx->independent_bs[c]) {
                    ret = write_block(ctx, &ctx->blocks[c][b], c, b);
                    if (ret < 0)
                        return ret;
                } else {
                    // write channel & channel+1
                }
            }
        }
    } else {

        // MCC: to be implemented

    }


    flush_put_bits(&ctx->pb);
    ret = put_bits_count(&ctx->pb) >> 3;

    // write ra_unit_size
    if (sconf->ra_flag == RA_FLAG_FRAMES && sconf->ra_distance == 1) {
        AV_WB32(frame, ret);
    }


    return ret;
}


static void quantize_parcor_coeffs(const double *parcor, int order,
                                   int32_t *q_parcor, int32_t *r_parcor)
{
    int i;

    // first coefficient
    q_parcor[0] = (int)floor(64.0 * (sqrt(2.0*(parcor[0]+1.0)) - 1.0));
    q_parcor[0] = av_clip(q_parcor[0], -64, 63);
    r_parcor[0] = 32 * ff_als_parcor_scaled_values[q_parcor[0] + 64];

    if (order > 1) {
        // second coefficient
        q_parcor[1] = (int)floor(64.0 * ((sqrt(2.0*(1.0-parcor[1]))) - 1.0));
        q_parcor[1] = av_clip(q_parcor[1], -64, 63);
        r_parcor[1] = -32 * ff_als_parcor_scaled_values[q_parcor[1] + 64];

        // remaining coefficients
        for (i = 2; i < order; i++) {
            q_parcor[i] = (int)floor(64.0 * parcor[i]);
            q_parcor[i] = av_clip(q_parcor[i], -64, 63);
            r_parcor[i] = (q_parcor[i] << 14) + (1 << 13);
        }
    }
}


static unsigned int subblock_rice_count_exact(const int32_t *res_ptr,
                                              int sb_length, int rice_param,
                                              int max_param, int ra_subblock,
                                              int opt_order)
{
    unsigned int count = 0;
    int i = 0;

    if (ra_subblock) {
        int len = FFMIN(opt_order, sb_length);
        if (len > 0) {
            int v = *res_ptr++;
            count += rice_count(v, max_param - 3);
            i++;
            if (len > 1) {
                v = *res_ptr++;
                count += rice_count(v, FFMIN(rice_param+3, max_param));
                i++;
                if (len > 2) {
                    v = *res_ptr++;
                    count += rice_count(v, FFMIN(rice_param+1, max_param));
                    i++;
                }
            }
        }
    }
    for (; i < sb_length; i++) {
        int v = *res_ptr++;
        count += rice_count(v, rice_param);
    }

    return count;
}


static unsigned int block_rice_count_exact(const int32_t *res_ptr,
                                           int block_length, int sub_blocks,
                                           int *rice_param, int max_param,
                                           int ra_block, int opt_order)
{
    unsigned int count = 0;
    int sb_length, sb;

    sb_length = block_length / sub_blocks;

    for (sb = 0; sb < sub_blocks; sb++) {
        count += subblock_rice_count_exact(res_ptr, sb_length, rice_param[sb],
                                           max_param, ra_block && !sb, opt_order);
        if (!sb)
            count += 4 + (max_param > 15);
        else
            count += rice_count(rice_param[sb]-rice_param[sb-1], 0);

        res_ptr += sb_length;
    }

    return count;
}


#define rice_encode_count(sum, n, k) (((n)*((k)+1))+((sum-(n>>1))>>(k)))


static inline int optimal_rice_param(uint64_t sum, int length, int max_param)
{
    int k;

    if (sum <= length >> 1)
        return 0;

    if (max_param > 15) {
        sum = FFMAX((sum - (length >> 1)) / length, 1);
        k = (int)floor(log(sum) / log(2));
    } else {
        unsigned int sum1 = sum;
        sum1 = sum1 - (length >> 1);
        k = av_log2(length < 256 ? FASTDIV(sum1, length) : sum1 / length);
    }

    return FFMIN(k, max_param);
}


static int find_block_rice_params_est(const int32_t *res_ptr, int block_length,
                                      int max_param, int sb_part, int ra_block,
                                      int opt_order, int count_algorithm,
                                      int *sub_blocks, int *rice_param)
{
    int i, sb, sb_max, sb_length;
    uint64_t sum[5] = {0,};
    int param[5];
    unsigned int count1, count4;

    if (!sb_part || block_length & 0x3 || block_length < 16)
        sb_max = 1;
    else
        sb_max = 4;
    sb_length = block_length / sb_max;

    for (sb = 0; sb < sb_max; sb++) {
        const int32_t *res_ptr1 = res_ptr + (sb * sb_length);
        for (i = 0; i < sb_length; i++) {
            int v = *res_ptr1++;
            if (max_param > 15) {
                unsigned int v0 = (unsigned int)((2LL*v) ^ (int64_t)(v>>31));
                sum[sb] += v0;
            } else {
                v = (2 * v) ^ (v >> 31);
                sum[sb] += v;
            }
        }
        sum[4] += sum[sb];

        param[sb] = optimal_rice_param(sum[sb], sb_length, max_param);
    }

    param[4] = optimal_rice_param(sum[4], block_length, max_param);

    if (count_algorithm == RICE_BIT_COUNT_ALGORITHM_EXACT) {
        count1 = block_rice_count_exact(res_ptr, block_length, 1, &param[4],
                                        max_param, ra_block, opt_order);
    } else {
        count1 = rice_encode_count(sum[4], block_length, param[4]);
        count1 += 4 + (max_param > 15);
    }

    if (sb_max == 1 || ((param[0] == param[1]) && (param[1] == param[2]) &&
        (param[2] == param[3]))) {
        *sub_blocks = 1;
        rice_param[0] = param[4];
        return count1;
    }

    if (count_algorithm == RICE_BIT_COUNT_ALGORITHM_EXACT) {
        count4 = block_rice_count_exact(res_ptr, block_length, 4, param,
                                        max_param, ra_block, opt_order);
    } else {
        count4 = 0;
        for (sb = 0; sb < sb_max; sb++) {
            count4 += rice_encode_count(sum[sb], sb_length, param[sb]);

            if (!sb)
                count4 += 4 + (max_param > 15);
            else
                count4 += rice_count(param[sb] - param[sb-1], 0);
        }
    }

    if (count1 <= count4) {
        *sub_blocks = 1;
        rice_param[0] = param[4];
    } else {
        *sub_blocks = 4;
        rice_param[0] = param[0];
        rice_param[1] = param[1];
        rice_param[2] = param[2];
        rice_param[3] = param[3];
    }

    if (count_algorithm == RICE_BIT_COUNT_ALGORITHM_ESTIMATE)
        return block_rice_count_exact(res_ptr, block_length, *sub_blocks,
                                      rice_param, max_param, ra_block,
                                      opt_order);
    return FFMIN(count1, count4);
}


static int find_block_rice_params_exact(const int32_t *res_ptr, int block_length,
                                        int max_param, int sb_part, int ra_block,
                                        int opt_order, int *sub_blocks,
                                        int *rice_param)
{
    unsigned int count[5][32] = {{0,}};
    int param[5];
    int k, step, sb, sb_max, sb_length;
    int best_k;
    unsigned int count1, count4;

    if (!sb_part || block_length & 0x3 || block_length < 16)
        sb_max = 1;
    else
        sb_max = 4;
    sb_length = block_length / sb_max;

    for (sb = 0; sb < sb_max; sb++) {
        /* start 1/3 distance between 0 and max_param */
        k = max_param / 3;
        count[sb][k] = subblock_rice_count_exact(res_ptr + (sb * sb_length),
                                                sb_length, k, max_param,
                                                !sb && ra_block, opt_order);
        k++;
        count[sb][k] = subblock_rice_count_exact(res_ptr + (sb * sb_length),
                                                sb_length, k, max_param,
                                                !sb && ra_block, opt_order);
        if (count[sb][k] < count[sb][k-1]) {
            best_k = k;
            step = 1;
            k++;
        } else {
            best_k = k - 1;
            step = -1;
            k -= 2;
        }

        for (; k >= 0 && k <= max_param; k += step) {
            count[sb][k] = subblock_rice_count_exact(res_ptr + (sb * sb_length),
                                                    sb_length, k, max_param,
                                                    !sb && ra_block, opt_order);

            if (count[sb][k] < count[sb][best_k]) {
                best_k = k;
            } else {
                break;
            }
        }
        param[sb] = best_k;
    }


    /* if sub-block partitioning is not used, stop here */
    if (sb_max == 1) {
        *sub_blocks = 1;
        rice_param[0] = param[0];
        return count[0][param[0]] + 4 + (max_param > 15);
    }


    /* start 1/3 distance between 0 and max_param */
    k = max_param / 3;
    count[4][k] = subblock_rice_count_exact(res_ptr, block_length, k, max_param,
                                            ra_block, opt_order);
    k++;
    count[4][k] = subblock_rice_count_exact(res_ptr, block_length, k, max_param,
                                            ra_block, opt_order);
    if (count[4][k] < count[4][k-1]) {
        best_k = k;
        step = 1;
        k++;
    } else {
        best_k = k - 1;
        step = -1;
        k -= 2;
    }

    for (; k >= 0 && k <= max_param; k += step) {
        count[4][k] = subblock_rice_count_exact(res_ptr, block_length, k,
                                                max_param, ra_block, opt_order);

        if (count[4][k] < count[4][best_k]) {
            best_k = k;
        } else {
            break;
        }
    }
    param[4] = best_k;


    count1  = count[4][param[4]];
    count1 += 4 + (max_param > 15);

    count4 = 0;
    for (sb = 0; sb < sb_max; sb++) {
        count4 += count[sb][param[sb]];
        if (!sb)
            count4 += 4 + (max_param > 15);
        else
            count4 += rice_count(param[sb]-param[sb-1], 0);
    }

    if (count1 <= count4) {
        *sub_blocks = 1;
        rice_param[0] = param[4];
        return count1;
    } else {
        *sub_blocks = 4;
        rice_param[0] = param[0];
        rice_param[1] = param[1];
        rice_param[2] = param[2];
        rice_param[3] = param[3];
        return count4;
    }
}


/**
 * Calculate optimal sub-block division and Rice parameters for a block.
 * @param[in] param_algorithm   which algorithm to use for determining Rice parameters
 * @param[in] count_algorithm   which bit count algorithm to use for determining sb_part
 * @param[in] res_ptr           residual samples
 * @param[in] block_length      number of samples in the block
 * @param[in] max_param         maximum Rice parameter allowed
 * @param[in] sb_part           indicates if entropy coding partitioning is used
 * @param[in] ra_block          indicates if this is a random access block
 * @param[in] opt_order         LPC order
 * @param[out] sub_blocks       optimal number of sub-blocks
 * @param[out] rice_param       optimal Rice parameter(s)
 * @return                      estimated number of bits used for residuals and rice params
 */
static int find_block_rice_params(int param_algorithm, int count_algorithm,
                                  const int32_t *res_ptr,
                                  int block_length, int max_param, int sb_part,
                                  int ra_block, int opt_order, int *sub_blocks,
                                  int *rice_param)
{
    int bit_count = -1;

    if (param_algorithm == RICE_PARAM_ALGORITHM_ESTIMATE) {
        bit_count = find_block_rice_params_est(res_ptr, block_length, max_param,
                                               sb_part, ra_block, opt_order,
                                               count_algorithm, sub_blocks,
                                               rice_param);
    } else if (param_algorithm == RICE_PARAM_ALGORITHM_EXACT) {
        bit_count = find_block_rice_params_exact(res_ptr, block_length,
                                                 max_param, sb_part, ra_block,
                                                 opt_order, sub_blocks,
                                                 rice_param);
    }

    return bit_count;
}


static inline int estimate_best_order(int32_t *r_parcor, int order)
{
    int i;
    for (i = order-1; i > 0; i--) {
        if (abs(r_parcor[i]) > 104858) // 104858 = 0.10 * (1 << 20)
            break;
    }
    return i+1;
}


static void calc_short_term_prediction(ALSEncContext *ctx, ALSBlock *block,
                                       unsigned int c, unsigned int b,
                                       int opt_order)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    int i, j;

    int32_t *res_ptr = block->res_ptr;
    int32_t *smp_ptr = block->smp_ptr;

#define LPC_PREDICT_SAMPLE(lpc, smp_ptr, res_ptr, order)\
{\
    int64_t y = 1 << 19;\
    for (j = 1; j <= order; j++)\
        y += (int64_t)lpc[j-1] * (smp_ptr)[-j];\
    y = *(smp_ptr++) + (y >> 20);\
    if (y < INT32_MIN || y > INT32_MAX)\
        av_log(ctx->avctx, AV_LOG_ERROR, "32-bit overflow in LPC prediction\n");\
    *(res_ptr++) = y;\
}

    i = 0;
    if (!b) { // should be: if (!ra_block) or: if (!b && ra_frame) as soon as non-ra frames are supported
        int ra_opt_order = FFMIN(opt_order, block->length);

        // copy first residual sample verbatim
        *(res_ptr++) = *(smp_ptr++);

        // progressive prediction
        ff_als_parcor_to_lpc(0, ctx->r_parcor_coeff, ctx->lpc_coeff);
        for (i = 1; i < ra_opt_order; i++) {
            LPC_PREDICT_SAMPLE(ctx->lpc_coeff, smp_ptr, res_ptr, i);
            ff_als_parcor_to_lpc(i, ctx->r_parcor_coeff, ctx->lpc_coeff);
        }
        // zero unused coeffs for small frames since they are all written
        // to the bitstream if adapt_order is not used.
        if (!sconf->adapt_order) {
            for (; i < sconf->max_order; i++)
                block->q_parcor_coeff[i] = ctx->r_parcor_coeff[i] = 0;
        }
    } else {
        for (j = 0; j < opt_order; j++)
            ff_als_parcor_to_lpc(j, ctx->r_parcor_coeff, ctx->lpc_coeff);
    }
    // remaining residual samples
    for (; i < block->length; i++) {
        LPC_PREDICT_SAMPLE(ctx->lpc_coeff, smp_ptr, res_ptr, opt_order);
    }
}


/** Encode a given block of a given channel
 */
static void find_block_params(ALSEncContext *ctx, ALSBlock *block,
                              unsigned int c, unsigned int b)
{
    ALSSpecificConfig *sconf = &ctx->sconf;

    int32_t *res_ptr = block->res_ptr;
    int32_t *smp_ptr = block->smp_ptr;

    // check for constant block
    // to be implemented
    //
    // Read 1st sample, then check remaining samples.  Benchmark
    // with and w/o the check.  Maybe can be skipped for only the fastest mode.
    //
    // just say it is non-const while not implemented

    block->constant       = 0;
    block->constant_value = 0;


    // shifting samples:
    // to be implemented
    //
    // Again, maybe skip in fastest mode.  The flags
    // to indicate use of these sorts of searches should be in the context and
    // set during init based on compression_level.
    //
    // determine if the samples can be shifted
    // while not implemented, don't shift anyway

    block->shift_lsbs = 0;


    // difference coding(?):
    // to be implemented
    //
    // choose if this block can be difference coded if joint-stereo is enabled
    // (and the block can be encoded in a channel pair)
    // while not implemented, don't indicate js

    block->js_block = 0;


    // short-term prediction:
    // use the mode chosen at encode_init() to find optimal parameters
    //
    // LPC / PARCOR coefficients to be stored in context
    // they depend on js_block and opt_order which may be changing later on

    if (sconf->max_order) {
        double autoc[sconf->max_order+1];
        double parcor[sconf->max_order];

        // calculate PARCOR coefficients
        if (block->length == sconf->frame_length)
            ctx->dsp.lpc_compute_autocorr(smp_ptr, ctx->acf_window,
                                          block->length, sconf->max_order,
                                          autoc);
        else
            ctx->dsp.lpc_compute_autocorr(smp_ptr, NULL, block->length,
                                          sconf->max_order, autoc);
        compute_ref_coefs(autoc, sconf->max_order, parcor);

        // quantize PARCOR coefficients to 7-bit and reconstruct to 21-bit
        quantize_parcor_coeffs(parcor, sconf->max_order, block->q_parcor_coeff,
                               ctx->r_parcor_coeff);
    }


    // Determine optimal LPC order:
    //
    // quick estimate for LPC order. better searches will give better
    // significantly better results.
    if (sconf->max_order && sconf->adapt_order) {
        block->opt_order = estimate_best_order(ctx->r_parcor_coeff,
                                               sconf->max_order);
    } else {
        block->opt_order = sconf->max_order;
    }


    // generate residuals using parameters:

    if (block->opt_order) {
        calc_short_term_prediction(ctx, block, c, b, block->opt_order);
    } else {
        memcpy(res_ptr, smp_ptr, sizeof(*res_ptr) * block->length);
    }


    // long-term prediction:
    // to be implemented
    //
    // if enabled, search for ltp coefficients, lag, and whether ltp
    // should be enabled in this block


    // final joint or multi channel coding:
    // to be implemented
    //
    // up to now only independent coding...


    // search for rice parameter:
    find_block_rice_params(RICE_PARAM_ALGORITHM_EXACT,
                           RICE_BIT_COUNT_ALGORITHM_EXACT,
                           res_ptr, block->length,
                           ctx->max_rice_param, sconf->sb_part,
                           !b, // ra_block
                           block->opt_order,
                           &block->sub_blocks, block->rice_param);
}


#if BS_DETERMINE_SIZE_BY_COUNT
/** Very roughly estimates overhead for one block
 */
static int get_block_overhead(ALSEncContext *ctx, ALSBlock *block)
{
    int overhead = block->sub_blocks * ctx->max_rice_param // s[k]
                 + 10                            // opt_order_length
                 + block->opt_order  * 7         // quant_cof
                 + 0                             // LTP
                 + 0                             // progressively coded ra-samples
                 ;

    return overhead;
}
#endif


/** Generates all possible block sizes for all possible block-switching stages
 */
static void gen_sizes(ALSEncContext *ctx, unsigned int channel, int stage)
{
    ALSBlock *block          = ctx->blocks[channel];
    unsigned int num_blocks  = 1 << stage;
    uint32_t bs_info_tmp     = 0;
    unsigned int b;

    ctx->num_blocks[channel] = num_blocks;

    if (stage) {
        for (b = 1; b < num_blocks; b++) {
            bs_info_tmp |= (1 << (31 - b));
        }
    }

    get_block_sizes(ctx, &bs_info_tmp, channel, channel);

    for (b = 0; b < num_blocks; b++) {
        unsigned int *bs_sizes = ctx->bs_sizes[channel] + num_blocks - 1;

#if BS_DETERMINE_SIZE_BY_COUNT
        // count residuals + estimate block overhead
        find_block_params(ctx, block, channel, b);

        bs_sizes[b] += get_block_overhead(ctx, block);
#else
        // get exact bit count by writing

        // save original PutBitContext
        PutBitContext pb  = ctx->pb;

        // initialize a new buffer into ctx->pb
        init_put_bits(&ctx->pb, (uint8_t*)ctx->bs_tmp_buffer, ctx->sconf.frame_length << 3);

        // write into temporary buffer
        find_block_params(ctx, block, channel, b);

        write_block(ctx, block, channel, b);

        // get written bits
        bs_sizes[b] = put_bits_count(&ctx->pb);

        // restore original PutBitContext
        ctx->pb = pb;
#endif

        block++;
    }

    if (stage < ctx->sconf.block_switching + 2)
        gen_sizes(ctx, channel, stage + 1);
    else
        ctx->bs_info[channel] = bs_info_tmp;
}


static int encode_frame(AVCodecContext *avctx, uint8_t *frame,
                        int buf_size, void *data)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int b, c;
    int frame_data_size;

    // last frame has been encoded, update extradata
    if (!data) {
        // rewrite AudioSpecificConfig & ALSSpecificConfig to extradata
        int ret = write_specific_config(avctx);
        if (ret) {
            av_log(avctx, AV_LOG_ERROR, "Rewriting of extradata failed.\n");
            return ret;
        }
        return 0;
    }


    ctx->cur_frame_length = avctx->frame_size;

    // preprocessing
    deinterleave_raw_samples(ctx, data);
    select_difference_coding_mode(ctx);
    block_partitioning(ctx);


    // encoding loop
    if (!sconf->mc_coding || ctx->js_switch) {
        for (b= 0; b < 32; b++) {
            for (c = 0; c < avctx->channels; c++) {
                if (b >= ctx->num_blocks[c])
                    continue;

                if (ctx->independent_bs[c]) {
                    find_block_params(ctx, &ctx->blocks[c][b], c, b);
                } else {
                    // encode channel & channel+1
                }
            }
        }
    } else {

        // MCC: to be implemented

    }


    // bitstream assembly
    frame_data_size = write_frame(ctx, frame, buf_size);
    if (frame_data_size < 0)
        av_log(avctx, AV_LOG_ERROR, "Error writing frame\n");

    // update sample count
    if (frame_data_size >= 0)
        sconf->samples += ctx->cur_frame_length;

    //memset(frame, 0, buf_size);

    return frame_data_size;
}


/** Rearranges internal order of channels to optimize joint-channel coding
 */
static void channel_sorting(ALSEncContext *ctx)
{
    // to be implemented...
    // just arrange ctx->raw_samples array
    // according to specific channel order
}


/** Determines the number of samples in each frame,
 *  constant for all frames in the stream except the
 *  very last one which may differ
 */
static void frame_partitioning(ALSEncContext *ctx)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;

    // choose standard value 2048 if not user-defined
    // maybe choose an appropriate size that depends on
    // the sample rate
    if (avctx->frame_size <= 0)
        avctx->frame_size = 2048;

    // ensure a certain boundary for the frame size
    // maximum value is 0xFFFF using 16 bit in ALSSpecificConf
    avctx->frame_size = av_clip(avctx->frame_size, 1024, 0xFFFF);

    // enforce a frame length that is a power of 2?
    // or discard block-switching if it is not?
    // if block-switching is enabled by default
    // then do check
    // exceptions for last frame which might consist
    // of an arbitrary number of samples
    // do special block-switching in that case like the
    // reference enc? (2-2-1-0, refer to decoder)

    sconf->frame_length = avctx->frame_size;
}


static av_cold int get_specific_config(AVCodecContext *avctx)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;


    // total number of samples unknown
    // TODO: keep track of actual number of samples written
    //       and update after the last frame has been encoded
    sconf->samples = 0xFFFFFFFF;


    // determine sample format
    switch (avctx->sample_fmt) {
    case SAMPLE_FMT_U8:
        sconf->resolution = 0; break;
    case SAMPLE_FMT_S16:
        sconf->resolution = 1; break;
    case SAMPLE_FMT_FLT:
        sconf->floating   = 1;
        av_log_missing_feature(avctx, "floating-point samples\n", 0);
    case SAMPLE_FMT_S32:
        sconf->resolution = 3; break;
    default:
        av_log(avctx, AV_LOG_ERROR, "unsupported sample format: %s\n",
               avcodec_get_sample_fmt_name(avctx->sample_fmt));
        return -1;
    }

    avctx->bits_per_raw_sample = (sconf->resolution + 1) << 3;
    ctx->max_rice_param = sconf->resolution > 1 ? 31 : 15;


    // determine frame length
    frame_partitioning(ctx);


    // determine distance between ra-frames. 0 = no ra, 1 = all ra
    // default for now = 1. should be changed when implemented.
    // If we try to do it in the header, we would have to keep track
    // of all RA unit info during encoding, then free/realloc the
    // extradata at the end to make enough space for the RA info.
    sconf->ra_distance = 1;


    // determine where to store ra_flag (01: beginning of frame_data)
    // default for now = RA_FLAG_FRAMES. Would make decoding more robust
    // in case of seeking although not implemented in FFmpeg decoder yet
    sconf->ra_flag = RA_FLAG_FRAMES;


    // determine if adaptive prediction order is used
    // always use adaptive order unless the fastest compression level
    // has been selected.
    //
    // while not implemented just set to non-adaptive
    sconf->adapt_order = 0;


    // determine the coef_table to be used
    sconf->coef_table = (avctx->sample_rate > 48000) +
                        (avctx->sample_rate > 96000);


    // determine if long-term prediction is used
    // should also be user-defineable
    // TODO: make this depend on compression level
    sconf->long_term_prediction = 0;


    // determine a maximum prediction order
    // no samples: not yet possible to determine..
    // TODO: do evaluation to find a suitable standard value?
    //       if adapt_order is set and compression level is high,
    //       use maximum value to be able to find the best order
    sconf->max_order = 10;


    // determine if block-switching is used
    // depends on simple profile or not (user-defined?)
    // may be always enable this by default with the maximum level,
    // simple profile supports up to 3 stages
    // disable for the fastest compression mode
    // should be set when implemented
    sconf->block_switching = 0; // set to 1 to test block-switching
                                // with two blocks per frame


    // determine if BGMC mode is used
    // should be user-defineable
    sconf->bgmc = 0;


    // determine what sub-block partitioning is used
    sconf->sb_part = 1;


    // determine if joint-stereo is used
    // planned to be determined for each frame
    // set = 1 if #channels > 1 (?)
    // should be set when implemented
    // turn off for fastest compression level
    sconf->joint_stereo = 0;


    // determine if multi-channel coding is used
    // enable for better compression levels if implemented
    // may be sanity check: channels > 2
    // (although specs allow mc_coding for 2 channels...
    // maybe give a warning)
    sconf->mc_coding = 0;


    // determine manual channel configuration
    // using avctx->channel_layout
    // to be implemented
    sconf->chan_config = 0;
    sconf->chan_config_info = 0;


    // determine channel sorting
    // using avctx->channel_layout
    // to be implemented
    sconf->chan_sort = 0;
    sconf->chan_pos  = NULL;


    // determine if backward adaptive is used
    // user defined by explicit option and/or compression level
    sconf->rlslms = 0;


    // determine if CRC checksums are used
    // depends on compression level
    // should be enabled by default, not to use
    // in the fastest mode
    //
    // to be implemented and added to ALSSpecificConfig

    return 0;
}


static int write_specific_config(AVCodecContext *avctx)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    PutBitContext pb;
    MPEG4AudioConfig m4ac;
    int config_offset;

    unsigned int header_size = 6; // Maximum size of AudioSpecificConfig before ALSSpecificConfig

    // determine header size
    // crc & aux_data not yet supported
    header_size += ALS_SPECIFIC_CFG_SIZE;
    header_size += (sconf->chan_config > 0) << 1;                       // chan_config_info
    header_size += avctx->channels          << 1;                       // chan_pos[c]
//    header_size += (sconf->crc_enabled > 0) << 2;                     // crc TODO: include CRC computation
    if (sconf->ra_flag == RA_FLAG_HEADER && sconf->ra_distance > 0)     // ra_unit_size
        header_size += (sconf->samples / sconf->frame_length + 1) << 2;

    if (avctx->extradata)
        av_freep(&avctx->extradata);

    avctx->extradata = av_mallocz(header_size + FF_INPUT_BUFFER_PADDING_SIZE);
    if (!avctx->extradata)
        return AVERROR(ENOMEM);

    init_put_bits(&pb, avctx->extradata, header_size);


    // AudioSpecificConfig, reference to ISO/IEC 14496-3 section 1.6.2.1 & 1.6.3
    memset(&m4ac, 0, sizeof(MPEG4AudioConfig));
    m4ac.object_type    = AOT_ALS;
    m4ac.sampling_index = 0x0f;
    m4ac.sample_rate    = avctx->sample_rate;
    m4ac.chan_config    = 0;
    m4ac.sbr            = -1;

    config_offset = ff_mpeg4audio_write_config(&m4ac, avctx->extradata,
                                               avctx->extradata_size);

    if (config_offset < 0)
        return config_offset;

    skip_put_bits(&pb, config_offset);
    // switch(AudioObjectType) -> case 36: ALSSpecificConfig

    // fillBits (align)
    align_put_bits(&pb);

    put_bits32(&pb,     MKBETAG('A', 'L', 'S', '\0'));
    put_bits32(&pb,     avctx->sample_rate);
    put_bits32(&pb,     sconf->samples);
    put_bits  (&pb, 16, avctx->channels - 1);
    put_bits  (&pb,  3, 0);                      // original file_type (0 = unknown, 1 = wav, ...)
    put_bits  (&pb,  3, sconf->resolution);
    put_bits  (&pb,  1, sconf->floating);
    put_bits  (&pb,  1, 0);                      // msb first (0 = LSB, 1 = MSB)
    put_bits  (&pb, 16, sconf->frame_length - 1);
    put_bits  (&pb,  8, sconf->ra_distance);
    put_bits  (&pb,  2, sconf->ra_flag);
    put_bits  (&pb,  1, sconf->adapt_order);
    put_bits  (&pb,  2, sconf->coef_table);
    put_bits  (&pb,  1, sconf->long_term_prediction);
    put_bits  (&pb, 10, sconf->max_order);
    put_bits  (&pb,  2, sconf->block_switching);
    put_bits  (&pb,  1, sconf->bgmc);
    put_bits  (&pb,  1, sconf->sb_part);
    put_bits  (&pb,  1, sconf->joint_stereo);
    put_bits  (&pb,  1, sconf->mc_coding);
    put_bits  (&pb,  1, sconf->chan_config);
    put_bits  (&pb,  1, sconf->chan_sort);
    put_bits  (&pb,  1, 0);                     // crc_enabled (0 = none, TODO! add to sconf!)
    put_bits  (&pb,  1, sconf->rlslms);
    put_bits  (&pb,  5, 0);                     // reserved bits
    put_bits  (&pb,  1, 0);                     // aux_data_enabled (0 = false)

    // align
    align_put_bits(&pb);

    put_bits32(&pb, 0);                         // original header size
    put_bits32(&pb, 0);                         // original trailer size


    // writing in local header finished,
    // set the real size
    avctx->extradata_size = put_bits_count(&pb) >> 3;

    return 0;
}


static av_cold int encode_end(AVCodecContext *avctx)
{
    ALSEncContext *ctx = avctx->priv_data;

    av_freep(&ctx->independent_bs);
    av_freep(&ctx->raw_buffer);
    av_freep(&ctx->raw_samples);
    av_freep(&ctx->res_buffer);
    av_freep(&ctx->res_samples);
    av_freep(&ctx->block_buffer);
    av_freep(&ctx->blocks);
    av_freep(&ctx->bs_info);
    av_freep(&ctx->num_blocks);
    av_freep(&ctx->bs_sizes_buffer);
    av_freep(&ctx->bs_sizes);
#if BS_DETERMINE_SIZE_BY_WRITE
    av_freep(&ctx->bs_tmp_buffer);
#endif
    av_freep(&ctx->q_parcor_coeff_buffer);
    av_freep(&ctx->r_parcor_coeff);
    av_freep(&ctx->lpc_coeff);

    av_freep(&avctx->extradata);
    avctx->extradata_size = 0;
    av_freep(&avctx->coded_frame);

    return 0;
}


static void init_hanning_window(double *window, int len)
{
    int i, n2;
    double w, c;

    /* use a rectangle window for 1 to 3 samples */
    if (len < 3) {
        window[0] = 1.0;
        if (len > 1)
            window[1] = 1.0;
    }

    n2 = len >> 1;
    c = 2.0 * M_PI / (len - 1);

    for (i = 0; i < n2; i++) {
        w = 0.5 - 0.5 * cos(c * i);
        window[i]       = w;
        window[len-i-1] = w;
    }
    if (len & 1) {
        w = 0.5 - 0.5 * cos(c * i);
        window[i] = w;
    }
}


static av_cold int encode_init(AVCodecContext *avctx)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int channel_size;
    int ret, b, c;

    ctx->avctx = avctx;


    // determine ALSSpecificConfig
    if (get_specific_config(avctx))
        return -1;


    // write AudioSpecificConfig & ALSSpecificConfig to extradata
    ret = write_specific_config(avctx);
    if (ret) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        encode_end(avctx);
        return AVERROR(ENOMEM);
    }


    // initialize sample count
    sconf->samples = 0;


    channel_size = sconf->frame_length + sconf->max_order;


    // allocate buffers
    ctx->independent_bs    = av_malloc (sizeof(*ctx->independent_bs) * avctx->channels);
    ctx->raw_buffer        = av_mallocz(sizeof(*ctx->raw_buffer)     * avctx->channels * channel_size);
    ctx->raw_samples       = av_malloc (sizeof(*ctx->raw_samples)    * avctx->channels);
    ctx->res_buffer        = av_mallocz(sizeof(*ctx->raw_buffer)     * avctx->channels * channel_size);
    ctx->res_samples       = av_malloc (sizeof(*ctx->raw_samples)    * avctx->channels);
    ctx->num_blocks        = av_malloc (sizeof(*ctx->num_blocks)     * avctx->channels);
    ctx->bs_info           = av_malloc (sizeof(*ctx->bs_info)        * avctx->channels);
    ctx->block_buffer      = av_mallocz(sizeof(*ctx->block_buffer)   * avctx->channels * 32);
    ctx->blocks            = av_malloc (sizeof(*ctx->blocks)         * avctx->channels);
    ctx->q_parcor_coeff_buffer = av_malloc (sizeof(*ctx->q_parcor_coeff_buffer) * avctx->channels * 32 * sconf->max_order);
    ctx->lpc_coeff         = av_malloc (sizeof(*ctx->lpc_coeff)      * sconf->max_order);
    ctx->r_parcor_coeff    = av_malloc (sizeof(*ctx->r_parcor_coeff) * sconf->max_order);
    ctx->acf_window        = av_malloc (sizeof(*ctx->acf_window)     * sconf->frame_length);

    // check buffers
    if (!ctx->independent_bs    ||
        !ctx->raw_buffer        || !ctx->raw_samples ||
        !ctx->res_buffer        || !ctx->res_samples ||
        !ctx->num_blocks        || !ctx->bs_info     ||
        !ctx->block_buffer      || !ctx->blocks) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        encode_end(avctx);
        return AVERROR(ENOMEM);
    }


    // assign buffer pointers
    ctx->raw_samples[0] = ctx->raw_buffer + sconf->max_order;
    ctx->res_samples[0] = ctx->res_buffer + sconf->max_order;
    ctx->blocks     [0] = ctx->block_buffer;

    for (c = 1; c < avctx->channels; c++) {
        ctx->raw_samples[c] = ctx->raw_samples[c - 1] + channel_size;
        ctx->res_samples[c] = ctx->res_samples[c - 1] + channel_size;
        ctx->blocks     [c] = ctx->blocks     [c - 1] + 32;
    }

    ctx->blocks[0][0].q_parcor_coeff = ctx->q_parcor_coeff_buffer;
    for (c = 0; c < avctx->channels; c++) {
        for (b = 0; b < 32; b++) {
            if (b)
                ctx->blocks[c][b].q_parcor_coeff = ctx->blocks[c][b-1].q_parcor_coeff + sconf->max_order;
            else if (c)
                ctx->blocks[c][b].q_parcor_coeff = ctx->blocks[c-1][0].q_parcor_coeff + 32 * sconf->max_order;
        }
    }


    // channel sorting
    if ((sconf->joint_stereo || sconf->mc_coding) && sconf->chan_sort)
        channel_sorting(ctx);


    // allocate bs buffers
    if (sconf->block_switching) {
        int num_bs_sizes = (8 << sconf->block_switching) - 1;

        ctx->bs_sizes_buffer = av_malloc(sizeof(*ctx->bs_sizes_buffer) * num_bs_sizes * avctx->channels);
        ctx->bs_sizes        = av_malloc(sizeof(*ctx->bs_sizes)        * num_bs_sizes);
#if BS_DETERMINE_SIZE_BY_WRITE
        ctx->bs_tmp_buffer   = av_malloc(sizeof(*ctx->bs_tmp_buffer)   * sconf->frame_length);
#endif

        if (!ctx->bs_sizes || !ctx->bs_sizes_buffer) {
            av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
            encode_end(avctx);
            return AVERROR(ENOMEM);
        }

        for (c = 0; c < avctx->channels; c++) {
            ctx->bs_sizes[c] = ctx->bs_sizes_buffer + c * num_bs_sizes;
        }
    }


    avctx->coded_frame = avcodec_alloc_frame();
    avctx->coded_frame->key_frame = 1;


    dsputil_init(&ctx->dsp, avctx);

    init_hanning_window(ctx->acf_window, sconf->frame_length);

    return 0;
}


AVCodec als_encoder = {
    "als",
    CODEC_TYPE_AUDIO,
    CODEC_ID_MP4ALS,
    sizeof(ALSEncContext),
    encode_init,
    encode_frame,
    encode_end,
    NULL,
    .capabilities = CODEC_CAP_SMALL_LAST_FRAME | CODEC_CAP_DELAY,
    .long_name = NULL_IF_CONFIG_SMALL("MPEG-4 Audio Lossless Coding (ALS)"),
};
