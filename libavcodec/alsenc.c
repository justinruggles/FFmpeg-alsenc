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
#include "bgmc.h"
#include "libavutil/crc.h"


/** Total size of fixed-size fields in ALSSpecificConfig */
#define ALS_SPECIFIC_CFG_SIZE  30


/** Maximum number of blocks in a frame */
#define ALS_MAX_BLOCKS  32

/** Maximum lag value for LTP */
#define ALS_MAX_LTP_LAG 2048


/** Total number of stages used for allocation */
#define NUM_STAGES 3

/** Give the different stages used for encoding a readable name */
#define STAGE_JOINT_STEREO     0
#define STAGE_BLOCK_SWITCHING  1
#define STAGE_FINAL            2


/** Sets the current stage pointer in the context to the desired stage
 *  and writes all overriding options into specific config */
#define SET_OPTIONS(stage)                                \
{                                                         \
    ctx->cur_stage         = ctx->stages + (stage);       \
}


/** Determines entropy coding partitioning level using estimated Rice coding bit counts */
#define EC_SUB_ALGORITHM_RICE_ESTIMATE      0
/** Determines entropy coding partitioning level using exact Rice coding bit counts */
#define EC_SUB_ALGORITHM_RICE_EXACT         1
/** Determines entropy coding partitioning using exact BGMC coding bit counts */
#define EC_SUB_ALGORITHM_BGMC_EXACT         2

/** Estimates Rice parameters using sum of unsigned residual samples */
#define EC_PARAM_ALGORITHM_RICE_ESTIMATE    0
/** Calculates Rice parameters using a search algorithm based on exact bit count */
#define EC_PARAM_ALGORITHM_RICE_EXACT       1
/** Estimates BGMC parameters using mean of unsigned residual samples */
#define EC_PARAM_ALGORITHM_BGMC_ESTIMATE    2

/** Uses estimate for returned entropy coding bit count */
#define EC_BIT_COUNT_ALGORITHM_ESTIMATE     0
/** Uses exact calculation for returned entropy coding bit count */
#define EC_BIT_COUNT_ALGORITHM_EXACT        1

/** Find adaptive LPC order using a PARCOR coeff threshold value */
#define ADAPT_ORDER_ALGORITHM_THRESHOLD     0
/** Find adaptive LPC order by doing a full bit count for each possible order */
#define ADAPT_ORDER_ALGORITHM_FULL_SEARCH   1

/** Find optimal block partitioning using Full-Search */
#define BS_ALGORITHM_FULL_SEARCH    1
/** Find optimal block partitioning using Bottom-Up */
#define BS_ALGORITHM_BOTTOM_UP      0


/** Get the bit at position pos+1 in uint32_t *ptr_bs_info */
#define GET_BS_BIT(ptr_bs_info, pos) ((*ptr_bs_info & (1 << (30 - pos))) > 0)


/** grouped encoding algorithms and options */
typedef struct {
    // encoding options used during the processing of the stage
    int check_constant;             ///< check for constant sample value during this stage
    int check_lsbs;                 ///< check for zero LSB values during this stage
    int adapt_order;                ///< adaptive order to use during this stage
    int max_order;                  ///< max_oder to use during this stage
    int sb_part;                    ///< sb_part to use during this stage

    // encoding algorithms used during the processing of the stage
    int ecsub_algorithm;            ///< algorithm to use to determine entropy coding sub-block partitioning
    int param_algorithm;            ///< algorithm to use to determine Rice parameters
    int count_algorithm;            ///< algorithm to use for residual + rice param bit count
    int adapt_algorithm;            ///< algorithm to use for adaptive LPC order
    int merge_algorithm;            ///< algorithm to use to determine final block partitioning
} ALSEncStage;


typedef struct {
    int use_ltp;                    ///< ltp flag
    int lag;                        ///< lag value for long-term prediction
    int gain[5];                    ///< gain values for ltp 5-tap filter
    int bits_ltp;                   ///< bit count for LTP lag, gain and use_ltp flag
} ALSLTPInfo;


typedef struct {
    unsigned int sub_blocks;        ///< number of entropy coding sub-blocks in this block
    unsigned int rice_param[8];     ///< rice parameters to encode the residuals
                                    ///< of this block
    unsigned int bgmc_param[8];     ///< LSB's of estimated Rice parameters in case of BGMC mode
    int bits_ec_param_and_res;      ///< bit count for Rice/BGMC params and residuals
} ALSEntropyInfo;


// probably mergeable or the very same as ALSBlockData from the decoder
typedef struct {
    int ra_block;                   ///< indicates if this is an RA block
    int constant;                   ///< indicates constant block values
    int32_t constant_value;         ///< if constant, this is the value
    unsigned int length;            ///< length of the block in # of samples
    int div_block;                  ///< if > 0, this block length is 1/(1<<div_block) of a full frame
    unsigned int opt_order;         ///< prediction order for this block
    int32_t *q_parcor_coeff;        ///< 7-bit quantized PARCOR coefficients
    unsigned int js_block;          ///< indicates actual usage of joint-stereo coding
    unsigned int shift_lsbs;        ///< number of bits the samples have been right shifted
    ALSLTPInfo ltp_info[2];         ///< one set of LTPInfo for non-js- and js-residuals
    ALSEntropyInfo ent_info[2];     ///< one set of EntropyInfo for non-LTP- and LTP-residuals
    int32_t *res_ptr;               ///< points to the first residual for this block
    int32_t *smp_ptr;               ///< points to the first raw sample for this block
    int32_t *dif_ptr;               ///< points to the first difference sample for this block
    int32_t *lsb_ptr;               ///< points to the first LSB shifted sample for this block
    int32_t *cur_ptr;               ///< points to the current sample buffer for this block
    int bits_const_block;           ///< bit count for const block params
    int bits_misc;                  ///< bit count for js_block and shift_lsbs
    int bits_parcor_coeff;          ///< bit count for LPC order and PARCOR coeffs
} ALSBlock;


typedef struct {
    AVCodecContext *avctx;
    ALSSpecificConfig sconf;
    PutBitContext pb;
    DSPContext dsp;
    const AVCRC *crc_table;
    uint32_t crc;                   ///< CRC value calculated from decoded data
    ALSEncStage *stages;            ///< array containing all grouped encoding and algorithm options for each possible stage
    ALSEncStage *cur_stage;         ///< points to the currently used encoding stage
    unsigned int cur_frame_length;  ///< length of the current frame, in samples
    int js_switch;                  ///< force joint-stereo in case of MCC
    int *independent_bs;            ///< array containing independent_bs flag for each channel
    int32_t *raw_buffer;            ///< buffer containing all raw samples of the frame plus max_order samples from the previous frame (or zeroes) for all channels
    int32_t **raw_samples;          ///< pointer to the beginning of the current frame's samples in the buffer for each channel
    int32_t *raw_dif_buffer;        ///< buffer containing all raw difference samples of the frame plus max_order samples from the previous frame (or zeroes) for all channels
    int32_t **raw_dif_samples;      ///< pointer to the beginning of the current frame's difference samples in the buffer for each channel
    int32_t *raw_lsb_buffer;        ///< buffer containing all shifted raw samples of the frame plus max_order samples from the previous frame for all channels
    int32_t **raw_lsb_samples;      ///< pointer to the beginning of the current frame's shifted samples in the buffer for each channel
    int32_t *res_buffer;            ///< buffer containing all residual samples of the frame plus max_order samples from the previous frame (or zeroes) for all channels
    int32_t **res_samples;          ///< pointer to the beginning of the current frame's samples in the buffer for each channel
    uint32_t *bs_info;              ///< block-partitioning used for the current frame
    int *num_blocks;                ///< number of blocks used for the block partitioning
    unsigned int *bs_sizes_buffer;  ///< buffer containing all block sizes for all channels
    unsigned int **bs_sizes;        ///< pointer to the beginning of the channel's block sizes for each channel
    unsigned int *js_sizes_buffer;  ///< buffer containing all block sizes for all channel-pairs of the difference signal
    unsigned int **js_sizes;        ///< pointer to the beginning of the channel's block sizes for each channel-pair difference signal
    uint8_t *js_infos_buffer;       ///< buffer containing all joint-stereo flags for all channel-pairs
    uint8_t **js_infos;             ///< pointer to the beginning of the channel's joint-stereo flags for each channel-pair
    ALSBlock *block_buffer;         ///< buffer containing all ALSBlocks for each channel
    ALSBlock **blocks;              ///< array of 32 ALSBlock pointers per channel pointing into the block_buffer
    int32_t *q_parcor_coeff_buffer; ///< buffer containing 7-bit PARCOR coefficients for all blocks in all channels
    double *acf_coeff;              ///< autocorrelation function coefficients for the current block
    double *parcor_coeff;           ///< double-precision PARCOR coefficients for the current block
    int32_t *r_parcor_coeff;        ///< scaled 21-bit quantized PARCOR coefficients for the current block
    int32_t *lpc_coeff;             ///< LPC coefficients for the current block
    unsigned int max_rice_param;    ///< maximum Rice param, depends on sample depth
    double *acf_window_buffer;      ///< buffer containing all pre-calculated autocorrelation windows
    double *acf_window[6];          ///< pre-calculated autocorrelation windows for each block switching depth
    int32_t *ltp_buffer;            ///< temporary buffer to store long-term predicted samples
    double *ltp_corr_buffer;        ///< temporary buffer to store the signal during LTP autocorrelation
} ALSEncContext;


/** compression level 0 global options **/
static const ALSSpecificConfig spc_config_c0 = {
    .adapt_order            = 0,
    .long_term_prediction   = 0,
    .max_order              = 4,
    .block_switching        = 0,
    .bgmc                   = 0,
    .sb_part                = 0,
    .joint_stereo           = 0,
    .mc_coding              = 0,
    .rlslms                 = 0,
    .crc_enabled            = 0,
};

/** compression level 0 joint-stereo options
    note: compression level 0 does not use joint-stereo */
static const ALSEncStage stage_js_c0 = {
    .check_constant         = 0,
    .check_lsbs             = 0,
    .max_order              = 0,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_RICE_ESTIMATE,
    .param_algorithm        = EC_PARAM_ALGORITHM_RICE_ESTIMATE,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_ESTIMATE,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_BOTTOM_UP,
};

/** compression level 0 block switching stage options */
static const ALSEncStage stage_bs_c0 = {
    .check_constant         = 0,
    .check_lsbs             = 0,
    .max_order              = 4,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_RICE_ESTIMATE,
    .param_algorithm        = EC_PARAM_ALGORITHM_RICE_ESTIMATE,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_ESTIMATE,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_BOTTOM_UP,
};

/** compression level 0 final stage options */
static const ALSEncStage stage_final_c0 = {
    .check_constant         = 0,
    .check_lsbs             = 0,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_RICE_ESTIMATE,
    .param_algorithm        = EC_PARAM_ALGORITHM_RICE_ESTIMATE,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_ESTIMATE,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_BOTTOM_UP,
};


/** compression level 1 global options **/
static const ALSSpecificConfig spc_config_c1 = {
    .adapt_order            = 0,
    .long_term_prediction   = 0,
    .max_order              = 10,
    .block_switching        = 0,
    .bgmc                   = 0,
    .sb_part                = 1,
    .joint_stereo           = 1,
    .mc_coding              = 0,
    .rlslms                 = 0,
    .crc_enabled            = 1,
};

/** compression level 1 joint-stereo stage options */
static const ALSEncStage stage_js_c1 = {
    .check_constant         = 1,
    .check_lsbs             = 1,
    .max_order              = 4,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_RICE_EXACT,
    .param_algorithm        = EC_PARAM_ALGORITHM_RICE_EXACT,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_EXACT,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_FULL_SEARCH,
};

/** compression level 1 block switching stage options */
static const ALSEncStage stage_bs_c1 = {
    .check_constant         = 1,
    .check_lsbs             = 1,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_RICE_EXACT,
    .param_algorithm        = EC_PARAM_ALGORITHM_RICE_EXACT,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_EXACT,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_FULL_SEARCH,
};

/** compression level 1 final stage options */
static const ALSEncStage stage_final_c1 = {
    .check_constant         = 1,
    .check_lsbs             = 1,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_RICE_EXACT,
    .param_algorithm        = EC_PARAM_ALGORITHM_RICE_EXACT,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_EXACT,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_FULL_SEARCH,
};


/** compression level 2 global options **/
static const ALSSpecificConfig spc_config_c2 = {
    .adapt_order            = 0,
    .long_term_prediction   = 1,
    .max_order              = 10,
    .block_switching        = 1,
    .bgmc                   = 1,
    .sb_part                = 1,
    .joint_stereo           = 1,
    .mc_coding              = 0,
    .rlslms                 = 0,
    .crc_enabled            = 1,
};

/** compression level 2 joint-stereo stage options */
static const ALSEncStage stage_js_c2 = {
    .check_constant         = 1,
    .check_lsbs             = 1,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_BGMC_EXACT,
    .param_algorithm        = EC_PARAM_ALGORITHM_BGMC_ESTIMATE,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_EXACT,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_FULL_SEARCH,
};

/** compression level 2 block switching stage options */
static const ALSEncStage stage_bs_c2 = {
    .check_constant         = 1,
    .check_lsbs             = 1,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_BGMC_EXACT,
    .param_algorithm        = EC_PARAM_ALGORITHM_BGMC_ESTIMATE,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_EXACT,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_FULL_SEARCH,
};

/** compression level 2 final stage options */
static const ALSEncStage stage_final_c2 = {
    .check_constant         = 1,
    .check_lsbs             = 1,
    .ecsub_algorithm        = EC_SUB_ALGORITHM_BGMC_EXACT,
    .param_algorithm        = EC_PARAM_ALGORITHM_BGMC_ESTIMATE,
    .count_algorithm        = EC_BIT_COUNT_ALGORITHM_EXACT,
    .adapt_algorithm        = ADAPT_ORDER_ALGORITHM_FULL_SEARCH,
    .merge_algorithm        = BS_ALGORITHM_FULL_SEARCH,
};


/** global options for each compression level */
static const ALSSpecificConfig * const spc_config_settings[3] = {
    &spc_config_c0,
    &spc_config_c1,
    &spc_config_c2,
};

/** joint-stereo stage options for each compression level */
static const ALSEncStage * const stage_js_settings[3] = {
    &stage_js_c0,
    &stage_js_c1,
    &stage_js_c2,
};

/** block switching stage options for each compression level */
static const ALSEncStage * const stage_bs_settings[3] = {
    &stage_bs_c0,
    &stage_bs_c1,
    &stage_bs_c2,
};

/** final stage options for each compression level */
static const ALSEncStage * const stage_final_settings[3] = {
    &stage_final_c0,
    &stage_final_c1,
    &stage_final_c2,
};


static void dprint_stage_options(AVCodecContext *avctx, ALSEncStage *stage)
{
    dprintf(avctx, "check_constant = %d\n", stage->check_constant);
    dprintf(avctx, "check_lsbs = %d\n", stage->check_lsbs);
    dprintf(avctx, "adapt_order = %d\n", stage->adapt_order);
    dprintf(avctx, "max_order = %d\n", stage->max_order);
    dprintf(avctx, "sb_part = %d\n", stage->sb_part);

    switch (stage->ecsub_algorithm) {
    case EC_SUB_ALGORITHM_RICE_ESTIMATE: dprintf(avctx, "ecsub_algorithm = rice estimate\n"); break;
    case EC_SUB_ALGORITHM_RICE_EXACT:    dprintf(avctx, "ecsub_algorithm = rice exact\n");    break;
    case EC_SUB_ALGORITHM_BGMC_EXACT:    dprintf(avctx, "ecsub_algorithm = bgmc exact\n");    break;
    }

    switch (stage->param_algorithm) {
    case EC_PARAM_ALGORITHM_RICE_ESTIMATE: dprintf(avctx, "param_algorithm = rice estimate\n"); break;
    case EC_PARAM_ALGORITHM_RICE_EXACT:    dprintf(avctx, "param_algorithm = rice exact\n");    break;
    case EC_PARAM_ALGORITHM_BGMC_ESTIMATE: dprintf(avctx, "param_algorithm = bgmc estimate\n"); break;
    }

    switch (stage->count_algorithm) {
    case EC_BIT_COUNT_ALGORITHM_ESTIMATE: dprintf(avctx, "count_algorithm = estimate\n"); break;
    case EC_BIT_COUNT_ALGORITHM_EXACT:    dprintf(avctx, "count_algorithm = exact\n");    break;
    }

    switch (stage->adapt_algorithm) {
    case ADAPT_ORDER_ALGORITHM_THRESHOLD:   dprintf(avctx, "adapt_algorithm = threshold\n");   break;
    case ADAPT_ORDER_ALGORITHM_FULL_SEARCH: dprintf(avctx, "adapt_algorithm = full search\n"); break;
    }

    switch (stage->merge_algorithm) {
    case BS_ALGORITHM_FULL_SEARCH: dprintf(avctx, "merge_algorithm = full search\n"); break;
    case BS_ALGORITHM_BOTTOM_UP:   dprintf(avctx, "merge_algorithm = bottom-up\n");   break;
    }
}


static int write_specific_config(AVCodecContext *avctx);
static void gen_sizes(ALSEncContext *ctx, unsigned int channel, int stage);
static void gen_js_infos(ALSEncContext *ctx, unsigned int channel, int stage);
static void use_js_sizes(ALSEncContext *ctx, unsigned int channel, int stage);
static void reset_js(ALSEncContext *ctx, unsigned int channel, int stage);

static void gen_ltp_residuals(ALSEncContext *ctx, ALSBlock *block);
static void find_block_ltp_params(ALSEncContext *ctx, ALSBlock *block);

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


/** Generates the difference signals for each channel pair channel & channel+1
 */
static void gen_dif_signal(ALSEncContext *ctx, unsigned int channel)
{
    unsigned int n;
    int32_t *c1 = ctx->raw_samples    [channel     ];
    int32_t *c2 = ctx->raw_samples    [channel  + 1];
    int32_t *d  = ctx->raw_dif_samples[channel >> 1];

    for (n = 0; n < ctx->cur_frame_length; n++) {
        *d++ = *c2 - *c1;
        c1++;
        c2++;
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
    c              = 0;


    // if joint-stereo is enabled, dependently code each channel pair
    if (sconf->joint_stereo) {
        for (; c < avctx->channels - 1; c += 2) {
            ctx->independent_bs[c    ] = 0;
            ctx->independent_bs[c + 1] = 0;
        }
    }


    // set (the remaining) channels to independent coding
    for (; c < avctx->channels; c++)
        ctx->independent_bs[c] = 1;


    // generate difference signal if needed
    if (sconf->joint_stereo) {
        for (c = 0; c < avctx->channels - 1; c+=2) {
            gen_dif_signal(ctx, c);
        }
    }


    // generate all block sizes for this frame
    for (c = 0; c < avctx->channels; c++) {
        gen_sizes(ctx, c, 0);
    }


    // select difference signals wherever suitable
    if (sconf->joint_stereo) {
        for (c = 0; c < avctx->channels - 1; c+=2) {
            gen_js_infos(ctx, c, 0);
        }
    }
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


/** Recursively parses a given block partitioning and
 *  set all joint-stereo block flags
 */
static void parse_bs_js(const uint32_t bs_info, unsigned int n,
                        uint8_t *js_info,
                        ALSBlock **block_c1, ALSBlock **block_c2)
{
    if (n < 31 && ((bs_info << n) & 0x40000000)) {
        // if the level is valid and the investigated bit n is set
        // then recursively check both children at bits (2n+1) and (2n+2)
        n   *= 2;
        parse_bs_js(bs_info, n + 1, js_info, block_c1, block_c2);
        parse_bs_js(bs_info, n + 2, js_info, block_c1, block_c2);
    } else {
        // else the bit is not set or the last level has been reached
        // (bit implicitly not set)
        (*block_c1)->js_block = (js_info[n] == 1);
        (*block_c2)->js_block = (js_info[n] == 2);
        (*block_c1)++;
        (*block_c2)++;
    }
}


/** Successively merging the subblocks of the frame until
 *  the minimal bit count for the frame is found.
 *  Using Full-Search strategy.
 */
static void merge_bs_fullsearch(ALSEncContext *ctx, unsigned int n,
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

        if (GET_BS_BIT(bs_info, a)) {
            merge_bs_fullsearch(ctx, a, c1, c2);
        }

        if (GET_BS_BIT(bs_info, b)) {
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

        if (GET_BS_BIT(bs_info, a) && GET_BS_BIT(bs_info, b)) {
            merge_bs_bottomup(ctx, a, c1, c2);
            merge_bs_bottomup(ctx, b, c1, c2);
        }

        if (!GET_BS_BIT(bs_info, a) && !GET_BS_BIT(bs_info, b)) {
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
    int32_t *dif_ptr = ctx->raw_dif_samples[c1 >> 1];
    int32_t *lsb_ptr = ctx->raw_lsb_samples[c1];

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
        block->div_block = div_blocks[b];
        div_blocks[b]  = ctx->sconf.frame_length >> div_blocks[b];
        block->length  = div_blocks[b];
        block->res_ptr = res_ptr;
        block->smp_ptr = smp_ptr;
        block->dif_ptr = dif_ptr;
        block->lsb_ptr = lsb_ptr;
        res_ptr       += block->length;
        smp_ptr       += block->length;
        dif_ptr       += block->length;
        lsb_ptr       += block->length;
        block++;
    }

    if (ctx->cur_frame_length != sconf->frame_length) {
        unsigned int remaining = ctx->cur_frame_length;

        for (b = 0; b < ctx->num_blocks[c1]; b++) {
            if (remaining <= div_blocks[b]) {
                ctx->blocks[c1][b].div_block = -1;
                ctx->blocks[c1][b].length = remaining;
                ctx->num_blocks[c1] = b + 1;
                break;
            }
            remaining -= ctx->blocks[c1][b].length;
        }
    }

    if (c1 != c2) {
        res_ptr             = ctx->res_samples[c2];
        smp_ptr             = ctx->raw_samples[c2];
        dif_ptr             = ctx->raw_dif_samples[c1 >> 1];
        lsb_ptr             = ctx->raw_lsb_samples[c2];
        block               = ctx->blocks[c2];
        ctx->num_blocks[c2] = ctx->num_blocks[c1];

        for (b = 0; b < ctx->num_blocks[c1]; b++) {
            block->div_block = ctx->blocks[c1][b].div_block;
            block->length  = ctx->blocks[c1][b].length;
            block->res_ptr = res_ptr;
            block->smp_ptr = smp_ptr;
            block->dif_ptr = dif_ptr;
            block->lsb_ptr = lsb_ptr;
            res_ptr       += block->length;
            smp_ptr       += block->length;
            dif_ptr       += block->length;
            lsb_ptr       += block->length;
            block++;
        }
    }
}


/** Selects the best block-partitioning for the current frame
 *  depending on the chosen algorithm and sets the block sizes
 *  accordingly
 */
static unsigned int get_partition(ALSEncContext *ctx, unsigned int c1, unsigned int c2)
{
    ALSEncStage *stage = ctx->cur_stage;
    unsigned int *sizes_c1 = ctx->bs_sizes[c1];
    unsigned int *sizes_c2 = ctx->bs_sizes[c2];
    unsigned int bit_count = 0;

    if(stage->merge_algorithm == BS_ALGORITHM_BOTTOM_UP) {
        merge_bs_bottomup(ctx, 0, c1, c2);
    } else {
        merge_bs_fullsearch(ctx, 0, c1, c2);
    }

    get_block_sizes(ctx, &ctx->bs_info[c1], c1, c2);

    if (c1 != c2) {
        ALSBlock *ptr_blocks_c1 = ctx->blocks[c1];
        ALSBlock *ptr_blocks_c2 = ctx->blocks[c2];
        parse_bs_js(ctx->bs_info[c1], 0, ctx->js_infos[c1 >> 1], &ptr_blocks_c1, &ptr_blocks_c2);
    }

    parse_bs_size(ctx->bs_info[c1], 0, sizes_c1, &bit_count);
    if (c1 != c2)
        parse_bs_size(ctx->bs_info[c1], 0, sizes_c2, &bit_count);

    return bit_count;
}



/** Subdivide the frame into smaller blocks
 */
static void block_partitioning(ALSEncContext *ctx)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int c;

    // find the best partitioning for each channel
    if (!sconf->mc_coding || ctx->js_switch) {
        for (c = 0; c < avctx->channels - 1; c += 2) {
            if (sconf->joint_stereo) {
                unsigned int bits_ind, bits_dep;
                unsigned int bs_info_len = 1 << (sconf->block_switching + 2);
                int32_t bs_info_c1, bs_info_c2;
                int32_t bs_info = ctx->bs_info[c];

                bits_ind    = get_partition(ctx, c    , c    );
                bits_ind   += get_partition(ctx, c + 1, c + 1);
                bs_info_c1  = ctx->bs_info[c    ];
                bs_info_c2  = ctx->bs_info[c + 1];

                ctx->bs_info[c] = bs_info;

                use_js_sizes(ctx, c, 0);
                bits_dep = get_partition(ctx, c, c + 1);

                if (bits_ind + bs_info_len < bits_dep) {
                    reset_js(ctx, c, 0);
                    ctx->independent_bs[c    ] = 1;
                    ctx->independent_bs[c + 1] = 1;
                    ctx->bs_info       [c    ] = bs_info_c1;
                    ctx->bs_info       [c + 1] = bs_info_c2;
                    get_block_sizes(ctx, &ctx->bs_info[c    ], c    , c    );
                    get_block_sizes(ctx, &ctx->bs_info[c + 1], c + 1, c + 1);
                }
            } else {
                get_partition(ctx, c    , c    );
                get_partition(ctx, c + 1, c + 1);
            }
        }
        if (c < avctx->channels) {
            get_partition(ctx, c, c);
        }
    } else {

        // MCC: to be implemented

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


static inline int urice_count(unsigned int v, int k)
{
    return (v >> k) + 1 + k;
}


static inline int set_ur_golomb_als(PutBitContext *pb, unsigned int v, int k)
{
    int q;

    /* write quotient in zero-terminated unary */
    q = (v >> k) + 1;

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
        put_bits(pb, k, v - ((q - 1) << k));

    return 0;
}


static inline int set_sr_golomb_als(PutBitContext *pb, int v, int k)
{
    unsigned int v0;
    int q;

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


/** Encode the LSB part of the given symbols
 */
void bgmc_encode_lsb(PutBitContext *pb, int32_t *symbols, unsigned int n,
                     unsigned int k, unsigned int max, unsigned int s)
{
    int lsb_mask       = (1 << k) - 1;
    int abs_max        = (max + 1) >> 1;
    int high_offset = -(abs_max      << k);
    int low_offset  =  (abs_max - 1) << k;

    for (; n > 0; n--) {
        int32_t res = *symbols++;

        if (res >> k >=  abs_max || res >> k <= -abs_max) {
            res += res >> k >= abs_max ? high_offset : low_offset;
            set_sr_golomb_als(pb, res, s);
        } else if (k) {
            put_sbits(pb, k, res & lsb_mask);
        }
    }
}


/** Count bits needed to encode the LSB part of the given symbols
 */
static void bgmc_encode_lsb_count(unsigned int *bits, const int32_t *symbols, unsigned int n,
                                  unsigned int k, unsigned int max, unsigned int s)
{
    int abs_max        = (max + 1) >> 1;
    int high_offset = -(abs_max      << k);
    int low_offset  =  (abs_max - 1) << k;

    for (; n > 0; n--) {
        int32_t res = *symbols++;

        if (res >> k <= -abs_max || res >> k >= abs_max) {
            res += res >> k >= abs_max ? high_offset : low_offset;
            *bits += rice_count(res, s);
        } else if (k) {
            *bits += k;
        }
    }
}


/** Map LTP gain value to nearest flattened array index
 */
static int map_to_index(int gain)
{
    int i, diff, min_diff, best_index;
    const uint8_t *g_ptr = &ff_als_ltp_gain_values[0][0];

    min_diff   = abs((int)(*g_ptr++) - gain);
    best_index = 0;
    for (i = 1; i < 16; i++) {
        diff = abs((int)(*g_ptr++) - gain);

        if (!diff) {
            return i;
        } else if (diff < min_diff) {
            min_diff   = diff;
            best_index = i;
        } else {
            return best_index;
        }
    }

    return best_index;
}


/** Write a given block of a given channel
 */
static int write_block(ALSEncContext *ctx, ALSBlock *block)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    PutBitContext *pb        = &ctx->pb;
    unsigned int i;
    int start = 0;


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
        ALSLTPInfo *ltp     = &block->ltp_info[block->js_block];
        ALSEntropyInfo *ent = &block->ent_info[ltp->use_ltp];
        int32_t *res_ptr;
        int sb, sb_length;
        unsigned int high;
        unsigned int low;
        unsigned int follow;
        unsigned int delta[8];
        unsigned int k    [8];
        unsigned int max  [8];
        unsigned int *s  = ent->rice_param;
        unsigned int *sx = ent->bgmc_param;

        // js_block
        put_bits(pb, 1, block->js_block);


        // ec_sub
        if (sconf->sb_part) {
            if (sconf->bgmc)
                put_bits(pb, 2, av_log2(ent->sub_blocks));
            else
                put_bits(pb, 1, ent->sub_blocks > 1);
        }


        // s[k], sx[k]
        if (sconf->bgmc) {
            unsigned int S[8];

            for (sb = 0; sb < ent->sub_blocks; sb++)
                S[sb] = (ent->rice_param[sb] << 4) | ent->bgmc_param[sb];

            put_bits(pb, 8 + (avctx->bits_per_raw_sample > 16), S[0]);
            for (sb = 1; sb < ent->sub_blocks; sb++)
                if (set_sr_golomb_als(pb, S[sb] - S[sb-1], 2))
                    return -1;
        } else {
            put_bits(pb, 4 + (avctx->bits_per_raw_sample > 16), ent->rice_param[0]);

            for (sb = 1; sb < ent->sub_blocks; sb++) {
                if (set_sr_golomb_als(pb, ent->rice_param[sb] - ent->rice_param[sb-1], 0))
                    return -1;
            }
        }


        // shift_lsbs && shift_pos
        put_bits(pb, 1, block->shift_lsbs > 0);

        if (block->shift_lsbs)
            put_bits(pb, 4, block->shift_lsbs - 1);


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
                    if (set_sr_golomb_als(pb, block->q_parcor_coeff[i] - (i & 1), 2))
                        return -1;

                // write coefficients 127 to opt_order
                for (; i < block->opt_order; i++)
                    if (set_sr_golomb_als(pb, block->q_parcor_coeff[i], 1))
                        return -1;
            }
        }


        // LPTenable && LTPgain && LTPlag
        if (sconf->long_term_prediction) {
            put_bits(pb, 1, block->ltp_info[block->js_block].use_ltp);

            if (block->ltp_info[block->js_block].use_ltp) {
                ALSLTPInfo *ltp    = &block->ltp_info[block->js_block];
                int ltp_lag_length = 8 + (avctx->sample_rate >=  96000) +
                                         (avctx->sample_rate >= 192000);

                set_sr_golomb_als(pb, ltp->gain[0] >> 3,          1);
                set_sr_golomb_als(pb, ltp->gain[1] >> 3,          2);
                set_ur_golomb_als(pb, map_to_index(ltp->gain[2]), 2);
                set_sr_golomb_als(pb, ltp->gain[3] >> 3,          2);
                set_sr_golomb_als(pb, ltp->gain[4] >> 3,          1);

                put_bits(pb, ltp_lag_length, ltp->lag - FFMAX(4, block->opt_order + 1));

                gen_ltp_residuals(ctx, block);
                block->cur_ptr = ctx->ltp_buffer;
            }
        }


        // write residuals
        // for now, all frames are RA frames, so use progressive prediction for
        // the first 3 residual samples, up to opt_order

        res_ptr = block->cur_ptr;
        sb_length = block->length / ent->sub_blocks;

        if (sconf->bgmc)
            ff_bgmc_encode_init(&high, &low, &follow);

        for (sb = 0; sb < ent->sub_blocks; sb++) {
            i = 0;
            if (!sb && block->ra_block) {
                int len = block->opt_order;
                int32_t write;
                if (len > 0) {
                    if (set_sr_golomb_als(pb, *res_ptr++, avctx->bits_per_raw_sample-4))
                        return -1;
                    i++;
                    if (len > 1) {
                        write = sb_length <= 1 ? 0 : *res_ptr++;
                        if (set_sr_golomb_als(pb, write, FFMIN(ent->rice_param[sb]+3, ctx->max_rice_param)))
                            return -1;
                        i++;
                        if (len > 2) {
                            write = sb_length <= 2 ? 0 : *res_ptr++;
                            if (set_sr_golomb_als(pb, write, FFMIN(ent->rice_param[sb]+1, ctx->max_rice_param)))
                                return -1;
                            i++;
                        }
                    }
                }
                start = i;
            }
            if (sconf->bgmc) {
                unsigned int b = av_clip((av_ceil_log2(block->length) - 3) >> 1, 0, 5);
                k    [sb] = s[sb] > b ? s[sb] - b : 0;
                delta[sb] = 5 - s[sb] + k[sb];
                max  [sb] = ff_bgmc_max[sx[sb]] >> delta[sb];

                ff_bgmc_encode_msb(pb, res_ptr, sb_length - i,
                                   k[sb], delta[sb], max[sb],
                                   s[sb], sx[sb],
                                   &high, &low, &follow);

                res_ptr += sb_length - i;
            } else {
                for (; i < sb_length; i++) {
                    if (set_sr_golomb_als(pb, *res_ptr++, ent->rice_param[sb]))
                        return -1;
                }
            }
        }

        if (sconf->bgmc) {
            ff_bgmc_encode_end(pb, &low, &follow);

            res_ptr = block->res_ptr + start;

            for (sb = 0; sb < ent->sub_blocks; sb++, start = 0) {
                bgmc_encode_lsb(pb, res_ptr, sb_length - start, k[sb], max[sb], s[sb]);
                res_ptr += sb_length - start;
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

                if (sconf->joint_stereo && ctx->independent_bs[c])
                    bs_info |= (1 << 31);

                if (bs_info_len == 32)
                    put_bits32(&ctx->pb, bs_info);
                else
                    put_bits(&ctx->pb, bs_info_len, bs_info >> (32 - bs_info_len));
            }

            for (b= 0; b < ctx->num_blocks[c]; b++) {
                if (ctx->independent_bs[c]) {
                    ret = write_block(ctx, &ctx->blocks[c][b]);
                    if (ret < 0)
                        return ret;
                } else {
                    ret = write_block(ctx, &ctx->blocks[c][b]);
                    if (ret < 0)
                        return ret;
                    ret = write_block(ctx, &ctx->blocks[c+1][b]);
                    if (ret < 0)
                        return ret;
                }
            }

            if(!ctx->independent_bs[c]) c++;
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


static void calc_parcor_coeff_bit_size(ALSEncContext *ctx, ALSBlock *block,
                                       int order)
{
    int i, next_max_order, bit_count;

    bit_count = 0;

    if (ctx->sconf.adapt_order) {
        bit_count += av_ceil_log2(av_clip((block->length >> 3) - 1,
                                          2, ctx->sconf.max_order + 1));
    }

    next_max_order = FFMIN(order, 20);
    for (i = 0; i < next_max_order; i++) {
        int rice_param = ff_als_parcor_rice_table[ctx->sconf.coef_table][i][1];
        int offset     = ff_als_parcor_rice_table[ctx->sconf.coef_table][i][0];
        bit_count += rice_count(block->q_parcor_coeff[i] - offset, rice_param);
    }
    next_max_order = FFMIN(order, 127);
    for (; i < next_max_order; i++) {
        bit_count += rice_count(block->q_parcor_coeff[i] - (i & 1), 2);
    }
    for (; i < order; i++) {
        bit_count += rice_count(block->q_parcor_coeff[i], 1);
    }

    block->bits_parcor_coeff = bit_count;
}


static unsigned int subblock_rice_count_exact(const int32_t *res_ptr,
                                              int sb_length, int rice_param,
                                              int max_param, int ra_subblock,
                                              int order)
{
    unsigned int count = 0;
    int i = 0;

    if (ra_subblock) {
        if (order > 0) {
            int32_t v = *res_ptr++;
            count += rice_count(v, max_param - 3);
            i++;
            if (order > 1) {
                v = sb_length <= 1 ? 0 : *res_ptr++;
                count += rice_count(v, FFMIN(rice_param+3, max_param));
                i++;
                if (order > 2) {
                    v = sb_length <= 2 ? 0 : *res_ptr++;
                    count += rice_count(v, FFMIN(rice_param+1, max_param));
                    i++;
                }
            }
        }
    }
    for (; i < sb_length; i++) {
        int32_t v = *res_ptr++;
        count += rice_count(v, rice_param);
    }

    return count;
}


/** Count bits needed to encode all symbols of a given subblock
 *  using given parameters
 */
static unsigned int subblock_bgmc_count_exact(const int32_t *res_ptr,
                                              int b_length, int sb_length,
                                              int rice_param, int sx,
                                              int max_param, int ra_subblock,
                                              int order)
{
    unsigned int count = 0;
    unsigned int len   = 0;
    unsigned int high, low, follow;
    unsigned int delta, k, max, b;

    if (ra_subblock) {
        if (order > 0) {
            int32_t v = *res_ptr++;
            len++;
            count += rice_count(v, max_param - 3);
            if (order > 1) {
                v = sb_length <= 1 ? 0 : *res_ptr++;
                len++;
                count += rice_count(v, FFMIN(rice_param+3, max_param));
                if (order > 2) {
                    v = sb_length <= 2 ? 0 : *res_ptr++;
                    len++;
                    count += rice_count(v, FFMIN(rice_param+1, max_param));
                }
            }
        }
    }

    // count msb's
    ff_bgmc_encode_init(&high, &low, &follow);

    b     = av_clip((av_ceil_log2(b_length) - 3) >> 1, 0, 5);
    k     = rice_param > b ? rice_param - b : 0;
    delta = 5 - rice_param + k;
    max = ff_bgmc_max[sx] >> delta;

    ff_bgmc_encode_msb_count(&count, res_ptr, sb_length - len,
                             k, delta, max, rice_param, sx,
                             &high, &low, &follow);

    ff_bgmc_encode_end_count(&count, &follow);

    // count lsb's
    bgmc_encode_lsb_count(&count, res_ptr, sb_length - len,
                          k, max, rice_param);

    return count;
}


static unsigned int block_rice_count_exact(ALSEncContext *ctx, ALSBlock *block,
                                           int sub_blocks, int *rice_param,
                                           int order)
{
    int32_t *res_ptr = block->cur_ptr;
    unsigned int count = 0;
    int sb_length, sb;

    sb_length = block->length / sub_blocks;

    for (sb = 0; sb < sub_blocks; sb++) {
        count += subblock_rice_count_exact(res_ptr, sb_length, rice_param[sb],
                                           ctx->max_rice_param,
                                           !sb && block->ra_block, order);
        if (!sb)
            count += 4 + (ctx->max_rice_param > 15);
        else
            count += rice_count(rice_param[sb]-rice_param[sb-1], 0);

        res_ptr += sb_length;
    }

    count += !!ctx->sconf.sb_part;

    return count;
}


static unsigned int block_bgmc_count_exact(ALSEncContext *ctx, ALSBlock *block,
                                           int sub_blocks, int *s, int *sx,
                                           int order)
{
    int32_t *res_ptr = block->cur_ptr;
    unsigned int count = 0;
    int sb_length, sb;

    sb_length = block->length / sub_blocks;

    for (sb = 0; sb < sub_blocks; sb++) {
        count += subblock_bgmc_count_exact(res_ptr, block->length,
                                           sb_length, s[sb], sx[sb],
                                           ctx->max_rice_param,
                                           !sb && block->ra_block, order);

        if (!sb) {
            count += 8 + (ctx->max_rice_param > 15);    // S[0]
        } else {
            int S = ((s[sb    ] << 4) | sx[sb    ]) -
                    ((s[sb - 1] << 4) | sx[sb - 1]);
            count += rice_count(S, 2);                  // S[i]
        }

        res_ptr += sb_length;
    }

    count += 2 * !!ctx->sconf.sb_part; // ec_sub

    return count;
}


#define rice_encode_count(sum, n, k) (((n)*((k)+1))+((sum-(n>>1))>>(k)))


static inline int estimate_rice_param(uint64_t sum, int length, int max_param)
{
    int k;

    if (sum <= length >> 1)
        return 0;

    if (max_param > 15) {
        sum = FFMAX((sum - (length >> 1)) / length, 1);
        k = (int)floor(log2(sum));
    } else {
        unsigned int sum1 = sum;
        sum1 = sum1 - (length >> 1);
        k = av_log2(length < 256 ? FASTDIV(sum1, length) : sum1 / length);
    }

    return FFMIN(k, max_param);
}


/** Get an estimated Rice parameter and split it into its LSB and MSB
 *  for further processing in BGMC
 */
static inline void estimate_bgmc_params(uint64_t sum, unsigned int n, int *s,
                                        int *sx)
{
#define OFFSET 0.97092725747512664825  /* 0.5 + log2(1.386) */

    if (!sum) { // avoid log2(0)
        *sx = *s = 0;
    } else {
        int tmp = (int)(16.0 * (log2(sum) - log2(n) + OFFSET));
        tmp = FFMAX(tmp, 0);
        *sx = tmp & 0x0F;
        *s  = tmp >> 4;
    }
}


static void find_block_rice_params_est(ALSEncContext *ctx, ALSBlock *block,
                                      int order)
{
    int i, sb, sb_max, sb_length;
    uint64_t sum[5] = {0,};
    int param[5];
    unsigned int count1, count4;
    ALSEncStage *stage = ctx->cur_stage;
    ALSLTPInfo *ltp     = &block->ltp_info[block->js_block];
    ALSEntropyInfo *ent = &block->ent_info[ltp->use_ltp];

    if (!stage->sb_part || block->length & 0x3 || block->length < 16)
        sb_max = 1;
    else
        sb_max = 4;
    sb_length = block->length / sb_max;

    for (sb = 0; sb < sb_max; sb++) {
        const int32_t *res_ptr1 = block->cur_ptr + (sb * sb_length);
        for (i = 0; i < sb_length; i++) {
            int v = *res_ptr1++;
            if (ctx->max_rice_param > 15) {
                unsigned int v0 = (unsigned int)((2LL*v) ^ (int64_t)(v>>31));
                sum[sb] += v0;
            } else {
                v = (2 * v) ^ (v >> 31);
                sum[sb] += v;
            }
        }
        sum[4] += sum[sb];

        param[sb] = estimate_rice_param(sum[sb], sb_length, ctx->max_rice_param);
    }

    param[4] = estimate_rice_param(sum[4], block->length, ctx->max_rice_param);

    if (stage->count_algorithm == EC_BIT_COUNT_ALGORITHM_EXACT) {
        count1 = block_rice_count_exact(ctx, block, 1, &param[4], order);
    } else {
        count1 = rice_encode_count(sum[4], block->length, param[4]);
        count1 += 4 + (ctx->max_rice_param > 15);
    }

    if (sb_max == 1 || ((param[0] == param[1]) && (param[1] == param[2]) &&
        (param[2] == param[3]))) {
        ent->sub_blocks = 1;
        ent->rice_param[0] = param[4];
        ent->bits_ec_param_and_res = count1;
        return;
    }

    if (stage->count_algorithm == EC_BIT_COUNT_ALGORITHM_EXACT) {
        count4 = block_rice_count_exact(ctx, block, 4, param, order);
    } else {
        count4 = 0;
        for (sb = 0; sb < sb_max; sb++) {
            count4 += rice_encode_count(sum[sb], sb_length, param[sb]);

            if (!sb)
                count4 += 4 + (ctx->max_rice_param > 15);
            else
                count4 += rice_count(param[sb] - param[sb-1], 0);
        }
    }

    if (count1 <= count4) {
        ent->sub_blocks = 1;
        ent->rice_param[0] = param[4];
    } else {
        ent->sub_blocks = 4;
        ent->rice_param[0] = param[0];
        ent->rice_param[1] = param[1];
        ent->rice_param[2] = param[2];
        ent->rice_param[3] = param[3];
    }

    ent->bits_ec_param_and_res = FFMIN(count1, count4);
}


/** Full search for optimal BGMC parameters and and sub-block devision
 */
static void find_block_bgmc_params(ALSEncContext *ctx, ALSBlock *block, int order)
{
    ALSEncStage    *stage = ctx->cur_stage;
    ALSLTPInfo     *ltp   = &block->ltp_info[block->js_block];
    ALSEntropyInfo *ent   = &block->ent_info[ltp->use_ltp];
    int s[4][8], sx[4][8];
    int p, sb, i;
    int p_max;
    int p_best;
    unsigned int count_best = UINT_MAX;
    uint64_t sum[4][8];


    if (!stage->sb_part || block->length & 0x3 || block->length < 16)
        p_max = 0;
    else
        p_max = 3;

    p_best = p_max;

    for (p = p_max; p >= 0; p--) {
        int num_subblocks  = 1 << p;
        int sb_length      = block->length / num_subblocks;
        unsigned int count = 0;
        int32_t *res_ptr   = block->cur_ptr;

        for (sb = 0; sb < num_subblocks; sb++) {
            if (p == p_max) {
                int32_t *r_ptr = res_ptr;
                sum[p][sb] = 0;
                for (i = 0; i < sb_length; i++)
                    sum[p][sb] += abs(*r_ptr++);
            } else {
                sum[p][sb] = sum[p+1][sb<<1] + sum[p+1][(sb<<1)+1];
            }
            estimate_bgmc_params(sum[p][sb], sb_length, &s[p][sb], &sx[p][sb]);

            if (stage->ecsub_algorithm == EC_SUB_ALGORITHM_RICE_ESTIMATE) {
                int k = estimate_rice_param (sum[p][sb], sb_length,
                                             ctx->max_rice_param);
                count += rice_encode_count(sum[p][sb], sb_length, k);
            }

            res_ptr += sb_length;
        }

        if (stage->ecsub_algorithm == EC_SUB_ALGORITHM_BGMC_EXACT) {
            count = block_bgmc_count_exact(ctx, block, num_subblocks,
                                           s[p], sx[p], order);
        }

        if (count <= count_best) {
            count_best = count;
            p_best     = p;
        }
    }

    ent->sub_blocks = 1 << p_best;
    for (sb = 0; sb < ent->sub_blocks; sb++) {
        ent->rice_param[sb] = s [p_best][sb];
        ent->bgmc_param[sb] = sx[p_best][sb];
    }

    if (stage->ecsub_algorithm == EC_SUB_ALGORITHM_RICE_ESTIMATE &&
        stage->count_algorithm == EC_BIT_COUNT_ALGORITHM_EXACT) {
        ent->bits_ec_param_and_res = block_bgmc_count_exact(ctx, block,
                                                            ent->sub_blocks,
                                                            ent->rice_param,
                                                            ent->bgmc_param,
                                                            order);
    } else {
        ent->bits_ec_param_and_res = count_best;
    }
}


static void find_block_rice_params_exact(ALSEncContext *ctx, ALSBlock *block,
                                        int order)
{
    unsigned int count[5][32] = {{0,}};
    int param[5];
    int k, step, sb, sb_max, sb_length;
    int best_k;
    unsigned int count1, count4;
    ALSEncStage *stage = ctx->cur_stage;
    ALSLTPInfo *ltp     = &block->ltp_info[block->js_block];
    ALSEntropyInfo *ent = &block->ent_info[ltp->use_ltp];

    if (!stage->sb_part || block->length & 0x3 || block->length < 16)
        sb_max = 1;
    else
        sb_max = 4;
    sb_length = block->length / sb_max;

    for (sb = 0; sb < sb_max; sb++) {
        /* start 1/3 distance between 0 and max_param */
        k = ctx->max_rice_param / 3;
        count[sb][k] = subblock_rice_count_exact(block->cur_ptr + (sb * sb_length),
                                                 sb_length, k, ctx->max_rice_param,
                                                 !sb && block->ra_block, order);
        k++;
        count[sb][k] = subblock_rice_count_exact(block->cur_ptr + (sb * sb_length),
                                                 sb_length, k, ctx->max_rice_param,
                                                 !sb && block->ra_block, order);
        if (count[sb][k] < count[sb][k-1]) {
            best_k = k;
            step = 1;
            k++;
        } else {
            best_k = k - 1;
            step = -1;
            k -= 2;
        }

        for (; k >= 0 && k <= ctx->max_rice_param; k += step) {
            count[sb][k] = subblock_rice_count_exact(block->cur_ptr + (sb * sb_length),
                                                     sb_length, k, ctx->max_rice_param,
                                                     !sb && block->ra_block, order);

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
        ent->sub_blocks = 1;
        ent->rice_param[0] = param[0];
        ent->bits_ec_param_and_res = block_rice_count_exact(ctx, block, 1,
                                                              param, order);
    }


    /* start 1/3 distance between 0 and max_param */
    k = ctx->max_rice_param / 3;
    count[4][k] = block_rice_count_exact(ctx, block, 1, &k, order);
    k++;
    count[4][k] = block_rice_count_exact(ctx, block, 1, &k, order);
    if (count[4][k] < count[4][k-1]) {
        best_k = k;
        step = 1;
        k++;
    } else {
        best_k = k - 1;
        step = -1;
        k -= 2;
    }

    for (; k >= 0 && k <= ctx->max_rice_param; k += step) {
        count[4][k] = block_rice_count_exact(ctx, block, 1, &k, order);

        if (count[4][k] < count[4][best_k]) {
            best_k = k;
        } else {
            break;
        }
    }
    param[4] = best_k;


    count1  = count[4][param[4]];
    count4 = block_rice_count_exact(ctx, block, 4, param, order);
    if (count1 <= count4) {
        ent->sub_blocks = 1;
        ent->rice_param[0] = param[4];
        ent->bits_ec_param_and_res = count1;
    } else {
        ent->sub_blocks = 4;
        ent->rice_param[0] = param[0];
        ent->rice_param[1] = param[1];
        ent->rice_param[2] = param[2];
        ent->rice_param[3] = param[3];
        ent->bits_ec_param_and_res = count4;
    }
}


/**
 * Calculate optimal sub-block division and Rice parameters for a block.
 * @param ctx                   encoder context
 * @param block                 current block
 * @param[in] ra_block          indicates if this is a random access block
 * @param[in] order             LPC order
 */
static void find_block_entropy_params(ALSEncContext *ctx, ALSBlock *block,
                                   int order)
{
    ALSEncStage *stage = ctx->cur_stage;

    if        (stage->param_algorithm == EC_PARAM_ALGORITHM_BGMC_ESTIMATE) {
        find_block_bgmc_params(ctx, block, order);
    } else if (stage->param_algorithm == EC_PARAM_ALGORITHM_RICE_ESTIMATE) {
        find_block_rice_params_est(ctx, block, order);
    } else if (stage->param_algorithm == EC_PARAM_ALGORITHM_RICE_EXACT) {
        find_block_rice_params_exact(ctx, block, order);
    }
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


static int calc_short_term_prediction(ALSEncContext *ctx, ALSBlock *block,
                                       int order)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    int i, j;

    int32_t *res_ptr = block->res_ptr;
    int32_t *smp_ptr = block->cur_ptr;

#define LPC_PREDICT_SAMPLE(lpc, smp_ptr, res_ptr, order)\
{\
    int64_t y = 1 << 19;\
    for (j = 1; j <= order; j++)\
        y += MUL64(lpc[j-1], (smp_ptr)[-j]);\
    *(res_ptr++) = *(smp_ptr++) + (int32_t)(y >> 20);\
}

    i = 0;
    if (block->ra_block) {
        int ra_order = FFMIN(order, block->length);

        // copy first residual sample verbatim
        *(res_ptr++) = *(smp_ptr++);

        // progressive prediction
        ff_als_parcor_to_lpc(0, ctx->r_parcor_coeff, ctx->lpc_coeff);
        for (i = 1; i < ra_order; i++) {
            LPC_PREDICT_SAMPLE(ctx->lpc_coeff, smp_ptr, res_ptr, i);
            if (ff_als_parcor_to_lpc(i, ctx->r_parcor_coeff, ctx->lpc_coeff))
                return -1;
        }
        // zero unused coeffs for small frames since they are all written
        // to the bitstream if adapt_order is not used.
        if (!sconf->adapt_order) {
            for (; i < sconf->max_order; i++)
                block->q_parcor_coeff[i] = ctx->r_parcor_coeff[i] = 0;
        }
    } else {
        for (j = 0; j < order; j++)
            if (ff_als_parcor_to_lpc(j, ctx->r_parcor_coeff, ctx->lpc_coeff))
                return -1;
    }
    // remaining residual samples
    for (; i < block->length; i++) {
        LPC_PREDICT_SAMPLE(ctx->lpc_coeff, smp_ptr, res_ptr, order);
    }
    return 0;
}


static void check_ltp(ALSEncContext *ctx, ALSBlock *block, int *bit_count)
{
    ALSLTPInfo *ltp     = &block->ltp_info[block->js_block];
    ALSEntropyInfo *ent = &block->ent_info[ltp->use_ltp];
    int bit_count_ltp;
    int32_t *save_ptr  = block->cur_ptr;
    int ltp_lag_length = 8 + (ctx->avctx->sample_rate >=  96000) +
                             (ctx->avctx->sample_rate >= 192000);

    find_block_ltp_params(ctx, block);
    gen_ltp_residuals(ctx, block);

    // generate bit count for LTP signal
    block->cur_ptr = ctx->ltp_buffer;
    ltp->use_ltp = 1;
    ent = &block->ent_info[ltp->use_ltp];
    find_block_entropy_params(ctx, block, block->opt_order);

    ltp->bits_ltp = 1 + ltp_lag_length +
                    rice_count(ltp->gain[0],               1) +
                    rice_count(ltp->gain[1],               2) +
                   urice_count(map_to_index(ltp->gain[2]), 2) +
                    rice_count(ltp->gain[3],               2) +
                    rice_count(ltp->gain[4],               1);

    // test if LTP pays off
    bit_count_ltp = block->bits_misc +
                    block->bits_parcor_coeff +
                    ent->bits_ec_param_and_res +
                    ltp->bits_ltp;
    bit_count_ltp += (8 - (bit_count_ltp & 7)) & 7;

    if (bit_count_ltp < *bit_count) {
        block->cur_ptr = save_ptr;
        *bit_count     = bit_count_ltp;
    } else {
        ltp->use_ltp  = 0;
        ent = &block->ent_info[ltp->use_ltp];
        ltp->bits_ltp = 1;
        block->cur_ptr = save_ptr;
    }
}


static int calc_block_size_fixed_order(ALSEncContext *ctx, ALSBlock *block,
                                       int order)
{
    int32_t count;
    int32_t *save_ptr = block->cur_ptr;
    ALSLTPInfo *ltp     = &block->ltp_info[block->js_block];
    ALSEntropyInfo *ent = &block->ent_info[ltp->use_ltp];

    if (order) {
        if (calc_short_term_prediction(ctx, block, order))
            return -1;
        block->cur_ptr = block->res_ptr;
    }
    calc_parcor_coeff_bit_size(ctx, block, order);


    find_block_entropy_params (ctx, block, order);

    count  = block->bits_misc + block->bits_parcor_coeff +
                    ent->bits_ec_param_and_res;
    count += (8 - (count & 7)) & 7; // byte align
#if 0
    if (ctx->sconf.long_term_prediction)
        check_ltp(ctx, block, &count);
#endif
    block->cur_ptr = save_ptr;

    return count;
}


static void find_block_adapt_order_full_search(ALSEncContext *ctx,
                                               ALSBlock *block, int max_order)
{
    int i;
    int32_t count[max_order+1];
    int best = 0;

    count[0] = INT32_MAX;

    for (i = 0; i <= max_order; i++) {

        count[i] = calc_block_size_fixed_order(ctx, block, i);

        if (count[i] >= 0 && count[i] < count[best])
            best = i;
    }
    block->opt_order = best;
}


/** Tests given block samples to be of constant value
 * Sets block->const_block_bits to the number of bits used for encoding the
 * constant block, or to zero if the block is not a constant block.
 */
static void test_const_value(ALSEncContext *ctx, ALSBlock *block)
{
    if (ctx->cur_stage->check_constant) {
        unsigned int n = block->length;
        int32_t *smp_ptr = block->cur_ptr;
        int32_t val = *smp_ptr++;

        block->constant = 1;
        while (--n > 0) {
            if (*smp_ptr++ != val) {
                block->constant = 0;
                break;
            }
        }

        block->bits_const_block = 0;
        if (block->constant) {
            block->constant_value = val;
            block->bits_const_block += 6;   // const_block + reserved
            if (block->constant_value) {
                block->bits_const_block += ctx->sconf.floating ? 24 :
                                        ctx->avctx->bits_per_raw_sample;
            }
        }
    } else {
        block->constant = 0;
    }
}


/** Tests given block samples to share common zero LSBs.
 *  Sets block->shift_lsbs to the number common zero bits
 */
static void test_zero_lsb(ALSEncContext *ctx, ALSBlock *block)
{
    int i;
    int32_t common = 0;

    block->shift_lsbs = 0;

    if (ctx->cur_stage->check_lsbs) {
        // test for zero LSBs
        for (i = 0; i < (int)block->length; i++) {
            common |= block->cur_ptr[i];

            if (common & 1)
                return;
        }

        while (!(common & 1)) {
            block->shift_lsbs++;
            common >>= 1;
        }

        // generate shifted signal and point block->cur_ptr to the LSB buffer
        if (block->shift_lsbs) {
            for (i = -ctx->sconf.max_order; i < (int)block->length; i++)
                block->lsb_ptr[i] = block->cur_ptr[i] >> block->shift_lsbs;

            block->cur_ptr = block->lsb_ptr;
        }
    }
}


#if 1
/* Generate a weighted residual signal for autocorrelation detection
 * used in LTP mode
 */
static void get_weighted_signal(ALSEncContext *ctx, ALSBlock *block,
                                int lag_max)
{
    int len          = (int)block->length;
    int32_t *cur_ptr = block->cur_ptr;
    uint64_t sum      = 0;
    double *corr_ptr = ctx->ltp_corr_buffer + lag_max;
    double mean_quot;
    int i;

    // determine absolute mean of residual signal,
    // including previous samples
    for (i = -lag_max; i < len; i++)
        sum += abs(cur_ptr[i]);
    mean_quot = (double)(sum) / (block->length + lag_max);

    // apply weighting:
    // x *= 1 / [ sqrt(abs(x)) / 5*sqrt(mean) + 1 ]  // XXX: what weighting function is this?
    mean_quot = (sqrt(mean_quot) * 5);
    for (i = -lag_max; i < len; i++)
        corr_ptr[i] = 1 / (sqrt(abs(cur_ptr[i])) / mean_quot + 1);
}


/* Generate the autocorrelation function and find
 * its positive maximum value to be used for LTP lag
 */
static void find_best_autocorr(ALSEncContext *ctx, ALSBlock *block,
                               int lag_max, int start)
{
    int i, i_max;
    double autoc_max;
    double autoc[lag_max];
    double *corr_ptr = ctx->ltp_corr_buffer + lag_max;

    ff_lpc_compute_autocorr(block->cur_ptr, corr_ptr, block->length, lag_max, autoc);

    autoc_max = autoc[start];
    i_max     = start;
    for (i = start + 1; i < lag_max; i++) {
        // find best positive autocorrelation
        if (autoc[i] > 0 && autoc[i] > autoc_max) {
            autoc_max = autoc[i];
            i_max     = i;
        }
    }

    block->ltp_info[block->js_block].lag = i_max;
}

static void get_ltp_coeffs(ALSEncContext *ctx, ALSBlock *block,
                                int lag_max)
{
    int *ltp_gain = block->ltp_info[block->js_block].gain;

    ltp_gain[0] = 8;
    ltp_gain[1] = 8;
    ltp_gain[2] = 16;
    ltp_gain[3] = 8;
    ltp_gain[4] = 8;
}

#else

static void get_weighted_signal(ALSEncContext *ctx, ALSBlock *block,
                                int lag_max)
{
    int len          = (int)block->length;
    int32_t *cur_ptr = block->cur_ptr;
    uint64_t sum      = 0;
    double *corr_ptr = ctx->ltp_corr_buffer + lag_max;
    double mean_quot;
    int i;

    // determine absolute mean of residual signal,
    // including previous samples
    for (i = -lag_max; i < len; i++)
        sum += abs(cur_ptr[i]);
    mean_quot = (double)(sum) / (block->length + lag_max);

    // apply weighting:
    // x *= 1 / [ sqrt(abs(x)) / 5*sqrt(mean) + 1 ]  // XXX: what weighting function is this?
    mean_quot = (sqrt(mean_quot) * 5);
    for (i = -lag_max - 2; i < len; i++)
        corr_ptr[i] = cur_ptr[i] / (sqrt(abs(cur_ptr[i])) / mean_quot + 1);
}


static void find_best_autocorr(ALSEncContext *ctx, ALSBlock *block,
                               int lag_max, int start)
{
    int smp, lag;
    int begin        = 0;
    int end          = block->length;
    int lag_best     = 0;
    double *corr_ptr = ctx->ltp_corr_buffer + lag_max;
    double sum_a     = 0;
    double acorr_max = -1;


    // XXX: sum_a can become zero if there is no check
    // for constant zero signal prior to LTP!

    // calculate autocorrelation coefficient of original signal
    for (smp = begin - start; smp < end - start; smp++)
        sum_a += corr_ptr[smp] * corr_ptr[smp];

    // compute normalized autocorrelation for each possible lag
    for (lag = start; lag < lag_max; lag++) {
        double sum_b = 0;
        double acorr;
        int first = begin - lag - 1;
        int last  = end   - lag - 1;

        // calculate autocorrelation coefficient
        for (smp = begin - lag; smp < end - lag; smp++)
            sum_b += corr_ptr[smp + lag] * corr_ptr[smp];

        // skip negative autocorrelation coefficients
        if (sum_b >= 0) {
            // normalize autocorrelation coefficient
            acorr = sum_b * sum_b / sum_a;

            // save maximum autocorrelation
            if (acorr > acorr_max) {
                lag_best   = lag;
                acorr_max = acorr;
            }
        }

        // update autocorrelation coefficient of
        // original signal for next iteration
        sum_a += corr_ptr[first] * corr_ptr[first] -
                 corr_ptr[ last] * corr_ptr[ last];
    }

    block->ltp_info[block->js_block].lag = lag_best;
}

// XXX: replace me
///////////////////// begin of shameless adapted copy from reference
static void	Cholesky( double* a, double* b, const double* c, int n )
{
    int        i, j, k, zeroflag = 0;
    double    acc;
    static    const double    eps = 1.e-16;

    double t[ n*n ];
    double invt[ n ];


    t[0] = sqrt( a[0] + eps );
    invt[0] = 1. / t[0];
    for( k=1; k<n; k++ ) t[k*n] = a[k*n] * invt[0];
    for( i=1; i<n; i++ ) {
        acc = a[i*n+i] + eps;
        for( k=0; k<i; k++ ) acc -= t[i*n+k] * t[i*n+k];
        if(acc <= 0.0){ zeroflag=1; break;}
        t[i*n+i] = sqrt( acc );
        invt[i] = 1. / t[i*n+i];
        for( j=i+1; j<n; j++ ) {
            acc = a[j*n+i] + eps;
            for( k=0; k<i; k++ ) acc -= t[j*n+k] * t[i*n+k];
            t[j*n+i] = acc * invt[i];
        }
    }

    if(zeroflag) {
        for(i = 0; i < n; i++) b[i] = 0.0;
        return;
    }

    for( i=0; i<n; i++ ) {
        acc = c[i];
        for( k=0; k<i; k++ ) acc -= t[i*n+k] * a[k];
        a[i] = acc * invt[i];
    }

    for( i=n-1; i>=0; i-- ) {
        acc = a[i];
        for( k=i+1; k<n; k++ ) acc -= t[k*n+i] * b[k];
        b[i] = acc * invt[i];
    }
}


static void get_ltp_coeffs(ALSEncContext *ctx, ALSBlock *block,
                                int lag_max)
{
    int icc;
    int smp, i;
    int taumax = block->ltp_info[block->js_block].lag;
    double *corr_ptr = ctx->ltp_corr_buffer + lag_max;

    double ioa[25], ob[5], ic[5];
    double powc0  = 0.0, powc1  = 0.0, powc2  = 0.0, powc3  = 0.0, powc4  = 0.0;
    double pwm2m2 = 0.0, pwm2m1 = 0.0, pwm20  = 0.0, pwm2p1 = 0.0, pwm2p2 = 0.0;
    double pwm1m1 = 0.0, pwm10  = 0.0, pwm1p1 = 0.0, pwm1p2 = 0.0;
    double pw00   = 0.0, pw0p1  = 0.0, pw0p2  = 0.0;
    double pwp1p1 = 0.0, pwp1p2 = 0.0;
    double pwp2p2 = 0.0;

    for( smp=-taumax; smp<(int)(block->length)-taumax-2; smp++ ) {
        powc0 += corr_ptr[smp+taumax] * corr_ptr[smp-2];
        powc1 += corr_ptr[smp+taumax] * corr_ptr[smp-1];
        powc2 += corr_ptr[smp+taumax] * corr_ptr[smp];
        powc3 += corr_ptr[smp+taumax] * corr_ptr[smp+1];
        powc4 += corr_ptr[smp+taumax] * corr_ptr[smp+2];

        pwm2m2 += corr_ptr[smp-2] * corr_ptr[smp-2];
        pwm2m1 += corr_ptr[smp-2] * corr_ptr[smp-1];
        pwm20  += corr_ptr[smp-2] * corr_ptr[smp];
        pwm2p1 += corr_ptr[smp-2] * corr_ptr[smp+1];
        pwm2p2 += corr_ptr[smp-2] * corr_ptr[smp+2];
        pwm1m1 += corr_ptr[smp-1] * corr_ptr[smp-1];
        pwm10  += corr_ptr[smp-1] * corr_ptr[smp];
        pwm1p1 += corr_ptr[smp-1] * corr_ptr[smp+1];
        pwm1p2 += corr_ptr[smp-1] * corr_ptr[smp+2];
        pw00   += corr_ptr[smp]   * corr_ptr[smp];
        pw0p1  += corr_ptr[smp]   * corr_ptr[smp+1];
        pw0p2  += corr_ptr[smp]   * corr_ptr[smp+2];
        pwp1p1 += corr_ptr[smp+1] * corr_ptr[smp+1];
        pwp1p2 += corr_ptr[smp+1] * corr_ptr[smp+2];
        pwp2p2 += corr_ptr[smp+2] * corr_ptr[smp+2];
    }

    ic[0] = powc0;
    ic[1] = powc1;
    ic[2] = powc2;
    ic[3] = powc3;
    ic[4] = powc4;
    ioa[ 0] = pwm2m2;
    ioa[ 1] = ioa[ 5] = pwm2m1;
    ioa[ 2] = ioa[10] = pwm20;
    ioa[ 3] = ioa[15] = pwm2p1;
    ioa[ 4] = ioa[20] = pwm2p2;
    ioa[ 6] = pwm1m1;
    ioa[ 7] = ioa[11] = pwm10;
    ioa[ 8] = ioa[16] = pwm1p1;
    ioa[ 9] = ioa[21] = pwm1p2;
    ioa[12] = pw00;
    ioa[13] = ioa[17] = pw0p1;
    ioa[14] = ioa[22] = pw0p2;
    ioa[18] = pwp1p1;
    ioa[19] = ioa[23] = pwp1p2;
    ioa[24] = pwp2p2;

    Cholesky(ioa, ob, ic, 5);


    // calculate coefficients
    {
        // 0,1 and 3,4
        int *ltp_gain = block->ltp_info[block->js_block].gain;
        int quant     = ceil(ob[2] * 256 + 0.5);

        for(icc=0; icc<5; icc++) {
            if (icc == 2) continue;
            ltp_gain[icc] = ob[icc] * 16 + ((ob[icc] > 0) ? 0.5 : -0.5);
            ltp_gain[icc] = av_clip(ltp_gain[icc], -8, 8) << 3;
        }

        // 2
        ltp_gain[2] = 0;
        for(i = 15; i >= 1; i--) {
            uint8_t cur = ff_als_ltp_gain_values[i >> 2][i & 3];

            if (quant > cur + ff_als_ltp_gain_values[(i-1) >> 2][(i-1) & 3]) {
                ltp_gain[2] = cur;
                return;
            }
        }
    }
}
///////////////////// end of shameless adapted copy from reference
#endif



/** Select the best set of LTP parameters based on maximum autocorrelation
 *  value of the weighted residual signal
 */
static void find_block_ltp_params(ALSEncContext *ctx, ALSBlock *block)
{
    AVCodecContext *avctx    = ctx->avctx;

    int start   = FFMAX(4, block->opt_order + 1);
    int lag     = 256 << (  (avctx->sample_rate >=  96000)
                          + (avctx->sample_rate >= 192000));
    int lag_max = FFMIN(lag + start, FFMIN(ALS_MAX_LTP_LAG, block->length));

    get_weighted_signal(ctx, block, lag_max);
    find_best_autocorr (ctx, block, lag_max, start);
    get_ltp_coeffs     (ctx, block, lag_max);
}


/** Generate the long-term predicted residuals for a given block
 *  using the current set of LTP parameters
 */
static void gen_ltp_residuals(ALSEncContext *ctx, ALSBlock *block)
{
    ALSLTPInfo *ltp  = &block->ltp_info[block->js_block];
    int32_t *ltp_ptr = ctx->ltp_buffer;
    int offset = FFMAX(ltp->lag - 2, 0);
    unsigned int ltp_smp;
    int64_t y;

    memcpy(ltp_ptr, block->cur_ptr, sizeof(*ltp_ptr) * offset);

    for (ltp_smp = offset; ltp_smp < block->length; ltp_smp++) {
        int center = ltp_smp - ltp->lag;
        int begin  = FFMAX(0, center - 2);
        int end    = center + 3;
        int tab    = 5 - (end - begin);
        int base;

        y = 1 << 6;

        for (base = begin; base < end; base++, tab++)
            y += MUL64(ltp->gain[tab], block->cur_ptr[base]);

        ltp_ptr[ltp_smp] = block->cur_ptr[ltp_smp] - (y >> 7);
    }
}


/** Encode a given block of a given channel
 * @return number of bits that will be used to encode the block using the
 *         determined parameters
 */
static int find_block_params(ALSEncContext *ctx, ALSBlock *block)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    int bit_count, max_order;
    ALSLTPInfo *ltp     = &block->ltp_info[block->js_block];
    ALSEntropyInfo *ent = &block->ent_info[ltp->use_ltp];

    block->cur_ptr = block->js_block ? block->dif_ptr : block->smp_ptr;

    block->bits_misc = 1;   // block_type

    // check for constant block
    //
    // Read 1st sample, then check remaining samples.  Benchmark
    // with and w/o the check.  Maybe can be skipped for only the fastest mode.

    test_const_value(ctx, block);


    // shifting samples:
    // to be implemented
    //
    // Again, maybe skip in fastest mode.  The flags
    // to indicate use of these sorts of searches should be in the context and
    // set during init based on compression_level.
    //
    // determine if the samples can be shifted
    // while not implemented, don't shift anyway

    if (!block->constant) {
        test_zero_lsb(ctx, block);
        block->bits_misc++;         // shift_lsbs
        if (block->shift_lsbs)
            block->bits_misc += 4;  // shift_pos
    }


    // difference coding(?):
    // to be implemented
    //
    // choose if this block can be difference coded if joint-stereo is enabled
    // (and the block can be encoded in a channel pair)
    // while not implemented, don't indicate js

    block->bits_misc++; // add one bit for block->js_block


    // if this is a constant block, we don't need to find any other parameters
    if (block->constant)
        return block->bits_misc + block->bits_const_block;


    // short-term prediction:
    // use the mode chosen at encode_init() to find optimal parameters
    //
    // LPC / PARCOR coefficients to be stored in context
    // they depend on js_block and opt_order which may be changing later on

    max_order = ctx->cur_stage->max_order;
    if (sconf->max_order) {
        if (sconf->adapt_order) {
            int opt_order_length = av_ceil_log2(av_clip((block->length >> 3) - 1,
                                                2, sconf->max_order + 1));
            max_order = FFMIN(sconf->max_order, (1 << opt_order_length) - 1);
        }

        // calculate PARCOR coefficients
        if (block->div_block >= 0)
            ctx->dsp.lpc_compute_autocorr(block->cur_ptr, ctx->acf_window[block->div_block],
                                          block->length, max_order,
                                          ctx->acf_coeff);
        else
            ctx->dsp.lpc_compute_autocorr(block->cur_ptr, NULL, block->length,
                                          max_order, ctx->acf_coeff);
        compute_ref_coefs(ctx->acf_coeff, max_order, ctx->parcor_coeff);

        // quantize PARCOR coefficients to 7-bit and reconstruct to 21-bit
        quantize_parcor_coeffs(ctx->parcor_coeff, max_order,
                               block->q_parcor_coeff, ctx->r_parcor_coeff);
    }


    // Determine optimal LPC order:
    //
    // quick estimate for LPC order. better searches will give better
    // significantly better results.
    if (sconf->max_order && sconf->adapt_order && ctx->cur_stage->adapt_order) {
        if (ctx->cur_stage->adapt_algorithm == ADAPT_ORDER_ALGORITHM_THRESHOLD) {
            block->opt_order = estimate_best_order(ctx->r_parcor_coeff, max_order);
        } else if (ctx->cur_stage->adapt_algorithm == ADAPT_ORDER_ALGORITHM_FULL_SEARCH) {
            find_block_adapt_order_full_search(ctx, block, max_order);
        }
    } else {
        block->opt_order = max_order;
    }
    calc_parcor_coeff_bit_size(ctx, block, block->opt_order);


    // generate residuals using parameters:

    if (block->opt_order) {
        if (calc_short_term_prediction(ctx, block, block->opt_order)) {
            // if PARCOR to LPC conversion has 32-bit integer overflow,
            // fallback to using 1st order prediction
            double parcor[block->opt_order];

            if (ctx->cur_stage->adapt_order)
                block->opt_order = 1;

            memset(parcor, 0, sizeof(parcor));
            parcor[0] = -0.9;
            quantize_parcor_coeffs(parcor, block->opt_order,
                                   block->q_parcor_coeff, ctx->r_parcor_coeff);

            calc_parcor_coeff_bit_size(ctx, block, block->opt_order);
            calc_short_term_prediction(ctx, block, block->opt_order);
        }
        block->cur_ptr = block->res_ptr;
    }


    // final joint or multi channel coding:
    // to be implemented
    //
    // up to now only independent coding...


    // search for rice parameter:
    ltp->use_ltp = 0;
    ent = &block->ent_info[ltp->use_ltp];
    find_block_entropy_params(ctx, block, block->opt_order);

    bit_count = block->bits_misc + block->bits_parcor_coeff +
                ent->bits_ec_param_and_res + ltp->bits_ltp;
    bit_count += (8 - (bit_count & 7)) & 7; // byte align


    // determine lag and gain values for long-term prediction and
    // check if long-term prediction pays off for this block
    // and use LTP for this block if it does
    if (sconf->long_term_prediction)
        check_ltp(ctx, block, &bit_count);

    return bit_count;
}


/** Generates all possible block sizes for all possible block-switching stages
 */
static void gen_sizes(ALSEncContext *ctx, unsigned int channel, int stage)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    ALSBlock *block          = ctx->blocks[channel];
    unsigned int num_blocks  = sconf->block_switching ? (1 << stage) : 1;
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
        unsigned int *bs_sizes = ctx->bs_sizes[channel     ] + num_blocks - 1;
        unsigned int *js_sizes = ctx->js_sizes[channel >> 1] + num_blocks - 1;

        // count residuals + block overhead
        block->js_block = 0;
        bs_sizes[b] = find_block_params(ctx, block);

        if (sconf->joint_stereo && !(channel & 1)) {
            block->js_block = 1;
            js_sizes[b]    = find_block_params(ctx, block);
            block->js_block = 0;
        }

        block++;
    }

    if (sconf->block_switching && stage < sconf->block_switching + 2)
        gen_sizes(ctx, channel, stage + 1);
    else
        ctx->bs_info[channel] = bs_info_tmp;
}


/** Generates all suitable difference coding infos for all possible block-switching stages
 */
static void gen_js_infos(ALSEncContext *ctx, unsigned int channel, int stage)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int num_blocks  = sconf->block_switching ? (1 << stage) : 1;
    unsigned int b;

    for (b = 0; b < num_blocks; b++) {
        unsigned int *block_size = ctx->bs_sizes[channel     ] + num_blocks - 1;
        unsigned int *buddy_size = ctx->bs_sizes[channel +  1] + num_blocks - 1;
        unsigned int *js_size    = ctx->js_sizes[channel >> 1] + num_blocks - 1;
        uint8_t      *js_info    = ctx->js_infos[channel >> 1] + num_blocks - 1;

        // replace normal signal with difference signal if suitable
        if (js_size[b] < block_size[b] ||
            js_size[b] < buddy_size[b]) {
            // mark the larger encoded block with the difference signal
            // and update this blocks size in bs_sizes
            if (block_size[b] > buddy_size[b]) {
                js_info[b] = 1;
            } else {
                js_info[b] = 2;
            }
        } else {
            js_info[b] = 0;
        }
    }

    if (sconf->block_switching && stage < sconf->block_switching + 2)
        gen_js_infos(ctx, channel, stage + 1);
}


/** Use all suitable difference coding infos for all possible block-switching stages
 */
static void use_js_sizes(ALSEncContext *ctx, unsigned int channel, int stage)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int num_blocks  = sconf->block_switching ? (1 << stage) : 1;
    unsigned int b;

    for (b = 0; b < num_blocks; b++) {
        unsigned int *block_size = ctx->bs_sizes[channel     ] + num_blocks - 1;
        unsigned int *buddy_size = ctx->bs_sizes[channel +  1] + num_blocks - 1;
        unsigned int *js_size    = ctx->js_sizes[channel >> 1] + num_blocks - 1;
        uint8_t      *js_info    = ctx->js_infos[channel >> 1] + num_blocks - 1;

        // replace normal signal size with
        // difference signal size if suitable
        if        (js_info[b] == 1) {
            FFSWAP(unsigned int, block_size[b], js_size[b]);
        } else if (js_info[b] == 2)
            FFSWAP(unsigned int, buddy_size[b], js_size[b]);
    }

    if (sconf->block_switching && stage < sconf->block_switching + 2)
        use_js_sizes(ctx, channel, stage + 1);
}


/** Reset all difference coding infos for all possible block-switching stages
 */
static void reset_js(ALSEncContext *ctx, unsigned int channel, int stage)
{
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int num_blocks  = sconf->block_switching ? (1 << stage) : 1;
    unsigned int b;

    for (b = 0; b < num_blocks; b++) {
        ALSBlock     *blocks     = ctx->blocks  [channel     ] + num_blocks - 1;
        ALSBlock     *buddys     = ctx->blocks  [channel +  1] + num_blocks - 1;
        unsigned int *block_size = ctx->bs_sizes[channel     ] + num_blocks - 1;
        unsigned int *buddy_size = ctx->bs_sizes[channel +  1] + num_blocks - 1;
        unsigned int *js_size    = ctx->js_sizes[channel >> 1] + num_blocks - 1;
        uint8_t      *js_info    = ctx->js_infos[channel >> 1] + num_blocks - 1;

        // replace normal signal size with
        // difference signal size if suitable
        if        (js_info[b] == 1) {
            FFSWAP(unsigned int, block_size[b], js_size[b]);
        } else if (js_info[b] == 2)
            FFSWAP(unsigned int, buddy_size[b], js_size[b]);

        js_info[b] = 0;
        blocks [b].js_block = 0;
        buddys [b].js_block = 0;
    }

    if (sconf->block_switching && stage < sconf->block_switching + 2)
        reset_js(ctx, channel, stage + 1);
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


    // determine if this is an RA frame
    if (sconf->ra_distance) {
        // TODO: should be based on frame_id and ra_distance
        int ra_frame = 1;
        for (c = 0; c < avctx->channels; c++)
            ctx->blocks[c][0].ra_block = ra_frame;
    }


    // update CRC
    if (sconf->crc_enabled) {
        frame_data_size = (ctx->avctx->bits_per_raw_sample > 16) ? 4 : 2;
        ctx->crc        = av_crc(ctx->crc_table, ctx->crc, data,
                                 ctx->cur_frame_length * avctx->channels *
                                 frame_data_size);
    }


    // preprocessing
    deinterleave_raw_samples(ctx, data);


    // find optimal encoding parameters

    SET_OPTIONS(STAGE_JOINT_STEREO)
    select_difference_coding_mode(ctx);

    SET_OPTIONS(STAGE_BLOCK_SWITCHING);
    block_partitioning(ctx);

    SET_OPTIONS(STAGE_FINAL);
    if (!sconf->mc_coding || ctx->js_switch) {
        for (b = 0; b < ALS_MAX_BLOCKS; b++) {
            for (c = 0; c < avctx->channels; c++) {
                if (b >= ctx->num_blocks[c])
                    continue;

                if (ctx->independent_bs[c]) {
                    find_block_params(ctx, &ctx->blocks[c][b]);
                } else {
                    find_block_params(ctx, &ctx->blocks[c    ][b]);
                    find_block_params(ctx, &ctx->blocks[c + 1][b]);
                    c++;
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

    // choose default frame size if not specified by the user
    if (avctx->frame_size <= 0) {
        if (avctx->sample_rate <= 24000)
            avctx->frame_size = 1024;
        else if(avctx->sample_rate <= 48000)
            avctx->frame_size = 2048;
        else if(avctx->sample_rate <= 96000)
            avctx->frame_size = 4096;
        else
            avctx->frame_size = 8192;
    }

    // ensure a certain boundary for the frame size
    // frame length - 1 in ALSSpecificConfig is 16-bit, so max value is 65536
    // frame size == 1 is not allowed because it is used in ffmpeg as a
    // special-case value to indicate PCM audio
    avctx->frame_size = av_clip(avctx->frame_size, 2, 65536);

    sconf->frame_length = avctx->frame_size;
}


static av_cold int get_specific_config(AVCodecContext *avctx)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    const ALSSpecificConfig *compr;


    // total number of samples unknown
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


    // set default compression level and clip to allowed range
    if (avctx->compression_level == FF_COMPRESSION_DEFAULT)
        avctx->compression_level = 1;
    else
        avctx->compression_level = av_clip(avctx->compression_level, 0, 2);
    compr = spc_config_settings[avctx->compression_level];


    // determine frame length
    frame_partitioning(ctx);


    // determine distance between ra-frames. 0 = no ra, 1 = all ra
    // default for now = 1. should be changed when implemented.
    // maybe use AVCodecContext.gop_size. it is user-configurable, and its
    // default value is 12, which is every 1/2 sec. for 2048 frame size and
    // 44100 Hz sample rate.
    sconf->ra_distance = 1;


    // determine where to store ra_flag (01: beginning of frame_data)
    // default for now = RA_FLAG_NONE.
    // Using RA_FLAG_FRAMES would make decoding more robust in case of seeking
    // with raw ALS.  However, raw ALS is not supported in FFmpeg yet.
    sconf->ra_flag = RA_FLAG_NONE;


    // determine if adaptive prediction order is used.
    // since the current implementation is excruciatingly slow, don't use it
    sconf->adapt_order = compr->adapt_order;


    // determine the coef_table to be used
    sconf->coef_table = (avctx->sample_rate > 48000) +
                        (avctx->sample_rate > 96000);


    // determine if long-term prediction is used
    // should also be user-defineable
    sconf->long_term_prediction = compr->long_term_prediction;


    // determine a maximum prediction order
    // no samples: not yet possible to determine..
    // TODO: do evaluation to find a suitable standard value?
    //       if adapt_order is set and compression level is high,
    //       use maximum value to be able to find the best order
    sconf->max_order = compr->max_order;
    if (avctx->max_prediction_order >= 0)
        sconf->max_order = av_clip(avctx->max_prediction_order, 0, 1023);


    // determine if block-switching is used
    // depends on simple profile or not (user-defined?)
    // may be always enable this by default with the maximum level,
    // simple profile supports up to 3 stages
    // disable for the fastest compression mode
    // should be set when implemented
    sconf->block_switching = compr->block_switching;


    // limit the block_switching depth based on whether the full frame length
    // is evenly divisible by the minimum block size.
    // FIXME: should we do this in sconf or just limit the actual depth used?
    while (sconf->block_switching > 0 &&
           sconf->frame_length % (1 << (sconf->block_switching + 2))) {
        sconf->block_switching--;
    }


    // determine if BGMC mode is used
    // should be user-defineable
    sconf->bgmc = compr->bgmc;
    if (avctx->coder_type == FF_CODER_TYPE_AC)
        sconf->bgmc = 1;


    // determine what sub-block partitioning is used
    sconf->sb_part = compr->sb_part;


    // determine if joint-stereo is used
    // planned to be determined for each frame
    // set = 1 if #channels > 1 (?)
    // should be set when implemented
    // turn off for fastest compression level
    sconf->joint_stereo = compr->joint_stereo;


    // determine if multi-channel coding is used
    // enable for better compression levels if implemented
    // may be sanity check: channels > 2
    // (although specs allow mc_coding for 2 channels...
    // maybe give a warning)
    sconf->mc_coding = compr->mc_coding;


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
    sconf->rlslms = compr->rlslms;


    // determine if CRC checksums are used
    // depends on compression level
    // should be enabled by default, not to use
    // in the fastest mode
    sconf->crc_enabled = compr->crc_enabled;


    // print ALSSpecificConfig info
    ff_als_dprint_specific_config(avctx, sconf);


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
    header_size += (sconf->crc_enabled > 0) << 2;                       // crc
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
    put_bits  (&pb,  1, sconf->crc_enabled);    // crc_enabled
    put_bits  (&pb,  1, sconf->rlslms);
    put_bits  (&pb,  5, 0);                     // reserved bits
    put_bits  (&pb,  1, 0);                     // aux_data_enabled (0 = false)

    // align
    align_put_bits(&pb);

    put_bits32(&pb, 0);                         // original header size
    put_bits32(&pb, 0);                         // original trailer size
    put_bits32(&pb, ctx->crc);                  // CRC


    // writing in local header finished,
    // set the real size
    avctx->extradata_size = put_bits_count(&pb) >> 3;

    return 0;
}


static av_cold int encode_end(AVCodecContext *avctx)
{
    ALSEncContext *ctx = avctx->priv_data;

    av_freep(&ctx->stages);
    av_freep(&ctx->independent_bs);
    av_freep(&ctx->raw_buffer);
    av_freep(&ctx->raw_samples);
    av_freep(&ctx->raw_dif_buffer);
    av_freep(&ctx->raw_dif_samples);
    av_freep(&ctx->raw_lsb_buffer);
    av_freep(&ctx->raw_lsb_samples);
    av_freep(&ctx->res_buffer);
    av_freep(&ctx->res_samples);
    av_freep(&ctx->block_buffer);
    av_freep(&ctx->blocks);
    av_freep(&ctx->bs_info);
    av_freep(&ctx->num_blocks);
    av_freep(&ctx->bs_sizes_buffer);
    av_freep(&ctx->bs_sizes);
    av_freep(&ctx->js_sizes_buffer);
    av_freep(&ctx->js_sizes);
    av_freep(&ctx->js_infos_buffer);
    av_freep(&ctx->js_infos);
    av_freep(&ctx->q_parcor_coeff_buffer);
    av_freep(&ctx->acf_coeff);
    av_freep(&ctx->parcor_coeff);
    av_freep(&ctx->r_parcor_coeff);
    av_freep(&ctx->lpc_coeff);
    av_freep(&ctx->acf_window);
    av_freep(&ctx->ltp_buffer);
    av_freep(&ctx->ltp_corr_buffer);

    av_freep(&avctx->extradata);
    avctx->extradata_size = 0;
    av_freep(&avctx->coded_frame);

    return 0;
}


static void init_rect_window(double *window, int len)
{
    int i;
    for (i = 0; i < len; i++)
        window[i] = 1.0;
}


static void init_hannrect_window(double *window, int len, double param)
{
    int i;
    int side_len = lrint(len / (2 * param));
    double phi   = param * 2.0 * M_PI / (len - 1);

    init_rect_window(window, len);

    if (side_len < 4)
        return;

    for (i = 0; i < side_len; i++) {
        double w = 0.5 - 0.5 * cos(phi * i);
        window[i]       = w;
        window[len-i-1] = w;
    }
}


static void init_sinerect_window(double *window, int len, double param)
{
    int i;
    int side_len = lrint(len / (2 * param));
    double phi   = param * M_PI / (len - 1);

    init_rect_window(window, len);

    if (side_len < 4)
        return;

    for (i = 0; i < side_len; i++) {
        double w = sin(phi * i);
        window[i]       = w;
        window[len-i-1] = w;
    }
}


static av_cold int encode_init(AVCodecContext *avctx)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int channel_size, channel_offset;
    int ret, b, c;
    int num_bs_sizes;

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


    channel_offset = sconf->long_term_prediction ? ALS_MAX_LTP_LAG : sconf->max_order;
    channel_size = sconf->frame_length + channel_offset;


    // set up stage options
    ctx->stages = av_malloc(sizeof(*ctx->stages) * NUM_STAGES);
    if (!ctx->stages) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        encode_end(avctx);
        return AVERROR(ENOMEM);
    }

    /* fill stage options based on compression level */
    ctx->stages[STAGE_JOINT_STEREO]     = *stage_js_settings[avctx->compression_level];
    ctx->stages[STAGE_BLOCK_SWITCHING]  = *stage_bs_settings[avctx->compression_level];
    ctx->stages[STAGE_FINAL]            = *stage_final_settings[avctx->compression_level];

    /* joint-stereo stage sconf overrides */
    ctx->stages[STAGE_JOINT_STEREO].adapt_order = sconf->adapt_order;
    ctx->stages[STAGE_JOINT_STEREO].sb_part     = sconf->sb_part;
    if (avctx->compression_level > 1)
        ctx->stages[STAGE_JOINT_STEREO].max_order = sconf->max_order;

    /* block switching stage sconf overrides */
    ctx->stages[STAGE_BLOCK_SWITCHING].adapt_order = sconf->adapt_order;
    ctx->stages[STAGE_BLOCK_SWITCHING].sb_part     = sconf->sb_part;
    if (avctx->compression_level > 0)
        ctx->stages[STAGE_BLOCK_SWITCHING].max_order = sconf->max_order;

    /* final stage sconf overrides */
    ctx->stages[STAGE_FINAL].adapt_order = sconf->adapt_order;
    ctx->stages[STAGE_FINAL].sb_part     = sconf->sb_part;
    ctx->stages[STAGE_FINAL].max_order   = sconf->max_order;
    if (sconf->bgmc && avctx->compression_level < 2) {
        ctx->stages[STAGE_FINAL].ecsub_algorithm = EC_SUB_ALGORITHM_RICE_ESTIMATE;
        ctx->stages[STAGE_FINAL].param_algorithm = EC_PARAM_ALGORITHM_BGMC_ESTIMATE;
    }

    dprintf(avctx, "\n");
    dprintf(avctx, "Joint-Stereo:\n");
    dprint_stage_options(avctx, &ctx->stages[STAGE_JOINT_STEREO]);
    dprintf(avctx, "\n");
    dprintf(avctx, "Block-Switching:\n");
    dprint_stage_options(avctx, &ctx->stages[STAGE_BLOCK_SWITCHING]);
    dprintf(avctx, "\n");
    dprintf(avctx, "Final:\n");
    dprint_stage_options(avctx, &ctx->stages[STAGE_FINAL]);
    dprintf(avctx, "\n");


    // set cur_stage pointer to the first stage
    ctx->cur_stage = ctx->stages;


    // allocate buffers
    ctx->independent_bs    = av_malloc (sizeof(*ctx->independent_bs) * avctx->channels);
    ctx->raw_buffer        = av_mallocz(sizeof(*ctx->raw_buffer)     * avctx->channels * channel_size);
    ctx->raw_samples       = av_malloc (sizeof(*ctx->raw_samples)    * avctx->channels);
    ctx->raw_dif_buffer    = av_mallocz(sizeof(*ctx->raw_dif_buffer)  * (avctx->channels >> 1) * channel_size);
    ctx->raw_dif_samples   = av_malloc (sizeof(*ctx->raw_dif_samples) * (avctx->channels >> 1));
    ctx->raw_lsb_buffer    = av_mallocz(sizeof(*ctx->raw_lsb_buffer)  * (avctx->channels) * channel_size);
    ctx->raw_lsb_samples   = av_malloc (sizeof(*ctx->raw_lsb_samples) * (avctx->channels));
    ctx->res_buffer        = av_mallocz(sizeof(*ctx->raw_buffer)     * avctx->channels * channel_size);
    ctx->res_samples       = av_malloc (sizeof(*ctx->res_samples)    * avctx->channels);
    ctx->num_blocks        = av_malloc (sizeof(*ctx->num_blocks)     * avctx->channels);
    ctx->bs_info           = av_malloc (sizeof(*ctx->bs_info)        * avctx->channels);
    ctx->block_buffer      = av_mallocz(sizeof(*ctx->block_buffer)   * avctx->channels * ALS_MAX_BLOCKS);
    ctx->blocks            = av_malloc (sizeof(*ctx->blocks)         * avctx->channels);
    ctx->q_parcor_coeff_buffer = av_malloc (sizeof(*ctx->q_parcor_coeff_buffer) * avctx->channels * ALS_MAX_BLOCKS * sconf->max_order);
    ctx->acf_coeff         = av_malloc (sizeof(*ctx->acf_coeff)      *(sconf->max_order + 1));
    ctx->parcor_coeff      = av_malloc (sizeof(*ctx->parcor_coeff)   * sconf->max_order);
    ctx->lpc_coeff         = av_malloc (sizeof(*ctx->lpc_coeff)      * sconf->max_order);
    ctx->r_parcor_coeff    = av_malloc (sizeof(*ctx->r_parcor_coeff) * sconf->max_order);
    ctx->acf_window_buffer = av_malloc (sizeof(*ctx->acf_window_buffer) * sconf->frame_length * 2);

    // check buffers
    if (!ctx->independent_bs    ||
        !ctx->raw_buffer        || !ctx->raw_samples     ||
        !ctx->raw_dif_buffer    || !ctx->raw_dif_samples ||
        !ctx->raw_lsb_buffer    || !ctx->raw_lsb_samples ||
        !ctx->res_buffer        || !ctx->res_samples     ||
        !ctx->num_blocks        || !ctx->bs_info         ||
        !ctx->block_buffer      || !ctx->blocks) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        encode_end(avctx);
        return AVERROR(ENOMEM);
    }

    if (sconf->long_term_prediction) {
        ctx->ltp_buffer = av_malloc(sizeof(*ctx->ltp_buffer) * sconf->frame_length);
        ctx->ltp_corr_buffer = av_malloc (sizeof(*ctx->ltp_corr_buffer) *
                                          (sconf->frame_length +
                                           FFMIN(ALS_MAX_LTP_LAG, sconf->frame_length)));

        if (!ctx->ltp_buffer || !ctx->ltp_corr_buffer) {
            av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
            encode_end(avctx);
            return AVERROR(ENOMEM);
        }
    }

    // assign buffer pointers
    ctx->raw_samples    [0] = ctx->raw_buffer     + channel_offset;
    ctx->raw_dif_samples[0] = ctx->raw_dif_buffer + channel_offset;
    ctx->raw_lsb_samples[0] = ctx->raw_lsb_buffer + channel_offset;
    ctx->res_samples    [0] = ctx->res_buffer     + channel_offset;
    ctx->blocks         [0] = ctx->block_buffer;

    for (c = 1; c < avctx->channels; c++) {
        ctx->raw_samples[c] = ctx->raw_samples[c - 1] + channel_size;
        ctx->res_samples[c] = ctx->res_samples[c - 1] + channel_size;
        ctx->raw_lsb_samples[c] = ctx->raw_lsb_samples[c - 1] + channel_size;
        ctx->blocks     [c] = ctx->blocks     [c - 1] + 32;
    }

    for (c = 1; c < (avctx->channels >> 1); c++) {
        ctx->raw_dif_samples[c] = ctx->raw_dif_samples[c - 1] + channel_size;
    }

    ctx->blocks[0][0].q_parcor_coeff = ctx->q_parcor_coeff_buffer;
    for (c = 0; c < avctx->channels; c++) {
        for (b = 0; b < ALS_MAX_BLOCKS; b++) {
            if (b)
                ctx->blocks[c][b].q_parcor_coeff = ctx->blocks[c][b-1].q_parcor_coeff + sconf->max_order;
            else if (c)
                ctx->blocks[c][b].q_parcor_coeff = ctx->blocks[c-1][0].q_parcor_coeff + ALS_MAX_BLOCKS * sconf->max_order;
        }
    }


    // channel sorting
    if ((sconf->joint_stereo || sconf->mc_coding) && sconf->chan_sort)
        channel_sorting(ctx);


    // allocate block-switching and joint-stereo buffers
    num_bs_sizes = (8 << sconf->block_switching) - 1;

    ctx->bs_sizes_buffer = av_malloc(sizeof(*ctx->bs_sizes_buffer) * num_bs_sizes * avctx->channels);
    ctx->bs_sizes        = av_malloc(sizeof(*ctx->bs_sizes)        * num_bs_sizes);
    ctx->js_sizes_buffer = av_malloc(sizeof(*ctx->js_sizes_buffer) * num_bs_sizes * (avctx->channels >> 1));
    ctx->js_sizes        = av_malloc(sizeof(*ctx->js_sizes)        * num_bs_sizes);
    ctx->js_infos_buffer = av_malloc(sizeof(*ctx->js_infos_buffer) * num_bs_sizes * (avctx->channels >> 1));
    ctx->js_infos        = av_malloc(sizeof(*ctx->js_infos)        * num_bs_sizes);

    if (!ctx->bs_sizes || !ctx->bs_sizes_buffer ||
        !ctx->js_sizes || !ctx->js_sizes_buffer ||
        !ctx->js_infos || !ctx->js_infos_buffer) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        encode_end(avctx);
        return AVERROR(ENOMEM);
    }

    for (c = 0; c < avctx->channels; c++) {
        ctx->bs_sizes[c] = ctx->bs_sizes_buffer + c * num_bs_sizes;
    }

    for (c = 0; c < avctx->channels - 1; c += 2) {
        ctx->js_sizes[c    ] = ctx->js_sizes_buffer + (c    ) * num_bs_sizes;
        ctx->js_sizes[c + 1] = ctx->js_sizes_buffer + (c + 1) * num_bs_sizes;
        ctx->js_infos[c    ] = ctx->js_infos_buffer + (c    ) * num_bs_sizes;
        ctx->js_infos[c + 1] = ctx->js_infos_buffer + (c + 1) * num_bs_sizes;
    }


    avctx->coded_frame = avcodec_alloc_frame();
    avctx->coded_frame->key_frame = 1;


    dsputil_init(&ctx->dsp, avctx);

    // initialize autocorrelation window for each block size
    ctx->acf_window[0] = ctx->acf_window_buffer;
    for (b = 0; b <= sconf->block_switching + 2; b++) {
        int block_length = sconf->frame_length / (1 << b);
        if (avctx->sample_rate <= 48000)
            init_sinerect_window(ctx->acf_window[b], block_length, 4);
        else
            init_hannrect_window(ctx->acf_window[b], block_length, 4);
        ctx->acf_window[b+1] = ctx->acf_window[b] + block_length;
        if (!sconf->block_switching)
            break;
    }

    // initialize CRC calculation
    if (sconf->crc_enabled) {
        ctx->crc_table = av_crc_get_table(AV_CRC_32_IEEE_LE);
        ctx->crc       = 0xFFFFFFFFL;
    }

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
