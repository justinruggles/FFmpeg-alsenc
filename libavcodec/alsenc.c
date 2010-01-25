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

#include "avcodec.h"
#include "put_bits.h"
#include "mpeg4audio.h"
#include "audioconvert.h"


#define ALS_SPECIFIC_CFG_SIZE  30
#define ALS_EXTRADATA_MAX_SIZE (6 + ALS_SPECIFIC_CFG_SIZE)


// probably mergeable or the very same as ALSBlockData from the decoder
typedef struct {
    int constant;                   ///< indicates constant block values
    int32_t constant_value;         ///< if constant, this is the value
    unsigned int length;            ///< length of the block in # of samples
    unsigned int sub_blocks;        ///< number of entropy coding sub-blocks in this block
    unsigned int rice_param;        ///< rice parameter to encode the residuals
                                    ///< of this block in case of not bgmc
                                    ///< has to be an array if sub_blocks are implemented!
    unsigned int opt_order;         ///< prediction order for this block
    unsigned int js_block;          ///< indicates actual usage of joint-stereo coding
    unsigned int shift_lsbs;        ///< number of bits the samples have been right shifted
} ALSBlock;


typedef struct {
    AVCodecContext *avctx;
    ALSSpecificConfig sconf;
    PutBitContext pb;
    unsigned int cur_frame_length;   ///< length of the current frame, in samples
    int js_switch;                   ///< force joint-stereo in case of MCC
    int *independent_bs;             ///< array containing independent_bs flag for each channel
    int32_t *raw_buffer;             ///< buffer containing all raw samples of the frame plus max_order samples from the previous frame (or zeroes) for all channels
    int32_t **raw_samples;           ///< pointer to the beginning of the current frame's samples in the buffer for each channel
    int32_t *res_buffer;             ///< buffer containing all residual samples of the frame plus max_order samples from the previous frame (or zeroes) for all channels
    int32_t **res_samples;           ///< pointer to the beginning of the current frame's samples in the buffer for each channel
    ALSBlock *block_buffer;          ///< buffer containing all ALSBlocks for each channel
    ALSBlock **blocks;               ///< array of 32 ALSBlock pointers per channel pointing into the block_buffer
} ALSEncContext;


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


/** Subdivide the frame into smaller blocks
 */
static void block_partitioning(ALSEncContext *ctx)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int b, c;

    if (sconf->block_switching) {

        // maybe set a bs_info[c] in the context, but maybe
        // this should be generated when needed in bitstream assembly
        // because it should not be needed elsewhere in the encoder
        // if block[c][x].length is there

    } else {

        // generate result for no block-switching,
        // one block per channel as long as the frame

        for (c = 0; c < avctx->channels; c++) {
            ctx->blocks[c][0].length = ctx->cur_frame_length;

            for (b = 1; b < 32; b++)
                ctx->blocks[c][b].length = 0;
        }
    }
}


/** Write a given block of a given channel
 */
static void write_block(ALSEncContext *ctx, ALSBlock *block,
                        unsigned int c, unsigned int b)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    PutBitContext *pb        = &ctx->pb;
    unsigned int i;


    // block_type
    put_bits(pb, 1, block->constant);


    if (block->constant) {
        // const_block
        put_bits(pb, 1, block->constant_value > 0);


        // js_block
        put_bits(pb, 1, block->js_block);
        align_put_bits(pb);


        if (block->constant_value) {
            unsigned int const_val_bits = sconf->floating ? 24 : avctx->bits_per_raw_sample;
            put_bits(pb, const_val_bits, block->constant_value);
        }
    } else {
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
            for (i = 0; i < block->sub_blocks; i++) {
                // write rice param per sub_block
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


            // quant_cof
            // to be implemented
            //
            // while not implemented, no need to output anything

            // for each quant_cof, put(quant_cof) in rice code
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


        // smp_val[0] && res[1,2] in case of random_access block
        // to be implemented
        //
        // while not implemented, sconf->ra-distance == 0 and
        // no need to write anything
        // see decoder definition of ra_frame/ra_block:
        //  -> ra_frame = sconf->ra_distance && !(ctx->frame_id % sconf->ra_distance);


        // write residuals
        for (i = 0; i < block->length; i++) {
            // write all residuals of the block
        }
    }


    if (!sconf->mc_coding || ctx->js_switch)
        align_put_bits(pb);
}


/** Write the frame
 */
static void write_frame(ALSEncContext *ctx)
{
    AVCodecContext *avctx    = ctx->avctx;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int c, b;

    // ra_unit_size
    if (sconf->ra_flag == RA_FLAG_FRAMES) { // && this is a ra-frame
        // to be implemented
        // write ra_unit_size into the stream
    }


    // js_switch
    if (ctx->js_switch) {
        // to be implemented
        // not yet supported anyway
    }


    // write blocks
    if (!sconf->mc_coding || ctx->js_switch) {
        for (c = 0; c < avctx->channels; c++) {
            for (b= 0; b < 32; b++) {
                if (!ctx->blocks[c][b].length) // todo, see below
                    continue;

                if (ctx->independent_bs[c]) {
                    write_block(ctx, &ctx->blocks[c][b], c, b);
                } else {
                    // write channel & channel+1
                }
            }
        }
    } else {

        // MCC: to be implemented

    }
}


/** Encode a given block of a given channel
 */
static void find_block_params(ALSEncContext *ctx, ALSBlock *block,
                              unsigned int c, unsigned int b)
{
    ALSSpecificConfig *sconf = &ctx->sconf;


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
    // while not implemented: ensure max_order = 0 && adapt_order = 0 (in init)
    // LPC / PARCOR coefficients to be stored in context
    // they depend on js_block and opt_order which may be changing later on

    block->opt_order = sconf->max_order;

    if (sconf->adapt_order) {
        // to be implemented
        // search for coefficients and opt_order
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


    // generate residuals using parameters:
    // just verbatim mode (no prediction) supported right now

    if (block->opt_order) {

        // to be implemented

    } else {
        memcpy(ctx->res_samples[c] + b, ctx->raw_samples[c] + b,
               sizeof(*ctx->res_samples[c]) * block->length);
    }


    // determine how many sub-blocks to use(?)
    // to be implemented
    //
    // while not implemented, just don't use sub-blocks

    block->sub_blocks = 1;


    // search for rice parameter:
    // to be implemented
    //
    // use predefined rice parameter while not implemented

    block->rice_param = 2;
}


static int encode_frame(AVCodecContext *avctx, uint8_t *frame,
                        int buf_size, void *data)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int b, c;

    // last frame has been encoded, update extradata
    if (!data) {

        // to be implemented:
        // update CRC value
        // update number of samples (if being tracked)

        return 0;
    }


    ctx->cur_frame_length = avctx->frame_size;
    init_put_bits(&ctx->pb, frame, buf_size);


    // preprocessing
    deinterleave_raw_samples(ctx, data);
    select_difference_coding_mode(ctx);
    block_partitioning(ctx);


    // encoding loop
    if (!sconf->mc_coding || ctx->js_switch) {
        for (b= 0; b < 32; b++) {
            for (c = 0; c < avctx->channels; c++) {
                if (!ctx->blocks[c][b].length) // maybe store the max number of blocks somewhere not to do unnecessary loops
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
    write_frame(ctx);

    memset(frame, 0, buf_size);

    return (avctx->bits_per_raw_sample >> 3) *
           avctx->channels *
           ctx->cur_frame_length;
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


    // determine frame length
    frame_partitioning(ctx);


    // determine distance between ra-frames. 0 = no ra, 1 = all ra
    // default for now = 0. should be changed when implemented.
    // If we try to do it in the header, we would have to keep track
    // of all RA unit info during encoding, then free/realloc the
    // extradata at the end to make enough space for the RA info.
    sconf->ra_distance = 0;


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
    //
    // while stp is not implemented, set to 0
    sconf->max_order = 0;


    // determine if block-switching is used
    // depends on simple profile or not (user-defined?)
    // may be always enable this by default with the maximum level,
    // simple profile supports up to 3 stages
    // disable for the fastest compression mode
    // should be set when implemented
    sconf->block_switching = 0;


    // determine if BGMC mode is used
    // should be user-defineable
    sconf->bgmc = 0;


    // determine what sub-block partitioning is used
    sconf->sb_part = 0;


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
        av_freep(avctx->extradata);

    avctx->extradata = av_mallocz(header_size + FF_INPUT_BUFFER_PADDING_SIZE);
    if (!avctx->extradata)
        return AVERROR(ENOMEM);

    init_put_bits(&pb, avctx->extradata, header_size);


    // AudioSpecificConfig, reference to ISO/IEC 14496-3 section 1.6.2.1 & 1.6.3
    // TODO: create a function in mpeg4audio.c/h to write AudioSpecificConfig

    // object type (GetAudioObjectType(), 14496-3 1.6.2.1.1 Table 1.16)
    put_bits(&pb, 5, AOT_ESCAPE);
    put_bits(&pb, 6, AOT_ALS - 32);

    // samplingFrequencyIndex
    put_bits(&pb,  4, 0xF);
    put_bits(&pb, 24, avctx->sample_rate); // samplingFrequency

    // channelConfiguraiton
    put_bits(&pb, 4, 0); // 0 = defined in DecoderSpecificConfig


    // switch(AudioObjectType) -> case 36: ALSSpecificConfig

    // fillBits (align)
    align_put_bits(&pb);

    put_bits32(&pb,     MKBETAG('A', 'L', 'S', '\0'));
    put_bits32(&pb,     avctx->sample_rate);
    put_bits32(&pb,     sconf->samples);
    put_bits  (&pb, 16, avctx->channels - 1);
    put_bits  (&pb,  3, 1);                      // original file_type (0 = unknown, 1 = wav, ...)
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

    av_freep(&avctx->extradata);
    avctx->extradata_size = 0;
    av_freep(&avctx->coded_frame);

    return 0;
}


static av_cold int encode_init(AVCodecContext *avctx)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    unsigned int channel_size;
    int ret, c;

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


    channel_size = sconf->frame_length + sconf->max_order;


    // allocate buffers
    ctx->independent_bs    = av_malloc (sizeof(*ctx->independent_bs) * avctx->channels);
    ctx->raw_buffer        = av_mallocz(sizeof(*ctx->raw_buffer)     * avctx->channels * channel_size);
    ctx->raw_samples       = av_malloc (sizeof(*ctx->raw_samples)    * avctx->channels);
    ctx->res_buffer        = av_mallocz(sizeof(*ctx->raw_buffer)     * avctx->channels * channel_size);
    ctx->res_samples       = av_malloc (sizeof(*ctx->raw_samples)    * avctx->channels);
    ctx->block_buffer      = av_mallocz(sizeof(*ctx->block_buffer)   * avctx->channels * 32);
    ctx->blocks            = av_malloc (sizeof(*ctx->blocks)         * avctx->channels);

    // check buffers
    if (!ctx->independent_bs    ||
        !ctx->raw_buffer        || !ctx->raw_samples ||
        !ctx->res_buffer        || !ctx->res_samples ||
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


    // channel sorting
    if ((sconf->joint_stereo || sconf->mc_coding) && sconf->chan_sort)
        channel_sorting(ctx);


    avctx->coded_frame = avcodec_alloc_frame();
    avctx->coded_frame->key_frame = 1;


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
