/*
 * MPEG-4 ALS encoder
 * Copyright (c) 2010 Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
 * Copyright (c) 2010 Justin Ruggles <justin.ruggles _at_ gmail.com>
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
 * @author Justin Ruggles <justin.ruggles _at_ gmail.com>
 */


#define DEBUG


#include "als.h"

#include "avcodec.h"
#include "put_bits.h"
#include "mpeg4audio.h"


#define ALS_SPECIFIC_CFG_SIZE  30
#define ALS_EXTRADATA_MAX_SIZE (6 + ALS_SPECIFIC_CFG_SIZE)


typedef struct {
    AVCodecContext *avctx;
    ALSSpecificConfig sconf;
    PutBitContext pb;
    unsigned int cur_frame_length; ///< length of the current frame
    int32_t *raw_buffer;           ///< buffer containing all raw samples of the frame plut max_order samples from the previous frame (or zeroes) for all channels
    int32_t **raw_samples;         ///< pointer to the beginning of this frames samples in the buffer for each channel
} ALSEncContext;


/** Converts an array of channel-interleaved samples into
 *  multiple arrays of samples per sample
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


static int encode_frame(AVCodecContext *avctx, uint8_t *frame,
                        int buf_size, void *data)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;

    deinterleave_raw_samples(ctx, data);

    memset(frame, 0, buf_size);

    return (avctx->bits_per_raw_sample >> 3) *
           avctx->channels *
           sconf->frame_length; // to be replaced by #samples actually written
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
    if (avctx->frame_size <= 0)
        avctx->frame_size = 2048;

    // ensure a certain boundary for the frame size
    // maximum value is 0xFFFF using 16 bit in ALSSpecificConf
    av_clip(avctx->frame_size, 1024, 0xFFFF);

    // enforce a frame length that is a power of 2?
    // or discard block-switching if it is not?
    // if block-switching is enabled by default
    // then do check

    sconf->frame_length = avctx->frame_size;
}


static av_cold int get_specific_config(AVCodecContext *avctx)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;


    // total number of samples unknown
    sconf->samples = 0xFFFFFFFF;


    // determine sample format
    if        (avctx->sample_fmt == SAMPLE_FMT_U8) {
        sconf->resolution = 0;
    } else if (avctx->sample_fmt == SAMPLE_FMT_S16) {
        sconf->resolution = 1;
    } else if (avctx->sample_fmt == SAMPLE_FMT_S32) {
        sconf->resolution = 3;
    } else if (avctx->sample_fmt == SAMPLE_FMT_FLT) {
        sconf->resolution = 3;
        sconf->floating   = 1;
        av_log(avctx, AV_LOG_ERROR, "floating-point samples are not supported\n");
    } else {
        av_log(avctx, AV_LOG_ERROR, "unsupported sample format\n");
        return -1;
    }

    avctx->bits_per_raw_sample = (sconf->resolution + 1) << 3;


    // determine frame length
    frame_partitioning(ctx);


    // determine distance between ra-frames. 0 = no ra, 1 = all ra
    // default for now = 0. should be changed when implemented.
    // should also be user-defineable
    sconf->ra_distance = 0;


    // determine where to store ra_flag (01: beginning of frame_data)
    // default for now = RA_FLAG_FRAMES. Would make decoding more robust
    // in case of seeking although not implemented in FFmpeg decoder yet
    sconf->ra_flag = RA_FLAG_FRAMES;


    // determine if adaptive prediction order is used
    sconf->adapt_order = 0;


    // determine the coef_table to be used
    // no samples: not yet possible?
    // so FFmpeg has to choose one and live with it(?)
    sconf->coef_table = 0;


    // determine if long-term prediction is used
    // should also be user-defineable
    sconf->long_term_prediction = 0;


    // determine a maximum prediction order
    // no samples: not yet possible to determine..
    // do evaluation to find a suitable standard value?
    sconf->max_order = 10;


    // determine if block-switching is used
    // should be user-defineable
    // may be always enable this by default,
    // even simple profile supports up to 3 stages
    // not user-defined -> 3
    // user-defined otherwise
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
    sconf->joint_stereo = 0;


    // determine if multi-channel coding is used
    // should be user-defineable
    // may be sanity check: channels > 2
    // (although specs allow mc_coding for 2 channels...
    // maybe give a warning)
    sconf->mc_coding = 0;


    // determine manual channel configuration
    // should be user-defineable
    sconf->chan_config = 0;
    sconf->chan_config_info = 0;


    // determine channel sorting (user defined)
    sconf->chan_sort = 0;
    sconf->chan_pos  = NULL;


    // determine if backward adaptive is used
    // user defined by explicit option and/or compression level
    sconf->rlslms = 0;


    // determine size of original header/trailer
    // (possible in ffmpeg? at all useful?)
    sconf->header_size  = 0;
    sconf->trailer_size = 0;


    return 0;
}


static int write_specific_config(AVCodecContext *avctx)
{
    ALSEncContext *ctx       = avctx->priv_data;
    ALSSpecificConfig *sconf = &ctx->sconf;
    PutBitContext pb;

    uint8_t header[ALS_EXTRADATA_MAX_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];

    memset(header, 0, sizeof(header));
    init_put_bits(&pb, header, ALS_EXTRADATA_MAX_SIZE);


    // AudioSpecificConfig, reference to ISO/IEC 14496-3 section 1.6.2.1 & 1.6.3

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
    put_bits  (&pb,  3, 0);                      // original file_type (0 = unknown)
    put_bits  (&pb,  3, sconf->resolution);
    put_bits  (&pb,  1, sconf->floating);
    put_bits  (&pb,  1, 0);                      // msb first
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

    put_bits32(&pb,     sconf->header_size);
    put_bits32(&pb,     sconf->trailer_size);


    // writing in local header finished, now copy that to extradata
    if (avctx->extradata)
        av_freep(avctx->extradata);

    avctx->extradata_size = put_bits_count(&pb) >> 3;

    avctx->extradata = av_mallocz(avctx->extradata_size +
                                  FF_INPUT_BUFFER_PADDING_SIZE);
    if (!avctx->extradata)
        return AVERROR(ENOMEM);

    memcpy(avctx->extradata, header, avctx->extradata_size);

    return 0;
}


static av_cold int encode_end(AVCodecContext *avctx)
{
    ALSEncContext *ctx = avctx->priv_data;

    av_freep(&ctx->raw_buffer);
    av_freep(&ctx->raw_samples);

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


    // write AudioSpecificConfig & ALSSpecificConfig
    ret = write_specific_config(avctx);
    if (ret)
        return ret;


    channel_size = sconf->frame_length + sconf->max_order;


    // allocate buffers
    ctx->raw_buffer  = av_mallocz(sizeof(*ctx->raw_buffer)  * avctx->channels * channel_size);
    ctx->raw_samples = av_malloc (sizeof(*ctx->raw_samples) * avctx->channels);


    // check buffers
    if (!ctx->raw_buffer|| !ctx->raw_samples) {
        av_log(avctx, AV_LOG_ERROR, "Allocating buffer memory failed.\n");
        encode_end(avctx);
        return AVERROR(ENOMEM);
    }


    // assign raw samples buffers
    ctx->raw_samples[0] = ctx->raw_buffer + sconf->max_order;
    for (c = 1; c < avctx->channels; c++)
        ctx->raw_samples[c] = ctx->raw_samples[c - 1] + channel_size;


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
    .capabilities = CODEC_CAP_SMALL_LAST_FRAME,         // CAP_DELAY?
    .long_name = NULL_IF_CONFIG_SMALL("MPEG-4 Audio Lossless Coding (ALS)"),
};
