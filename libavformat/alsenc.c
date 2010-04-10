/*
 * raw ALS muxer
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

#include "libavcodec/mpeg4audio.h"
#include "avformat.h"

typedef struct AlsEncContext {
    int header_size;
} AlsEncContext;

static int als_write_header(struct AVFormatContext *s)
{
    MPEG4AudioConfig m4ac;
    AVCodecContext *avctx = s->streams[0]->codec;
    AlsEncContext *ctx = s->priv_data;
    int config_offset;

    /* get offset of ALSSpecificConfig in extradata */
    config_offset = ff_mpeg4audio_get_config(&m4ac, avctx->extradata,
                                             avctx->extradata_size);

    config_offset = (config_offset + 7) >> 3;
    ctx->header_size = avctx->extradata_size - config_offset;

    /* write STREAMINFO or full header */
    put_buffer(s->pb, &avctx->extradata[config_offset], ctx->header_size);

    return 0;
}

static int als_write_trailer(struct AVFormatContext *s)
{
    ByteIOContext *pb = s->pb;
    AVCodecContext *avctx = s->streams[0]->codec;
    AlsEncContext *ctx = s->priv_data;
    int header_size;

    header_size = ctx->header_size;

    if (!url_is_streamed(pb)) {
        /* rewrite the header */
        int64_t file_size   = url_ftell(pb);
        int     header_size = ctx->header_size;

        url_fseek(pb, 0, SEEK_SET);
        if (als_write_header(s))
            return -1;

        if (header_size != ctx->header_size) {
            av_log(s, AV_LOG_WARNING, "ALS header size mismatch. Unable to rewrite header.\n");
        }
        url_fseek(pb, file_size, SEEK_SET);
        put_flush_packet(pb);
    } else {
        av_log(s, AV_LOG_WARNING, "unable to rewrite ALS header.\n");
    }

    return 0;
}

static int als_write_packet(struct AVFormatContext *s, AVPacket *pkt)
{
    put_buffer(s->pb, pkt->data, pkt->size);
    put_flush_packet(s->pb);
    return 0;
}

AVOutputFormat als_muxer = {
    "als",
    NULL_IF_CONFIG_SMALL("raw MPEG-4 Audio Lossless Coding (ALS)"),
    NULL,
    "als",
    sizeof(AlsEncContext),
    CODEC_ID_MP4ALS,
    CODEC_ID_NONE,
    als_write_header,
    als_write_packet,
    als_write_trailer,
    .flags= AVFMT_NOTIMESTAMPS,
};
