/**
 * FLAC audio encoder
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

#ifndef AVCODEC_FLACENC_H
#define AVCODEC_FLACENC_H

enum {
    FLAC_CHMODE_LEFT_RIGHT=0,
    FLAC_CHMODE_LEFT_SIDE,
    FLAC_CHMODE_RIGHT_SIDE,
    FLAC_CHMODE_MID_SIDE,
    FLAC_CHMODE_NOT_STEREO
};

int ff_flac_estimate_stereo_mode(const int32_t *left_ch,
                                 const int32_t *right_ch,
                                 int n, int max_k, int mode_mask);

#endif /* AVCODEC_FLACENC_H */

