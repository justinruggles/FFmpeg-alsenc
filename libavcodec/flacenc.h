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

#include <stdint.h>

#include "flac.h"

typedef struct FlacRiceContext {
    int porder;
    int params[256];
} FlacRiceContext;

uint32_t ff_flac_calc_rice_params(FlacRiceContext *rc, int pmin, int pmax,
                                  int32_t *data, int n, int pred_order);

#endif /* AVCODEC_FLACENC_H */

