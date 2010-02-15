/*
 * MPEG-4 ALS shared functions
 * Copyright (c) 2010 Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
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
 * @file libavcodec/als.c
 * MPEG-4 ALS functions shared between ALS decoder and ALS encoder
 * @author Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
 */

#include "als.h"
#include "mathops.h"

void ff_als_parcor_to_lpc(unsigned int k, const int32_t *par, int32_t *cof)
{
    int i, j;

    for (i = 0, j = k - 1; i < j; i++, j--) {
        int tmp1 = ((MUL64(par[k], cof[j]) + (1 << 19)) >> 20);
        cof[j]  += ((MUL64(par[k], cof[i]) + (1 << 19)) >> 20);
        cof[i]  += tmp1;
    }
    if (i == j)
        cof[i] += ((MUL64(par[k], cof[j]) + (1 << 19)) >> 20);

    cof[k] = par[k];
}
