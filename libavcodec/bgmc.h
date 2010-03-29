/*
 * Block Gilbert-Moore decoder
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
 * @file libavcodec/bgmc.h
 * Block Gilbert-Moore decoder header
 * @author Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
 */


#ifndef AVCODEC_BGMC_H
#define AVCODEC_BGMC_H


#include "avcodec.h"
#include "get_bits.h"
#include "put_bits.h"

extern int ff_bgmc_max[16];
extern const uint8_t ff_bgmc_tail_code[16][6];


/* arithmetic decoding of MSB and LSB parts */
int ff_bgmc_init(AVCodecContext *avctx, uint8_t **cf_lut, unsigned int **cf_lut_status);


void ff_bgmc_end(uint8_t **cf_lut, unsigned int **cf_lut_status);


void ff_bgmc_decode_init(GetBitContext *gb,
                      unsigned int *h, unsigned int *l, unsigned int *v);


void ff_bgmc_decode_end(GetBitContext *gb);


void ff_bgmc_decode(GetBitContext *gb, unsigned int num, int32_t *dst,
                 unsigned int delta, unsigned int sx,
                 unsigned int *h, unsigned int *l, unsigned int *v,
                 uint8_t *cf_lut, unsigned int *cf_lut_status);


/* arithmetic encoding of MSB parts */
void ff_bgmc_encode_init(unsigned int *h, unsigned int *l, unsigned int *f);


void ff_bgmc_encode_msb(PutBitContext *pb, int32_t *symbols, unsigned int n,
                        unsigned int k, unsigned int delta, unsigned int max,
                        unsigned int s, unsigned int sx,
                        unsigned int *h, unsigned int *l, unsigned int *f);


void ff_bgmc_encode(PutBitContext *pb, int32_t symbol,
                    unsigned int delta, unsigned int sx,
                    unsigned int *h, unsigned int *l, unsigned int *f);


void ff_bgmc_encode_end(PutBitContext *pb, unsigned int *l, unsigned int *f);


/* bit count of arithmetic encoded MSB parts */
void ff_bgmc_encode_msb_count(unsigned int *bits, const int32_t *symbols, unsigned int n,
                              unsigned int k, unsigned int delta, unsigned int max,
                              unsigned int s, unsigned int sx,
                              unsigned int *h, unsigned int *l, unsigned int *f);


void ff_bgmc_encode_count(unsigned int *bits, const int32_t symbol,
                          unsigned int delta, unsigned int sx,
                          unsigned int *h, unsigned int *l, unsigned int *f);


void ff_bgmc_encode_end_count(unsigned int *bits, unsigned int *f);

#endif /* AVCODEC_BGMC_H */
