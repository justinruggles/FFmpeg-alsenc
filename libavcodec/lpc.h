/**
 * LPC utility code
 * Copyright (c) 2006  Justin Ruggles <justin.ruggles@gmail.com>
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

#ifndef AVCODEC_LPC_H
#define AVCODEC_LPC_H

#include <stdint.h>
#include "dsputil.h"

#define ORDER_METHOD_EST     0
#define ORDER_METHOD_2LEVEL  1
#define ORDER_METHOD_4LEVEL  2
#define ORDER_METHOD_8LEVEL  3
#define ORDER_METHOD_SEARCH  4
#define ORDER_METHOD_LOG     5

#define LPC_TYPE_DEFAULT   -1
#define LPC_TYPE_NONE       0
#define LPC_TYPE_FIXED      1
#define LPC_TYPE_LEVINSON   2
#define LPC_TYPE_CHOLESKY   3

#define MIN_LPC_ORDER        1
#define MAX_LPC_ORDER       32


/**
 * Calculate LPC coefficients for multiple orders
 */
int ff_lpc_calc_coefs(DSPContext *s,
                      const int32_t *samples, int blocksize, int min_order,
                      int max_order, int precision,
                      int32_t coefs[][MAX_LPC_ORDER], int *shift, int lpc_type,
                      int lpc_passes, int omethod, int max_shift,
                      int zero_shift);

#ifdef LPC_USE_DOUBLE
#define LPC_TYPE double
#else
#define LPC_TYPE float
#endif

/**
 * Schur recursion.
 * Produces reflection coefficients from autocorrelation data.
 */
static inline LPC_TYPE compute_ref_coefs(const LPC_TYPE *autoc, int max_order,
                                         LPC_TYPE *ref)
{
    int i, j;
    LPC_TYPE err;
    LPC_TYPE *gen0, *gen1;

    gen0 = av_malloc(sizeof(LPC_TYPE) * 2 * max_order);
    gen1 = gen0 + max_order;
    for (i = 0; i < max_order; i++)
        gen0[i] = gen1[i] = autoc[i+1];

    err = autoc[0];
    ref[0] = -gen1[0] / err;
    err += gen1[0] * ref[0];
    for (i = 1; i < max_order; i++) {
        for (j = 0; j < max_order-i; j++) {
            gen1[j] = gen1[j+1] + ref[i-1] * gen0[j];
            gen0[j] = gen1[j+1] * ref[i-1] + gen0[j];
        }
        ref[i] = -gen1[0] / err;
        err += gen1[0] * ref[i];
    }

    av_freep(&gen0);
    return err;
}

/**
 * Levinson-Durbin recursion.
 * Produces LPC coefficients from autocorrelation data or reflection coefficients.
 */
static inline int compute_lpc_coefs(const LPC_TYPE *autoc, const LPC_TYPE *ref,
                                    int max_order,
                                    LPC_TYPE *lpc, int lpc_stride, int fail,
                                    int normalize)
{
    int i, j;
    LPC_TYPE err = 0;
    LPC_TYPE *lpc_last = lpc;

    assert(autoc || ref);

    if (autoc && normalize)
        err = *autoc++;

    if (fail && autoc && (autoc[max_order - 1] == 0 || err <= 0))
        return -1;

    for(i=0; i<max_order; i++) {
        LPC_TYPE r;
        if (autoc) {
            r = -autoc[i];
        if (normalize) {
            for(j=0; j<i; j++)
                r -= lpc_last[j] * autoc[i-j-1];

            r /= err;
            err *= 1.0 - (r * r);
        }
        } else {
            r = ref[i];
        }

        lpc[i] = r;

        for(j=0; j < (i+1)>>1; j++) {
            LPC_TYPE f = lpc_last[    j];
            LPC_TYPE b = lpc_last[i-1-j];
            lpc[    j] = f + r * b;
            lpc[i-1-j] = b + r * f;
        }

        if (fail && err < 0)
            return -1;

        lpc_last = lpc;
        lpc += lpc_stride;
    }

    return 0;
}

#endif /* AVCODEC_LPC_H */
