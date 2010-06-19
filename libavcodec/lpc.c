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

#include "libavutil/lls.h"
#include "dsputil.h"

#define LPC_USE_DOUBLE
#include "lpc.h"


/**
 * Apply Welch window function to audio block
 */
static void apply_welch_window(const int32_t *data, int len, double *w_data)
{
    int i, n2, offset;
    double w;
    double c;

    n2 = (len >> 1);
    c = 2.0 / (len - 1.0);

    w_data+=n2;
      data+=n2;
    offset = 1;
    if (len & 1) {
        w_data[0] = data[0];
        w_data++;
        data++;
        offset++;
    }
    for(i=0; i<n2; i++) {
        w = c * (n2 + i) - 1.0;
        w = 1.0 - (w * w);
        w_data[-i-offset] = data[-i-offset] * w;
        w_data[+i       ] = data[+i       ] * w;
    }
}

void ff_lpc_compute_autocorr(const int32_t *data, const double *window,
                             int len, int lag, double *autoc)
{
    int i, j;
    double tmp[len + lag + 1];
    double *data1= tmp + lag;

    if (window) {
        for (i = 0; i < len; i++)
            data1[i] = data[i] * window[i];
    } else {
        apply_welch_window(data, len, data1);
    }

    for(j=0; j<lag; j++)
        data1[j-lag]= 0.0;
    data1[len] = 0.0;

    for(j=0; j<lag; j+=2){
        double sum0 = 1.0, sum1 = 1.0;
        for(i=j; i<len; i++){
            sum0 += data1[i] * data1[i-j];
            sum1 += data1[i] * data1[i-j-1];
        }
        autoc[j  ] = sum0;
        autoc[j+1] = sum1;
    }

    if(j==lag){
        double sum = 1.0;
        for(i=j-1; i<len; i+=2){
            sum += data1[i  ] * data1[i-j  ]
                 + data1[i+1] * data1[i-j+1];
        }
        autoc[j] = sum;
    }
}

/**
 * Quantize LPC coefficients
 */
static void quantize_lpc_coefs(double *lpc_in, int order, int precision,
                               int32_t *lpc_out, int *shift, int max_shift, int zero_shift)
{
    int i;
    double cmax, error;
    int32_t qmax;
    int sh;

    /* define maximum levels */
    qmax = (1 << (precision - 1)) - 1;

    /* find maximum coefficient value */
    cmax = 0.0;
    for(i=0; i<order; i++) {
        cmax= FFMAX(cmax, fabs(lpc_in[i]));
    }

    /* if maximum value quantizes to zero, return all zeros */
    if(cmax * (1 << max_shift) < 1.0) {
        *shift = zero_shift;
        memset(lpc_out, 0, sizeof(int32_t) * order);
        return;
    }

    /* calculate level shift which scales max coeff to available bits */
    sh = max_shift;
    while((cmax * (1 << sh) > qmax) && (sh > 0)) {
        sh--;
    }

    /* since negative shift values are unsupported in decoder, scale down
       coefficients instead */
    if(sh == 0 && cmax > qmax) {
        double scale = ((double)qmax) / cmax;
        for(i=0; i<order; i++) {
            lpc_in[i] *= scale;
        }
    }

    /* output quantized coefficients and level shift */
    error=0;
    for(i=0; i<order; i++) {
        error -= lpc_in[i] * (1 << sh);
        lpc_out[i] = av_clip(lrintf(error), -qmax, qmax);
        error -= lpc_out[i];
    }
    *shift = sh;
}

void ff_lpc_calc_coefs_cholesky(const int32_t *samples, int blocksize,
                                int max_order, int passes, double *ref_out,
                                double *lpc, int lpc_stride)
{
    LLSModel m[2];
    double var[MAX_LPC_ORDER+1], av_uninit(weight);
    int i, j, pass;

    assert(lpc_passes > 0);
    for(pass=0; pass<passes; pass++){
        av_init_lls(&m[pass&1], max_order);

        weight=0;
        for(i=max_order; i<blocksize; i++){
            for(j=0; j<=max_order; j++)
                var[j]= samples[i-j];

            if(pass){
                double eval, inv, rinv;
                eval= av_evaluate_lls(&m[(pass-1)&1], var+1, max_order-1);
                eval= (512>>pass) + fabs(eval - var[0]);
                inv = 1/eval;
                rinv = sqrt(inv);
                for(j=0; j<=max_order; j++)
                    var[j] *= rinv;
                weight += inv;
            }else
                weight++;

            av_update_lls(&m[pass&1], var, 1.0);
        }
        av_solve_lls(&m[pass&1], 0.001, 0);
    }

    for(i=0; i<max_order; i++){
        if (lpc) {
            for(j=0; j<max_order; j++)
                lpc[j]=-m[(pass-1)&1].coeff[i][j];
            lpc += lpc_stride;
        }
        if (ref_out)
            ref_out[i]= sqrt(m[(pass-1)&1].variance[i] / weight) * (blocksize - max_order) / 4000;
    }
    if (ref_out) {
        for(i=max_order-1; i>0; i--)
            ref_out[i] = ref_out[i-1] - ref_out[i];
    }
}

static int estimate_best_order(double *ref, int min_order, int max_order)
{
    int i, est;

    est = min_order;
    for(i=max_order-1; i>=min_order-1; i--) {
        if(FFABS(ref[i]) > 0.10) {
            est = i+1;
            break;
        }
    }
    return est;
}

/**
 * Calculate LPC coefficients for multiple orders
 *
 * @param lpc_type LPC method for determining coefficients
 * 0  = None
 * 1  = LPC with fixed pre-defined coeffs
 * 2  = LPC with coeffs determined by Levinson-Durbin recursion
 * 3  = LPC with coeffs determined by Cholesky factorization
 * @param lpc_passes Number of passes to use for Cholesky factorization
 */
int ff_lpc_calc_coefs(DSPContext *s,
                      const int32_t *samples, int blocksize, int min_order,
                      int max_order, int precision,
                      int32_t coefs[][MAX_LPC_ORDER], int *shift, int lpc_type,
                      int lpc_passes, int omethod, int max_shift,
                      int zero_shift)
{
    double autoc[MAX_LPC_ORDER+1];
    double ref[MAX_LPC_ORDER];
    double lpc[MAX_LPC_ORDER][MAX_LPC_ORDER];
    int i;
    int opt_order;

    assert(max_order >= MIN_LPC_ORDER && max_order <= MAX_LPC_ORDER &&
           lpc_type > FF_LPC_TYPE_FIXED && lpc_type < FF_LPC_TYPE_CHOLESKY);

    if (lpc_type == FF_LPC_TYPE_LEVINSON) {
        s->lpc_compute_autocorr(samples, NULL, blocksize, max_order, autoc);

        compute_lpc_coefs(autoc, max_order, ref, &lpc[0][0], MAX_LPC_ORDER, 0, 1, NULL);
    } else if (lpc_type == FF_LPC_TYPE_CHOLESKY) {
        ff_lpc_calc_coefs_cholesky(samples, blocksize, max_order, lpc_passes,
                                   omethod == ORDER_METHOD_EST ? ref : NULL,
                                   &lpc[0][0], MAX_LPC_ORDER);
    }
    opt_order = max_order;

    if(omethod == ORDER_METHOD_EST) {
        opt_order = estimate_best_order(ref, min_order, max_order);
        i = opt_order-1;
        quantize_lpc_coefs(lpc[i], i+1, precision, coefs[i], &shift[i], max_shift, zero_shift);
    } else {
        for(i=min_order-1; i<max_order; i++) {
            quantize_lpc_coefs(lpc[i], i+1, precision, coefs[i], &shift[i], max_shift, zero_shift);
        }
    }

    return opt_order;
}
