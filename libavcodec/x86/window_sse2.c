/*
 * SSE2-optimized audio window functions
 * Copyright (c) 2007 Loren Merritt
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

#include "libavutil/x86_cpu.h"
#include "dsputil_mmx.h"
#include "window.h"

DECLARE_ALIGNED(16, const double, ff_pd_0_1)[2] = { 0.0, 1.0 };

void ff_apply_welch_window_sse2(WindowContext *wctx, const int32_t *data,
                                double *w_data, int len)
{
    double c = 2.0 / (len-1.0);
    int n2 = len>>1;
    x86_reg i = -n2*sizeof(int32_t);
    x86_reg j =  n2*sizeof(int32_t);
    __asm__ volatile(
        "movsd   %0,     %%xmm7                \n\t"
        "movapd  "MANGLE(ff_pd_1)", %%xmm6     \n\t"
        "movapd  "MANGLE(ff_pd_2)", %%xmm5     \n\t"
        "movapd  "MANGLE(ff_pd_0_1)", %%xmm4   \n\t"
        "movlhps %%xmm7, %%xmm7                \n\t"
        ::"m"(c)
    );
#define WELCH(MOVPD, offset)\
    __asm__ volatile(\
        "1:                                    \n\t"\
        "movapd   %%xmm7,  %%xmm1              \n\t"\
        "mulpd    %%xmm4,  %%xmm1              \n\t"\
        "subpd    %%xmm6,  %%xmm1              \n\t"\
        "mulpd    %%xmm1,  %%xmm1              \n\t"\
        "movapd   %%xmm6,  %%xmm0              \n\t"\
        "subpd    %%xmm1,  %%xmm0              \n\t"\
        "pshufd   $0x4e,   %%xmm0, %%xmm1      \n\t"\
        "cvtpi2pd (%3,%0), %%xmm2              \n\t"\
        "cvtpi2pd "#offset"*4(%3,%1), %%xmm3   \n\t"\
        "mulpd    %%xmm0,  %%xmm2              \n\t"\
        "mulpd    %%xmm1,  %%xmm3              \n\t"\
        "movapd   %%xmm2, (%2,%0,2)            \n\t"\
        MOVPD"    %%xmm3, "#offset"*8(%2,%1,2) \n\t"\
        "addpd    %%xmm5,  %%xmm4              \n\t"\
        "sub      $8,      %1                  \n\t"\
        "add      $8,      %0                  \n\t"\
        "jl 1b                                 \n\t"\
        :"+&r"(i), "+&r"(j)\
        :"r"(w_data+n2), "r"(data+n2)\
    );
    if(len&1)
        WELCH("movupd", -1)
    else
        WELCH("movapd", -2)
#undef WELCH
}

void ff_apply_fixed_window_sse2(WindowContext *wctx, const int32_t *input,
                                  double *output)
{
    int length = wctx->length;
    int rem    = length & 1;
    int len2   = length - rem;
    x86_reg i  = (len2 - 2) * sizeof(int32_t);

    if (rem)
        output[length-1] = input[length-1] * wctx->precalc_window[length-1];

#define FIXED_WINDOW(MOVPD)\
    __asm__ volatile(\
        "1:                                 \n\t"\
        "cvtpi2pd   (%1,%0),    %%xmm0      \n\t"\
        "mulpd      (%2,%0,2),  %%xmm0      \n\t"\
        MOVPD"      %%xmm0,     (%3,%0,2)   \n\t"\
        "sub        $8,         %0          \n\t"\
        "jge 1b                             \n\t"\
        :"+&r"(i)\
        :"r"(input), "r"(wctx->precalc_window), "r"(output)\
    );
    if((x86_reg)output & 15)
        FIXED_WINDOW("movupd")
    else
        FIXED_WINDOW("movapd")
#undef FIXED_WINDOW
}
