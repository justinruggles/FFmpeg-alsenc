/*
 * MMX optimized LPC DSP utils
 * Copyright (c) 2007 Loren Merritt
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


void ff_lpc_compute_autocorr_sse2(const double *data,
                                  int len, int lag, double *autoc)
{
    int j;

    for(j=0; j<lag; j+=2){
        x86_reg i = -len*sizeof(double);
        if(j == lag-2) {
            __asm__ volatile(
                "movsd    "MANGLE(ff_pd_1)", %%xmm0 \n\t"
                "movsd    "MANGLE(ff_pd_1)", %%xmm1 \n\t"
                "movsd    "MANGLE(ff_pd_1)", %%xmm2 \n\t"
                "1:                                 \n\t"
                "movapd   (%2,%0), %%xmm3           \n\t"
                "movupd -8(%3,%0), %%xmm4           \n\t"
                "movapd   (%3,%0), %%xmm5           \n\t"
                "mulpd     %%xmm3, %%xmm4           \n\t"
                "mulpd     %%xmm3, %%xmm5           \n\t"
                "mulpd -16(%3,%0), %%xmm3           \n\t"
                "addpd     %%xmm4, %%xmm1           \n\t"
                "addpd     %%xmm5, %%xmm0           \n\t"
                "addpd     %%xmm3, %%xmm2           \n\t"
                "add       $16,    %0               \n\t"
                "jl 1b                              \n\t"
                "movhlps   %%xmm0, %%xmm3           \n\t"
                "movhlps   %%xmm1, %%xmm4           \n\t"
                "movhlps   %%xmm2, %%xmm5           \n\t"
                "addsd     %%xmm3, %%xmm0           \n\t"
                "addsd     %%xmm4, %%xmm1           \n\t"
                "addsd     %%xmm5, %%xmm2           \n\t"
                "movsd     %%xmm0,   (%1)           \n\t"
                "movsd     %%xmm1,  8(%1)           \n\t"
                "movsd     %%xmm2, 16(%1)           \n\t"
                :"+&r"(i)
                :"r"(autoc+j), "r"(data+len), "r"(data+len-j)
                :"memory"
            );
        } else {
            __asm__ volatile(
                "movsd    "MANGLE(ff_pd_1)", %%xmm0 \n\t"
                "movsd    "MANGLE(ff_pd_1)", %%xmm1 \n\t"
                "1:                                 \n\t"
                "movapd   (%3,%0), %%xmm3           \n\t"
                "movupd -8(%4,%0), %%xmm4           \n\t"
                "mulpd     %%xmm3, %%xmm4           \n\t"
                "mulpd    (%4,%0), %%xmm3           \n\t"
                "addpd     %%xmm4, %%xmm1           \n\t"
                "addpd     %%xmm3, %%xmm0           \n\t"
                "add       $16,    %0               \n\t"
                "jl 1b                              \n\t"
                "movhlps   %%xmm0, %%xmm3           \n\t"
                "movhlps   %%xmm1, %%xmm4           \n\t"
                "addsd     %%xmm3, %%xmm0           \n\t"
                "addsd     %%xmm4, %%xmm1           \n\t"
                "movsd     %%xmm0, %1               \n\t"
                "movsd     %%xmm1, %2               \n\t"
                :"+&r"(i), "=m"(autoc[j]), "=m"(autoc[j+1])
                :"r"(data+len), "r"(data+len-j)
            );
        }
    }
}
