/**
 * x86 audio window functions
 * Copyright (c) 2010  Justin Ruggles <justin.ruggles@gmail.com>
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

#include "libavcodec/dsputil.h"
#include "window.h"

void ff_window_init_mmx(WindowContext *wctx)
{
    int has_vectors = mm_support();
    if (has_vectors & FF_MM_SSE2 && HAVE_SSE) {
        switch (wctx->type) {
        case WINDOW_TYPE_WELCH:
            wctx->apply_window_var = ff_apply_welch_window_sse2;
            if (wctx->length > 0)
                wctx->apply_window_fixed = ff_apply_fixed_window_sse2;
            break;
        default:
            if (wctx->length > 0)
                wctx->apply_window_fixed = ff_apply_fixed_window_sse2;
        }
    }
}
