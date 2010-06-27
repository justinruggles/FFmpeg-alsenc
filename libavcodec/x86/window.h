/**
 * x86 audio window functions
 * Copyright (c) 2010  Justin Ruggles <justin.ruggles@gmail.com>
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

#ifndef AVCODEC_X86_WINDOW_H
#define AVCODEC_X86_WINDOW_H

#include "libavcodec/window.h"

void ff_apply_welch_window_sse2(WindowContext *wctx, const int32_t *data,
                                double *w_data, int len);

void ff_apply_fixed_window_sse2(WindowContext *wctx, const int32_t *data,
                                double *w_data);

#endif /* AVCODEC_X86_WINDOW_H */
