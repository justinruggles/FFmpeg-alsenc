/**
 * Audio window functions
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

#ifndef AVCODEC_WINDOW_H
#define AVCODEC_WINDOW_H

#include "config.h"
#include <stdint.h>

enum WindowType {
    WINDOW_TYPE_NONE,
    WINDOW_TYPE_RECTANGLE,
    WINDOW_TYPE_WELCH,
    WINDOW_TYPE_SINERECT,
    WINDOW_TYPE_HANNRECT,
};

typedef struct WindowContext {
    enum WindowType type;       ///< Window type
    int length;                 ///< Window length, or -1 for variable length only
    double *precalc_window;     ///< Pre-calculated window, if applicable
    double param;               ///< input parameter for the window calculation

    void (*apply_window_fixed)(struct WindowContext *wctx, const int32_t *input,
                               double *output);
    void (*apply_window_var)(struct WindowContext *wctx, const int32_t *input,
                             double *output, int length);
} WindowContext;

/**
 * Initialize window
 * @param wctx window context
 * @param length window length, or -1 for variable length only
 * @param param optional input parameter to window calculation
 * @return 0 on success, less than 0 on error
 */
int ff_window_init(WindowContext *wctx, enum WindowType type, int length,
                   double param);

#if HAVE_MMX
void ff_window_init_mmx(WindowContext *wctx);
#endif

/**
 * Apply window
 * @param wctx window context
 * @param length window length
 * @return 0 on success, less than 0 on error
 */
int ff_window_apply(WindowContext *wctx, const int32_t *input, double *output,
                    int length);

/**
 * Close window and free memory if needed
 * @param wctx window context
 */
void ff_window_close(WindowContext *wctx);


#endif /* AVCODEC_WINDOW_H */
