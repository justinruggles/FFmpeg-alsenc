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

#include "window.h"
#include "dsputil.h"


static void apply_fixed_window_c(WindowContext *wctx, const int32_t *input,
                                 double *output)
{
    int i;
    for (i = 0; i < wctx->length; i++)
        output[i] = input[i] * wctx->precalc_window[i];
}


#define APPLY_RECTANGLE_WINDOW(in)  \
{                                   \
    int i;                          \
    for (i = 0; i < length; i++)    \
        output[i] = in;             \
}

static void apply_rectangle_window_c(WindowContext *wctx, const int32_t *input,
                                     double *output, int length)
{
    if (input) {
        APPLY_RECTANGLE_WINDOW((double)input[i])
    } else {
        APPLY_RECTANGLE_WINDOW(1.0)
    }
}


#define APPLY_WELCH_WINDOW(in0, in1, in2)   \
{                                           \
    int i, n2, offset;                      \
    double w;                               \
    double c;                               \
                                            \
    n2 = (length >> 1);                     \
    c = 2.0 / (length - 1.0);               \
                                            \
    output+=n2;                             \
     input+=n2;                             \
    offset = 1;                             \
    if (length & 1) {                       \
        output[0] = in0;                    \
        input++;                            \
        output++;                           \
        offset++;                           \
    }                                       \
    for (i = 0; i < n2; i++) {              \
        w = c * (n2 + i) - 1.0;             \
        w = 1.0 - (w * w);                  \
        output[-i-offset] = (in1) * w;      \
        output[+i       ] = (in2) * w;      \
    }                                       \
}

static void apply_welch_window_c(WindowContext *wctx, const int32_t *input,
                                 double *output, int length)
{
    if (input) {
        APPLY_WELCH_WINDOW(input[0], input[-i-offset], input[i])
    } else {
        APPLY_WELCH_WINDOW(1.0, 1.0, 1.0)
    }
}


#define APPLY_SINERECT_WINDOW(in1, in2)                     \
{                                                           \
    int i;                                                  \
    int side_len = lrint(length / (2 * wctx->param));       \
    double phi   = wctx->param * M_PI / (length - 1);       \
                                                            \
    apply_rectangle_window_c(wctx, input, output, length);  \
                                                            \
    if (side_len < 4)                                       \
        return;                                             \
                                                            \
    for (i = 0; i < side_len; i++) {                        \
        double w = sin(phi * i);                            \
        output[i]          = (in1) * w;                     \
        output[length-i-1] = (in2) * w;                     \
    }                                                       \
}

static void apply_sinerect_window_c(WindowContext *wctx, const int32_t *input,
                                    double *output, int length)
{
    if (input) {
        APPLY_SINERECT_WINDOW(input[i], input[length-i-1]);
    } else {
        APPLY_SINERECT_WINDOW(1.0, 1.0);
    }
}


#define APPLY_HANNRECT_WINDOW(in1, in2)                     \
{                                                           \
    int i;                                                  \
    int side_len = lrint(length / (2 * wctx->param));       \
    double phi   = wctx->param * 2.0 * M_PI / (length - 1); \
                                                            \
    apply_rectangle_window_c(wctx, input, output, length);  \
                                                            \
    if (side_len < 4)                                       \
        return;                                             \
                                                            \
    for (i = 0; i < side_len; i++) {                        \
        double w = 0.5 - 0.5 * cos(phi * i);                \
        output[i]          = (in1) * w;                     \
        output[length-i-1] = (in2) * w;                     \
    }                                                       \
}

static void apply_hannrect_window_c(WindowContext *wctx, const int32_t *input,
                                    double *output, int length)
{
    if (input) {
        APPLY_HANNRECT_WINDOW(input[i], input[length-i-1]);
    } else {
        APPLY_HANNRECT_WINDOW(1.0, 1.0);
    }
}


static int rectangle_init(WindowContext *wctx)
{
    wctx->apply_window_var   = apply_rectangle_window_c;
    wctx->apply_window_fixed = NULL;
    return 0;
}


#define WINDOW_INIT(name)                                                   \
    wctx->apply_window_var   = apply_##name##_window_c;                     \
    wctx->apply_window_fixed = NULL;                                        \
    if (wctx->length > 0) {                                                 \
        wctx->precalc_window = av_malloc(wctx->length * sizeof(double));    \
        if (!wctx->precalc_window)                                          \
            return AVERROR(ENOMEM);                                         \
                                                                            \
        apply_##name##_window_c(wctx, NULL, wctx->precalc_window,           \
                                wctx->length);                              \
        wctx->apply_window_fixed = apply_fixed_window_c;                    \
    }                                                                       \

int ff_window_init(WindowContext *wctx, enum WindowType type, int length,
                   double param)
{
    if (!length || length < -1)
        return AVERROR(EINVAL);

    wctx->type   = type;
    wctx->length = length;
    wctx->param  = param;

    switch (type) {
    case WINDOW_TYPE_RECTANGLE:
        rectangle_init(wctx);
        break;
    case WINDOW_TYPE_WELCH:
        WINDOW_INIT(welch)
        break;
    case WINDOW_TYPE_SINERECT:
        WINDOW_INIT(sinerect)
        break;
    case WINDOW_TYPE_HANNRECT:
        WINDOW_INIT(hannrect)
        break;
    default:
        return AVERROR(EINVAL);
    }

    if (HAVE_MMX)
        ff_window_init_mmx(wctx);

    return 0;
}


int ff_window_apply(WindowContext *wctx, const int32_t *input, double *output,
                    int length)
{
    if (length == wctx->length && wctx->apply_window_fixed)
        wctx->apply_window_fixed(wctx, input, output);
    else if (wctx->apply_window_var) {
        wctx->apply_window_var(wctx, input, output, length);
    } else
        return AVERROR(EINVAL);

    return 0;
}


void ff_window_close(WindowContext *wctx)
{
    wctx->type               = WINDOW_TYPE_NONE;
    wctx->length             = 0;
    wctx->param              = 0.0;
    wctx->apply_window_var   = NULL;
    wctx->apply_window_fixed = NULL;
    av_freep(&wctx->precalc_window);
}
