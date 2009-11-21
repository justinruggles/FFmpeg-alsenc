/**
 * MPEG-4 ALS data
 * Copyright (c) 2008 Justin Ruggles
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

#ifndef AVCODEC_ALSDATA_H
#define AVCODEC_ALSDATA_H

#include <stdint.h>

/**
 * Table 11.20 – Rice code parameters used for encoding of parcor coefficients
 * parcor_rice_table[table number][offset, rice param][parcor value]
 * (0-19) = coef index
 * 20 = odd indices 21 to 127
 * 21 = even indices 22 to 126
 * 22 = indices greater than 127
 */
static const int8_t parcor_rice_table[3][2][23] = {
    { {-52,-29,-31, 19,-16, 12, -7,  9, -5,  6, -4,  3, -3,  3, -2,  3, -1,  2, -1,  2,  0,  1,  0 },
      {  4,  5,  4,  4,  4,  3,  3,  3,  3,  3,  3,  3,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  1 } },

    { {-58,-42,-46, 37,-36, 29,-29, 25,-23, 20,-17, 16,-12, 12, 10,  7, -4,  3, -1,  1,  0,  1,  0 },
      {  3,  4,  4,  5,  4,  4,  4,  4,  4,  4,  4,  4,  4,  3,  4,  3,  4,  3,  3,  3,  2,  2,  1 } },

    { {-59,-45,-50, 38,-39, 32,-30, 25,-23, 20,-20, 16,-13, 10, -7,  3,  0, -1,  2, -1,  0,  1,  0 },
      {  3,  5,  4,  4,  4,  4,  4,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  2,  2,  2,  1 } }
};

#endif /* AVCODEC_ALSDATA_H */
