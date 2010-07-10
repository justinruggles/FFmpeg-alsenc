/*
 * MPEG-4 ALS common header
 * Copyright (c) 2010 Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
 * Copyright (c) 2010 Justin Ruggles <justin.ruggles _at_ gmail.com>
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
 * @file libavcodec/als.h
 * MPEG-4 ALS common header
 * @author Thilo Borgmann <thilo.borgmann _at_ googlemail.com>
 * @author Justin Ruggles <justin.ruggles@gmail.com>
 */

#ifndef AVCODEC_ALS_H
#define AVCODEC_ALS_H

#include <stdint.h>

#include "avcodec.h"


enum RA_Flag {
    RA_FLAG_NONE,
    RA_FLAG_FRAMES,
    RA_FLAG_HEADER
};


typedef struct {
    uint32_t samples;         ///< number of samples, 0xFFFFFFFF if unknown
    int resolution;           ///< 000 = 8-bit; 001 = 16-bit; 010 = 24-bit; 011 = 32-bit
    int floating;             ///< 1 = IEEE 32-bit floating-point, 0 = integer
    int msb_first;            ///< 1 = original CRC calculated on big-endian system, 0 = little-endian
    int frame_length;         ///< frame length for each frame (last frame may differ)
    int ra_distance;          ///< distance between RA frames (in frames, 0...255)
    enum RA_Flag ra_flag;     ///< indicates where the size of ra units is stored
    int adapt_order;          ///< adaptive order: 1 = on, 0 = off
    int coef_table;           ///< table index of Rice code parameters
    int long_term_prediction; ///< long term prediction (LTP): 1 = on, 0 = off
    int max_order;            ///< maximum prediction order (0..1023)
    int block_switching;      ///< number of block switching levels
    int bgmc;                 ///< "Block Gilbert-Moore Code": 1 = on, 0 = off (Rice coding only)
    int sb_part;              ///< sub-block partition
    int joint_stereo;         ///< joint stereo: 1 = on, 0 = off
    int mc_coding;            ///< extended inter-channel coding (multi channel coding): 1 = on, 0 = off
    int chan_config;          ///< indicates that a chan_config_info field is present
    int chan_sort;            ///< channel rearrangement: 1 = on, 0 = off
    int rlslms;               ///< use "Recursive Least Square-Least Mean Square" predictor: 1 = on, 0 = off
    int chan_config_info;     ///< mapping of channels to loudspeaker locations. Unused until setting channel configuration is implemented.
    int *chan_pos;            ///< original channel positions
    int crc_enabled;          ///< enable Cyclic Redundancy Checksum
} ALSSpecificConfig;


/**
 * Print out AlSSpecificConfig
 */
void ff_als_dprint_specific_config(AVCodecContext *avctx,
                                   ALSSpecificConfig *sconf);


/**
 * Convert PARCOR coefficient k to direct filter coefficient.
 */
int ff_als_parcor_to_lpc(unsigned int k, const int32_t *par, int32_t *cof);


/** Parse the bs_info field to extract the block partitioning used in
 *  block switching mode, refer to ISO/IEC 14496-3, section 11.6.2.
 */
void ff_als_parse_bs_info(const uint32_t bs_info, unsigned int n,
                          unsigned int div, unsigned int **div_blocks,
                          unsigned int *num_blocks);

#endif /* AVCODEC_ALS_H */
