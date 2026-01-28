/*
 * Copyright © 2018-2026, VideoLAN and dav2d authors
 * Copyright © 2018-2026, Two Orioles, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DAV2D_SRC_LF_MASK_H
#define DAV2D_SRC_LF_MASK_H

#include <stddef.h>
#include <stdint.h>

#include "src/levels.h"

typedef struct Av2FilterLUT {
    uint16_t thr[2 /* col, row */][2 /* 0 = q_thr, 1 = side_thr */][16 /* seg_id */];
    uint16_t thr_uv[2 /* u, v */][2 /* 0 = q_thr, 1 = side_thr */][16 /* seg_id */];
} Av2FilterLUT;

typedef struct Av2RestorationUnit {
    uint8_t /* enum Dav2dRestorationType */ type;
    int8_t ns_filter[16][32];
} Av2RestorationUnit;

// each struct describes one 256x256 area
typedef struct Av2Filter {
    // each bit is 1 col
    uint16_t filter_y[2 /* 0=col, 1=row */][64][5][4];
    uint16_t filter_uv[2 /* 0=col, 1=row */][64][4][4];
    uint8_t gdf[4];
    int8_t cdef_idx[16]; // -1 means "unset"
    uint8_t ccso[3];
    uint16_t noskip_mask[32][4]; // for 8x8 blocks, but stored on a 4x8 basis
} Av2Filter;

// each struct describes one 256x256 area (1, 4, or 16 SBs)
typedef struct Av2Restoration {
    Av2RestorationUnit lr[3][16];
} Av2Restoration;

void dav2d_create_lf_mask_luma(Av2Filter *lflvl, const Av2Block *b,
                               enum BlockSize lbs, int bx, int by,
                               int iw, int ih, uint8_t *ay, uint8_t *ly,
                               const Dav2dFrameHeader *frame_hdr,
                               const Dav2dSequenceHeader *seq_hdr);
void dav2d_create_lf_mask_chroma(Av2Filter *lflvl, const Av2Block *b,
                                 enum BlockSize cbs, int cbx, int cby,
                                 int iw, int ih, enum Dav2dPixelLayout layout,
                                 uint8_t *auv, uint8_t *luv,
                                 const Dav2dFrameHeader *frame_hdr,
                                 const Dav2dSequenceHeader *seq_hdr);

#endif /* DAV2D_SRC_LF_MASK_H */
