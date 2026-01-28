/*
 * Copyright © 2018, VideoLAN and dav2d authors
 * Copyright © 2018, Two Orioles, LLC
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

#ifndef DAV2D_SRC_WEDGE_H
#define DAV2D_SRC_WEDGE_H

#include "src/levels.h"

typedef struct {
    /* Offsets, in units of 8 bytes, relative to the start of the struct. */
    struct {
        uint8_t wedge[N_BS_SIZES - BS_64x64 - 6];
        uint16_t ii_nondc[N_BS_SIZES - BS_64x64];
    } offsets;
    uint8_t *wedge[3];
    uint8_t ALIGN(wedge_444[68 * (64 + 32 + 16 + 8) * (64 + 32 + 16 + 8)], 64);
    uint8_t ALIGN(wedge_422[68 * (32 + 16 + 8 + 4) * (64 + 32 + 16 + 8)], 64);
    uint8_t ALIGN(wedge_420[68 * (32 + 16 + 8 + 4) * (32 + 16 + 8 + 4)], 64);
    uint8_t ALIGN(wedge_tmvp[68 * (8 + 4 + 2 + 1) * (8 + 4 + 2 + 1)], 64);
    uint8_t ALIGN(ii_dc[64 * 64], 64);
    uint8_t ALIGN(ii_nondc[(64 + 32 + 16 + 8 + 4) * (64 + 32 + 16 + 8 + 4) * 3], 64);
} Dav2dMasks;

#define II_MASK(bs, bw4, bh4, ii_mode) \
    (ii_mode == II_DC_PRED ? dav2d_masks.ii_dc : \
     &dav2d_masks.ii_nondc[dav2d_masks.offsets.ii_nondc[bs - BS_64x64] * 0x60 + \
                           16 * (bw4) * (bh4) * (ii_mode - 1)])

#define WEDGE_MASK(bs, bw4, bh4, widx, ssidx) \
    &dav2d_masks.wedge[ssidx][(dav2d_masks.offsets.wedge[bs - BS_64x64] * 0x1100 + \
                               16 * bw4 * bh4 * widx) >> (ssidx)]

#define WEDGE_TMVP(bs, bw4, bh4, widx) \
    &dav2d_masks.wedge_tmvp[dav2d_masks.offsets.wedge[bs - BS_64x64] * 68 + \
                            ((bw4) * (bh4) >> 2) * widx]

EXTERN Dav2dMasks dav2d_masks;

void dav2d_init_ii_wedge_masks(void);

#endif /* DAV2D_SRC_WEDGE_H */
