/*
 * Copyright © 2018, VideoLAN and dav1d authors
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

#include <stdlib.h>

#include "common/intops.h"

#include "src/levels.h"
#include "src/stx.h"
#include "src/stx_tables.h"
#include "src/tables.h"

static int stxfm8_c(coef *const cf, const enum RectTxfmSize tx, const int stx,
                    const int eob, const int transpose HIGHBD_DECL_SUFFIX)
{
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
    assert((stx & 3) != 0);
    assert(stx >= 0 && stx < (16 << 2));
    assert(eob >= 0 && eob < 32);
    assert(transpose == 0 || transpose == 1);
    assert(t_dim->lw && t_dim->lh);

    const int h = eob + 1;
    const int type = (stx & 3) - 1;
    const int set = (stx >> 2) & 15;
    const int8_t (*kernel)[48] = stx_8x8_kernel[set][type];
    int sums[48];
    for (int x = 0; x < 48; x++) {
        int sum = 0;
        for (int y = 0; y < h; y++)
            sum += cf[y] * kernel[y][x];
        sums[x] = apply_sign((abs(sum) + 64) >> 7, sum);
    }

    memset(cf, 0, 32 * sizeof(coef));
    // Subtract 1 to map {8,16,32} to idx {0,1,2}
    const int idx = t_dim->lw == t_dim->lh ? imin(t_dim->lw, 3) - 1 : 0;
    const uint8_t *scan_out = stx_scan_orders_8x8[idx][transpose];
    const uint8_t *mapping = coeff8x8_mapping[set * 3 + type];
    const int min = -128 * (1 + BITDEPTH_MAX);
    const int max = 128 * (1 + BITDEPTH_MAX) - 1;
    for (int x = 0; x < 48; x++) {
        cf[scan_out[mapping[x]]] = iclip(sums[x], min, max);
    }

    // Return the scan index (EOB) of the last coeff affected by the 8x8 STX
    // 8x8 -> 63, 8x16+ -> 91, 16x8 & 32x8 -> 84, otherwise -> 112
    return 63 + !!idx * 21 + !!(t_dim->lh - 1) * 28;
}

static int stxfm4_c(coef *const cf, const enum RectTxfmSize tx, const int stx,
                     const int eob, const int transpose HIGHBD_DECL_SUFFIX)
{
    assert((stx & 3) != 0);
    assert(stx >= 0 && stx < (16 << 2));
    assert(eob >= 0 && eob < 8);
    assert(transpose == 0 || transpose == 1);

    const int type = (stx & 3) - 1;
    const int set = (stx >> 2) & 15;
    const int8_t (*kernel)[16] = stx_4x4_kernel[set][type];
    const int h = eob + 1;
    int sums[16];
    for (int x = 0; x < 16; x++) {
        int sum = 0;
        for (int y = 0; y < h; y++) {
            sum += cf[y] * kernel[y][x];
        }
        sums[x] = apply_sign((abs(sum) + 64) >> 7, sum);
    }

    memset(cf, 0, 8 * sizeof(coef));
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
    const int idx = imin(t_dim->lw, 3);
    const uint8_t *scan_out = stx_scan_orders_4x4[idx][transpose];
    const int min = -128 * (1 + BITDEPTH_MAX);
    const int max = 128 * (1 + BITDEPTH_MAX) - 1;
    for (int x = 0; x < 16; x++) {
        cf[scan_out[x]] = iclip(sums[x], min, max);
    }

    // Return the scan index (EOB) of the last coeff affected by the 4x4 STX
    // 4x4 -> 15, 8x4+ -> 18, 4x8+ -> 21
    return 15 + !!t_dim->lw * 3 + !!t_dim->lh * 6;
}

COLD void bitfn(dav1d_stx_dsp_init)(Dav1dStxDSPContext *const c) {
    c->stxfm[0] = stxfm4_c;
    c->stxfm[1] = stxfm8_c;
}
