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

static void stxfm_c(coef *const cf, const enum RectTxfmSize tx, const int stx,
                    const int eob, const int transpose HIGHBD_DECL_SUFFIX)
{
    assert((stx & 3) != 0);
    assert(stx >= 0 && stx < (16 << 2));
    assert(eob >= 0 && eob < 32);
    assert(transpose == 0 || transpose == 1);

    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
    const int w = (t_dim->w < 2 || t_dim->h < 2) ? 16 : 48;
    const int h = eob + 1;
    const int type = (stx & 3) - 1;
    const int set = (stx >> 2) & 15;
    const int8_t (*kernel)[48] = stx_8x8_kernel[set][type];
    int sums[48];
    for (int x = 0; x < w; x++) {
        int sum = 0;
        for (int y = 0; y < h; y++)
            sum += cf[y] * kernel[y][x];
        sums[x] = apply_sign((abs(sum) + 64) >> 7, sum);
    }

    memset(cf, 0, 32 * sizeof(coef));
    const int sqr_tx = imin(t_dim->min, TX_32X32) - 1;
    const uint8_t *scan_out = stx_scan_orders_8x8[sqr_tx][transpose];
    const uint8_t *mapping = coeff8x8_mapping[set * 3 + type];
    const int min = -128 * (1 + BITDEPTH_MAX);
    const int max = 128 * (1 + BITDEPTH_MAX) - 1;
    for (int x = 0; x < w; x++) {
        cf[scan_out[mapping[x]]] = iclip(sums[x], min, max);
    }
}

COLD void bitfn(dav1d_stx_dsp_init)(Dav1dStxDSPContext *const c) {
    c->stxfm = stxfm_c;
}
