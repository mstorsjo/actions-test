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

#include "config.h"

#include <stdlib.h>

#include "common/intops.h"

#include "src/levels.h"
#include "src/stx.h"
#include "src/stx_tables.h"
#include "src/tables.h"

static void stxfm_c(coef *const cf_out,
                    const coef *const cf,
                    const int8_t *kernel,
                    const int sz,
                    const int eob HIGHBD_DECL_SUFFIX)
{
    assert(sz == 16 || sz == 48);
    assert(eob >= 0 && eob < (sz == 16 ? 8 : 32));
    const int min = -128 * (1 + BITDEPTH_MAX);
    const int max = 128 * (1 + BITDEPTH_MAX) - 1;
    const int h = eob + 1;
    for (int x = 0; x < sz; x++) {
        int sum = 0;
        for (int y = 0; y < h; y++)
            sum += cf[y] * kernel[y * sz + x];
        sum = apply_sign((abs(sum) + 64) >> 7, sum);
        cf_out[x] = iclip(sum, min, max);
    }
}

COLD void bitfn(dav2d_stx_dsp_init)(Dav2dStxDSPContext *const c) {
    c->stxfm = stxfm_c;
}
