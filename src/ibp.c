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

#include "common/intops.h"

#include "src/ibp.h"
#include "src/levels.h"
#include "src/tables.h"

uint8_t dav2d_ibp_weights[7][16][16];

static inline unsigned fast_div32(const unsigned num, const unsigned den) {
    unsigned shift = ulog2(den);
    const unsigned rem = den - (1 << shift);
    // Quantize fractional part of divisor between powers of two (0..128)
    const unsigned idx = ((rem << 7) + (1 << (shift - 1))) >> shift;
    assert(idx <= 128);
    shift += 2;
    const unsigned res = ((num * dav2d_div_recip[idx]) + ((1 << shift) >> 1)) >> shift;
    assert(res < 256);
    return res;
}

COLD void dav2d_init_ibp_weights(void) {
    static const int dr_dy_q6[7] = { 682, 256, 170, 128, 81, 64, 50 };
    for (int m = 0; m < 7; m++) {
        const int dy = dr_dy_q6[m];
        for (int y = 0; y < 16; y++) {
            const int yy = (y + 1) << 6;
            int y_pos = dy;
            for (int x = 0; x < 16; x++, y_pos += dy) {
                dav2d_ibp_weights[m][y][x] = fast_div32(y_pos, yy + y_pos);
            }
        }
    }
}
