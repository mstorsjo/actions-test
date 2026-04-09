/*
 * Copyright © 2026, VideoLAN and dav2d authors
 * Copyright © 2026, Two Orioles, LLC
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

#include "common/attributes.h"
#include "common/intops.h"

#include "src/tables.h"

static inline int derive_alpha(const int num, const int den, int alpha) {
    const int max = (2 << 8) - 1;
    if (num && den) {
        const int num_abs = abs(num);
        const int shift_n = ulog2(num_abs);
        assert(den >= 0);
        const int shift_d = ulog2(den);
        const int e_d = den - (1U << shift_d);
        int f_d, f_n;
        if (shift_d > 7)
            f_d = (e_d + (1 << (shift_d - 8))) >> (shift_d - 7);
        else
            f_d = e_d << (7 - shift_d);

        if (shift_n > 7)
            f_n = (num_abs + (1 << (shift_n - 8))) >> (shift_n - 7);
        else
            f_n = num_abs << (7 - shift_n);

        const int shift_add = shift_d - shift_n - 8;
        if (shift_add <= 1) {
            const int shift0 = 9 + 7 + shift_add;
            const int tmp_alpha = shift0 < 0 ? max :
                imin((dav2d_div_recip[f_d] * f_n) >> shift0, max);
            if (tmp_alpha)
                alpha = apply_sign(tmp_alpha, num);
        }
    }
    return alpha;
}

