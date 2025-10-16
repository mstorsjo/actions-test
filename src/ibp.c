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

#include "common/intops.h"

#include "src/ibp.h"
#include "src/levels.h"

uint8_t dav1d_ibp_weights[7][16][16];

static inline unsigned fast_div32(const unsigned num, const unsigned den) {
    static const uint16_t recip[128 + 1] = {
        512, 508, 504, 500, 496, 493, 489, 485, 482, 478, 475, 471, 468, 465, 462,
        458, 455, 452, 449, 446, 443, 440, 437, 434, 431, 428, 426, 423, 420, 417,
        415, 412, 410, 407, 405, 402, 400, 397, 395, 392, 390, 388, 386, 383, 381,
        379, 377, 374, 372, 370, 368, 366, 364, 362, 360, 358, 356, 354, 352, 350,
        349, 347, 345, 343, 341, 340, 338, 336, 334, 333, 331, 329, 328, 326, 324,
        323, 321, 320, 318, 317, 315, 314, 312, 311, 309, 308, 306, 305, 303, 302,
        301, 299, 298, 297, 295, 294, 293, 291, 290, 289, 287, 286, 285, 284, 282,
        281, 280, 279, 278, 277, 275, 274, 273, 272, 271, 270, 269, 267, 266, 265,
        264, 263, 262, 261, 260, 259, 258, 257, 256
    };

    const unsigned l2_den = ulog2(den);
    const unsigned rem = den - (1 << l2_den);
    // Quantize fractional part of divisor between powers of two (0..128)
    const unsigned idx = ((rem << 7) + (1 << (l2_den - 1))) >> l2_den;
    assert(idx <= 128);
    const unsigned shift = l2_den + 2;
    const unsigned res = ((num * recip[idx]) + ((1 << shift) >> 1)) >> shift;
    assert(res < 256);
    return res;
}

COLD void dav1d_init_ibp_weights(void) {
    static const int dr_dy_q6[7] = { 682, 256, 170, 128, 81, 64, 50 };
    for (int m = 0; m < 7; m++) {
        const int dy = dr_dy_q6[m];
        for (int y = 0; y < 16; y++) {
            const int yy = (y + 1) << 6;
            int y_pos = dy;
            for (int x = 0; x < 16; x++, y_pos += dy) {
                dav1d_ibp_weights[m][y][x] = fast_div32(y_pos, yy + y_pos);
            }
        }
    }
}
