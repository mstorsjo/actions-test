/*
 * Copyright © 2019, VideoLAN and dav2d authors
 * Copyright © 2019, Martin Storsjo
 * Copyright © 2026, Arm Limited
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

#include "src/arm/asm.S"
#include "util.S"

#define BUF_POS 0
#define BUF_END 8
#define DIF 16
#define RNG 24
#define CNT 28
#define ALLOW_UPDATE_CDF 32

function msac_decode_bool_bypass_neon, export=1
        ldp             w5,  w6,  [x0, #RNG]   // + CNT
        ldr             x7,  [x0, #DIF]
        lsl             x5,  x5,  #47          // r << 47
        mov             x15, #0                // ret = 0
        subs            x2,  x7,  x5           // dif - vw
        mov             x8,  #1
        csel            x7,  x2,  x7, hs       // if (dif >= vw) dif -= vw
        cinc            x15, x15, lo           // if (dif < vw) ret |= 1
        orr             x7,  x8,  x7, lsl #1   // dif << bits
        subs            w6,  w6,  #1           // cnt -= bits
        adr             x10, #8
        b.mi            L(refill)
        str             x7,  [x0, #DIF]
        str             w6,  [x0, #CNT]
        mov             x0,  x15
        ret

L(refill):
        // refill
        ldp             x3,  x4,  [x0]         // BUF_POS, BUF_END
        add             x5,  x3,  #8
        subs            x5,  x5,  x4
        b.hi            5f

        ldr             x8,  [x3]              // next_bits
        add             w4,  w6,  #-48         // shift_bits = cnt + 16 (- 64)
        neg             w9,  w6
        neg             w5,  w4
        and             w9,  w9,  #7
        rev             x8,  x8                // next_bits = bswap(next_bits)
        lsr             w5,  w5,  #3           // num_bytes_read
        lsr             x8,  x8,  x4           // next_bits >>= (shift_bits & 63)

2:      // refill_end
        add             x3,  x3,  x5
        add             w6,  w6,  w5, lsl #3   // cnt += num_bits_read
        str             x3,  [x0, #BUF_POS]

3:      // refill_end2
        lsr             x8,  x8,  x9           // throw away the lowest bits
        lsl             x8,  x8,  x9           // shift back in place
        eor             x7,  x7,  x8           // dif ^= next_bits

4:      // end
        str             w6,  [x0, #CNT]
        str             x7,  [x0, #DIF]

        mov             w0,  w15
        ret

5:      // refill_eob
        cmp             x3,  x4
        b.hs            4b

        ldr             x8,  [x4, #-8]
        neg             w9,  w6
        lsl             w5,  w5,  #3
        and             w9,  w9,  #7
        lsr             x8,  x8,  x5
        add             w5,  w6,  #-48
        rev             x8,  x8
        sub             w4,  w4,  w3           // num_bytes_left
        lsr             x8,  x8,  x5
        neg             w5,  w5
        lsr             w5,  w5,  #3
        cmp             w5,  w4
        csel            w5,  w5,  w4,  lo      // num_bytes_read
        b               2b

endfunc

function msac_decode_bools_bypass_neon, export=1
        ldp             w11, w6,  [x0, #RNG]   // + CNT
        ldr             x7,  [x0, #DIF]
        cmp             w1,  w6
        b.ls            3f
        // We can't ret directly from refill here so inline a special version
        ldp             x3,  x4,  [x0]         // BUF_POS, BUF_END
        add             x5,  x3,  #8
        subs            x5,  x5,  x4
        b.hi            1f

        ldr             x8,  [x3]              // next_bits
        add             w4,  w6,  #-48         // shift_bits = cnt + 16 (- 64)
        neg             w9,  w6
        neg             w5,  w4
        and             w9,  w9,  #7
        rev             x8,  x8                // next_bits = bswap(next_bits)
        lsr             w5,  w5,  #3           // num_bytes_read
        lsr             x8,  x8,  x4           // next_bits >>= (shift_bits & 63)
        b               2f

1:      // refill_eob
        cmp             x3,  x4
        b.hs            3f

        ldr             x8,  [x4, #-8]
        neg             w9,  w6
        lsl             w5,  w5,  #3
        and             w9,  w9,  #7
        lsr             x8,  x8,  x5
        add             w5,  w6,  #-48
        rev             x8,  x8
        sub             w4,  w4,  w3           // num_bytes_left
        lsr             x8,  x8,  x5
        neg             w5,  w5
        lsr             w5,  w5,  #3
        cmp             w5,  w4
        csel            w5,  w5,  w4,  lo      // num_bytes_read

2:      // refill_end
        add             x3,  x3,  x5
        add             w6,  w6,  w5, lsl #3   // cnt += num_bits_read
        str             x3,  [x0, #BUF_POS]
        lsr             x8,  x8,  x9           // throw away the lowest bits
        lsl             x8,  x8,  x9           // shift back in place
        eor             x7,  x7,  x8           // dif ^= next_bits

3:
        lsl             x5,  x11, #47          // r << 47
        mov             x15, #0
        mov             w3,  w1
4:
        sub             w1,  w1,  #1           // n_bits
        subs            x2,  x7,  x5           // dif - vw
        lsl             x15, x15, #1           // ret <<= 1
        lsr             x5,  x5,  #1           // vw >> 1
        csel            x7,  x2,  x7, hs       // if (dif >= vw) dif -= vw
        cinc            x15, x15, lo           // if (dif < vw) ret |= 1
        cbnz            w1,  4b

        mvn             x7,  x7
        sub             w6,  w6,  w3           // cnt -= bits
        lsl             x7,  x7,  x3           // dif << bits
        mvn             x7,  x7
        str             w6,  [x0, #CNT]
        str             x7,  [x0, #DIF]
        mov             x0,  x15
        ret
endfunc

function msac_decode_unary_bypass_neon, export=1
        ldp             w11, w6,  [x0, #RNG]   // + CNT
        ldr             x7,  [x0, #DIF]
        cmp             w1,  w6
        b.ls            3f
        // We can't ret directly from refill here so inline a special version
        ldp             x3,  x4,  [x0]         // BUF_POS, BUF_END
        add             x5,  x3,  #8
        subs            x5,  x5,  x4
        b.hi            1f

        ldr             x8,  [x3]              // next_bits
        add             w4,  w6,  #-48         // shift_bits = cnt + 16 (- 64)
        neg             w9,  w6
        neg             w5,  w4
        and             w9,  w9,  #7
        rev             x8,  x8                // next_bits = bswap(next_bits)
        lsr             w5,  w5,  #3           // num_bytes_read
        lsr             x8,  x8,  x4           // next_bits >>= (shift_bits & 63)
        b               2f

1:      // refill_eob
        cmp             x3,  x4
        b.hs            3f

        ldr             x8,  [x4, #-8]
        neg             w9,  w6
        lsl             w5,  w5,  #3
        and             w9,  w9,  #7
        lsr             x8,  x8,  x5
        add             w5,  w6,  #-48
        rev             x8,  x8
        sub             w4,  w4,  w3           // num_bytes_left
        lsr             x8,  x8,  x5
        neg             w5,  w5
        lsr             w5,  w5,  #3
        cmp             w5,  w4
        csel            w5,  w5,  w4,  lo      // num_bytes_read

2:      // refill_end
        add             x3,  x3,  x5
        add             w6,  w6,  w5, lsl #3   // cnt += num_bits_read
        str             x3,  [x0, #BUF_POS]
        lsr             x8,  x8,  x9           // throw away the lowest bits
        lsl             x8,  x8,  x9           // shift back in place
        eor             x7,  x7,  x8           // dif ^= next_bits

3:
        lsl             x5,  x11, #47          // r << 47
        mov             x15, #0                // ret
        mov             w3,  #0                // bit
4:
        add             w3,  w3,  #1
        subs            x2,  x7,  x5           // dif - vw
        b.lo            5f
        lsr             x5,  x5,  #1           // vw >> 1
        mov             x7,  x2                // dif -= vw
        add             x15, x15, #1           // ret++
        cmp             w3,  w1
        b.ne            4b
5:
        mvn             x7,  x7
        sub             w6,  w6,  w3            // cnt -= bits
        lsl             x7,  x7,  x3            // dif << bits
        str             w6,  [x0, #CNT]
        mvn             x7,  x7
        str             x7,  [x0, #DIF]
        mov             x0,  x15
        ret
endfunc

function msac_decode_bool_adapt_neon, export=1
        ldp             w5,  w6,  [x0, #RNG]   // + CNT
        ldrh            w9,  [x1]              // cdf[0]
        mov             w2,  #0xFFF0
        ldr             x7,  [x0, #DIF]
        mov             w8,  #0xFFFFFFF8
        lsr             w4,  w5,  #8           // r >> 8
        and             w2,  w2,  w9, lsr #3   // (f >> 7) << 4
        add             w2,  w2,  #8           // p = ((f >> 7) << 4) + 8
        mul             w4,  w4,  w2           // (r >> 8) * p
        ldr             w10, [x0, #ALLOW_UPDATE_CDF]
        and             w4,  w8,  w4,  lsr #4  // v = ((r >> 8) * p >> 7) << 3
        subs            x8,  x7,  x4,  lsl #48 // dif - vw
        sub             w5,  w5,  w4           // r - v
        cset            w15, lo
        csel            w4,  w5,  w4,  hs      // if (ret) v = r - v;
        csel            x7,  x8,  x7,  hs      // if (ret) dif = dif - vw;

        clz             w5,  w4                // clz(rng)
        eor             w5,  w5,  #16          // d = clz(rng) ^ 16

        cbz             w10, 1f

        ldrh            w2,  [x1, #2]          // cdf[1]

        lsr             w12, w2,  #8           // pc >> 8
        and             w8,  w2,  #0xff        // (uint8_t)count = pc & 0xff
        add             w12, w12, w12, lsl #1  // pc >> 8 * 3
        sub             w3,  w2,  w8,  lsr #5  // pc - (count >= 32)
        movrel          x11, X(msac_rate)
        add             w8,  w12,  w8, lsr #4  // pc >> 8 * 3 + count >> 4
        sub             w9,  w9,  w15          // cdf[0] -= bit
        ldrb            w2,  [x11, x8]         // rate
        sub             w11, w9,  w15, lsl #15 // {cdf[0], cdf[0] - 32769}
        add             w10, w3,  #1           // pc + (count < 32)
        asr             w11, w11, w2           // {cdf[0], cdf[0] - 32769} >> rate
        sub             w9,  w9,  w11          // cdf[0]

        strh            w10, [x1, #2]
        strh            w9,  [x1]

1:
        mvn             x7,  x7                // mvn/lsl/mvn is equivalent to ((dif + 1) << d) - 1
        lsl             w4,  w4,  w5           // rng << d
        lsl             x7,  x7,  x5           // dif << d
        subs            w6,  w6,  w5           // cnt -= d
        mvn             x7,  x7
        str             w4,  [x0, #RNG]
        b.lo            L(refill)

        str             w6,  [x0, #CNT]
        str             x7,  [x0, #DIF]
        mov             w0,  w15
        ret
endfunc
