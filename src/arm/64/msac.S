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

#define COEFFS_BASE_OFFSET 30
#define MASKS8_OFFSET (64-COEFFS_BASE_OFFSET)

const coeffs
        .short 60, 56, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 12, 8, 4, 0
        .short 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 0, 0
        // masks8
        .short -0x202, -0x202, -0x202, -0x202, -0x202, -0x202, -0x202, 0xF0E
endconst

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

function msac_decode_hi_tok_neon, export=1
        ld1             {v0.4h},  [x1]            // cdf
        add             x16, x0,  #RNG
        movi            v31.4h, #0x7f, lsl #8     // 0x7f00
        movrel          x17, coeffs, COEFFS_BASE_OFFSET-2*3
        mvni            v30.4h, #0x3f             // 0xffc0
        ldrh            w9,  [x1, #6]             // count = cdf[n_symbols]
        ld1r            {v3.4h},  [x16]           // rng
        ld1             {v29.4h}, [x17]           // EC_MIN_PROB * (n_symbols - ret)
        add             x17, x0,  #DIF + 6
        mov             w13, #-24*8
        and             v17.8b,  v0.8b,   v30.8b  // cdf & 0xffc0
        ldr             w10, [x0, #ALLOW_UPDATE_CDF]
        ld1r            {v1.8h},  [x17]           // dif >> (EC_WIN_SIZE - 16)
        ldr             w6,  [x0, #CNT]
        ldr             x7,  [x0, #DIF]
1:
        and             v7.8b,   v3.8b,   v31.8b  // rng & 0x7f00
        sqdmulh         v6.4h,   v17.4h,  v7.4h   // ((cdf >> EC_PROB_SHIFT) * (r - 128)) >> 1
        add             v4.4h,   v17.4h,  v29.4h  // v = cdf + EC_MIN_PROB * (n_symbols - ret)
        add             v4.4h,   v6.4h,   v4.4h   // v = ((cdf >> EC_PROB_SHIFT) * r) >> 1 + EC_MIN_PROB * (n_symbols - ret)
        cmhs            v2.4h,   v1.4h,   v4.4h   // c >= v
        add             w13, w13, #5*8
        ext             v18.8b, v3.8b,  v4.8b, #6 // u
        umov            x15, v2.d[0]
        rev             x15, x15
        sub             v18.4h, v18.4h, v4.4h     // rng = u-v
        // rev + clz = count trailing zeros
        clz             x15, x15                  // 16*ret

        cbz             w10, 2f
        // update_cdf
        sub             v5.4h,   v0.4h,   v2.4h   // cdf[i] + (i >= val ? 1 : 0)
        mov             w4,  #-5
        orr             v2.4h, #0x80, lsl #8      // i >= val ? -1 : 32768
        sub             w4,  w4,  w9, lsr #4      // -((count >> 4) + 5)
        sub             v2.4h,   v2.4h,   v0.4h   // (32768 - cdf[i]) or (-1 - cdf[i])
        dup             v6.4h,    w4              // -rate

        sub             w9,  w9,  w9, lsr #5      // count - (count == 32)
        sshl            v2.4h,   v2.4h,   v6.4h   // ({32768,-1} - cdf[i]) >> rate
        add             w9,  w9,  #1              // count + (count < 32)
        add             v0.4h,   v5.4h,   v2.4h   // cdf[i] + (32768 - cdf[i]) >> rate
        st1             {v0.4h},  [x1]
        and             v17.8b,  v0.8b,   v30.8b  // cdf & 0xffc0
        strh            w9,  [x1, #6]

2:
        mov             x4,  v18.d[0]          // rng (packed)
        mov             x3,  v4.d[0]           // v (packed)

        // Shift 'v'/'rng' for ret into the 16 least sig bits. There is
        //  garbage in the remaining bits, but we can work around this.
        lsr             x4,  x4,  x15          // rng
        lsr             x3,  x3,  x15          // v
        lsl             w5,  w4,  #16          // rng << 16
        sub             x7,  x7,  x3, lsl #48  // dif - (v << 48)
        clz             w5,  w5                // d = clz(rng << 16)
        lsl             w4,  w4,  w5           // rng << d
        subs            w6,  w6,  w5           // cnt -= d
        lsl             x7,  x7,  x5           // (dif - (v << 48)) << d
        strh            w4,  [x0, #RNG]
        dup             v3.4h,   w4
        b.hs            5f

        // refill
        ldp             x3,  x4,  [x0]         // BUF_POS, BUF_END
        add             x5,  x3,  #8
        subs            x5,  x5,  x4
        b.hi            7f

        ldr             x8,  [x3]              // next_bits
        add             w4,  w6,  #-48         // shift_bits = cnt + 16 (- 64)
        mvn             x8,  x8
        neg             w5,  w4
        rev             x8,  x8                // next_bits = bswap(next_bits)
        lsr             w5,  w5,  #3           // num_bytes_read
        lsr             x8,  x8,  x4           // next_bits >>= (shift_bits & 63)

3:      // refill_end
        add             x3,  x3,  x5
        add             w6,  w6,  w5, lsl #3   // cnt += num_bits_read
        str             x3,  [x0, #BUF_POS]

4:      // refill_end2
        orr             x7,  x7,  x8           // dif |= next_bits

5:      // end
        sub             w15, w15, #5*8
        lsr             x12, x7,  #48
        adds            w13, w13, w15          // carry = tok_br < 3 || tok == 15
        dup             v1.8h,   w12
        b.cc            1b                     // loop if !carry
        add             w13, w13, #30*8
        str             w6,  [x0, #CNT]
        str             x7,  [x0, #DIF]
        lsr             w0,  w13, #4
        ret

6:      // pad_with_ones
        add             w8,  w6,  #-16
        ror             x8,  x8,  x8
        b               4b

7:      // refill_eob
        cmp             x3,  x4
        b.hs            6b

        ldr             x8,  [x4, #-8]
        lsl             w5,  w5,  #3
        lsr             x8,  x8,  x5
        add             w5,  w6,  #-48
        mvn             x8,  x8
        sub             w4,  w4,  w3           // num_bytes_left
        rev             x8,  x8
        lsr             x8,  x8,  x5
        neg             w5,  w5
        lsr             w5,  w5,  #3
        cmp             w5,  w4
        csel            w5,  w5,  w4,  lo      // num_bytes_read
        b               3b
endfunc

function msac_decode_bool_equi_neon, export=1
        ldp             w5,  w6,  [x0, #RNG]   // + CNT
        ldr             x7,  [x0, #DIF]
        bic             w4,  w5,  #0xff        // r &= 0xff00
        add             w4,  w4,  #8
        subs            x8,  x7,  x4, lsl #47  // dif - vw
        lsr             w4,  w4,  #1           // v
        sub             w5,  w5,  w4           // r - v
        cset            w15, lo
        csel            w4,  w5,  w4,  hs      // if (ret) v = r - v;
        csel            x7,  x8,  x7,  hs      // if (ret) dif = dif - vw;

        clz             w5,  w4                // clz(rng)
        eor             w5,  w5,  #16          // d = clz(rng) ^ 16
        lsl             w4,  w4,  w5           // rng << d
        subs            w6,  w6,  w5           // cnt -= d
        lsl             x7,  x7,  x5           // (dif - (v << 48)) << d
        str             w4,  [x0, #RNG]
        b.lo            L(refill)

        str             w6,  [x0, #CNT]
        str             x7,  [x0, #DIF]
        mov             w0,  w15
        ret
endfunc

function msac_decode_bool_neon, export=1
        ldp             w5,  w6,  [x0, #RNG]   // + CNT
        ldr             x7,  [x0, #DIF]
        lsr             w4,  w5,  #8           // r >> 8
        bic             w1,  w1,  #0x3f        // f &= ~63
        mul             w4,  w4,  w1
        lsr             w4,  w4,  #7
        add             w4,  w4,  #4           // v
        subs            x8,  x7,  x4, lsl #48  // dif - vw
        sub             w5,  w5,  w4           // r - v
        cset            w15, lo
        csel            w4,  w5,  w4,  hs      // if (ret) v = r - v;
        csel            x7,  x8,  x7,  hs      // if (ret) dif = dif - vw;

        clz             w5,  w4                // clz(rng)
        eor             w5,  w5,  #16          // d = clz(rng) ^ 16
        lsl             w4,  w4,  w5           // rng << d
        subs            w6,  w6,  w5           // cnt -= d
        lsl             x7,  x7,  x5           // (dif - (v << 48)) << d
        str             w4,  [x0, #RNG]
        b.lo            L(refill)

        str             w6,  [x0, #CNT]
        str             x7,  [x0, #DIF]
        mov             w0,  w15
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
