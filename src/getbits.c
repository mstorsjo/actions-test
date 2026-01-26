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

#include <limits.h>

#include "common/intops.h"

#include "src/getbits.h"

void dav2d_init_get_bits(GetBits *const c, const uint8_t *const data,
                         const size_t sz)
{
    assert(sz);
    c->ptr = c->ptr_start = data;
    c->ptr_end = &c->ptr_start[sz];
    c->state = 0;
    c->bits_left = 0;
    c->error = 0;
}

unsigned dav2d_get_bit(GetBits *const c) {
    if (!c->bits_left) {
        if (c->ptr >= c->ptr_end) {
            c->error = 1;
        } else {
            const unsigned state = *c->ptr++;
            c->bits_left = 7;
            c->state = (uint64_t) state << 57;
            return state >> 7;
        }
    }

    const uint64_t state = c->state;
    c->bits_left--;
    c->state = state << 1;
    return (unsigned) (state >> 63);
}

static inline void refill(GetBits *const c, const int n) {
    assert(c->bits_left >= 0 && c->bits_left < 32);
    unsigned state = 0;
    do {
        if (c->ptr >= c->ptr_end) {
            c->error = 1;
            if (state) break;
            return;
        }
        state = (state << 8) | *c->ptr++;
        c->bits_left += 8;
    } while (n > c->bits_left);
    c->state |= (uint64_t) state << (64 - c->bits_left);
}

#define GET_BITS(name, type, type64)            \
type name(GetBits *const c, const int n) {      \
    assert(n > 0 && n <= 32);                   \
    /* Unsigned cast avoids refill after eob */ \
    if ((unsigned) n > (unsigned) c->bits_left) \
        refill(c, n);                           \
    const uint64_t state = c->state;            \
    c->bits_left -= n;                          \
    c->state = state << n;                      \
    return (type) ((type64) state >> (64 - n)); \
}

GET_BITS(dav2d_get_bits,  unsigned, uint64_t)
GET_BITS(dav2d_get_sbits, int,      int64_t)

unsigned dav2d_get_uleb128(GetBits *const c) {
    uint64_t val = 0;
    unsigned i = 0, more;

    do {
        const int v = dav2d_get_bits(c, 8);
        more = v & 0x80;
        val |= ((uint64_t) (v & 0x7F)) << i;
        i += 7;
    } while (more && i < 56);

    if (val > UINT32_MAX || more) {
        c->error = 1;
        return 0;
    }

    return (unsigned) val;
}

unsigned dav2d_get_uniform(GetBits *const c, const unsigned max) {
    // Output in range [0..max-1]
    // max must be > 1, or else nothing is read from the bitstream
    assert(max > 1);
    const int l = ulog2(max) + 1;
    assert(l > 1);
    const unsigned m = (1U << l) - max;
    const unsigned v = dav2d_get_bits(c, l - 1);
    return v < m ? v : (v << 1) - m + dav2d_get_bit(c);
}

unsigned dav2d_get_vlc(GetBits *const c) {
    if (dav2d_get_bit(c))
        return 0;

    int n_bits = 0;
    do {
        if (++n_bits == 32)
            return UINT32_MAX;
    } while (!dav2d_get_bit(c));

    return ((1U << n_bits) - 1) + dav2d_get_bits(c, n_bits);
}

unsigned dav2d_get_bits_subexp_u(GetBits *const c, const unsigned ref,
                                 const unsigned n, const int k)
{
    unsigned v = 0;

    for (int i = 0;; i++) {
        const int b = i ? k + i - 1 : k;
        const int a = 1 << b;

        if (n <= v + 3 * a) {
            v += dav2d_get_uniform(c, n - v);
            break;
        }

        if (!dav2d_get_bit(c)) {
            v += dav2d_get_bits(c, b);
            break;
        }

        v += a;
    }

    return ref * 2 <= n ? inv_recenter(ref, v) :
                          n - 1 - inv_recenter(n - 1 - ref, v);
}

int dav2d_get_bits_subexp(GetBits *const c, const int ref, const unsigned n) {
    const int off = n - 1, n2 = n + off;
    return (int) dav2d_get_bits_subexp_u(c, ref + off, n2, 3) - off;
}
