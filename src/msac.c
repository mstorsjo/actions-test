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

#include "config.h"

#include <limits.h>

#include "common/intops.h"

#include "src/msac.h"

const uint8_t dav1d_msac_rate[125][3] = {
    { 4, 5, 6 }, { 4, 5, 5 }, { 4, 5, 4 }, { 4, 5, 7 }, { 4, 5, 7 },
    { 4, 4, 6 }, { 4, 4, 5 }, { 4, 4, 4 }, { 4, 4, 7 }, { 4, 4, 7 },
    { 4, 3, 6 }, { 4, 3, 5 }, { 4, 3, 4 }, { 4, 3, 7 }, { 4, 3, 7 },
    { 4, 6, 6 }, { 4, 6, 5 }, { 4, 6, 4 }, { 4, 6, 7 }, { 4, 6, 7 },
    { 4, 6, 6 }, { 4, 6, 5 }, { 4, 6, 4 }, { 4, 6, 7 }, { 4, 6, 7 },
    { 3, 5, 6 }, { 3, 5, 5 }, { 3, 5, 4 }, { 3, 5, 7 }, { 3, 5, 7 },
    { 3, 4, 6 }, { 3, 4, 5 }, { 3, 4, 4 }, { 3, 4, 7 }, { 3, 4, 7 },
    { 3, 3, 6 }, { 3, 3, 5 }, { 3, 3, 4 }, { 3, 3, 7 }, { 3, 3, 7 },
    { 3, 6, 6 }, { 3, 6, 5 }, { 3, 6, 4 }, { 3, 6, 7 }, { 3, 6, 7 },
    { 3, 6, 6 }, { 3, 6, 5 }, { 3, 6, 4 }, { 3, 6, 7 }, { 3, 6, 7 },
    { 2, 5, 6 }, { 2, 5, 5 }, { 2, 5, 4 }, { 2, 5, 7 }, { 2, 5, 7 },
    { 2, 4, 6 }, { 2, 4, 5 }, { 2, 4, 4 }, { 2, 4, 7 }, { 2, 4, 7 },
    { 2, 3, 6 }, { 2, 3, 5 }, { 2, 3, 4 }, { 2, 3, 7 }, { 2, 3, 7 },
    { 2, 6, 6 }, { 2, 6, 5 }, { 2, 6, 4 }, { 2, 6, 7 }, { 2, 6, 7 },
    { 2, 6, 6 }, { 2, 6, 5 }, { 2, 6, 4 }, { 2, 6, 7 }, { 2, 6, 7 },
    { 5, 5, 6 }, { 5, 5, 5 }, { 5, 5, 4 }, { 5, 5, 7 }, { 5, 5, 7 },
    { 5, 4, 6 }, { 5, 4, 5 }, { 5, 4, 4 }, { 5, 4, 7 }, { 5, 4, 7 },
    { 5, 3, 6 }, { 5, 3, 5 }, { 5, 3, 4 }, { 5, 3, 7 }, { 5, 3, 7 },
    { 5, 6, 6 }, { 5, 6, 5 }, { 5, 6, 4 }, { 5, 6, 7 }, { 5, 6, 7 },
    { 5, 6, 6 }, { 5, 6, 5 }, { 5, 6, 4 }, { 5, 6, 7 }, { 5, 6, 7 },
    { 5, 5, 6 }, { 5, 5, 5 }, { 5, 5, 4 }, { 5, 5, 7 }, { 5, 5, 7 },
    { 5, 4, 6 }, { 5, 4, 5 }, { 5, 4, 4 }, { 5, 4, 7 }, { 5, 4, 7 },
    { 5, 3, 6 }, { 5, 3, 5 }, { 5, 3, 4 }, { 5, 3, 7 }, { 5, 3, 7 },
    { 5, 6, 6 }, { 5, 6, 5 }, { 5, 6, 4 }, { 5, 6, 7 }, { 5, 6, 7 },
    { 5, 6, 6 }, { 5, 6, 5 }, { 5, 6, 4 }, { 5, 6, 7 }, { 5, 6, 7 },
};

const uint16_t ALIGN(dav1d_msac_min_prob[7][8], 16) = {
    {    63, 65535, 65535, 65535, 65535, 65535, 65535, 65535 },
    {    47,    87, 65535, 65535, 65535, 65535, 65535, 65535 },
    {    31,    63,    95, 65535, 65535, 65535, 65535, 65535 },
    {    31,    55,    79,   103, 65535, 65535, 65535, 65535 },
    {    23,    47,    63,    87,   111, 65535, 65535, 65535 },
    {    23,    39,    55,    79,    95,   111, 65535, 65535 },
    {    15,    31,    47,    63,    79,    95,   111, 65535 },
};

static inline void ctx_refill(MsacContext *const s) {
    const uint8_t *buf_pos = s->buf_pos;
    const uint8_t *buf_end = s->buf_end;
    int c = 40 - s->cnt;
    uint64_t dif = s->dif;
    do {
        if (buf_pos >= buf_end) break;
        dif ^= (uint64_t)*buf_pos++ << c;
        c -= 8;
    } while (c >= 0);
    s->dif = dif;
    s->cnt = 40 - c;
    s->buf_pos = buf_pos;
}

int dav1d_msac_decode_subexp(MsacContext *const s, const int ref,
                             const int n, unsigned k)
{
    int v = 0;
    for (int i = 0, b = k, a = 1 << k;; v += a, b += !!i, a <<= !!i, i++) {
        if (n <= v + a * 3) {
            v += dav1d_msac_decode_uniform(s, n - v);
            break;
        } else if (!dav1d_msac_decode_bool_bypass(s)) {
            v += dav1d_msac_decode_bools_bypass(s, b);
            break;
        }
    }
    return ref * 2 <= n ? inv_recenter(ref, v) :
                          n - 1 - inv_recenter(n - 1 - ref, v);
}

int dav1d_msac_decode_4way(MsacContext *const s, const int ref,
                           uint16_t *const cdf, int n_bits)
{
    assert(n_bits >= 3);
    const int bin = dav1d_msac_decode_symbol_adapt4(s, cdf, 3);
    const int rem = dav1d_msac_decode_bools_bypass(s, n_bits + bin + !bin - 4);
    const int v = (bin ? (1 << (n_bits + bin - 4)) : 0) + rem;
    const int n = 1 << n_bits;
    return ref * 2 <= n ? inv_recenter(ref, v) :
                          n - 1 - inv_recenter(n - 1 - ref, v);
}

unsigned dav1d_msac_decode_bools_bypass_c(MsacContext *const s,
                                          const unsigned n_bits)
{
    assert(n_bits > 0 && n_bits <= 32);
    if ((unsigned)s->cnt < n_bits)
        ctx_refill(s);

    const uint64_t r = s->rng;
    uint64_t dif = s->dif;
    assert((dif >> 48) < r);
    uint64_t vw = r << 47;
    unsigned ret = 0;
    for (unsigned n = 0; n < n_bits; n++) {
        ret <<= 1;
        if (dif >= vw)
            dif -= vw;
        else
            ret |= 1;
        vw >>= 1;
    }
    s->dif = ((dif + 1) << n_bits) - 1;
    s->cnt -= n_bits;
    return ret;
}

unsigned dav1d_msac_decode_unary_bypass_c(MsacContext *const s,
                                          const unsigned max_bits)
{
    assert(max_bits > 0 && max_bits <= 32);
    if ((unsigned)s->cnt < max_bits)
        ctx_refill(s);

    const uint64_t r = s->rng;
    uint64_t dif = s->dif;
    assert((dif >> 48) < r);
    uint64_t vw = r << 47;
    unsigned ret = 0, bit;
    for (bit = 0; bit < max_bits; bit++) {
        if (dif >= vw) {
            dif -= vw;
            vw >>= 1;
            ret++;
        } else {
            bit++;
            break;
        }
    }
    s->dif = ((dif + 1) << bit) - 1;
    s->cnt -= bit;
    return ret;
}

/* Takes updated dif and range values, renormalizes them so that
 * 32768 <= rng < 65536 (reading more bytes from the stream into dif if
 * necessary), and stores them back in the decoder context.
 * dif: The new value of dif.
 * rng: The new value of the range. */
static inline void ctx_norm(MsacContext *const s, const uint64_t dif,
                            const unsigned rng)
{
    const unsigned d = 15 ^ (31 ^ clz(rng));
    const unsigned cnt = s->cnt;
    assert(rng <= 65535U);
    s->dif = ((dif + 1) << d) - 1;
    s->rng = rng << d;
    s->cnt = cnt - d;
    if (cnt < d) // unsigned compare avoids redundant refills at eob
        ctx_refill(s);
}

/* Decode a single binary value.
 * f: The probability that the bit is one
 * Return: The value decoded (0 or 1). */
static unsigned dav1d_msac_decode_bool_c(MsacContext *const s, const unsigned f) {
    const unsigned r = s->rng;
    uint64_t dif = s->dif;
    assert((dif >> 48) < r);
    const unsigned p = ((f >> 7) << 4) + 8;
    unsigned v = ((r >> 8) * p >> 7) << 3;
    const uint64_t vw = (uint64_t)v << 48;
    const unsigned ret = dif >= vw;
    dif -= ret * vw;
    v += ret * (r - 2 * v);
    ctx_norm(s, dif, v);
    return !ret;
}

/* Decodes a symbol given an inverse cumulative distribution function (CDF)
 * table in Q15. */
unsigned dav1d_msac_decode_symbol_adapt_c(MsacContext *const s,
                                          uint16_t *const cdf,
                                          const size_t n_symbols)
{
    const unsigned c = s->dif >> 48, r = s->rng >> 8;
    unsigned u, v = s->rng, val = -1;
    const uint16_t *const min_prob = dav1d_msac_min_prob[n_symbols - 1];

    assert(n_symbols <= 7);

    do {
        val++;
        u = v;
        const unsigned p = imax((cdf[val] | 127) - min_prob[val], 0);
        v = (r * p >> 10) << 3;
    } while (c < v);

    assert(u <= s->rng);

    ctx_norm(s, s->dif - ((uint64_t)v << 48), u - v);

    if (s->allow_update_cdf) {
        const unsigned pc = cdf[n_symbols];
        const unsigned count = (uint8_t)pc;
        assert(count <= 32);
        const int rate = dav1d_msac_rate[pc >> 8][count >> 4] + (n_symbols > 2);
        unsigned i;
        for (i = 0; i < val; i++)
            cdf[i] += (32768 - cdf[i]) >> rate;
        for (; i < n_symbols; i++)
            cdf[i] -= cdf[i] >> rate;
        cdf[n_symbols] = pc + (count < 32);
    }

    return val;
}

unsigned dav1d_msac_decode_bool_adapt_c(MsacContext *const s,
                                        uint16_t *const cdf)
{
    const unsigned bit = dav1d_msac_decode_bool_c(s, *cdf);

    if (s->allow_update_cdf) {
        // update_cdf() specialized for boolean CDFs
        const unsigned pc = cdf[1];
        const unsigned count = (uint8_t)pc;
        const int rate = dav1d_msac_rate[pc >> 8][count >> 4];
        if (bit)
            cdf[0] += (32768 - cdf[0]) >> rate;
        else
            cdf[0] -= cdf[0] >> rate;
        cdf[1] = pc + (count < 32);
    }

    return bit;
}

void dav1d_msac_init(MsacContext *const s, const uint8_t *const data,
                     const size_t sz, const int disable_cdf_update_flag)
{
    s->buf_pos = data;
    s->buf_end = data + sz;
    s->dif = (~(uint64_t)0) >> 1;
    s->rng = 0x8000;
    s->cnt = -15;
    s->allow_update_cdf = !disable_cdf_update_flag;
    ctx_refill(s);
}
