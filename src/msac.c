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

#define EC_PROB_SHIFT 7

#define EC_WIN_SIZE (sizeof(ec_win) << 3)

static inline void ctx_refill(MsacContext *const s) {
    const uint8_t *buf_pos = s->buf_pos;
    const uint8_t *buf_end = s->buf_end;
    int c = EC_WIN_SIZE - s->cnt - 24;
    ec_win dif = s->dif;
    do {
        if (buf_pos >= buf_end) {
            // set remaining bits to 1;
            dif |= ~(~(ec_win)0xff << c);
            break;
        }
        dif |= (ec_win)(*buf_pos++ ^ 0xff) << c;
        c -= 8;
    } while (c >= 0);
    s->dif = dif;
    s->cnt = EC_WIN_SIZE - c - 24;
    s->buf_pos = buf_pos;
}

int dav1d_msac_decode_subexp(MsacContext *const s, const int ref,
                             const int n, unsigned k)
{
    int v = 0;
    for (int i = 0, b = k, a = 1 << k;; v += a, b += !!i, a <<= !!i, i++) {
        if (n <= v * 3 + a) {
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

#if !(HAVE_ASM && TRIM_DSP_FUNCTIONS && ( \
  ARCH_AARCH64 || \
  (ARCH_ARM && (defined(__ARM_NEON) || defined(__APPLE__) || defined(_WIN32))) \
))
static inline void ctx_norm_bypass(MsacContext *const s, ec_win dif,
                                   const unsigned n_bits)
{
    s->cnt -= n_bits;
    s->dif = dif << n_bits;
    if (s->cnt < 8) ctx_refill(s);
}

unsigned dav1d_msac_decode_bools_bypass_c(MsacContext *const s,
                                          const unsigned n_bits)
{
    const unsigned r = s->rng;
    ec_win dif = s->dif;
    assert((dif >> (EC_WIN_SIZE - 16)) < r);
    ec_win vw = (ec_win) r << (EC_WIN_SIZE - 16);
    unsigned ret = 0;
    for (unsigned n = 0; n < n_bits; n++) {
        vw >>= 1;
        ret <<= 1;
        if (dif >= vw) {
            dif -= vw;
        } else {
            ret |= 1;
        }
    }
    ctx_norm_bypass(s, dif, n_bits);
    return ret;
}

unsigned dav1d_msac_decode_unary_bypass_c(MsacContext *const s,
                                          const int max_bits)
{
    assert(max_bits > 0 && max_bits <= 32);
    if (s->cnt < max_bits - 1) ctx_refill(s);
    const unsigned r = s->rng;
    ec_win dif = s->dif;
    assert((dif >> (EC_WIN_SIZE - 16)) < r);
    ec_win vw = (ec_win) r << (EC_WIN_SIZE - 16);
    int ret = 0, bit;
    for (bit = 0; bit < max_bits; bit++) {
        vw >>= 1;
        if (dif >= vw) {
            dif -= vw;
            ret++;
        } else {
            bit++;
            break;
        }
    }
    ctx_norm_bypass(s, dif, bit);
    return ret;
}

/* Takes updated dif and range values, renormalizes them so that
 * 32768 <= rng < 65536 (reading more bytes from the stream into dif if
 * necessary), and stores them back in the decoder context.
 * dif: The new value of dif.
 * rng: The new value of the range. */
static inline void ctx_norm(MsacContext *const s, const ec_win dif,
                            const unsigned rng)
{
    const int d = 15 ^ (31 ^ clz(rng));
    const int cnt = s->cnt;
    assert(rng <= 65535U);
    s->dif = dif << d;
    s->rng = rng << d;
    s->cnt = cnt - d;
    // unsigned compare avoids redundant refills at eob
    if ((unsigned)cnt < (unsigned)d)
        ctx_refill(s);
}

/* Decode a single binary value.
 * f: The probability that the bit is one
 * Return: The value decoded (0 or 1). */
static unsigned dav1d_msac_decode_bool_c(MsacContext *const s, const unsigned f) {
    const unsigned r = s->rng;
    ec_win dif = s->dif;
    assert((dif >> (EC_WIN_SIZE - 16)) < r);
    const int p = ((f >> EC_PROB_SHIFT) << 4) + 8;
    unsigned v = ((r >> 8) * p >> (14 - EC_PROB_SHIFT)) << 3;
    const ec_win vw = (ec_win)v << (EC_WIN_SIZE - 16);
    const unsigned ret = dif >= vw;
    dif -= ret * vw;
    v += ret * (r - 2 * v);
    ctx_norm(s, dif, v);
    return !ret;
}

static const int8_t para_adjustment_list[][3] = {
    { 0, 0, 0 },    { 0, 0, -1 },   { 0, 0, -2 },   { 0, 0, 1 },
    { 0, 0, 1 },    { 0, -1, 0 },   { 0, -1, -1 },  { 0, -1, -2 },
    { 0, -1, 1 },   { 0, -1, 1 },   { 0, -2, 0 },   { 0, -2, -1 },
    { 0, -2, -2 },  { 0, -2, 1 },   { 0, -2, 1 },   { 0, 1, 0 },
    { 0, 1, -1 },   { 0, 1, -2 },   { 0, 1, 1 },    { 0, 1, 1 },
    { 0, 1, 0 },    { 0, 1, -1 },   { 0, 1, -2 },   { 0, 1, 1 },
    { 0, 1, 1 },    { -1, 0, 0 },   { -1, 0, -1 },  { -1, 0, -2 },
    { -1, 0, 1 },   { -1, 0, 1 },   { -1, -1, 0 },  { -1, -1, -1 },
    { -1, -1, -2 }, { -1, -1, 1 },  { -1, -1, 1 },  { -1, -2, 0 },
    { -1, -2, -1 }, { -1, -2, -2 }, { -1, -2, 1 },  { -1, -2, 1 },
    { -1, 1, 0 },   { -1, 1, -1 },  { -1, 1, -2 },  { -1, 1, 1 },
    { -1, 1, 1 },   { -1, 1, 0 },   { -1, 1, -1 },  { -1, 1, -2 },
    { -1, 1, 1 },   { -1, 1, 1 },   { -2, 0, 0 },   { -2, 0, -1 },
    { -2, 0, -2 },  { -2, 0, 1 },   { -2, 0, 1 },   { -2, -1, 0 },
    { -2, -1, -1 }, { -2, -1, -2 }, { -2, -1, 1 },  { -2, -1, 1 },
    { -2, -2, 0 },  { -2, -2, -1 }, { -2, -2, -2 }, { -2, -2, 1 },
    { -2, -2, 1 },  { -2, 1, 0 },   { -2, 1, -1 },  { -2, 1, -2 },
    { -2, 1, 1 },   { -2, 1, 1 },   { -2, 1, 0 },   { -2, 1, -1 },
    { -2, 1, -2 },  { -2, 1, 1 },   { -2, 1, 1 },   { 1, 0, 0 },
    { 1, 0, -1 },   { 1, 0, -2 },   { 1, 0, 1 },    { 1, 0, 1 },
    { 1, -1, 0 },   { 1, -1, -1 },  { 1, -1, -2 },  { 1, -1, 1 },
    { 1, -1, 1 },   { 1, -2, 0 },   { 1, -2, -1 },  { 1, -2, -2 },
    { 1, -2, 1 },   { 1, -2, 1 },   { 1, 1, 0 },    { 1, 1, -1 },
    { 1, 1, -2 },   { 1, 1, 1 },    { 1, 1, 1 },    { 1, 1, 0 },
    { 1, 1, -1 },   { 1, 1, -2 },   { 1, 1, 1 },    { 1, 1, 1 },
    { 1, 0, 0 },    { 1, 0, -1 },   { 1, 0, -2 },   { 1, 0, 1 },
    { 1, 0, 1 },    { 1, -1, 0 },   { 1, -1, -1 },  { 1, -1, -2 },
    { 1, -1, 1 },   { 1, -1, 1 },   { 1, -2, 0 },   { 1, -2, -1 },
    { 1, -2, -2 },  { 1, -2, 1 },   { 1, -2, 1 },   { 1, 1, 0 },
    { 1, 1, -1 },   { 1, 1, -2 },   { 1, 1, 1 },    { 1, 1, 1 },
    { 1, 1, 0 },    { 1, 1, -1 },   { 1, 1, -2 },   { 1, 1, 1 },
    { 1, 1, 1 },
};

static const int8_t av1_prob_inc_tbl[15][16] = {
    { 8, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 10, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 12, 8, 4, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 12, 9, 6, 3, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 13, 10, 8, 5, 2, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 13, 11, 9, 6, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 14, 12, 10, 8, 6, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1, -1 },
    { 14, 12, 10, 8, 7, 5, 3, 1, 0, -1, -1, -1, -1, -1, -1, -1 },
    { 14, 12, 11, 9, 8, 6, 4, 3, 1, 0, -1, -1, -1, -1, -1, -1 },
    { 14, 13, 11, 10, 8, 7, 5, 4, 2, 1, 0, -1, -1, -1, -1, -1 },
    { 14, 13, 12, 10, 9, 8, 6, 5, 4, 2, 1, 0, -1, -1, -1, -1 },
    { 14, 13, 12, 11, 9, 8, 7, 6, 4, 3, 2, 1, 0, -1, -1, -1 },
    { 14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0, -1, -1 },
    { 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, -1 },
    { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 }
};

/* Decodes a symbol given an inverse cumulative distribution function (CDF)
 * table in Q15. */
unsigned dav1d_msac_decode_symbol_adapt_c(MsacContext *const s,
                                          uint16_t *const cdf,
                                          const size_t n_symbols)
{
    const unsigned c = s->dif >> (EC_WIN_SIZE - 16), r = s->rng >> 8;
    unsigned u, v = s->rng, val = -1;
    const int8_t *const inc_tbl = av1_prob_inc_tbl[n_symbols - 1];

    assert(n_symbols <= 15);
    assert(cdf[n_symbols] <= 32);

    do {
        val++;
        u = v;
        const int p = ((cdf[val] >> EC_PROB_SHIFT) << 4) + inc_tbl[val];
        v = (r * p >> (14 - EC_PROB_SHIFT)) << 3;
    } while (c < v);

    assert(u <= s->rng);

    ctx_norm(s, s->dif - ((ec_win)v << (EC_WIN_SIZE - 16)), u - v);

    if (s->allow_update_cdf) {
        const unsigned count = cdf[n_symbols];
        const unsigned time_int = count >> 4;
        const unsigned rate = 4 + time_int + (n_symbols > 2) +
                              para_adjustment_list[cdf[n_symbols + 1]][time_int];
        unsigned i;
        for (i = 0; i < val; i++)
            cdf[i] += (32768 - cdf[i]) >> rate;
        for (; i < n_symbols; i++)
            cdf[i] -= cdf[i] >> rate;
        cdf[n_symbols] = count + (count < 32);
    }

    return val;
}

unsigned dav1d_msac_decode_bool_adapt_c(MsacContext *const s,
                                        uint16_t *const cdf)
{
    const unsigned bit = dav1d_msac_decode_bool_c(s, *cdf);

    if (s->allow_update_cdf) {
        // update_cdf() specialized for boolean CDFs
        const unsigned count = cdf[1];
        const unsigned time_int = count >> 4;
        const unsigned rate = 4 + time_int +
                              para_adjustment_list[cdf[2]][time_int];
        if (bit)
            cdf[0] += (32768 - cdf[0]) >> rate;
        else
            cdf[0] -= cdf[0] >> rate;
        cdf[1] = count + (count < 32);
    }

    return bit;
}
#endif

void dav1d_msac_init(MsacContext *const s, const uint8_t *const data,
                     const size_t sz, const int disable_cdf_update_flag)
{
    s->buf_pos = data;
    s->buf_end = data + sz;
    s->dif = 0;
    s->rng = 0x8000;
    s->cnt = -15;
    s->allow_update_cdf = !disable_cdf_update_flag;
    ctx_refill(s);

#if ARCH_X86_64 && HAVE_ASM
    s->symbol_adapt16 = dav1d_msac_decode_symbol_adapt_c;

    msac_init_x86(s);
#endif
}
