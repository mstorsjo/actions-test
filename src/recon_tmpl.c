/*
 * Copyright © 2018-2021, VideoLAN and dav1d authors
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

#include <inttypes.h>
#include <string.h>
#include <stdio.h>

#include "common/attributes.h"
#include "common/bitdepth.h"
#include "common/dump.h"
#include "common/frame.h"
#include "common/intops.h"

#include "src/cdef_apply.h"
#include "src/ctx.h"
#include "src/ipred_prepare.h"
#include "src/itx_1d.h"
#include "src/lf_apply.h"
#include "src/lr_apply.h"
#include "src/recon.h"
#include "src/scan.h"
#include "src/stx_tables.h"
#include "src/tables.h"
#include "src/warpmv.h"
#include "src/wedge.h"

static inline unsigned decode_exp_golomb(MsacContext *const s, const int k) {
    const int length = dav1d_msac_decode_unary_bypass21(s) + k;
    const int x = (1 << length) + dav1d_msac_decode_bools_bypass(s, length);
    return x - (1 << k);
}

static inline int decode_hr(MsacContext *const s, const int hr_avg) {
    const int m = ulog2(iclip(hr_avg, 2, 64)); // 1..6
    const int cmax = imin(m + 4, 6); // 5 or 6
    const int q = dav1d_msac_decode_unary_bypass6(s, cmax);
    const int rem = (q == cmax) ? decode_exp_golomb(s, m + 1) :
                                  dav1d_msac_decode_bools_bypass(s, m);
    return rem + (q << m);
}


static inline unsigned get_skip_ctx(const TxfmInfo *const t_dim,
                                    const enum BlockSize bs,
                                    const uint8_t *const a,
                                    const uint8_t *const l,
                                    const int plane,
                                    const enum Dav1dPixelLayout layout)
{
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];

    if (plane) {
        const int ss_ver = layout == DAV1D_PIXEL_LAYOUT_I420;
        const int ss_hor = layout != DAV1D_PIXEL_LAYOUT_I444;
        const int not_one_blk = b_dim[2] - (!!b_dim[2] && ss_hor) > t_dim->lw ||
                                b_dim[3] - (!!b_dim[3] && ss_ver) > t_dim->lh;
        unsigned ca, cl;

#define MERGE_CTX(dir, type, no_val) \
        c##dir = ((union alias##type*)dir)->u##type != no_val; \
        break

        switch (t_dim->lw) {
        /* For some reason the MSVC CRT _wassert() function is not flagged as
         * __declspec(noreturn), so when using those headers the compiler will
         * expect execution to continue after an assertion has been triggered
         * and will therefore complain about the use of uninitialized variables
         * when compiled in debug mode if we put the default case at the end. */
        default: assert(0); /* fall-through */
        case TX_4X4:   MERGE_CTX(a,  8, 0x40);
        case TX_8X8:   MERGE_CTX(a, 16, 0x4040);
        case TX_16X16: MERGE_CTX(a, 32, 0x40404040U);
        case TX_32X32: MERGE_CTX(a, 64, 0x4040404040404040ULL);
        case TX_64X64: ca = (*(const uint64_t *) a |
                             *(const uint64_t *) &a[8]) != 0x4040404040404040ULL;
        }
        switch (t_dim->lh) {
        default: assert(0); /* fall-through */
        case TX_4X4:   MERGE_CTX(l,  8, 0x40);
        case TX_8X8:   MERGE_CTX(l, 16, 0x4040);
        case TX_16X16: MERGE_CTX(l, 32, 0x40404040U);
        case TX_32X32: MERGE_CTX(l, 64, 0x4040404040404040ULL);
        case TX_64X64: cl = (((union alias64*)l)->u64 |
                             ((union alias64*)&l[8])->u64) != 0x4040404040404040ULL;
        }
#undef MERGE_CTX

        // we assume here that the ccoef array is [2][64], and for V, ca is
        // ccoef[1][x], and U has been decoded before. Therefore, we can
        // go "up" 64 bytes in ca/cl and get the "skip" state of the U plane.
        const int offset = plane == 1 ? 7 :
            6 * (((uint8_t(*)[64]) a)[-1][0] != 0x40) + not_one_blk * 3;
        return offset + ca + cl;
    } else if (b_dim[2] == t_dim->lw && b_dim[3] == t_dim->lh) {
        return 0;
    } else {
        unsigned la, ll;

#define MERGE_CTX(dir, type, tx) \
        if (tx == TX_64X64) { \
            uint64_t tmp = ((union alias64*)dir)->u64; \
            tmp |= ((union alias64*)&dir[8])->u64; \
            l##dir = (unsigned) (tmp >> 32) | (unsigned) tmp; \
        } else \
            l##dir = ((union alias##type*)dir)->u##type; \
        if (tx == TX_32X32) l##dir |= ((union alias32*)&dir[4])->u32; \
        if (tx >= TX_16X16) l##dir |= l##dir >> 16; \
        if (tx >= TX_8X8)   l##dir |= l##dir >> 8; \
        break

        switch (t_dim->lw) {
        default: assert(0); /* fall-through */
        case TX_4X4:   MERGE_CTX(a,  8, TX_4X4);
        case TX_8X8:   MERGE_CTX(a, 16, TX_8X8);
        case TX_16X16: MERGE_CTX(a, 32, TX_16X16);
        case TX_32X32: MERGE_CTX(a, 32, TX_32X32);
        case TX_64X64: MERGE_CTX(a, 32, TX_64X64);
        }
        switch (t_dim->lh) {
        default: assert(0); /* fall-through */
        case TX_4X4:   MERGE_CTX(l,  8, TX_4X4);
        case TX_8X8:   MERGE_CTX(l, 16, TX_8X8);
        case TX_16X16: MERGE_CTX(l, 32, TX_16X16);
        case TX_32X32: MERGE_CTX(l, 32, TX_32X32);
        case TX_64X64: MERGE_CTX(l, 32, TX_64X64);
        }
#undef MERGE_CTX

        return (umin(la & 0x3F, 4) + umin(ll & 0x3F, 4) + 3) >> 1;
    }
}

static inline unsigned get_dc_sign_ctx(const TxfmInfo *const t_dim,
                                       const uint8_t *const a,
                                       const uint8_t *const l)
{
    uint64_t mask = 0xC0C0C0C0C0C0C0C0ULL, mul = 0x0101010101010101ULL;

#if ARCH_X86_64 && defined(__GNUC__)
    /* Coerce compilers into producing better code. For some reason
     * every x86-64 compiler is awful at handling 64-bit constants. */
    __asm__("" : "+r"(mask), "+r"(mul));
#endif

    uint64_t t = 0;
    const uint8_t *edge = a;
    for (int dir = 0, len = t_dim->lw; dir < 2; dir++, edge = l, len = t_dim->lh) {
        switch(len) {
        default: assert(0); /* fall-through */
        case TX_4X4:
            t += *edge >> 6;
            break;
        case TX_8X8:
            t += (((union alias16*)edge)->u16 & (uint32_t) mask) >> 6;
            break;
        case TX_16X16:
            t += (((union alias32*)edge)->u32 & (uint32_t) mask) >> 6;
            break;
        case TX_64X64:
            t += (((union alias64*)&edge[8])->u64 & mask) >> 6;
            // fall-through
        case TX_32X32:
            t += (((union alias64*)edge)->u64 & mask) >> 6;
            break;
        }
    }

    t *= mul;
    const int s = (int) (t >> 56) - t_dim->w - t_dim->h;
    return (s != 0) + (s > 0);
}

static inline unsigned get_lo_ctx(const int8_t *const levels,
                                  const enum TxClass tx_class,
                                  unsigned *const hi_mag_ptr,
                                  const unsigned xy, const int plane,
                                  const ptrdiff_t stride)
{
    const int chroma = !!plane;
#define add(v) do { \
    const unsigned val = v; \
    lo_mag += imin(val, lim); \
    hi_mag += imin(val, 5); \
} while (0)
    unsigned lo_freq = xy < (chroma ? 1 : tx_class == TX_CLASS_2D ? 4 : 2);
    unsigned lim = lo_freq ? 5 : 3;
    unsigned lo_mag = 0, hi_mag = 0;
    add(levels[0 * stride + 1]);
    add(levels[1 * stride + 0]);
    // for the initial token:
    // br(l) = min(R, l) + min(B, l) - where l is 3 [hi-freq] or 5 [lo-freq]
    // brhvc(l) = (br(l) + 1) >> 1
    // br2dc(l) = (br(l) + min(RB, l) + 1) >> 1
    // br2dl(l) = (br(l) + min(RB, l) + min(B2, l) + min(R2, l) + 1) >> 1
    // brvl(l1, l2) = (br(l1) + min(B2, l2) + min(B3, l2) + min(B4, l2) + 1) >> 1
    // brhl(l1, l2) = (br(l1) + min(R2, l2) + min(R3, l2) + min(R4, l2) + 1) >> 1
    // luma, hf:        2d: x + y < 6: min(br2dl(3), 4) +  0
    //                      x + y < 8: min(br2dl(3), 4) +  5
    //                      else:      min(br2dl(3), 4) + 10
    //                  h:             min(brhl(3, 3), 4) + 15
    //                  v:             min(brvl(3, 3), 4) + 15
    // luma, lf:        2d: is_dc:     min(br2dl(5), 8) +  0
    //                      x + y < 2: min(br2dl(5), 6) +  9
    //                      else:      min(br2dl(5), 4) + 16
    //                  h:  x == 0:    min(brhl(5, 3), 6) + 21
    //                      else:      min(brhl(5, 3), 4) + 28
    //                  v:  y == 0:    min(brvl(5, 3), 6) + 21
    //                      else:      min(brvl(5, 3), 4) + 28
    // chroma, hf:      2d: min(br2dc(3), 3) + (plane == u ? 0 : 4)
    //                  hv: brhvc(3) + 8
    // chroma, lf:      2d: min(br2dc(5), 3) + (plane == u ? 0 : 4)
    //                  hv: min(brhvc(5), 3) + 8
    unsigned offset;
    if (tx_class == TX_CLASS_2D) {
        add(levels[1 * stride + 1]);
        if (!chroma) {
            lo_mag += imin(levels[0 * stride + 2], lim) +
                      imin(levels[2 * stride + 0], lim);
            if (lo_freq) {
                offset = !xy ? 0 : xy < 2 ? 9 : 16;
                lim    = !xy ? 8 : xy < 2 ? 6 :  4;
            } else {
                offset = xy < 6 ? 0 : xy < 8 ? 5 : 10;
                lim    = 4;
            }
        } else {
            lim = 3;
            offset = plane == 1 ? 0 : 4;
        }
    } else {
        if (!chroma) {
            lim = 3;
            add(levels[0 * stride + 2]);
            lo_mag += imin(levels[0 * stride + 3], 3) +
                      imin(levels[0 * stride + 4], 3);
            if (lo_freq) {
                offset = !xy ? 21 : 28;
                lim    = !xy ?  6 :  4;
            } else {
                offset = 15;
                lim = 4;
            }
        } else {
            offset = 8;
            lim = 3;
        }
    }
    // for the base_range component:
    // br   = min(R, 5) + min(B, 5)
    // br2d = (br + min(RB, 5) + 1) >> 1
    // brh  = (br + min(R2, 5) + 1) >> 1
    // brv  = (br + min(B2, 5) + 1) >> 1
    // luma, hf:        2d: min(br2d, 6)
    //                  h:  min(brh,  6)
    //                  v:  min(brv,  6)
    // luma, lf:        2d: min(br2d, 6) + (is_dc ? 0 : 7)
    //                  h:  min(brh,  6) + 7
    //                  v:  min(brv,  6) + 7
    // chroma, hf:      2d: min(br2d, 3)
    //                  hv: min((br + 1) >> 1, 3)
    // chroma, lf:      N/A
    *hi_mag_ptr = (!chroma && lo_freq & (xy > 0 || tx_class != TX_CLASS_2D) ?
                   7 : 0) + umin((hi_mag + 1) >> 1, chroma ? 3 : 6);
    return offset + umin((lo_mag + 1) >> 1, lim);
}

static inline unsigned get_lo_ctx_idtx(const int8_t *const levels,
                                       unsigned *const hi_mag_ptr,
                                       const ptrdiff_t stride)
{
#define lim 3
    unsigned lo_mag = 0, hi_mag = 0;
    add(levels[ 0 * stride - 1]);
    add(levels[-1 * stride + 0]);
#undef lim
#undef add
    *hi_mag_ptr = imin(hi_mag, 6);
    return lo_mag;
}

static inline unsigned get_sign_ctx_idtx(const int8_t *const levels,
                                         const ptrdiff_t stride)
{
    const int sum = levels[ 0 * stride - 1] +
                    levels[-1 * stride + 0] +
                    levels[-1 * stride - 1];
    const int offset = *levels > 3 ? 2 : 0;
    switch (sum) {
    case -3: return offset + 6;
    case -2:
    case -1: return offset + 2;
    default: assert(0);
    case 0: return 0;
    case 1:
    case 2: return offset + 1;
    case 3: return offset + 5;
    }
}

static inline int tcq_next_state(const int state, const int abs_level) {
    // bit0-1 are shifted in from bit1-2
    // bit2 is set based on bit0 ^ bit2 & (abs_level & 1)
    // bit31 is to always set it to 0 if tcq is disabled
    return (((state & 0x4) ^ (((abs_level & 1) ^ (state & 0x1)) << 2)) |
            ((state & 0x6) >> 1) | -0x80000000) & (state >> 31);
}


static int decode_coefs(Dav1dTaskContext *const t, DB_ONLY(const int depth)
                        uint8_t *const a, uint8_t *const l,
                        const enum RectTxfmSize tx, const enum BlockSize bs,
                        const Av1Block *const b, const int plane, coef *cf,
                        enum TxfmType *const txtp, uint8_t *res_ctx)
{
    Dav1dTileState *const ts = t->ts;
    const int chroma = !!plane; // FIXME perhaps make this an inlined function arg?
    const int intra = b->intra && !b->intrabc;
    const Dav1dFrameContext *const f = t->f;
    const int lossless = f->frame_hdr->segmentation.lossless[b->seg_id];
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
#if DEBUG_BLOCK_INFO
    const int dbg = BLOCK_TO_DEBUG && plane > -1 && 1;
#define DEBUG_CF_printf(fmt...) \
    if (dbg) printf(fmt)
#else
#define DEBUG_CF_printf(fmt...)
#endif

    DEBUG_CF_printf("%*sdecode_cf[y=%d,x=%d,pl=%d,tx=%dx%d]: r=%d\n",
                    depth - 1, "", t->by, t->bx, plane, t_dim->w * 4, t_dim->h * 4,
                    ts->msac.rng);

    // does this block have any non-zero coefficients
    const int sctx = (b->fsc && !chroma) ? 13 :
                     get_skip_ctx(t_dim, bs, a, l, plane, f->cur.p.layout);
    const int all_skip =
        dav1d_msac_decode_bool_adapt(&ts->msac,
            (plane == 2 ? ts->cdf.coef.skip_v :
                          ts->cdf.coef.skip[!intra || b->fsc][t_dim->ctx])[sctx]);
    DEBUG_CF_printf("%*sPost-all_zero[ctx=%d|%d|%d,%d]: r=%d\n",
                    depth, "", plane == 2 ? -1 : (!intra || b->fsc),
                    t_dim->ctx, sctx, all_skip, ts->msac.rng);
    if (all_skip) {
        *res_ctx = 0x40;
        *txtp = (!chroma && b->fsc) ? IDTX :
                lossless * WHT_WHT; /* lossless ? WHT_WHT : DCT_DCT */
        return -1;
    }

    // find end-of-block (eob)
    int eob;
    const int slw = imin(t_dim->lw, TX_32X32), slh = imin(t_dim->lh, TX_32X32);
    const int tx2dszctx = slw + slh;
    const int eob_ctx = chroma ? 2 : !intra;
    switch (tx2dszctx) {
#define case_sz(sz, bin, bits, eb) \
    case sz: { \
        uint16_t *const eob_bin_cdf = ts->cdf.coef.eob_bin_##bin[eob_ctx]; \
        eob = dav1d_msac_decode_symbol_adapt8(&ts->msac, eob_bin_cdf, bits); \
        if (eb && eob == 7) { \
            eob += dav1d_msac_decode_bools_bypass(&ts->msac, eb); \
            if (bin == 512 && eob == 10) return -1; /* FIXME set error bit */ \
        } \
        break; \
    }
    case_sz(0,   16, 4, 0);
    case_sz(1,   32, 5, 0);
    case_sz(2,   64, 6, 0);
    case_sz(3,  128, 7, 0);
    case_sz(4,  256, 7, 1);
    case_sz(5,  512, 7, 2);
    case_sz(6, 1024, 7, 2);
#undef case_sz
    }
    DEBUG_CF_printf("%*sPost-eob_bin_%d[ctx=%d,%d]: r=%d\n",
                    depth, "", 16 << tx2dszctx, eob_ctx, eob, ts->msac.rng);

    if (eob > 1) {
        const int eob_hi_bit = dav1d_msac_decode_bool_adapt(&ts->msac,
                                   ts->cdf.coef.eob_hi_bit);
        const int eob_bin = eob - 2;
        eob = eob_hi_bit | 2;
        if (eob_bin)
            eob = (eob << eob_bin) | dav1d_msac_decode_bools_bypass(&ts->msac, eob_bin);
        DEBUG_CF_printf("%*sPost-eob[%d]: r=%d\n", depth, "", eob, ts->msac.rng);
    }
    assert(eob >= 0 && eob < (16 << tx2dszctx));

    // transform type (chroma: derived, luma: explicitly coded)
    static const uint8_t txtp_long_tbl[2][2][4] = {
        {
            { V_DCT, V_ADST, V_FLIPADST, IDTX },
            { H_DCT, H_ADST, H_FLIPADST, IDTX },
        }, {
            { DCT_DCT, ADST_DCT, FLIPADST_DCT, H_DCT },
            { DCT_DCT, DCT_ADST, DCT_FLIPADST, V_DCT },
        },
    };
    if (lossless) {
        // FIXME this can be IDTX or WHT_WHT
        assert(t_dim->max == TX_4X4);
        *txtp = WHT_WHT;
    } else if (chroma) {
        // inferred from either the luma txtp (inter) or a LUT (intra)
        if (intra) *txtp = dav1d_txtp_from_uvmode[b->uv_mode];
        if ((t_dim->w >= 8 && *txtp & 0x02 /* horizontal is (flip)adst */) ||
            (t_dim->h >= 8 && *txtp & 0x40 /* vertical is (flip)adst */) ||
            (tx == (int) TX_16X16 &&
             ((*txtp & 0x47) == 0x41 /* (flip)adst ver, identity hor */ ||
              (*txtp & 0xe2) == 0x22 /* identity ver, (flip)adst hor */)))
        {
            *txtp = DCT_DCT;
        }
    } else if (intra) {
        if (t_dim->sub == TX_32X32 /* 64x64, 64x32 or 32x64 */) {
            *txtp = DCT_DCT;
        } else if (b->fsc) {
            *txtp = IDTX;
        } else if (!eob /* dc-only */ || tx == (enum RectTxfmSize)TX_32X32) {
            *txtp = DCT_DCT;
        } else if (t_dim->max >= TX_32X32 /* {64,32}x{16,8,4} */) {
            // long64/32
            const int long_dct = t_dim->max == TX_64X64 ||
                                 dav1d_msac_decode_bool_adapt(&ts->msac,
                                     ts->cdf.m.txtp_long32_dct[0]);
            const int short_idx = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                      ts->cdf.m.txtp_intra_short_1d[t_dim->min], 3);
            *txtp = txtp_long_tbl[long_dct][t_dim->w < t_dim->h][short_idx];
        } else if (f->frame_hdr->reduced_txtp_set == 2) {
            // ext_tx_set_dct_idtx
            *txtp = DCT_DCT;
        } else {
            // ext_new_tx_set
            const int sz_ctx = (t_dim->lw + t_dim->lh) >> 1;
            assert(sz_ctx < 3);
            const int tx_idx = f->frame_hdr->reduced_txtp_set ?
                dav1d_msac_decode_bool_adapt(&ts->msac,
                    ts->cdf.m.txtp_ext_reduced[t_dim->min]) :
                dav1d_msac_decode_symbol_adapt8(&ts->msac,
                    ts->cdf.m.txtp_ext[t_dim->min], 6);
            static const uint8_t /*enum TxfmType*/ av1_md_idx2type[][13][7] = {
                {
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, FLIPADST_ADST, H_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, V_DCT, V_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_ADST, H_DCT, H_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, FLIPADST_ADST, H_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, V_ADST, V_FLIPADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, FLIPADST_ADST, H_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_ADST, H_DCT, H_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, V_DCT, V_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, FLIPADST_ADST, V_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, FLIPADST_ADST, H_ADST },
                    { DCT_DCT, ADST_ADST, DCT_ADST, V_DCT,
                      H_DCT, V_ADST, H_ADST },
                }, {
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      ADST_FLIPADST, FLIPADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_ADST, FLIPADST_DCT, ADST_FLIPADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      DCT_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      DCT_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, FLIPADST_ADST, ADST_FLIPADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      DCT_FLIPADST, FLIPADST_FLIPADST, ADST_FLIPADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      DCT_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      V_DCT, H_DCT, H_ADST },
                }, {
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      DCT_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, FLIPADST_ADST, ADST_FLIPADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      DCT_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, FLIPADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      DCT_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      FLIPADST_DCT, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      DCT_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { DCT_DCT, ADST_ADST, ADST_DCT, DCT_ADST,
                      V_DCT, H_DCT, V_ADST },
                },
            };
            *txtp = av1_md_idx2type[sz_ctx][b->y_mode][tx_idx];
        }
    } else {
        if (t_dim->sub == TX_32X32 /* 64x64, 64x32 or 32x64 */) {
            *txtp = DCT_DCT;
        } else {
            const int y = eob >> (2 + slw), x = eob & ((4 << slw) - 1);
            const int xy = x + y;
            // transform dimensions are not truncated for tx64 (to tx32) here,
            // see AVM bug #943
            const int ctx = xy < 2 ? 1 : xy > 4 * (t_dim->w + t_dim->h) - 4 ? 2 : 0;
            if (tx == (enum RectTxfmSize)TX_32X32) {
                *txtp = dav1d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.txtp_inter_dct_idtx[ctx][TX_32X32]) ?
                        DCT_DCT : IDTX;
            } else if (t_dim->max >= TX_32X32 /* {64,32}x{16,8,4} */) {
                // long64/32
                const int long_dct = t_dim->max == TX_64X64 ||
                                     dav1d_msac_decode_bool_adapt(&ts->msac,
                                         ts->cdf.m.txtp_long32_dct[1]);
                const int short_idx = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                          ts->cdf.m.txtp_inter_short_1d[ctx]
                                                                [t_dim->min], 3);
                *txtp = txtp_long_tbl[long_dct][t_dim->w < t_dim->h][short_idx];
            } else if (f->frame_hdr->reduced_txtp_set == 3) {
                // FIXME EXT_TX_SET_DCT_IDTX_IDDCT
                printf("FIXME\n");
            } else if (f->frame_hdr->reduced_txtp_set) {
                // FIXME EXT_TX_SET_DCT_IDTX
                printf("FIXME\n");
            } else {
                const int setidx = tx == (enum RectTxfmSize)TX_16X16;
                const int set = dav1d_msac_decode_bool_adapt(&ts->msac,
                                    ts->cdf.m.txtp_inter_tx_set[setidx][ctx]
                                                               [t_dim->min]);
                if (!set) {
                    *txtp = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                ts->cdf.m.txtp_inter_set0[setidx][ctx], 7);
                } else if (setidx) {
                    *txtp = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                ts->cdf.m.txtp_inter_set2[ctx], 3) + 8;
                } else {
                    *txtp = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                ts->cdf.m.txtp_inter_set1[ctx], 7) + 8;
                }
                static const uint8_t txtp_inv_tbl[][16] = {
                    { IDTX, V_DCT, H_DCT,
                      V_ADST, H_ADST, V_FLIPADST, H_FLIPADST,
                      DCT_DCT, ADST_DCT, DCT_ADST, FLIPADST_DCT, DCT_FLIPADST,
                      ADST_ADST, FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                    { IDTX, V_DCT, H_DCT,
                      DCT_DCT, ADST_DCT, DCT_ADST, FLIPADST_DCT, DCT_FLIPADST,
                      ADST_ADST, FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST },
                };
                *txtp = txtp_inv_tbl[setidx][*txtp];
            }
        }
    }
    DEBUG_CF_printf("%*sPost-txtp[%s/%s]: r=%d\n",
                    depth, "", dav1d_tx1d_names[*txtp & 7],
                    dav1d_tx1d_names[*txtp >> 5], ts->msac.rng);

    const enum TxClass tx_class = (*txtp >> 3) & 0x3;

    // secondary transform
    int stx_type = 0;
    if (f->seq_hdr->ist[!intra] && !chroma) {
        if (intra) {
            if (eob >= 1 && b->y_mode != PAETH_PRED &&
                (*txtp == DCT_DCT || *txtp == ADST_ADST))
            {
                int lim;
                if (tx == (enum RectTxfmSize)TX_8X8 && *txtp == DCT_DCT)
                    lim = 20;
                else if (t_dim->min >= TX_8X8)
                    lim = *txtp == DCT_DCT ? 32 : 20;
                else
                    lim = 8;
                stx_type = eob < lim;
            }
        } else {
            stx_type = t_dim->min >= TX_16X16 && *txtp == DCT_DCT &&
                       eob >= 3 && eob < 32;
        }
        if (stx_type) {
            stx_type = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                ts->cdf.m.stx[!intra][t_dim->min], 3);
            int stx_set = 0;
            if (stx_type && intra) {
                if (t_dim->min >= TX_8X8 && *txtp == ADST_ADST) {
                    // FIXME last 3 are unused
                    static const uint8_t inv_most_probable_stx_mapping_adst[][7] = {
                        { 6, 1, 0, 4, 5, 3, 2 },  // DC_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // V_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // H_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // D45_PRED
                        { 0, 4, 6, 1, 3, 2, 5 },  // D135_PRED
                        { 4, 1, 0, 6, 3, 5, 2 },  // D113_PRED
                        { 4, 1, 0, 6, 3, 5, 2 },  // D157_PRED
                        { 1, 0, 6, 4, 5, 2, 3 },  // D203_PRED
                        { 1, 0, 6, 4, 5, 2, 3 },  // D67_PRED
                        { 6, 1, 0, 4, 5, 3, 2 },  // SMOOTH_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // SMOOTH_V_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // SMOOTH_H_PRED
                    };
                    stx_set = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                  ts->cdf.m.stx_set_adst, 3);
                    stx_set = inv_most_probable_stx_mapping_adst[b->y_mode][stx_set];
                } else {
                    static const uint8_t inv_most_probable_stx_mapping[][7] = {
                        { 6, 1, 0, 5, 4, 3, 2 },  // DC_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // V_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // H_PRED
                        { 2, 6, 0, 5, 1, 4, 3 },  // D45_PRED
                        { 3, 4, 6, 1, 0, 2, 5 },  // D135_PRED
                        { 4, 1, 3, 6, 0, 5, 2 },  // D113_PRED
                        { 4, 1, 3, 6, 0, 5, 2 },  // D157_PRED
                        { 5, 0, 6, 2, 1, 4, 3 },  // D203_PRED
                        { 5, 0, 6, 2, 1, 4, 3 },  // D67_PRED
                        { 6, 1, 0, 5, 4, 3, 2 },  // SMOOTH_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // SMOOTH_V_PRED
                        { 1, 6, 0, 4, 2, 5, 3 },  // SMOOTH_H_PRED
                    };
                    stx_set = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                  ts->cdf.m.stx_set, 6);
                    stx_set = inv_most_probable_stx_mapping[b->y_mode][stx_set];
                }
                stx_set += 7 * (*txtp == ADST_ADST);
                *txtp |= stx_set << 10;
            }
            *txtp |= stx_type << 8;
            DEBUG_CF_printf("%*sPost-stx[type=%d,set=%d]: r=%d\n",
                            depth, "", stx_type, stx_set, ts->msac.rng);
        }
    } else if (f->seq_hdr->cctx && plane == 1 && eob >= intra &&
               (f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420 || t_dim->max < 8))
    {
        const int cctx = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                                         ts->cdf.m.cctx, 6);
        DEBUG_CF_printf("%*sPost-cctx[%d]: r=%d\n",
                        depth, "", cctx, ts->msac.rng);
    }

    // base tokens
    unsigned cul_level = 0;
    int dc_tok;
    const int tcq_enabled = !chroma && f->frame_hdr->tcq && tx_class == TX_CLASS_2D;
    int hr_avg = 0, tcq_state = tcq_enabled * -0x80000000;
    const uint8_t *const qm_tbl = *txtp < IDTX ? f->qm[tx][plane] : NULL;
    int dq_shift = tcq_enabled + 3 + imax(0, t_dim->ctx - 2);
    const uint32_t *const dq_tbl = ts->dq[b->seg_id][plane];
    const int cf_max = ~(~127U << (BITDEPTH == 8 ? 8 : f->cur.p.bpc));
    unsigned dc_sign_level = 1 << 6;

    if (f->seq_hdr->fsc && (!intra || b->fsc) &&
        *txtp == IDTX && !chroma)
    {
        assert(!stx_type);
        int8_t *const levels = t->scratch.levels;
        const ptrdiff_t stride = 1 + (4 << slh);
        memset(levels, 0, stride * ((4 << slw) + 1));
        const uint16_t *scan = dav1d_scans[tx];
        const int sz_ctx = imin(t_dim->ctx, 2);
        const int sz = (16 << tx2dszctx) - 1;
        const int bob = sz - eob;
        unsigned ctx = (bob > 2 << tx2dszctx) + (bob > 4 << tx2dszctx);
        uint16_t (*hi_cdf)[4] = ts->cdf.coef.br_y_tok_idtx[sz_ctx];
        int tok = 1 + dav1d_msac_decode_symbol_adapt4(&ts->msac,
                          ts->cdf.coef.bob_base_y_tok[sz_ctx][ctx], 2);
        if (tok == 3) {
            tok += dav1d_msac_decode_symbol_adapt4(&ts->msac, hi_cdf[0], 3);
        }
        const unsigned shift = slh + 2;
        const unsigned mask = (4 << slh) - 1;
        int rc = scan[bob];
        int x = rc >> shift, y = rc & mask;
        cf[rc] = levels[(1 + x) * stride + (y + 1)] = tok;
        DEBUG_CF_printf("%*sPost-bob_tok[pos=%d,ctx=%d|%d|%d,plane=%s,%d]: r=%d\n",
                        depth, "", bob, sz_ctx, ctx, tok < 3 ? -1 : 0,
                        chroma ? "uv" : "y", tok, ts->msac.rng);

        uint16_t (*lo_cdf)[4] = ts->cdf.coef.base_y_tok_idtx[sz_ctx];
        for (int i = bob + 1; i <= sz; i++) {
            rc = scan[i];
            x = rc >> shift;
            y = rc & mask;
            int8_t *const level = &levels[(1 + x) * stride + (1 + y)];
            unsigned hr_ctx;
            ctx = get_lo_ctx_idtx(level, &hr_ctx, stride);
            int tok = dav1d_msac_decode_symbol_adapt4(&ts->msac, lo_cdf[ctx], 3);
            if (tok == 3) {
                tok += dav1d_msac_decode_symbol_adapt4(&ts->msac, hi_cdf[hr_ctx], 3);
            }
            cf[rc] = *level = tok;
            DEBUG_CF_printf("%*sPost-tok[pos=%d,ctx=%d|%d|%d,plane=%s,%d]: r=%d\n",
                            depth, "", i, sz_ctx, ctx, tok < 3 ? -1 : hr_ctx,
                            chroma ? "uv" : "y", tok, ts->msac.rng);
        }

        int hr_avg = 0;
        uint16_t (*sign_cdf)[2] = ts->cdf.coef.sign_idtx[sz_ctx];
        const unsigned dq = dq_tbl[1]; // FIXME qm
        dq_shift -= tcq_enabled;
        for (int i = bob; i <= sz; i++) {
            rc = scan[i];
            int tok = cf[rc];
            if (!tok) continue;
            x = rc >> shift;
            y = rc & mask;
            int8_t *const level = &levels[(1 + x) * stride + (1 + y)];
            ctx = get_sign_ctx_idtx(level, stride);
            int sign = dav1d_msac_decode_bool_adapt(&ts->msac, sign_cdf[ctx]);
            if (!i)
                dc_sign_level = (sign - 1) & (2 << 6);
            DEBUG_CF_printf("%*sPost-sign[pos=%d,ctx=%d|%d,plane=%s,%d]: r=%d\n",
                            depth, "", i, sz_ctx, ctx, chroma ? "uv" : "y", sign,
                            ts->msac.rng);
            *level = 1 - 2 * sign;

            // residual
            int val;
            if (tok >= 6) {
                const int hr = decode_hr(&ts->msac, hr_avg);
                tok += hr;
                DEBUG_CF_printf("%*sPost-residual[pos=%d,%d->%d]: r=%d\n",
                                depth, "", i, hr, tok, ts->msac.rng);
                hr_avg = (hr_avg + hr) >> 1;
                tok &= 0xfffff;
                val = (((tok * dq) & 0xffffff) + 4) >> dq_shift;
                val = umin(val, cf_max + sign);
            } else {
                val = (tok * dq + 4) >> dq_shift;
            }
            cul_level += tok;
            cf[rc] = sign ? -val : val;
        }

        goto end;
    } else if (eob) {
        int8_t *const levels = t->scratch.levels;

#define DECODE_COEFS_CLASS(tx_class, xy, is_stx) \
        int lim, tok; \
        uint16_t (*hi_cdf)[4]; \
        union { \
            uint16_t (*hf)[4], (*lf)[8]; \
        } lo_cdf;\
        unsigned ctx = 1 + (eob > 2 << tx2dszctx) + (eob > 4 << tx2dszctx); \
        if (eob >= hi_to_low_tx) { \
            uint16_t (*eob_cdf)[4]; \
            lim = 3; \
            if (!chroma) { \
                eob_cdf = ts->cdf.coef.eob_base_y_tok_hf[t_dim->ctx]; \
                hi_cdf = ts->cdf.coef.br_y_tok_hf; \
                lo_cdf.hf = ts->cdf.coef.base_y_tok_hf[t_dim->ctx][0]; \
            } else { \
                eob_cdf = ts->cdf.coef.eob_base_uv_tok_hf; \
                hi_cdf = ts->cdf.coef.br_uv_tok_hf; \
                lo_cdf.hf = ts->cdf.coef.base_uv_tok_hf; \
            } \
            tok = 1 + dav1d_msac_decode_symbol_adapt4(&ts->msac, eob_cdf[ctx], 2); \
        } else { \
            uint16_t (*eob_cdf)[8]; \
            lim = 5; \
            if (!chroma) { \
                eob_cdf = ts->cdf.coef.eob_base_y_tok_lf[t_dim->ctx]; \
                hi_cdf = ts->cdf.coef.br_y_tok_lf; \
                lo_cdf.lf = ts->cdf.coef.base_y_tok_lf[t_dim->ctx][0]; \
            } else { \
                eob_cdf = ts->cdf.coef.eob_base_uv_tok_lf; \
                hi_cdf = NULL; \
                lo_cdf.lf = ts->cdf.coef.base_uv_tok_lf; \
            } \
            tok = 1 + dav1d_msac_decode_symbol_adapt4(&ts->msac, eob_cdf[ctx], 4); \
        } \
        unsigned rc; \
        unsigned x, y; \
        int8_t *level; \
        if (tx_class == TX_CLASS_2D) \
            rc = scan[eob], x = rc >> shift, y = rc & mask; \
        else if (tx_class == TX_CLASS_H) \
            /* Transposing reduces the stride and padding requirements */ \
            x = eob & mask, y = eob >> shift, rc = eob; \
        else /* tx_class == TX_CLASS_V */ \
            x = eob & mask, y = eob >> shift, rc = (x << shift2) | y; \
        if (tok == lim && hi_cdf) { \
            tok += dav1d_msac_decode_symbol_adapt4(&ts->msac, \
                       hi_cdf[lim == 5 ? 7 : 0], 3); \
        } \
        DEBUG_CF_printf("%*sPost-eob_tok[pos=%d,ctx=%d|%d|%d,freq=%s,plane=%s,%d]: r=%d\n", \
                        depth, "", eob, t_dim->ctx, ctx, \
                        tok < lim || !hi_cdf ? -1 : lim == 5 ? 7 : 0, \
                        lim == 5 ? "lo" : "hi", chroma ? "uv" : "y", \
                        tok, ts->msac.rng); \
        tcq_state = tcq_next_state(tcq_state, tok); \
        cf[is_stx ? (unsigned)eob : rc] = tok; \
        if (tx_class == TX_CLASS_2D) \
            level = levels + rc; \
        else \
            level = levels + x * stride + y; \
        *level = tok; \
        for (int i = eob - 1;; i--) { /* ac */ \
            if (i == hi_to_low_tx - 1) { \
                lim = 5; \
                if (!chroma) { \
                    hi_cdf = ts->cdf.coef.br_y_tok_lf; \
                    lo_cdf.lf = ts->cdf.coef.base_y_tok_lf[t_dim->ctx][0]; \
                } else { \
                    hi_cdf = NULL; \
                    lo_cdf.lf = ts->cdf.coef.base_uv_tok_lf; \
                } \
            } \
            if (!i) break; \
            if (tx_class == TX_CLASS_2D) \
                rc = scan[i], x = rc >> shift, y = rc & mask; \
            else if (tx_class == TX_CLASS_H) \
                x = i & mask, y = i >> shift, rc = i; \
            else /* tx_class == TX_CLASS_V */ \
                x = i & mask, y = i >> shift, rc = (x << shift2) | y; \
            assert(x < 32 && y < 32); \
            if (tx_class == TX_CLASS_2D) \
                level = levels + rc; \
            else \
                level = levels + x * stride + y; \
            unsigned hr_ctx; \
            ctx = get_lo_ctx(level, tx_class, &hr_ctx, xy, plane, stride); \
            const int tcq = (tcq_state & 2) >> 1; \
            const int lo_cdf_idx = ctx * (2 - chroma) + tcq; \
            if (lim == 5) \
                tok = dav1d_msac_decode_symbol_adapt8(&ts->msac, lo_cdf.lf[lo_cdf_idx], 5); \
            else \
                tok = dav1d_msac_decode_symbol_adapt4(&ts->msac, lo_cdf.hf[lo_cdf_idx], 3); \
            if (tok == lim && hi_cdf) \
                tok += dav1d_msac_decode_symbol_adapt4(&ts->msac, hi_cdf[hr_ctx], 3); \
            DEBUG_CF_printf("%*sPost-tok[pos=%d,ctx=%d|%d|%d|%d,freq=%s,plane=%s,%d]: r=%d\n", \
                            depth, "", i, t_dim->ctx, ctx, tcq, \
                            tok < lim || !hi_cdf ? -1 : hr_ctx, \
                            lim == 5 ? "lo" : "hi", \
                            chroma ? "uv" : "y", tok, ts->msac.rng); \
            tcq_state = tcq_next_state(tcq_state, tok); \
            *level = tok; \
            cf[is_stx ? (unsigned)i : rc] = tok; \
        } \
        /* dc */ \
        unsigned hr_ctx; \
        ctx = get_lo_ctx(levels, tx_class, &hr_ctx, 0, plane, stride); \
        const int tcq = (tcq_state & 2) >> 1; \
        const int lo_cdf_idx = ctx * (2 - chroma) + tcq; \
        if (lim == 5) \
            dc_tok = dav1d_msac_decode_symbol_adapt8(&ts->msac, lo_cdf.lf[lo_cdf_idx], 5); \
        else \
            dc_tok = dav1d_msac_decode_symbol_adapt4(&ts->msac, lo_cdf.hf[lo_cdf_idx], 3); \
        if (dc_tok == lim && hi_cdf) { \
            dc_tok += dav1d_msac_decode_symbol_adapt4(&ts->msac, hi_cdf[hr_ctx], 3); \
        } \
        DEBUG_CF_printf("%*sPost-dc_tok[pos=0,ctx=%d|%d|%d|%d,freq=%s,plane=%s,%d]: r=%d\n", \
                        depth, "", t_dim->ctx, ctx, tcq, \
                        dc_tok < lim || !hi_cdf ? -1 : hr_ctx, \
                        lim == 5 ? "lo" : "hi", chroma ? "uv" : "y", \
                        dc_tok, ts->msac.rng); \
        tcq_state = tcq_enabled * -0x80000000; \
        const unsigned ac_dq = dq_tbl[1]; /* FIXME qm */ \
        for (int i = eob; i > 0; i--) { \
            if (tx_class == TX_CLASS_2D) \
                rc = is_stx ? i : scan[i]; \
            else if (tx_class == TX_CLASS_H) \
                y = i >> shift, rc = i; \
            else /* tx_class == TX_CLASS_V */ \
                x = i & mask, y = i >> shift, rc = (x << shift2) | y; \
            int tok = cf[rc]; \
            if (!tok) { \
                tcq_state = tcq_next_state(tcq_state, 0); \
                continue; \
            } \
            int sign; \
            if (tx_class == TX_CLASS_2D || y > 0 || chroma) { \
                sign = dav1d_msac_decode_bool_bypass(&ts->msac); \
                DEBUG_CF_printf("%*sPost-%ssign[pos=%d,%d]: r=%d\n", \
                                depth, "", (tx_class != TX_CLASS_2D && \
                                            !y) ? "dc_" : "", i, sign, \
                                ts->msac.rng); \
            } else { \
                sign = dav1d_msac_decode_bool_adapt(&ts->msac, \
                           ts->cdf.coef.dc_sign[chroma][0][0]); \
                DEBUG_CF_printf("%*sPost-dc_sign[pos=%d,ctx=0,%d]: r=%d\n", \
                                depth, "", i, sign, ts->msac.rng); \
            } \
            const int tcq = (tcq_state & 2) >> 1; \
            tcq_state = tcq_next_state(tcq_state, tok); \
            /* residual */ \
            const int max_br = i < hi_to_low_tx ? (chroma ? 5 : 8) : 6; \
            int ac_val; \
            if (tok >= max_br - tcq_enabled) { \
                const int hr = decode_hr(&ts->msac, hr_avg); \
                tok += hr << tcq_enabled; \
                DEBUG_CF_printf("%*sPost-residual[pos=%d,%d->%d]: r=%d\n", \
                                depth, "", i, hr, tok, ts->msac.rng); \
                hr_avg = (hr_avg + hr) >> 1; \
                tok &= 0xfffff; \
                ac_val = (tok << tcq_enabled) - tcq; \
                ac_val = (((ac_val * ac_dq) & 0xffffff) + 4) >> dq_shift; \
                ac_val = umin(ac_val, cf_max + sign); \
            } else { \
                ac_val = (tok << tcq_enabled) - tcq; \
                ac_val = (ac_val * ac_dq + 4) >> dq_shift; \
            } \
            cul_level += tok; \
            cf[rc] = sign ? -ac_val : ac_val; \
        } \
        break

        const uint16_t *scan;
        switch (tx_class) {
        case TX_CLASS_2D: {
            scan = dav1d_scans[tx];
            const ptrdiff_t stride = 4 << slh;
            const unsigned shift = slh + 2, shift2 = 0;
            const unsigned mask = (4 << slh) - 1;
            memset(levels, 0, stride * ((4 << slw) + 2));
            const int hi_to_low_tx = chroma ? 1 : 10;
            if (stx_type) {
                DECODE_COEFS_CLASS(TX_CLASS_2D, x + y, 1);
            } else {
                DECODE_COEFS_CLASS(TX_CLASS_2D, x + y, 0);
            }
        }
        case TX_CLASS_H: {
            const ptrdiff_t stride = 32;
            const unsigned shift = slh + 2, shift2 = 0;
            const unsigned mask = (4 << slh) - 1;
            memset(levels, 0, stride * ((4 << slh) + 2));
            const int hi_to_low_tx = (8 << slh) >> chroma;
            DECODE_COEFS_CLASS(TX_CLASS_H, y, 0);
        }
        case TX_CLASS_V: {
            const ptrdiff_t stride = 32;
            const unsigned shift = slw + 2, shift2 = slh + 2;
            const unsigned mask = (4 << slw) - 1;
            memset(levels, 0, stride * ((4 << slw) + 2));
            const int hi_to_low_tx = (8 << slw) >> chroma;
            DECODE_COEFS_CLASS(TX_CLASS_V, y, 0);
        }
#undef DECODE_COEFS_CLASS
        default: assert(0);
        }
    } else if (chroma) { // dc-only
        dc_tok = 1 + dav1d_msac_decode_symbol_adapt8(&ts->msac,
                         ts->cdf.coef.eob_base_uv_tok_lf[0], 4);
        DEBUG_CF_printf("%*sPost-eob_tok[pos=%d,ctx=%d|0|-1,freq=lo,plane=uv,%d]: r=%d\n",
                        depth, "", eob, t_dim->ctx, dc_tok, ts->msac.rng);
    } else {
        dc_tok = 1 + dav1d_msac_decode_symbol_adapt8(&ts->msac,
                         ts->cdf.coef.eob_base_y_tok_lf[t_dim->ctx][0], 4);
        if (dc_tok == 5) {
            dc_tok += dav1d_msac_decode_symbol_adapt4(&ts->msac,
                          ts->cdf.coef.br_y_tok_lf[tx_class == TX_CLASS_2D ? 0 : 7], 3);
        }
        DEBUG_CF_printf("%*sPost-eob_tok[pos=%d,ctx=%d|0|%d,freq=lo,plane=y,%d]: r=%d\n",
                        depth, "", eob, t_dim->ctx,
                        dc_tok < 5 ? -1 : tx_class == TX_CLASS_2D ? 0 : 7,
                        dc_tok, ts->msac.rng);
    }

    if (!dc_tok) goto end;

    // dc sign & residual
    int dc_sign;
    if (chroma) {
        dc_sign = dav1d_msac_decode_bool_bypass(&ts->msac);
        DEBUG_CF_printf("%*sPost-dc_sign[pos=0,%d]: r=%d\n",
                        depth, "", dc_sign, ts->msac.rng);
    } else {
        const int dc_sign_ctx = get_dc_sign_ctx(t_dim, a, l);
        uint16_t *const dc_sign_cdf = ts->cdf.coef.dc_sign[chroma][0][dc_sign_ctx];
        dc_sign = dav1d_msac_decode_bool_adapt(&ts->msac, dc_sign_cdf);
        DEBUG_CF_printf("%*sPost-dc_sign[pos=0,ctx=%d,%d]: r=%d\n",
                        depth, "", dc_sign_ctx, dc_sign, ts->msac.rng);
    }

    int dc_dq = dq_tbl[0];
    dc_sign_level = (dc_sign - 1) & (2 << 6);

    if (qm_tbl) {
        // FIXME
        dc_dq = (dc_dq * qm_tbl[0] + 16) >> 5;

        if (dc_tok == 15) {
            dc_tok = 0; //read_golomb(&ts->msac) + 15;
            DEBUG_CF_printf("%*sPost-dc_residual[%d->%d]: r=%d\n",
                            depth, "", dc_tok - 15, dc_tok, ts->msac.rng);

            dc_tok &= 0xfffff;
            dc_dq = (dc_dq * dc_tok) & 0xffffff;
        } else {
            dc_dq *= dc_tok;
            assert(dc_dq <= 0xffffff);
        }
        cul_level = dc_tok;
        dc_dq >>= dq_shift;
        dc_dq = umin(dc_dq, cf_max + dc_sign);
        cf[0] = (coef) (dc_sign ? -dc_dq : dc_dq);
    } else {
        // non-qmatrix is the common case and allows for additional optimizations
        const int max_br = chroma ? 5 : 8;
        int dc_val;
        int tcq = (tcq_state & 2) >> 1;
        if (dc_tok >= max_br - tcq_enabled) {
            const int hr = decode_hr(&ts->msac, hr_avg);
            dc_tok += hr << tcq_enabled;
            DEBUG_CF_printf("%*sPost-residual[pos=0,%d->%d]: r=%d\n",
                            depth, "", hr, dc_tok, ts->msac.rng);
            dc_tok &= 0xfffff;
            dc_val = (dc_tok << tcq_enabled) - tcq;
            dc_val = (((dc_val * dc_dq) & 0xffffff) + 4) >> dq_shift;
            dc_val = umin(dc_val, cf_max + dc_sign);
        } else {
            dc_val = (dc_tok << tcq_enabled) - tcq;
            dc_val = (dc_val * dc_dq + 4) >> dq_shift;
        }
        cul_level += dc_tok;
        cf[0] = dc_sign ? -dc_val : dc_val;
    }

end:
    // context
    *res_ctx = umin(cul_level, 63) | dc_sign_level;

    return eob;
}

void bytefn(dav1d_read_coef_blocks)(Dav1dTaskContext *const t,
                                    const enum BlockSize bs, const Av1Block *const b)
{
#if 0
    const Dav1dFrameContext *const f = t->f;
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int bx4 = t->bx & 63, by4 = t->by & 63;
    const int cbx4 = bx4 >> ss_hor, cby4 = by4 >> ss_ver;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int cbw4 = (bw4 + ss_hor) >> ss_hor, cbh4 = (bh4 + ss_ver) >> ss_ver;
    const int has_chroma = 0 && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400 &&
                           (bw4 > ss_hor || t->bx & 1) &&
                           (bh4 > ss_ver || t->by & 1);

    if (b->skip_txfm) {
        BlockContext *const a = t->a;
        dav1d_memset_pow2[b_dim[2]](&a->lcoef[bx4], 0x40);
        dav1d_memset_pow2[b_dim[3]](&t->l.lcoef[by4], 0x40);
        if (has_chroma) {
            dav1d_memset_pow2_fn memset_cw = dav1d_memset_pow2[ulog2(cbw4)];
            dav1d_memset_pow2_fn memset_ch = dav1d_memset_pow2[ulog2(cbh4)];
            memset_cw(&a->ccoef[0][cbx4], 0x40);
            memset_cw(&a->ccoef[1][cbx4], 0x40);
            memset_ch(&t->l.ccoef[0][cby4], 0x40);
            memset_ch(&t->l.ccoef[1][cby4], 0x40);
        }
        return;
    }

    Dav1dTileState *const ts = t->ts;
    const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
    const int cw4 = (w4 + ss_hor) >> ss_hor, ch4 = (h4 + ss_ver) >> ss_ver;
    assert(t->frame_thread.pass == 1);
    assert(!b->skip_txfm);
    const TxfmInfo *const uv_t_dim = &dav1d_txfm_dimensions[b->uvtx];
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[b->intra ? b->tx : b->max_ytx];
    const uint16_t tx_split[2] = { b->tx_split0, b->tx_split1 };

    for (int init_y = 0; init_y < h4; init_y += 16) {
        const int sub_h4 = imin(h4, 16 + init_y);
        for (int init_x = 0; init_x < w4; init_x += 16) {
            const int sub_w4 = imin(w4, init_x + 16);
            int y_off = !!init_y, y, x;
            for (y = init_y, t->by += init_y; y < sub_h4;
                 y += t_dim->h, t->by += t_dim->h, y_off++)
            {
                int x_off = !!init_x;
                for (x = init_x, t->bx += init_x; x < sub_w4;
                     x += t_dim->w, t->bx += t_dim->w, x_off++)
                {
                    if (!b->intra) {
                        read_coef_tree(t, bs, b, b->max_ytx, 0, tx_split,
                                       x_off, y_off, NULL);
                    } else {
                        uint8_t cf_ctx = 0x40;
                        enum TxfmType txtp;
                        const int eob =
                            decode_coefs(t, &t->a->lcoef[bx4 + x],
                                         &t->l.lcoef[by4 + y], b->tx, bs, b,
                                         0, ts->frame_thread[1].cf, &txtp, &cf_ctx);
                        DEBUG_BLOCK_printf("Post-y_cf_blk[tx=%dx%d,txtp=%d,eob=%d]: r=%d\n",
                                           t_dim->w * 4, t_dim->h * 4, txtp, eob,
                                           ts->msac.rng);
                        txtp &= 0xf; // FIXME
                        *ts->frame_thread[1].cbi++ = eob * (1 << 5) + txtp;
                        ts->frame_thread[1].cf += imin(t_dim->w, 8) * imin(t_dim->h, 8) * 16;
                        dav1d_memset_likely_pow2(&t->a->lcoef[bx4 + x], cf_ctx, imin(t_dim->w, f->bw - t->bx));
                        dav1d_memset_likely_pow2(&t->l.lcoef[by4 + y], cf_ctx, imin(t_dim->h, f->bh - t->by));
                    }
                }
                t->bx -= x;
            }
            t->by -= y;

            if (!has_chroma) continue;

            const int sub_ch4 = imin(ch4, (init_y + 16) >> ss_ver);
            const int sub_cw4 = imin(cw4, (init_x + 16) >> ss_hor);
            for (int pl = 0; pl < 2; pl++) {
                for (y = init_y >> ss_ver, t->by += init_y; y < sub_ch4;
                     y += uv_t_dim->h, t->by += uv_t_dim->h << ss_ver)
                {
                    for (x = init_x >> ss_hor, t->bx += init_x; x < sub_cw4;
                         x += uv_t_dim->w, t->bx += uv_t_dim->w << ss_hor)
                    {
                        uint8_t cf_ctx = 0x40;
                        enum TxfmType txtp;
                        if (!b->intra)
                            txtp = t->scratch.txtp_map[((t->by + (y << ss_ver)) & 15) * 16 +
                                                       ((t->bx + (x << ss_hor)) & 15)];
                        const int eob =
                            decode_coefs(t, &t->a->ccoef[pl][cbx4 + x],
                                         &t->l.ccoef[pl][cby4 + y], b->uvtx, bs,
                                         b, 1 + pl, ts->frame_thread[1].cf,
                                         &txtp, &cf_ctx);
                        if (DEBUG_BLOCK_INFO)
                            printf("Post-uv-cf-blk[pl=%d,tx=%d,"
                                   "txtp=%d,eob=%d]: r=%d\n",
                                   pl, b->uvtx, txtp, eob, ts->msac.rng);
                        *ts->frame_thread[1].cbi++ = eob * (1 << 5) + txtp;
                        ts->frame_thread[1].cf += uv_t_dim->w * uv_t_dim->h * 16;
                        int ctw = imin(uv_t_dim->w, (f->bw - t->bx + ss_hor) >> ss_hor);
                        int cth = imin(uv_t_dim->h, (f->bh - t->by + ss_ver) >> ss_ver);
                        dav1d_memset_likely_pow2(&t->a->ccoef[pl][cbx4 + x], cf_ctx, ctw);
                        dav1d_memset_likely_pow2(&t->l.ccoef[pl][cby4 + y], cf_ctx, cth);
                    }
                    t->bx -= x << ss_hor;
                }
                t->by -= y << ss_ver;
            }
        }
    }
#endif
}

static void mc(Dav1dTaskContext *const t,
               pixel *dst8, int16_t *const dst16, const ptrdiff_t dst_stride,
               int bw4, const int bh4,
               const int bx, const int by, const int pl,
               const mv mv, const Dav1dThreadPicture *const refp, const int refidx,
               const enum Dav1dFilterMode filter,
               const int left, const int right, const int top, const int bottom)
{
    assert((dst8 != NULL) ^ (dst16 != NULL));
    const Dav1dFrameContext *const f = t->f;
    const int ss_ver = !!pl && f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = !!pl && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;
    const int mvx = mv.x, mvy = mv.y;
    ptrdiff_t ref_stride = refp->p.stride[!!pl];
    const pixel *ref;

    assert(left >= 0 && top >= 0 && left < right && top < bottom &&
           right <= f->bw * 4 && bottom <= f->bh * 4);

    if (refp->p.p.w == f->cur.p.w && refp->p.p.h == f->cur.p.h) {
        const int mx = mvx & (15 >> !ss_hor), my = mvy & (15 >> !ss_ver);
        const int dx = bx * h_mul + (mvx >> (3 + ss_hor));
        const int dy = by * v_mul + (mvy >> (3 + ss_ver));

        if (dx - !!mx * 3 < left || dy - !!my * 3 < top ||
            dx + bw4 * h_mul + !!mx * 4 > right ||
            dy + bh4 * v_mul + !!my * 4 > bottom)
        {
            pixel *const emu_edge_buf = bitfn(t->scratch.emu_edge);
            ref = refp->p.data[pl];
            f->dsp->mc.emu_edge(bw4 * h_mul + !!mx * 7, bh4 * v_mul + !!my * 7,
                                right - left, bottom - top, dx - !!mx * 3 - left,
                                dy - !!my * 3 - top,
                                emu_edge_buf, 192 * sizeof(pixel),
                                &ref[left + top * PXSTRIDE(ref_stride)],
                                ref_stride);
            ref = &emu_edge_buf[192 * !!my * 3 + !!mx * 3];
            ref_stride = 192 * sizeof(pixel);
        } else {
            ref = ((pixel *) refp->p.data[pl]) + PXSTRIDE(ref_stride) * dy + dx;
        }

        if (dst8 != NULL) {
            if (bw4 & (bw4 - 1)) {
                assert(!ss_hor && !ss_ver && v_mul == 4 && h_mul == 4 &&
                       filter == DAV1D_FILTER_BILINEAR);
                assert(!((bw4 - 2) & (bw4 - 3)));
                f->dsp->mc.mc[filter](dst8, dst_stride, ref, ref_stride, bw4 * 4 - 8,
                                      bh4 * 4, mx << 1, my << 1 HIGHBD_CALL_SUFFIX);
                dst8 += 4 * (bw4 - 2);
                ref += 4 * (bw4 - 2);
                bw4 = 2;
            }
            f->dsp->mc.mc[filter](dst8, dst_stride, ref, ref_stride, bw4 * h_mul,
                                  bh4 * v_mul, mx << !ss_hor, my << !ss_ver
                                  HIGHBD_CALL_SUFFIX);
        } else {
            f->dsp->mc.mct[filter](dst16, dst_stride, ref, ref_stride, bw4 * h_mul,
                                   bh4 * v_mul, mx << !ss_hor, my << !ss_ver
                                   HIGHBD_CALL_SUFFIX);
        }
    } else {
        assert(refp != &f->sr_cur);

        const int orig_pos_y = (by * v_mul << 4) + mvy * (1 << !ss_ver);
        const int orig_pos_x = (bx * h_mul << 4) + mvx * (1 << !ss_hor);
#define scale_mv(res, val, scale) do { \
            const int64_t tmp = (int64_t)(val) * scale + (scale - 0x4000) * 8; \
            res = apply_sign64((llabs(tmp) + 128) >> 8, tmp) + 32;     \
        } while (0)
        int pos_y, pos_x;
        scale_mv(pos_x, orig_pos_x, f->svc[refidx][0].scale);
        scale_mv(pos_y, orig_pos_y, f->svc[refidx][1].scale);
#undef scale_mv
        const int left = pos_x >> 10;
        const int top = pos_y >> 10;
        const int right =
            ((pos_x + (bw4 * h_mul - 1) * f->svc[refidx][0].step) >> 10) + 1;
        const int bottom =
            ((pos_y + (bh4 * v_mul - 1) * f->svc[refidx][1].step) >> 10) + 1;

        if (DEBUG_BLOCK_INFO)
            printf("Off %dx%d [%d,%d,%d], size %dx%d [%d,%d]\n",
                   left, top, orig_pos_x, f->svc[refidx][0].scale, refidx,
                   right-left, bottom-top,
                   f->svc[refidx][0].step, f->svc[refidx][1].step);

        const int w = (refp->p.p.w + ss_hor) >> ss_hor;
        const int h = (refp->p.p.h + ss_ver) >> ss_ver;
        if (left < 3 || top < 3 || right + 4 > w || bottom + 4 > h) {
            pixel *const emu_edge_buf = bitfn(t->scratch.emu_edge);
            f->dsp->mc.emu_edge(right - left + 7, bottom - top + 7,
                                w, h, left - 3, top - 3,
                                emu_edge_buf, 320 * sizeof(pixel),
                                refp->p.data[pl], ref_stride);
            ref = &emu_edge_buf[320 * 3 + 3];
            ref_stride = 320 * sizeof(pixel);
            if (DEBUG_BLOCK_INFO) printf("Emu\n");
        } else {
            ref = ((pixel *) refp->p.data[pl]) + PXSTRIDE(ref_stride) * top + left;
        }

        if (dst8 != NULL) {
            f->dsp->mc.mc_scaled[filter](dst8, dst_stride, ref, ref_stride,
                                         bw4 * h_mul, bh4 * v_mul,
                                         pos_x & 0x3ff, pos_y & 0x3ff,
                                         f->svc[refidx][0].step,
                                         f->svc[refidx][1].step
                                         HIGHBD_CALL_SUFFIX);
        } else {
            f->dsp->mc.mct_scaled[filter](dst16, dst_stride, ref, ref_stride,
                                          bw4 * h_mul, bh4 * v_mul,
                                          pos_x & 0x3ff, pos_y & 0x3ff,
                                          f->svc[refidx][0].step,
                                          f->svc[refidx][1].step
                                          HIGHBD_CALL_SUFFIX);
        }
    }
}

// like mc(), but:
// - mv subpel precision is 4 instead of 3 bits
// - we support custom edge limits, since the opfl reference area cannot exceed
//   the original mv's bounding box (the remaining pixels are emulated)
// - no scaled support, and no dst8 support
static void mc_opfl(Dav1dTaskContext *const t,
                    int16_t *const dst16, const ptrdiff_t dst_stride,
                    const int bw4, const int bh4, const int bx4, const int by4,
                    const mv mv, const Dav1dThreadPicture *const refp,
                    const int left, const int right, const int top, const int bottom)
{
    const Dav1dFrameContext *const f = t->f;
    assert(refp->p.p.w == f->cur.p.w && refp->p.p.h == f->cur.p.h);
    const int mvx = mv.x, mvy = mv.y;
    ptrdiff_t ref_stride = refp->p.stride[0];
    const pixel *ref;
    const int mx = mvx & 15, my = mvy & 15;
    const int dx = bx4 * 4 + (mvx >> 4);
    const int dy = by4 * 4 + (mvy >> 4);
    assert(top >= 0 && left >= 0 && left < right && top < bottom &&
           right <= f->bw * 4 && bottom <= f->bh * 4);

    if (dx - !!mx * 3 < left || dy - !!my * 3 < top ||
        dx + bw4 * 4 + !!mx * 4 > right ||
        dy + bh4 * 4 + !!my * 4 > bottom)
    {
        pixel *const emu_edge_buf = bitfn(t->scratch.emu_edge);
        ref = refp->p.data[0];
        f->dsp->mc.emu_edge(bw4 * 4 + !!mx * 7, bh4 * 4 + !!my * 7,
                            right - left, bottom - top, dx - !!mx * 3 - left,
                            dy - !!my * 3 - top,
                            emu_edge_buf, 192 * sizeof(pixel),
                            &ref[left + top * PXSTRIDE(ref_stride)],
                            ref_stride);
        ref = &emu_edge_buf[192 * !!my * 3 + !!mx * 3];
        ref_stride = 192 * sizeof(pixel);
    } else {
        ref = ((pixel *) refp->p.data[0]) + PXSTRIDE(ref_stride) * dy + dx;
    }

    f->dsp->mc.mct[DAV1D_FILTER_8TAP_SHARP](dst16, dst_stride, ref, ref_stride,
                                            bw4 * 4, bh4 * 4, mx, my
                                            HIGHBD_CALL_SUFFIX);
}

static void ext_warp(Dav1dTaskContext *const t,
                     pixel *dst8, int16_t *dst16, const ptrdiff_t dstride,
                     const uint8_t *const b_dim, const int pl,
                     const Dav1dThreadPicture *const refp,
                     const Dav1dWarpedMotionParams *const wmp)
{
    assert((dst8 != NULL) ^ (dst16 != NULL));
    const Dav1dFrameContext *const f = t->f;
    const Dav1dDSPContext *const dsp = f->dsp;
    const int ss_ver = !!pl && f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = !!pl && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;
    assert(!((b_dim[0] * h_mul) & 7) && !((b_dim[1] * v_mul) & 7));
    const int32_t *const mat = wmp->matrix;
    const int w = f->bw * 4 >> ss_hor;
    const int h = f->bh * 4 >> ss_ver;
    const int sw = imin(b_dim[0] * 4, 8), hsw = sw >> 1;
    const int sh = imin(b_dim[1] * 4, 8), hsh = sh >> 1;

    for (int y = 0; y < b_dim[1] * v_mul; y += sh) {
        const int src_y = t->by * 4 + ((y + hsh) << ss_ver);
        const int64_t mat3_y = (int64_t) mat[3] * src_y + mat[0];
        const int64_t mat5_y = (int64_t) mat[5] * src_y + mat[1];
        for (int x = 0; x < b_dim[0] * h_mul; x += sw) {
            // calculate transformation relative to center of 8x8 block in
            // luma pixel units
            const int src_x = t->bx * 4 + ((x + hsw) << ss_hor);
            const int64_t mvx = ((int64_t) mat[2] * src_x + mat3_y) >> ss_hor;
            const int64_t mvy = ((int64_t) mat[4] * src_x + mat5_y) >> ss_ver;
            const int left_window = (int) (mvx >> 16) - hsw - 3;
            const int top_window = (int) (mvy >> 16) - hsh - 3;
            const int left = iclip(left_window, 0, w - 1);
            const int right = iclip(left_window + sw + 7, 1, w);
            const int top = iclip(top_window, 0, h - 1);
            const int bottom = iclip(top_window + sh + 7, 1, h);

            for (int yy = y; yy < y + sh; yy += 4) {
                const int src_y = t->by * 4 + ((yy + 2) << ss_ver);
                const int64_t mat3_y = (int64_t) mat[3] * src_y + mat[0];
                const int64_t mat5_y = (int64_t) mat[5] * src_y + mat[1];
                for (int xx = x; xx < x + sw; xx += 4) {
                    const int src_x = t->bx * 4 + ((xx + 2) << ss_hor);
                    const int64_t mvx =
                        (((int64_t) mat[2] * src_x + mat3_y) >> ss_hor) + 0x200;
                    const int64_t mvy =
                        (((int64_t) mat[4] * src_x + mat5_y) >> ss_ver) + 0x200;

                    const int dx = (int) (mvx >> 16) - 2;
                    const int mx = (int) ((mvx >> 10) & 63);
                    const int dy = (int) (mvy >> 16) - 2;
                    const int my = (int) ((mvy >> 10) & 63);

                    const pixel *ref_ptr = refp->p.data[pl];
                    ptrdiff_t ref_stride = refp->p.stride[!!pl];

                    if (dx - 3 < left || dx + 4 + 4 > right ||
                        dy - 3 < top || dy + 4 + 4 > bottom)
                    {
                        pixel *const emu_edge_buf = bitfn(t->scratch.emu_edge);
                        f->dsp->mc.emu_edge(11, 11, right - left, bottom - top,
                                            dx - 3 - left, dy - 3 - top,
                                            emu_edge_buf, 16 * sizeof(pixel),
                                            &ref_ptr[left + top * PXSTRIDE(ref_stride)],
                                            ref_stride);
                        ref_ptr = &emu_edge_buf[16 * 3 + 3];
                        ref_stride = 16 * sizeof(pixel);
                    } else {
                        ref_ptr = &ref_ptr[PXSTRIDE(ref_stride) * dy + dx];
                    }

                    if (dst16 != NULL)
                        dsp->mc.ext_warp4x4t(&dst16[yy * dstride + xx], dstride,
                                             ref_ptr, ref_stride,
                                             mx, my HIGHBD_CALL_SUFFIX);
                    else
                        dsp->mc.ext_warp4x4(&dst8[yy * PXSTRIDE(dstride) + xx],
                                            dstride, ref_ptr, ref_stride,
                                            mx, my HIGHBD_CALL_SUFFIX);
                }
            }
        }
    }
}

static void warp_affine(Dav1dTaskContext *const t,
                        pixel *dst8, int16_t *dst16, const ptrdiff_t dstride,
                        const uint8_t *const b_dim, const int pl,
                        const Dav1dThreadPicture *const refp,
                        const Dav1dWarpedMotionParams *const wmp)
{
    if (!wmp->affine) {
        ext_warp(t, dst8, dst16, dstride, b_dim, pl, refp, wmp);
        return;
    }

    assert((dst8 != NULL) ^ (dst16 != NULL));
    const Dav1dFrameContext *const f = t->f;
    const Dav1dDSPContext *const dsp = f->dsp;
    const int ss_ver = !!pl && f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = !!pl && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;
    assert(!((b_dim[0] * h_mul) & 7) && !((b_dim[1] * v_mul) & 7));
    const int32_t *const mat = wmp->matrix;
    const int width = f->bw * 4 >> ss_hor;
    const int height = f->bh * 4 >> ss_ver;

    for (int y = 0; y < b_dim[1] * v_mul; y += 8) {
        const int src_y = t->by * 4 + ((y + 4) << ss_ver);
        const int64_t mat3_y = (int64_t) mat[3] * src_y + mat[0];
        const int64_t mat5_y = (int64_t) mat[5] * src_y + mat[1];
        for (int x = 0; x < b_dim[0] * h_mul; x += 8) {
            // calculate transformation relative to center of 8x8 block in
            // luma pixel units
            const int src_x = t->bx * 4 + ((x + 4) << ss_hor);
            const int64_t mvx = ((int64_t) mat[2] * src_x + mat3_y) >> ss_hor;
            const int64_t mvy = ((int64_t) mat[4] * src_x + mat5_y) >> ss_ver;

            const int dx = (int) (mvx >> 16) - 4;
            const int mx = (((int) mvx & 0xffff) - wmp->u.p.alpha * 4 -
                                                   wmp->u.p.beta  * 7) & ~0x3f;
            const int dy = (int) (mvy >> 16) - 4;
            const int my = (((int) mvy & 0xffff) - wmp->u.p.gamma * 4 -
                                                   wmp->u.p.delta * 4) & ~0x3f;

            const pixel *ref_ptr;
            ptrdiff_t ref_stride = refp->p.stride[!!pl];

            if (dx < 3 || dx + 8 + 4 > width || dy < 3 || dy + 8 + 4 > height) {
                pixel *const emu_edge_buf = bitfn(t->scratch.emu_edge);
                f->dsp->mc.emu_edge(15, 15, width, height, dx - 3, dy - 3,
                                    emu_edge_buf, 32 * sizeof(pixel),
                                    refp->p.data[pl], ref_stride);
                ref_ptr = &emu_edge_buf[32 * 3 + 3];
                ref_stride = 32 * sizeof(pixel);
            } else {
                ref_ptr = ((pixel *) refp->p.data[pl]) + PXSTRIDE(ref_stride) * dy + dx;
            }
            if (dst16 != NULL)
                dsp->mc.warp8x8t(&dst16[x], dstride, ref_ptr, ref_stride,
                                 wmp->u.abcd, mx, my HIGHBD_CALL_SUFFIX);
            else
                dsp->mc.warp8x8(&dst8[x], dstride, ref_ptr, ref_stride,
                                wmp->u.abcd, mx, my HIGHBD_CALL_SUFFIX);
        }
        if (dst8) dst8  += 8 * PXSTRIDE(dstride);
        else      dst16 += 8 * dstride;
    }
}

static void gen_mask(uint8_t *mask, const ptrdiff_t stride,
                     const int bw, const int bh,
                     const int x0, const int y0,
                     const int x1, const int y1,
                     const unsigned fw, const unsigned fh)
{
    for (int y = 0; y < bh; y++) {
        for (int x = 0; x < bw; x++) {
            int p0 = (unsigned) (x0 + x) < fw && (unsigned) (y0 + y) < fh;
            int p1 = (unsigned) (x1 + x) < fw && (unsigned) (y1 + y) < fh;
            mask[x] = 32 * (p0 - p1 + 1);
        }
        mask += stride;
    }
}

static void opfl_mv_adj(const struct OpflRegressionData *const r,
                        union OpflMvDeltaBlock *const dd, const int8_t d[2])
{
    int su2 = r->su2, suv = r->suv, sv2 = r->sv2, suw = r->suw, svw = r->svw;
    const int nbits_su2 = 1 + ulog2(su2 + !su2);
    const int nbits_sv2 = 1 + ulog2(sv2 + !sv2);
    const int nbits_suv = 1 + ulog2(abs(suv) + !suv);
    const int nbits_suw = 1 + ulog2(abs(suw) + !suw);
    const int nbits_svw = 1 + ulog2(abs(svw) + !svw);
    const int nbits_max =
        imax(nbits_su2 + nbits_sv2,
             imax(imax(nbits_sv2 + nbits_suw, nbits_suv + nbits_svw),
                  imax(nbits_su2 + nbits_svw, nbits_suv + nbits_suw)));
    const int rbits = imax(0, nbits_max - 23) >> 1;
    if (rbits) {
        const int rnd = (1 << rbits) >> 1;
        su2 = (su2 + rnd) >> rbits;
        sv2 = (sv2 + rnd) >> rbits;
        suv = (suv + rnd - (suv < 0)) >> rbits;
        suw = (suw + rnd - (suw < 0)) >> rbits;
        svw = (svw + rnd - (svw < 0)) >> rbits;
    }
    const int det = su2 * sv2 - suv * suv;
    if (det > 0) {
        int s[2] = { sv2 * suw - suv * svw, su2 * svw - suv * suw }, shift;
        const int idet = resolve_divisor_32(det, &shift), idet_bits = ulog2(idet);
        for (int i = 0; i < 2; i++) {
            if (!s[i]) continue;
            int abss = abs(s[i]);
            const int rbits = imax(0, ulog2(abss) + idet_bits - 22);
            if (rbits > 0) abss = (abss + ((1 << rbits) >> 1)) >> rbits;
            const int ibits = 3 + rbits - shift;
            if (ibits >= 0)
                abss = abss * idet * (1 << ibits);
            else
                abss = (abss * idet + ((1 << -ibits) >> 1)) >> -ibits;
            s[i] = apply_sign(abss, s[i]);
        }
        dd->d[0].x = -iclip(d[0] * s[0], -16, 16);
        dd->d[0].y = -iclip(d[0] * s[1], -16, 16);
        dd->d[1].x = -iclip(d[1] * s[0], -16, 16);
        dd->d[1].y = -iclip(d[1] * s[1], -16, 16);
    } else dd->n = 0;
}

static int tip_pred(Dav1dTaskContext *const t,
                    // we don't actually fill dst with any useful data, but
                    // need it as one of our scratch buffers
                    pixel *const p0, const ptrdiff_t p0_stride,
                    int16_t (*const tmp)[128 * 128], const Av1Block *const b,
                    const int bw4, const int bh4, const int w4, const int h4)
{
    const Dav1dFrameContext *const f = t->f;
    const int refine = f->seq_hdr->tip_refine_mv;
    const int step = 2 << (f->frame_hdr->tip.frame_mode == 1 /* reference */ &&
                           ((!refine && imin(bw4, bh4) >= 4) || b->bs == BS_256x256));
    const uint8_t *const refs = f->frame_hdr->tip.refs;
    ptrdiff_t off_y = 0;
    uint8_t *const mask = t->scratch.seg_mask;
    const int bacp = f->seq_hdr->imp_msk_bld && b->cwp_idx == 8 &&
        !f->svc[refs[0]][0].scale && !f->svc[refs[1]][0].scale;
    const int w = f->bw * 4, h = f->bh * 4;
    if (bacp) memset(mask, 0x20, bw4 * bh4 * 16);
    int have_bacp = 0;

    const Dav1dThreadPicture *const refp[2] = { &f->refp[refs[0]],
                                                &f->refp[refs[1]] };
    pixel *p1, *p[2];
    ptrdiff_t p1_stride, p_stride[2];
    int8_t d[2];
    if (refine) {
        p1 = bitfn(t->scratch.interintra);
        p[0] = p0;
        p[1] = p1;
        p_stride[0] = p0_stride;
        p1_stride = p_stride[1] = ((step + 2) * 4 * sizeof(pixel) + 63) & ~63;
        const int d0 = f->absrefdist[refs[0]], d1 = f->absrefdist[refs[1]];
        d[0] = apply_sign(1 + (d0 > d1), -f->refdist[refs[0]]);
        d[1] = apply_sign(1 + (d1 > d0), -f->refdist[refs[1]]);
    }

    const unsigned sad8x8_thr = f->frame_hdr->tip.frame_mode == 1 /* reference */ ? 6 : 15;
    for (int y = 0, yy = 0; y < h4; y += step, yy++) {
        const ptrdiff_t off_y8 = (((t->by + y) & (f->sb_step - 1)) >> 1) * f->rf.rp_stride;
        for (int x = 0, xx = 0; x < w4; x += step, xx++) {
            const ptrdiff_t off_8x8 = off_y8 + ((t->bx + x) >> 1);
            mv tmv = t->rt.rp_proj[off_8x8].mv;
            if (tmv.n == INVALID_MV) tmv.n = 0;
            union mv cmv[2];
            int left[2], top[2];
            for (int i = 0; i < 2; i++) {
                const mv tipmv = scale_mv(tmv, f->rf.tip_sf[i]);
                const union mv mv = (union mv) {
                    .y = iclip(tipmv.y + b->mv[0].y, -0xffff, 0xffff),
                    .x = iclip(tipmv.x + b->mv[0].x, -0xffff, 0xffff),
                };
                cmv[i] = mv;
                top[i] = t->by * 4 + y * 4 + (cmv[i].y >> 3) - 3;
                left[i] = t->bx * 4 + x * 4 + (cmv[i].x >> 3) - 3;
            }
            if (refine) {
                // refinement
                for (int i = 0; i < 2; i++)
                    mc(t, p[i], NULL, p_stride[i],
                       step + 2, step + 2, t->bx + x, t->by + y, 0,
                       (union mv) { .y = cmv[i].y - 32, .x = cmv[i].x - 32 },
                       refp[i], refs[i], DAV1D_FILTER_BILINEAR,
                       iclip(left[i], 0, w - 1), iclip(left[i] + 7 + 8, 1, w),
                       iclip(top[i], 0, h - 1), iclip(top[i] + 7 + 8, 1, h));
                struct OpflOffset o;
                f->dsp->mc.sad_refine_mv(p0, p0_stride, p1, p1_stride,
                                         step * 4, step * 4, 1, &o
                                         HIGHBD_CALL_SUFFIX);
                const int dy = o.y, dx = o.x;
                union OpflMvDeltaBlock *const dd = &t->opfl[yy * ((bw4 + 1) >> 1) + xx];
                const unsigned sad = f->dsp->mc.sad8x8(&p0[(4 + dy) * PXSTRIDE(p0_stride) +
                                                           (4 + dx)], p0_stride,
                                                       &p1[(4 - dy) * PXSTRIDE(p1_stride) +
                                                           (4 - dx)], p1_stride
                                                       HIGHBD_CALL_SUFFIX);
                if (sad >= sad8x8_thr) {
                    // FIXME for 256x256 blocks, sad-refinement is done at 16x16,
                    // but opfl-refinement is done at 8x8, so the code below may
                    // need a loop to reconstruct that correctly. Alternatively,
                    // opfl refinement might need to be done at 16x16.
                    struct OpflRegressionData res[4];
                    f->dsp->mc.opfl_derive_mv(res,
                                              &p0[(4 + dy) * PXSTRIDE(p0_stride) +
                                                  (4 + dx)], p0_stride,
                                              &p1[(4 - dy) * PXSTRIDE(p1_stride) +
                                                  (4 - dx)], p1_stride,
                                              step * 4, step * 4, 8, d
                                              HIGHBD_CALL_SUFFIX);
                    opfl_mv_adj(res, dd, d);
                } else {
                    dd->n = 0;
                }
                cmv[0].x = cmv[0].x * 2 + dx * 16 + dd->d[0].x;
                cmv[0].y = cmv[0].y * 2 + dy * 16 + dd->d[0].y;
                cmv[1].x = cmv[1].x * 2 - dx * 16 + dd->d[1].x;
                cmv[1].y = cmv[1].y * 2 - dy * 16 + dd->d[1].y;
                for (int i = 0; i < 2; i++)
                    mc_opfl(t, &tmp[i][y * bw4 * 16 + x * 4], bw4 * 4, step, step,
                            t->bx + x, t->by + y, cmv[i], refp[i],
                            iclip(left[i], 0, w - 1),
                            iclip(left[i] + 7 + step * 4, 1, w),
                            iclip(top[i], 0, h - 1),
                            iclip(top[i] + 7 + step * 4, 1, h));
                dd->d[0].x = ((dd->d[0].x + (dd->d[0].x > 0)) >> 1) + dx * 8;
                dd->d[0].y = ((dd->d[0].y + (dd->d[0].y > 0)) >> 1) + dy * 8;
                dd->d[1].x = ((dd->d[1].x + (dd->d[1].x > 0)) >> 1) - dx * 8;
                dd->d[1].y = ((dd->d[1].y + (dd->d[1].y > 0)) >> 1) - dy * 8;
            } else {
                for (int i = 0; i < 2; i++)
                    mc(t, NULL, &tmp[i][off_y + x * 4], bw4 * 4,
                       step, step, t->bx + x, t->by + y, 0,
                       cmv[i], refp[i], refs[i], b->filter,
                       0, f->bw * 4, 0, f->bh * 4);
            }
            if (bacp) {
                const int x0 = (t->bx + x) * 4 + (cmv[0].x >> (3 + refine));
                const int y0 = (t->by + y) * 4 + (cmv[0].y >> (3 + refine));
                const int x1 = (t->bx + x) * 4 + (cmv[1].x >> (3 + refine));
                const int y1 = (t->by + y) * 4 + (cmv[1].y >> (3 + refine));
                if (x0 < 0 || x1 < 0 || y0 < 0 || y1 < 0 ||
                    x0 + step * 4 >= w || x1 + step * 4 >= w ||
                    y0 + step * 4 >= h || y1 + step * 4 >= h)
                {
                    gen_mask(&mask[(y * bw4 * 4 + x) * 4], bw4 * 4,
                             step * 4, step * 4, x0, y0, x1, y1, w, h);
                    have_bacp = 1;
                }
            }
        }
        off_y += bw4 * 4 * 4 * step;
    }
    return bacp && have_bacp;
}

static int opfl_pred(Dav1dTaskContext *const t,
                     // we don't actually fill dst with any useful data, but
                     // need it as one of our scratch buffers
                     pixel *const p0, const ptrdiff_t p0_stride,
                     int16_t (*const tmp)[128 * 128], const Av1Block *const b,
                     const int bw4, const int bh4, const int w4, const int h4)
{
    const Dav1dFrameContext *const f = t->f;
    assert(!f->svc[b->ref[0]][0].scale && !f->svc[b->ref[1]][0].scale);
    const int refine = b->comp_type == COMP_INTER_AVG && b->refine_mv;
    const int opfl = b->inter_mode >= OPFL_NEARMV_NEARMV;
    assert(opfl || refine);
    assert(bw4 >= 2 && bh4 >= 2);
    const int w = f->bw * 4, h = f->bh * 4;
    pixel *const p1 = bitfn(t->scratch.interintra);
    const ptrdiff_t p1_stride = ((bw4 + refine * 2) * 4 * sizeof(pixel) + 63) & ~63;

    const Dav1dThreadPicture *refp[2] = { &f->refp[b->ref[0]],
                                          &f->refp[b->ref[1]] };
    pixel *p[2] = { p0, p1 };
    const ptrdiff_t p_stride[2] = { p0_stride, p1_stride };
    int top[2] = { t->by * 4 + (b->mv[0].y >> 3) - 3,
                   t->by * 4 + (b->mv[1].y >> 3) - 3 };

    // FIXME namespace bacp symbols
    uint8_t *const mask = t->scratch.seg_mask;
    const int bacp = f->seq_hdr->imp_msk_bld && b->cwp_idx == 8;
    if (bacp) memset(mask, 0x20, bw4 * bh4 * 16);
    int have_bacp = 0;

    // FIXME namespace opfl symbols
    // find reduced distance as inverse weights
    const int d0 = f->absrefdist[b->ref[0]], d1 = f->absrefdist[b->ref[1]];
    const int8_t d[2] = {
        apply_sign(1 + (d0 > d1), -f->refdist[b->ref[0]]),
        apply_sign(1 + (d1 > d0), -f->refdist[b->ref[1]]),
    };
    const int bs = 2 - (b->bs == BS_8x8 /* FIXME not tip */);
    const ptrdiff_t opfl_stride = bw4 >> (bs == 2);

    const int sh4 = imin(4, bh4), sw4 = imin(4, bw4);
    for (int y = 0; y < h4; y += sh4) {
        int left[2] = { t->bx * 4 + (b->mv[0].x >> 3) - 3,
                        t->bx * 4 + (b->mv[1].x >> 3) - 3 };
        if (refine) {
            for (int x = 0; x < w4; x += sw4) {
                for (int n = 0; n < 2; n++)
                    mc(t, p[n], NULL, p_stride[n], sw4 + 2, sh4 + 2,
                       t->bx + x, t->by + y, 0,
                       (union mv) { .y = b->mv[n].y - 32, .x = b->mv[n].x - 32 },
                       refp[n], b->ref[n], DAV1D_FILTER_BILINEAR,
                       iclip(left[n], 0, w - 1), iclip(left[n] + 4 * sw4 + 7, 1, w),
                       iclip(top[n], 0, h - 1), iclip(top[n] + 4 * sh4 + 7, 1, h));
                struct OpflOffset o;
                f->dsp->mc.sad_refine_mv(p0, p0_stride, p1, p1_stride,
                                         sw4 * 4, sh4 * 4, b->refine_mv == 2,
                                         &o HIGHBD_CALL_SUFFIX);
                const int dy = o.y, dx = o.x;
                union OpflMvDeltaBlock *delta_line =
                    &t->opfl[(y >> 1) * opfl_stride + (x >> 1)];
                if (opfl) {
                    struct OpflRegressionData res[2 * 2];
                    // subpel-gradient based mv refinement (optical flow = opfl)
                    f->dsp->mc.opfl_derive_mv(res,
                                              &p0[(4 + dy) * PXSTRIDE(p0_stride) +
                                                  (4 + dx)], p0_stride,
                                              &p1[(4 - dy) * PXSTRIDE(p1_stride) +
                                                  (4 - dx)], p1_stride,
                                              sw4 * 4, sh4 * 4, bs * 4, d
                                              HIGHBD_CALL_SUFFIX);
                    const struct OpflRegressionData *r = res;
                    for (int by = 0; by < sh4; by += 2, delta_line += opfl_stride) {
                        union OpflMvDeltaBlock *dd = delta_line;
                        for (int bx = 0; bx < sw4; bx += 2, dd++, r++) {
                            opfl_mv_adj(r, dd, d);
                            const union mv mv[2] = {
                                [0] = { .y = b->mv[0].y * 2 + dd->d[0].y + dy * 16,
                                        .x = b->mv[0].x * 2 + dd->d[0].x + dx * 16 },
                                [1] = { .y = b->mv[1].y * 2 + dd->d[1].y - dy * 16,
                                        .x = b->mv[1].x * 2 + dd->d[1].x - dx * 16 },
                            };
                            for (int i = 0; i < 2; i++)
                                mc_opfl(t, &tmp[i][((y + by) * bw4 * 4 + x + bx) * 4],
                                        bw4 * 4, bs, bs, t->bx + x + bx, t->by + y + by,
                                        mv[i], refp[i],
                                        iclip(left[i], 0, w - 1),
                                        iclip(left[i] + sw4 * 4 + 7, 1, w),
                                        iclip(top[i], 0, h - 1),
                                        iclip(top[i] + sh4 * 4 + 7, 1, h));
                            dd->d[0].x = ((dd->d[0].x + (dd->d[0].x > 0)) >> 1) + dx * 8;
                            dd->d[0].y = ((dd->d[0].y + (dd->d[0].y > 0)) >> 1) + dy * 8;
                            dd->d[1].x = ((dd->d[1].x + (dd->d[1].x > 0)) >> 1) - dx * 8;
                            dd->d[1].y = ((dd->d[1].y + (dd->d[1].y > 0)) >> 1) - dy * 8;
                            if (bacp) {
                                const int x0 = (t->bx + x + bx) * 4 + (mv[0].x >> 4);
                                const int x1 = (t->bx + x + bx) * 4 + (mv[1].x >> 4);
                                const int y0 = (t->by + y + by) * 4 + (mv[0].y >> 4);
                                const int y1 = (t->by + y + by) * 4 + (mv[1].y >> 4);
                                if (x0 < 0 || x1 < 0 || y0 < 0 || y1 < 0 ||
                                    x0 + 8 >= w || x1 + 8 >= w ||
                                    y0 + 8 >= h || y1 + 8 >= h)
                                {
                                    gen_mask(&mask[((y + by) * bw4 * 4 + x + bx) * 4],
                                             bw4 * 4, 8, 8, x0, y0, x1, y1, w, h);
                                    have_bacp = 1;
                                }
                            }
                        }
                    }
                } else {
                    union OpflMvDeltaBlock *const dd = delta_line;
                    dd->d[0].x = +dx * 8;
                    dd->d[0].y = +dy * 8;
                    dd->d[1].x = -dx * 8;
                    dd->d[1].y = -dy * 8;
                    dd[1] = dd[opfl_stride] = dd[opfl_stride + 1] = *dd;
                    const union mv mv[2] = {
                        [0] = { .y = b->mv[0].y + dy * 8,
                                .x = b->mv[0].x + dx * 8 },
                        [1] = { .y = b->mv[1].y - dy * 8,
                                .x = b->mv[1].x - dx * 8 },
                    };
                    for (int i = 0; i < 2; i++)
                        mc(t, NULL, &tmp[i][(y * 4 * bw4 + x) * 4], bw4 * 4,
                           sw4, sh4, t->bx + x, t->by + y, 0,
                           mv[i], refp[i], b->ref[i], b->filter,
                           iclip(left[i], 0, w - 1),
                           iclip(left[i] + sw4 * 4 + 7, 1, w),
                           iclip(top[i], 0, h - 1),
                           iclip(top[i] + sh4 * 4 + 7, 1, h));
                    if (bacp) {
                        const int x0 = (t->bx + x) * 4 + (mv[0].x >> 3);
                        const int y0 = (t->by + y) * 4 + (mv[0].y >> 3);
                        const int x1 = (t->bx + x) * 4 + (mv[1].x >> 3);
                        const int y1 = (t->by + y) * 4 + (mv[1].y >> 3);
                        if (x0 < 0 || x1 < 0 || y0 < 0 || y1 < 0 ||
                            x0 + sw4 * 4 >= w || x1 + sw4 * 4 >= w ||
                            y0 + sh4 * 4 >= h || y1 + sh4 * 4 >= h)
                        {
                            gen_mask(&mask[(y * 4 * bw4 + x) * 4], bw4 * 4,
                                     sw4 * 4, sh4 * 4, x0, y0, x1, y1, w, h);
                            have_bacp = 1;
                        }
                    }
                }
                for (int n = 0; n < 2; n++)
                    left[n] += 16;
            }
        } else {
            assert(opfl);
            for (int n = 0; n < 2; n++)
                mc(t, p[n], NULL, p_stride[n], bw4, sh4, t->bx, t->by + y,
                   0, b->mv[n], refp[n], b->ref[n], DAV1D_FILTER_BILINEAR,
                   0, w, 0, h);
            struct OpflRegressionData res[2 * 8];
            f->dsp->mc.opfl_derive_mv(res, p0, p0_stride, p1, p1_stride,
                                      bw4 * 4, sh4 * 4, bs * 4, d
                                      HIGHBD_CALL_SUFFIX);
            union OpflMvDeltaBlock *delta_line = &t->opfl[(y >> 1) * opfl_stride];
            const struct OpflRegressionData *r_line = res;
            for (int by = 0; by < sh4; by += bs) {
                union OpflMvDeltaBlock *dd = delta_line;
                const struct OpflRegressionData *r = r_line;
                for (int bx = 0; bx < w4; bx += bs, dd++, r++) {
                    opfl_mv_adj(r, dd, d);
                    const union mv mv[2] = {
                        [0] = { .y = b->mv[0].y * 2 + dd->d[0].y,
                                .x = b->mv[0].x * 2 + dd->d[0].x },
                        [1] = { .y = b->mv[1].y * 2 + dd->d[1].y,
                                .x = b->mv[1].x * 2 + dd->d[1].x },
                    };
                    for (int i = 0; i < 2; i++)
                        mc_opfl(t, &tmp[i][((y + by) * bw4 * 4 + bx) * 4],
                                bw4 * 4, bs, bs, t->bx + bx, t->by + y + by,
                                mv[i], refp[i],
                                iclip(left[i] + bx * 4, 0, w - 1),
                                iclip(left[i] + bx * 4 + 7 + 8, 1, w),
                                iclip(top[i] + by * 4, 0, h - 1),
                                iclip(top[i] + by * 4 + 7 + 8, 1, h));
                    if (bs > 1) {
                        dd->d[0].x = (dd->d[0].x + (dd->d[0].x > 0)) >> 1;
                        dd->d[0].y = (dd->d[0].y + (dd->d[0].y > 0)) >> 1;
                        dd->d[1].x = (dd->d[1].x + (dd->d[1].x > 0)) >> 1;
                        dd->d[1].y = (dd->d[1].y + (dd->d[1].y > 0)) >> 1;
                    }
                    if (bacp) {
                        const int x0 = (t->bx + bx) * 4 + (mv[0].x >> 4);
                        const int x1 = (t->bx + bx) * 4 + (mv[1].x >> 4);
                        const int y0 = (t->by + y + by) * 4 + (mv[0].y >> 4);
                        const int y1 = (t->by + y + by) * 4 + (mv[1].y >> 4);
                        if (x0 < 0 || x1 < 0 || y0 < 0 || y1 < 0 ||
                            x0 + bs * 4 >= w || x1 + bs * 4 >= w ||
                            y0 + bs * 4 >= h || y1 + bs * 4 >= h)
                        {
                            gen_mask(&mask[((y + by) * bw4 * 4 + bx) * 4],
                                     bw4 * 4, bs * 4, bs * 4, x0, y0, x1, y1, w, h);
                            have_bacp = 1;
                        }
                    }
                }
                delta_line += opfl_stride;
                r_line += bw4 >> (bs == 2);
            }
            if (bs == 1) {
                union OpflMvDeltaBlock *const dd = &t->opfl[0];
                dd->d[0].x = dd[0].d[0].x + dd[1].d[0].x + dd[2].d[0].x + dd[3].d[0].x;
                dd->d[0].x = (dd->d[0].x + 3 + (dd->d[0].x > 0)) >> 3;
                dd->d[0].y = dd[0].d[0].y + dd[1].d[0].y + dd[2].d[0].y + dd[3].d[0].y;
                dd->d[0].y = (dd->d[0].y + 3 + (dd->d[0].y > 0)) >> 3;
                dd->d[1].x = dd[0].d[1].x + dd[1].d[1].x + dd[2].d[1].x + dd[3].d[1].x;
                dd->d[1].x = (dd->d[1].x + 3 + (dd->d[1].x > 0)) >> 3;
                dd->d[1].y = dd[0].d[1].y + dd[1].d[1].y + dd[2].d[1].y + dd[3].d[1].y;
                dd->d[1].y = (dd->d[1].y + 3 + (dd->d[1].y > 0)) >> 3;
            }
        }
        for (int n = 0; n < 2; n++)
            top[n] += 4 * sh4;
    }

    return bacp && have_bacp;
}

static enum IntraPredMode wide_angle_remap(const TxfmInfo *const t_dim,
                                           enum IntraPredMode mode,
                                           int *const angle,
                                           const int mrl_idx)
{
    if ((unsigned) mode - 1 > VERT_LEFT_PRED - 1) return mode;

    // map directional modes
    const int mrl_adj = (mrl_idx == 1) - (mrl_idx == 2);
    *angle = av1_mode_to_angle_map[mode - 1] + *angle * 3 + mrl_adj;
    static const uint8_t thresh[] = { 61, 73, 82, 86 };
    const int rect = t_dim->lw - t_dim->lh;
    // FIXME below, we should return 180 +/- angle after mode remapping,
    // otherwise the actual intra prediction won't work correctly
    if (rect > 0) {
        assert(rect <= 4);
        if (*angle > 270 - thresh[rect - 1]){
            *angle -= 180;
            return DIAG_DOWN_LEFT_PRED;
        }
    } else if (rect < 0) {
        assert(rect >= -4);
        if (*angle < thresh[-1 - rect]) {
            *angle += 180;
            return HOR_UP_PRED;
        }
    }

    return mode;
}

static void recon_b_luma_tx(Dav1dTaskContext *const t, DB_ONLY(const int depth)
                            const enum RectTxfmSize tx, Av1Block *const b)
{
    const Dav1dFrameContext *const f = t->f;
    const Dav1dDSPContext *const dsp = f->dsp;
    Dav1dTileState *const ts = t->ts;
    const int bx4 = t->bx & 63, by4 = t->by & 63;
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
    const int tw = t_dim->w * 4, th = t_dim->h * 4;

    const enum IntraPredMode orig_y_mode = b->y_mode;
    int angle = b->y_angle;
    if (b->intra && !b->intrabc)
        b->y_mode = wide_angle_remap(t_dim, b->y_mode, &angle, b->mrl_index);

    // decode coefficients
    uint8_t cf_ctx;
    enum TxfmType txtp;
    coef *cf;
    int eob;
    int stx;
    if (b->skip_txfm) {
        cf_ctx = 0x40;
        txtp = DCT_DCT;
        eob = -1;
        stx = 0;
    } else {
        cf = bitfn(t->cf);
        eob = decode_coefs(t, DB_ONLY(depth + 1)
                           &t->a->lcoef[bx4], &t->l.lcoef[by4],
                           tx, b->bs, b, 0, cf, &txtp, &cf_ctx);
        stx = txtp >> 8;
        txtp = txtp & 0xff;
        DEBUG_BLOCK_printf("%*sPost-y_cf_blk[tx=%dx%d,txtp=%s/%s,eob=%d]: r=%d\n",
                           depth + 1, "", tw, th,
                           dav1d_tx1d_names[txtp & 7],
                           dav1d_tx1d_names[txtp >> 5],
                           eob, ts->msac.rng);
    }
    dav1d_memset_likely_pow2(&t->a->lcoef[bx4], cf_ctx,
                             imin(t_dim->w, f->bw - t->bx));
    dav1d_memset_likely_pow2(&t->l.lcoef[by4], cf_ctx,
                             imin(t_dim->h, f->bh - t->by));
    t->scratch.txtp_map[(t->by & 15) * 16 + (t->bx & 15)] = txtp & 0xff;

    pixel *dst = ((pixel *) f->cur.data[0]) +
        4 * (t->by * PXSTRIDE(f->cur.stride[0]) + t->bx);
    if (b->intra && !b->intrabc && !b->pal_sz) {
        const int sbsz = f->sb_step;
        const int mrl_idx = b->mrl_index;
        const int mrl_mul = b->multi_mrl && tx != (int) TX_4X4;
        pixel *const edge = bitfn(t->scratch.edge) + (mrl_idx ? 384 : 128);

        const int is_hv5 = (t->by > t->pb.row_start || t->bx > t->pb.col_start) &&
            (b->tx_part == TX_PARTITION_H5 || b->tx_part == TX_PARTITION_V5);
        int n_tr = 0, n_bl = 0;
        if (t->by > ts->tiling.row_start) {
            int w = imin(t_dim->w, ts->tiling.col_end - t->bx - t_dim->w);
            if (is_hv5) {
                n_tr = 0;
            } else if (!(t->by & (sbsz - 1))) {
                // top sb boundary
                n_tr = w;
            } else {
                const int end = imin((t->bx + sbsz) & ~(sbsz - 1),
                                     ts->tiling.col_end);
                w = imin(w, end - t->bx - t_dim->w);
                if (!w) {
                    // right sb or tile/frame boundary
                    n_tr = 0;
                } else {
                    const int xpos = (bx4 + t_dim->w) & 63;
                    const unsigned bits = (unsigned) (t->is_coded[0][by4 - 1] >> xpos);
                    n_tr = imin(ctz(~bits), w);
                }
            }
        }

        if (t->bx > ts->tiling.col_start) {
            const int end = imin((t->by + sbsz) & ~(sbsz - 1), ts->tiling.row_end);
            const int h = imin(t_dim->h, end - t->by - t_dim->h);
            if (is_hv5) {
                n_bl = 0;
            } else if (!h) {
                // bottom sb or tile/frame boundary
                n_bl = 0;
            } else if (!(t->bx & (sbsz - 1))) {
                // left sb boundary
                n_bl = h;
            } else {
                const uint64_t mask = 1ULL << ((bx4 - 1) & 63);
                int y;
                for (y = 0; y < h; y++) {
                    if (!(t->is_coded[0][by4 + y + t_dim->h] & mask))
                        break;
                }
                n_bl = y;
            }
        }

        const pixel *top_sb_edge = NULL;
        if (!(t->by & (f->sb_step - 1))) {
            top_sb_edge = f->ipred_edge[0];
            const int sby = t->by >> f->sb_shift;
            top_sb_edge += f->sb256w * 256 * (sby - 1);
        }
        const int apply_ibp = f->seq_hdr->ibp && tx != (enum RectTxfmSize) TX_4X4 && !mrl_idx;
        const int dip = b->dip - 1;
        const int sm_top = t->pb.a_is_sm;
        const int sm_left = t->pb.l_is_sm;
        const int is_sm_flag = apply_ibp ?
            ((sm_top * ANGLE_SMOOTH_TOP_EDGE_FLAG) |
             (sm_left * ANGLE_SMOOTH_LEFT_EDGE_FLAG)) :
                (sm_top | sm_left) *
                    (ANGLE_SMOOTH_TOP_EDGE_FLAG | ANGLE_SMOOTH_LEFT_EDGE_FLAG);
        int intra_flags = ANGLE_IS_LUMA | is_sm_flag |
            (f->seq_hdr->intra_edge_filter ? ANGLE_USE_EDGE_FILTER_FLAG : 0) |
            (apply_ibp ? ANGLE_IBP_FLAG : 0) |
            (mrl_idx << ANGLE_MRL_IDX_SHIFT) |
            (mrl_mul ? ANGLE_MULTI_MRL_FLAG : 0) |
            ((t->bx > ts->tiling.col_start) ? ANGLE_HAS_LEFT_FLAG : 0) |
            ((t->by > ts->tiling.row_start) ? ANGLE_HAS_TOP_FLAG  : 0) |
            (dip >= 0 ? ANGLE_DIP_FLAG : 0);
        angle = dip >= 0 ? dip : angle;
        const enum IntraPredMode m = bytefn(dav1d_prepare_intra_edges)(
            DB_ONLY(BLOCK_TO_DEBUG && DEBUG_B_PIXELS) t->bx, t->by,
            ts->tiling.col_end, ts->tiling.row_end, n_tr, n_bl, dst,
            f->cur.stride[0], top_sb_edge, b->y_mode, &angle,
            t_dim->w, t_dim->h, intra_flags, edge HIGHBD_CALL_SUFFIX);
        // FIXME this is a hack so that we fill in the edges orip needs,
        // but we normally might not fill as the predictor itself might not
        // need them
        if (b->y_angle & 1) intra_flags &= ~ANGLE_IBP_FLAG;

        dsp->ipred.intra_pred[m](dst, f->cur.stride[0],
                                 edge, tw, th, angle | intra_flags,
                                 4 * f->bw - 4 * t->bx,
                                 4 * f->bh - 4 * t->by
                                 HIGHBD_CALL_SUFFIX);

        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
            hex_dump(dst, f->cur.stride[0], tw, th, "y-intra-pred");
        }

        // XXX fix m to y_mode
        const int has_orip = !mrl_idx && tx && (
            m == VERT_PRED ? t_dim->w < 8 : m == HOR_PRED ? t_dim->h < 8 :
                m == SMOOTH_PRED && t_dim->w < 8 && t_dim->h < 8);
        if (has_orip) {
            const unsigned th_mask = ((m == VERT_PRED) << 1) | (m == HOR_PRED);
            dsp->ipred.orip(dst, f->cur.stride[0], edge, th_mask,
                            tw, th HIGHBD_CALL_SUFFIX);

            if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                hex_dump(dst, f->cur.stride[0], tw, th, "orip");
        }
    }

    if (eob != -1) {
        if (stx) {
            if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                coef_dump(cf, 8, 8, 3, "dq");
            }
            const int mask = (1 << HOR_PRED)       | (1 << HOR_DOWN_PRED) |
                             (1 << VERT_LEFT_PRED) | (1 << SMOOTH_H_PRED);
            const int transpose = b->intrabc || !b->intra || !((mask >> b->y_mode) & 1);
            const int type = (stx & 3) - 1;
            const int set = (stx >> 2) & 15;
            if (tw >= 8 && th >= 8) {
                const int8_t *kernel = &stx_8x8_kernel[set][type][0][0];
                coef sums[48];
                dsp->stx.stxfm(sums, cf, kernel, 48, eob HIGHBD_CALL_SUFFIX);
                memset(cf, 0, 32 * sizeof(coef));
                // Subtract 1 to map {8,16,32} to idx {0,1,2}
                const int idx = imin(t_dim->lh, 3) - 1;
                const uint8_t *scan_out = stx_scan_orders_8x8[idx][transpose];
                const uint8_t *mapping = coeff8x8_mapping[set * 3 + type];
                for (int x = 0; x < 48; x++) {
                    cf[scan_out[mapping[x]]] = sums[x];
                }
                eob = (uint8_t[]){ 63, 119, 231 }[idx];
            } else {
                const int8_t *kernel = &stx_4x4_kernel[set][type][0][0];
                coef sums[16];
                dsp->stx.stxfm(sums, cf, kernel, 16, eob HIGHBD_CALL_SUFFIX);
                const int idx = imin(t_dim->lh, 3);
                const uint8_t *scan_out = stx_scan_orders_4x4[idx][transpose];
                memset(&cf[4], 0, 4 * sizeof(coef));
                for (int x = 0; x < 16; x++) {
                    cf[scan_out[x]] = sums[x];
                }
                eob = (uint8_t[]){ 15, 15, 51, 99 }[idx];
            }
            if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                coef_dump(cf, imin(t_dim->h, 8) * 4,
                          imin(t_dim->w, 8) * 4, 3, "stx");
            }
        } else {
            if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                coef_dump(cf, imin(t_dim->h, 8) * 4,
                          imin(t_dim->w, 8) * 4, 3, "dq");
            }
        }
        if (f->seq_hdr->inter_ddt && !b->intra)
            txtp += txtp & dav1d_tx_ddt_mask[tx]; // (flip)adst -> (f)ddt
        dsp->itx.itxfm_add[tx](dst, f->cur.stride[0],
                               cf, txtp, eob HIGHBD_CALL_SUFFIX);
        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
            hex_dump(dst, f->cur.stride[0], t_dim->w * 4, t_dim->h * 4, "recon");
        }
    }

    const uint64_t mask = ((1ULL << t_dim->w) - 1) << bx4;
    for (int y = 0; y < t_dim->h; y++) {
        t->is_coded[0][by4 + y] |= mask;
    }

    b->y_mode = orig_y_mode;
}

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
            alpha = shift0 < 0 ? max :
                imin((dav1d_div_recip[f_d] * f_n) >> shift0, max);
            if (!alpha) return 1 << 8;
            alpha = apply_sign(alpha, num);
        }
    }
    return alpha;
}

static void bawp(Dav1dTaskContext *const t,
                 const int bawp_idx, const union mv mv,
                 pixel *const dst, const ptrdiff_t stride,
                 const Dav1dThreadPicture *const refp, const int refidx,
                 const int bw4, const int bh4, const int w4, const int h4)
{
    Dav1dTileState *const ts = t->ts;
    const Dav1dFrameContext *const f = t->f;
    const Dav1dDSPContext *const dsp = f->dsp;

    const int tile_top_edge = ts->tiling.row_start * 4;
    const int tile_left_edge = ts->tiling.col_start * 4;
    const int mvx = (mv.x + 3 + (mv.x >= 0)) >> 3;
    const int mvy = (mv.y + 3 + (mv.y >= 0)) >> 3;
    const int ref_y = (t->by * 4 + mvy);
    const int ref_x = (t->bx * 4 + mvx);
    const int ref_tmplt_x = ref_x - 1;
    const int ref_tmplt_y = ref_y - 1;
    const int ref_bottom_edge = ref_y + h4 * 4;
    const int ref_right_edge = ref_x + w4 * 4;
    const int tile_bottom_edge = ts->tiling.row_end * 4;
    const int tile_right_edge = ts->tiling.col_end * 4;

    const int can_morph =
        ref_bottom_edge <= tile_bottom_edge &&
        ref_right_edge <= tile_right_edge &&
        ref_tmplt_y >= tile_top_edge && ref_tmplt_x >= tile_left_edge;
    if (!can_morph) return;

    // TODO (optimization): Consider moving this code (and associated
    // size lookup tables) to a DSP function. SIMD could specialize on
    // edge sizes (4/8/16/32/64) and step values.
    static const uint8_t n_edge_samples[3 /* have edges */][3 /* h */]
                                       [3 /* w */][2 /* above, left */] = {
        { // !have_above && have_left
            { { 0, 2 }, { 0, 2 }, { 0, 2 } },
            { { 0, 3 }, { 0, 3 }, { 0, 3 } },
            { { 0, 4 }, { 0, 4 }, { 0, 4 } },
        }, { // have_above && !have_left
            { { 2, 0 }, { 3, 0 }, { 4, 0 } },
            { { 2, 0 }, { 3, 0 }, { 4, 0 } },
            { { 2, 0 }, { 3, 0 }, { 4, 0 } },
        }, { // have_above && have_left
            { { 2, 2 }, { 2, 2 }, { 4, 0 } },
            { { 2, 2 }, { 3, 3 }, { 3, 3 } },
            { { 0, 4 }, { 3, 3 }, { 4, 4 } },
        }
    };
    const int have_left = t->bx > ts->tiling.col_start;
    const int have_above = t->by > ts->tiling.row_start;
    if (!have_left && !have_above) return;
    const int lw4 = imin(ulog2(w4), 2), lh4 = imin(ulog2(h4), 2);
    const int idx = ((have_above << 1) | have_left) - 1;
    const int n_above_l2 = n_edge_samples[idx][lh4][lw4][0];
    const int n_left_l2 = n_edge_samples[idx][lh4][lw4][1];

    const pixel *const ref =
        &((const pixel *) refp->p.data[0])[ref_y * PXSTRIDE(refp->p.stride[0]) +
                                           ref_x];

    assert(n_above_l2 == 0 || n_left_l2 == 0 || n_above_l2 == n_left_l2);
    const int count_l2 =
        n_above_l2 + (n_above_l2 == n_left_l2 ? 1 : n_left_l2);
    int sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    if (n_above_l2) {
        const int bw = 4 << lw4;
        const int step = bw >> n_above_l2;
        assert(step > 0);
        const int start = step >> 1;
        for (int i = start; i < bw; i += step) {
            const int x = ref[i - PXSTRIDE(refp->p.stride[0])];
            const int y = dst[i - PXSTRIDE(f->cur.stride[0])];
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }
    }

    if (n_left_l2) {
        const int bh = 4 << lh4;
        const int step = bh >> n_left_l2;
        assert(step > 0);
        const int start = step >> 1;
        for (int i = start; i < bh; i += step) {
            const int x = ref[(i * PXSTRIDE(refp->p.stride[0])) - 1];
            const int y = dst[(i * PXSTRIDE(f->cur.stride[0])) - 1];
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }
    }
    assert(count_l2);

    int alpha, beta;
    if (bawp_idx == 1) {
        const int num = sum_xy - (int)(((int64_t)sum_x * sum_y) >> count_l2);
        const int den = sum_x2 - (int)(((int64_t)sum_x * sum_x) >> count_l2);
        alpha = derive_alpha(num, den, 256);
    } else {
        assert(bawp_idx & 2);
        const int idx = (1 + (bawp_idx >> 2) + (f->absrefdist[refidx] > 4)) *
                         (bawp_idx & 1 ? 1 : -1);
        alpha = 256 + 16 * idx;
    }

    const int diff = (sum_y << 8) - sum_x * alpha;
    const int abs_diff = abs(diff);
    beta = apply_sign(abs_diff >> count_l2, diff);

    dsp->mc.morph(dst, f->cur.stride[0], alpha, beta,
                  bw4 * 4, bh4 * 4 HIGHBD_CALL_SUFFIX);
}

void bytefn(dav1d_recon_b)(Dav1dTaskContext *const t,
                           DB_ONLY(const int depth)
                           const enum BlockSize lbs,
                           const enum BlockSize cbs,
                           Av1Block *const b)
{
#if 1
    Dav1dTileState *const ts = t->ts;
    const Dav1dFrameContext *const f = t->f;
    const Dav1dDSPContext *const dsp = f->dsp;
    const enum BlockSize bs = lbs == BS_INVALID ? cbs : lbs;
    assert(bs != BS_INVALID);
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
    const int ss_hor = f->ss_hor, ss_ver = f->ss_ver;
    const uint8_t csplit[6][3] = {
        [BS_256x256] = {  BS_64x64, BS_128x64, BS_128x128 },
        [BS_256x128] = {  BS_64x64, BS_128x64, BS_128x128 },
        [BS_128x256] = {  BS_64x64, BS_128x64, BS_128x128 },
        [BS_128x128] = {  BS_64x64, BS_128x64, BS_128x128 },
        [BS_128x64]  = {  BS_64x64, BS_128x64, BS_128x64  },
        [BS_64x128]  = {  BS_64x64, BS_64x64,  BS_64x128  },
    };
    if (imax(bw4, bh4) > 16) {
        assert(bw4 * 2 >= bh4 && bh4 * 2 >= bw4); // 1:2, 1:1 or 2:1 ratios only
        assert(t->cbx == t->bx && t->cby == t->by);
        const int y_start = t->by, y_end = imin(y_start + bh4, f->bh);
        const int x_start = t->bx, x_end = imin(x_start + bw4, f->bw);
        int step;
        enum BlockSize lbs2, cbs2i;
        if (imax(bw4, bh4) == 64) {
            step = 32;
            lbs2 = lbs == BS_INVALID ? BS_INVALID : BS_128x128;
            cbs2i = cbs == BS_INVALID ? BS_INVALID : BS_128x128;
        } else {
            step = 16;
            lbs2 = lbs == BS_INVALID ? BS_INVALID : BS_64x64;
            cbs2i = cbs == BS_INVALID ? BS_INVALID : csplit[cbs][ss_hor + ss_ver];
        }
        for (int y = 0; t->by < y_end; t->cby = t->by += step, y++) {
            for (int x = 0; t->bx < x_end; t->cbx = t->bx += step, x++) {
                // FIXME it's possible we can call directly into a sub-function
                // here that manages one transform-block, since tx_part=none
                // (at least if not lossless)
                const enum BlockSize cbs2 = step == 32 ||
                    !((x & ss_hor) | (y & ss_ver)) ? cbs2i : BS_INVALID;
                bytefn(dav1d_recon_b)(t, DB_ONLY(depth) lbs2, cbs2, b);
                // FIXME this may be correct only for luma, whereas chroma may
                // have to be dealt with at 64x64 *subsampled* pixels (i.e.
                // 128x128 luma pixels for 4:2:0), b/c of chroma-large-tx
            }
            t->cbx = t->bx = x_start;
        }
        t->cby = t->by = y_start;
        return;
    }
    // FIXME lossless handling (i.e. where one prediction block contains
    // multiple transform blocks

    if (lbs == BS_INVALID) goto chroma;

    const int8_t *const tp = dav1d_tx_part_tbl[bs];
    // FIXME do error reporting, to shortcut further decoding
    if (tp[b->tx_part] == -1) return;

    pixel *const dst = ((pixel *) f->cur.data[0]) +
                           4 * (t->by * PXSTRIDE(f->cur.stride[0]) + t->bx);
    if (b->intrabc) {
        mc(t, dst, NULL, f->cur.stride[0], bw4, bh4, t->bx, t->by, 0,
           b->mv[0], &f->sr_cur, 0 /* unused */, DAV1D_FILTER_BILINEAR,
           0, f->bw * 4, 0, f->bh * 4);
        if (b->morph_pred)
            bawp(t, 1, b->mv[0], dst, f->cur.stride[0],
                 &f->sr_cur, 0 /* unused */, bw4, bh4, w4, h4);
        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
            hex_dump(dst, f->cur.stride[0], bw4 * 4, bh4 * 4, "y-pred");
        }
    } else if (!b->intra) {
        if (b->ref[1] == -1 && b->ref[0] != TIP_FRAME) {
            const Dav1dThreadPicture *const refp = &f->refp[b->ref[0]];
            if ((b->inter_mode == GLOBALMV && f->gmv_warp_allowed[b->ref[0]]) ||
                (b->motion_mode >= MM_WARP_CAUSAL &&
                 t->warpmv[0].type > DAV1D_WM_TYPE_INVALID))
            {
                warp_affine(t, dst, NULL, f->cur.stride[0], b_dim, 0, refp,
                            b->motion_mode >= MM_WARP_CAUSAL ? &t->warpmv[0] :
                                &f->frame_hdr->gmv[b->ref[0]]);
            } else {
                mc(t, dst, NULL, f->cur.stride[0], bw4, bh4,
                   t->bx, t->by, 0, b->mv[0], refp, b->ref[0], b->filter,
                   0, f->bw * 4, 0, f->bh * 4);
            }
            if (b->bawp[0]) {
                bawp(t, b->bawp[0], b->mv[0], dst, f->cur.stride[0],
                     refp, b->ref[0], bw4, bh4, w4, h4);
            } else if (b->motion_mode == MM_INTERINTRA || b->warp_ii) {
                pixel *const tl_edge = bitfn(t->scratch.edge) + 32;
                enum IntraPredMode m = b->interintra_mode == II_SMOOTH_PRED ?
                                       SMOOTH_PRED : b->interintra_mode;
                pixel *const tmp = bitfn(t->scratch.interintra);
                int angle = (const uint8_t[4]) { 0, 90, 180, 0 }[b->interintra_mode];
                int n_tr = 0, n_bl = 0;
                const int bx4 = t->bx & 63, by4 = t->by & 63, sbsz = f->sb_step;
                if (t->by > ts->tiling.row_start) {
                    int w = imin(bw4, ts->tiling.col_end - t->bx - bw4);
                    if (!(t->by & (sbsz - 1))) {
                        // top sb boundary
                        n_tr = w;
                    } else {
                        const int end = imin((t->bx + sbsz) & ~(sbsz - 1),
                                             ts->tiling.col_end);
                        w = imin(w, end - t->bx - bw4);
                        if (!w) {
                            // right sb or tile/frame boundary
                            n_tr = 0;
                        } else {
                            const int xpos = (bx4 + bw4) & 63;
                            const unsigned bits = (unsigned) (t->is_coded[0][by4 - 1] >> xpos);
                            n_tr = imin(ctz(~bits), w);
                        }
                    }
                }

                if (t->bx > ts->tiling.col_start) {
                    const int end = imin((t->by + sbsz) & ~(sbsz - 1), ts->tiling.row_end);
                    const int h = imin(bh4, end - t->by - bh4);
                    if (!h) {
                        // bottom sb or tile/frame boundary
                        n_bl = 0;
                    } else if (!(t->bx & (sbsz - 1))) {
                        // left sb boundary
                        n_bl = h;
                    } else {
                        const uint64_t mask = 1ULL << ((bx4 - 1) & 63);
                        int y;
                        for (y = 0; y < h; y++) {
                            if (!(t->is_coded[0][by4 + y + bh4] & mask))
                                break;
                        }
                        n_bl = y;
                    }
                }
                const pixel *top_sb_edge = NULL;
                if (!(t->by & (f->sb_step - 1))) {
                    top_sb_edge = f->ipred_edge[0];
                    const int sby = t->by >> f->sb_shift;
                    top_sb_edge += f->sb256w * 256 * (sby - 1);
                }
                const int intra_flags = ANGLE_IBP_FLAG /* for dc; or orip */ |
                    ((t->bx > ts->tiling.col_start) ? ANGLE_HAS_LEFT_FLAG : 0) |
                    ((t->by > ts->tiling.row_start) ? ANGLE_HAS_TOP_FLAG  : 0);
                m = bytefn(dav1d_prepare_intra_edges)(
                        DB_ONLY(BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                        t->bx, t->by, ts->tiling.col_end, ts->tiling.row_end,
                        n_tr, n_bl, dst, f->cur.stride[0], top_sb_edge, m, &angle,
                        bw4, bh4, intra_flags, tl_edge HIGHBD_CALL_SUFFIX);
                dsp->ipred.intra_pred[m](tmp, 4 * bw4 * sizeof(pixel),
                                         tl_edge, bw4 * 4, bh4 * 4,
                                         intra_flags, 0, 0 HIGHBD_CALL_SUFFIX);
                if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                    hex_dump(tmp, bw4 * 4 * sizeof(pixel),
                             bw4 * 4, bh4 * 4, "y-intra-pred");
                }
                const int has_orip = bs != BS_4x4 && (
                    m == VERT_PRED ? bw4 < 8 : m == HOR_PRED ? bh4 < 8 :
                    m == SMOOTH_PRED && bw4 < 8 && bh4 < 8);
                if (has_orip) {
                    const unsigned th_mask =
                        ((m == VERT_PRED) << 1) | (m == HOR_PRED);
                    dsp->ipred.orip(tmp, 4 * bw4 * sizeof(pixel), tl_edge, th_mask,
                                    bw4 * 4, bh4 * 4 HIGHBD_CALL_SUFFIX);

                    if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                        hex_dump(tmp, 4 * bw4 * sizeof(pixel),
                                 bw4 * 4, bh4 * 4, "orip");
                }
                const uint8_t *const mask = b->wedge_idx == -1 ?
                    II_MASK(bs, bw4, bh4, b->interintra_mode) :
                    WEDGE_MASK(bs, bw4, bh4, b->wedge_idx);
                if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                    hex_dump(dst, f->cur.stride[0],
                             bw4 * 4, bh4 * 4, "y-inter-pred");
                }
                dsp->mc.blend(dst, f->cur.stride[0], tmp,
                              bw4 * 4, bh4 * 4, mask);
            }
        } else {
            int16_t (*const tmp)[128 * 128] = t->scratch.compinter;
            int bacp;

            if (b->ref[0] == TIP_FRAME) {
                bacp = tip_pred(t, dst, f->cur.stride[0], tmp, b, bw4, bh4, w4, h4);
                if (bacp < 0) return;
            } else if (b->inter_mode >= OPFL_NEARMV_NEARMV ||
                       (b->refine_mv && b->comp_type == COMP_INTER_AVG))
            {
                bacp = opfl_pred(t, dst, f->cur.stride[0], tmp, b, bw4, bh4, w4, h4);
                if (bacp < 0) return;
            } else {
                bacp = 2 * (f->seq_hdr->imp_msk_bld &&
                            b->motion_mode != MM_WARP_CAUSAL &&
                            b->inter_mode != GLOBALMV_GLOBALMV &&
                            !f->svc[b->ref[0]][0].scale && !f->svc[b->ref[1]][0].scale);
                for (int i = 0; i < 2; i++) {
                    const Dav1dThreadPicture *const refp = &f->refp[b->ref[i]];

                    if ((b->inter_mode == GLOBALMV_GLOBALMV &&
                         f->gmv_warp_allowed[b->ref[i]]) ||
                        (b->motion_mode == MM_WARP_CAUSAL &&
                         t->warpmv[i].type > DAV1D_WM_TYPE_INVALID))
                    {
                        warp_affine(t, NULL, tmp[i], bw4 * 4, b_dim, 0, refp,
                                    b->motion_mode >= MM_WARP_CAUSAL ?
                                        &t->warpmv[i] : &f->frame_hdr->gmv[b->ref[i]]);
                    } else {
                        mc(t, NULL, tmp[i], bw4 * 4, bw4, bh4, t->bx, t->by, 0,
                           b->mv[i], refp, b->ref[i], b->filter,
                           0, f->bw * 4, 0, f->bh * 4);
                    }
                    if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                        ac_dump(tmp[i], bw4 * 4, bh4 * 4, "y-single-pred");
                }
            }
            switch (b->comp_type) {
            case COMP_INTER_WEDGE: {
                const uint8_t *const mask =
                    WEDGE_MASK(bs, bw4, bh4, b->wedge_idx);
                dsp->mc.mask(dst, f->cur.stride[0],
                             tmp[b->wedge_sign], tmp[!b->wedge_sign],
                             bw4 * 4, bh4 * 4, mask HIGHBD_CALL_SUFFIX);
                break;
            }
            case COMP_INTER_SEG: {
                const int chr_layout_idx =
                    f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I400 ? 0 :
                    DAV1D_PIXEL_LAYOUT_I444 - f->cur.p.layout;
                uint8_t *const seg_mask = t->scratch.seg_mask;
                dsp->mc.w_mask[chr_layout_idx](dst, f->cur.stride[0],
                                               tmp[b->mask_sign], tmp[!b->mask_sign],
                                               bw4 * 4, bh4 * 4, seg_mask,
                                               b->mask_sign HIGHBD_CALL_SUFFIX);
                break;
            }
            default: assert(0);
            case COMP_INTER_NONE:
                assert(b->ref[0] == TIP_FRAME);
                // fall-through
            case COMP_INTER_AVG: {
                const int wt = b->cwp_idx;
                if (wt == 8) {
                    int y0, y1, x0, x1, w, h;
                    if (bacp == 2) {
                        w = f->bw * 4;
                        h = f->bh * 4;
                        x0 = t->bx * 4 + (b->mv[0].x >> 3);
                        y0 = t->by * 4 + (b->mv[0].y >> 3);
                        x1 = t->bx * 4 + (b->mv[1].x >> 3);
                        y1 = t->by * 4 + (b->mv[1].y >> 3);
                        bacp = x0 < 0 || x1 < 0 || y0 < 0 || y1 < 0 ||
                               x0 + bw4 * 4 >= w || x1 + bw4 * 4 >= w ||
                               y0 + bh4 * 4 >= h || y1 + bh4 * 4 >= h;
                        if (bacp)
                            gen_mask(t->scratch.seg_mask, bw4 * 4, bw4 * 4, bh4 * 4,
                                     x0, y0, x1, y1, w, h);
                    }
                    if (bacp) {
                        dsp->mc.mask(dst, f->cur.stride[0], tmp[0], tmp[1],
                                     bw4 * 4, bh4 * 4, t->scratch.seg_mask
                                     HIGHBD_CALL_SUFFIX);
                    } else {
                        dsp->mc.avg(dst, f->cur.stride[0], tmp[0], tmp[1],
                                    bw4 * 4, bh4 * 4 HIGHBD_CALL_SUFFIX);
                    }
                } else {
                    dsp->mc.w_avg(dst, f->cur.stride[0], tmp[0], tmp[1],
                                  bw4 * 4, bh4 * 4, wt HIGHBD_CALL_SUFFIX);
                }
                break;
            }}
        }
        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
            hex_dump(dst, f->cur.stride[0], bw4 * 4, bh4 * 4, "y-pred");
    } else if (b->pal_sz) {
        const uint8_t *pal_idx;
        if (t->frame_thread.pass) {
            const int p = t->frame_thread.pass & 1;
            assert(ts->frame_thread[p].pal_idx);
            pal_idx = ts->frame_thread[p].pal_idx;
            ts->frame_thread[p].pal_idx += bw4 * bh4 * 8;
        } else {
            pal_idx = t->scratch.pal_idx_y;
        }
        const pixel *const pal = t->frame_thread.pass ?
            f->frame_thread.pal[((t->by >> 1) + (t->bx & 1)) * (f->b4_stride >> 1) +
                                ((t->bx >> 1) + (t->by & 1))] :
            bytefn(t->scratch.pal);
        f->dsp->ipred.pal_pred(dst, f->cur.stride[0], pal,
                               pal_idx, bw4 * 4, bh4 * 4);
        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
            hex_dump(dst, f->cur.stride[0], bw4 * 4, bh4 * 4, "y-pal-pred");
    }

    // luma
    const enum RectTxfmSize tx = tp[b->tx_part];
    t->pb.col_start = t->bx;
    t->pb.row_start = t->by;
    switch (b->tx_part) {
    case TX_PARTITION_NONE:
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        break;
    case TX_PARTITION_SPLIT: {
        const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
        const int tw4 = t_dim->w, th4 = t_dim->h;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        const int have_v_split = t->bx + tw4 < f->bw;
        if (have_v_split) {
            t->bx += tw4;
            recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4;
        }
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (have_v_split) {
            t->bx += tw4;
            recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4;
        }
        t->by -= th4;
        break;
    }
    case TX_PARTITION_H: {
        const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
        const int th4 = t_dim->h;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        t->by -= th4;
        break;
    }
    case TX_PARTITION_V: {
        const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
        const int tw4 = t_dim->w;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (t->bx + tw4 >= f->bw) break;
        t->bx += tw4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        t->bx -= tw4;
        break;
    }
    case TX_PARTITION_H4: {
        const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
        const int th4 = t_dim->h;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (t->by + th4 >= f->bh) { t->by -= th4; break; }
        t->by += th4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (t->by + th4 >= f->bh) { t->by -= 2 * th4; break; }
        t->by += th4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        t->by -= 3 * th4;
        break;
    }
    case TX_PARTITION_V4: {
        const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
        const int tw4 = t_dim->w;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (t->bx + tw4 >= f->bw) break;
        t->bx += tw4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (t->bx + tw4 >= f->bw) { t->bx -= tw4; break; }
        t->bx += tw4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (t->bx + tw4 >= f->bw) { t->bx -= 2 * tw4; break; }
        t->bx += tw4;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        t->bx -= 3 * tw4;
        break;
    }
    case TX_PARTITION_H5: {
        const enum RectTxfmSize tx_big = tp[TX_PARTITION_H];
        const TxfmInfo *const t_dim_small = &dav1d_txfm_dimensions[tx],
                       *const t_dim_big = &dav1d_txfm_dimensions[tx_big];
        const int tw4_small = t_dim_small->w, th4_small = t_dim_small->h;
        const int th4_big = t_dim_big->h;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        const int have_v_split = t->bx + tw4_small < f->bw;
        if (have_v_split) {
            t->bx += tw4_small;
            recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4_small;
        }
        if (t->by + th4_small >= f->bh) break;
        t->by += th4_small;
        recon_b_luma_tx(t, DB_ONLY(depth) tx_big, b);
        if (t->by + th4_big >= f->bh) { t->by -= th4_small; break; }
        t->by += th4_big;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (have_v_split) {
            t->bx += tw4_small;
            recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4_small;
        }
        t->by -= th4_small + th4_big;
        break;
    }
    case TX_PARTITION_V5: {
        const enum RectTxfmSize tx_big = tp[TX_PARTITION_V];
        const TxfmInfo *const t_dim_small = &dav1d_txfm_dimensions[tx],
                       *const t_dim_big = &dav1d_txfm_dimensions[tx_big];
        const int tw4_small = t_dim_small->w, th4_small = t_dim_small->h;
        const int tw4_big = t_dim_big->w;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        const int have_h_split = t->by + th4_small < f->bh;
        if (have_h_split) {
            t->by += th4_small;
            recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->by -= th4_small;
        }
        if (t->bx + tw4_small >= f->bw) break;
        t->bx += tw4_small;
        recon_b_luma_tx(t, DB_ONLY(depth) tx_big, b);
        if (t->bx + tw4_big >= f->bw) { t->bx -= tw4_small; break; }
        t->bx += tw4_big;
        recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (have_h_split) {
            t->by += th4_small;
            recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->by -= th4_small;
        }
        t->bx -= tw4_small + tw4_big;
        break;
    }
    default: assert(0);
    }

    refmvs_block *rb = &t->rt.r[(t->by & 63) * 128 + (t->bx & 127)];
    if (rb->mf & 4) {
        if (b->comp_type == COMP_INTER_WEDGE) {
            const uint8_t *wedge = WEDGE_TMVP(bs, bw4, bh4, b->wedge_idx);
            for (int y = 0; y < h4; y += 2) {
                for (int x = 0; x < w4; x += 2) {
                    const int d = wedge[x >> 1];
                    rb[x].tmv.mv[0].n = d ==  b->wedge_sign ?
                                        INVALID_MV : rb[x].tmv.mv[0].n;
                    rb[x].tmv.mv[1].n = d == !b->wedge_sign ?
                                        INVALID_MV : rb[x].tmv.mv[1].n;
                    rb[x + 1].tmv = rb[x].tmv;
                    rb[x + 128].tmv = rb[x].tmv;
                    rb[x + 129].tmv = rb[x].tmv;
                }
                wedge += bw4 >> 1;
                rb += 128 * 2;
            }
        } else {
            const ptrdiff_t opfl_stride = (bw4 + 1) >> 1;
            const union OpflMvDeltaBlock *opfl_dxy = t->opfl;
            const int si = b->ref[0] != TIP_FRAME;
            for (int y = 0; y < h4; y += 2) {
                for (int x = 0; x < w4; x += 2) {
                    const union OpflMvDeltaBlock *const o = &opfl_dxy[x >> 1];
                    rb[x].tmv.mv[0].x = rb[x].mv.mv[0].x  + o->d[0].x;
                    rb[x].tmv.mv[0].y = rb[x].mv.mv[0].y  + o->d[0].y;
                    rb[x].tmv.mv[1].x = rb[x].mv.mv[si].x + o->d[1].x;
                    rb[x].tmv.mv[1].y = rb[x].mv.mv[si].y + o->d[1].y;
                    if (x + 1 < w4) rb[x + 1].tmv = rb[x].tmv;
                    if (y + 1 < h4) {
                        rb[x + 128].tmv = rb[x].tmv;
                        if (x + 1 < w4) rb[x + 129].tmv = rb[x].tmv;
                    }
                }
                opfl_dxy += opfl_stride;
                rb += 128 * 2;
            }
        }
    }

    if (cbs == BS_INVALID) return;

    // chroma
chroma: {}
    uint16_t cfl_y_edge_mem[256], *const cfl_y_edge = &cfl_y_edge_mem[128];
    coef *const cf = bitfn(t->cf);
    const enum RectTxfmSize uvtx = dav1d_max_txfm_size_for_bs[cbs][f->cur.p.layout];
    const TxfmInfo *const uv_t_dim = &dav1d_txfm_dimensions[uvtx];
    const int ctw4 = imin(uv_t_dim->w, (f->bw - t->cbx + ss_hor) >> ss_hor);
    const int cth4 = imin(uv_t_dim->h, (f->bh - t->cby + ss_ver) >> ss_ver);
    const int ctw = uv_t_dim->w * 4, cth = uv_t_dim->h * 4;
    const int bx4 = t->cbx & 63, by4 = t->cby & 63;
    const int cbx4 = bx4 >> ss_hor, cby4 = by4 >> ss_ver;
    const int ssbx = t->cbx >> ss_hor, ssby = t->cby >> ss_ver;
    const ptrdiff_t stride = f->cur.stride[1];
    const int sbsz = f->sb_step;

    const enum IntraPredMode orig_uv_mode = b->uv_mode;
    int angle = b->uv_angle;
    if (b->intra && !b->intrabc)
        b->uv_mode = wide_angle_remap(uv_t_dim, b->uv_mode, &angle, 0);

    // CFL calc AC / gen Y edge
    int16_t *const ac = t->scratch.ac;
    if (b->intra && !b->intrabc &&
        b->uv_mode == CFL_PRED && b->cfl_type < CFL_MHCCP)
    {
        const int sby = t->cby >> f->sb_shift;
        const ptrdiff_t ystride = f->cur.stride[0];
        const pixel *const y_src = ((pixel *) f->cur.data[0]) +
            (t->cby * PXSTRIDE(ystride) + t->cbx) * 4;
        const int is_top_sb_edge = !(t->cby & (sbsz - 1));
        const pixel *const top = t->cby == ts->tiling.row_start ? NULL :
            !is_top_sb_edge ? y_src - (1 + ss_ver) * PXSTRIDE(ystride) :
            f->ipred_edge[0] + f->sb256w * 256 * (sby - 1);
        const pixel *const left = t->cbx > ts->tiling.col_start ? y_src - (1 + ss_hor) : NULL;
        const int filter_type = f->c->seq_hdr->cfl_ds_filter_index |
            (is_top_sb_edge ? CFL_IS_TOP_SB_EDGE : 0);

        const int cbw4 = (bw4 + ss_hor) >> ss_hor, cbh4 = (bh4 + ss_ver) >> ss_ver;
        const int wpad = uv_t_dim->w > cbw4 ? uv_t_dim->w - cbw4 : cbw4 - ctw4;
        const int hpad = uv_t_dim->h > cbh4 ? uv_t_dim->h - cbh4 : cbh4 - cth4;
        const int dc = (!top && !left) ? 4 << f->cur.p.bpc :
            dsp->ipred.cfl_dc[f->cur.p.layout - 1](cfl_y_edge, top, left, ystride,
                                                   wpad << ss_hor, hpad << ss_ver,
                                                   top ? uv_t_dim->w * 4 << ss_hor : 0,
                                                   left ? uv_t_dim->h * 4 << ss_ver : 0,
                                                   filter_type);
        dsp->ipred.cfl_ac[f->cur.p.layout - 1](ac, dc, y_src, ystride, wpad, hpad,
                                               uv_t_dim->w * 4, uv_t_dim->h * 4,
                                               filter_type);
    }

    for (int pl = 0; pl < 2; pl++) {
        // decode coefficients
        uint8_t cf_ctx;
        enum TxfmType txtp;
        int eob;
        if (b->skip_txfm) {
            eob = -1;
            cf_ctx = 0x40;
        } else {
            txtp = t->scratch.txtp_map[(t->by & 15) * 16 + (t->bx & 15)];
            eob = decode_coefs(t, DB_ONLY(depth + 1)
                                         &t->a->ccoef[pl][cbx4],
                                         &t->l.ccoef[pl][cby4], uvtx, b->bs,
                                         b, 1 + pl, cf, &txtp, &cf_ctx);
            DEBUG_BLOCK_printf("%*sPost-%c_cf_blk[tx=%dx%d,txtp=%s/%s,eob=%d]: r=%d\n",
                               depth + 1, "", "uv"[pl], uv_t_dim->w * 4,
                               uv_t_dim->h * 4,
                               dav1d_tx1d_names[txtp & 7],
                               dav1d_tx1d_names[txtp >> 5],
                               eob, t->ts->msac.rng);
        }
        dav1d_memset_likely_pow2(&t->a->ccoef[pl][cbx4], cf_ctx, ctw4);
        dav1d_memset_likely_pow2(&t->l.ccoef[pl][cby4], cf_ctx, cth4);
        pixel *dst = ((pixel *) f->cur.data[1 + pl]) +
            4 * (ssby * PXSTRIDE(stride) + ssbx);
        if (b->intra && !b->intrabc) {
            // intra prediction
            pixel *const edge = bitfn(t->scratch.edge) + 128;
            const pixel *top_sb_edge = NULL;
            if (!(t->cby & (sbsz - 1))) {
                top_sb_edge = f->ipred_edge[1 + pl];
                const int sby = t->cby >> f->sb_shift;
                top_sb_edge += (sby - 1) * f->sb256w * 256 >> ss_hor;
            }

            if (b->uv_mode == CFL_PRED &&
                (b->cfl_alpha[pl] || b->cfl_type > CFL_EXPLICIT))
            {
                int alpha = b->cfl_alpha[pl] * 32;
                if (b->cfl_type == CFL_IMPLICIT) {
                    const int have_top = t->cby > ts->tiling.row_start;
                    const int have_left = t->cbx > ts->tiling.col_start;
                    int n_top = 0, n_left = 0;
                    if (have_top && have_left) {
                        if (ctw > 2 * cth) {
                            n_top = 8;
                            n_left = 0;
                        } else if (cth > 2 * ctw) {
                            n_top = 0;
                            n_left = 8;
                        } else {
                            n_top = 4;
                            n_left = 4;
                        }
                    } else {
                        n_top = have_top ? 4 : 0;
                        n_left = have_left ? 4 : 0;
                    }

                    int sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
                    if (n_top) {
                        const pixel *const src = top_sb_edge ? top_sb_edge :
                            ((pixel *) f->cur.data[1 + pl]) +
                            (ssby * 4 - 1) * PXSTRIDE(stride) + ssbx * 4;
                        const int step = ctw >> ctz(n_top);
                        const int start = step >> 1;
                        for (int i = start; i < ctw; i += step) {
                            const int l = cfl_y_edge[i] >> 3, c = src[i];
                            sum_x += l;
                            sum_y += c;
                            sum_xx += l * l;
                            sum_xy += l * c;
                        }
                    }
                    if (n_left) {
                        const pixel *const src = ((pixel *) f->cur.data[1 + pl]) +
                            (ssby * PXSTRIDE(stride) + ssbx) * 4 - 1;
                        const int step = cth >> ctz(n_left);
                        const int start = step >> 1;
                        for (int i = start; i < cth; i += step) {
                            const int l = cfl_y_edge[-1 - i] >> 3;
                            const int c = src[i * PXSTRIDE(stride)];
                            sum_x += l;
                            sum_y += c;
                            sum_xx += l * l;
                            sum_xy += l * c;
                        }
                    }
                    const int count_l2 = ctz(n_top + n_left);
                    const int num = sum_xy - (int)(((int64_t)sum_x * sum_y) >> count_l2);
                    const int den = sum_xx - (int)(((int64_t)sum_x * sum_x) >> count_l2);
                    alpha = derive_alpha(num, den, 0);
                }
                const int intra_flags =
                    ((t->cbx > ts->tiling.col_start) ? ANGLE_HAS_LEFT_FLAG : 0) |
                    ((t->cby > ts->tiling.row_start) ? ANGLE_HAS_TOP_FLAG  : 0);
                pixel *src = ((pixel *) f->cur.data[1 + pl]) +
                    4 * (ssby * PXSTRIDE(stride) + ssbx);
                const enum IntraPredMode m = bytefn(dav1d_prepare_intra_edges)(
                    DB_ONLY(0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) ssbx, ssby,
                    ts->tiling.col_end >> ss_hor, ts->tiling.row_end >> ss_ver,
                    0, 0, src, stride, top_sb_edge, DC_PRED, NULL,
                    uv_t_dim->w, uv_t_dim->h, intra_flags, edge HIGHBD_CALL_SUFFIX);
                dsp->ipred.cfl_pred[m](dst, stride, edge,
                                       uv_t_dim->w * 4, uv_t_dim->h * 4,
                                       ac, alpha HIGHBD_CALL_SUFFIX);
                if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                    hex_dump(dst, stride, ctw, cth, pl ? "v-intra-pred" : "u-intra-pred");
                }
            } else {
                int n_tr = 0, n_bl = 0;
                if (t->cby > ts->tiling.row_start) {
                    const int csbsz = sbsz >> ss_hor;
                    const int end = imin((ssbx + csbsz) & ~(csbsz - 1),
                                         ts->tiling.col_end >> ss_hor);
                    const int w = imin(ctw4, end - ssbx - ctw4);
                    if (!(t->cby & (sbsz - 1)) || !w) {
                        // top or right sb boundary
                        n_tr = w;
                    } else {
                        const unsigned bits = (unsigned)
                            (t->is_coded[1][cby4 - 1] >> (cbx4 + ctw4));
                        n_tr = imin(ctz(~bits), w);
                    }
                }
                if (t->cbx > ts->tiling.col_start) {
                    const int csbsz = sbsz >> ss_ver;
                    const int end = imin((ssby + csbsz) & ~(csbsz - 1),
                                         ts->tiling.row_end >> ss_ver);
                    const int h = imin(cth4, end - ssby - cth4);
                    if (!(t->cbx & (sbsz - 1)) || !h) {
                        // left or bottom sb boundary
                        n_bl = h;
                    } else {
                        const uint64_t mask = 1ULL << (cbx4 - 1);
                        for (; n_bl < h; n_bl++)
                            if (!(t->is_coded[1][cby4 + n_bl + cth4] & mask))
                                break;
                    }
                }

                const int apply_ibp = f->seq_hdr->ibp &&
                    uvtx != (enum RectTxfmSize) TX_4X4 && b->uv_mode == DC_PRED;
                const int sm_top = sm_uv_flag(t->a, cbx4);
                const int sm_left = sm_uv_flag(&t->l, cby4);
                const int is_sm_flag = (sm_top | sm_left) *
                    (ANGLE_SMOOTH_TOP_EDGE_FLAG | ANGLE_SMOOTH_LEFT_EDGE_FLAG);
                int intra_flags = is_sm_flag |
                    ANGLE_IBP_FLAG |
                    (f->seq_hdr->intra_edge_filter ? ANGLE_USE_EDGE_FILTER_FLAG : 0) |
                    ((t->cbx > ts->tiling.col_start) ? ANGLE_HAS_LEFT_FLAG : 0) |
                    ((t->cby > ts->tiling.row_start) ? ANGLE_HAS_TOP_FLAG  : 0);
                const enum IntraPredMode uv_mode =
                    b->uv_mode == CFL_PRED ? DC_PRED : b->uv_mode;

                const enum IntraPredMode m = bytefn(dav1d_prepare_intra_edges)(
                    // don't print chroma as avm does things in a different order
                    // (decode coefs of both planes first then pred + itx)
                    DB_ONLY(0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) ssbx, ssby,
                    ts->tiling.col_end >> ss_hor, ts->tiling.row_end >> ss_ver,
                    n_tr, n_bl, dst, stride, top_sb_edge, uv_mode,
                    &angle, uv_t_dim->w, uv_t_dim->h, intra_flags, edge HIGHBD_CALL_SUFFIX);

                // FIXME this is a hack so that we fill in the edges orip needs,
                // but we normally might not fill as the predictor itself might
                // not need them
                if (!apply_ibp) intra_flags &= ~ANGLE_IBP_FLAG;
                dsp->ipred.intra_pred[m](dst, stride,
                                         edge, ctw, cth, angle | intra_flags,
                                         4 * f->bw - 4 * t->cbx,
                                         4 * f->bh - 4 * t->cby HIGHBD_CALL_SUFFIX);

                if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                    hex_dump(dst, stride, ctw, cth, pl ? "v-intra-pred" : "u-intra-pred");
                }
                const int has_orip = uvtx && (
                    b->uv_mode == VERT_PRED ? uv_t_dim->w < 8 : b->uv_mode == HOR_PRED ? uv_t_dim->h < 8 :
                        b->uv_mode == SMOOTH_PRED && uv_t_dim->w < 8 && uv_t_dim->h < 8);
                if (has_orip) {
                    const unsigned cth_mask = ((m == VERT_PRED) << 1) | (m == HOR_PRED);
                    dsp->ipred.orip(dst, stride, edge, cth_mask,
                                    ctw, cth HIGHBD_CALL_SUFFIX);

                    if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                        hex_dump(dst, stride, ctw, cth, "orip");
                }
            }
        }

        // inverse transform
        if (eob != -1) {
            // don't print chroma as avm does things in a different order
            // (decode coefs of both planes first then pred + itx)
            if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                coef_dump(cf, imin(uv_t_dim->w, 8) * 4,
                          imin(uv_t_dim->h, 8) * 4, 3, "dq");
            }
            dsp->itx.itxfm_add[uvtx](dst, stride, cf, txtp, eob HIGHBD_CALL_SUFFIX);
        }
        if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
            hex_dump(dst, stride, uv_t_dim->w * 4, uv_t_dim->h * 4, "recon");
        }
    }

    const uint64_t mask = ((1ULL << ctw4) - 1) << cbx4;
    for (int y = 0; y < cth4; y++)
        t->is_coded[1][cby4 + y] |= mask;

    b->uv_mode = orig_uv_mode;
#else
    Dav1dTileState *const ts = t->ts;
    const Dav1dFrameContext *const f = t->f;
    const Dav1dDSPContext *const dsp = f->dsp;
    const int bx4 = t->bx & 63, by4 = t->by & 63;
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int cbx4 = bx4 >> ss_hor, cby4 = by4 >> ss_ver;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
    const int cw4 = (w4 + ss_hor) >> ss_hor, ch4 = (h4 + ss_ver) >> ss_ver;
    const int has_chroma = 0 && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400 &&
                           (bw4 > ss_hor || t->bx & 1) &&
                           (bh4 > ss_ver || t->by & 1);
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[b->tx];
    const TxfmInfo *const uv_t_dim = &dav1d_txfm_dimensions[b->uvtx];

    // coefficient coding
    pixel *const edge = bitfn(t->scratch.edge) + 128;
    const int cbw4 = (bw4 + ss_hor) >> ss_hor, cbh4 = (bh4 + ss_ver) >> ss_ver;

    const int intra_edge_filter_flag = f->seq_hdr->intra_edge_filter << 10;

    for (int init_y = 0; init_y < h4; init_y += 16) {
        const int sub_h4 = imin(h4, 16 + init_y);
        const int sub_ch4 = imin(ch4, (init_y + 16) >> ss_ver);
        for (int init_x = 0; init_x < w4; init_x += 16) {
            if (b->pal_sz) {
                pixel *dst = ((pixel *) f->cur.data[0]) +
                             4 * (t->by * PXSTRIDE(f->cur.stride[0]) + t->bx);
                const uint8_t *pal_idx;
                if (t->frame_thread.pass) {
                    const int p = t->frame_thread.pass & 1;
                    assert(ts->frame_thread[p].pal_idx);
                    pal_idx = ts->frame_thread[p].pal_idx;
                    ts->frame_thread[p].pal_idx += bw4 * bh4 * 8;
                } else {
                    pal_idx = t->scratch.pal_idx_y;
                }
                const pixel *const pal = t->frame_thread.pass ?
                    f->frame_thread.pal[((t->by >> 1) + (t->bx & 1)) * (f->b4_stride >> 1) +
                                        ((t->bx >> 1) + (t->by & 1))][0] :
                    bytefn(t->scratch.pal)[0];
                f->dsp->ipred.pal_pred(dst, f->cur.stride[0], pal,
                                       pal_idx, bw4 * 4, bh4 * 4);
                if (DEBUG_BLOCK_INFO && DEBUG_B_PIXELS)
                    hex_dump(dst, PXSTRIDE(f->cur.stride[0]),
                             bw4 * 4, bh4 * 4, "y-pal-pred");
            }

            const int intra_flags = (sm_flag(t->a, bx4) |
                                     sm_flag(&t->l, by4) |
                                     intra_edge_filter_flag);
            const int sb_has_tr = init_x + 16 < w4 ? 1 : init_y ? 0 :
                              intra_edge_flags & EDGE_I444_TOP_HAS_RIGHT;
            const int sb_has_bl = init_x ? 0 : init_y + 16 < h4 ? 1 :
                              intra_edge_flags & EDGE_I444_LEFT_HAS_BOTTOM;
            int y, x;
            const int sub_w4 = imin(w4, init_x + 16);
            for (y = init_y, t->by += init_y; y < sub_h4;
                 y += t_dim->h, t->by += t_dim->h)
            {
                pixel *dst = ((pixel *) f->cur.data[0]) +
                               4 * (t->by * PXSTRIDE(f->cur.stride[0]) +
                                    t->bx + init_x);
                for (x = init_x, t->bx += init_x; x < sub_w4;
                     x += t_dim->w, t->bx += t_dim->w)
                {
                    if (b->pal_sz) goto skip_y_pred;

                    int angle = b->y_angle;
                    const enum EdgeFlags edge_flags =
                        (((y > init_y || !sb_has_tr) && (x + t_dim->w >= sub_w4)) ?
                             0 : EDGE_I444_TOP_HAS_RIGHT) |
                        ((x > init_x || (!sb_has_bl && y + t_dim->h >= sub_h4)) ?
                             0 : EDGE_I444_LEFT_HAS_BOTTOM);
                    const pixel *top_sb_edge = NULL;
                    if (!(t->by & (f->sb_step - 1))) {
                        top_sb_edge = f->ipred_edge[0];
                        const int sby = t->by >> f->sb_shift;
                        top_sb_edge += f->sb256w * 256 * (sby - 1);
                    }
                    const enum IntraPredMode m =
                        bytefn(dav1d_prepare_intra_edges)(t->bx,
                                                          t->bx > ts->tiling.col_start,
                                                          t->by,
                                                          t->by > ts->tiling.row_start,
                                                          ts->tiling.col_end,
                                                          ts->tiling.row_end,
                                                          edge_flags, dst,
                                                          f->cur.stride[0], top_sb_edge,
                                                          b->y_mode, &angle,
                                                          t_dim->w, t_dim->h,
                                                          f->seq_hdr->intra_edge_filter,
                                                          edge HIGHBD_CALL_SUFFIX);
                    dsp->ipred.intra_pred[m](dst, f->cur.stride[0], edge,
                                             t_dim->w * 4, t_dim->h * 4,
                                             angle | intra_flags,
                                             4 * f->bw - 4 * t->bx,
                                             4 * f->bh - 4 * t->by
                                             HIGHBD_CALL_SUFFIX);

                    if (DEBUG_BLOCK_INFO && DEBUG_B_PIXELS) {
                        hex_dump(edge - t_dim->h * 4, t_dim->h * 4,
                                 t_dim->h * 4, 2, "l");
                        hex_dump(edge, 0, 1, 1, "tl");
                        hex_dump(edge + 1, t_dim->w * 4,
                                 t_dim->w * 4, 2, "t");
                        hex_dump(dst, f->cur.stride[0],
                                 t_dim->w * 4, t_dim->h * 4, "y-intra-pred");
                    }

                skip_y_pred: {}
                    if (!b->skip_txfm) {
                        coef *cf;
                        int eob;
                        enum TxfmType txtp;
                        if (t->frame_thread.pass) {
                            const int p = t->frame_thread.pass & 1;
                            const int cbi = *ts->frame_thread[p].cbi++;
                            cf = ts->frame_thread[p].cf;
                            ts->frame_thread[p].cf += imin(t_dim->w, 8) * imin(t_dim->h, 8) * 16;
                            eob  = cbi >> 5;
                            txtp = cbi & 0x1f;
                        } else {
                            uint8_t cf_ctx;
                            cf = bitfn(t->cf);
                            eob = decode_coefs(t, &t->a->lcoef[bx4 + x],
                                               &t->l.lcoef[by4 + y], b->tx, bs,
                                               b, 0, cf, &txtp, &cf_ctx);
                            DEBUG_BLOCK_printf("Post-y_cf_blk[tx=%dx%d,txtp=%d,eob=%d]: r=%d\n",
                                               t_dim->w * 4, t_dim->h * 4, txtp, eob,
                                               ts->msac.rng);
                            txtp &= 0xf; // FIXME
                            dav1d_memset_likely_pow2(&t->a->lcoef[bx4 + x], cf_ctx, imin(t_dim->w, f->bw - t->bx));
                            dav1d_memset_likely_pow2(&t->l.lcoef[by4 + y], cf_ctx, imin(t_dim->h, f->bh - t->by));
                        }
                        if (eob >= 0) {
                            if (DEBUG_BLOCK_INFO && DEBUG_B_PIXELS)
                                coef_dump(cf, imin(t_dim->h, 8) * 4,
                                          imin(t_dim->w, 8) * 4, 3, "dq");
                            dsp->itx.itxfm_add[b->tx]
                                              [txtp](dst,
                                                     f->cur.stride[0],
                                                     cf, eob HIGHBD_CALL_SUFFIX);
                            if (DEBUG_BLOCK_INFO && DEBUG_B_PIXELS)
                                hex_dump(dst, f->cur.stride[0],
                                         t_dim->w * 4, t_dim->h * 4, "recon");
                        }
                    } else if (!t->frame_thread.pass) {
                        dav1d_memset_pow2[t_dim->lw](&t->a->lcoef[bx4 + x], 0x40);
                        dav1d_memset_pow2[t_dim->lh](&t->l.lcoef[by4 + y], 0x40);
                    }
                    dst += 4 * t_dim->w;
                }
                t->bx -= x;
            }
            t->by -= y;

            if (!has_chroma) continue;

            const ptrdiff_t stride = f->cur.stride[1];

            if (b->uv_mode == CFL_PRED) {
                assert(!init_x && !init_y);

                int16_t *const ac = t->scratch.ac;
                pixel *y_src = ((pixel *) f->cur.data[0]) + 4 * (t->bx & ~ss_hor) +
                                 4 * (t->by & ~ss_ver) * PXSTRIDE(f->cur.stride[0]);
                const ptrdiff_t uv_off = 4 * ((t->bx >> ss_hor) +
                                              (t->by >> ss_ver) * PXSTRIDE(stride));
                pixel *const uv_dst[2] = { ((pixel *) f->cur.data[1]) + uv_off,
                                           ((pixel *) f->cur.data[2]) + uv_off };

                const int furthest_r =
                    ((cw4 << ss_hor) + t_dim->w - 1) & ~(t_dim->w - 1);
                const int furthest_b =
                    ((ch4 << ss_ver) + t_dim->h - 1) & ~(t_dim->h - 1);
                dsp->ipred.cfl_ac[f->cur.p.layout - 1](ac, y_src, f->cur.stride[0],
                                                         cbw4 - (furthest_r >> ss_hor),
                                                         cbh4 - (furthest_b >> ss_ver),
                                                         cbw4 * 4, cbh4 * 4);
                for (int pl = 0; pl < 2; pl++) {
                    if (!b->cfl_alpha[pl]) continue;
                    int angle = 0;
                    const pixel *top_sb_edge = NULL;
                    if (!((t->by & ~ss_ver) & (f->sb_step - 1))) {
                        top_sb_edge = f->ipred_edge[pl + 1];
                        const int sby = t->by >> f->sb_shift;
                        top_sb_edge += f->sb256w * 256 * (sby - 1);
                    }
                    const int xpos = t->bx >> ss_hor, ypos = t->by >> ss_ver;
                    const int xstart = ts->tiling.col_start >> ss_hor;
                    const int ystart = ts->tiling.row_start >> ss_ver;
                    const enum IntraPredMode m =
                        bytefn(dav1d_prepare_intra_edges)(xpos, xpos > xstart,
                                                          ypos, ypos > ystart,
                                                          ts->tiling.col_end >> ss_hor,
                                                          ts->tiling.row_end >> ss_ver,
                                                          0, uv_dst[pl], stride,
                                                          top_sb_edge, DC_PRED, &angle,
                                                          uv_t_dim->w, uv_t_dim->h, 0,
                                                          edge HIGHBD_CALL_SUFFIX);
                    dsp->ipred.cfl_pred[m](uv_dst[pl], stride, edge,
                                           uv_t_dim->w * 4,
                                           uv_t_dim->h * 4,
                                           ac, b->cfl_alpha[pl]
                                           HIGHBD_CALL_SUFFIX);
                }
                if (DEBUG_BLOCK_INFO && DEBUG_B_PIXELS) {
                    ac_dump(ac, 4*cbw4, 4*cbh4, "ac");
                    hex_dump(uv_dst[0], stride, cbw4 * 4, cbh4 * 4, "u-cfl-pred");
                    hex_dump(uv_dst[1], stride, cbw4 * 4, cbh4 * 4, "v-cfl-pred");
                }
            }

            const int sm_uv_fl = sm_uv_flag(t->a, cbx4) |
                                 sm_uv_flag(&t->l, cby4);
            const int uv_sb_has_tr =
                ((init_x + 16) >> ss_hor) < cw4 ? 1 : init_y ? 0 :
                intra_edge_flags & (EDGE_I420_TOP_HAS_RIGHT >> (f->cur.p.layout - 1));
            const int uv_sb_has_bl =
                init_x ? 0 : ((init_y + 16) >> ss_ver) < ch4 ? 1 :
                intra_edge_flags & (EDGE_I420_LEFT_HAS_BOTTOM >> (f->cur.p.layout - 1));
            const int sub_cw4 = imin(cw4, (init_x + 16) >> ss_hor);
            for (int pl = 0; pl < 2; pl++) {
                for (y = init_y >> ss_ver, t->by += init_y; y < sub_ch4;
                     y += uv_t_dim->h, t->by += uv_t_dim->h << ss_ver)
                {
                    pixel *dst = ((pixel *) f->cur.data[1 + pl]) +
                                   4 * ((t->by >> ss_ver) * PXSTRIDE(stride) +
                                        ((t->bx + init_x) >> ss_hor));
                    for (x = init_x >> ss_hor, t->bx += init_x; x < sub_cw4;
                         x += uv_t_dim->w, t->bx += uv_t_dim->w << ss_hor)
                    {
                        if ((b->uv_mode == CFL_PRED && b->cfl_alpha[pl]))
                            goto skip_uv_pred;

                        int angle = b->uv_angle;
                        // this probably looks weird because we're using
                        // luma flags in a chroma loop, but that's because
                        // prepare_intra_edges() expects luma flags as input
                        const enum EdgeFlags edge_flags =
                            (((y > (init_y >> ss_ver) || !uv_sb_has_tr) &&
                              (x + uv_t_dim->w >= sub_cw4)) ?
                                 0 : EDGE_I444_TOP_HAS_RIGHT) |
                            ((x > (init_x >> ss_hor) ||
                              (!uv_sb_has_bl && y + uv_t_dim->h >= sub_ch4)) ?
                                 0 : EDGE_I444_LEFT_HAS_BOTTOM);
                        const pixel *top_sb_edge = NULL;
                        if (!((t->by & ~ss_ver) & (f->sb_step - 1))) {
                            top_sb_edge = f->ipred_edge[1 + pl];
                            const int sby = t->by >> f->sb_shift;
                            top_sb_edge += f->sb256w * 256 * (sby - 1);
                        }
                        const enum IntraPredMode uv_mode =
                             b->uv_mode == CFL_PRED ? DC_PRED : b->uv_mode;
                        const int xpos = t->bx >> ss_hor, ypos = t->by >> ss_ver;
                        const int xstart = ts->tiling.col_start >> ss_hor;
                        const int ystart = ts->tiling.row_start >> ss_ver;
                        const enum IntraPredMode m =
                            bytefn(dav1d_prepare_intra_edges)(xpos, xpos > xstart,
                                                              ypos, ypos > ystart,
                                                              ts->tiling.col_end >> ss_hor,
                                                              ts->tiling.row_end >> ss_ver,
                                                              edge_flags, dst, stride,
                                                              top_sb_edge, uv_mode,
                                                              &angle, uv_t_dim->w,
                                                              uv_t_dim->h,
                                                              f->seq_hdr->intra_edge_filter,
                                                              edge HIGHBD_CALL_SUFFIX);
                        angle |= intra_edge_filter_flag;
                        dsp->ipred.intra_pred[m](dst, stride, edge,
                                                 uv_t_dim->w * 4,
                                                 uv_t_dim->h * 4,
                                                 angle | sm_uv_fl,
                                                 (4 * f->bw + ss_hor -
                                                  4 * (t->bx & ~ss_hor)) >> ss_hor,
                                                 (4 * f->bh + ss_ver -
                                                  4 * (t->by & ~ss_ver)) >> ss_ver
                                                 HIGHBD_CALL_SUFFIX);
                        if (DEBUG_BLOCK_INFO && DEBUG_B_PIXELS) {
                            hex_dump(edge - uv_t_dim->h * 4, uv_t_dim->h * 4,
                                     uv_t_dim->h * 4, 2, "l");
                            hex_dump(edge, 0, 1, 1, "tl");
                            hex_dump(edge + 1, uv_t_dim->w * 4,
                                     uv_t_dim->w * 4, 2, "t");
                            hex_dump(dst, stride, uv_t_dim->w * 4,
                                     uv_t_dim->h * 4, pl ? "v-intra-pred" : "u-intra-pred");
                        }

                    skip_uv_pred: {}
                        if (!b->skip_txfm) {
                            enum TxfmType txtp;
                            int eob;
                            coef *cf;
                            if (t->frame_thread.pass) {
                                const int p = t->frame_thread.pass & 1;
                                const int cbi = *ts->frame_thread[p].cbi++;
                                cf = ts->frame_thread[p].cf;
                                ts->frame_thread[p].cf += uv_t_dim->w * uv_t_dim->h * 16;
                                eob  = cbi >> 5;
                                txtp = cbi & 0x1f;
                            } else {
                                uint8_t cf_ctx;
                                cf = bitfn(t->cf);
                                eob = decode_coefs(t, &t->a->ccoef[pl][cbx4 + x],
                                                   &t->l.ccoef[pl][cby4 + y],
                                                   b->uvtx, bs, b, 1 + pl, cf,
                                                   &txtp, &cf_ctx);
                                if (DEBUG_BLOCK_INFO)
                                    printf("Post-uv-cf-blk[pl=%d,tx=%d,"
                                           "txtp=%d,eob=%d]: r=%d [x=%d,cbx4=%d]\n",
                                           pl, b->uvtx, txtp, eob, ts->msac.rng, x, cbx4);
                                int ctw = imin(uv_t_dim->w, (f->bw - t->bx + ss_hor) >> ss_hor);
                                int cth = imin(uv_t_dim->h, (f->bh - t->by + ss_ver) >> ss_ver);
                                dav1d_memset_likely_pow2(&t->a->ccoef[pl][cbx4 + x], cf_ctx, ctw);
                                dav1d_memset_likely_pow2(&t->l.ccoef[pl][cby4 + y], cf_ctx, cth);
                            }
                            if (eob >= 0) {
                                if (DEBUG_BLOCK_INFO && DEBUG_B_PIXELS)
                                    coef_dump(cf, uv_t_dim->h * 4,
                                              uv_t_dim->w * 4, 3, "dq");
                                dsp->itx.itxfm_add[b->uvtx]
                                                  [txtp](dst, stride,
                                                         cf, eob HIGHBD_CALL_SUFFIX);
                                if (DEBUG_BLOCK_INFO && DEBUG_B_PIXELS)
                                    hex_dump(dst, stride, uv_t_dim->w * 4,
                                             uv_t_dim->h * 4, "recon");
                            }
                        } else if (!t->frame_thread.pass) {
                            dav1d_memset_pow2[uv_t_dim->lw](&t->a->ccoef[pl][cbx4 + x], 0x40);
                            dav1d_memset_pow2[uv_t_dim->lh](&t->l.ccoef[pl][cby4 + y], 0x40);
                        }
                        dst += uv_t_dim->w * 4;
                    }
                    t->bx -= x << ss_hor;
                }
                t->by -= y << ss_ver;
            }
        }
    }
#endif
}

#if 0
int bytefn(dav1d_recon_b_inter)(Dav1dTaskContext *const t, const enum BlockSize bs,
                                const Av1Block *const b)
{
    Dav1dTileState *const ts = t->ts;
    const Dav1dFrameContext *const f = t->f;
    const Dav1dDSPContext *const dsp = f->dsp;
    const int bx4 = t->bx & 63, by4 = t->by & 63;
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int cbx4 = bx4 >> ss_hor, cby4 = by4 >> ss_ver;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
    const int has_chroma = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400 &&
                           (bw4 > ss_hor || t->bx & 1) &&
                           (bh4 > ss_ver || t->by & 1);
    const int chr_layout_idx = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I400 ? 0 :
                               DAV1D_PIXEL_LAYOUT_I444 - f->cur.p.layout;
    int res;

    // prediction
    const int cbh4 = (bh4 + ss_ver) >> ss_ver, cbw4 = (bw4 + ss_hor) >> ss_hor;
    pixel *dst = ((pixel *) f->cur.data[0]) +
        4 * (t->by * PXSTRIDE(f->cur.stride[0]) + t->bx);
    const ptrdiff_t uvdstoff =
        4 * ((t->bx >> ss_hor) + (t->by >> ss_ver) * PXSTRIDE(f->cur.stride[1]));
    if (IS_KEY_OR_INTRA(f->frame_hdr)) {
        // intrabc
        res = mc(t, dst, NULL, f->cur.stride[0], bw4, bh4, t->bx, t->by, 0,
                 b->mv[0], &f->sr_cur, 0 /* unused */, FILTER_2D_BILINEAR);
        if (res) return res;
        if (has_chroma) for (int pl = 1; pl < 3; pl++) {
            res = mc(t, ((pixel *)f->cur.data[pl]) + uvdstoff, NULL, f->cur.stride[1],
                     bw4 << (bw4 == ss_hor), bh4 << (bh4 == ss_ver),
                     t->bx & ~ss_hor, t->by & ~ss_ver, pl, b->mv[0],
                     &f->sr_cur, 0 /* unused */, FILTER_2D_BILINEAR);
            if (res) return res;
        }
    } else if (b->comp_type == COMP_INTER_NONE) {
        const Dav1dThreadPicture *const refp = &f->refp[b->ref[0]];
        const enum Filter2d filter_2d = b->filter2d;

        if (imin(bw4, bh4) > 1 &&
            ((b->inter_mode == GLOBALMV && f->gmv_warp_allowed[b->ref[0]]) ||
             (b->motion_mode == MM_WARP && t->warpmv.type > DAV1D_WM_TYPE_TRANSLATION)))
        {
            res = warp_affine(t, dst, NULL, f->cur.stride[0], b_dim, 0, refp,
                              b->motion_mode == MM_WARP ? &t->warpmv :
                                  &f->frame_hdr->gmv[b->ref[0]]);
            if (res) return res;
        } else {
            res = mc(t, dst, NULL, f->cur.stride[0],
                     bw4, bh4, t->bx, t->by, 0, b->mv[0], refp, b->ref[0], filter_2d);
            if (res) return res;
            if (b->motion_mode == MM_OBMC) {
                res = obmc(t, dst, f->cur.stride[0], b_dim, 0, bx4, by4, w4, h4);
                if (res) return res;
            }
        }
        if (b->interintra_type) {
            pixel *const tl_edge = bitfn(t->scratch.edge) + 32;
            enum IntraPredMode m = b->interintra_mode == II_SMOOTH_PRED ?
                                   SMOOTH_PRED : b->interintra_mode;
            pixel *const tmp = bitfn(t->scratch.interintra);
            int angle = 0;
            const pixel *top_sb_edge = NULL;
            if (!(t->by & (f->sb_step - 1))) {
                top_sb_edge = f->ipred_edge[0];
                const int sby = t->by >> f->sb_shift;
                top_sb_edge += f->sb256w * 256 * (sby - 1);
            }
            m = bytefn(dav1d_prepare_intra_edges)(DB_ONLY(0)
                                                  t->bx, t->bx > ts->tiling.col_start,
                                                  t->by, t->by > ts->tiling.row_start,
                                                  ts->tiling.col_end, ts->tiling.row_end,
                                                  0, dst, f->cur.stride[0], top_sb_edge,
                                                  m, &angle, bw4, bh4, 0, tl_edge
                                                  HIGHBD_CALL_SUFFIX);
            dsp->ipred.intra_pred[m](tmp, 4 * bw4 * sizeof(pixel),
                                     tl_edge, bw4 * 4, bh4 * 4, 0, 0, 0
                                     HIGHBD_CALL_SUFFIX);
            dsp->mc.blend(dst, f->cur.stride[0], tmp,
                          bw4 * 4, bh4 * 4, II_MASK(0, bs, b));
        }

        if (!has_chroma) goto skip_inter_chroma_pred;

        // sub8x8 derivation
        int is_sub8x8 = bw4 == ss_hor || bh4 == ss_ver;
        refmvs_block *const *r;
        if (is_sub8x8) {
            assert(ss_hor == 1);
            r = &t->rt.r[(t->by & 31) + 5];
            if (bw4 == 1) is_sub8x8 &= r[0][t->bx - 1].ref.ref[0] > 0;
            if (bh4 == ss_ver) is_sub8x8 &= r[-1][t->bx].ref.ref[0] > 0;
            if (bw4 == 1 && bh4 == ss_ver)
                is_sub8x8 &= r[-1][t->bx - 1].ref.ref[0] > 0;
        }

        // chroma prediction
        if (is_sub8x8) {
            assert(ss_hor == 1);
            ptrdiff_t h_off = 0, v_off = 0;
            if (bw4 == 1 && bh4 == ss_ver) {
                for (int pl = 0; pl < 2; pl++) {
                    res = mc(t, ((pixel *) f->cur.data[1 + pl]) + uvdstoff,
                             NULL, f->cur.stride[1],
                             bw4, bh4, t->bx - 1, t->by - 1, 1 + pl,
                             r[-1][t->bx - 1].mv.mv[0],
                             &f->refp[r[-1][t->bx - 1].ref.ref[0] - 1],
                             r[-1][t->bx - 1].ref.ref[0] - 1,
                             t->frame_thread.pass != 2 ? t->tl_4x4_filter :
                                 f->frame_thread.b[((t->by - 1) * f->b4_stride) + t->bx - 1].filter2d);
                    if (res) return res;
                }
                v_off = 2 * PXSTRIDE(f->cur.stride[1]);
                h_off = 2;
            }
            if (bw4 == 1) {
                const enum Filter2d left_filter_2d =
                    dav1d_filter_2d[t->l.filter[1][by4]][t->l.filter[0][by4]];
                for (int pl = 0; pl < 2; pl++) {
                    res = mc(t, ((pixel *) f->cur.data[1 + pl]) + uvdstoff + v_off, NULL,
                             f->cur.stride[1], bw4, bh4, t->bx - 1,
                             t->by, 1 + pl, r[0][t->bx - 1].mv.mv[0],
                             &f->refp[r[0][t->bx - 1].ref.ref[0] - 1],
                             r[0][t->bx - 1].ref.ref[0] - 1,
                             t->frame_thread.pass != 2 ? left_filter_2d :
                                 f->frame_thread.b[(t->by * f->b4_stride) + t->bx - 1].filter2d);
                    if (res) return res;
                }
                h_off = 2;
            }
            if (bh4 == ss_ver) {
                const enum Filter2d top_filter_2d =
                    dav1d_filter_2d[t->a->filter[1][bx4]][t->a->filter[0][bx4]];
                for (int pl = 0; pl < 2; pl++) {
                    res = mc(t, ((pixel *) f->cur.data[1 + pl]) + uvdstoff + h_off, NULL,
                             f->cur.stride[1], bw4, bh4, t->bx, t->by - 1,
                             1 + pl, r[-1][t->bx].mv.mv[0],
                             &f->refp[r[-1][t->bx].ref.ref[0] - 1],
                             r[-1][t->bx].ref.ref[0] - 1,
                             t->frame_thread.pass != 2 ? top_filter_2d :
                                 f->frame_thread.b[((t->by - 1) * f->b4_stride) + t->bx].filter2d);
                    if (res) return res;
                }
                v_off = 2 * PXSTRIDE(f->cur.stride[1]);
            }
            for (int pl = 0; pl < 2; pl++) {
                res = mc(t, ((pixel *) f->cur.data[1 + pl]) + uvdstoff + h_off + v_off, NULL, f->cur.stride[1],
                         bw4, bh4, t->bx, t->by, 1 + pl, b->mv[0],
                         refp, b->ref[0], filter_2d);
                if (res) return res;
            }
        } else {
            if (imin(cbw4, cbh4) > 1 &&
                ((b->inter_mode == GLOBALMV && f->gmv_warp_allowed[b->ref[0]]) ||
                 (b->motion_mode == MM_WARP && t->warpmv.type > DAV1D_WM_TYPE_TRANSLATION)))
            {
                for (int pl = 0; pl < 2; pl++) {
                    res = warp_affine(t, ((pixel *) f->cur.data[1 + pl]) + uvdstoff, NULL,
                                      f->cur.stride[1], b_dim, 1 + pl, refp,
                                      b->motion_mode == MM_WARP ? &t->warpmv :
                                          &f->frame_hdr->gmv[b->ref[0]]);
                    if (res) return res;
                }
            } else {
                for (int pl = 0; pl < 2; pl++) {
                    res = mc(t, ((pixel *) f->cur.data[1 + pl]) + uvdstoff,
                             NULL, f->cur.stride[1],
                             bw4 << (bw4 == ss_hor), bh4 << (bh4 == ss_ver),
                             t->bx & ~ss_hor, t->by & ~ss_ver,
                             1 + pl, b->mv[0], refp, b->ref[0], filter_2d);
                    if (res) return res;
                    if (b->motion_mode == MM_OBMC) {
                        res = obmc(t, ((pixel *) f->cur.data[1 + pl]) + uvdstoff,
                                   f->cur.stride[1], b_dim, 1 + pl, bx4, by4, w4, h4);
                        if (res) return res;
                    }
                }
            }
            if (b->interintra_type) {
                // FIXME for 8x32 with 4:2:2 subsampling, this probably does
                // the wrong thing since it will select 4x16, not 4x32, as a
                // transform size...
                const uint8_t *const ii_mask = II_MASK(chr_layout_idx, bs, b);

                for (int pl = 0; pl < 2; pl++) {
                    pixel *const tmp = bitfn(t->scratch.interintra);
                    pixel *const tl_edge = bitfn(t->scratch.edge) + 32;
                    enum IntraPredMode m =
                        b->interintra_mode == II_SMOOTH_PRED ?
                        SMOOTH_PRED : b->interintra_mode;
                    int angle = 0;
                    pixel *const uvdst = ((pixel *) f->cur.data[1 + pl]) + uvdstoff;
                    const pixel *top_sb_edge = NULL;
                    if (!(t->by & (f->sb_step - 1))) {
                        top_sb_edge = f->ipred_edge[pl + 1];
                        const int sby = t->by >> f->sb_shift;
                        top_sb_edge += f->sb256w * 256 * (sby - 1);
                    }
                    m = bytefn(dav1d_prepare_intra_edges)(DB_ONLY(0)
                                                          t->bx >> ss_hor,
                                                          (t->bx >> ss_hor) >
                                                              (ts->tiling.col_start >> ss_hor),
                                                          t->by >> ss_ver,
                                                          (t->by >> ss_ver) >
                                                              (ts->tiling.row_start >> ss_ver),
                                                          ts->tiling.col_end >> ss_hor,
                                                          ts->tiling.row_end >> ss_ver,
                                                          0, uvdst, f->cur.stride[1],
                                                          top_sb_edge, m,
                                                          &angle, cbw4, cbh4, 0, tl_edge
                                                          HIGHBD_CALL_SUFFIX);
                    dsp->ipred.intra_pred[m](tmp, cbw4 * 4 * sizeof(pixel),
                                             tl_edge, cbw4 * 4, cbh4 * 4, 0, 0, 0
                                             HIGHBD_CALL_SUFFIX);
                    dsp->mc.blend(uvdst, f->cur.stride[1], tmp,
                                  cbw4 * 4, cbh4 * 4, ii_mask);
                }
            }
        }

    skip_inter_chroma_pred: {}
        t->tl_4x4_filter = filter_2d;
    } else {
        const enum Filter2d filter_2d = b->filter2d;
        // Maximum super block size is 128x128
        int16_t (*tmp)[128 * 128] = t->scratch.compinter;
        int jnt_weight;
        uint8_t *const seg_mask = t->scratch.seg_mask;
        const uint8_t *mask;

        for (int i = 0; i < 2; i++) {
            const Dav1dThreadPicture *const refp = &f->refp[b->ref[i]];

            if (b->inter_mode == GLOBALMV_GLOBALMV && f->gmv_warp_allowed[b->ref[i]]) {
                res = warp_affine(t, NULL, tmp[i], bw4 * 4, b_dim, 0, refp,
                                  &f->frame_hdr->gmv[b->ref[i]]);
                if (res) return res;
            } else {
                res = mc(t, NULL, tmp[i], 0, bw4, bh4, t->bx, t->by, 0,
                         b->mv[i], refp, b->ref[i], filter_2d);
                if (res) return res;
            }
        }
        switch (b->comp_type) {
        case COMP_INTER_AVG:
            dsp->mc.avg(dst, f->cur.stride[0], tmp[0], tmp[1],
                        bw4 * 4, bh4 * 4 HIGHBD_CALL_SUFFIX);
            break;
        case COMP_INTER_WEIGHTED_AVG:
            jnt_weight = f->jnt_weights[b->ref[0]][b->ref[1]];
            dsp->mc.w_avg(dst, f->cur.stride[0], tmp[0], tmp[1],
                          bw4 * 4, bh4 * 4, jnt_weight HIGHBD_CALL_SUFFIX);
            break;
        case COMP_INTER_SEG:
            dsp->mc.w_mask[chr_layout_idx](dst, f->cur.stride[0],
                                           tmp[b->mask_sign], tmp[!b->mask_sign],
                                           bw4 * 4, bh4 * 4, seg_mask,
                                           b->mask_sign HIGHBD_CALL_SUFFIX);
            mask = seg_mask;
            break;
        case COMP_INTER_WEDGE:
            mask = WEDGE_MASK(0, bs, 0, b->wedge_idx);
            dsp->mc.mask(dst, f->cur.stride[0],
                         tmp[b->mask_sign], tmp[!b->mask_sign],
                         bw4 * 4, bh4 * 4, mask HIGHBD_CALL_SUFFIX);
            if (has_chroma)
                mask = WEDGE_MASK(chr_layout_idx, bs, b->mask_sign, b->wedge_idx);
            break;
        }

        // chroma
        if (has_chroma) for (int pl = 0; pl < 2; pl++) {
            for (int i = 0; i < 2; i++) {
                const Dav1dThreadPicture *const refp = &f->refp[b->ref[i]];
                if (b->inter_mode == GLOBALMV_GLOBALMV &&
                    imin(cbw4, cbh4) > 1 && f->gmv_warp_allowed[b->ref[i]])
                {
                    res = warp_affine(t, NULL, tmp[i], bw4 * 4 >> ss_hor,
                                      b_dim, 1 + pl,
                                      refp, &f->frame_hdr->gmv[b->ref[i]]);
                    if (res) return res;
                } else {
                    res = mc(t, NULL, tmp[i], 0, bw4, bh4, t->bx, t->by,
                             1 + pl, b->mv[i], refp, b->ref[i], filter_2d);
                    if (res) return res;
                }
            }
            pixel *const uvdst = ((pixel *) f->cur.data[1 + pl]) + uvdstoff;
            switch (b->comp_type) {
            case COMP_INTER_AVG:
                dsp->mc.avg(uvdst, f->cur.stride[1], tmp[0], tmp[1],
                            bw4 * 4 >> ss_hor, bh4 * 4 >> ss_ver
                            HIGHBD_CALL_SUFFIX);
                break;
            case COMP_INTER_WEIGHTED_AVG:
                dsp->mc.w_avg(uvdst, f->cur.stride[1], tmp[0], tmp[1],
                              bw4 * 4 >> ss_hor, bh4 * 4 >> ss_ver, jnt_weight
                              HIGHBD_CALL_SUFFIX);
                break;
            case COMP_INTER_WEDGE:
            case COMP_INTER_SEG:
                dsp->mc.mask(uvdst, f->cur.stride[1],
                             tmp[b->mask_sign], tmp[!b->mask_sign],
                             bw4 * 4 >> ss_hor, bh4 * 4 >> ss_ver, mask
                             HIGHBD_CALL_SUFFIX);
                break;
            }
        }
    }

    if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
        hex_dump(dst, f->cur.stride[0], b_dim[0] * 4, b_dim[1] * 4, "y-pred");
        if (has_chroma) {
            hex_dump(&((pixel *) f->cur.data[1])[uvdstoff], f->cur.stride[1],
                     cbw4 * 4, cbh4 * 4, "u-pred");
            hex_dump(&((pixel *) f->cur.data[2])[uvdstoff], f->cur.stride[1],
                     cbw4 * 4, cbh4 * 4, "v-pred");
        }
    }

    const int cw4 = (w4 + ss_hor) >> ss_hor, ch4 = (h4 + ss_ver) >> ss_ver;

    if (b->skip_txfm) {
        // reset coef contexts
        BlockContext *const a = t->a;
        dav1d_memset_pow2[b_dim[2]](&a->lcoef[bx4], 0x40);
        dav1d_memset_pow2[b_dim[3]](&t->l.lcoef[by4], 0x40);
        if (has_chroma) {
            dav1d_memset_pow2_fn memset_cw = dav1d_memset_pow2[ulog2(cbw4)];
            dav1d_memset_pow2_fn memset_ch = dav1d_memset_pow2[ulog2(cbh4)];
            memset_cw(&a->ccoef[0][cbx4], 0x40);
            memset_cw(&a->ccoef[1][cbx4], 0x40);
            memset_ch(&t->l.ccoef[0][cby4], 0x40);
            memset_ch(&t->l.ccoef[1][cby4], 0x40);
        }
        return 0;
    }

    const TxfmInfo *const uvtx = &dav1d_txfm_dimensions[b->uvtx];
    const TxfmInfo *const ytx = &dav1d_txfm_dimensions[b->max_ytx];
    const uint16_t tx_split[2] = { b->tx_split0, b->tx_split1 };

    for (int init_y = 0; init_y < bh4; init_y += 16) {
        for (int init_x = 0; init_x < bw4; init_x += 16) {
            // coefficient coding & inverse transforms
            int y_off = !!init_y, y;
            dst += PXSTRIDE(f->cur.stride[0]) * 4 * init_y;
            for (y = init_y, t->by += init_y; y < imin(h4, init_y + 16);
                 y += ytx->h, y_off++)
            {
                int x, x_off = !!init_x;
                for (x = init_x, t->bx += init_x; x < imin(w4, init_x + 16);
                     x += ytx->w, x_off++)
                {
                    read_coef_tree(t, bs, b, b->max_ytx, 0, tx_split,
                                   x_off, y_off, &dst[x * 4]);
                    t->bx += ytx->w;
                }
                dst += PXSTRIDE(f->cur.stride[0]) * 4 * ytx->h;
                t->bx -= x;
                t->by += ytx->h;
            }
            dst -= PXSTRIDE(f->cur.stride[0]) * 4 * y;
            t->by -= y;

            // chroma coefs and inverse transform
            if (has_chroma) for (int pl = 0; pl < 2; pl++) {
                pixel *uvdst = ((pixel *) f->cur.data[1 + pl]) + uvdstoff +
                    (PXSTRIDE(f->cur.stride[1]) * init_y * 4 >> ss_ver);
                for (y = init_y >> ss_ver, t->by += init_y;
                     y < imin(ch4, (init_y + 16) >> ss_ver); y += uvtx->h)
                {
                    int x;
                    for (x = init_x >> ss_hor, t->bx += init_x;
                         x < imin(cw4, (init_x + 16) >> ss_hor); x += uvtx->w)
                    {
                        coef *cf;
                        int eob;
                        enum TxfmType txtp;
                        if (t->frame_thread.pass) {
                            const int p = t->frame_thread.pass & 1;
                            const int cbi = *ts->frame_thread[p].cbi++;
                            cf = ts->frame_thread[p].cf;
                            ts->frame_thread[p].cf += uvtx->w * uvtx->h * 16;
                            eob  = cbi >> 5;
                            txtp = cbi & 0x1f;
                        } else {
                            uint8_t cf_ctx;
                            cf = bitfn(t->cf);
                            txtp = t->scratch.txtp_map[((t->by + (y << ss_ver)) & 15) * 16 +
                                                       ((t->bx + (x << ss_hor)) & 15)];
                            eob = decode_coefs(t, DB_ONLY(0) &t->a->ccoef[pl][cbx4 + x],
                                               &t->l.ccoef[pl][cby4 + y],
                                               b->uvtx, bs, b, 1 + pl,
                                               cf, &txtp, &cf_ctx);
                            if (DEBUG_BLOCK_INFO)
                                printf("Post-uv-cf-blk[pl=%d,tx=%d,"
                                       "txtp=%d,eob=%d]: r=%d\n",
                                       pl, b->uvtx, txtp, eob, ts->msac.rng);
                            int ctw = imin(uvtx->w, (f->bw - t->bx + ss_hor) >> ss_hor);
                            int cth = imin(uvtx->h, (f->bh - t->by + ss_ver) >> ss_ver);
                            dav1d_memset_likely_pow2(&t->a->ccoef[pl][cbx4 + x], cf_ctx, ctw);
                            dav1d_memset_likely_pow2(&t->l.ccoef[pl][cby4 + y], cf_ctx, cth);
                        }
                        if (eob >= 0) {
                            if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                                coef_dump(cf, uvtx->h * 4, uvtx->w * 4, 3, "dq");
                            dsp->itx.itxfm_add[b->uvtx]
                                              [txtp](&uvdst[4 * x],
                                                     f->cur.stride[1],
                                                     cf, eob HIGHBD_CALL_SUFFIX);
                            if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                                hex_dump(&uvdst[4 * x], f->cur.stride[1],
                                         uvtx->w * 4, uvtx->h * 4, "recon");
                        }
                        t->bx += uvtx->w << ss_hor;
                    }
                    uvdst += PXSTRIDE(f->cur.stride[1]) * 4 * uvtx->h;
                    t->bx -= x << ss_hor;
                    t->by += uvtx->h << ss_ver;
                }
                t->by -= y << ss_ver;
            }
        }
    }
    return 0;
}
#endif

void bytefn(dav1d_filter_sbrow_deblock_cols)(Dav1dFrameContext *const f, const int sby) {
    if (!(f->c->inloop_filters & DAV1D_INLOOPFILTER_DEBLOCK) ||
        (!f->frame_hdr->loopfilter.level_y[0] && !f->frame_hdr->loopfilter.level_y[1]))
    {
        return;
    }
    const int y = sby * f->sb_step * 4;
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    pixel *const p[3] = {
        f->lf.p[0] + y * PXSTRIDE(f->cur.stride[0]),
        f->lf.p[1] + (y * PXSTRIDE(f->cur.stride[1]) >> ss_ver),
        f->lf.p[2] + (y * PXSTRIDE(f->cur.stride[1]) >> ss_ver)
    };
    Av1Filter *mask = f->lf.mask + (sby >> (2 - f->frame_hdr->sb128)) * f->sb256w;
    bytefn(dav1d_loopfilter_sbrow_cols)(f, p, mask, sby, f->lf.start_of_tile_row[sby]);
}

void bytefn(dav1d_filter_sbrow_deblock_rows)(Dav1dFrameContext *const f, const int sby) {
    const int y = sby * f->sb_step * 4;
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    pixel *const p[3] = {
        f->lf.p[0] + y * PXSTRIDE(f->cur.stride[0]),
        f->lf.p[1] + (y * PXSTRIDE(f->cur.stride[1]) >> ss_ver),
        f->lf.p[2] + (y * PXSTRIDE(f->cur.stride[1]) >> ss_ver)
    };
    Av1Filter *mask = f->lf.mask + (sby >> (2 - f->frame_hdr->sb128)) * f->sb256w;
    if (f->c->inloop_filters & DAV1D_INLOOPFILTER_DEBLOCK &&
        (f->frame_hdr->loopfilter.level_y[0] || f->frame_hdr->loopfilter.level_y[1]))
    {
        bytefn(dav1d_loopfilter_sbrow_rows)(f, p, mask, sby);
    }
    if ((f->seq_hdr->cdef &&
         f->c->inloop_filters & DAV1D_INLOOPFILTER_CDEF) ||
        (f->lf.restore_planes &&
         f->c->inloop_filters & DAV1D_INLOOPFILTER_RESTORATION))
    {
        // Store loop filtered pixels required by CDEF / LR
        bytefn(dav1d_copy_lpf)(f, p, sby);
    }
}

void bytefn(dav1d_filter_sbrow_cdef)(Dav1dTaskContext *const tc, const int sby) {
    const Dav1dFrameContext *const f = tc->f;
    if (!(f->c->inloop_filters & (DAV1D_INLOOPFILTER_CDEF | DAV1D_INLOOPFILTER_CCSO))) return;
    const int sbsz = f->sb_step;
    const int y = sby * sbsz * 4;
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    pixel *const p[3] = {
        f->lf.p[0] + y * PXSTRIDE(f->cur.stride[0]),
        f->lf.p[1] + (y * PXSTRIDE(f->cur.stride[1]) >> ss_ver),
        f->lf.p[2] + (y * PXSTRIDE(f->cur.stride[1]) >> ss_ver)
    };
    Av1Filter *prev_mask = f->lf.mask + ((sby - 1) >> (2 - f->frame_hdr->sb128)) * f->sb256w;
    Av1Filter *mask = f->lf.mask + (sby >> (2 - f->frame_hdr->sb128)) * f->sb256w;
    const int start = sby * sbsz;
    if (sby) {
        const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
        pixel *p_up[3] = {
            p[0] - 8 * PXSTRIDE(f->cur.stride[0]),
            p[1] - (8 * PXSTRIDE(f->cur.stride[1]) >> ss_ver),
            p[2] - (8 * PXSTRIDE(f->cur.stride[1]) >> ss_ver),
        };
        bytefn(dav1d_cdef_brow)(tc, p_up, prev_mask, start - 2, start, 1, sby);
    }
    const int n_blks = sbsz - 2 * (sby + 1 < f->sbh);
    const int end = imin(start + n_blks, f->bh);
    bytefn(dav1d_cdef_brow)(tc, p, mask, start, end, 0, sby);
}

void bytefn(dav1d_filter_sbrow_lr)(Dav1dFrameContext *const f, const int sby) {
    if (!(f->c->inloop_filters & DAV1D_INLOOPFILTER_RESTORATION)) return;
    const int y = sby * f->sb_step * 4;
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    pixel *const sr_p[3] = {
        f->lf.sr_p[0] + y * PXSTRIDE(f->sr_cur.p.stride[0]),
        f->lf.sr_p[1] + (y * PXSTRIDE(f->sr_cur.p.stride[1]) >> ss_ver),
        f->lf.sr_p[2] + (y * PXSTRIDE(f->sr_cur.p.stride[1]) >> ss_ver)
    };
    bytefn(dav1d_lr_sbrow)(f, sr_p, sby);
}

void bytefn(dav1d_filter_sbrow)(Dav1dFrameContext *const f, const int sby) {
    bytefn(dav1d_filter_sbrow_deblock_cols)(f, sby);
    bytefn(dav1d_filter_sbrow_deblock_rows)(f, sby);
    if (f->seq_hdr->cdef)
        bytefn(dav1d_filter_sbrow_cdef)(f->c->tc, sby);
#if 0
    if (f->lf.restore_planes)
        bytefn(dav1d_filter_sbrow_lr)(f, sby);
#endif
}

void bytefn(dav1d_backup_ipred_edge)(Dav1dTaskContext *const t) {
    const Dav1dFrameContext *const f = t->f;
    Dav1dTileState *const ts = t->ts;
    if (t->by + f->sb_step >= ts->tiling.row_end) return;
    const int sby = t->by >> f->sb_shift;
    const int sby_off = f->sb256w * 256 * sby;
    const int x_off = ts->tiling.col_start;

    const pixel *const y =
        ((const pixel *) f->cur.data[0]) + x_off * 4 +
                    ((t->by + f->sb_step) * 4 - 1) * PXSTRIDE(f->cur.stride[0]);
    pixel_copy(&f->ipred_edge[0][sby_off + x_off * 4], y,
               4 * (ts->tiling.col_end - x_off));

    if (f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400) {
        const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
        const int ss_hor = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;

        const ptrdiff_t uv_off = (x_off * 4 >> ss_hor) +
            (((t->by + f->sb_step) * 4 >> ss_ver) - 1) * PXSTRIDE(f->cur.stride[1]);
        for (int pl = 1; pl <= 2; pl++)
            pixel_copy(&f->ipred_edge[pl][sby_off + (x_off * 4 >> ss_hor)],
                       &((const pixel *) f->cur.data[pl])[uv_off],
                       4 * (ts->tiling.col_end - x_off) >> ss_hor);
    }
}

void bytefn(dav1d_copy_pal_block_y)(Dav1dTaskContext *const t,
                                    const int bx4, const int by4,
                                    const int bw4, const int bh4)

{
    const Dav1dFrameContext *const f = t->f;
    pixel *const pal = t->frame_thread.pass ?
        f->frame_thread.pal[((t->by >> 1) + (t->bx & 1)) * (f->b4_stride >> 1) +
                            ((t->bx >> 1) + (t->by & 1))] : bytefn(t->scratch.pal);
    for (int x = 0; x < bw4; x++)
        memcpy(bytefn(t->al_pal)[0][bx4 + x], pal, 8 * sizeof(pixel));
    for (int y = 0; y < bh4; y++)
        memcpy(bytefn(t->al_pal)[1][by4 + y], pal, 8 * sizeof(pixel));
}

void bytefn(dav1d_read_pal_plane)(DB_ONLY(const int depth)
                                  Dav1dTaskContext *const t, Av1Block *const b,
                                  const int bx4, const int by4)
{
    Dav1dTileState *const ts = t->ts;
    const Dav1dFrameContext *const f = t->f;
    const int pal_sz = b->pal_sz =
        dav1d_msac_decode_symbol_adapt8(&ts->msac, ts->cdf.m.pal_sz, 6) + 2;
    // don't reuse above palette outside SB64 boundaries
    const int a_cache = by4 & 15 ? t->a->pal_sz[bx4] : 0;
    const int l_cache = t->l.pal_sz[by4];
    const pixel *l = bytefn(t->al_pal)[1][by4], *a = bytefn(t->al_pal)[0][bx4];

    // find cached entries (but don't load them yet)
    const int n_cache = l_cache + a_cache;
    int n_used_cache = 0;
    unsigned cache_reuse_mask = 0;
    int off = 0;
    for (int n = imin(n_cache, pal_sz); n;
         off += n, n = imin(n_cache - off, pal_sz - n_used_cache))
    {
        const unsigned m = dav1d_msac_decode_bools_bypass(&ts->msac, n);
        cache_reuse_mask <<= n;
        cache_reuse_mask |= m;
        n_used_cache += popcnt(m);
    }
    pixel cache[8];
    if (n_used_cache) {
        if (!l_cache) {
#define select(dir) \
            /* directly copy the selected cache entries into cache[] */ \
            assert(!(cache_reuse_mask & ~0xff)); \
            unsigned mask = cache_reuse_mask << (32 - off); \
            int i = 0, n = 0; \
            do { \
                int n_zero = clz(mask); \
                cache[i++] = dir[n + n_zero++]; \
                n += n_zero; \
                mask <<= n_zero; \
            } while (i < pal_sz)
            select(a);
        } else if (!a_cache) {
            select(l);
#undef select
        } else {
            // sort selected cache entries from a & l into cache[]
            const int min_n = imin(a_cache, l_cache);
            unsigned mask = cache_reuse_mask << (32 - off);
            const unsigned rem_mask = mask << (min_n * 2) >> min_n;
            unsigned shared_mask = mask - (rem_mask >> min_n);
            shared_mask = (shared_mask & 0xaaaa0000) |
                         ((shared_mask & 0x55550000) >> 15);
            shared_mask |= shared_mask << 1;
            shared_mask &= 0xcccccccc;
            shared_mask |= shared_mask << 2;
            shared_mask &= 0xf0f0f0f0;
            shared_mask |= shared_mask << 4;
            const int a_gt_l = a_cache > l_cache;
            unsigned a_mask = (shared_mask & 0xff000000) + a_gt_l * rem_mask;
            unsigned l_mask = ((shared_mask & 0xff00) << 16) + !a_gt_l * rem_mask;

            int i = 0, a_n = 0, l_n = 0;
            if (a_mask && l_mask) {
#define cnt_zero(dir) do { \
                const int n_zero = clz(dir##_mask); \
                dir##_n += n_zero; \
                dir##_mask <<= n_zero; \
            } while (0)
                cnt_zero(a);
                cnt_zero(l);
                for (;;) {
                    assert((a_mask & l_mask) & 0x80000000);
                    if (a[a_n] < l[l_n]) {
#define consume(dir) \
                        cache[i++] = dir[dir##_n]; \
                        dir##_mask <<= 1; \
                        if (!dir##_mask) break; \
                        const int n_zero = clz(dir##_mask); \
                        dir##_n += 1 + n_zero; \
                        dir##_mask <<= n_zero
                        consume(a);
                    } else {
                        consume(l);
                    }
                }
            }
            assert(i < pal_sz);
            if (a_mask) {
                cnt_zero(a);
                for (;;) {
                    consume(a);
                }
            } else {
                cnt_zero(l);
                for (;;) {
                    consume(l);
#undef cnt_zero
#undef consume
                }
            }
        }
    }

    // parse new entries
    pixel *const pal = t->frame_thread.pass ?
        f->frame_thread.pal[((t->by >> 1) + (t->bx & 1)) * (f->b4_stride >> 1) +
                            ((t->bx >> 1) + (t->by & 1))] : bytefn(t->scratch.pal);
    if (n_used_cache < pal_sz) {
        int i = n_used_cache;
        const int bpc = BITDEPTH == 8 ? 8 : f->cur.p.bpc;
        int prev = pal[i++] = dav1d_msac_decode_bools_bypass(&ts->msac, bpc);

        if (i < pal_sz) {
            int bits = bpc - 3 + dav1d_msac_decode_bools_bypass(&ts->msac, 2);
            const int max = (1 << bpc) - 1;

            do {
                const int delta = dav1d_msac_decode_bools_bypass(&ts->msac, bits);
                prev = pal[i++] = imin(prev + delta + 1, max);
                if (prev + 1 >= max) {
                    for (; i < pal_sz; i++)
                        pal[i] = max;
                    break;
                }
                bits = imin(bits, 1 + ulog2(max - prev - 1));
            } while (i < pal_sz);
        }

        // merge selected cache & new entries into pal while sorting cache
        if (n_used_cache) {
            int n = 0, m = n_used_cache;
            for (i = 0; i < pal_sz; i++) {
                if (n < n_used_cache && (m >= pal_sz || cache[n] <= pal[m])) {
                    pal[i] = cache[n++];
                } else {
                    assert(m < pal_sz);
                    pal[i] = pal[m++];
                }
            }
        }
    } else {
        pixel_copy(pal, cache, pal_sz);
    }

#if DEBUG_BLOCK_INFO
#define bitmask(x) /* emulate %b (up to 16 bits) */ \
    ((uint64_t) ((x) & 0x8000) << 45) | \
    ((uint64_t) ((x) & 0x4000) << 42) | \
    ((uint64_t) ((x) & 0x2000) << 39) | \
    ((uint64_t) ((x) & 0x1000) << 36) | \
    ((uint64_t) ((x) &  0x800) << 33) | \
    ((uint64_t) ((x) &  0x400) << 30) | \
    ((uint64_t) ((x) &  0x200) << 27) | \
    ((uint64_t) ((x) &  0x100) << 24) | \
    ((uint64_t) ((x) &   0x80) << 21) | \
    ((uint64_t) ((x) &   0x40) << 18) | \
    ((uint64_t) ((x) &   0x20) << 15) | \
    ((uint64_t) ((x) &   0x10) << 12) | \
    ((uint64_t) ((x) &    0x8) <<  9) | \
    ((uint64_t) ((x) &    0x4) <<  6) | \
    ((uint64_t) ((x) &    0x2) <<  3) | \
    ((uint64_t) ((x) &    0x1) <<  0)
    if (BLOCK_TO_DEBUG) {
        printf("%*sPost-ypal[sz=%d,cache_sz=%d,mask=%0*"PRIx64"|%d]: r=%d, cache=",
               depth, "", pal_sz, n_cache, off, bitmask(cache_reuse_mask),
               n_used_cache, ts->msac.rng);
        const int min_n = imin(a_cache, l_cache), max_n = n_cache - min_n;
        for (int n = 0; n < min_n; n++)
            printf("%c"PIX_HEX_FMT","PIX_HEX_FMT,
                   n ? ',' : '[', a[n], l[n]);
        const pixel *dir = a_cache > l_cache ? a : l;
        for (int n = min_n; n < max_n; n++)
            printf("%c"PIX_HEX_FMT, n ? ',' : '[', dir[n]);
        printf("%s, pal=", n_cache ? "]" : "[]");
        for (int n = 0; n < pal_sz; n++)
            printf("%c"PIX_HEX_FMT, n ? ',' : '[', pal[n]);
        printf("]\n");
    }
#endif
}

