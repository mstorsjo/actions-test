/*
 * Copyright © 2018-2026, VideoLAN and dav2d authors
 * Copyright © 2018-2026, Two Orioles, LLC
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
#include <limits.h>
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
#include "src/derivation.h"
#include "src/db_apply.h"
#include "src/lr_apply.h"
#include "src/recon.h"
#include "src/scan.h"
#include "src/stx_tables.h"
#include "src/tables.h"
#include "src/warpmv.h"
#include "src/wedge.h"

static inline unsigned decode_exp_golomb(MsacContext *const s, const int k) {
    const int length = dav2d_msac_decode_unary_bypass21(s) + k;
    const int x = (1 << length) + dav2d_msac_decode_bools_bypass(s, length);
    return x - (1 << k);
}

static inline int decode_hr(MsacContext *const s, const int hr_avg) {
    const int m = ulog2(iclip(hr_avg, 2, 64)); // 1..6
    const int cmax = imin(m + 4, 6); // 5 or 6
    const int q = dav2d_msac_decode_unary_bypass6(s, cmax);
    const int rem = (q == cmax) ? decode_exp_golomb(s, m + 1) :
                                  dav2d_msac_decode_bools_bypass(s, m);
    return rem + (q << m);
}


static inline unsigned get_skip_ctx(const TxfmInfo *const t_dim,
                                    const enum BlockSize bs,
                                    const uint8_t *const a,
                                    const uint8_t *const l,
                                    const int plane, const int u_has_cf,
                                    const enum Dav2dPixelLayout layout)
{
    const uint8_t *const b_dim = dav2d_block_dimensions[bs];

    if (plane) {
        const int ss_ver = layout == DAV2D_PIXEL_LAYOUT_I420;
        const int ss_hor = layout != DAV2D_PIXEL_LAYOUT_I444;
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

        const int offset = plane == 1 ? 6 : 6 * u_has_cf + not_one_blk * 3;
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

static int decode_coefs(Dav2dTaskContext *const t, DB_ONLY(const int depth)
                        uint8_t *const a, uint8_t *const l,
                        const enum RectTxfmSize tx, const enum BlockSize bs,
                        const int sdp_active,
                        const Av2Block *const b, const int plane, coef *cf,
                        enum TxfmType *const txtp, uint8_t *res_ctx)
{
    Dav2dTileState *const ts = t->ts;
    const int chroma = !!plane; // FIXME perhaps make this an inlined function arg?
    const int intra = b->intra && (sdp_active || !b->intrabc);
    const Dav2dFrameContext *const f = t->f;
    const int lossless = f->frame_hdr->segmentation.lossless[b->seg_id];
    const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
#if DEBUG_BLOCK_INFO
    const int dbg = BLOCK_TO_DEBUG && plane > -1 && 1;
#define DEBUG_CF_printf(...) \
    if (dbg) printf(__VA_ARGS__)
#else
#define DEBUG_CF_printf(...)
#endif

    DEBUG_CF_printf("%*sdecode_cf[y=%d,x=%d,pl=%d,tx=%dx%d]: r=%d\n",
                    depth - 1, "", t->by, t->bx, plane, t_dim->w * 4, t_dim->h * 4,
                    ts->msac.rng);

    // does this block have any non-zero coefficients
    const int sctx = (b->fsc && !chroma && f->seq_hdr->fsc) ? 9 :
                     get_skip_ctx(t_dim, bs, a, l, plane, t->u_has_cf, f->cur.p.p.layout);
    const int all_skip =
        dav2d_msac_decode_bool_adapt(&ts->msac,
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
        eob = dav2d_msac_decode_symbol_adapt8(&ts->msac, eob_bin_cdf, bits); \
        if (eb && eob == 7) { \
            eob += dav2d_msac_decode_bools_bypass(&ts->msac, eb); \
            if (bin == 512 && eob == 10) return INT_MIN; \
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
        const int eob_hi_bit = dav2d_msac_decode_bool_adapt(&ts->msac,
                                   ts->cdf.coef.eob_hi_bit);
        const int eob_bin = eob - 2;
        eob = eob_hi_bit | 2;
        if (eob_bin)
            eob = (eob << eob_bin) | dav2d_msac_decode_bools_bypass(&ts->msac, eob_bin);
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
        // if luma but inter, this can be coded
        if (chroma) {
            if (intra) {
                const int y_fsc = !sdp_active ? b->fsc :
                    t->luma_fsc_map[(t->cby & 15) * 16 + (t->cbx & 15)];
                *txtp = y_fsc ? IDTX : WHT_WHT;
            } else {
                assert(*txtp == WHT_WHT || *txtp == IDTX || *txtp == IDTX_INV);
                *txtp &= 0xe7; // IDTX_INV -> IDTX
            }
        } else if (intra) {
            *txtp = b->fsc ? IDTX : WHT_WHT;
        } else if (t_dim->max == TX_4X4) {
            *txtp = dav2d_msac_decode_bool_adapt(&ts->msac,
                        ts->cdf.m.txtp_lossless) ? IDTX : WHT_WHT;
        } else {
            *txtp = IDTX;
        }
    } else if (chroma) {
        if (f->seq_hdr->chroma_dctonly) {
            *txtp = DCT_DCT;
        } else {
            // inferred from either the luma txtp (inter) or a LUT (intra)
            if (intra) *txtp = dav2d_txtp_from_uvmode[b->uv_mode];
            if ((t_dim->w >= 8 && *txtp & 0x02 /* horizontal is (flip)adst */) ||
                (t_dim->h >= 8 && *txtp & 0x40 /* vertical is (flip)adst */) ||
                (tx == (int) TX_16X16 &&
                 ((*txtp & 0x47) == 0x41 /* (flip)adst ver, identity hor */ ||
                  (*txtp & 0xe2) == 0x22 /* identity ver, (flip)adst hor */)))
            {
                *txtp = DCT_DCT;
            } else if (*txtp == IDTX_INV) *txtp = IDTX;
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
                                 dav2d_msac_decode_bool_adapt(&ts->msac,
                                     ts->cdf.m.txtp_long32_dct[0]);
            const int short_idx = dav2d_msac_decode_symbol_adapt4(&ts->msac,
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
                dav2d_msac_decode_bool_adapt(&ts->msac,
                    ts->cdf.m.txtp_ext_reduced[t_dim->min]) :
                dav2d_msac_decode_symbol_adapt8(&ts->msac,
                    ts->cdf.m.txtp_ext[t_dim->min], 6);
            static const uint8_t /*enum TxfmType*/ md_idx2type[][13][7] = {
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
            *txtp = md_idx2type[sz_ctx][b->y_mode][tx_idx];
        }
    } else {
        if (t_dim->sub == TX_32X32 /* 64x64, 64x32 or 32x64 */) {
            *txtp = DCT_DCT;
        } else {
            const int y = eob >> (2 + slw), x = eob & ((4 << slw) - 1);
            const int xy = x + y;
            const int ctx = xy < 2 ? 1 : xy > 4 * (imin(8, t_dim->w) +
                                                   imin(8, t_dim->h)) - 4 ? 2 : 0;
            if (tx == (enum RectTxfmSize)TX_32X32) {
                *txtp = dav2d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.txtp_inter_dct_idtx[ctx][TX_32X32]) ?
                        DCT_DCT : IDTX;
            } else if (t_dim->max >= TX_32X32 /* {64,32}x{16,8,4} */) {
                // long64/32
                const int long_dct = t_dim->max == TX_64X64 ||
                                     dav2d_msac_decode_bool_adapt(&ts->msac,
                                         ts->cdf.m.txtp_long32_dct[1]);
                const int short_idx = dav2d_msac_decode_symbol_adapt4(&ts->msac,
                                          ts->cdf.m.txtp_inter_short_1d[ctx]
                                                                [t_dim->min], 3);
                *txtp = txtp_long_tbl[long_dct][t_dim->w < t_dim->h][short_idx];
            } else if (f->frame_hdr->reduced_txtp_set == 1 ||
                       f->frame_hdr->reduced_txtp_set == 2)
            {
                *txtp = dav2d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.txtp_inter_dct_idtx[ctx][t_dim->min]) ?
                        DCT_DCT : IDTX;
            } else if (f->frame_hdr->reduced_txtp_set == 3) {
                const int tx_idx = dav2d_msac_decode_symbol_adapt4(
                    &ts->msac,
                    ts->cdf.m.txtp_inter_dct_idtx_iddct[ctx][t_dim->min], 3);

                static const uint8_t txtp_dct_idtx_iddct_tbl[4] =
                    { DCT_DCT, V_DCT, H_DCT, IDTX };
                *txtp = txtp_dct_idtx_iddct_tbl[tx_idx];
            } else {
                const int setidx = tx == (enum RectTxfmSize)TX_16X16;
                const int set = dav2d_msac_decode_bool_adapt(&ts->msac,
                                    ts->cdf.m.txtp_inter_tx_set[setidx][ctx]
                                                               [t_dim->min]);
                if (!set) {
                    *txtp = dav2d_msac_decode_symbol_adapt8(&ts->msac,
                                ts->cdf.m.txtp_inter_set0[setidx][ctx], 7);
                } else if (setidx) {
                    *txtp = dav2d_msac_decode_symbol_adapt4(&ts->msac,
                                ts->cdf.m.txtp_inter_set2[ctx], 3) + 8;
                } else {
                    *txtp = dav2d_msac_decode_symbol_adapt8(&ts->msac,
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
                    depth, "", dav2d_tx1d_names[*txtp & 7],
                    dav2d_tx1d_names[*txtp >> 5], ts->msac.rng);

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
            stx_type = dav2d_msac_decode_symbol_adapt4(&ts->msac,
                ts->cdf.m.stx[!intra][t_dim->min], 3);
            int stx_set = 0;
            if (stx_type && intra) {
                if (t_dim->min >= TX_8X8 && *txtp == ADST_ADST) {
                    static const uint8_t inv_most_probable_stx_mapping_adst[][7] = {
                        { 3, 1, 0, 2 },  // DC_PRED
                        { 1, 3, 0, 2 },  // V_PRED
                        { 1, 3, 0, 2 },  // H_PRED
                        { 1, 3, 0, 2 },  // D45_PRED
                        { 0, 2, 3, 1 },  // D135_PRED
                        { 2, 1, 0, 3 },  // D113_PRED
                        { 2, 1, 0, 3 },  // D157_PRED
                        { 1, 0, 3, 2 },  // D203_PRED
                        { 1, 0, 3, 2 },  // D67_PRED
                        { 3, 1, 0, 2 },  // SMOOTH_PRED
                        { 1, 3, 0, 2 },  // SMOOTH_V_PRED
                        { 1, 3, 0, 2 },  // SMOOTH_H_PRED
                    };
                    stx_set = dav2d_msac_decode_symbol_adapt4(&ts->msac,
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
                    stx_set = dav2d_msac_decode_symbol_adapt8(&ts->msac,
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
    } else if (f->seq_hdr->cctx && plane == 1 && eob >= intra && !lossless &&
               (f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420 || t_dim->max < 8))
    {
        const int cctx = dav2d_msac_decode_symbol_adapt8(&ts->msac,
                                                         ts->cdf.m.cctx, 6);
        DEBUG_CF_printf("%*sPost-cctx[%d]: r=%d\n",
                        depth, "", cctx, ts->msac.rng);
        *txtp |= cctx << 8;
    }

    // base tokens
    unsigned cul_level = 0;
    int dc_tok;
    const int tcq_enabled = !chroma && f->frame_hdr->tcq &&
                            tx_class == TX_CLASS_2D && !lossless;
    int hr_avg = 0, tcq_state = tcq_enabled * -0x80000000;
    const uint8_t *const qm_tbl = *txtp < IDTX ? f->qm[tx][plane] : NULL;
    int dq_shift = tcq_enabled + 3 + imax(0, t_dim->ctx - 2);
    const uint32_t *const dq_tbl = ts->dq[b->seg_id][plane];
    const int cf_max = ~(~127U << (BITDEPTH == 8 ? 8 : f->cur.p.p.bpc));
    unsigned dc_sign_level = 1 << 6;

    if (f->seq_hdr->fsc && (!intra || b->fsc) &&
        *txtp == IDTX && !chroma)
    {
        assert(!stx_type);
        *txtp = IDTX_INV;
        int8_t *const levels = t->scratch.levels;
        const ptrdiff_t stride = 1 + (4 << slh);
        memset(levels, 0, stride * ((4 << slw) + 1));
        const uint16_t *scan = dav2d_scans[tx];
        const int sz_ctx = imin(t_dim->ctx, 2);
        const int sz = (16 << tx2dszctx) - 1;
        const int bob = sz - eob;
        unsigned ctx = (bob > 2 << tx2dszctx) + (bob > 4 << tx2dszctx);
        uint16_t (*hi_cdf)[4] = ts->cdf.coef.br_y_tok_idtx[sz_ctx];
        int tok = 1 + dav2d_msac_decode_symbol_adapt4(&ts->msac,
                          ts->cdf.coef.bob_base_y_tok[sz_ctx][ctx], 2);
        if (tok == 3) {
            tok += dav2d_msac_decode_symbol_adapt4(&ts->msac, hi_cdf[0], 3);
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
            int tok = dav2d_msac_decode_symbol_adapt4(&ts->msac, lo_cdf[ctx], 3);
            if (tok == 3) {
                tok += dav2d_msac_decode_symbol_adapt4(&ts->msac, hi_cdf[hr_ctx], 3);
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
            int sign = dav2d_msac_decode_bool_adapt(&ts->msac, sign_cdf[ctx]);
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
            tok = 1 + dav2d_msac_decode_symbol_adapt4(&ts->msac, eob_cdf[ctx], 2); \
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
            tok = 1 + dav2d_msac_decode_symbol_adapt4(&ts->msac, eob_cdf[ctx], 4); \
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
            tok += dav2d_msac_decode_symbol_adapt4(&ts->msac, \
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
                tok = dav2d_msac_decode_symbol_adapt8(&ts->msac, lo_cdf.lf[lo_cdf_idx], 5); \
            else \
                tok = dav2d_msac_decode_symbol_adapt4(&ts->msac, lo_cdf.hf[lo_cdf_idx], 3); \
            if (tok == lim && hi_cdf) \
                tok += dav2d_msac_decode_symbol_adapt4(&ts->msac, hi_cdf[hr_ctx], 3); \
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
            dc_tok = dav2d_msac_decode_symbol_adapt8(&ts->msac, lo_cdf.lf[lo_cdf_idx], 5); \
        else \
            dc_tok = dav2d_msac_decode_symbol_adapt4(&ts->msac, lo_cdf.hf[lo_cdf_idx], 3); \
        if (dc_tok == lim && hi_cdf) { \
            dc_tok += dav2d_msac_decode_symbol_adapt4(&ts->msac, hi_cdf[hr_ctx], 3); \
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
                sign = dav2d_msac_decode_bool_bypass(&ts->msac); \
                DEBUG_CF_printf("%*sPost-%ssign[pos=%d,%d]: r=%d\n", \
                                depth, "", (tx_class != TX_CLASS_2D && \
                                            !y) ? "dc_" : "", i, sign, \
                                ts->msac.rng); \
            } else { \
                sign = dav2d_msac_decode_bool_adapt(&ts->msac, \
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
            scan = dav2d_scans[tx];
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
        dc_tok = 1 + dav2d_msac_decode_symbol_adapt8(&ts->msac,
                         ts->cdf.coef.eob_base_uv_tok_lf[0], 4);
        DEBUG_CF_printf("%*sPost-eob_tok[pos=%d,ctx=%d|0|-1,freq=lo,plane=uv,%d]: r=%d\n",
                        depth, "", eob, t_dim->ctx, dc_tok, ts->msac.rng);
    } else {
        dc_tok = 1 + dav2d_msac_decode_symbol_adapt8(&ts->msac,
                         ts->cdf.coef.eob_base_y_tok_lf[t_dim->ctx][0], 4);
        if (dc_tok == 5) {
            dc_tok += dav2d_msac_decode_symbol_adapt4(&ts->msac,
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
        dc_sign = dav2d_msac_decode_bool_bypass(&ts->msac);
        DEBUG_CF_printf("%*sPost-dc_sign[pos=0,%d]: r=%d\n",
                        depth, "", dc_sign, ts->msac.rng);
    } else {
        const int dc_sign_ctx = get_dc_sign_ctx(t_dim, a, l);
        uint16_t *const dc_sign_cdf = ts->cdf.coef.dc_sign[chroma][0][dc_sign_ctx];
        dc_sign = dav2d_msac_decode_bool_adapt(&ts->msac, dc_sign_cdf);
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

static enum IntraPredMode wide_angle_remap(const TxfmInfo *const t_dim,
                                           enum IntraPredMode mode,
                                           int *const angle,
                                           const int mrl_idx)
{
    if ((unsigned) mode - 1 > VERT_LEFT_PRED - 1) return mode;

    // map directional modes
    const int mrl_adj = (mrl_idx == 1) - (mrl_idx == 2);
    *angle = dav2d_mode_to_angle_map[mode - 1] + *angle * 3 + mrl_adj;
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

static int read_luma_tx_cf(Dav2dTaskContext *const t, DB_ONLY(const int depth)
                           const enum RectTxfmSize tx, Av2Block *const b)
{
    const Dav2dFrameContext *const f = t->f;
    Dav2dTileState *const ts = t->ts;
    const int bx4 = t->bx & 63, by4 = t->by & 63;
    const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
    const int tw = t_dim->w * 4, th = t_dim->h * 4;

    const enum IntraPredMode orig_y_mode = b->y_mode;
    int angle = b->y_angle;
    if (b->intra && !b->intrabc)
        b->y_mode = wide_angle_remap(t_dim, b->y_mode, &angle, b->mrl_index);

    // decode coefficients
    uint8_t cf_ctx;
    enum TxfmType txtp;
    coef *cf = ts->frame_thread[0].cf;
    ts->frame_thread[0].cf += imin(tw, 32) * imin(th, 32);
    struct CodedBlockInfo *const cbi =
        &f->frame_thread.cbi[t->by * f->b4_stride + t->bx];
    const int eob = decode_coefs(t, DB_ONLY(depth + 1)
                        &t->a->lcoef[bx4], &t->l.lcoef[by4],
                        tx, b->bs, 0, b, 0, cf, &txtp, &cf_ctx);
    if (eob == INT_MIN) return -1;
    cbi->txtp[0] = txtp;
    cbi->eob[0] = eob;
    txtp &= 0xff;
    DEBUG_BLOCK_printf("%*sPost-y_cf_blk[tx=%dx%d,txtp=%s/%s,eob=%d]: r=%d\n",
                       depth + 1, "", tw, th,
                       dav2d_tx1d_names[txtp & 7],
                       dav2d_tx1d_names[txtp >> 5],
                       eob, ts->msac.rng);
    dav2d_memset_likely_pow2(&t->a->lcoef[bx4], cf_ctx,
                             imin(t_dim->w, f->bw - t->bx));
    dav2d_memset_likely_pow2(&t->l.lcoef[by4], cf_ctx,
                             imin(t_dim->h, f->bh - t->by));
    uint8_t *txtp_map = &t->txtp_map[(t->by & 15) * 16 + (t->bx & 15)];
#define set_ctx(rep_macro) \
    for (int y = 0; y < t_dim->h; y++) { \
        rep_macro(txtp_map, 0, txtp); \
        txtp_map += 16; \
    }
    case_set(t_dim->lw);
#undef set_ctx

    b->y_mode = orig_y_mode;

    return 0;
}

int bytefn(dav2d_read_coef_blocks)(Dav2dTaskContext *const t,
                                   DB_ONLY(const int depth)
                                   const enum BlockSize lbs,
                                   const enum BlockSize cbs,
                                   Av2Block *const b)
{
    const enum BlockSize bs = lbs == BS_INVALID ? cbs : lbs;
    assert(bs != BS_INVALID);
    const int has_luma = lbs != BS_INVALID, has_chroma = cbs != BS_INVALID;
    const Dav2dFrameContext *const f = t->f;
    const int bx4 = t->bx & 63, by4 = t->by & 63;
    const uint8_t *const b_dim = dav2d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);

    if (has_luma && b->skip_txfm) {
        BlockContext *const a = t->a;
        dav2d_memset_pow2[b_dim[2]](&a->lcoef[bx4], 0x40);
        dav2d_memset_pow2[b_dim[3]](&t->l.lcoef[by4], 0x40);
        if (has_chroma) {
            const int ss_ver = f->ss_ver, ss_hor = f->ss_hor;
            const uint8_t *const cb_dim = dav2d_block_dimensions[cbs];
            const int cbx4ss = (t->cbx & 63) >> ss_hor, cby4ss = (t->cby & 63) >> ss_ver;

            dav2d_memset_pow2_fn memset_cw = dav2d_memset_pow2[cb_dim[2] - ss_hor];
            dav2d_memset_pow2_fn memset_ch = dav2d_memset_pow2[cb_dim[3] - ss_ver];
            memset_cw(&a->ccoef[0][cbx4ss], 0x40);
            memset_cw(&a->ccoef[1][cbx4ss], 0x40);
            memset_ch(&t->l.ccoef[0][cby4ss], 0x40);
            memset_ch(&t->l.ccoef[1][cby4ss], 0x40);
        }
        return 0;
    }

    const uint8_t csplit[3][3] = {
        [BS_128x128 - BS_128x128] = {  BS_64x64, BS_128x64, BS_128x128 },
        [BS_128x64  - BS_128x128] = {  BS_64x64, BS_128x64, BS_128x64  },
        [BS_64x128  - BS_128x128] = {  BS_64x64, BS_64x64,  BS_64x128  },
    };
    if (imax(bw4, bh4) > 16) {
        const int ss_ver = f->ss_ver, ss_hor = f->ss_hor;
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
            cbs2i = cbs == BS_INVALID ? BS_INVALID :
                    csplit[cbs - BS_128x128][ss_hor + ss_ver];
        }
        for (int y = 0; t->by < y_end; t->by += step, y++) {
            for (int x = 0; t->bx < x_end; t->bx += step, x++) {
                enum BlockSize cbs2;
                if (step == 32) {
                    cbs2 = cbs2i;
                } else {
                    // coef reading is done with the first luma 64x64
                    cbs2 = !((x & ss_hor) | (y & ss_ver)) ? cbs2i : BS_INVALID;
                }
                const int res = bytefn(dav2d_read_coef_blocks)(t, DB_ONLY(depth) lbs2, cbs2, b);
                if (step == 32) {
                    t->cbx += step;
                } else if ((x & ss_hor) == ss_hor) {
                    t->cbx += step << ss_hor;
                }
                if (res < 0) {
                    t->cbx = t->bx = x_start;
                    t->cby = t->by = y_start;
                    return res;
                }
            }
            t->cbx = t->bx = x_start;
            if (step == 32) {
                t->cby += step;
            } else if ((y & ss_ver) == ss_ver) {
                t->cby += step << ss_ver;
            }
        }
        t->cby = t->by = y_start;
        return 0;
    }

    if (lbs == BS_INVALID) goto chroma;

    const int8_t *const tp = dav2d_tx_part_tbl[bs];
    if (tp[b->tx_part] == -1) return -1;

    // luma
    enum RectTxfmSize tx = tp[b->tx_part];
    t->pb.col_start = t->bx;
    t->pb.row_start = t->by;
    if (f->frame_hdr->segmentation.lossless[b->seg_id]) {
        int res = 0, y, x;
        tx = b->tx_size_ll ? dav2d_max_txfm_size_for_bs[bs][3] : (int) TX_4X4;
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int tw4 = t_dim->w, th4 = t_dim->h;
        for (y = 0; y < h4 && !res; y += th4, t->by += th4) {
            for (x = 0; x < w4 && !res; x += tw4, t->bx += tw4) {
                res = read_luma_tx_cf(t, DB_ONLY(depth) (int) tx, b);
            }
            t->bx -= x;
        }
        t->by -= y;
        if (res < 0) return res;
    } else switch (b->tx_part) {
    case TX_PARTITION_NONE: {
        const int res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_SPLIT: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int tw4 = t_dim->w, th4 = t_dim->h;
        int res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        const int have_v_split = t->bx + tw4 < f->bw;
        if (have_v_split) {
            t->bx += tw4;
            res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4;
            if (res < 0) return res;
        }
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (!res && have_v_split) {
            t->bx += tw4;
            res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4;
        }
        t->by -= th4;
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_H: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int th4 = t_dim->h;
        int res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        t->by -= th4;
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_V: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int tw4 = t_dim->w;
        int res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        if (t->bx + tw4 >= f->bw) break;
        t->bx += tw4;
        res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        t->bx -= tw4;
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_H4: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int th4 = t_dim->h;
        int res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0 || t->by + th4 >= f->bh) {
            t->by -= th4;
        } else {
            t->by += th4;
            res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
            if (res < 0 || t->by + th4 >= f->bh) {
                t->by -= 2 * th4;
            } else {
                t->by += th4;
                res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
                t->by -= 3 * th4;
            }
        }
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_V4: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int tw4 = t_dim->w;
        int res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        if (t->bx + tw4 >= f->bw) break;
        t->bx += tw4;
        res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0 || t->bx + tw4 >= f->bw) {
            t->bx -= tw4;
        } else {
            t->bx += tw4;
            res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
            if (res < 0 || t->bx + tw4 >= f->bw) {
                t->bx -= 2 * tw4;
            } else {
                t->bx += tw4;
                res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
                t->bx -= 3 * tw4;
            }
        }
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_H5: {
        const enum RectTxfmSize tx_big = tp[TX_PARTITION_H];
        const TxfmInfo *const t_dim_small = &dav2d_txfm_dimensions[tx],
                       *const t_dim_big = &dav2d_txfm_dimensions[tx_big];
        const int tw4_small = t_dim_small->w, th4_small = t_dim_small->h;
        const int th4_big = t_dim_big->h;
        int res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        const int have_v_split = t->bx + tw4_small < f->bw;
        if (have_v_split) {
            t->bx += tw4_small;
            res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4_small;
            if (res < 0) return res;
        }
        if (t->by + th4_small >= f->bh) break;
        t->by += th4_small;
        res = read_luma_tx_cf(t, DB_ONLY(depth) tx_big, b);
        if (res < 0 || t->by + th4_big >= f->bh) {
            t->by -= th4_small;
        } else {
            t->by += th4_big;
            res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
            if (!res && have_v_split) {
                t->bx += tw4_small;
                res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
                t->bx -= tw4_small;
            }
            t->by -= th4_small + th4_big;
        }
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_V5: {
        const enum RectTxfmSize tx_big = tp[TX_PARTITION_V];
        const TxfmInfo *const t_dim_small = &dav2d_txfm_dimensions[tx],
                       *const t_dim_big = &dav2d_txfm_dimensions[tx_big];
        const int tw4_small = t_dim_small->w, th4_small = t_dim_small->h;
        const int tw4_big = t_dim_big->w;
        int res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        const int have_h_split = t->by + th4_small < f->bh;
        if (have_h_split) {
            t->by += th4_small;
            res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
            t->by -= th4_small;
            if (res < 0) return res;
        }
        if (t->bx + tw4_small >= f->bw) break;
        t->bx += tw4_small;
        res = read_luma_tx_cf(t, DB_ONLY(depth) tx_big, b);
        if (res < 0 || t->bx + tw4_big >= f->bw) {
            t->bx -= tw4_small;
        } else {
            t->bx += tw4_big;
            res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
            if (!res && have_h_split) {
                t->by += th4_small;
                res = read_luma_tx_cf(t, DB_ONLY(depth) tx, b);
                t->by -= th4_small;
            }
            t->bx -= tw4_small + tw4_big;
        }
        if (res < 0) return res;
        break;
    }
    default: assert(0);
    }

    if (cbs == BS_INVALID) return 0;

    // chroma
chroma: {}
    const int ss_ver = f->ss_ver, ss_hor = f->ss_hor;
    const uint8_t *const cb_dim = dav2d_block_dimensions[cbs];
    const int cbw4 = cb_dim[0], cw4 = imin(f->bw - t->cbx, cbw4);
    const int cbh4 = cb_dim[1], ch4 = imin(f->bh - t->cby, cbh4);
    const int cbw4ss = (cbw4 + ss_hor) >> ss_hor, cbh4ss = (cbh4 + ss_ver) >> ss_ver;
    const int cw4ss = (cw4 + ss_hor) >> ss_hor, ch4ss = (ch4 + ss_ver) >> ss_ver;
    const enum RectTxfmSize uvtx =
        f->frame_hdr->segmentation.lossless[b->seg_id] ? (int) TX_4X4 :
        dav2d_max_txfm_size_for_bs[cbs][DAV2D_PIXEL_LAYOUT_I444 - f->cur.p.p.layout];
    const TxfmInfo *const uv_t_dim = &dav2d_txfm_dimensions[uvtx];
    const int sdp_active = lbs == BS_INVALID;
    const int intra = b->intra && (sdp_active || !b->intrabc);
    const int cbx4 = t->cbx & 63, cby4 = t->cby & 63;
    const int cbx4ss = cbx4 >> ss_hor, cby4ss = cby4 >> ss_ver;
    const int ctw4 = imin(uv_t_dim->w, (f->bw - t->cbx + ss_hor) >> ss_hor);
    const int cth4 = imin(uv_t_dim->h, (f->bh - t->cby + ss_ver) >> ss_ver);
    const enum IntraPredMode orig_uv_mode = b->uv_mode;
    int angle = b->uv_angle;
    if (intra)
        b->uv_mode = wide_angle_remap(uv_t_dim, b->uv_mode, &angle, 0);

    Dav2dTileState *const ts = t->ts;
    enum TxfmType y_txtp = t->txtp_map[(t->by & 15) * 16 + (t->bx & 15)];
    coef *cf[2] = { ts->frame_thread[0].cf };
    ts->frame_thread[0].cf += cbw4ss * cbh4ss * 16 * 2;
    cf[1] = &cf[0][cbw4ss * cbh4ss * 16];
    // decode coefficients
    for (int pl = 0; pl < 2; pl++) {
        int y;
        for (y = 0; y < ch4ss; y += uv_t_dim->h) {
            int x;
            for (x = 0; x < cw4ss; x += uv_t_dim->w) {
                const ptrdiff_t i = y * cbw4ss + x;
                if (b->bs == b->cbs)
                    y_txtp = t->txtp_map[(t->by & 15) * 16 + (t->bx & 15)];
                enum TxfmType uv_txtp = y_txtp;
                uint8_t cf_ctx;
                const int eob =
                    decode_coefs(t, DB_ONLY(depth + 1)
                                 &t->a->ccoef[pl][cbx4ss + x],
                                 &t->l.ccoef[pl][cby4ss + y],
                                 uvtx, b->cbs, sdp_active, b, pl + 1,
                                 &cf[pl][i * 16], &uv_txtp, &cf_ctx);
                if (!pl) t->u_has_cf = eob >= 0;
                struct CodedBlockInfo *const cbi =
                    &f->frame_thread.cbi[(t->cby + (y << ss_ver)) * f->b4_stride +
                                         (t->cbx + (x << ss_hor))];
                cbi->txtp[pl + 1] = uv_txtp;
                if (eob == INT_MIN) return -1;
                DEBUG_BLOCK_printf("%*sPost-%c_cf_blk[tx=%dx%d,txtp=%s/%s,"
                                   "eob=%d]: r=%d\n",
                                   depth + 1, "", "uv"[pl],
                                   uv_t_dim->w * 4, uv_t_dim->h * 4,
                                   dav2d_tx1d_names[uv_txtp & 7],
                                   dav2d_tx1d_names[(uv_txtp >> 5) & 7],
                                   eob, t->ts->msac.rng);
                cbi->eob[pl + 1] = eob;
                dav2d_memset_likely_pow2(&t->a->ccoef[pl][cbx4ss + x],
                                         cf_ctx, ctw4);
                dav2d_memset_likely_pow2(&t->l.ccoef[pl][cby4ss + y],
                                         cf_ctx, cth4);
                t->bx += uv_t_dim->w << ss_hor;
            }
            t->bx -= x << ss_hor;
            t->by += uv_t_dim->h << ss_ver;
        }
        t->by -= y << ss_ver;
    }

    b->uv_mode = orig_uv_mode;

    return 0;
}

static void mc(Dav2dTaskContext *const t,
               pixel *dst8, int16_t *const dst16, const ptrdiff_t dst_stride,
               int bw4, const int bh4,
               const int bx, const int by, const int pl,
               const mv mv, const Dav2dThreadPicture *const refp, const int refidx,
               const enum Dav2dFilterMode filter,
               const int left, const int right, const int top, const int bottom)
{
    assert((dst8 != NULL) ^ (dst16 != NULL));
    const Dav2dFrameContext *const f = t->f;
    const int ss_ver = !!pl && f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
    const int ss_hor = !!pl && f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444;
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;
    const int mvx = mv.x, mvy = mv.y;
    ptrdiff_t ref_stride = refp->p.stride[!!pl];
    const pixel *ref;

    assert(left >= 0 && top >= 0 && left < right && top < bottom &&
           right <= f->bw * 4 && bottom <= f->bh * 4);

    if (refp->p.p.w == f->cur.p.p.w && refp->p.p.h == f->cur.p.p.h) {
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
                       filter == DAV2D_FILTER_BILINEAR);
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
        assert(refp != &f->cur);

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
static void mc_opfl(Dav2dTaskContext *const t,
                    int16_t *const dst16, const ptrdiff_t dst_stride,
                    const int bw4, const int bh4, const int bx4, const int by4,
                    const int pl, const mv mv, const Dav2dThreadPicture *const refp,
                    const enum Dav2dFilterMode filter,
                    const int left, const int right, const int top, const int bottom)
{
    const Dav2dFrameContext *const f = t->f;
    assert(refp->p.p.w == f->cur.p.p.w && refp->p.p.h == f->cur.p.p.h);
    const int mvx = mv.x, mvy = mv.y;
    ptrdiff_t ref_stride = refp->p.stride[!!pl];
    const pixel *ref;
    const int mx = mvx & 15, my = mvy & 15;
    const int dx = bx4 * 4 + (mvx >> 4);
    const int dy = by4 * 4 + (mvy >> 4);
    assert(top >= 0 && left >= 0 && left < right && top < bottom &&
           right <= (f->bw * 4) >> !!pl * f->ss_hor &&
           bottom <= (f->bh * 4) >> !!pl * f->ss_ver);

    if (dx - !!mx * 3 < left || dy - !!my * 3 < top ||
        dx + bw4 * 4 + !!mx * 4 > right ||
        dy + bh4 * 4 + !!my * 4 > bottom)
    {
        pixel *const emu_edge_buf = bitfn(t->scratch.emu_edge);
        ref = refp->p.data[pl];
        f->dsp->mc.emu_edge(bw4 * 4 + !!mx * 7, bh4 * 4 + !!my * 7,
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

    f->dsp->mc.mct[filter](dst16, dst_stride, ref, ref_stride,
                           bw4 * 4, bh4 * 4, mx, my
                           HIGHBD_CALL_SUFFIX);
}

static void ext_warp(Dav2dTaskContext *const t,
                     pixel *dst8, int16_t *dst16, const ptrdiff_t dstride,
                     const uint8_t *const b_dim, const int pl,
                     const Dav2dThreadPicture *const refp,
                     const Dav2dWarpedMotionParams *const wmp)
{
    assert((dst8 != NULL) ^ (dst16 != NULL));
    const Dav2dFrameContext *const f = t->f;
    const Dav2dDSPContext *const dsp = f->dsp;
    const int ss_ver = !!pl && f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
    const int ss_hor = !!pl && f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444;
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;
    assert(!((b_dim[0] * h_mul) & 3) && !((b_dim[1] * v_mul) & 3));
    const int32_t *const mat = wmp->matrix;
    const int w = f->bw * 4 >> ss_hor;
    const int h = f->bh * 4 >> ss_ver;
    const int sw = imin(b_dim[0] * h_mul, 8), hsw = sw >> 1;
    const int sh = imin(b_dim[1] * v_mul, 8), hsh = sh >> 1;
    const int bx = pl ? t->cbx : t->bx, by = pl ? t->cby : t->by;

    for (int y = 0; y < b_dim[1] * v_mul; y += sh) {
        const int src_y = by * 4 + ((y + hsh) << ss_ver);
        const int64_t mat3_y = (int64_t) mat[3] * src_y + mat[0];
        const int64_t mat5_y = (int64_t) mat[5] * src_y + mat[1];
        for (int x = 0; x < b_dim[0] * h_mul; x += sw) {
            // calculate transformation relative to center of 8x8 block in
            // luma pixel units
            const int src_x = bx * 4 + ((x + hsw) << ss_hor);
            const int64_t mvx = ((int64_t) mat[2] * src_x + mat3_y) >> ss_hor;
            const int64_t mvy = ((int64_t) mat[4] * src_x + mat5_y) >> ss_ver;
            const int left_window = (int) (mvx >> 16) - hsw - 3;
            const int top_window = (int) (mvy >> 16) - hsh - 3;
            const int left = iclip(left_window, 0, w - 1);
            const int right = iclip(left_window + sw + 7, 1, w);
            const int top = iclip(top_window, 0, h - 1);
            const int bottom = iclip(top_window + sh + 7, 1, h);

            for (int yy = y; yy < y + sh; yy += 4) {
                const int src_y = by * 4 + ((yy + 2) << ss_ver);
                const int64_t mat3_y = (int64_t) mat[3] * src_y + mat[0];
                const int64_t mat5_y = (int64_t) mat[5] * src_y + mat[1];
                for (int xx = x; xx < x + sw; xx += 4) {
                    const int src_x = bx * 4 + ((xx + 2) << ss_hor);
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
                                            emu_edge_buf, 32 * sizeof(pixel),
                                            &ref_ptr[left + top * PXSTRIDE(ref_stride)],
                                            ref_stride);
                        ref_ptr = &emu_edge_buf[32 * 3 + 3];
                        ref_stride = 32 * sizeof(pixel);
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

static void warp_affine(Dav2dTaskContext *const t,
                        pixel *dst8, int16_t *dst16, const ptrdiff_t dstride,
                        const uint8_t *const b_dim, const int pl,
                        const Dav2dThreadPicture *const refp,
                        const Dav2dWarpedMotionParams *const wmp)
{
    assert((dst8 != NULL) ^ (dst16 != NULL));
    const Dav2dFrameContext *const f = t->f;
    const int ss_ver = !!pl && f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
    const int ss_hor = !!pl && f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444;
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;

    if (!wmp->affine || imin(b_dim[0] * h_mul, b_dim[1] * v_mul) < 8) {
        ext_warp(t, dst8, dst16, dstride, b_dim, pl, refp, wmp);
        return;
    }

    const Dav2dDSPContext *const dsp = f->dsp;
    assert(!((b_dim[0] * h_mul) & 7) && !((b_dim[1] * v_mul) & 7));
    const int32_t *const mat = wmp->matrix;
    const int width = f->bw * 4 >> ss_hor;
    const int height = f->bh * 4 >> ss_ver;
    const int bx = pl ? t->cbx : t->bx, by = pl ? t->cby : t->by;

    for (int y = 0; y < b_dim[1] * v_mul; y += 8) {
        const int src_y = by * 4 + ((y + 4) << ss_ver);
        const int64_t mat3_y = (int64_t) mat[3] * src_y + mat[0];
        const int64_t mat5_y = (int64_t) mat[5] * src_y + mat[1];
        for (int x = 0; x < b_dim[0] * h_mul; x += 8) {
            // calculate transformation relative to center of 8x8 block in
            // luma pixel units
            const int src_x = bx * 4 + ((x + 4) << ss_hor);
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

static ALWAYS_INLINE int get_mask(uint8_t *const mask, const ptrdiff_t stride,
                                  const int bx4, const int x4,
                                  const int by4, const int y4,
                                  const union mv mv[2],
                                  const int h_subpel_bits, const int v_subpel_bits,
                                  const int bw4, const int bh4,
                                  const int iw, const int ih)
{
    const int x0 = (bx4 + x4) * 4 + (mv[0].x >> h_subpel_bits);
    const int y0 = (by4 + y4) * 4 + (mv[0].y >> v_subpel_bits);
    const int x1 = (bx4 + x4) * 4 + (mv[1].x >> h_subpel_bits);
    const int y1 = (by4 + y4) * 4 + (mv[1].y >> v_subpel_bits);
    if (x0 < 0 || x1 < 0 || y0 < 0 || y1 < 0 ||
        x0 + bw4 * 4 >= iw || x1 + bw4 * 4 >= iw ||
        y0 + bh4 * 4 >= ih || y1 + bh4 * 4 >= ih)
    {
        gen_mask(&mask[(y4 * stride + x4) * 4], stride,
                 bw4 * 4, bh4 * 4, x0, y0, x1, y1, iw, ih);
        return 1;
    }
    return 0;
}

static void update_temporal(const Dav2dFrameContext *const f,
                            refmvs_temporal_block *t_dst, const ptrdiff_t t_stride,
                            const int w8, const int h8, const union refpair ref,
                            const union mv mv[2], const int swap)
{
    if (!f->seq_hdr->ref_frame_mvs) return;

    refmvs_temporal_block t_src;
    t_src.ref.ref[0] = ref.ref[swap];
    t_src.ref.ref[1] = ref.ref[!swap];
    t_src.mv.mv[0] = quantize_mv(mv[swap]);
    t_src.mv.mv[1] = quantize_mv(mv[!swap]);
    if (t_src.mv.mv[0].n == INVALID_TRAJ) {
        if (t_src.mv.mv[1].n == INVALID_TRAJ) {
            t_src.ref.pair = -1;
        } else {
            t_src.mv.mv[0] = t_src.mv.mv[1];
            t_src.ref.ref[0] = t_src.ref.ref[1];
        }
    } else if (t_src.mv.mv[1].n == INVALID_TRAJ) {
        t_src.mv.mv[1] = t_src.mv.mv[0];
        t_src.ref.ref[1] = t_src.ref.ref[0];
    }
    for (int y = 0; y < h8; y++) {
        for (int x = 0; x < w8; x++)
            t_dst[x] = t_src;
        t_dst += t_stride;
    }
}

union OpflMvDeltaBlock {
    struct OpflMvDelta {
        int8_t x, y;
    } d[2];
    uint32_t n;
};

static void opfl_mv_adj(const struct OpflRegressionData *const r,
                        union OpflMvDeltaBlock *const dd, const union aliasi16 d)
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
        const int idet = dav2d_resolve_divisor_32(det, &shift), idet_bits = ulog2(idet);
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
        dd->d[0].x = -iclip(d.i8[0] * s[0], -16, 16);
        dd->d[0].y = -iclip(d.i8[0] * s[1], -16, 16);
        dd->d[1].x = +iclip(d.i8[1] * s[0], -16, 16);
        dd->d[1].y = +iclip(d.i8[1] * s[1], -16, 16);
    } else dd->n = 0;
}

static ALWAYS_INLINE void
scaledown_16pel_mv_for_chroma(union mv *const mv, const enum Dav2dPixelLayout layout) {
    switch (layout) {
    case DAV2D_PIXEL_LAYOUT_I420:
        for (int i = 0; i < 2; i++)
            mv[i].y = (mv[i].y + (mv[i].y > 0)) >> 1;
        // fall-through
    case DAV2D_PIXEL_LAYOUT_I422:
        for (int i = 0; i < 2; i++)
            mv[i].x = (mv[i].x + (mv[i].x > 0)) >> 1;
        break;
    default: break;
    }
}

static ALWAYS_INLINE void
scaleup_8pel_mv_for_chroma(union mv *const mv, const enum Dav2dPixelLayout layout) {
    switch (layout) {
    case DAV2D_PIXEL_LAYOUT_I444:
        for (int i = 0; i < 2; i++) mv[i].x <<= 1;
        // fall-through
    case DAV2D_PIXEL_LAYOUT_I422:
        for (int i = 0; i < 2; i++) mv[i].y <<= 1;
        break;
    default: break;
    }
}

static int tip_pred(Dav2dTaskContext *const t,
                    int16_t (*const tmp)[64 * 64], const Av2Block *const b,
                    const int bw4, const int bh4, const int w4, const int h4)
{
    const Dav2dFrameContext *const f = t->f;
    int opfl = f->seq_hdr->tip_refine_mv &&
        (f->frame_hdr->tip.frame_mode == 1 ||
         f->frame_hdr->tip.subpel_filter == DAV2D_FILTER_8TAP_SHARP);
    const union refpair ref = f->rf.tip.ref;
    const int refine = opfl && f->frame_hdr->tip.frame_mode == 1 &&
                       f->refdist[ref.ref[0]] == -f->refdist[ref.ref[1]];
    const int step = 2 << (f->frame_hdr->tip.frame_mode == 2 /* frame */ ? !opfl :
                           ((!opfl && imin(bw4, bh4) >= 4) || b->bs == BS_256x256));
    opfl &= !!f->seq_hdr->opfl_refine && f->frame_hdr->has_bothside_refs;
    ptrdiff_t off_y = 0;
    uint8_t *const mask = t->scratch.seg_mask;
    const int bacp = f->seq_hdr->imp_msk_bld && b->cwp_idx == 8 &&
        !f->svc[ref.ref[0]][0].scale && !f->svc[ref.ref[1]][0].scale;
    const int w = f->bw * 4, h = f->bh * 4;
    if (bacp) memset(mask, 0x20, bw4 * bh4 * 16);
    int have_bacp = 0;

    const Dav2dThreadPicture *const refp[2] = { &f->refp[ref.ref[0]], &f->refp[ref.ref[1]] };
    pixel *p[2];
    ptrdiff_t p_stride;
    union aliasi16 d;
    if (opfl) {
        p[0] = bitfn(t->scratch.p)[0];
        p[1] = bitfn(t->scratch.p)[1];
        p_stride = ((step + 2) * 4 * sizeof(pixel) + 63) & ~63;
        const int d0 = f->absrefdist[ref.ref[0]], d1 = f->absrefdist[ref.ref[1]];
        d.i8[0] = apply_sign(1 + (d0 > d1), -f->refdist[ref.ref[0]]);
        d.i8[1] = apply_sign(1 + (d1 > d0), +f->refdist[ref.ref[1]]);
    }

    union mv (*rmv_line)[2][2] = &t->rmv[((t->by & 31) >> 1) * 16 + ((t->bx & 31) >> 1)];
    const unsigned sad8x8_thr = f->frame_hdr->tip.frame_mode == 1 /* reference */ ? 6 : 15;
    const ptrdiff_t t_stride = f->rf.rp_stride;
    refmvs_temporal_block *t_dst = &f->rf.rp[(t->by >> 1) * t_stride + (t->bx >> 1)];
    const int t_swap = !!(f->rf.ref_flip & (1ULL << (ref.ref[0] * 8 + ref.ref[1])));
    for (int y = 0, yy = 0; y < h4; y += step, yy++, rmv_line += 16 * step >> 1) {
        const ptrdiff_t off_y8 = (((t->by + y) & (f->sb_step - 1)) >> 1) * t_stride;
        for (int x = 0; x < w4; x += step) {
            const ptrdiff_t off_8x8 = off_y8 + ((t->bx + x) >> 1);
            mv tmv = t->rt.rp_proj[off_8x8].mv;
            if (tmv.y == INVALID_MV) tmv.n = 0;
            union mv (*const rmv)[2] = rmv_line[x >> 1], *const cmv = rmv[0];
            int left[2], top[2];
            for (int i = 0; i < 2; i++) {
                const mv tipmv = scale_mv(tmv, f->rf.tip.sf[i]);
                rmv[1][i].y = cmv[i].y = iclip(tipmv.y + b->mv[0].y, -0xffff, 0xffff);
                rmv[1][i].x = cmv[i].x = iclip(tipmv.x + b->mv[0].x, -0xffff, 0xffff);
                top[i] = t->by * 4 + y * 4 + (cmv[i].y >> 3) - 3;
                left[i] = t->bx * 4 + x * 4 + (cmv[i].x >> 3) - 3;
            }
            scaleup_8pel_mv_for_chroma(rmv[1], f->cur.p.p.layout);
            if (opfl) {
                // refinement
                for (int i = 0; i < 2; i++)
                    mc(t, p[i], NULL, p_stride,
                       step + 2, step + 2, t->bx + x, t->by + y, 0,
                       (union mv) { .y = cmv[i].y - 32, .x = cmv[i].x - 32 },
                       refp[i], ref.ref[i], DAV2D_FILTER_BILINEAR,
                       iclip(left[i], 0, w - 1), iclip(left[i] + 7 + step * 4, 1, w),
                       iclip(top[i], 0, h - 1), iclip(top[i] + 7 + step * 4, 1, h));
                int dy, dx;
                if (refine) {
                    struct OpflOffset o;
                    f->dsp->mc.sad_refine_mv(p[0], p_stride, p[1], p_stride,
                                             step * 4, step * 4, 1, &o
                                             HIGHBD_CALL_SUFFIX);
                    dy = o.y;
                    cmv[0].y += 8 * dy;
                    cmv[1].y -= 8 * dy;
                    dx = o.x;
                    cmv[0].x += 8 * dx;
                    cmv[1].x -= 8 * dx;
                } else dy = dx = 0;
                union OpflMvDeltaBlock dd;
                const unsigned sad = b->bs == BS_256x256 && f->frame_hdr->tip.frame_mode == 1 ? 0 :
                                     f->dsp->mc.sad8x8(&p[0][(4 + dy) * PXSTRIDE(p_stride) +
                                                             (4 + dx)], p_stride,
                                                       &p[1][(4 - dy) * PXSTRIDE(p_stride) +
                                                             (4 - dx)], p_stride
                                                       HIGHBD_CALL_SUFFIX);
                if (sad >= sad8x8_thr) {
                    struct OpflRegressionData res[4];
                    f->dsp->mc.opfl_derive_mv(res,
                                              &p[0][(4 + dy) * PXSTRIDE(p_stride) +
                                                    (4 + dx)], p_stride,
                                              &p[1][(4 - dy) * PXSTRIDE(p_stride) +
                                                    (4 - dx)], p_stride,
                                              step * 4, step * 4, 8, d
                                              HIGHBD_CALL_SUFFIX);
                    opfl_mv_adj(res, &dd, d);
                } else {
                    dd.n = 0;
                }
                cmv[0].x = cmv[0].x * 2 + dd.d[0].x;
                cmv[0].y = cmv[0].y * 2 + dd.d[0].y;
                cmv[1].x = cmv[1].x * 2 + dd.d[1].x;
                cmv[1].y = cmv[1].y * 2 + dd.d[1].y;
                for (int i = 0; i < 2; i++)
                    mc_opfl(t, &tmp[i][y * bw4 * 16 + x * 4], bw4 * 4, step, step,
                            t->bx + x, t->by + y, 0, cmv[i], refp[i], b->filter,
                            iclip(left[i], 0, w - 1),
                            iclip(left[i] + 7 + step * 4, 1, w),
                            iclip(top[i], 0, h - 1),
                            iclip(top[i] + 7 + step * 4, 1, h));
                const union mv dmv[2] = {
                    [0] = { .y = (cmv[0].y + (dd.d[0].y > 0)) >> 1,
                            .x = (cmv[0].x + (dd.d[0].x > 0)) >> 1 },
                    [1] = { .y = (cmv[1].y + (dd.d[1].y > 0)) >> 1,
                            .x = (cmv[1].x + (dd.d[1].x > 0)) >> 1 },
                };
                update_temporal(f, &t_dst[x >> 1], t_stride, step >> 1, step >> 1,
                                ref, dmv, t_swap);
                if (bacp)
                    have_bacp |= get_mask(mask, bw4 * 4, t->bx, x, t->by, y,
                                          cmv, 4, 4, step, step, w, h);
                scaledown_16pel_mv_for_chroma(cmv, f->cur.p.p.layout);
            } else {
                for (int i = 0; i < 2; i++)
                    mc(t, NULL, &tmp[i][off_y + x * 4], bw4 * 4,
                       step, step, t->bx + x, t->by + y, 0,
                       cmv[i], refp[i], ref.ref[i], b->filter,
                       0, f->bw * 4, 0, f->bh * 4);
                // when refinement is disabled, each sub-block in the temporal
                // MV buffer gets its own 8x8 tip MV even if the tip blocksize
                // is 16x16 (see #945)
                update_temporal(f, &t_dst[x >> 1], t_stride, step >> 1,
                                step >> 1, ref, cmv, t_swap);
                if (step == 4 && f->frame_hdr->tip.frame_mode == 1 /* reference */) {
                    union mv dmv[2];
                    for (int p = 1; p < 4; p++) {
                        mv tmv = t->rt.rp_proj[off_8x8 + (p & 1) +
                                               ((p & 2) >> 1) * t_stride].mv;
                        if (tmv.y == INVALID_MV) tmv.n = 0;
                        for (int i = 0; i < 2; i++) {
                            const mv tipmv = scale_mv(tmv, f->rf.tip.sf[i]);
                            dmv[i].y = iclip(tipmv.y + b->mv[0].y, -0xffff, 0xffff);
                            dmv[i].x = iclip(tipmv.x + b->mv[0].x, -0xffff, 0xffff);
                        }
                        update_temporal(f, &t_dst[((p & 2) >> 1) * t_stride +
                                                  (x >> 1) + (p & 1)], t_stride, 1,
                                        1, ref, dmv, t_swap);
                    }
                }
                if (bacp)
                    have_bacp |= get_mask(mask, bw4 * 4, t->bx, x, t->by, y,
                                          cmv, 3, 3, step, step, w, h);
                scaleup_8pel_mv_for_chroma(cmv, f->cur.p.p.layout);
            }
        }
        off_y += bw4 * 4 * 4 * step;
        t_dst += (step >> 1) * t_stride;
    }
    return bacp && have_bacp;
}

static int opfl_pred(Dav2dTaskContext *const t,
                     int16_t (*const tmp)[64 * 64], const Av2Block *const b,
                     const int bw4, const int bh4, const int w4, const int h4)
{
    const Dav2dFrameContext *const f = t->f;
    assert(!f->svc[b->ref.ref[0]][0].scale && !f->svc[b->ref.ref[1]][0].scale);
    const int refine = b->comp_type == COMP_INTER_AVG && b->refine_mv;
    const int opfl = b->inter_mode >= OPFL_NEARMV_NEARMV;
    assert(opfl || refine);
    assert(bw4 >= 2 && bh4 >= 2);
    const int w = f->bw * 4, h = f->bh * 4;
    pixel *p[2] = { bitfn(t->scratch.p)[0], bitfn(t->scratch.p)[1] };
    const ptrdiff_t p_stride = ((bw4 + refine * 2) * 4 * sizeof(pixel) + 63) & ~63;

    const Dav2dThreadPicture *refp[2] = { &f->refp[b->ref.ref[0]],
                                          &f->refp[b->ref.ref[1]] };
    int top[2] = { t->by * 4 + (b->mv[0].y >> 3) - 3,
                   t->by * 4 + (b->mv[1].y >> 3) - 3 };

    // FIXME namespace bacp symbols
    uint8_t *const mask = t->scratch.seg_mask;
    const int bacp = f->seq_hdr->imp_msk_bld && b->cwp_idx == 8;
    if (bacp) memset(mask, 0x20, bw4 * bh4 * 16);
    int have_bacp = 0;

    // FIXME namespace opfl symbols
    // find reduced distance as inverse weights
    const int d0 = f->absrefdist[b->ref.ref[0]], d1 = f->absrefdist[b->ref.ref[1]];
    const union aliasi16 d = { .i8 = {
        apply_sign(1 + (d0 > d1), -f->refdist[b->ref.ref[0]]),
        apply_sign(1 + (d1 > d0), +f->refdist[b->ref.ref[1]]),
    }};
    const int bs = 2 - (b->bs == BS_8x8 /* FIXME not tip */);
    union OpflMvDeltaBlock dd[2 * 2];

    union mv (*rmv_line)[2][2] = &t->rmv[((t->by & 31) >> 1) * 16 + ((t->bx & 31) >> 1)];
    const ptrdiff_t t_stride = f->rf.rp_stride;
    refmvs_temporal_block *t_dst = &f->rf.rp[(t->by >> 1) * t_stride + (t->bx >> 1)];
    const int t_swap = !!(f->rf.ref_flip & (1ULL << (b->ref.ref[0] * 8 + b->ref.ref[1])));
    const int sh4 = imin(4, bh4), sw4 = imin(4, bw4);
    for (int y = 0; y < h4; y += sh4, rmv_line += 16 * sh4 >> 1) {
        int left[2] = { t->bx * 4 + (b->mv[0].x >> 3) - 3,
                        t->bx * 4 + (b->mv[1].x >> 3) - 3 };
        if (refine) {
            for (int x = 0; x < w4; x += sw4) {
                for (int n = 0; n < 2; n++)
                    mc(t, p[n], NULL, p_stride, sw4 + 2, sh4 + 2,
                       t->bx + x, t->by + y, 0,
                       (union mv) { .y = b->mv[n].y - 32, .x = b->mv[n].x - 32 },
                       refp[n], b->ref.ref[n], DAV2D_FILTER_BILINEAR,
                       iclip(left[n], 0, w - 1), iclip(left[n] + 4 * sw4 + 7, 1, w),
                       iclip(top[n], 0, h - 1), iclip(top[n] + 4 * sh4 + 7, 1, h));
                struct OpflOffset o;
                f->dsp->mc.sad_refine_mv(p[0], p_stride, p[1], p_stride,
                                         sw4 * 4, sh4 * 4, b->refine_mv == 2,
                                         &o HIGHBD_CALL_SUFFIX);
                const int dy = o.y, dx = o.x;
                if (opfl) {
                    struct OpflRegressionData res[2 * 2];
                    // subpel-gradient based mv refinement (optical flow = opfl)
                    f->dsp->mc.opfl_derive_mv(res,
                                              &p[0][(4 + dy) * PXSTRIDE(p_stride) +
                                                    (4 + dx)], p_stride,
                                              &p[1][(4 - dy) * PXSTRIDE(p_stride) +
                                                    (4 - dx)], p_stride,
                                              sw4 * 4, sh4 * 4, bs * 4, d
                                              HIGHBD_CALL_SUFFIX);
                    const struct OpflRegressionData *r = res;
                    for (int by = 0; by < sh4; by += 2) {
                        for (int bx = 0; bx < sw4; bx += 2, r++) {
                            opfl_mv_adj(r, dd, d);
                            union mv *const mv = rmv_line[!!by * 16 + ((x + bx) >> 1)][0];
                            mv[0].y = b->mv[0].y * 2 + dd[0].d[0].y + dy * 16;
                            mv[0].x = b->mv[0].x * 2 + dd[0].d[0].x + dx * 16;
                            mv[1].y = b->mv[1].y * 2 + dd[0].d[1].y - dy * 16;
                            mv[1].x = b->mv[1].x * 2 + dd[0].d[1].x - dx * 16;
                            for (int i = 0; i < 2; i++)
                                mc_opfl(t, &tmp[i][((y + by) * bw4 * 4 + x + bx) * 4],
                                        bw4 * 4, bs, bs, t->bx + x + bx, t->by + y + by,
                                        0, mv[i], refp[i], b->filter,
                                        iclip(left[i], 0, w - 1),
                                        iclip(left[i] + sw4 * 4 + 7, 1, w),
                                        iclip(top[i], 0, h - 1),
                                        iclip(top[i] + sh4 * 4 + 7, 1, h));
                            const union mv dmv[2] = {
                                [0] = { .y = (mv[0].y + (dd[0].d[0].y > 0)) >> 1,
                                        .x = (mv[0].x + (dd[0].d[0].x > 0)) >> 1 },
                                [1] = { .y = (mv[1].y + (dd[0].d[1].y > 0)) >> 1,
                                        .x = (mv[1].x + (dd[0].d[1].x > 0)) >> 1 },
                            };
                            update_temporal(f, &t_dst[((x + bx) >> 1) + !!by * t_stride],
                                            t_stride, 1, 1, b->ref, dmv, t_swap);
                            if (bacp)
                                have_bacp |= get_mask(mask, bw4 * 4, t->bx, x + bx,
                                                      t->by, y + by, mv, 4, 4, 2, 2, w, h);
                            scaledown_16pel_mv_for_chroma(mv, f->cur.p.p.layout);
                        }
                    }
                } else {
                    union mv *const mv = rmv_line[x >> 1][0];
                    mv[0].y = b->mv[0].y + dy * 8;
                    mv[0].x = b->mv[0].x + dx * 8;
                    mv[1].y = b->mv[1].y - dy * 8;
                    mv[1].x = b->mv[1].x - dx * 8;
                    for (int i = 0; i < 2; i++)
                        mc(t, NULL, &tmp[i][(y * 4 * bw4 + x) * 4], bw4 * 4,
                           sw4, sh4, t->bx + x, t->by + y, 0,
                           mv[i], refp[i], b->ref.ref[i], b->filter,
                           iclip(left[i], 0, w - 1),
                           iclip(left[i] + sw4 * 4 + 7, 1, w),
                           iclip(top[i], 0, h - 1),
                           iclip(top[i] + sh4 * 4 + 7, 1, h));
                    update_temporal(f, &t_dst[x >> 1], t_stride, sw4 >> 1, sh4 >> 1,
                                    b->ref, mv, t_swap);
                    scaleup_8pel_mv_for_chroma(mv, f->cur.p.p.layout);
                    if (bacp)
                        have_bacp |= get_mask(mask, bw4 * 4, t->bx, x,
                                              t->by, y, mv, 3, 3, sw4, sh4, w, h);
                }
                for (int n = 0; n < 2; n++)
                    left[n] += 16;
            }
        } else {
            assert(opfl);
            for (int n = 0; n < 2; n++)
                mc(t, p[n], NULL, p_stride, bw4, sh4, t->bx, t->by + y,
                   0, b->mv[n], refp[n], b->ref.ref[n], DAV2D_FILTER_BILINEAR,
                   0, w, 0, h);
            struct OpflRegressionData res[2 * 8];
            f->dsp->mc.opfl_derive_mv(res, p[0], p_stride, p[1], p_stride,
                                      bw4 * 4, sh4 * 4, bs * 4, d
                                      HIGHBD_CALL_SUFFIX);
            const struct OpflRegressionData *r_line = res;
            union OpflMvDeltaBlock *ddl = dd;
            for (int by = 0; by < sh4; by += bs) {
                const struct OpflRegressionData *r = r_line;
                for (int bx = 0, xx = 0; bx < w4; bx += bs, r++, xx++) {
                    opfl_mv_adj(r, ddl, d);
                    union mv mv_8x8[2];
                    // for 8x8, the opfl blocksize is 4x4; for inter, we
                    // only need to store the first (top/left) one, and
                    // the rest can be discarded. (Not sure if this is
                    // true for 422/444, but it's definitely true for 420.)
                    union mv *const mv = bs == 1 && (bx || by) ? mv_8x8 :
                                         rmv_line[!!by * 16 + xx][0];
                    mv[0].y = b->mv[0].y * 2 + ddl->d[0].y;
                    mv[0].x = b->mv[0].x * 2 + ddl->d[0].x;
                    mv[1].y = b->mv[1].y * 2 + ddl->d[1].y;
                    mv[1].x = b->mv[1].x * 2 + ddl->d[1].x;
                    for (int i = 0; i < 2; i++)
                        mc_opfl(t, &tmp[i][((y + by) * bw4 * 4 + bx) * 4],
                                bw4 * 4, bs, bs, t->bx + bx, t->by + y + by,
                                0, mv[i], refp[i], b->filter,
                                iclip(left[i] + bx * 4, 0, w - 1),
                                iclip(left[i] + bx * 4 + 7 + 8, 1, w),
                                iclip(top[i] + by * 4, 0, h - 1),
                                iclip(top[i] + by * 4 + 7 + 8, 1, h));
                    if (bs > 1) {
                        const union mv dmv[2] = {
                            [0] = { .y = (mv[0].y + (ddl->d[0].y > 0)) >> 1,
                                    .x = (mv[0].x + (ddl->d[0].x > 0)) >> 1 },
                            [1] = { .y = (mv[1].y + (ddl->d[1].y > 0)) >> 1,
                                    .x = (mv[1].x + (ddl->d[1].x > 0)) >> 1 },
                        };
                        update_temporal(f, &t_dst[(bx >> 1) + !!by * t_stride],
                                        t_stride, bs >> 1, bs >> 1,
                                        b->ref, dmv, t_swap);
                    } else {
                        assert(b->bs == BS_8x8);
                        ddl++;
                    }
                    if (bacp)
                        have_bacp |= get_mask(mask, bw4 * 4, t->bx, bx,
                                              t->by, y + by, mv, 4, 4, bs, bs, w, h);
                    scaledown_16pel_mv_for_chroma(mv, f->cur.p.p.layout);
                }
                r_line += bw4 >> (bs == 2);
            }
            if (bs == 1) {
                union mv dmv[2];
                int tmp = dd[0].d[0].x + dd[1].d[0].x + dd[2].d[0].x + dd[3].d[0].x;
                dmv[0].x = (b->mv[0].x * 8 + tmp + 3 + (tmp > 0)) >> 3;
                tmp = dd[0].d[0].y + dd[1].d[0].y + dd[2].d[0].y + dd[3].d[0].y;
                dmv[0].y = (b->mv[0].y * 8 + tmp + 3 + (tmp > 0)) >> 3;
                tmp = dd[0].d[1].x + dd[1].d[1].x + dd[2].d[1].x + dd[3].d[1].x;
                dmv[1].x = (b->mv[1].x * 8 + tmp + 3 + (tmp > 0)) >> 3;
                tmp = dd[0].d[1].y + dd[1].d[1].y + dd[2].d[1].y + dd[3].d[1].y;
                dmv[1].y = (b->mv[1].y * 8 + tmp + 3 + (tmp > 0)) >> 3;
                update_temporal(f, t_dst, t_stride, 1, 1, b->ref, dmv, t_swap);
            }
        }
        for (int n = 0; n < 2; n++)
            top[n] += 4 * sh4;
        t_dst += t_stride * (sh4 >> 1);
    }

    return bacp && have_bacp;
}

static int rmv_uvpred(Dav2dTaskContext *const t, const Av2Block *const b,
                      const int plane, const int r_step, const int o_step,
                      const int bw4, const int bh4)
{
    assert(r_step >= o_step);
    const Dav2dFrameContext *const f = t->f;
    const int ss_hor = f->ss_hor, ss_ver = f->ss_ver;
    const int tip = b->ref.ref[0] == TIP_FRAME;
    const union refpair ref = tip ? f->rf.tip.ref : b->ref;
    int16_t (*const tmp)[64 * 64] = t->scratch.compinter;
    union mv (*rmv_line)[2][2] = &t->rmv[((t->cby & 31) >> 1) * 16 + ((t->cbx & 31) >> 1)];
    const ptrdiff_t stride = bw4 * 4 >> ss_hor;
    ptrdiff_t uvoff = 0;

    uint8_t *const mask = t->scratch.seg_mask;
    const int bacp = !plane && f->seq_hdr->imp_msk_bld && b->cwp_idx == 8;
    if (bacp) memset(mask, 0x20, bw4 * bh4 * 16);
    int have_bacp = 0;

    const int w = f->bw * 4 >> ss_hor, h = f->bh * 4 >> ss_hor;
    const int rw4 = imin(bw4, r_step), rh4 = imin(bh4, r_step);
    const int ow4 = imin(bw4, o_step), oh4 = imin(bh4, o_step);
    const int hhtaps = 2 + 2 * (rw4 > 1 + ss_hor);
    const int hvtaps = 2 + 2 * (rh4 > 1 + ss_ver);
    const int h4 = imin(bh4, f->bh - t->cby);
    const int w4 = imin(bw4, f->bw - t->cbx);
    for (int y = 0; y < h4; y += rh4, rmv_line += 16 * r_step >> 1) {
        for (int x = 0; x < w4; x += rw4) {
            union mv (*const rmv)[2] = rmv_line[x >> 1];
            int top[2], left[2], bottom[2], right[2];
            for (int i = 0; i < 2; i++) {
                top[i] = ((t->cby + y) * 4 >> ss_ver) +
                          ((tip ? rmv[1][i].y : b->mv[i].y) >> 4);
                left[i] = ((t->cbx + x) * 4 >> ss_hor) +
                           ((tip ? rmv[1][i].x : b->mv[i].x) >> 4);
                bottom[i] = iclip(top[i] + (4 * rh4 >> ss_ver) + hvtaps, 1, h);
                right[i] = iclip(left[i] + (4 * rw4 >> ss_hor) + hhtaps, 1, w);
                top[i] = iclip(top[i] + 1 - hvtaps, 0, h - 1);
                left[i] = iclip(left[i] + 1 - hhtaps, 0, w - 1);
            }
            ptrdiff_t uvoffi = uvoff;
            for (int by = 0; by < rh4; by += oh4) {
                for (int bx = 0; bx < rw4; bx += ow4) {
                    union mv (*const rmv)[2] = rmv_line[!!by * 16 + ((x + bx) >> 1)];
                    for (int i = 0; i < 2; i++)
                        mc_opfl(t, &tmp[i][uvoffi + ((x + bx) * 4 >> ss_hor)], stride,
                                ow4 >> ss_hor, oh4 >> ss_ver,
                                (t->cbx + x + bx) >> ss_hor,
                                (t->cby + y + by) >> ss_ver,
                                1 + plane, rmv[0][i], &f->refp[ref.ref[i]], b->filter,
                                left[i], right[i], top[i], bottom[i]);
                    if (bacp)
                        have_bacp |= get_mask(mask, bw4 * 4 >> ss_hor,
                                              t->cbx >> ss_hor, (x + bx) >> ss_hor,
                                              t->cby >> ss_ver, (y + by) >> ss_ver,
                                              rmv[0], 4, 4, ow4 >> ss_hor, oh4 >> ss_ver,
                                              f->bw * 4 >> ss_hor, f->bh * 4 >> ss_ver);
                }
                uvoffi += oh4 * 4 * stride >> ss_ver;
            }
        }
        uvoff += rh4 * 4 * stride >> ss_ver;
    }
    return bacp && have_bacp;
}

static int recon_b_luma_tx(Dav2dTaskContext *const t, DB_ONLY(const int depth)
                           const enum RectTxfmSize tx, Av2Block *const b)
{
    const Dav2dFrameContext *const f = t->f;
    const Dav2dDSPContext *const dsp = f->dsp;
    Dav2dTileState *const ts = t->ts;
    const int bx4 = t->bx & 63, by4 = t->by & 63;
    const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
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
        if (t->task_thread.pass & PASS_ENTROPY) {
            dav2d_memset_pow2[t_dim->lw](&t->a->lcoef[bx4], cf_ctx);
            dav2d_memset_pow2[t_dim->lh](&t->l.lcoef[by4], cf_ctx);
        }
    } else if (!(t->task_thread.pass & PASS_ENTROPY)) {
        cf = ts->frame_thread[1].cf;
        ts->frame_thread[1].cf += imin(tw, 32) * imin(th, 32);
        const struct CodedBlockInfo *const cbi =
            &f->frame_thread.cbi[t->by * f->b4_stride + t->bx];
        txtp = cbi->txtp[0];
        stx = txtp >> 8;
        txtp &= 0xff;
        eob = cbi->eob[0];
    } else {
        cf = bitfn(t->cf_y);
        eob = decode_coefs(t, DB_ONLY(depth + 1)
                           &t->a->lcoef[bx4], &t->l.lcoef[by4],
                           tx, b->bs, 0, b, 0, cf, &txtp, &cf_ctx);
        if (eob == INT_MIN) return -1;
        stx = txtp >> 8;
        txtp = txtp & 0xff;
        DEBUG_BLOCK_printf("%*sPost-y_cf_blk[tx=%dx%d,txtp=%s/%s,eob=%d]: r=%d\n",
                           depth + 1, "", tw, th,
                           dav2d_tx1d_names[txtp & 7],
                           dav2d_tx1d_names[txtp >> 5],
                           eob, ts->msac.rng);
        dav2d_memset_likely_pow2(&t->a->lcoef[bx4], cf_ctx,
                                 imin(t_dim->w, f->bw - t->bx));
        dav2d_memset_likely_pow2(&t->l.lcoef[by4], cf_ctx,
                                 imin(t_dim->h, f->bh - t->by));
        uint8_t *txtp_map = &t->txtp_map[(t->by & 15) * 16 + (t->bx & 15)];
#define set_ctx(rep_macro) \
        for (int y = 0; y < t_dim->h; y++) { \
            rep_macro(txtp_map, 0, txtp); \
            txtp_map += 16; \
        }
        case_set(t_dim->lw);
#undef set_ctx
    }

    pixel *dst = ((pixel *) f->cur.p.data[0]) +
        4 * (t->by * PXSTRIDE(f->cur.p.stride[0]) + t->bx);
    if (b->intra && !b->intrabc && !b->pal_sz) {
        const int sbsz = f->sb_step;
        const int mrl_idx = b->mrl_index;
        const int mrl_mul = b->multi_mrl && tx != (int) TX_4X4;
        pixel *const edge = bitfn(t->scratch.edge) + 128 + !!mrl_idx * 9;

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
                    n_tr = imin(ctz(0x10000 | ~bits), w);
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
        int apply_ibp = f->seq_hdr->ibp && tx != (enum RectTxfmSize) TX_4X4 &&
                        !mrl_idx;
        const int dip = b->dip - 1;
        const int sm_top = b->is_sm[0].a;
        const int sm_left = b->is_sm[0].l;
        const int is_sm_flag = apply_ibp ?
            (sm_top * ANGLE_SMOOTH_TOP_EDGE_FLAG) |
            (sm_left * ANGLE_SMOOTH_LEFT_EDGE_FLAG) :
            (sm_top | sm_left) * (ANGLE_SMOOTH_TOP_EDGE_FLAG |
                                  ANGLE_SMOOTH_LEFT_EDGE_FLAG);
        if (b->y_angle & 1) apply_ibp = 0;
        const int intra_flags = ANGLE_IS_LUMA | is_sm_flag |
            (f->seq_hdr->intra_edge_filter ? ANGLE_USE_EDGE_FILTER_FLAG : 0) |
            (apply_ibp ? ANGLE_IBP_FLAG : 0) |
            (mrl_idx << ANGLE_MRL_IDX_SHIFT) |
            (mrl_mul ? ANGLE_MULTI_MRL_FLAG : 0) |
            ((t->bx > ts->tiling.col_start) ? ANGLE_HAS_LEFT_FLAG : 0) |
            ((t->by > ts->tiling.row_start) ? ANGLE_HAS_TOP_FLAG  : 0) |
            (dip >= 0 ? ANGLE_DIP_FLAG : 0);
        angle = dip >= 0 ? dip : angle;
        const enum IntraPredMode m = bytefn(dav2d_prepare_intra_edges)(
            DB_ONLY(BLOCK_TO_DEBUG && DEBUG_B_PIXELS) t->bx, t->by,
            ts->tiling.col_end, ts->tiling.row_end, n_tr, n_bl, dst,
            f->cur.p.stride[0], top_sb_edge, b->y_mode, t_dim->w, t_dim->h,
            angle | intra_flags, edge HIGHBD_CALL_SUFFIX);

        dsp->ipred.intra_pred[m](dst, f->cur.p.stride[0],
                                 edge, tw, th, angle | intra_flags,
                                 4 * f->bw - 4 * t->bx,
                                 4 * f->bh - 4 * t->by
                                 HIGHBD_CALL_SUFFIX);

        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
            hex_dump(dst, f->cur.p.stride[0], tw, th, "y-intra-pred");
        }
    }

    if (eob != -1) {
        const int mask_idx = bx4 >> 4;
        const int mask = ((1 << t_dim->w) - 1) << (bx4 & 0xf);
        for (int y = 0; y < t_dim->h; y++)
            t->lf_mask->lr_noskip_mask[by4 + y][mask_idx] |= mask;

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
                const int8_t *kernel = &dav2d_stx_8x8_kernel[set][type][0][0];
                coef sums[48];
                dsp->stx.stxfm(sums, cf, kernel, 48, eob HIGHBD_CALL_SUFFIX);
                memset(cf, 0, 32 * sizeof(coef));
                // Subtract 1 to map {8,16,32} to idx {0,1,2}
                const int idx = imin(t_dim->lh, 3) - 1;
                const uint8_t *scan_out = dav2d_stx_scan_orders_8x8[idx][transpose];
                const uint8_t *mapping = dav2d_coeff8x8_mapping[set * 3 + type];
                for (int x = 0; x < 48; x++) {
                    cf[scan_out[mapping[x]]] = sums[x];
                }
                eob = (uint8_t[]){ 63, 119, 231 }[idx];
            } else {
                const int8_t *kernel = &dav2d_stx_4x4_kernel[set][type][0][0];
                coef sums[16];
                dsp->stx.stxfm(sums, cf, kernel, 16, eob HIGHBD_CALL_SUFFIX);
                const int idx = imin(t_dim->lh, 3);
                const uint8_t *scan_out = dav2d_stx_scan_orders_4x4[idx][transpose];
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
        if (f->frame_hdr->segmentation.lossless[b->seg_id] &&
            b->intra && !b->intrabc && b->dpcm[0])
        {
            txtp += (1 + (b->y_mode == VERT_PRED)) << 8;
        } else if (f->seq_hdr->inter_ddt && !b->intra)
            txtp += txtp & dav2d_tx_ddt_mask[tx]; // (flip)adst -> (f)ddt
        dsp->itx.itxfm_add[tx](dst, f->cur.p.stride[0],
                               cf, txtp, eob HIGHBD_CALL_SUFFIX);
        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
            hex_dump(dst, f->cur.p.stride[0], t_dim->w * 4, t_dim->h * 4, "recon");
        }
    }

    const uint64_t mask = ((1ULL << t_dim->w) - 1) << bx4;
    for (int y = 0; y < t_dim->h; y++) {
        t->is_coded[0][by4 + y] |= mask;
    }

    b->y_mode = orig_y_mode;
    return 0;
}

static void bawp(Dav2dTaskContext *const t,
                 const int bawp_idx, const union mv mv,
                 pixel *const dst, const ptrdiff_t stride,
                 const Dav2dThreadPicture *const refp, const int refidx,
                 const int bw4, const int bh4, const int w4, const int h4,
                 const int plane, const enum BlockSize sb_bs)
{
    const Dav2dFrameContext *const f = t->f;
    const int chroma = !!plane;
    const int ss_hor = f->ss_hor * chroma, ss_ver = f->ss_ver * chroma;
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;
    const Dav2dDSPContext *const dsp = f->dsp;
    const uint8_t *const sb_dim = dav2d_block_dimensions[sb_bs];
    const int bx = chroma ? t->cbx : t->bx, by = chroma ? t->cby : t->by;
    if ((sb_dim[0] > (16 << ss_hor) && bx & (sb_dim[0] - 1)) ||
        (sb_dim[1] > (16 << ss_ver) && by & (sb_dim[1] - 1)))
    {
        const int alpha = t->pb.bawp[plane].alpha, beta = t->pb.bawp[plane].beta;
        if (alpha != 256 || beta)
            dsp->mc.morph(dst, stride, alpha, beta,
                          bw4 * h_mul, bh4 * v_mul HIGHBD_CALL_SUFFIX);
        return;
    }
    // defaults
    t->pb.bawp[plane].alpha = 256;
    t->pb.bawp[plane].beta = 0;
    Dav2dTileState *const ts = t->ts;
    int tile_top_edge, tile_left_edge, tile_bottom_edge, tile_right_edge;
    if (refp == &f->cur) {
        tile_top_edge = ts->tiling.row_start * v_mul;
        tile_left_edge = ts->tiling.col_start * h_mul;
        tile_bottom_edge = ts->tiling.row_end * v_mul;
        tile_right_edge = ts->tiling.col_end * h_mul;
    } else {
        tile_top_edge = tile_left_edge = 0;
        tile_bottom_edge = f->bh * v_mul;
        tile_right_edge = f->bw * h_mul;
    }
    const int mvx = (mv.x + 3 + (mv.x >= 0)) >> (3 + ss_hor);
    const int mvy = (mv.y + 3 + (mv.y >= 0)) >> (3 + ss_ver);
    const int ref_y = (by * v_mul + mvy);
    const int ref_x = (bx * h_mul + mvx);
    const int ref_tmplt_x = ref_x - 1;
    const int ref_tmplt_y = ref_y - 1;
    const int sb_w4 = imin(sb_dim[0], f->bw - bx);
    const int sb_h4 = imin(sb_dim[1], f->bh - by);
    const int ref_bottom_edge = ref_y + sb_h4 * v_mul;
    const int ref_right_edge = ref_x + sb_w4 * h_mul;

    const int can_morph =
        ref_bottom_edge <= tile_bottom_edge &&
        ref_right_edge <= tile_right_edge &&
        ref_tmplt_y >= tile_top_edge && ref_tmplt_x >= tile_left_edge;
    if (!can_morph) return;

    // TODO (optimization): Consider moving this code (and associated
    // size lookup tables) to a DSP function. SIMD could specialize on
    // edge sizes (4/8/16/32/64) and step values.
    static const uint8_t n_edge_samples[2 /* have edges */][3 /* h */]
                                       [3 /* w */][2 /* above, left */] = {
        { // !have_above || !have_left
            { { 2, 2 }, { 3, 2 }, { 4, 2 } },
            { { 2, 3 }, { 3, 3 }, { 4, 3 } },
            { { 2, 4 }, { 3, 4 }, { 4, 4 } },
        }, { // have_above && have_left
            { { 2, 2 }, { 2, 2 }, { 4, 0 } },
            { { 2, 2 }, { 3, 3 }, { 3, 3 } },
            { { 0, 4 }, { 3, 3 }, { 4, 4 } },
        }
    };
    const int have_left = bx > ts->tiling.col_start;
    const int have_above = by > ts->tiling.row_start;
    const int lw4 = imin(ulog2(w4), 2) - ss_hor, lh4 = imin(ulog2(h4), 2) - ss_ver;
    const int idx = have_above && have_left;
    const int n_above_l2 = have_above * n_edge_samples[idx][lh4][lw4][0];
    const int n_left_l2 = have_left * n_edge_samples[idx][lh4][lw4][1];

    const pixel *const ref =
        &((const pixel *) refp->p.data[plane])[ref_y * PXSTRIDE(refp->p.stride[chroma]) +
                                               ref_x];

    assert(n_above_l2 == 0 || n_left_l2 == 0 || n_above_l2 == n_left_l2);
    const int count_l2 =
        n_above_l2 + (n_above_l2 == n_left_l2 ? !!n_above_l2 : n_left_l2);
    int sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    if (n_above_l2) {
        const int bw = 4 << lw4;
        const int step = bw >> n_above_l2;
        assert(step > 0);
        const int start = step >> 1;
        for (int i = start; i < bw; i += step) {
            const int x = ref[i - PXSTRIDE(refp->p.stride[chroma])];
            const int y = dst[i - PXSTRIDE(stride)];
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
            const int x = ref[(i * PXSTRIDE(refp->p.stride[chroma])) - 1];
            const int y = dst[(i * PXSTRIDE(stride)) - 1];
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_x2 += x * x;
        }
    }

    int alpha, beta;
    if (plane) {
        alpha = have_left || have_above ? t->pb.bawp[0].alpha : 256;
    } else {
        if (bawp_idx != 1) {
            assert(bawp_idx & 2);
            const int idx = (1 + (bawp_idx >> 2) + (f->absrefdist[refidx] > 4)) *
                             (bawp_idx & 1 ? 1 : -1);
            alpha = 256 + 16 * idx;
        } else if (count_l2) {
            const int num = sum_xy - (int)(((int64_t)sum_x * sum_y) >> count_l2);
            const int den = sum_x2 - (int)(((int64_t)sum_x * sum_x) >> count_l2);
            alpha = derive_alpha(num, den, 256);
        } else {
            alpha = 256;
        }
    }
    t->pb.bawp[plane].alpha = alpha;

    if (count_l2) {
        const int diff = (sum_y << 8) - sum_x * alpha;
        const int abs_diff = abs(diff);
        beta = apply_sign(abs_diff >> count_l2, diff);
    } else {
        beta = -128;
    }
    t->pb.bawp[plane].beta = beta;

    dsp->mc.morph(dst, stride, alpha, beta,
                  bw4 * h_mul, bh4 * v_mul HIGHBD_CALL_SUFFIX);
}

static void iiblend(Dav2dTaskContext *const t, const Av2Block *const b,
                    pixel *const dst, const ptrdiff_t stride, const int plane,
                    const int bw4, const int bh4, const int by, const int bx,
                    const enum BlockSize ss_bs)
{
    const Dav2dTileState *const ts = t->ts;
    const Dav2dFrameContext *const f = t->f;
    const Dav2dDSPContext *const dsp = f->dsp;
    pixel *const tl_edge = bitfn(t->scratch.edge) + 128;
    enum IntraPredMode m = b->interintra_mode == II_SMOOTH_PRED ?
                           SMOOTH_PRED : b->interintra_mode;
    pixel *const tmp = bitfn(t->scratch.interintra);
    int angle = (const uint8_t[4]) { 0, 90, 180, 0 }[b->interintra_mode];
    int n_tr = 0, n_bl = 0;
    const int chroma = !!plane;
    const int ss_hor = chroma * f->ss_hor, ss_ver = chroma * f->ss_ver;
    if (m == SMOOTH_PRED) {
        const int bx4 = bx & 63, by4 = by & 63, sbsz = f->sb_step;
        if (by > ts->tiling.row_start) {
            int w = imin(bw4, ts->tiling.col_end - bx - bw4);
            if (!(by & (sbsz - 1))) {
                // top sb boundary
                n_tr = w;
            } else {
                const int end = imin((bx + sbsz) & ~(sbsz - 1),
                                     ts->tiling.col_end);
                w = imin(w, end - t->bx - bw4);
                if (w <= 0) {
                    // right sb or tile/frame boundary
                    n_tr = 0;
                } else {
                    // smooth pred uses 1px max
                    n_tr = (t->is_coded[chroma][(by4 >> ss_ver) - 1] >> ((bx4 + bw4) >> ss_hor)) & 1;
                }
            }
        }

        if (bx > ts->tiling.col_start) {
            const int end = imin((by + sbsz) & ~(sbsz - 1), ts->tiling.row_end);
            const int h = imin(bh4, end - by - bh4);
            if (h <= 0) {
                // bottom sb or tile/frame boundary
                n_bl = 0;
            } else if (!(bx & (sbsz - 1))) {
                // left sb boundary
                n_bl = h;
            } else {
                // smooth pred uses 1px max
                n_bl = (t->is_coded[chroma][(by4 + bh4) >> ss_ver] >> ((bx4 - 1) >> ss_hor)) & 1;
            }
        }
    }
    const pixel *top_sb_edge = NULL;
    if (!(t->by & (f->sb_step - 1))) {
        top_sb_edge = f->ipred_edge[plane];
        const int sby = by >> f->sb_shift;
        top_sb_edge += f->sb256w * 256 * (sby - 1) >> ss_hor;
    }
    const int ssbw4 = bw4 >> ss_hor;
    const int ssbh4 = bh4 >> ss_ver;
    const int apply_ibp = f->seq_hdr->ibp && imax(ssbw4, ssbh4) > 1;
    const int intra_flags =
        (apply_ibp ? ANGLE_IBP_FLAG /* for dc */ : 0) |
        ((bx > ts->tiling.col_start) ? ANGLE_HAS_LEFT_FLAG : 0) |
        ((by > ts->tiling.row_start) ? ANGLE_HAS_TOP_FLAG  : 0);
    m = bytefn(dav2d_prepare_intra_edges)(
            DB_ONLY(!plane && BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
            bx >> ss_hor, by >> ss_ver,
            ts->tiling.col_end >> ss_hor, ts->tiling.row_end >> ss_ver,
            n_tr, n_bl, dst, stride, top_sb_edge, m,
            ssbw4, ssbh4, angle | intra_flags, tl_edge HIGHBD_CALL_SUFFIX);
    dsp->ipred.intra_pred[m](tmp, 4 * ssbw4 * sizeof(pixel),
                             tl_edge, ssbw4 * 4, ssbh4 * 4,
                             intra_flags, 0, 0 HIGHBD_CALL_SUFFIX);
    if (!plane && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
        hex_dump(tmp, ssbw4 * 4 * sizeof(pixel), ssbw4 * 4, ssbh4 * 4, "y-intra-pred");
        if (0) hex_dump(dst, stride, ssbw4 * 4, ssbh4 * 4, "inter-pred");
    }
    const uint8_t *const mask = b->wedge_idx == -1 ?
        II_MASK(ss_bs, ssbw4, ssbh4, b->interintra_mode) :
        WEDGE_MASK(ss_bs, bw4, bh4, b->wedge_idx, ss_hor + ss_ver);
    dsp->mc.blend(dst, stride, tmp, ssbw4 * 4, ssbh4 * 4, mask);
}

static inline void cfl(Dav2dTaskContext *const t, const Av2Block *const b,
                       const enum BlockSize bs, const TxfmInfo *const t_dim)
{
    const Dav2dTileState *const ts = t->ts;
    const Dav2dFrameContext *const f = t->f;
    const Dav2dDSPContext *const dsp = f->dsp;
    const enum Dav2dPixelLayout layout = f->cur.p.p.layout - 1;
    const ptrdiff_t ystride = f->cur.p.stride[0];
    const ptrdiff_t cstride = f->cur.p.stride[1];
    const int sby = t->cby >> f->sb_shift;
    const int sbsz = f->sb_step;
    const int ss_hor = f->ss_hor, ss_ver = f->ss_ver;
    const int ssbx = t->cbx >> ss_hor, ssby = t->cby >> ss_ver;
    const int has_top = t->cby > ts->tiling.row_start;
    const int has_left = t->cbx > ts->tiling.col_start;
    const int is_top_sb_edge = !(t->cby & (sbsz - 1));
    const int ctw4 = imin(t_dim->w, (f->bw - t->cbx + ss_hor) >> ss_hor);
    const int cth4 = imin(t_dim->h, (f->bh - t->cby + ss_ver) >> ss_ver);
    const int ctw = t_dim->w * 4, cth = t_dim->h * 4;
    const int filter_type = f->c->seq_hdr->cfl_ds_filter_index;
    pixel *const ysrc = ((pixel *) f->cur.p.data[0]) +
        (t->cby * PXSTRIDE(ystride) + t->cbx) * 4;
    pixel *const ytop_sb_edge = !is_top_sb_edge ? NULL :
        f->ipred_edge[0] + f->sb256w * 256 * (sby - 1) + t->cbx * 4;

    if (b->cfl_type < CFL_MHCCP) { // CFL EXPLICIT / IMPLICIT
        const ptrdiff_t off = (ssby * PXSTRIDE(cstride) + ssbx) * 4;
        pixel *const usrc = ((pixel *) f->cur.p.data[1]) + off;
        pixel *const vsrc = ((pixel *) f->cur.p.data[2]) + off;

        const ptrdiff_t sboff = (sby - 1) * f->sb256w * 256 >> ss_hor;
        pixel *const u_top_sb_edge = f->ipred_edge[1] + sboff + ssbx * 4;
        pixel *const v_top_sb_edge = f->ipred_edge[2] + sboff + ssbx * 4;

        pixel *const ytop = is_top_sb_edge ?
            ytop_sb_edge : ysrc - (1 + ss_ver) * PXSTRIDE(ystride);
        pixel *const utop = is_top_sb_edge ?
            u_top_sb_edge : usrc - PXSTRIDE(cstride);
        pixel *const vtop = is_top_sb_edge ?
            v_top_sb_edge : vsrc - PXSTRIDE(cstride);

        pixel *const ptrs[6] = { ytop, utop, vtop, ysrc, usrc, vsrc };

        const int cbw4 = (dav2d_block_dimensions[bs][0] + ss_hor) >> ss_hor;
        const int cbh4 = (dav2d_block_dimensions[bs][1] + ss_ver) >> ss_ver;
        const int wpad = cbw4 - ctw4;
        const int hpad = cbh4 - cth4;

        const unsigned flags = filter_type |
            (t->cby > ts->tiling.row_start ? CFL_HAS_TOP : 0) |
            (t->cbx > ts->tiling.col_start ? CFL_HAS_LEFT : 0) |
            (is_top_sb_edge ? CFL_IS_TOP_SB_EDGE : 0) |
            (((unsigned)b->cfl_alpha[0] << CFL_ALPHA_U_SHIFT) & CFL_ALPHA_U_MASK) |
            (((unsigned)b->cfl_alpha[1] << CFL_ALPHA_V_SHIFT) & CFL_ALPHA_V_MASK);

        dsp->ipred.cfl_pred[b->cfl_type][layout](ptrs, f->cur.p.stride,
                                                 wpad, hpad, ctw, cth, flags
                                                 HIGHBD_CALL_SUFFIX);
        if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
            hex_dump(ptrs[1], cstride, ctw, cth, "u-intra-pred");
            hex_dump(ptrs[2], cstride, ctw, cth, "v-intra-pred");
        }

    } else { // CFL MHCCP
        const int cbx4 = (t->cbx & 63) >> ss_hor, cby4 = (t->cby & 63) >> ss_ver;
        ALIGN(pixel luma[CFL_MHCCP_MAX_LUMA_SIZE], 64);
        int refw = ctw4 * 4, refh = cth4 * 4, luma_top_stride;
        uint16_t imat[2][CFL_MHCCP_MAX_EDGE_SAMPLES];
        int32_t mat[3][3] = { 0 };
        int n_tr = 0, n_bl = 0;
        if (has_top) {
            const int csbsz = sbsz >> ss_hor;
            const int tile_end = ts->tiling.col_end >> ss_hor;
            int w = imax(0, imin(ctw4, tile_end - ssbx - ctw4));
            if (is_top_sb_edge) {
                n_tr = w;
            } else {
                const int end = imin((ssbx + csbsz) & ~(csbsz - 1), tile_end);
                w = imin(ctw4, end - ssbx - ctw4);
                if (!w) { // right sb boundary
                    n_tr = 0;
                } else {
                    const unsigned bits = (unsigned)
                        (t->is_coded[1][cby4 - 1] >> (cbx4 + ctw4));
                    n_tr = imin(ctz(0x10000 | ~bits), w);
                }
            }
            refw += n_tr * 4;
        }
        int subleft = 0;
        if (has_left) {
            const int csbsz = sbsz >> ss_ver;
            const int end = imax(0, imin((ssby + csbsz) & ~(csbsz - 1),
                                         ts->tiling.row_end >> ss_ver));
            const int h = imin(cth4, end - ssby - cth4);
            if (!(t->cbx & (sbsz - 1)) || !h) { // left or bottom sb boundary
                n_bl = h;
            } else {
                const uint64_t mask = 1ULL << (cbx4 - 1);
                for (; n_bl < h; n_bl++)
                    if (!(t->is_coded[1][cby4 + n_bl + cth4] & mask))
                        break;
            }
            refh += n_bl * 4;
            refw += 2;
            subleft = b->cfl_mh_dir != CFL_DIR_LEFT;
        }
        if (refw > (128 >> ss_hor)) {
            refw = 128 >> ss_hor;
            subleft = 0;
        }
        refh = imin(refh, (128 >> ss_ver) - 2 * has_top);

        luma_top_stride = (refw * sizeof(pixel) + 63) & ~63;
        const int edge_flags = (has_top ? CFL_HAS_TOP : 0) |
                               (has_left ? CFL_HAS_LEFT : 0) |
                               (is_top_sb_edge ? CFL_IS_TOP_SB_EDGE : 0);
        dsp->ipred.cfl_gen_y[layout][filter_type](luma, luma_top_stride,
                                                  ysrc, ytop_sb_edge, ystride,
                                                  refw - subleft, refh, ctw, cth,
                                                  edge_flags | b->cfl_mh_dir);
        refh += has_top;
        if (has_top || has_left)
            dsp->ipred.cfl_gen_mat[b->cfl_mh_dir](
                    mat, imat, luma, luma_top_stride,
                    refw, refh, edge_flags HIGHBD_CALL_SUFFIX);

        for (int pl = 1; pl <= 2; pl++) {
            int alpha[3] = { 0 };
            pixel *chroma = ((pixel *) f->cur.p.data[pl]) +
                4 * (ssby * PXSTRIDE(cstride) + ssbx);
            const pixel *const ctop_sb_edge = is_top_sb_edge ? f->ipred_edge[pl] +
                ((sby - 1) * f->sb256w * 256 >> ss_hor) + ssbx * 4 : NULL;

            if (has_top || has_left) {
                dsp->ipred.cfl_calc_alphas(alpha, chroma, ctop_sb_edge, cstride,
                                           refw, refh, mat, imat, edge_flags
                                           HIGHBD_CALL_SUFFIX);
            } else { // XXX optimize for no edge case? (single const alpha)
                alpha[2] = 0x10000;
            }
            const int n_top = has_top ? has_top + (b->cfl_mh_dir == CFL_DIR_TOP) : 0;
            const pixel *const src = luma + n_top * PXSTRIDE(luma_top_stride);
            dsp->ipred.cfl_mhccp_pred[b->cfl_mh_dir](chroma, cstride, src,
                                                     luma_top_stride, ctw, cth,
                                                     alpha, edge_flags HIGHBD_CALL_SUFFIX);
            if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                hex_dump(chroma, cstride, ctw, cth, pl == 1 ? "u-intra-pred" : "v-intra-pred");
            }
        }
    }
}

int bytefn(dav2d_recon_b)(Dav2dTaskContext *const t, DB_ONLY(const int depth)
                          const enum BlockSize lbs,
                          // [0] = coef reading, [1] = reconstruction
                          const enum BlockSize cbs_stage[2],
                          Av2Block *const b)
{
    Dav2dTileState *const ts = t->ts;
    const Dav2dFrameContext *const f = t->f;
    const Dav2dDSPContext *const dsp = f->dsp;
    const enum BlockSize cbs = cbs_stage[cbs_stage[0] == BS_INVALID];
    const enum BlockSize bs = lbs == BS_INVALID ? cbs : lbs;
    assert(cbs_stage[0] == cbs_stage[1] ||
           ((cbs_stage[0] == BS_INVALID || cbs_stage[1] == BS_INVALID) &&
            (lbs == BS_64x64 && (f->ss_ver || f->ss_hor))));
    assert(bs != BS_INVALID);
    const uint8_t *const b_dim = dav2d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
    const int ss_hor = f->ss_hor, ss_ver = f->ss_ver;
    const uint8_t csplit[3][3] = {
        [BS_128x128 - BS_128x128] = {  BS_64x64, BS_128x64, BS_128x128 },
        [BS_128x64  - BS_128x128] = {  BS_64x64, BS_128x64, BS_128x64  },
        [BS_64x128  - BS_128x128] = {  BS_64x64, BS_64x64,  BS_64x128  },
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
            cbs2i = cbs == BS_INVALID ? BS_INVALID :
                    csplit[cbs - BS_128x128][ss_hor + ss_ver];
        }
        for (int y = 0; t->by < y_end; t->by += step, y++) {
            for (int x = 0; t->bx < x_end; t->bx += step, x++) {
                enum BlockSize cbs2[2];
                if (step == 32) {
                    cbs2[0] = cbs2[1] = cbs2i;
                } else {
                    // coef reading is done with the first luma 64x64
                    cbs2[0] = !((x & ss_hor) | (y & ss_ver)) ? cbs2i : BS_INVALID;
                    // reconstruction should be done with the last luma 64x64,
                    // so that COMP_INTER_SEG or refine-mv work correctly
                    cbs2[1] = (!ss_hor || t->bx + step >= x_end) &&
                              (!ss_ver || t->by + step >= y_end) ?
                              cbs2i : BS_INVALID;
                }
                const int res = bytefn(dav2d_recon_b)(t, DB_ONLY(depth) lbs2, cbs2, b);
                if (step == 32) {
                    t->cbx += step;
                } else if ((x & ss_hor) == ss_hor) {
                    t->cbx += step << ss_hor;
                }
                if (res < 0) {
                    t->cbx = t->bx = x_start;
                    t->cby = t->by = y_start;
                    return res;
                }
            }
            t->cbx = t->bx = x_start;
            if (step == 32) {
                t->cby += step;
            } else if ((y & ss_ver) == ss_ver) {
                t->cby += step << ss_ver;
            }
        }
        t->cby = t->by = y_start;
        return 0;
    }

    if (lbs == BS_INVALID) goto chroma;

    const int8_t *const tp = dav2d_tx_part_tbl[bs];
    if (tp[b->tx_part] == -1) return -1;

    pixel *const dst = ((pixel *) f->cur.p.data[0]) +
                           4 * (t->by * PXSTRIDE(f->cur.p.stride[0]) + t->bx);
    if (b->intrabc) {
        mc(t, dst, NULL, f->cur.p.stride[0], bw4, bh4, t->bx, t->by, 0,
           b->mv[0], &f->cur, 0 /* unused */, DAV2D_FILTER_BILINEAR,
           0, f->bw * 4, 0, f->bh * 4);
        if (b->morph_pred)
            bawp(t, 1, b->mv[0], dst, f->cur.p.stride[0],
                 &f->cur, 0 /* unused */, bw4, bh4, w4, h4, 0, b->bs);
        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
            hex_dump(dst, f->cur.p.stride[0], imin(f->bw - t->bx, bw4) * 4,
                     imin(f->bh - t->by, bh4) * 4, "y-pred");
        }
    } else if (!b->intra) {
        if (b->ref.ref[1] == -1 && b->ref.ref[0] != TIP_FRAME) {
            const Dav2dThreadPicture *const refp = &f->refp[b->ref.ref[0]];
            if (!f->frame_hdr->force_integer_mv &&
                ((b->inter_mode == GLOBALMV && imin(bw4, bh4) > 1 &&
                  f->gmv_warp_allowed[b->ref.ref[0]]) ||
                 (b->motion_mode >= MM_WARP_CAUSAL &&
                  t->warpmv[0].type > DAV2D_WM_TYPE_INVALID)))
            {
                warp_affine(t, dst, NULL, f->cur.p.stride[0], b_dim, 0, refp,
                            b->motion_mode >= MM_WARP_CAUSAL ? &t->warpmv[0] :
                                &f->frame_hdr->gmv.m[b->ref.ref[0]]);
            } else {
                mc(t, dst, NULL, f->cur.p.stride[0], bw4, bh4,
                   t->bx, t->by, 0, b->mv[0], refp, b->ref.ref[0], b->filter,
                   0, f->bw * 4, 0, f->bh * 4);
            }
            if (b->bawp[0]) {
                bawp(t, b->bawp[0], b->mv[0], dst, f->cur.p.stride[0],
                     refp, b->ref.ref[0], bw4, bh4, w4, h4, 0, b->bs);
            } else if (b->motion_mode == MM_INTERINTRA || b->warp_ii) {
                iiblend(t, b, dst, f->cur.p.stride[0], 0, bw4, bh4, t->by, t->bx, bs);
            }
        } else {
            int16_t (*const tmp)[64 * 64] = t->scratch.compinter;
            int bacp;

            if (b->ref.ref[0] == TIP_FRAME) {
                bacp = tip_pred(t, tmp, b, bw4, bh4, w4, h4);
            } else if (b->inter_mode >= OPFL_NEARMV_NEARMV ||
                       (b->refine_mv && b->comp_type == COMP_INTER_AVG))
            {
                bacp = opfl_pred(t, tmp, b, bw4, bh4, w4, h4);
            } else {
                bacp = 2 * (f->seq_hdr->imp_msk_bld &&
                            b->motion_mode != MM_WARP_CAUSAL &&
                            b->inter_mode != GLOBALMV_GLOBALMV &&
                            !f->svc[b->ref.ref[0]][0].scale &&
                            !f->svc[b->ref.ref[1]][0].scale);
                for (int i = 0; i < 2; i++) {
                    const Dav2dThreadPicture *const refp = &f->refp[b->ref.ref[i]];

                    if ((b->inter_mode == GLOBALMV_GLOBALMV && imin(bw4, bh4) > 1 &&
                         f->gmv_warp_allowed[b->ref.ref[i]]) ||
                        (b->motion_mode == MM_WARP_CAUSAL &&
                         t->warpmv[i].type > DAV2D_WM_TYPE_INVALID))
                    {
                        warp_affine(t, NULL, tmp[i], bw4 * 4, b_dim, 0, refp,
                                    b->motion_mode >= MM_WARP_CAUSAL ?
                                        &t->warpmv[i] : &f->frame_hdr->gmv.m[b->ref.ref[i]]);
                    } else {
                        mc(t, NULL, tmp[i], bw4 * 4, bw4, bh4, t->bx, t->by, 0,
                           b->mv[i], refp, b->ref.ref[i], b->filter,
                           0, f->bw * 4, 0, f->bh * 4);
                    }
                    if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                        ac_dump(tmp[i], bw4 * 4, bh4 * 4, "y-single-pred");
                }
            }
            switch (b->comp_type) {
            case COMP_INTER_WEDGE: {
                const uint8_t *const mask =
                    WEDGE_MASK(bs, bw4, bh4, b->wedge_idx, 0);
                dsp->mc.mask(dst, f->cur.p.stride[0],
                             tmp[b->wedge_sign], tmp[!b->wedge_sign],
                             bw4 * 4, bh4 * 4, mask HIGHBD_CALL_SUFFIX);
                break;
            }
            case COMP_INTER_SEG: {
                const int chr_layout_idx =
                    f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I400 ? 0 :
                    DAV2D_PIXEL_LAYOUT_I444 - f->cur.p.p.layout;
                const ptrdiff_t mask_stride =
                    imin(dav2d_block_dimensions[b->bs][0] * 4 >> f->ss_hor, 64);
                uint8_t *const seg_mask = imin(bw4, bh4) < 16 ? t->scratch.seg_mask :
                    &t->scratch.seg_mask[((t->by >> f->ss_ver) & 15) * 4 * mask_stride +
                                         ((t->bx >> f->ss_hor) & 15) * 4];
                dsp->mc.w_mask[chr_layout_idx](dst, f->cur.p.stride[0],
                                               tmp[b->mask_sign], tmp[!b->mask_sign],
                                               bw4 * 4, bh4 * 4, seg_mask, mask_stride,
                                               b->mask_sign HIGHBD_CALL_SUFFIX);
                break;
            }
            default: assert(0);
            case COMP_INTER_NONE:
                assert(b->ref.ref[0] == TIP_FRAME);
                // fall-through
            case COMP_INTER_AVG: {
                const int wt = b->cwp_idx;
                if (wt == 8) {
                    if (bacp == 2)
                        bacp = get_mask(t->scratch.seg_mask, bw4 * 4, t->bx, 0,
                                        t->by, 0, b->mv, 3, 3, bw4, bh4,
                                        f->bw * 4, f->bh * 4);
                    if (bacp) {
                        dsp->mc.mask(dst, f->cur.p.stride[0], tmp[0], tmp[1],
                                     bw4 * 4, bh4 * 4, t->scratch.seg_mask
                                     HIGHBD_CALL_SUFFIX);
                    } else {
                        dsp->mc.avg(dst, f->cur.p.stride[0], tmp[0], tmp[1],
                                    bw4 * 4, bh4 * 4 HIGHBD_CALL_SUFFIX);
                    }
                } else {
                    dsp->mc.w_avg(dst, f->cur.p.stride[0], tmp[0], tmp[1],
                                  bw4 * 4, bh4 * 4, wt HIGHBD_CALL_SUFFIX);
                }
                break;
            }}
        }
        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
            hex_dump(dst, f->cur.p.stride[0], imin(f->bw - t->bx, bw4) * 4,
                     imin(f->bh - t->by, bh4) * 4, "y-pred");
    } else if (b->pal_sz) {
        const uint8_t *pal_idx;
        const pixel *pal;
        if (t->task_thread.pass != PASS_ALL) {
            const int p = !!(t->task_thread.pass & PASS_ENTROPY);
            assert(ts->frame_thread[p].pal_idx);
            pal_idx = ts->frame_thread[p].pal_idx;
            ts->frame_thread[p].pal_idx += bw4 * bh4 * 8;
            pal = *ts->frame_thread[0].pal++;
        } else {
            pal_idx = t->scratch.pal_idx_y;
            pal = bytefn(t->scratch.pal);
        }
        f->dsp->ipred.pal_pred(dst, f->cur.p.stride[0], pal,
                               pal_idx, bw4 * 4, bh4 * 4);
        if (BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
            hex_dump(dst, f->cur.p.stride[0], imin(f->bw - t->bx, bw4) * 4,
                     imin(f->bh - t->by, bh4) * 4, "y-pal-pred");
    }

    // luma
    enum RectTxfmSize tx = tp[b->tx_part];
    t->pb.col_start = t->bx;
    t->pb.row_start = t->by;
    if (f->frame_hdr->segmentation.lossless[b->seg_id]) {
        int res = 0, y, x;
        tx = b->tx_size_ll ? dav2d_max_txfm_size_for_bs[bs][3] : (int) TX_4X4;
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int tw4 = t_dim->w, th4 = t_dim->h;
        for (y = 0; y < h4 && !res; y += th4, t->by += th4) {
            for (x = 0; x < w4 && !res; x += tw4, t->bx += tw4) {
                res = recon_b_luma_tx(t, DB_ONLY(depth) (int) tx, b);
            }
            t->bx -= x;
        }
        t->by -= y;
        if (res < 0) return res;
    } else switch (b->tx_part) {
    case TX_PARTITION_NONE: {
        const int res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_SPLIT: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int tw4 = t_dim->w, th4 = t_dim->h;
        int res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        const int have_v_split = t->bx + tw4 < f->bw;
        if (have_v_split) {
            t->bx += tw4;
            res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4;
            if (res < 0) return res;
        }
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (!res && have_v_split) {
            t->bx += tw4;
            res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4;
        }
        t->by -= th4;
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_H: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int th4 = t_dim->h;
        int res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        t->by -= th4;
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_V: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int tw4 = t_dim->w;
        int res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        if (t->bx + tw4 >= f->bw) break;
        t->bx += tw4;
        res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        t->bx -= tw4;
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_H4: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int th4 = t_dim->h;
        int res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        if (t->by + th4 >= f->bh) break;
        t->by += th4;
        res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0 || t->by + th4 >= f->bh) {
            t->by -= th4;
        } else {
            t->by += th4;
            res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            if (res < 0 || t->by + th4 >= f->bh) {
                t->by -= 2 * th4;
            } else {
                t->by += th4;
                res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
                t->by -= 3 * th4;
            }
        }
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_V4: {
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        const int tw4 = t_dim->w;
        int res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        if (t->bx + tw4 >= f->bw) break;
        t->bx += tw4;
        res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0 || t->bx + tw4 >= f->bw) {
            t->bx -= tw4;
        } else {
            t->bx += tw4;
            res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            if (res < 0 || t->bx + tw4 >= f->bw) {
                t->bx -= 2 * tw4;
            } else {
                t->bx += tw4;
                res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
                t->bx -= 3 * tw4;
            }
        }
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_H5: {
        const enum RectTxfmSize tx_big = tp[TX_PARTITION_H];
        const TxfmInfo *const t_dim_small = &dav2d_txfm_dimensions[tx],
                       *const t_dim_big = &dav2d_txfm_dimensions[tx_big];
        const int tw4_small = t_dim_small->w, th4_small = t_dim_small->h;
        const int th4_big = t_dim_big->h;
        int res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        const int have_v_split = t->bx + tw4_small < f->bw;
        if (have_v_split) {
            t->bx += tw4_small;
            res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->bx -= tw4_small;
            if (res < 0) return res;
        }
        if (t->by + th4_small >= f->bh) break;
        t->by += th4_small;
        res = recon_b_luma_tx(t, DB_ONLY(depth) tx_big, b);
        if (res < 0 || t->by + th4_big >= f->bh) {
            t->by -= th4_small;
        } else {
            t->by += th4_big;
            res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            if (!res && have_v_split) {
                t->bx += tw4_small;
                res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
                t->bx -= tw4_small;
            }
            t->by -= th4_small + th4_big;
        }
        if (res < 0) return res;
        break;
    }
    case TX_PARTITION_V5: {
        const enum RectTxfmSize tx_big = tp[TX_PARTITION_V];
        const TxfmInfo *const t_dim_small = &dav2d_txfm_dimensions[tx],
                       *const t_dim_big = &dav2d_txfm_dimensions[tx_big];
        const int tw4_small = t_dim_small->w, th4_small = t_dim_small->h;
        const int tw4_big = t_dim_big->w;
        int res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
        if (res < 0) return res;
        const int have_h_split = t->by + th4_small < f->bh;
        if (have_h_split) {
            t->by += th4_small;
            res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            t->by -= th4_small;
            if (res < 0) return res;
        }
        if (t->bx + tw4_small >= f->bw) break;
        t->bx += tw4_small;
        res = recon_b_luma_tx(t, DB_ONLY(depth) tx_big, b);
        if (res < 0 || t->bx + tw4_big >= f->bw) {
            t->bx -= tw4_small;
        } else {
            t->bx += tw4_big;
            res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
            if (!res && have_h_split) {
                t->by += th4_small;
                res = recon_b_luma_tx(t, DB_ONLY(depth) tx, b);
                t->by -= th4_small;
            }
            t->bx -= tw4_small + tw4_big;
        }
        if (res < 0) return res;
        break;
    }
    default: assert(0);
    }

    if (cbs == BS_INVALID) return 0;

    // chroma
chroma: {}
    const uint8_t *const cb_dim = dav2d_block_dimensions[cbs];
    const int cbw4 = cb_dim[0], cw4 = imin(f->bw - t->cbx, cbw4);
    const int cbh4 = cb_dim[1], ch4 = imin(f->bh - t->cby, cbh4);
    const int cbw4ss = (cbw4 + ss_hor) >> ss_hor, cbh4ss = (cbh4 + ss_ver) >> ss_ver;
    const int cw4ss = (cw4 + ss_hor) >> ss_hor, ch4ss = (ch4 + ss_ver) >> ss_ver;
    const enum RectTxfmSize uvtx =
        f->frame_hdr->segmentation.lossless[b->seg_id] ? (int) TX_4X4 :
        dav2d_max_txfm_size_for_bs[cbs][DAV2D_PIXEL_LAYOUT_I444 - f->cur.p.p.layout];
    const TxfmInfo *const uv_t_dim = &dav2d_txfm_dimensions[uvtx];
    const int ctw4 = imin(uv_t_dim->w, (f->bw - t->cbx + ss_hor) >> ss_hor);
    const int cth4 = imin(uv_t_dim->h, (f->bh - t->cby + ss_ver) >> ss_ver);
    const int ctw = uv_t_dim->w * 4, cth = uv_t_dim->h * 4;
    const int bx4 = t->cbx & 63, by4 = t->cby & 63;
    const int cbx4 = bx4 >> ss_hor, cby4 = by4 >> ss_ver;
    const int ssbx = t->cbx >> ss_hor, ssby = t->cby >> ss_ver;
    const ptrdiff_t stride = f->cur.p.stride[1];
    const ptrdiff_t uvdstoff = 4 * (ssby * PXSTRIDE(stride) + ssbx);
    const int sbsz = f->sb_step;
    const int sdp_active = lbs == BS_INVALID;
    const int intra = b->intra && (sdp_active || !b->intrabc);
    const int skip_txfm = !sdp_active && b->skip_txfm;

    const enum IntraPredMode orig_uv_mode = b->uv_mode;
    int angle = b->uv_angle;
    if (intra)
        b->uv_mode = wide_angle_remap(uv_t_dim, b->uv_mode, &angle, 0);

    if (cbs_stage[0] != BS_INVALID) {
        if (!(t->task_thread.pass & PASS_ENTROPY)) {
            if (!skip_txfm) {
                uint16_t /*enum TxfmType*/ (*const txtp)[2] = t->chroma_txtp;
                int16_t (*const uv_eob)[2] = t->chroma_eob;
                for (int y = 0; y < ch4ss; y += uv_t_dim->h) {
                    for (int x = 0; x < cw4ss; x += uv_t_dim->w) {
                        const ptrdiff_t i = y * cbw4ss + x;
                        const struct CodedBlockInfo *const cbi =
                            &f->frame_thread.cbi[(t->cby + (y << ss_ver)) * f->b4_stride +
                                                 (t->cbx + (x << ss_hor))];
                        for (int pl = 0; pl < 2; pl++) {
                            txtp[i][pl] = cbi->txtp[1 + pl];
                            uv_eob[i][pl] = cbi->eob[1 + pl];
                        }
                    }
                }
                t->cf_uv = ts->frame_thread[1].cf;
                ts->frame_thread[1].cf += cbw4ss * cbh4ss * 16 * 2;
            }
        } else if (skip_txfm) {
            for (int pl = 0; pl < 2; pl++) {
                dav2d_memset_likely_pow2(&t->a->ccoef[pl][cbx4], 0x40, cw4ss);
                dav2d_memset_likely_pow2(&t->l.ccoef[pl][cby4], 0x40, ch4ss);
            }
        } else {
            enum TxfmType y_txtp = t->txtp_map[(t->by & 15) * 16 + (t->bx & 15)];
            uint16_t /*enum TxfmType*/ (*const txtp)[2] = t->chroma_txtp;
            int16_t (*const uv_eob)[2] = t->chroma_eob;
            uint8_t cf_ctx[2];
            coef (*const cf)[64 * 64] = bitfn(t->cf_uv);
            // decode coefficients
            for (int pl = 0; pl < 2; pl++) {
                int y;
                for (y = 0; y < ch4ss; y += uv_t_dim->h) {
                    int x;
                    for (x = 0; x < cw4ss; x += uv_t_dim->w) {
                        const ptrdiff_t i = y * cbw4ss + x;
                        if (b->bs == b->cbs)
                            y_txtp = t->txtp_map[(t->by & 15) * 16 + (t->bx & 15)];
                        enum TxfmType uv_txtp = y_txtp;
                        const int eob =
                            decode_coefs(t, DB_ONLY(depth + 1)
                                         &t->a->ccoef[pl][cbx4 + x],
                                         &t->l.ccoef[pl][cby4 + y],
                                         uvtx, b->cbs,
                                         sdp_active, b, pl + 1,
                                         &cf[pl][i * 16],
                                         &uv_txtp, &cf_ctx[pl]);
                        if (!pl) t->u_has_cf = eob >= 0;
                        txtp[i][pl] = uv_txtp;
                        if (eob == INT_MIN) return -1;
                        DEBUG_BLOCK_printf("%*sPost-%c_cf_blk[tx=%dx%d,txtp=%s/%s,"
                                           "eob=%d]: r=%d\n",
                                           depth + 1, "", "uv"[pl], ctw, cth,
                                           dav2d_tx1d_names[uv_txtp & 7],
                                           dav2d_tx1d_names[(uv_txtp >> 5) & 7],
                                           eob, t->ts->msac.rng);
                        uv_eob[i][pl] = eob;
                        dav2d_memset_likely_pow2(&t->a->ccoef[pl][cbx4 + x],
                                                 cf_ctx[pl], ctw4);
                        dav2d_memset_likely_pow2(&t->l.ccoef[pl][cby4 + y],
                                                 cf_ctx[pl], cth4);
                        t->bx += uv_t_dim->w << ss_hor;
                    }
                    t->bx -= x << ss_hor;
                    t->by += uv_t_dim->h << ss_ver;
                }
                t->by -= y << ss_ver;
            }
        }
        if (cbs_stage[1] == BS_INVALID) {
            b->uv_mode = orig_uv_mode;
            return 0;
        }
    }

    if (intra) {
        if (b->uv_mode == CFL_PRED)
            cfl(t, b, cbs, uv_t_dim);
    } else if (!sdp_active && b->intrabc) {
        for (int pl = 0; pl < 2; pl++) {
            mc(t, ((pixel *)f->cur.p.data[1 + pl]) + uvdstoff, NULL,
               stride, cbw4, cbh4, t->cbx, t->cby, 1 + pl,
               b->mv[0], &f->cur, 0 /* unused */, DAV2D_FILTER_BILINEAR,
               0, f->bw * 4 >> ss_hor, 0, f->bh * 4 >> ss_ver);
            // FIXME morph_pred?
            if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                hex_dump(((pixel *) f->cur.p.data[1 + pl]) + uvdstoff,
                         stride, cbw4 * 4 >> ss_hor, cbh4 * 4 >> ss_ver,
                         pl ? "v-pred" : "u-pred");
        }
    } else if (cbs != lbs && imin(bw4, bh4) < 16) {
        // sub8x8 coding
        const refmvs_block *r = &t->rt.r[(t->cby & 63) * 128 + (t->cbx & 127)];
        ptrdiff_t uvoff = uvdstoff;
        for (int y = 0; y < ch4; y++, r += 128,
             uvoff += 4 * PXSTRIDE(stride) >> ss_ver)
        {
            for (int x = 0; x < cw4; x++) {
                // grab ref/MV from spatial refmvs
                const refmvs_block *const r2 = &r[x];
                if (r2->ox4 || r2->oy4) continue;
                const int ref = r2->ref.ref[0];
                const union mv mv = r2->mf & 2 ? r2->lmv[0] : r2->mv[0];
                const Dav2dThreadPicture *const refp = &f->refp[ref];
                const uint8_t *const sdim = dav2d_block_dimensions[r2->bs];
                for (int pl = 0; pl < 2; pl++) {
                    mc(t, ((pixel *) f->cur.p.data[1 + pl]) + uvoff + (x * 4 >> ss_hor),
                       NULL, stride, sdim[0], sdim[1], t->cbx + x, t->cby + y,
                       1 + pl, mv, refp, ref, r2->subpel_filter,
                       0, f->bw * 4 >> ss_hor, 0, f->bh * 4 >> ss_ver);
                }
            }
        }
        if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
            for (int pl = 0; pl < 2; pl++)
                hex_dump(((pixel *) f->cur.p.data[1 + pl]) + uvdstoff,
                         stride, cw4 * 4 >> ss_hor, ch4 * 4 >> ss_ver,
                         pl ? "v-pred" : "u-pred");
    } else if (b->ref.ref[1] == -1 && b->ref.ref[0] != TIP_FRAME) {
        const Dav2dThreadPicture *const refp = &f->refp[b->ref.ref[0]];
        for (int pl = 0; pl < 2; pl++) {
            pixel *const dst = ((pixel *) f->cur.p.data[1 + pl]) + uvdstoff;
            if (!f->frame_hdr->force_integer_mv &&
                ((b->inter_mode == GLOBALMV && imin(bw4, bh4) > 1 &&
                  f->gmv_warp_allowed[b->ref.ref[0]]) ||
                 (b->motion_mode >= MM_WARP_CAUSAL &&
                  t->warpmv[0].type > DAV2D_WM_TYPE_INVALID)))
            {
                warp_affine(t, dst, NULL, stride, cb_dim, 1 + pl, refp,
                            b->motion_mode >= MM_WARP_CAUSAL ? &t->warpmv[0] :
                                &f->frame_hdr->gmv.m[b->ref.ref[0]]);
            } else {
                mc(t, dst, NULL, stride,
                   cbw4, cbh4, t->cbx, t->cby, 1 + pl, b->mv[0], refp, b->ref.ref[0],
                   b->filter, 0, f->bw * 4 >> ss_hor, 0, f->bh * 4 >> ss_ver);
            }
            if (b->bawp[1]) {
                bawp(t, 1, b->mv[0], dst, f->cur.p.stride[1],
                     refp, b->ref.ref[0], cbw4, cbh4, cw4, ch4, pl + 1, b->bs);
            } else if (b->motion_mode == MM_INTERINTRA || b->warp_ii) {
                iiblend(t, b, dst, stride, 1 + pl, cbw4, cbh4, t->cby, t->cbx,
                        b->wedge_idx == -1 ? dav2d_ss_bs[cbs][f->cur.p.p.layout - 1] : cbs);
            }
            if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                hex_dump(dst, stride, cbw4 * 4 >> ss_hor, cbh4 * 4 >> ss_ver,
                         pl ? "v-pred" : "u-pred");
        }
    } else /* compound-inter */ {
        int16_t (*tmp)[64 * 64] = t->scratch.compinter;
        for (int pl = 0, bacp, bacpu; pl < 2; pl++, bacpu = bacp, bacp = 0) {
            if (b->ref.ref[0] == TIP_FRAME) {
                const int opfl = f->seq_hdr->tip_refine_mv &&
                    (f->frame_hdr->tip.frame_mode == 1 ||
                     f->frame_hdr->tip.subpel_filter == DAV2D_FILTER_8TAP_SHARP);
                const int step = 2 << (f->frame_hdr->tip.frame_mode == 2 /* frame */ ? !opfl :
                                       ((!opfl && imin(bw4, bh4) >= 4) || b->bs == BS_256x256));
                bacp = rmv_uvpred(t, b, pl, step, step, cbw4, cbh4);
            } else if (b->inter_mode >= OPFL_NEARMV_NEARMV ||
                       (b->refine_mv && b->comp_type == COMP_INTER_AVG))
            {
                const int refine = b->comp_type == COMP_INTER_AVG && b->refine_mv;
                const int opfl = b->inter_mode >= OPFL_NEARMV_NEARMV;
                bacp = rmv_uvpred(t, b, pl, 2 << refine, 4 >> opfl, cbw4, cbh4);
            } else {
                if (!pl)
                    bacp = 2 * (f->seq_hdr->imp_msk_bld &&
                                b->motion_mode != MM_WARP_CAUSAL &&
                                b->inter_mode != GLOBALMV_GLOBALMV &&
                                !f->svc[b->ref.ref[0]][0].scale &&
                                !f->svc[b->ref.ref[1]][0].scale);
                for (int i = 0; i < 2; i++) {
                    const Dav2dThreadPicture *const refp = &f->refp[b->ref.ref[i]];
                    if ((b->inter_mode == GLOBALMV_GLOBALMV && imin(bw4, bh4) > 1 &&
                         f->gmv_warp_allowed[b->ref.ref[i]]) ||
                        (b->motion_mode == MM_WARP_CAUSAL &&
                         t->warpmv[i].type > DAV2D_WM_TYPE_INVALID))
                    {
                        warp_affine(t, NULL, tmp[i], cbw4 * 4 >> ss_hor,
                                    cb_dim, 1 + pl, refp,
                                    b->motion_mode >= MM_WARP_CAUSAL ? &t->warpmv[i] :
                                        &f->frame_hdr->gmv.m[b->ref.ref[i]]);
                    } else {
                        mc(t, NULL, tmp[i], cbw4 * 4 >> ss_hor, cbw4, cbh4,
                           t->cbx, t->cby, 1 + pl, b->mv[i],
                           refp, b->ref.ref[i], b->filter,
                           0, f->bw * 4 >> ss_hor, 0, f->bh * 4 >> ss_ver);
                    }
                }
            }
            switch (b->comp_type) {
            case COMP_INTER_SEG: {
                const ptrdiff_t mask_stride = cbw4 * 4 >> ss_hor;
                assert(mask_stride <= 64);
                uint8_t *const seg_mask = imin(cbw4, cbh4) < 16 ? t->scratch.seg_mask :
                    &t->scratch.seg_mask[(ssby & 15) * 4 * mask_stride + (ssbx & 15) * 4];
                dsp->mc.mask(((pixel *) f->cur.p.data[1 + pl]) + uvdstoff, stride,
                             tmp[b->mask_sign], tmp[!b->mask_sign],
                             cbw4 * 4 >> ss_hor, cbh4 * 4 >> ss_ver, seg_mask
                             HIGHBD_CALL_SUFFIX);
                break;
            }
            case COMP_INTER_WEDGE: {
                const uint8_t *const mask =
                    WEDGE_MASK(cbs, cbw4, cbh4, b->wedge_idx, ss_hor + ss_ver);
                dsp->mc.mask(((pixel *) f->cur.p.data[1 + pl]) + uvdstoff, stride,
                             tmp[b->wedge_sign], tmp[!b->wedge_sign],
                             cbw4 * 4 >> ss_hor, cbh4 * 4 >> ss_ver,
                             mask HIGHBD_CALL_SUFFIX);
                break;
            }
            default: assert(0);
            case COMP_INTER_NONE:
                assert(b->ref.ref[0] == TIP_FRAME);
                // fall-through
            case COMP_INTER_AVG: {
                const int wt = b->cwp_idx;
                if (wt == 8) {
                    if (bacp == 2)
                        bacp = get_mask(t->scratch.seg_mask, cbw4 * 4 >> ss_hor,
                                        t->cbx >> ss_hor, 0, t->cby >> ss_ver, 0,
                                        b->mv, 3 + ss_hor, 3 + ss_ver,
                                        cbw4 >> ss_hor, cbh4 >> ss_ver,
                                        f->bw * 4 >> ss_hor, f->bh * 4 >> ss_ver);
                    if (pl) bacp = bacpu;
                    if (bacp) {
                        dsp->mc.mask(((pixel *) f->cur.p.data[1 + pl]) + uvdstoff, stride,
                                     tmp[0], tmp[1], cbw4 * 4 >> ss_hor,
                                     cbh4 * 4 >> ss_ver, t->scratch.seg_mask
                                     HIGHBD_CALL_SUFFIX);
                    } else {
                        dsp->mc.avg(((pixel *) f->cur.p.data[1 + pl]) + uvdstoff, stride,
                                    tmp[0], tmp[1], cbw4 * 4 >> ss_hor,
                                    cbh4 * 4 >> ss_ver HIGHBD_CALL_SUFFIX);
                    }
                } else {
                    dsp->mc.w_avg(((pixel *) f->cur.p.data[1 + pl]) + uvdstoff, stride,
                                  tmp[0], tmp[1], cbw4 * 4 >> ss_hor,
                                  cbh4 * 4 >> ss_ver, wt HIGHBD_CALL_SUFFIX);
                }
                break;
            }}
            if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS)
                hex_dump(((pixel *) f->cur.p.data[1 + pl]) + uvdstoff,
                         stride, cbw4 * 4 >> ss_hor, cbh4 * 4 >> ss_ver,
                         pl ? "v-pred" : "u-pred");
        }
    }

    // x/y recon loop
    for (int y = 0; y < ch4ss; y += uv_t_dim->h) {
        for (int x = 0; x < cw4ss; x += uv_t_dim->w) {
            const ptrdiff_t i = y * cbw4ss + x;
            for (int pl = 0; pl < 2; pl++) {
                pixel *const dst = ((pixel *) f->cur.p.data[1 + pl]) +
                    4 * ((ssby + y) * PXSTRIDE(stride) + ssbx + x);
                if (intra && b->uv_mode != CFL_PRED) {
                    // intra prediction
                    pixel *const edge = bitfn(t->scratch.edge) + 128;
                    const pixel *top_sb_edge = NULL;
                    // We're skipping upsampling y here as this condition is
                    // only true when y is 0.
                    if (!((t->cby + y) & (sbsz - 1))) {
                        top_sb_edge = f->ipred_edge[1 + pl];
                        const int sby = t->cby >> f->sb_shift;
                        top_sb_edge += (sby - 1) * f->sb256w * 256 >> ss_hor;
                    }

                    int n_tr = 0, n_bl = 0;
                    if (t->cby + (y << ss_ver) > ts->tiling.row_start && ctw < 64) {
                        const int csbsz = sbsz >> ss_hor;
                        const int tile_end = ts->tiling.col_end >> ss_hor;
                        int w = imin(ctw4, tile_end - (ssbx + x) - ctw4);
                        if (!((t->cby + y) & (sbsz - 1))) {
                            n_tr = w; // top sb boundary
                        } else {
                            const int end = imin((ssbx + x + csbsz) & ~(csbsz - 1), tile_end);
                            int w = imin(ctw4, end - (ssbx + x) - ctw4);
                            if (!w) {
                                // right sb boundary
                                n_tr = w;
                            } else {
                                const unsigned bits = (unsigned)
                                    (t->is_coded[1][cby4 + y - 1] >> (cbx4 + x + ctw4));
                                n_tr = imin(ctz(0x10000 | ~bits), w);
                            }
                        }
                    }
                    if (t->cbx + (x << ss_hor) > ts->tiling.col_start && cth < 64) {
                        const int csbsz = sbsz >> ss_ver;
                        const int end = imin((ssby + y + csbsz) & ~(csbsz - 1),
                                             ts->tiling.row_end >> ss_ver);
                        const int h = imin(cth4, end - (ssby + y) - cth4);
                        if (!((t->cbx + x) & (sbsz - 1)) || !h) {
                            // left or bottom sb boundary
                            n_bl = h;
                        } else {
                            const uint64_t mask = 1ULL << (cbx4 + x - 1);
                            for (; n_bl < h; n_bl++)
                                if (!(t->is_coded[1][cby4 + y + n_bl + cth4] & mask))
                                    break;
                        }
                    }

                    int apply_ibp = f->seq_hdr->ibp && uvtx != (enum RectTxfmSize) TX_4X4;
                    const int sm_top = b->is_sm[1].a;
                    const int sm_left = b->is_sm[1].l;
                    const int is_sm_flag = apply_ibp ?
                        (sm_top * ANGLE_SMOOTH_TOP_EDGE_FLAG) |
                        (sm_left * ANGLE_SMOOTH_LEFT_EDGE_FLAG) :
                        (sm_top | sm_left) * (ANGLE_SMOOTH_TOP_EDGE_FLAG |
                                              ANGLE_SMOOTH_LEFT_EDGE_FLAG);
                    apply_ibp &= b->uv_mode == DC_PRED;
                    int intra_flags = is_sm_flag |
                        (apply_ibp ? ANGLE_IBP_FLAG : 0) |
                        (f->seq_hdr->intra_edge_filter ? ANGLE_USE_EDGE_FILTER_FLAG : 0) |
                        ((t->cbx + (x << ss_hor) > ts->tiling.col_start) ? ANGLE_HAS_LEFT_FLAG : 0) |
                        ((t->cby + (y << ss_ver) > ts->tiling.row_start) ? ANGLE_HAS_TOP_FLAG  : 0);
                    const enum IntraPredMode uv_mode =
                        b->uv_mode == CFL_PRED ? DC_PRED : b->uv_mode;

                    const enum IntraPredMode m = bytefn(dav2d_prepare_intra_edges)(
                        // don't print chroma as avm does things in a different order
                        // (decode coefs of both planes first then pred + itx)
                        DB_ONLY(0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) ssbx + x, ssby + y,
                        ts->tiling.col_end >> ss_hor, ts->tiling.row_end >> ss_ver,
                        n_tr, n_bl, dst, stride, top_sb_edge, uv_mode, uv_t_dim->w,
                        uv_t_dim->h, angle | intra_flags, edge HIGHBD_CALL_SUFFIX);

                    dsp->ipred.intra_pred[m](dst, stride,
                                             edge, ctw, cth, angle | intra_flags,
                                             4 * f->bw - 4 * (t->cbx + x),
                                             4 * f->bh - 4 * (t->cby + y) HIGHBD_CALL_SUFFIX);

                    if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                        hex_dump(dst, stride, ctw, cth, pl ? "v-intra-pred" : "u-intra-pred");
                    }
                }
            }

            if (!skip_txfm) {
                const int cctx = f->seq_hdr->cctx &&
                    (f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420 || uv_t_dim->max < 8);
                uint16_t /*enum TxfmType*/ (*const txtp)[2] = t->chroma_txtp;
                int16_t (*const eob)[2] = t->chroma_eob;
                coef *cf[2];
                if (!(t->task_thread.pass & PASS_ENTROPY)) {
                    cf[0] = t->cf_uv;
                    cf[1] = &t->cf_uv[cbw4ss * cbh4ss * 16];
                } else {
                    cf[0] = bitfn(t->cf_uv)[0];
                    cf[1] = bitfn(t->cf_uv)[1];
                }
                // FIXME I'm not convinced the above is correct for pass==2 if we
                // do block coding vs. reconstruction in different orders for
                // e.g. 128x128 chroma blocks.
                int cctx_type = cctx && eob[i][0] >= intra ? (txtp[i][0] >> 8) : 0;
                if (cctx_type) {
                    dsp->itx.cctx(&cf[0][i * 16], &cf[1][i * 16],
                                  dav2d_cctx_angle[cctx_type - 1],
                                  umin(ctw, 32) * umin(cth, 32) HIGHBD_CALL_SUFFIX);
                    const int gt = eob[i][1] > eob[i][0];
                    eob[i][!gt] = eob[i][gt];
                    txtp[i][1] = txtp[i][0] &= 0xff;
                }
                // inverse transform
                for (int pl = 0; pl < 2; pl++) {
                    if (eob[i][pl] != -1) {
                        // don't print chroma as avm does things in a different order
                        // (decode coefs of both planes first then pred + itx)
                        if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                            coef_dump(&cf[pl][i * 16], imin(cth, 32), imin(ctw, 32), 3, "dq");
                        }
                        pixel *const dst = ((pixel *) f->cur.p.data[1 + pl]) +
                            4 * ((ssby + y) * PXSTRIDE(stride) + ssbx + x);
                        if (f->frame_hdr->segmentation.lossless[b->seg_id] &&
                            b->intra && (sdp_active || !b->intrabc) && b->dpcm[1])
                        {
                            txtp[i][pl] += (1 + (b->uv_mode == VERT_PRED)) << 8;
                        } else if (f->seq_hdr->inter_ddt && !b->intra) // (flip)adst -> (f)ddt
                            txtp[i][pl] += txtp[i][pl] & dav2d_tx_ddt_mask[uvtx];
                        dsp->itx.itxfm_add[uvtx](dst, stride, &cf[pl][i * 16], txtp[i][pl], eob[i][pl]
                                                 HIGHBD_CALL_SUFFIX);
                    }
                }
            }
            for (int pl = 1; pl <= 2; pl++) {
                if (0 && BLOCK_TO_DEBUG && DEBUG_B_PIXELS) {
                    const pixel *const dst = ((pixel *) f->cur.p.data[pl]) +
                        4 * ((ssby + y) * PXSTRIDE(stride) + ssbx + x);
                    hex_dump(dst, stride, ctw, cth, "recon");
                }
            }
            const uint64_t mask = ((1ULL << ctw4) - 1) << (cbx4 + x);
            for (int yy = 0; yy < cth4; yy++)
                t->is_coded[1][cby4 + y + yy] |= mask;
        }
    }

    b->uv_mode = orig_uv_mode;
    return 0;
}

void bytefn(dav2d_filter_sbrow_deblock_cols)(Dav2dFrameContext *const f, const int sby) {
    if (!(f->c->inloop_filters & DAV2D_INLOOPFILTER_DEBLOCK) ||
        (!f->frame_hdr->deblock.level_y[0] && !f->frame_hdr->deblock.level_y[1]))
    {
        return;
    }
    const int y = sby * f->sb_step * 4;
    const int ss_ver = f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
    pixel *const p[3] = {
        f->lf.p[0] + y * PXSTRIDE(f->cur.p.stride[0]),
        f->lf.p[1] + (y * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver),
        f->lf.p[2] + (y * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver)
    };
    Av2Filter *mask = f->lf.mask + (sby >> (2 - f->frame_hdr->sb128)) * f->sb256w;
    const int start_of_tile_row = f->lf.start_of_tile_row[sby];
    bytefn(dav2d_deblock_sbrow_cols)(f, p, mask, sby,
                                     start_of_tile_row & 1 ? start_of_tile_row >> 1 : 0);
}

void bytefn(dav2d_filter_sbrow_deblock_rows)(Dav2dFrameContext *const f, const int sby) {
    const int y = sby * f->sb_step * 4;
    const int ss_ver = f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
    pixel *const p[3] = {
        f->lf.p[0] + y * PXSTRIDE(f->cur.p.stride[0]),
        f->lf.p[1] + (y * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver),
        f->lf.p[2] + (y * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver)
    };
    Av2Filter *mask = f->lf.mask + (sby >> (2 - f->frame_hdr->sb128)) * f->sb256w;
    if (f->c->inloop_filters & DAV2D_INLOOPFILTER_DEBLOCK &&
        (f->frame_hdr->deblock.level_y[0] || f->frame_hdr->deblock.level_y[1]))
    {
        bytefn(dav2d_deblock_sbrow_rows)(f, p, mask, sby);
    }
    if ((f->seq_hdr->cdef &&
         f->c->inloop_filters & DAV2D_INLOOPFILTER_CDEF) ||
        (f->lf.restore_planes &&
         f->c->inloop_filters & (DAV2D_INLOOPFILTER_WIENER | DAV2D_INLOOPFILTER_GDF)))
    {
        // Store deblocked pixels required by CDEF / LR
        bytefn(dav2d_copy_db)(f, p, sby);
    }
}

void bytefn(dav2d_filter_sbrow_cdef)(Dav2dTaskContext *const tc, const int sby) {
    const Dav2dFrameContext *const f = tc->f;
    if (!(f->c->inloop_filters & (DAV2D_INLOOPFILTER_CDEF | DAV2D_INLOOPFILTER_CCSO))) return;
    const int sbsz = f->sb_step;
    const int y = sby * sbsz * 4;
    const int ss_ver = f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
    pixel *const p[3] = {
        f->lf.p[0] + y * PXSTRIDE(f->cur.p.stride[0]),
        f->lf.p[1] + (y * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver),
        f->lf.p[2] + (y * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver)
    };
    Av2Filter *prev_mask = f->lf.mask + ((sby - 1) >> (2 - f->frame_hdr->sb128)) * f->sb256w;
    Av2Filter *mask = f->lf.mask + (sby >> (2 - f->frame_hdr->sb128)) * f->sb256w;
    const int start = sby * sbsz;
    if (sby) {
        const int ss_ver = f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
        pixel *p_up[3] = {
            p[0] - 8 * PXSTRIDE(f->cur.p.stride[0]),
            p[1] - (8 * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver),
            p[2] - (8 * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver),
        };
        bytefn(dav2d_cdef_brow)(tc, p_up, prev_mask, start - 2, start, 1, sby);
    }
    const int n_blks = sbsz - 2 * (sby + 1 < f->sbh);
    const int end = imin(start + n_blks, f->bh);
    bytefn(dav2d_cdef_brow)(tc, p, mask, start, end, 0, sby);
}

void bytefn(dav2d_filter_sbrow_lr)(Dav2dFrameContext *const f, const int sby) {
    if (!(f->c->inloop_filters & (DAV2D_INLOOPFILTER_WIENER | DAV2D_INLOOPFILTER_GDF)))
        return;
    const int y = sby * f->sb_step * 4;
    const int ss_ver = f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
    pixel *const sr_p[3] = {
        f->lf.sr_p[0] + y * PXSTRIDE(f->cur.p.stride[0]),
        f->lf.sr_p[1] + (y * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver),
        f->lf.sr_p[2] + (y * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver)
    };
    bytefn(dav2d_lr_sbrow)(f, sr_p, sby);
}

void bytefn(dav2d_filter_sbrow)(Dav2dFrameContext *const f, const int sby) {
    bytefn(dav2d_filter_sbrow_deblock_cols)(f, sby);
    bytefn(dav2d_filter_sbrow_deblock_rows)(f, sby);
    if (f->seq_hdr->cdef)
        bytefn(dav2d_filter_sbrow_cdef)(f->c->tc, sby);
    if (f->lf.restore_planes)
        bytefn(dav2d_filter_sbrow_lr)(f, sby);
}

void bytefn(dav2d_backup_ipred_edge)(Dav2dTaskContext *const t) {
    const Dav2dFrameContext *const f = t->f;
    Dav2dTileState *const ts = t->ts;
    if (t->by + f->sb_step >= ts->tiling.row_end) return;
    const int sby = t->by >> f->sb_shift;
    const int sby_off = f->sb256w * 256 * sby;
    const int x_off = ts->tiling.col_start;

    const pixel *const y =
        ((const pixel *) f->cur.p.data[0]) + x_off * 4 +
                    ((t->by + f->sb_step) * 4 - 1) * PXSTRIDE(f->cur.p.stride[0]);
    pixel_copy(&f->ipred_edge[0][sby_off + x_off * 4], y,
               4 * (ts->tiling.col_end - x_off));

    if (f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I400) {
        const int ss_ver = f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
        const int ss_hor = f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444;

        const ptrdiff_t uv_off = (x_off * 4 >> ss_hor) +
            (((t->by + f->sb_step) * 4 >> ss_ver) - 1) * PXSTRIDE(f->cur.p.stride[1]);
        for (int pl = 1; pl <= 2; pl++)
            pixel_copy(&f->ipred_edge[pl][(sby_off + x_off * 4) >> ss_hor],
                       &((const pixel *) f->cur.p.data[pl])[uv_off],
                       4 * (ts->tiling.col_end - x_off) >> ss_hor);
    }
}

void bytefn(dav2d_copy_pal_block_y)(Dav2dTaskContext *const t,
                                    const int bx4, const int by4,
                                    const int bw4, const int bh4)

{
    pixel *const pal = t->task_thread.pass != PASS_ALL ?
        *t->ts->frame_thread[1].pal++ : bytefn(t->scratch.pal);
    for (int x = 0; x < bw4; x++)
        memcpy(bytefn(t->al_pal)[0][bx4 + x], pal, 8 * sizeof(pixel));
    for (int y = 0; y < bh4; y++)
        memcpy(bytefn(t->al_pal)[1][by4 + y], pal, 8 * sizeof(pixel));
}

void bytefn(dav2d_read_pal_plane)(DB_ONLY(const int depth)
                                  Dav2dTaskContext *const t, Av2Block *const b,
                                  const int bx4, const int by4)
{
    Dav2dTileState *const ts = t->ts;
    const Dav2dFrameContext *const f = t->f;
    const int pal_sz = b->pal_sz =
        dav2d_msac_decode_symbol_adapt8(&ts->msac, ts->cdf.m.pal_sz, 6) + 2;
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
        const unsigned m = dav2d_msac_decode_bools_bypass(&ts->msac, n);
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
            } while (mask)
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
    pixel *const pal = t->task_thread.pass != PASS_ALL ?
        *ts->frame_thread[1].pal : bytefn(t->scratch.pal);
    if (n_used_cache < pal_sz) {
        int i = n_used_cache;
        const int bpc = BITDEPTH == 8 ? 8 : f->cur.p.p.bpc;
        int prev = pal[i++] = dav2d_msac_decode_bools_bypass(&ts->msac, bpc);

        if (i < pal_sz) {
            int bits = bpc - 3 + dav2d_msac_decode_bools_bypass(&ts->msac, 2);
            const int max = (1 << bpc) - 1;

            do {
                const int delta = dav2d_msac_decode_bools_bypass(&ts->msac, bits);
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

