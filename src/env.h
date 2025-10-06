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

#ifndef DAV1D_SRC_ENV_H
#define DAV1D_SRC_ENV_H

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

#include "src/levels.h"
#include "src/refmvs.h"
#include "src/tables.h"

typedef struct BlockContext {
    uint8_t ALIGN(fsc[64], 8);
    uint8_t ALIGN(mode[64], 8);
    uint8_t ALIGN(midx[64], 8);
    uint8_t ALIGN(mrl[64], 8);
    uint8_t ALIGN(multi_mrl[64], 8);
    uint8_t ALIGN(dip[64], 8);
    uint8_t ALIGN(lcoef[64], 8);
    uint8_t ALIGN(ccoef[2][64], 8);
    uint8_t ALIGN(seg_pred[64], 8);
    uint8_t ALIGN(skip_txfm[64], 8);
    uint8_t ALIGN(skip_mode[64], 8);
    uint8_t ALIGN(intra[64], 8);
    uint8_t ALIGN(intrabc[64], 8);
    uint8_t ALIGN(morph_pred[64], 8);
    uint8_t ALIGN(comp_type[64], 8);
    int8_t ALIGN(ref[2][64], 8); // -1 means intra
    uint8_t ALIGN(motion_mode[64], 8);
    uint8_t ALIGN(amvd[64], 8);
    uint8_t ALIGN(mvprec[64], 8);
    uint8_t ALIGN(filter[64], 8); // DAV1D_N_SWITCHABLE_FILTERS=3 means unset
    uint8_t ALIGN(tx_lpf_y[64], 8);
    uint8_t ALIGN(tx_lpf_uv[64], 8);
    uint8_t ALIGN(partition[2][64], 8);
    uint8_t ALIGN(uvmode[64], 8);
    uint8_t ALIGN(pal_sz[64], 8);
} BlockContext;

static inline int get_intra_ctx(const BlockContext *nx[2],
                                const int xoff[2], const int n_ctx)
{
    if (!n_ctx) return 0;
    const int sum = nx[0]->intra[xoff[0]] + nx[n_ctx - 1]->intra[xoff[n_ctx - 1]];
    return sum + (sum == n_ctx);
}

static inline int get_partition_ctx(const BlockContext *const a,
                                    const BlockContext *const l,
                                    const uint8_t *const b_dim,
                                    const int plane,
                                    const int yb4, const int xb4)
{
    return ((a->partition[plane][xb4] >> imax(b_dim[2] - 1, 0)) & 1) +
          (((l->partition[plane][yb4] >> imax(b_dim[3] - 1, 0)) & 1) << 1);
}

static inline int get_partition2_ctx(const BlockContext *const a,
                                     const BlockContext *const l,
                                     const uint8_t *const b_dim,
                                     const int plane, const int dir,
                                     const int yb4, const int xb4)
{
    if (!dir /* horizontal */) {
        const int hh4 = b_dim[1] >> 1;
        return ((l->partition[plane][yb4 + hh4] >> (b_dim[3] - 2)) & 1) +
              (((l->partition[plane][yb4] >> (b_dim[3] - 2)) & 1) << 1);
    } else /* vertical */ {
        const int hw4 = b_dim[0] >> 1;
        return ((a->partition[plane][xb4 + hw4] >> (b_dim[2] - 2)) & 1) +
              (((a->partition[plane][xb4] >> (b_dim[2] - 2)) & 1) << 1);
    }
}

static inline enum TxfmType get_uv_inter_txtp(const TxfmInfo *const uvt_dim,
                                              const enum TxfmType ytxtp)
{
    if (uvt_dim->max == TX_32X32)
        return ytxtp == IDTX ? IDTX : DCT_DCT;
    if (uvt_dim->min == TX_16X16 &&
        ((1 << ytxtp) & ((1 << H_FLIPADST) | (1 << V_FLIPADST) |
                         (1 << H_ADST) | (1 << V_ADST))))
    {
        return DCT_DCT;
    }

    return ytxtp;
}

static inline int get_filter_ctx(const BlockContext *nb[2],
                                 const int boff[2], const int8_t refs[2])
{
    const int ref = refs[0];
    const enum Dav1dFilterMode flt[2] = {
        (boff[0] != -1 && (nb[0]->ref[0][boff[0]] == ref ||
                           nb[0]->ref[1][boff[0]] == ref)) ?
        nb[0]->filter[boff[0]] : DAV1D_N_SWITCHABLE_FILTERS,
        (boff[1] != -1 && (nb[1]->ref[0][boff[1]] == ref ||
                           nb[1]->ref[1][boff[1]] == ref)) ?
        nb[1]->filter[boff[1]] : DAV1D_N_SWITCHABLE_FILTERS,
    };

    return (refs[1] != -1) * 4 + flt[flt[0] == flt[1] ||
                                     flt[0] == DAV1D_N_SWITCHABLE_FILTERS];
}

static inline int get_comp_ctx(const BlockContext *nx[2],
                               const int xoff[2], const int n_ctx,
                               const uint8_t *const refdir)
{
    switch (n_ctx) {
    default: assert(0);
    case 2: {
        const int refa2 = nx[0]->ref[1][xoff[0]];
        const int refb2 = nx[1]->ref[1][xoff[1]];
        if (refa2 == -1) {
            const int refa1 = nx[0]->ref[0][xoff[0]];
            if (refb2 == -1) {
                const int refb1 = nx[1]->ref[0][xoff[1]];
                return refdir[refa1] ^ refdir[refb1];
            } else return nx[0]->intra[xoff[0]] || refdir[refa1];
        } else if (refb2 == -1) {
            const int refb1 = nx[1]->ref[0][xoff[1]];
            return nx[1]->intra[xoff[1]] || refdir[refb1];
        } else return 4;
    }
    case 1: {
        const int ref2 = nx[0]->ref[1][xoff[0]];
        if (ref2 == -1) {
            const int ref1 = nx[0]->ref[0][xoff[0]];
            return nx[0]->intra[xoff[0]] || refdir[ref1];
        } else return 3;
    }
    case 0: return 1;
    }
}

static inline int get_warp_ctx(const BlockContext *const a,
                               const BlockContext *const l,
                               const int yb4, const int xb4,
                               const int have_top, const int have_left,
                               const int have_top_right, const int have_bottom_left,
                               const unsigned top_is_at_tile_boundary,
                               const uint8_t *const b_dim, const int ref)
{
    int ctx = 0;

#define add_matching(dir, idx) do { \
    ctx += dir->ref[0][idx] == ref && dir->motion_mode[idx] >= 2; \
} while (0)
    if (have_top) {
        const unsigned mask = ~top_is_at_tile_boundary;
        add_matching(a, xb4 & mask);
        if (have_top_right && (b_dim[0] >= 4 || !top_is_at_tile_boundary))
            add_matching(a, (xb4 + b_dim[0] - 1 - top_is_at_tile_boundary) & mask);
    }
    if (have_left) {
        add_matching(l, yb4);
        if (have_bottom_left)
            add_matching(l, yb4 + b_dim[1] - 1);
    }
#undef add_matching

    return ctx;
}

static inline int get_sngl_ctx(const BlockContext *const a,
                               const BlockContext *const l,
                               const int yb4, const int xb4,
                               const int have_top, const int have_left,
                               const int have_top_right, const int have_bottom_left,
                               const uint8_t *const b_dim, const int ref)
{
    int row = 0, col = 0, newmv = 0;

#define NEWMV0_MODE_MASK (1 << NEWMV)
#define add_matching(dir, cnt, idx) do { \
    if (dir->ref[0][idx] == ref) { \
        cnt++; \
        newmv += !!((1 << dir->mode[idx]) & NEWMV0_MODE_MASK); \
    } else if (dir->ref[1][idx] == idx) { \
        cnt++; \
        newmv += !!((1 << dir->mode[idx]) & 0); \
    } \
} while (0)
    if (have_top) {
        add_matching(a, col, xb4);
        if (have_top_right)
            add_matching(a, col, xb4 + b_dim[0] - 1);
    }
    if (have_left) {
        add_matching(l, row, yb4);
        add_matching(l, row, yb4 + b_dim[1] - 1);
    }
#undef NEWMV0_MODE_MASK
#undef add_matching

    return !!row + !!col + 2 * !!newmv;
}

static inline int get_comp_dir_ctx(const BlockContext *const a,
                                   const BlockContext *const l,
                                   const int yb4, const int xb4,
                                   const int have_top, const int have_left)
{
#define has_uni_comp(edge, off) \
    ((edge->ref[0][off] < 4) == (edge->ref[1][off] < 4))

    if (have_top && have_left) {
        const int a_intra = a->intra[xb4], l_intra = l->intra[yb4];

        if (a_intra && l_intra) return 2;
        if (a_intra || l_intra) {
            const BlockContext *const edge = a_intra ? l : a;
            const int off = a_intra ? yb4 : xb4;

            if (edge->comp_type[off] == COMP_INTER_NONE) return 2;
            return 1 + 2 * has_uni_comp(edge, off);
        }

        const int a_comp = a->comp_type[xb4] != COMP_INTER_NONE;
        const int l_comp = l->comp_type[yb4] != COMP_INTER_NONE;
        const int a_ref0 = a->ref[0][xb4], l_ref0 = l->ref[0][yb4];

        if (!a_comp && !l_comp) {
            return 1 + 2 * ((a_ref0 >= 4) == (l_ref0 >= 4));
        } else if (!a_comp || !l_comp) {
            const BlockContext *const edge = a_comp ? a : l;
            const int off = a_comp ? xb4 : yb4;

            if (!has_uni_comp(edge, off)) return 1;
            return 3 + ((a_ref0 >= 4) == (l_ref0 >= 4));
        } else {
            const int a_uni = has_uni_comp(a, xb4), l_uni = has_uni_comp(l, yb4);

            if (!a_uni && !l_uni) return 0;
            if (!a_uni || !l_uni) return 2;
            return 3 + ((a_ref0 == 4) == (l_ref0 == 4));
        }
    } else if (have_top || have_left) {
        const BlockContext *const edge = have_left ? l : a;
        const int off = have_left ? yb4 : xb4;

        if (edge->intra[off]) return 2;
        if (edge->comp_type[off] == COMP_INTER_NONE) return 2;
        return 4 * has_uni_comp(edge, off);
    } else {
        return 2;
    }
}

static inline int get_poc_diff(const int order_hint_n_bits,
                               const int poc0, const int poc1)
{
    if (!order_hint_n_bits) return 0;
    const int mask = 1 << (order_hint_n_bits - 1);
    const int diff = poc0 - poc1;
    return (diff & (mask - 1)) - (diff & mask);
}

static inline int get_jnt_comp_ctx(const int order_hint_n_bits, const int poc,
                                   const int ref0poc, const int ref1poc,
                                   const BlockContext *const a,
                                   const BlockContext *const l,
                                   const int yb4, const int xb4)
{
    const int d0 = abs(get_poc_diff(order_hint_n_bits, ref0poc, poc));
    const int d1 = abs(get_poc_diff(order_hint_n_bits, poc, ref1poc));
    const int offset = d0 == d1;
    const int a_ctx = a->comp_type[xb4] >= COMP_INTER_AVG ||
                      a->ref[0][xb4] == 6;
    const int l_ctx = l->comp_type[yb4] >= COMP_INTER_AVG ||
                      l->ref[0][yb4] == 6;

    return 3 * offset + a_ctx + l_ctx;
}

static inline int get_mask_comp_ctx(const BlockContext *const a,
                                    const BlockContext *const l,
                                    const int yb4, const int xb4)
{
    const int a_ctx = a->comp_type[xb4] >= COMP_INTER_SEG ? 1 :
                      a->ref[0][xb4] == 6 ? 3 : 0;
    const int l_ctx = l->comp_type[yb4] >= COMP_INTER_SEG ? 1 :
                      l->ref[0][yb4] == 6 ? 3 : 0;

    return imin(a_ctx + l_ctx, 5);
}

#define av1_get_ref_2_ctx av1_get_bwd_ref_ctx
#define av1_get_ref_3_ctx av1_get_fwd_ref_ctx
#define av1_get_ref_4_ctx av1_get_fwd_ref_1_ctx
#define av1_get_ref_5_ctx av1_get_fwd_ref_2_ctx
#define av1_get_ref_6_ctx av1_get_bwd_ref_1_ctx
#define av1_get_uni_p_ctx av1_get_ref_ctx
#define av1_get_uni_p2_ctx av1_get_fwd_ref_2_ctx

static inline int av1_get_ref_ctx(const BlockContext *const a,
                                  const BlockContext *const l,
                                  const int yb4, const int xb4,
                                  int have_top, int have_left)
{
    int cnt[2] = { 0 };

    if (have_top && !a->intra[xb4]) {
        cnt[a->ref[0][xb4] >= 4]++;
        if (a->comp_type[xb4]) cnt[a->ref[1][xb4] >= 4]++;
    }

    if (have_left && !l->intra[yb4]) {
        cnt[l->ref[0][yb4] >= 4]++;
        if (l->comp_type[yb4]) cnt[l->ref[1][yb4] >= 4]++;
    }

    return cnt[0] == cnt[1] ? 1 : cnt[0] < cnt[1] ? 0 : 2;
}

static inline int av1_get_fwd_ref_ctx(const BlockContext *const a,
                                      const BlockContext *const l,
                                      const int yb4, const int xb4,
                                      const int have_top, const int have_left)
{
    int cnt[4] = { 0 };

    if (have_top && !a->intra[xb4]) {
        if (a->ref[0][xb4] < 4) cnt[a->ref[0][xb4]]++;
        if (a->comp_type[xb4] && a->ref[1][xb4] < 4) cnt[a->ref[1][xb4]]++;
    }

    if (have_left && !l->intra[yb4]) {
        if (l->ref[0][yb4] < 4) cnt[l->ref[0][yb4]]++;
        if (l->comp_type[yb4] && l->ref[1][yb4] < 4) cnt[l->ref[1][yb4]]++;
    }

    cnt[0] += cnt[1];
    cnt[2] += cnt[3];

    return cnt[0] == cnt[2] ? 1 : cnt[0] < cnt[2] ? 0 : 2;
}

static inline int av1_get_fwd_ref_1_ctx(const BlockContext *const a,
                                        const BlockContext *const l,
                                        const int yb4, const int xb4,
                                        const int have_top, const int have_left)
{
    int cnt[2] = { 0 };

    if (have_top && !a->intra[xb4]) {
        if (a->ref[0][xb4] < 2) cnt[a->ref[0][xb4]]++;
        if (a->comp_type[xb4] && a->ref[1][xb4] < 2) cnt[a->ref[1][xb4]]++;
    }

    if (have_left && !l->intra[yb4]) {
        if (l->ref[0][yb4] < 2) cnt[l->ref[0][yb4]]++;
        if (l->comp_type[yb4] && l->ref[1][yb4] < 2) cnt[l->ref[1][yb4]]++;
    }

    return cnt[0] == cnt[1] ? 1 : cnt[0] < cnt[1] ? 0 : 2;
}

static inline int av1_get_fwd_ref_2_ctx(const BlockContext *const a,
                                        const BlockContext *const l,
                                        const int yb4, const int xb4,
                                        const int have_top, const int have_left)
{
    int cnt[2] = { 0 };

    if (have_top && !a->intra[xb4]) {
        if ((a->ref[0][xb4] ^ 2U) < 2) cnt[a->ref[0][xb4] - 2]++;
        if (a->comp_type[xb4] && (a->ref[1][xb4] ^ 2U) < 2) cnt[a->ref[1][xb4] - 2]++;
    }

    if (have_left && !l->intra[yb4]) {
        if ((l->ref[0][yb4] ^ 2U) < 2) cnt[l->ref[0][yb4] - 2]++;
        if (l->comp_type[yb4] && (l->ref[1][yb4] ^ 2U) < 2) cnt[l->ref[1][yb4] - 2]++;
    }

    return cnt[0] == cnt[1] ? 1 : cnt[0] < cnt[1] ? 0 : 2;
}

static inline int av1_get_bwd_ref_ctx(const BlockContext *const a,
                                      const BlockContext *const l,
                                      const int yb4, const int xb4,
                                      const int have_top, const int have_left)
{
    int cnt[3] = { 0 };

    if (have_top && !a->intra[xb4]) {
        if (a->ref[0][xb4] >= 4) cnt[a->ref[0][xb4] - 4]++;
        if (a->comp_type[xb4] && a->ref[1][xb4] >= 4) cnt[a->ref[1][xb4] - 4]++;
    }

    if (have_left && !l->intra[yb4]) {
        if (l->ref[0][yb4] >= 4) cnt[l->ref[0][yb4] - 4]++;
        if (l->comp_type[yb4] && l->ref[1][yb4] >= 4) cnt[l->ref[1][yb4] - 4]++;
    }

    cnt[1] += cnt[0];

    return cnt[2] == cnt[1] ? 1 : cnt[1] < cnt[2] ? 0 : 2;
}

static inline int av1_get_bwd_ref_1_ctx(const BlockContext *const a,
                                        const BlockContext *const l,
                                        const int yb4, const int xb4,
                                        const int have_top, const int have_left)
{
    int cnt[3] = { 0 };

    if (have_top && !a->intra[xb4]) {
        if (a->ref[0][xb4] >= 4) cnt[a->ref[0][xb4] - 4]++;
        if (a->comp_type[xb4] && a->ref[1][xb4] >= 4) cnt[a->ref[1][xb4] - 4]++;
    }

    if (have_left && !l->intra[yb4]) {
        if (l->ref[0][yb4] >= 4) cnt[l->ref[0][yb4] - 4]++;
        if (l->comp_type[yb4] && l->ref[1][yb4] >= 4) cnt[l->ref[1][yb4] - 4]++;
    }

    return cnt[0] == cnt[1] ? 1 : cnt[0] < cnt[1] ? 0 : 2;
}

static inline int av1_get_uni_p1_ctx(const BlockContext *const a,
                                     const BlockContext *const l,
                                     const int yb4, const int xb4,
                                     const int have_top, const int have_left)
{
    int cnt[3] = { 0 };

    if (have_top && !a->intra[xb4]) {
        if (a->ref[0][xb4] - 1U < 3) cnt[a->ref[0][xb4] - 1]++;
        if (a->comp_type[xb4] && a->ref[1][xb4] - 1U < 3) cnt[a->ref[1][xb4] - 1]++;
    }

    if (have_left && !l->intra[yb4]) {
        if (l->ref[0][yb4] - 1U < 3) cnt[l->ref[0][yb4] - 1]++;
        if (l->comp_type[yb4] && l->ref[1][yb4] - 1U < 3) cnt[l->ref[1][yb4] - 1]++;
    }

    cnt[1] += cnt[2];

    return cnt[0] == cnt[1] ? 1 : cnt[0] < cnt[1] ? 0 : 2;
}

static inline int get_drl_context(const refmvs_candidate *const ref_mv_stack,
                                  const int ref_idx)
{
    if (ref_mv_stack[ref_idx].weight >= 640)
        return ref_mv_stack[ref_idx + 1].weight < 640;

    return ref_mv_stack[ref_idx + 1].weight < 640 ? 2 : 0;
}

static inline unsigned get_cur_frame_segid(const int by, const int bx,
                                           const int have_top,
                                           const int have_left,
                                           int *const seg_ctx,
                                           const uint8_t *cur_seg_map,
                                           const ptrdiff_t stride)
{
    cur_seg_map += bx + by * stride;
    if (have_left && have_top) {
        const int l = cur_seg_map[-1];
        const int a = cur_seg_map[-stride];
        const int al = cur_seg_map[-(stride + 1)];

        if (l == a && al == l) *seg_ctx = 2;
        else if (l == a || al == l || a == al) *seg_ctx = 1;
        else *seg_ctx = 0;
        return a == al ? a : l;
    } else {
        *seg_ctx = 0;
        return have_left ? cur_seg_map[-1] : have_top ? cur_seg_map[-stride] : 0;
    }
}

static inline void fix_int_mv_precision(mv *const mv) {
    mv->x = (mv->x - (mv->x >> 15) + 3) & ~7U;
    mv->y = (mv->y - (mv->y >> 15) + 3) & ~7U;
}

static inline void fix_mv_precision(const Dav1dFrameHeader *const hdr,
                                    mv *const mv)
{
    if (hdr->force_integer_mv) {
        fix_int_mv_precision(mv);
    } else if (hdr->mv_precision < 3) {
        mv->x = (mv->x - (mv->x >> 15)) & ~1U;
        mv->y = (mv->y - (mv->y >> 15)) & ~1U;
    }
}

// mv_prec=0..6 for {8,4,2,f,h,q,e}pel
static inline void mv_reduce_prec(mv *const mv, const int mv_prec) {
    if (mv_prec == 6) return;
    const int rnd = 32 >> mv_prec;
    mv->x = mv->x + rnd - (mv->x > 0);
    mv->y = mv->y + rnd - (mv->y > 0);
    const unsigned mask = ~(rnd * 2U - 1);
    mv->x &= mask;
    mv->y &= mask;
}

static inline mv get_gmv_2d(const Dav1dWarpedMotionParams *const gmv,
                            const int bx4, const int by4,
                            const int bw4, const int bh4,
                            const Dav1dFrameHeader *const hdr)
{
    switch (gmv->type) {
    case DAV1D_WM_TYPE_ROT_ZOOM:
        assert(gmv->matrix[5] ==  gmv->matrix[2]);
        assert(gmv->matrix[4] == -gmv->matrix[3]);
        // fall-through
    default:
    case DAV1D_WM_TYPE_AFFINE: {
        const int x = bx4 * 4 + bw4 * 2 - 1;
        const int y = by4 * 4 + bh4 * 2 - 1;
        const int xc = (gmv->matrix[2] - (1 << 16)) * x +
                       gmv->matrix[3] * y + gmv->matrix[0];
        const int yc = (gmv->matrix[5] - (1 << 16)) * y +
                       gmv->matrix[4] * x + gmv->matrix[1];
        const int shift = 16 - hdr->mv_precision;
        const int round = (1 << shift) >> 1;
        mv res = (mv) {
            .y = apply_sign(((abs(yc) + round) >> shift) << (3 - hdr->mv_precision), yc),
            .x = apply_sign(((abs(xc) + round) >> shift) << (3 - hdr->mv_precision), xc),
        };
        if (hdr->force_integer_mv)
            fix_int_mv_precision(&res);
        return res;
    }
    case DAV1D_WM_TYPE_TRANSLATION: {
        mv res = (mv) {
            .y = gmv->matrix[0] >> 13,
            .x = gmv->matrix[1] >> 13,
        };
        if (hdr->force_integer_mv)
            fix_int_mv_precision(&res);
        return res;
    }
    case DAV1D_WM_TYPE_IDENTITY:
        return (mv) { .x = 0, .y = 0 };
    }
}

#endif /* DAV1D_SRC_ENV_H */
