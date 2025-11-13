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

struct SBEdgeCtx {
    uint8_t ref[2][64];
    uint8_t motion_mode[64];
};

static inline int get_intra_ctx(const BlockContext *nx[2],
                                const int xoff[2], const int n_ctx)
{
    if (!n_ctx) return 0;
    const int i = n_ctx - 1;
    const int sum = (nx[0]->intra[xoff[0]] && !nx[0]->intrabc[xoff[0]]) +
                    (nx[i]->intra[xoff[i]] && !nx[i]->intrabc[xoff[i]]);
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

static inline int get_filter_ctx(const BlockContext *nb[2],
                                 const int boff[2], const int8_t refs[2])
{
    const int ref = refs[0], comp = refs[1] != -1;
    const int flt0 = (boff[0] != -1 && (nb[0]->ref[0][boff[0]] == ref ||
                                        nb[0]->ref[1][boff[0]] == ref)) ?
                     nb[0]->filter[boff[0]] : DAV1D_N_SWITCHABLE_FILTERS;
    const int flt1 = (boff[1] != -1 && (nb[1]->ref[0][boff[1]] == ref ||
                                        nb[1]->ref[1][boff[1]] == ref)) ?
                     nb[1]->filter[boff[1]] : DAV1D_N_SWITCHABLE_FILTERS;

    if (flt0 == flt1 || flt1 == DAV1D_N_SWITCHABLE_FILTERS) {
        return comp * 4 + flt0;
    } else if (flt0 == DAV1D_N_SWITCHABLE_FILTERS) {
        return comp * 4 + flt1;
    } else {
        return comp * 4 + DAV1D_N_SWITCHABLE_FILTERS;
    }
}

static inline int get_comp_ctx(const BlockContext *nx[2],
                               const int xoff[2], const int n_ctx,
                               const int8_t *const refdir)
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
                return (refdir[refa1] == 1) ^ (refdir[refb1] == 1);
            } else return 2 + (!nx[0]->intrabc[xoff[0]] && refdir[refa1]);
        } else if (refb2 == -1) {
            const int refb1 = nx[1]->ref[0][xoff[1]];
            return 2 + (!nx[1]->intrabc[xoff[1]] && refdir[refb1]);
        } else return 4;
    }
    case 1: {
        const int ref2 = nx[0]->ref[1][xoff[0]];
        if (ref2 == -1) {
            const int ref1 = nx[0]->ref[0][xoff[0]];
            return !nx[0]->intrabc[xoff[0]] && refdir[ref1];
        } else return 3;
    }
    case 0: return 1;
    }
}

static inline int get_warp_ctx(const BlockContext *const a,
                               const struct SBEdgeCtx *const a_sb_cache,
                               const BlockContext *const l,
                               const int yb4, const int xb4,
                               const int have_top, const int have_left,
                               const int have_top_right, const int have_bottom_left,
                               const unsigned top_is_at_tile_boundary,
                               const uint8_t *const b_dim, const int ref)
{
    int ctx = 0;

#define add_matching(dir, idx) do { \
    ctx += (dir->ref[0][idx] == ref || dir->ref[1][idx] == ref) && \
           dir->motion_mode[idx] >= 2; \
} while (0)
    if (have_top) {
        if (top_is_at_tile_boundary) {
            add_matching(a_sb_cache, xb4 & ~1);
            if (have_top_right && b_dim[0] >= 4)
                add_matching(a_sb_cache, (xb4 + b_dim[0] - 2) & ~1);
        } else {
            add_matching(a, xb4);
            if (have_top_right)
                add_matching(a, xb4 + b_dim[0] - 1);
        }
    }
    if (have_left) {
        add_matching(l, yb4);
        if (have_bottom_left)
            add_matching(l, yb4 + b_dim[1] - 1);
    }
#undef add_matching

    return ctx;
}

static inline int get_snglref_ctx(const BlockContext *const a,
                                  const BlockContext *const l,
                                  const int yb4, const int xb4,
                                  const int have_top, const int have_left,
                                  const int have_top_right,
                                  const int have_bottom_left,
                                  const uint8_t *const b_dim, const int ref)
{
    int row = 0, col = 0, newmv = 0;

#define NEWMV0_MODE_MASK ((1 << NEWMV) | \
                          (1 << NEWMV_NEARMV) | \
                          (1 << NEWMV_NEWMV) | \
                          (1 << JOINT_NEWMV) | \
                          (1 << OPFL_NEWMV_NEARMV) | \
                          (1 << OPFL_NEWMV_NEWMV) | \
                          (1 << OPFL_JOINT_NEWMV))
    // the joint_newmv modes are missing in NEWMV1_MODE_MASK,
    // see compound_ref1_mode() in AVM
#define NEWMV1_MODE_MASK ((1 << NEARMV_NEWMV) | \
                          (1 << NEWMV_NEWMV) | \
                          (1 << OPFL_NEARMV_NEWMV) | \
                          (1 << OPFL_NEWMV_NEWMV))
#define add_matching(dir, cnt, idx) do { \
    if (dir->ref[0][idx] == ref) { \
        cnt++; \
        newmv += !!((1 << dir->mode[idx]) & NEWMV0_MODE_MASK); \
    } else if (dir->ref[1][idx] == ref) { \
        cnt++; \
        newmv += !!((1 << dir->mode[idx]) & NEWMV1_MODE_MASK); \
    } \
} while (0)
    if (have_top) {
        add_matching(a, col, xb4);
        if (have_top_right)
            add_matching(a, col, xb4 + b_dim[0] - 1);
    }
    if (have_left) {
        add_matching(l, row, yb4);
        if (have_bottom_left)
            add_matching(l, row, yb4 + b_dim[1] - 1);
    }
#undef NEWMV0_MODE_MASK
#undef NEWMV1_MODE_MASK
#undef add_matching

    return !!row + !!col + 2 * !!newmv;
}

static inline int get_compref_ctx(const BlockContext *const a,
                                  const BlockContext *const l,
                                  const int yb4, const int xb4,
                                  const int have_top, const int have_left,
                                  const int have_top_right,
                                  const int have_bottom_left,
                                  const uint8_t *const b_dim,
                                  const int8_t ref[2], const uint8_t tipref[2])
{
    int row = 0, col = 0, newmv = 0;

#define NEWMV_MODE_MASK ((1 << NEWMV) | \
                         (1 << NEARMV_NEWMV) | \
                         (1 << NEWMV_NEARMV) | \
                         (1 << NEWMV_NEWMV) | \
                         (1 << JOINT_NEWMV) | \
                         (1 << OPFL_NEARMV_NEWMV) | \
                         (1 << OPFL_NEWMV_NEARMV) | \
                         (1 << OPFL_NEWMV_NEWMV) | \
                         (1 << OPFL_JOINT_NEWMV))
#define add_matching(dir, cnt, idx) do { \
    if (dir->ref[0][idx] == TIP_FRAME && \
        tipref[0] == ref[0] && tipref[1] == ref[1]) \
    { \
        cnt++; \
        newmv += dir->mode[idx] == NEWMV; \
    } else if (dir->ref[0][idx] == ref[0] && dir->ref[1][idx] == ref[1]) { \
        cnt++; \
        newmv += !!((1 << dir->mode[idx]) & NEWMV_MODE_MASK); \
    } \
} while (0)
    if (have_top) {
        add_matching(a, col, xb4);
        if (have_top_right)
            add_matching(a, col, xb4 + b_dim[0] - 1);
    }
    if (have_left) {
        add_matching(l, row, yb4);
        if (have_bottom_left)
            add_matching(l, row, yb4 + b_dim[1] - 1);
    }
#undef NEWMV_MODE_MASK
#undef add_matching

    return !!row + !!col + 2 * !!newmv;
}

static inline int get_poc_diff(const int order_hint_n_bits,
                               const int poc0, const int poc1)
{
    if (!order_hint_n_bits) return 0;
    const int mask = 1 << (order_hint_n_bits - 1);
    const int diff = poc0 - poc1;
    return (diff & (mask - 1)) - (diff & mask);
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
