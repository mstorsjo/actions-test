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

#include <string.h>

#include "common/intops.h"

#include "src/ctx.h"
#include "src/levels.h"
#include "src/lf_mask.h"
#include "src/tables.h"

static ALWAYS_INLINE void mask_outer_edge_l(uint16_t (*const masks)[4],
                                            const int by4, const int h4,
                                            const int bwl4c, uint8_t *const l)
{
    assert((unsigned) bwl4c <= 3U);

    // left block edge
    uint64_t mask = 1ULL << by4;
    for (int y = 0; y < h4; y++, mask <<= 1) {
        const int sidx = (by4 + y) >> 4;
        const unsigned smask = (unsigned) (mask >> (sidx << 4));
        masks[imin(bwl4c, l[y])][sidx] |= smask;
    }
    dav2d_memset_likely_pow2(l, bwl4c, h4);
}

static ALWAYS_INLINE void mask_outer_edge_t(uint16_t (*const masks)[4],
                                            const int bx4, const int w4,
                                            const int bhl4c, uint8_t *const a)
{
    assert((unsigned) bhl4c <= 3U);

    // top block edge
    uint64_t mask = 1ULL << bx4;
    for (int x = 0; x < w4; x++, mask <<= 1) {
        const int sidx = (bx4 + x) >> 4;
        const unsigned smask = (unsigned) (mask >> (sidx << 4));
        masks[imin(bhl4c, a[x])][sidx] |= smask;
    }
    dav2d_memset_likely_pow2(a, bhl4c, w4);
}

static ALWAYS_INLINE void mask_inner_edges_v(uint16_t (*const masks)[64][5][4],
                                             const uint64_t inner,
                                             const int bx4, const int w4,
                                             const int twl4c, const int xoff,
                                             const int hstep)
{
    assert((unsigned) twl4c <= 3U);

    // inner (tx) left|right edges
    const unsigned inner1 = (unsigned) (inner & 0xffff);
    const unsigned inner2 = (unsigned) ((inner >> 16) & 0xffff);
    const unsigned inner3 = (unsigned) ((inner >> 32) & 0xffff);
    const unsigned inner4 = (unsigned) ((inner >> 48));
    for (int x = xoff; x < w4; x += hstep) {
        if (inner1) masks[0][bx4 + x][twl4c][0] |= inner1;
        if (inner2) masks[0][bx4 + x][twl4c][1] |= inner2;
        if (inner3) masks[0][bx4 + x][twl4c][2] |= inner3;
        if (inner4) masks[0][bx4 + x][twl4c][3] |= inner4;
    }
}

static ALWAYS_INLINE void mask_inner_edges_h(uint16_t (*const masks)[64][5][4],
                                             const uint64_t inner,
                                             const int by4, const int h4,
                                             const int thl4c, const int yoff,
                                             const int vstep)
{
    assert((unsigned) thl4c <= 3U);

    //            top
    // inner (tx) --- edges
    //           bottom
    const unsigned inner1 = (unsigned) (inner & 0xffff);
    const unsigned inner2 = (unsigned) ((inner >> 16) & 0xffff);
    const unsigned inner3 = (unsigned) ((inner >> 32) & 0xffff);
    const unsigned inner4 = (unsigned) ((inner >> 48));
    for (int y = yoff; y < h4; y += vstep) {
        if (inner1) masks[1][by4 + y][thl4c][0] |= inner1;
        if (inner2) masks[1][by4 + y][thl4c][1] |= inner2;
        if (inner3) masks[1][by4 + y][thl4c][2] |= inner3;
        if (inner4) masks[1][by4 + y][thl4c][3] |= inner4;
    }
}

static inline void mask_edges_part(uint16_t (*const masks)[64][5][4],
                                   const int by4, const int bx4,
                                   const int w4, const int h4,
                                   const enum TxPartition tx_part,
                                   const TxfmInfo *const t_dim,
                                   const int lim,
                                   uint8_t *const a, uint8_t *const l)
{
    const int tw4 = t_dim->w, th4 = t_dim->h;
    const int twl4c = imin(lim, t_dim->lw), thl4c = imin(lim, t_dim->lh);

    if (tx_part < TX_PARTITION_H5) {
        mask_outer_edge_l(masks[0][bx4], by4, h4, twl4c, l);
        mask_outer_edge_t(masks[1][by4], bx4, w4, thl4c, a);
        if (w4 > tw4) {
            const uint64_t inner = (~0ULL >> (64 - h4)) << by4;
            mask_inner_edges_v(masks, inner, bx4, w4, twl4c, tw4, tw4);
        }
        if (h4 > th4) {
            const uint64_t inner = (~0ULL >> (64 - w4)) << bx4;
            mask_inner_edges_h(masks, inner, by4, h4, thl4c, th4, th4);
        }
    } else if (tx_part == TX_PARTITION_H5) {
        assert(th4 * 4 >= h4 && tw4 * 2 >= w4);
        mask_outer_edge_t(masks[1][by4], bx4, w4, thl4c, a);
        mask_outer_edge_l(masks[0][bx4], by4, imin(th4, h4), twl4c, l);
        if (h4 > th4) {
            mask_outer_edge_l(masks[0][bx4], by4 + th4, imin(2 * th4, h4 - th4),
                              imin(twl4c + 1, lim), &l[th4]);
            if (h4 > th4 * 3)
                mask_outer_edge_l(masks[0][bx4], by4 + th4 * 3,
                                  imin(th4, h4 - 3 * th4), twl4c, &l[th4 * 3]);
        }
        const uint64_t inner = (~0ULL >> (64 - w4)) << bx4;
        mask_inner_edges_h(masks, inner, by4, h4, thl4c, th4, th4 * 2);
        const uint64_t inner_a = (~0ULL >> (64 - h4)) << by4;
        const uint64_t inner_b = (~0ULL >> (64 - th4 * 2)) << (by4 + th4);
        const uint64_t inner_c = inner_a & ~inner_b;
        mask_inner_edges_v(masks, inner_c, bx4, w4, twl4c, tw4, tw4);
    } else {
        assert(tx_part == TX_PARTITION_V5 && tw4 * 4 >= w4 && th4 * 2 >= h4);
        mask_outer_edge_l(masks[0][bx4], by4, h4, twl4c, l);
        mask_outer_edge_t(masks[1][by4], bx4, imin(tw4, w4), thl4c, a);
        if (w4 > tw4) {
            mask_outer_edge_t(masks[1][by4], bx4 + tw4, imin(2 * tw4, w4 - tw4),
                              imin(thl4c + 1, lim), &a[tw4]);
            if (w4 > tw4 * 3)
                mask_outer_edge_t(masks[1][by4], bx4 + tw4 * 3,
                                  imin(tw4, w4 - 3 * tw4), thl4c, &a[tw4 * 3]);
        }
        const uint64_t inner = (~0ULL >> (64 - h4)) << by4;
        mask_inner_edges_v(masks, inner, bx4, w4, twl4c, tw4, tw4 * 2);
        const uint64_t inner_a = (~0ULL >> (64 - w4)) << bx4;
        const uint64_t inner_b = (~0ULL >> (64 - tw4 * 2)) << (bx4 + tw4);
        const uint64_t inner_c = inner_a & ~inner_b;
        mask_inner_edges_h(masks, inner_c, by4, h4, thl4c, th4, th4);
    }
}

static ALWAYS_INLINE void mask_subpu_edges(uint16_t (*const masks)[64][5][4],
                                           const int by4, const int bx4,
                                           const int w4, const int h4,
                                           const int sz, const int twl4c,
                                           const int thl4c,
                                           const int ds_sub_pu_mask)
{
    assert(!(sz & (sz - 1)) && sz >= 1 && sz <= 8);
    assert((unsigned) thl4c <= 2U && (unsigned) twl4c <= 2U);
    assert(ds_sub_pu_mask == 15 || ds_sub_pu_mask == 0);

    // inner (subpu) left|right edges
    uint64_t inner = (~0ULL >> (64 - h4)) << by4;
    unsigned inner0 = (unsigned) (inner & 0xffff);
    unsigned inner1 = (unsigned) ((inner >> 16) & 0xffff);
    unsigned inner2 = (unsigned) ((inner >> 32) & 0xffff);
    unsigned inner3 = (unsigned) ((inner >> 48));
    for (int x = sz; x < w4; x += sz) {
#define mask_subpu(a, b, c, d, e) \
        if (inner##e) { \
            const unsigned m = masks[a][b + c][d][e]; \
            masks[a][b + c][d][e] |= inner##e; \
            if (c & ds_sub_pu_mask) \
                masks[a][b + c][4][e] |= inner##e & ~m; \
        }
        mask_subpu(0, bx4, x, twl4c, 0);
        mask_subpu(0, bx4, x, twl4c, 1);
        mask_subpu(0, bx4, x, twl4c, 2);
        mask_subpu(0, bx4, x, twl4c, 3);
    }

    //               top
    // inner (subpu) --- edges
    //              bottom
    inner = (~0ULL >> (64 - w4)) << bx4;
    inner0 = (unsigned) (inner & 0xffff);
    inner1 = (unsigned) ((inner >> 16) & 0xffff);
    inner2 = (unsigned) ((inner >> 32) & 0xffff);
    inner3 = (unsigned) ((inner >> 48));
    for (int y = sz; y < h4; y += sz) {
        mask_subpu(1, by4, y, thl4c, 0);
        mask_subpu(1, by4, y, thl4c, 1);
        mask_subpu(1, by4, y, thl4c, 2);
        mask_subpu(1, by4, y, thl4c, 3);
#undef mask_subpu
    }
}

void dav2d_create_lf_mask_luma(Av2Filter *const lflvl,
                               const Av2Block *const b,
                               const enum BlockSize lbs,
                               const int bx, const int by,
                               const int iw, const int ih,
                               uint8_t *const ay, uint8_t *const ly,
                               const Dav2dFrameHeader *const frame_hdr,
                               const Dav2dSequenceHeader *const seq_hdr)
{
    const uint8_t *const b_dim = dav2d_block_dimensions[lbs];
    const int bw4 = imin(iw - bx, b_dim[0]);
    const int bh4 = imin(ih - by, b_dim[1]);
    const int bx4 = bx & 63;
    const int by4 = by & 63;
    assert(bw4 > 0 && bh4 > 0);

    int subpu_sz = 0;
    if (b->intra || !frame_hdr->loopfilter.lf_sub_pu) {
        /* do nothing */
    } else if (b->ref.ref[0] == TIP_FRAME) {
        const int opfl = seq_hdr->tip_refine_mv &&
            (frame_hdr->tip.frame_mode == 1 ||
             frame_hdr->tip.subpel_filter == DAV2D_FILTER_8TAP_SHARP);
        subpu_sz = 2 << (frame_hdr->tip.frame_mode == 2 /* frame */ ? !opfl :
                         ((!opfl && imin(bw4, bh4) >= 4) || lbs == BS_256x256));
    } else if (b->ref.ref[1] != -1) {
        if (b->inter_mode >= OPFL_NEARMV_NEARMV) {
            subpu_sz = 2 - (lbs == BS_8x8);
        } else if (b->refine_mv && b->comp_type == COMP_INTER_AVG) {
            subpu_sz = 4;
        }
    }
    const int subpu_l2 = subpu_sz ? ulog2(subpu_sz) : 3;
    const int ds_subpu_mask = (frame_hdr->tip.frame_mode != 2) * 15;
    int twl4c, thl4c;

    if (b->intra || !b->skip_txfm) {
        const enum TxPartition tx_part = b->tx_part;
        const int8_t *const tp = dav2d_tx_part_tbl[lbs];
        const enum RectTxfmSize tx = tp[tx_part];
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        mask_edges_part(lflvl->filter_y, by4, bx4, bw4, bh4, b->tx_part, t_dim,
                        subpu_l2, ay, ly);
        twl4c = imin(subpu_l2, t_dim->lw);
        thl4c = imin(subpu_l2, t_dim->lh);
    } else {
        mask_outer_edge_l(lflvl->filter_y[0][bx4], by4, bh4,
                          imin(subpu_l2, b_dim[2]), ly);
        mask_outer_edge_t(lflvl->filter_y[1][by4], bx4, bw4,
                          imin(subpu_l2, b_dim[3]), ay);
        twl4c = thl4c = subpu_l2;
    }

    if (subpu_sz)
        mask_subpu_edges(lflvl->filter_y, by4, bx4, bw4, bh4, subpu_sz,
                         twl4c, thl4c, ds_subpu_mask);
}

void dav2d_create_lf_mask_chroma(Av2Filter *const lflvl,
                                 const Av2Block *const b,
                                 const enum BlockSize cbs,
                                 const int cbx, const int cby,
                                 const int iw, const int ih,
                                 const enum Dav2dPixelLayout layout,
                                 uint8_t *const auv, uint8_t *const luv,
                                 const Dav2dFrameHeader *const frame_hdr,
                                 const Dav2dSequenceHeader *const seq_hdr)
{
    const uint8_t *const cb_dim = dav2d_block_dimensions[cbs];
    const int ss_ver = layout == DAV2D_PIXEL_LAYOUT_I420;
    const int ss_hor = layout != DAV2D_PIXEL_LAYOUT_I444;
    const int cbw4 = imin(iw - cbx, cb_dim[0]) >> ss_hor;
    const int cbh4 = imin(ih - cby, cb_dim[1]) >> ss_ver;
    const int cbx4 = (cbx & 63) >> ss_hor;
    const int cby4 = (cby & 63) >> ss_ver;
    assert(cbw4 > 0 && cbh4 > 0);

    mask_outer_edge_l(lflvl->filter_uv[0][cbx4], cby4, cbh4,
                      imin(2, cb_dim[2] >> ss_hor), luv);
    mask_outer_edge_t(lflvl->filter_uv[1][cby4], cbx4, cbw4,
                      imin(2, cb_dim[3] >> ss_ver), auv);
    // FIXME tx edges (for 256xN/Nx256 where tx=64x64)
    // FIXME subpu edges
}
