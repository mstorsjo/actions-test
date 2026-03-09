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
                                   const int hlim, const int vlim,
                                   uint8_t *const a, uint8_t *const l)
{
    const int tw4 = t_dim->w, th4 = t_dim->h;
    const int twl4c = imin(hlim, t_dim->lw), thl4c = imin(vlim, t_dim->lh);

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
                              imin(twl4c + 1, hlim), &l[th4]);
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
                              imin(thl4c + 1, vlim), &a[tw4]);
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
                                           const int twl4c, const int thl4c,
                                           const int hsz, const int vsz,
                                           const int ds_sub_pu_mask)
{
    assert(!(hsz & (hsz - 1)) && hsz >= 0 && hsz <= 8);
    assert(!(vsz & (vsz - 1)) && vsz >= 0 && vsz <= 8);
    assert((unsigned) thl4c <= 2U && (unsigned) twl4c <= 2U);
    assert(ds_sub_pu_mask == 15 || ds_sub_pu_mask == 0);

    if (hsz) {
        // inner (subpu) left|right edges
        const uint64_t inner = (~0ULL >> (64 - h4)) << by4;
        const unsigned inner0 = (unsigned) (inner & 0xffff);
        const unsigned inner1 = (unsigned) ((inner >> 16) & 0xffff);
        const unsigned inner2 = (unsigned) ((inner >> 32) & 0xffff);
        const unsigned inner3 = (unsigned) ((inner >> 48));
        for (int x = hsz; x < w4; x += hsz) {
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
    }

    if (vsz) {
        //               top
        // inner (subpu) --- edges
        //              bottom
        const uint64_t inner = (~0ULL >> (64 - w4)) << bx4;
        const unsigned inner0 = (unsigned) (inner & 0xffff);
        const unsigned inner1 = (unsigned) ((inner >> 16) & 0xffff);
        const unsigned inner2 = (unsigned) ((inner >> 32) & 0xffff);
        const unsigned inner3 = (unsigned) ((inner >> 48));
        for (int y = vsz; y < h4; y += vsz) {
            mask_subpu(1, by4, y, thl4c, 0);
            mask_subpu(1, by4, y, thl4c, 1);
            mask_subpu(1, by4, y, thl4c, 2);
            mask_subpu(1, by4, y, thl4c, 3);
#undef mask_subpu
        }
    }
}

static int subpu_flt_lvl(const Dav2dSequenceHeader *const seq_hdr,
                         const Dav2dFrameHeader *const frame_hdr,
                         const enum BlockSize bs, const int bw4, const int bh4,
                         const Av2Block *const b, const int max_lvl)
{
    if (b->intra || !frame_hdr->deblock.sub_pu) {
        /* do nothing */
    } else if (b->ref.ref[0] == TIP_FRAME) {
        const int opfl = seq_hdr->tip_refine_mv &&
            (frame_hdr->tip.frame_mode == 1 ||
             frame_hdr->tip.subpel_filter == DAV2D_FILTER_8TAP_SHARP);
        return 1 + (frame_hdr->tip.frame_mode == 2 /* frame */ ? !opfl :
                    ((!opfl && imin(bw4, bh4) >= 4) || bs == BS_256x256));
    } else if (b->ref.ref[1] != -1) {
        if (b->inter_mode >= OPFL_NEARMV_NEARMV) {
            return 1 - (bs == BS_8x8);
        } else if (b->refine_mv && b->comp_type == COMP_INTER_AVG) {
            return 2;
        }
    }
    return max_lvl;
}

void dav2d_create_db_mask(uint16_t (*const masks)[64][5][4],
                          const Av2Block *const b,
                          const enum BlockSize bs,
                          const int bx, const int by,
                          const int iw, const int ih,
                          const enum Dav2dPixelLayout layout,
                          const int chroma,
                          uint8_t *const a, uint8_t *const l,
                          const Dav2dFrameHeader *const frame_hdr,
                          const Dav2dSequenceHeader *const seq_hdr)
{
    const int ss_ver = chroma && layout == DAV2D_PIXEL_LAYOUT_I420;
    const int ss_hor = chroma && layout != DAV2D_PIXEL_LAYOUT_I444;
    const uint8_t *const b_dim = dav2d_block_dimensions[bs];
    const int bw4 = imin(iw - bx, b_dim[0]) >> ss_hor;
    const int bh4 = imin(ih - by, b_dim[1]) >> ss_ver;
    const int bx4 = (bx & 63) >> ss_hor;
    const int by4 = (by & 63) >> ss_ver;
    assert(bw4 > 0 && bh4 > 0);

    const int subpu_l2 = subpu_flt_lvl(seq_hdr, frame_hdr,
                                       bs, b_dim[0], b_dim[1], b, 3);
    const int ds_subpu_mask = (frame_hdr->tip.frame_mode != 2) * 15;
    int twl4c, thl4c;

    if (b->intra || !b->skip_txfm) {
        const enum TxPartition tx_part = chroma ? TX_PARTITION_NONE : b->tx_part;
        const enum RectTxfmSize tx = chroma ?
            dav2d_max_txfm_size_for_bs[bs][DAV2D_PIXEL_LAYOUT_I444 - layout] :
            dav2d_tx_part_tbl[bs][tx_part];
        const TxfmInfo *const t_dim = &dav2d_txfm_dimensions[tx];
        mask_edges_part(masks, by4, bx4, bw4, bh4, tx_part, t_dim,
                        iclip(subpu_l2 - ss_hor, 0, 3 - chroma),
                        iclip(subpu_l2 - ss_ver, 0, 3 - chroma), a, l);
        twl4c = imin(subpu_l2, t_dim->lw);
        thl4c = imin(subpu_l2, t_dim->lh);
    } else {
        mask_outer_edge_l(masks[0][bx4], by4, bh4,
                          iclip(imin(subpu_l2, b_dim[2]) - ss_hor, 0, 3 - chroma), l);
        mask_outer_edge_t(masks[1][by4], bx4, bw4,
                          iclip(imin(subpu_l2, b_dim[3]) - ss_ver, 0, 3 - chroma), a);
        twl4c = thl4c = subpu_l2;
    }

    if (subpu_l2 != 3) {
        const int h_subpu_l2 = twl4c - (ss_hor && twl4c);
        const int v_subpu_l2 = thl4c - (ss_ver && thl4c);
        mask_subpu_edges(masks, by4, bx4, bw4, bh4, h_subpu_l2, v_subpu_l2,
                         (1 << subpu_l2) >> ss_hor, (1 << subpu_l2) >> ss_ver,
                         // this variable isn't subsampled for some reason
                         ds_subpu_mask);
    }
}
