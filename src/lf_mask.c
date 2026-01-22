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

#include <string.h>

#include "common/intops.h"

#include "src/ctx.h"
#include "src/levels.h"
#include "src/lf_mask.h"
#include "src/tables.h"

static void decomp_tx(uint8_t (*const txa)[2 /* txsz, step */][32 /* y */][32 /* x */],
                      const enum RectTxfmSize from,
                      const int depth,
                      const int y_off, const int x_off,
                      const uint16_t *const tx_masks)
{
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[from];
    const int is_split = (from == (int) TX_4X4 || depth > 1) ? 0 :
        (tx_masks[depth] >> (y_off * 4 + x_off)) & 1;

    if (is_split) {
        const enum RectTxfmSize sub = t_dim->sub;
        const int htw4 = t_dim->w >> 1, hth4 = t_dim->h >> 1;

        decomp_tx(txa, sub, depth + 1, y_off * 2 + 0, x_off * 2 + 0, tx_masks);
        if (t_dim->w >= t_dim->h)
            decomp_tx((uint8_t(*)[2][32][32]) &txa[0][0][0][htw4],
                      sub, depth + 1, y_off * 2 + 0, x_off * 2 + 1, tx_masks);
        if (t_dim->h >= t_dim->w) {
            decomp_tx((uint8_t(*)[2][32][32]) &txa[0][0][hth4][0],
                      sub, depth + 1, y_off * 2 + 1, x_off * 2 + 0, tx_masks);
            if (t_dim->w >= t_dim->h)
                decomp_tx((uint8_t(*)[2][32][32]) &txa[0][0][hth4][htw4],
                          sub, depth + 1, y_off * 2 + 1, x_off * 2 + 1, tx_masks);
        }
    } else {
        const int lw = imin(2, t_dim->lw), lh = imin(2, t_dim->lh);

#define set_ctx(rep_macro) \
        for (int y = 0; y < t_dim->h; y++) { \
            rep_macro(txa[0][0][y], 0, lw); \
            rep_macro(txa[1][0][y], 0, lh); \
            txa[0][1][y][0] = t_dim->w; \
        }
        case_set_upto16(t_dim->lw);
#undef set_ctx
        dav1d_memset_pow2[t_dim->lw](txa[1][1][0], t_dim->h);
    }
}

static inline void mask_edges_inter(uint16_t (*const masks)[64][4][4],
                                    const int by4, const int bx4,
                                    const int w4, const int h4, const int skip,
                                    const enum RectTxfmSize max_tx,
                                    const uint16_t *const tx_masks,
                                    uint8_t *const a, uint8_t *const l)
{
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[max_tx];
    int y, x;

    ALIGN_STK_16(uint8_t, txa, 2 /* edge */, [2 /* txsz, step */][32 /* y */][32 /* x */]);
    for (int y_off = 0, y = 0; y < h4; y += t_dim->h, y_off++)
        for (int x_off = 0, x = 0; x < w4; x += t_dim->w, x_off++)
            decomp_tx((uint8_t(*)[2][32][32]) &txa[0][0][y][x],
                      max_tx, 0, y_off, x_off, tx_masks);

    // left block edge
    unsigned mask = 1U << by4;
    for (y = 0; y < h4; y++, mask <<= 1) {
        const int sidx = mask >= 0x10000;
        const unsigned smask = mask >> (sidx << 4);
        masks[0][bx4][imin(txa[0][0][y][0], l[y])][sidx] |= smask;
    }

    // top block edge
    for (x = 0, mask = 1U << bx4; x < w4; x++, mask <<= 1) {
        const int sidx = mask >= 0x10000;
        const unsigned smask = mask >> (sidx << 4);
        masks[1][by4][imin(txa[1][0][0][x], a[x])][sidx] |= smask;
    }

    if (!skip) {
        // inner (tx) left|right edges
        for (y = 0, mask = 1U << by4; y < h4; y++, mask <<= 1) {
            const int sidx = mask >= 0x10000U;
            const unsigned smask = mask >> (sidx << 4);
            int ltx = txa[0][0][y][0];
            int step = txa[0][1][y][0];
            for (x = step; x < w4; x += step) {
                const int rtx = txa[0][0][y][x];
                masks[0][bx4 + x][imin(rtx, ltx)][sidx] |= smask;
                ltx = rtx;
                step = txa[0][1][y][x];
            }
        }

        //            top
        // inner (tx) --- edges
        //           bottom
        for (x = 0, mask = 1U << bx4; x < w4; x++, mask <<= 1) {
            const int sidx = mask >= 0x10000U;
            const unsigned smask = mask >> (sidx << 4);
            int ttx = txa[1][0][0][x];
            int step = txa[1][1][0][x];
            for (y = step; y < h4; y += step) {
                const int btx = txa[1][0][y][x];
                masks[1][by4 + y][imin(ttx, btx)][sidx] |= smask;
                ttx = btx;
                step = txa[1][1][y][x];
            }
        }
    }

    for (y = 0; y < h4; y++)
        l[y] = txa[0][0][y][w4 - 1];
    memcpy(a, txa[1][0][h4 - 1], w4);
}

static inline void mask_edges(uint16_t (*const masks)[64][4][4],
                              const int by4, const int bx4,
                              const int w4, const int h4,
                              const int bwl4, const int bhl4,
                              uint8_t *const a, uint8_t *const l)
{
    const int bwl4c = imin(3, bwl4), bhl4c = imin(3, bhl4);
    int y, x;

    // left block edge
    uint64_t mask = 1ULL << by4;
    for (y = 0; y < h4; y++, mask <<= 1) {
        const int sidx = (by4 + y) >> 4;
        const unsigned smask = (unsigned) (mask >> (sidx << 4));
        masks[0][bx4][imin(bwl4c, l[y])][sidx] |= smask;
    }

    // top block edge
    for (x = 0, mask = 1ULL << bx4; x < w4; x++, mask <<= 1) {
        const int sidx = (bx4 + x) >> 4;
        const unsigned smask = (unsigned) (mask >> (sidx << 4));
        masks[1][by4][imin(bhl4c, a[x])][sidx] |= smask;
    }

    dav1d_memset_likely_pow2(a, bhl4c, w4);
    dav1d_memset_likely_pow2(l, bwl4c, h4);
}

static inline void mask_edges_part(uint16_t (*const masks)[64][4][4],
                                   const int by4, const int bx4,
                                   const int w4, const int h4,
                                   const enum RectTxfmSize tx,
                                   uint8_t *const a, uint8_t *const l)
{
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
    const int twl4 = t_dim->lw, thl4 = t_dim->lh;
    const int twl4c = imin(3, twl4), thl4c = imin(3, thl4);
    int y, x;

    // left block edge
    uint64_t mask = 1ULL << by4;
    for (y = 0; y < h4; y++, mask <<= 1) {
        const int sidx = (by4 + y) >> 4;
        const unsigned smask = (unsigned) (mask >> (sidx << 4));
        masks[0][bx4][imin(twl4c, l[y])][sidx] |= smask;
    }

    // top block edge
    for (x = 0, mask = 1ULL << bx4; x < w4; x++, mask <<= 1) {
        const int sidx = (bx4 + x) >> 4;
        const unsigned smask = (unsigned) (mask >> (sidx << 4));
        masks[1][by4][imin(thl4c, a[x])][sidx] |= smask;
    }

    // inner (tx) left|right edges
    const int hstep = t_dim->w;
    uint64_t inner = (~0ULL >> (64 - h4)) << by4;
    unsigned inner1 = inner & 0xffff;
    unsigned inner2 = (inner >> 16) & 0xffff;
    unsigned inner3 = (inner >> 32) & 0xffff;
    unsigned inner4 = (inner >> 48);
    for (x = hstep; x < w4; x += hstep) {
        if (inner1) masks[0][bx4 + x][twl4c][0] |= inner1;
        if (inner2) masks[0][bx4 + x][twl4c][1] |= inner2;
        if (inner3) masks[0][bx4 + x][twl4c][2] |= inner3;
        if (inner4) masks[0][bx4 + x][twl4c][3] |= inner4;
    }

    //            top
    // inner (tx) --- edges
    //           bottom
    const int vstep = t_dim->h;
    inner =(~0ULL >> (64 - w4)) << bx4;
    inner1 = inner & 0xffff;
    inner2 = (inner >> 16) & 0xffff;
    inner3 = (inner >> 32) & 0xffff;
    inner4 = (inner >> 48);
    for (y = vstep; y < h4; y += vstep) {
        if (inner1) masks[1][by4 + y][thl4c][0] |= inner1;
        if (inner2) masks[1][by4 + y][thl4c][1] |= inner2;
        if (inner3) masks[1][by4 + y][thl4c][2] |= inner3;
        if (inner4) masks[1][by4 + y][thl4c][3] |= inner4;
    }

    dav1d_memset_likely_pow2(a, thl4c, w4);
    dav1d_memset_likely_pow2(l, twl4c, h4);
}

static void mask_edges_chroma(uint16_t (*const masks)[64][2][4],
                              const int cby4, const int cbx4,
                              const int cw4, const int ch4,
                              const int skip_inter,
                              const enum RectTxfmSize tx,
                              uint8_t *const a, uint8_t *const l,
                              const int ss_hor, const int ss_ver)
{
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
    const int twl4 = t_dim->lw, thl4 = t_dim->lh;
    const int twl4c = !!twl4, thl4c = !!thl4;
    int y, x;
    const int vbits = 4 - ss_ver, hbits = 4 - ss_hor;
    const int vmask = 16 >> ss_ver, hmask = 16 >> ss_hor;
    const unsigned vmax = 1 << vmask, hmax = 1 << hmask;

    // left block edge
    unsigned mask = 1U << cby4;
    for (y = 0; y < ch4; y++, mask <<= 1) {
        const int sidx = mask >= vmax;
        const unsigned smask = mask >> (sidx << vbits);
        masks[0][cbx4][imin(twl4c, l[y])][sidx] |= smask;
    }

    // top block edge
    for (x = 0, mask = 1U << cbx4; x < cw4; x++, mask <<= 1) {
        const int sidx = mask >= hmax;
        const unsigned smask = mask >> (sidx << hbits);
        masks[1][cby4][imin(thl4c, a[x])][sidx] |= smask;
    }

    if (!skip_inter) {
        // inner (tx) left|right edges
        const int hstep = t_dim->w;
        unsigned t = 1U << cby4;
        unsigned inner = (unsigned) ((((uint64_t) t) << ch4) - t);
        unsigned inner1 = inner & ((1 << vmask) - 1), inner2 = inner >> vmask;
        for (x = hstep; x < cw4; x += hstep) {
            if (inner1) masks[0][cbx4 + x][twl4c][0] |= inner1;
            if (inner2) masks[0][cbx4 + x][twl4c][1] |= inner2;
        }

        //            top
        // inner (tx) --- edges
        //           bottom
        const int vstep = t_dim->h;
        t = 1U << cbx4;
        inner = (unsigned) ((((uint64_t) t) << cw4) - t);
        inner1 = inner & ((1 << hmask) - 1), inner2 = inner >> hmask;
        for (y = vstep; y < ch4; y += vstep) {
            if (inner1) masks[1][cby4 + y][thl4c][0] |= inner1;
            if (inner2) masks[1][cby4 + y][thl4c][1] |= inner2;
        }
    }

    dav1d_memset_likely_pow2(a, thl4c, cw4);
    dav1d_memset_likely_pow2(l, twl4c, ch4);
}

void dav1d_create_lf_mask_intra(Av1Filter *const lflvl,
                                const Av1Block *const b,
                                const int bx, const int by,
                                const int iw, const int ih,
                                const enum Dav1dPixelLayout layout,
                                uint8_t *ay, uint8_t *ly,
                                uint8_t *const auv, uint8_t *const luv)
{
    const enum BlockSize bs = b->bs;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = imin(iw - bx, b_dim[0]);
    const int bh4 = imin(ih - by, b_dim[1]);
    const int bx4 = bx & 63;
    const int by4 = by & 63;
    assert(bw4 >= 0 && bh4 >= 0);

    if (bw4 && bh4) {
        if (b->intra || !b->skip_txfm) {
            const enum TxPartition tx_part = b->tx_part;
            const int8_t *const tp = dav1d_tx_part_tbl[bs];
            const enum RectTxfmSize tx = tp[tx_part];
            if (tx_part < TX_PARTITION_H5) {
                mask_edges_part(lflvl->filter_y, by4, bx4, bw4, bh4, tx, ay, ly);
            } else if (tx_part == TX_PARTITION_H5) {
                const enum RectTxfmSize tx_big = tp[TX_PARTITION_H];
                const TxfmInfo *const t_dim_small = &dav1d_txfm_dimensions[tx],
                               *const t_dim_big = &dav1d_txfm_dimensions[tx_big];
                const int th4_small = t_dim_small->h;
                const int th4_big = t_dim_big->h;
                int cby4 = by4;
                int rem_h4 = bh4; // remaining height
                mask_edges_part(lflvl->filter_y, cby4, bx4, bw4, imin(rem_h4, th4_small), tx, ay, ly);

                rem_h4 -= th4_small;
                if (rem_h4 > 0) {
                    cby4 += th4_small;
                    ly += th4_small;
                    mask_edges_part(lflvl->filter_y, cby4, bx4, bw4, imin(rem_h4, th4_big), tx_big, ay, ly);
                    rem_h4 -= th4_big;
                    if (rem_h4 > 0) {
                        cby4 += th4_big;
                        ly += th4_big;
                        mask_edges_part(lflvl->filter_y, cby4, bx4, bw4, imin(rem_h4, th4_small), tx, ay, ly);
                    }
                }
            } else if (tx_part == TX_PARTITION_V5) {
                const enum RectTxfmSize tx_big = tp[TX_PARTITION_V];
                const TxfmInfo *const t_dim_small = &dav1d_txfm_dimensions[tx],
                               *const t_dim_big = &dav1d_txfm_dimensions[tx_big];
                const int tw4_small = t_dim_small->w;
                const int tw4_big = t_dim_big->w;
                int cbx4 = bx4;
                int rem_w4 = bw4; // remaining width
                mask_edges_part(lflvl->filter_y, by4, cbx4, imin(rem_w4, tw4_small), bh4, tx, ay, ly);
                rem_w4 -= tw4_small;
                if (rem_w4 > 0) {
                    cbx4 += tw4_small;
                    ay += tw4_small;
                    mask_edges_part(lflvl->filter_y, by4, cbx4, imin(rem_w4, tw4_big), bh4, tx_big, ay, ly);
                    rem_w4 -= tw4_big;
                    if (rem_w4 > 0) {
                        cbx4 += tw4_big;
                        ay += tw4_big;
                        mask_edges_part(lflvl->filter_y, by4, cbx4, imin(rem_w4, tw4_small), bh4, tx, ay, ly);
                    }
                }
            }
        } else {
            mask_edges(lflvl->filter_y, by4, bx4, bw4, bh4, b_dim[2], b_dim[3], ay, ly);
        }
    }

#if 0
    if (!auv) return;

    const int ss_ver = layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = layout != DAV1D_PIXEL_LAYOUT_I444;
    const int cbw4 = imin(((iw + ss_hor) >> ss_hor) - (bx >> ss_hor),
                          (b_dim[0] + ss_hor) >> ss_hor);
    const int cbh4 = imin(((ih + ss_ver) >> ss_ver) - (by >> ss_ver),
                          (b_dim[1] + ss_ver) >> ss_ver);
    assert(cbw4 >= 0 && cbh4 >= 0);

    if (!cbw4 || !cbh4) return;

    const int cbx4 = bx4 >> ss_hor;
    const int cby4 = by4 >> ss_ver;

    mask_edges_chroma(lflvl->filter_uv, cby4, cbx4, cbw4, cbh4, 0, uvtx,
                      auv, luv, ss_hor, ss_ver);
#endif
}

void dav1d_create_lf_mask_inter(Av1Filter *const lflvl,
                                const int bx, const int by,
                                const int iw, const int ih,
                                const int skip, const enum BlockSize bs,
                                const enum RectTxfmSize max_ytx,
                                const uint16_t *const tx_masks,
                                const enum RectTxfmSize uvtx,
                                const enum Dav1dPixelLayout layout,
                                uint8_t *const ay, uint8_t *const ly,
                                uint8_t *const auv, uint8_t *const luv)
{
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = imin(iw - bx, b_dim[0]);
    const int bh4 = imin(ih - by, b_dim[1]);
    const int bx4 = bx & 31;
    const int by4 = by & 31;
    assert(bw4 >= 0 && bh4 >= 0);

    if (bw4 && bh4) {
        mask_edges_inter(lflvl->filter_y, by4, bx4, bw4, bh4, skip,
                         max_ytx, tx_masks, ay, ly);
    }

    return;
    if (!auv) return;

    const int ss_ver = layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = layout != DAV1D_PIXEL_LAYOUT_I444;
    const int cbw4 = imin(((iw + ss_hor) >> ss_hor) - (bx >> ss_hor),
                          (b_dim[0] + ss_hor) >> ss_hor);
    const int cbh4 = imin(((ih + ss_ver) >> ss_ver) - (by >> ss_ver),
                          (b_dim[1] + ss_ver) >> ss_ver);
    assert(cbw4 >= 0 && cbh4 >= 0);

    if (!cbw4 || !cbh4) return;

    const int cbx4 = bx4 >> ss_hor;
    const int cby4 = by4 >> ss_ver;

    mask_edges_chroma(lflvl->filter_uv, cby4, cbx4, cbw4, cbh4, skip, uvtx,
                      auv, luv, ss_hor, ss_ver);
}
