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

#include <errno.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "dav1d/data.h"

#include "common/frame.h"
#include "common/intops.h"

#include "src/ctx.h"
#include "src/decode.h"
#include "src/dequant_tables.h"
#include "src/env.h"
#include "src/filmgrain.h"
#include "src/log.h"
#include "src/qm.h"
#include "src/recon.h"
#include "src/ref.h"
#include "src/tables.h"
#include "src/thread_task.h"
#include "src/warpmv.h"

static inline int dq_lookup(const int hbd, int qidx) {
    if (!qidx) return 64;
    qidx--;
    const int shift = qidx / 24;
    qidx %= 24;
    static const uint8_t dq_lookup_tbl[] = {
        40, 41, 43, 44, 45, 47, 48, 49, 51, 52, 54, 55,
        57, 59, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78,
    };
    return dq_lookup_tbl[qidx] << shift;
}

static void init_quant_tables(const Dav1dSequenceHeader *const seq_hdr,
                              const Dav1dFrameHeader *const frame_hdr,
                              const int qidx, uint16_t (*dq)[3][2])
{
    // and then ac == dc
    for (int i = 0; i < (frame_hdr->segmentation.enabled ? 8 : 1); i++) {
        const int yac = frame_hdr->segmentation.enabled ?
            iclip_u8(qidx + frame_hdr->segmentation.seg_data.d[i].delta_q) : qidx;
        const int ydc = iclip_u8(yac + frame_hdr->quant.ydc_delta);
        const int uac = iclip_u8(yac + frame_hdr->quant.uac_delta);
        const int udc = iclip_u8(yac + frame_hdr->quant.udc_delta);
        const int vac = iclip_u8(yac + frame_hdr->quant.vac_delta);
        const int vdc = iclip_u8(yac + frame_hdr->quant.vdc_delta);

        dq[i][0][0] = dq_lookup(seq_hdr->hbd, ydc);
        dq[i][0][1] = dq_lookup(seq_hdr->hbd, yac);
        dq[i][1][0] = dq_lookup(seq_hdr->hbd, udc);
        dq[i][1][1] = dq_lookup(seq_hdr->hbd, uac);
        dq[i][2][0] = dq_lookup(seq_hdr->hbd, vdc);
        dq[i][2][1] = dq_lookup(seq_hdr->hbd, vac);
    }
}

static int read_mv_component_diff(MsacContext *const msac,
                                  CdfMvComponent *const mv_comp,
                                  const int mv_prec)
{
    const int sign = dav1d_msac_decode_bool_adapt(msac, mv_comp->sign);
    const int cl = dav1d_msac_decode_symbol_adapt16(msac, mv_comp->classes, 10);
    int up, fp = 3, hp = 1;

    if (!cl) {
        up = dav1d_msac_decode_bool_adapt(msac, mv_comp->class0);
        if (mv_prec >= 0) {  // !force_integer_mv
            fp = dav1d_msac_decode_symbol_adapt4(msac, mv_comp->class0_fp[up], 3);
            if (mv_prec > 0) // allow_high_precision_mv
                hp = dav1d_msac_decode_bool_adapt(msac, mv_comp->class0_hp);
        }
    } else {
        up = 1 << cl;
        for (int n = 0; n < cl; n++)
            up |= dav1d_msac_decode_bool_adapt(msac, mv_comp->classN[n]) << n;
        if (mv_prec >= 0) {  // !force_integer_mv
            fp = dav1d_msac_decode_symbol_adapt4(msac, mv_comp->classN_fp, 3);
            if (mv_prec > 0) // allow_high_precision_mv
                hp = dav1d_msac_decode_bool_adapt(msac, mv_comp->classN_hp);
        }
    }

    const int diff = ((up << 3) | (fp << 1) | hp) + 1;

    return sign ? -diff : diff;
}

static void read_mv_residual(Dav1dTileState *const ts, mv *const ref_mv,
                             const int mv_prec)
{
    MsacContext *const msac = &ts->msac;
    const enum MVJoint mv_joint =
        dav1d_msac_decode_symbol_adapt4(msac, ts->cdf.mv.joint, N_MV_JOINTS - 1);
    if (mv_joint & MV_JOINT_V)
        ref_mv->y += read_mv_component_diff(msac, &ts->cdf.mv.comp[0], mv_prec);
    if (mv_joint & MV_JOINT_H)
        ref_mv->x += read_mv_component_diff(msac, &ts->cdf.mv.comp[1], mv_prec);
}

static void read_tx_tree(Dav1dTaskContext *const t,
                         const enum RectTxfmSize from,
                         const int depth, uint16_t *const masks,
                         const int x_off, const int y_off)
{
    const Dav1dFrameContext *const f = t->f;
    const int bx4 = t->bx & 31, by4 = t->by & 31;
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[from];
    const int txw = t_dim->lw, txh = t_dim->lh;
    int is_split;

    if (depth < 2 && from > (int) TX_4X4) {
        const int cat = 2 * (TX_64X64 - t_dim->max) - depth;
        const int a = t->a->tx[bx4] < txw;
        const int l = t->l.tx[by4] < txh;

        is_split = dav1d_msac_decode_bool_adapt(&t->ts->msac,
                       t->ts->cdf.m.txpart[cat][a + l]);
        if (is_split)
            masks[depth] |= 1 << (y_off * 4 + x_off);
    } else {
        is_split = 0;
    }

    if (is_split && t_dim->max > TX_8X8) {
        const enum RectTxfmSize sub = t_dim->sub;
        const TxfmInfo *const sub_t_dim = &dav1d_txfm_dimensions[sub];
        const int txsw = sub_t_dim->w, txsh = sub_t_dim->h;

        read_tx_tree(t, sub, depth + 1, masks, x_off * 2 + 0, y_off * 2 + 0);
        t->bx += txsw;
        if (txw >= txh && t->bx < f->bw)
            read_tx_tree(t, sub, depth + 1, masks, x_off * 2 + 1, y_off * 2 + 0);
        t->bx -= txsw;
        t->by += txsh;
        if (txh >= txw && t->by < f->bh) {
            read_tx_tree(t, sub, depth + 1, masks, x_off * 2 + 0, y_off * 2 + 1);
            t->bx += txsw;
            if (txw >= txh && t->bx < f->bw)
                read_tx_tree(t, sub, depth + 1, masks,
                             x_off * 2 + 1, y_off * 2 + 1);
            t->bx -= txsw;
        }
        t->by -= txsh;
    } else {
        dav1d_memset_pow2[t_dim->lw](&t->a->tx[bx4], is_split ? TX_4X4 : txw);
        dav1d_memset_pow2[t_dim->lh](&t->l.tx[by4], is_split ? TX_4X4 : txh);
    }
}

static int neg_deinterleave(int diff, int ref, int max) {
    if (!ref) return diff;
    if (ref >= (max - 1)) return max - diff - 1;
    if (2 * ref < max) {
        if (diff <= 2 * ref) {
            if (diff & 1)
                return ref + ((diff + 1) >> 1);
            else
                return ref - (diff >> 1);
        }
        return diff;
    } else {
        if (diff <= 2 * (max - ref - 1)) {
            if (diff & 1)
                return ref + ((diff + 1) >> 1);
            else
                return ref - (diff >> 1);
        }
        return max - (diff + 1);
    }
}

static void find_matching_ref(const Dav1dTaskContext *const t,
                              const enum EdgeFlags intra_edge_flags,
                              const int bw4, const int bh4,
                              const int w4, const int h4,
                              const int have_left, const int have_top,
                              const int ref, uint64_t masks[2])
{
    /*const*/ refmvs_block *const *r = &t->rt.r[(t->by & 31) + 5];
    int count = 0;
    int have_topleft = have_top && have_left;
    int have_topright = imax(bw4, bh4) < 32 &&
                        have_top && t->bx + bw4 < t->ts->tiling.col_end &&
                        (intra_edge_flags & EDGE_I444_TOP_HAS_RIGHT);

#define bs(rp) dav1d_block_dimensions[(rp)->bs]
#define matches(rp) ((rp)->ref.ref[0] == ref + 1 && (rp)->ref.ref[1] == -1)

    if (have_top) {
        const refmvs_block *r2 = &r[-1][t->bx];
        if (matches(r2)) {
            masks[0] |= 1;
            count = 1;
        }
        int aw4 = bs(r2)[0];
        if (aw4 >= bw4) {
            const int off = t->bx & (aw4 - 1);
            if (off) have_topleft = 0;
            if (aw4 - off > bw4) have_topright = 0;
        } else {
            unsigned mask = 1 << aw4;
            for (int x = aw4; x < w4; x += aw4) {
                r2 += aw4;
                if (matches(r2)) {
                    masks[0] |= mask;
                    if (++count >= 8) return;
                }
                aw4 = bs(r2)[0];
                mask <<= aw4;
            }
        }
    }
    if (have_left) {
        /*const*/ refmvs_block *const *r2 = r;
        if (matches(&r2[0][t->bx - 1])) {
            masks[1] |= 1;
            if (++count >= 8) return;
        }
        int lh4 = bs(&r2[0][t->bx - 1])[1];
        if (lh4 >= bh4) {
            if (t->by & (lh4 - 1)) have_topleft = 0;
        } else {
            unsigned mask = 1 << lh4;
            for (int y = lh4; y < h4; y += lh4) {
                r2 += lh4;
                if (matches(&r2[0][t->bx - 1])) {
                    masks[1] |= mask;
                    if (++count >= 8) return;
                }
                lh4 = bs(&r2[0][t->bx - 1])[1];
                mask <<= lh4;
            }
        }
    }
    if (have_topleft && matches(&r[-1][t->bx - 1])) {
        masks[1] |= 1ULL << 32;
        if (++count >= 8) return;
    }
    if (have_topright && matches(&r[-1][t->bx + bw4])) {
        masks[0] |= 1ULL << 32;
    }
#undef matches
}

static void derive_warpmv(const Dav1dTaskContext *const t,
                          const int bw4, const int bh4,
                          const uint64_t masks[2], const union mv mv,
                          Dav1dWarpedMotionParams *const wmp)
{
    int pts[8][2 /* in, out */][2 /* x, y */], np = 0;
    /*const*/ refmvs_block *const *r = &t->rt.r[(t->by & 31) + 5];

#define add_sample(dx, dy, sx, sy, rp) do { \
    pts[np][0][0] = 16 * (2 * dx + sx * bs(rp)[0]) - 8; \
    pts[np][0][1] = 16 * (2 * dy + sy * bs(rp)[1]) - 8; \
    pts[np][1][0] = pts[np][0][0] + (rp)->mv.mv[0].x; \
    pts[np][1][1] = pts[np][0][1] + (rp)->mv.mv[0].y; \
    np++; \
} while (0)

    // use masks[] to find the projectable motion vectors in the edges
    if ((unsigned) masks[0] == 1 && !(masks[1] >> 32)) {
        const int off = t->bx & (bs(&r[-1][t->bx])[0] - 1);
        add_sample(-off, 0, 1, -1, &r[-1][t->bx]);
    } else for (unsigned off = 0, xmask = (uint32_t) masks[0]; np < 8 && xmask;) { // top
        const int tz = ctz(xmask);
        off += tz;
        xmask >>= tz;
        add_sample(off, 0, 1, -1, &r[-1][t->bx + off]);
        xmask &= ~1;
    }
    if (np < 8 && masks[1] == 1) {
        const int off = t->by & (bs(&r[0][t->bx - 1])[1] - 1);
        add_sample(0, -off, -1, 1, &r[-off][t->bx - 1]);
    } else for (unsigned off = 0, ymask = (uint32_t) masks[1]; np < 8 && ymask;) { // left
        const int tz = ctz(ymask);
        off += tz;
        ymask >>= tz;
        add_sample(0, off, -1, 1, &r[off][t->bx - 1]);
        ymask &= ~1;
    }
    if (np < 8 && masks[1] >> 32) // top/left
        add_sample(0, 0, -1, -1, &r[-1][t->bx - 1]);
    if (np < 8 && masks[0] >> 32) // top/right
        add_sample(bw4, 0, 1, -1, &r[-1][t->bx + bw4]);
    assert(np > 0 && np <= 8);
#undef bs

    // select according to motion vector difference against a threshold
    int mvd[8], ret = 0;
    const int thresh = 4 * iclip(imax(bw4, bh4), 4, 28);
    for (int i = 0; i < np; i++) {
        mvd[i] = abs(pts[i][1][0] - pts[i][0][0] - mv.x) +
                 abs(pts[i][1][1] - pts[i][0][1] - mv.y);
        if (mvd[i] > thresh)
            mvd[i] = -1;
        else
            ret++;
    }
    if (!ret) {
        ret = 1;
    } else for (int i = 0, j = np - 1, k = 0; k < np - ret; k++, i++, j--) {
        while (mvd[i] != -1) i++;
        while (mvd[j] == -1) j--;
        assert(i != j);
        if (i > j) break;
        // replace the discarded samples;
        mvd[i] = mvd[j];
        memcpy(pts[i], pts[j], sizeof(*pts));
    }

    if (!dav1d_find_affine_int(pts, ret, bw4, bh4, mv, wmp, t->bx, t->by) &&
        !dav1d_get_shear_params(wmp))
    {
        wmp->type = DAV1D_WM_TYPE_AFFINE;
    } else
        wmp->type = DAV1D_WM_TYPE_IDENTITY;
}

static inline int findoddzero(const uint8_t *buf, int len) {
    for (int n = 0; n < len; n++)
        if (!buf[n * 2]) return 1;
    return 0;
}

// meant to be SIMD'able, so that theoretical complexity of this function
// times block size goes from w4*h4 to w4+h4-1
// a and b are previous two lines containing (a) top/left entries or (b)
// top/left entries, with a[0] being either the first top or first left entry,
// depending on top_offset being 1 or 0, and b being the first top/left entry
// for whichever has one. left_offset indicates whether the (len-1)th entry
// has a left neighbour.
// output is order[] and ctx for each member of this diagonal.
static void order_palette(const uint8_t *pal_idx, const ptrdiff_t stride,
                          const int i, const int first, const int last,
                          uint8_t (*const order)[8], uint8_t *const ctx)
{
    int have_top = i > first;

    assert(pal_idx);
    pal_idx += first + (i - first) * stride;
    for (int j = first, n = 0; j >= last; have_top = 1, j--, n++, pal_idx += stride - 1) {
        const int have_left = j > 0;

        assert(have_left || have_top);

#define add(v_in) do { \
        const int v = v_in; \
        assert((unsigned)v < 8U); \
        order[n][o_idx++] = v; \
        mask |= 1 << v; \
    } while (0)

        unsigned mask = 0;
        int o_idx = 0;
        if (!have_left) {
            ctx[n] = 0;
            add(pal_idx[-stride]);
        } else if (!have_top) {
            ctx[n] = 0;
            add(pal_idx[-1]);
        } else {
            const int l = pal_idx[-1], t = pal_idx[-stride], tl = pal_idx[-(stride + 1)];
            const int same_t_l = t == l;
            const int same_t_tl = t == tl;
            const int same_l_tl = l == tl;
            const int same_all = same_t_l & same_t_tl & same_l_tl;

            if (same_all) {
                ctx[n] = 4;
                add(t);
            } else if (same_t_l) {
                ctx[n] = 3;
                add(t);
                add(tl);
            } else if (same_t_tl | same_l_tl) {
                ctx[n] = 2;
                add(tl);
                add(same_t_tl ? l : t);
            } else {
                ctx[n] = 1;
                add(imin(t, l));
                add(imax(t, l));
                add(tl);
            }
        }
        for (unsigned m = 1, bit = 0; m < 0x100; m <<= 1, bit++)
            if (!(mask & m))
                order[n][o_idx++] = bit;
        assert(o_idx == 8);
#undef add
    }
}

static void read_pal_indices(Dav1dTaskContext *const t,
                             uint8_t *const pal_idx,
                             const int pal_sz, const int pl,
                             const int w4, const int h4,
                             const int bw4, const int bh4)
{
    Dav1dTileState *const ts = t->ts;
    const ptrdiff_t stride = bw4 * 4;
    assert(pal_idx);
    uint8_t *const pal_tmp = t->scratch.pal_idx_uv;
    pal_tmp[0] = dav1d_msac_decode_uniform(&ts->msac, pal_sz);
    uint16_t (*const color_map_cdf)[8] =
        ts->cdf.m.color_map[pl][pal_sz - 2];
    uint8_t (*const order)[8] = t->scratch.pal_order;
    uint8_t *const ctx = t->scratch.pal_ctx;
    for (int i = 1; i < 4 * (w4 + h4) - 1; i++) {
        // top/left-to-bottom/right diagonals ("wave-front")
        const int first = imin(i, w4 * 4 - 1);
        const int last = imax(0, i - h4 * 4 + 1);
        order_palette(pal_tmp, stride, i, first, last, order, ctx);
        for (int j = first, m = 0; j >= last; j--, m++) {
            const int color_idx = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                      color_map_cdf[ctx[m]], pal_sz - 1);
            pal_tmp[(i - j) * stride + j] = order[m][color_idx];
        }
    }

    t->c->pal_dsp.pal_idx_finish(pal_idx, pal_tmp, bw4 * 4, bh4 * 4,
                                 w4 * 4, h4 * 4);
}

static void read_vartx_tree(Dav1dTaskContext *const t,
                            Av1Block *const b, const enum BlockSize bs,
                            const int bx4, const int by4)
{
    const Dav1dFrameContext *const f = t->f;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];

    // var-tx tree coding
    uint16_t tx_split[2] = { 0 };
    b->max_ytx = dav1d_max_txfm_size_for_bs[bs][0];
    if (!b->skip_txfm && (f->frame_hdr->segmentation.lossless[b->seg_id] ||
                          b->max_ytx == TX_4X4))
    {
        b->max_ytx = b->uvtx = TX_4X4;
        if (f->frame_hdr->txfm_mode == DAV1D_TX_SWITCHABLE) {
            dav1d_memset_pow2[b_dim[2]](&t->a->tx[bx4], TX_4X4);
            dav1d_memset_pow2[b_dim[3]](&t->l.tx[by4], TX_4X4);
        }
    } else if (f->frame_hdr->txfm_mode != DAV1D_TX_SWITCHABLE || b->skip_txfm) {
        if (f->frame_hdr->txfm_mode == DAV1D_TX_SWITCHABLE) {
            dav1d_memset_pow2[b_dim[2]](&t->a->tx[bx4], b_dim[2 + 0]);
            dav1d_memset_pow2[b_dim[3]](&t->l.tx[by4], b_dim[2 + 1]);
        }
        b->uvtx = dav1d_max_txfm_size_for_bs[bs][f->cur.p.layout];
    } else {
        assert(bw4 <= 16 || bh4 <= 16 || b->max_ytx == TX_64X64);
        int y, x, y_off, x_off;
        const TxfmInfo *const ytx = &dav1d_txfm_dimensions[b->max_ytx];
        for (y = 0, y_off = 0; y < bh4; y += ytx->h, y_off++) {
            for (x = 0, x_off = 0; x < bw4; x += ytx->w, x_off++) {
                read_tx_tree(t, b->max_ytx, 0, tx_split, x_off, y_off);
                // contexts are updated inside read_tx_tree()
                t->bx += ytx->w;
            }
            t->bx -= x;
            t->by += ytx->h;
        }
        t->by -= y;
        if (DEBUG_BLOCK_INFO)
            printf("Post-vartxtree[%x/%x]: r=%d\n",
                   tx_split[0], tx_split[1], t->ts->msac.rng);
        b->uvtx = dav1d_max_txfm_size_for_bs[bs][f->cur.p.layout];
    }
    assert(!(tx_split[0] & ~0x33));
    b->tx_split0 = (uint8_t)tx_split[0];
    b->tx_split1 = tx_split[1];
}

static inline unsigned get_prev_frame_segid(const Dav1dFrameContext *const f,
                                            const int by, const int bx,
                                            const int w4, int h4,
                                            const uint8_t *ref_seg_map,
                                            const ptrdiff_t stride)
{
    assert(f->frame_hdr->primary_ref_frame != DAV1D_PRIMARY_REF_NONE);

    unsigned seg_id = 8;
    ref_seg_map += by * stride + bx;
    do {
        for (int x = 0; x < w4; x++)
            seg_id = imin(seg_id, ref_seg_map[x]);
        ref_seg_map += stride;
    } while (--h4 > 0 && seg_id);
    assert(seg_id < 8);

    return seg_id;
}

static inline void splat_oneref_mv(const Dav1dContext *const c,
                                   Dav1dTaskContext *const t,
                                   const enum BlockSize bs,
                                   const Av1Block *const b,
                                   const int bw4, const int bh4)
{
    const enum InterPredMode mode = b->inter_mode;
    const refmvs_block ALIGN(tmpl, 16) = (refmvs_block) {
        .ref.ref = { b->ref[0] + 1, b->interintra_type ? 0 : -1 },
        .mv.mv[0] = b->mv[0],
        .bs = bs,
        .mf = (mode == GLOBALMV && imin(bw4, bh4) >= 2) | ((mode == NEWMV) * 2),
    };
    c->refmvs_dsp.splat_mv(&t->rt.r[(t->by & 31) + 5], &tmpl, t->bx, bw4, bh4);
}

static inline void splat_intrabc_mv(const Dav1dContext *const c,
                                    Dav1dTaskContext *const t,
                                    const enum BlockSize bs,
                                    const Av1Block *const b,
                                    const int bw4, const int bh4)
{
    const refmvs_block ALIGN(tmpl, 16) = (refmvs_block) {
        .ref.ref = { 0, -1 },
        .mv.mv[0] = b->mv[0],
        .bs = bs,
        .mf = 0,
    };
    c->refmvs_dsp.splat_mv(&t->rt.r[(t->by & 31) + 5], &tmpl, t->bx, bw4, bh4);
}

static inline void splat_tworef_mv(const Dav1dContext *const c,
                                   Dav1dTaskContext *const t,
                                   const enum BlockSize bs,
                                   const Av1Block *const b,
                                   const int bw4, const int bh4)
{
    assert(bw4 >= 2 && bh4 >= 2);
    const enum CompInterPredMode mode = b->inter_mode;
    const refmvs_block ALIGN(tmpl, 16) = (refmvs_block) {
        .ref.ref = { b->ref[0] + 1, b->ref[1] + 1 },
        .mv.mv = { b->mv[0], b->mv[1] },
        .bs = bs,
        .mf = (mode == GLOBALMV_GLOBALMV) | !!((1 << mode) & (0xbc)) * 2,
    };
    c->refmvs_dsp.splat_mv(&t->rt.r[(t->by & 31) + 5], &tmpl, t->bx, bw4, bh4);
}

static inline void splat_intraref(const Dav1dContext *const c,
                                  Dav1dTaskContext *const t,
                                  const enum BlockSize bs,
                                  const int bw4, const int bh4)
{
    const refmvs_block ALIGN(tmpl, 16) = (refmvs_block) {
        .ref.ref = { 0, -1 },
        .mv.mv[0].n = INVALID_MV,
        .bs = bs,
        .mf = 0,
    };
    c->refmvs_dsp.splat_mv(&t->rt.r[(t->by & 31) + 5], &tmpl, t->bx, bw4, bh4);
}

static void mc_lowest_px(int *const dst, const int by4, const int bh4,
                         const int mvy, const int ss_ver,
                         const struct ScalableMotionParams *const smp)
{
    const int v_mul = 4 >> ss_ver;
    if (!smp->scale) {
        const int my = mvy >> (3 + ss_ver), dy = mvy & (15 >> !ss_ver);
        *dst = imax(*dst, (by4 + bh4) * v_mul + my + 4 * !!dy);
    } else {
        int y = (by4 * v_mul << 4) + mvy * (1 << !ss_ver);
        const int64_t tmp = (int64_t)(y) * smp->scale + (smp->scale - 0x4000) * 8;
        y = apply_sign64((int)((llabs(tmp) + 128) >> 8), tmp) + 32;
        const int bottom = ((y + (bh4 * v_mul - 1) * smp->step) >> 10) + 1 + 4;
        *dst = imax(*dst, bottom);
    }
}

static ALWAYS_INLINE void affine_lowest_px(Dav1dTaskContext *const t, int *const dst,
                                           const uint8_t *const b_dim,
                                           const Dav1dWarpedMotionParams *const wmp,
                                           const int ss_ver, const int ss_hor)
{
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;
    assert(!((b_dim[0] * h_mul) & 7) && !((b_dim[1] * v_mul) & 7));
    const int32_t *const mat = wmp->matrix;
    const int y = b_dim[1] * v_mul - 8; // lowest y

    const int src_y = t->by * 4 + ((y + 4) << ss_ver);
    const int64_t mat5_y = (int64_t) mat[5] * src_y + mat[1];
    // check left- and right-most blocks
    for (int x = 0; x < b_dim[0] * h_mul; x += imax(8, b_dim[0] * h_mul - 8)) {
        // calculate transformation relative to center of 8x8 block in
        // luma pixel units
        const int src_x = t->bx * 4 + ((x + 4) << ss_hor);
        const int64_t mvy = ((int64_t) mat[4] * src_x + mat5_y) >> ss_ver;
        const int dy = (int) (mvy >> 16) - 4;
        *dst = imax(*dst, dy + 4 + 8);
    }
}

static NOINLINE void affine_lowest_px_luma(Dav1dTaskContext *const t, int *const dst,
                                           const uint8_t *const b_dim,
                                           const Dav1dWarpedMotionParams *const wmp)
{
    affine_lowest_px(t, dst, b_dim, wmp, 0, 0);
}

static NOINLINE void affine_lowest_px_chroma(Dav1dTaskContext *const t, int *const dst,
                                             const uint8_t *const b_dim,
                                             const Dav1dWarpedMotionParams *const wmp)
{
    const Dav1dFrameContext *const f = t->f;
    assert(f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400);
    if (f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I444)
        affine_lowest_px_luma(t, dst, b_dim, wmp);
    else
        affine_lowest_px(t, dst, b_dim, wmp, f->cur.p.layout & DAV1D_PIXEL_LAYOUT_I420, 1);
}

static void obmc_lowest_px(Dav1dTaskContext *const t,
                           int (*const dst)[2], const int is_chroma,
                           const uint8_t *const b_dim,
                           const int bx4, const int by4, const int w4, const int h4)
{
    assert(!(t->bx & 1) && !(t->by & 1));
    const Dav1dFrameContext *const f = t->f;
    /*const*/ refmvs_block **r = &t->rt.r[(t->by & 31) + 5];
    const int ss_ver = is_chroma && f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = is_chroma && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const int h_mul = 4 >> ss_hor, v_mul = 4 >> ss_ver;

    if (t->by > t->ts->tiling.row_start &&
        (!is_chroma || b_dim[0] * h_mul + b_dim[1] * v_mul >= 16))
    {
        for (int i = 0, x = 0; x < w4 && i < imin(b_dim[2], 4); ) {
            // only odd blocks are considered for overlap handling, hence +1
            const refmvs_block *const a_r = &r[-1][t->bx + x + 1];
            const uint8_t *const a_b_dim = dav1d_block_dimensions[a_r->bs];

            if (a_r->ref.ref[0] > 0) {
                const int oh4 = imin(b_dim[1], 16) >> 1;
                mc_lowest_px(&dst[a_r->ref.ref[0] - 1][is_chroma], t->by,
                             (oh4 * 3 + 3) >> 2, a_r->mv.mv[0].y, ss_ver,
                             &f->svc[a_r->ref.ref[0] - 1][1]);
                i++;
            }
            x += imax(a_b_dim[0], 2);
        }
    }

    if (t->bx > t->ts->tiling.col_start)
        for (int i = 0, y = 0; y < h4 && i < imin(b_dim[3], 4); ) {
            // only odd blocks are considered for overlap handling, hence +1
            const refmvs_block *const l_r = &r[y + 1][t->bx - 1];
            const uint8_t *const l_b_dim = dav1d_block_dimensions[l_r->bs];

            if (l_r->ref.ref[0] > 0) {
                const int oh4 = iclip(l_b_dim[1], 2, b_dim[1]);
                mc_lowest_px(&dst[l_r->ref.ref[0] - 1][is_chroma],
                             t->by + y, oh4, l_r->mv.mv[0].y, ss_ver,
                             &f->svc[l_r->ref.ref[0] - 1][1]);
                i++;
            }
            y += imax(l_b_dim[1], 2);
        }
}

static int decode_b(Dav1dTaskContext *const t, DB_ONLY(const int depth)
                    const enum BlockSize lbs, const enum BlockSize cbs)
{
    const enum BlockSize bs = lbs == BS_INVALID ? cbs : lbs;
    assert(bs != BS_INVALID);
    Dav1dTileState *const ts = t->ts;
    const Dav1dFrameContext *const f = t->f;
    Av1Block b_mem, *const b = t->frame_thread.pass ?
        &f->frame_thread.b[t->by * f->b4_stride + t->bx] : &b_mem;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bx4 = t->bx & 31, by4 = t->by & 31;
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
    const int have_left = t->bx > ts->tiling.col_start;
    const int have_top = t->by > ts->tiling.row_start;
    const int has_luma = lbs != BS_INVALID, has_chroma = cbs != BS_INVALID;
    int ss_hor, ss_ver, cbx4, cby4, cbw4, cbh4, cw4, ch4;
    if (has_chroma) {
        ss_ver = f->ss_ver;
        ss_hor = f->ss_hor;
        cbx4 = (t->cbx & 31) >> ss_hor;
        cby4 = (t->cby & 31) >> ss_ver;
        const uint8_t *const cb_dim = dav1d_block_dimensions[cbs];
        cbw4 = cb_dim[0] >> ss_hor;
        cbh4 = cb_dim[1] >> ss_ver;
        assert(cbw4 >= 1 && cbh4 >= 1);
        cw4 = imin(cbw4, (f->bw - t->cbx) >> ss_hor);
        ch4 = imin(cbh4, (f->bh - t->cby) >> ss_ver);
        assert(cw4 >= 1 && ch4 >= 1);
    }

    DEBUG_BLOCK_printf("%*sdecode_b[y=%d,x=%d,bs=%dx%d,plane=%s]: r=%d\n",
                       depth - 1, "", t->by, t->bx, bw4 * 4, bh4 * 4,
                       !has_chroma ? "y" : !has_luma ? "uv" : "yuv",
                       ts->msac.rng);

    if (t->frame_thread.pass == 2) {
        if (b->intra) {
            f->bd_fn.recon_b_intra(t, DB_ONLY(depth) lbs, cbs, 0, b);

            const enum IntraPredMode y_mode_nofilt =
                b->y_mode == FILTER_PRED ? DC_PRED : b->y_mode;
#define set_ctx(rep_macro) \
            rep_macro(edge->mode, off, y_mode_nofilt); \
            rep_macro(edge->intra, off, 1)
            BlockContext *edge = t->a;
            for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
                case_set(b_dim[2 + i]);
            }
#undef set_ctx
            if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
                refmvs_block *const r = &t->rt.r[(t->by & 31) + 5 + bh4 - 1][t->bx];
                for (int x = 0; x < bw4; x++) {
                    r[x].ref.ref[0] = 0;
                    r[x].bs = bs;
                }
                refmvs_block *const *rr = &t->rt.r[(t->by & 31) + 5];
                for (int y = 0; y < bh4 - 1; y++) {
                    rr[y][t->bx + bw4 - 1].ref.ref[0] = 0;
                    rr[y][t->bx + bw4 - 1].bs = bs;
                }
            }

            if (has_chroma) {
                uint8_t uv_mode = b->uv_mode;
                dav1d_memset_pow2[ulog2(cbw4)](&t->a->uvmode[cbx4], uv_mode);
                dav1d_memset_pow2[ulog2(cbh4)](&t->l.uvmode[cby4], uv_mode);
            }
        } else {
            if (IS_INTER_OR_SWITCH(f->frame_hdr) /* not intrabc */ &&
                b->comp_type == COMP_INTER_NONE && b->motion_mode == MM_WARP)
            {
                if (b->matrix[0] == INT16_MIN) {
                    t->warpmv.type = DAV1D_WM_TYPE_IDENTITY;
                } else {
                    t->warpmv.type = DAV1D_WM_TYPE_AFFINE;
                    t->warpmv.matrix[2] = b->matrix[0] + 0x10000;
                    t->warpmv.matrix[3] = b->matrix[1];
                    t->warpmv.matrix[4] = b->matrix[2];
                    t->warpmv.matrix[5] = b->matrix[3] + 0x10000;
                    dav1d_set_affine_mv2d(bw4, bh4, b->mv2d, &t->warpmv,
                                          t->bx, t->by);
                    dav1d_get_shear_params(&t->warpmv);
#define signabs(v) v < 0 ? '-' : ' ', abs(v)
                    if (DEBUG_BLOCK_INFO)
                        printf("[ %c%x %c%x %c%x\n  %c%x %c%x %c%x ]\n"
                               "alpha=%c%x, beta=%c%x, gamma=%c%x, delta=%c%x, mv=y:%d,x:%d\n",
                               signabs(t->warpmv.matrix[0]),
                               signabs(t->warpmv.matrix[1]),
                               signabs(t->warpmv.matrix[2]),
                               signabs(t->warpmv.matrix[3]),
                               signabs(t->warpmv.matrix[4]),
                               signabs(t->warpmv.matrix[5]),
                               signabs(t->warpmv.u.p.alpha),
                               signabs(t->warpmv.u.p.beta),
                               signabs(t->warpmv.u.p.gamma),
                               signabs(t->warpmv.u.p.delta),
                               b->mv2d.y, b->mv2d.x);
#undef signabs
                }
            }
            if (f->bd_fn.recon_b_inter(t, bs, b)) return -1;

            const uint8_t *const filter = dav1d_filter_dir[b->filter2d];
            BlockContext *edge = t->a;
            for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
#define set_ctx(rep_macro) \
                rep_macro(edge->filter[0], off, filter[0]); \
                rep_macro(edge->filter[1], off, filter[1]); \
                rep_macro(edge->intra, off, 0)
                case_set(b_dim[2 + i]);
#undef set_ctx
            }

            if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
                refmvs_block *const r = &t->rt.r[(t->by & 31) + 5 + bh4 - 1][t->bx];
                for (int x = 0; x < bw4; x++) {
                    r[x].ref.ref[0] = b->ref[0] + 1;
                    r[x].mv.mv[0] = b->mv[0];
                    r[x].bs = bs;
                }
                refmvs_block *const *rr = &t->rt.r[(t->by & 31) + 5];
                for (int y = 0; y < bh4 - 1; y++) {
                    rr[y][t->bx + bw4 - 1].ref.ref[0] = b->ref[0] + 1;
                    rr[y][t->bx + bw4 - 1].mv.mv[0] = b->mv[0];
                    rr[y][t->bx + bw4 - 1].bs = bs;
                }
            }

            if (has_chroma) {
                dav1d_memset_pow2[ulog2(cbw4)](&t->a->uvmode[cbx4], DC_PRED);
                dav1d_memset_pow2[ulog2(cbh4)](&t->l.uvmode[cby4], DC_PRED);
            }
        }
        return 0;
    }

    b->bs = bs;

    const Dav1dSegmentationData *seg = NULL;

    // segment_id (if seg_feature for skip/ref/gmv is enabled)
    int seg_pred = 0;
    if (f->frame_hdr->segmentation.enabled) {
        if (!f->frame_hdr->segmentation.update_map) {
            if (f->prev_segmap) {
                unsigned seg_id = get_prev_frame_segid(f, t->by, t->bx, w4, h4,
                                                       f->prev_segmap,
                                                       f->b4_stride);
                if (seg_id >= 8) return -1;
                b->seg_id = seg_id;
            } else {
                b->seg_id = 0;
            }
            seg = &f->frame_hdr->segmentation.seg_data.d[b->seg_id];
        } else if (f->frame_hdr->segmentation.seg_data.preskip) {
            if (f->frame_hdr->segmentation.temporal &&
                (seg_pred = dav1d_msac_decode_bool_adapt(&ts->msac,
                                ts->cdf.m.seg_pred[t->a->seg_pred[bx4] +
                                t->l.seg_pred[by4]])))
            {
                // temporal predicted seg_id
                if (f->prev_segmap) {
                    unsigned seg_id = get_prev_frame_segid(f, t->by, t->bx,
                                                           w4, h4,
                                                           f->prev_segmap,
                                                           f->b4_stride);
                    if (seg_id >= 8) return -1;
                    b->seg_id = seg_id;
                } else {
                    b->seg_id = 0;
                }
            } else {
                int seg_ctx;
                const unsigned pred_seg_id =
                    get_cur_frame_segid(t->by, t->bx, have_top, have_left,
                                        &seg_ctx, f->cur_segmap, f->b4_stride);
                const unsigned diff = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                          ts->cdf.m.seg_id[seg_ctx],
                                          DAV1D_MAX_SEGMENTS - 1);
                const unsigned last_active_seg_id =
                    f->frame_hdr->segmentation.seg_data.last_active_segid;
                b->seg_id = neg_deinterleave(diff, pred_seg_id,
                                             last_active_seg_id + 1);
                if (b->seg_id > last_active_seg_id) b->seg_id = 0; // error?
                if (b->seg_id >= DAV1D_MAX_SEGMENTS) b->seg_id = 0; // error?
            }

            if (DEBUG_BLOCK_INFO)
                printf("Post-segid[preskip;%d]: r=%d\n",
                       b->seg_id, ts->msac.rng);

            seg = &f->frame_hdr->segmentation.seg_data.d[b->seg_id];
        }
    } else {
        b->seg_id = 0;
    }

    // cross-sb boundary neighbours
    const BlockContext *nx[2];
    int xoff[2], idx = 0;
    if (have_left && t->by + bh4 <= ts->tiling.row_end) {
        nx[0] = &t->l; xoff[0] = by4 + bh4 - 1; idx++;
    }
    if (have_top && t->bx + bw4 <= ts->tiling.col_end) {
        nx[idx] = t->a; xoff[idx] = bx4 + bw4 - 1; idx++;
    }
    if (idx < 2 && have_left) {
        nx[idx] = &t->l; xoff[idx] = by4; idx++;
    }
    if (idx < 2) {
        nx[idx] = t->a; xoff[idx] = bx4;
        if (!idx) {
            nx[idx + 1] = t->a; xoff[idx + 1] = bx4;
        }
    }

    // skip_mode
    if ((!seg || (!seg->globalmv && seg->ref == -1 && !seg->skip)) &&
        f->frame_hdr->skip_mode_enabled && imin(bw4, bh4) > 1)
    {
        const int smctx = t->a->skip_mode[bx4] + t->l.skip_mode[by4];
        b->skip_mode = dav1d_msac_decode_bool_adapt(&ts->msac,
                           ts->cdf.m.skip_mode[smctx]);
        if (DEBUG_BLOCK_INFO)
            printf("Post-skipmode[%d]: r=%d\n", b->skip_mode, ts->msac.rng);
    } else {
        b->skip_mode = 0;
    }

    if (b->skip_mode) {
        b->intra = 0;
    } else if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
        if (seg && (seg->ref >= 0 || seg->globalmv)) {
            b->intra = !seg->ref;
        } else {
            const int ictx = get_intra_ctx(t->a, &t->l, by4, bx4,
                                           have_top, have_left);
            b->intra = !dav1d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.intra[ictx]);
            if (DEBUG_BLOCK_INFO)
                printf("Post-intra[%d]: r=%d\n", b->intra, ts->msac.rng);
        }
    } else {
        b->intra = 1;
    }

    const BlockContext *nb0, *nb1;
    int boff0, boff1;

    b->intrabc = 0;
    if (has_luma) {
        // get "spatial neighbours", depending on edge availability;
        // do not cross SB boundaries vertically
        const int have_top_in_sb = !!(t->by & (f->sb_step - 1));
        boff0 = -1;

        // FIXME deal with bottom/right overhangs
        if (have_top_in_sb) {
            if (have_left) {
                nb0 = t->a;  boff0 = bx4 + bw4 - 1;
                nb1 = &t->l; boff1 = by4 + bh4 - 1;
            } else {
                nb0 = nb1 = t->a; boff0 = bx4; boff1 = bx4 + bw4 - 1;
            }
        } else if (have_left) {
            // we use left by default, which is initialized to zero
            nb0 = nb1 = &t->l; boff0 = by4; boff1 = by4 + bh4 - 1;
        }

        // FIXME inter frames have extra conditions for enabling intrabc
        if (f->frame_hdr->allow_intrabc && imin(bw4, bh4) < 64) {
            const int ctx = boff0 == -1 ? 0 : nb0->intrabc[boff0] +
                                              nb1->intrabc[boff1];
            b->intrabc = dav1d_msac_decode_bool_adapt(&ts->msac,
                             ts->cdf.m.intrabc[ctx]);
            DEBUG_BLOCK_printf("%*sPost-intrabc[ctx=%d,%d]: r=%d\n",
                               depth, "", ctx, b->intrabc, ts->msac.rng);
        }
    }

    // skip_txfm
    if (b->skip_mode || (seg && seg->skip)) {
        b->skip_txfm = 1;
    } else if (b->intra && !b->intrabc) {
        b->skip_txfm = 0;
    } else {
        const int ctx = nx[0]->skip_txfm[xoff[0]] + nx[1]->skip_txfm[xoff[1]] +
                        b->skip_mode * 3;
        b->skip_txfm = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                    ts->cdf.m.skip_txfm[ctx]);
        DEBUG_BLOCK_printf("%*sPost-skip_txfm[ctx=%d,%d]: r=%d\n",
                           depth, "", ctx, b->skip_txfm, ts->msac.rng);
    }

    // segment_id
    if (f->frame_hdr->segmentation.enabled &&
        f->frame_hdr->segmentation.update_map &&
        !f->frame_hdr->segmentation.seg_data.preskip)
    {
        if (!b->skip_txfm && f->frame_hdr->segmentation.temporal &&
            (seg_pred = dav1d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.seg_pred[t->a->seg_pred[bx4] +
                            t->l.seg_pred[by4]])))
        {
            // temporal predicted seg_id
            if (f->prev_segmap) {
                unsigned seg_id = get_prev_frame_segid(f, t->by, t->bx, w4, h4,
                                                       f->prev_segmap,
                                                       f->b4_stride);
                if (seg_id >= 8) return -1;
                b->seg_id = seg_id;
            } else {
                b->seg_id = 0;
            }
        } else {
            int seg_ctx;
            const unsigned pred_seg_id =
                get_cur_frame_segid(t->by, t->bx, have_top, have_left,
                                    &seg_ctx, f->cur_segmap, f->b4_stride);
            if (b->skip_txfm) {
                b->seg_id = pred_seg_id;
            } else {
                const unsigned diff = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                          ts->cdf.m.seg_id[seg_ctx],
                                          DAV1D_MAX_SEGMENTS - 1);
                const unsigned last_active_seg_id =
                    f->frame_hdr->segmentation.seg_data.last_active_segid;
                b->seg_id = neg_deinterleave(diff, pred_seg_id,
                                             last_active_seg_id + 1);
                if (b->seg_id > last_active_seg_id) b->seg_id = 0; // error?
            }
            if (b->seg_id >= DAV1D_MAX_SEGMENTS) b->seg_id = 0; // error?
        }

        seg = &f->frame_hdr->segmentation.seg_data.d[b->seg_id];

        if (DEBUG_BLOCK_INFO)
            printf("Post-segid[postskip;%d]: r=%d\n",
                   b->seg_id, ts->msac.rng);
    }

    if (has_luma) {
        // FIXME some of these can be pre-calculated at th estart of a frame
        const int gdf_bs = f->frame_hdr->frame_type == DAV1D_FRAME_TYPE_KEY ?
                           32 : imax(32, 16 << f->seq_hdr->sb128);
        if (!((t->bx | t->by) & (gdf_bs - 1)) &&
            imax(f->cur.p.w, f->cur.p.h) > gdf_bs)
        {
            for (int y = 0; y < bh4; y += gdf_bs) {
                for (int x = 0; x < bw4; x += gdf_bs) {
                    // FIXME separate storage sites for 256x256 blocks
                    t->lf_mask->gdf = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                                   ts->cdf.m.gdf);
                    DEBUG_BLOCK_printf("%*sPost-gdf[y=%d,x=%d,gdf=%d]: r=%d\n",
                                       depth, "", t->by + y, t->bx + x,
                                       t->lf_mask->gdf, ts->msac.rng);
                }
            }
        }
    }

    // cdef index
    if (f->frame_hdr->cdef.enabled &&
        (!b->skip_txfm || f->frame_hdr->cdef.on_skiptx))
    {
        // FIXME 256x256 block size support
        const int idx = f->seq_hdr->sb128 ? ((t->bx & 16) >> 4) +
                                            ((t->by & 16) >> 3) : 0;
        if (t->cur_sb_cdef_idx_ptr[idx] == -1) {
            int v;
            if (f->frame_hdr->cdef.n_strengths == 1) {
                v = 0;
            } else {
                const int left_cdef_idx =
                    t->bx - 16 < ts->tiling.col_start ? -1 :
                    idx & 1 ? t->cur_sb_cdef_idx_ptr[idx - 1] :
                    t->lf_mask[-1].cdef_idx[idx + 1];
                const int top_cdef_idx =
                    t->by - 16 < ts->tiling.row_start ? -1 :
                    idx & 2 ? t->cur_sb_cdef_idx_ptr[idx - 2] :
                    t->lf_mask[-f->sb128w].cdef_idx[idx + 2];
                // cdef_idx=-1: --, 0: true, 1-7: false, edge combo -> context
                // ctx=0: false/false, false/--, --/false, --/--
                // ctx=1: false/true, true/false
                // ctx=2: true/--, --/true, true/true [same coded block]
                // ctx=3: true/true [different coded block]
                int ctx;
                if ((left_cdef_idx | top_cdef_idx) != -1) {
                    // both edges are available
                    ctx = !left_cdef_idx + !top_cdef_idx;
                    // FIXME this should only be done when both edges are *not*
                    // from the same coded block
                    ctx += ctx == 2;
                } else {
                    ctx = !(left_cdef_idx & top_cdef_idx) * 2;
                }
                if (dav1d_msac_decode_bool_adapt(&ts->msac,
                                                 ts->cdf.m.cdef_idx0[ctx]))
                {
                    v = 0;
                } else if (f->frame_hdr->cdef.n_strengths == 2) {
                    v = 1;
                } else {
                    const int rem = f->frame_hdr->cdef.n_strengths - 3;
                    v = 1 + (rem < 3 ?
                             dav1d_msac_decode_symbol_adapt4 :
                             dav1d_msac_decode_symbol_adapt8)(&ts->msac,
                                 ts->cdf.m.cdef_idx[rem], rem + 1);
                }
                DEBUG_BLOCK_printf("%*sPost-cdef_idx[ctx=%d,%d]: r=%d\n",
                                   depth, "", ctx, v, ts->msac.rng);
            }
            t->cur_sb_cdef_idx_ptr[idx] = v;
            if (bw4 > 16) t->cur_sb_cdef_idx_ptr[idx + 1] = v;
            if (bh4 > 16) t->cur_sb_cdef_idx_ptr[idx + 2] = v;
            if (bw4 == 32 && bh4 == 32) t->cur_sb_cdef_idx_ptr[idx + 3] = v;
        }
    }

    // ccso
    if (!((t->bx | t->by) & 63)) {
        for (int p = 0; p < 3; p++) {
            if (!f->frame_hdr->ccso.p[p].enabled) continue;
            if (f->frame_hdr->ccso.p[p].sb_reuse) {
                // FIXME copy from reference
            } else {
                // for left/left-bottom [if no overhang] context:
                // ctx=0: --/--, false/--, --/false, false/false
                // ctx=1: false/true, true/false
                // ctx=2: true/--, --/true, true/true [same coded block]
                // ctx=3: true/true [different coded block]
                const int ctx = have_left ? t->lf_mask[-1].ccso[p] * 2 : 0;
                t->lf_mask->ccso[p] = dav1d_msac_decode_bool_adapt(&ts->msac,
                                            ts->cdf.m.ccso[p][ctx]);
                DEBUG_BLOCK_printf("%*sPost-ccso[pl=%c,%d]: r=%d\n",
                                   depth, "", "yuv"[p], t->lf_mask->ccso[p],
                                   ts->msac.rng);
            }
        }
    }

    // delta-q/lf
    if (!((t->bx | t->by) & (31 >> !f->seq_hdr->sb128))) {
        const int prev_qidx = ts->last_qidx;
        const int have_delta_q = f->frame_hdr->delta.q.present &&
            (bs != (f->seq_hdr->sb128 ? BS_128x128 : BS_64x64) || !b->skip_txfm);

        uint32_t prev_delta_lf = ts->last_delta_lf.u32;

        if (have_delta_q) {
            int delta_q = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                                          ts->cdf.m.delta_q, 3);
            if (delta_q == 3) {
                const int n_bits = 1 + dav1d_msac_decode_bools_bypass(&ts->msac, 3);
                delta_q = dav1d_msac_decode_bools_bypass(&ts->msac, n_bits) +
                          1 + (1 << n_bits);
            }
            if (delta_q) {
                if (dav1d_msac_decode_bool_bypass(&ts->msac)) delta_q = -delta_q;
                delta_q *= 1 << f->frame_hdr->delta.q.res_log2;
            }
            ts->last_qidx = iclip(ts->last_qidx + delta_q, 1, 255);
            if (have_delta_q && DEBUG_BLOCK_INFO)
                printf("Post-delta_q[%d->%d]: r=%d\n",
                       delta_q, ts->last_qidx, ts->msac.rng);

            if (f->frame_hdr->delta.lf.present) {
                const int n_lfs = f->frame_hdr->delta.lf.multi ?
                    f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400 ? 4 : 2 : 1;

                for (int i = 0; i < n_lfs; i++) {
                    int delta_lf = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                        ts->cdf.m.delta_lf[i + f->frame_hdr->delta.lf.multi], 3);
                    if (delta_lf == 3) {
                        const int n_bits = 1 + dav1d_msac_decode_bools_bypass(&ts->msac, 3);
                        delta_lf = dav1d_msac_decode_bools_bypass(&ts->msac, n_bits) +
                                   1 + (1 << n_bits);
                    }
                    if (delta_lf) {
                        if (dav1d_msac_decode_bool_bypass(&ts->msac))
                            delta_lf = -delta_lf;
                        delta_lf *= 1 << f->frame_hdr->delta.lf.res_log2;
                    }
                    ts->last_delta_lf.i8[i] =
                        iclip(ts->last_delta_lf.i8[i] + delta_lf, -63, 63);
                    if (have_delta_q && DEBUG_BLOCK_INFO)
                        printf("Post-delta_lf[%d:%d]: r=%d\n", i, delta_lf,
                               ts->msac.rng);
                }
            }
        }
        if (ts->last_qidx == f->frame_hdr->quant.yac) {
            // assign frame-wide q values to this sb
            ts->dq = f->dq;
        } else if (ts->last_qidx != prev_qidx) {
            // find sb-specific quant parameters
            init_quant_tables(f->seq_hdr, f->frame_hdr, ts->last_qidx, ts->dqmem);
            ts->dq = ts->dqmem;
        }
        if (!ts->last_delta_lf.u32) {
            // assign frame-wide lf values to this sb
            ts->lflvl = f->lf.lvl;
        } else if (ts->last_delta_lf.u32 != prev_delta_lf) {
            // find sb-specific lf lvl parameters
            ts->lflvl = ts->lflvlmem;
            dav1d_calc_lf_values(ts->lflvlmem, f->frame_hdr, ts->last_delta_lf.i8);
        }
    }

    b->fsc = 0;

    // intra/inter-specific stuff
    int midx = 0xff; // intra/luma directional intra prediction index, if set
    if (b->intra && !b->intrabc) {
        static const uint8_t reordered_nondir_y_mode[] = {
            DC_PRED, SMOOTH_PRED, SMOOTH_V_PRED, SMOOTH_H_PRED, PAETH_PRED,
        };
        static const uint8_t reordered_dir_y_mode[] = {
            DIAG_DOWN_LEFT_PRED,  VERT_LEFT_PRED, VERT_PRED, VERT_RIGHT_PRED,
            DIAG_DOWN_RIGHT_PRED, HOR_DOWN_PRED,  HOR_PRED,  HOR_UP_PRED,
        };

        if (has_luma) {
            const int y_set = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                  ts->cdf.m.intra_y_set, 3);
            int y_mode_idx, y_mode_ctx;
            if (!y_set) {
                y_mode_ctx = (t->a->midx[bx4 + bw4 - 1] != 0xff) +
                             (t->l.midx[by4 + bh4 - 1] != 0xff);
                y_mode_idx = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                 ts->cdf.m.intra_y_idx0[y_mode_ctx], 7);
                if (y_mode_idx == 7)
                    y_mode_idx += dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                      ts->cdf.m.intra_y_idx1[y_mode_ctx], 5);
            } else {
                y_mode_idx = y_set * 16 - 3 +
                             dav1d_msac_decode_bools_bypass(&ts->msac, 4);
            }
            if (y_mode_idx < 5) {
                b->y_mode = reordered_nondir_y_mode[y_mode_idx];
                b->y_angle = 0;
            } else {
                const int dir_y_mode_idx = y_mode_idx - 5;
                static const uint8_t default_mode_list_y[] = {
                    17, 45, 3, 10, 24, 31, 38, 52,
                    //  (-2, +2)
                    15, 19, 43, 47, 1, 5, 8, 12, 22, 26, 29, 33, 36, 40, 50, 54,
                    //  (-1, +1)
                    16, 18, 44, 46, 2, 4, 9, 11, 23, 25, 30, 32, 37, 39, 51, 53,
                    //  (-3, +3)
                    14, 20, 42, 48, 0, 6, 7, 13, 21, 27, 28, 34, 35, 41, 49, 55
                };
                uint8_t custom_mode_list_y[56];
                const uint8_t *reorder = default_mode_list_y;
                if (bw4 * bh4 > 2) {
                    // modes are reordered if neighbour (above/left) modes used
                    // directional intra prediction modes
                    uint64_t mask = 0;
                    uint8_t *ptr = custom_mode_list_y;
                    *ptr = -1;
                    if (t->l.midx[by4 + bh4 - 1] != 0xff) {
                        const int lmidx = t->l.midx[by4 + bh4 - 1];
                        *ptr++ = lmidx;
                        mask |= 1ULL << lmidx;
                    }
                    if (t->a->midx[bx4 + bw4 - 1] != 0xff) {
                        const int amidx = t->a->midx[bx4 + bw4 - 1];
                        if (amidx != custom_mode_list_y[0]) {
                            *ptr++ = amidx;
                            mask |= 1ULL << amidx;
                        }
                    }
                    long n_dirs = ptr - custom_mode_list_y;
                    if (n_dirs > 0) {
                        reorder = custom_mode_list_y;
                        if (bw4 * bh4 > 4 && dir_y_mode_idx >= n_dirs) {
                            // add surrounding [-3..+3] angles
                            for (int i = 1; i < 5; i++) {
                                for (int n = 0; n < n_dirs; n++) {
                                    const int cmidx = custom_mode_list_y[n];
                                    for (int delta = -i, j = 0; j < 2; delta = +i, j++) {
                                        // FIXME replace modulo with fastdiv
                                        const int dmidx = (cmidx + delta) % 56;
                                        if (!(mask & (1ULL << dmidx))) {
                                            *ptr++ = dmidx;
                                            mask |= 1ULL << dmidx;
                                        }
                                    }
                                }
                            }
                        }

                        n_dirs = ptr - custom_mode_list_y;
                        if (dir_y_mode_idx >= n_dirs) {
                            // remainder of modes in default order
                            for (unsigned long n = 0;
                                 n < ARRAY_SIZE(default_mode_list_y); n++)
                            {
                                const int fmidx = default_mode_list_y[n];
                                const uint64_t bit = 1ULL << fmidx;
                                if (!(mask & bit)) *ptr++ = fmidx;
                            }
                        }
                    }
                }
                const int dir_y_mode_reord = midx = reorder[dir_y_mode_idx];
                // FIXME division/modulo can be replaced with fastdiv
                b->y_mode = reordered_dir_y_mode[dir_y_mode_reord / 7];
                b->y_angle = dir_y_mode_reord % 7 - 3;
            }
            DEBUG_BLOCK_printf("%*sPost-intra_y_mode[set=%d,idx=%d,ctx=%d,mode=%d,angle=%d]: r=%d\n",
                               depth, "", y_set, y_mode_idx,
                               y_set > 0 ? -1 : y_mode_ctx,
                               b->y_mode, b->y_angle, ts->msac.rng);

            // =min(5,floor(log2(bw4+bh4)*1.99-1.62)) or
            // =      floor(log2(bw4+bh4)*1.55-0.555) - or anything in between
            if (imax(bw4, bh4) <= 8 && f->seq_hdr->fsc) {
                static const uint8_t fsc_bsize_groups[N_BS_SIZES] = {
                    [BS_32x32] = 5,
                    [BS_32x16] = 5,
                    [BS_32x8] = 4,
                    [BS_32x4] = 4,
                    [BS_16x32] = 5,
                    [BS_16x16] = 4,
                    [BS_16x8] = 3,
                    [BS_16x4] = 3,
                    [BS_8x32] = 4,
                    [BS_8x16] = 3,
                    [BS_8x8] = 2,
                    [BS_8x4] = 1,
                    [BS_4x32] = 4,
                    [BS_4x16] = 3,
                    [BS_4x8] = 1,
                    [BS_4x4] = 0,
                };
                const int sz_ctx = fsc_bsize_groups[bs];
                const int ctx = !b->intra ? 3 : boff0 == -1 ? 0 :
                                nb0->fsc[boff0] + nb1->fsc[boff1];
                b->fsc = dav1d_msac_decode_bool_adapt(&ts->msac,
                             ts->cdf.m.fsc[ctx][sz_ctx]);
                DEBUG_BLOCK_printf("%*sPost-fsc[ctx=%d|%d,%d]: r=%d\n",
                                   depth, "", ctx, sz_ctx, b->fsc, ts->msac.rng);
            }

            b->mrl_index = b->multi_mrl = 0;
            if (midx != 0xff /* directional mode */) {
                const int ctx = boff0 == -1 ? 0 : nb0->mrl[boff0] + nb1->mrl[boff1];
                b->mrl_index = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                   ts->cdf.m.mrl_index[ctx], 3);
                DEBUG_BLOCK_printf("%*sPost-mrl_index[ctx=%d,%d]: r=%d\n",
                                   depth, "", ctx, b->mrl_index, ts->msac.rng);
                if (b->mrl_index > 0) {
                    const int ctx2 = boff0 == -1 ? 0 :
                                     nb0->multi_mrl[boff0] + nb1->multi_mrl[boff1];
                    b->multi_mrl = dav1d_msac_decode_bool_adapt(&ts->msac,
                                       ts->cdf.m.multi_mrl[ctx2]);
                    DEBUG_BLOCK_printf("%*sPost-multi_line_mrl[ctx=%d,%d]: r=%d\n",
                                       depth, "", ctx2, b->multi_mrl, ts->msac.rng);
                }
            }
        }

        if (has_chroma) {
            const int ll = f->frame_hdr->segmentation.lossless[b->seg_id];
            const int cfl_allowed = f->seq_hdr->cfl &&
                                    imax(cbw4, cbh4) <= (ll ? 1 : 16);
            int is_cfl = 0, uv_mode_idx, cfl_ctx, uv_mode_ctx;
            if (cfl_allowed) {
                cfl_ctx = (t->a->uvmode[cbx4] == CFL_PRED) +
                          (t->l.uvmode[cby4] == CFL_PRED);
                is_cfl = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                      ts->cdf.m.cfl[cfl_ctx]);
            }
            if (is_cfl) {
                b->uv_mode = CFL_PRED;
                b->uv_angle = 0;
            } else {
                if (lbs == BS_INVALID)
                    midx = t->luma_intra_dir_mode_map[by4 * 32 + bx4];
                uv_mode_ctx = midx != 0xff;
                uv_mode_idx = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                  ts->cdf.m.intra_uv_mode[uv_mode_ctx], 7);
                if (uv_mode_idx == 7)
                    uv_mode_idx += dav1d_msac_decode_bools_bypass(&ts->msac, 3);
                // FIXME set error bit to shortcut decoding
                if (uv_mode_idx > 12) return -1;
                if (uv_mode_idx < uv_mode_ctx) {
                    b->uv_mode = reordered_dir_y_mode[midx / 7];
                    b->uv_angle = (midx % 7) - 3;
                } else {
                    if (uv_mode_idx - uv_mode_ctx < 5) {
                        b->uv_mode = reordered_nondir_y_mode[uv_mode_idx -
                                                             uv_mode_ctx];
                        b->uv_angle = 0;
                    } else {
                        static const uint8_t default_mode_list_uv[] = {
                            VERT_PRED, HOR_PRED, DIAG_DOWN_LEFT_PRED,
                            DIAG_DOWN_RIGHT_PRED, VERT_LEFT_PRED,
                            VERT_RIGHT_PRED, HOR_DOWN_PRED, HOR_UP_PRED,
                        };
                        int idx = uv_mode_idx - 5 - uv_mode_ctx;
                        idx += idx >= midx / 7;
                        b->uv_mode = default_mode_list_uv[idx];
                        b->uv_angle = 0;
                    }
                }
            }
            DEBUG_BLOCK_printf("%*sPost-intra_uv_mode[cfl=%d,idx=%d,ctx=%d|%d,mode=%d,angle=%d]: r=%d\n",
                               depth, "", is_cfl, is_cfl ? -1 : uv_mode_idx,
                               cfl_allowed ? cfl_ctx : -1,
                               is_cfl ? -1 : uv_mode_ctx, b->uv_mode,
                               b->uv_angle, ts->msac.rng);
            if (b->uv_mode == CFL_PRED) {
                memset(b->cfl_alpha, 0, sizeof(b->cfl_alpha));
                if (f->seq_hdr->mhccp &&
                    dav1d_msac_decode_bool_adapt(&ts->msac, ts->cdf.m.mhccp))
                {
                    static const uint8_t size_group_lookup[] = {
                        [BS_4x4] = 0,
                        [BS_4x8] = 0,
                        [BS_8x4] = 0,
                        [BS_8x8] = 1,
                        [BS_8x16] = 1,
                        [BS_16x8] = 1,
                        [BS_16x16] = 2,
                        [BS_16x32] = 2,
                        [BS_32x16] = 2,
                        [BS_32x32] = 3,
                        [BS_32x64] = 3,
                        [BS_64x32] = 3,
                        [BS_64x64] = 3,
                        [BS_64x128] = 3,
                        [BS_128x64] = 3,
                        [BS_128x128] = 3,
                        [BS_128x256] = 3,
                        [BS_256x128] = 3,
                        [BS_256x256] = 3,
                        [BS_4x16] = 0,
                        [BS_16x4] = 0,
                        [BS_8x32] = 1,
                        [BS_32x8] = 1,
                        [BS_16x64] = 2,
                        [BS_64x16] = 2,
                        [BS_4x32] = 1,
                        [BS_32x4]= 1,
                        [BS_8x64] = 2,
                        [BS_64x8] = 2,
                        [BS_4x64] = 2,
                        [BS_64x4] = 2,
                    };
                    const int sz_ctx = size_group_lookup[bs];
                    b->cfl_type = CFL_MHCCP;
                    b->mh_dir =
                        dav1d_msac_decode_symbol_adapt4(&ts->msac,
                            ts->cdf.m.mhccp_filter_dir[sz_ctx], 2);
                } else {
                    b->cfl_type = dav1d_msac_decode_bool_adapt(&ts->msac,
                                      ts->cdf.m.cfl_type);
                    if (b->cfl_type == CFL_EXPLICIT) {
                        const int sign = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                             ts->cdf.m.cfl_sign, 7) + 1;
                        const int sign_u = sign * 0x56 >> 8;
                        const int sign_v = sign - sign_u * 3;
                        assert(sign_u == sign / 3);
                        if (sign_u) {
                            const int ctx = (sign_u == 2) * 3 + sign_v;
                            b->cfl_alpha[0] = dav1d_msac_decode_symbol_adapt16(&ts->msac,
                                    ts->cdf.m.cfl_alpha[ctx], 7) + 1;
                            if (sign_u == 1) b->cfl_alpha[0] = -b->cfl_alpha[0];
                        }
                        if (sign_v) {
                            const int ctx = (sign_v == 2) * 3 + sign_u;
                            b->cfl_alpha[1] = dav1d_msac_decode_symbol_adapt16(&ts->msac,
                                    ts->cdf.m.cfl_alpha[ctx], 7) + 1;
                            if (sign_v == 1) b->cfl_alpha[1] = -b->cfl_alpha[1];
                        }
                    }
                }
                DEBUG_BLOCK_printf("%*sPost-cfl[type=%d,%s=%d|%d]: r=%d\n",
                                   depth, "", b->cfl_type,
                                   b->cfl_type == CFL_MHCCP ? "mhdir" : "alpha",
                                   b->cfl_type == CFL_MHCCP ? b->mh_dir :
                                                              b->cfl_alpha[0],
                                   b->cfl_alpha[1], ts->msac.rng);
            }
        }

        b->pal_sz[0] = b->pal_sz[1] = 0;
        if (f->frame_hdr->allow_screen_content_tools &&
            imax(bw4, bh4) <= 16 && bw4 + bh4 >= 4)
        {
            const int sz_ctx = b_dim[2] + b_dim[3] - 2;
            if (has_luma) {
                if (b->y_mode == DC_PRED) {
                    const int pal_ctx = (t->a->pal_sz[bx4] > 0) + (t->l.pal_sz[by4] > 0);
                    const int use_y_pal = dav1d_msac_decode_bool_adapt(&ts->msac,
                                              ts->cdf.m.pal_y[sz_ctx][pal_ctx]);
                    if (DEBUG_BLOCK_INFO)
                        printf("Post-y_pal[%d]: r=%d\n", use_y_pal, ts->msac.rng);
                    if (use_y_pal)
                        f->bd_fn.read_pal_plane(t, b, 0, sz_ctx, bx4, by4);
                }
            }

            if (has_chroma && b->uv_mode == DC_PRED) {
                const int pal_ctx = b->pal_sz[0] > 0;
                const int use_uv_pal = dav1d_msac_decode_bool_adapt(&ts->msac,
                                           ts->cdf.m.pal_uv[pal_ctx]);
                if (DEBUG_BLOCK_INFO)
                    printf("Post-uv_pal[%d]: r=%d\n", use_uv_pal, ts->msac.rng);
                if (use_uv_pal) // see aomedia bug 2183 for why we use luma coordinates
                    f->bd_fn.read_pal_uv(t, b, sz_ctx, bx4, by4);
            }
        }

        if (has_luma && b->pal_sz[0]) {
            uint8_t *pal_idx;
            if (t->frame_thread.pass) {
                const int p = t->frame_thread.pass & 1;
                assert(ts->frame_thread[p].pal_idx);
                pal_idx = ts->frame_thread[p].pal_idx;
                ts->frame_thread[p].pal_idx += bw4 * bh4 * 8;
            } else
                pal_idx = t->scratch.pal_idx_y;
            read_pal_indices(t, pal_idx, b->pal_sz[0], 0, w4, h4, bw4, bh4);
            if (DEBUG_BLOCK_INFO)
                printf("Post-y-pal-indices: r=%d\n", ts->msac.rng);
        }

        if (has_chroma && b->pal_sz[1]) {
            uint8_t *pal_idx;
            if (t->frame_thread.pass) {
                const int p = t->frame_thread.pass & 1;
                assert(ts->frame_thread[p].pal_idx);
                pal_idx = ts->frame_thread[p].pal_idx;
                ts->frame_thread[p].pal_idx += cbw4 * cbh4 * 8;
            } else
                pal_idx = t->scratch.pal_idx_uv;
            read_pal_indices(t, pal_idx, b->pal_sz[1], 1, cw4, ch4, cbw4, cbh4);
            if (DEBUG_BLOCK_INFO)
                printf("Post-uv-pal-indices: r=%d\n", ts->msac.rng);
        }

        if (has_luma) {
            b->dip = 0;
            if (b->y_mode == DC_PRED && f->seq_hdr->intra_dip &&
                !b->mrl_index && imin(bw4, bh4) >= 2 && bw4 * bh4 >= 8)
            {
                const int ctx = boff0 == -1 ? 0 : nb0->dip[boff0] + nb1->dip[boff1];
                b->dip = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                      ts->cdf.coef.dip[ctx]);
                if (b->dip) {
                    b->dip = 1 |
                        (dav1d_msac_decode_bool_bypass(&ts->msac) << 1) |
                        (dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                                         ts->cdf.m.dip_mode, 5) << 2);
                }
                DEBUG_BLOCK_printf("%*sPost-dip[ctx=%d,%d]: r=%d\n",
                                   depth, "", ctx, !!b->dip, ts->msac.rng);
            }

            b->tx_part = TX_PARTITION_NONE;
            if (f->frame_hdr->segmentation.lossless[b->seg_id]) {
                // FIXME I believe this can be wht as well as idtx?
            } else {
                b->uvtx = dav1d_max_txfm_size_for_bs[bs][f->cur.p.layout];

                if (f->frame_hdr->txfm_mode == DAV1D_TX_SWITCHABLE &&
                    bs != BS_4x4 && imax(bw4, bh4) <= 16)
                {
                    static const uint8_t size_to_tx_part_group_lookup[] = {
                        [BS_64x64] = 7,
                        [BS_64x32] = 6,
                        [BS_64x16] = 8,
                        [BS_64x8] = 8,
                        [BS_64x4] = 8,
                        [BS_32x64] = 6,
                        [BS_32x32] = 5,
                        [BS_32x16] = 4,
                        [BS_32x8] = 8,
                        [BS_32x4] = 8,
                        [BS_16x64] = 8,
                        [BS_16x32] = 4,
                        [BS_16x16] = 3,
                        [BS_16x8] = 2,
                        [BS_16x4] = 8,
                        [BS_8x64] = 8,
                        [BS_8x32] = 8,
                        [BS_8x16] = 2,
                        [BS_8x8] = 1,
                        [BS_8x4] = 0,
                        [BS_4x64] = 8,
                        [BS_4x32] = 8,
                        [BS_4x16] = 8,
                        [BS_4x8] = 0,
                        [BS_4x4] = 0,
                    };
                    const int szctx = size_to_tx_part_group_lookup[bs];
                    int is_split = dav1d_msac_decode_bool_adapt(&ts->msac,
                                       ts->cdf.m.tx_split[b->fsc][0][szctx]);
                    if (is_split) {
                        if (imin(bw4, bh4) >= 2) {
                            static const uint8_t size_to_tx_type_group_vh_lookup[] = {
                                [BS_64x64] = 9,
                                [BS_64x32] = 8,
                                [BS_64x16] = 13,
                                [BS_64x8] = 11,
                                [BS_32x64] = 7,
                                [BS_32x32] = 6,
                                [BS_32x16] = 5,
                                [BS_32x8] = 11,
                                [BS_16x64] = 12,
                                [BS_16x32] = 4,
                                [BS_16x16] = 3,
                                [BS_16x8] = 2,
                                [BS_8x64] = 10,
                                [BS_8x32] = 10,
                                [BS_8x16] = 1,
                                [BS_8x8] = 0,
                            };
                            const int ctx = size_to_tx_type_group_vh_lookup[bs];
                            b->tx_part = 1 +
                                dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                    ts->cdf.m.tx_part_2d[b->fsc][0][ctx], 6);
                        } else if (imax(bw4, bh4) >= 4) {
                            const int ctx = bw4 >= 4;
                            const int tx_part_4way =
                                dav1d_msac_decode_bool_adapt(&ts->msac,
                                    ts->cdf.m.tx_part_1d[b->fsc][0][ctx]);
                            b->tx_part = TX_PARTITION_H + ctx + tx_part_4way * 2;
                        } else {
                            assert(bs == BS_4x8 || bs == BS_8x4);
                            b->tx_part = bs == BS_4x8 ? TX_PARTITION_H :
                                                        TX_PARTITION_V;
                        }
                    }
                }
                DEBUG_BLOCK_printf("%*sPost-tx[%d]: r=%d\n",
                                   depth, "", b->tx_part, ts->msac.rng);
            }
        }

        // reconstruction
        if (t->frame_thread.pass == 1) {
            f->bd_fn.read_coef_blocks(t, bs, b);
        } else {
            f->bd_fn.recon_b_intra(t, DB_ONLY(depth) lbs, cbs, 0, b);
        }

        if (f->frame_hdr->loopfilter.level_y[0] ||
            f->frame_hdr->loopfilter.level_y[1])
        {
            dav1d_create_lf_mask_intra(t->lf_mask, f->lf.level, f->b4_stride,
                                       (const uint8_t (*)[8][2])
                                       &ts->lflvl[b->seg_id][0][0][0],
                                       t->bx, t->by, f->w4, f->h4, bs,
                                       0 /*b->tx*/, b->uvtx, f->cur.p.layout,
                                       &t->a->tx_lpf_y[bx4], &t->l.tx_lpf_y[by4],
                                       has_chroma ? &t->a->tx_lpf_uv[cbx4] : NULL,
                                       has_chroma ? &t->l.tx_lpf_uv[cby4] : NULL);
        }
        // update contexts
        const enum IntraPredMode y_mode_nofilt =
            b->y_mode == FILTER_PRED ? DC_PRED : b->y_mode;
        BlockContext *edge = t->a;
        for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
#define set_ctx(rep_macro) \
            rep_macro(edge->tx, off, 0 /* t_lsz */); \
            rep_macro(edge->fsc, off, b->fsc); \
            rep_macro(edge->mode, off, y_mode_nofilt); \
            rep_macro(edge->midx, off, midx); \
            rep_macro(edge->mrl, off, !!b->mrl_index); \
            rep_macro(edge->multi_mrl, off, b->multi_mrl); \
            rep_macro(edge->dip, off, !!b->dip); \
            rep_macro(edge->pal_sz, off, b->pal_sz[0]); \
            rep_macro(edge->seg_pred, off, seg_pred); \
            rep_macro(edge->skip_mode, off, 0); \
            rep_macro(edge->intra, off, 1); \
            rep_macro(edge->intrabc, off, 0); \
            rep_macro(edge->skip_txfm, off, b->skip_txfm); \
            /* see aomedia bug 2183 for why we use luma coordinates here */ \
            rep_macro(t->pal_sz_uv[i], off, (has_chroma ? b->pal_sz[1] : 0)); \
            if (IS_INTER_OR_SWITCH(f->frame_hdr)) { \
                rep_macro(edge->comp_type, off, COMP_INTER_NONE); \
                rep_macro(edge->ref[0], off, ((uint8_t) -1)); \
                rep_macro(edge->ref[1], off, ((uint8_t) -1)); \
                rep_macro(edge->filter[0], off, DAV1D_N_SWITCHABLE_FILTERS); \
                rep_macro(edge->filter[1], off, DAV1D_N_SWITCHABLE_FILTERS); \
            }
            case_set(b_dim[2 + i]);
#undef set_ctx
        }
        if (b->pal_sz[0])
            f->bd_fn.copy_pal_block_y(t, bx4, by4, bw4, bh4);
        if (has_chroma) {
            uint8_t uv_mode = b->uv_mode;
            dav1d_memset_pow2[ulog2(cbw4)](&t->a->uvmode[cbx4], uv_mode);
            dav1d_memset_pow2[ulog2(cbh4)](&t->l.uvmode[cby4], uv_mode);
            if (b->pal_sz[1])
                f->bd_fn.copy_pal_block_uv(t, bx4, by4, bw4, bh4);
        }
        if (IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc)
            splat_intraref(f->c, t, bs, bw4, bh4);
    } else if (b->intrabc) {
        // intra block copy
        refmvs_candidate mvstack[8];
        int n_mvs, ctx;
        dav1d_refmvs_find(&t->rt, mvstack, &n_mvs, &ctx,
                          (union refmvs_refpair) { .ref = { 0, -1 }},
                          bs, 0, t->by, t->bx);

        if (mvstack[0].mv.mv[0].n)
            b->mv[0] = mvstack[0].mv.mv[0];
        else if (mvstack[1].mv.mv[0].n)
            b->mv[0] = mvstack[1].mv.mv[0];
        else {
            if (t->by - (16 << f->seq_hdr->sb128) < ts->tiling.row_start) {
                b->mv[0].y = 0;
                b->mv[0].x = -(512 << f->seq_hdr->sb128) - 2048;
            } else {
                b->mv[0].y = -(512 << f->seq_hdr->sb128);
                b->mv[0].x = 0;
            }
        }

        const union mv ref = b->mv[0];
        read_mv_residual(ts, &b->mv[0], -1);

        // clip intrabc motion vector to decoded parts of current tile
        int border_left = ts->tiling.col_start * 4;
        int border_top  = ts->tiling.row_start * 4;
        if (has_chroma) {
            if (bw4 < 2 &&  ss_hor)
                border_left += 4;
            if (bh4 < 2 &&  ss_ver)
                border_top  += 4;
        }
        int src_left   = t->bx * 4 + (b->mv[0].x >> 3);
        int src_top    = t->by * 4 + (b->mv[0].y >> 3);
        int src_right  = src_left + bw4 * 4;
        int src_bottom = src_top  + bh4 * 4;
        const int border_right = ((ts->tiling.col_end + (bw4 - 1)) & ~(bw4 - 1)) * 4;

        // check against left or right tile boundary and adjust if necessary
        if (src_left < border_left) {
            src_right += border_left - src_left;
            src_left  += border_left - src_left;
        } else if (src_right > border_right) {
            src_left  -= src_right - border_right;
            src_right -= src_right - border_right;
        }
        // check against top tile boundary and adjust if necessary
        if (src_top < border_top) {
            src_bottom += border_top - src_top;
            src_top    += border_top - src_top;
        }

        const int sbx = (t->bx >> (4 + f->seq_hdr->sb128)) << (6 + f->seq_hdr->sb128);
        const int sby = (t->by >> (4 + f->seq_hdr->sb128)) << (6 + f->seq_hdr->sb128);
        const int sb_size = 1 << (6 + f->seq_hdr->sb128);
        // check for overlap with current superblock
        if (src_bottom > sby && src_right > sbx) {
            if (src_top - border_top >= src_bottom - sby) {
                // if possible move src up into the previous suberblock row
                src_top    -= src_bottom - sby;
                src_bottom -= src_bottom - sby;
            } else if (src_left - border_left >= src_right - sbx) {
                // if possible move src left into the previous suberblock
                src_left  -= src_right - sbx;
                src_right -= src_right - sbx;
            }
        }
        // move src up if it is below current superblock row
        if (src_bottom > sby + sb_size) {
            src_top    -= src_bottom - (sby + sb_size);
            src_bottom -= src_bottom - (sby + sb_size);
        }
        // error out if mv still overlaps with the current superblock
        if (src_bottom > sby && src_right > sbx)
            return -1;

        b->mv[0].x = (src_left - t->bx * 4) * 8;
        b->mv[0].y = (src_top  - t->by * 4) * 8;

        if (DEBUG_BLOCK_INFO)
            printf("Post-dmv[%d/%d,ref=%d/%d|%d/%d]: r=%d\n",
                   b->mv[0].y, b->mv[0].x, ref.y, ref.x,
                   mvstack[0].mv.mv[0].y, mvstack[0].mv.mv[0].x, ts->msac.rng);
        read_vartx_tree(t, b, bs, bx4, by4);

        // reconstruction
        if (t->frame_thread.pass == 1) {
            f->bd_fn.read_coef_blocks(t, bs, b);
            b->filter2d = FILTER_2D_BILINEAR;
        } else {
            if (f->bd_fn.recon_b_inter(t, bs, b)) return -1;
        }

        splat_intrabc_mv(f->c, t, bs, b, bw4, bh4);
        BlockContext *edge = t->a;
        for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
#define set_ctx(rep_macro) \
            rep_macro(edge->fsc, off, 0); \
            rep_macro(edge->mode, off, DC_PRED); \
            rep_macro(edge->midx, off, 0xff); \
            rep_macro(edge->mrl, off, 0); \
            rep_macro(edge->multi_mrl, off, 0); \
            rep_macro(edge->dip, off, 0); \
            rep_macro(edge->pal_sz, off, 0); \
            /* see aomedia bug 2183 for why this is outside if (has_chroma) */ \
            rep_macro(t->pal_sz_uv[i], off, 0); \
            rep_macro(edge->seg_pred, off, seg_pred); \
            rep_macro(edge->skip_mode, off, 0); \
            rep_macro(edge->intrabc, off, 1); \
            rep_macro(edge->intra, off, 1); \
            rep_macro(edge->skip_txfm, off, b->skip_txfm)
            case_set(b_dim[2 + i]);
#undef set_ctx
        }
        if (has_chroma) {
            dav1d_memset_pow2[ulog2(cbw4)](&t->a->uvmode[cbx4], DC_PRED);
            dav1d_memset_pow2[ulog2(cbh4)](&t->l.uvmode[cby4], DC_PRED);
        }
    } else {
        // inter-specific mode/mv coding
        int is_comp, has_subpel_filter;

        if (b->skip_mode) {
            is_comp = 1;
        } else if ((!seg || (seg->ref == -1 && !seg->globalmv && !seg->skip)) &&
                   f->frame_hdr->switchable_comp_refs && imin(bw4, bh4) > 1)
        {
            const int ctx = get_comp_ctx(t->a, &t->l, by4, bx4,
                                         have_top, have_left);
            is_comp = dav1d_msac_decode_bool_adapt(&ts->msac,
                          ts->cdf.m.comp[ctx]);
            if (DEBUG_BLOCK_INFO)
                printf("Post-compflag[%d]: r=%d\n", is_comp, ts->msac.rng);
        } else {
            is_comp = 0;
        }

        if (b->skip_mode) {
            b->ref[0] = f->frame_hdr->skip_mode_refs[0];
            b->ref[1] = f->frame_hdr->skip_mode_refs[1];
            b->comp_type = COMP_INTER_AVG;
            b->inter_mode = NEARESTMV_NEARESTMV;
            b->drl_idx = NEAREST_DRL;
            has_subpel_filter = 0;

            refmvs_candidate mvstack[8];
            int n_mvs, ctx;
            dav1d_refmvs_find(&t->rt, mvstack, &n_mvs, &ctx,
                              (union refmvs_refpair) { .ref = {
                                    b->ref[0] + 1, b->ref[1] + 1 }},
                              bs, 0, t->by, t->bx);

            b->mv[0] = mvstack[0].mv.mv[0];
            b->mv[1] = mvstack[0].mv.mv[1];
            fix_mv_precision(f->frame_hdr, &b->mv[0]);
            fix_mv_precision(f->frame_hdr, &b->mv[1]);
            if (DEBUG_BLOCK_INFO)
                printf("Post-skipmodeblock[mv=1:y=%d,x=%d,2:y=%d,x=%d,refs=%d+%d\n",
                       b->mv[0].y, b->mv[0].x, b->mv[1].y, b->mv[1].x,
                       b->ref[0], b->ref[1]);
        } else if (is_comp) {
            const int dir_ctx = get_comp_dir_ctx(t->a, &t->l, by4, bx4,
                                                 have_top, have_left);
            if (dav1d_msac_decode_bool_adapt(&ts->msac,
                    ts->cdf.m.comp_dir[dir_ctx]))
            {
                // bidir - first reference (fw)
                const int ctx1 = av1_get_fwd_ref_ctx(t->a, &t->l, by4, bx4,
                                                     have_top, have_left);
                if (dav1d_msac_decode_bool_adapt(&ts->msac,
                        ts->cdf.m.comp_fwd_ref[0][ctx1]))
                {
                    const int ctx2 = av1_get_fwd_ref_2_ctx(t->a, &t->l, by4, bx4,
                                                           have_top, have_left);
                    b->ref[0] = 2 + dav1d_msac_decode_bool_adapt(&ts->msac,
                                        ts->cdf.m.comp_fwd_ref[2][ctx2]);
                } else {
                    const int ctx2 = av1_get_fwd_ref_1_ctx(t->a, &t->l, by4, bx4,
                                                           have_top, have_left);
                    b->ref[0] = dav1d_msac_decode_bool_adapt(&ts->msac,
                                    ts->cdf.m.comp_fwd_ref[1][ctx2]);
                }

                // second reference (bw)
                const int ctx3 = av1_get_bwd_ref_ctx(t->a, &t->l, by4, bx4,
                                                     have_top, have_left);
                if (dav1d_msac_decode_bool_adapt(&ts->msac,
                        ts->cdf.m.comp_bwd_ref[0][ctx3]))
                {
                    b->ref[1] = 6;
                } else {
                    const int ctx4 = av1_get_bwd_ref_1_ctx(t->a, &t->l, by4, bx4,
                                                           have_top, have_left);
                    b->ref[1] = 4 + dav1d_msac_decode_bool_adapt(&ts->msac,
                                        ts->cdf.m.comp_bwd_ref[1][ctx4]);
                }
            } else {
                // unidir
                const int uctx_p = av1_get_uni_p_ctx(t->a, &t->l, by4, bx4,
                                                     have_top, have_left);
                if (dav1d_msac_decode_bool_adapt(&ts->msac,
                        ts->cdf.m.comp_uni_ref[0][uctx_p]))
                {
                    b->ref[0] = 4;
                    b->ref[1] = 6;
                } else {
                    const int uctx_p1 = av1_get_uni_p1_ctx(t->a, &t->l, by4, bx4,
                                                           have_top, have_left);
                    b->ref[0] = 0;
                    b->ref[1] = 1 + dav1d_msac_decode_bool_adapt(&ts->msac,
                                        ts->cdf.m.comp_uni_ref[1][uctx_p1]);
                    if (b->ref[1] == 2) {
                        const int uctx_p2 = av1_get_uni_p2_ctx(t->a, &t->l, by4, bx4,
                                                               have_top, have_left);
                        b->ref[1] += dav1d_msac_decode_bool_adapt(&ts->msac,
                                         ts->cdf.m.comp_uni_ref[2][uctx_p2]);
                    }
                }
            }
            if (DEBUG_BLOCK_INFO)
                printf("Post-refs[%d/%d]: r=%d\n",
                       b->ref[0], b->ref[1], ts->msac.rng);

            refmvs_candidate mvstack[8];
            int n_mvs, ctx;
            dav1d_refmvs_find(&t->rt, mvstack, &n_mvs, &ctx,
                              (union refmvs_refpair) { .ref = {
                                    b->ref[0] + 1, b->ref[1] + 1 }},
                              bs, 0, t->by, t->bx);

            b->inter_mode = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                ts->cdf.m.comp_inter_mode[ctx],
                                N_COMP_INTER_PRED_MODES - 1);
            if (DEBUG_BLOCK_INFO)
                printf("Post-compintermode[%d,ctx=%d,n_mvs=%d]: r=%d\n",
                       b->inter_mode, ctx, n_mvs, ts->msac.rng);

            const uint8_t *const im = dav1d_comp_inter_pred_modes[b->inter_mode];
            b->drl_idx = NEAREST_DRL;
            if (b->inter_mode == NEWMV_NEWMV) {
                if (n_mvs > 1) { // NEARER, NEAR or NEARISH
                    const int drl_ctx_v1 = get_drl_context(mvstack, 0);
                    b->drl_idx += dav1d_msac_decode_bool_adapt(&ts->msac,
                                      ts->cdf.m.drl_bit[drl_ctx_v1]);
                    if (b->drl_idx == NEARER_DRL && n_mvs > 2) {
                        const int drl_ctx_v2 = get_drl_context(mvstack, 1);
                        b->drl_idx += dav1d_msac_decode_bool_adapt(&ts->msac,
                                          ts->cdf.m.drl_bit[drl_ctx_v2]);
                    }
                    if (DEBUG_BLOCK_INFO)
                        printf("Post-drlidx[%d,n_mvs=%d]: r=%d\n",
                               b->drl_idx, n_mvs, ts->msac.rng);
                }
            } else if (im[0] == NEARMV || im[1] == NEARMV) {
                b->drl_idx = NEARER_DRL;
                if (n_mvs > 2) { // NEAR or NEARISH
                    const int drl_ctx_v2 = get_drl_context(mvstack, 1);
                    b->drl_idx += dav1d_msac_decode_bool_adapt(&ts->msac,
                                      ts->cdf.m.drl_bit[drl_ctx_v2]);
                    if (b->drl_idx == NEAR_DRL && n_mvs > 3) {
                        const int drl_ctx_v3 = get_drl_context(mvstack, 2);
                        b->drl_idx += dav1d_msac_decode_bool_adapt(&ts->msac,
                                          ts->cdf.m.drl_bit[drl_ctx_v3]);
                    }
                    if (DEBUG_BLOCK_INFO)
                        printf("Post-drlidx[%d,n_mvs=%d]: r=%d\n",
                               b->drl_idx, n_mvs, ts->msac.rng);
                }
            }
            assert(b->drl_idx >= NEAREST_DRL && b->drl_idx <= NEARISH_DRL);

#define assign_comp_mv(idx) \
            switch (im[idx]) { \
            case NEARMV: \
            case NEARESTMV: \
                b->mv[idx] = mvstack[b->drl_idx].mv.mv[idx]; \
                fix_mv_precision(f->frame_hdr, &b->mv[idx]); \
                break; \
            case GLOBALMV: \
                has_subpel_filter |= \
                    f->frame_hdr->gmv[b->ref[idx]].type == DAV1D_WM_TYPE_TRANSLATION; \
                b->mv[idx] = get_gmv_2d(&f->frame_hdr->gmv[b->ref[idx]], \
                                        t->bx, t->by, bw4, bh4, f->frame_hdr); \
                break; \
            case NEWMV: \
                b->mv[idx] = mvstack[b->drl_idx].mv.mv[idx]; \
                const int mv_prec = f->frame_hdr->hp - f->frame_hdr->force_integer_mv; \
                read_mv_residual(ts, &b->mv[idx], mv_prec); \
                break; \
            }
            has_subpel_filter = imin(bw4, bh4) == 1 ||
                                b->inter_mode != GLOBALMV_GLOBALMV;
            assign_comp_mv(0);
            assign_comp_mv(1);
#undef assign_comp_mv
            if (DEBUG_BLOCK_INFO)
                printf("Post-residual_mv[1:y=%d,x=%d,2:y=%d,x=%d]: r=%d\n",
                       b->mv[0].y, b->mv[0].x, b->mv[1].y, b->mv[1].x,
                       ts->msac.rng);

            // jnt_comp vs. seg vs. wedge
            int is_segwedge = 0;
            if (f->seq_hdr->masked_compound) {
                const int mask_ctx = get_mask_comp_ctx(t->a, &t->l, by4, bx4);

                is_segwedge = dav1d_msac_decode_bool_adapt(&ts->msac,
                                  ts->cdf.m.mask_comp[mask_ctx]);
                if (DEBUG_BLOCK_INFO)
                    printf("Post-segwedge_vs_jntavg[%d,ctx=%d]: r=%d\n",
                           is_segwedge, mask_ctx, ts->msac.rng);
            }

            if (!is_segwedge) {
#if 0
                if (f->seq_hdr->jnt_comp) {
                    const int jnt_ctx =
                        get_jnt_comp_ctx(f->seq_hdr->order_hint_n_bits,
                                         f->cur.frame_hdr->frame_offset,
                                         f->refp[b->ref[0]].p.frame_hdr->frame_offset,
                                         f->refp[b->ref[1]].p.frame_hdr->frame_offset,
                                         t->a, &t->l, by4, bx4);
                    b->comp_type = COMP_INTER_WEIGHTED_AVG +
                                   dav1d_msac_decode_bool_adapt(&ts->msac,
                                       ts->cdf.m.jnt_comp[jnt_ctx]);
                    if (DEBUG_BLOCK_INFO)
                        printf("Post-jnt_comp[%d,ctx=%d[ac:%d,ar:%d,lc:%d,lr:%d]]: r=%d\n",
                               b->comp_type == COMP_INTER_AVG,
                               jnt_ctx, t->a->comp_type[bx4], t->a->ref[0][bx4],
                               t->l.comp_type[by4], t->l.ref[0][by4],
                               ts->msac.rng);
                } else
#endif
                {
                    b->comp_type = COMP_INTER_AVG;
                }
            } else {
                if (wedge_allowed_mask & (1 << bs)) {
                    const int ctx = dav1d_wedge_ctx_lut[bs];
                    b->comp_type = COMP_INTER_WEDGE -
                                   dav1d_msac_decode_bool_adapt(&ts->msac,
                                       ts->cdf.m.wedge_comp[ctx]);
                    if (b->comp_type == COMP_INTER_WEDGE)
                        b->wedge_idx = dav1d_msac_decode_symbol_adapt16(&ts->msac,
                                           ts->cdf.m.wedge_idx[ctx], 15);
                } else {
                    b->comp_type = COMP_INTER_SEG;
                }
                b->mask_sign = dav1d_msac_decode_bool_bypass(&ts->msac);
                if (DEBUG_BLOCK_INFO)
                    printf("Post-seg/wedge[%d,wedge_idx=%d,sign=%d]: r=%d\n",
                           b->comp_type == COMP_INTER_WEDGE,
                           b->wedge_idx, b->mask_sign, ts->msac.rng);
            }
        } else {
            b->comp_type = COMP_INTER_NONE;

            // ref
            if (seg && seg->ref > 0) {
                b->ref[0] = seg->ref - 1;
            } else if (seg && (seg->globalmv || seg->skip)) {
                b->ref[0] = 0;
            } else {
                const int ctx1 = av1_get_ref_ctx(t->a, &t->l, by4, bx4,
                                                 have_top, have_left);
                if (dav1d_msac_decode_bool_adapt(&ts->msac,
                                                 ts->cdf.m.ref[0][ctx1]))
                {
                    const int ctx2 = av1_get_ref_2_ctx(t->a, &t->l, by4, bx4,
                                                       have_top, have_left);
                    if (dav1d_msac_decode_bool_adapt(&ts->msac,
                                                     ts->cdf.m.ref[1][ctx2]))
                    {
                        b->ref[0] = 6;
                    } else {
                        const int ctx3 = av1_get_ref_6_ctx(t->a, &t->l, by4, bx4,
                                                           have_top, have_left);
                        b->ref[0] = 4 + dav1d_msac_decode_bool_adapt(&ts->msac,
                                            ts->cdf.m.ref[5][ctx3]);
                    }
                } else {
                    const int ctx2 = av1_get_ref_3_ctx(t->a, &t->l, by4, bx4,
                                                       have_top, have_left);
                    if (dav1d_msac_decode_bool_adapt(&ts->msac,
                                                     ts->cdf.m.ref[2][ctx2]))
                    {
                        const int ctx3 = av1_get_ref_5_ctx(t->a, &t->l, by4, bx4,
                                                           have_top, have_left);
                        b->ref[0] = 2 + dav1d_msac_decode_bool_adapt(&ts->msac,
                                            ts->cdf.m.ref[4][ctx3]);
                    } else {
                        const int ctx3 = av1_get_ref_4_ctx(t->a, &t->l, by4, bx4,
                                                           have_top, have_left);
                        b->ref[0] = dav1d_msac_decode_bool_adapt(&ts->msac,
                                        ts->cdf.m.ref[3][ctx3]);
                    }
                }
                if (DEBUG_BLOCK_INFO)
                    printf("Post-ref[%d]: r=%d\n", b->ref[0], ts->msac.rng);
            }
            b->ref[1] = -1;

            refmvs_candidate mvstack[8];
            int n_mvs, ctx;
            dav1d_refmvs_find(&t->rt, mvstack, &n_mvs, &ctx,
                              (union refmvs_refpair) { .ref = { b->ref[0] + 1, -1 }},
                              bs, 0, t->by, t->bx);

            // mode parsing and mv derivation from ref_mvs
            if ((seg && (seg->skip || seg->globalmv)) ||
                dav1d_msac_decode_bool_adapt(&ts->msac,
                                             ts->cdf.m.newmv_mode[ctx & 7]))
            {
                if ((seg && (seg->skip || seg->globalmv)) ||
                    !dav1d_msac_decode_bool_adapt(&ts->msac,
                         ts->cdf.m.globalmv_mode[(ctx >> 3) & 1]))
                {
                    b->inter_mode = GLOBALMV;
                    b->mv[0] = get_gmv_2d(&f->frame_hdr->gmv[b->ref[0]],
                                          t->bx, t->by, bw4, bh4, f->frame_hdr);
                    has_subpel_filter = imin(bw4, bh4) == 1 ||
                        f->frame_hdr->gmv[b->ref[0]].type == DAV1D_WM_TYPE_TRANSLATION;
                } else {
                    has_subpel_filter = 1;
                    if (dav1d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.refmv_mode[(ctx >> 4) & 15]))
                    { // NEAREST, NEARER, NEAR or NEARISH
                        b->inter_mode = NEARMV;
                        b->drl_idx = NEARER_DRL;
                        if (n_mvs > 2) { // NEARER, NEAR or NEARISH
                            const int drl_ctx_v2 = get_drl_context(mvstack, 1);
                            b->drl_idx += dav1d_msac_decode_bool_adapt(&ts->msac,
                                              ts->cdf.m.drl_bit[drl_ctx_v2]);
                            if (b->drl_idx == NEAR_DRL && n_mvs > 3) { // NEAR or NEARISH
                                const int drl_ctx_v3 =
                                    get_drl_context(mvstack, 2);
                                b->drl_idx += dav1d_msac_decode_bool_adapt(&ts->msac,
                                                  ts->cdf.m.drl_bit[drl_ctx_v3]);
                            }
                        }
                    } else {
                        b->inter_mode = NEARESTMV;
                        b->drl_idx = NEAREST_DRL;
                    }
                    assert(b->drl_idx >= NEAREST_DRL && b->drl_idx <= NEARISH_DRL);
                    b->mv[0] = mvstack[b->drl_idx].mv.mv[0];
                    if (b->drl_idx < NEAR_DRL)
                        fix_mv_precision(f->frame_hdr, &b->mv[0]);
                }

                if (DEBUG_BLOCK_INFO)
                    printf("Post-intermode[%d,drl=%d,mv=y:%d,x:%d,n_mvs=%d]: r=%d\n",
                           b->inter_mode, b->drl_idx, b->mv[0].y, b->mv[0].x, n_mvs,
                           ts->msac.rng);
            } else {
                has_subpel_filter = 1;
                b->inter_mode = NEWMV;
                b->drl_idx = NEAREST_DRL;
                if (n_mvs > 1) { // NEARER, NEAR or NEARISH
                    const int drl_ctx_v1 = get_drl_context(mvstack, 0);
                    b->drl_idx += dav1d_msac_decode_bool_adapt(&ts->msac,
                                      ts->cdf.m.drl_bit[drl_ctx_v1]);
                    if (b->drl_idx == NEARER_DRL && n_mvs > 2) { // NEAR or NEARISH
                        const int drl_ctx_v2 = get_drl_context(mvstack, 1);
                        b->drl_idx += dav1d_msac_decode_bool_adapt(&ts->msac,
                                          ts->cdf.m.drl_bit[drl_ctx_v2]);
                    }
                }
                assert(b->drl_idx >= NEAREST_DRL && b->drl_idx <= NEARISH_DRL);
                if (n_mvs > 1) {
                    b->mv[0] = mvstack[b->drl_idx].mv.mv[0];
                } else {
                    assert(!b->drl_idx);
                    b->mv[0] = mvstack[0].mv.mv[0];
                    fix_mv_precision(f->frame_hdr, &b->mv[0]);
                }
                if (DEBUG_BLOCK_INFO)
                    printf("Post-intermode[%d,drl=%d]: r=%d\n",
                           b->inter_mode, b->drl_idx, ts->msac.rng);
                const int mv_prec = f->frame_hdr->hp - f->frame_hdr->force_integer_mv;
                read_mv_residual(ts, &b->mv[0], mv_prec);
                if (DEBUG_BLOCK_INFO)
                    printf("Post-residualmv[mv=y:%d,x:%d]: r=%d\n",
                           b->mv[0].y, b->mv[0].x, ts->msac.rng);
            }

            // interintra flags
            const int ii_sz_grp = dav1d_ymode_size_context[bs];
            if (f->seq_hdr->motion_modes & 2 && //inter_intra &&
                interintra_allowed_mask & (1 << bs) &&
                dav1d_msac_decode_bool_adapt(&ts->msac,
                                             ts->cdf.m.interintra[ii_sz_grp]))
            {
                b->interintra_mode = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                         ts->cdf.m.interintra_mode[ii_sz_grp],
                                         N_INTER_INTRA_PRED_MODES - 1);
                const int wedge_ctx = dav1d_wedge_ctx_lut[bs];
                b->interintra_type = INTER_INTRA_BLEND +
                                     dav1d_msac_decode_bool_adapt(&ts->msac,
                                         ts->cdf.m.interintra_wedge[wedge_ctx]);
                if (b->interintra_type == INTER_INTRA_WEDGE)
                    b->wedge_idx = dav1d_msac_decode_symbol_adapt16(&ts->msac,
                                       ts->cdf.m.wedge_idx[wedge_ctx], 15);
            } else {
                b->interintra_type = INTER_INTRA_NONE;
            }
            if (DEBUG_BLOCK_INFO && f->seq_hdr->motion_modes & 2 && //inter_intra &&
                interintra_allowed_mask & (1 << bs))
            {
                printf("Post-interintra[t=%d,m=%d,w=%d]: r=%d\n",
                       b->interintra_type, b->interintra_mode,
                       b->wedge_idx, ts->msac.rng);
            }

            // motion variation
            if (f->frame_hdr->switchable_motion_mode &&
                b->interintra_type == INTER_INTRA_NONE && imin(bw4, bh4) >= 2 &&
                // is not warped global motion
                !(!f->frame_hdr->force_integer_mv && b->inter_mode == GLOBALMV &&
                  f->frame_hdr->gmv[b->ref[0]].type > DAV1D_WM_TYPE_TRANSLATION) &&
                // has overlappable neighbours
                ((have_left && findoddzero(&t->l.intra[by4 + 1], h4 >> 1)) ||
                 (have_top && findoddzero(&t->a->intra[bx4 + 1], w4 >> 1))))
            {
                // reaching here means the block allows obmc - check warp by
                // finding matching-ref blocks in top/left edges
                uint64_t mask[2] = { 0, 0 };
                find_matching_ref(t, 0, bw4, bh4, w4, h4,
                                  have_left, have_top, b->ref[0], mask);
                const int allow_warp = !f->svc[b->ref[0]][0].scale &&
                    !f->frame_hdr->force_integer_mv &&
                    f->frame_hdr->warp_motion && (mask[0] | mask[1]);

                b->motion_mode = allow_warp ?
                    dav1d_msac_decode_symbol_adapt4(&ts->msac,
                        ts->cdf.m.motion_mode[bs], 2) :
                    dav1d_msac_decode_bool_adapt(&ts->msac, ts->cdf.m.obmc[bs]);
                if (b->motion_mode == MM_WARP) {
                    has_subpel_filter = 0;
                    derive_warpmv(t, bw4, bh4, mask, b->mv[0], &t->warpmv);
#define signabs(v) v < 0 ? '-' : ' ', abs(v)
                    if (DEBUG_BLOCK_INFO)
                        printf("[ %c%x %c%x %c%x\n  %c%x %c%x %c%x ]\n"
                               "alpha=%c%x, beta=%c%x, gamma=%c%x, delta=%c%x, "
                               "mv=y:%d,x:%d\n",
                               signabs(t->warpmv.matrix[0]),
                               signabs(t->warpmv.matrix[1]),
                               signabs(t->warpmv.matrix[2]),
                               signabs(t->warpmv.matrix[3]),
                               signabs(t->warpmv.matrix[4]),
                               signabs(t->warpmv.matrix[5]),
                               signabs(t->warpmv.u.p.alpha),
                               signabs(t->warpmv.u.p.beta),
                               signabs(t->warpmv.u.p.gamma),
                               signabs(t->warpmv.u.p.delta),
                               b->mv[0].y, b->mv[0].x);
#undef signabs
                    if (t->frame_thread.pass) {
                        if (t->warpmv.type == DAV1D_WM_TYPE_AFFINE) {
                            b->matrix[0] = t->warpmv.matrix[2] - 0x10000;
                            b->matrix[1] = t->warpmv.matrix[3];
                            b->matrix[2] = t->warpmv.matrix[4];
                            b->matrix[3] = t->warpmv.matrix[5] - 0x10000;
                        } else {
                            b->matrix[0] = INT16_MIN;
                        }
                    }
                }

                if (DEBUG_BLOCK_INFO)
                    printf("Post-motionmode[%d]: r=%d [mask: 0x%" PRIx64 "/0x%"
                           PRIx64 "]\n", b->motion_mode, ts->msac.rng, mask[0],
                            mask[1]);
            } else {
                b->motion_mode = MM_TRANSLATION;
            }
        }

        // subpel filter
        enum Dav1dFilterMode filter[2];
        if (f->frame_hdr->subpel_filter_mode == DAV1D_FILTER_SWITCHABLE) {
            if (has_subpel_filter) {
                const int comp = b->comp_type != COMP_INTER_NONE;
                const int ctx1 = get_filter_ctx(t->a, &t->l, comp, 0, b->ref[0],
                                                by4, bx4);
                filter[0] = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                               ts->cdf.m.filter[0][ctx1],
                               DAV1D_N_SWITCHABLE_FILTERS - 1);
#if 0
                if (f->seq_hdr->dual_filter) {
                    const int ctx2 = get_filter_ctx(t->a, &t->l, comp, 1,
                                                    b->ref[0], by4, bx4);
                    if (DEBUG_BLOCK_INFO)
                        printf("Post-subpel_filter1[%d,ctx=%d]: r=%d\n",
                               filter[0], ctx1, ts->msac.rng);
                    filter[1] = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                    ts->cdf.m.filter[1][ctx2],
                                    DAV1D_N_SWITCHABLE_FILTERS - 1);
                    if (DEBUG_BLOCK_INFO)
                        printf("Post-subpel_filter2[%d,ctx=%d]: r=%d\n",
                               filter[1], ctx2, ts->msac.rng);
                } else
#endif
                {
                    filter[1] = filter[0];
                    if (DEBUG_BLOCK_INFO)
                        printf("Post-subpel_filter[%d,ctx=%d]: r=%d\n",
                               filter[0], ctx1, ts->msac.rng);
                }
            } else {
                filter[0] = filter[1] = DAV1D_FILTER_8TAP_REGULAR;
            }
        } else {
            filter[0] = filter[1] = f->frame_hdr->subpel_filter_mode;
        }
        b->filter2d = dav1d_filter_2d[filter[1]][filter[0]];

        read_vartx_tree(t, b, bs, bx4, by4);

        // reconstruction
        if (t->frame_thread.pass == 1) {
            f->bd_fn.read_coef_blocks(t, bs, b);
        } else {
            if (f->bd_fn.recon_b_inter(t, bs, b)) return -1;
        }

        if (f->frame_hdr->loopfilter.level_y[0] ||
            f->frame_hdr->loopfilter.level_y[1])
        {
            const int is_globalmv =
                b->inter_mode == (is_comp ? GLOBALMV_GLOBALMV : GLOBALMV);
            const uint8_t (*const lf_lvls)[8][2] = (const uint8_t (*)[8][2])
                &ts->lflvl[b->seg_id][0][b->ref[0] + 1][!is_globalmv];
            const uint16_t tx_split[2] = { b->tx_split0, b->tx_split1 };
            enum RectTxfmSize ytx = b->max_ytx, uvtx = b->uvtx;
            if (f->frame_hdr->segmentation.lossless[b->seg_id]) {
                ytx  = (enum RectTxfmSize) TX_4X4;
                uvtx = (enum RectTxfmSize) TX_4X4;
            }
            dav1d_create_lf_mask_inter(t->lf_mask, f->lf.level, f->b4_stride, lf_lvls,
                                       t->bx, t->by, f->w4, f->h4, b->skip_txfm,
                                       bs, ytx, tx_split, uvtx, f->cur.p.layout,
                                       &t->a->tx_lpf_y[bx4], &t->l.tx_lpf_y[by4],
                                       has_chroma ? &t->a->tx_lpf_uv[cbx4] : NULL,
                                       has_chroma ? &t->l.tx_lpf_uv[cby4] : NULL);
        }

        // context updates
        if (is_comp)
            splat_tworef_mv(f->c, t, bs, b, bw4, bh4);
        else
            splat_oneref_mv(f->c, t, bs, b, bw4, bh4);
        BlockContext *edge = t->a;
        for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
#define set_ctx(rep_macro) \
            rep_macro(edge->seg_pred, off, seg_pred); \
            rep_macro(edge->skip_mode, off, b->skip_mode); \
            rep_macro(edge->intra, off, 0); \
            rep_macro(edge->intrabc, off, 0); \
            rep_macro(edge->midx, off, 0xff); \
            rep_macro(edge->fsc, off, 0); \
            rep_macro(edge->skip_txfm, off, b->skip_txfm); \
            rep_macro(edge->pal_sz, off, 0); \
            /* see aomedia bug 2183 for why this is outside if (has_chroma) */ \
            rep_macro(t->pal_sz_uv[i], off, 0); \
            rep_macro(edge->comp_type, off, b->comp_type); \
            rep_macro(edge->filter[0], off, filter[0]); \
            rep_macro(edge->filter[1], off, filter[1]); \
            rep_macro(edge->mode, off, b->inter_mode); \
            rep_macro(edge->mrl, off, 0); \
            rep_macro(edge->multi_mrl, off, 0); \
            rep_macro(edge->dip, off, 0); \
            rep_macro(edge->ref[0], off, b->ref[0]); \
            rep_macro(edge->ref[1], off, ((uint8_t) b->ref[1]))
            case_set(b_dim[2 + i]);
#undef set_ctx
        }
        if (has_chroma) {
            dav1d_memset_pow2[ulog2(cbw4)](&t->a->uvmode[cbx4], DC_PRED);
            dav1d_memset_pow2[ulog2(cbh4)](&t->l.uvmode[cby4], DC_PRED);
        }
    }

    // update contexts
    if (f->frame_hdr->segmentation.enabled &&
        f->frame_hdr->segmentation.update_map)
    {
        uint8_t *seg_ptr = &f->cur_segmap[t->by * f->b4_stride + t->bx];
#define set_ctx(rep_macro) \
        for (int y = 0; y < bh4; y++) { \
            rep_macro(seg_ptr, 0, b->seg_id); \
            seg_ptr += f->b4_stride; \
        }
        case_set(b_dim[2]);
#undef set_ctx
    }
    if (!b->skip_txfm) {
        uint16_t (*noskip_mask)[2] = &t->lf_mask->noskip_mask[by4 >> 1];
        const unsigned mask = (~0U >> (32 - bw4)) << (bx4 & 15);
        const int bx_idx = (bx4 & 16) >> 4;
        for (int y = 0; y < bh4; y += 2, noskip_mask++) {
            (*noskip_mask)[bx_idx] |= mask;
            if (bw4 == 32) // this should be mask >> 16, but it's 0xffffffff anyway
                (*noskip_mask)[1] |= mask;
        }
    }
    if (f->seq_hdr->sdp && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400 &&
        cbs == BS_INVALID)
    {
        uint8_t *dirmap = &t->luma_intra_dir_mode_map[by4 * 32 + bx4];
#define set_ctx(rep_macro) \
        for (int y = 0; y < bh4; y++) { \
            rep_macro(dirmap, 0, midx); \
            dirmap += 32; \
        }
        case_set(b_dim[2]);
#undef set_ctx
    }

    if (t->frame_thread.pass == 1 && !b->intra && IS_INTER_OR_SWITCH(f->frame_hdr)) {
        const int sby = (t->by - ts->tiling.row_start) >> f->sb_shift;
        int (*const lowest_px)[2] = ts->lowest_pixel[sby];

        // keep track of motion vectors for each reference
        if (b->comp_type == COMP_INTER_NONE) {
            // y
            if (imin(bw4, bh4) > 1 &&
                ((b->inter_mode == GLOBALMV && f->gmv_warp_allowed[b->ref[0]]) ||
                 (b->motion_mode == MM_WARP && t->warpmv.type > DAV1D_WM_TYPE_TRANSLATION)))
            {
                affine_lowest_px_luma(t, &lowest_px[b->ref[0]][0], b_dim,
                                      b->motion_mode == MM_WARP ? &t->warpmv :
                                      &f->frame_hdr->gmv[b->ref[0]]);
            } else {
                mc_lowest_px(&lowest_px[b->ref[0]][0], t->by, bh4, b->mv[0].y,
                             0, &f->svc[b->ref[0]][1]);
                if (b->motion_mode == MM_OBMC) {
                    obmc_lowest_px(t, lowest_px, 0, b_dim, bx4, by4, w4, h4);
                }
            }

            // uv
            if (has_chroma) {
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
                    if (bw4 == 1 && bh4 == ss_ver) {
                        const refmvs_block *const rr = &r[-1][t->bx - 1];
                        mc_lowest_px(&lowest_px[rr->ref.ref[0] - 1][1],
                                     t->by - 1, bh4, rr->mv.mv[0].y, ss_ver,
                                     &f->svc[rr->ref.ref[0] - 1][1]);
                    }
                    if (bw4 == 1) {
                        const refmvs_block *const rr = &r[0][t->bx - 1];
                        mc_lowest_px(&lowest_px[rr->ref.ref[0] - 1][1],
                                     t->by, bh4, rr->mv.mv[0].y, ss_ver,
                                     &f->svc[rr->ref.ref[0] - 1][1]);
                    }
                    if (bh4 == ss_ver) {
                        const refmvs_block *const rr = &r[-1][t->bx];
                        mc_lowest_px(&lowest_px[rr->ref.ref[0] - 1][1],
                                     t->by - 1, bh4, rr->mv.mv[0].y, ss_ver,
                                     &f->svc[rr->ref.ref[0] - 1][1]);
                    }
                    mc_lowest_px(&lowest_px[b->ref[0]][1], t->by, bh4,
                                 b->mv[0].y, ss_ver, &f->svc[b->ref[0]][1]);
                } else {
                    if (imin(cbw4, cbh4) > 1 &&
                        ((b->inter_mode == GLOBALMV && f->gmv_warp_allowed[b->ref[0]]) ||
                         (b->motion_mode == MM_WARP && t->warpmv.type > DAV1D_WM_TYPE_TRANSLATION)))
                    {
                        affine_lowest_px_chroma(t, &lowest_px[b->ref[0]][1], b_dim,
                                                b->motion_mode == MM_WARP ? &t->warpmv :
                                                &f->frame_hdr->gmv[b->ref[0]]);
                    } else {
                        mc_lowest_px(&lowest_px[b->ref[0]][1],
                                     t->by & ~ss_ver, bh4 << (bh4 == ss_ver),
                                     b->mv[0].y, ss_ver, &f->svc[b->ref[0]][1]);
                        if (b->motion_mode == MM_OBMC) {
                            obmc_lowest_px(t, lowest_px, 1, b_dim, bx4, by4, w4, h4);
                        }
                    }
                }
            }
        } else {
            // y
            for (int i = 0; i < 2; i++) {
                if (b->inter_mode == GLOBALMV_GLOBALMV && f->gmv_warp_allowed[b->ref[i]]) {
                    affine_lowest_px_luma(t, &lowest_px[b->ref[i]][0], b_dim,
                                          &f->frame_hdr->gmv[b->ref[i]]);
                } else {
                    mc_lowest_px(&lowest_px[b->ref[i]][0], t->by, bh4,
                                 b->mv[i].y, 0, &f->svc[b->ref[i]][1]);
                }
            }

            // uv
            if (has_chroma) for (int i = 0; i < 2; i++) {
                if (b->inter_mode == GLOBALMV_GLOBALMV &&
                    imin(cbw4, cbh4) > 1 && f->gmv_warp_allowed[b->ref[i]])
                {
                    affine_lowest_px_chroma(t, &lowest_px[b->ref[i]][1], b_dim,
                                            &f->frame_hdr->gmv[b->ref[i]]);
                } else {
                    mc_lowest_px(&lowest_px[b->ref[i]][1], t->by, bh4,
                                 b->mv[i].y, ss_ver, &f->svc[b->ref[i]][1]);
                }
            }
        }
    }

    return 0;
}

#if __has_feature(memory_sanitizer)

#include <sanitizer/msan_interface.h>

static int checked_decode_b(Dav1dTaskContext *const t, const enum BlockSize bs) {
    const Dav1dFrameContext *const f = t->f;
    const int err = decode_b(t, bs);

    if (err == 0 && !(t->frame_thread.pass & 1)) {
        const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
        const int ss_hor = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
        const uint8_t *const b_dim = dav1d_block_dimensions[bs];
        const int bw4 = b_dim[0], bh4 = b_dim[1];
        const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
        const int has_chroma = f->seq_hdr->layout != DAV1D_PIXEL_LAYOUT_I400 &&
                               (bw4 > ss_hor || t->bx & 1) &&
                               (bh4 > ss_ver || t->by & 1);

        for (int p = 0; p < 1 + 2 * has_chroma; p++) {
            const int ss_ver = p && f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
            const int ss_hor = p && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
            const ptrdiff_t stride = f->cur.stride[!!p];
            const int bx = t->bx & ~ss_hor;
            const int by = t->by & ~ss_ver;
            const int width  = w4 << (2 - ss_hor + (bw4 == ss_hor));
            const int height = h4 << (2 - ss_ver + (bh4 == ss_ver));

            const uint8_t *data = f->cur.data[p] + (by << (2 - ss_ver)) * stride +
                                  (bx << (2 - ss_hor + !!f->seq_hdr->hbd));

            for (int y = 0; y < height; data += stride, y++) {
                const size_t line_sz = width << !!f->seq_hdr->hbd;
                if (__msan_test_shadow(data, line_sz) != -1) {
                    fprintf(stderr, "B[%d](%d, %d) w4:%d, h4:%d, row:%d\n",
                            p, bx, by, w4, h4, y);
                    __msan_check_mem_is_initialized(data, line_sz);
                }
            }
        }
    }

    return err;
}

#define decode_b checked_decode_b

#endif /* defined(__has_feature) */

static int decode_sb(Dav1dTaskContext *const t, DB_ONLY(const int depth)
                     const enum BlockSize lbs, const enum BlockSize cbs)
{
    const enum BlockSize bs = lbs == BS_INVALID ? cbs : lbs;
    assert(bs != BS_INVALID);
    const Dav1dFrameContext *const f = t->f;
    Dav1dTileState *const ts = t->ts;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int hw4 = bw4 >> 1, hh4 = bh4 >> 1;
    const int qw4 = hw4 >> 1, qh4 = hh4 >> 1;
    const int have_h_split = f->bw > t->bx + hw4;
    const int have_v_split = f->bh > t->by + hh4;

    // key/intraonly frames always apply SDP at the 64x64 boundary
    if (lbs == BS_64x64 && cbs == BS_64x64 &&
        f->seq_hdr->sdp && !(f->frame_hdr->frame_type & 1))
    {
        if (decode_sb(t, DB_ONLY(depth) lbs, BS_INVALID)) return -1;
        return decode_sb(t, DB_ONLY(depth) BS_INVALID, cbs);
    }

    DEBUG_BLOCK_printf("%*sdecode_sb[y=%d,x=%d,bs=%dx%d,plane=%s]: r=%d\n",
                       depth - 1, "", t->by, t->bx, bw4 * 4, bh4 * 4,
                       cbs == BS_INVALID ? "y" : lbs == BS_INVALID ? "uv" : "yuv",
                       ts->msac.rng);

    static const struct PartitionConstants {
        // FIXME part[0][split] and part[1][split] are identical, maybe
        // we can save a byte by merging these together
        int8_t part[2 /* h, v */][4 /* half, quarter, eighth, split */];
        int8_t ctx[2 /* _, direction */];
    } subb[] = {
        [BS_256x256] = {
            { { BS_256x128, -1, -1, BS_128x128 },
              { BS_128x256, -1, -1, BS_128x128 } },
            { 9, 12 },
        }, [BS_256x128] = {
            { { -1, -1, -1, -1 },
              { BS_128x128, -1, -1, -1 } },
            { 8, -1 /* 11 */ },
        }, [BS_128x256] = {
            { { BS_128x128, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 7, -1 /* 10 */ },
        }, [BS_128x128] = {
            { { BS_128x64, -1, -1, BS_64x64 },
              { BS_64x128, -1, -1, BS_64x64 } },
            { 6, 9 },
        }, [BS_128x64] = {
            { { -1, -1, -1, -1 },
              { BS_64x64, -1, -1, -1 } },
            { 5, -1 /* 8 */ },
        }, [BS_64x128] = {
            { { BS_64x64, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 4, -1 /* 7 */ },
        }, [BS_64x64] = {
            { { BS_64x32, BS_64x16, BS_64x8, BS_32x32 },
              { BS_32x64, BS_16x64, BS_8x64, BS_32x32 } },
            { 3, 6 },
        }, [BS_64x32] = {
            { { BS_64x16, BS_64x8, BS_64x4, BS_32x16 },
              { BS_32x32, BS_16x32, BS_8x32, BS_32x16 } },
            { 3, 5 },
        }, [BS_64x16] = {
            { { BS_64x8, BS_64x4, -1, BS_32x8 },
              { BS_32x16, BS_16x16, BS_8x16, BS_32x8 } },
            { 15, 14 },
        }, [BS_64x8] = {
            { { -1, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 0, 0 },
        }, [BS_64x4] = {
            { { -1, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 0, -1 },
        }, [BS_32x64] = {
            { { BS_32x32, BS_32x16, BS_32x8, BS_16x32 },
              { BS_16x64, BS_8x64, BS_4x64, BS_16x32 } },
            { 3, 4 },
        }, [BS_32x32] = {
            { { BS_32x16, BS_32x8, BS_32x4, BS_16x16 },
              { BS_16x32, BS_8x32, BS_4x32, BS_16x16 } },
            { 2, 3 },
        }, [BS_32x16] = {
            { { BS_32x8, BS_32x4, -1, BS_16x8 },
              { BS_16x16, BS_8x16, BS_4x16, BS_16x8 } },
            { 2, 2 },
        }, [BS_32x8] = {
            { { BS_32x4, -1, -1, -1 },
              { BS_16x8, BS_8x8, BS_4x8, BS_16x4 } },
            { 13, 14 },
        }, [BS_32x4] = {
            { { -1, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 0, -1 },
        }, [BS_16x64] = {
            { { BS_16x32, BS_16x16, BS_16x8, BS_8x32 },
              { BS_8x64, BS_4x64, -1, BS_8x32 } },
            { 14, 13 },
        }, [BS_16x32] = {
            { { BS_16x16, BS_16x8, BS_16x4, BS_8x16 },
              { BS_8x32, BS_4x32, -1, BS_8x16 } },
            { 2, 1 },
        }, [BS_16x16] = {
            { { BS_16x8, BS_16x4, -1, BS_8x8 },
              { BS_8x16, BS_4x16, -1, BS_8x8 } },
            { 1, 0 },
        }, [BS_16x8] = {
            { { BS_16x4, -1, -1, -1 },
              { BS_8x8, BS_4x8, -1, BS_8x4 } },
            { 1, 2 },
        }, [BS_16x4] = {
            { { -1, -1, -1, -1 },
              { BS_8x4, -1, -1, -1 } },
            { 11, 14 },
        }, [BS_8x64] = {
            { { -1, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 0, 0 },
        }, [BS_8x32] = {
            { { BS_8x16, BS_8x8, BS_8x4, BS_4x16 },
              { BS_4x32, -1, -1, -1 } },
            { 12, 13 },
        }, [BS_8x16] = {
            { { BS_8x8, BS_8x4, -1, BS_4x8 },
              { BS_4x16, -1, -1, -1 } },
            { 1, 1 },
        }, [BS_8x8] = {
            { { BS_8x4, -1, -1, -1 },
              { BS_4x8, -1, -1, -1 } },
            { 0, 0 },
        }, [BS_8x4] = {
            { { -1, -1, -1, -1 },
              { BS_4x4, -1, -1, -1 } },
            { 0, -1 },
        }, [BS_4x64] = {
            { { -1, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 0, -1 },
        }, [BS_4x32] = {
            { { -1, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 0, -1 },
        }, [BS_4x16] = {
            { { BS_4x8, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 10, -1 },
        }, [BS_4x8] = {
            { { BS_4x4, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { 0, -1 },
        }, [BS_4x4] = {
            { { -1, -1, -1, -1 },
              { -1, -1, -1, -1 } },
            { -1, -1 },
        },
    };
    const int pl = lbs == BS_INVALID;
    const struct PartitionConstants *const pcc = &subb[bs];
    enum BlockPartition bp = PARTITION_INVALID;
    int bx4, by4;
    if (t->frame_thread.pass != 2) {
        bx4 = t->bx & 31;
        by4 = t->by & 31;
        // FIXME some of the code below needs to be tested for 4:2:2 w/ SDP=1
        const int eff_ss_ver = f->ss_ver & (lbs == BS_INVALID);
        const int eff_ss_hor = f->ss_hor & (lbs == BS_INVALID);
        const int bwh4ss[2] = { bw4 >> eff_ss_hor, bh4 >> eff_ss_ver };
        assert(bwh4ss[0] >= 1 && bwh4ss[1] >= 1);
        if (imax(bwh4ss[0], bwh4ss[1]) == 1 ||
            // 1:8/1:16 partitions don't recursive (normatively)
            (pcc->part[0][0] & pcc->part[1][0]) == -1)
        {
            bp = PARTITION_NONE;
        } else if (!have_h_split || !have_v_split) {
            if (bw4 == bh4) {
                bp = !have_v_split ? PARTITION_H : PARTITION_V;
            } else if (bw4 > bh4) {
                if (!have_h_split || f->bh <= t->by + qh4)
                    bp = PARTITION_V;
            } else if (bh4 > bw4) {
                if (!have_v_split || f->bw <= t->bx + qw4)
                    bp = PARTITION_H;
            }
        }
        if (bp == PARTITION_INVALID) {
#if DEBUG_BLOCK_INFO
            if (0 && bs == (f->seq_hdr->sb128 == 2 ? BS_256x256 :
                            f->seq_hdr->sb128 == 1 ? BS_128x128 : BS_64x64))
                printf("poc=%d,y=%d,x=%d,bs=%d,r=%d\n",
                       f->frame_hdr->frame_offset, t->by, t->bx, bs, ts->msac.rng);
#endif
            const int ctx1 = get_partition_ctx(t->a, &t->l, b_dim, pl, by4, bx4);
            const int ctx2 = ctx1 + pcc->ctx[0] * 4;
            const int is_split =
                dav1d_msac_decode_bool_adapt(&ts->msac,
                                             ts->cdf.m.part_split[pl][ctx2]);
            if (!is_split) {
                bp = PARTITION_NONE;
            } else {
                if ((bs == BS_128x128 || bs == BS_256x256) &&
                    have_v_split && have_h_split)
                {
                    assert(lbs == cbs || f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I400);
                    const int ctx3 = ctx1 + (bs == BS_256x256) * 4;
                    const int is_square =
                        dav1d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.part_square[ctx3]);
                    if (is_square)
                        bp = PARTITION_SPLIT;
                } else if (imax(bw4, bh4) >= 32) {
                    assert(lbs == cbs || f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I400);
                    assert(bw4 != bh4);
                    bp = bw4 > bh4 ? PARTITION_V : PARTITION_H;
                }
                if (bp == PARTITION_INVALID) {
                    // split - find direction
                    int dir;
                    const int aspect = 1 << f->seq_hdr->max_pb_aspect_ratio_log2;
                    assert(bw4 * aspect >= bh4 && bh4 * aspect >= bw4);
                    const int v_aspect = bw4 * aspect >= bh4 * 2;
                    const int h_aspect = bh4 * aspect >= bw4 * 2;
                    assert(v_aspect || h_aspect);
                    if (imin(bwh4ss[0], bwh4ss[1]) == 1) {
                        dir = bwh4ss[0] > bwh4ss[1];
                    } else if (!(v_aspect && h_aspect)) {
                        dir = v_aspect;
                    } else {
                        const int ctx4 = ctx1 + pcc->ctx[1] * 4;
                        dir = dav1d_msac_decode_bool_adapt(&ts->msac,
                                  ts->cdf.m.part_dir[pl][ctx4]);
                    }
                    assert(pcc->part[dir][0] != -1);
                    bp = dir ? PARTITION_V : PARTITION_H;

                    if (imax(bw4, bh4) <= 16) {
                        // v3/h3 [ext-partition]
                        const int has_hv3 = f->seq_hdr->ext_partitions &&
                                            bwh4ss[!dir] >= 4 && bwh4ss[dir] >= 2 &&
                                            b_dim[!dir] * aspect >= b_dim[dir] * 4;
                        const int has_hv4ab = f->seq_hdr->uneven_4way_partitions &&
                                              bwh4ss[!dir] >= 8 &&
                                              b_dim[!dir] * aspect >= b_dim[dir] * 8;
                        if (has_hv3 || has_hv4ab) {
                            assert(pcc->part[dir][1] != -1);
                            const int ctx5 = get_partition2_ctx(t->a, &t->l, b_dim,
                                                                pl, dir, by4, bx4);
                            const int ctx6 = ctx5 + pcc->ctx[0] * 4;
                            const int is_ext =
                                dav1d_msac_decode_bool_adapt(&ts->msac,
                                    ts->cdf.m.part_ext[pl][ctx6]);
                            if (is_ext) {
                                bp = dir ? PARTITION_V3 : PARTITION_H3;
                                if (has_hv4ab) {
                                    assert(pcc->part[dir][2] != -1);
                                    const int is_4way = !has_hv3 ||
                                        dav1d_msac_decode_bool_adapt(&ts->msac,
                                            ts->cdf.m.part_4way[pl][ctx6]);
                                    if (is_4way) {
                                        const int is_a_or_b =
                                            dav1d_msac_decode_bool_bypass(&ts->msac);
                                        bp = PARTITION_H4A + dir * 2 + is_a_or_b;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
#if DEBUG_BLOCK_INFO
        static const char *const names[] = {
            [PARTITION_NONE]="none",
            [PARTITION_H4A]="h4a",
            [PARTITION_H4B]="h4b",
            [PARTITION_V4A]="v4a",
            [PARTITION_V4B]="v4b",
            [PARTITION_SPLIT]="s",
            [PARTITION_H]="h",
            [PARTITION_V]="v",
            [PARTITION_H3]="h3",
            [PARTITION_V3]="v3",
        };
        DEBUG_BLOCK_printf("%*sread_partition[y=%d,x=%d,bs=%dx%d,bp=%d|%s]: r=%d\n",
                           depth, "", t->by, t->bx, 4 * bw4, 4 * bh4, bp,
                           names[bp], ts->msac.rng);
#endif
    } else {
        //.. FIXME 2-pass decoding
        abort();
    }

    if (bs == cbs) {
        t->cbx = t->bx;
        t->cby = t->by;
    }
    switch (bp) {
    case PARTITION_NONE:
        if (decode_b(t, DB_ONLY(depth + 1) lbs, cbs)) return -1;
        if (t->frame_thread.pass != 2) {
#define set_ctx(rep_macro) \
            rep_macro(edge->partition[pl], off, (uint8_t) ~(b_dim[i] - 1))
            BlockContext *edge = t->a;
            for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
                case_set(b_dim[2 + i]);
            }
#undef set_ctx
        }
        break;
    case PARTITION_V: {
        assert(hw4 > 0);
        const int sub4 = bs == cbs && (hw4 >> f->ss_hor) > 0;
        assert(sub4 || !pl);
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][0],
                      sub4 ? pcc->part[1][0] : BS_INVALID))
        {
            return -1;
        }
        if (t->bx + hw4 >= f->bw) break;
        t->bx += hw4;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][0],
                      sub4 ? pcc->part[1][0] : cbs))
        {
            return -1;
        }
        t->bx -= hw4;
        break;
    }
    case PARTITION_H: {
        assert(hh4 > 0);
        const int sub4 = bs == cbs && (hh4 >> f->ss_ver) > 0;
        assert(sub4 || !pl);
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][0],
                      sub4 ? pcc->part[0][0] : BS_INVALID))
        {
            return -1;
        }
        if (t->by + hh4 >= f->bh) break;
        t->by += hh4;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][0],
                      sub4 ? pcc->part[0][0] : cbs))
        {
            return -1;
        }
        t->by -= hh4;
        break;
    }
    case PARTITION_SPLIT: {
        assert(have_v_split && have_h_split && cbs == lbs);
        const enum BlockSize sbs = pcc->part[0][3];
        if (decode_sb(t, DB_ONLY(depth + 1) sbs, sbs)) return -1;
        t->bx += hw4;
        if (decode_sb(t, DB_ONLY(depth + 1) sbs, sbs)) return -1;
        t->bx -= hw4;
        t->by += hh4;
        if (decode_sb(t, DB_ONLY(depth + 1) sbs, sbs)) return -1;
        t->bx += hw4;
        if (decode_sb(t, DB_ONLY(depth + 1) sbs, sbs)) return -1;
        t->bx -= hw4;
        t->by -= hh4;
        break;
    }
    case PARTITION_V3: {
        assert(qw4 > 0 && hh4 > 0);
        const int sub4 = bs == cbs && (qw4 >> f->ss_hor) > 0 &&
                                      (hh4 >> f->ss_ver) > 0;
        assert(sub4 || !pl);
        const int i_3only = !sub4 && bs != BS_32x8;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][1],
                      i_3only ? BS_INVALID : pcc->part[1][1]))
        {
            return -1;
        }
        if (t->bx + qw4 >= f->bw) break;
        t->bx += qw4;
        if (bs == cbs) t->cbx = t->bx;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][3],
                      sub4 ? pcc->part[1][3] : BS_INVALID))
        {
            return -1;
        }
        if (t->by + hh4 < f->bh) {
            t->by += hh4;
            if (decode_sb(t, DB_ONLY(depth + 1)
                          pl ? BS_INVALID : pcc->part[1][3],
                          i_3only ? BS_INVALID : pcc->part[1][sub4 * 3]))
            {
                return -1;
            }
            t->by -= hh4;
        }
        if (t->bx + hw4 >= f->bw) { t->bx -= qw4; break; }
        t->bx += hw4;
        if (bs == cbs) t->cbx = t->bx;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][1],
                      i_3only ? cbs : pcc->part[1][1]))
        {
            return -1;
        }
        t->bx -= 3 * qw4;
        break;
    }
    case PARTITION_H3: {
        assert(qh4 > 0 && hw4 > 0);
        const int sub4 = bs == cbs && (qh4 >> f->ss_ver) > 0 &&
                                      (hw4 >> f->ss_hor) > 0;
        assert(sub4 || !pl);
        const int i_3only = !sub4 && bs != BS_8x32;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][1],
                      i_3only ? BS_INVALID : pcc->part[0][1]))
        {
            return -1;
        }
        if (t->by + qh4 >= f->bh) break;
        t->by += qh4;
        if (bs == cbs) t->cby = t->by;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][3],
                      sub4 ? pcc->part[0][3] : BS_INVALID))
        {
            return -1;
        }
        if (t->bx + hw4 < f->bw) {
            t->bx += hw4;
            if (decode_sb(t, DB_ONLY(depth + 1)
                          pl ? BS_INVALID : pcc->part[0][3],
                          i_3only ? BS_INVALID : pcc->part[0][sub4 * 3]))
            {
                return -1;
            }
            t->bx -= hw4;
        }
        if (t->by + hh4 >= f->bh) { t->by -= qh4; break; }
        t->by += hh4;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][1],
                      i_3only ? cbs : pcc->part[0][1]))
        {
            return -1;
        }
        t->by -= 3 * qh4;
        break;
    }
    case PARTITION_V4A:
    case PARTITION_V4B: {
        const int ew4 = qw4 >> 1;
        assert(ew4 > 0);
        const int sub4 = bs == cbs && (ew4 >> f->ss_hor) > 0;
        assert(sub4 || !pl);
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][2],
                      sub4 ? pcc->part[1][2] : BS_INVALID))
        {
            return -1;
        }
        if (t->bx + ew4 >= f->bw) break;
        t->bx += ew4;
        const int var = bp - PARTITION_V4A; // v4b: 1, v4a: 0
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][!var],
                      sub4 ? pcc->part[1][!var] : -1))
        {
            return -1;
        }
        const int w4a = qw4 << var, w4b = hw4 >> var;
        if (t->bx + w4a >= f->bw) { t->bx -= ew4; break; }
        t->bx += w4a;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][var],
                      sub4 ? pcc->part[1][var] : -1))
        {
            return -1;
        }
        if (t->bx + w4b >= f->bw) { t->bx -= ew4 + w4a; break; }
        t->bx += w4b;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][2],
                      sub4 ? pcc->part[1][2] : cbs))
        {
            return -1;
        }
        t->bx -= 7 * ew4;
        break;
    }
    case PARTITION_H4A:
    case PARTITION_H4B: {
        const int eh4 = bs == cbs && qh4 >> 1;
        assert(eh4 > 0);
        const int sub4 = (eh4 >> f->ss_ver) > 0;
        assert(sub4 || !pl);
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][2],
                      sub4 ? pcc->part[0][2] : BS_INVALID))
        {
            return -1;
        }
        if (t->by + eh4 >= f->bh) break;
        t->by += eh4;
        const int var = bp - PARTITION_H4A; // h4b: 1, h4a: 0
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][!var],
                      sub4 ? pcc->part[0][!var] : -1))
        {
            return -1;
        }
        const int h4a = qh4 << var, h4b = hh4 >> var;
        if (t->by + h4a >= f->bh) { t->by -= eh4; break; }
        t->by += h4a;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][var],
                      sub4 ? pcc->part[0][var] : -1))
        {
            return -1;
        }
        if (t->by + h4b >= f->bh) { t->by -= eh4 + h4a; break; }
        t->by += h4b;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][2],
                      sub4 ? pcc->part[0][2] : cbs))
        {
            return -1;
        }
        t->by -= 7 * eh4;
        break;
    }
    default:
        assert(0);
    }

    return 0;
}

static void reset_context(BlockContext *const ctx, const int keyframe, const int pass) {
    memset(ctx->midx, 0xff, sizeof(ctx->midx));
    memset(ctx->intra, keyframe, sizeof(ctx->intra));
    memset(ctx->uvmode, DC_PRED, sizeof(ctx->uvmode));
    if (keyframe)
        memset(ctx->mode, DC_PRED, sizeof(ctx->mode));

    if (pass == 2) return;

    memset(ctx->partition, 0, sizeof(ctx->partition));
    memset(ctx->skip_txfm, 0, sizeof(ctx->skip_txfm));
    memset(ctx->skip_mode, 0, sizeof(ctx->skip_mode));
    memset(ctx->tx_lpf_y, 2, sizeof(ctx->tx_lpf_y));
    memset(ctx->tx_lpf_uv, 1, sizeof(ctx->tx_lpf_uv));
    memset(ctx->tx, TX_64X64, sizeof(ctx->tx));
    if (!keyframe) {
        memset(ctx->ref, -1, sizeof(ctx->ref));
        memset(ctx->comp_type, 0, sizeof(ctx->comp_type));
        memset(ctx->mode, NEARESTMV, sizeof(ctx->mode));
    }
    memset(ctx->mrl, 0, sizeof(ctx->mrl));
    memset(ctx->lcoef, 0x40, sizeof(ctx->lcoef));
    memset(ctx->ccoef, 0x40, sizeof(ctx->ccoef));
    memset(ctx->filter, DAV1D_N_SWITCHABLE_FILTERS, sizeof(ctx->filter));
    memset(ctx->seg_pred, 0, sizeof(ctx->seg_pred));
    memset(ctx->pal_sz, 0, sizeof(ctx->pal_sz));
}

// { Y+U+V, Y+U } * 4
static const uint8_t ss_size_mul[4][2] = {
    [DAV1D_PIXEL_LAYOUT_I400] = {  4, 4 },
    [DAV1D_PIXEL_LAYOUT_I420] = {  6, 5 },
    [DAV1D_PIXEL_LAYOUT_I422] = {  8, 6 },
    [DAV1D_PIXEL_LAYOUT_I444] = { 12, 8 },
};

static void setup_tile(Dav1dTileState *const ts,
                       const Dav1dFrameContext *const f,
                       const uint8_t *const data, const size_t sz,
                       const int tile_row, const int tile_col,
                       const unsigned tile_start_off)
{
    const int col_sb_start = f->frame_hdr->tiling.col_start_sb[tile_col];
    const int col_sb_end = f->frame_hdr->tiling.col_start_sb[tile_col + 1];
    const int row_sb_start = f->frame_hdr->tiling.row_start_sb[tile_row];
    const int row_sb_end = f->frame_hdr->tiling.row_start_sb[tile_row + 1];
    const int sb_shift = f->sb_shift;

    const uint8_t *const size_mul = ss_size_mul[f->cur.p.layout];
    for (int p = 0; p < 2; p++) {
        ts->frame_thread[p].pal_idx = f->frame_thread.pal_idx ?
            &f->frame_thread.pal_idx[(size_t)tile_start_off * size_mul[1] / 8] :
            NULL;
        ts->frame_thread[p].cbi = f->frame_thread.cbi ?
            &f->frame_thread.cbi[(size_t)tile_start_off * size_mul[0] / 64] :
            NULL;
        ts->frame_thread[p].cf = f->frame_thread.cf ?
            (uint8_t*)f->frame_thread.cf +
                (((size_t)tile_start_off * size_mul[0]) >> !f->seq_hdr->hbd) :
            NULL;
    }

    dav1d_cdf_thread_copy(&ts->cdf, &f->in_cdf);
    ts->last_qidx = f->frame_hdr->quant.yac;
    ts->last_delta_lf.u32 = 0;

    dav1d_msac_init(&ts->msac, data, sz, f->frame_hdr->disable_cdf_update);
#if DEBUG_BLOCK_INFO
    struct { int by, bx; } tmem = {
        .by = row_sb_start << sb_shift, .bx = col_sb_start << sb_shift,
    }, *const t = &tmem;
    DEBUG_BLOCK_printf("decode_tile[tilerow=%d/col=%d,y=%d-%d,x=%d-%d,size=%td]: r=%d\n",
                       tile_row, tile_col,
                       row_sb_start << sb_shift,
                       imin(row_sb_end << sb_shift, f->bh),
                       col_sb_start << sb_shift,
                       imin(col_sb_end << sb_shift, f->bw),
                       sz, ts->msac.rng);
#endif

    ts->tiling.row = tile_row;
    ts->tiling.col = tile_col;
    ts->tiling.col_start = col_sb_start << sb_shift;
    ts->tiling.col_end = imin(col_sb_end << sb_shift, f->bw);
    ts->tiling.row_start = row_sb_start << sb_shift;
    ts->tiling.row_end = imin(row_sb_end << sb_shift, f->bh);

    for (int pl = 0; pl < 3; pl++) {
        if (f->frame_hdr->restoration.p[pl].type == DAV1D_RESTORATION_NS_WIENER ||
            f->frame_hdr->restoration.p[pl].type == DAV1D_RESTORATION_SWITCHABLE)
        {
            struct NsWienerBank *const bank = &ts->ns_wiener_bank[pl];
            const int8_t (*const cf_range)[2] = pl ? dav1d_ns_wiener_coef_range_uv :
                                                     dav1d_ns_wiener_coef_range_y;
            memset(bank->bank_size, 0, sizeof(bank->bank_size));
            memset(bank->bank_idx, 0, sizeof(bank->bank_idx));
            const int n_classes = f->frame_hdr->restoration.p[pl].ns.num_classes;
            for (int n = 0; n < n_classes; n++) {
                for (int m = 0; m < 16 + !!pl * 2; m++) {
                    bank->filter[0][n][m] = cf_range[m][1] +
                                            ((1 << cf_range[m][0]) >> 1);
                }
            }
        }
    }

    if (f->c->n_tc > 1) {
        for (int p = 0; p < 2; p++)
            atomic_init(&ts->progress[p], row_sb_start);
    }
}

static void read_restoration_info(Dav1dTaskContext *const t,
                                  Av1RestorationUnit *const lr, const int p,
                                  const enum Dav1dRestorationType frame_type)
{
    const Dav1dFrameContext *const f = t->f;
    Dav1dTileState *const ts = t->ts;

    if (frame_type == DAV1D_RESTORATION_SWITCHABLE) {
        assert(!p);
        if (dav1d_msac_decode_bool_adapt(&ts->msac, ts->cdf.m.rst_switchable[0])) {
            lr->type = DAV1D_RESTORATION_NONE;
        } else {
            const int type = dav1d_msac_decode_bool_adapt(&ts->msac,
                                 ts->cdf.m.rst_switchable[1]);
            lr->type = type ? DAV1D_RESTORATION_NS_WIENER :
                              DAV1D_RESTORATION_PC_WIENER;
        }
    } else {
        assert(!p || frame_type == DAV1D_RESTORATION_NS_WIENER);
        uint16_t *const cdf = frame_type == DAV1D_RESTORATION_NS_WIENER ?
                              ts->cdf.m.rst_ns_wiener : ts->cdf.m.rst_pc_wiener;
        const int type = dav1d_msac_decode_bool_adapt(&ts->msac, cdf);
        lr->type = type ? frame_type : DAV1D_RESTORATION_NONE;
    }

    if (lr->type == DAV1D_RESTORATION_NS_WIENER &&
        !f->frame_hdr->restoration.p[p].ns.frame_filters_on)
    {
        const int n_classes = f->frame_hdr->restoration.p[p].ns.num_classes;
        unsigned exact_match_mask = 0;
        struct NsWienerBank *const bank = &ts->ns_wiener_bank[p];
        uint8_t bank_refs[16];
        for (int n = 0, mask = 1; n < n_classes; n++, mask <<= 1) {
            const int exact_match = dav1d_msac_decode_bool_bypass(&ts->msac);
            const int bank_size = bank->bank_size[n];
            int r;
            for (r = 0; r < bank_size - 1; r++) {
                const int found = dav1d_msac_decode_bool_bypass(&ts->msac);
                if (found) break;
            }
            r = (bank->bank_idx[n] - r) & 3;
            exact_match_mask |= mask * exact_match;
            bank_refs[n] = r;
        }

        static const unsigned subset_masks_y[] = { 0x3f, 0xfc3, 0xfff, 0xffff };
        static const unsigned subset_masks_uv[] = { 0x3f, 0xfff, 0x3ffff };
        const unsigned *const masks = p ? subset_masks_uv : subset_masks_y;
        const int8_t (*const cf_range)[2] = p ? dav1d_ns_wiener_coef_range_uv :
                                                dav1d_ns_wiener_coef_range_y;
        for (int n = 0; n < n_classes; n++, exact_match_mask >>= 1) {
            const int r = bank_refs[n];
            int8_t *const filter = lr->ns_filter[n];
            const int8_t *const ref_filter = bank->filter[r][n];
            if (exact_match_mask & 1) {
                memcpy(filter, ref_filter, 16 + 2 * !!p);
                if (!bank->bank_size[n])
                    bank->bank_size[n] = 1;
                continue;
            }
            int s;
            for (s = 0; s < 3 - !!p; s++) {
                const int found = dav1d_msac_decode_bool_adapt(&ts->msac,
                                      ts->cdf.m.wiener_ns_len[!!p]);
                if (!found) break;
            }
            const unsigned mask = masks[s];
            // FIXME read sym bit (chroma only) if ref filter subset "s" is
            // assymetric and has space
            for (int i = 0, m = mask; i < 16 + !!p * 2; i++, m >>= 1) {
                if (!(m & 1)) continue;
                filter[i] = dav1d_msac_decode_4way(&ts->msac,
                                ref_filter[i] - cf_range[i][1],
                                ts->cdf.m.wiener_ns_cf, cf_range[i][0]) +
                            cf_range[i][1];
                // FIXME if sym is set and this coef is assymetric, insert an
                // extra coef here
            }
            const int bidx = bank->bank_idx[n] = (1 + bank->bank_idx[n]) & 3;
            memcpy(bank->filter[bidx][n], filter, sizeof(*filter) * (16 + 2 * !!p));
            bank->bank_size[n] += bank->bank_size[n] < 4;
        }
    }
}

// modeled after the equivalent function in aomdec:decodeframe.c
static int check_trailing_bits_after_symbol_coder(const MsacContext *const msac) {
    // check marker bit (single 1), followed by zeroes
    const int n_bits = -(msac->cnt + 14);
    assert(n_bits <= 0); // this assumes we errored out when cnt <= -15 in caller
    const int n_bytes = (n_bits + 7) >> 3;
    const uint8_t *p = &msac->buf_pos[n_bytes];
    const int pattern = 128 >> ((n_bits - 1) & 7);
    if ((p[-1] & (2 * pattern - 1)) != pattern)
        return 1;

    // check remainder zero bytes
    for (; p < msac->buf_end; p++)
        if (*p)
            return 1;

    return 0;
}

int dav1d_decode_tile_sbrow(Dav1dTaskContext *const t) {
    const Dav1dFrameContext *const f = t->f;
    const enum BlockSize root_bs =
        (const uint8_t[]) { BS_64x64, BS_128x128, BS_256x256 }[f->seq_hdr->sb128];
    const enum BlockSize c_root_bs =
        f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I400 ? BS_INVALID : root_bs;
    Dav1dTileState *const ts = t->ts;
    const Dav1dContext *const c = f->c;
    const int sb_step = f->sb_step;
    const int tile_row = ts->tiling.row, tile_col = ts->tiling.col;
    const int col_sb_start = f->frame_hdr->tiling.col_start_sb[tile_col];
    const int col_sb128_start = col_sb_start >> !f->seq_hdr->sb128;

    if (IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc) {
        dav1d_refmvs_tile_sbrow_init(&t->rt, &f->rf, ts->tiling.col_start,
                                     ts->tiling.col_end, ts->tiling.row_start,
                                     ts->tiling.row_end, t->by >> f->sb_shift,
                                     ts->tiling.row, t->frame_thread.pass);
    }

    if (IS_INTER_OR_SWITCH(f->frame_hdr) && c->n_fc > 1) {
        const int sby = (t->by - ts->tiling.row_start) >> f->sb_shift;
        int (*const lowest_px)[2] = ts->lowest_pixel[sby];
        for (int n = 0; n < 7; n++)
            for (int m = 0; m < 2; m++)
                lowest_px[n][m] = INT_MIN;
    }

    reset_context(&t->l, IS_KEY_OR_INTRA(f->frame_hdr), t->frame_thread.pass);
    if (t->frame_thread.pass == 2) {
        const int off_2pass = c->n_tc > 1 ? f->sb128w * f->frame_hdr->tiling.rows : 0;
        for (t->bx = ts->tiling.col_start,
             t->a = f->a + off_2pass + col_sb128_start + tile_row * f->sb128w;
             t->bx < ts->tiling.col_end; t->bx += sb_step)
        {
            if (atomic_load_explicit(c->flush, memory_order_acquire))
                return 1;
            if (decode_sb(t, DB_ONLY(1) root_bs, c_root_bs))
                return 1;
            if (t->bx & 16 || f->seq_hdr->sb128)
                t->a++;
        }
        f->bd_fn.backup_ipred_edge(t);
        return 0;
    }

    if (f->c->n_tc > 1 && f->frame_hdr->use_ref_frame_mvs) {
        f->c->refmvs_dsp.load_tmvs(&f->rf, ts->tiling.row,
                                   ts->tiling.col_start >> 1, ts->tiling.col_end >> 1,
                                   t->by >> 1, (t->by + sb_step) >> 1);
    }
    memset(t->pal_sz_uv[1], 0, sizeof(*t->pal_sz_uv));
    const int sb128y = t->by >> 5;
    for (t->bx = ts->tiling.col_start, t->a = f->a + col_sb128_start + tile_row * f->sb128w,
         t->lf_mask = f->lf.mask + sb128y * f->sb128w + col_sb128_start;
         t->bx < ts->tiling.col_end; t->bx += sb_step)
    {
        if (atomic_load_explicit(c->flush, memory_order_acquire))
            return 1;
        if (root_bs == BS_128x128) {
            t->cur_sb_cdef_idx_ptr = t->lf_mask->cdef_idx;
            t->cur_sb_cdef_idx_ptr[0] = -1;
            t->cur_sb_cdef_idx_ptr[1] = -1;
            t->cur_sb_cdef_idx_ptr[2] = -1;
            t->cur_sb_cdef_idx_ptr[3] = -1;
        } else {
            t->cur_sb_cdef_idx_ptr =
                &t->lf_mask->cdef_idx[((t->bx & 16) >> 4) +
                                      ((t->by & 16) >> 3)];
            t->cur_sb_cdef_idx_ptr[0] = -1;
        }
        // Restoration filter
        const int sbsz = 4 << f->sb_step;
        for (int p = 0, ss_ver = 0, ss_hor = 0; p < 3;
             p++, ss_ver = f->ss_ver, ss_hor = f->ss_hor)
        {
            if (!((f->lf.restore_planes >> p) & 1U))
                continue;

            const int x = 4 * t->bx >> ss_hor, y = t->by * 4 >> ss_ver;
            const int unit_sz_log2 = f->frame_hdr->restoration.unit_size[!!p];
            const int unit_sz = 1 << unit_sz_log2;
            const unsigned mask = unit_sz - 1;
            if ((x | y) & mask) continue;
            const int w = (f->cur.p.w + ss_hor) >> ss_hor;
            const int h = (f->cur.p.h + ss_ver) >> ss_ver;
            const int half_unit = unit_sz >> 1;
            // Round half up at frame boundaries, if there's more than one
            // restoration unit
            if ((y && y + half_unit > h) || (x && x + half_unit > w)) continue;

            const enum Dav1dRestorationType frame_type = f->frame_hdr->restoration.p[p].type;

            // FIXME many of these values can be pre-calculated at frame-level
            const int sbw = sbsz >> ss_hor, sbh = sbsz >> ss_ver;
            const int lruw = imax(1, imin(w - x + half_unit, sbw) >> unit_sz_log2);
            const int lruh = imax(1, imin(h - y + half_unit, sbh) >> unit_sz_log2);
            const int vsh = unit_sz_log2 - 7 + ss_ver;
            const int hsh = unit_sz_log2 - 7 + ss_hor;
            int sb_idx = (t->by >> 5) * f->sr_sb128w + (t->bx >> 5);
            for (int y = 0; y < lruh; y++, sb_idx += f->sr_sb128w << vsh) {
                for (int x = 0; x < lruw; x++) {
                    Av1RestorationUnit *const lr =
                        &f->lf.lr_mask[sb_idx + (x << hsh)].lr[p][0];
                    read_restoration_info(t, lr, p, frame_type);
                    DEBUG_BLOCK_printf("Post-restoration[p=%d,type=%d]: r=%d\n",
                                       p, lr->type, ts->msac.rng);
                }
            }
        }
        if (decode_sb(t, DB_ONLY(1) root_bs, c_root_bs))
            return 1;
        if (t->bx & 16 || f->seq_hdr->sb128) {
            t->a++;
            t->lf_mask++;
        }
    }

    if (f->seq_hdr->ref_frame_mvs && f->c->n_tc > 1 && IS_INTER_OR_SWITCH(f->frame_hdr)) {
        dav1d_refmvs_save_tmvs(&f->c->refmvs_dsp, &t->rt,
                               ts->tiling.col_start >> 1, ts->tiling.col_end >> 1,
                               t->by >> 1, (t->by + sb_step) >> 1);
    }

    // backup pre-loopfilter pixels for intra prediction of the next sbrow
    if (t->frame_thread.pass != 1)
        f->bd_fn.backup_ipred_edge(t);

    // backup t->a/l.tx_lpf_y/uv at tile boundaries to use them to "fix"
    // up the initial value in neighbour tiles when running the loopfilter
    int align_h = (f->bh + 31) & ~31;
    memcpy(&f->lf.tx_lpf_right_edge[0][align_h * tile_col + t->by],
           &t->l.tx_lpf_y[t->by & 16], sb_step);
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    align_h >>= ss_ver;
    memcpy(&f->lf.tx_lpf_right_edge[1][align_h * tile_col + (t->by >> ss_ver)],
           &t->l.tx_lpf_uv[(t->by & 16) >> ss_ver], sb_step >> ss_ver);

    // error out on symbol decoder overread
    if (ts->msac.cnt <= -15) return 1;

    return c->strict_std_compliance &&
           (t->by >> f->sb_shift) + 1 >= f->frame_hdr->tiling.row_start_sb[tile_row + 1] &&
           check_trailing_bits_after_symbol_coder(&ts->msac);
}

int dav1d_decode_frame_init(Dav1dFrameContext *const f) {
    const Dav1dContext *const c = f->c;
    int retval = DAV1D_ERR(ENOMEM);

    if (f->sbh > f->lf.start_of_tile_row_sz) {
        dav1d_free(f->lf.start_of_tile_row);
        f->lf.start_of_tile_row = dav1d_malloc(ALLOC_TILE, f->sbh * sizeof(uint8_t));
        if (!f->lf.start_of_tile_row) {
            f->lf.start_of_tile_row_sz = 0;
            goto error;
        }
        f->lf.start_of_tile_row_sz = f->sbh;
    }
    int sby = 0;
    for (int tile_row = 0; tile_row < f->frame_hdr->tiling.rows; tile_row++) {
        f->lf.start_of_tile_row[sby++] = tile_row;
        while (sby < f->frame_hdr->tiling.row_start_sb[tile_row + 1])
            f->lf.start_of_tile_row[sby++] = 0;
    }

    const int n_ts = f->frame_hdr->tiling.cols * f->frame_hdr->tiling.rows;
    if (n_ts != f->n_ts) {
        if (c->n_fc > 1) {
            dav1d_free(f->frame_thread.tile_start_off);
            f->frame_thread.tile_start_off =
                dav1d_malloc(ALLOC_TILE, sizeof(*f->frame_thread.tile_start_off) * n_ts);
            if (!f->frame_thread.tile_start_off) {
                f->n_ts = 0;
                goto error;
            }
        }
        dav1d_free_aligned(f->ts);
        f->ts = dav1d_alloc_aligned(ALLOC_TILE, sizeof(*f->ts) * n_ts, 32);
        if (!f->ts) goto error;
        f->n_ts = n_ts;
    }

    const int a_sz = f->sb128w * f->frame_hdr->tiling.rows * (1 + (c->n_fc > 1 && c->n_tc > 1));
    if (a_sz != f->a_sz) {
        dav1d_free(f->a);
        f->a = dav1d_malloc(ALLOC_TILE, sizeof(*f->a) * a_sz);
        if (!f->a) {
            f->a_sz = 0;
            goto error;
        }
        f->a_sz = a_sz;
    }

    const int num_sb128 = f->sb128w * f->sb128h;
    const uint8_t *const size_mul = ss_size_mul[f->cur.p.layout];
    const int hbd = !!f->seq_hdr->hbd;
    if (c->n_fc > 1) {
        const unsigned sb_step4 = f->sb_step * 4;
        int tile_idx = 0;
        for (int tile_row = 0; tile_row < f->frame_hdr->tiling.rows; tile_row++) {
            const unsigned row_off = f->frame_hdr->tiling.row_start_sb[tile_row] *
                                     sb_step4 * f->sb128w * 128;
            const unsigned b_diff = (f->frame_hdr->tiling.row_start_sb[tile_row + 1] -
                                     f->frame_hdr->tiling.row_start_sb[tile_row]) * sb_step4;
            for (int tile_col = 0; tile_col < f->frame_hdr->tiling.cols; tile_col++) {
                f->frame_thread.tile_start_off[tile_idx++] = row_off + b_diff *
                    f->frame_hdr->tiling.col_start_sb[tile_col] * sb_step4;
            }
        }

        const int lowest_pixel_mem_sz = f->frame_hdr->tiling.cols * f->sbh;
        if (lowest_pixel_mem_sz != f->tile_thread.lowest_pixel_mem_sz) {
            dav1d_free(f->tile_thread.lowest_pixel_mem);
            f->tile_thread.lowest_pixel_mem =
                dav1d_malloc(ALLOC_TILE, lowest_pixel_mem_sz *
                             sizeof(*f->tile_thread.lowest_pixel_mem));
            if (!f->tile_thread.lowest_pixel_mem) {
                f->tile_thread.lowest_pixel_mem_sz = 0;
                goto error;
            }
            f->tile_thread.lowest_pixel_mem_sz = lowest_pixel_mem_sz;
        }
        int (*lowest_pixel_ptr)[7][2] = f->tile_thread.lowest_pixel_mem;
        for (int tile_row = 0, tile_row_base = 0; tile_row < f->frame_hdr->tiling.rows;
             tile_row++, tile_row_base += f->frame_hdr->tiling.cols)
        {
            const int tile_row_sb_h = f->frame_hdr->tiling.row_start_sb[tile_row + 1] -
                                      f->frame_hdr->tiling.row_start_sb[tile_row];
            for (int tile_col = 0; tile_col < f->frame_hdr->tiling.cols; tile_col++) {
                f->ts[tile_row_base + tile_col].lowest_pixel = lowest_pixel_ptr;
                lowest_pixel_ptr += tile_row_sb_h;
            }
        }

        const int cbi_sz = num_sb128 * size_mul[0];
        if (cbi_sz != f->frame_thread.cbi_sz) {
            dav1d_free_aligned(f->frame_thread.cbi);
            f->frame_thread.cbi =
                dav1d_alloc_aligned(ALLOC_BLOCK, sizeof(*f->frame_thread.cbi) *
                                    cbi_sz * 32 * 32 / 4, 64);
            if (!f->frame_thread.cbi) {
                f->frame_thread.cbi_sz = 0;
                goto error;
            }
            f->frame_thread.cbi_sz = cbi_sz;
        }

        const int cf_sz = (num_sb128 * size_mul[0]) << hbd;
        if (cf_sz != f->frame_thread.cf_sz) {
            dav1d_free_aligned(f->frame_thread.cf);
            f->frame_thread.cf =
                dav1d_alloc_aligned(ALLOC_COEF, (size_t)cf_sz * 128 * 128 / 2, 64);
            if (!f->frame_thread.cf) {
                f->frame_thread.cf_sz = 0;
                goto error;
            }
            memset(f->frame_thread.cf, 0, (size_t)cf_sz * 128 * 128 / 2);
            f->frame_thread.cf_sz = cf_sz;
        }

        if (f->frame_hdr->allow_screen_content_tools) {
            const int pal_sz = num_sb128 << hbd;
            if (pal_sz != f->frame_thread.pal_sz) {
                dav1d_free_aligned(f->frame_thread.pal);
                f->frame_thread.pal =
                    dav1d_alloc_aligned(ALLOC_PAL, sizeof(*f->frame_thread.pal) *
                                        pal_sz * 16 * 16, 64);
                if (!f->frame_thread.pal) {
                    f->frame_thread.pal_sz = 0;
                    goto error;
                }
                f->frame_thread.pal_sz = pal_sz;
            }

            const int pal_idx_sz = num_sb128 * size_mul[1];
            if (pal_idx_sz != f->frame_thread.pal_idx_sz) {
                dav1d_free_aligned(f->frame_thread.pal_idx);
                f->frame_thread.pal_idx =
                    dav1d_alloc_aligned(ALLOC_PAL, sizeof(*f->frame_thread.pal_idx) *
                                        pal_idx_sz * 128 * 128 / 8, 64);
                if (!f->frame_thread.pal_idx) {
                    f->frame_thread.pal_idx_sz = 0;
                    goto error;
                }
                f->frame_thread.pal_idx_sz = pal_idx_sz;
            }
        } else if (f->frame_thread.pal) {
            dav1d_freep_aligned(&f->frame_thread.pal);
            dav1d_freep_aligned(&f->frame_thread.pal_idx);
            f->frame_thread.pal_sz = f->frame_thread.pal_idx_sz = 0;
        }
    }

    // update allocation of block contexts for above
    ptrdiff_t y_stride = f->cur.stride[0], uv_stride = f->cur.stride[1];
    if (y_stride * f->sbh * 4 != f->lf.cdef_buf_plane_sz[0] ||
        uv_stride * f->sbh * 8 != f->lf.cdef_buf_plane_sz[1] ||
        f->sbh != f->lf.cdef_buf_sbh)
    {
        dav1d_free_aligned(f->lf.cdef_line_buf);
        size_t alloc_sz = 64;
        alloc_sz += (size_t)llabs(y_stride) * 4 * f->sbh;
        alloc_sz += (size_t)llabs(uv_stride) * 8 * f->sbh;
        uint8_t *ptr = f->lf.cdef_line_buf = dav1d_alloc_aligned(ALLOC_CDEF, alloc_sz, 32);
        if (!ptr) {
            f->lf.cdef_buf_plane_sz[0] = f->lf.cdef_buf_plane_sz[1] = 0;
            goto error;
        }

        ptr += 32;
        if (y_stride < 0) {
            f->lf.cdef_line[0][0] = ptr - y_stride * (f->sbh * 4 - 1);
            f->lf.cdef_line[1][0] = ptr - y_stride * (f->sbh * 4 - 3);
        } else {
            f->lf.cdef_line[0][0] = ptr + y_stride * 0;
            f->lf.cdef_line[1][0] = ptr + y_stride * 2;
        }
        ptr += llabs(y_stride) * f->sbh * 4;
        if (uv_stride < 0) {
            f->lf.cdef_line[0][1] = ptr - uv_stride * (f->sbh * 8 - 1);
            f->lf.cdef_line[0][2] = ptr - uv_stride * (f->sbh * 8 - 3);
            f->lf.cdef_line[1][1] = ptr - uv_stride * (f->sbh * 8 - 5);
            f->lf.cdef_line[1][2] = ptr - uv_stride * (f->sbh * 8 - 7);
        } else {
            f->lf.cdef_line[0][1] = ptr + uv_stride * 0;
            f->lf.cdef_line[0][2] = ptr + uv_stride * 2;
            f->lf.cdef_line[1][1] = ptr + uv_stride * 4;
            f->lf.cdef_line[1][2] = ptr + uv_stride * 6;
        }

        f->lf.cdef_buf_plane_sz[0] = (int) y_stride * f->sbh * 4;
        f->lf.cdef_buf_plane_sz[1] = (int) uv_stride * f->sbh * 8;
        f->lf.cdef_buf_sbh = f->sbh;
    }

    const int sb128 = f->seq_hdr->sb128;
    const int num_lines = c->n_tc > 1 ? f->sbh * 4 << sb128 : 12;
    y_stride = f->sr_cur.p.stride[0], uv_stride = f->sr_cur.p.stride[1];
    if (y_stride * num_lines != f->lf.lr_buf_plane_sz[0] ||
        uv_stride * num_lines * 2 != f->lf.lr_buf_plane_sz[1])
    {
        dav1d_free_aligned(f->lf.lr_line_buf);
        // lr simd may overread the input, so slightly over-allocate the lpf buffer
        size_t alloc_sz = 128;
        alloc_sz += (size_t)llabs(y_stride) * num_lines;
        alloc_sz += (size_t)llabs(uv_stride) * num_lines * 2;
        uint8_t *ptr = f->lf.lr_line_buf = dav1d_alloc_aligned(ALLOC_LR, alloc_sz, 64);
        if (!ptr) {
            f->lf.lr_buf_plane_sz[0] = f->lf.lr_buf_plane_sz[1] = 0;
            goto error;
        }

        ptr += 64;
        if (y_stride < 0)
            f->lf.lr_lpf_line[0] = ptr - y_stride * (num_lines - 1);
        else
            f->lf.lr_lpf_line[0] = ptr;
        ptr += llabs(y_stride) * num_lines;
        if (uv_stride < 0) {
            f->lf.lr_lpf_line[1] = ptr - uv_stride * (num_lines * 1 - 1);
            f->lf.lr_lpf_line[2] = ptr - uv_stride * (num_lines * 2 - 1);
        } else {
            f->lf.lr_lpf_line[1] = ptr;
            f->lf.lr_lpf_line[2] = ptr + uv_stride * num_lines;
        }

        f->lf.lr_buf_plane_sz[0] = (int) y_stride * num_lines;
        f->lf.lr_buf_plane_sz[1] = (int) uv_stride * num_lines * 2;
    }

    // update allocation for loopfilter masks
    if (num_sb128 != f->lf.mask_sz) {
        dav1d_free(f->lf.mask);
        dav1d_free(f->lf.level);
        f->lf.mask = dav1d_malloc(ALLOC_LF, sizeof(*f->lf.mask) * num_sb128);
        // over-allocate by 3 bytes since some of the SIMD implementations
        // index this from the level type and can thus over-read by up to 3
        f->lf.level = dav1d_malloc(ALLOC_LF, sizeof(*f->lf.level) * num_sb128 * 32 * 32 + 3);
        if (!f->lf.mask || !f->lf.level) {
            f->lf.mask_sz = 0;
            goto error;
        }
        if (c->n_fc > 1) {
            dav1d_free(f->frame_thread.b);
            f->frame_thread.b = dav1d_malloc(ALLOC_BLOCK, sizeof(*f->frame_thread.b) *
                                             num_sb128 * 32 * 32);
            if (!f->frame_thread.b) {
                f->lf.mask_sz = 0;
                goto error;
            }
        }
        f->lf.mask_sz = num_sb128;
    }

    f->sr_sb128w = (f->sr_cur.p.p.w + 127) >> 7;
    const int lr_mask_sz = f->sr_sb128w * f->sb128h;
    if (lr_mask_sz != f->lf.lr_mask_sz) {
        dav1d_free(f->lf.lr_mask);
        f->lf.lr_mask = dav1d_malloc(ALLOC_LR, sizeof(*f->lf.lr_mask) * lr_mask_sz);
        if (!f->lf.lr_mask) {
            f->lf.lr_mask_sz = 0;
            goto error;
        }
        f->lf.lr_mask_sz = lr_mask_sz;
    }
    f->lf.restore_planes =
        ((f->frame_hdr->restoration.p[0].type != DAV1D_RESTORATION_NONE) << 0) +
        ((f->frame_hdr->restoration.p[1].type != DAV1D_RESTORATION_NONE) << 1) +
        ((f->frame_hdr->restoration.p[2].type != DAV1D_RESTORATION_NONE) << 2);
    dav1d_calc_lf_values(f->lf.lvl, f->frame_hdr, (int8_t[4]) { 0, 0, 0, 0 });
    memset(f->lf.mask, 0, sizeof(*f->lf.mask) * num_sb128);

    const int ipred_edge_sz = f->sbh * f->sb128w << hbd;
    if (ipred_edge_sz != f->ipred_edge_sz) {
        dav1d_free_aligned(f->ipred_edge[0]);
        uint8_t *ptr = f->ipred_edge[0] =
            dav1d_alloc_aligned(ALLOC_IPRED, ipred_edge_sz * 128 * 3, 64);
        if (!ptr) {
            f->ipred_edge_sz = 0;
            goto error;
        }
        f->ipred_edge[1] = ptr + ipred_edge_sz * 128 * 1;
        f->ipred_edge[2] = ptr + ipred_edge_sz * 128 * 2;
        f->ipred_edge_sz = ipred_edge_sz;
    }

    const int re_sz = f->sb128h * f->frame_hdr->tiling.cols;
    if (re_sz != f->lf.re_sz) {
        dav1d_free(f->lf.tx_lpf_right_edge[0]);
        f->lf.tx_lpf_right_edge[0] = dav1d_malloc(ALLOC_LF, re_sz * 32 * 2);
        if (!f->lf.tx_lpf_right_edge[0]) {
            f->lf.re_sz = 0;
            goto error;
        }
        f->lf.tx_lpf_right_edge[1] = f->lf.tx_lpf_right_edge[0] + re_sz * 32;
        f->lf.re_sz = re_sz;
    }

    // init ref mvs
    if (IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc) {
        const int ret =
            dav1d_refmvs_init_frame(&f->rf, f->seq_hdr, f->frame_hdr,
                                    f->refpoc, f->mvs, f->refrefpoc, f->ref_mvs,
                                    f->c->n_tc, f->c->n_fc);
        if (ret < 0) goto error;
    }

    // setup dequant tables
    init_quant_tables(f->seq_hdr, f->frame_hdr, f->frame_hdr->quant.yac, f->dq);
    if (f->frame_hdr->quant.qm.enabled)
        for (int i = 0; i < N_RECT_TX_SIZES; i++) {
            f->qm[i][0] = dav1d_qm_tbl[f->frame_hdr->quant.qm.y[0]][0][i];
            f->qm[i][1] = dav1d_qm_tbl[f->frame_hdr->quant.qm.u[0]][1][i];
            f->qm[i][2] = dav1d_qm_tbl[f->frame_hdr->quant.qm.v[0]][1][i];
        }
    else
        memset(f->qm, 0, sizeof(f->qm));

    // setup jnt_comp weights
    if (f->frame_hdr->switchable_comp_refs) {
        for (int i = 0; i < 7; i++) {
            const unsigned ref0poc = f->refp[i].p.frame_hdr->frame_offset;

            for (int j = i + 1; j < 7; j++) {
                const unsigned ref1poc = f->refp[j].p.frame_hdr->frame_offset;

                const unsigned d1 =
                    imin(abs(get_poc_diff(f->seq_hdr->order_hint_n_bits, ref0poc,
                                          f->cur.frame_hdr->frame_offset)), 31);
                const unsigned d0 =
                    imin(abs(get_poc_diff(f->seq_hdr->order_hint_n_bits, ref1poc,
                                          f->cur.frame_hdr->frame_offset)), 31);
                const int order = d0 <= d1;

                static const uint8_t quant_dist_weight[3][2] = {
                    { 2, 3 }, { 2, 5 }, { 2, 7 }
                };
                static const uint8_t quant_dist_lookup_table[4][2] = {
                    { 9, 7 }, { 11, 5 }, { 12, 4 }, { 13, 3 }
                };

                int k;
                for (k = 0; k < 3; k++) {
                    const int c0 = quant_dist_weight[k][order];
                    const int c1 = quant_dist_weight[k][!order];
                    const int d0_c0 = d0 * c0;
                    const int d1_c1 = d1 * c1;
                    if ((d0 > d1 && d0_c0 < d1_c1) || (d0 <= d1 && d0_c0 > d1_c1)) break;
                }

                f->jnt_weights[i][j] = quant_dist_lookup_table[k][order];
            }
        }
    }

    /* Init loopfilter pointers. Increasing NULL pointers is technically UB,
     * so just point the chroma pointers in 4:0:0 to the luma plane here to
     * avoid having additional in-loop branches in various places. We never
     * dereference those pointers so it doesn't really matter what they
     * point at, as long as the pointers are valid. */
    const int has_chroma = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400;
    f->lf.p[0] = f->cur.data[0];
    f->lf.p[1] = f->cur.data[has_chroma ? 1 : 0];
    f->lf.p[2] = f->cur.data[has_chroma ? 2 : 0];
    f->lf.sr_p[0] = f->sr_cur.p.data[0];
    f->lf.sr_p[1] = f->sr_cur.p.data[has_chroma ? 1 : 0];
    f->lf.sr_p[2] = f->sr_cur.p.data[has_chroma ? 2 : 0];

    retval = 0;
error:
    return retval;
}

int dav1d_decode_frame_init_cdf(Dav1dFrameContext *const f) {
    const Dav1dContext *const c = f->c;
    int retval = DAV1D_ERR(EINVAL);

    if (f->frame_hdr->refresh_context)
        dav1d_cdf_thread_copy(f->out_cdf.data.cdf, &f->in_cdf);

    // parse individual tiles per tile group
    int tile_row = 0, tile_col = 0;
    f->task_thread.update_set = 0;
    for (int i = 0; i < f->n_tile_data; i++) {
        const uint8_t *data = f->tile[i].data.data;
        size_t size = f->tile[i].data.sz;

        for (int j = f->tile[i].start; j <= f->tile[i].end; j++) {
            size_t tile_sz;
            if (j == f->tile[i].end) {
                tile_sz = size;
            } else {
                if (f->frame_hdr->tiling.n_bytes > size) goto error;
                tile_sz = 0;
                for (unsigned k = 0; k < f->frame_hdr->tiling.n_bytes; k++)
                    tile_sz |= (unsigned)*data++ << (k * 8);
                tile_sz++;
                size -= f->frame_hdr->tiling.n_bytes;
                if (tile_sz > size) goto error;
            }

            setup_tile(&f->ts[j], f, data, tile_sz, tile_row, tile_col++,
                       c->n_fc > 1 ? f->frame_thread.tile_start_off[j] : 0);

            if (tile_col == f->frame_hdr->tiling.cols) {
                tile_col = 0;
                tile_row++;
            }
            if (j == f->frame_hdr->tiling.update && f->frame_hdr->refresh_context)
                f->task_thread.update_set = 1;
            data += tile_sz;
            size -= tile_sz;
        }
    }

    if (c->n_tc > 1) {
        const int uses_2pass = c->n_fc > 1;
        for (int n = 0; n < f->sb128w * f->frame_hdr->tiling.rows * (1 + uses_2pass); n++)
            reset_context(&f->a[n], IS_KEY_OR_INTRA(f->frame_hdr),
                          uses_2pass ? 1 + (n >= f->sb128w * f->frame_hdr->tiling.rows) : 0);
    }

    retval = 0;
error:
    return retval;
}

int dav1d_decode_frame_main(Dav1dFrameContext *const f) {
    const Dav1dContext *const c = f->c;
    int retval = DAV1D_ERR(EINVAL);

    assert(f->c->n_tc == 1);

    Dav1dTaskContext *const t = &c->tc[f - c->fc];
    t->f = f;
    t->frame_thread.pass = 0;

    for (int n = 0; n < f->sb128w * f->frame_hdr->tiling.rows; n++)
        reset_context(&f->a[n], IS_KEY_OR_INTRA(f->frame_hdr), 0);

    // no threading - we explicitly interleave tile/sbrow decoding
    // and post-filtering, so that the full process runs in-line
    for (int tile_row = 0; tile_row < f->frame_hdr->tiling.rows; tile_row++) {
        const int sbh_end =
            imin(f->frame_hdr->tiling.row_start_sb[tile_row + 1], f->sbh);
        for (int sby = f->frame_hdr->tiling.row_start_sb[tile_row];
             sby < sbh_end; sby++)
        {
            t->by = sby << (4 + f->seq_hdr->sb128);
            const int by_end = (t->by + f->sb_step) >> 1;
            if (f->frame_hdr->use_ref_frame_mvs) {
                f->c->refmvs_dsp.load_tmvs(&f->rf, tile_row,
                                           0, f->bw >> 1, t->by >> 1, by_end);
            }
            for (int tile_col = 0; tile_col < f->frame_hdr->tiling.cols; tile_col++) {
                t->ts = &f->ts[tile_row * f->frame_hdr->tiling.cols + tile_col];
                if (dav1d_decode_tile_sbrow(t)) goto error;
            }
            if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
                dav1d_refmvs_save_tmvs(&f->c->refmvs_dsp, &t->rt,
                                       0, f->bw >> 1, t->by >> 1, by_end);
            }

            // loopfilter + cdef + restoration
            f->bd_fn.filter_sbrow(f, sby);
        }
    }

    retval = 0;
error:
    return retval;
}

void dav1d_decode_frame_exit(Dav1dFrameContext *const f, int retval) {
    const Dav1dContext *const c = f->c;

    if (f->sr_cur.p.data[0])
        atomic_init(&f->task_thread.error, 0);

    if (c->n_fc > 1 && retval && f->frame_thread.cf) {
        memset(f->frame_thread.cf, 0,
               (size_t)f->frame_thread.cf_sz * 128 * 128 / 2);
    }
    for (int i = 0; i < 7; i++) {
        if (f->refp[i].p.frame_hdr) {
            if (!retval && c->n_fc > 1 && c->strict_std_compliance &&
                atomic_load(&f->refp[i].progress[1]) == FRAME_ERROR)
            {
                retval = DAV1D_ERR(EINVAL);
                atomic_store(&f->task_thread.error, 1);
                atomic_store(&f->sr_cur.progress[1], FRAME_ERROR);
            }
            dav1d_thread_picture_unref(&f->refp[i]);
        }
        dav1d_ref_dec(&f->ref_mvs_ref[i]);
    }

    dav1d_picture_unref_internal(&f->cur);
    dav1d_thread_picture_unref(&f->sr_cur);
    dav1d_cdf_thread_unref(&f->in_cdf);
    if (f->frame_hdr && f->frame_hdr->refresh_context) {
        if (f->out_cdf.progress)
            atomic_store(f->out_cdf.progress, retval == 0 ? 1 : TILE_ERROR);
        dav1d_cdf_thread_unref(&f->out_cdf);
    }
    dav1d_ref_dec(&f->cur_segmap_ref);
    dav1d_ref_dec(&f->prev_segmap_ref);
    dav1d_ref_dec(&f->mvs_ref);
    dav1d_ref_dec(&f->seq_hdr_ref);
    dav1d_ref_dec(&f->frame_hdr_ref);

    for (int i = 0; i < f->n_tile_data; i++)
        dav1d_data_unref_internal(&f->tile[i].data);
    f->task_thread.retval = retval;
}

int dav1d_decode_frame(Dav1dFrameContext *const f) {
    assert(f->c->n_fc == 1);
    // if n_tc > 1 (but n_fc == 1), we could run init/exit in the task
    // threads also. Not sure it makes a measurable difference.
    int res = dav1d_decode_frame_init(f);
    if (!res) res = dav1d_decode_frame_init_cdf(f);
    // wait until all threads have completed
    if (!res) {
        if (f->c->n_tc > 1) {
            res = dav1d_task_create_tile_sbrow(f, 0, 1);
            pthread_mutex_lock(&f->task_thread.ttd->lock);
            pthread_cond_signal(&f->task_thread.ttd->cond);
            if (!res) {
                while (!f->task_thread.done[0] ||
                       atomic_load(&f->task_thread.task_counter) > 0)
                {
                    pthread_cond_wait(&f->task_thread.cond,
                                      &f->task_thread.ttd->lock);
                }
            }
            pthread_mutex_unlock(&f->task_thread.ttd->lock);
            res = f->task_thread.retval;
        } else {
            res = dav1d_decode_frame_main(f);
            if (!res && f->frame_hdr->refresh_context && f->task_thread.update_set) {
                dav1d_cdf_thread_update(f->frame_hdr, f->out_cdf.data.cdf,
                                        &f->ts[f->frame_hdr->tiling.update].cdf);
            }
        }
    }
    dav1d_decode_frame_exit(f, res);
    res = f->task_thread.retval;
    f->n_tile_data = 0;
    return res;
}

int dav1d_submit_frame(Dav1dContext *const c) {
    Dav1dFrameContext *f;
    int res = -1;

    // wait for c->out_delayed[next] and move into c->out if visible
    Dav1dThreadPicture *out_delayed;
    if (c->n_fc > 1) {
        pthread_mutex_lock(&c->task_thread.lock);
        const unsigned next = c->frame_thread.next++;
        if (c->frame_thread.next == c->n_fc)
            c->frame_thread.next = 0;

        f = &c->fc[next];
        while (f->n_tile_data > 0)
            pthread_cond_wait(&f->task_thread.cond,
                              &c->task_thread.lock);
        out_delayed = &c->frame_thread.out_delayed[next];
        if (out_delayed->p.data[0] || atomic_load(&f->task_thread.error)) {
            unsigned first = atomic_load(&c->task_thread.first);
            if (first + 1U < c->n_fc)
                atomic_fetch_add(&c->task_thread.first, 1U);
            else
                atomic_store(&c->task_thread.first, 0);
            atomic_compare_exchange_strong(&c->task_thread.reset_task_cur,
                                           &first, UINT_MAX);
            if (c->task_thread.cur && c->task_thread.cur < c->n_fc)
                c->task_thread.cur--;
        }
        const int error = f->task_thread.retval;
        if (error) {
            f->task_thread.retval = 0;
            c->cached_error = error;
            dav1d_data_props_copy(&c->cached_error_props, &out_delayed->p.m);
            dav1d_thread_picture_unref(out_delayed);
        } else if (out_delayed->p.data[0]) {
            const unsigned progress = atomic_load_explicit(&out_delayed->progress[1],
                                                           memory_order_relaxed);
            if ((out_delayed->visible || c->output_invisible_frames) &&
                progress != FRAME_ERROR)
            {
                dav1d_thread_picture_ref(&c->out, out_delayed);
                c->event_flags |= dav1d_picture_get_event_flags(out_delayed);
            }
            dav1d_thread_picture_unref(out_delayed);
        }
    } else {
        f = c->fc;
    }

    f->seq_hdr = c->seq_hdr;
    f->seq_hdr_ref = c->seq_hdr_ref;
    dav1d_ref_inc(f->seq_hdr_ref);
    f->frame_hdr = c->frame_hdr;
    f->frame_hdr_ref = c->frame_hdr_ref;
    c->frame_hdr = NULL;
    c->frame_hdr_ref = NULL;
    f->dsp = &c->dsp[f->seq_hdr->hbd];

    const int bpc = 8 + 2 * f->seq_hdr->hbd;

    if (!f->dsp->ipred.intra_pred[DC_PRED]) {
        Dav1dDSPContext *const dsp = &c->dsp[f->seq_hdr->hbd];

        switch (bpc) {
#define assign_bitdepth_case(bd) \
            dav1d_cdef_dsp_init_##bd##bpc(&dsp->cdef); \
            dav1d_intra_pred_dsp_init_##bd##bpc(&dsp->ipred); \
            dav1d_itx_dsp_init_##bd##bpc(&dsp->itx, bpc); \
            dav1d_loop_filter_dsp_init_##bd##bpc(&dsp->lf); \
            dav1d_loop_restoration_dsp_init_##bd##bpc(&dsp->lr, bpc); \
            dav1d_mc_dsp_init_##bd##bpc(&dsp->mc); \
            dav1d_film_grain_dsp_init_##bd##bpc(&dsp->fg); \
            break
#if CONFIG_8BPC
        case 8:
            assign_bitdepth_case(8);
#endif
#if CONFIG_16BPC
        case 10:
        case 12:
            assign_bitdepth_case(16);
#endif
#undef assign_bitdepth_case
        default:
            dav1d_log(c, "Compiled without support for %d-bit decoding\n",
                    8 + 2 * f->seq_hdr->hbd);
            res = DAV1D_ERR(ENOPROTOOPT);
            goto error;
        }
    }

#define assign_bitdepth_case(bd) \
        f->bd_fn.recon_b_inter = dav1d_recon_b_inter_##bd##bpc; \
        f->bd_fn.recon_b_intra = dav1d_recon_b_intra_##bd##bpc; \
        f->bd_fn.filter_sbrow = dav1d_filter_sbrow_##bd##bpc; \
        f->bd_fn.filter_sbrow_deblock_cols = dav1d_filter_sbrow_deblock_cols_##bd##bpc; \
        f->bd_fn.filter_sbrow_deblock_rows = dav1d_filter_sbrow_deblock_rows_##bd##bpc; \
        f->bd_fn.filter_sbrow_cdef = dav1d_filter_sbrow_cdef_##bd##bpc; \
        f->bd_fn.filter_sbrow_lr = dav1d_filter_sbrow_lr_##bd##bpc; \
        f->bd_fn.backup_ipred_edge = dav1d_backup_ipred_edge_##bd##bpc; \
        f->bd_fn.read_coef_blocks = dav1d_read_coef_blocks_##bd##bpc; \
        f->bd_fn.copy_pal_block_y = dav1d_copy_pal_block_y_##bd##bpc; \
        f->bd_fn.copy_pal_block_uv = dav1d_copy_pal_block_uv_##bd##bpc; \
        f->bd_fn.read_pal_plane = dav1d_read_pal_plane_##bd##bpc; \
        f->bd_fn.read_pal_uv = dav1d_read_pal_uv_##bd##bpc
    if (!f->seq_hdr->hbd) {
#if CONFIG_8BPC
        assign_bitdepth_case(8);
#endif
    } else {
#if CONFIG_16BPC
        assign_bitdepth_case(16);
#endif
    }
#undef assign_bitdepth_case

    int ref_coded_width[7];
    if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
        if (f->frame_hdr->primary_ref_frame != DAV1D_PRIMARY_REF_NONE) {
            const int pri_ref = f->frame_hdr->refidx[f->frame_hdr->primary_ref_frame];
            if (!c->refs[pri_ref].p.p.data[0]) {
                res = DAV1D_ERR(EINVAL);
                goto error;
            }
        }
        for (int i = 0; i < 7; i++) {
            const int refidx = f->frame_hdr->refidx[i];
            if (!c->refs[refidx].p.p.data[0] ||
                f->frame_hdr->width * 2 < c->refs[refidx].p.p.p.w ||
                f->frame_hdr->height * 2 < c->refs[refidx].p.p.p.h ||
                f->frame_hdr->width > c->refs[refidx].p.p.p.w * 16 ||
                f->frame_hdr->height > c->refs[refidx].p.p.p.h * 16 ||
                f->seq_hdr->layout != c->refs[refidx].p.p.p.layout ||
                bpc != c->refs[refidx].p.p.p.bpc)
            {
                for (int j = 0; j < i; j++)
                    dav1d_thread_picture_unref(&f->refp[j]);
                res = DAV1D_ERR(EINVAL);
                goto error;
            }
            dav1d_thread_picture_ref(&f->refp[i], &c->refs[refidx].p);
            ref_coded_width[i] = c->refs[refidx].p.p.frame_hdr->width;
            if (f->frame_hdr->width != c->refs[refidx].p.p.p.w ||
                f->frame_hdr->height != c->refs[refidx].p.p.p.h)
            {
#define scale_fac(ref_sz, this_sz) \
    ((((ref_sz) << 14) + ((this_sz) >> 1)) / (this_sz))
                f->svc[i][0].scale = scale_fac(c->refs[refidx].p.p.p.w,
                                               f->frame_hdr->width);
                f->svc[i][1].scale = scale_fac(c->refs[refidx].p.p.p.h,
                                               f->frame_hdr->height);
                f->svc[i][0].step = (f->svc[i][0].scale + 8) >> 4;
                f->svc[i][1].step = (f->svc[i][1].scale + 8) >> 4;
            } else {
                f->svc[i][0].scale = f->svc[i][1].scale = 0;
            }
            f->gmv_warp_allowed[i] = f->frame_hdr->gmv[i].type > DAV1D_WM_TYPE_TRANSLATION &&
                                     !f->frame_hdr->force_integer_mv &&
                                     !dav1d_get_shear_params(&f->frame_hdr->gmv[i]) &&
                                     !f->svc[i][0].scale;
        }
    }

    // setup entropy
    if (f->frame_hdr->primary_ref_frame == DAV1D_PRIMARY_REF_NONE) {
        dav1d_cdf_thread_init_static(&f->in_cdf, f->frame_hdr->quant.yac);
    } else {
        const int pri_ref = f->frame_hdr->refidx[f->frame_hdr->primary_ref_frame];
        dav1d_cdf_thread_ref(&f->in_cdf, &c->cdf[pri_ref]);
    }
    if (f->frame_hdr->refresh_context) {
        res = dav1d_cdf_thread_alloc(c, &f->out_cdf, c->n_fc > 1);
        if (res < 0) goto error;
    }

    // FIXME qsort so tiles are in order (for frame threading)
    if (f->n_tile_data_alloc < c->n_tile_data) {
        dav1d_free(f->tile);
        assert(c->n_tile_data < INT_MAX / (int)sizeof(*f->tile));
        f->tile = dav1d_malloc(ALLOC_TILE, c->n_tile_data * sizeof(*f->tile));
        if (!f->tile) {
            f->n_tile_data_alloc = f->n_tile_data = 0;
            res = DAV1D_ERR(ENOMEM);
            goto error;
        }
        f->n_tile_data_alloc = c->n_tile_data;
    }
    memcpy(f->tile, c->tile, c->n_tile_data * sizeof(*f->tile));
    memset(c->tile, 0, c->n_tile_data * sizeof(*c->tile));
    f->n_tile_data = c->n_tile_data;
    c->n_tile_data = 0;

    // allocate frame
    res = dav1d_thread_picture_alloc(c, f, bpc);
    if (res < 0) goto error;
    dav1d_picture_ref(&f->cur, &f->sr_cur.p);

    // move f->cur into output queue
    if (c->n_fc == 1) {
        if (f->frame_hdr->show_frame || c->output_invisible_frames) {
            dav1d_thread_picture_ref(&c->out, &f->sr_cur);
            c->event_flags |= dav1d_picture_get_event_flags(&f->sr_cur);
        }
    } else {
        dav1d_thread_picture_ref(out_delayed, &f->sr_cur);
    }

    // ss_ver is set for 4:2:0, and ss_hor for 4:2:0 & 4:2:2
    f->ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    f->ss_hor = f->cur.p.layout - 1 < (unsigned) DAV1D_PIXEL_LAYOUT_I444 - 1;
    f->w4 = (f->frame_hdr->width + 3) >> 2;
    f->h4 = (f->frame_hdr->height + 3) >> 2;
    f->bw = ((f->frame_hdr->width + 7) >> 3) << 1;
    f->bh = ((f->frame_hdr->height + 7) >> 3) << 1;
    f->sb128w = (f->bw + 31) >> 5;
    f->sb128h = (f->bh + 31) >> 5;
    f->sb_shift = 4 + f->seq_hdr->sb128;
    f->sb_step = 16 << f->seq_hdr->sb128;
    f->sbh = (f->bh + f->sb_step - 1) >> f->sb_shift;
    f->b4_stride = (f->bw + 31) & ~31;
    f->bitdepth_max = (1 << f->cur.p.bpc) - 1;
    atomic_init(&f->task_thread.error, 0);
    const int uses_2pass = c->n_fc > 1;
    const int cols = f->frame_hdr->tiling.cols;
    const int rows = f->frame_hdr->tiling.rows;
    atomic_store(&f->task_thread.task_counter,
                 (cols * rows + f->sbh) << uses_2pass);

    // ref_mvs
    if (IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc) {
        f->mvs_ref = dav1d_ref_create_using_pool(c->refmvs_pool,
            sizeof(*f->mvs) * f->sb128h * 16 * (f->b4_stride >> 1));
        if (!f->mvs_ref) {
            res = DAV1D_ERR(ENOMEM);
            goto error;
        }
        f->mvs = f->mvs_ref->data;
        if (!f->frame_hdr->allow_intrabc) {
            for (int i = 0; i < 7; i++)
                f->refpoc[i] = f->refp[i].p.frame_hdr->frame_offset;
        } else {
            memset(f->refpoc, 0, sizeof(f->refpoc));
        }
        if (f->frame_hdr->use_ref_frame_mvs) {
            for (int i = 0; i < 7; i++) {
                const int refidx = f->frame_hdr->refidx[i];
                const int ref_w = ((ref_coded_width[i] + 7) >> 3) << 1;
                const int ref_h = ((f->refp[i].p.p.h + 7) >> 3) << 1;
                if (c->refs[refidx].refmvs != NULL &&
                    ref_w == f->bw && ref_h == f->bh)
                {
                    f->ref_mvs_ref[i] = c->refs[refidx].refmvs;
                    dav1d_ref_inc(f->ref_mvs_ref[i]);
                    f->ref_mvs[i] = c->refs[refidx].refmvs->data;
                } else {
                    f->ref_mvs[i] = NULL;
                    f->ref_mvs_ref[i] = NULL;
                }
                memcpy(f->refrefpoc[i], c->refs[refidx].refpoc,
                       sizeof(*f->refrefpoc));
            }
        } else {
            memset(f->ref_mvs_ref, 0, sizeof(f->ref_mvs_ref));
        }
    } else {
        f->mvs_ref = NULL;
        memset(f->ref_mvs_ref, 0, sizeof(f->ref_mvs_ref));
    }

    // segmap
    if (f->frame_hdr->segmentation.enabled) {
        // By default, the previous segmentation map is not initialised.
        f->prev_segmap_ref = NULL;
        f->prev_segmap = NULL;

        // We might need a previous frame's segmentation map. This
        // happens if there is either no update or a temporal update.
        if (f->frame_hdr->segmentation.temporal || !f->frame_hdr->segmentation.update_map) {
            const int pri_ref = f->frame_hdr->primary_ref_frame;
            assert(pri_ref != DAV1D_PRIMARY_REF_NONE);
            const int ref_w = ((ref_coded_width[pri_ref] + 7) >> 3) << 1;
            const int ref_h = ((f->refp[pri_ref].p.p.h + 7) >> 3) << 1;
            if (ref_w == f->bw && ref_h == f->bh) {
                f->prev_segmap_ref = c->refs[f->frame_hdr->refidx[pri_ref]].segmap;
                if (f->prev_segmap_ref) {
                    dav1d_ref_inc(f->prev_segmap_ref);
                    f->prev_segmap = f->prev_segmap_ref->data;
                }
            }
        }

        if (f->frame_hdr->segmentation.update_map) {
            // We're updating an existing map, but need somewhere to
            // put the new values. Allocate them here (the data
            // actually gets set elsewhere)
            f->cur_segmap_ref = dav1d_ref_create_using_pool(c->segmap_pool,
                sizeof(*f->cur_segmap) * f->b4_stride * 32 * f->sb128h);
            if (!f->cur_segmap_ref) {
                dav1d_ref_dec(&f->prev_segmap_ref);
                res = DAV1D_ERR(ENOMEM);
                goto error;
            }
            f->cur_segmap = f->cur_segmap_ref->data;
        } else if (f->prev_segmap_ref) {
            // We're not updating an existing map, and we have a valid
            // reference. Use that.
            f->cur_segmap_ref = f->prev_segmap_ref;
            dav1d_ref_inc(f->cur_segmap_ref);
            f->cur_segmap = f->prev_segmap_ref->data;
        } else {
            // We need to make a new map. Allocate one here and zero it out.
            const size_t segmap_size = sizeof(*f->cur_segmap) * f->b4_stride * 32 * f->sb128h;
            f->cur_segmap_ref = dav1d_ref_create_using_pool(c->segmap_pool, segmap_size);
            if (!f->cur_segmap_ref) {
                res = DAV1D_ERR(ENOMEM);
                goto error;
            }
            f->cur_segmap = f->cur_segmap_ref->data;
            memset(f->cur_segmap, 0, segmap_size);
        }
    } else {
        f->cur_segmap = NULL;
        f->cur_segmap_ref = NULL;
        f->prev_segmap_ref = NULL;
    }

    // update references etc.
    const unsigned refresh_frame_flags = f->frame_hdr->refresh_frame_flags;
    for (int i = 0; i < 8; i++) {
        if (refresh_frame_flags & (1 << i)) {
            if (c->refs[i].p.p.frame_hdr)
                dav1d_thread_picture_unref(&c->refs[i].p);
            dav1d_thread_picture_ref(&c->refs[i].p, &f->sr_cur);

            dav1d_cdf_thread_unref(&c->cdf[i]);
            if (f->frame_hdr->refresh_context) {
                dav1d_cdf_thread_ref(&c->cdf[i], &f->out_cdf);
            } else {
                dav1d_cdf_thread_ref(&c->cdf[i], &f->in_cdf);
            }

            dav1d_ref_dec(&c->refs[i].segmap);
            c->refs[i].segmap = f->cur_segmap_ref;
            if (f->cur_segmap_ref)
                dav1d_ref_inc(f->cur_segmap_ref);
            dav1d_ref_dec(&c->refs[i].refmvs);
            if (!f->frame_hdr->allow_intrabc) {
                c->refs[i].refmvs = f->mvs_ref;
                if (f->mvs_ref)
                    dav1d_ref_inc(f->mvs_ref);
            }
            memcpy(c->refs[i].refpoc, f->refpoc, sizeof(f->refpoc));
        }
    }

    if (c->n_fc == 1) {
        if ((res = dav1d_decode_frame(f)) < 0) {
            dav1d_thread_picture_unref(&c->out);
            for (int i = 0; i < 8; i++) {
                if (refresh_frame_flags & (1 << i)) {
                    if (c->refs[i].p.p.frame_hdr)
                        dav1d_thread_picture_unref(&c->refs[i].p);
                    dav1d_cdf_thread_unref(&c->cdf[i]);
                    dav1d_ref_dec(&c->refs[i].segmap);
                    dav1d_ref_dec(&c->refs[i].refmvs);
                }
            }
            goto error;
        }
    } else {
        dav1d_task_frame_init(f);
        pthread_mutex_unlock(&c->task_thread.lock);
    }

    return 0;
error:
    atomic_init(&f->task_thread.error, 1);
    dav1d_cdf_thread_unref(&f->in_cdf);
    if (f->frame_hdr->refresh_context)
        dav1d_cdf_thread_unref(&f->out_cdf);
    for (int i = 0; i < 7; i++) {
        if (f->refp[i].p.frame_hdr)
            dav1d_thread_picture_unref(&f->refp[i]);
        dav1d_ref_dec(&f->ref_mvs_ref[i]);
    }
    if (c->n_fc == 1)
        dav1d_thread_picture_unref(&c->out);
    else
        dav1d_thread_picture_unref(out_delayed);
    dav1d_picture_unref_internal(&f->cur);
    dav1d_thread_picture_unref(&f->sr_cur);
    dav1d_ref_dec(&f->mvs_ref);
    dav1d_ref_dec(&f->seq_hdr_ref);
    dav1d_ref_dec(&f->frame_hdr_ref);
    dav1d_data_props_copy(&c->cached_error_props, &c->in.m);

    for (int i = 0; i < f->n_tile_data; i++)
        dav1d_data_unref_internal(&f->tile[i].data);
    f->n_tile_data = 0;

    if (c->n_fc > 1)
        pthread_mutex_unlock(&c->task_thread.lock);

    return res;
}
