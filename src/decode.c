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
                              const int qidx, uint32_t (*dq)[3][2])
{
    // and then ac == dc
    for (int i = 0; i < (frame_hdr->segmentation.enabled ? 8 : 1); i++) {
        const int yac = frame_hdr->segmentation.enabled ?
            qidx + frame_hdr->segmentation.seg_data.d[i].delta_q : qidx;
        const int ydc = yac + frame_hdr->quant.ydc_delta;
        const int uac = yac + frame_hdr->quant.uac_delta;
        const int udc = yac + frame_hdr->quant.udc_delta;
        const int vac = yac + frame_hdr->quant.vac_delta;
        const int vdc = yac + frame_hdr->quant.vdc_delta;

        dq[i][0][0] = dq_lookup(seq_hdr->hbd, ydc);
        dq[i][0][1] = dq_lookup(seq_hdr->hbd, yac);
        dq[i][1][0] = dq_lookup(seq_hdr->hbd, udc);
        dq[i][1][1] = dq_lookup(seq_hdr->hbd, uac);
        dq[i][2][0] = dq_lookup(seq_hdr->hbd, vdc);
        dq[i][2][1] = dq_lookup(seq_hdr->hbd, vac);
    }
}

static uint16_t deblock_quant_thr(const int hbd, const int qidx) {
    const int qmax = 255 + 48 * hbd;
    return (dq_lookup(hbd, iclip(qidx, 0, qmax)) + 4) >> (3 + 6);
}

static uint16_t deblock_side_thr(const int hbd, const int qidx) {
    const int bitdepth_min_8 = 2 * hbd;
    const int q_ind = imin(imax(qidx - 24 * bitdepth_min_8, 0), 296 - 1);
    const int side_thr = dav1d_deblock_side_thresholds[q_ind];
    return imax(side_thr + (1 << 4 >> bitdepth_min_8), 0) >> (5 - bitdepth_min_8);
}

static void init_deblock_lut(const Dav1dSequenceHeader *const seq_hdr,
                             const Dav1dFrameHeader *const frame_hdr,
                             const int qidx, Av1FilterLUT *const lut)
{
    const int bitdepth_min_8 = 2 * seq_hdr->hbd;
    const int qmax = 255 + bitdepth_min_8;
    for (int i = 0; i < (frame_hdr->segmentation.enabled ? 8 : 1); i++) {
        const int yac = frame_hdr->segmentation.enabled ?
            iclip(qidx + frame_hdr->segmentation.seg_data.d[i].delta_q, 0, qmax) : qidx;
        for (int dir = 0; dir < 2; dir++) {
            const int dir_yac = yac + 8 * frame_hdr->loopfilter.delta_q_y[dir];
            lut->thr[dir][0][i] = deblock_quant_thr(seq_hdr->hbd, dir_yac);
            lut->thr[dir][1][i] = deblock_side_thr(seq_hdr->hbd, dir_yac);
        }
    }
}

static inline void read_amvd(Dav1dTileState *const ts, mv *const mv) {
    const int joint = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                          ts->cdf.m.amvd_joint, 3);
    if (!joint) {
        mv->n = 0;
        return;
    }
    if (joint & 2) {
        const int s = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                          ts->cdf.m.amvd_index[0], 7);
        mv->y = s < 3 ? 2 + s * 2 : 1 << s;
    } else mv->y = 0;
    if (joint & 1) {
        const int s = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                          ts->cdf.m.amvd_index[1], 7);
        mv->x = s < 3 ? 2 + s * 2 : 1 << s;
    } else mv->x = 0;
}

// mv_prec=0..6 for {8,4,2,f,h,q,e}pel
static inline void read_mv_residual(Dav1dTileState *const ts,
                                    CdfMvContext *const cdf_mv, mv *const mv,
                                    const int mv_prec)
{
    int sh_class;
    const int n_syms = 9 + mv_prec, h_syms = n_syms >> 1;

    if (dav1d_msac_decode_bool_adapt(&ts->msac, cdf_mv->shell_set)) {
        const int h_syms2 = n_syms - h_syms;
        sh_class = h_syms + 1 +
            dav1d_msac_decode_symbol_adapt8(&ts->msac,
                cdf_mv->shell_upper[mv_prec], imin(h_syms2, 7));
        if (mv_prec + sh_class == 21)
            sh_class += dav1d_msac_decode_bool_adapt(&ts->msac,
                                                     ts->cdf.mv.shell_tip);
    } else {
        sh_class = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                       cdf_mv->shell_lower[mv_prec], h_syms);
    }

    int sh_index;
    if (sh_class < 2) {
        sh_index = dav1d_msac_decode_bool_adapt(&ts->msac,
                       cdf_mv->shell_offset_low[sh_class]);
    } else if (sh_class == 2) {
        sh_index = dav1d_msac_decode_bool_adapt(&ts->msac,
                       cdf_mv->shell_offset_cl2);
        if (sh_index) {
            sh_index += dav1d_msac_decode_bool_bypass(&ts->msac);
            if (sh_index == 2)
                sh_index += dav1d_msac_decode_bool_bypass(&ts->msac);
        }
    } else {
        sh_index = 0;
        for (int i = 0, m = 1; i < sh_class; i++, m <<= 1) {
            sh_index |= m * dav1d_msac_decode_bool_adapt(&ts->msac,
                                cdf_mv->shell_offset_hi[i]);
        }
    }

    if (sh_class) sh_index += 1 << sh_class;
    if (!sh_index) {
        mv->n = 0;
        return;
    }

    int pair_index = 0;
    if (sh_index >= 2) {
        pair_index = dav1d_msac_decode_bool_adapt(&ts->msac,
                         cdf_mv->col_component[0]);
        if (pair_index && sh_index >= 4) {
            pair_index += dav1d_msac_decode_bool_adapt(&ts->msac,
                              cdf_mv->col_component[1]);
            if (pair_index == 2 && sh_index >= 6)
                pair_index += dav1d_msac_decode_uniform(&ts->msac,
                                  (sh_index >> 1) - 1);
        }
    }

    const int sh = 6 - mv_prec;
    if (pair_index * 2 == sh_index) {
        mv->x = mv->y = (sh_index >> 1) << sh;
    } else {
        const int b = dav1d_msac_decode_bool_adapt(&ts->msac,
                          cdf_mv->col_index[imin(sh_class, 3)]);
        if (b) {
            mv->y = pair_index << sh;
            mv->x = (sh_index - pair_index) << sh;
        } else {
            mv->x = pair_index << sh;
            mv->y = (sh_index - pair_index) << sh;
        }
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

static void derive_warpmv(const Dav1dTaskContext *const t,
                          const int have_top, const int have_left,
                          const int bw4, const int bh4,
                          const int w4, const int h4,
                          const int ref, const union mv mv,
                          Dav1dWarpedMotionParams *const wmp)
{
    int pts[8][2 /* in, out */][2 /* x, y */], np = 0;
    const refmvs_block *const r = &t->rt.r[(t->by & 63) * 128], *ra;

#define bs(rp) dav1d_block_dimensions[(rp)->bs]
    // FIXME manage ref1 also (once that's part of rp[])
#define add_sample(dx, dy, sx, sy, rp) do { \
    pts[np][0][0] = 16 * (2 * dx + sx * bs(rp)[0]) - 8; \
    pts[np][0][1] = 16 * (2 * dy + sy * bs(rp)[1]) - 8; \
    const union mv *const rmv = (rp)->mf & 2 ? (rp)->lmv.mv : (rp)->mv.mv; \
    pts[np][1][0] = pts[np][0][0] + rmv[0].x; \
    pts[np][1][1] = pts[np][0][1] + rmv[0].y; \
    np++; \
} while (0)

    const Dav1dFrameContext *const f = t->f;
    int have_topleft = 0;
    int have_topright = 0;
    const int is_not_sb_boundary = t->by & (f->sb_step - 1);
    if (have_top) {
        int off;
        if (is_not_sb_boundary) {
            ra = &t->rt.r[((t->by - 1) & 63) * 128];
            const refmvs_block *r2 = &ra[(t->bx & 63)];
            off = r2->bx4 - t->bx;
            have_topleft = !off;
            do {
                add_sample(off, 0, 1, -1, &r2[off]);
                off += bs(&r2[off])[0];
            } while (off < w4 && np < 8);
        } else {
            const refmvs_block *r2 = ra = t->rt.ra;
            off = r2->bx4 - t->bx;
            have_topleft = !off;
            do {
                add_sample(off, 0, 1, -1, &r2[off >> 1]);
                off += bs(&r2[off >> 1])[0];
            } while (off < w4 && np < 8);
        }
        have_topright = off <= bw4 && t->bx + bw4 < t->ts->tiling.col_end &&
            (!(t->by & (f->sb_step - 1)) || // top sb boundary
             ((t->bx + bw4) & (f->sb_step - 1) && // right sb boundary
              t->is_coded[0][(t->by - 1) & 63] & (1ULL << ((t->bx + bw4) & 63))));
    }

    if (np < 8 && have_left) {
        const refmvs_block *r2 = &r[(t->bx - 1) & 63];
        int off = r2->by4 - t->by;
        have_topleft &= !off;
        do {
            add_sample(0, off, -1, 1, &r2[off * 128]);
            off += bs(&r2[off * 128])[1];
        } while (off < h4 && np < 8);
    } else
        have_topleft = 0;

    if (is_not_sb_boundary) {
        if (np < 8 && have_topleft) // top/left
            add_sample(0, 0, -1, -1, &ra[((t->bx - 1) & 63)]);
        if (np < 8 && have_topright) // top/right
            add_sample(bw4, 0, 1, -1, &ra[((t->bx + bw4) & 63)]);
    } else {
        if (np < 8 && have_topleft) // top/left
            add_sample(0, 0, -1, -1, (t->bx & ~1) & (f->sb_step - 1) ?
                       &ra[(t->bx >> 1) - 1] : &t->rt.ra_tl);
        if (np < 8 && have_topright) // top/right
            add_sample(bw4, 0, 1, -1, &ra[(t->bx >> 1) + ((bw4 + 1) >> 1)]);
    }
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
        wmp->type = DAV1D_WM_TYPE_INVALID;
}

static void extend_warpmv(Dav1dTaskContext *const t,
                          const int x_off, const int y_off,
                          const uint8_t *const b_dim,
                          const Av1Block *const b,
                          Dav1dWarpedMotionParams *const wmp)
{
    const ptrdiff_t off = ((t->by + y_off) & 63) * 128 + ((t->bx + x_off) & 127);
    const refmvs_block *const r = &t->rt.r[off];
    int32_t *const m = wmp->matrix;

    if (r->mf & 2) {
        memcpy(m, r->m, sizeof(*m) * 6);
    } else if (r->mf & 1) {
        memcpy(m, t->f->frame_hdr->gmv[b->ref[0]].matrix, sizeof(*m) * 6);
    } else {
        memcpy(&m[2], &dav1d_default_wm_params.matrix[2], sizeof(*m) * 4);
        const int ref = r->ref.ref[0] - 1 != b->ref[0];
        m[0] = r->mv.mv[ref].x * (1 << 13);
        m[1] = r->mv.mv[ref].y * (1 << 13);
    }

    // extend warpmv using (quasi-)matrix from neighbour
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int sx = t->bx * 4 + 2 * bw4 - 1, sy = t->by * 4 + 2 * bh4 - 1;
    const int px = (sx << 16) + b->mv[0].x * (1 << 13);
    const int py = (sy << 16) + b->mv[0].y * (1 << 13);
    if (x_off >= 0) {
        assert(y_off == -1);
        const int ay = t->by * 4 - 1, sh = 1 + b_dim[3];
        const int64_t apx = (int64_t) m[2] * sx + (int64_t) m[3] * ay + m[0];
        const int64_t apy = (int64_t) m[4] * sx + (int64_t) m[5] * ay + m[1];
        m[3] = (int) ((px - apx + bh4 - (px < apx)) >> sh);
        m[5] = (int) ((py - apy + bh4 - (py < apy)) >> sh);
        m[3] += 0x20 - (m[3] < 0);
        m[5] += 0x20 - (m[5] < 0);
        m[3] &= ~0x3f;
        m[5] &= ~0x3f;
    } else {
        assert(x_off == -1 || !(t->by & (t->f->sb_step - 1)));
        const int ax = t->bx * 4 - 1, sh = 1 + b_dim[2];
        const int64_t lpx = (int64_t) m[2] * ax + (int64_t) m[3] * sy + m[0];
        const int64_t lpy = (int64_t) m[4] * ax + (int64_t) m[5] * sy + m[1];
        m[2] = (int) ((px - lpx + bh4 - (px < lpx)) >> sh);
        m[4] = (int) ((py - lpy + bh4 - (py < lpy)) >> sh);
        m[2] += 0x20 - (m[3] < 0);
        m[4] += 0x20 - (m[5] < 0);
        m[2] &= ~0x3f;
        m[4] &= ~0x3f;
    }
    dav1d_set_affine_mv2d(bw4, bh4, b->mv[0], wmp, t->bx, t->by);
    wmp->type = dav1d_get_shear_params(wmp) ? DAV1D_WM_TYPE_INVALID :
                                              DAV1D_WM_TYPE_AFFINE;
}

static void read_pal_indices(Dav1dTaskContext *const t, uint8_t *const pal_out,
                             const int pal_sz, const int sz[4])
{
    Dav1dTileState *const ts = t->ts;
    uint16_t (*const pal_cdf)[8] = ts->cdf.m.pal_idx[pal_sz - 2];
    uint8_t *const pal_idx = t->scratch.pal_idx_y;

    const int dir = imax(sz[2], sz[3]) < 64 &&
                    dav1d_msac_decode_bool_bypass(&ts->msac);
    const ptrdiff_t strides[2] = { dir ? 1 : sz[2], dir ? sz[2] : 1 };

    const int lim1 = sz[!dir], lim2 = sz[dir];
    int copy = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                   ts->cdf.m.pal_idx_identity[3], 2);
    if (copy == 2) return; // FIXME set error bit to abort decoding
    int prev_v = pal_idx[0] = dav1d_msac_decode_uniform(&ts->msac, pal_sz);
    if (copy == 1) {
        // FIXME if dir=0, maybe use memset()?
        for (int m = 1; m < lim2; m++)
            pal_idx[m * strides[1]] = prev_v;
    } else {
        int prev_h = prev_v;
        for (int m = 1; m < lim2; m++) {
            const int v = dav1d_msac_decode_symbol_adapt8(&ts->msac, pal_cdf[0],
                                                          pal_sz - 1);
            prev_h = pal_idx[m * strides[1]] = !v ? prev_h : v - (v <= prev_h);
        }
    }
    ptrdiff_t off = strides[0];
    for (int n = 1; n < lim1; n++, off += strides[0]) {
        copy = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                   ts->cdf.m.pal_idx_identity[copy], 2);
        if (copy == 2) {
            // FIXME if dir=0, maybe use memcpy()?
            for (int m = 0; m < lim2; m++)
                pal_idx[off + m * strides[1]] =
                    pal_idx[off - strides[0] + m * strides[1]];
        } else {
            const int v = dav1d_msac_decode_symbol_adapt8(&ts->msac, pal_cdf[0],
                                                          pal_sz - 1);
            const int next_v = pal_idx[off] = !v ? prev_v : v - (v <= prev_v);

            if (copy == 1) {
                // FIXME if dir=0, maybe use memset()?
                for (int m = 1; m < lim2; m++)
                    pal_idx[off + m * strides[1]] = next_v;
            } else {
                int prev_tl = prev_v, prev_l = next_v;
                for (int m = 1; m < lim2; m++) {
                    int prev_t = pal_idx[off - strides[0] + m * strides[1]];
                    int ctx;
                    if (prev_t == prev_l) {
                        ctx = 3 + (prev_tl == prev_l);
                    } else {
                        ctx = 1 + (prev_t == prev_tl || prev_l == prev_tl);
                    }
                    const int v = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                      pal_cdf[ctx], pal_sz - 1);
                    int p;
                    switch (ctx) {
                    default: assert(0);
                    case 1: {
                        switch (v) {
                        case 0:
                        case 1: p = v == dir ? prev_l : prev_t; break;
                        case 2: p = prev_tl; break;
                        default: {
                            const int s1 = prev_l < prev_t;
                            const int s2 = prev_l < prev_tl;
                            const int s3 = prev_t < prev_tl;
                            p = v - (v <= prev_l + s1 + s2) -
                                    (v <= prev_t + s3 + !s1) -
                                    (v <= prev_tl + !s2 + !s3);
                            break;
                        }}
                        break;
                    }
                    case 2: {
                        const int prev_l_or_t = prev_l + prev_t - prev_tl;
                        switch (v) {
                        case 0: p = prev_tl; break;
                        case 1: p = prev_l_or_t; break;
                        default: {
                            const int s = prev_l_or_t < prev_tl;
                            p = v - (v <= prev_l_or_t + s) - (v <= prev_tl + !s);
                            break;
                        }}
                        break;
                    }
                    case 3: {
                        switch (v) {
                        case 0: p = prev_l; break;
                        case 1: p = prev_tl; break;
                        default: {
                            const int s = prev_l < prev_tl;
                            p = v - (v <= prev_l + s) - (v <= prev_tl + !s);
                            break;
                        }}
                        break;
                    }
                    case 4:
                        p = !v ? prev_l : v - (v <= prev_l);
                        break;
                    }
                    prev_l = pal_idx[off + m * strides[1]] = p;
                    prev_tl = prev_t;
                }
            }
            prev_v = next_v;
        }
    }

    t->c->pal_dsp.pal_idx_finish(pal_out, pal_idx, sz[2], sz[3], sz[0], sz[1]);
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

#if DEBUG_BLOCK_INFO
static void debug_warp_matrix(const int depth,
                              const Dav1dFrameContext *const f,
                              const Dav1dTaskContext *const t,
                              const Av1Block *const b)
{
#define signabs(v) v < 0 ? '-' : ' ', abs(v)
    DEBUG_BLOCK_printf("%*s[ %c%x, %c%x | %c%x, %c%x, %c%x, %c%x ], "
                       "mv=y:%d,x:%d\n", depth, "",
                       signabs(t->warpmv.matrix[0]),
                       signabs(t->warpmv.matrix[1]),
                       signabs(t->warpmv.matrix[2]),
                       signabs(t->warpmv.matrix[3]),
                       signabs(t->warpmv.matrix[4]),
                       signabs(t->warpmv.matrix[5]),
                       b->mv[0].y, b->mv[0].x);
#undef signabs
}
#else
#define debug_warp_matrix(...)
#endif

static inline void splat_oneref_mv(DB_ONLY(const int depth)
                                   const Dav1dFrameContext *const f,
                                   Dav1dTaskContext *const t,
                                   const enum BlockSize bs,
                                   const Av1Block *const b,
                                   const int by4, const int bw4, const int bh4)
{
    refmvs_block *const rb = &t->rt.r[by4 * 128 + (t->bx & 127)];
    refmvs_block ALIGN(tmpl, 16);
    tmpl.ref.ref[0] = b->ref[0] + 1;
    tmpl.ref.ref[1] = -1;
    tmpl.bs = bs;
    tmpl.mv.mv[0] = b->mv[0];
    tmpl.bx4 = t->bx;
    tmpl.by4 = t->by;
    if (b->motion_mode > MM_INTERINTRA) {
        assert(bw4 > 1 && bh4 > 1 && b->inter_mode != GLOBALMV);
        tmpl.mf = 2;
        const int32_t *const mat = t->warpmv.matrix;
        const int64_t mvx = (int64_t) (mat[2] - 0x10000) * (t->bx + 1) * 4 +
                            (int64_t) mat[3] * (t->by + 1) * 4 + mat[0];
        const int64_t mvy = (int64_t) mat[4] * (t->bx + 1) * 4 + mat[1] +
                            (int64_t) (mat[5] - 0x10000) * (t->by + 1) * 4;
        f->c->refmvs_dsp.splat_warpmv(rb, &tmpl, mvy, mvx, &t->warpmv, bw4, bh4);
    } else {
        if (b->ref[0] == TIP_FRAME && f->seq_hdr->tip_refine_mv) {
            tmpl.mf = 4;
        } else {
            tmpl.mf = b->inter_mode == GLOBALMV;
        }
        f->c->refmvs_dsp.splat_mv(rb, &tmpl, bw4, bh4);
    }
}

static inline void splat_intrabc_mv(DB_ONLY(const int depth)
                                    const Dav1dFrameContext *const f,
                                    Dav1dTaskContext *const t,
                                    const enum BlockSize bs,
                                    const Av1Block *const b,
                                    const int by4, const int bw4, const int bh4)
{
    refmvs_block *const rb = &t->rt.r[by4 * 128 + (t->bx & 127)];
    refmvs_block ALIGN(tmpl, 16) = (refmvs_block) {
        .ref.ref = { 0, -1 },
        .mv.mv[0] = b->mv[0],
        .bs = bs,
        .mf = 0,
        .bx4 = t->bx,
        .by4 = t->by,
    };
    f->c->refmvs_dsp.splat_mv(rb, &tmpl, bw4, bh4);

    if (t->f->seq_hdr->refmv_bank)
        dav1d_refmvs_bank_add(&t->rt, bs, t->by, t->bx, b);
}

static inline void splat_tworef_mv(DB_ONLY(const int depth)
                                   const Dav1dFrameContext *const f,
                                   Dav1dTaskContext *const t,
                                   const enum BlockSize bs,
                                   const Av1Block *const b,
                                   const int by4, const int bw4, const int bh4)
{
    refmvs_block *const rb = &t->rt.r[by4 * 128 + (t->bx & 127)];
    refmvs_block ALIGN(tmpl, 16);
    tmpl.ref.ref[0] = b->ref[0] + 1;
    tmpl.ref.ref[1] = b->ref[1] + 1;
    tmpl.bs = bs;
    tmpl.mf = (b->cwp_idx + 4) << 3;
    tmpl.mv.mv[0] = b->mv[0];
    tmpl.mv.mv[1] = b->mv[1];
    tmpl.bx4 = t->bx;
    tmpl.by4 = t->by;
    if (b->motion_mode > MM_INTERINTRA) {
        assert(bw4 > 1 && bh4 > 1 && b->inter_mode != GLOBALMV);
        const int32_t *const mat = t->warpmv.matrix;
        const int64_t mvx = (int64_t) mat[2] * (t->bx + 1) * 4 +
                            (int64_t) mat[3] * (t->by + 1) * 4 + mat[0];
        const int64_t mvy = (int64_t) mat[4] * (t->bx + 1) * 4 +
                            (int64_t) mat[5] * (t->by + 1) * 4 + mat[1];
        f->c->refmvs_dsp.splat_warpmv(rb, &tmpl, mvy, mvx, &t->warpmv, bw4, bh4);
    } else {
        if (b->inter_mode >= OPFL_NEARMV_NEARMV ||
            (b->refine_mv && b->comp_type == COMP_INTER_AVG))
        {
            tmpl.mf |= 4;
        } else {
            tmpl.mf |= b->inter_mode == GLOBALMV_GLOBALMV;
        }
        f->c->refmvs_dsp.splat_mv(rb, &tmpl, bw4, bh4);
    }
}

static inline void splat_intraref(const Dav1dContext *const c,
                                  Dav1dTaskContext *const t,
                                  const enum BlockSize bs,
                                  const int by4, const int bw4, const int bh4)
{
    refmvs_block *const rb = &t->rt.r[by4 * 128 + (t->bx & 127)];
    refmvs_block ALIGN(tmpl, 16) = (refmvs_block) {
        .ref.ref = { -1, -1 },
        .mv.mv[0].n = INVALID_MV,
        .bs = bs,
        .mf = 0,
        .bx4 = t->bx,
        .by4 = t->by,
    };
    c->refmvs_dsp.splat_mv(rb, &tmpl, bw4, bh4);

    if (t->f->seq_hdr->refmv_bank)
        dav1d_refmvs_bank_update(&t->rt, bs, t->by, t->bx);
}

#if 0
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
        y = apply_sign64((llabs(tmp) + 128) >> 8, tmp) + 32;
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
#endif

static void read_tx_part(Dav1dTaskContext *const t,
                         DB_ONLY(const int depth) Av1Block *const b,
                         const enum BlockSize bs)
{
    Dav1dTileState *const ts = t->ts;
    const Dav1dFrameContext *const f = t->f;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = b_dim[0], bh4 = b_dim[1];

    b->tx_part = TX_PARTITION_NONE;
    b->uvtx = dav1d_max_txfm_size_for_bs[bs][f->cur.p.layout];
    if (f->frame_hdr->segmentation.lossless[b->seg_id] || b->skip_txfm) {
        // FIXME I believe lossless can be wht as well as idtx?
    } else {
        if (f->frame_hdr->txfm_mode == DAV1D_TX_SWITCHABLE &&
            bs != BS_4x4 && imax(bw4, bh4) <= 16)
        {
            const int inter = !b->intra || b->intrabc;
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
                               ts->cdf.m.tx_split[b->fsc][inter][szctx]);
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
                            ts->cdf.m.tx_part_2d[b->fsc][inter][ctx], 6);
                } else if (imax(bw4, bh4) >= 4) {
                    const int ctx = bw4 >= 4;
                    const int tx_part_4way =
                        dav1d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.tx_part_1d[b->fsc][inter][ctx]);
                    b->tx_part = TX_PARTITION_H + ctx + tx_part_4way * 2;
                } else {
                    assert(bs == BS_4x8 || bs == BS_8x4);
                    b->tx_part = bs == BS_4x8 ? TX_PARTITION_H :
                                                TX_PARTITION_V;
                }
            }
        }
    }
    DEBUG_BLOCK_printf("%*sPost-tx[%d]: r=%d\n",
                       depth, "", b->tx_part, ts->msac.rng);
}

static int read_wedge_idx(Dav1dTileState *const ts) {
    static const int8_t wedge_angle_dist2idx[20][4] = {
        { -1, 0, 1, 2 },     // WEDGE_0
        { 3, 4, 5, 6 },      // WEDGE_14
        { 7, 8, 9, 10 },     // WEDGE_27
        { 11, 12, 13, 14 },  // WEDGE_45
        { 15, 16, 17, 18 },  // WEDGE_63
        { -1, 19, 20, 21 },  // WEDGE_90
        { 22, 23, 24, 25 },  // WEDGE_117
        { 26, 27, 28, 29 },  // WEDGE_135
        { 30, 31, 32, 33 },  // WEDGE_153
        { 34, 35, 36, 37 },  // WEDGE_166
        { -1, 38, 39, 40 },  // WEDGE_180
        { -1, 41, 42, 43 },  // WEDGE_194
        { -1, 44, 45, 46 },  // WEDGE_207
        { -1, 47, 48, 49 },  // WEDGE_225
        { -1, 50, 51, 52 },  // WEDGE_243
        { -1, 53, 54, 55 },  // WEDGE_270
        { -1, 56, 57, 58 },  // WEDGE_297
        { -1, 59, 60, 61 },  // WEDGE_315
        { -1, 62, 63, 64 },  // WEDGE_333
        { -1, 65, 66, 67 },  // WEDGE_346
    };
    const int quad =
        dav1d_msac_decode_symbol_adapt4(&ts->msac, ts->cdf.m.wedge_quad, 3);
    const int angle = 5 * quad + dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                     ts->cdf.m.wedge_angle[quad], 4);
    const int dist = (angle - 1U >= 9U || angle == 5) ?
        1 + dav1d_msac_decode_symbol_adapt4(&ts->msac, ts->cdf.m.wedge_dist2, 2) :
        dav1d_msac_decode_symbol_adapt4(&ts->msac, ts->cdf.m.wedge_dist, 3);
    return wedge_angle_dist2idx[angle][dist];
}

static inline void jmvd_scale(union mv *const mv, const int amvd,
                              const int jmvd_scale_mode)
{
    if (amvd) {
        switch (jmvd_scale_mode) {
        default: assert(0);
        case 0: break;
        case 1:
            mv->y *= 2;
            mv->x *= 2;
            break;
        case 2:
            mv->y /= 2;
            mv->x /= 2;
            break;
        }
    } else {
        switch (jmvd_scale_mode) {
        default: assert(0);
        case 0: break;
        case 1: mv->y *= 2; break;
        case 2: mv->x *= 2; break;
        case 3: mv->y /= 2; break;
        case 4: mv->x /= 2; break;
        }
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
    const int bx4 = t->bx & 63, by4 = t->by & 63;
    const int bw4 = b_dim[0], bh4 = b_dim[1];
    const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
    const int have_left = t->bx > ts->tiling.col_start;
    const int have_top = t->by > ts->tiling.row_start;
    const int has_luma = lbs != BS_INVALID, has_chroma = cbs != BS_INVALID;
    int ss_hor, ss_ver, cbx4, cby4, cbw4, cbh4, cw4, ch4;
    if (has_chroma) {
        ss_ver = f->ss_ver;
        ss_hor = f->ss_hor;
        cbx4 = (t->cbx & 63) >> ss_hor;
        cby4 = (t->cby & 63) >> ss_ver;
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
            f->bd_fn.recon_b(t, DB_ONLY(depth) lbs, cbs, b);

#define set_ctx(rep_macro) \
            rep_macro(edge->mode, off, b->y_mode); \
            rep_macro(edge->intra, off, 1)
            BlockContext *edge = t->a;
            for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
                case_set(b_dim[2 + i]);
            }
#undef set_ctx
            if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
#if 0
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
#endif
            }

            if (has_chroma) {
                uint8_t uv_mode = b->uv_mode;
                dav1d_memset_pow2[ulog2(cbw4)](&t->a->uvmode[cbx4], uv_mode);
                dav1d_memset_pow2[ulog2(cbh4)](&t->l.uvmode[cby4], uv_mode);
            }
        } else {
            if (IS_INTER_OR_SWITCH(f->frame_hdr) /* not intrabc */ &&
                b->comp_type == COMP_INTER_NONE && b->motion_mode == MM_WARP_CAUSAL)
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
                    debug_warp_matrix(depth, f, t, b);
                }
            }
            f->bd_fn.recon_b(t, DB_ONLY(depth) lbs, cbs, b);

            BlockContext *edge = t->a;
            for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
#define set_ctx(rep_macro) \
                rep_macro(edge->filter, off, b->filter); \
                rep_macro(edge->intra, off, 0)
                case_set(b_dim[2 + i]);
#undef set_ctx
            }

            if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
#if 0
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
#endif
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
        idx += have_top;
    }

    // skip_mode
    if ((!seg || (!seg->globalmv && seg->ref == -1 && !seg->skip)) &&
        f->frame_hdr->skip_mode_enabled && bw4 * bh4 > 2 && !t->intra_region)
    {
        const int ctx = nx[0]->skip_mode[xoff[0]] + nx[1]->skip_mode[xoff[1]];
        b->skip_mode = dav1d_msac_decode_bool_adapt(&ts->msac,
                           ts->cdf.m.skip_mode[ctx]);
        DEBUG_BLOCK_printf("%*sPost-skip_mode[ctx=%d,%d]: r=%d\n",
                           depth, "", ctx, b->skip_mode, ts->msac.rng);
    } else {
        b->skip_mode = 0;
    }

    if (b->skip_mode) {
        b->intra = 0;
    } else if (IS_INTER_OR_SWITCH(f->frame_hdr) && !t->intra_region) {
        if (f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400 && lbs != cbs) {
            // mixed-intra/inter regions in inter frames with chroma planes
            // of different sizes (in AVM language: with offsets) are inter
            b->intra = 0;
        } else if (seg && (seg->ref >= 0 || seg->globalmv)) {
            b->intra = !seg->ref;
        } else {
            const int ictx = get_intra_ctx(nx, xoff, idx);
            b->intra = !dav1d_msac_decode_bool_adapt(&ts->msac,
                            ts->cdf.m.intra[ictx]);
            DEBUG_BLOCK_printf("%*sPost-is_inter[ctx=%d,%d]: r=%d\n",
                               depth, "", ictx, !b->intra, ts->msac.rng);
        }
    } else {
        b->intra = 1;
    }

    const BlockContext *nb[2];
    int boff[2];

    b->intrabc = 0;
    if (has_luma) {
        // get "spatial neighbours", depending on edge availability;
        // do not cross SB boundaries vertically
        const int have_top_in_sb = !!(t->by & (f->sb_step - 1));

        int idx = 0;
        if (have_left && bh4 == h4) {
            nb[0] = &t->l;
            boff[0] = by4 + bh4 - 1;
            idx++;
        }
        if (have_top_in_sb && bw4 == w4) {
            nb[idx] = t->a;
            boff[idx] = bx4 + bw4 - 1;
            idx++;
        }
        if (have_left && idx < 2) {
            nb[idx] = &t->l;
            boff[idx] = by4;
            idx++;
        }
        if (have_top_in_sb && idx < 2) {
            nb[idx] = t->a;
            boff[idx] = bx4;
            idx++;
        }
        if (idx < 2) {
            boff[idx] = -1;
            if (!idx) boff[1] = -1;
        }

        if (f->frame_hdr->allow_intrabc && imin(bw4, bh4) < 16 &&
            b->intra && !t->intra_region)
        {
            const int ctx = (boff[0] == -1 ? 0 : nb[0]->intrabc[boff[0]]) +
                            (boff[1] == -1 ? 0 : nb[1]->intrabc[boff[1]]);
            b->intrabc = dav1d_msac_decode_bool_adapt(&ts->msac,
                             ts->cdf.m.intrabc[ctx]);
            DEBUG_BLOCK_printf("%*sPost-intrabc[ctx=%d,%d]: r=%d\n",
                               depth, "", ctx, b->intrabc, ts->msac.rng);
        }
    }

    // skip_txfm
    if ((seg && seg->skip)) {
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
        // FIXME some of these can be pre-calculated at the start of a frame
        const int gdf_bs = f->frame_hdr->frame_type == DAV1D_FRAME_TYPE_KEY ?
                           32 : imax(32, 16 << f->frame_hdr->sb128);
        if (!((t->bx | t->by) & (gdf_bs - 1))) {
            int idx = ((t->by & 16) >> 3) + ((t->bx & 16) >> 4);
            if (f->frame_hdr->gdf.enabled == DAV1D_ADAPTIVE &&
                imax(f->cur.p.w, f->cur.p.h) > 4 * gdf_bs)
            {
                for (int y = 0; y < bh4; y += gdf_bs, idx += 2) {
                    for (int x = 0; x < bw4; x += gdf_bs) {
                        // FIXME separate storage sites for 256x256 blocks
                        t->lf_mask->gdf[idx + !!x] =
                            dav1d_msac_decode_bool_adapt(&ts->msac,
                                                         ts->cdf.m.gdf);
                        DEBUG_BLOCK_printf("%*sPost-gdf[y=%d,x=%d,gdf=%d]: r=%d\n",
                                           depth, "", t->by + y, t->bx + x,
                                           t->lf_mask->gdf[idx + !!x],
                                           ts->msac.rng);
                    }
                }
            } else
                for (int y = 0; y < bh4; y += gdf_bs, idx += 2)
                    for (int x = 0; x < bw4; x += gdf_bs)
                        t->lf_mask->gdf[idx + !!x] = !!f->frame_hdr->gdf.enabled;
        }
    }

    // cdef index
    if (f->frame_hdr->cdef.enabled &&
        (!b->skip_txfm || f->frame_hdr->cdef.on_skiptx))
    {
        const int idx = ((t->bx & 0x30) >> 4) + ((t->by & 0x30) >> 2);
        int8_t *const cdef_ptr = &t->lf_mask->cdef_idx[idx];
        if (*cdef_ptr == -1) {
            int v;
            if (f->frame_hdr->cdef.n_strengths == 1) {
                v = 0;
            } else {
                const int left_cdef_idx =
                    t->bx - 16 < ts->tiling.col_start ? -1 :
                    idx & 3 ? cdef_ptr[-1] :
                    t->lf_mask[-1].cdef_idx[idx + 3];
                const int top_cdef_idx =
                    t->by - 16 < ts->tiling.row_start ? -1 :
                    idx & 0xc ? cdef_ptr[-4] :
                    t->lf_mask[-f->sb256w].cdef_idx[idx + 12];
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
                    v = 1 + dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                ts->cdf.m.cdef_idx[rem], rem + 1);
                }
                DEBUG_BLOCK_printf("%*sPost-cdef_idx[ctx=%d,%d]: r=%d\n",
                                   depth, "", ctx, v, ts->msac.rng);
            }
            const int splat_idx = imax(0, b_dim[2] - 4);
            dav1d_memset_pow2[splat_idx](cdef_ptr, v);
            if (bh4 >= 32) {
                dav1d_memset_pow2[splat_idx](&cdef_ptr[4], v);
                if (bh4 == 64) {
                    dav1d_memset_pow2[splat_idx](&cdef_ptr[8], v);
                    dav1d_memset_pow2[splat_idx](&cdef_ptr[12], v);
                }
            }
        }
    }

    // ccso
    if (has_luma && !((t->bx | t->by) & 63)) {
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
                const int ctx = t->bx - 64 >= ts->tiling.col_start ?
                                t->lf_mask[-1].ccso[p] * 2 : 0;
                t->lf_mask->ccso[p] = dav1d_msac_decode_bool_adapt(&ts->msac,
                                            ts->cdf.m.ccso[p][ctx]);
                DEBUG_BLOCK_printf("%*sPost-ccso[pl=%c,ctx=%d,%d]: r=%d\n",
                                   depth, "", "yuv"[p], ctx,
                                   t->lf_mask->ccso[p], ts->msac.rng);
            }
        }
    }

    // delta-q/lf
    if (!((t->bx | t->by) & (63 >> (2 - f->frame_hdr->sb128)))) {
        const int prev_qidx = ts->last_qidx;
        const int have_delta_q = f->frame_hdr->delta.q.present &&
                                 (bs != f->root_bs || !b->skip_txfm);

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

            // TODO: will be removed in v13
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
                y_mode_ctx = (w4 == bw4 && t->a->midx[bx4 + bw4 - 1] != 0xff) +
                             (h4 == bh4 && t->l.midx[by4 + bh4 - 1] != 0xff);
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
                    if (h4 == bh4 && t->l.midx[by4 + bh4 - 1] != 0xff) {
                        const int lmidx = t->l.midx[by4 + bh4 - 1];
                        *ptr++ = lmidx;
                        mask |= 1ULL << lmidx;
                    }
                    if (w4 == bw4 && t->a->midx[bx4 + bw4 - 1] != 0xff) {
                        const int amidx = t->a->midx[bx4 + bw4 - 1];
                        if (amidx != custom_mode_list_y[0]) {
                            *ptr++ = amidx;
                            mask |= 1ULL << amidx;
                        }
                    }
                    int n_dirs = (int)(ptr - custom_mode_list_y);
                    if (n_dirs > 0) {
                        reorder = custom_mode_list_y;
                        if (bw4 * bh4 > 4 && dir_y_mode_idx >= n_dirs) {
                            // add surrounding [-3..+3] angles
                            for (int i = 1; i < 5; i++) {
                                for (int n = 0; n < n_dirs; n++) {
                                    const int cmidx = custom_mode_list_y[n];
                                    for (int delta = -i, j = 0; j < 2; delta = +i, j++) {
                                        // FIXME replace modulo with fastdiv
                                        const int dmidx = (cmidx + delta + 56) % 56;
                                        if (!(mask & (1ULL << dmidx))) {
                                            *ptr++ = dmidx;
                                            mask |= 1ULL << dmidx;
                                        }
                                    }
                                }
                            }
                        }

                        n_dirs = (int)(ptr - custom_mode_list_y);
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
                const int ctx = (IS_INTER_OR_SWITCH(f->frame_hdr) &&
                                 !t->intra_region) ? 3 :
                                (boff[0] == -1 ? 0 : nb[0]->fsc[boff[0]]) +
                                (boff[1] == -1 ? 0 : nb[1]->fsc[boff[1]]);
                b->fsc = dav1d_msac_decode_bool_adapt(&ts->msac,
                             ts->cdf.m.fsc[ctx][sz_ctx]);
                DEBUG_BLOCK_printf("%*sPost-fsc[ctx=%d|%d,%d]: r=%d\n",
                                   depth, "", ctx, sz_ctx, b->fsc, ts->msac.rng);
            }

            b->mrl_index = b->multi_mrl = 0;
            if (midx != 0xff /* directional mode */) {
                const int ctx = (boff[0] == -1 ? 0 : nb[0]->mrl[boff[0]]) +
                                (boff[1] == -1 ? 0 : nb[1]->mrl[boff[1]]);
                b->mrl_index = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                   ts->cdf.m.mrl_index[ctx], 3);
                DEBUG_BLOCK_printf("%*sPost-mrl_index[ctx=%d,%d]: r=%d\n",
                                   depth, "", ctx, b->mrl_index, ts->msac.rng);
                if (b->mrl_index > 0) {
                    const int ctx2 =
                        (boff[0] == -1 ? 0 : nb[0]->multi_mrl[boff[0]]) +
                        (boff[1] == -1 ? 0 : nb[1]->multi_mrl[boff[1]]);
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
                (imax(bw4, bh4) > 16 || !t->sdp_cfl_disallowed) &&
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
                    midx = t->luma_intra_dir_mode_map[(t->by & 15) * 16 +
                                                      (t->bx & 15)];
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
                        static const uint8_t intra_dir_mode_y_to_uv_idx[] = {
                            2, 4, 0, 5, 3, 6, 1, 7
                        };
                        int idx = uv_mode_idx - 5 - uv_mode_ctx;
                        idx += uv_mode_ctx &&
                               idx >= intra_dir_mode_y_to_uv_idx[midx / 7];
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
                if (f->seq_hdr->mhccp && imax(cbw4, cbh4) <= 8 && cbw4 * cbh4 > 1 &&
                    dav1d_msac_decode_bool_adapt(&ts->msac, ts->cdf.m.mhccp))
                {
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
                            b->cfl_alpha[0] = dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                    ts->cdf.m.cfl_alpha[ctx], 7) + 1;
                            if (sign_u == 1) b->cfl_alpha[0] = -b->cfl_alpha[0];
                        }
                        if (sign_v) {
                            const int ctx = (sign_v == 2) * 3 + sign_u;
                            b->cfl_alpha[1] = dav1d_msac_decode_symbol_adapt8(&ts->msac,
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

        b->pal_sz = 0;
        if (has_luma) {
            if (f->frame_hdr->allow_screen_content_tools &&
                b->y_mode == DC_PRED && imax(bw4, bh4) <= 16 && bw4 + bh4 >= 4)
            {
                const int use_y_pal = dav1d_msac_decode_bool_adapt(&ts->msac,
                                          ts->cdf.m.pal_y);
                if (use_y_pal) {
                    f->bd_fn.read_pal_plane(DB_ONLY(depth) t, b, bx4, by4);
                } else
                    DEBUG_BLOCK_printf("%*sPost-ypal[0]: r=%d\n",
                                       depth, "", ts->msac.rng);
            }

            b->dip = 0;
            if (b->y_mode == DC_PRED && f->seq_hdr->intra_dip &&
                !b->pal_sz && imin(bw4, bh4) >= 2 && bw4 * bh4 >= 8)
            {
                const int ctx = (boff[0] == -1 ? 0 : nb[0]->dip[boff[0]]) +
                                (boff[1] == -1 ? 0 : nb[1]->dip[boff[1]]);
                b->dip = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                      ts->cdf.coef.dip[ctx]);
                if (b->dip) {
                    const int tp = dav1d_msac_decode_bool_bypass(&ts->msac);
                    const int m =
                        dav1d_msac_decode_symbol_adapt8(&ts->msac, ts->cdf.m.dip_mode, 5);
                    b->dip = (tp << 4) | (m + 1);
                }
                DEBUG_BLOCK_printf("%*sPost-dip[ctx=%d,%d,tp=%d,mode=%d]: r=%d\n",
                                   depth, "", ctx, !!b->dip, b->dip >> 4,
                                   (b->dip - !!b->dip) & 7, ts->msac.rng);
            }

            if (IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc)
                splat_intraref(f->c, t, bs, by4, bw4, bh4);

            if (b->pal_sz) {
                uint8_t *pal_idx;
                if (t->frame_thread.pass) {
                    const int p = t->frame_thread.pass & 1;
                    assert(ts->frame_thread[p].pal_idx);
                    pal_idx = ts->frame_thread[p].pal_idx;
                    ts->frame_thread[p].pal_idx += bw4 * bh4 * 8;
                } else
                    pal_idx = t->scratch.pal_idx_y;
                read_pal_indices(t, pal_idx, b->pal_sz,
                                 (int[4]) { w4 * 4, h4 * 4, bw4 * 4, bh4 * 4 });
                DEBUG_BLOCK_printf("%*sPost-y-pal-indices: r=%d\n",
                                   depth, "", ts->msac.rng);
            }
            read_tx_part(t, DB_ONLY(depth) b, bs);
        }

        // reconstruction
        if (t->frame_thread.pass == 1) {
            f->bd_fn.read_coef_blocks(t, bs, b);
        } else {
            t->pb.a_is_sm = sm_flag(t->a, bx4);
            t->pb.l_is_sm = sm_flag(&t->l, by4);
            f->bd_fn.recon_b(t, DB_ONLY(depth) lbs, cbs, b);
        }

        if (has_luma) {
            // update contexts
            BlockContext *edge = t->a;
            for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
#define set_ctx(rep_macro) \
                rep_macro(edge->fsc, off, b->fsc); \
                rep_macro(edge->mode, off, b->y_mode); \
                rep_macro(edge->midx, off, midx); \
                rep_macro(edge->mrl, off, !!b->mrl_index); \
                rep_macro(edge->multi_mrl, off, b->multi_mrl); \
                rep_macro(edge->dip, off, !!b->dip); \
                rep_macro(edge->pal_sz, off, b->pal_sz); \
                rep_macro(edge->seg_pred, off, seg_pred); \
                rep_macro(edge->skip_mode, off, 0); \
                rep_macro(edge->intra, off, 1); \
                rep_macro(edge->intrabc, off, 0); \
                rep_macro(edge->morph_pred, off, 0); \
                rep_macro(edge->skip_txfm, off, b->skip_txfm); \
                if (IS_INTER_OR_SWITCH(f->frame_hdr)) { \
                    rep_macro(edge->amvd, off, 0); \
                    rep_macro(edge->mvprec, off, 0); \
                    rep_macro(edge->motion_mode, off, 0); \
                    rep_macro(edge->comp_type, off, COMP_INTER_NONE); \
                    rep_macro(edge->ref[0], off, ((uint8_t) -1)); \
                    rep_macro(edge->ref[1], off, ((uint8_t) -1)); \
                }
                case_set(b_dim[2 + i]);
#undef set_ctx
            }
        }
        if (b->pal_sz)
            f->bd_fn.copy_pal_block_y(t, bx4, by4, bw4, bh4);
        if (has_chroma) {
            uint8_t uv_mode = b->uv_mode;
            dav1d_memset_pow2[ulog2(cbw4)](&t->a->uvmode[cbx4], uv_mode);
            dav1d_memset_pow2[ulog2(cbh4)](&t->l.uvmode[cby4], uv_mode);
        }
    } else if (b->intrabc) {
        // intra block copy
        refmvs_candidate mvstack[6];
        int n_mvs;
        dav1d_refmvs_find(&t->rt, mvstack, NULL, &n_mvs,
                          (union refmvs_refpair) { .ref = { 0, -1 }},
                          bs, t->by, t->bx);
#if DEBUG_BLOCK_INFO
        if (BLOCK_TO_DEBUG) {
            printf("%*sfind_mv_refs(intra)\n", depth, "");
            for (int n = 0; n < n_mvs; n++)
                printf("%*smv[%d/%d]: y=%d,x=%d,w=%d\n",
                       depth + 1, "", n, n_mvs, mvstack[n].mv.mv[0].y,
                       mvstack[n].mv.mv[0].x, mvstack[n].weight);
        }
#endif

        const int is_refmv = dav1d_msac_decode_bool_adapt(&ts->msac,
                                 ts->cdf.m.intrabc_mode);
        int drl_idx;
        for (drl_idx = 0; drl_idx < f->frame_hdr->max_bvp_drl_bits; drl_idx++)
            if (!dav1d_msac_decode_bool_bypass(&ts->msac)) break;

        b->ref[0] = b->ref[1] = -1;
        b->mv[0] = mvstack[drl_idx].mv.mv[0];
        if (!b->mv[0].n) {
            // I don't know if this can actually happen, but AVM has code here
            // to force the refmv to a nonzero value
            const int sbsz = 64 << f->frame_hdr->sb128;
            if (t->by - f->sb_step < ts->tiling.row_start) {
                b->mv[0].x = -(8 * (sbsz + 256));
            } else {
                b->mv[0].y = -(8 * sbsz);
            }
        }
        int is_qpel = !f->frame_hdr->force_integer_mv;
        if (!is_refmv && !f->frame_hdr->force_integer_mv) {
            is_qpel = dav1d_msac_decode_bool_adapt(&ts->msac,
                          ts->cdf.m.intrabc_precision);
        }
        if (!is_refmv) {
            mv diff;
            read_mv_residual(ts, &ts->cdf.dmv, &diff, 3 + 2 * is_qpel);
            if (diff.y) {
                const int s = dav1d_msac_decode_bool_bypass(&ts->msac);
                if (s) diff.y = -diff.y;
            }
            if (diff.x) {
                const int s = dav1d_msac_decode_bool_bypass(&ts->msac);
                if (s) diff.x = -diff.x;
            }
            if (!is_qpel) fix_int_mv_precision(&b->mv[0]);
            b->mv[0].x += diff.x;
            b->mv[0].y += diff.y;
            DEBUG_BLOCK_printf("%*sPost-mvdiff[y:%d,x:%d]: r=%d\n",
                               depth, "", diff.y, diff.x, ts->msac.rng);
        }
        int morphctx = -1;
        b->morph_pred = 0;
        if (!(f->frame_hdr->frame_type & 1) && f->seq_hdr->bawp &&
            f->frame_hdr->allow_screen_content_tools)
        {
            morphctx = (boff[0] == -1 ? 0 : nb[0]->morph_pred[boff[0]]) +
                       (boff[1] == -1 ? 0 : nb[1]->morph_pred[boff[1]]);
            b->morph_pred = dav1d_msac_decode_bool_adapt(&ts->msac,
                                ts->cdf.m.morph_pred[morphctx]);
        }

#if 0
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

        const int sbx = (t->bx >> (4 + f->frame_hdr->sb128)) << (6 + f->frame_hdr->sb128);
        const int sby = (t->by >> (4 + f->frame_hdr->sb128)) << (6 + f->frame_hdr->sb128);
        const int sb_size = 1 << (6 + f->frame_hdr->sb128);
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
#endif

        DEBUG_BLOCK_printf("%*sPost-intrabc_info[mode=%d,drl=%d,mv=y:%d,x:%d,"
                           "prec=%d,morphctx=%d,morph=%d]: r=%d\n",
                           depth, "", is_refmv, drl_idx, b->mv[0].y, b->mv[0].x,
                           is_qpel, morphctx, b->morph_pred, ts->msac.rng);
        if (has_luma)
            splat_intrabc_mv(DB_ONLY(depth) f, t, bs, b, by4, bw4, bh4);
        read_tx_part(t, DB_ONLY(depth) b, bs);

        // reconstruction
        if (t->frame_thread.pass == 1) {
            f->bd_fn.read_coef_blocks(t, bs, b);
            b->filter = DAV1D_FILTER_BILINEAR;
        } else {
            f->bd_fn.recon_b(t, DB_ONLY(depth) lbs, cbs, b);
        }

        if (has_luma) {
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
                rep_macro(edge->seg_pred, off, seg_pred); \
                rep_macro(edge->skip_mode, off, 0); \
                rep_macro(edge->intrabc, off, 1); \
                rep_macro(edge->morph_pred, off, b->morph_pred); \
                rep_macro(edge->intra, off, 1); \
                rep_macro(edge->skip_txfm, off, b->skip_txfm); \
                if (IS_INTER_OR_SWITCH(f->frame_hdr)) { \
                    rep_macro(edge->amvd, off, 0); \
                    rep_macro(edge->mvprec, off, 0); \
                    rep_macro(edge->comp_type, off, COMP_INTER_NONE); \
                    rep_macro(edge->motion_mode, off, 0); \
                    rep_macro(edge->ref[0], off, ((uint8_t) -1)); \
                    rep_macro(edge->ref[1], off, ((uint8_t) -1)); \
                }
                case_set(b_dim[2 + i]);
#undef set_ctx
            }
        }
        if (has_chroma) {
            dav1d_memset_pow2[ulog2(cbw4)](&t->a->uvmode[cbx4], DC_PRED);
            dav1d_memset_pow2[ulog2(cbh4)](&t->l.uvmode[cby4], DC_PRED);
        }
    } else {
        // inter-specific mode/mv coding
        int is_comp, has_subpel_filter, is_tip = 0;

        if (!b->skip_mode && f->frame_hdr->tip.frame_mode &&
            cbs == lbs && imax(bw4, bh4) >= 2)
        {
            const int ctx = (idx < 1 ? 0 : nx[0]->ref[0][xoff[0]] == TIP_FRAME) +
                            (idx < 2 ? 0 : nx[1]->ref[0][xoff[1]] == TIP_FRAME);
            is_tip = dav1d_msac_decode_bool_adapt(&ts->msac, ts->cdf.m.tip[ctx]);
            DEBUG_BLOCK_printf("%*sPost-tip[ctx=%d,%d]: r=%d\n",
                               depth, "", ctx, is_tip, ts->msac.rng);
        }

        if (b->skip_mode) {
            is_comp = 1;
        } else if (!is_tip &&
                   (!seg || (seg->ref == -1 && !seg->globalmv && !seg->skip)) &&
                   f->frame_hdr->switchable_comp_refs && bw4 * bh4 >= 4)
        {
            const int ctx = get_comp_ctx(nx, xoff, idx, &f->refdir_with_intra[1]);
            is_comp = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                   ts->cdf.m.comp[ctx]);
        } else {
            is_comp = 0;
        }

        static const uint8_t mv_prec_tbl[][3] = {
            { 3, 1, 0 },
            { 4, 3, 1 },
        };

        int mvprec_def = 1, amvd = 0;
        b->motion_mode = MM_TRANSLATION;
        b->refine_mv = 0;
        if (b->skip_mode) {
            const int max_drl_bits = f->frame_hdr->max_drl_bits;
            int drl_idx = 0;
            for (int ctx = 0; drl_idx < max_drl_bits; drl_idx++, ctx += ctx < 2) {
                if (!dav1d_msac_decode_bool_adapt(&ts->msac,
                         ts->cdf.m.skip_mode_drl_idx[ctx]))
                {
                    break;
                }
            }
            DEBUG_BLOCK_printf("%*sPost-drl[%d,%d]: r=%d\n",
                               depth, "", drl_idx, drl_idx, ts->msac.rng);

            b->ref[0] = f->skip_mode_refs[0];
            b->ref[1] = f->skip_mode_refs[1];
            for (int n = 0; n < idx; n++) {
                if (nx[n]->ref[0][xoff[n]] == TIP_FRAME) {
                    b->ref[0] = imin(f->frame_hdr->tip.refs[0],
                                     f->frame_hdr->tip.refs[1]);
                    b->ref[1] = imax(f->frame_hdr->tip.refs[0],
                                     f->frame_hdr->tip.refs[1]);
                    break;
                } else if (nx[n]->ref[1][xoff[n]] != -1) {
                    b->ref[0] = nx[n]->ref[0][xoff[n]];
                    b->ref[1] = nx[n]->ref[1][xoff[n]];
                    break;
                } else if (nx[n]->ref[0][xoff[n]] != -1) break;
            }
            b->comp_type = COMP_INTER_AVG;
            b->inter_mode = NEARMV_NEARMV;
            has_subpel_filter = 0;

            refmvs_candidate mvstack[6];
            int n_mvs;
            dav1d_refmvs_find(&t->rt, mvstack, NULL, &n_mvs,
                              (union refmvs_refpair) { .ref = {
                                    b->ref[0] + 1, b->ref[1] + 1 }},
                              bs, t->by, t->bx);
#if DEBUG_BLOCK_INFO
            if (BLOCK_TO_DEBUG) {
                printf("%*sfind_mv_refs(%d,%d)\n", depth, "", b->ref[0], b->ref[1]);
                for (int n = 0; n < n_mvs; n++)
                    printf("%*smv[%d/%d]: y=%d,x=%d,y2=%d,x2=%d,w=%d\n",
                           depth + 1, "", n, n_mvs, mvstack[n].mv.mv[0].y,
                           mvstack[n].mv.mv[0].x, mvstack[n].mv.mv[1].y,
                           mvstack[n].mv.mv[1].x, mvstack[n].weight);
            }
#endif

            b->mv[0] = mvstack[drl_idx].mv.mv[0];
            b->mv[1] = mvstack[drl_idx].mv.mv[1];
            b->cwp_idx = mvstack[drl_idx].cwp_idx;
            mv_reduce_prec(&b->mv[0], 3 + f->frame_hdr->mv_precision);
            mv_reduce_prec(&b->mv[0], 3 + f->frame_hdr->mv_precision);
        } else if (is_comp) {
            const int n_refs = f->frame_hdr->n_ref_frames;
            if (n_refs > 1) {
                const int same_refs = f->seq_hdr->num_same_ref_comp;
                int n = 0;
                uint8_t cnt[9] = { 0 };
                if (idx > 0) {
                    cnt[nx[0]->ref[0][xoff[0]] + 1]++;
                    cnt[nx[0]->ref[1][xoff[0]] + 1]++;
                    if (idx > 1) {
                        cnt[nx[1]->ref[0][xoff[1]] + 1]++;
                        cnt[nx[1]->ref[1][xoff[1]] + 1]++;
                    }
                }
                int cnt_rem = idx * 2 - cnt[0] - cnt[8];
                for (int i = 0, maybe_same_ref = !!same_refs, dir;
                     i < n_refs + n - 2 + maybe_same_ref; i++)
                {
                    int bit;
                    const int cnt_cur = cnt[i + 1];
                    cnt_rem -= cnt_cur;
                    if (!n && (i == 2 || (i >= n_refs - 2 && i + 1 >= same_refs))) {
                        bit = 1;
                    } else {
                        const int ctx = iclip(cnt_cur - cnt_rem + 1, 0, 2);
                        uint16_t *const cdf =
                            n == 0 ? ts->cdf.m.comp0_ref[ctx][i] :
                            ts->cdf.m.comp1_ref[ctx][dir ^ f->refdir[i]][i];
                        bit = dav1d_msac_decode_bool_adapt(&ts->msac, cdf);
                    }
                    if (bit) {
                        b->ref[n++] = i;
                        if (n == 2) break;
                        dir = f->refdir[i];
                    }
                    if (maybe_same_ref) {
                        assert(i < same_refs);
                        maybe_same_ref = !bit && i + 1 < same_refs;
                        if (bit) {
                            i--;
                            cnt_rem += cnt_cur;
                        }
                    }
                }
                if (n < 2) {
                    b->ref[1] = n_refs - 1;
                    if (!n) b->ref[0] = n_refs - 1 - (same_refs < n_refs);
                }
            } else {
                b->ref[0] = b->ref[1] = 0;
            }
            DEBUG_BLOCK_printf("%*sPost-ref[%d,%d]: r=%d\n",
                               depth, "", b->ref[0], b->ref[1], ts->msac.rng);

            const int have_top_right = t->bx + bw4 <= ts->tiling.col_end;
            const int have_bottom_left = t->by + bh4 <= ts->tiling.row_end;
            const int comp_ctx =
                get_compref_ctx(t->a, &t->l, by4, bx4, have_top, have_left,
                                have_top_right, have_bottom_left, b_dim,
                                b->ref, f->frame_hdr->tip.refs);
            if (b->ref[0] == b->ref[1]) {
                b->inter_mode = NEARMV_NEARMV +
                    dav1d_msac_decode_symbol_adapt4(&ts->msac,
                        ts->cdf.m.comp_mode_sameref[comp_ctx], 3);
                b->inter_mode += b->inter_mode > NEARMV_NEWMV; // skip newmv_nearmv
            } else {
                const int joint_ctx =
                    f->refdist[b->ref[0]] != -f->refdist[b->ref[1]];
                if (dav1d_msac_decode_bool_adapt(&ts->msac,
                           ts->cdf.m.comp_mode_joint[joint_ctx]))
                {
                    b->inter_mode = JOINT_NEWMV;
                } else {
                    b->inter_mode = NEARMV_NEARMV +
                        dav1d_msac_decode_symbol_adapt8(&ts->msac,
                            ts->cdf.m.comp_mode[comp_ctx], 4);
                }
            }
            if (f->frame_hdr->opfl_refine_type == 1 /* switchable */ &&
                b->inter_mode != GLOBALMV_GLOBALMV &&
                imin(bw4, bh4) >= 2 && f->refdir[b->ref[0]] != f->refdir[b->ref[1]])
            {
                const int ctx = b->inter_mode > NEARMV_NEARMV;
                if (dav1d_msac_decode_bool_adapt(&ts->msac, ts->cdf.m.opfl[ctx]))
                    b->inter_mode += 6 - (b->inter_mode >= GLOBALMV_GLOBALMV);
            }
            DEBUG_BLOCK_printf("%*sPost-comp_inter_mode[ctx=%d,%d]: r=%d\n",
                               depth, "", comp_ctx, b->inter_mode, ts->msac.rng);

#define NEWMV_MASK ((1 << NEARMV_NEWMV) | \
                    (1 << NEWMV_NEARMV) | \
                    (1 << NEWMV_NEWMV) | \
                    (1 << JOINT_NEWMV) | \
                    (1 << OPFL_NEARMV_NEWMV) | \
                    (1 << OPFL_NEWMV_NEARMV) | \
                    (1 << OPFL_NEWMV_NEWMV) | \
                    (1 << OPFL_JOINT_NEWMV))
            const int is_newmv_mode = (1 << b->inter_mode) & NEWMV_MASK;
#undef NEWMV_MASK
            if (f->seq_hdr->adaptive_mvd && is_newmv_mode) {
                static uint8_t amvd_mode_context[] = {
                    [NEARMV_NEWMV      - NEARMV_NEWMV] = 0,
                    [NEWMV_NEARMV      - NEARMV_NEWMV] = 1,
                    [OPFL_NEARMV_NEWMV - NEARMV_NEWMV] = 2,
                    [OPFL_NEWMV_NEARMV - NEARMV_NEWMV] = 3,
                    [JOINT_NEWMV       - NEARMV_NEWMV] = 5,
                    [OPFL_JOINT_NEWMV  - NEARMV_NEWMV] = 6,
                    [NEWMV_NEWMV       - NEARMV_NEWMV] = 7,
                    [OPFL_NEWMV_NEWMV  - NEARMV_NEWMV] = 8,
                };
                const int mode_ctx = amvd_mode_context[b->inter_mode - NEARMV_NEWMV];
                const int ctx =
                    (nx[0]->amvd[xoff[0]] && nx[0]->ref[0][xoff[0]] == b->ref[0]) +
                    (nx[1]->amvd[xoff[1]] && nx[1]->ref[0][xoff[1]] == b->ref[0]);
                amvd = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                    ts->cdf.m.amvd[mode_ctx][ctx]);
                mvprec_def = 2 - (!amvd || f->frame_hdr->mv_precision < 3);
                DEBUG_BLOCK_printf("%*sPost-amvd[ctx=%d|%d,%d]: r=%d\n",
                                   depth, "", mode_ctx, ctx, amvd, ts->msac.rng);
            }

            refmvs_candidate mvstack[6];
            int n_mvs[2];
            if (b->inter_mode > NEARMV_NEWMV) {
                dav1d_refmvs_find(&t->rt, mvstack, NULL, &n_mvs[0],
                                  (union refmvs_refpair) { .ref = {
                                      b->ref[0] + 1, b->ref[1] + 1 } },
                                  bs, t->by, t->bx);
            } else if (b->ref[0] == b->ref[1]) {
                dav1d_refmvs_find(&t->rt, mvstack, NULL, &n_mvs[0],
                                  (union refmvs_refpair) { .ref = {
                                      b->ref[0] + 1, -1 } }, bs, t->by, t->bx);
                for (int n = 0; n < 6; n++) {
                    mvstack[n].mv.mv[1] = mvstack[n].mv.mv[0];
                    mvstack[n].weight *= 0x11;
                }
                n_mvs[1] = n_mvs[0];
            } else {
                dav1d_refmvs_find(&t->rt, mvstack, NULL, &n_mvs[0],
                                  (union refmvs_refpair) { .ref = {
                                      b->ref[0] + 1, -1 } }, bs, t->by, t->bx);
                refmvs_candidate mvstack2[6];
                dav1d_refmvs_find(&t->rt, mvstack2, NULL, &n_mvs[1],
                                  (union refmvs_refpair) { .ref = {
                                      b->ref[1] + 1, -1 } }, bs, t->by, t->bx);
                for (int n = 0; n < 6; n++) {
                    mvstack[n].mv.mv[1] = mvstack2[n].mv.mv[0];
                    mvstack[n].weight = (mvstack[n].weight & 0xf) |
                                         mvstack2[n].weight << 4;
                }
            }
#if DEBUG_BLOCK_INFO
            if (BLOCK_TO_DEBUG) {
                printf("%*sfind_mv_refs(%d,%d)\n", depth, "", b->ref[0], b->ref[1]);
                if (b->inter_mode <= NEARMV_NEWMV) {
                    for (int drl = 0; drl < 2; drl++)
                        for (int n = 0; n < n_mvs[drl]; n++)
                            printf("%*smv[%d:%d/%d]: y=%d,x=%d,w=%d\n",
                                   depth + 1, "", drl, n, n_mvs[drl],
                                   mvstack[n].mv.mv[drl].y,
                                   mvstack[n].mv.mv[drl].x,
                                   (mvstack[n].weight >> (4 * drl)) & 0xf);
                } else {
                    for (int n = 0; n < n_mvs[0]; n++)
                        printf("%*smv[%d/%d]: y=%d,x=%d,y2=%d,x2=%d,w=%d\n",
                               depth + 1, "", n, n_mvs[0], mvstack[n].mv.mv[0].y,
                               mvstack[n].mv.mv[0].x, mvstack[n].mv.mv[1].y,
                               mvstack[n].mv.mv[1].x, mvstack[n].weight);
                }
            }
#endif

            int jmvd_scale_mode = 0;
            if (b->inter_mode == JOINT_NEWMV || b->inter_mode == OPFL_JOINT_NEWMV) {
                jmvd_scale_mode = amvd ?
                    dav1d_msac_decode_symbol_adapt4(&ts->msac,
                        ts->cdf.m.jmvd_amvd_scale_mode, 2) :
                    dav1d_msac_decode_symbol_adapt8(&ts->msac,
                        ts->cdf.m.jmvd_scale_mode, 4);
                DEBUG_BLOCK_printf("%*sPost-jmvd_scale_mode[%d]: r=%d\n",
                                   depth, "", jmvd_scale_mode, ts->msac.rng);
            }

            if (b->inter_mode == NEWMV_NEWMV && imin(bw4, bh4) > 1 &&
                !f->frame_hdr->force_integer_mv && b->ref[0] != b->ref[1] &&
                f->frame_hdr->opfl_refine_type != 2 /* always */)
            {
                const int is_sb_boundary = !(t->by & (f->sb_step - 1));
                const int ref1 = b->ref[0], ref2 = b->ref[1];
#define match_ref(dir, off, refidx) \
                (t->dir ref[0][off] == refidx || t->dir ref[1][off] == refidx)
#define match_refs(refidx) \
                (match_ref(l., by4, refidx) || \
                 (t->by + bh4 <= ts->tiling.row_end && \
                  match_ref(l., by4 + bh4 - 1, refidx)) || \
                 (is_sb_boundary ? \
                  (match_ref(a_sb_cache., bx4 & ~1, refidx) || \
                   (((t->bx + bw4 - 2) & ~1) < ts->tiling.col_end && \
                    match_ref(a_sb_cache., (bx4 + bw4 - 2) & ~1, refidx))) : \
                  (match_ref(a->, bx4, refidx) || \
                   (t->bx + bw4 <= ts->tiling.col_end && \
                    match_ref(a->, bx4 + bw4 - 1, refidx)))))
                if (match_refs(ref1) && match_refs(ref2)) {
#undef match_refs
#undef match_ref
                    const enum MotionMode x1 = boff[0] == -1 ? MM_TRANSLATION :
                                               nb[0]->motion_mode[boff[0]],
                                          x2 = boff[1] == -1 ? MM_TRANSLATION :
                                               nb[1]->motion_mode[boff[1]];
                    const int cs_ctx =
                        (x1 >= MM_WARP_CAUSAL || x2 >= MM_WARP_CAUSAL) +
                        (x1 == MM_WARP_CAUSAL) + (x2 == MM_WARP_CAUSAL);
                    if (dav1d_msac_decode_bool_adapt(&ts->msac,
                                         ts->cdf.m.warp_causal[cs_ctx]))
                    {
                        b->motion_mode = MM_WARP_CAUSAL;
                    }
                    DEBUG_BLOCK_printf("%*sPost-comp_newmv_warp[%d]: r=%d\n",
                                       depth, "", b->motion_mode, ts->msac.rng);
                }
            }

            // drl
            int drl_idx[2] = { 0, 0 };
            if (b->inter_mode != GLOBALMV_GLOBALMV) {
                const int n_drls = 1 + (b->inter_mode <= NEARMV_NEWMV);
                const int max_drl_bits = f->frame_hdr->max_drl_bits;
                for (int r = 0, n = 0, ctx = 0; r < n_drls; r++) {
                    for (; n < max_drl_bits; n++, ctx += ctx < 2) {
                        if (!dav1d_msac_decode_bool_adapt(&ts->msac,
                                 ts->cdf.m.drl_idx[ctx][comp_ctx]))
                        {
                            break;
                        }
                    }
                    drl_idx[r] = n;
                    n = b->inter_mode == NEARMV_NEARMV && b->ref[0] == b->ref[1] ?
                        drl_idx[0] + (drl_idx[0] < max_drl_bits) : 0;
                    ctx = imin(n, 2);
                }
                if (n_drls == 1) drl_idx[1] = drl_idx[0];
                DEBUG_BLOCK_printf("%*sPost-drl[%d,%d]: r=%d\n",
                                   depth, "", drl_idx[0], drl_idx[1], ts->msac.rng);
            }

            // mv precision
            int mv_prec = 3 + f->frame_hdr->mv_precision;
            if (mv_prec > 3 && !amvd && f->seq_hdr->flex_mvres && is_newmv_mode) {
                const int mvprec1 = boff[0] == -1 ? 0 : nb[0]->mvprec[boff[0]];
                const int mvprec2 = boff[1] == -1 ? 0 : nb[1]->mvprec[boff[1]];
                const int ctx1 = (mvprec1 & 1) + (mvprec2 & 1);
                if (!dav1d_msac_decode_bool_adapt(&ts->msac,
                                                  ts->cdf.m.mvprec_def[ctx1]))
                {
                    const int ctx2 = (mvprec1 | mvprec2) >> 1;
                    const int idx =
                        dav1d_msac_decode_symbol_adapt4(&ts->msac,
                            ts->cdf.m.mvprec_rem[ctx2][mv_prec - 4], 2);
                    mv_prec = mv_prec_tbl[mv_prec == 6][idx];
                    mvprec_def = 2;
                }
                DEBUG_BLOCK_printf("%*sPost-mv_precision[ctx=%d|%d|%d,%d]: r=%d\n",
                                   depth, "", ctx1,
                                   mvprec_def == 1 ? -1 : (mvprec1 | mvprec2) >> 1,
                                   mvprec_def == 1 ? -1 :
                                       f->frame_hdr->mv_precision - 1,
                                   mv_prec, ts->msac.rng);
            }

            if (b->inter_mode != GLOBALMV_GLOBALMV) {
                int start = 0, end = 2;
                int refdist[2];
                if (b->inter_mode == JOINT_NEWMV ||
                    b->inter_mode == OPFL_JOINT_NEWMV)
                {
                    refdist[0] = f->absrefdist[b->ref[0]];
                    refdist[1] = f->absrefdist[b->ref[1]];
                    start = refdist[0] < refdist[1];
                    if (f->refdir[b->ref[0]] ^ f->refdir[b->ref[1]])
                        refdist[1] = -refdist[1];
                    end = start + 1;
                }
                mv diff[2];
                enum InterPredMode m[2];
                int n;
                int sum_mvd = 0, nnzc = 0;
                for (n = start; n < end; n++) {
                    b->mv[n] = mvstack[drl_idx[n]].mv.mv[n];
                    m[n] = dav1d_comp_inter_pred_modes[b->inter_mode -
                                                       NEARMV_NEARMV][n];
                    if (m[n] != NEWMV) continue;

                    if (amvd) {
                        read_amvd(ts, &diff[n]);
                        // nnzc remains zero if amvd=1, so mvd_sign_derive=>0
                    } else {
                        read_mv_residual(ts, &ts->cdf.mv, &diff[n], mv_prec);
                        sum_mvd += diff[n].y + diff[n].x;
                        nnzc += !!diff[n].y + !!diff[n].x;
                    }
                }
                if (b->inter_mode != NEARMV_NEARMV &&
                    b->inter_mode != OPFL_NEARMV_NEARMV)
                {
#define BIDIR_NEWMV_MASK ((1 << NEWMV_NEWMV) | \
                          (1 << OPFL_NEWMV_NEWMV) | \
                          (1 << JOINT_NEWMV) | \
                          (1 << OPFL_JOINT_NEWMV))
                    if (!f->seq_hdr->mvd_sign_derive || drl_idx[0] ||
                        drl_idx[1] || nnzc < 3 * (end - start) - 2 ||
                        f->frame_hdr->allow_screen_content_tools ||
                        f->frame_hdr->mv_precision == 3 || mv_prec >= 5 ||
                        !((1 << b->inter_mode) & BIDIR_NEWMV_MASK) ||
                        b->motion_mode != MM_TRANSLATION)
                    {
                        // this means nnzc2 never reaches nnzc below, so the
                        // sign-derive condition is never invoked
                        nnzc = 5;
                    }
#undef BIDIR_NEWMV_MASK
                    sum_mvd >>= (6 - mv_prec);
                    int nnzc2 = 0;
                    for (n = start; n < end; n++) {
                        if (m[n] != NEWMV) continue;
                        if (diff[n].y) {
                            const int s = ++nnzc2 == nnzc ? sum_mvd & 1 :
                                          dav1d_msac_decode_bool_bypass(&ts->msac);
                            if (s) diff[n].y = -diff[n].y;
                        }
                        if (diff[n].x) {
                            const int s = ++nnzc2 == nnzc ? sum_mvd & 1 :
                                          dav1d_msac_decode_bool_bypass(&ts->msac);
                            if (s) diff[n].x = -diff[n].x;
                        }
                        if (!amvd && mv_prec <= 3)
                            mv_reduce_prec(&b->mv[n], mv_prec);
                        b->mv[n].x += diff[n].x;
                        b->mv[n].y += diff[n].y;
                        DEBUG_BLOCK_printf("%*sPost-mvdiff[%d,y:%d,x:%d]: r=%d\n",
                                           depth, "", n, diff[n].y, diff[n].x,
                                           ts->msac.rng);
                    }
                    if (b->inter_mode == JOINT_NEWMV ||
                        b->inter_mode == OPFL_JOINT_NEWMV)
                    {
                        n &= 1; // "the one not handled above"
                        diff[n] = mv_projection(diff[!n], refdist[1], refdist[0]);
                        jmvd_scale(&diff[n], amvd, jmvd_scale_mode);
                        b->mv[n] = mvstack[drl_idx[n]].mv.mv[n];
                        if (!amvd && mv_prec <= 3)
                            mv_reduce_prec(&b->mv[n], mv_prec);
                        b->mv[n].x += diff[n].x;
                        b->mv[n].y += diff[n].y;
                    }
                }
            } else {
                b->mv[0] = get_gmv_2d(&f->frame_hdr->gmv[b->ref[0]],
                                      t->bx, t->by, bw4, bh4, f->frame_hdr);
                b->mv[1] = get_gmv_2d(&f->frame_hdr->gmv[b->ref[1]],
                                      t->bx, t->by, bw4, bh4, f->frame_hdr);
            }

            if (f->seq_hdr->refine_mv && imin(bw4, bh4) >= 2 && bw4 * bh4 > 4 &&
                b->inter_mode != GLOBALMV_GLOBALMV &&
                f->refdist[b->ref[0]] == -f->refdist[b->ref[1]] &&
                !f->svc[b->ref[0]][0].scale && !f->svc[b->ref[1]][0].scale &&
                (f->frame_hdr->opfl_refine_type != 1 /* switchable */ ||
                 !((1 << b->inter_mode) & ((1 << NEARMV_NEWMV) |
                                           (1 << NEWMV_NEARMV) |
                                           (1 << NEWMV_NEWMV) |
                                           (1 << JOINT_NEWMV)))))
            {
                if ((1 << b->inter_mode) & ((1 << NEARMV_NEARMV) |
                                            (1 << OPFL_NEARMV_NEARMV) |
                                            (1 << OPFL_JOINT_NEWMV)))
                {
                    b->refine_mv = 2; // implicitly enabled
                } else {
                    const int ctx = b->inter_mode - NEARMV_NEARMV;
                    b->refine_mv = dav1d_msac_decode_bool_adapt(&ts->msac,
                                       ts->cdf.m.refine_mv[ctx]);
                    DEBUG_BLOCK_printf("%*sPost-refinemv[ctx=%d,%d]: r=%d\n",
                                       depth, "", ctx + 1, b->refine_mv,
                                       ts->msac.rng);
                }
            }

            has_subpel_filter = b->inter_mode <= JOINT_NEWMV /* no opfl */ &&
                !b->refine_mv && b->motion_mode == MM_TRANSLATION &&
                (b->inter_mode != GLOBALMV_GLOBALMV || imin(bw4, bh4) == 1 ||
                 f->frame_hdr->gmv[b->ref[0]].type == DAV1D_WM_TYPE_TRANSLATION ||
                 f->frame_hdr->gmv[b->ref[1]].type == DAV1D_WM_TYPE_TRANSLATION);

            b->comp_type = COMP_INTER_AVG;
            if (b->inter_mode <= JOINT_NEWMV /* no opfl */ &&
                b->refine_mv != 1 /* disabled, or implicitly enabled */ &&
                !(b->inter_mode == JOINT_NEWMV && amvd) &&
                f->seq_hdr->masked_compound && imin(bw4, bh4) >= 2)
            {
                const int ffr = f->furthest_future_refidx;
#define comptype_ctx(num) \
                num >= idx ? 0 : nx[num]->ref[1][xoff[num]] != -1 ? \
                nx[num]->comp_type[xoff[num]] > COMP_INTER_AVG : \
                (nx[num]->ref[0][xoff[num]] == ffr) * 2
                const int cctx0 = comptype_ctx(0), cctx1 = comptype_ctx(1);
#undef comptype_ctx
                const int ctx = cctx0 + cctx1 + (cctx0 && cctx1) +
                    (f->absrefdist[b->ref[0]] == f->absrefdist[b->ref[1]]) * 6;
                const int has_mask = dav1d_msac_decode_bool_adapt(&ts->msac,
                                         ts->cdf.m.comp_type_masked[ctx]);
                if (has_mask) {
                    if (imax(bw4, bh4) <= 16 &&
                        !dav1d_msac_decode_bool_adapt(&ts->msac,
                             ts->cdf.m.comp_type_weighted))
                    {
                        b->comp_type = COMP_INTER_WEDGE;
                        b->wedge_idx = read_wedge_idx(ts);
                        b->wedge_sign = dav1d_msac_decode_bool_bypass(&ts->msac);
                    } else {
                        b->comp_type = COMP_INTER_SEG;
                        b->mask_sign = dav1d_msac_decode_bool_bypass(&ts->msac);
                    }
                }
                DEBUG_BLOCK_printf("%*sPost-comp_inter_type[ctx=%d,%d,%c=%d|%d]: r=%d\n",
                                   depth, "", ctx, b->comp_type - 1,
                                   "?wm"[b->comp_type - 1],
                                   b->comp_type == COMP_INTER_AVG ? -1 :
                                   b->comp_type == COMP_INTER_WEDGE ?
                                       b->wedge_idx : b->mask_sign,
                                   b->comp_type == COMP_INTER_WEDGE ?
                                       b->wedge_sign : -1, ts->msac.rng);
            }

            b->cwp_idx = 8;
            if (!b->refine_mv && !jmvd_scale_mode &&
                f->seq_hdr->cwp && b->comp_type == COMP_INTER_AVG &&
                (b->inter_mode == NEARMV_NEARMV || b->inter_mode == JOINT_NEWMV))
            {
                int n;
                for (n = 0; n < 4; n++) {
                    if (!dav1d_msac_decode_bool_adapt(&ts->msac,
                                                      ts->cdf.m.cwp_idx[n]))
                    {
                        break;
                    }
                }
                static const int8_t cwp_weighting_factor[2][5] = {
                    { 8, 12, 4, 10, 6 },
                    { 8, 12, 4, 20, -4 },
                };
                b->cwp_idx = cwp_weighting_factor[!(f->refdir[b->ref[0]] ^
                                                    f->refdir[b->ref[1]])][n];
                DEBUG_BLOCK_printf("%*sPost-compweightpred_idx[%d]: r=%d\n",
                                   depth, "", b->cwp_idx, ts->msac.rng);
            }
        } else {
            b->comp_type = COMP_INTER_NONE;

            // ref
            if (seg && seg->ref > 0) {
                b->ref[0] = seg->ref - 1;
            } else if (seg && (seg->globalmv || seg->skip)) {
                b->ref[0] = 0;
            } else {
                if (is_tip) {
                    static const int8_t tip_wts[] = { 8,  12, 16, 18, 20, 4, 6, -4 };
                    b->ref[0] = TIP_FRAME;
                    b->cwp_idx = tip_wts[f->frame_hdr->tip.global_wtd_idx];
                } else {
                    const int n_refs = f->frame_hdr->n_ref_frames;
                    int i = 0;
                    if (n_refs > 1) {
                        uint8_t cnt[9] = { 0 };
                        if (idx > 0) {
                            cnt[nx[0]->ref[0][xoff[0]] + 1]++;
                            cnt[nx[0]->ref[1][xoff[0]] + 1]++;
                            if (idx > 1) {
                                cnt[nx[1]->ref[0][xoff[1]] + 1]++;
                                cnt[nx[1]->ref[1][xoff[1]] + 1]++;
                            }
                        }
                        int cnt_rem = idx * 2 - cnt[0] - cnt[8];
                        do {
                            const int cnt_cur = cnt[i + 1];
                            cnt_rem -= cnt_cur;
                            const int ctx = iclip(cnt_cur - cnt_rem + 1, 0, 2);
                            if (dav1d_msac_decode_bool_adapt(&ts->msac,
                                    ts->cdf.m.single_ref[ctx][i]))
                            {
                                break;
                            }
                        } while (++i < n_refs - 1);
                    }
                    b->ref[0] = i;
                }
                DEBUG_BLOCK_printf("%*sPost-ref[%d,-1]: r=%d\n",
                                   depth, "", b->ref[0], ts->msac.rng);
            }
            b->ref[1] = -1;

            const int have_top_right = t->bx + bw4 <= ts->tiling.col_end;
            const int have_bottom_left = t->by + bh4 <= ts->tiling.row_end;
            const int sngl_ctx =
                get_snglref_ctx(t->a, &t->l, by4, bx4, have_top, have_left,
                                have_top_right, have_bottom_left, b_dim, b->ref[0]);
            const int is_sb_boundary = !(t->by & (f->sb_step - 1));

            if (seg && (seg->globalmv || seg->skip)) {
                b->inter_mode = GLOBALMV;
            } else if (is_tip) {
                b->inter_mode = NEARMV +
                    2 * dav1d_msac_decode_bool_adapt(&ts->msac, ts->cdf.m.tip_mode);
                DEBUG_BLOCK_printf("%*sPost-tip_mode[%d]: r=%d\n",
                                   depth, "", b->inter_mode, ts->msac.rng);
            } else {
                int allow_warp = 0;
                if (imin(bw4, bh4) >= 2 && f->frame_hdr->warp_motion) {
                    const int ctx =
                        get_warp_ctx(t->a, &t->a_sb_cache,
                                     &t->l, by4, bx4, have_top, have_left,
                                     is_sb_boundary ? ((t->bx + bw4 - 2) & ~1) <
                                        ts->tiling.col_end : have_top_right,
                                        have_bottom_left,
                                     is_sb_boundary, b_dim, b->ref[0]);
                    allow_warp = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                              ts->cdf.m.warp[ctx]);
                }
                if (allow_warp) {
                    b->inter_mode = !f->frame_hdr->force_integer_mv &&
                                    !dav1d_msac_decode_bool_adapt(&ts->msac,
                                         ts->cdf.m.warp_newmv) ? WARPNEWMV : WARPMV;
                } else {
                    b->inter_mode = NEARMV +
                        dav1d_msac_decode_symbol_adapt4(&ts->msac,
                            ts->cdf.m.inter_mode[sngl_ctx], 2);
                }
                DEBUG_BLOCK_printf("%*sPost-single_inter_mode[ctx=%d,%d]: r=%d\n",
                                   depth, "", sngl_ctx,
                                   b->inter_mode, ts->msac.rng);
            }

            if (f->seq_hdr->adaptive_mvd && b->inter_mode == NEWMV) {
                const int ctx =
                    (nx[0]->amvd[xoff[0]] && nx[0]->ref[0][xoff[0]] == b->ref[0]) +
                    (nx[1]->amvd[xoff[1]] && nx[1]->ref[0][xoff[1]] == b->ref[0]);
                amvd = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                    ts->cdf.m.amvd[4][ctx]);
                mvprec_def = 2 - (!amvd || f->frame_hdr->mv_precision < 3);
                DEBUG_BLOCK_printf("%*sPost-amvd[ctx=4|%d,%d]: r=%d\n",
                                   depth, "", ctx, amvd, ts->msac.rng);
            }

            refmvs_candidate mvstack[6];
            int32_t warp[4][7];
            int n_mvs[2];
            dav1d_refmvs_find(&t->rt, mvstack,
                              b->ref[0] != TIP_FRAME && b->inter_mode > NEWMV ?
                                  warp : NULL, n_mvs,
                              (union refmvs_refpair) { .ref = { b->ref[0] + 1, -1 }},
                              bs, t->by, t->bx);
#if DEBUG_BLOCK_INFO
            if (BLOCK_TO_DEBUG) {
                printf("%*sfind_mv_refs(%d,-1)\n", depth, "", b->ref[0]);
                for (int n = 0; n < n_mvs[0]; n++)
                    printf("%*smv[%d/%d]: y=%d,x=%d,w=%d,y_off=%d,x_off=%d\n",
                           depth + 1, "", n, n_mvs[0], mvstack[n].mv.mv[0].y,
                           mvstack[n].mv.mv[0].x, mvstack[n].weight,
                           mvstack[n].y_off, mvstack[n].x_off);
                if (b->ref[0] != TIP_FRAME && b->inter_mode > NEWMV)
                    for (int n = 0; n < n_mvs[1]; n++)
                        printf("%*swarp[%d/%d]: %d, %d, %d, %d, %d, %d, t=%d\n",
                               depth + 1, "", n, n_mvs[1],
                               warp[n][0], warp[n][1], warp[n][2],
                               warp[n][3], warp[n][4], warp[n][5], warp[n][6]);
            }
#endif

            int warp_ref_idx = 0, warpmv_with_mvd = 0;
            b->bawp[0] = b->bawp[1] = 0;
            if (is_tip) {
                /* do nothing */
            } else if (b->inter_mode <= NEWMV) {
                // block-adaptive weighted prediction
                if (f->frame_hdr->bawp &&
                    b->inter_mode != GLOBALMV &&
                    imin(bw4, bh4) >= 2 && !f->svc[b->ref[0]][0].scale)
                {
                    b->bawp[0] = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                              ts->cdf.m.bawp[0]);
                    if (b->bawp[0]) {
                        const int ctx = b->inter_mode == NEWMV ? 2 - amvd : 0;
                        b->bawp[0] += dav1d_msac_decode_bool_adapt(&ts->msac,
                                          ts->cdf.m.bawp_explicit[ctx]);
                        if (b->bawp[0] == 2) {
                            b->bawp[0] += dav1d_msac_decode_bool_adapt(&ts->msac,
                                              ts->cdf.m.bawp_explicit_scale);
                            b->bawp[0] |= ctx << 2;
                        }
                        if (has_chroma)
                            b->bawp[1] = dav1d_msac_decode_bool_adapt(&ts->msac,
                                             ts->cdf.m.bawp[1]);
                    }
                    DEBUG_BLOCK_printf("%*sPost-bawp[%d,%d]: r=%d\n",
                                       depth, "", b->bawp[0] & 3,
                                       b->bawp[1], ts->msac.rng);
                }

                // inter-intra (motion-mode)
                if (f->frame_hdr->motion_modes & (1 << MM_INTERINTRA) &&
                    !b->bawp[0] && bw4 * bh4 > 2 && imax(bw4, bh4) <= 16 &&
                    b->inter_mode >= NEARMV && b->inter_mode <= NEWMV)
                {
                    const int ctx = size_group_lookup[bs];
                    if (dav1d_msac_decode_bool_adapt(&ts->msac,
                                                     ts->cdf.m.interintra[ctx]))
                    {
                        b->motion_mode = MM_INTERINTRA;
                        b->interintra_mode = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                ts->cdf.m.interintra_mode[ctx], 3);
                        b->wedge_idx = -1;
                        if (imin(bw4, bh4) > 1 &&
                            dav1d_msac_decode_bool_adapt(&ts->msac,
                                ts->cdf.m.interintra_wedge))
                        {
                            b->wedge_idx = read_wedge_idx(ts);
                        }
                    }
                    DEBUG_BLOCK_printf("%*sPost-interintra[%d,%d,%d]: r=%d\n",
                                       depth, "", b->motion_mode,
                                       b->motion_mode ? b->interintra_mode : -1,
                                       b->motion_mode ? b->wedge_idx : -1,
                                       ts->msac.rng);
                }
            } else {
                // motion mode
                b->motion_mode = MM_WARP_DELTA;
                if (b->inter_mode == WARPNEWMV) {
                    const int ref = b->ref[0];
#define match_ref(dir, off) \
                    (t->dir ref[0][off] == ref || t->dir ref[1][off] == ref)
                    const int has_cs_ext = match_ref(l., by4) ||
                        (t->by + bh4 <= ts->tiling.row_end &&
                         match_ref(l., by4 + bh4 - 1)) ||
                        (is_sb_boundary ?
                         (match_ref(a_sb_cache., bx4 & ~1) ||
                          (((t->bx + bw4 - 2) & ~1) < ts->tiling.col_end &&
                           match_ref(a_sb_cache., (bx4 + bw4 - 2) & ~1))) :
                         (match_ref(a->, bx4) ||
                          (t->bx + bw4 <= ts->tiling.col_end &&
                           match_ref(a->, bx4 + bw4 - 1))));
#undef match_ref
                    b->motion_mode = MM_WARP_DELTA;
                    if (has_cs_ext) {
                        const enum MotionMode x1 = boff[0] == -1 ? MM_TRANSLATION :
                                                   nb[0]->motion_mode[boff[0]],
                                              x2 = boff[1] == -1 ? MM_TRANSLATION :
                                                   nb[1]->motion_mode[boff[1]];
                        const int ext_ctx = (x1 >= MM_WARP_CAUSAL) +
                                            (x2 >= MM_WARP_CAUSAL);
                        if (dav1d_msac_decode_bool_adapt(&ts->msac,
                               ts->cdf.m.warp_extend[ext_ctx]))
                        {
                            b->motion_mode = MM_WARP_EXTEND;
                        } else {
                            const int cs_ctx = (ext_ctx > 0) +
                                (x1 == MM_WARP_CAUSAL) + (x2 == MM_WARP_CAUSAL);
                            b->motion_mode = dav1d_msac_decode_bool_adapt(&ts->msac,
                                                 ts->cdf.m.warp_causal[cs_ctx]) ?
                                MM_WARP_CAUSAL : MM_WARP_DELTA;
                        }
                        DEBUG_BLOCK_printf("%*sPost-sngl_newmv_warp[%d]: r=%d\n",
                                           depth, "", b->motion_mode, ts->msac.rng);
                    }
                }

                if (b->motion_mode == MM_WARP_DELTA) {
                    for (; warp_ref_idx < 3; warp_ref_idx++) {
                        if (!dav1d_msac_decode_bool_adapt(&ts->msac,
                                 ts->cdf.m.warp_ref_idx[warp_ref_idx]))
                        {
                            break;
                        }
                    }
                    DEBUG_BLOCK_printf("%*sPost-warp_ref_idx[%d/%d]: r=%d\n",
                                       depth, "", warp_ref_idx, 4, ts->msac.rng);
                }

                if (b->inter_mode == WARPMV && warp_ref_idx < 2) {
                    warpmv_with_mvd = dav1d_msac_decode_bool_adapt(&ts->msac,
                                          ts->cdf.m.warpmv_with_mvd);
                    DEBUG_BLOCK_printf("%*sPost-warpmv_with_mvd[%d]: r=%d\n",
                                       depth, "", warpmv_with_mvd, ts->msac.rng);
                }
            }

            // drl
            int drl_idx = 0;
            if (b->inter_mode != WARPMV && b->inter_mode != GLOBALMV) {
                const int max_drl_bits = f->frame_hdr->max_drl_bits;
                int n = 0;
                for (int ctx = 0; n < max_drl_bits; n++, ctx += ctx < 2) {
                    if (!dav1d_msac_decode_bool_adapt(&ts->msac,
                             is_tip ? ts->cdf.m.tip_drl_idx[ctx] :
                                      ts->cdf.m.drl_idx[ctx][sngl_ctx]))
                    {
                        break;
                    }
                }
                drl_idx = n;
                DEBUG_BLOCK_printf("%*sPost-drl[%d,-1]: r=%d\n",
                                   depth, "", drl_idx, ts->msac.rng);
            }

            // mv precision
            int mv_prec = 3 + f->frame_hdr->mv_precision;
            if (mv_prec > 3 && !amvd && f->seq_hdr->flex_mvres &&
                (b->inter_mode == NEWMV || b->inter_mode == WARPNEWMV))
            {
                const int mvprec1 = boff[0] == -1 ? 0 : nb[0]->mvprec[boff[0]];
                const int mvprec2 = boff[1] == -1 ? 0 : nb[1]->mvprec[boff[1]];
                const int ctx1 = (mvprec1 & 1) + (mvprec2 & 1);
                if (!dav1d_msac_decode_bool_adapt(&ts->msac,
                                                  ts->cdf.m.mvprec_def[ctx1]))
                {
                    const int ctx2 = (mvprec1 | mvprec2) >> 1;
                    const int idx =
                        dav1d_msac_decode_symbol_adapt4(&ts->msac,
                            ts->cdf.m.mvprec_rem[ctx2][mv_prec - 4], 2);
                    mv_prec = mv_prec_tbl[mv_prec == 6][idx];
                    mvprec_def = 2;
                }
                DEBUG_BLOCK_printf("%*sPost-mv_precision[ctx=%d|%d|%d,%d]: r=%d\n",
                                   depth, "", ctx1,
                                   mvprec_def == 1 ? -1 : (mvprec1 | mvprec2) >> 1,
                                   mvprec_def == 1 ? -1 :
                                       f->frame_hdr->mv_precision - 1,
                                   mv_prec, ts->msac.rng);
            }

            mv diff;
            if (b->inter_mode != GLOBALMV) {
                b->mv[0] = mvstack[drl_idx].mv.mv[0];
                if (b->inter_mode == NEWMV || b->inter_mode == WARPNEWMV ||
                    (b->inter_mode == WARPMV && warpmv_with_mvd))
                {
                    int nnzc, nnzc2 = 0, sum_mvd;
                    if (amvd) {
                        read_amvd(ts, &diff);
                        nnzc = 3; // see comment a few lines down
                    } else {
                        read_mv_residual(ts, &ts->cdf.mv, &diff, mv_prec);
                        nnzc = !!diff.x + !!diff.y;
                        sum_mvd = (diff.x + diff.y) >> (6 - mv_prec);
                        if (b->inter_mode == WARPMV || !nnzc ||
                            !f->seq_hdr->mvd_sign_derive ||
                            b->motion_mode != MM_TRANSLATION ||
                            f->frame_hdr->allow_screen_content_tools ||
                            f->frame_hdr->mv_precision == 3 || mv_prec >= 5)
                        {
                            // this means nnzc2 never reaches nnzc below, so the
                            // sign-derive condition is never invoked
                            nnzc = 3;
                        }
                    }
                    if (diff.y) {
                        const int s = ++nnzc2 == nnzc ? sum_mvd & 1 :
                                      dav1d_msac_decode_bool_bypass(&ts->msac);
                        if (s) diff.y = -diff.y;
                    }
                    if (diff.x) {
                        const int s = ++nnzc2 == nnzc ? sum_mvd & 1 :
                                      dav1d_msac_decode_bool_bypass(&ts->msac);
                        if (s) diff.x = -diff.x;
                    }
                    if (!amvd && mv_prec <= 3)
                        mv_reduce_prec(&b->mv[0], mv_prec);
                    b->mv[0].x += diff.x;
                    b->mv[0].y += diff.y;
                    DEBUG_BLOCK_printf("%*sPost-mvdiff[%d,y:%d,x:%d]: r=%d\n",
                                       depth, "", 0, diff.y, diff.x,
                                       ts->msac.rng);
                }
            } else {
                b->mv[0] = get_gmv_2d(&f->frame_hdr->gmv[b->ref[0]],
                                      t->bx, t->by, bw4, bh4, f->frame_hdr);
            }

            if (b->inter_mode == WARPNEWMV && b->motion_mode == MM_WARP_DELTA &&
                ((f->seq_hdr->six_param_warp_delta && warp_ref_idx == 1) ||
                 warp_ref_idx == 0))
            {
                const int prec = dav1d_msac_decode_bool_adapt(&ts->msac,
                                     ts->cdf.m.warp_delta_prec[bs]);
                const int np = f->seq_hdr->six_param_warp_delta &&
                               warp_ref_idx == 1 ? 4 : 2;
                for (int n = 0; n < np; n++) {
                    const int ctx = n - 1U > 1U;
                    b->matrix[n] =
                        dav1d_msac_decode_symbol_adapt8(&ts->msac,
                            ts->cdf.m.warp_delta_param[0][!ctx], 7);
                    if (b->matrix[n] == 7 && prec)
                        b->matrix[n] +=
                            dav1d_msac_decode_symbol_adapt8(&ts->msac,
                                ts->cdf.m.warp_delta_param[1][!ctx], 7);
                    if (b->matrix[n]) {
                        const int step = 1 << (11 - prec);
                        const int sign = dav1d_msac_decode_bool_adapt(&ts->msac,
                                              ts->cdf.m.warp_delta_sign);
                        if (sign) b->matrix[n] = -b->matrix[n];
                        t->warpmv.matrix[2 + n] = warp[warp_ref_idx][n + 2] +
                                                  b->matrix[n] * step;
                    } else {
                        t->warpmv.matrix[2 + n] = warp[warp_ref_idx][n + 2];
                    }
                }
                if (np == 2) {
                    t->warpmv.matrix[5] = t->warpmv.matrix[2];
                    t->warpmv.matrix[4] = -t->warpmv.matrix[3];
                }
                const int xpos = 4 * t->bx + 2 * bw4 - 1;
                const int ypos = 4 * t->by + 2 * bh4 - 1;
                t->warpmv.matrix[0] = b->mv[0].x * (1 << 13) -
                    xpos * (t->warpmv.matrix[2] - 0x10000) -
                    ypos * t->warpmv.matrix[3];
                t->warpmv.matrix[1] = b->mv[0].y * (1 << 13) -
                    xpos * t->warpmv.matrix[4] -
                    ypos * (t->warpmv.matrix[5] - 0x10000);
                t->warpmv.type = dav1d_get_shear_params(&t->warpmv) ?
                    DAV1D_WM_TYPE_INVALID : np == 4 ?
                    DAV1D_WM_TYPE_AFFINE : DAV1D_WM_TYPE_ROT_ZOOM;
                DEBUG_BLOCK_printf("%*sPost-warp_param_signal[%d,%d,%d,%d]: r=%d\n",
                                   depth, "", b->matrix[0], b->matrix[1],
                                   (np == 4) ? b->matrix[2] : 0,
                                   (np == 4) ? b->matrix[3] : 0, ts->msac.rng);
            } else if (b->motion_mode == MM_WARP_DELTA) {
                memcpy(t->warpmv.matrix, warp[warp_ref_idx],
                       sizeof(int32_t) * 6);
                if (b->inter_mode == WARPMV) {
                    if (warpmv_with_mvd) {
                        t->warpmv.matrix[0] += diff.x * (1 << 13);
                        t->warpmv.matrix[1] += diff.y * (1 << 13);
                    }
                    b->mv[0] = get_warpmv_2d(t->warpmv.matrix, t->bx, t->by,
                                             bw4, bh4, warpmv_with_mvd ? mv_prec : 6);
                }
                // yes this re-calculates the warpmatrix from mv after (for
                // warpmv) we've just done the opposite. The round-trip error
                // from this operation is required for conformance.
                dav1d_set_affine_mv2d(bw4, bh4, b->mv[0], &t->warpmv, t->bx, t->by);
                t->warpmv.type = dav1d_get_shear_params(&t->warpmv) ?
                    DAV1D_WM_TYPE_INVALID : warp[warp_ref_idx][6];
            } else if (b->motion_mode == MM_WARP_CAUSAL) {
                derive_warpmv(t, have_top, have_left,
                              bw4, bh4, w4, h4, b->ref[0], b->mv[0], &t->warpmv);
            } else if (b->motion_mode == MM_WARP_EXTEND) {
                int y_off, x_off;
                if (mvstack[drl_idx].x_off == -1 ||
                    mvstack[drl_idx].y_off == -1)
                {
                    y_off = mvstack[drl_idx].y_off;
                    x_off = mvstack[drl_idx].x_off;
                } else if (have_bottom_left &&
                    (t->l.ref[0][by4 + bh4 - 1] == b->ref[0] ||
                     t->l.ref[1][by4 + bh4 - 1] == b->ref[0]))
                {
                    y_off = bh4 - 1;
                    x_off = -1;
                } else if (have_top_right &&
                           (t->a->ref[0][bx4 + bw4 - 1] == b->ref[0] ||
                            t->a->ref[1][bx4 + bw4 - 1] == b->ref[0]))
                {
                    y_off = -1;
                    x_off = bw4 - 1;
                } else if (have_left &&
                           (t->l.ref[0][by4] == b->ref[0] ||
                            t->l.ref[1][by4] == b->ref[0]))
                {
                    y_off = 0;
                    x_off = -1;
                } else if (have_top &&
                           (t->a->ref[0][bx4 + bw4 - 1] == b->ref[0] ||
                            t->a->ref[1][bx4 + bw4 - 1] == b->ref[0]))
                {
                    y_off = -1;
                    x_off = 0;
                } else {
                    y_off = x_off = 0; // invalid
                }
                if (y_off || x_off)
                    extend_warpmv(t, x_off, y_off, b_dim, b, &t->warpmv);
                else
                    t->warpmv.type = DAV1D_WM_TYPE_INVALID;
            }

            b->warp_ii = 0;
            if (b->inter_mode == WARPMV &&
                imin(bw4, bh4) >= 2 && imax(bw4, bh4) <= 16)
            {
                const int ctx = size_group_lookup[bs];
                if (dav1d_msac_decode_bool_adapt(&ts->msac,
                                                 ts->cdf.m.warp_interintra[ctx]))
                {
                    b->warp_ii = 1;
                    b->interintra_mode = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                            ts->cdf.m.interintra_mode[ctx], 3);
                    b->wedge_idx = dav1d_msac_decode_bool_adapt(&ts->msac,
                        ts->cdf.m.interintra_wedge) ? read_wedge_idx(ts) : -1;
                }
                DEBUG_BLOCK_printf("%*sPost-warp_ii[%d,%d,%d]: r=%d\n",
                                   depth, "", b->warp_ii,
                                   b->warp_ii ? b->interintra_mode : -1,
                                   b->warp_ii ? b->wedge_idx : -1, ts->msac.rng);
            }

            has_subpel_filter = !is_tip && b->inter_mode <= NEWMV &&
                (b->inter_mode != GLOBALMV || imin(bw4, bh4) == 1 ||
                 f->frame_hdr->gmv[b->ref[0]].type == DAV1D_WM_TYPE_TRANSLATION);
        }

        // subpel filter
        if (b->skip_mode || b->ref[0] == TIP_FRAME || b->refine_mv ||
            b->inter_mode >= OPFL_NEARMV_NEARMV)
        {
            assert(!has_subpel_filter);
            b->filter = DAV1D_FILTER_8TAP_SHARP;
        } else if (f->frame_hdr->subpel_filter_mode == DAV1D_FILTER_SWITCHABLE) {
            if (has_subpel_filter) {
                const int ctx = get_filter_ctx(nb, boff, b->ref);
                b->filter = dav1d_msac_decode_symbol_adapt4(&ts->msac,
                                ts->cdf.m.filter[ctx],
                                DAV1D_N_SWITCHABLE_FILTERS - 1);
                DEBUG_BLOCK_printf("%*sPost-subpelfilter[ctx=%d,%d]: r=%d\n",
                                   depth, "", ctx, b->filter, ts->msac.rng);
            } else b->filter = DAV1D_FILTER_8TAP_REGULAR;
        } else {
            b->filter = f->frame_hdr->subpel_filter_mode;
        }

        if (is_comp) {
            DEBUG_BLOCK_printf("%*sfinal 2dmv: y=%d,x=%d | y=%d,x=%d\n",
                               depth, "", b->mv[0].y, b->mv[0].x,
                               b->mv[1].y, b->mv[1].x);
        } else if (b->motion_mode > MM_INTERINTRA) {
            debug_warp_matrix(depth, f, t, b);
        } else {
            DEBUG_BLOCK_printf("%*sfinal 2dmv: y=%d,x=%d\n",
                               depth, "", b->mv[0].y, b->mv[0].x);
        }
        if (t->f->seq_hdr->refmv_bank)
            dav1d_refmvs_bank_add(&t->rt, bs, t->by, t->bx, b);
        if (b->motion_mode > MM_INTERINTRA)
            dav1d_refmvs_warp_add(&t->rt, &t->warpmv,
                                  DB_ONLY(t->by, t->bx) b->ref[0]);
        if (is_comp)
            splat_tworef_mv(DB_ONLY(depth) f, t, bs, b, by4, bw4, bh4);
        else
            splat_oneref_mv(DB_ONLY(depth) f, t, bs, b, by4, bw4, bh4);

        read_tx_part(t, DB_ONLY(depth) b, bs);

        // reconstruction
        if (t->frame_thread.pass == 1) {
            f->bd_fn.read_coef_blocks(t, bs, b);
        } else {
            f->bd_fn.recon_b(t, DB_ONLY(depth) lbs, cbs, b);
        }

#if 0
        if (f->frame_hdr->loopfilter.level_y[0] ||
            f->frame_hdr->loopfilter.level_y[1])
        {
            const uint16_t tx_split[2] = { 0, 0 };
            enum RectTxfmSize ytx = dav1d_max_txfm_size_for_bs[bs][0], uvtx = b->uvtx;
            if (f->frame_hdr->segmentation.lossless[b->seg_id]) {
                ytx  = (enum RectTxfmSize) TX_4X4;
                uvtx = (enum RectTxfmSize) TX_4X4;
            }
            dav1d_create_lf_mask_inter(t->lf_mask, t->bx, t->by, f->bw, f->bh, b->skip_txfm,
                                       bs, ytx, tx_split, uvtx, f->cur.p.layout,
                                       &t->a->tx_lpf_y[bx4], &t->l.tx_lpf_y[by4],
                                       has_chroma ? &t->a->tx_lpf_uv[cbx4] : NULL,
                                       has_chroma ? &t->l.tx_lpf_uv[cby4] : NULL);
        }
#endif

        // context updates
        BlockContext *edge = t->a;
        for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
#define set_ctx(rep_macro) \
            rep_macro(edge->seg_pred, off, seg_pred); \
            rep_macro(edge->skip_mode, off, b->skip_mode); \
            rep_macro(edge->intra, off, 0); \
            rep_macro(edge->intrabc, off, 0); \
            rep_macro(edge->morph_pred, off, 0); \
            rep_macro(edge->midx, off, 0xff); \
            rep_macro(edge->fsc, off, 0); \
            rep_macro(edge->skip_txfm, off, b->skip_txfm); \
            rep_macro(edge->pal_sz, off, 0); \
            rep_macro(edge->comp_type, off, b->comp_type); \
            rep_macro(edge->filter, off, b->filter); \
            rep_macro(edge->mode, off, b->inter_mode); \
            rep_macro(edge->mrl, off, 0); \
            rep_macro(edge->multi_mrl, off, 0); \
            rep_macro(edge->dip, off, 0); \
            rep_macro(edge->ref[0], off, b->ref[0]); \
            rep_macro(edge->ref[1], off, ((uint8_t) b->ref[1])); \
            rep_macro(edge->motion_mode, off, b->motion_mode); \
            rep_macro(edge->amvd, off, amvd); \
            rep_macro(edge->mvprec, off, mvprec_def)
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

    if (has_luma &&
        (f->frame_hdr->loopfilter.level_y[0] || f->frame_hdr->loopfilter.level_y[1]))
    {
        dav1d_create_lf_mask_intra(t->lf_mask, t->bx, t->by, f->bw, f->bh, bs,
                                   b->tx_part, b->uvtx, f->cur.p.layout,
                                   &t->a->tx_lpf_y[bx4], &t->l.tx_lpf_y[by4],
                                   has_chroma ? &t->a->tx_lpf_uv[cbx4] : NULL,
                                   has_chroma ? &t->l.tx_lpf_uv[cby4] : NULL);
    }

    if (!b->skip_txfm) {
        uint16_t (*noskip_mask)[4] = &t->lf_mask->noskip_mask[by4 >> 1];
        const unsigned mask = (~0U >> imax(0, 32 - bw4)) << (bx4 & 15);
        const int bx_idx = (bx4 & 16) >> 4;
        for (int y = 0; y < bh4; y += 2, noskip_mask++) {
            (*noskip_mask)[bx_idx] |= mask;
            if (bw4 >= 32) {
                assert(mask == ~0U);
                (*noskip_mask)[1] = mask;
                if (bw4 == 64)
                    (*noskip_mask)[2] = (*noskip_mask)[3] = mask;
            }
        }
    }
    if (f->seq_hdr->sdp && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400 &&
        cbs == BS_INVALID)
    {
        uint8_t *dirmap = &t->luma_intra_dir_mode_map[(t->by & 15) * 16 +
                                                      (t->bx & 15)];
        const int bh4_max16 = imin(bh4, 16);
#define set_ctx(rep_macro) \
        for (int y = 0; y < bh4_max16; y++) { \
            rep_macro(dirmap, 0, midx); \
            dirmap += 16; \
        }
        case_set(b_dim[2]);
#undef set_ctx
    }

#if 0
    if (t->frame_thread.pass == 1 && !b->intra && IS_INTER_OR_SWITCH(f->frame_hdr)) {
        const int sby = (t->by - ts->tiling.row_start) >> f->sb_shift;
        int (*const lowest_px)[2] = ts->lowest_pixel[sby];

        // keep track of motion vectors for each reference
        if (b->comp_type == COMP_INTER_NONE) {
            // y
            if (imin(bw4, bh4) > 1 &&
                ((b->inter_mode == GLOBALMV && f->gmv_warp_allowed[b->ref[0]]) ||
                 (b->motion_mode == MM_WARP_CAUSAL && t->warpmv.type > DAV1D_WM_TYPE_TRANSLATION)))
            {
                affine_lowest_px_luma(t, &lowest_px[b->ref[0]][0], b_dim,
                                      b->motion_mode == MM_WARP_CAUSAL ? &t->warpmv :
                                      &f->frame_hdr->gmv[b->ref[0]]);
            } else {
                mc_lowest_px(&lowest_px[b->ref[0]][0], t->by, bh4, b->mv[0].y,
                             0, &f->svc[b->ref[0]][1]);
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
                         (b->motion_mode == MM_WARP_CAUSAL && t->warpmv.type > DAV1D_WM_TYPE_TRANSLATION)))
                    {
                        affine_lowest_px_chroma(t, &lowest_px[b->ref[0]][1], b_dim,
                                                b->motion_mode == MM_WARP_CAUSAL ? &t->warpmv :
                                                &f->frame_hdr->gmv[b->ref[0]]);
                    } else {
                        mc_lowest_px(&lowest_px[b->ref[0]][1],
                                     t->by & ~ss_ver, bh4 << (bh4 == ss_ver),
                                     b->mv[0].y, ss_ver, &f->svc[b->ref[0]][1]);
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
#endif

    return 0;
}

#if __has_feature(memory_sanitizer)

#include <sanitizer/msan_interface.h>

static int checked_decode_b(Dav1dTaskContext *const t, DB_ONLY(const int depth)
                            const enum BlockSize lbs, const enum BlockSize cbs)
{
    const Dav1dFrameContext *const f = t->f;
    const int err = decode_b(t, DB_ONLY(depth) lbs, cbs);
    enum BlockSize bs[2] = { lbs, cbs };

    if (err == 0 && !(t->frame_thread.pass & 1))
        for (int i = 0; i < 2; i++) {
            if (bs[i] == BS_INVALID) continue;
            const uint8_t *const b_dim = dav1d_block_dimensions[bs[i]];
            const int bw4 = b_dim[0], bh4 = b_dim[1];
            const int w4 = imin(bw4, f->bw - t->bx), h4 = imin(bh4, f->bh - t->by);
            const ptrdiff_t stride = f->cur.stride[i];
            int bx = i ? t->cbx : t->bx, by = i ? t->cby : t->by;

            for (int p = i; p < 1 + i * 2; p++) {
                const int ss_ver = p && f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
                const int ss_hor = p && f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
                const int width  = w4 << (2 - ss_hor + (bw4 == ss_hor));
                const int height = h4 << (2 - ss_ver + (bh4 == ss_ver));
                bx &= ~ss_hor; by &= ~ss_ver;

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

// dir_ptr is a limited recursive partition tree indicator for (ext)sdp as well
// as sdp/cfl delay limits.
// For partitions with luma, the 24th bit indicates whether the parent partition
// prohibits extsdp in child partitions.
// Also for luma, the lower 24 bits are aggregated as the split-direction of
// the child partitions (1: hor, 2: ver, 3: mixed, -1: none] in bit 0-7, the
// next sub-partition (1: hor, 2: ver, 3: mixed, -1: none) in bit 16-23, and
// the child's partition choices in bit 8-15.
// For chroma, these choices can then be used to infer the partition/direction.
// sdp/cfl delay restrictions can also be calculated using these values.

static int decode_sb(Dav1dTaskContext *const t, DB_ONLY(const int depth)
                     const enum BlockSize lbs, enum BlockSize cbs,
                     int *const dir_ptr)
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
    const enum BlockSize cbs_orig = cbs;

    // key/intraonly frames always apply SDP at the 64x64 boundary
    if (lbs == BS_64x64 && cbs == BS_64x64 &&
        f->seq_hdr->sdp && !(f->frame_hdr->frame_type & 1))
    {
        int dir = 0;
        if (decode_sb(t, DB_ONLY(depth) lbs, BS_INVALID, &dir)) return -1;
        return decode_sb(t, DB_ONLY(depth) BS_INVALID, cbs, &dir);
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
            { 11, -1 },
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
        bx4 = t->bx & 63;
        by4 = t->by & 63;
        // FIXME some of the code below needs to be tested for 4:2:2 w/ SDP=1
        const int eff_ss_ver = f->ss_ver & (lbs == BS_INVALID);
        const int eff_ss_hor = f->ss_hor & (lbs == BS_INVALID);
        const int bwh4ss[2] = { bw4 >> eff_ss_hor, bh4 >> eff_ss_ver };
        assert(bwh4ss[0] >= 1 && bwh4ss[1] >= 1);
        int dir = -1;
        if (imax(bwh4ss[0], bwh4ss[1]) == 1 ||
            // 1:8/1:16 partitions don't recursive (normatively)
            (pcc->part[0][0] & pcc->part[1][0]) == -1)
        {
            bp = PARTITION_NONE;
        } else if (!have_h_split || !have_v_split) {
            if (bw4 == bh4) {
                dir = have_v_split;
                bp = !have_v_split ? PARTITION_H : PARTITION_V;
            } else if (bw4 > bh4) {
                if (!have_h_split || f->bh <= t->by + qh4) {
                    dir = 1;
                    bp = PARTITION_V;
                }
            } else if (bh4 > bw4) {
                if (!have_v_split || f->bw <= t->bx + qw4) {
                    dir = 0;
                    bp = PARTITION_H;
                }
            }
        }
        if (bp == PARTITION_INVALID) {
#if DEBUG_BLOCK_INFO
            if (0 && bs == f->root_bs)
                printf("poc=%d,y=%d,x=%d,bs=%d,r=%d\n",
                       f->frame_hdr->frame_offset, t->by, t->bx, bs, ts->msac.rng);
#endif
            if (cbs == BS_64x64 && lbs == BS_INVALID &&
                ((*dir_ptr & 0xff) == 0xff || (*dir_ptr & 0x30003) == 0x10002 ||
                                              (*dir_ptr & 0x30003) == 0x20001))
            {
                // F164: infer SDP chroma partitioning at 64x64 level
                if ((*dir_ptr & 0xff) == 0xff) {
                    bp = PARTITION_NONE; // if luma did not split, don't split chroma
                } else {
                    // if luma split one way, and all children split another way,
                    // then copy the initial first-way split for chroma
                    dir = (*dir_ptr & 0x30003) == 0x10002;
                    bp = (*dir_ptr >> 8) & 0xff;
                }
            } else {
                const int mix_inter = IS_INTER_OR_SWITCH(f->frame_hdr) &&
                                      !t->intra_region;
                const int ctx1 = get_partition_ctx(t->a, &t->l, b_dim, pl, by4, bx4);
                const int ctx2 = ctx1 + pcc->ctx[0] * 4;
                // cannot split 4x8/8x4 blocks in mixed-intra/inter regions of
                // inter frames, since 4x4 block sizes in such regions are invalid
                const int is_split = mix_inter && b_dim[2] + b_dim[3] == 1 ? 0 :
                    (!have_h_split || !have_v_split) ||
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
                            // FIXME do we need to keep track of mix_inter and
                            // resulting block sizes here to ensure we don't
                            // get 4x4 blocks?
                            const int bwh4ss2[2] = { bw4 >> f->ss_hor,
                                                     bh4 >> f->ss_ver };
                            const int has_hv3 = f->seq_hdr->ext_partitions &&
                                bwh4ss[!dir] >= 4 && bwh4ss[dir] >= 2 &&
                                b_dim[!dir] * aspect >= b_dim[dir] * 4 &&
                                (cbs != lbs || (bwh4ss2[!dir] >= 4 &&
                                                bwh4ss2[dir] >= 2) || (dir ?
                                 (lbs == BS_32x8 ? have_v_split :
                                  t->bx + qw4 * 3 < f->bw) :
                                 (lbs == BS_8x32 ? have_h_split :
                                  t->by + qh4 * 3 < f->bh)));
                            const int has_hv4ab = bwh4ss[!dir] >= 8 &&
                                f->seq_hdr->uneven_4way_partitions &&
                                b_dim[!dir] * aspect >= b_dim[dir] * 8 &&
                                (cbs != lbs || bwh4ss2[!dir] >= 8 || (dir ?
                                 (t->bx + (qw4 >> 1) * 7 < f->bw) :
                                 (t->by + (qh4 >> 1) * 7 < f->bh)));
                            if (has_hv3 || has_hv4ab) {
                                assert(pcc->part[dir][1] != -1);
                                const int ctx5 =
                                    get_partition2_ctx(t->a, &t->l, b_dim,
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
        dir += dir != -1; // -1 for PARTITION_NONE, 1 or 2 for hor/ver
        // F157 "limit SDP-imposed CfL delay"
        if (lbs == BS_INVALID && cbs == BS_64x64)
            t->sdp_cfl_disallowed = dir != -1 && dir != (*dir_ptr & 0x3);
        *dir_ptr |= (uint8_t) dir | (bp << 8);

        if (IS_INTER_OR_SWITCH(f->frame_hdr) && f->seq_hdr->ext_sdp &&
            (cbs | lbs) != BS_INVALID && bp != PARTITION_NONE &&
            !(*dir_ptr & (1 << 24)) && // parent partition limits recursive extsdp
            bp < PARTITION_H4A && imin(bw4, bh4) >= 2 && imax(bw4, bh4) <= 16)
        {
            const int sz = b_dim[2] + b_dim[3];
            const int ctx = iclip(sz - 4, 0, 3) + (sz == 4);
            t->intra_region = !dav1d_msac_decode_bool_adapt(&ts->msac,
                                   ts->cdf.m.region_type[ctx]);
            DEBUG_BLOCK_printf("%*sis_mixed_region[ctx=%d,%d]: r=%d\n",
                               depth, "", ctx, !t->intra_region, ts->msac.rng);
            if (t->intra_region) cbs = BS_INVALID;
        }
    } else {
        //.. FIXME 2-pass decoding
        abort();
    }

    if (bs == cbs) {
        t->cbx = t->bx;
        t->cby = t->by;
    }
    // can child partitions split?
    static const uint8_t lim[][2] = {
        [PARTITION_NONE]  = { 1, 1 },
        [PARTITION_H]     = { 1, 2 },
        [PARTITION_V]     = { 2, 1 },
        [PARTITION_H3]    = { 2, 4 },
        [PARTITION_V3]    = { 4, 2 },
        [PARTITION_H4A]   = { 1, 8 },
        [PARTITION_H4B]   = { 1, 8 },
        [PARTITION_V4A]   = { 8, 1 },
        [PARTITION_V4B]   = { 8, 1 },
        [PARTITION_SPLIT] = { 2, 2 },
    };
    const uint8_t *const l = lim[bp];
    int child_dir = (bw4 <= l[0] || bh4 <= l[1]) << 24;
    switch (bp) {
    case PARTITION_NONE:
        if (decode_b(t, DB_ONLY(depth + 1) lbs, cbs)) return -1;
        if (t->frame_thread.pass != 2) {
            if ((cbs | lbs) != BS_INVALID) {
                BlockContext *edge = t->a;
#define set_ctx(rep_macro) \
                rep_macro(edge->partition[0], off, (uint8_t) ~(b_dim[i] - 1)); \
                rep_macro(edge->partition[1], off, (uint8_t) ~(b_dim[i] - 1))
                for (int i = 0, off = bx4; i < 2; i++, off = by4, edge = &t->l) {
                    case_set(b_dim[2 + i]);
                }
#undef set_ctx
            } else {
                dav1d_memset_pow2[b_dim[2]](&t->a->partition[pl][bx4],
                                            (uint8_t) ~(b_dim[0] - 1));
                dav1d_memset_pow2[b_dim[3]](&t->l.partition[pl][by4],
                                            (uint8_t) ~(b_dim[1] - 1));
            }
        }
        break;
    case PARTITION_V: {
        assert(hw4 > 0);
        const int sub4 = bs == cbs && (hw4 >> f->ss_hor) > 0;
        assert(sub4 || !pl);
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][0],
                      sub4 ? pcc->part[1][0] : BS_INVALID, &child_dir))
        {
            return -1;
        }
        if (t->bx + hw4 >= f->bw) break;
        t->bx += hw4;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][0],
                      sub4 ? pcc->part[1][0] : cbs, &child_dir))
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
                      sub4 ? pcc->part[0][0] : BS_INVALID, &child_dir))
        {
            return -1;
        }
        if (t->by + hh4 >= f->bh) break;
        t->by += hh4;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][0],
                      sub4 ? pcc->part[0][0] : cbs, &child_dir))
        {
            return -1;
        }
        t->by -= hh4;
        break;
    }
    case PARTITION_SPLIT: {
        assert(have_v_split && have_h_split && cbs == lbs);
        const enum BlockSize sbs = pcc->part[0][3];
        if (decode_sb(t, DB_ONLY(depth + 1) sbs, sbs, &child_dir)) return -1;
        t->bx += hw4;
        if (decode_sb(t, DB_ONLY(depth + 1) sbs, sbs, &child_dir)) return -1;
        t->bx -= hw4;
        t->by += hh4;
        if (decode_sb(t, DB_ONLY(depth + 1) sbs, sbs, &child_dir)) return -1;
        t->bx += hw4;
        if (decode_sb(t, DB_ONLY(depth + 1) sbs, sbs, &child_dir)) return -1;
        t->bx -= hw4;
        t->by -= hh4;
        break;
    }
    case PARTITION_V3: {
        assert(qw4 > 0 && hh4 > 0);
        const int sub4 = bs == cbs && (qw4 >> f->ss_hor) > 0 &&
                                      (hh4 >> f->ss_ver) > 0;
        assert(sub4 || !pl);
        const int i_3only = cbs == BS_INVALID || (!sub4 && bs != BS_32x8);
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][1],
                      i_3only ? BS_INVALID : pcc->part[1][1], &child_dir))
        {
            return -1;
        }
        if (t->bx + qw4 >= f->bw) break;
        t->bx += qw4;
        if (!i_3only) t->cbx = t->bx;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][3],
                      sub4 ? pcc->part[1][3] : BS_INVALID, &child_dir))
        {
            return -1;
        }
        if (t->by + hh4 < f->bh) {
            t->by += hh4;
            if (decode_sb(t, DB_ONLY(depth + 1)
                          pl ? BS_INVALID : pcc->part[1][3],
                          i_3only ? BS_INVALID : pcc->part[1][sub4 * 3], &child_dir))
            {
                return -1;
            }
            t->by -= hh4;
        }
        if (t->bx + hw4 >= f->bw) { t->bx -= qw4; break; }
        t->bx += hw4;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][1],
                      i_3only ? cbs : pcc->part[1][1], &child_dir))
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
        const int i_3only = cbs == BS_INVALID || (!sub4 && bs != BS_8x32);
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][1],
                      i_3only ? BS_INVALID : pcc->part[0][1], &child_dir))
        {
            return -1;
        }
        if (t->by + qh4 >= f->bh) break;
        t->by += qh4;
        if (!i_3only) t->cby = t->by;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][3],
                      sub4 ? pcc->part[0][3] : BS_INVALID, &child_dir))
        {
            return -1;
        }
        if (t->bx + hw4 < f->bw) {
            t->bx += hw4;
            if (decode_sb(t, DB_ONLY(depth + 1)
                          pl ? BS_INVALID : pcc->part[0][3],
                          i_3only ? BS_INVALID : pcc->part[0][sub4 * 3], &child_dir))
            {
                return -1;
            }
            t->bx -= hw4;
        }
        if (t->by + hh4 >= f->bh) { t->by -= qh4; break; }
        t->by += hh4;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][1],
                      i_3only ? cbs : pcc->part[0][1], &child_dir))
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
                      sub4 ? pcc->part[1][2] : BS_INVALID, &child_dir))
        {
            return -1;
        }
        if (t->bx + ew4 >= f->bw) break;
        t->bx += ew4;
        const int var = bp - PARTITION_V4A; // v4b: 1, v4a: 0
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][!var],
                      sub4 ? pcc->part[1][!var] : -1, &child_dir))
        {
            return -1;
        }
        const int w4a = qw4 << var, w4b = hw4 >> var;
        if (t->bx + w4a >= f->bw) { t->bx -= ew4; break; }
        t->bx += w4a;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][var],
                      sub4 ? pcc->part[1][var] : -1, &child_dir))
        {
            return -1;
        }
        if (t->bx + w4b >= f->bw) { t->bx -= ew4 + w4a; break; }
        t->bx += w4b;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[1][2],
                      sub4 ? pcc->part[1][2] : cbs, &child_dir))
        {
            return -1;
        }
        t->bx -= 7 * ew4;
        break;
    }
    case PARTITION_H4A:
    case PARTITION_H4B: {
        const int eh4 = qh4 >> 1;
        assert(eh4 > 0);
        const int sub4 = bs == cbs && (eh4 >> f->ss_ver) > 0;
        assert(sub4 || !pl);
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][2],
                      sub4 ? pcc->part[0][2] : BS_INVALID, &child_dir))
        {
            return -1;
        }
        if (t->by + eh4 >= f->bh) break;
        t->by += eh4;
        const int var = bp - PARTITION_H4A; // h4b: 1, h4a: 0
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][!var],
                      sub4 ? pcc->part[0][!var] : -1, &child_dir))
        {
            return -1;
        }
        const int h4a = qh4 << var, h4b = hh4 >> var;
        if (t->by + h4a >= f->bh) { t->by -= eh4; break; }
        t->by += h4a;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][var],
                      sub4 ? pcc->part[0][var] : -1, &child_dir))
        {
            return -1;
        }
        if (t->by + h4b >= f->bh) { t->by -= eh4 + h4a; break; }
        t->by += h4b;
        if (decode_sb(t, DB_ONLY(depth + 1)
                      pl ? BS_INVALID : pcc->part[0][2],
                      sub4 ? pcc->part[0][2] : cbs, &child_dir))
        {
            return -1;
        }
        t->by -= 7 * eh4;
        break;
    }
    default:
        assert(0);
    }

    *dir_ptr |= (child_dir & 0xff) << 16;

    if (t->intra_region && cbs_orig != BS_INVALID) {
        t->cbx = t->bx;
        t->cby = t->by;
        if (decode_b(t, DB_ONLY(depth + 1) BS_INVALID, cbs_orig)) return -1;
        t->intra_region = 0;
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
    memset(ctx->tx_lpf_y, 3, sizeof(ctx->tx_lpf_y));
    memset(ctx->tx_lpf_uv, 2, sizeof(ctx->tx_lpf_uv));
    if (!keyframe) {
        memset(ctx->ref, -1, sizeof(ctx->ref));
        memset(ctx->comp_type, 0, sizeof(ctx->comp_type));
        memset(ctx->mode, NEARMV, sizeof(ctx->mode));
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
    const int col_sb_start = f->frame_hdr->tiling.t.col_start_sb[tile_col];
    const int col_sb_end = f->frame_hdr->tiling.t.col_start_sb[tile_col + 1];
    const int row_sb_start = f->frame_hdr->tiling.t.row_start_sb[tile_row];
    const int row_sb_end = f->frame_hdr->tiling.t.row_start_sb[tile_row + 1];
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

static inline int decode_4way(MsacContext *const s, const int ref,
                              uint16_t *const cdf, int n_bits)
{
    assert(n_bits >= 4);
    const int bin = dav1d_msac_decode_symbol_adapt4(s, cdf, 3);
    const int rem = dav1d_msac_decode_bools_bypass(s, n_bits + bin + !bin - 4);
    const int v = (bin ? (1 << (n_bits + bin - 4)) : 0) + rem;
    const int n = 1 << n_bits;
    return ref * 2 <= n ? inv_recenter(ref, v) :
                          n - 1 - inv_recenter(n - 1 - ref, v);
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
            lr->type = type ? DAV1D_RESTORATION_PC_WIENER :
                              DAV1D_RESTORATION_NS_WIENER;
        }
    } else {
        assert(!p || frame_type == DAV1D_RESTORATION_NS_WIENER);
        uint16_t *const cdf = frame_type == DAV1D_RESTORATION_NS_WIENER ?
                              ts->cdf.m.rst_ns_wiener : ts->cdf.m.rst_pc_wiener;
        const int type = dav1d_msac_decode_bool_adapt(&ts->msac, cdf);
        lr->type = type ? frame_type : DAV1D_RESTORATION_NONE;
    }

    const struct Dav1dNSWienerPlane *const pd = &f->frame_hdr->restoration.p[p].ns;
    if (lr->type == DAV1D_RESTORATION_NS_WIENER && !pd->frame_filters_on) {
        const int n_classes = pd->num_classes;
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

        const unsigned *const masks = p ? dav1d_subset_masks_uv : dav1d_subset_masks_y;
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
            memset(filter, 0, 16 + !!p * 2);
            int s;
            for (s = 0; s < 3 - !!p; s++) {
                const int found = dav1d_msac_decode_bool_adapt(&ts->msac,
                                      ts->cdf.m.wiener_ns_len[!!p]);
                if (!found) break;
            }
            const unsigned mask = masks[s];
            const int asym = p && s &&
                dav1d_msac_decode_bool_adapt(&ts->msac, ts->cdf.m.wiener_ns_sym);
            for (int i = 0, m = mask; i < 16 + !!p * 2; i++, m >>= 1) {
                if (!(m & 1)) continue;
                filter[i] = decode_4way(&ts->msac, ref_filter[i] - cf_range[i][1],
                                        ts->cdf.m.wiener_ns_cf, cf_range[i][0]) +
                            cf_range[i][1];
                if (asym && i >= 6) {
                    filter[i + 1] = filter[i];
                    i++;
                    m >>= 1;
                }
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
    const enum BlockSize root_bs = f->root_bs;
    const enum BlockSize c_root_bs =
        f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I400 ? BS_INVALID : root_bs;
    Dav1dTileState *const ts = t->ts;
    const Dav1dContext *const c = f->c;
    const int sb_step = f->sb_step;
    const int tile_row = ts->tiling.row, tile_col = ts->tiling.col;

    if (IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc) {
        dav1d_refmvs_tile_sbrow_init(&t->rt, &f->rf,
                                     ts->tiling.col_start, ts->tiling.col_end,
                                     ts->tiling.row_start, ts->tiling.row_end,
                                     t->by >> f->sb_shift, ts->tiling.row,
                                     t->frame_thread.pass);
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
        const int off_2pass = c->n_tc > 1 ? f->sb256w * f->frame_hdr->tiling.t.rows : 0;
        for (t->bx = ts->tiling.col_start;
             t->bx < ts->tiling.col_end; t->bx += sb_step)
        {
            t->a = f->a + (t->bx >> 6) + off_2pass + tile_row * f->sb256w;
            if (atomic_load_explicit(c->flush, memory_order_acquire))
                return 1;
            if (decode_sb(t, DB_ONLY(1) root_bs, c_root_bs, NULL))
                return 1;
        }
        f->bd_fn.backup_ipred_edge(t);
        return 0;
    }

    if (f->c->n_tc > 1 && f->frame_hdr->use_ref_frame_mvs) {
        dav1d_refmvs_load_tmvs(&f->rf, ts->tiling.row,
                               ts->tiling.col_start >> 1, ts->tiling.col_end >> 1,
                               t->by >> 1, (t->by + sb_step) >> 1);
    }
    const int sb256y = t->by >> 6;
    for (t->bx = ts->tiling.col_start;
         t->bx < ts->tiling.col_end; t->bx += sb_step)
    {
        memset(t->is_coded, 0, sizeof(t->is_coded));
        t->lf_mask = f->lf.mask + (t->bx >> 6) + sb256y * f->sb256w;
        t->a = f->a + tile_row * f->sb256w + (t->bx >> 6);
        if (atomic_load_explicit(c->flush, memory_order_acquire))
            return 1;
        switch (root_bs) {
        default: assert(0);
        case BS_64x64: {
            const int idx = ((t->bx & 0x30) >> 4) + ((t->by & 0x30) >> 2);
            t->lf_mask->cdef_idx[idx] = -1;
            break;
        }
        case BS_128x128: {
            const int idx = ((t->bx & 32) >> 4) + ((t->by & 32) >> 2);
            memset(&t->lf_mask->cdef_idx[idx + 0], -1, 2);
            memset(&t->lf_mask->cdef_idx[idx + 4], -1, 2);
            break;
        }
        case BS_256x256:
            memset(t->lf_mask->cdef_idx, -1, 16);
            break;
        }
        // Restoration filter
        const int sbsz = f->sb_step * 4;
        for (int p = 0, ss_ver = 0, ss_hor = 0; p < 3;
             p++, ss_ver = f->ss_ver, ss_hor = f->ss_hor)
        {
            if (!((f->lf.restore_planes >> p) & 1U))
                continue;

            const int tx = 4 * (t->bx - ts->tiling.col_start) >> ss_hor;
            const int ty = 4 * (t->by - ts->tiling.row_start) >> ss_ver;
            const int unit_sz_log2 = f->frame_hdr->restoration.unit_size[!!p];
            const int unit_sz = 1 << unit_sz_log2;
            const unsigned mask = unit_sz - 1;
            if ((tx | ty) & mask) continue;
            const int tw = ts->tiling.col_end * 4 >> ss_hor;
            const int th = ts->tiling.row_end * 4 >> ss_ver;
            const int half_unit = unit_sz >> 1;
            // Round half up at frame boundaries, if there's more than one
            // restoration unit
            const int fx = 4 * t->bx >> ss_hor, fy = t->by * 4 >> ss_ver;
            if ((ty && fy + half_unit > th) || (tx && fx + half_unit > tw))
                continue;

            const enum Dav1dRestorationType frame_type = f->frame_hdr->restoration.p[p].type;

            // FIXME many of these values can be pre-calculated at frame-level
            const int sbw = sbsz >> ss_hor, sbh = sbsz >> ss_ver;
            const int lruw = imax(1, imin(tw - fx + half_unit, sbw) >> unit_sz_log2);
            const int lruh = imax(1, imin(th - fy + half_unit, sbh) >> unit_sz_log2);
            const int vsh = unit_sz_log2 - 7 + ss_ver;
            const int hsh = unit_sz_log2 - 7 + ss_hor;
            int sb_idx = (t->by >> 6) * f->sb256w + (t->bx >> 6);
            // FIXME I think lruh is always 1, so this loop may be eliminated
            for (int y = 0; y < lruh; y++, sb_idx += f->sb256w << vsh) {
                for (int x = 0; x < lruw; x++) {
                    // FIXME [0] is probably not correct
                    Av1RestorationUnit *const lr =
                        &f->lf.lr_mask[sb_idx + (x << hsh)].lr[p][0];
                    read_restoration_info(t, lr, p, frame_type);
                    DEBUG_BLOCK_printf("Post-restoration[p=%d,type=%d]: r=%d\n",
                                       p, lr->type, ts->msac.rng);
                }
            }
        }
        int dir = 0;
        t->sdp_cfl_disallowed = 0;
        if (IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc) {
            dav1d_refmvs_reset_sb(&t->rt, t->by, t->bx);
        }
        if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
            // for some contexts related to warp-motion, AVM uses 8x8 (instead
            // of 4x4) context resolution when we cross SB boundaries. However,
            // the way this is implemented means we sometimes go outside the
            // bounds of our own block into data that has already been written
            // into by our neighbour blocks. For example, if we access "top" at
            // 8x8 resolution for x=25, this may round to x=24 (which our left-
            // neighbour just overwrote). To workaround this, we keep a copy of
            // all affected context bits at SB boundaries. See AVM #1091.
            memcpy(t->a_sb_cache.ref[0], t->a->ref[0], 64);
            memcpy(t->a_sb_cache.ref[1], t->a->ref[1], 64);
            if (t->by > ts->tiling.row_start)
                memcpy(t->a_sb_cache.motion_mode, t->a->motion_mode, 64);
        }
        if (decode_sb(t, DB_ONLY(1) root_bs, c_root_bs, &dir))
            return 1;
        if ((IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc)) {
            dav1d_refmvs_save_tmvs(&f->c->refmvs_dsp, &t->rt,
                                   t->bx >> 1, (t->bx + sb_step) >> 1,
                                   t->by >> 1, (t->by + sb_step) >> 1);
        }
    }

    // backup pre-loopfilter pixels for intra prediction of the next sbrow
    if (t->frame_thread.pass != 1)
        f->bd_fn.backup_ipred_edge(t);

    // backup t->a/l.tx_lpf_y/uv at tile boundaries to use them to "fix"
    // up the initial value in neighbour tiles when running the loopfilter
    int align_h = (f->bh + 63) & ~63;
    memcpy(&f->lf.tx_lpf_right_edge[0][align_h * tile_col + t->by],
           &t->l.tx_lpf_y[t->by & 0x30], sb_step);
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    align_h >>= ss_ver;
    memcpy(&f->lf.tx_lpf_right_edge[1][align_h * tile_col + (t->by >> ss_ver)],
           &t->l.tx_lpf_uv[(t->by & 0x30) >> ss_ver], sb_step >> ss_ver);

    // error out on symbol decoder overread
    if (ts->msac.cnt <= -15) return 1;

    return c->strict_std_compliance &&
           (t->by >> f->sb_shift) + 1 >= f->frame_hdr->tiling.t.row_start_sb[tile_row + 1] &&
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
    for (int tile_row = 0; tile_row < f->frame_hdr->tiling.t.rows; tile_row++) {
        f->lf.start_of_tile_row[sby++] = tile_row;
        while (sby < f->frame_hdr->tiling.t.row_start_sb[tile_row + 1])
            f->lf.start_of_tile_row[sby++] = 0;
    }

    const int n_ts = f->frame_hdr->tiling.t.cols * f->frame_hdr->tiling.t.rows;
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

    const int a_sz = f->sb256w * f->frame_hdr->tiling.t.rows * (1 + (c->n_fc > 1 && c->n_tc > 1));
    if (a_sz != f->a_sz) {
        dav1d_free(f->a);
        f->a = dav1d_malloc(ALLOC_TILE, sizeof(*f->a) * a_sz);
        if (!f->a) {
            f->a_sz = 0;
            goto error;
        }
        f->a_sz = a_sz;
    }

    const int num_sb256 = f->sb256w * f->sb256h;
    const uint8_t *const size_mul = ss_size_mul[f->cur.p.layout];
    const int hbd = !!f->seq_hdr->hbd;
    if (c->n_fc > 1) {
        const unsigned sb_step4 = f->sb_step * 4;
        int tile_idx = 0;
        for (int tile_row = 0; tile_row < f->frame_hdr->tiling.t.rows; tile_row++) {
            const unsigned row_off = f->frame_hdr->tiling.t.row_start_sb[tile_row] *
                                     sb_step4 * f->sb256w * 256;
            const unsigned b_diff = (f->frame_hdr->tiling.t.row_start_sb[tile_row + 1] -
                                     f->frame_hdr->tiling.t.row_start_sb[tile_row]) * sb_step4;
            for (int tile_col = 0; tile_col < f->frame_hdr->tiling.t.cols; tile_col++) {
                f->frame_thread.tile_start_off[tile_idx++] = row_off + b_diff *
                    f->frame_hdr->tiling.t.col_start_sb[tile_col] * sb_step4;
            }
        }

        const int lowest_pixel_mem_sz = f->frame_hdr->tiling.t.cols * f->sbh;
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
        for (int tile_row = 0, tile_row_base = 0; tile_row < f->frame_hdr->tiling.t.rows;
             tile_row++, tile_row_base += f->frame_hdr->tiling.t.cols)
        {
            const int tile_row_sb_h = f->frame_hdr->tiling.t.row_start_sb[tile_row + 1] -
                                      f->frame_hdr->tiling.t.row_start_sb[tile_row];
            for (int tile_col = 0; tile_col < f->frame_hdr->tiling.t.cols; tile_col++) {
                f->ts[tile_row_base + tile_col].lowest_pixel = lowest_pixel_ptr;
                lowest_pixel_ptr += tile_row_sb_h;
            }
        }

        const int cbi_sz = num_sb256 * size_mul[0];
        if (cbi_sz != f->frame_thread.cbi_sz) {
            dav1d_free_aligned(f->frame_thread.cbi);
            f->frame_thread.cbi =
                dav1d_alloc_aligned(ALLOC_BLOCK, sizeof(*f->frame_thread.cbi) *
                                    cbi_sz * 64 * 64 / 4, 64);
            if (!f->frame_thread.cbi) {
                f->frame_thread.cbi_sz = 0;
                goto error;
            }
            f->frame_thread.cbi_sz = cbi_sz;
        }

        const int cf_sz = (num_sb256 * size_mul[0]) << hbd;
        if (cf_sz != f->frame_thread.cf_sz) {
            dav1d_free_aligned(f->frame_thread.cf);
            f->frame_thread.cf =
                dav1d_alloc_aligned(ALLOC_COEF, (size_t)cf_sz * 256 * 256 / 2, 64);
            if (!f->frame_thread.cf) {
                f->frame_thread.cf_sz = 0;
                goto error;
            }
            memset(f->frame_thread.cf, 0, (size_t)cf_sz * 256 * 256 / 2);
            f->frame_thread.cf_sz = cf_sz;
        }

        if (f->frame_hdr->allow_screen_content_tools) {
            const int pal_sz = num_sb256 << hbd;
            if (pal_sz != f->frame_thread.pal_sz) {
                dav1d_free_aligned(f->frame_thread.pal);
                f->frame_thread.pal =
                    dav1d_alloc_aligned(ALLOC_PAL, sizeof(*f->frame_thread.pal) *
                                        pal_sz * 32 * 32, 64);
                if (!f->frame_thread.pal) {
                    f->frame_thread.pal_sz = 0;
                    goto error;
                }
                f->frame_thread.pal_sz = pal_sz;
            }

            const int pal_idx_sz = num_sb256 * size_mul[1];
            if (pal_idx_sz != f->frame_thread.pal_idx_sz) {
                dav1d_free_aligned(f->frame_thread.pal_idx);
                f->frame_thread.pal_idx =
                    dav1d_alloc_aligned(ALLOC_PAL, sizeof(*f->frame_thread.pal_idx) *
                                        pal_idx_sz * 256 * 256 / 8, 64);
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

    const int sb256 = f->frame_hdr->sb128;
    const int num_lines = c->n_tc > 1 ? f->sbh * 4 << sb256 : 12;
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
    if (num_sb256 != f->lf.mask_sz) {
        dav1d_free(f->lf.mask);
        f->lf.mask = dav1d_malloc(ALLOC_LF, sizeof(*f->lf.mask) * num_sb256);
        // over-allocate by 3 bytes since some of the SIMD implementations
        // index this from the level type and can thus over-read by up to 3
        if (!f->lf.mask) {
            f->lf.mask_sz = 0;
            goto error;
        }
        if (c->n_fc > 1) {
            dav1d_free(f->frame_thread.b);
            f->frame_thread.b = dav1d_malloc(ALLOC_BLOCK, sizeof(*f->frame_thread.b) *
                                             num_sb256 * 64 * 64);
            if (!f->frame_thread.b) {
                f->lf.mask_sz = 0;
                goto error;
            }
        }
        f->lf.mask_sz = num_sb256;
    }

    const int lr_mask_sz = f->sb256w * f->sb256h;
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
    if (f->frame_hdr->loopfilter.level_y[0] || f->frame_hdr->loopfilter.level_y[1]) {
        init_deblock_lut(f->seq_hdr, f->frame_hdr, f->frame_hdr->quant.yac, &f->lf.thr_lut);
    }
    dav1d_calc_lf_values(f->lf.lvl, f->frame_hdr, (int8_t[4]) { 0, 0, 0, 0 });
    memset(f->lf.mask, 0, sizeof(*f->lf.mask) * num_sb256);

    const int ipred_edge_sz = f->sbh * f->sb256w << hbd;
    if (ipred_edge_sz != f->ipred_edge_sz) {
        dav1d_free_aligned(f->ipred_edge[0]);
        uint8_t *ptr = f->ipred_edge[0] =
            dav1d_alloc_aligned(ALLOC_IPRED, ipred_edge_sz * 256 * 3, 64);
        if (!ptr) {
            f->ipred_edge_sz = 0;
            goto error;
        }
        f->ipred_edge[1] = ptr + ipred_edge_sz * 256 * 1;
        f->ipred_edge[2] = ptr + ipred_edge_sz * 256 * 2;
        f->ipred_edge_sz = ipred_edge_sz;
    }

    const int re_sz = f->sb256h * f->frame_hdr->tiling.t.cols;
    if (re_sz != f->lf.re_sz) {
        dav1d_free(f->lf.tx_lpf_right_edge[0]);
        f->lf.tx_lpf_right_edge[0] = dav1d_malloc(ALLOC_LF, re_sz * 64 * 2);
        if (!f->lf.tx_lpf_right_edge[0]) {
            f->lf.re_sz = 0;
            goto error;
        }
        f->lf.tx_lpf_right_edge[1] = f->lf.tx_lpf_right_edge[0] + re_sz * 64;
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

    if (f->use_pri_sec_cdf) {
        dav1d_cdf_pri_sec_average(f->in_cdf.data.cdf,
                                  &f->src_cdf[0], &f->src_cdf[1]);
    }
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

            if (tile_col == f->frame_hdr->tiling.t.cols) {
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
        for (int n = 0; n < f->sb256w * f->frame_hdr->tiling.t.rows * (1 + uses_2pass); n++)
            reset_context(&f->a[n], IS_KEY_OR_INTRA(f->frame_hdr),
                          uses_2pass ? 1 + (n >= f->sb256w * f->frame_hdr->tiling.t.rows) : 0);
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

    for (int n = 0; n < f->sb256w * f->frame_hdr->tiling.t.rows; n++)
        reset_context(&f->a[n], IS_KEY_OR_INTRA(f->frame_hdr), 0);

    for (int tile_row = 0; tile_row < f->frame_hdr->tiling.t.rows; tile_row++) {
        const int sby_start = f->frame_hdr->tiling.t.row_start_sb[tile_row];
        const int sbh_end =
            imin(f->frame_hdr->tiling.t.row_start_sb[tile_row + 1], f->sbh);
        for (int sby = sby_start; sby < sbh_end; sby++) {
            t->by = sby << (4 + f->frame_hdr->sb128);
            const int by_end = (t->by + f->sb_step) >> 1;
            if (f->frame_hdr->use_ref_frame_mvs) {
                dav1d_refmvs_load_tmvs(&f->rf, tile_row,
                                       0, f->bw >> 1, t->by >> 1, by_end);
            }
            for (int tile_col = 0; tile_col < f->frame_hdr->tiling.t.cols; tile_col++) {
                t->ts = &f->ts[tile_row * f->frame_hdr->tiling.t.cols + tile_col];
                if (dav1d_decode_tile_sbrow(t)) goto error;
            }
        }
        // post filters (deblock + cdef + ccso + ...)
        // do this after completing full tiles, so that intra bc works correctly
        for (int sby = sby_start; sby < sbh_end; sby++) {
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
               (size_t)f->frame_thread.cf_sz * 256 * 256 / 2);
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
    if (f->frame_hdr && f->use_pri_sec_cdf) {
        dav1d_cdf_thread_unref(&f->src_cdf[0]);
        dav1d_cdf_thread_unref(&f->src_cdf[1]);
    }
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
                const int shift = f->frame_hdr->tiling.t.log2_cols +
                                  f->frame_hdr->tiling.t.log2_rows;
                if (shift && f->seq_hdr->avg_cdf_type) {
                    const int n_tiles = 1 << shift;
                    dav1d_cdf_shift(f->out_cdf.data.cdf, &f->ts[0].cdf, shift);
                    for (int n = 1; n < n_tiles; n++)
                        dav1d_cdf_shift_accumulate(f->out_cdf.data.cdf,
                                                   &f->ts[n].cdf, shift);
                } else {
                    memcpy(f->out_cdf.data.cdf,
                           &f->ts[f->frame_hdr->tiling.update].cdf,
                           sizeof(CdfContext));
                }
                dav1d_cdf_reset_count(f->frame_hdr, f->out_cdf.data.cdf);
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

#if 0
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
    } else
#endif
    {
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
            dav1d_ccso_dsp_init_##bd##bpc(&dsp->ccso); \
            dav1d_cdef_dsp_init_##bd##bpc(&dsp->cdef); \
            dav1d_intra_pred_dsp_init_##bd##bpc(&dsp->ipred); \
            dav1d_itx_dsp_init_##bd##bpc(&dsp->itx, bpc); \
            dav1d_stx_dsp_init_##bd##bpc(&dsp->stx); \
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
        f->bd_fn.recon_b = dav1d_recon_b_##bd##bpc; \
        f->bd_fn.filter_sbrow = dav1d_filter_sbrow_##bd##bpc; \
        f->bd_fn.filter_sbrow_deblock_cols = dav1d_filter_sbrow_deblock_cols_##bd##bpc; \
        f->bd_fn.filter_sbrow_deblock_rows = dav1d_filter_sbrow_deblock_rows_##bd##bpc; \
        f->bd_fn.filter_sbrow_cdef = dav1d_filter_sbrow_cdef_##bd##bpc; \
        f->bd_fn.filter_sbrow_lr = dav1d_filter_sbrow_lr_##bd##bpc; \
        f->bd_fn.backup_ipred_edge = dav1d_backup_ipred_edge_##bd##bpc; \
        f->bd_fn.read_coef_blocks = dav1d_read_coef_blocks_##bd##bpc; \
        f->bd_fn.copy_pal_block_y = dav1d_copy_pal_block_y_##bd##bpc; \
        f->bd_fn.read_pal_plane = dav1d_read_pal_plane_##bd##bpc
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
    const int p_ref_idx = f->frame_hdr->primary_ref_frame;
    if (p_ref_idx == DAV1D_PRIMARY_REF_NONE) {
        dav1d_cdf_thread_init_static(&f->in_cdf, f->frame_hdr->quant.yac);
        f->use_pri_sec_cdf = 0;
    } else {
        const int s_ref_idx = f->frame_hdr->secondary_ref_frame;
        const int pri_ref = f->frame_hdr->refidx[p_ref_idx];
        f->use_pri_sec_cdf = s_ref_idx != DAV1D_PRIMARY_REF_NONE &&
                             f->frame_hdr->frame_type == DAV1D_FRAME_TYPE_INTER &&
                             f->seq_hdr->avg_cdf && !f->seq_hdr->avg_cdf_type &&
                             f->frame_hdr->tip.frame_mode != 2;
        if (!f->use_pri_sec_cdf) {
            dav1d_cdf_thread_ref(&f->in_cdf, &c->cdf[pri_ref]);
        } else {
            const int sec_ref = f->frame_hdr->refidx[s_ref_idx];
            res = dav1d_cdf_thread_alloc(c, &f->in_cdf, c->n_fc > 1);
            if (res < 0) goto error;
            dav1d_cdf_thread_ref(&f->src_cdf[0], &c->cdf[pri_ref]);
            dav1d_cdf_thread_ref(&f->src_cdf[1], &c->cdf[sec_ref]);
        }
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
    struct OutputQueue *q = NULL;
    if (f->frame_hdr->show_frame || c->output_invisible_frames) {
        q = queue_output(c, &f->sr_cur);
#if 0
        c->event_flags |= dav1d_picture_get_event_flags(&f->sr_cur);
#endif
    }

    // ss_ver is set for 4:2:0, and ss_hor for 4:2:0 & 4:2:2
    f->ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    f->ss_hor = f->cur.p.layout - 1 < (unsigned) DAV1D_PIXEL_LAYOUT_I444 - 1;
    f->root_bs = (const uint8_t[]) { BS_64x64, BS_128x128,
                                     BS_256x256 }[f->frame_hdr->sb128];
    f->bw = ((f->frame_hdr->width + 7) >> 3) << 1;
    f->bh = ((f->frame_hdr->height + 7) >> 3) << 1;
    f->sb256w = (f->bw + 63) >> 6;
    f->sb256h = (f->bh + 63) >> 6;
    f->sb_shift = 4 + f->frame_hdr->sb128;
    f->sb_step = 16 << f->frame_hdr->sb128;
    f->sbh = (f->bh + f->sb_step - 1) >> f->sb_shift;
    f->b4_stride = (f->bw + 63) & ~63;
    f->bitdepth_max = (1 << f->cur.p.bpc) - 1;
    atomic_init(&f->task_thread.error, 0);
    const int uses_2pass = c->n_fc > 1;
    const int cols = f->frame_hdr->tiling.t.cols;
    const int rows = f->frame_hdr->tiling.t.rows;
    atomic_store(&f->task_thread.task_counter,
                 (cols * rows + f->sbh) << uses_2pass);

    // ref_mvs
    if (IS_INTER_OR_SWITCH(f->frame_hdr) || f->frame_hdr->allow_intrabc) {
        f->mvs_ref = dav1d_ref_create_using_pool(c->refmvs_pool,
            sizeof(*f->mvs) * f->sb256h * 32 * (f->b4_stride >> 1));
        if (!f->mvs_ref) {
            res = DAV1D_ERR(ENOMEM);
            goto error;
        }
        f->mvs = f->mvs_ref->data;
        if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
            const int poc = f->cur.frame_hdr->frame_offset;
            // we use -2 here so it doesn't match b->ref==-1, which means intra
            int furthest_future_refidx = -2;
            for (int i = 0; i < 7; i++) {
                f->refpoc[i] = f->refp[i].p.frame_hdr->frame_offset;
                const int delta = f->refdist[i] =
                    get_poc_diff(f->seq_hdr->order_hint_n_bits, f->refpoc[i], poc);
                f->absrefdist[i] = abs(delta);
                f->refdir[i] = delta > 0;
                if (delta > 0 && (furthest_future_refidx < 0 ||
                                  f->refdist[furthest_future_refidx] < delta))
                {
                    furthest_future_refidx = i;
                }
            }
            f->furthest_future_refidx = furthest_future_refidx;
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
                sizeof(*f->cur_segmap) * f->b4_stride * 64 * f->sb256h);
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
            const size_t segmap_size = sizeof(*f->cur_segmap) * f->b4_stride * 64 * f->sb256h;
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

    // skipmode
    f->skip_mode_refs[0] = 0;
    f->skip_mode_refs[1] = f->frame_hdr->skip_mode_enabled &&
                           f->frame_hdr->n_ref_frames > 1 &&
                           abs(f->absrefdist[0] - f->absrefdist[1]) <= 1;

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
            if (IS_INTER_OR_SWITCH(f->frame_hdr)) {
                c->refs[i].refmvs = f->mvs_ref;
                if (f->mvs_ref)
                    dav1d_ref_inc(f->mvs_ref);
            }
            memcpy(c->refs[i].refpoc, f->refpoc, sizeof(f->refpoc));
        }
    }

    if (f->frame_hdr->tip.frame_mode == 2) {
        // FIXME run actual reconstruction
        // this will likely be like a pass=2-only reconstruction,
        // once that is implemented
        dav1d_cdf_thread_unref(&f->in_cdf);
        assert(!f->use_pri_sec_cdf);
        assert(!f->frame_hdr->refresh_context);
        for (int i = 0; i < 7; i++) {
            if (f->refp[i].p.frame_hdr)
                dav1d_thread_picture_unref(&f->refp[i]);
            dav1d_ref_dec(&f->ref_mvs_ref[i]);
        }
        dav1d_picture_unref_internal(&f->cur);
        dav1d_thread_picture_unref(&f->sr_cur);
        dav1d_ref_dec(&f->mvs_ref);
        dav1d_ref_dec(&f->seq_hdr_ref);
        dav1d_ref_dec(&f->frame_hdr_ref);
    } else if (c->n_fc == 1) {
        if ((res = dav1d_decode_frame(f)) < 0) {
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
#if 0
    } else {
        dav1d_task_frame_init(f);
        pthread_mutex_unlock(&c->task_thread.lock);
#endif
    }

    return 0;
error:
    atomic_init(&f->task_thread.error, 1);
    dav1d_cdf_thread_unref(&f->in_cdf);
    if (f->use_pri_sec_cdf) {
        dav1d_cdf_thread_unref(&f->src_cdf[0]);
        dav1d_cdf_thread_unref(&f->src_cdf[1]);
    }
    if (f->frame_hdr->refresh_context)
        dav1d_cdf_thread_unref(&f->out_cdf);
    for (int i = 0; i < 7; i++) {
        if (f->refp[i].p.frame_hdr)
            dav1d_thread_picture_unref(&f->refp[i]);
        dav1d_ref_dec(&f->ref_mvs_ref[i]);
    }
    if (q) q->res = res;
    dav1d_picture_unref_internal(&f->cur);
    dav1d_thread_picture_unref(&f->sr_cur);
    dav1d_ref_dec(&f->mvs_ref);
    dav1d_ref_dec(&f->seq_hdr_ref);
    dav1d_ref_dec(&f->frame_hdr_ref);
#if 0
    dav1d_data_props_copy(&c->cached_error_props, &c->in.m);
#endif

    for (int i = 0; i < f->n_tile_data; i++)
        dav1d_data_unref_internal(&f->tile[i].data);
    f->n_tile_data = 0;

    if (c->n_fc > 1)
        pthread_mutex_unlock(&c->task_thread.lock);

    return res;
}
