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

#include "src/cdef_apply.h"

enum Backup2x8Flags {
    BACKUP_2X8_Y = 1 << 0,
    BACKUP_2X8_UV = 1 << 1,
};

static void backup2lines(pixel *const dst[3], /*const*/ pixel *const src[3],
                         const ptrdiff_t stride[2],
                         const enum Dav2dPixelLayout layout)
{
    const ptrdiff_t y_stride = PXSTRIDE(stride[0]);
    if (y_stride < 0)
        pixel_copy(dst[0] + y_stride, src[0] + 7 * y_stride, -2 * y_stride);
    else
        pixel_copy(dst[0], src[0] + 6 * y_stride, 2 * y_stride);

    if (layout != DAV2D_PIXEL_LAYOUT_I400) {
        const ptrdiff_t uv_stride = PXSTRIDE(stride[1]);
        if (uv_stride < 0) {
            const int uv_off = layout == DAV2D_PIXEL_LAYOUT_I420 ? 3 : 7;
            pixel_copy(dst[1] + uv_stride, src[1] + uv_off * uv_stride, -2 * uv_stride);
            pixel_copy(dst[2] + uv_stride, src[2] + uv_off * uv_stride, -2 * uv_stride);
        } else {
            const int uv_off = layout == DAV2D_PIXEL_LAYOUT_I420 ? 2 : 6;
            pixel_copy(dst[1], src[1] + uv_off * uv_stride, 2 * uv_stride);
            pixel_copy(dst[2], src[2] + uv_off * uv_stride, 2 * uv_stride);
        }
    }
}

static void backup2x8(pixel dst[3][8][2],
                      /*const*/ pixel *const src[3],
                      const ptrdiff_t src_stride[2], int x_off,
                      const enum Dav2dPixelLayout layout,
                      const enum Backup2x8Flags flag)
{
    ptrdiff_t y_off = 0;
    if (flag & BACKUP_2X8_Y) {
        for (int y = 0; y < 8; y++, y_off += PXSTRIDE(src_stride[0]))
            pixel_copy(dst[0][y], &src[0][y_off + x_off - 2], 2);
    }

    if (layout == DAV2D_PIXEL_LAYOUT_I400 || !(flag & BACKUP_2X8_UV))
        return;

    const int ss_ver = layout == DAV2D_PIXEL_LAYOUT_I420;
    const int ss_hor = layout != DAV2D_PIXEL_LAYOUT_I444;

    x_off >>= ss_hor;
    y_off = 0;
    for (int y = 0; y < (8 >> ss_ver); y++, y_off += PXSTRIDE(src_stride[1])) {
        pixel_copy(dst[1][y], &src[1][y_off + x_off - 2], 2);
        pixel_copy(dst[2][y], &src[2][y_off + x_off - 2], 2);
    }
}

static int adjust_strength(const int strength, const unsigned var) {
    if (!var) return 0;
    const int i = var >> 6 ? imin(ulog2(var >> 6), 12) : 0;
    return (strength * (4 + i) + 8) >> 4;
}

void bytefn(dav2d_cdef_brow)(Dav2dTaskContext *const tc,
                             pixel *const p[3],
                             const Av2Filter *const lflvl,
                             const int by_start, const int by_end,
                             const int sbrow_start, const int sby)
{
    Dav2dFrameContext *const f = (Dav2dFrameContext *)tc->f;
    const int bitdepth_min_8 = BITDEPTH == 8 ? 0 : f->cur.p.p.bpc - 8;
    const Dav2dDSPContext *const dsp = f->dsp;
    enum CdefEdgeFlags edges = CDEF_HAVE_BOTTOM | (by_start > 0 ? CDEF_HAVE_TOP : 0);
    pixel *ptrs[3] = { p[0], p[1], p[2] };
    const int sbsz = 16;
    const int sb64w = (f->bw + sbsz - 1) >> 4;
    const int damping = f->frame_hdr->cdef.damping + bitdepth_min_8;
    const int on_skip_tx = f->frame_hdr->cdef.on_skiptx;
    const enum Dav2dPixelLayout layout = f->cur.p.p.layout;
    const int uv_idx = DAV2D_PIXEL_LAYOUT_I444 - layout;
    const int ss_hor = layout != DAV2D_PIXEL_LAYOUT_I444;
    const int ss_ver = layout == DAV2D_PIXEL_LAYOUT_I420;
    static const uint8_t uv_dirs[2][8] = { { 0, 1, 2, 3, 4, 5, 6, 7 },
                                           { 7, 0, 2, 4, 5, 6, 6, 6 } };
    const uint8_t *uv_dir = uv_dirs[layout == DAV2D_PIXEL_LAYOUT_I422];
    const int have_tt = f->c->n_tc > 1;
    const int sb128 = f->frame_hdr->sb128;
    const ptrdiff_t y_stride = PXSTRIDE(f->cur.p.stride[0]);
    const ptrdiff_t uv_stride = PXSTRIDE(f->cur.p.stride[1]);

    for (int by = by_start; by < by_end; by += 2, edges |= CDEF_HAVE_TOP) {
        const int tf = tc->top_pre_cdef_toggle;
        const int by_idx = (by & 0x3e) >> 1;
        if (by + 2 >= f->bh) edges &= ~CDEF_HAVE_BOTTOM;

        if ((!have_tt || sbrow_start || by + 2 < by_end) &&
            edges & CDEF_HAVE_BOTTOM)
        {
            // backup pre-filter data for next iteration
            pixel *const cdef_top_bak[3] = {
                f->lf.cdef_line[!tf][0] + have_tt * sby * 4 * y_stride,
                f->lf.cdef_line[!tf][1] + have_tt * sby * 8 * uv_stride,
                f->lf.cdef_line[!tf][2] + have_tt * sby * 8 * uv_stride
            };
            backup2lines(cdef_top_bak, ptrs, f->cur.p.stride, layout);
        }

        ALIGN_STK_16(pixel, lr_bak, 2 /* idx */, [3 /* plane */][8 /* y */][2 /* x */]);
        pixel *iptrs[3] = { ptrs[0], ptrs[1], ptrs[2] };
        edges &= ~CDEF_HAVE_LEFT;
        edges |= CDEF_HAVE_RIGHT;
        enum Backup2x8Flags prev_flag = 0;
        for (int sbx = 0, bit = 0; sbx < sb64w; sbx++, edges |= CDEF_HAVE_LEFT) {
            ALIGN_STK_64(uint8_t, ccso_lut_idx, 3, [64*8]);
            const int sb256x = sbx >> 2;
            const int sb64x_idx = sbx & 3;
            const int sb64_idx = ((by & 0x30) >> 2) + sb64x_idx;
            const int cdef_idx = lflvl[sb256x].cdef_idx[sb64_idx];

            if (f->c->inloop_filters & DAV2D_INLOOPFILTER_CCSO) {
                const enum Backup2x8Flags flag = lflvl[sb256x].ccso[0] |
                    lflvl[sb256x].ccso[1] | lflvl[sb256x].ccso[2];
                const enum Backup2x8Flags do_left = flag & ~prev_flag;
                prev_flag |= flag;
                if (do_left && edges & CDEF_HAVE_LEFT) {
                    // we didn't backup the prefilter data because we didn't
                    // filter it, so do it here instead
                    backup2x8(lr_bak[bit], iptrs, f->cur.p.stride, 0, layout, do_left);
                }

                for (int pl = 0; pl < 3; pl++) {
                    if (!lflvl[sb256x].ccso[pl]) continue;

                    const Dav2dFrameHeader *const hdr = f->frame_hdr;
                    const unsigned max_band = hdr->ccso.p[pl].max_band_log2;
                    const unsigned ext_filter = hdr->ccso.p[pl].ext_filter_support;
                    const unsigned scale_idx = hdr->ccso.p[pl].scale_idx;
                    const unsigned quant =
                        dav2d_ccso_quant_sz[scale_idx][hdr->ccso.p[pl].quant_idx];
                    const int edge_clf = hdr->ccso.p[pl].edge_clf;
                    const int bo_only = hdr->ccso.p[pl].bo_only;

                    const pixel *top, *bot;
                    ptrdiff_t offset;
                    enum CdefEdgeFlags sb_edges = edges;
                    if ((sbx + 1) * sbsz >= f->bw) sb_edges &= ~CDEF_HAVE_RIGHT;
                    const int w = imin(sbsz, f->bw - sbx * sbsz) * 4;
                    assert(w >= 0);

                    if (!have_tt) goto sb_st_y;
                    if (sbrow_start && by == by_start) {
                        offset = (sby * (4 << sb128) - 4) * y_stride +
                                 sbx * sbsz * 4;
                        top = &f->lf.lr_db_line[0][offset];
                        bot = iptrs[0] + 8 * y_stride;
                    } else if (!sbrow_start && by + 2 >= by_end) {
                        offset = sby * 4 * y_stride + sbx * sbsz * 4;
                        top = &f->lf.cdef_line[tf][0][offset];
                        offset = ((sby + 1) * (4 << sb128) - 2) * y_stride +
                                 sbx * sbsz * 4;
                        bot = &f->lf.lr_db_line[0][offset];
                    } else {
                sb_st_y:
                        offset = have_tt * sby * 4 * y_stride + sbx * sbsz * 4;
                        top = &f->lf.cdef_line[tf][0][offset];
                        bot = iptrs[0] + 8 * y_stride;
                    }

                    dsp->ccso.prep[!!pl * uv_idx](ccso_lut_idx[pl], 64 >> (!!pl * ss_hor),
                                                  iptrs[0], f->cur.p.stride[0],
                                                  lr_bak[bit][0], top, bot,
                                                  max_band, ext_filter, quant,
                                                  edge_clf, bo_only,
                                                  w >> (!!pl * ss_hor), 8 >> (!!pl * ss_ver),
                                                  sb_edges HIGHBD_CALL_SUFFIX);
                }
            }
            if (cdef_idx == -1 ||
                (!f->frame_hdr->cdef.y_strength[cdef_idx] &&
                 !f->frame_hdr->cdef.uv_strength[cdef_idx]) ||
                 !(f->c->inloop_filters & DAV2D_INLOOPFILTER_CDEF))
            {
                prev_flag = 0;
                goto next_sb;
            }

            const unsigned noskip_mask = on_skip_tx ? ~0U :
                lflvl[sb256x].noskip_mask[by_idx][sb64x_idx];

            const int y_lvl = f->frame_hdr->cdef.y_strength[cdef_idx];
            const int uv_lvl = f->frame_hdr->cdef.uv_strength[cdef_idx];
            const enum Backup2x8Flags flag = !!y_lvl + (!!uv_lvl << 1);

            const int y_pri_lvl = (y_lvl >> 2) << bitdepth_min_8;
            int y_sec_lvl = y_lvl & 3;
            y_sec_lvl += y_sec_lvl == 3;
            y_sec_lvl <<= bitdepth_min_8;

            const int uv_pri_lvl = (uv_lvl >> 2) << bitdepth_min_8;
            int uv_sec_lvl = uv_lvl & 3;
            uv_sec_lvl += uv_sec_lvl == 3;
            uv_sec_lvl <<= bitdepth_min_8;

            const uint16_t (*y_ll_mask)[4], (*uv_ll_mask)[4];
            if (f->frame_hdr->any_lossless /* segmentation + at least 1 lossless */) {
                y_ll_mask = (const uint16_t(*)[4])
                    &lflvl[sb256x].lossless_mask_y[2 * by_idx][sb64x_idx];
                uv_ll_mask = (const uint16_t(*)[4])
                    &lflvl[sb256x].lossless_mask_uv[2 * by_idx >> ss_ver][sb64x_idx];
            } else {
                static const uint16_t zero_ll_mask[2][4] = { { 0 } };
                assert(!f->frame_hdr->all_lossless);
                y_ll_mask = uv_ll_mask = zero_ll_mask;
            }

            pixel *bptrs[3] = { iptrs[0], iptrs[1], iptrs[2] };
            for (int bx = sbx * sbsz; bx < imin((sbx + 1) * sbsz, f->bw);
                 bx += 2, edges |= CDEF_HAVE_LEFT)
            {
                if (bx + 2 >= f->bw) edges &= ~CDEF_HAVE_RIGHT;

                // check if this 8x8 block had any coded coefficients; if not,
                // go to the next block
                const unsigned bx_mask = 3 << (bx & 14);
                const int y_lossless = (y_ll_mask[0][0] | y_ll_mask[1][0]) & bx_mask;
                const unsigned uvbx_mask = (3 >> ss_hor) << ((bx & 14) >> ss_hor);
                const int uv_lossless =
                    (uv_ll_mask[0][0] | uv_ll_mask[!ss_ver][0]) & uvbx_mask;
                if (!(noskip_mask & bx_mask) || (y_lossless && uv_lossless)) {
                    prev_flag = 0;
                    goto next_b;
                }
                const enum Backup2x8Flags do_left = flag & ~prev_flag;
                prev_flag = flag;
                if (do_left && edges & CDEF_HAVE_LEFT) {
                    // we didn't backup the prefilter data because it wasn't
                    // there, so do it here instead
                    backup2x8(lr_bak[bit], bptrs, f->cur.p.stride, 0, layout, do_left);
                }
                if (edges & CDEF_HAVE_RIGHT) {
                    // backup pre-filter data for next iteration
                    backup2x8(lr_bak[!bit], bptrs, f->cur.p.stride, 8, layout, flag);
                }

                int dir;
                unsigned variance;
                if (y_pri_lvl || uv_pri_lvl)
                    dir = dsp->cdef.dir(bptrs[0], f->cur.p.stride[0],
                                        &variance HIGHBD_CALL_SUFFIX);

                const pixel *top, *bot;
                ptrdiff_t offset;

                if (!have_tt) goto st_y;
                if (sbrow_start && by == by_start) {
                    offset = (sby * (4 << sb128) - 4) * y_stride + bx * 4;
                    top = &f->lf.lr_db_line[0][offset];
                    bot = bptrs[0] + 8 * y_stride;
                } else if (!sbrow_start && by + 2 >= by_end) {
                    top = &f->lf.cdef_line[tf][0][sby * 4 * y_stride + bx * 4];
                    const int line = (sby + 1) * (4 << sb128) - 2;
                    offset = line * y_stride + bx * 4;
                    bot = &f->lf.lr_db_line[0][offset];
                } else {
            st_y:;
                    offset = sby * 4 * y_stride;
                    top = &f->lf.cdef_line[tf][0][have_tt * offset + bx * 4];
                    bot = bptrs[0] + 8 * y_stride;
                }
                if (y_pri_lvl) {
                    const int adj_y_pri_lvl = adjust_strength(y_pri_lvl, variance);
                    if ((adj_y_pri_lvl || y_sec_lvl) && !y_lossless)
                        dsp->cdef.fb[0](bptrs[0], f->cur.p.stride[0], lr_bak[bit][0],
                                        top, bot, adj_y_pri_lvl, y_sec_lvl,
                                        dir, damping, edges HIGHBD_CALL_SUFFIX);
                } else if (y_sec_lvl && !y_lossless)
                    dsp->cdef.fb[0](bptrs[0], f->cur.p.stride[0], lr_bak[bit][0],
                                    top, bot, 0, y_sec_lvl, 0, damping,
                                    edges HIGHBD_CALL_SUFFIX);

                if (!uv_lvl || uv_lossless) goto skip_uv;
                assert(layout != DAV2D_PIXEL_LAYOUT_I400);

                const int uvdir = uv_pri_lvl ? uv_dir[dir] : 0;
                for (int pl = 1; pl <= 2; pl++) {
                    if (!have_tt) goto st_uv;
                    if (sbrow_start && by == by_start) {
                        const int line = sby * (4 << sb128) - 4;
                        offset = line * uv_stride + (bx * 4 >> ss_hor);
                        top = &f->lf.lr_db_line[pl][offset];
                        bot = bptrs[pl] + (8 >> ss_ver) * uv_stride;
                    } else if (!sbrow_start && by + 2 >= by_end) {
                        const ptrdiff_t top_offset = sby * 8 * uv_stride +
                                                     (bx * 4 >> ss_hor);
                        top = &f->lf.cdef_line[tf][pl][top_offset];
                        const int line = (sby + 1) * (4 << sb128) - 2;
                        offset = line * uv_stride + (bx * 4 >> ss_hor);
                        bot = &f->lf.lr_db_line[pl][offset];
                    } else {
                st_uv:;
                        const ptrdiff_t offset = sby * 8 * uv_stride;
                        top = &f->lf.cdef_line[tf][pl][have_tt * offset + (bx * 4 >> ss_hor)];
                        bot = bptrs[pl] + (8 >> ss_ver) * uv_stride;
                    }
                    dsp->cdef.fb[uv_idx](bptrs[pl], f->cur.p.stride[1],
                                         lr_bak[bit][pl], top, bot,
                                         uv_pri_lvl, uv_sec_lvl, uvdir,
                                         damping - 1, edges HIGHBD_CALL_SUFFIX);
                }

            skip_uv:
                bit ^= 1;

            next_b:
                bptrs[0] += 8;
                bptrs[1] += 8 >> ss_hor;
                bptrs[2] += 8 >> ss_hor;
            }

        next_sb:
            if (f->c->inloop_filters & DAV2D_INLOOPFILTER_CCSO) {
                const enum Backup2x8Flags flag = lflvl[sb256x].ccso[0] |
                    ((lflvl[sb256x].ccso[1] | lflvl[sb256x].ccso[2]) << 1);
                // XXX could improve the (actual) backups prior to running filter
                // (so the right side ones) by checking whether next sb/b needs
                // it (if has_cdef or has_ccso)
                const enum Backup2x8Flags do_right = flag & ~prev_flag;
                if (do_right && (sbx + 1) * sbsz < f->bw) {
                    backup2x8(lr_bak[bit], iptrs, f->cur.p.stride, sbsz * 4,
                              layout, do_right);
                    prev_flag |= do_right;
                }
                for (int pl = 0; pl < 3; pl++)
                    if (lflvl[sb256x].ccso[pl]) {
                        const int w = imin(sbsz, f->bw - sbx * sbsz) * 4;
                        dsp->ccso.add(iptrs[pl], f->cur.p.stride[!!pl],
                                      ccso_lut_idx[pl], 64 >> (!!pl * ss_hor),
                                      f->frame_hdr->ccso.p[pl].filter_off,
                                      dav2d_ccso_offset[f->frame_hdr->ccso.p[pl].scale_idx],
                                      w >> (!!pl * ss_hor), 8 >> (!!pl * ss_ver)
                                      HIGHBD_CALL_SUFFIX);
                    }
            }
            iptrs[0] += sbsz * 4;
            iptrs[1] += sbsz * 4 >> ss_hor;
            iptrs[2] += sbsz * 4 >> ss_hor;
        }

        ptrs[0] += 8 * PXSTRIDE(f->cur.p.stride[0]);
        ptrs[1] += 8 * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver;
        ptrs[2] += 8 * PXSTRIDE(f->cur.p.stride[1]) >> ss_ver;
        tc->top_pre_cdef_toggle ^= 1;
    }
}
