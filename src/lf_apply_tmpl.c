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

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "common/intops.h"

#include "src/lf_apply.h"
#include "src/lr_apply.h"

// The loop filter buffer stores 12 rows of pixels. A superblock block will
// contain at most 2 stripes. Each stripe requires 4 rows pixels (2 above
// and 2 below) the final 4 rows are used to swap the bottom of the last
// stripe with the top of the next super block row.
static void backup_lpf(const Dav1dFrameContext *const f,
                       pixel *dst, const ptrdiff_t dst_stride,
                       const pixel *src, const ptrdiff_t src_stride,
                       const int ss_ver, const int sb128,
                       int row, const int row_h, const int w,
                       const int h, const int ss_hor, const int lr_backup)
{
    const int cdef_backup = !lr_backup;

    // The first stripe of the frame is shorter by 8 luma pixel rows.
    int stripe_h = ((64 << (cdef_backup & sb128)) - 8 * !row) >> ss_ver;
    src += (stripe_h - 2) * PXSTRIDE(src_stride);

    if (f->c->n_tc == 1) {
        if (row) {
            const int top = 4 << sb128;
            // Copy the top part of the stored loop filtered pixels from the
            // previous sb row needed above the first stripe of this sb row.
            pixel_copy(&dst[PXSTRIDE(dst_stride) *  0],
                       &dst[PXSTRIDE(dst_stride) *  top],      w);
            pixel_copy(&dst[PXSTRIDE(dst_stride) *  1],
                       &dst[PXSTRIDE(dst_stride) * (top + 1)], w);
            pixel_copy(&dst[PXSTRIDE(dst_stride) *  2],
                       &dst[PXSTRIDE(dst_stride) * (top + 2)], w);
            pixel_copy(&dst[PXSTRIDE(dst_stride) *  3],
                       &dst[PXSTRIDE(dst_stride) * (top + 3)], w);
        }
        dst += 4 * PXSTRIDE(dst_stride);
    }

    while (row + stripe_h <= row_h) {
        const int n_lines = 4 - (row + stripe_h + 1 == h);
        for (int i = 0; i < 4; i++) {
            pixel_copy(dst, i == n_lines ? &dst[-PXSTRIDE(dst_stride)] :
                                           src, w);
            dst += PXSTRIDE(dst_stride);
            src += PXSTRIDE(src_stride);
        }
        row += stripe_h; // unmodified stripe_h for the 1st stripe
        stripe_h = 64 >> ss_ver;
        src += (stripe_h - 4) * PXSTRIDE(src_stride);
    }
}

void bytefn(dav1d_copy_lpf)(Dav1dFrameContext *const f,
                            /*const*/ pixel *const src[3], const int sby)
{
    const int have_tt = f->c->n_tc > 1;
    const int offset = 8 * !!sby;
    const ptrdiff_t *const src_stride = f->cur.stride;
    const ptrdiff_t *const lr_stride = f->sr_cur.p.stride;
    const int tt_off = have_tt * sby * (4 << f->frame_hdr->sb128);
    pixel *const dst[3] = {
        f->lf.lr_lpf_line[0] + tt_off * PXSTRIDE(lr_stride[0]),
        f->lf.lr_lpf_line[1] + tt_off * PXSTRIDE(lr_stride[1]),
        f->lf.lr_lpf_line[2] + tt_off * PXSTRIDE(lr_stride[1])
    };

    // TODO Also check block level restore type to reduce copying.
    const int restore_planes = f->lf.restore_planes;

    if (f->seq_hdr->cdef || restore_planes & LR_RESTORE_Y) {
        const int h = f->cur.p.h;
        const int w = f->bw << 2;
        const int row_h = imin((sby + 1) << (6 + f->frame_hdr->sb128), h - 1);
        const int y_stripe = (sby << (6 + f->frame_hdr->sb128)) - offset;
        backup_lpf(f, dst[0], lr_stride[0],
                   src[0] - offset * PXSTRIDE(src_stride[0]), src_stride[0],
                   0, f->frame_hdr->sb128, y_stripe, row_h, w, h, 0, 1);
    }
    if ((f->seq_hdr->cdef || restore_planes & (LR_RESTORE_U | LR_RESTORE_V)) &&
        f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400)
    {
        const int ss_ver = f->sr_cur.p.p.layout == DAV1D_PIXEL_LAYOUT_I420;
        const int ss_hor = f->sr_cur.p.p.layout != DAV1D_PIXEL_LAYOUT_I444;
        const int h = (f->cur.p.h + ss_ver) >> ss_ver;
        const int w = f->bw << (2 - ss_hor);
        const int row_h = imin((sby + 1) << ((6 - ss_ver) + f->frame_hdr->sb128), h - 1);
        const int offset_uv = offset >> ss_ver;
        const int y_stripe = (sby << ((6 - ss_ver) + f->frame_hdr->sb128)) - offset_uv;
        if (f->seq_hdr->cdef || restore_planes & LR_RESTORE_U) {
            backup_lpf(f, dst[1], lr_stride[1],
                       src[1] - offset_uv * PXSTRIDE(src_stride[1]),
                       src_stride[1], ss_ver, f->frame_hdr->sb128, y_stripe,
                       row_h, w, h, ss_hor, 1);
        }
        if (f->seq_hdr->cdef || restore_planes & LR_RESTORE_V) {
            backup_lpf(f, dst[2], lr_stride[1],
                       src[2] - offset_uv * PXSTRIDE(src_stride[1]),
                       src_stride[1], ss_ver, f->frame_hdr->sb128, y_stripe,
                       row_h, w, h, ss_hor, 1);
        }
    }
}

static inline void filter_plane_cols_y(const Dav1dFrameContext *const f,
                                       const int have_left,
                                       const uint16_t (*const mask)[4][4],
                                       pixel *dst, const ptrdiff_t ls,
                                       const int w,
                                       const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;
    const int starty64 = starty4 >> 4;
    const int endy64 = (endy4 + 15) >> 4;

    const unsigned q_thr = f->lf.thr_lut.thr[0][0][0];
    const unsigned side_thr = f->lf.thr_lut.thr[0][1][0];

    // filter edges between columns (e.g. block1 | block2)
    for (int x = 0; x < w; x++) {
        if (!have_left && !x) continue;
        uint64_t hmask[4] = { 0 };
        for (int y = starty64, shift = 0; y < endy64; y++, shift += 16) {
            hmask[0] |= (uint64_t) mask[x][0][y] << shift;
            hmask[1] |= (uint64_t) mask[x][1][y] << shift;
            hmask[2] |= (uint64_t) mask[x][2][y] << shift;
            hmask[3] |= (uint64_t) mask[x][3][y] << shift;
        }
        // TODO: Fix strength at end of tile columns
        dsp->lf.loop_filter_sb[0][0](&dst[x * 4], ls, hmask, q_thr, side_thr, 0,
                                     &f->lf.thr_lut, endy4 - starty4 HIGHBD_CALL_SUFFIX);
    }
}

static inline void filter_plane_rows_y(const Dav1dFrameContext *const f,
                                       const int have_top,
                                       const uint16_t (*const mask)[4][4],
                                       pixel *dst, const ptrdiff_t ls,
                                       const int w,
                                       const int starty4, const int endy4)
{
    const Dav1dDSPContext *const dsp = f->dsp;

    const unsigned q_thr = f->lf.thr_lut.thr[1][0][0];
    const unsigned side_thr = f->lf.thr_lut.thr[1][1][0];
    //                                 block1
    // filter edges between rows (e.g. ------)
    //                                 block2
    for (int y = starty4; y < endy4; y++, dst += 4 * PXSTRIDE(ls)) {
        if (!have_top && !y) continue;
        const uint64_t vmask[4] = {
            mask[y][0][0] | (uint64_t) mask[y][0][1] << 16 | (uint64_t) mask[y][0][2] << 32 | (uint64_t) mask[y][0][3] << 48,
            mask[y][1][0] | (uint64_t) mask[y][1][1] << 16 | (uint64_t) mask[y][1][2] << 32 | (uint64_t) mask[y][1][3] << 48,
            mask[y][2][0] | (uint64_t) mask[y][2][1] << 16 | (uint64_t) mask[y][2][2] << 32 | (uint64_t) mask[y][2][3] << 48,
            mask[y][3][0] | (uint64_t) mask[y][3][1] << 16 | (uint64_t) mask[y][3][2] << 32 | (uint64_t) mask[y][3][3] << 48,
        };
        dsp->lf.loop_filter_sb[0][1](dst, ls, vmask, q_thr, side_thr, (y & 15) == 0,
                                     &f->lf.thr_lut, w HIGHBD_CALL_SUFFIX);
    }
}

static inline void filter_plane_cols_uv(const Dav1dFrameContext *const f,
                                        const int have_left,
                                        const uint16_t (*const mask)[2][4],
                                        pixel *const u, pixel *const v,
                                        const ptrdiff_t ls, const int w,
                                        const int starty4, const int endy4,
                                        const int ss_ver)
{
#if 0
    const Dav1dDSPContext *const dsp = f->dsp;

    // filter edges between columns (e.g. block1 | block2)
    for (int x = 0; x < w; x++) {
        if (!have_left && !x) continue;
        uint32_t hmask[3];
        if (!starty4) {
            hmask[0] = mask[x][0][0];
            hmask[1] = mask[x][1][0];
            if (endy4 > (16 >> ss_ver)) {
                hmask[0] |= (unsigned) mask[x][0][1] << (16 >> ss_ver);
                hmask[1] |= (unsigned) mask[x][1][1] << (16 >> ss_ver);
            }
        } else {
            hmask[0] = mask[x][0][1];
            hmask[1] = mask[x][1][1];
        }
        hmask[2] = 0;
        dsp->lf.loop_filter_sb[1][0](&u[x * 4], ls, hmask,
                                     (const uint8_t(*)[4]) &lvl[x][2], b4_stride,
                                     &f->lf.lim_lut, endy4 - starty4 HIGHBD_CALL_SUFFIX);
        dsp->lf.loop_filter_sb[1][0](&v[x * 4], ls, hmask,
                                     (const uint8_t(*)[4]) &lvl[x][3], b4_stride,
                                     &f->lf.lim_lut, endy4 - starty4 HIGHBD_CALL_SUFFIX);
    }
#endif
}

static inline void filter_plane_rows_uv(const Dav1dFrameContext *const f,
                                        const int have_top,
                                        const uint16_t (*const mask)[2][4],
                                        pixel *const u, pixel *const v,
                                        const ptrdiff_t ls, const int w,
                                        const int starty4, const int endy4,
                                        const int ss_hor)
{
#if 0
    const Dav1dDSPContext *const dsp = f->dsp;
    ptrdiff_t off_l = 0;

    //                                 block1
    // filter edges between rows (e.g. ------)
    //                                 block2
    for (int y = starty4; y < endy4;
         y++, off_l += 4 * PXSTRIDE(ls), lvl += b4_stride)
    {
        if (!have_top && !y) continue;
        const uint32_t vmask[3] = {
            mask[y][0][0] | ((unsigned) mask[y][0][1] << (16 >> ss_hor)),
            mask[y][1][0] | ((unsigned) mask[y][1][1] << (16 >> ss_hor)),
            0,
        };
        dsp->lf.loop_filter_sb[1][1](&u[off_l], ls, vmask,
                                     (const uint8_t(*)[4]) &lvl[0][2], b4_stride,
                                     &f->lf.lim_lut, w HIGHBD_CALL_SUFFIX);
        dsp->lf.loop_filter_sb[1][1](&v[off_l], ls, vmask,
                                     (const uint8_t(*)[4]) &lvl[0][3], b4_stride,
                                     &f->lf.lim_lut, w HIGHBD_CALL_SUFFIX);
    }
#endif
}

void bytefn(dav1d_loopfilter_sbrow_cols)(const Dav1dFrameContext *const f,
                                         pixel *const p[3], Av1Filter *const lflvl,
                                         int sby)
{
    int x, have_left;
    // Don't filter outside the frame
    const int sbsz = f->sb_step;
    const int starty4 = (sby * sbsz) & 0x30;
#if 0
    const int sb128 = f->frame_hdr->sb128;
    const int sbl2 = 4 + sb128;
    const int halign = (f->bh + 31) & ~31;
#endif
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
#if 0
    const int vmask = 16 >> ss_ver, hmask = 16 >> ss_hor;
    const unsigned vmax = 1U << vmask, hmax = 1U << hmask;
#endif
    const unsigned endy4 = starty4 + imin(f->h4 - sby * sbsz, sbsz);
    const unsigned uv_endy4 = (endy4 + ss_ver) >> ss_ver;

    if (f->frame_hdr->segmentation.enabled) {
        fprintf(stderr, "segmentation not supported for deblocking\n");
    }

#if 0
    // fix lpf strength at tile col boundaries
    const uint8_t *lpf_y = &f->lf.tx_lpf_right_edge[0][sby << sbl2];
    const uint8_t *lpf_uv = &f->lf.tx_lpf_right_edge[1][sby << (sbl2 - ss_ver)];
    for (int tile_col = 1;; tile_col++) {
        x = f->frame_hdr->tiling.t.col_start_sb[tile_col];
        if ((x << sbl2) >= f->bw) break;
        const int bx4 = x & is_sb64 ? 16 : 0, cbx4 = bx4 >> ss_hor;
        x >>= is_sb64;

        uint16_t (*const y_hmask)[4] = lflvl[x].filter_y[0][bx4];
        for (unsigned y = starty4, mask = 1 << y; y < endy4; y++, mask <<= 1) {
            const int sidx = mask >= 0x10000U;
            const unsigned smask = mask >> (sidx << 4);
            const int idx = 2 * !!(y_hmask[2][sidx] & smask) +
                                !!(y_hmask[1][sidx] & smask);
            y_hmask[2][sidx] &= ~smask;
            y_hmask[1][sidx] &= ~smask;
            y_hmask[0][sidx] &= ~smask;
            y_hmask[imin(idx, lpf_y[y - starty4])][sidx] |= smask;
        }

        if (f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400) {
            uint16_t (*const uv_hmask)[4] = lflvl[x].filter_uv[0][cbx4];
            for (unsigned y = starty4 >> ss_ver, uv_mask = 1 << y; y < uv_endy4;
                 y++, uv_mask <<= 1)
            {
                const int sidx = uv_mask >= vmax;
                const unsigned smask = uv_mask >> (sidx << (4 - ss_ver));
                const int idx = !!(uv_hmask[1][sidx] & smask);
                uv_hmask[1][sidx] &= ~smask;
                uv_hmask[0][sidx] &= ~smask;
                uv_hmask[imin(idx, lpf_uv[y - (starty4 >> ss_ver)])][sidx] |= smask;
            }
        }
        lpf_y  += halign;
        lpf_uv += halign >> ss_ver;
    }

    // fix lpf strength at tile row boundaries
    if (start_of_tile_row) {
        const BlockContext *a;
        for (x = 0, a = &f->a[f->sb256w * (start_of_tile_row - 1)];
             x < f->sb256w; x++, a++)
        {
            uint16_t (*const y_vmask)[4] = lflvl[x].filter_y[1][starty4];
            const unsigned w = imin(32, f->w4 - (x << 5));
            for (unsigned mask = 1, i = 0; i < w; mask <<= 1, i++) {
                const int sidx = mask >= 0x10000U;
                const unsigned smask = mask >> (sidx << 4);
                const int idx = 2 * !!(y_vmask[2][sidx] & smask) +
                                    !!(y_vmask[1][sidx] & smask);
                y_vmask[2][sidx] &= ~smask;
                y_vmask[1][sidx] &= ~smask;
                y_vmask[0][sidx] &= ~smask;
                y_vmask[imin(idx, a->tx_lpf_y[i])][sidx] |= smask;
            }

            if (f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I400) {
                const unsigned cw = (w + ss_hor) >> ss_hor;
                uint16_t (*const uv_vmask)[4] = lflvl[x].filter_uv[1][starty4 >> ss_ver];
                for (unsigned uv_mask = 1, i = 0; i < cw; uv_mask <<= 1, i++) {
                    const int sidx = uv_mask >= hmax;
                    const unsigned smask = uv_mask >> (sidx << (4 - ss_hor));
                    const int idx = !!(uv_vmask[1][sidx] & smask);
                    uv_vmask[1][sidx] &= ~smask;
                    uv_vmask[0][sidx] &= ~smask;
                    uv_vmask[imin(idx, a->tx_lpf_uv[i])][sidx] |= smask;
                }
            }
        }
    }
#endif

    if (f->frame_hdr->loopfilter.level_y[0]) {
        pixel *ptr;
        for (ptr = p[0], have_left = 0, x = 0; x < f->sb256w; x++, have_left = 1, ptr += 256) {
            filter_plane_cols_y(f, have_left, lflvl[x].filter_y[0], ptr, f->cur.stride[0],
                                imin(64, f->w4 - x * 64), starty4, endy4);
        }
    }

    if (!f->frame_hdr->loopfilter.level_u && !f->frame_hdr->loopfilter.level_v)
        return;

    ptrdiff_t uv_off;
    for (uv_off = 0, have_left = 0, x = 0; x < f->sb256w;
         x++, have_left = 1, uv_off += 256 >> ss_hor)
    {
        filter_plane_cols_uv(f, have_left, lflvl[x].filter_uv[0],
                             &p[1][uv_off], &p[2][uv_off], f->cur.stride[1],
                             (imin(64, f->w4 - x * 64) + ss_hor) >> ss_hor,
                             starty4 >> ss_ver, uv_endy4, ss_ver);
    }
}

void bytefn(dav1d_loopfilter_sbrow_rows)(const Dav1dFrameContext *const f,
                                         pixel *const p[3], Av1Filter *const lflvl,
                                         int sby)
{
    int x;
    // Don't filter outside the frame
    const int have_top = sby > 0;
    const int sbsz = 64 >> (2 - f->frame_hdr->sb128);
    const int starty4 = (sby * sbsz) & 0x30;
    const int ss_ver = f->cur.p.layout == DAV1D_PIXEL_LAYOUT_I420;
    const int ss_hor = f->cur.p.layout != DAV1D_PIXEL_LAYOUT_I444;
    const unsigned endy4 = starty4 + imin(f->h4 - sby * sbsz, sbsz);
    const unsigned uv_endy4 = (endy4 + ss_ver) >> ss_ver;

    pixel *ptr;
    if (f->frame_hdr->loopfilter.level_y[1]) {
        for (ptr = p[0], x = 0; x < f->sb256w; x++, ptr += 256) {
            filter_plane_rows_y(f, have_top, lflvl[x].filter_y[1], ptr, f->cur.stride[0],
                                imin(64, f->w4 - x * 64), starty4, endy4);
        }
    }

    if (!f->frame_hdr->loopfilter.level_u && !f->frame_hdr->loopfilter.level_v)
        return;

    ptrdiff_t uv_off;
    for (uv_off = 0, x = 0; x < f->sb256w; x++, uv_off += 256 >> ss_hor) {
        filter_plane_rows_uv(f, have_top, lflvl[x].filter_uv[1],
                             &p[1][uv_off], &p[2][uv_off], f->cur.stride[1],
                             (imin(64, f->w4 - x * 64) + ss_hor) >> ss_hor,
                             starty4 >> ss_ver, uv_endy4, ss_hor);
    }
}
