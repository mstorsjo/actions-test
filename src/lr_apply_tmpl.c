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

#include <stdio.h>

#include "common/intops.h"

#include "src/lr_apply.h"

enum FirstSbInTileRow {
    FIRST_SB_NONE = 0,
    FIRST_SB_TOP,
    FIRST_SB_BOTTOM,
};

static void lr_stripe(const Dav2dFrameContext *const f, pixel *p,
                      const pixel (*left)[6], int x, int y,
                      const int plane, const int w, const int row_h,
                      const Av2RestorationUnit *const lr, enum LrEdgeFlags edges,
                      const enum FirstSbInTileRow first_sby_in_tile_row,
                      const int tile_row_m1)
{
    const Dav2dDSPContext *const dsp = f->dsp;
    const struct Dav2dNSWienerPlane *const pd = &f->frame_hdr->restoration.p[plane].ns;
    const int chroma = !!plane;
    const int ss_ver = chroma & (f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420);
    const int ss_hor = chroma & (f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444);
    const ptrdiff_t stride = f->cur.p.stride[chroma];
    const int sby = (y + (y ? 8 << ss_ver : 0)) >> (6 - ss_ver + f->frame_hdr->sb128);
    const int have_tt = f->c->n_tc > 1;
    const pixel *lpf = f->lf.lr_lpf_line[plane] +
        have_tt * (sby * (4 << f->frame_hdr->sb128) - 4) * PXSTRIDE(stride) + x;
    const pixel *top =
        ((edges & (LR_HAVE_TOP | LR_HAVE_TOP_INTEGRATED)) ==
                  (LR_HAVE_TOP | LR_HAVE_TOP_INTEGRATED)) ?
        f->lf.lr_cdef_line[plane] + tile_row_m1 * (4 >> chroma) * PXSTRIDE(stride) + x : NULL;
    const int sb256x = x >> 8;
    const int sb64x_idx = (x >> 6) & 3;

    // The first stripe of the frame is shorter by 8 luma pixel rows.
    int stripe_h = imin((64 - 8 * !!first_sby_in_tile_row) >> ss_ver, row_h - y);

    int ref_dst_idx = f->lf.gdf_ref_dst_idx;
    int qp_idx = f->frame_hdr->gdf.qp_idx;
    int gdf_scale = f->frame_hdr->gdf.scale;
    wienerfilter_fn wiener_fn = NULL;
    WienerParams wiener_params;
    uint16_t noskip_mask[64 + 2];
    int multi_wiener = 0;

    if (lr->type == DAV2D_RESTORATION_NS_WIENER) {
        if (pd->frame_filters_on) {
            if (pd->num_classes == 1) {
                wiener_fn = dsp->lr.ns_wiener_single[chroma];
                wiener_params.single.filter = pd->filter[0];
            } else {
                multi_wiener = 1;
                wiener_fn = dsp->lr.ns_wiener_multi;
                wiener_params.multi.base_q = f->lf.base_q;
                wiener_params.multi.subclass_lut = f->lf.ns_subclass_lut;
                wiener_params.multi.filters.user = pd->filter;
            }
        } else {
            wiener_fn = dsp->lr.ns_wiener_single[chroma];
            wiener_params.single.filter = lr->ns_filter[0];
        }
    } else if (lr->type == DAV2D_RESTORATION_PC_WIENER) {
        multi_wiener = 1;
        wiener_fn = dsp->lr.pc_wiener;
        wiener_params.multi.base_q = f->lf.base_q;
        wiener_params.multi.subclass_lut = f->lf.pc_subclass_lut;
        wiener_params.multi.filters.pretrained = f->lf.pc_filters;
    }

    if (multi_wiener) {
        wiener_params.multi.noskip_mask = noskip_mask;
        for (int by = y >> 2, r = 0; by < row_h >> 2; by++, r++) {
            int by_idx = by & 63;
            // TODO: add outer loop so we don't compute and offset by the same sb256_idx most iterations
            int sb256_idx = f->sb256w * (by >> 6) + sb256x;
            uint16_t* noskip_row = f->lf.mask[sb256_idx].lr_noskip_mask[by_idx];
            noskip_mask[r] = noskip_row[sb64x_idx];
            // extend masks on the right edge
            if (!(edges & LR_HAVE_RIGHT) && w & 63) {
                const int shift = ((w >> 2) & 15) - 1;
                const int mask = noskip_mask[r];
                const int edge = mask >> shift;
                noskip_mask[r] |= edge << (shift + 1);
            }
        }
    }
    const ptrdiff_t lstride = f->cur.p.stride[0];
    const pixel *llpf = f->lf.lr_lpf_line[0] +
        have_tt * (sby * (4 << f->frame_hdr->sb128) - 4) * PXSTRIDE(lstride) + x * 2;
    if (chroma) {
        wiener_params.single.ss_ver = ss_ver;
        wiener_params.single.ss_hor = ss_hor;
        wiener_params.single.stride = lstride;
        wiener_params.single.ds_flt = f->seq_hdr->cfl_ds_filter_index;
    }

    int8_t gdf_err[64*64];
    while (y + stripe_h <= row_h) {
        if (chroma) {
            wiener_params.single.luma =
                &((pixel *) f->cur.p.data[0])[(x << ss_hor) + (y << ss_ver) * PXSTRIDE(lstride)];
            wiener_params.single.luma_top = llpf;
            wiener_params.single.luma_bottom = llpf + 6 * PXSTRIDE(lstride);
        }
        // Change the HAVE_BOTTOM bit in edges to (sby + 1 != f->sbh || y + stripe_h != row_h)
        edges ^= (-(sby + 1 != f->sbh || y + stripe_h != row_h) ^ edges) & LR_HAVE_BOTTOM;
        int sb256_idx = f->sb256w * ((y + 8) >> 8) + sb256x;
        int gdf = !plane && f->lf.mask[sb256_idx].gdf[(((y + 8) >> 4) & 12) + sb64x_idx];

        if (gdf) {
            dsp->lr.gdf_prep(gdf_err, 64, p, stride, left, lpf,
                             w, stripe_h, ref_dst_idx, qp_idx, edges HIGHBD_CALL_SUFFIX);
        }
        if (wiener_fn) {
            wiener_fn(p, stride, left, top ? top : lpf,
                      lpf + 6 * PXSTRIDE(stride), w, stripe_h, &wiener_params,
                      edges HIGHBD_CALL_SUFFIX);
            if (multi_wiener)
                wiener_params.multi.noskip_mask += stripe_h >> 2;
        }
        if (gdf) {
            dsp->lr.gdf_add(p, stride, gdf_err, 64, w, stripe_h, gdf_scale
                            HIGHBD_CALL_SUFFIX);
        }
        edges &= ~(LR_HAVE_BOTTOM_INTEGRATED | LR_HAVE_TOP_INTEGRATED);
        left += stripe_h;
        y += stripe_h;
        p += stripe_h * PXSTRIDE(stride);
        edges |= LR_HAVE_TOP;
        stripe_h = imin(64 >> ss_ver, row_h - y);
        if (stripe_h == 0) break;
        lpf += 4 * PXSTRIDE(stride);
        llpf += 4 * PXSTRIDE(lstride);
        top = NULL;
    }
}

static void backupNxU(pixel (*dst)[6], const pixel *src, const ptrdiff_t src_stride,
                      int u, const int n)
{
    for (; u > 0; u--, dst++, src += PXSTRIDE(src_stride))
        pixel_copy(&dst[0][6 - n], src - n, n);
}

static void lr_sbrow(const Dav2dFrameContext *const f, pixel *p, const int y,
                     const int w, const int h, const int row_h, const int plane,
                     const enum FirstSbInTileRow first_sby_in_tile_row,
                     const int tile_row_m1)
{
    const int chroma = !!plane;
    const int ss_ver = chroma & (f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420);
    const int ss_hor = chroma & (f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444);
    const ptrdiff_t p_stride = f->cur.p.stride[chroma];

    const int unit_size_log2 = f->frame_hdr->restoration.unit_size[!!plane];
    const int unit_size = 1 << unit_size_log2;
    const int half_unit_size = unit_size >> 1;

    // Y coordinate of the sbrow (y is 8 luma pixel rows above row_y)
    const int row_y = y + ((8 >> ss_ver) * !first_sby_in_tile_row);

    // FIXME This is an ugly hack to lookup the proper AV2Filter unit for
    // chroma planes. Question: For Multithreaded decoding, is it better
    // to store the chroma LR information with collocated Luma information?
    // In other words. For a chroma restoration unit locate at 128,128 and
    // with a 4:2:0 chroma subsampling, do we store the filter information at
    // the AV2Filter unit located at (128,128) or (256,256)
    // TODO Support chroma subsampling.
    const int shift_hor = 8 - ss_hor;

    /* maximum sbrow height is 256 + 8 rows offset */
    ALIGN_STK_16(pixel, pre_lr_border, 2, [256 + 8][6]);
    const Av2RestorationUnit *lr[2];

    enum LrEdgeFlags edges = (y > 0 ? LR_HAVE_TOP : 0) | LR_HAVE_RIGHT;
    if (first_sby_in_tile_row == FIRST_SB_TOP) edges |= LR_HAVE_BOTTOM_INTEGRATED;
    if (first_sby_in_tile_row == FIRST_SB_BOTTOM && y > 0) edges |= LR_HAVE_TOP_INTEGRATED;

    int aligned_unit_pos = row_y & ~(unit_size - 1);
    if (aligned_unit_pos && aligned_unit_pos + half_unit_size > h)
        aligned_unit_pos -= unit_size;
    aligned_unit_pos <<= ss_ver;
    const int sb_idx = (aligned_unit_pos >> 8) * f->sb256w;
    const int unit_idx = ((aligned_unit_pos >> 6) & 0x3) << 2;
    lr[0] = &f->lf.lr_mask[sb_idx].lr[plane][unit_idx];
    int restore = 1; // TODO: restore logic for disabling backups
    int x = 0, bit = 0;
    for (; x + 64 < w; p += 64, edges |= LR_HAVE_LEFT, bit ^= 1) {
        const int next_x = x + 64;
        const int next_iter_lru_start_x = next_x & ~(unit_size - 1);
        const int next_u_idx = unit_idx + ((next_iter_lru_start_x >> (shift_hor - 2)) & 3);
        lr[!bit] =
            &f->lf.lr_mask[sb_idx + (next_iter_lru_start_x >> shift_hor)].lr[plane][next_u_idx];
        const int restore_next = 1;
        // FIXME could backup 4px for luma if gdf=off
        if (restore_next)
            backupNxU(pre_lr_border[bit], p + 64, p_stride, row_h - y,
                      plane ? 2 : 6);
        if (restore)
            lr_stripe(f, p, pre_lr_border[!bit], x, y, plane, 64, row_h,
                      lr[bit], edges, first_sby_in_tile_row, tile_row_m1);
        x = next_x;
        restore = restore_next;
    }
    if (restore) {
        edges &= ~LR_HAVE_RIGHT;
        const int end_w = w - x;
        lr_stripe(f, p, pre_lr_border[!bit], x, y, plane, end_w, row_h,
                  lr[bit], edges, first_sby_in_tile_row, tile_row_m1);
    }
}

static inline void copyNlines(pixel *dst, const pixel *src, const ptrdiff_t stride,
                              const int n)
{
    memcpy(dst, src, stride * n);
}

void bytefn(dav2d_lr_sbrow)(Dav2dFrameContext *const f, pixel *const dst[3],
                            const int sby, const int tile_row)
{
    // TODO: strips starting at each tile row need to be shorted, not just the first row.
    const Dav2dFrameHeader *const hdr = f->frame_hdr;
    const ptrdiff_t *const dst_stride = f->cur.p.stride;
    const int restore_planes = f->lf.restore_planes;
    const int not_last = sby + 1 < f->sbh;
    const int first_sby_in_tile_row =
        sby == hdr->tiling.t.row_start_sb[tile_row];

    if (restore_planes & (LR_RESTORE_U | LR_RESTORE_V)) {
        const int ss_ver = f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
        const int ss_hor = f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444;
        const int h = f->bh * 4 >> ss_ver;
        const int w = f->bw * 4 >> ss_hor;
        const int next_row_y = (sby + 1) << ((6 - ss_ver) + f->frame_hdr->sb128);
        const int row_h = imin(next_row_y - (8 >> ss_ver) * not_last, h);
        int offset_uv = 8 * !!sby >> ss_ver;
        int y_stripe = (sby << ((6 - ss_ver) + f->frame_hdr->sb128)) - offset_uv;
        if (sby && first_sby_in_tile_row) {
            if (restore_planes & LR_RESTORE_U) {
                copyNlines(&f->lf.lr_cdef_line[1][2 * PXSTRIDE(dst_stride[1]) *
                                                  (tile_row - 1)],
                           dst[1] - 2 * PXSTRIDE(dst_stride[1]), dst_stride[1], 2);
                lr_sbrow(f, dst[1] - offset_uv * PXSTRIDE(dst_stride[1]),
                         y_stripe, w, h, y_stripe + (8 >> ss_ver), 1,
                         first_sby_in_tile_row * FIRST_SB_TOP, tile_row - 1);
            }
            if (restore_planes & LR_RESTORE_V) {
                copyNlines(&f->lf.lr_cdef_line[2][2 * PXSTRIDE(dst_stride[1]) *
                                                  (tile_row - 1)],
                           dst[2] - 2 * PXSTRIDE(dst_stride[1]), dst_stride[1], 2);
                lr_sbrow(f, dst[2] - offset_uv * PXSTRIDE(dst_stride[1]),
                         y_stripe, w, h, y_stripe + (8 >> ss_ver), 2,
                         first_sby_in_tile_row * FIRST_SB_TOP, tile_row - 1);
            }
            offset_uv = 0;
            y_stripe += 8 >> ss_ver;
        }

        if (restore_planes & LR_RESTORE_U)
            lr_sbrow(f, dst[1] - offset_uv * PXSTRIDE(dst_stride[1]), y_stripe,
                     w, h, row_h, 1, first_sby_in_tile_row * FIRST_SB_BOTTOM,
                     tile_row - 1);
        if (restore_planes & LR_RESTORE_V)
            lr_sbrow(f, dst[2] - offset_uv * PXSTRIDE(dst_stride[1]), y_stripe,
                     w, h, row_h, 2, first_sby_in_tile_row * FIRST_SB_BOTTOM,
                     tile_row - 1);
    }

    if (restore_planes & LR_RESTORE_Y) {
        const int h = f->bh * 4;
        const int w = f->bw * 4;
        const int next_row_y = (sby + 1) << (6 + f->frame_hdr->sb128);
        int row_h = imin(next_row_y - 8 * not_last, h);
        int offset_y = 8 * !!sby;
        int y_stripe = (sby << (6 + f->frame_hdr->sb128)) - offset_y;
        if (sby && first_sby_in_tile_row) {
            copyNlines(&f->lf.lr_cdef_line[0][4 * PXSTRIDE(dst_stride[0]) * (tile_row - 1)],
                       dst[0] - 4 * PXSTRIDE(dst_stride[0]), dst_stride[0], 4);
            lr_sbrow(f, dst[0] - offset_y * PXSTRIDE(dst_stride[0]), y_stripe, w,
                     h, y_stripe + 8, 0, first_sby_in_tile_row * FIRST_SB_TOP,
                     tile_row - 1);
            offset_y = 0;
            y_stripe += 8;
        }
        lr_sbrow(f, dst[0] - offset_y * PXSTRIDE(dst_stride[0]), y_stripe, w,
                 h, row_h, 0, first_sby_in_tile_row * FIRST_SB_BOTTOM,
                 tile_row - 1);
    }
}
