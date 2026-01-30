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

static void lr_stripe(const Dav2dFrameContext *const f, pixel *p,
                      const pixel (*left)[4], int x, int y,
                      const int plane, const int unit_w, const int row_h,
                      const Av2RestorationUnit *const lr, enum LrEdgeFlags edges)
{
    const Dav2dDSPContext *const dsp = f->dsp;
    const struct Dav2dNSWienerPlane *const pd = &f->frame_hdr->restoration.p[0].ns;
    const int chroma = !!plane;
    const int ss_ver = chroma & (f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420);
    const ptrdiff_t stride = f->cur.p.stride[chroma];
    const int sby = (y + (y ? 8 << ss_ver : 0)) >> (6 - ss_ver + f->frame_hdr->sb128);
    const int have_tt = f->c->n_tc > 1;
    const pixel *lpf = f->lf.lr_lpf_line[plane] +
        have_tt * (sby * (4 << f->frame_hdr->sb128) - 4) * PXSTRIDE(stride) + x;

    // The first stripe of the frame is shorter by 8 luma pixel rows.
    int stripe_h = imin((64 - 8 * !y) >> ss_ver, row_h - y);

    wienerfilter_fn wiener_fn;
    WienerParams wiener_params;
    uint16_t noskip_mask[64 + 2][12];
    int multi_wiener = 0;

    if (lr->type == DAV2D_RESTORATION_NS_WIENER) {
        if (pd->frame_filters_on) {
            if (pd->num_classes == 1) {
                wiener_fn = dsp->lr.ns_wiener_single;
                wiener_params.single.filter = pd->filter[0];
            } else {
                multi_wiener = 1;
                wiener_fn = dsp->lr.ns_wiener_multi;
                wiener_params.multi.base_q = f->lf.base_q;
                wiener_params.multi.subclass_lut = f->lf.ns_subclass_lut;
                wiener_params.multi.filters.user = pd->filter;
            }
        } else {
            wiener_fn = dsp->lr.ns_wiener_single;
            wiener_params.single.filter = lr->ns_filter[0];
        }
    } else if (lr->type == DAV2D_RESTORATION_PC_WIENER) {
        multi_wiener = 1;
        wiener_fn = dsp->lr.pc_wiener;
        wiener_params.multi.base_q = f->lf.base_q;
        wiener_params.multi.subclass_lut = f->lf.pc_subclass_lut;
        wiener_params.multi.filters.pretrained = f->lf.pc_filters;
    } else {
        return;
    }

    if (multi_wiener) {
        wiener_params.multi.noskip_mask = noskip_mask;
        // TODO: The below is a bit hacky to make wiener work over full restoration widths.
        //       We should refactor restoration to work over blocks 64 pixels wide at a time instead.
        for (int by = y >> 2, r = 0; by < row_h >> 2; by++, r++) {
            // Iterate with sb64 precision, otherwise the coded wouldn't support unit sizes of 64 or 128
            for (int sb64x = x >> 6, c = 0; sb64x < (x + unit_w + 63) >> 6; sb64x++, c++) {
                int by_idx = by & 63;
                int sb256_idx = f->sb256w * (by >> 6) + (sb64x >> 2);
                uint16_t* noskip_row = f->lf.mask[sb256_idx].lr_noskip_mask[by_idx];
                noskip_mask[r][c] = noskip_row[sb64x & 3];
            }
            // extend masks on the right edge
            if (!(edges & LR_HAVE_RIGHT) && unit_w & 63) {
                const int c = unit_w >> 6;
                const int shift = ((unit_w >> 2) & 15) - 1;
                const int mask = noskip_mask[r][c];
                const int edge = mask >> shift;
                noskip_mask[r][c] |= edge << (shift + 1);
            }
        }
    }

    while (y + stripe_h <= row_h) {
        // Change the HAVE_BOTTOM bit in edges to (sby + 1 != f->sbh || y + stripe_h != row_h)
        edges ^= (-(sby + 1 != f->sbh || y + stripe_h != row_h) ^ edges) & LR_HAVE_BOTTOM;
        wiener_fn(p, stride, left, lpf, unit_w, stripe_h, &wiener_params, edges HIGHBD_CALL_SUFFIX);
        left += stripe_h;
        y += stripe_h;
        if (multi_wiener) {
            wiener_params.multi.noskip_mask += stripe_h >> 2;
        }
        p += stripe_h * PXSTRIDE(stride);
        edges |= LR_HAVE_TOP;
        stripe_h = imin(64 >> ss_ver, row_h - y);
        if (stripe_h == 0) break;
        lpf += 4 * PXSTRIDE(stride);
    }
}

static void backup4xU(pixel (*dst)[4], const pixel *src, const ptrdiff_t src_stride,
                      int u)
{
    for (; u > 0; u--, dst++, src += PXSTRIDE(src_stride))
        pixel_copy(dst, src, 4);
}

static void lr_sbrow(const Dav2dFrameContext *const f, pixel *p, const int y,
                     const int w, const int h, const int row_h, const int plane)
{
    const int chroma = !!plane;
    const int ss_ver = chroma & (f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420);
    const int ss_hor = chroma & (f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444);
    const ptrdiff_t p_stride = f->cur.p.stride[chroma];

    const int unit_size_log2 = f->frame_hdr->restoration.unit_size[!!plane];
    const int unit_size = 1 << unit_size_log2;
    const int half_unit_size = unit_size >> 1;
    const int max_unit_size = unit_size + half_unit_size;

    // Y coordinate of the sbrow (y is 8 luma pixel rows above row_y)
    const int row_y = y + ((8 >> ss_ver) * !!y);

    // FIXME This is an ugly hack to lookup the proper AV2Filter unit for
    // chroma planes. Question: For Multithreaded decoding, is it better
    // to store the chroma LR information with collocated Luma information?
    // In other words. For a chroma restoration unit locate at 128,128 and
    // with a 4:2:0 chroma subsampling, do we store the filter information at
    // the AV2Filter unit located at (128,128) or (256,256)
    // TODO Support chroma subsampling.
    const int shift_hor = 8 - ss_hor;

    /* maximum sbrow height is 256 + 8 rows offset */
    ALIGN_STK_16(pixel, pre_lr_border, 2, [256 + 8][4]);
    const Av2RestorationUnit *lr[2];

    enum LrEdgeFlags edges = (y > 0 ? LR_HAVE_TOP : 0) | LR_HAVE_RIGHT;

    int aligned_unit_pos = row_y & ~(unit_size - 1);
    if (aligned_unit_pos && aligned_unit_pos + half_unit_size > h)
        aligned_unit_pos -= unit_size;
    aligned_unit_pos <<= ss_ver;
    const int sb_idx = (aligned_unit_pos >> 8) * f->sb256w;
    const int unit_idx = ((aligned_unit_pos >> 6) & 0x3) << 2;
    lr[0] = &f->lf.lr_mask[sb_idx].lr[plane][unit_idx];
    int restore = lr[0]->type != DAV2D_RESTORATION_NONE;
    int x = 0, bit = 0;
    for (; x + max_unit_size <= w; p += unit_size, edges |= LR_HAVE_LEFT, bit ^= 1) {
        const int next_x = x + unit_size;
        const int next_u_idx = unit_idx + ((next_x >> (shift_hor - 2)) & 0x3);
        lr[!bit] =
            &f->lf.lr_mask[sb_idx + (next_x >> shift_hor)].lr[plane][next_u_idx];
        const int restore_next = lr[!bit]->type != DAV2D_RESTORATION_NONE;
        if (restore_next)
            backup4xU(pre_lr_border[bit], p + unit_size - 4, p_stride, row_h - y);
        if (restore)
            lr_stripe(f, p, pre_lr_border[!bit], x, y, plane, unit_size, row_h,
                      lr[bit], edges);
        x = next_x;
        restore = restore_next;
    }
    if (restore) {
        edges &= ~LR_HAVE_RIGHT;
        const int unit_w = w - x;
        lr_stripe(f, p, pre_lr_border[!bit], x, y, plane, unit_w, row_h, lr[bit], edges);
    }
}

void bytefn(dav2d_lr_sbrow)(Dav2dFrameContext *const f, pixel *const dst[3],
                            const int sby)
{
    // TODO: strips starting at each tile row need to be shorted, not just the first row.
    const int offset_y = 8 * !!sby;
    const ptrdiff_t *const dst_stride = f->cur.p.stride;
    const int restore_planes = f->lf.restore_planes;
    const int not_last = sby + 1 < f->sbh;

    if (restore_planes & LR_RESTORE_Y) {
        const int h = f->bh * 4;
        const int w = f->bw * 4;
        const int next_row_y = (sby + 1) << (6 + f->frame_hdr->sb128);
        const int row_h = imin(next_row_y - 8 * not_last, h);
        const int y_stripe = (sby << (6 + f->frame_hdr->sb128)) - offset_y;
        lr_sbrow(f, dst[0] - offset_y * PXSTRIDE(dst_stride[0]), y_stripe, w,
                 h, row_h, 0);
    }
#if 0
    if (restore_planes & (LR_RESTORE_U | LR_RESTORE_V)) {
        const int ss_ver = f->cur.p.p.layout == DAV2D_PIXEL_LAYOUT_I420;
        const int ss_hor = f->cur.p.p.layout != DAV2D_PIXEL_LAYOUT_I444;
        const int h = (f->cur.p.p.h + ss_ver) >> ss_ver;
        const int w = (f->cur.p.p.w + ss_hor) >> ss_hor;
        const int next_row_y = (sby + 1) << ((6 - ss_ver) + f->frame_hdr->sb128);
        const int row_h = imin(next_row_y - (8 >> ss_ver) * not_last, h);
        const int offset_uv = offset_y >> ss_ver;
        const int y_stripe = (sby << ((6 - ss_ver) + f->frame_hdr->sb128)) - offset_uv;
        if (restore_planes & LR_RESTORE_U)
            lr_sbrow(f, dst[1] - offset_uv * PXSTRIDE(dst_stride[1]), y_stripe,
                     w, h, row_h, 1);

        if (restore_planes & LR_RESTORE_V)
            lr_sbrow(f, dst[2] - offset_uv * PXSTRIDE(dst_stride[1]), y_stripe,
                     w, h, row_h, 2);
    }
#endif
}
