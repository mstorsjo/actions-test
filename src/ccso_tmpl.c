/*
 * Copyright © 2025, VideoLAN and dav1d authors
 * Copyright © 2025, Two Orioles, LLC
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

#include "ccso.h"

#include "common/bitdepth.h"
#include "common/intops.h"

static int8_t ccso_pos[7][2] = {
    { -1, 0 },
    { 0, -1 },
    { -1, -1 },
    { -1, 1 },
    { -1, -2 },
    { 1, -2 },
    { 0, 2 }
};

static void padding(pixel *const tmp, const ptrdiff_t tmp_stride,
                    const pixel *const src, const ptrdiff_t src_stride,
                    const pixel (*left)[2],
                    const pixel *top, const pixel *bottom,
                    const int w, const int h, enum CdefEdgeFlags edges)
{
    int x_min = edges & CDEF_HAVE_LEFT ? -2 : 0;
    int x_max = w - 1 + (edges & CDEF_HAVE_RIGHT ? 2 : 0);
    int y_min = edges & CDEF_HAVE_TOP ? -2 : 0;
    int y_max = h - 1 + (edges & CDEF_HAVE_BOTTOM ? 2 : 0);
    for (int y = -2; y < h + 2; y++) {
        int src_y = iclip(y, y_min, y_max);
        for (int x = -2; x < w + 2; x++) {
            pixel v;
            int src_x = iclip(x, x_min, x_max);
            if (src_y < 0) {
                v = top[src_x + (2 + src_y) * PXSTRIDE(src_stride)];
            } else if (src_y >= h) {
                v = bottom[src_x + (src_y - h) * PXSTRIDE(src_stride)];
            } else if (src_x < 0) {
                v = left[src_y][2 + src_x];
            } else {
                v = src[src_x + src_y * PXSTRIDE(src_stride)];
            }
            tmp[x + y * tmp_stride] = v;
        }
    }
}

static unsigned ccso_score(int diff, int quant_step, unsigned edge_classifier) {
    if (diff > quant_step && !edge_classifier)
        return 2;
    if (diff < -quant_step)
        return 0;
    return 1;
}

static NOINLINE void
ccso_prep_c(uint8_t *dst, ptrdiff_t dst_stride, const pixel *src, const ptrdiff_t src_stride,
            const pixel (*left)[2], const pixel *top, const pixel *bottom,
            unsigned max_band_log2, const unsigned ext_filter,
            const unsigned quant_step, const int edge_cfl,
            const int bo_only, enum CdefEdgeFlags edges,
            const int w, const int h, const int ss_hor, const int ss_ver
            HIGHBD_DECL_SUFFIX)
{
    const unsigned shift = bitdepth_from_max(bitdepth_max) - max_band_log2;
    const int dy = ccso_pos[ext_filter][0];
    const int dx = ccso_pos[ext_filter][1];
    const ptrdiff_t tmp_stride = 68;
    const ptrdiff_t luma_offset = dx + dy * tmp_stride;
    pixel tmp_buf[68 * 12]; // 68*12 is the maximum value of tmp_stride * (h + 4)
    pixel *tmp = tmp_buf + 2 * tmp_stride + 2;
    padding(tmp, tmp_stride, src, src_stride, left, top, bottom, w, h, edges);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const int x_luma = x << ss_hor;
            const int c = tmp[x_luma];
            const int band = c >> shift;
            if (bo_only)
                dst[x] = band;
            else {
                unsigned cls0 = 0, cls1 = 0;
                cls0 = ccso_score(tmp[x_luma + luma_offset] - c, quant_step, edge_cfl);
                cls1 = ccso_score(tmp[x_luma - luma_offset] - c, quant_step, edge_cfl);
                dst[x] = (cls0 << 5) | (cls1 << 3) | band;
            }
        }
        tmp += tmp_stride << ss_ver;
        dst += dst_stride;
    }
}

#define ccso_prep_fn(ss_hor, ss_ver, name) \
static void ccso_prep_##name##_c(uint8_t *const dst, \
                                 ptrdiff_t dst_stride, \
                                 const pixel *const src, \
                                 const ptrdiff_t src_stride, \
                                 const pixel (*left)[2], \
                                 const pixel *top, \
                                 const pixel *bottom, \
                                 const unsigned max_band_log2, \
                                 const unsigned ext_filter, \
                                 const unsigned quant_step, \
                                 const int ccso_edge_cfl, \
                                 const int bo_only, \
                                 const int w, \
                                 const int h, \
                                 const enum CdefEdgeFlags edges \
                                 HIGHBD_DECL_SUFFIX) \
{ \
    ccso_prep_c(dst, dst_stride, src, src_stride, left, top, bottom, max_band_log2, ext_filter, \
                quant_step, ccso_edge_cfl, bo_only, edges, w, h, ss_hor, ss_ver HIGHBD_TAIL_SUFFIX); \
}

ccso_prep_fn(1, 1, 420);
ccso_prep_fn(1, 0, 422);
ccso_prep_fn(0, 0, 444);

static void ccso_add_c(pixel *dst, const ptrdiff_t dst_stride,
                       const uint8_t *idx, const ptrdiff_t idx_stride,
                       const int8_t *const filter_offsets, const int w, const int h
                       HIGHBD_DECL_SUFFIX)
{
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++)
            dst[x] = iclip_pixel(dst[x] + filter_offsets[idx[x]]);
        dst += PXSTRIDE(dst_stride);
        idx += idx_stride;
    }
}

COLD void bitfn(dav1d_ccso_dsp_init)(Dav1dCcsoDSPContext *const c) {
    c->prep[0] = ccso_prep_444_c;
    c->prep[1] = ccso_prep_422_c;
    c->prep[2] = ccso_prep_420_c;

    c->add = ccso_add_c;
}
