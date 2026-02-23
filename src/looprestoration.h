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

#ifndef DAV2D_SRC_LOOPRESTORATION_H
#define DAV2D_SRC_LOOPRESTORATION_H

#include <stdint.h>
#include <stddef.h>

#include "common/bitdepth.h"

enum LrEdgeFlags {
    LR_HAVE_LEFT = 1 << 0,
    LR_HAVE_RIGHT = 1 << 1,
    LR_HAVE_TOP = 1 << 2,
    LR_HAVE_BOTTOM = 1 << 3,
    // top (or bottom) edges are at tile row boundaries and consist of
    // post-cdef/ccso instead of pre-cdef/ccso data. This also means they
    // contain 4 lines of pixel data instead of 2.
    LR_HAVE_TOP_INTEGRATED = 1 << 4,
    LR_HAVE_BOTTOM_INTEGRATED = 1 << 5,
};

#ifdef BITDEPTH
typedef const pixel (*const_left_pixel_row)[6];
#else
typedef const void *const_left_pixel_row;
#endif

typedef union WienerParams {
    struct {
        const int8_t *filter;
        const pixel *luma, *luma_top, *luma_bottom;
        ptrdiff_t stride;
        int ss_ver, ss_hor, ds_flt;
    } single;
    struct {
        union {
            const int8_t (*user)[18];
            const int16_t (*pretrained)[13];
        } filters;
        const uint8_t *subclass_lut;
        const uint16_t *noskip_mask;
        int base_q;
    } multi;
} WienerParams;

// Although the spec applies restoration filters over 4x4 blocks,
// they can be applied to a bigger surface.
//    * w is constrained by the smallest gdf block size (w <= 64)
//    * h is constrained by the stripe height (h <= 64)
// The filter functions are allowed to do aligned writes past the right
// edge of the buffer, aligned up to the minimum loop restoration unit size
// (which is 32 pixels for subsampled chroma and 64 pixels for luma).
#define decl_wiener_filter_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const_left_pixel_row left, \
            const pixel *top, const pixel *bottom, int w, int h, \
            const WienerParams *params, \
            enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
typedef decl_wiener_filter_fn(*wienerfilter_fn);

#define decl_gdf_prep_fn(name) \
void (name)(int8_t *dst, ptrdiff_t dst_stride, \
            const pixel *p, ptrdiff_t stride, \
            const_left_pixel_row left, \
            const pixel *lpf, int w, int h, \
            int ref_dst_idx, int qp_idx, \
            enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
typedef decl_gdf_prep_fn(*gdf_prep_fn);

#define decl_gdf_add_fn(name) \
void (name)(pixel *p, ptrdiff_t dst_stride, \
            const int8_t *err, const ptrdiff_t err_stride, \
            const int w, const int h, const int scale \
            HIGHBD_DECL_SUFFIX)
typedef decl_gdf_add_fn(*gdf_add_fn);

typedef struct Dav2dLoopRestorationDSPContext {
    wienerfilter_fn ns_wiener_single[2 /* y, uv */];
    wienerfilter_fn ns_wiener_multi;
    wienerfilter_fn pc_wiener;
    gdf_prep_fn gdf_prep;
    gdf_add_fn gdf_add;
} Dav2dLoopRestorationDSPContext;

bitfn_decls(void dav2d_loop_restoration_dsp_init, Dav2dLoopRestorationDSPContext *c, int bpc);

#endif /* DAV2D_SRC_LOOPRESTORATION_H */
