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

#ifndef DAV2D_SRC_MC_H
#define DAV2D_SRC_MC_H

#include <stdint.h>
#include <stddef.h>

#include "common/bitdepth.h"

#include "src/levels.h"

#define decl_mc_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            int w, int h, int mx, int my HIGHBD_DECL_SUFFIX)
typedef decl_mc_fn(*mc_fn);

#define decl_mc_scaled_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            int w, int h, int mx, int my, int dx, int dy HIGHBD_DECL_SUFFIX)
typedef decl_mc_scaled_fn(*mc_scaled_fn);

#define decl_warp8x8_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            const int16_t *abcd, int mx, int my HIGHBD_DECL_SUFFIX)
typedef decl_warp8x8_fn(*warp8x8_fn);

#define decl_mct_fn(name) \
void (name)(int16_t *tmp, ptrdiff_t dst_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            int w, int h, int mx, int my HIGHBD_DECL_SUFFIX)
typedef decl_mct_fn(*mct_fn);

#define decl_ext_warp4x4_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            int mx, int my HIGHBD_DECL_SUFFIX)
typedef decl_ext_warp4x4_fn(*ext_warp4x4_fn);

#define decl_mct_scaled_fn(name) \
void (name)(int16_t *tmp, ptrdiff_t dst_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            int w, int h, int mx, int my, int dx, int dy HIGHBD_DECL_SUFFIX)
typedef decl_mct_scaled_fn(*mct_scaled_fn);

#define decl_warp8x8t_fn(name) \
void (name)(int16_t *tmp, const ptrdiff_t tmp_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            const int16_t *abcd, int mx, int my HIGHBD_DECL_SUFFIX)
typedef decl_warp8x8t_fn(*warp8x8t_fn);

#define decl_ext_warp4x4t_fn(name) \
void (name)(int16_t *tmp, const ptrdiff_t tmp_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            int mx, int my HIGHBD_DECL_SUFFIX)
typedef decl_ext_warp4x4t_fn(*ext_warp4x4t_fn);

#define decl_avg_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const int16_t *tmp1, const int16_t *tmp2, int w, int h \
            HIGHBD_DECL_SUFFIX)
typedef decl_avg_fn(*avg_fn);

#define decl_w_avg_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const int16_t *tmp1, const int16_t *tmp2, int w, int h, int weight \
            HIGHBD_DECL_SUFFIX)
typedef decl_w_avg_fn(*w_avg_fn);

#define decl_mask_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const int16_t *tmp1, const int16_t *tmp2, int w, int h, \
            const uint8_t *mask HIGHBD_DECL_SUFFIX)
typedef decl_mask_fn(*mask_fn);

#define decl_w_mask_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const int16_t *tmp1, const int16_t *tmp2, int w, int h, \
            uint8_t *mask, int sign HIGHBD_DECL_SUFFIX)
typedef decl_w_mask_fn(*w_mask_fn);

#define decl_blend_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, const pixel *tmp, \
            int w, int h, const uint8_t *mask)
typedef decl_blend_fn(*blend_fn);

#define decl_emu_edge_fn(name) \
void (name)(intptr_t bw, intptr_t bh, intptr_t iw, intptr_t ih, intptr_t x, intptr_t y, \
            pixel *dst, ptrdiff_t dst_stride, const pixel *src, ptrdiff_t src_stride)
typedef decl_emu_edge_fn(*emu_edge_fn);

#define decl_resize_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const pixel *src, ptrdiff_t src_stride, \
            int dst_w, int h, int src_w, int dx, int mx HIGHBD_DECL_SUFFIX)
typedef decl_resize_fn(*resize_fn);

#define decl_morph_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, int alpha, int beta, \
            int w, int h HIGHBD_DECL_SUFFIX)
typedef decl_morph_fn(*morph_fn);

struct OpflOffset {
    int8_t y, x;
};
#define decl_sad_refine_mv_fn(name) \
void (name)(const pixel *p0, ptrdiff_t p0_stride, \
            const pixel *p1, ptrdiff_t p1_stride, \
            int w, int h, int is_implicit, \
            struct OpflOffset *o HIGHBD_DECL_SUFFIX)
typedef decl_sad_refine_mv_fn(*sad_refine_mv_fn);

struct OpflRegressionData {
    int32_t su2, suv, sv2, suw, svw;
};
#define decl_opfl_derive_mv_fn(name) \
void (name)(struct OpflRegressionData *out, \
            const pixel *p0, ptrdiff_t p0_stride, \
            const pixel *p1, ptrdiff_t p1_stride, \
            int w, int h, int bs, const int8_t d[2] HIGHBD_DECL_SUFFIX)
typedef decl_opfl_derive_mv_fn(*opfl_derive_mv_fn);

#define decl_sad8x8_fn(name) \
unsigned (name)(const pixel *p0, ptrdiff_t p0_stride, \
                const pixel *p1, ptrdiff_t p1_stride HIGHBD_DECL_SUFFIX)
typedef decl_sad8x8_fn(*sad8x8_fn);

#define decl_8tap_gen(decl_name, fn_name, opt) \
    decl_##decl_name##_fn(BF(dav2d_##fn_name##_8tap_regular, opt)); \
    decl_##decl_name##_fn(BF(dav2d_##fn_name##_8tap_smooth,  opt)); \
    decl_##decl_name##_fn(BF(dav2d_##fn_name##_8tap_sharp,   opt))

#define decl_8tap_fns(opt) \
    decl_8tap_gen(mc,  put,  opt); \
    decl_8tap_gen(mct, prep, opt)

#define init_8tap_gen(name, opt) \
    init_##name##_fn(DAV2D_FILTER_8TAP_REGULAR, 8tap_regular, opt); \
    init_##name##_fn(DAV2D_FILTER_8TAP_SMOOTH,  8tap_smooth,  opt); \
    init_##name##_fn(DAV2D_FILTER_8TAP_SHARP,   8tap_sharp,   opt)

#define init_8tap_fns(opt) \
    init_8tap_gen(mc,  opt); \
    init_8tap_gen(mct, opt)

typedef struct Dav2dMCDSPContext {
    mc_fn mc[DAV2D_N_FILTERS];
    mc_scaled_fn mc_scaled[DAV2D_N_FILTERS];
    mct_fn mct[DAV2D_N_FILTERS];
    mct_scaled_fn mct_scaled[DAV2D_N_FILTERS];
    avg_fn avg;
    w_avg_fn w_avg;
    mask_fn mask;
    w_mask_fn w_mask[3 /* 444, 422, 420 */];
    blend_fn blend;
    warp8x8_fn warp8x8;
    warp8x8t_fn warp8x8t;
    ext_warp4x4_fn ext_warp4x4;
    ext_warp4x4t_fn ext_warp4x4t;
    emu_edge_fn emu_edge;
    resize_fn resize;
    morph_fn morph;
    opfl_derive_mv_fn opfl_derive_mv;
    sad_refine_mv_fn sad_refine_mv;
    sad8x8_fn sad8x8;
} Dav2dMCDSPContext;

bitfn_decls(void dav2d_mc_dsp_init, Dav2dMCDSPContext *c);

#endif /* DAV2D_SRC_MC_H */
