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

#ifndef DAV2D_SRC_IPRED_H
#define DAV2D_SRC_IPRED_H

#include <stddef.h>

#include "common/bitdepth.h"

#include "src/levels.h"

// These flags are OR'ed with the angle parameter in intra predictors.
// They encode intra mode features such as edge availability, multi-line MRL,
// DIP, IBP, edge filtering, and whether we're luma or chroma.
//
// ANGLE_IS_LUMA               - z1-3 use this flag to switch between a 4-tap
//                               luma filter and a 2-tap (bilinear) chroma filter
// ANGLE_DIP_FLAG              – enables directional intra prediction (DIP) mode
// ANGLE_HAS_TOP_FLAG          – top reference edge is available for prediction
// ANGLE_HAS_LEFT_FLAG         – left reference edge is available for prediction
// ANGLE_MULTI_MRL_FLAG        – enables multi-line MRL mode
// ANGLE_MRL_IDX               – multi-reference line (MRL) index bits (bits 13–14)
// ANGLE_IBP_FLAG              – enables intra bi-prediction for the current block
// ANGLE_USE_EDGE_FILTER_FLAG  – apply edge filtering (convolution) to reference samples
// ANGLE_SMOOTH_TOP_EDGE_FLAG  – indicates smooth top edge; use reduced filter strength
// ANGLE_SMOOTH_LEFT_EDGE_FLAG – indicates smooth left edge; use reduced filter strength
#define ANGLE_IS_LUMA               (1 << 19)
#define ANGLE_DIP_FLAG              (1 << 18)
#define ANGLE_HAS_TOP_FLAG          (1 << 17)
#define ANGLE_HAS_LEFT_FLAG         (1 << 16)
#define ANGLE_MULTI_MRL_FLAG        (1 << 15)
#define ANGLE_MRL_IDX_SHIFT (13)
#define ANGLE_MRL_IDX_MASK          (3 << ANGLE_MRL_IDX_SHIFT)
#define ANGLE_IBP_FLAG              (1 << 12)
#define ANGLE_USE_EDGE_FILTER_FLAG  (1 << 11)
#define ANGLE_SMOOTH_TOP_EDGE_FLAG  (1 << 10)
#define ANGLE_SMOOTH_LEFT_EDGE_FLAG (1 << 9)

#define CFL_HAS_TOP        (1 << 2)
#define CFL_HAS_LEFT       (1 << 3)
#define CFL_IS_TOP_SB_EDGE (1 << 4)

/*
 * Intra prediction.
 * - a is the angle (in degrees) for directional intra predictors. For other
 *   modes, it is ignored;
 * - topleft is the same as the argument given to dav2d_prepare_intra_edges(),
 *   see ipred_prepare.h for more detailed documentation.
 */
#define decl_angular_ipred_fn(name) \
void (name)(pixel *dst, ptrdiff_t stride, const pixel *topleft, \
            int width, int height, int angle, int max_width, int max_height \
            HIGHBD_DECL_SUFFIX)
typedef decl_angular_ipred_fn(*angular_ipred_fn);

/*
 * Create a subsampled Y edge and calculate its DC.
 */
#define decl_cfl_dc_fn(name) \
int (name)(uint16_t *edge, const pixel *top, const pixel *left, \
           ptrdiff_t stride, int wpad, int hpad, int w, int h, int filter_type)
typedef decl_cfl_dc_fn(*cfl_dc_fn);

/*
 * Create a subsampled Y plane with the edge DC subtracted.
 * - w/h_pad is the edge of the width/height that extends outside the visible
 *   portion of the frame in 4px units;
 * - ac has a stride of 16.
 */
#define decl_cfl_ac_fn(name) \
void (name)(int16_t *ac, int dc, const pixel *y, ptrdiff_t stride, \
            int w_pad, int h_pad, int cw, int ch, int filter_type)
typedef decl_cfl_ac_fn(*cfl_ac_fn);

/*
 * dst[x,y] += alpha * ac[x,y]
 * - alpha contains a q3 scalar in [-16,16] range;
 */
#define decl_cfl_pred_fn(name) \
void (name)(pixel *dst, ptrdiff_t stride, const pixel *topleft, \
            int width, int height, const int16_t *ac, int alpha \
            HIGHBD_DECL_SUFFIX)
typedef decl_cfl_pred_fn(*cfl_pred_fn);

/* CFL MHCCP */

#define decl_cfl_gen_y_fn(name) \
void (name)(uint16_t *dst, int dst_stride, \
            const pixel *src, const pixel *top_sb_edge, ptrdiff_t src_stride, \
            int refw, int refh, int tw, int th, int flags)
typedef decl_cfl_gen_y_fn(*cfl_gen_y_fn);

/*
 * max edge samples
 *  = 2*(1tl+64a+64tr)+64l+64bl = 2tl+64a+64tr+2*(64l+64bl)
 *  = 2tl+128a+128tr+64l+64bl   = 2tl+64a+64tr+128l+128bl
 *  = 386
 */
#define CFL_MAX_EDGE_SAMPLES 386

#define decl_cfl_gen_mat_fn(name) \
void (name)(int32_t mat[3][3], uint16_t imat[2][CFL_MAX_EDGE_SAMPLES], \
            const uint16_t *y, int ystride, int refw, int refh, int edge_flags \
            HIGHBD_DECL_SUFFIX)
typedef decl_cfl_gen_mat_fn(*cfl_gen_mat_fn);

/*
 * alpha[3] = gauss elimination(mat <- imat)
 */
#define decl_cfl_calc_alphas_fn(name) \
void (name)(int alpha[3], const pixel *c, const pixel *top_sb_edge, \
            ptrdiff_t stride, int w, int h, int32_t mat[3][3], \
            const uint16_t imat[2][CFL_MAX_EDGE_SAMPLES], int edge_flags \
            HIGHBD_DECL_SUFFIX)
typedef decl_cfl_calc_alphas_fn(*cfl_calc_alphas_fn);

/*
 * dst[x,y] = alpha[0] * (luma[center/above/left] >> 3) +
 *            alpha[1] * ((luma[center]/8)^2 + (1 << (bd/2)) >> 3) +
 *            alpha[2] << (bd/2)
 */
#define decl_cfl_mhccp_pred_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, const uint16_t *src, int src_stride, \
            int w, int h, const int alpha[3], int edge_flags HIGHBD_DECL_SUFFIX)
typedef decl_cfl_mhccp_pred_fn(*cfl_mhccp_pred_fn);

/*
 * dst[x,y] = pal[idx[x,y]]
 * - palette indices are [0-7]
 * - only 16-byte alignment is guaranteed for idx.
 */
#define decl_pal_pred_fn(name) \
void (name)(pixel *dst, ptrdiff_t stride, const pixel *pal, \
            const uint8_t *idx, int w, int h)
typedef decl_pal_pred_fn(*pal_pred_fn);

typedef struct Dav2dIntraPredDSPContext {
    angular_ipred_fn intra_pred[N_IMPL_INTRA_PRED_MODES];

    // chroma-from-luma (implicit and explicit alpha)
    cfl_dc_fn cfl_dc[3 /* 420, 422, 444 */];
    cfl_ac_fn cfl_ac[3 /* 420, 422, 444 */];
    cfl_pred_fn cfl_pred[DC_128_PRED + 1];

    // cfl mhccp
    cfl_gen_y_fn        cfl_gen_y[3 /* 420, 422, 444 */][3 /* cfl_ds_filter_type */];
    cfl_gen_mat_fn      cfl_gen_mat[3 /* CflMhDir */];
    cfl_calc_alphas_fn  cfl_calc_alphas;
    cfl_mhccp_pred_fn   cfl_mhccp_pred[3 /* CflMhDir */];

    // palette
    pal_pred_fn pal_pred;
} Dav2dIntraPredDSPContext;

bitfn_decls(void dav2d_intra_pred_dsp_init, Dav2dIntraPredDSPContext *c);

#endif /* DAV2D_SRC_IPRED_H */
