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

#ifndef DAV1D_SRC_IPRED_H
#define DAV1D_SRC_IPRED_H

#include <stddef.h>

#include "common/bitdepth.h"

#include "src/levels.h"

// These flags are OR'ed with the angle parameter in intra predictors.
// They encode intra mode features such as edge availability, MRL, IBP,
// and filtering.
//
// ANGLE_DIP_FLAG             – enables directional intra prediction (DIP) mode
// ANGLE_HAS_TOP              – top reference edge is available for prediction
// ANGLE_HAS_LEFT             – left reference edge is available for prediction
// ANGLE_MULTI_MRL_FLAG       – indicates multi-line MRL mode is enabled
// ANGLE_MRL_IDX              – multi-reference line (MRL) index bits
// ANGLE_IBP_FLAG             – enables intra bi-prediction for the current block
// ANGLE_USE_EDGE_FILTER_FLAG – apply edge filtering (convolution) to reference samples
// ANGLE_SMOOTH_EDGE_FLAG     – indicates smooth edges; use reduced filter strength
#define ANGLE_DIP_FLAG             (1 << 17)
#define ANGLE_HAS_TOP              (1 << 16)
#define ANGLE_HAS_LEFT             (1 << 15)
#define ANGLE_MULTI_MRL_FLAG       (1 << 14)
#define ANGLE_MRL_IDX              ((1 << 12) | (1 << 13))
#define ANGLE_IBP_FLAG             (1 << 11)
#define ANGLE_USE_EDGE_FILTER_FLAG (1 << 10)
#define ANGLE_SMOOTH_EDGE_FLAG     (1 << 9)

/*
 * Intra prediction.
 * - a is the angle (in degrees) for directional intra predictors. For other
 *   modes, it is ignored;
 * - topleft is the same as the argument given to dav1d_prepare_intra_edges(),
 *   see ipred_prepare.h for more detailed documentation.
 */
#define decl_angular_ipred_fn(name) \
void (name)(pixel *dst, ptrdiff_t stride, const pixel *topleft, \
            int width, int height, int angle, int max_width, int max_height \
            HIGHBD_DECL_SUFFIX)
typedef decl_angular_ipred_fn(*angular_ipred_fn);

/*
 * Create a subsampled Y plane with the DC subtracted.
 * - w/h_pad is the edge of the width/height that extends outside the visible
 *   portion of the frame in 4px units;
 * - ac has a stride of 16.
 */
#define decl_cfl_ac_fn(name) \
void (name)(int16_t *ac, const pixel *y, ptrdiff_t stride, \
            int w_pad, int h_pad, int cw, int ch)
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

/*
 * dst[x,y] = pal[idx[x,y]]
 * - palette indices are [0-7]
 * - only 16-byte alignment is guaranteed for idx.
 */
#define decl_pal_pred_fn(name) \
void (name)(pixel *dst, ptrdiff_t stride, const pixel *pal, \
            const uint8_t *idx, int w, int h)
typedef decl_pal_pred_fn(*pal_pred_fn);

/*
 * 9-tap filter that can be applied to intra prediction
 * - topleft is the same as the argument given to dav1d_prepare_intra_edges(),
 *   see ipred_prepare.h for more detailed documentation.
 * - th_mask disables vertical (bit: 1) or horizontal (bit:0) filtering
 */
#define decl_orip_fn(name) \
void (name)(pixel *dst, ptrdiff_t stride, const pixel *topleft, \
            unsigned th_mask, int width, int height HIGHBD_DECL_SUFFIX)
typedef decl_orip_fn(*orip_fn);

typedef struct Dav1dIntraPredDSPContext {
    angular_ipred_fn intra_pred[N_IMPL_INTRA_PRED_MODES];

    // chroma-from-luma
    cfl_ac_fn cfl_ac[3 /* 420, 422, 444 */];
    cfl_pred_fn cfl_pred[DC_128_PRED + 1];

    // palette
    pal_pred_fn pal_pred;

    // offset-based refinement for intra prediction
    orip_fn orip;
} Dav1dIntraPredDSPContext;

bitfn_decls(void dav1d_intra_pred_dsp_init, Dav1dIntraPredDSPContext *c);

#endif /* DAV1D_SRC_IPRED_H */
