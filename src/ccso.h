/*
 * Copyright © 2025-2026, VideoLAN and dav2d authors
 * Copyright © 2025-2026, Two Orioles, LLC
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

#ifndef DAV2D_SRC_CCSO_H
#define DAV2D_SRC_CCSO_H

#include "cdef.h"

#include "common/bitdepth.h"

#define decl_ccso_prep_fn(name) \
void (name)(uint8_t *dst, ptrdiff_t dst_stride, const pixel *src, ptrdiff_t src_stride, \
            const_left_pixel_row_2px left, const pixel *top, const pixel *bottom, \
            unsigned max_band_log2, unsigned ext_filter, unsigned quant_step, \
            int edge_cfl, int bo_only, int w, int h, enum CdefEdgeFlags edges \
            HIGHBD_DECL_SUFFIX)
typedef decl_ccso_prep_fn(*ccso_prep_fn);

#define decl_ccso_add_fn(name) \
void (name)(pixel *dst, const ptrdiff_t dst_stride, const uint8_t *idx, \
            ptrdiff_t idx_stride, const uint8_t *offset_idxs, \
            const int8_t *offset_lut, int w, int h, \
            const uint16_t (*ll_mask)[4] HIGHBD_DECL_SUFFIX)
typedef decl_ccso_add_fn(*ccso_add_fn);

typedef struct Dav2dCcsoDSPContext {
    ccso_prep_fn prep[3 /* 444/luma, 422, 420 */];
    ccso_add_fn add;
} Dav2dCcsoDSPContext;

bitfn_decls(void dav2d_ccso_dsp_init, Dav2dCcsoDSPContext *c);

#endif /* DAV2D_SRC_CCSO_H */
