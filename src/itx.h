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

#ifndef DAV2D_SRC_ITX_H
#define DAV2D_SRC_ITX_H

#include <stddef.h>

#include "common/bitdepth.h"

#include "src/levels.h"

#define decl_cctx_fn(name) \
void (name)(coef *u, coef *v, const int16_t angle[2], \
            size_t sz HIGHBD_DECL_SUFFIX)
typedef decl_cctx_fn(*cctx_fn);

#define decl_itxfm_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, coef *coeff, \
            enum TxfmType txtp, int eob HIGHBD_DECL_SUFFIX)
typedef decl_itxfm_fn(*itxfm_fn);

#define decl_itx_w_fns(w, ext) \
decl_itxfm_fn(BF(dav2d_inv_txfm_add_##w##x4,  ext)); \
decl_itxfm_fn(BF(dav2d_inv_txfm_add_##w##x8,  ext)); \
decl_itxfm_fn(BF(dav2d_inv_txfm_add_##w##x16, ext)); \
decl_itxfm_fn(BF(dav2d_inv_txfm_add_##w##x32, ext)); \
decl_itxfm_fn(BF(dav2d_inv_txfm_add_##w##x64, ext))

#define decl_itx_fns(ext) \
decl_itx_w_fns( 4, ext); \
decl_itx_w_fns( 8, ext); \
decl_itx_w_fns(16, ext); \
decl_itx_w_fns(32, ext); \
decl_itx_w_fns(64, ext)

typedef struct Dav2dInvTxfmDSPContext {
    cctx_fn cctx;
    itxfm_fn itxfm_add[N_RECT_TX_SIZES];
    itxfm_fn iwht_add_4x4;
} Dav2dInvTxfmDSPContext;

bitfn_decls(void dav2d_itx_dsp_init, Dav2dInvTxfmDSPContext *c, int bpc);

#define assign_itx_fn(pfx, w, h, ext) \
    c->itxfm_add[pfx##TX_##w##X##h] = BF(dav2d_inv_txfm_add_##w##x##h, ext)

#endif /* DAV2D_SRC_ITX_H */
