/*
 * Copyright © 2018-2025, VideoLAN and dav1d authors
 * Copyright © 2018-2025, Two Orioles, LLC
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

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "common/attributes.h"
#include "common/intops.h"

#include "src/itx.h"
#include "src/itx_1d.h"
#include "src/scan.h"
#include "src/tables.h"

static NOINLINE void
inv_txfm_add_c(pixel *dst, const ptrdiff_t stride, coef *const coeff,
               const enum TxfmType txtp, const int eob,
               const /*enum RectTxfmSize*/ int tx HIGHBD_DECL_SUFFIX)
{
    const TxfmInfo *const t_dim = &dav1d_txfm_dimensions[tx];
    const uint8_t *const tx_shift = dav1d_tx_shift[tx];
    const int w = 4 * t_dim->w, h = 4 * t_dim->h;
    assert(w >= 4 && w <= 64);
    assert(h >= 4 && h <= 64);
    assert(eob >= 0);

    const int is_rect2 = (t_dim->lw + t_dim->lh) & 1;
    if (eob + txtp == 0) { // DC-only DCT_DCT
        const int shift_p1 = tx_shift[0];
        const int shift = shift_p1 + tx_shift[1] - 12;
        const int rnd = (1 << (shift - 1)) + shift_p1 - 6;
        int dc = coeff[0];
        coeff[0] = 0;

        if (is_rect2)
            dc = (dc * 181 + 128) >> 8;
        dc = (dc + rnd) >> shift;

        for (int y = 0; y < h; y++, dst += PXSTRIDE(stride))
            for (int x = 0; x < w; x++)
                dst[x] = iclip_pixel(dst[x] + dc);
        return;
    }

    const uint8_t *const txtps = dav1d_tx1d_types[txtp];
    const itx_1d_fn first_1d_fn = dav1d_tx1d_fns[t_dim->lw][txtps[1]];
    const itx_1d_fn second_1d_fn = dav1d_tx1d_fns[t_dim->lh][txtps[0]];
    const int sh = imin(h, 32), sw = imin(w, 32);
#if BITDEPTH == 8
    const int row_clip_min = INT16_MIN;
#else
    const int row_clip_min = (int) ((unsigned) ~bitdepth_max << 7);
#endif
    const int row_clip_max = ~row_clip_min;

    int32_t tmp[64 * 64], *c = tmp;
#if 0
    // FIXME Disabled for now,this needs to be updated to AVM
    int last_nonzero_col; // in first 1d itx
    if (txtps[1] == IDENTITY && txtps[0] != IDENTITY) {
        last_nonzero_col = imin(sh - 1, eob);
    } else if (txtps[0] == IDENTITY && txtps[1] != IDENTITY) {
        last_nonzero_col = eob >> (t_dim->lw + 2);
    } else {
        last_nonzero_col = dav1d_last_nonzero_col_from_eob[tx][eob];
    }
    assert(last_nonzero_col < sh);
#else
    int last_nonzero_col = sh - 1;
#endif
    for (int y = 0; y <= last_nonzero_col; y++, c += w) {
        if (is_rect2)
            for (int x = 0; x < sw; x++)
                c[x] = (coeff[y + x * sh] * 181 + 128) >> 8;
        else
            for (int x = 0; x < sw; x++)
                c[x] = coeff[y + x * sh];
        first_1d_fn(c, 1);
    }

#if 0
    if (last_nonzero_col + 1 < sh)
        memset(c, 0, sizeof(*c) * (sh - last_nonzero_col - 1) * w);
#endif
    memset(coeff, 0, sizeof(*coeff) * sw * sh);
    int shift = tx_shift[0];
    int rnd = (1 << shift) >> 1;
    for (int i = 0; i < w * sh; i++)
        tmp[i] = iclip((tmp[i] + rnd) >> shift, row_clip_min, row_clip_max);

    for (int x = 0; x < w; x++)
        second_1d_fn(&tmp[x], w);

    shift = tx_shift[1];
    rnd = (1 << shift) >> 1;
    c = tmp;
    for (int y = 0; y < h; y++, dst += PXSTRIDE(stride))
        for (int x = 0; x < w; x++)
            dst[x] = iclip_pixel(dst[x] + ((*c++ + rnd) >> shift));
}

#define inv_txfm_fn(pfx, w, h) \
static void \
inv_txfm_add_##w##x##h##_c(pixel *dst, const ptrdiff_t stride, \
                           coef *const coeff, const enum TxfmType txtp, \
                           const int eob HIGHBD_DECL_SUFFIX) \
{ \
    inv_txfm_add_c(dst, stride, coeff, txtp, eob, pfx##TX_##w##X##h \
                   HIGHBD_TAIL_SUFFIX); \
}

inv_txfm_fn( ,  4,  4)
inv_txfm_fn(R,  4,  8)
inv_txfm_fn(R,  4, 16)
inv_txfm_fn(R,  4, 32)
inv_txfm_fn(R,  4, 64)
inv_txfm_fn(R,  8,  4)
inv_txfm_fn( ,  8,  8)
inv_txfm_fn(R,  8, 16)
inv_txfm_fn(R,  8, 32)
inv_txfm_fn(R,  8, 64)
inv_txfm_fn(R, 16,  4)
inv_txfm_fn(R, 16,  8)
inv_txfm_fn( , 16, 16)
inv_txfm_fn(R, 16, 32)
inv_txfm_fn(R, 16, 64)
inv_txfm_fn(R, 32,  4)
inv_txfm_fn(R, 32,  8)
inv_txfm_fn(R, 32, 16)
inv_txfm_fn( , 32, 32)
inv_txfm_fn(R, 32, 64)
inv_txfm_fn(R, 64,  4)
inv_txfm_fn(R, 64,  8)
inv_txfm_fn(R, 64, 16)
inv_txfm_fn(R, 64, 32)
inv_txfm_fn( , 64, 64)

static void inv_txfm_add_wht_wht_4x4_c(pixel *dst, const ptrdiff_t stride,
                                       coef *const coeff, const enum TxfmType txtp,
                                       const int eob HIGHBD_DECL_SUFFIX)
{
    int32_t tmp[4 * 4], *c = tmp;
    for (int y = 0; y < 4; y++, c += 4) {
        for (int x = 0; x < 4; x++)
            c[x] = coeff[y + x * 4] >> 2;
        dav1d_inv_wht4_1d_c(c, 1);
    }
    memset(coeff, 0, sizeof(*coeff) * 4 * 4);

    for (int x = 0; x < 4; x++)
        dav1d_inv_wht4_1d_c(&tmp[x], 4);

    c = tmp;
    for (int y = 0; y < 4; y++, dst += PXSTRIDE(stride))
        for (int x = 0; x < 4; x++)
            dst[x] = iclip_pixel(dst[x] + *c++);
}

#if HAVE_ASM && 0
#if ARCH_AARCH64 || ARCH_ARM
#include "src/arm/itx.h"
#elif ARCH_LOONGARCH64
#include "src/loongarch/itx.h"
#elif ARCH_PPC64LE
#include "src/ppc/itx.h"
#elif ARCH_RISCV
#include "src/riscv/itx.h"
#elif ARCH_X86
#include "src/x86/itx.h"
#endif
#endif

COLD void bitfn(dav1d_itx_dsp_init)(Dav1dInvTxfmDSPContext *const c, int bpc) {
#define assign_itx(w, h, pfx) \
    c->itxfm_add[pfx##TX_##w##X##h] = inv_txfm_add_##w##x##h##_c

    c->iwht_add_4x4 = inv_txfm_add_wht_wht_4x4_c;
    assign_itx( 4,  4, );
    assign_itx( 4,  8, R);
    assign_itx( 4, 16, R);
    assign_itx( 4, 32, R);
    assign_itx( 4, 64, R);
    assign_itx( 8,  4, R);
    assign_itx( 8,  8, );
    assign_itx( 8, 16, R);
    assign_itx( 8, 32, R);
    assign_itx( 8, 64, R);
    assign_itx(16,  4, R);
    assign_itx(16,  8, R);
    assign_itx(16, 16, );
    assign_itx(16, 32, R);
    assign_itx(16, 64, R);
    assign_itx(32,  4, R);
    assign_itx(32,  8, R);
    assign_itx(32, 16, R);
    assign_itx(32, 32, );
    assign_itx(32, 64, R);
    assign_itx(64,  4, R);
    assign_itx(64,  8, R);
    assign_itx(64, 16, R);
    assign_itx(64, 32, R);
    assign_itx(64, 64, );

    int all_simd = 0;
#if 0
#if HAVE_ASM
#if ARCH_AARCH64 || ARCH_ARM
    itx_dsp_init_arm(c, bpc, &all_simd);
#endif
#if ARCH_LOONGARCH64
    itx_dsp_init_loongarch(c, bpc);
#endif
#if ARCH_PPC64LE
    itx_dsp_init_ppc(c, bpc);
#endif
#if ARCH_RISCV
    itx_dsp_init_riscv(c, bpc);
#endif
#if ARCH_X86
    itx_dsp_init_x86(c, bpc, &all_simd);
#endif
#endif
#endif

    if (!all_simd)
        dav1d_init_last_nonzero_col_from_eob_tables();
}
