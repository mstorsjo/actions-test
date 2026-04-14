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

#include "tests/checkasm/internal.h"

#include <math.h>
#include <stdio.h>

#include "src/itx.h"
#include "src/levels.h"
#include "src/scan.h"
#include "src/tables.h"

static const char *const itx_1d_names[] = {
    [DCT]      = "dct",
    [IDENTITY] = "identity",
    [ADST]     = "adst",
    [FLIPADST] = "flipadst",
    [DDT]      = "ddt",
    [FLIPDDT]  = "flipddt",
    [WHT]      = "wht",
};

static const char *const dpcm_names[] = {
    "", "_hdpcm", "_vdpcm"
};

static int generate_coefs(coef *coeff, const enum RectTxfmSize tx,
                          const enum TxfmType txtp, const int sw, const int sh,
                          const int subsh, int *const max_eob, const int coef_max)
{
    /* Generate topleft coefficients such that the return value (being the
     * coefficient scantable index for the eob token) guarantees that only
     * the topleft $sub out of $sz (where $sz >= $sub) coefficients in both
     * dimensions are non-zero. This leads to braching to specific optimized
     * simd versions (e.g. dc-only) so that we get full asm coverage in this
     * test */

    const enum TxClass tx_class = (txtp >> 3) & 3;
    const uint16_t *const scan = dav2d_scans[tx];
    const int sub_high = subsh > 0 ? subsh * 8 - 1 : 0;
    const int sub_low  = subsh > 1 ? sub_high - 8 : 0;
    const int coef_sign = (coef_max + 1) >> 1;
    int n, eob;

    for (n = 0, eob = 0; n < sw * sh; n++) {
        int rc, rcx, rcy;
        if (tx_class == TX_CLASS_H)
            rcx = n % sh, rcy = n / sh, rc = n;
        else if (tx_class == TX_CLASS_V)
            rcx = n / sw, rcy = n % sw, rc = rcy * sh + rcx;
        else
            rc = scan[n], rcx = rc % sh, rcy = rc / sh;

        /* Pick a random eob within this sub-itx */
        if (rcx > sub_high || rcy > sub_high)
            break; /* upper boundary */
        if (!eob && (rcx > sub_low || rcy > sub_low))
            eob = n; /* lower boundary */

        coeff[rc] = (rnd() & coef_max) - coef_sign;
    }
    *max_eob = n - 1;

    if (eob)
        eob += rnd() % (n - eob - 1);
    if (tx_class == TX_CLASS_H)
        for (n = eob + 1; n < sw * sh; n++)
            coeff[n] = 0;
    else if (tx_class == TX_CLASS_V) {
        for (int rcx = eob / sw, rcy = eob % sw; rcx < sh; rcx++, rcy = -1)
            while (++rcy < sw)
                coeff[rcy * sh + rcx] = 0;
        n = sw * sh;
    } else {
        for (n = eob + 1; n < sw * sh; n++)
            coeff[scan[n]] = 0;
        if (tx_class == TX_CLASS_2D_INV) {
            /* Reverse the coefficient array */
            for (int i = 0, j = n - 1; i < j; i++, j--) {
                int tmp = coeff[i];
                coeff[i] = coeff[j];
                coeff[j] = tmp;
            }
        }
    }
    for (; n < 32 * 32; n++)
        coeff[n] = rnd();

    return eob;
}

#define TXTP_MASK_DCT_ONLY DCT_DCT, 0xff /* invalid */
#define TXTP_MASK_IDTX IDTX, IDTX_INV
#define TXTP_MASK_DCT_ID_ONLY TXTP_MASK_IDTX, TXTP_MASK_DCT_ONLY
#define TXTP_MASK_DCT_HOR ADST_DCT, FLIPADST_DCT, H_DCT, TXTP_MASK_DCT_ONLY
#define TXTP_MASK_DCT_VER DCT_ADST, DCT_FLIPADST, V_DCT, TXTP_MASK_DCT_ONLY
#define TXTP_MASK_DCT_ID_HOR \
    V_DCT, V_ADST, V_FLIPADST, TXTP_MASK_IDTX, TXTP_MASK_DCT_HOR
#define TXTP_MASK_DCT_ID_VER \
    H_DCT, H_ADST, H_FLIPADST, TXTP_MASK_IDTX, TXTP_MASK_DCT_VER
#define TXTP_MASK_16x16 \
    FLIPADST_FLIPADST, ADST_FLIPADST, FLIPADST_ADST, TXTP_MASK_IDTX, \
    ADST_ADST, DCT_ADST, DCT_FLIPADST, V_DCT, TXTP_MASK_DCT_HOR
#define TXTP_MASK_ALL V_ADST, H_ADST, V_FLIPADST, H_FLIPADST, TXTP_MASK_16x16
#define TXTP_MASK_ALL_LOSSLESS WHT_WHT, TXTP_MASK_ALL

#define TXTP_MASK_DCT_VER_W_DDT DCT_DDT, DCT_FLIPDDT, TXTP_MASK_DCT_VER
#define TXTP_MASK_DCT_HOR_W_DDT DDT_DCT, FLIPDDT_DCT, TXTP_MASK_DCT_HOR
#define TXTP_MASK_DCT_ID_VER_W_DDT \
    IDENTITY_DDT, IDENTITY_FLIPDDT, DCT_DDT, DCT_FLIPDDT, TXTP_MASK_DCT_ID_VER
#define TXTP_MASK_DCT_ID_HOR_W_DDT \
    DDT_IDENTITY, FLIPDDT_IDENTITY, DDT_DCT, FLIPDDT_DCT, TXTP_MASK_DCT_ID_HOR
#define TXTP_MASK_DDT_NOID \
    FLIPDDT_FLIPDDT, DDT_FLIPDDT, FLIPDDT_DDT, DDT_DDT, \
    DCT_DDT, DDT_DCT, DCT_FLIPDDT, FLIPDDT_DCT
#define TXTP_MASK_16x16_W_DDT TXTP_MASK_DDT_NOID, TXTP_MASK_16x16
#define TXTP_MASK_ALL_W_DDT_2D \
    TXTP_MASK_DDT_NOID, DDT_IDENTITY, FLIPDDT_IDENTITY, \
    IDENTITY_DDT, IDENTITY_FLIPDDT, TXTP_MASK_ALL
#define TXTP_MASK_ALL_W_DDT_HOR \
    ADST_DDT, ADST_FLIPDDT, DCT_DDT, DCT_FLIPDDT, FLIPADST_DDT, \
    FLIPADST_FLIPDDT, IDENTITY_DDT, IDENTITY_FLIPDDT, TXTP_MASK_ALL
#define TXTP_MASK_ALL_W_DDT_VER \
    DDT_ADST, FLIPDDT_ADST, DDT_DCT, FLIPDDT_DCT, DDT_FLIPADST, \
    FLIPDDT_FLIPADST, DDT_IDENTITY, FLIPDDT_IDENTITY, TXTP_MASK_ALL

static const uint8_t valid_txtp_per_txsz[N_RECT_TX_SIZES][30] = {
    [TX_4X4]    = { TXTP_MASK_ALL_LOSSLESS },
    [TX_8X8]    = { TXTP_MASK_ALL_W_DDT_2D },
    [TX_16X16]  = { TXTP_MASK_16x16_W_DDT },
    [TX_32X32]  = { TXTP_MASK_DCT_ID_ONLY },
    [TX_64X64]  = { TXTP_MASK_DCT_ONLY },
    [RTX_4X8]   = { TXTP_MASK_ALL_W_DDT_VER },
    [RTX_8X4]   = { TXTP_MASK_ALL_W_DDT_HOR },
    [RTX_8X16]  = { TXTP_MASK_ALL_W_DDT_2D },
    [RTX_16X8]  = { TXTP_MASK_ALL_W_DDT_2D },
    [RTX_16X32] = { TXTP_MASK_DCT_ID_VER_W_DDT },
    [RTX_32X16] = { TXTP_MASK_DCT_ID_HOR_W_DDT },
    [RTX_32X64] = { TXTP_MASK_DCT_ONLY },
    [RTX_64X32] = { TXTP_MASK_DCT_ONLY },
    [RTX_4X16]  = { TXTP_MASK_ALL_W_DDT_VER },
    [RTX_16X4]  = { TXTP_MASK_ALL_W_DDT_HOR },
    [RTX_8X32]  = { TXTP_MASK_DCT_ID_VER_W_DDT },
    [RTX_32X8]  = { TXTP_MASK_DCT_ID_HOR_W_DDT },
    [RTX_16X64] = { TXTP_MASK_DCT_VER_W_DDT },
    [RTX_64X16] = { TXTP_MASK_DCT_HOR_W_DDT },
    [RTX_4X32]  = { TXTP_MASK_DCT_ID_VER },
    [RTX_32X4]  = { TXTP_MASK_DCT_ID_HOR },
    [RTX_8X64]  = { TXTP_MASK_DCT_VER_W_DDT },
    [RTX_64X8]  = { TXTP_MASK_DCT_HOR_W_DDT },
    [RTX_4X64]  = { TXTP_MASK_DCT_VER },
    [RTX_64X4]  = { TXTP_MASK_DCT_HOR },
};

static void check_itxfm_add(const Dav2dInvTxfmDSPContext *const c,
                            const enum RectTxfmSize tx)
{
    ALIGN_STK_64(coef, coeff, 2, [32 * 32]);
    PIXEL_RECT(c_dst, 64, 64);
    PIXEL_RECT(a_dst, 64, 64);

    static const uint8_t subsh_iters[5] = { 2, 2, 3, 5, 5 };

    const int w = dav2d_txfm_dimensions[tx].w * 4;
    const int h = dav2d_txfm_dimensions[tx].h * 4;
    const int sw = imin(w, 32), sh = imin(h, 32);
    const int subsh_max = subsh_iters[imax(dav2d_txfm_dimensions[tx].lw,
                                           dav2d_txfm_dimensions[tx].lh)];
    pixel *const u_dst = w == 64 ? a_dst : a_dst + 4;

    declare_func(void, pixel *dst, ptrdiff_t dst_stride, coef *coeff,
                 enum TxfmType txtp, int eob HIGHBD_DECL_SUFFIX);

    /* Always using the largest possible coef_max just results in
     * most of the output being clipped to either 0 or bitdepth_max.
     * Randomize the range a bit to cover more scenarios. */
#if BITDEPTH == 16
    const int bpc = (rnd() & 1) ? 10 : 12;
#else
    const int bpc = 8;
#endif
    const int coef_max = (1 << ((rnd() % (bpc + 5)) + 4)) - 1;
    const int bitdepth_max = (1 << bpc) - 1;

    for (int txtp_idx = 0; valid_txtp_per_txsz[tx][txtp_idx] != 0xff;
         txtp_idx++)
    {
        const enum TxfmType txtp = valid_txtp_per_txsz[tx][txtp_idx];
        const enum Tx1dType hor1d = txtp & 0x7, ver1d = txtp >> 5;
        const int is_inv = ((txtp >> 3) & 0x3) == TX_CLASS_2D_INV;
        const int n_dpcm =
            1 + 2 * (txtp == WHT_WHT || (hor1d == IDENTITY && ver1d == IDENTITY));
        for (int dpcm = 0; dpcm < n_dpcm; dpcm++)
            for (int subsh = !!txtp; subsh < subsh_max; subsh++)
                if (check_func(c->itxfm_add[tx],
                               "inv_txfm_add_%dx%d_%s_%s%s%s_%d_%dbpc",
                               w, h, itx_1d_names[hor1d], itx_1d_names[ver1d],
                               is_inv ? "_inv" : "", dpcm_names[dpcm], subsh,
                               BITDEPTH))
                {
                    int max_eob;
                    const int eob = generate_coefs(coeff[0], tx, txtp, sw, sh,
                                                   subsh, &max_eob, coef_max);
                    memcpy(coeff[1], coeff[0], sizeof(*coeff));

                    CLEAR_PIXEL_RECT(c_dst);
                    CLEAR_PIXEL_RECT(a_dst);

                    for (int y = 0; y < h; y++)
                        for (int x = 0; x < w; x++)
                            c_dst[y*PXSTRIDE(c_dst_stride) + x] =
                            u_dst[y*PXSTRIDE(a_dst_stride) + x] = rnd() & bitdepth_max;

                    call_ref(c_dst, c_dst_stride, coeff[0], txtp | (dpcm << 8), eob
                             HIGHBD_TAIL_SUFFIX);
                    call_new(u_dst, a_dst_stride, coeff[1], txtp | (dpcm << 8), eob
                             HIGHBD_TAIL_SUFFIX);

                    if (checkasm_check_pixel_padded(c_dst, c_dst_stride,
                                                    u_dst, a_dst_stride,
                                                    w, h, "dst"))
                    {
                        fprintf(stderr, "eob = %d\n", eob);
                    }
                    if (memcmp(coeff[0], coeff[1], sizeof(*coeff)))
                        fail();

                    if ((hor1d <= ADST && ver1d <= ADST) || txtp == WHT_WHT)
                        bench_new(alternate(c_dst, a_dst), a_dst_stride,
                                  alternate(coeff[0], coeff[1]), txtp | (dpcm << 8),
                                  max_eob HIGHBD_TAIL_SUFFIX);
                }
    }
}

static void check_cctx(const Dav2dInvTxfmDSPContext *const c) {
    ALIGN_STK_64(coef, c_coef, 2, [33 * 32]);
    ALIGN_STK_64(coef, a_coef, 2, [33 * 32]);

    declare_func(void, coef *u, coef *v, const int16_t angle[3],
                 size_t sz HIGHBD_DECL_SUFFIX);

    if (check_func(c->cctx, "cctx_%dbpc", BITDEPTH)) {
#if BITDEPTH == 16
        const int bpc = (rnd() & 1) ? 10 : 12;
        const int bitdepth_max = (1 << bpc) - 1;
#else
        const int bpc = 8;
#endif
        const int coef_max = (1 << (bpc + 8)) - 1;
        const int coef_sign = (coef_max + 1) >> 1;
        for (int sz = 4 * 4; sz <= 32 * 32; sz <<= 1)
        {
            const int16_t *const angle = dav2d_cctx_angle[rnd() % 6];
            for (int i = 0; i <= sz; i++) {
                c_coef[0][i] = (rnd() & coef_max) - coef_sign;
                c_coef[1][i] = (rnd() & coef_max) - coef_sign;
            }
            /* +1 to check for buffer overwrite */
            const size_t mem_sz = sizeof(coef) * (sz + 1);
            memcpy(a_coef[0], c_coef[0], mem_sz);
            memcpy(a_coef[1], c_coef[1], mem_sz);

            call_ref(c_coef[0], c_coef[1], angle, sz HIGHBD_TAIL_SUFFIX);
            call_new(a_coef[0], a_coef[1], angle, sz HIGHBD_TAIL_SUFFIX);
            if (memcmp(c_coef[0], a_coef[0], mem_sz) ||
                memcmp(c_coef[1], a_coef[1], mem_sz))
            {
                fail();
            }

            if (sz == 16 * 16)
                bench_new(alternate(c_coef[0], a_coef[0]),
                          alternate(c_coef[1], a_coef[1]),
                          angle, sz HIGHBD_TAIL_SUFFIX);
        }
    }
    report("cctx");
}

void bitfn(checkasm_check_itx)(void) {
    static const uint8_t txfm_size_order[N_RECT_TX_SIZES] = {
        // tx4
        TX_4X4,
        // tx8
        RTX_4X8,   RTX_8X4,   TX_8X8,
        // tx16
        RTX_4X16,  RTX_16X4,  RTX_8X16,  RTX_16X8,  TX_16X16,
        // tx32
        RTX_4X32,  RTX_32X4,  RTX_8X32,  RTX_32X8,  RTX_16X32,
        RTX_32X16, TX_32X32,
        // tx64
        RTX_4X64,  RTX_64X4,  RTX_8X64,  RTX_64X8,  RTX_16X64,
        RTX_64X16, RTX_32X64, RTX_64X32, TX_64X64,
    };

    Dav2dInvTxfmDSPContext c;
    bitfn(dav2d_itx_dsp_init)(&c);

    const uint8_t *txfm = txfm_size_order;
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j <= i * 2; j++)
            check_itxfm_add(&c, *txfm++);
        report("add_tx%d", 4 << i);
    }
    assert(txfm == &txfm_size_order[N_RECT_TX_SIZES]);

    check_cctx(&c);
}
