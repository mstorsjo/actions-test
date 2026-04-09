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
#include "src/ipred.h"
#include "src/levels.h"

#include <stdio.h>

static const char *const intra_pred_mode_names[N_IMPL_INTRA_PRED_MODES] = {
    [DC_PRED]       = "dc",
    [DC_128_PRED]   = "dc_128",
    [TOP_DC_PRED]   = "dc_top",
    [LEFT_DC_PRED]  = "dc_left",
    [HOR_PRED]      = "h",
    [VERT_PRED]     = "v",
    [PAETH_PRED]    = "paeth",
    [SMOOTH_PRED]   = "smooth",
    [SMOOTH_V_PRED] = "smooth_v",
    [SMOOTH_H_PRED] = "smooth_h",
    [Z1_PRED]       = "z1",
    [Z2_PRED]       = "z2",
    [Z3_PRED]       = "z3",
    [DIP_PRED]      = "dip"
};


static const uint8_t z_angles[27] = {
     3,  6,  9,
    14, 17, 20, 23, 26, 29, 32,
    36, 39, 42, 45, 48, 51, 54,
    58, 61, 64, 67, 70, 73, 76,
    81, 84, 87
};

/* Generate max_width/max_height values that covers all edge cases */
static int gen_z_max_wh(const int sz) {
    const int n = rnd();
    if (n & (1 << 17)) /* edge block */
        return (n & (sz - 1)) + 1;
    if (n & (1 << 16)) /* max size, exceeds uint16_t */
        return 65536;
    return (n & 65535) + 1;
}

static void check_intra_pred(Dav2dIntraPredDSPContext *const c) {
    PIXEL_RECT(c_dst, 64, 64);
    PIXEL_RECT(a_dst, 64, 64);
    ALIGN_STK_64(pixel, topleft_buf, 64 * 8 + 2 * 1 + 2 * 9,);

    int bitdepth_max;
    if (BITDEPTH == 16)
        bitdepth_max = rnd() & 1 ? 0x3ff : 0xfff;
    else
        bitdepth_max = (1 << BITDEPTH) - 1;
    for (int i = 0; i < 64 * 8 + 2 * 1 + 2 * 9; i++)
        topleft_buf[i] = rnd() & bitdepth_max;

    declare_func(void, pixel *dst, ptrdiff_t stride, const pixel *topleft,
                 int width, int height, int angle, int max_width, int max_height
                 HIGHBD_DECL_SUFFIX);

    for (int mode = 0; mode < N_IMPL_INTRA_PRED_MODES; mode++) {
        for (int w = 4; w <= 64; w <<= 1) {
            pixel *const u_dst = w == 64 ? a_dst : a_dst + 4;
            if (check_func(c->intra_pred[mode], "intra_pred_%s_w%d_%dbpc",
                intra_pred_mode_names[mode], w, BITDEPTH))
            {
                for (int h = 4; h <= 64; h <<= 1) {
                    const ptrdiff_t stride = c_dst_stride;
                    int nb_iters = (mode >= Z1_PRED && mode <= Z3_PRED) ? 5 : 1;

                    for (int iter = 0; iter < nb_iters; iter++) {
                        int a = 0, maxw = 0, maxh = 0;
                        if (mode >= Z1_PRED && mode <= Z3_PRED) { /* angle */
                            a = (90 * (mode - Z1_PRED) + z_angles[rnd() % 27]) |
                                (rnd() & 0xffe00);
                            if (imin(w, h) == 4)
                                a &= ~(ANGLE_MULTI_MRL_FLAG | ANGLE_IBP_FLAG);
                            if (a & ANGLE_MULTI_MRL_FLAG) a |= ANGLE_IS_LUMA;
                            maxw = gen_z_max_wh(w);
                            maxh = gen_z_max_wh(h);
                        } else if (mode == DIP_PRED) {
                            /* dip_idx */
                            a = (rnd() % 5) | (rnd() & 16);
                        } else if (mode == HOR_PRED || mode == VERT_PRED) {
                            if (imax(w, h) > 4)
                                a = (rnd() & 1) * ANGLE_MULTI_MRL_FLAG;
                        } else if (mode == DC_PRED || mode == LEFT_DC_PRED ||
                                   mode == TOP_DC_PRED)
                        {
                            if (imax(w, h) > 4)
                                a = (rnd() & 1) * ANGLE_IBP_FLAG;
                        }
                        pixel *const topleft = topleft_buf + 128 + 9;

                        CLEAR_PIXEL_RECT(c_dst);
                        CLEAR_PIXEL_RECT(a_dst);
                        call_ref(c_dst, stride, topleft, w, h, a, maxw, maxh
                                 HIGHBD_TAIL_SUFFIX);
                        call_new(u_dst, stride, topleft, w, h, a, maxw, maxh
                                 HIGHBD_TAIL_SUFFIX);
                        if (checkasm_check_pixel_padded(c_dst, stride,
                                                        u_dst, stride,
                                                        w, h, "dst"))
                        {
                            if (mode >= Z1_PRED && mode <= Z3_PRED)
                                fprintf(stderr, "angle = %d (0x%05x), "
                                        "max_width = %d, max_height = %d\n",
                                        a & 0x1ff, a & 0xffe00, maxw, maxh);
                            else if (mode == DIP_PRED)
                                fprintf(stderr, "dip tp = %d, mode = %d\n", a > 7, a & 7);
                            else if (mode == HOR_PRED || mode == VERT_PRED)
                                fprintf(stderr, "multimrl=%d\n", !!a);
                            else if (mode == DC_PRED || mode == LEFT_DC_PRED ||
                                     mode == TOP_DC_PRED)
                            {
                                fprintf(stderr, "ibp=%d\n", !!a);
                            }
                            break;
                        }

                        bench_new(a_dst, stride, topleft, w, h, a, 128, 128
                                  HIGHBD_TAIL_SUFFIX);
                    }
                }
            }
        }
    }
    report("intra_pred");
}

static void check_pal_pred(Dav2dIntraPredDSPContext *const c) {
    PIXEL_RECT(c_dst, 64, 64);
    PIXEL_RECT(a_dst, 64, 64);
    ALIGN_STK_64(uint8_t, idx, 32 * 64,);
    ALIGN_STK_16(pixel, pal, 8,);

    declare_func(void, pixel *dst, ptrdiff_t stride, const pixel *pal,
                 const uint8_t *idx, int w, int h);

    for (int w = 4; w <= 64; w <<= 1) {
        pixel *const u_dst = w == 64 ? a_dst : a_dst + 4;
        if (check_func(c->pal_pred, "pal_pred_w%d_%dbpc", w, BITDEPTH))
            for (int h = imax(4, 64 / w); h <= 64; h <<= 1) {
#if BITDEPTH == 16
                const int bitdepth_max = rnd() & 1 ? 0x3ff : 0xfff;
#else
                const int bitdepth_max = 0xff;
#endif

                for (int i = 0; i < 8; i++)
                    pal[i] = rnd() & bitdepth_max;

                for (int i = 0; i < w * h / 2; i++)
                    idx[i] = rnd() & 0x77;

                CLEAR_PIXEL_RECT(c_dst);
                CLEAR_PIXEL_RECT(a_dst);

                call_ref(c_dst, c_dst_stride, pal, idx, w, h);
                call_new(u_dst, a_dst_stride, pal, idx, w, h);
                checkasm_check_pixel_padded(c_dst, c_dst_stride,
                                            u_dst, a_dst_stride, w, h, "dst");

                bench_new(a_dst, a_dst_stride, pal, idx, w, h);
            }
    }
    report("pal_pred");
}

void bitfn(checkasm_check_ipred)(void) {
    Dav2dIntraPredDSPContext c;
    bitfn(dav2d_intra_pred_dsp_init)(&c);

    check_intra_pred(&c);
    check_pal_pred(&c);
}
