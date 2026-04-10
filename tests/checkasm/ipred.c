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

static const char *const cfl_pred_type_names[3] = { "cfl_explicit", "cfl_implicit" };
static const char *const cfl_luma_filter_names[3] = { "uniform", "vstrip", "gauss" };
static const char *const layout_names[3] = { "420", "422", "444" };

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

static void check_cfl_pred(Dav2dIntraPredDSPContext *const c) {
    PIXEL_RECT(c_y, 128, 128);
    PIXEL_RECT(c_u, 128, 128);
    PIXEL_RECT(c_v, 128, 128);
    PIXEL_RECT(a_y, 128, 128);
    PIXEL_RECT(a_u, 128, 128);
    PIXEL_RECT(a_v, 128, 128);
    pixel c_top_sb[3 * 128], a_top_sb[3 * 128];

    declare_func(void, pixel *const *ptrs, const ptrdiff_t *stride,
                 int wpad, int hpad, int w, int h, int flags HIGHBD_DECL_SUFFIX);

    for (enum CflType type = CFL_EXPLICIT; type <= CFL_IMPLICIT; type++) {
        for (int layout = 1; layout <= DAV2D_PIXEL_LAYOUT_I444; layout++) {
            const int ss_hor = layout != DAV2D_PIXEL_LAYOUT_I444;
            const int ss_ver = layout == DAV2D_PIXEL_LAYOUT_I420;
            const ptrdiff_t strides[2] = { c_y_stride, c_u_stride >> ss_hor };
            for (int w = 4; w <= 64; w <<= 1)
                for (int padding = 0; padding <= 1; padding++)
                    for (int rng0 = 1; rng0 >= 0; rng0--)
                        for (int flt_type = CFL_FLT_TYPE_UNIFORM;
                             flt_type <= CFL_FLT_TYPE_GAUSS; flt_type++)
                        {
                            if (check_func(c->cfl_pred[type][layout - 1],
                                "%s_%s_w%d_pad%d_%s_%s_%dbpc",
                                cfl_pred_type_names[type], layout_names[layout - 1],
                                w, padding, rng0 ? "uv" : "u|v",
                                cfl_luma_filter_names[flt_type], BITDEPTH))
                            {
                                const int h0 = padding && w < 16 ? 16 : 4;
                                for (int h = h0; h <= 64; h <<= 1) {
#if BITDEPTH == 16
                                    const int bitdepth_max = rnd() & 1 ? 0x3ff : 0xfff;
#else
                                    const int bitdepth_max = 0xff;
#endif
                                    int wpad = 0, hpad = 0;
                                    if (padding) {
                                        wpad = imax(w > 8, rnd() & imax(w / 8 - 1, 0));
                                        hpad = imax(h > 8, rnd() & imax(h / 8 - 1, 0));
                                    }

                                    int flags = flt_type |
                                        (rnd() & (CFL_HAS_TOP | CFL_HAS_LEFT |
                                                  CFL_IS_TOP_SB_EDGE));
                                    if (type == CFL_EXPLICIT) {
                                        const int sign_u = 1 - (rnd() & 2);
                                        const int sign_v = 1 - (rnd() & 2);
                                        int alpha_u = rng0 + (rnd() % (9 - rng0));
                                        rng0 |= !alpha_u;
                                        int alpha_v = rng0 + (rnd() % (9 - rng0));
                                        if (rng0 == 0 && alpha_u)
                                            alpha_v = 0;
                                        alpha_u *= sign_u;
                                        alpha_v *= sign_v;
                                        flags |= (alpha_u << CFL_ALPHA_U_SHIFT) & CFL_ALPHA_U_MASK;
                                        flags |= (alpha_v << CFL_ALPHA_V_SHIFT) & CFL_ALPHA_V_MASK;
                                    }

                                    const int itse = flags & CFL_IS_TOP_SB_EDGE;
                                    pixel *c_ytop = itse ? c_top_sb : c_y - (1 + ss_ver) * strides[0];
                                    pixel *c_utop = itse ? c_top_sb + 128 : c_u - strides[1];
                                    pixel *c_vtop = itse ? c_top_sb + 256 : c_v - strides[1];
                                    pixel *a_ytop = itse ? a_top_sb : a_y - (1 + ss_ver) * strides[0];
                                    pixel *a_utop = itse ? a_top_sb + 128 : a_u - strides[1];
                                    pixel *a_vtop = itse ? a_top_sb + 256 : a_v - strides[1];
                                    pixel *const c_ptrs[6] = { c_ytop, c_utop, c_vtop, c_y, c_u, c_v };
                                    pixel *const a_ptrs[6] = { a_ytop, a_utop, a_vtop, a_y, a_u, a_v };

                                    INIT_PIXEL_RECT(c_y_buf);
                                    INIT_PIXEL_RECT(c_u_buf);
                                    INIT_PIXEL_RECT(c_v_buf);
                                    memcpy(a_y_buf, c_y_buf, c_y_buf_h * c_y_stride);
                                    memcpy(a_u_buf, c_u_buf, c_u_buf_h * c_u_stride);
                                    memcpy(a_v_buf, c_v_buf, c_v_buf_h * c_v_stride);
                                    if (itse) {
                                        INIT_PIXEL_RECT(c_top_sb);
                                        memcpy(a_top_sb, c_top_sb, 3 * 128 * sizeof(pixel));
                                    }

                                    call_ref(c_ptrs, strides, wpad, hpad, w, h, flags HIGHBD_TAIL_SUFFIX);
                                    call_new(a_ptrs, strides, wpad, hpad, w, h, flags HIGHBD_TAIL_SUFFIX);
                                    checkasm_check_pixel_padded(c_u, strides[1],
                                                                a_u, strides[1],
                                                                w, h, "u_dst");
                                    checkasm_check_pixel_padded(c_v, strides[1],
                                                                a_v, strides[1],
                                                                w, h, "v_dst");

                                    if (padding) {
                                        wpad = imax(w / 8 - 1, 0);
                                        hpad = imax(h / 8 - 1, 0);
                                    }
                                    bench_new(a_ptrs, strides, wpad, hpad, w, h,
                                              flags | (CFL_HAS_TOP | CFL_HAS_LEFT)
                                              HIGHBD_TAIL_SUFFIX);
                                }
                            }
                        }
        }
        report("%s", cfl_pred_type_names[type]);
    }
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
    check_cfl_pred(&c);
    check_pal_pred(&c);
}
