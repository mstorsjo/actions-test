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

#include "config.h"

#include <stdlib.h>

#include "common/attributes.h"
#include "common/intops.h"

#include "src/deblock.h"

static const int8_t max_width_y[4] = { 1, 3, 6, 8 };
static const int8_t max_width_uv[3] = { 1, 3, 4 };

static const int8_t q_first[5] = { 45, 40, 32 };
static const int8_t q_thresh_mults[8] = { 32, 25, 19, 19, 0, 18, 0, 17 };
static const int8_t w_mult[8] = { 85, 51, 37, 28, 0, 20, 0, 15 };

static int filter_choice(const pixel *const s, const pixel *const t, const ptrdiff_t stride,
                         const int max_width_neg, const int max_width_pos,
                         unsigned q_thr, unsigned side_thr) {
    unsigned deriv_s, deriv_t;
    unsigned second_derivs_buf[4];
    unsigned *second_deriv = &second_derivs_buf[2];

    for (int dist = -2; dist < 2; dist++) {
        deriv_s = abs(s[(dist - 1) * stride] - (s[dist * stride] << 1) + s[(dist + 1) * stride]);
        deriv_t = abs(t[(dist - 1) * stride] - (t[dist * stride] << 1) + t[(dist + 1) * stride]);
        second_deriv[dist] = (deriv_s + deriv_t + 1) >> 1;
    }

    if (second_deriv[-2] > side_thr || second_deriv[1] > side_thr) return 0;
    if (max_width_pos == 1) return 1;

    const unsigned side_thr2 = side_thr >> 2;
    if (second_deriv[-2] > side_thr2 || second_deriv[1] > side_thr2) return 1;
    if (second_deriv[-1] + second_deriv[0] > q_thr * 4) return 1;

    const unsigned side_thr3 = side_thr >> 3;
    if (second_deriv[-2] > side_thr3 || second_deriv[1] > side_thr3) return 2;
    if (second_deriv[-1] + second_deriv[0] > q_thr * 3) return 2;

    const unsigned end_thr = (side_thr * 3) >> 4;
    if (max_width_neg > 2) {
        deriv_s = abs(s[-1 * stride] - s[-4 * stride] - 3 * (s[-1 * stride] - s[-2 * stride]));
        deriv_t = abs(t[-1 * stride] - t[-4 * stride] - 3 * (t[-1 * stride] - t[-2 * stride]));
        if (((deriv_s + deriv_t + 1) >> 1) > end_thr) return 2;
    }
    deriv_s = abs(s[0] - s[3 * stride] - 3 * (s[0] - s[stride]));
    deriv_t = abs(t[0] - t[3 * stride] - 3 * (t[0] - t[stride]));
    if (((deriv_s + deriv_t + 1) >> 1) > end_thr) return 2;
    if (max_width_pos == 3) return 3;

    const unsigned transition = (second_deriv[-1] + second_deriv[0]) << 4;
    int prev_dist = 3;
    for (int dist = 4; dist <= max_width_pos; dist += 2) {
        const unsigned q_thr4 = q_thr * q_first[(dist - 4) >> 1];
        const unsigned end_thr4 = (side_thr * dist) >> 4;
        if (transition > q_thr4) return prev_dist;
        const int dist2 = imin(7, dist);
        if (max_width_neg >= dist2) {
            deriv_s = abs(s[-stride] - s[(-dist2 - 1) * stride] - dist2 * (s[-stride] - s[-2 * stride]));
            deriv_t = abs(t[-stride] - t[(-dist2 - 1) * stride] - dist2 * (t[-stride] - t[-2 * stride]));
            if (((deriv_s + deriv_t + 1) >> 1) > end_thr4) return prev_dist;
        }
        deriv_s = abs(s[0] - s[dist2 * stride] - dist2 * (s[0] - s[stride]));
        deriv_t = abs(t[0] - t[dist2 * stride] - dist2 * (t[0] - t[stride]));
        if (((deriv_s + deriv_t + 1) >> 1) > end_thr4) return prev_dist;
        prev_dist = dist;
    }

    return max_width_pos;
}

static NOINLINE void
deblock(pixel *dst, unsigned q_thr, unsigned side_thr,
        const ptrdiff_t stridea, const ptrdiff_t strideb,
        const int max_width_pos, const int max_width_neg
        HIGHBD_DECL_SUFFIX)
{
    const int width = filter_choice(dst, dst + 3 * stridea, strideb, max_width_neg, max_width_pos, q_thr, side_thr);
    const int width_neg = imin(width, max_width_neg);
    const int width_pos = width;

    if (width_pos < 1) return;

    const int q_thr_clamp = q_thr * q_thresh_mults[width - 1];
    for (int i = 0; i < 4; i++, dst += stridea) {
        int delta_m2 = iclip(4 * (3 * (dst[0] - dst[-1 * strideb]) - (dst[strideb] - dst[-2 * strideb])), -q_thr_clamp, q_thr_clamp);
        int delta_m2_neg = delta_m2 * w_mult[width_neg - 1];
        for (int j = 0; j < width_neg; j++) {
            pixel *dst_pix = &dst[(-j - 1) * strideb];
            *dst_pix = iclip(*dst_pix + ((delta_m2_neg * (width_neg - j) + (1 << 10)) >> 11), 0, BITDEPTH_MAX);
        }

        int delta_m2_pos = delta_m2 * w_mult[width_pos - 1];
        for (int j = 0; j < width_pos; j++) {
            pixel *dst_pix = &dst[j * strideb];
            *dst_pix = iclip(*dst_pix - ((delta_m2_pos * (width_pos - j) + (1 << 10)) >> 11), 0, BITDEPTH_MAX);
        }
    }
}

static void deblock_h_sb64y_c(pixel *dst, const ptrdiff_t stride,
                              const uint16_t *const vmask,
                              const pixel *q_thr,
                              const pixel *side_thr,
                              const int edge,
                              const int h
                              HIGHBD_DECL_SUFFIX)
{
    const unsigned vm = vmask[0] | vmask[1] | vmask[2] | vmask[3];
    for (unsigned y = 1; vm & ~(y - 1);
         y <<= 1, dst += 4 * PXSTRIDE(stride), q_thr++, side_thr++)
    {
        if (vm & y) {
            const int idx = (vmask[3] & y) ? 3 : (vmask[2] & y) ? 2 : !!(vmask[1] & y);
            const int max_width_pos = max_width_y[idx];
            const int max_width_neg = max_width_y[edge ? imin(idx, 2) : idx];
            const int is_sub_pu = !!(vmask[4] & y) * 3;
            deblock(dst, *q_thr >> is_sub_pu, *side_thr >> is_sub_pu, PXSTRIDE(stride), 1,
                        max_width_pos, max_width_neg HIGHBD_TAIL_SUFFIX);
        }
    }
}

static void deblock_v_sb64y_c(pixel *dst, const ptrdiff_t stride,
                              const uint16_t *const vmask,
                              const pixel *q_thr,
                              const pixel *side_thr,
                              const int edge,
                              const int w
                              HIGHBD_DECL_SUFFIX)
{
    const unsigned vm = vmask[0] | vmask[1] | vmask[2] | vmask[3];
    for (unsigned x = 1; vm & ~(x - 1);
         x <<= 1, dst += 4, q_thr++, side_thr++)
    {
        if (vm & x) {
            const int idx = (vmask[3] & x) ? 3 : (vmask[2] & x) ? 2 : !!(vmask[1] & x);
            const int max_width_pos = max_width_y[idx];
            const int max_width_neg = max_width_y[edge ? imin(idx, 2) : idx];
            const int is_sub_pu = !!(vmask[4] & x) * 3;
            deblock(dst, *q_thr >> is_sub_pu, *side_thr >> is_sub_pu, 1, PXSTRIDE(stride),
                        max_width_pos, max_width_neg HIGHBD_TAIL_SUFFIX);
        }
    }
}

static void deblock_h_sb64uv_c(pixel *dst, const ptrdiff_t stride,
                               const uint16_t *const vmask,
                               const pixel *q_thr,
                               const pixel *side_thr,
                               const int edge,
                               const int h
                               HIGHBD_DECL_SUFFIX)
{
    const unsigned vm = vmask[0] | vmask[1] | vmask[2];
    for (unsigned y = 1; vm & ~(y - 1);
        y <<= 1, dst += 4 * PXSTRIDE(stride), q_thr++, side_thr++)
    {
        if (vm & y) {
            const int idx = (vmask[2] & y) ? 2 : !!(vmask[1] & y);
            const int max_width_pos = max_width_uv[idx];
            const int max_width_neg = edge ? imin(2, max_width_pos) : max_width_pos;
            const int is_sub_pu = !!(vmask[3] & y) * 3;
            deblock(dst, *q_thr >> is_sub_pu, *side_thr >> is_sub_pu, PXSTRIDE(stride), 1,
                        max_width_pos, max_width_neg HIGHBD_TAIL_SUFFIX);
        }
    }
}

static void deblock_v_sb64uv_c(pixel *dst, const ptrdiff_t stride,
                               const uint16_t *const vmask,
                               const pixel *q_thr,
                               const pixel *side_thr,
                               const int edge,
                               const int w
                               HIGHBD_DECL_SUFFIX)
{
    const unsigned vm = vmask[0] | vmask[1] | vmask[2];
    for (unsigned x = 1; vm & ~(x - 1);
         x <<= 1, dst += 4, q_thr++, side_thr++)
    {
        if (vm & x) {
            const int idx = (vmask[2] & x) ? 2 : !!(vmask[1] & x);
            const int max_width_pos = max_width_uv[idx];
            const int max_width_neg = edge ? imin(2, max_width_pos) : max_width_pos;
            const int is_sub_pu = !!(vmask[3] & x) * 3;
            deblock(dst, *q_thr >> is_sub_pu, *side_thr >> is_sub_pu, 1, PXSTRIDE(stride),
                        max_width_pos, max_width_neg HIGHBD_TAIL_SUFFIX);
        }
    }
}

#if 0
#if HAVE_ASM
#if ARCH_AARCH64 || ARCH_ARM
#include "src/arm/deblock.h"
#elif ARCH_LOONGARCH64
#include "src/loongarch/deblock.h"
#elif ARCH_PPC64LE
#include "src/ppc/deblock.h"
#elif ARCH_X86
#include "src/x86/deblock.h"
#endif
#endif
#endif

COLD void bitfn(dav2d_deblock_dsp_init)(Dav2dDeblockDSPContext *const c) {
    c->deblock_sb[0][0] = deblock_h_sb64y_c;
    c->deblock_sb[0][1] = deblock_v_sb64y_c;
    c->deblock_sb[1][0] = deblock_h_sb64uv_c;
    c->deblock_sb[1][1] = deblock_v_sb64uv_c;

#if 0
#if HAVE_ASM
#if ARCH_AARCH64 || ARCH_ARM
    loop_filter_dsp_init_arm(c);
#elif ARCH_LOONGARCH64
    loop_filter_dsp_init_loongarch(c);
#elif ARCH_PPC64LE
    loop_filter_dsp_init_ppc(c);
#elif ARCH_X86
    loop_filter_dsp_init_x86(c);
#endif
#endif
#endif
}
