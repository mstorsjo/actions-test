/*
 * Copyright © 2018, VideoLAN and dav2d authors
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

#include "tests/checkasm/internal.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "src/deblock.h"
#include "src/tables.h"
#include "src/quantizer.h"

static unsigned deblock_quant_thr(const int hbd, const int qidx) {
    const int qmax = 255 + 48 * hbd;
    return (dav2d_dq_lookup(iclip(qidx, 0, qmax)) + 4) >> (3 + 6);
}

static unsigned deblock_side_thr(const int hbd, const int qidx) {
    const int bitdepth_min_8 = 2 * hbd;
    const int q_ind = iclip(qidx - 24 * bitdepth_min_8, 0, 296 - 1);
    const int side_thr = dav2d_deblock_side_thresholds[q_ind];
    return imax(side_thr + (1 << 4 >> bitdepth_min_8), 0) >> (5 - bitdepth_min_8);
}

// Given a target width and pixel data, check if the deblock filter is
// triggered for at least that width. If not, find the amount you can scale all
// pixels to make that width trigger or almost trigger.
static int check_width(const pixel *const s, const pixel *const t,
                       unsigned q_thr, unsigned side_thr, int target_width,
                       int edge, int is_chroma, double *scale)
{
    unsigned deriv_s, deriv_t;
    unsigned second_derivs_buf[4];
    unsigned *second_deriv = &second_derivs_buf[2];

    int pass = 1;
    *scale = 1.0;

    if (target_width == 0) return pass;

    for (int dist = -2; dist < 2; dist++) {
        deriv_s = abs(s[(dist - 1)] - (s[dist] << 1) + s[(dist + 1)]);
        deriv_t = abs(t[(dist - 1)] - (t[dist] << 1) + t[(dist + 1)]);
        second_deriv[dist] = (deriv_s + deriv_t + 1) >> 1;
    }

#define TEST(val, thr) do { \
        if ((val) > (thr)) { \
            *scale = fmin((double)(thr)/(val), *scale); \
            pass = 0; \
        } \
    } while (0);
    TEST(second_deriv[-2], side_thr);
    TEST(second_deriv[1], side_thr);
    if (target_width == 1) return pass;

    const unsigned side_thr2 = side_thr >> 2;
    TEST(second_deriv[-2], side_thr2);
    TEST(second_deriv[1], side_thr2);
    TEST(second_deriv[-1] + second_deriv[0], q_thr * 4);
    if (target_width == 2) return pass;

    const unsigned side_thr3 = side_thr >> 3;
    TEST(second_deriv[-2], side_thr3);
    TEST(second_deriv[1], side_thr3);
    TEST(second_deriv[-1] + second_deriv[0], q_thr * 3);

    const unsigned end_thr = (side_thr * 3) >> 4;
    // if chroma && !edge
    if (!(is_chroma && edge)) {
        deriv_s = abs(s[-1] - s[-4] - 3 * (s[-1] - s[-2]));
        deriv_t = abs(t[-1] - t[-4] - 3 * (t[-1] - t[-2]));

        TEST(((deriv_s + deriv_t + 1) >> 1), end_thr);
    }
    deriv_s = abs(s[0] - s[3] - 3 * (s[0] - s[1]));
    deriv_t = abs(t[0] - t[3] - 3 * (t[0] - t[1]));

    TEST((deriv_s + deriv_t + 1) >> 1, end_thr);
    if (target_width == 3) return pass;

    const unsigned transition = (second_deriv[-1] + second_deriv[0]) << 4;
    for (int dist = 4; dist <= target_width; dist += 2) {
        const int8_t q_first[5] = { 45, 40, 32 };
        const unsigned q_thr4 = q_thr * q_first[(dist - 4) >> 1];
        const unsigned end_thr4 = (side_thr * dist) >> 4;
        TEST(transition, q_thr4);
        const int dist2 = imin(7, dist);

        // if !(luma && edge && dist2 == 8)
        if (!(!is_chroma && edge && dist2 == 8)) {
            deriv_s = abs(s[-1] - s[-dist2 - 1] - dist2 * (s[-1] - s[-2]));
            deriv_t = abs(t[-1] - t[-dist2 - 1] - dist2 * (t[-1] - t[-2]));
            TEST((deriv_s + deriv_t + 1) >> 1, end_thr4);
        }
        deriv_s = abs(s[0] - s[dist2] - dist2 * (s[0] - s[1]));
        deriv_t = abs(t[0] - t[dist2] - dist2 * (t[0] - t[1]));
        TEST((deriv_s + deriv_t + 1) >> 1, end_thr4);
    }

    return pass;
}

static void init_deblock_border(pixel *const dst, const ptrdiff_t stridea,
                                const ptrdiff_t strideb,
                                int q_thr, int side_thr, int edge,
                                int is_chroma, const int bitdepth_max)
{
    // pixels tested when choosing a filter
    const int tested_pixels[7] = { 0, 1, 2, 3, 4, 6, 7 };
    const int filter_widths[7] = { 0, 1, 2, 3, 4, 6, 8 };
    // number of pixels that are tested on a side of the deblocked edge
    const int max_tested_pixels[7] = { 0, 3, 3, 4, 5, 6, 7 };

    const int filter_width_idx = rnd() % 7;
    pixel s[16], t[16];

    for (int i = 0; i < 16; i++) {
        s[i] = rnd() & bitdepth_max;
        t[i] = rnd() & bitdepth_max;
    }

    double scale;
    if (!check_width(s + 8, t + 8, q_thr, side_thr,
                     filter_widths[filter_width_idx],
                     edge, is_chroma, &scale))
    {
        const int mid = s[8];
        scale *= checkasm_randf();
        int n_tested = max_tested_pixels[filter_width_idx];
        for (int i = 0; i < n_tested; i++) {
            const int off = tested_pixels[i];
            s[off + 8] = iclip_pixel(mid + (int)((s[off + 8] - mid) * scale));
            t[off + 8] = iclip_pixel(mid + (int)((t[off + 8] - mid) * scale));
            if (!edge || off < (is_chroma ? 3 : 7)) {
                s[7 - off] = iclip_pixel(mid +
                                         (int)((s[7 - off] - mid) * scale));
                t[7 - off] = iclip_pixel(mid +
                                         (int)((t[7 - off] - mid) * scale));
            }
        }
    }

    for (int j = 1; j <= 2; j++)
        for (int i = -8; i < 8; i++)
            dst[i * strideb + j * stridea] = rnd() & bitdepth_max;
    for (int i = -8; i < 8; i++) {
        dst[i * strideb + 0 * stridea] = s[i + 8];
        dst[i * strideb + 3 * stridea] = t[i + 8];
    }
}

static void check_deblock_sb(deblock_sb_fn fn, const char *const name,
                             const int n_blks,
                             const int is_chroma, const int dir)
{
    ALIGN_STK_64(pixel, c_dst_mem, 64 * 16,);
    ALIGN_STK_64(pixel, a_dst_mem, 64 * 16,);
    ALIGN_STK_16(pixel, q_thr, 16,);
    ALIGN_STK_16(pixel, side_thr, 16,);

    declare_func(void, pixel *dst, ptrdiff_t dst_stride, const uint16_t *mask,
                 const uint16_t *ll_mask,
                 const pixel *q_thr, const pixel *side_thr, int edge,
                 int w HIGHBD_DECL_SUFFIX);

    pixel *a_dst, *c_dst;
    ptrdiff_t stride;
    int w, h;
    if (dir) {
        a_dst = a_dst_mem + n_blks * 4 * 8;
        c_dst = c_dst_mem + n_blks * 4 * 8;
        w = n_blks * 4;
        h = 16;
    } else {
        a_dst = a_dst_mem + 8;
        c_dst = c_dst_mem + 8;
        w = 16;
        h = n_blks * 4;
    }
    stride = w * sizeof(pixel);

    const int n_strengths = is_chroma ? 3 : 4;
    const int widths[] = { 1, 3, 6, 8 };
    const int chroma_widths[] = { 1, 3, 4 };
    for (int i = 0; i < n_strengths; i++) {
        if (check_func(fn, "%s_w%d_%dbpc", name,
                       is_chroma ? chroma_widths[i] : widths[i], BITDEPTH))
        {
            for (int edge = 0; edge <= 1; edge++) {
#if BITDEPTH == 16
                const int bitdepth_max = rnd() & 1 ? 0x3ff : 0xfff;
#else
                const int bitdepth_max = 0xff;
#endif
                uint16_t ll_mask[2] = { 0 };
                uint16_t vmask[4] = { 0 };
                int hbd = (bitdepth_from_max(bitdepth_max) - 8) >> 1;
                const int qidx_max = 255 + 48 * hbd;

                for (int j = 0; j < n_blks; j++) {
                    const int idx = rnd() % (i + 2);
                    if (idx) vmask[idx - 1] |= 1U << j;
                    int qidx = rnd() % (qidx_max + 1);
                    q_thr[j] = deblock_quant_thr(hbd, qidx);
                    side_thr[j] = deblock_side_thr(hbd, qidx);
                }
                ll_mask[0] = rnd() & 0xffff;
                ll_mask[1] = rnd() & 0xffff;

                for (int x = 0; x < n_blks; x++) {
                    const ptrdiff_t stridea = dir ? 1 : 16;
                    const ptrdiff_t strideb = dir ? n_blks * 4 : 1;
                    init_deblock_border(c_dst + 4 * x * stridea,
                                        stridea, strideb,
                                        q_thr[x], side_thr[x],
                                        edge, is_chroma, bitdepth_max);
                }
                memcpy(a_dst_mem, c_dst_mem, 64 * sizeof(pixel) * 16);

                call_ref(c_dst, stride, vmask, ll_mask, q_thr, side_thr, edge,
                         n_blks HIGHBD_TAIL_SUFFIX);
                call_new(a_dst, stride, vmask, ll_mask, q_thr, side_thr, edge,
                         n_blks HIGHBD_TAIL_SUFFIX);

                if (checkasm_check_pixel(c_dst_mem, stride,
                                         a_dst_mem, stride,
                                         w, h, "dst"))
                {
                    fprintf(stderr, "edge = %d\n", edge);
                }
                bench_new(alternate(c_dst, a_dst), stride, vmask, ll_mask,
                          q_thr, side_thr, edge, n_blks HIGHBD_TAIL_SUFFIX);
            }
        }
    }
    report(name);
}

void bitfn(checkasm_check_deblock)(void) {
    Dav2dDeblockDSPContext c;

    bitfn(dav2d_deblock_dsp_init)(&c);

    check_deblock_sb(c.deblock_sb[0][0], "deblock_h_sb_y", 16, 0, 0);
    check_deblock_sb(c.deblock_sb[0][1], "deblock_v_sb_y", 16, 0, 1);
    check_deblock_sb(c.deblock_sb[1][0], "deblock_h_sb_uv", 8, 1, 0);
    check_deblock_sb(c.deblock_sb[1][1], "deblock_v_sb_uv", 8, 1, 1);
}
