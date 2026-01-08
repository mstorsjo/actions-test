/*
 * Copyright © 2018, VideoLAN and dav1d authors
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

#include "config.h"

#include <stdlib.h>
#include <string.h>

#include "common/attributes.h"
#include "common/intops.h"

#include "src/mc.h"
#include "src/tables.h"

#if BITDEPTH == 8
#define get_intermediate_bits(bitdepth_max) 4
// Output in interval [-5132, 9212], fits in int16_t as is
#define PREP_BIAS 0
#else
// 4 for 10 bits/component, 2 for 12 bits/component
#define get_intermediate_bits(bitdepth_max) (14 - bitdepth_from_max(bitdepth_max))
// Output in interval [-20588, 36956] (10-bit), [-20602, 36983] (12-bit)
// Subtract a bias to ensure the output fits in int16_t
#define PREP_BIAS 8192
#endif

static NOINLINE void
put_c(pixel *dst, const ptrdiff_t dst_stride,
      const pixel *src, const ptrdiff_t src_stride, const int w, int h)
{
    do {
        pixel_copy(dst, src, w);

        dst += dst_stride;
        src += src_stride;
    } while (--h);
}

static NOINLINE void
prep_c(int16_t *tmp, const ptrdiff_t tmp_stride,
       const pixel *src, const ptrdiff_t src_stride,
       const int w, int h HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    do {
        for (int x = 0; x < w; x++)
            tmp[x] = (src[x] << intermediate_bits) - PREP_BIAS;

        tmp += tmp_stride;
        src += src_stride;
    } while (--h);
}

#define FILTER_8TAP(src, x, F, stride) \
    (F[0] * src[x + -3 * stride] + \
     F[1] * src[x + -2 * stride] + \
     F[2] * src[x + -1 * stride] + \
     F[3] * src[x + +0 * stride] + \
     F[4] * src[x + +1 * stride] + \
     F[5] * src[x + +2 * stride] + \
     F[6] * src[x + +3 * stride] + \
     F[7] * src[x + +4 * stride])

#define FILTER_8TAP2(src, x, F) \
    (F[0] * src[0][x] + \
     F[1] * src[1][x] + \
     F[2] * src[2][x] + \
     F[3] * src[3][x] + \
     F[4] * src[4][x] + \
     F[5] * src[5][x] + \
     F[6] * src[6][x] + \
     F[7] * src[7][x])

#define DAV1D_FILTER_8TAP_RND(src, x, F, stride, sh) \
    ((FILTER_8TAP(src, x, F, stride) + ((1 << (sh)) >> 1)) >> (sh))

#define DAV1D_FILTER_8TAP_RND2(src, x, F, stride, rnd, sh) \
    ((FILTER_8TAP(src, x, F, stride) + (rnd)) >> (sh))

#define DAV1D_FILTER_8TAP_RND3(src, x, F, sh) \
    ((FILTER_8TAP2(src, x, F) + ((1 << (sh)) >> 1)) >> (sh))

#define DAV1D_FILTER_8TAP_CLIP(src, x, F, stride, sh) \
    iclip_pixel(DAV1D_FILTER_8TAP_RND(src, x, F, stride, sh))

#define DAV1D_FILTER_8TAP_CLIP2(src, x, F, stride, rnd, sh) \
    iclip_pixel(DAV1D_FILTER_8TAP_RND2(src, x, F, stride, rnd, sh))

#define DAV1D_FILTER_8TAP_CLIP3(src, x, F, sh) \
    iclip_pixel(DAV1D_FILTER_8TAP_RND3(src, x, F, sh))

#define GET_H_FILTER(mx) \
    const int8_t *const fh = !(mx) ? NULL : w > 4 ? \
        dav1d_mc_subpel_filters[filter_type][(mx) - 1] : \
        dav1d_mc_subpel_filters[3 + (filter_type & 1)][(mx) - 1]

#define GET_V_FILTER(my) \
    const int8_t *const fv = !(my) ? NULL : h > 4 ? \
        dav1d_mc_subpel_filters[filter_type][(my) - 1] : \
        dav1d_mc_subpel_filters[3 + (filter_type & 1)][(my) - 1]

#define GET_FILTERS() \
    GET_H_FILTER(mx); \
    GET_V_FILTER(my)

static NOINLINE void
put_8tap_c(pixel *dst, ptrdiff_t dst_stride,
           const pixel *src, ptrdiff_t src_stride,
           const int w, int h, const int mx, const int my,
           const int filter_type HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    const int intermediate_rnd = 32 + ((1 << (6 - intermediate_bits)) >> 1);

    GET_FILTERS();
    dst_stride = PXSTRIDE(dst_stride);
    src_stride = PXSTRIDE(src_stride);

    assert(!(w & (w - 1)) && w >= 2 && w <= 64); // w2/h2 used by sub8x8 chroma?
    assert(!(h & (h - 1)) && h >= 2 && h <= 64);

    if (fh) {
        if (fv) {
            int tmp_h = h + 7;
            int16_t mid[64 * (64 + 7)], *mid_ptr = mid;

            src -= src_stride * 3;
            do {
                for (int x = 0; x < w; x++)
                    mid_ptr[x] = DAV1D_FILTER_8TAP_RND(src, x, fh, 1,
                                                       6 - intermediate_bits);

                mid_ptr += 64;
                src += src_stride;
            } while (--tmp_h);

            mid_ptr = mid + 64 * 3;
            do {
                for (int x = 0; x < w; x++)
                    dst[x] = DAV1D_FILTER_8TAP_CLIP(mid_ptr, x, fv, 64,
                                                    6 + intermediate_bits);

                mid_ptr += 64;
                dst += dst_stride;
            } while (--h);
        } else {
            do {
                for (int x = 0; x < w; x++) {
                    dst[x] = DAV1D_FILTER_8TAP_CLIP2(src, x, fh, 1,
                                                     intermediate_rnd, 6);
                }

                dst += dst_stride;
                src += src_stride;
            } while (--h);
        }
    } else if (fv) {
        do {
            for (int x = 0; x < w; x++)
                dst[x] = DAV1D_FILTER_8TAP_CLIP(src, x, fv, src_stride, 6);

            dst += dst_stride;
            src += src_stride;
        } while (--h);
    } else
        put_c(dst, dst_stride, src, src_stride, w, h);
}

static NOINLINE void
put_8tap_scaled_c(pixel *dst, const ptrdiff_t dst_stride,
                  const pixel *src, ptrdiff_t src_stride,
                  const int w, int h, const int mx, int my,
                  const int dx, const int dy, const int filter_type
                  HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    const int intermediate_rnd = (1 << intermediate_bits) >> 1;
    int16_t mid[8][64];
    int16_t *mid_ptrs[8];
    int in_y = -8;
    src_stride = PXSTRIDE(src_stride);

    assert(!(w & (w - 1)) && w >= 2 && w <= 64);
    assert(!(h & (h - 1)) && h >= 2 && h <= 64);

    for (int i = 0; i < 8; i++)
        mid_ptrs[i] = mid[i];

    src -= src_stride * 3;

    for (int y = 0; y < h; y++) {
        int x;
        int src_y = my >> 10;
        GET_V_FILTER((my & 0x3ff) >> 6);

        while (in_y < src_y) {
            int imx = mx, ioff = 0;
            int16_t *mid_ptr = mid_ptrs[0];

            for (int i = 0; i < 7; i++)
                mid_ptrs[i] = mid_ptrs[i + 1];
            mid_ptrs[7] = mid_ptr;

            for (x = 0; x < w; x++) {
                GET_H_FILTER(imx >> 6);
                mid_ptr[x] = fh ? DAV1D_FILTER_8TAP_RND(src, ioff, fh, 1,
                                                        6 - intermediate_bits) :
                                  src[ioff] << intermediate_bits;
                imx += dx;
                ioff += imx >> 10;
                imx &= 0x3ff;
            }

            src += src_stride;
            in_y++;
        }

        for (x = 0; x < w; x++)
            dst[x] = fv ? DAV1D_FILTER_8TAP_CLIP3(mid_ptrs, x, fv,
                                                  6 + intermediate_bits) :
                          iclip_pixel((mid_ptrs[3][x] + intermediate_rnd) >>
                                              intermediate_bits);

        my += dy;
        dst += PXSTRIDE(dst_stride);
    }
}

static NOINLINE void
prep_8tap_c(int16_t *tmp, const ptrdiff_t tmp_stride,
            const pixel *src, ptrdiff_t src_stride,
            const int w, int h, const int mx, const int my,
            const int filter_type HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    GET_FILTERS();
    src_stride = PXSTRIDE(src_stride);

    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    if (fh) {
        if (fv) {
            int tmp_h = h + 7;
            int16_t mid[64 * (64 + 7)], *mid_ptr = mid;

            src -= src_stride * 3;
            do {
                for (int x = 0; x < w; x++)
                    mid_ptr[x] = DAV1D_FILTER_8TAP_RND(src, x, fh, 1,
                                                       6 - intermediate_bits);

                mid_ptr += 64;
                src += src_stride;
            } while (--tmp_h);

            mid_ptr = mid + 64 * 3;
            do {
                for (int x = 0; x < w; x++) {
                    int t = DAV1D_FILTER_8TAP_RND(mid_ptr, x, fv, 64, 6) -
                                  PREP_BIAS;
                    assert(t >= INT16_MIN && t <= INT16_MAX);
                    tmp[x] = t;
                }

                mid_ptr += 64;
                tmp += tmp_stride;
            } while (--h);
        } else {
            do {
                for (int x = 0; x < w; x++)
                    tmp[x] = DAV1D_FILTER_8TAP_RND(src, x, fh, 1,
                                                   6 - intermediate_bits) -
                             PREP_BIAS;

                tmp += tmp_stride;
                src += src_stride;
            } while (--h);
        }
    } else if (fv) {
        do {
            for (int x = 0; x < w; x++)
                tmp[x] = DAV1D_FILTER_8TAP_RND(src, x, fv, src_stride,
                                               6 - intermediate_bits) -
                         PREP_BIAS;

            tmp += tmp_stride;
            src += src_stride;
        } while (--h);
    } else
        prep_c(tmp, tmp_stride, src, src_stride, w, h HIGHBD_TAIL_SUFFIX);
}

static NOINLINE void
prep_8tap_scaled_c(int16_t *tmp, const ptrdiff_t tmp_stride,
                   const pixel *src, ptrdiff_t src_stride,
                   const int w, int h, const int mx, int my,
                   const int dx, const int dy, const int filter_type
                   HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    int16_t mid[8][64];
    int16_t *mid_ptrs[8];
    int in_y = -8;
    src_stride = PXSTRIDE(src_stride);

    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    for (int i = 0; i < 8; i++)
        mid_ptrs[i] = mid[i];

    src -= src_stride * 3;

    for (int y = 0; y < h; y++) {
        int x;
        int src_y = my >> 10;
        GET_V_FILTER((my & 0x3ff) >> 6);

        while (in_y < src_y) {
            int imx = mx, ioff = 0;
            int16_t *mid_ptr = mid_ptrs[0];

            for (int i = 0; i < 7; i++)
                mid_ptrs[i] = mid_ptrs[i + 1];
            mid_ptrs[7] = mid_ptr;

            for (x = 0; x < w; x++) {
                GET_H_FILTER(imx >> 6);
                mid_ptr[x] = fh ? DAV1D_FILTER_8TAP_RND(src, ioff, fh, 1,
                                                        6 - intermediate_bits) :
                                  src[ioff] << intermediate_bits;
                imx += dx;
                ioff += imx >> 10;
                imx &= 0x3ff;
            }

            src += src_stride;
            in_y++;
        }

        for (x = 0; x < w; x++)
            tmp[x] = (fv ? DAV1D_FILTER_8TAP_RND3(mid_ptrs, x, fv, 6)
                         : mid_ptrs[3][x]) - PREP_BIAS;

        my += dy;
        tmp += tmp_stride;
    }
}

#define filter_fns(name, type) \
static void put_8tap_##name##_c(pixel *const dst, \
                                const ptrdiff_t dst_stride, \
                                const pixel *const src, \
                                const ptrdiff_t src_stride, \
                                const int w, const int h, \
                                const int mx, const int my \
                                HIGHBD_DECL_SUFFIX) \
{ \
    put_8tap_c(dst, dst_stride, src, src_stride, w, h, mx, my, \
               type HIGHBD_TAIL_SUFFIX); \
} \
static void put_8tap_##name##_scaled_c(pixel *const dst, \
                                       const ptrdiff_t dst_stride, \
                                       const pixel *const src, \
                                       const ptrdiff_t src_stride, \
                                       const int w, const int h, \
                                       const int mx, const int my, \
                                       const int dx, const int dy \
                                       HIGHBD_DECL_SUFFIX) \
{ \
    put_8tap_scaled_c(dst, dst_stride, src, src_stride, w, h, mx, my, dx, dy, \
                      type HIGHBD_TAIL_SUFFIX); \
} \
static void prep_8tap_##name##_c(int16_t *const tmp, \
                                 const ptrdiff_t tmp_stride, \
                                 const pixel *const src, \
                                 const ptrdiff_t src_stride, \
                                 const int w, const int h, \
                                 const int mx, const int my \
                                 HIGHBD_DECL_SUFFIX) \
{ \
    prep_8tap_c(tmp, tmp_stride, src, src_stride, w, h, mx, my, \
                type HIGHBD_TAIL_SUFFIX); \
} \
static void prep_8tap_##name##_scaled_c(int16_t *const tmp, \
                                        const ptrdiff_t tmp_stride, \
                                        const pixel *const src, \
                                        const ptrdiff_t src_stride, \
                                        const int w, const int h, \
                                        const int mx, const int my, \
                                        const int dx, const int dy \
                                        HIGHBD_DECL_SUFFIX) \
{ \
    prep_8tap_scaled_c(tmp, tmp_stride, src, src_stride, w, h, mx, my, dx, dy, \
                       type HIGHBD_TAIL_SUFFIX); \
}

filter_fns(regular, DAV1D_FILTER_8TAP_REGULAR)
filter_fns(smooth,  DAV1D_FILTER_8TAP_SMOOTH)
filter_fns(sharp,   DAV1D_FILTER_8TAP_SHARP)

#define FILTER_BILIN(src, x, mxy, stride) \
    (16 * src[x] + ((mxy) * (src[x + stride] - src[x])))

#define FILTER_BILIN_RND(src, x, mxy, stride, sh) \
    ((FILTER_BILIN(src, x, mxy, stride) + ((1 << (sh)) >> 1)) >> (sh))

#define FILTER_BILIN_CLIP(src, x, mxy, stride, sh) \
    iclip_pixel(FILTER_BILIN_RND(src, x, mxy, stride, sh))

#define FILTER_BILIN2(src1, src2, x, mxy) \
    (16 * src1[x] + ((mxy) * (src2[x] - src1[x])))

#define FILTER_BILIN_RND2(src1, src2, x, mxy, sh) \
    ((FILTER_BILIN2(src1, src2, x, mxy) + ((1 << (sh)) >> 1)) >> (sh))

#define FILTER_BILIN_CLIP2(src1, src2, x, mxy, sh) \
    iclip_pixel(FILTER_BILIN_RND2(src1, src2, x, mxy, sh))

static void put_bilin_c(pixel *dst, ptrdiff_t dst_stride,
                        const pixel *src, ptrdiff_t src_stride,
                        const int w, int h, const int mx, const int my
                        HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    const int intermediate_rnd = (1 << intermediate_bits) >> 1;
    dst_stride = PXSTRIDE(dst_stride);
    src_stride = PXSTRIDE(src_stride);

    assert(!(w & (w - 1)) && w >= 2 && w <= 64);
    // h=24 can happen for refinemv slices of height 16 and 8 pixels padding
    assert((!(h & (h - 1)) && h >= 2 && h <= 64) || h == 24);

    if (mx) {
        if (my) {
            int16_t mid[64 * (64 + 7)], *mid_ptr = mid;
            int tmp_h = h + 1;

            do {
                for (int x = 0; x < w; x++)
                    mid_ptr[x] = FILTER_BILIN_RND(src, x, mx, 1,
                                                  4 - intermediate_bits);

                mid_ptr += 64;
                src += src_stride;
            } while (--tmp_h);

            mid_ptr = mid;
            do {
                for (int x = 0; x < w; x++)
                    dst[x] = FILTER_BILIN_CLIP(mid_ptr, x, my, 64,
                                               4 + intermediate_bits);

                mid_ptr += 64;
                dst += dst_stride;
            } while (--h);
        } else {
            do {
                for (int x = 0; x < w; x++) {
                    const int px = FILTER_BILIN_RND(src, x, mx, 1,
                                                    4 - intermediate_bits);
                    dst[x] = iclip_pixel((px + intermediate_rnd) >> intermediate_bits);
                }

                dst += dst_stride;
                src += src_stride;
            } while (--h);
        }
    } else if (my) {
        do {
            for (int x = 0; x < w; x++)
                dst[x] = FILTER_BILIN_CLIP(src, x, my, src_stride, 4);

            dst += dst_stride;
            src += src_stride;
        } while (--h);
    } else
        put_c(dst, dst_stride, src, src_stride, w, h);
}

static void put_bilin_scaled_c(pixel *dst, ptrdiff_t dst_stride,
                               const pixel *src, ptrdiff_t src_stride,
                               const int w, int h, const int mx, int my,
                               const int dx, const int dy
                               HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    int16_t mid[2][64];
    int in_y = -2;

    assert(!(w & (w - 1)) && w >= 2 && w <= 64);
    assert(!(h & (h - 1)) && h >= 2 && h <= 64);

    do {
        int x;
        int y = my >> 10;
        int16_t *mid1 = mid[y & 1];
        int16_t *mid2 = mid[(y & 1) ^ 1];
        int dmy = my & 0x3ff;

        while (in_y < y) {
            int imx = mx, ioff = 0;
            int16_t *mid_ptr = mid[in_y & 1];

            for (x = 0; x < w; x++) {
                mid_ptr[x] = FILTER_BILIN_RND(src, ioff, imx >> 6, 1,
                                              4 - intermediate_bits);
                imx += dx;
                ioff += imx >> 10;
                imx &= 0x3ff;
            }

            src += PXSTRIDE(src_stride);
            in_y++;
        }

        for (x = 0; x < w; x++)
            dst[x] = FILTER_BILIN_CLIP2(mid1, mid2, x, dmy >> 6,
                                       4 + intermediate_bits);

        my += dy;
        dst += PXSTRIDE(dst_stride);
    } while (--h);
}

static void prep_bilin_c(int16_t *tmp, const ptrdiff_t tmp_stride,
                         const pixel *src, ptrdiff_t src_stride,
                         const int w, int h, const int mx, const int my
                         HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    src_stride = PXSTRIDE(src_stride);

    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    if (mx) {
        if (my) {
            int16_t mid[64 * (64 + 1)], *mid_ptr = mid;
            int tmp_h = h + 1;

            do {
                for (int x = 0; x < w; x++)
                    mid_ptr[x] = FILTER_BILIN_RND(src, x, mx, 1,
                                                  4 - intermediate_bits);

                mid_ptr += 64;
                src += src_stride;
            } while (--tmp_h);

            mid_ptr = mid;
            do {
                for (int x = 0; x < w; x++)
                    tmp[x] = FILTER_BILIN_RND(mid_ptr, x, my, 64, 4) -
                             PREP_BIAS;

                mid_ptr += 64;
                tmp += tmp_stride;
            } while (--h);
        } else {
            do {
                for (int x = 0; x < w; x++)
                    tmp[x] = FILTER_BILIN_RND(src, x, mx, 1,
                                              4 - intermediate_bits) -
                             PREP_BIAS;

                tmp += tmp_stride;
                src += src_stride;
            } while (--h);
        }
    } else if (my) {
        do {
            for (int x = 0; x < w; x++)
                tmp[x] = FILTER_BILIN_RND(src, x, my, src_stride,
                                          4 - intermediate_bits) - PREP_BIAS;

            tmp += tmp_stride;
            src += src_stride;
        } while (--h);
    } else
        prep_c(tmp, tmp_stride, src, src_stride, w, h HIGHBD_TAIL_SUFFIX);
}

static void prep_bilin_scaled_c(int16_t *tmp, const ptrdiff_t tmp_stride,
                                const pixel *src, ptrdiff_t src_stride,
                                const int w, int h, const int mx, int my,
                                const int dx, const int dy HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    int16_t mid[2][64];
    int in_y = -2;

    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    do {
        int x;
        int y = my >> 10;
        int16_t *mid1 = mid[y & 1];
        int16_t *mid2 = mid[(y & 1) ^ 1];
        int dmy = my & 0x3ff;

        while (in_y < y) {
            int imx = mx, ioff = 0;
            int16_t *mid_ptr = mid[in_y & 1];

            for (x = 0; x < w; x++) {
                mid_ptr[x] = FILTER_BILIN_RND(src, ioff, imx >> 6, 1,
                                              4 - intermediate_bits);
                imx += dx;
                ioff += imx >> 10;
                imx &= 0x3ff;
            }

            src += PXSTRIDE(src_stride);
            in_y++;
        }

        for (x = 0; x < w; x++)
            tmp[x] = FILTER_BILIN_RND2(mid1, mid2, x, dmy >> 6, 4) - PREP_BIAS;

        my += dy;
        tmp += tmp_stride;
    } while (--h);
}

static void avg_c(pixel *dst, const ptrdiff_t dst_stride,
                  const int16_t *tmp1, const int16_t *tmp2, const int w, int h
                  HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    const int sh = intermediate_bits + 1;
    const int rnd = (1 << intermediate_bits) + PREP_BIAS * 2;

    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    do {
        for (int x = 0; x < w; x++)
            dst[x] = iclip_pixel((tmp1[x] + tmp2[x] + rnd) >> sh);

        tmp1 += w;
        tmp2 += w;
        dst += PXSTRIDE(dst_stride);
    } while (--h);
}

static void w_avg_c(pixel *dst, const ptrdiff_t dst_stride,
                    const int16_t *tmp1, const int16_t *tmp2, const int w, int h,
                    const int weight HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    const int sh = intermediate_bits + 4;
    const int rnd = (8 << intermediate_bits) + PREP_BIAS * 16;

    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    do {
        for (int x = 0; x < w; x++)
            dst[x] = iclip_pixel((tmp1[x] * weight +
                                  tmp2[x] * (16 - weight) + rnd) >> sh);

        tmp1 += w;
        tmp2 += w;
        dst += PXSTRIDE(dst_stride);
    } while (--h);
}

static void mask_c(pixel *dst, const ptrdiff_t dst_stride,
                   const int16_t *tmp1, const int16_t *tmp2, const int w, int h,
                   const uint8_t *mask HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    const int sh = intermediate_bits + 6;
    const int rnd = (32 << intermediate_bits) + PREP_BIAS * 64;

    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    do {
        for (int x = 0; x < w; x++)
            dst[x] = iclip_pixel((tmp1[x] * mask[x] +
                                  tmp2[x] * (64 - mask[x]) + rnd) >> sh);

        tmp1 += w;
        tmp2 += w;
        mask += w;
        dst += PXSTRIDE(dst_stride);
    } while (--h);
}

static void blend_c(pixel *dst, const ptrdiff_t dst_stride, const pixel *tmp,
                    const int w, int h, const uint8_t *mask)
{
    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    do {
        for (int x = 0; x < w; x++)
            dst[x] = ((dst[x] * (64 - mask[x]) + tmp[x] * mask[x]) + 32) >> 6;
        dst += PXSTRIDE(dst_stride);
        tmp += w;
        mask += w;
    } while (--h);
}

static void w_mask_c(pixel *dst, const ptrdiff_t dst_stride,
                     const int16_t *tmp1, const int16_t *tmp2, const int w, int h,
                     uint8_t *mask, const int sign,
                     const int ss_hor, const int ss_ver HIGHBD_DECL_SUFFIX)
{
    // store mask at 2x2 resolution, i.e. store 2x1 sum for even rows,
    // and then load this intermediate to calculate final value for odd rows
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    const int bitdepth = bitdepth_from_max(bitdepth_max);
    const int sh = intermediate_bits + 6;
    const int rnd = (32 << intermediate_bits) + PREP_BIAS * 64;
    const int mask_sh = bitdepth + intermediate_bits - 4;
    const int mask_rnd = 1 << (mask_sh - 5);

    assert(!(w & (w - 1)) && w >= 4 && w <= 64);
    assert(!(h & (h - 1)) && h >= 4 && h <= 64);

    do {
        for (int x = 0; x < w; x++) {
            const int m = imin(38 + ((abs(tmp1[x] - tmp2[x]) + mask_rnd) >> mask_sh), 64);
            dst[x] = iclip_pixel((tmp1[x] * m +
                                  tmp2[x] * (64 - m) + rnd) >> sh);

            if (ss_hor) {
                x++;

                const int n = imin(38 + ((abs(tmp1[x] - tmp2[x]) + mask_rnd) >> mask_sh), 64);
                dst[x] = iclip_pixel((tmp1[x] * n +
                                      tmp2[x] * (64 - n) + rnd) >> sh);

                if (h & ss_ver) {
                    mask[x >> 1] = (m + n + mask[x >> 1] + 2 - sign) >> 2;
                } else if (ss_ver) {
                    mask[x >> 1] = m + n;
                } else {
                    mask[x >> 1] = (m + n + 1 - sign) >> 1;
                }
            } else {
                mask[x] = m;
            }
        }

        tmp1 += w;
        tmp2 += w;
        dst += PXSTRIDE(dst_stride);
        if (!ss_ver || (h & 1)) mask += w >> ss_hor;
    } while (--h);
}

#define w_mask_fns(ssn, ss_hor, ss_ver) \
static void w_mask_##ssn##_c(pixel *const dst, const ptrdiff_t dst_stride, \
                             const int16_t *const tmp1, const int16_t *const tmp2, \
                             const int w, const int h, uint8_t *mask, \
                             const int sign HIGHBD_DECL_SUFFIX) \
{ \
    w_mask_c(dst, dst_stride, tmp1, tmp2, w, h, mask, sign, ss_hor, ss_ver \
             HIGHBD_TAIL_SUFFIX); \
}

w_mask_fns(444, 0, 0);
w_mask_fns(422, 1, 0);
w_mask_fns(420, 1, 1);

#undef w_mask_fns

#define FILTER_WARP_RND(src, x, F, stride, sh) \
    ((F[0] * src[x - 3 * stride] + \
      F[1] * src[x - 2 * stride] + \
      F[2] * src[x - 1 * stride] + \
      F[3] * src[x + 0 * stride] + \
      F[4] * src[x + 1 * stride] + \
      F[5] * src[x + 2 * stride] + \
      F[6] * src[x + 3 * stride] + \
      F[7] * src[x + 4 * stride] + \
      ((1 << (sh)) >> 1)) >> (sh))

#define FILTER_WARP_CLIP(src, x, F, stride, sh) \
    iclip_pixel(FILTER_WARP_RND(src, x, F, stride, sh))

static void warp_affine_8x8_c(pixel *dst, const ptrdiff_t dst_stride,
                              const pixel *src, const ptrdiff_t src_stride,
                              const int16_t *const abcd, int mx, int my
                              HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    int16_t mid[15 * 8], *mid_ptr = mid;

    src -= 3 * PXSTRIDE(src_stride);
    for (int y = 0; y < 15; y++, mx += abcd[1]) {
        for (int x = 0, tmx = mx; x < 8; x++, tmx += abcd[0]) {
            const int8_t *const filter =
                dav1d_mc_warp_filter[3*64 + ((tmx + 512) >> 10)];

            mid_ptr[x] = FILTER_WARP_RND(src, x, filter, 1,
                                         7 - intermediate_bits);
        }
        src += PXSTRIDE(src_stride);
        mid_ptr += 8;
    }

    mid_ptr = &mid[3 * 8];
    for (int y = 0; y < 8; y++, my += abcd[3]) {
        for (int x = 0, tmy = my; x < 8; x++, tmy += abcd[2]) {
            const int8_t *const filter =
                dav1d_mc_warp_filter[3*64 + ((tmy + 512) >> 10)];

            dst[x] = FILTER_WARP_CLIP(mid_ptr, x, filter, 8,
                                      7 + intermediate_bits);
        }
        mid_ptr += 8;
        dst += PXSTRIDE(dst_stride);
    }
}

static void warp_affine_8x8t_c(int16_t *tmp, const ptrdiff_t tmp_stride,
                               const pixel *src, const ptrdiff_t src_stride,
                               const int16_t *const abcd, int mx, int my
                               HIGHBD_DECL_SUFFIX)
{
    const int intermediate_bits = get_intermediate_bits(bitdepth_max);
    int16_t mid[15 * 8], *mid_ptr = mid;

    src -= 3 * PXSTRIDE(src_stride);
    for (int y = 0; y < 15; y++, mx += abcd[1]) {
        for (int x = 0, tmx = mx; x < 8; x++, tmx += abcd[0]) {
            const int8_t *const filter =
                dav1d_mc_warp_filter[64*3 + ((tmx + 512) >> 10)];

            mid_ptr[x] = FILTER_WARP_RND(src, x, filter, 1,
                                         7 - intermediate_bits);
        }
        src += PXSTRIDE(src_stride);
        mid_ptr += 8;
    }

    mid_ptr = &mid[3 * 8];
    for (int y = 0; y < 8; y++, my += abcd[3]) {
        for (int x = 0, tmy = my; x < 8; x++, tmy += abcd[2]) {
            const int8_t *const filter =
                dav1d_mc_warp_filter[64*3 + ((tmy + 512) >> 10)];

            tmp[x] = FILTER_WARP_RND(mid_ptr, x, filter, 8, 7) - PREP_BIAS;
        }
        mid_ptr += 8;
        tmp += tmp_stride;
    }
}

static void emu_edge_c(const intptr_t bw, const intptr_t bh,
                       const intptr_t iw, const intptr_t ih,
                       const intptr_t x, const intptr_t y,
                       pixel *dst, const ptrdiff_t dst_stride,
                       const pixel *ref, const ptrdiff_t ref_stride)
{
    // find offset in reference of visible block to copy
    ref += iclip((int) y, 0, (int) ih - 1) * PXSTRIDE(ref_stride) +
           iclip((int) x, 0, (int) iw - 1);

    // number of pixels to extend (left, right, top, bottom)
    const int left_ext = iclip((int) -x, 0, (int) bw - 1);
    const int right_ext = iclip((int) (x + bw - iw), 0, (int) bw - 1);
    assert(left_ext + right_ext < bw);
    const int top_ext = iclip((int) -y, 0, (int) bh - 1);
    const int bottom_ext = iclip((int) (y + bh - ih), 0, (int) bh - 1);
    assert(top_ext + bottom_ext < bh);

    // copy visible portion first
    pixel *blk = dst + top_ext * PXSTRIDE(dst_stride);
    const int center_w = (int) (bw - left_ext - right_ext);
    const int center_h = (int) (bh - top_ext - bottom_ext);
    for (int y = 0; y < center_h; y++) {
        pixel_copy(blk + left_ext, ref, center_w);
        // extend left edge for this line
        if (left_ext)
            pixel_set(blk, blk[left_ext], left_ext);
        // extend right edge for this line
        if (right_ext)
            pixel_set(blk + left_ext + center_w, blk[left_ext + center_w - 1],
                      right_ext);
        ref += PXSTRIDE(ref_stride);
        blk += PXSTRIDE(dst_stride);
    }

    // copy top
    blk = dst + top_ext * PXSTRIDE(dst_stride);
    for (int y = 0; y < top_ext; y++) {
        pixel_copy(dst, blk, bw);
        dst += PXSTRIDE(dst_stride);
    }

    // copy bottom
    dst += center_h * PXSTRIDE(dst_stride);
    for (int y = 0; y < bottom_ext; y++) {
        pixel_copy(dst, &dst[-PXSTRIDE(dst_stride)], bw);
        dst += PXSTRIDE(dst_stride);
    }
}

static void resize_c(pixel *dst, const ptrdiff_t dst_stride,
                     const pixel *src, const ptrdiff_t src_stride,
                     const int dst_w, int h, const int src_w,
                     const int dx, const int mx0 HIGHBD_DECL_SUFFIX)
{
    do {
        int mx = mx0, src_x = -1;
        for (int x = 0; x < dst_w; x++) {
            const int8_t *const F = dav1d_resize_filter[mx >> 8];
            dst[x] = iclip_pixel((-(F[0] * src[iclip(src_x - 3, 0, src_w - 1)] +
                                    F[1] * src[iclip(src_x - 2, 0, src_w - 1)] +
                                    F[2] * src[iclip(src_x - 1, 0, src_w - 1)] +
                                    F[3] * src[iclip(src_x + 0, 0, src_w - 1)] +
                                    F[4] * src[iclip(src_x + 1, 0, src_w - 1)] +
                                    F[5] * src[iclip(src_x + 2, 0, src_w - 1)] +
                                    F[6] * src[iclip(src_x + 3, 0, src_w - 1)] +
                                    F[7] * src[iclip(src_x + 4, 0, src_w - 1)]) +
                                  64) >> 7);
            mx += dx;
            src_x += mx >> 14;
            mx &= 0x3fff;
        }

        dst += PXSTRIDE(dst_stride);
        src += PXSTRIDE(src_stride);
    } while (--h);
}

static void morph_c(pixel *dst, const ptrdiff_t dst_stride,
                    const int alpha, const int beta,
                    const int w, const int h HIGHBD_DECL_SUFFIX)
{
    assert(!(w & (w - 1)) && !(h & (h - 1)));
    assert(w >= 4 && w <= 64 && h >= 4 && h <= 64);
    assert(alpha > -512 && alpha < 512);

    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            dst[x] = iclip_pixel((alpha * dst[x] + beta) >> 8);
        }
        dst += PXSTRIDE(dst_stride);
    }
}

static int sad_nxn(const pixel *p0, const ptrdiff_t p0_stride,
                   const pixel *p1, const ptrdiff_t p1_stride,
                   const int w, const int h,
                   const int emulate_l0a, const int emulate_l0b)
{
    int sad = 0;
    for (int y = 0; y < h; y += 2) {
        for (int x = 0, e0 = emulate_l0a, e1 = emulate_l0b; x < w; x++, e0 = e1 = 0) {
            sad += abs(p0[x + e0] - p1[x + e1]);
        }
        p0 += PXSTRIDE(p0_stride) * 2;
        p1 += PXSTRIDE(p1_stride) * 2;
    }
    return sad;
}

static void sad_refine_mv_c(const pixel *p0, const ptrdiff_t p0_stride,
                            const pixel *p1, const ptrdiff_t p1_stride,
                            const int w, const int h, const int is_implicit,
                            struct OpflOffset *o HIGHBD_DECL_SUFFIX)
{
    const int bd_min8 = bitdepth_from_max(bitdepth_max) - 8;

    assert(w >= 8 && w <= 64 && !(w & (w - 1)));
    assert(h == 8 || h == 16);
    assert(w * h >= 64);

    const int bw = imin(w, 16);
    const int sadw = bw + 4, sadh = h + 4;
    const unsigned sad_thr = sadw * sadh * 2 << bd_min8;
    for (int x = 0; x < w; x += bw, o++) {
        unsigned best_sad = ~0U;
        int best_dx = 0, best_dy = 0;
        if (is_implicit) {
            best_sad = sad_nxn(&p0[2 * PXSTRIDE(p0_stride) + 2], p0_stride,
                               &p1[2 * PXSTRIDE(p1_stride) + 2], p1_stride,
                               sadw, sadh, 0, 0);
            best_sad = (best_sad * 7 + 7) >> 3;
            if (best_sad < sad_thr) goto next;
        }
        for (int y_off = -2; y_off <= 2; y_off++) {
            for (int x_off = -2; x_off <= 2; x_off++) {
                if (!(x_off | y_off)) continue;
                const unsigned sad =
                    sad_nxn(&p0[(2 + y_off) * PXSTRIDE(p0_stride) + (2 + x_off)],
                            p0_stride,
                            &p1[(2 - y_off) * PXSTRIDE(p1_stride) + (2 - x_off)],
                            p1_stride, sadw, sadh, x_off == -2, x_off == 2);
                if (sad >= best_sad) continue;
                best_sad = sad;
                best_dx = x_off;
                best_dy = y_off;
            }
        }
    next:
        o->x = best_dx;
        o->y = best_dy;
        assert(best_sad != ~0U);
        p0 += bw;
        p1 += bw;
    }
}

static void opfl_derive_mv_c(struct OpflRegressionData *out,
                             const pixel *p0, const ptrdiff_t p0_stride,
                             const pixel *p1, const ptrdiff_t p1_stride,
                             const int w, const int h, const int bs,
                             const struct OpflOffset *o, const int8_t d[2]
                             HIGHBD_DECL_SUFFIX)
{
#if BITDEPTH != 8
    const int bd_min8 = bitdepth_from_max(bitdepth_max) - 8;
    const int rnd = (1 << bd_min8) >> 1;
#endif

    assert(bs == 4 || bs == 8);
    assert(bs == 8 || (w == 8 && h == 8));
    assert(!(w & (w - 1)));
    assert(h == 8 || h == 16);
    assert(w >= 8 && w <= 64);

    // distance-weighted pixel difference & regular pixel difference
    int16_t tmp0[64 * 16], tmp1[64 * 16];
    for (int bx = 0; bx < w; bx += 16, o++) {
        const int x_end = imin(bx + 16, w);
        const pixel *p0p = &p0[+o->y * PXSTRIDE(p0_stride) + o->x];
        const pixel *p1p = &p1[-o->y * PXSTRIDE(p1_stride) - o->x];
        for (int y = 0; y < h; y++) {
            for (int x = bx; x < x_end; x++) {
                const int p0pp = p0p[y * PXSTRIDE(p0_stride) + x];
                const int p1pp = p1p[y * PXSTRIDE(p1_stride) + x];
                const int v = d[0] * p0pp - d[1] * p1pp;
#if BITDEPTH == 8
                tmp0[y * 64 + x] = v;
                tmp1[y * 64 + x] = p0pp - p1pp;
#else
                tmp0[y * 64 + x] = (v + rnd - (v < 0)) >> bd_min8;
                tmp1[y * 64 + x] = (p0pp - p1pp + rnd - (p1pp > p0pp)) >> bd_min8;
#endif
            }
        }
    }

    // subpel gradient in both directions
    int16_t gx0[64 * 16], gy0[64 * 16];
    for (int bx = 0; bx < w; bx += 16, o++) {
        const int x_end = imin(bx + 16, w);
        const int min_x = bx & ~15, max_x = x_end - 1;
        const int min_y = 0, max_y = h - 1;
        for (int y = 0; y < h; y++) {
            for (int x = bx; x < x_end; x++) {
                const int p0 = tmp0[y * 64 + imax(min_x, x - 2)];
                const int p1 = tmp0[y * 64 + imax(min_x, x - 1)];
                const int p2 = tmp0[y * 64 + imin(max_x, x + 1)];
                const int p3 = tmp0[y * 64 + imin(max_x, x + 2)];
                const int e1 = x + 1 > max_x || x - 1 < min_x;
                const int x0 = ((p2 - p1) * 42 + (p3 - p0) * -5) * (1 + e1);
                gx0[y * 64 + x] = (x0 + 63 + (x0 > 0)) >> 7;

                const int q0 = tmp0[imax(min_y, y - 2) * 64 + x];
                const int q1 = tmp0[imax(min_y, y - 1) * 64 + x];
                const int q2 = tmp0[imin(max_y, y + 1) * 64 + x];
                const int q3 = tmp0[imin(max_y, y + 2) * 64 + x];
                const int e2 = y + 1 > max_y || y - 1 < min_y;
                const int y0 = ((q2 - q1) * 42 + (q3 - q0) * -5) * (1 + e2);
                gy0[y * 64 + x] = (y0 + 63 + (y0 > 0)) >> 7;
            }
        }
    }

    // set up regression data for flow-derived sub-pixel offset
    for (int y = 0; y < h; y += bs) {
        for (int x = 0; x < w; x += bs, out++) {
            int su2 = bs * bs, suv = 0, sv2 = bs * bs, suw = 0, svw = 0;
            for (int py = y; py < y + bs; py++) {
                for (int px = x; px < x + bs; px++) {
                    const int u = gx0[py * 64 + px];
                    const int v = gy0[py * 64 + px];
                    const int w = tmp1[py * 64 + px];
                    su2 += u * u;
                    suv += u * v;
                    sv2 += v * v;
                    suw += u * w;
                    svw += v * w;
                }
            }
            out->su2 = su2;
            out->suv = suv;
            out->sv2 = sv2;
            out->suw = suw;
            out->svw = svw;
        }
    }
}

#if HAVE_ASM && 0
#if ARCH_AARCH64 || ARCH_ARM
#include "src/arm/mc.h"
#elif ARCH_LOONGARCH64
#include "src/loongarch/mc.h"
#elif ARCH_PPC64LE
#include "src/ppc/mc.h"
#elif ARCH_RISCV
#include "src/riscv/mc.h"
#elif ARCH_X86
#include "src/x86/mc.h"
#endif
#endif

COLD void bitfn(dav1d_mc_dsp_init)(Dav1dMCDSPContext *const c) {
#define init_mc_fns(type, name) do { \
    c->mc        [type] = put_##name##_c; \
    c->mc_scaled [type] = put_##name##_scaled_c; \
    c->mct       [type] = prep_##name##_c; \
    c->mct_scaled[type] = prep_##name##_scaled_c; \
} while (0)

    init_mc_fns(DAV1D_FILTER_8TAP_REGULAR, 8tap_regular);
    init_mc_fns(DAV1D_FILTER_8TAP_SHARP,   8tap_sharp);
    init_mc_fns(DAV1D_FILTER_8TAP_SMOOTH,  8tap_smooth);
    init_mc_fns(DAV1D_FILTER_BILINEAR,     bilin);

    c->avg      = avg_c;
    c->w_avg    = w_avg_c;
    c->mask     = mask_c;
    c->blend    = blend_c;
    c->w_mask[0] = w_mask_444_c;
    c->w_mask[1] = w_mask_422_c;
    c->w_mask[2] = w_mask_420_c;
    c->warp8x8  = warp_affine_8x8_c;
    c->warp8x8t = warp_affine_8x8t_c;
    c->emu_edge = emu_edge_c;
    c->resize   = resize_c;
    c->morph    = morph_c;
    c->opfl_derive_mv = opfl_derive_mv_c;
    c->sad_refine_mv = sad_refine_mv_c;

#if HAVE_ASM && 0
#if ARCH_AARCH64 || ARCH_ARM
    mc_dsp_init_arm(c);
#elif ARCH_LOONGARCH64
    mc_dsp_init_loongarch(c);
#elif ARCH_PPC64LE
    mc_dsp_init_ppc(c);
#elif ARCH_RISCV
    mc_dsp_init_riscv(c);
#elif ARCH_X86
    mc_dsp_init_x86(c);
#endif
#endif
}
