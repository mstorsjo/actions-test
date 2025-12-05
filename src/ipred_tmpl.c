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

#include "src/dip_tables.h"
#include "src/ibp.h"
#include "src/ipred.h"
#include "src/tables.h"

typedef struct {
    int8_t a;
    uint8_t b, c;
    int8_t d;
} DRFilter4Tap;

static const DRFilter4Tap av1_dr_interp_filter[32] = {
    {   0, 128,   0,   0 },
    {  -2, 127,   4,  -1 },
    {  -3, 125,   8,  -2 },
    {  -5, 123,  13,  -3 },
    {  -6, 121,  17,  -4 },
    {  -7, 118,  22,  -5 },
    {  -9, 116,  27,  -6 },
    {  -9, 112,  32,  -7 },
    { -10, 109,  37,  -8 },
    { -11, 106,  41,  -8 },
    { -11, 102,  46,  -9 },
    { -12,  98,  52, -10 },
    { -12,  94,  56, -10 },
    { -12,  90,  61, -11 },
    { -12,  85,  66, -11 },
    { -12,  81,  71, -12 },
    { -12,  76,  76, -12 },
    { -12,  71,  81, -12 },
    { -11,  66,  85, -12 },
    { -11,  61,  90, -12 },
    { -10,  56,  94, -12 },
    { -10,  52,  98, -12 },
    {  -9,  46, 102, -11 },
    {  -8,  41, 106, -11 },
    {  -8,  37, 109, -10 },
    {  -7,  32, 112,  -9 },
    {  -6,  27, 116,  -9 },
    {  -5,  22, 118,  -7 },
    {  -4,  17, 121,  -6 },
    {  -3,  13, 123,  -5 },
    {  -2,   8, 125,  -3 },
    {  -1,   4, 127,  -2 }
};

static const uint8_t ibp_weights[32] = {
    /* Unused */ 0,
    /* len  1 */ 96,
    /* len  2 */ 86, 107,
    /* len  4 */ 77,  90, 102, 115,
    /* len  8 */ 71,  78,  86,  92, 100, 107, 114, 121,
    /* len 16 */ 68,  72,  76,  79,  83,  87,  90,  94,
                 98, 102, 106, 109, 113, 117, 121, 124
};

static NOINLINE void
splat_dc(pixel *dst, const ptrdiff_t stride,
         const int width, int height, const int dc)
{
    do {
        for (int x = 0; x < width; x++)
            dst[x] = dc;
        dst += PXSTRIDE(stride);
    } while (--height);
}

static NOINLINE void
cfl_pred(pixel *dst, const ptrdiff_t stride,
         const int width, const int height, const int dc,
         const int16_t *ac, const int alpha HIGHBD_DECL_SUFFIX)
{
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const int diff = alpha * ac[x];
            dst[x] = iclip_pixel(dc + apply_sign((abs(diff) + 32) >> 6, diff));
        }
        ac += width;
        dst += PXSTRIDE(stride);
    }
}

static unsigned dc_gen_top(const pixel *const topleft, const int width) {
    unsigned dc = width >> 1;
    for (int i = 0; i < width; i++)
       dc += topleft[1 + i];
    return dc >> ctz(width);
}

static void ipred_dc_top_c(pixel *dst, const ptrdiff_t stride,
                           const pixel *const topleft,
                           const int width, int height, const int a,
                           const int max_width, const int max_height
                           HIGHBD_DECL_SUFFIX)
{
    const unsigned dc = dc_gen_top(topleft, width);

    if (a & ANGLE_IBP_FLAG) {
        const int h = height >> 2;
        const uint8_t *w_y = &ibp_weights[h];
        for (int y = 0; y < h; y++) {
            const int wy = 128 - w_y[y];
            const int dc_wy = dc * w_y[y];
            for (int x = 0; x < width; x++) {
                dst[x] = (topleft[x + 1] * wy + dc_wy + 64) >> 7;
            }
            dst += PXSTRIDE(stride);
        }
        height -= h;
    }

    splat_dc(dst, stride, width, height, dc);
}

static void ipred_cfl_top_c(pixel *dst, const ptrdiff_t stride,
                            const pixel *const topleft,
                            const int width, const int height,
                            const int16_t *ac, const int alpha
                            HIGHBD_DECL_SUFFIX)
{
    cfl_pred(dst, stride, width, height, dc_gen_top(topleft, width), ac, alpha
             HIGHBD_TAIL_SUFFIX);
}

static unsigned dc_gen_left(const pixel *const topleft, const int height) {
    unsigned dc = height >> 1;
    for (int i = 0; i < height; i++)
       dc += topleft[-(1 + i)];
    return dc >> ctz(height);
}

static void ipred_dc_left_c(pixel *dst, const ptrdiff_t stride,
                            const pixel *const topleft,
                            int width, const int height, const int a,
                            const int max_width, const int max_height
                            HIGHBD_DECL_SUFFIX)
{
    const unsigned dc = dc_gen_left(topleft, height);

    if (a & ANGLE_IBP_FLAG) {
        const int w = width >> 2;
        const uint8_t *w_x = &ibp_weights[w];
        for (int y = 0; y < height; y++) {
            const int left = topleft[-(y + 1)];
            for (int x = 0; x < w; x++) {
                dst[x] = (left * (128 - w_x[x]) + dc * w_x[x] + 64) >> 7;
            }
            dst += PXSTRIDE(stride);
        }
        dst -= PXSTRIDE(stride) * height;
        width -= w;
        dst += w;
    }

    splat_dc(dst, stride, width, height, dc);
}

static void ipred_cfl_left_c(pixel *dst, const ptrdiff_t stride,
                             const pixel *const topleft,
                             const int width, const int height,
                             const int16_t *ac, const int alpha
                             HIGHBD_DECL_SUFFIX)
{
    const unsigned dc = dc_gen_left(topleft, height);
    cfl_pred(dst, stride, width, height, dc, ac, alpha HIGHBD_TAIL_SUFFIX);
}

#if BITDEPTH == 8
#define MULTIPLIER_1x2 0x5556
#define MULTIPLIER_1x4 0x3334
#define BASE_SHIFT 16
#else
#define MULTIPLIER_1x2 0xAAAB
#define MULTIPLIER_1x4 0x6667
#define BASE_SHIFT 17
#endif

static inline unsigned fast_div32_dc(const unsigned num, const unsigned den) {
    assert(den > 0 && den <= 255);
    int shift = ulog2(den);
    const int rem = den - (1 << shift);
    const int idx = rem << (7 - shift);
    assert(idx <= 128);
    shift += 9;
    return ((num * dav1d_div_recip[idx]) + ((1 << shift) >> 1)) >> shift;
}

static unsigned dc_gen(const pixel *const topleft,
                       const int width, const int height)
{
    const int n_pel = width + height;
    unsigned dc = 0;
    for (int i = 0; i < width; i++)
       dc += topleft[i + 1];
    for (int i = 0; i < height; i++)
       dc += topleft[-(i + 1)];
    if (width == height)
        return (dc + width) >> ctz(n_pel);

    return fast_div32_dc(dc, n_pel);
}

static void ipred_dc_c(pixel *dst, const ptrdiff_t stride,
                       const pixel *const topleft,
                       int width, int height, const int a,
                       const int max_width, const int max_height
                       HIGHBD_DECL_SUFFIX)
{
    const unsigned dc = dc_gen(topleft, width, height);

    if (a & ANGLE_IBP_FLAG) {
        pixel *const p_dst = dst;
        const int h = height >> 2;
        const int w = width >> 2;
        const int x_start = width < height ? w : 0;
        const uint8_t *const w_y = &ibp_weights[h];
        for (int y = 0; y < h; y++) {
            const int wy = 128 - w_y[y];
            const int dc_wy = dc * w_y[y];
            for (int x = x_start; x < width; x++) {
                dst[x] = (topleft[x + 1] * wy + dc_wy + 64) >> 7;
            }
            dst += PXSTRIDE(stride);
        }

        const int y_start = width >= height ? h : 0;
        dst = p_dst + y_start * PXSTRIDE(stride);
        const uint8_t *const w_x = &ibp_weights[w];
        for (int y = y_start; y < height; y++) {
            const int left = topleft[-(y + 1)];
            for (int x = 0; x < w; x++) {
                dst[x] = (left * (128 - w_x[x]) + dc * w_x[x] + 64) >> 7;
            }
            dst += PXSTRIDE(stride);
        }
        dst = p_dst + (h * PXSTRIDE(stride) + w);
        width -= w;
        height -= h;
    }

    splat_dc(dst, stride, width, height, dc);
}

static void ipred_cfl_c(pixel *dst, const ptrdiff_t stride,
                        const pixel *const topleft,
                        const int width, const int height,
                        const int16_t *ac, const int alpha
                        HIGHBD_DECL_SUFFIX)
{
    unsigned dc = dc_gen(topleft, width, height);
    cfl_pred(dst, stride, width, height, dc, ac, alpha HIGHBD_TAIL_SUFFIX);
}

#undef MULTIPLIER_1x2
#undef MULTIPLIER_1x4
#undef BASE_SHIFT

static void ipred_dc_128_c(pixel *dst, const ptrdiff_t stride,
                           const pixel *const topleft,
                           const int width, const int height, const int a,
                           const int max_width, const int max_height
                           HIGHBD_DECL_SUFFIX)
{
#if BITDEPTH == 16
    const int dc = (bitdepth_max + 1) >> 1;
#else
    const int dc = 128;
#endif
    splat_dc(dst, stride, width, height, dc);
}

static void ipred_cfl_128_c(pixel *dst, const ptrdiff_t stride,
                            const pixel *const topleft,
                            const int width, const int height,
                            const int16_t *ac, const int alpha
                            HIGHBD_DECL_SUFFIX)
{
#if BITDEPTH == 16
    const int dc = (bitdepth_max + 1) >> 1;
#else
    const int dc = 128;
#endif
    cfl_pred(dst, stride, width, height, dc, ac, alpha HIGHBD_TAIL_SUFFIX);
}

static void ipred_v_c(pixel *dst, const ptrdiff_t stride,
                      const pixel *const topleft,
                      const int width, const int height, const int angle,
                      const int max_width, const int max_height
                      HIGHBD_DECL_SUFFIX)
{
    const int mrl_idx = (angle & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int mrl_mul = !!(angle & ANGLE_MULTI_MRL_FLAG);
    const pixel *const top = &topleft[mrl_idx + 1];

    if (mrl_mul) {
        // Safe maximum size for edge buffers
        const int e_stride = (width + height + (mrl_idx << 1) + 3) * 2;
        const pixel *const top2 = &topleft[1 - e_stride];
        for (int x = 0; x < width; x++) {
            dst[x] = (top[x] + top2[x]) >> 1;
        }
        const pixel *const edge = dst;
        int y = 1;
        do {
            dst += PXSTRIDE(stride);
            pixel_copy(dst, edge, width);
        } while (++y < height);
        return;
    }

    for (int y = 0; y < height; y++) {
        pixel_copy(dst, top, width);
        dst += PXSTRIDE(stride);
    }
}

static void ipred_h_c(pixel *dst, const ptrdiff_t stride,
                      const pixel *const topleft,
                      const int width, const int height, const int angle,
                      const int max_width, const int max_height
                      HIGHBD_DECL_SUFFIX)
{
    const int mrl_idx = (angle & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int mrl_mul = !!(angle & ANGLE_MULTI_MRL_FLAG);
    const pixel *left = &topleft[-(mrl_idx + 1)];

    if (mrl_mul) {
        // Safe maximum size for edge buffers
        const int e_stride = (width + height + (mrl_idx << 1) + 3) * 2;
        const pixel *left2 = &topleft[-(1 + e_stride)];
        for (int y = 0; y < height; y++) {
            const int v = (left[-y] + left2[-y]) >> 1;
            pixel_set(dst, v, width);
            dst += PXSTRIDE(stride);
        }
        return;
    }

    for (int y = 0; y < height; y++) {
        pixel_set(dst, left[-y], width);
        dst += PXSTRIDE(stride);
    }
}

static void ipred_paeth_c(pixel *dst, const ptrdiff_t stride,
                          const pixel *const tl_ptr,
                          const int width, const int height, const int a,
                          const int max_width, const int max_height
                          HIGHBD_DECL_SUFFIX)
{
    const int topleft = tl_ptr[0];
    for (int y = 0; y < height; y++) {
        const int left = tl_ptr[-(y + 1)];
        for (int x = 0; x < width; x++) {
            const int top = tl_ptr[1 + x];
            const int base = left + top - topleft;
            const int ldiff = abs(left - base);
            const int tdiff = abs(top - base);
            const int tldiff = abs(topleft - base);

            dst[x] = ldiff <= tdiff && ldiff <= tldiff ? left :
                     tdiff <= tldiff ? top : topleft;
        }
        dst += PXSTRIDE(stride);
    }
}

static void ipred_smooth_c(pixel *dst, const ptrdiff_t stride,
                           const pixel *const topleft,
                           const int width, const int height, const int a,
                           const int max_width, const int max_height
                           HIGHBD_DECL_SUFFIX)
{
    const int bwl2 = ulog2(width), bhl2 = ulog2(height);
    const int rnd_ver = height >> 1;
    const int rnd_hor = width >> 1;
    const int n_pel = width * height;
    const int scale = (n_pel >= 64) + (n_pel > 512);
    const uint8_t *const weights = dav1d_avm_sm_weights[scale];
    const int right = topleft[width + 1], bottom = topleft[-(height + 1)];

    for (int y = 0; y < height; y++) {
        const int left = topleft[-(y + 1)];
        const int diff_hor = left - right;
        const int off_ver = height - 1 - y;
        const int w_ver = weights[y];
        for (int x = 0; x < width; x++) {
            const int above = topleft[1 + x];
            const int mul_ver = (above - bottom) * off_ver;
            const int mul_hor = diff_hor * (width - 1 - x);
            int pred_ver = bottom + ((mul_ver + rnd_ver) >> bhl2);
            int pred_hor = right + ((mul_hor + rnd_hor) >> bwl2);
            pred_ver += ((above - pred_ver) * w_ver + 32) >> 6;
            pred_hor += ((left - pred_hor) * weights[x] + 32) >> 6;
            dst[x] = (pred_ver + pred_hor + 1) >> 1;
        }
        dst += PXSTRIDE(stride);
    }
}

static void ipred_smooth_v_c(pixel *dst, const ptrdiff_t stride,
                             const pixel *const topleft,
                             const int width, const int height, const int a,
                             const int max_width, const int max_height
                             HIGHBD_DECL_SUFFIX)
{
    const int bhl2 = ulog2(height);
    const int rnd = height >> 1;
    const int n_pel = width * height;
    const int scale = (n_pel >= 64) + (n_pel > 512);
    const uint8_t *const weights = dav1d_avm_sm_weights[scale];
    const int bottom = topleft[-(height + 1)];

    for (int y = 0; y < height; y++) {
        const int off = height - 1 - y;
        const int w_ver = weights[y];
        for (int x = 0; x < width; x++) {
            const int above = topleft[1 + x];
            const int mul = (above - bottom) * off;
            const int pred = bottom + ((mul + rnd) >> bhl2);
            dst[x] = pred + (((above - pred) * w_ver + 32) >> 6);
        }
        dst += PXSTRIDE(stride);
    }
}

static void ipred_smooth_h_c(pixel *dst, const ptrdiff_t stride,
                             const pixel *const topleft,
                             const int width, const int height, const int a,
                             const int max_width, const int max_height
                             HIGHBD_DECL_SUFFIX)
{
    const int bwl2 = ulog2(width);
    const int rnd = width >> 1;
    const int n_pel = width * height;
    const int scale = (n_pel >= 64) + (n_pel > 512);
    const uint8_t *const weights = dav1d_avm_sm_weights[scale];
    const int right = topleft[width + 1];

    for (int y = 0; y < height; y++) {
        const int left = topleft[-(y + 1)];
        const int diff = left - right;
        for (int x = 0; x < width; x++) {
            const int mul = diff * (width - 1 - x);
            const int pred = right + ((mul + rnd) >> bwl2);
            dst[x] = pred + (((left - pred) * weights[x] + 32) >> 6);
        }
        dst += PXSTRIDE(stride);
    }
}

static NOINLINE int get_filter_strength(const int wh, const int angle,
                                        const int is_sm)
{
    if (is_sm) {
        if (wh <= 8) {
            if (angle >= 64) return 2;
            if (angle >= 40) return 1;
        } else if (wh <= 16) {
            if (angle >= 48) return 2;
            if (angle >= 20) return 1;
        } else if (wh <= 24) {
            if (angle >=  4) return 3;
        } else {
            return 3;
        }
    } else {
        if (wh <= 8) {
            if (angle >= 56) return 1;
        } else if (wh <= 16) {
            if (angle >= 40) return 1;
        } else if (wh <= 24) {
            if (angle >= 32) return 3;
            if (angle >= 16) return 2;
            if (angle >=  8) return 1;
        } else if (wh <= 32) {
            if (angle >= 32) return 3;
            if (angle >=  4) return 2;
            return 1;
        } else {
            return 3;
        }
    }
    return 0;
}

static NOINLINE void filter_edge(pixel *const out, const int sz,
                                 const int lim_from, const int lim_to,
                                 const pixel *const in, const int from,
                                 const int to, const int strength)
{
    static const uint8_t kernel[3][5] = {
        { 0, 4, 8, 4, 0 },
        { 0, 5, 6, 5, 0 },
        { 2, 4, 4, 4, 2 }
    };

    assert(strength > 0);
    int i = 0;
    for (; i < imin(sz, lim_from); i++)
        out[i] = in[iclip(i, from, to - 1)];
    for (; i < imin(lim_to, sz); i++) {
        int s = 0;
        for (int j = 0; j < 5; j++)
            s += in[iclip(i - 2 + j, from, to - 1)] * kernel[strength - 1][j];
        out[i] = (s + 8) >> 4;
    }
    for (; i < sz; i++)
        out[i] = in[iclip(i, from, to - 1)];
}

static void idif_z1_ibp_z3(pixel *dst, const ptrdiff_t stride,
                           const pixel *const topleft,
                           const int width, const int height,
                           const int delta, const uint8_t weights[16][16]
                           HIGHBD_DECL_SUFFIX)
{
    assert(delta > 0);
    const int x_shift = width >> (4 + 1);
    const int y_shift = height >> (4 + 1);
    const int max_base_x = (width + height) - 1;

    int x_pos = delta;
    int y;
    for (y = 0; y < height; y++, x_pos += delta) {
        int base = x_pos >> 6;
        if (base > max_base_x) {
            break;
        }

        const int wy = y >> y_shift;
        const int shift = (x_pos & 0x3F) >> 1;
        const DRFilter4Tap f = av1_dr_interp_filter[shift];
        const int w = imin(width, max_base_x - base);
        int x;
        for (x = 0; x < w; x++, base++) {
            const int v = f.a * topleft[base - 1] + f.b * topleft[base] +
                          f.c * topleft[base + 1] + f.d * topleft[base + 2];
            const pixel pred = iclip_pixel((v + 64) >> 7);

            const int wx = x >> x_shift;
            const int weight = weights[wx][wy];
            const int blend = dst[x] * weight + pred * (128 - weight);
            dst[x] = (blend + 64) >> 7;
        }

        for (; x < width; x++) {
            const int wx = x >> x_shift;
            const int weight = weights[wx][wy];
            const int blend = dst[x] * weight + topleft[max_base_x] * (128 - weight);
            dst[x] = (blend + 64) >> 7;
        }
        dst += PXSTRIDE(stride);
    }

    for (; y < height; y++) {
        const int wy = y >> y_shift;
        for (int x = 0; x < width; x++) {
            const int wx = x >> x_shift;
            const int weight = weights[wx][wy];
            const int blend = dst[x] * weight + topleft[max_base_x] * (128 - weight);
            dst[x] = (blend + 64) >> 7;
        }
        dst += PXSTRIDE(stride);
    }
}

static void idif_z3_ibp_z1(pixel *dst, const ptrdiff_t stride,
                           const pixel *const left,
                           const int width, const int height,
                           const int delta, const uint8_t weights[16][16]
                           HIGHBD_DECL_SUFFIX)
{
    assert(delta > 0);
    const int x_shift = width >> (4 + 1);
    const int y_shift = height >> (4 + 1);
    const int max_base_y = (width + height) - 1;

    int y_pos = delta;
    int x;
    for (x = 0; x < width; x++, y_pos += delta) {
        const int wx = x >> x_shift;
        int base = y_pos >> 6;
        if (base > max_base_y) {
            break;
        }

        const int h = imin(height, max_base_y - base);
        const int shift = (y_pos & 0x3F) >> 1;
        const DRFilter4Tap f = av1_dr_interp_filter[shift];
        int y;
        for (y = 0; y < h; y++, base++) {
            const int wy = y >> y_shift;
            const int weight = weights[wy][wx];

            const int v = f.a * left[-(base - 1)] + f.b * left[-base] +
                          f.c * left[-(base + 1)] + f.d * left[-(base + 2)];
            const pixel pred = iclip_pixel((v + 64) >> 7);
            const int blend =
                dst[y * PXSTRIDE(stride) + x] * weight + pred * (128 - weight);
            dst[y * PXSTRIDE(stride) + x] = (blend + 64) >> 7;
        }

        for (; y < height; y++) {
            const int wy = y >> y_shift;
            const int weight = weights[wy][wx];
            const int blend = dst[y * PXSTRIDE(stride) + x] * weight +
                              left[-max_base_y] * (128 - weight);
            dst[y * PXSTRIDE(stride) + x] = (blend + 64) >> 7;
        }
    }

    for (; x < width; x++) {
        const int wx = x >> x_shift;
        for (int y = 0; y < height; y++) {
            const int wy = y >> y_shift;
            const int weight = weights[wy][wx];
            const int blend = dst[y * PXSTRIDE(stride) + x] * weight +
                              left[-max_base_y] * (128 - weight);
            dst[y * PXSTRIDE(stride) + x] = (blend + 64) >> 7;
        }
    }
}

static void ipred_z1_c(pixel *dst, const ptrdiff_t stride,
                       const pixel *const topleft_in,
                       const int width, const int height, int angle,
                       const int max_width, const int max_height
                       HIGHBD_DECL_SUFFIX)
{
    const int is_sm_l = !!(angle & ANGLE_SMOOTH_LEFT_EDGE_FLAG);
    const int is_sm_t = !!(angle & ANGLE_SMOOTH_TOP_EDGE_FLAG);
    const int enable_intra_edge_filter = !!(angle & ANGLE_USE_EDGE_FILTER_FLAG);
    const int enable_ibp = !!(angle & ANGLE_IBP_FLAG);
    const int mrl_idx = (angle & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int mrl_mul = !!(angle & ANGLE_MULTI_MRL_FLAG);
    const int have_top = !!(angle & ANGLE_HAS_TOP_FLAG);
    const int have_left = !!(angle & ANGLE_HAS_LEFT_FLAG);
    angle &= 511;
    assert(angle < 90);
    const int dx = dav1d_dr_intra_derivative[angle];
    int max_base_x = (width + height) - 1 + (mrl_idx << 1);
    const pixel *top = &topleft_in[mrl_idx + 1];
    int xpos = dx * (1 + mrl_idx);

    if (mrl_mul) {
        // Safe maximum size for edge buffers
        const int e_stride = (width + height + (mrl_idx << 1) + 3) * 2;
        const int max_base_x2 = (width + height) - 1;
        const pixel *top2 = &topleft_in[1 - e_stride];
        int xpos2 = dx;
        for (int y = 0; y < height; y++, xpos += dx, xpos2 += dx) {
            int base = xpos >> 6;
            int base2 = xpos2 >> 6;
            if (base > max_base_x) {
                assert(base2 > max_base_x2);
                const int v = (top[max_base_x] + top2[max_base_x2]) >> 1;
                do {
                    pixel_set(dst, v, width);
                    dst += PXSTRIDE(stride);
                } while (++y < height);
                return;
            }

            const DRFilter4Tap f1 = av1_dr_interp_filter[(xpos & 0x3F) >> 1];
            const DRFilter4Tap f2 = av1_dr_interp_filter[(xpos2 & 0x3F) >> 1];
            for (int x = 0; x < width; x++, base++, base2++) {
                int v1, v2;
                if (base <= max_base_x) {
                    v1 = f1.a * top[base - 1] + f1.b * top[base] +
                         f1.c * top[base + 1] + f1.d * top[base + 2];
                    v1 = iclip_pixel((v1 + 64) >> 7);
                } else {
                    v1 = top[max_base_x];
                }

                if (base2 <= max_base_x2) {
                    v2 = f2.a * top2[base2 - 1] + f2.b * top2[base2] +
                         f2.c * top2[base2 + 1] + f2.d * top2[base2 + 2];
                    v2 = iclip_pixel((v2 + 64) >> 7);
                } else {
                    v2 = top2[max_base_x2];
                }
                dst[x] = (v1 + v2) >> 1;
            }
            dst += PXSTRIDE(stride);
        }
        return;
    }

    // Max size = 1 (topleft) + 64 (width) + 64 (height) + 4 extra = 133
    pixel filt[133];
    const int str = enable_intra_edge_filter && have_top && !mrl_idx ?
            get_filter_strength(width + height, 90 - angle, is_sm_t) : 0;
    if (str) {
        const int sz = width + height + 1;
        filter_edge(&filt[2], sz, 1, sz, &topleft_in[1], 0, sz, str);
        filt[0] = filt[1] = filt[2];
        const int end = sz + 1;
        filt[end + 2] = filt[end + 1] = filt[end];
        top = &filt[2];
        max_base_x = width + height - 1;
    } else {
        top = &topleft_in[1];
        max_base_x = (width + height) - 1 + (mrl_idx << 1);
    }
    for (int y = 0, xpos = dx; y < height; y++, xpos += dx) {
        int base = xpos >> 6;
        if (base > max_base_x) {
            for (; y < height; y++) {
                pixel_set(&dst[y * PXSTRIDE(stride)], top[max_base_x], width);
            }
            break;
        }
        const DRFilter4Tap f = av1_dr_interp_filter[(xpos & 0x3F) >> 1];
        for (int x = 0; x < width; x++, base++) {
            if (base > max_base_x) {
                pixel_set(&dst[y * PXSTRIDE(stride) + x], top[max_base_x],
                          width - x);
                break;
            }
            const int v = f.a * top[base - 1] + f.b * top[base] +
                          f.c * top[base + 1] + f.d * top[base + 2];
            dst[y * PXSTRIDE(stride) + x] = iclip_pixel((v + 64) >> 7);
        }
    }

    if (enable_ibp && !mrl_idx) {
        const int mode_index = av1_angle_to_mode_index[angle / 3 - 12];
        if (mode_index) {
            const pixel *left = &topleft_in[-1];
            assert(have_left);
            const int filter_strength = enable_intra_edge_filter && have_left ?
                get_filter_strength(width + height, angle, is_sm_l) : 0;
            if (filter_strength) {
                const int sz = width + height + 1;
                filter_edge(&filt[2], sz, 0, width + height,
                            &topleft_in[-(width + height)],
                            0, sz, filter_strength);
                filt[0] = filt[1] = filt[2];
                const int end = sz + 1;
                filt[end + 2] = filt[end + 1] = filt[end];
                left = &filt[width + height + 1];
            }
            idif_z3_ibp_z1(dst, stride, left, width, height,
                           dav1d_dr_intra_derivative[90 - angle],
                           dav1d_ibp_weights[mode_index - 1] HIGHBD_TAIL_SUFFIX);
        }
    }
}

static void ipred_z2_c(pixel *dst, const ptrdiff_t stride,
                       const pixel *const topleft_in,
                       const int width, const int height, int angle,
                       const int max_width, const int max_height
                       HIGHBD_DECL_SUFFIX)
{
    const int is_sm_l = !!(angle & ANGLE_SMOOTH_LEFT_EDGE_FLAG);
    const int is_sm_t = !!(angle & ANGLE_SMOOTH_TOP_EDGE_FLAG);
    const int enable_intra_edge_filter = !!(angle & ANGLE_USE_EDGE_FILTER_FLAG);
    const int mrl_idx = (angle & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int have_top = !!(angle & ANGLE_HAS_TOP_FLAG);
    const int have_left = !!(angle & ANGLE_HAS_LEFT_FLAG);
    angle &= 511;
    assert(angle > 90 && angle < 180);
    const int dy = dav1d_dr_intra_derivative[angle - 90];
    const int dx = dav1d_dr_intra_derivative[180 - angle];

    // Max size = 1 (topleft) + 64 (width) + 64 (height) + 4 extra = 133
    pixel filt[133];
    pixel *const topleft = &filt[66];

    const int n_px_t = width + 1;
    const int str_t = enable_intra_edge_filter && have_top && !mrl_idx ?
        get_filter_strength(width + height, angle - 90, is_sm_t) : 0;
    if (str_t) {
        filter_edge(&topleft[1], n_px_t + 1, 1, n_px_t, &topleft_in[0],
                    0, n_px_t, str_t);
    } else {
        pixel_copy(&topleft[1], &topleft_in[0], n_px_t);
    }
    topleft[0] = topleft[1];
    topleft[n_px_t + 1] = topleft[n_px_t];

    const int n_px_l = height + 1;
    const int str_l = enable_intra_edge_filter && have_left && !mrl_idx ?
        get_filter_strength(width + height, 180 - angle, is_sm_l) : 0;
    if (str_l) {
        filter_edge(&topleft[-n_px_l], height, height - max_height, height,
                    &topleft_in[-height], 0, height + 1, str_l);
    } else {
        pixel_copy(&topleft[-n_px_l], &topleft_in[-height], n_px_l);
    }
    topleft[-1] = topleft[0];
    topleft[-(n_px_l + 1)] = topleft[-n_px_l];

    for (int y = 0; y < height; y++) {
        const int ypos = y + 1;
        int xpos = -(ypos + 0) * dx;
        int x;
        for (x = 0; x < width && xpos < -64; x++, xpos += 64) {
            const int xpos_l = x + 1;
            const int ypos_l = (y << 6) - (xpos_l + 0) * dy;
            const int base_y = ypos_l >> 6;
            assert(base_y >= -1);
            const int shift = (ypos_l & 0x3F) >> 1;
            const int v =
                av1_dr_interp_filter[shift].a * topleft[-(base_y + 1)] +
                av1_dr_interp_filter[shift].b * topleft[-(base_y + 2)] +
                av1_dr_interp_filter[shift].c * topleft[-(base_y + 3)] +
                av1_dr_interp_filter[shift].d * topleft[-(base_y + 4)];
            dst[x] = iclip_pixel((v + 64) >> 7);
        }

        for (; x < width; x++, xpos += 64) {
            const int base_x = xpos >> 6;
            const int shift = (xpos & 0x3F) >> 1;
            const int v =
                av1_dr_interp_filter[shift].a * topleft[base_x + 1] +
                av1_dr_interp_filter[shift].b * topleft[base_x + 2] +
                av1_dr_interp_filter[shift].c * topleft[base_x + 3] +
                av1_dr_interp_filter[shift].d * topleft[base_x + 4];
            dst[x] = iclip_pixel((v + 64) >> 7);
        }
        dst += PXSTRIDE(stride);
    }
}

static void ipred_z3_c(pixel *dst, const ptrdiff_t stride,
                       const pixel *const topleft_in,
                       const int width, const int height, int angle,
                       const int max_width, const int max_height
                       HIGHBD_DECL_SUFFIX)
{
    const int is_sm_l = !!(angle & ANGLE_SMOOTH_LEFT_EDGE_FLAG);
    const int is_sm_t = !!(angle & ANGLE_SMOOTH_TOP_EDGE_FLAG);
    const int enable_intra_edge_filter = !!(angle & ANGLE_USE_EDGE_FILTER_FLAG);
    const int have_left = !!(angle & ANGLE_HAS_LEFT_FLAG);
    const int have_top = !!(angle & ANGLE_HAS_TOP_FLAG);
    const int enable_ibp = !!(angle & ANGLE_IBP_FLAG);
    const int mrl_idx =
        (angle & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int mrl_mul = !!(angle & ANGLE_MULTI_MRL_FLAG);
    angle &= 511;
    assert(angle > 180);
    const int dy = dav1d_dr_intra_derivative[270 - angle];
    const int n_px = width + height;
    int max_base_y = height + imin(width, height) - 1 + (mrl_idx << 1);

    const pixel *left = &topleft_in[-(mrl_idx + 1)];
    int ypos = dy * (1 + mrl_idx);
    if (mrl_mul) {
        // Safe maximum size for edge buffers
        const int e_stride = mrl_idx ? (n_px + (mrl_idx << 1) + 3) * 2 : 0;
        const pixel *left2 = &topleft_in[-(1 + e_stride)];
        const int max_base_y2 = height + imin(width, height) - 1;
        int ypos2 = dy;

        for (int x = 0; x < width; x++, ypos += dy, ypos2 += dy) {
            int base = ypos >> 6;
            int base2 = ypos2 >> 6;
            if (base > max_base_y) {
                assert(base2 > max_base_y2);
                const int rem = width - x;
                dst += x;
                const int v = (left[-max_base_y] + left2[-max_base_y2]) >> 1;
                for (int y = 0; y < height; y++) {
                    pixel_set(dst, v, rem);
                    dst += PXSTRIDE(stride);
                }
                return;
            }

            const DRFilter4Tap f1 = av1_dr_interp_filter[(ypos & 0x3F) >> 1];
            const DRFilter4Tap f2 = av1_dr_interp_filter[(ypos2 & 0x3F) >> 1];
            for (int y = 0; y < height; y++, base++, base2++) {
                int v1, v2;
                if (base < max_base_y) {
                    v1 = f1.a * left[-(base - 1)] + f1.b * left[-base] +
                         f1.c * left[-(base + 1)] + f1.d * left[-(base + 2)];
                    v1 = iclip_pixel((v1 + 64) >> 7);
                } else {
                    v1 = left[-max_base_y];
                }
                if (base2 < max_base_y2) {
                    v2 = f2.a * left2[-(base2 - 1)] + f2.b * left2[-base2] +
                         f2.c * left2[-(base2 + 1)] + f2.d * left2[-(base2 + 2)];
                    v2 = iclip_pixel((v2 + 64) >> 7);
                } else {
                    v2 = left2[-max_base_y2];
                }
                dst[y * PXSTRIDE(stride) + x] = (v1 + v2) >> 1;
            }
        }
        return;
    }

    // Max size = 1 (topleft) + 64 (width) + 64 (height) + 4 extra = 133
    pixel filt[133];
    assert(have_left);
    const int str = enable_intra_edge_filter && !mrl_idx ?
        get_filter_strength(n_px, angle - 180, is_sm_l) : 0;
    if (str) {
        filter_edge(&filt[2], n_px + 1, 0, n_px,
                    &topleft_in[-n_px], imax(width - height, 0),
                    n_px + 1, str);
        filt[0] = filt[1] = filt[2];
        const int end = 2 + n_px;
        filt[end + 2] = filt[end + 1] = filt[end];
        left = &filt[n_px + 1];
        max_base_y = n_px - 1;
    }

    for (int x = 0; x < width; x++, ypos += dy) {
        const DRFilter4Tap f = av1_dr_interp_filter[(ypos & 0x3F) >> 1];
        for (int y = 0, base = ypos >> 6; y < height; y++, base++) {
            if (base <= max_base_y) {
                const int v = f.a * left[-(base - 1)] + f.b * left[-base] +
                              f.c * left[-(base + 1)] + f.d * left[-(base + 2)];
                dst[y * PXSTRIDE(stride) + x] = iclip_pixel((v + 64) >> 7);
            } else {
                do {
                    dst[y * PXSTRIDE(stride) + x] = left[-max_base_y];
                } while (++y < height);
                break;
            }
        }
    }

    if (enable_ibp && !mrl_idx) {
        const int mode_idx = av1_angle_to_mode_index[78 - ((angle + 2) / 3)];
        if (mode_idx) {
            const pixel *top = &topleft_in[1];
            const int str = enable_intra_edge_filter && have_top ?
                get_filter_strength(n_px, 270 - angle, is_sm_t) : 0;
                if (str) {
                    filter_edge(filt, n_px + 1, 1, n_px, &top[-1],
                                0, n_px, str);
                    filt[n_px + 2] = filt[n_px + 1] = filt[n_px];
                    top = &filt[1];
            }
            idif_z1_ibp_z3(dst, stride, top, width, height,
                dav1d_dr_intra_derivative[angle - 180],
                dav1d_ibp_weights[mode_idx - 1] HIGHBD_TAIL_SUFFIX);
        }
    }
}

#if ARCH_X86
#define FILTER(flt_ptr, p0, p1, p2, p3, p4, p5, p6) \
    flt_ptr[ 0] * p0 + flt_ptr[ 1] * p1 +           \
    flt_ptr[16] * p2 + flt_ptr[17] * p3 +           \
    flt_ptr[32] * p4 + flt_ptr[33] * p5 +           \
    flt_ptr[48] * p6
#define FLT_INCR 2
#else
#define FILTER(flt_ptr, p0, p1, p2, p3, p4, p5, p6) \
    flt_ptr[ 0] * p0 + flt_ptr[ 8] * p1 +           \
    flt_ptr[16] * p2 + flt_ptr[24] * p3 +           \
    flt_ptr[32] * p4 + flt_ptr[40] * p5 +           \
    flt_ptr[48] * p6
#define FLT_INCR 1
#endif

static NOINLINE void
cfl_ac_c(int16_t *ac, const pixel *ypx, const ptrdiff_t stride,
         const int w_pad, const int h_pad, const int width, const int height,
         const int ss_hor, const int ss_ver)
{
    int y, x;
    int16_t *const ac_orig = ac;

    assert(w_pad >= 0 && w_pad * 4 < width);
    assert(h_pad >= 0 && h_pad * 4 < height);

    for (y = 0; y < height - 4 * h_pad; y++) {
        for (x = 0; x < width - 4 * w_pad; x++) {
            int ac_sum = ypx[x << ss_hor];
            if (ss_hor) ac_sum += ypx[x * 2 + 1];
            if (ss_ver) {
                ac_sum += ypx[(x << ss_hor) + PXSTRIDE(stride)];
                if (ss_hor) ac_sum += ypx[x * 2 + 1 + PXSTRIDE(stride)];
            }
            ac[x] = ac_sum << (1 + !ss_ver + !ss_hor);
        }
        for (; x < width; x++)
            ac[x] = ac[x - 1];
        ac += width;
        ypx += PXSTRIDE(stride) << ss_ver;
    }
    for (; y < height; y++) {
        memcpy(ac, &ac[-width], width * sizeof(*ac));
        ac += width;
    }

    const int log2sz = ctz(width) + ctz(height);
    int sum = (1 << log2sz) >> 1;
    for (ac = ac_orig, y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            sum += ac[x];
        ac += width;
    }
    sum >>= log2sz;

    // subtract DC
    for (ac = ac_orig, y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            ac[x] -= sum;
        ac += width;
    }
}

#define cfl_ac_fn(fmt, ss_hor, ss_ver) \
static void cfl_ac_##fmt##_c(int16_t *const ac, const pixel *const ypx, \
                             const ptrdiff_t stride, const int w_pad, \
                             const int h_pad, const int cw, const int ch) \
{ \
    cfl_ac_c(ac, ypx, stride, w_pad, h_pad, cw, ch, ss_hor, ss_ver); \
}

cfl_ac_fn(420, 1, 1)
cfl_ac_fn(422, 1, 0)
cfl_ac_fn(444, 0, 0)

static void pal_pred_c(pixel *dst, const ptrdiff_t stride,
                       const pixel *const pal, const uint8_t *idx,
                       const int w, const int h)
{
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x += 2) {
            const int i = *idx++;
            assert(!(i & 0x88));
            dst[x + 0] = pal[i & 7];
            dst[x + 1] = pal[i >> 4];
        }
        dst += PXSTRIDE(stride);
    }
}

static const int8_t orip_taps_4x4[16][9] = {
    { 4, 16,  4,  0,  0,  0,  0,  4, 16 },
    { 2,  4, 16,  4,  0,  0,  0,  2,  8 },
    { 1,  0,  4, 16,  4,  0,  0,  1,  4 },
    { 0,  0,  2,  4, 16,  0,  0,  0,  2 },
    { 2,  8,  2,  0,  0,  0,  4, 16,  4 },
    { 0,  2,  8,  2,  0,  0,  2,  8,  2 },
    { 0,  0,  2,  8,  2,  0,  1,  4,  1 },
    { 0,  0,  0,  2,  8,  0,  0,  2,  1 },
    { 0,  4,  0,  0,  0,  4, 16,  4,  0 },
    { 0,  0,  4,  0,  0,  2,  8,  2,  0 },
    { 0,  0,  1,  4,  1,  1,  4,  1,  0 },
    { 0,  0,  0,  2,  4,  0,  4,  0,  0 },
    { 0,  0,  1,  0,  0, 16,  4,  2,  0 },
    { 0,  0,  0,  1,  0,  8,  2,  1,  0 },
    { 0,  0,  1,  2,  1,  4,  1,  0,  0 },
    { 0,  0,  0,  1,  2,  2,  1,  0,  0 },
};

static void orip_c(pixel *dst, const ptrdiff_t stride,
                   const pixel *const edge,
                   const unsigned th_mask,
                   const int width, const int height
                   HIGHBD_DECL_SUFFIX)
{
    assert(width >= 4 && height >= 4);
    assert(th_mask < 3);

    const ptrdiff_t s = PXSTRIDE(stride);
    const int w_th = (th_mask & 0x1U) ? 0 : imin(width >> 2, 4);
    const int h_th = (th_mask & 0x2U) ? 0 : imin(height >> 2, 4);

    // [0..4]: Row above
    // [5..8]: Column to the left
    pixel topleft[9];
    pixel_copy(&topleft[5], &edge[-4], 4);

    // Carry the last row of first 4x4 for the block below it
    pixel top[4];
    pixel_copy(top, &dst[3 * s], 4);

    // First row of 4x4 blocks
    for (int bx = 0; bx < width; bx += 4) {
        pixel_copy(topleft, &edge[bx], 5); // Top edge

        // Carry the last col for the next block
        const pixel left[4] = {
            dst[3 * s + bx + 3],
            dst[2 * s + bx + 3],
            dst[1 * s + bx + 3],
            dst[0 * s + bx + 3],
        };

        for (int y = 0, i = 0; y < 4; y++) {
            for (int x = bx; x < bx + 4; x++, i++) {
                if (x >= w_th && y >= h_th)
                    continue;

                const ptrdiff_t p = y * s + x;
                const int v = dst[p];
                int off = 0;
                for (int tap = 0; tap < 9; tap++)
                    off += orip_taps_4x4[i][tap] * (topleft[tap] - v);

                off = (off + 32) >> 6;
                dst[p] = iclip_pixel(v + off);
            }
        }

        pixel_copy(&topleft[5], left, 4);
    }

    pixel_copy(&topleft[1], top, 4);

    // Column of 4x4 blocks
    for (int by = 4; by < height; by += 4) {
        // Left edge
        topleft[0] = edge[-by];
        pixel_copy(&topleft[5], &edge[-(by + 4)], 4);

        // Carry last row for the next block
        pixel_copy(top, &dst[(by + 3) * s], 4);

        for (int y = by, i = 0; y < by + 4; y++) {
            for (int x = 0; x < w_th; x++, i++) {
                const ptrdiff_t p = y * s + x;
                const int v = dst[p];
                int off = 0;
                for (int tap = 0; tap < 9; tap++)
                    off += orip_taps_4x4[i][tap] * (topleft[tap] - v);

                off = (off + 32) >> 6;
                dst[p] = iclip_pixel(v + off);
            }
            i += 4 - w_th; // skip unprocessed column taps
        }

        pixel_copy(&topleft[1], top, 4);
    }
}

static void ipred_dip_c(pixel *dst, const ptrdiff_t stride,
                        const pixel *const topleft,
                        const int width, const int height, int mode,
                        const int max_width, const int max_height
                        HIGHBD_DECL_SUFFIX)
{
    const int trans = !!(mode & 16);
    const int wd = width >> 2;
    const int hd = height >> 2;
    const int wl2 = ulog2(wd);
    const int hl2 = ulog2(hd);
    const int wrnd = width >> 3;
    const int hrnd = height >> 3;
    const int i_t = 1 + 4 * trans;
    const int i_l = 5 - 4 * trans;
    pixel in[11];
    int sum;
    int in_sum = in[0] = topleft[0];

    const pixel *tl = &topleft[1];
    for (int i = 0; i < 4; i++) {
        sum = 0;
        for (int x = 0; x < wd; x++)
            sum += *tl++;
        in_sum += in[i_t + i] = (sum + wrnd) >> wl2;
    }

    tl = &topleft[-1];
    for (int i = 0; i < 4; i++) {
        sum = 0;
        for (int y = 0; y < hd; y++)
            sum += *tl--;
        in_sum += in[i_l + i] = (sum + hrnd) >> hl2;
    }

    sum = 0;
    for (int x = 0; x < wd; x++)
        sum += topleft[x + width + 1];
    in_sum += in[9 + trans] = (sum + wrnd) >> wl2;

    sum = 0;
    for (int y = 0; y < hd; y++)
        sum += topleft[-(y + height + 1)];
    in_sum += in[10 - trans] = (sum + hrnd) >> hl2;

    const int m = mode & 7;
    assert(m < 6);

    int uwl2 = wl2 - 1;
    int dwl2 = 0;
    if (uwl2 < 0) {
        dwl2 = -uwl2;
        uwl2 = 0;
    }
    const int step_x = 1 << uwl2;
    const int dw = 1 << dwl2;
    int uhl2 = hl2 - 1;
    int dhl2 = 0;
    if (uhl2 < 0) {
        dhl2 = -uhl2;
        uhl2 = 0;
    }
    const int step_y = 1 << uhl2;
    const int dh = 1 << dhl2;
    const int grid_h = 8 >> dhl2;
    const int grid_w = 8 >> dwl2;

    // Run DIP prediction at each coarse grid position
    int y = step_y - 1;
    for (int gy = 0; gy < grid_h; gy++) {
        const int iy = gy * dh;
        int x = step_x - 1;
        for (int gx = 0; gx < grid_w; gx++) {
            const int ix = gx * dw;
            const int idx = trans ? (ix * 8 + iy) : (iy * 8 + ix);
            int sum = 0;
            for (int i = 0; i < 11; i++) {
                sum += dav1d_dip_weights[m][idx][i] * in[i];
            }
            dst[y * PXSTRIDE(stride) + x] = ((sum + 2048) >> 12) - in_sum;
            x += step_x;
        }
        y += step_y;
    }

    if (step_x > 1) {
        // Horizontal interpolation between coarse DIP samples
        y = step_y - 1;
        for (int gy = 0; gy < grid_h; gy++) {
            int p1 = topleft[-(y + 1)];
            int x = 0;
            for (int gx = 0; gx < grid_w; gx++) {
                const int p0 = p1;
                p1 = dst[y * PXSTRIDE(stride) + x + step_x - 1];
                for (int z = 0; z < step_x - 1; z++) {
                    const int z1 = z + 1;
                    dst[y * PXSTRIDE(stride) + x + z] =
                        (p0 * (step_x - z1) + (p1 * z1)) >> uwl2;
                }
                x += step_x;
            }
            y += step_y;
        }
    }
    if (step_y > 1) {
        // Vertical interpolation between coarse DIP samples.
        for (int x = 0; x < width; x++) {
            int p1 = topleft[x + 1];
            y = 0;
            for (int gy = 0; gy < grid_h; gy++) {
                const int p0 = p1;
                p1 = dst[(y + step_y - 1) * PXSTRIDE(stride) + x];
                for (int z = 0; z < step_y - 1; z++) {
                    const int z1 = z + 1;
                    dst[(y + z) * PXSTRIDE(stride) + x] =
                        (p0 * (step_y - z1) + (p1 * z1)) >> uhl2;
                }
                y += step_y;
            }
        }
    }
 }

#if HAVE_ASM
#if ARCH_AARCH64 || ARCH_ARM
#include "src/arm/ipred.h"
#elif ARCH_RISCV
#include "src/riscv/ipred.h"
#elif ARCH_X86
#include "src/x86/ipred.h"
#elif ARCH_LOONGARCH64
#include "src/loongarch/ipred.h"
#endif
#endif

COLD void bitfn(dav1d_intra_pred_dsp_init)(Dav1dIntraPredDSPContext *const c) {
    c->intra_pred[DC_PRED      ] = ipred_dc_c;
    c->intra_pred[DC_128_PRED  ] = ipred_dc_128_c;
    c->intra_pred[TOP_DC_PRED  ] = ipred_dc_top_c;
    c->intra_pred[LEFT_DC_PRED ] = ipred_dc_left_c;
    c->intra_pred[HOR_PRED     ] = ipred_h_c;
    c->intra_pred[VERT_PRED    ] = ipred_v_c;
    c->intra_pred[PAETH_PRED   ] = ipred_paeth_c;
    c->intra_pred[SMOOTH_PRED  ] = ipred_smooth_c;
    c->intra_pred[SMOOTH_V_PRED] = ipred_smooth_v_c;
    c->intra_pred[SMOOTH_H_PRED] = ipred_smooth_h_c;
    c->intra_pred[Z1_PRED      ] = ipred_z1_c;
    c->intra_pred[Z2_PRED      ] = ipred_z2_c;
    c->intra_pred[Z3_PRED      ] = ipred_z3_c;
    c->intra_pred[DIP_PRED     ] = ipred_dip_c;

    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I420 - 1] = cfl_ac_420_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I422 - 1] = cfl_ac_422_c;
    c->cfl_ac[DAV1D_PIXEL_LAYOUT_I444 - 1] = cfl_ac_444_c;

    c->cfl_pred[DC_PRED     ] = ipred_cfl_c;
    c->cfl_pred[DC_128_PRED ] = ipred_cfl_128_c;
    c->cfl_pred[TOP_DC_PRED ] = ipred_cfl_top_c;
    c->cfl_pred[LEFT_DC_PRED] = ipred_cfl_left_c;

    c->pal_pred = pal_pred_c;
    c->orip = orip_c;

#if 0
#if HAVE_ASM
#if ARCH_AARCH64 || ARCH_ARM
    intra_pred_dsp_init_arm(c);
#elif ARCH_RISCV
    intra_pred_dsp_init_riscv(c);
#elif ARCH_X86
    intra_pred_dsp_init_x86(c);
#elif ARCH_LOONGARCH64
    intra_pred_dsp_init_loongarch(c);
#endif
#endif
#endif
}
