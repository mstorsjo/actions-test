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

static const DRFilter4Tap dr_interp_filter[32] = {
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
            dst[x] = iclip_pixel(dc + apply_sign((abs(diff) + 1024) >> 11, diff));
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
    return ((num * dav2d_div_recip[idx]) + ((1 << shift) >> 1)) >> shift;
}

static unsigned dc_gen(const pixel *const topleft,
                       const int width, const int height
                       HIGHBD_DECL_SUFFIX)
{
    const int n_pel = width + height;
    unsigned dc = 0;
    for (int i = 0; i < width; i++)
       dc += topleft[i + 1];
    for (int i = 0; i < height; i++)
       dc += topleft[-(i + 1)];
    if (width == height)
        return (dc + width) >> ctz(n_pel);

    return iclip_pixel(fast_div32_dc(dc, n_pel));
}

static void ipred_dc_c(pixel *dst, const ptrdiff_t stride,
                       const pixel *const topleft,
                       int width, int height, const int a,
                       const int max_width, const int max_height
                       HIGHBD_DECL_SUFFIX)
{
    const unsigned dc = dc_gen(topleft, width, height HIGHBD_TAIL_SUFFIX);

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
    unsigned dc = dc_gen(topleft, width, height HIGHBD_TAIL_SUFFIX);
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
            dst[x] = (top[x] + top2[x] + 1) >> 1;
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
            const int v = (left[-y] + left2[-y] + 1) >> 1;
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
    const uint8_t *const weights = dav2d_avm_sm_weights[scale];
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
    const uint8_t *const weights = dav2d_avm_sm_weights[scale];
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
    const uint8_t *const weights = dav2d_avm_sm_weights[scale];
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

static void ibp_blend(pixel *dst, const ptrdiff_t stride,
                      const pixel *tmp /* stride=64 */,
                      const int width, const int height, const int inv,
                      const uint8_t weights[16][16] HIGHBD_DECL_SUFFIX)
{
    const int x_shift = width >> (4 + 1);
    const int y_shift = height >> (4 + 1);

    for (int y = 0; y < height; y++) {
        const int wy = y >> y_shift;
        for (int x = 0; x < width; x++) {
            const int wx = x >> x_shift;
            const int weight = weights[inv ? wx : wy][inv ? wy : wx];
            dst[x] = (tmp[x] * (128 - weight) + dst[x] * weight + 64) >> 7;
        }
        dst += PXSTRIDE(stride);
        tmp += 64;
    }
}

static decl_angular_ipred_fn(ipred_z3_c);
static void ipred_z1_c(pixel *dst, const ptrdiff_t stride,
                       const pixel *const topleft_in,
                       const int width, const int height, int angle,
                       const int max_width, const int max_height
                       HIGHBD_DECL_SUFFIX)
{
    const int angle_flags = angle & ~(511 | ANGLE_IBP_FLAG);
    const int is_luma = angle & ANGLE_IS_LUMA;
    const int is_sm_t = !!(angle & ANGLE_SMOOTH_TOP_EDGE_FLAG);
    const int enable_intra_edge_filter = !!(angle & ANGLE_USE_EDGE_FILTER_FLAG);
    const int enable_ibp = !!(angle & ANGLE_IBP_FLAG);
    const int mrl_idx = (angle & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int mrl_mul = !!(angle & ANGLE_MULTI_MRL_FLAG);
    const int have_top = !!(angle & ANGLE_HAS_TOP_FLAG);
    angle &= 511;
    assert(angle < 90);

    if (mrl_mul) {
        const int e_stride = (width + height + (mrl_idx << 1) + 3) * 2;
        const pixel *tl2 = &topleft_in[-e_stride];
        pixel tmp[64 * 64];
        assert(is_luma);
        ipred_z1_c(tmp, 64 * sizeof(pixel), topleft_in, width, height,
                   angle | (mrl_idx << ANGLE_MRL_IDX_SHIFT) | ANGLE_IS_LUMA,
                   max_width, max_height HIGHBD_TAIL_SUFFIX);
        ipred_z1_c(dst, stride, tl2, width, height, angle | ANGLE_IS_LUMA,
                   max_width, max_height HIGHBD_TAIL_SUFFIX);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++)
                dst[x] = (tmp[y * 64 + x] + dst[x] + 1) >> 1;
            dst += PXSTRIDE(stride);
        }
        return;
    }

    const int dx = dav2d_dr_intra_derivative[angle];
    const int max_base_x = (width + height) - 1 + (mrl_idx << 1);

    // Buffer organization:
    // - 1 pixel left padding;
    // - 1 pixel top/left;
    // - mrl_idx pixels extra between top/left and top (max 3);
    // - width pixels top;
    // - height pixels top/right;
    // - 2 * mrl_idx pixels extra between top/right and right padding (max 2 * 3);
    // - 2 pixels right padding.
    pixel filt[1 + 1 + 3 + 64 + 64 + 2 * 3 + 2], *const top = &filt[2 + mrl_idx];
    const int str = enable_intra_edge_filter && have_top && !mrl_idx ?
            get_filter_strength(width + height, 90 - angle, is_sm_t) : 0;
    const int sz = 1 + mrl_idx + width + height + mrl_idx * 2;
    if (str) {
        filter_edge(&filt[1], sz, 1, sz + max_width - width,
                    topleft_in, 0, sz, str);
    } else {
        pixel_copy(&filt[1], topleft_in, sz);
    }
    filt[0] = filt[1];
    filt[sz + 2] = filt[sz + 1] = filt[sz];

    for (int y = 0, xpos = dx * (1 + mrl_idx); y < height; y++, xpos += dx) {
        int base = xpos >> 6;
        if (base > max_base_x) {
            for (; y < height; y++) {
                pixel_set(&dst[y * PXSTRIDE(stride)], top[max_base_x], width);
            }
            break;
        }
        const int shift = (xpos & 0x3F) >> 1;
        const DRFilter4Tap f = dr_interp_filter[shift];
        for (int x = 0; x < width; x++, base++) {
            if (base > max_base_x) {
                pixel_set(&dst[y * PXSTRIDE(stride) + x], top[max_base_x],
                          width - x);
                break;
            }
            if (is_luma) {
                const int v = f.a * top[base - 1] + f.b * top[base] +
                              f.c * top[base + 1] + f.d * top[base + 2];
                dst[y * PXSTRIDE(stride) + x] = iclip_pixel((v + 64) >> 7);
            } else {
                const int v = (32 - shift) * top[base] + shift * top[base + 1];
                dst[y * PXSTRIDE(stride) + x] = iclip_pixel((v + 16) >> 5);
            }
        }
    }

    if (enable_ibp) {
        // I've observed the following values here:
        // angle | mode_idx |      intra mode | angle_delta | comment
        //    84 |        0 |       vert_pred |          -2 |
        //    73 |        1 |       vert_left |          +2 |
        //    67 |        2 |       vert_left |           0 |
        //    61 |        3 |       vert_left |          -2 |
        //    51 |        4 |  diag_down_left |          +2 |
        //    45 |        5 |  diag_down_left |           0 |
        //    39 |        6 |  diag_down_left |          -2 |
        //    29 |        6 | diag_down_right |          +2 | wide_angle_remap
        //    23 |        6 | diag_down_right |           0 | wide_angle_remap
        const int mode_idx = imin(10 - (angle >> 3), 6);
        pixel tmp[64 * 64];
        ipred_z3_c(tmp, 64 * sizeof(pixel), topleft_in, width, height,
                   (180 + angle) | angle_flags,
                   max_width, max_height HIGHBD_TAIL_SUFFIX);
        ibp_blend(dst, stride, tmp, width, height, 0,
                  dav2d_ibp_weights[mode_idx] HIGHBD_TAIL_SUFFIX);
    }
}

static void ipred_z2_c(pixel *dst, const ptrdiff_t stride,
                       const pixel *const topleft_in,
                       const int width, const int height, int angle,
                       const int max_width, const int max_height
                       HIGHBD_DECL_SUFFIX)
{
    const int mrl_mul = !!(angle & ANGLE_MULTI_MRL_FLAG);
    const int is_luma = angle & ANGLE_IS_LUMA;
    const int is_sm_l = !!(angle & ANGLE_SMOOTH_LEFT_EDGE_FLAG);
    const int is_sm_t = !!(angle & ANGLE_SMOOTH_TOP_EDGE_FLAG);
    const int enable_intra_edge_filter = !!(angle & ANGLE_USE_EDGE_FILTER_FLAG);
    const int mrl_idx = (angle & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int have_top = !!(angle & ANGLE_HAS_TOP_FLAG);
    const int have_left = !!(angle & ANGLE_HAS_LEFT_FLAG);
    angle &= 511;
    assert(angle > 90 && angle < 180);

    if (mrl_mul) {
        const int e_stride = (width + height + (mrl_idx << 1) + 3) * 2;
        const pixel *tl2 = &topleft_in[-e_stride];
        pixel tmp[64 * 64];
        assert(is_luma);
        ipred_z2_c(tmp, 64 * sizeof(pixel), topleft_in, width, height,
                   angle | (mrl_idx << ANGLE_MRL_IDX_SHIFT) | ANGLE_IS_LUMA,
                   max_width, max_height HIGHBD_TAIL_SUFFIX);
        ipred_z2_c(dst, stride, tl2, width, height, angle | ANGLE_IS_LUMA,
                   max_width, max_height HIGHBD_TAIL_SUFFIX);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++)
                dst[x] = (tmp[y * 64 + x] + dst[x] + 1) >> 1;
            dst += PXSTRIDE(stride);
        }
        return;
    }

    const int dy = dav2d_dr_intra_derivative[angle - 90];
    const int dx = dav2d_dr_intra_derivative[180 - angle];

    // Buffer organization:
    // - 1 pixels top|left padding;
    // - 1 pixel top/left;
    // - mrl_idx pixels extra between top/left and left|top (max 3);
    // - height|width pixels left|top;
    // - 1 pixel bottom|right padding.
    pixel filt[1 + 1 + 3 + 64 + 1], *const top = &filt[mrl_idx];
    const int str_t = enable_intra_edge_filter && have_top && !mrl_idx ?
        get_filter_strength(width + height, angle - 90, is_sm_t) : 0;
    const int sz_t = 1 + width + mrl_idx;
    if (str_t) {
        filter_edge(&filt[1], sz_t, 1, sz_t + max_width - width,
                    topleft_in, 0, sz_t, str_t);
    } else {
        pixel_copy(&filt[1], topleft_in, sz_t);
    }
    filt[0] = filt[1];
    filt[sz_t + 1] = filt[sz_t];

    pixel filt2[1 + 1 + 3 + 64 + 1], *const left = &filt2[height + 2];
    const int str_l = enable_intra_edge_filter && have_left && !mrl_idx ?
        get_filter_strength(width + height, 180 - angle, is_sm_l) : 0;
    const int sz_l = 1 + height + mrl_idx;
    if (str_l) {
        filter_edge(&filt2[1], sz_l, height - max_height, sz_l - 1,
                    &topleft_in[-height], 0, sz_l, str_l);
    } else {
        pixel_copy(&filt2[1], &topleft_in[-(height + mrl_idx)], sz_l);
    }
    filt2[1 + sz_l] = filt2[sz_l];
    filt2[0] = filt2[1];

    for (int y = 0; y < height; y++) {
        const int ypos = y + 1;
        int xpos = -(ypos + mrl_idx) * dx;
        int x;
        for (x = 0; x < width && xpos < -(64 * (1 + mrl_idx)); x++, xpos += 64) {
            const int xpos_l = x + 1;
            const int ypos_l = (y << 6) - (xpos_l + mrl_idx) * dy;
            const int base_y = ypos_l >> 6;
            assert(base_y >= -(1 + mrl_idx));
            const int shift = (ypos_l & 0x3F) >> 1;
            if (is_luma) {
                const int v =
                    dr_interp_filter[shift].a * left[-(base_y + 1)] +
                    dr_interp_filter[shift].b * left[-(base_y + 2)] +
                    dr_interp_filter[shift].c * left[-(base_y + 3)] +
                    dr_interp_filter[shift].d * left[-(base_y + 4)];
                dst[x] = iclip_pixel((v + 64) >> 7);
            } else {
                const int v = (32 - shift) * left[-(base_y + 2)] +
                              shift * left[-(base_y + 3)];
                dst[x] = iclip_pixel((v + 16) >> 5);
            }
        }

        for (; x < width; x++, xpos += 64) {
            const int base_x = xpos >> 6;
            const int shift = (xpos & 0x3F) >> 1;
            if (is_luma) {
                const int v =
                    dr_interp_filter[shift].a * top[base_x + 1] +
                    dr_interp_filter[shift].b * top[base_x + 2] +
                    dr_interp_filter[shift].c * top[base_x + 3] +
                    dr_interp_filter[shift].d * top[base_x + 4];
                dst[x] = iclip_pixel((v + 64) >> 7);
            } else {
                const int v = (32 - shift) * top[base_x + 2] +
                              shift * top[base_x + 3];
                dst[x] = iclip_pixel((v + 16) >> 5);
            }
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
    const int angle_flags = angle & ~(511 | ANGLE_IBP_FLAG);
    const int is_luma = angle & ANGLE_IS_LUMA;
    const int is_sm_l = !!(angle & ANGLE_SMOOTH_LEFT_EDGE_FLAG);
    const int enable_intra_edge_filter = !!(angle & ANGLE_USE_EDGE_FILTER_FLAG);
    const int have_left = !!(angle & ANGLE_HAS_LEFT_FLAG);
    const int enable_ibp = !!(angle & ANGLE_IBP_FLAG);
    const int mrl_idx =
        (angle & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int mrl_mul = !!(angle & ANGLE_MULTI_MRL_FLAG);
    angle &= 511;
    assert(angle > 180);

    if (mrl_mul) {
        const int e_stride = (width + height + (mrl_idx << 1) + 3) * 2;
        const pixel *tl2 = &topleft_in[-e_stride];
        pixel tmp[64 * 64];
        assert(is_luma);
        ipred_z3_c(tmp, 64 * sizeof(pixel), topleft_in, width, height,
                   angle | (mrl_idx << ANGLE_MRL_IDX_SHIFT) | ANGLE_IS_LUMA,
                   max_width, max_height HIGHBD_TAIL_SUFFIX);
        ipred_z3_c(dst, stride, tl2, width, height, angle | ANGLE_IS_LUMA,
                   max_width, max_height HIGHBD_TAIL_SUFFIX);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++)
                dst[x] = (tmp[y * 64 + x] + dst[x] + 1) >> 1;
            dst += PXSTRIDE(stride);
        }
        return;
    }

    const int dy = dav2d_dr_intra_derivative[270 - angle];
    const int max_base_y = width + height - 1 + (mrl_idx << 1);

    // Buffer organization:
    // - 1 pixel top padding;
    // - 1 pixel top/left;
    // - mrl_idx pixels extra between top/left and left (max 3);
    // - height pixels left;
    // - width pixels bottom/left;
    // - 2 * mrl_idx pixels extra between bottom/left and bottom padding (max 2 * 3);
    // - 2 pixels bottom padding.
    pixel filt[1 + 1 + 3 + 64 + 64 + 2 * 3 + 2];
    pixel *const left = &filt[1 + width + height + mrl_idx * 2];
    const int n_px = width + height;
    const int str = enable_intra_edge_filter && !mrl_idx && have_left ?
        get_filter_strength(n_px, angle - 180, is_sm_l) : 0;
    const int sz = 1 + mrl_idx + width + height + mrl_idx * 2;
    if (str) {
        filter_edge(&filt[2], sz, height - max_height, sz - 1,
                    &topleft_in[1 - sz], 0, sz, str);
    } else {
        pixel_copy(&filt[2], &topleft_in[1 - sz], sz);
    }
    filt[0] = filt[1] = filt[2];
    filt[sz + 2] = filt[sz + 1];

    int ypos = dy * (1 + mrl_idx);
    for (int x = 0; x < width; x++, ypos += dy) {
        const int shift = (ypos & 0x3F) >> 1;
        const DRFilter4Tap f = dr_interp_filter[shift];
        for (int y = 0, base = ypos >> 6; y < height; y++, base++) {
            if (base <= max_base_y) {
                if (is_luma) {
                    const int v = f.a * left[-(base - 1)] + f.b * left[-base] +
                                  f.c * left[-(base + 1)] + f.d * left[-(base + 2)];
                    dst[y * PXSTRIDE(stride) + x] = iclip_pixel((v + 64) >> 7);
                } else {
                    const int v = (32 - shift) * left[-base] +
                                  shift * left[-(base + 1)];
                    dst[y * PXSTRIDE(stride) + x] = iclip_pixel((v + 16) >> 5);
                }
            } else {
                do {
                    dst[y * PXSTRIDE(stride) + x] = left[-max_base_y];
                } while (++y < height);
                break;
            }
        }
    }

    if (enable_ibp) {
        // I've observed the following values here:
        // angle | mode_idx |      intra mode | angle_delta | comment
        //   186 |        0 |        hor_pred |          +2 |
        //   197 |        1 |     hor_up_pred |          -2 |
        //   203 |        2 |     hor_up_pred |           0 |
        //   209 |        3 |     hor_up_pred |          +2 |
        //   219 |        4 | diag_down_right |          -2 |
        //   225 |        5 | diag_down_right |           0 |
        //   231 |        6 | diag_down_right |          +2 |
        //   241 |        6 |  diag_down_left |          -2 | wide_angle_remap
        //   247 |        6 |  diag_down_left |           0 | wide_angle_remap
        //   253 |        6 |  diag_down_left |          +2 | wide_angle_remap
        const int mode_idx = imin((angle - 183) >> 3, 6);
        pixel tmp[64 * 64];
        ipred_z1_c(tmp, 64 * sizeof(pixel), topleft_in, width, height,
                   (angle - 180) | angle_flags,
                   max_width, max_height HIGHBD_TAIL_SUFFIX);
        ibp_blend(dst, stride, tmp, width, height, 1,
                  dav2d_ibp_weights[mode_idx] HIGHBD_TAIL_SUFFIX);
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

static int cfl_dc_420(uint16_t *const edge,
                      const pixel *const top, const pixel *left,
                      const ptrdiff_t stride, const int w, const int h,
                      const int filter_type)
{
    const int is_top_sb_edge = filter_type & CFL_IS_TOP_SB_EDGE;
    int dc = 0, v;
    if (filter_type & 2) {
        const ptrdiff_t above = is_top_sb_edge ? 0 : -PXSTRIDE(stride);
        for (int i = 0; i < w; i += 2) {
            v = top[imax(0, i - 1)] + 4 * top[i] + top[i + 1] +
                top[i + above] + top[i + PXSTRIDE(stride)];
            edge[i >> 1] = v;
            dc += v;
        }
        for (int i = 0; i < h; i += 2, left += 2 * PXSTRIDE(stride)) {
            v = left[-1] + 4 * left[0] + left[1] +
                left[i ? -PXSTRIDE(stride) : 0] + left[PXSTRIDE(stride)];
            edge[-1 - (i >> 1)] = v;
            dc += v;
        }
    } else if (filter_type & 1) {
        for (int i = 0; i < w; i += 2) {
            v = top[imax(0, i - 1)] + 2 * top[i] + top[i + 1] +
                top[imax(0, i - 1) + PXSTRIDE(stride)] +
                2 * top[i + PXSTRIDE(stride)] + top[i + 1 + PXSTRIDE(stride)];
            edge[i >> 1] = v;
            dc += v;
        }
        for (int i = 0; i < h; i += 2, left += 2 * PXSTRIDE(stride)) {
            v = left[-1] + 2 * left[0] + left[1] + left[-1 + PXSTRIDE(stride)] +
                2 * left[PXSTRIDE(stride)] + left[1 + PXSTRIDE(stride)];
            edge[-1 - (i >> 1)] = v;
            dc += v;
        }
    } else {
        for (int i = 0; i < w; i += 2) {
            v = (top[i] + top[i + 1] +
                 top[i + PXSTRIDE(stride)] + top[i + 1 + PXSTRIDE(stride)]) << 1;
            edge[i >> 1] = v;
            dc += v;
        }
        for (int i = 0; i < h; i += 2, left += 2 * PXSTRIDE(stride)) {
            v = (left[0] + left[1] +
                 left[PXSTRIDE(stride)] + left[1 + PXSTRIDE(stride)]) << 1;
            edge[-1 - (i >> 1)] = v;
            dc += v;
        }
    }
    return dc;
}

static int cfl_dc_422(uint16_t *const edge,
                      const pixel *const top, const pixel *left,
                      const ptrdiff_t stride, const int w, const int h,
                      const int filter_type)
{
    int dc = 0, v;
    if (filter_type & 2) {
        for (int i = 0; i < w; i += 2) {
            v = top[i] << 3;
            edge[i >> 1] = v;
            dc += v;
        }
        for (int i = 0; i < h; i += 2, left += PXSTRIDE(stride)) {
            v = left[0] << 3;
            edge[-1 - i] = v;
            dc += v;
        }
    } else if (filter_type & 1) {
        for (int i = 0; i < w; i += 2) {
            v = (top[imax(0, i - 1)] + 2 * top[i] + top[i + 1]) << 1;
            edge[i >> 1] = v;
            dc += v;
        }
        for (int i = 0; i < h; i += 2, left += PXSTRIDE(stride)) {
            v = (left[-1] + 2 * left[0] + left[1]) << 1;
            edge[-1 - i] = v;
            dc += v;
        }
    } else {
        for (int i = 0; i < w; i += 2) {
            v = (top[i] + top[i + 1]) << 2;
            edge[i >> 1] = v;
            dc += v;
        }
        for (int i = 0; i < h; i += 2, left += PXSTRIDE(stride)) {
            v = (left[0] + left[1]) << 2;
            edge[-1 - i] = v;
            dc += v;
        }
    }
    return dc;
}

static int cfl_dc_444(uint16_t *const edge,
                      const pixel *const top, const pixel *left,
                      const ptrdiff_t stride, const int w, const int h,
                      const int filter_type)
{
    int dc = 0, v;
    for (int i = 0; i < w; i++) {
        v = top[i] << 3;
        edge[i] = v;
        dc += v;
    }
    for (int i = 0; i < h; i++) {
        v = left[i * PXSTRIDE(stride)] << 3;
        edge[-1 - i] = v;
        dc += v;
    }
    return dc;
}

#define cfl_dc_fn(fmt, ss_hor, ss_ver) \
static int cfl_dc_##fmt##_c(uint16_t *const edge, \
                            const pixel *const top, const pixel *const left, \
                            const ptrdiff_t stride, const int wpad, const int hpad, \
                            const int w, const int h, const int filter_type) \
{ \
    const int xlim = w - 4 * (wpad << ss_hor); \
    const int ylim = h - 4 * (hpad << ss_ver); \
    int dc = cfl_dc_##fmt(edge, top, left, stride, xlim, ylim, filter_type); \
    for (int i = xlim >> ss_hor; i < w >> ss_hor; i++) { \
        edge[i] = edge[(xlim >> ss_hor) - 1]; \
        dc += edge[i]; \
    } \
    for (int i = ylim >> ss_ver; i < h >> ss_ver; i++) { \
        edge[-1 - i] = edge[-(ylim >> ss_ver)]; \
        dc += edge[-1 - i]; \
    } \
    return fast_div32_dc(dc, (w >> ss_hor) + (h >> ss_ver)); \
}

cfl_dc_fn(420, 1, 1)
cfl_dc_fn(422, 1, 0)
cfl_dc_fn(444, 0, 0)

static NOINLINE void
cfl_ac_c(int16_t *ac, const int dc, const pixel *ypx, const ptrdiff_t stride,
         const int w_pad, const int h_pad, const int width, const int height,
         const int filter_type, const int ss_hor, const int ss_ver)
{
    int y, x;

    assert(w_pad >= 0 && w_pad * 4 < width);
    assert(h_pad >= 0 && h_pad * 4 < height);

    for (y = 0; y < height - 4 * h_pad; y++) {
        for (x = 0; x < width - 4 * w_pad; x++) {
            const int left = imax((x * 2) & -64, x * 2 - 1);
            if (!(ss_hor | ss_ver)) {
                ac[x] = ypx[x << ss_hor] << 3;
            } else if (!(ss_hor ^ ss_ver)) {
                const ptrdiff_t bot = x * 2 + PXSTRIDE(stride);
                if (filter_type & 2) {
                    const ptrdiff_t top = (y & 63) == 0 ? x * 2 : (x * 2 - PXSTRIDE(stride));
                    ac[x] = ypx[left] + 4 * ypx[x * 2] + ypx[x * 2 + 1] +
                            ypx[top] + ypx[bot];
                } else if (filter_type & 1) {
                    ac[x] = ypx[left] + 2 * ypx[x * 2] + ypx[x * 2 + 1] +
                            ypx[left + PXSTRIDE(stride)] +
                            2 * ypx[bot] + ypx[bot + 1];
                } else {
                    ac[x] = (ypx[x * 2] + ypx[x * 2 + 1] +
                             ypx[bot] + ypx[bot + 1]) << 1;
                }
            } else {
                if (filter_type & 2)
                    ac[x] = ypx[x * 2] << 3;
                else if (filter_type & 1)
                    ac[x] = (ypx[left] + 2 * ypx[x * 2] + ypx[x * 2 + 1]) << 1;
                else
                    ac[x] = (ypx[x * 2] + ypx[x * 2 + 1]) << 2;
            }
            ac[x] -= dc;
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
}

#define cfl_ac_fn(fmt, ss_hor, ss_ver) \
static void cfl_ac_##fmt##_c(int16_t *const ac, const int dc, \
                             const pixel *const ypx, const ptrdiff_t stride, \
                             const int w_pad, const int h_pad, \
                             const int cw, const int ch, const int filter_type) \
{ \
    cfl_ac_c(ac, dc, ypx, stride, w_pad, h_pad, cw, ch, filter_type, ss_hor, ss_ver); \
}

cfl_ac_fn(420, 1, 1)
cfl_ac_fn(422, 1, 0)
cfl_ac_fn(444, 0, 0)

static NOINLINE void
cfl_gen_y_420_c(uint16_t *dst, const int dst_stride,
                const pixel *src, const pixel *const top_sb_edge,
                const ptrdiff_t src_stride, const int refw, const int refh,
                const int tw, const int th, const int flags, const int filter_type)
{
    const int has_t = flags & CFL_HAS_TOP;
    const int has_l = flags & CFL_HAS_LEFT;
    const int dir = flags & CFL_DIR_ALL;
    const int n_left = has_l ? 1 + (dir == CFL_DIR_LEFT): 0;
    const int n_top = has_t ? 1 + (dir == CFL_DIR_TOP) : 0;
    src -= n_left << 1;

// ::
#define FILTER_CENTER(src) \
    (src[c] + src[r] + src[b + c] + src[b + r]) >> 2
// :::
#define FILTER_RECT(src) \
    (src[l] + 2 * src[c] + src[r] + src[b + l] + 2 * src[b + c] + src[b + r]) >> 3
// -|-
#define FILTER_CROSS(src, top) \
    (src[l] + 4 * src[c] + src[r] + top[c] + src[b + c]) >> 3

    // tl+t+tr
    if (has_t) {
        const pixel *top = top_sb_edge ?
            top_sb_edge - n_left * 2 : src - n_top * 2 * src_stride;
        const ptrdiff_t a = !top_sb_edge ? -src_stride : 0;
        const ptrdiff_t b = !top_sb_edge ? src_stride : 0;
        for (int y = 0; y < n_top; y++) {
            for (int x = 0; x < refw; x++) {
                const int c = x * 2, l = imax(c - 1, 0), r = c + 1;
                dst[x] = filter_type & 2 ? FILTER_CROSS(top, (&top[a])) :
                         filter_type & 1 ? FILTER_RECT(top) : FILTER_CENTER(top);
            }
            if (!top_sb_edge)
                top += 2 * src_stride;
            dst += dst_stride;
        }
    }

    // l+blk
    const ptrdiff_t b = src_stride;
    const pixel *top = top_sb_edge ? top_sb_edge - n_left * 2 : src - src_stride;
    for (int y = 0; y < th; y++) {
        for (int x = 0; x < n_left + tw; x++) {
            const int c = x * 2, l = imax(c - 1, 0), r = c + 1;
            dst[x] = filter_type & 2 ? FILTER_CROSS(src, (top)) :
                     filter_type & 1 ? FILTER_RECT(src) : FILTER_CENTER(src);
        }
        src += src_stride << 1;
        dst += dst_stride;
        top = src - src_stride;
    }

    // bl
    if (refh > th) {
        const int n_bl = refh - th;
        for (int y = 0; y < n_bl; y++) {
            for (int x = 0; x < n_left; x++) {
                const int c = x * 2, l = imax(c - 1, 0), r = c + 1;
                dst[x] = filter_type & 2 ? FILTER_CROSS(src, (top)) :
                         filter_type & 1 ? FILTER_RECT(src) : FILTER_CENTER(src);
            }
            src += src_stride << 1;
            dst += dst_stride;
        }
    }
}

#define cfl_gen_y_420_fn(filter_type, name) \
static void \
cfl_gen_y_420_##name##_c(uint16_t *const dst, const int dst_stride, \
                         const pixel *const src, const pixel *const top_sb_edge, \
                         ptrdiff_t const src_stride, const int refw, const int refh, \
                         int const tw, int const th, int flags) \
{ \
    cfl_gen_y_420_c(dst, dst_stride >> 1, src, top_sb_edge, PXSTRIDE(src_stride), \
                    refw, refh, tw, th, flags, filter_type); \
}

#define cfl_gen_y_fn(fmt) \
cfl_gen_y_##fmt##_fn(0, center) \
cfl_gen_y_##fmt##_fn(1, rect) \
cfl_gen_y_##fmt##_fn(2, cross)

cfl_gen_y_fn(420)

#define SQRND(v) (((v) * (v) + mid) >> bd)
#define GEN_MATRIX() \
    do { \
        mat[0][0] += v0 * v0; \
        mat[0][1] += v0 * v1; \
        mat[0][2] += v0 << (bd - 1); \
        mat[1][1] += v1 * v1; \
        mat[1][2] += v1 << (bd - 1); \
    } while (0);

static void
cfl_gen_mat_c(int32_t mat[3][3], uint16_t imat[2][CFL_MAX_EDGE_SAMPLES],
              const uint16_t *const y, const int ystride,
              const int refw, const int refh, const int edge_flags,
              const enum CflMhDir dir HIGHBD_DECL_SUFFIX)
{
    const int bd = bitdepth_from_max(bitdepth_max);
    const int mid = 1 << (bd - 1);
    const int has_t = !!(edge_flags & CFL_HAS_TOP);
    const int has_l = !!(edge_flags & CFL_HAS_LEFT);
    const int dir_t = dir == CFL_DIR_TOP;
    const int dir_l = dir == CFL_DIR_LEFT;

    int n = 0;
    if (has_t) {
        for (int i = !dir_l && !has_l; i < refw - 1 - (dir_l && !has_l); i++, n++) {
            const int v0 = y[i];
            const int v1 = SQRND(y[dir_t * ystride + i + dir_l]);
            imat[0][n] = v0;
            imat[1][n] = v1;
            GEN_MATRIX();
        }
    }
    // XXX it will probably be faster to fill an additional left edge buffer
    // for SIMD, so that we can do a single load of all left, vs h - n_top
    // loads of 1 or 2 px...
    // Must be additional tho, as mhccp_pred(dir=left) will tap into the edge
    // => don't need extra arg, can just offset luma buffer and store left then
    // everything (including left again then)
    // One more thing, if dir=left the left edge tap (in mhccp_pred) is of 1px,
    // but the system of linear equations uses 2px, so there will be a
    // difference in what *needs* to be stored in the left part and in the
    // general part, altho it will probably be faster to not care and just
    // store 2px in the general part anyways.
    if (has_l) {
        for (int i = has_t; i < refh - 1; i++, n++) {
            const int v0 = y[i * ystride];
            const int v1 = SQRND(y[(i + dir_t) * ystride + dir_l]);
            imat[0][n] = v0;
            imat[1][n] = v1;
            GEN_MATRIX();
        }
    }
    mat[2][2] = n << ((bd - 1) << 1);

    const int nl2 = 31 - clz(n);
    const int mat_sh = 22 - 2 * bd - nl2 - !!(n & ((1 << nl2) - 1));
    if (mat_sh > 0)
        for (int i = 0; i < 3; i++)
            for (int j = i; j < 3; j++)
                mat[i][j] <<= mat_sh;
    else if (mat_sh < 0)
        for (int i = 0; i < 3; i++)
            for (int j = i; j < 3; j++)
                mat[i][j] >>= -mat_sh;
    mat[0][0] += 2 << (bd - 8);
    mat[1][1] += 2 << (bd - 8);
    mat[2][2] += 2 << (bd - 8);
    mat[1][0] = mat[0][1];
    mat[2][0] = mat[0][2];
    mat[2][1] = mat[1][2];
}

#define cfl_gen_mat_fn(name, dir) \
static void \
cfl_gen_mat_##name##_c(int32_t mat[3][3], uint16_t imat[2][CFL_MAX_EDGE_SAMPLES], \
                       const uint16_t *const y, const int ystride, \
                       const int refw, const int refh, const int edge_flags \
                       HIGHBD_DECL_SUFFIX) \
{ \
    cfl_gen_mat_c(mat, imat, y, ystride >> 1, refw, refh, edge_flags, dir \
                  HIGHBD_TAIL_SUFFIX); \
}

cfl_gen_mat_fn(c, CFL_DIR_CENTER)
cfl_gen_mat_fn(t, CFL_DIR_TOP)
cfl_gen_mat_fn(l, CFL_DIR_LEFT)

static void get_div_scale_sh(int d, int *scale, int *sh) {
    *sh = ulog2(d);
    // 1. Normalize D into fixed-point format with 14 fractional bits.
    const int nsh = *sh - 14;
    if (nsh >= 0) {
        const int rnd = (nsh > 0) ? 1 << (nsh - 1) : 0;
        d = (d + rnd) >> nsh;
    } else {
        d <<= -nsh;
    }
    // 2. Clip the scaled value to make sure it's within the valid range
    // [1, 2), represented as： [1 << 14, (1 << 15) - 1].
    // The rounding in 1. may push the value out of range, so clipping is needed.
    // XXX looks like this could just be an imin
    d = iclip(d, 1, 0x7fff);
    // 3. Extract the fractional part of the normalized denominator d.
    d &= (1 << 14) - 1;

    const int idx = d >> 11;
    const uint8_t coefw = dav2d_div_scale_sh_coefw[idx];
    const uint16_t bias = dav2d_div_scale_sh_bias[idx];
    d -= dav2d_div_scale_sh_offset[idx];
    *scale = (((coefw * ((d * d) >> 14)) >> 8) - (d >> 1) + bias) << 2;
}

/*
 * Approximate ((a * b) + round) >> shift using only 32-bit intermediates.
 * Strategy:
 *   1. Right-shift a and/or b by sh1/sh2 so that (bits(a)-sh1)+(bits(b)-sh2) <= 31.
 *   2. Compensate in the final right shift: adj = sh - (ash + bsh).
 *   3. Perform symmetric round-to-nearest (round half away from zero).
 * This keeps all intermediates in 32-bit while keeping the mean error low.
 */
static int mul32(int a, int b, int sh) {
    const int a2 = ulog2(abs(a)) + 1;
    const int b2 = ulog2(abs(b)) + 1;
    // 1. Decide how many bits to drop in total to avoid mul overflow
    const int drop = a2 + b2 > 29 ? a2 + b2 - 29 : 0;
    // 2. Split the drop across a and b to minimize error
    const int ash = drop >> 1;
    const int bsh = drop - ash;
    const int adj = sh - (ash + bsh);
    const int mul = (a >> ash) * (b >> bsh);
    if (adj <= 0) return mul;
    assert(adj <= 29);
    // 3. Final right shift with symmetric rounding to nearest
    const unsigned bias = 1U << (adj - 1);
    return mul >= 0 ?
        (int) (((unsigned) mul + bias) >> adj) :
        -(int) (((unsigned) -mul + bias) >> adj);
}

static void
cfl_calc_alphas_c(int alpha[3], const pixel *const c,
                  const pixel *const top_sb_edge, ptrdiff_t stride,
                  const int refw, const int refh,
                  int32_t mat[3][3], const uint16_t imat[2][CFL_MAX_EDGE_SAMPLES],
                  const int edge_flags HIGHBD_DECL_SUFFIX)
{
    const int bd = bitdepth_from_max(bitdepth_max);
    const int has_t = !!(edge_flags & CFL_HAS_TOP);
    const int has_l = !!(edge_flags & CFL_HAS_LEFT);

    int n = 0;
    if (has_t) {
        const pixel *const top = top_sb_edge ?
            top_sb_edge - has_l : c - PXSTRIDE(stride) - has_l;
        for (int i = !has_l; i < refw - 1; i++, n++) {
            alpha[0] += imat[0][n] * top[i];
            alpha[1] += imat[1][n] * top[i];
            alpha[2] += top[i] << (bd - 1);
        }
    }
    if (has_l) {
        for (int i = 0; i < refh - 2; i++, n++) {
            const int v = c[i * PXSTRIDE(stride) - 1];
            alpha[0] += imat[0][n] * v;
            alpha[1] += imat[1][n] * v;
            alpha[2] += v << (bd - 1);
        }
    }
    const int nl2 = 31 - clz(n);
    const int mat_sh = 22 - 2 * bd - nl2 - !!(n & ((1 << nl2) - 1));
    if (mat_sh > 0) {
        alpha[0] <<= mat_sh;
        alpha[1] <<= mat_sh;
        alpha[2] <<= mat_sh;
    } else if (mat_sh < 0) {
        alpha[0] >>= -mat_sh;
        alpha[1] >>= -mat_sh;
        alpha[2] >>= -mat_sh;
    }
    // alpha holds the results of the system of linear equations

    // Gaussian elimination
    int tmp[3][2], scale, sh;
    // row0
    get_div_scale_sh(mat[0][0], &scale, &sh);
    tmp[0][0] = mul32(mat[0][1], scale, sh);
    tmp[0][1] = mul32(mat[0][2], scale, sh);
    alpha[0]  = mul32(alpha[0],  scale, sh);
    tmp[1][0] = mat[1][1] - mul32(mat[1][0], tmp[0][0], 16);
    tmp[1][1] = mat[1][2] - mul32(mat[1][0], tmp[0][1], 16);
    alpha[1]  -= mul32(mat[1][0], alpha[0],  16);
    tmp[2][0] = mat[2][1] - mul32(mat[2][0], tmp[0][0], 16);
    tmp[2][1] = mat[2][2] - mul32(mat[2][0], tmp[0][1], 16);
    alpha[2]  -= mul32(mat[2][0], alpha[0],  16);
    // row1
    get_div_scale_sh(tmp[1][0], &scale, &sh);
    tmp[1][1] = mul32(tmp[1][1], scale, sh);
    alpha[1]  = mul32(alpha[1],  scale, sh);
    tmp[2][1] -= mul32(tmp[2][0], tmp[1][1], 16);
    alpha[2]  -= mul32(tmp[2][0], alpha[1],  16);
    // row2
    get_div_scale_sh(tmp[2][1], &scale, &sh);
    alpha[2] = mul32(alpha[2], scale, sh);
    alpha[1] -= mul32(tmp[1][1], alpha[2], 16);
    alpha[0] -= mul32(tmp[0][0], alpha[1], 16) + mul32(tmp[0][1], alpha[2], 16);
}

static void
cfl_mhccp_pred_c(pixel *dst, const ptrdiff_t dst_stride,
                 const uint16_t *src, const int src_stride,
                 const int w, const int h, const int alpha[3],
                 const int edge_flags, const enum CflMhDir dir HIGHBD_DECL_SUFFIX)
{
    const int bd = bitdepth_from_max(bitdepth_max);
    const int mid = 1 << (bd - 1);
    const int has_t = !!(edge_flags & CFL_HAS_TOP);
    const int has_l = !!(edge_flags & CFL_HAS_LEFT);

    const int a2v2 = mul32(alpha[2], mid, 16);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const int v0 = src[dir == CFL_DIR_TOP ? x - ((!!y) | has_t) * src_stride :
                               dir == CFL_DIR_LEFT ? x - ((!!x) | has_l) : x];
            const int v1 = SQRND(src[x]);
            dst[x] = iclip_pixel(mul32(alpha[0], v0, 16) +
                                 mul32(alpha[1], v1, 16) + a2v2);
        }
        src += src_stride;
        dst += dst_stride;
    }
}

#define cfl_mhccp_pred_fn(name, dir) \
static void \
cfl_mhccp_pred_##name##_c(pixel *dst, ptrdiff_t dst_stride, \
                          const uint16_t *src, const int src_stride, \
                          int w, int h, const int alpha[3], int edge_flags \
                          HIGHBD_DECL_SUFFIX) \
{ \
    cfl_mhccp_pred_c(dst, dst_stride, src, src_stride >> 1, w, h, alpha, \
                     edge_flags, dir HIGHBD_TAIL_SUFFIX); \
}

cfl_mhccp_pred_fn(c, CFL_DIR_CENTER)
cfl_mhccp_pred_fn(t, CFL_DIR_TOP)
cfl_mhccp_pred_fn(l, CFL_DIR_LEFT)

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
                sum += dav2d_dip_weights[m][idx][i] * in[i];
            }
            dst[y * PXSTRIDE(stride) + x] =
                iclip_pixel(((sum + 2048) >> 12) - in_sum);
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

COLD void bitfn(dav2d_intra_pred_dsp_init)(Dav2dIntraPredDSPContext *const c) {
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

    /* CFL EXPLICIT / IMPLICIT */
    c->cfl_dc[DAV2D_PIXEL_LAYOUT_I420 - 1] = cfl_dc_420_c;
    c->cfl_dc[DAV2D_PIXEL_LAYOUT_I422 - 1] = cfl_dc_422_c;
    c->cfl_dc[DAV2D_PIXEL_LAYOUT_I444 - 1] = cfl_dc_444_c;
    c->cfl_ac[DAV2D_PIXEL_LAYOUT_I420 - 1] = cfl_ac_420_c;
    c->cfl_ac[DAV2D_PIXEL_LAYOUT_I422 - 1] = cfl_ac_422_c;
    c->cfl_ac[DAV2D_PIXEL_LAYOUT_I444 - 1] = cfl_ac_444_c;
    c->cfl_pred[DC_PRED     ] = ipred_cfl_c;
    c->cfl_pred[DC_128_PRED ] = ipred_cfl_128_c;
    c->cfl_pred[TOP_DC_PRED ] = ipred_cfl_top_c;
    c->cfl_pred[LEFT_DC_PRED] = ipred_cfl_left_c;

    /* CFL_MHCCP */
#define assign_cfl_mhccp(dir, name) \
    c->cfl_gen_y[DAV2D_PIXEL_LAYOUT_I420 - 1][0] = cfl_gen_y_420_center_c; \
    c->cfl_gen_y[DAV2D_PIXEL_LAYOUT_I420 - 1][1] = cfl_gen_y_420_rect_c; \
    c->cfl_gen_y[DAV2D_PIXEL_LAYOUT_I420 - 1][2] = cfl_gen_y_420_cross_c; \
    c->cfl_gen_mat[dir] = cfl_gen_mat_##name##_c; \
    c->cfl_calc_alphas = cfl_calc_alphas_c; \
    c->cfl_mhccp_pred[dir] = cfl_mhccp_pred_##name##_c;

    assign_cfl_mhccp(CFL_DIR_CENTER, c);
    assign_cfl_mhccp(CFL_DIR_TOP   , t);
    assign_cfl_mhccp(CFL_DIR_LEFT  , l);

    c->pal_pred = pal_pred_c;

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
