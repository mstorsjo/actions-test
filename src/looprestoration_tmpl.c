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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "common/attributes.h"
#include "common/bitdepth.h"
#include "common/intops.h"

#include "src/looprestoration.h"
#include "src/tables.h"

// 512 * 1.5 + 4 + 4
#define REST_UNIT_STRIDE (776)
// (512 * 1.5) >> 2
#define CLASS_BUF_SIZE (192)

static const int8_t wiener_ns_config_y[32][2] = {
    { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 },
    { 2, 0 }, { -2, 0 }, { 0, 2 }, { 0, -2 },
    { 1, 1 }, { -1, -1 }, { -1, 1 }, { 1, -1 },
    { 2, 1 }, { -2, -1 }, { 2, -1 }, { -2, 1 },
    { 1, 2 }, { -1, -2 }, { 1, -2 }, { -1, 2 },
    { 3, 0 }, { -3, 0 }, { 0, 3 }, { 0, -3 },
    { 4, 0 }, { -4, 0 }, { 0, 4 }, { 0, -4 },
    { 3, 3 }, { -3, -3 }, { 3, -3 }, { -3, 3 }
};

static const int8_t pc_wiener_config[25][2] = {
    {  1,  0 }, { -1,  0 }, {  0,  1 }, {  0, -1 }, {  2,  0 },
    { -2,  0 }, {  0,  2 }, {  0, -2 }, {  1,  1 }, { -1, -1 },
    { -1,  1 }, {  1, -1 }, {  2,  1 }, { -2, -1 }, {  2, -1 },
    { -2,  1 }, {  1,  2 }, { -1, -2 }, {  1, -2 }, { -1,  2 },
    {  3,  0 }, { -3,  0 }, {  0,  3 }, {  0, -3 }, {  0,  0 }
};

static const uint16_t pc_wiener_normalizer[ 4 ] = { 3739, 3273, 3074, 7 };

static const int16_t mode_weights[ 4 ][ 3 ] = {
    { -527, 15325, 321 }, { 26436, -17705, 17905 }, { 366, -147, -194 }, { 202, -267, -179 }
};

static const int16_t mode_offsets[ 4 ] = { -547, -21565, -573, -680 };

static void backup_row(pixel *dst, const pixel *src, const pixel *left, const int w, const enum LrEdgeFlags edges) {
    if (edges & LR_HAVE_LEFT)
        for (int x = -4; x < 0; x++)
            dst[x] = left[x + 4];
    else
        for (int x = -4; x < 0; x++)
            dst[x] = src[0];

    for (int x = 0; x < w; x++)
        dst[x] = src[x];

    if (edges & LR_HAVE_RIGHT)
        for (int x = w; x < w + 4; x++)
            dst[x] = src[x];
    else
        for (int x = w; x < w + 4; x++)
            dst[x] = src[w - 1];
}

static void backup_row_lpf(pixel *dst, const pixel *src, const int w, const enum LrEdgeFlags edges) {
    if (edges & LR_HAVE_LEFT) {
        for (int x = -4; x < 0; x++)
            dst[x] = src[x];
    } else {
        for (int x = -4; x < 0; x++)
            dst[x] = src[0];
    }

    for (int x = 0; x < w; x++)
        dst[x] = src[x];

    if (edges & LR_HAVE_RIGHT) {
        for (int x = w; x < w + 4; x++)
            dst[x] = src[x];
    } else {
        for (int x = w; x < w + 4; x++)
            dst[x] = src[w - 1];
    }
}

static void ns_wiener_single_y_c(pixel *p, const ptrdiff_t stride,
                                 const pixel (*left)[4],
                                 const pixel *lpf, const int w, int h,
                                 const WienerParams *params,
                                 const enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
    const int8_t *filter = params->single.filter;
    pixel row_buffers[9][REST_UNIT_STRIDE];
    pixel *bak_rows[9];
    const pixel *ptrs[9];
    const pixel *lpf_bottom = lpf + 6*PXSTRIDE(stride);

    for (int i = 0; i < 9; i++)
        bak_rows[i] = row_buffers[i] + 4;

    backup_row(bak_rows[4], p, left[0], w, edges);
    ptrs[4] = bak_rows[4];
    if (edges & LR_HAVE_TOP) {
        // y = -2,-1
        backup_row_lpf(bak_rows[2], lpf, w, edges);
        ptrs[2] = bak_rows[2];
        backup_row_lpf(bak_rows[3], lpf + PXSTRIDE(stride), w, edges);
        ptrs[3] = bak_rows[3];

        // y = -3,-4
        ptrs[0] = ptrs[1] = ptrs[2];
    } else {
        ptrs[0] = ptrs[1] = ptrs[2] = ptrs[3] = ptrs[4];
    }

    backup_row(bak_rows[5], p + PXSTRIDE(stride), left[1], w, edges);
    ptrs[5] = bak_rows[5];
    backup_row(bak_rows[6], p + 2*PXSTRIDE(stride), left[2], w, edges);
    ptrs[6] = bak_rows[6];
    backup_row(bak_rows[7], p + 3*PXSTRIDE(stride), left[3], w, edges);
    ptrs[7] = bak_rows[7];
    int bak_idx = 8;

    for (int y = 0; y < h; y++) {
        if (y + 4 < h) {
            backup_row(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), left[y + 4], w, edges);
            ptrs[8] = bak_rows[bak_idx];
        } else if (y + 2 < h && edges & LR_HAVE_BOTTOM) {
            int offset_y = y + 4 - h;
            assert(offset_y < 2);
            backup_row_lpf(bak_rows[bak_idx], lpf_bottom + offset_y * PXSTRIDE(stride), w, edges);
            ptrs[8] = bak_rows[bak_idx];
        } else {
            ptrs[8] = ptrs[7];
        }
        if (++bak_idx == 9) bak_idx = 0;

        for (int x = 0; x < w; x++) {
            const int m = ptrs[4][x];
            int s = m << 7;
            for (int i = 0; i < 32; i++) {
                const int dy = wiener_ns_config_y[i][0];
                const int dx = wiener_ns_config_y[i][1];
                const int diff = ptrs[4 + dy][x + dx] - m;
                s += diff * filter[i >> 1];
            }
            // TODO: chroma: if (plane > 0) {...}
            const int v = (s + 64) >> 7;
            p[x] = iclip_pixel(v);
        }

        for (int r = 0; r < 8; r++) ptrs[r] = ptrs[r+1];
        p += PXSTRIDE(stride);
    }
}


static int get_qval_given_tskip(int qstep, int tskip, int i, int bitdepth_min_8) {
    qstep = (qstep + (1 << bitdepth_min_8 >> 1)) >> bitdepth_min_8;
    int prod = (tskip * qstep + 128) >> 8;
    int qval = mode_weights[i][0] * (tskip << 5) + mode_weights[i][1] * qstep + mode_weights[i][2] * prod;
    int abs_qval = abs(qval);
    qval = apply_sign((abs_qval + (1 << 12)) >> 13, qval);
    qval = 255 * (mode_offsets[i] + qval);
    return qval;
}

static int get_class_lut_idx(const pixel *ptrs[10], const uint16_t (*noskip_mask)[12], const int base_q,
                             const int bx, const int by, const int bh, const int bitdepth_min_8) {
    int f[3] = {0, 0, 0};
    int s = 0;

    for (int dy = -1; dy <= 4; dy++) {
        for (int dx = -1; dx <= 4; dx++) {
            const int x = (bx << 2) + dx;
            const int y = 4 + dy;
            const int m = ptrs[y][x];
            const int up = ptrs[y - 1][x];
            const int down = ptrs[y + 1][x];
            const int vert = up - 2 * m + down;

            const int up_right = ptrs[y - 1][x + 1];
            const int down_left = ptrs[y + 1][x - 1];
            const int anti_diag = up_right - 2 * m + down_left;

            const int down_right = ptrs[y + 1][x + 1];
            const int up_left = ptrs[y - 1][x - 1];
            const int diag = up_left - 2 * m + down_right;

            f[0] += abs(vert);
            f[1] += abs(anti_diag);
            f[2] += abs(diag);
        }
    }

    // count skip masks for center, sides, and corners.
    const uint8_t num_pixels[3] = {16, 4, 1};
    const int sb64x = bx >> 4;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            const int edge = !!dy + !!dx;
            const int fx = iclip((bx & 15) + dx, 0, 15);
            const int fy = iclip(by + dy, 0, bh - 1);
            s += num_pixels[edge] * !((noskip_mask[fy][sb64x] >> fx) & 1);
        }
    }

    const int rnd = 1 << bitdepth_min_8 >> 1;
    for (int i = 0; i < 3; i++) {
        f[i] = (f[i] * pc_wiener_normalizer[i] + rnd) >> bitdepth_min_8;
    }
    s = s * pc_wiener_normalizer[3];

    int qval = (imax(0, get_qval_given_tskip(base_q, s, 0, bitdepth_min_8)) + (1 << 13)) >> 14;
    qval = imin(qval, 255) >> 5;
    int lut_idx = qval << 9;
    for (int i = 0; i < 3; i++) {
        qval = (imax(0, f[i] + get_qval_given_tskip(base_q, s, i + 1, bitdepth_min_8)) + (1 << 13)) >> 14;
        qval = imin(qval, 255) >> 5;
        lut_idx |= qval << (3 * (2-i));
    }
    return lut_idx;
}

static void wiener_multi(pixel *p, const ptrdiff_t stride,
                         const pixel (*left)[4],
                         const pixel *lpf, const int w, int h,
                         const int8_t (*filters_user)[18],
                         const int16_t (*filters_pretrained)[13],
                         const uint8_t *subclass_lut,
                         const uint16_t (*noskip_mask)[12],
                         const int base_q,
                         const enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
    const int bitdepth_min_8 = bitdepth_from_max(bitdepth_max) - 8;

    uint8_t classes[CLASS_BUF_SIZE];
    pixel row_buffers[10][REST_UNIT_STRIDE];
    pixel *bak_rows[10];
    const pixel *ptrs[10];
    const pixel *lpf_bottom = lpf + 6*PXSTRIDE(stride);

    for (int i = 0; i < 10; i++)
        bak_rows[i] = row_buffers[i] + 4;

    backup_row(bak_rows[4], p, left[0], w, edges);
    ptrs[4] = bak_rows[4];
    if (edges & LR_HAVE_TOP) {
        // y = -2,-1
        backup_row_lpf(bak_rows[2], lpf, w, edges);
        ptrs[2] = bak_rows[2];
        backup_row_lpf(bak_rows[3], lpf + PXSTRIDE(stride), w, edges);
        ptrs[3] = bak_rows[3];

        // y = -3,-4
        ptrs[0] = ptrs[1] = ptrs[2];
    } else {
        ptrs[0] = ptrs[1] = ptrs[2] = ptrs[3] = ptrs[4];
    }

    backup_row(bak_rows[5], p + PXSTRIDE(stride), left[1], w, edges);
    ptrs[5] = bak_rows[5];
    backup_row(bak_rows[6], p + 2*PXSTRIDE(stride), left[2], w, edges);
    ptrs[6] = bak_rows[6];
    backup_row(bak_rows[7], p + 3*PXSTRIDE(stride), left[3], w, edges);
    ptrs[7] = bak_rows[7];
    int bak_idx = 8;

    const int bh = h >> 2;
    const int bw = w >> 2;
    for (int by = 0; by < bh; by++) {
        // Backup an extra row to compute class
        if (by + 1 < bh) {
            // TODO: don't backup lines twice
            backup_row(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), left[(by << 2) + 4], w, edges);
            ptrs[8] = bak_rows[bak_idx];
            backup_row(bak_rows[9], p + 5*PXSTRIDE(stride), left[(by << 2) + 5], w, edges);
            ptrs[9] = bak_rows[9];
        } else if (edges & LR_HAVE_BOTTOM) {
            backup_row_lpf(bak_rows[bak_idx], lpf_bottom + 0 * PXSTRIDE(stride), w, edges);
            ptrs[8] = bak_rows[bak_idx];
            backup_row_lpf(bak_rows[9], lpf_bottom + 1 * PXSTRIDE(stride), w, edges);
            ptrs[9] = bak_rows[9];
        } else {
            ptrs[8] = ptrs[7];
            ptrs[9] = ptrs[7];
        }
        for (int bx = 0; bx < bw; bx++) {
            int lut_idx = get_class_lut_idx(ptrs, noskip_mask, base_q, bx, by, bh, bitdepth_min_8);
            // TODO: Convert these 2 lookup tables to a  Pre compute a single lookup table ahead of time
            int cls = dav2d_pc_weiner_lut_to_class[lut_idx];
            classes[bx] = subclass_lut[cls];
        }
        for (int y = by << 2; y < (by << 2) + 4; y++) {
            if (y + 4 < h) {
                backup_row(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), left[y + 4], w, edges);
                ptrs[8] = bak_rows[bak_idx];
            } else if (y + 2 < h && edges & LR_HAVE_BOTTOM) {
                int offset_y = y + 4 - h;
                assert(offset_y < 2);
                backup_row_lpf(bak_rows[bak_idx], lpf_bottom + offset_y * PXSTRIDE(stride), w, edges);
                ptrs[8] = bak_rows[bak_idx];
            } else {
                ptrs[8] = ptrs[7];
            }
            if (++bak_idx == 9) bak_idx = 0;

            for (int bx = 0; bx < bw; bx++) {
                if (filters_user) {
                    const int8_t *filter = filters_user[classes[bx]];
                    for (int x = bx << 2; x < (bx << 2) + 4; x++) {
                        const int m = ptrs[4][x];
                        int s = m << 7;
                        for (int i = 0; i < 32; i++) {
                            const int dy = wiener_ns_config_y[i][0];
                            const int dx = wiener_ns_config_y[i][1];
                            const int diff = ptrs[4 + dy][x + dx] - m;
                            s += diff * filter[i >> 1];
                        }
                        const int v = (s + 64) >> 7;
                        p[x] = iclip_pixel(v);
                    }
                } else {
                    const int16_t *filter = filters_pretrained[classes[bx]];
                    for (int x = bx << 2; x < (bx << 2) + 4; x++) {
                        int s = 0;
                        for (int i = 0; i < 25; i++) {
                            const int dy = pc_wiener_config[i][0];
                            const int dx = pc_wiener_config[i][1];
                            s += ptrs[4 + dy][x + dx] * filter[i >> 1];
                        }
                        const int v = (s + 64) >> 7;
                        p[x] = iclip_pixel(v);
                    }
                }
            }

            for (int r = 0; r < 8; r++) ptrs[r] = ptrs[r+1];
            p += PXSTRIDE(stride);
        }
    }
}

static void ns_wiener_multi_c(pixel *p, const ptrdiff_t stride,
                              const pixel (*left)[4],
                              const pixel *lpf, const int w, int h,
                              const WienerParams *params,
                              const enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
    wiener_multi(p, stride, left, lpf, w, h, params->multi.filters.user, NULL,
                 params->multi.subclass_lut, params->multi.noskip_mask, params->multi.base_q,
                 edges HIGHBD_TAIL_SUFFIX);
}

static void pc_wiener_c(pixel *p, const ptrdiff_t stride,
                        const pixel (*left)[4],
                        const pixel *lpf, const int w, int h,
                        const WienerParams *params,
                        const enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
    wiener_multi(p, stride, left, lpf, w, h, NULL, params->multi.filters.pretrained,
                 params->multi.subclass_lut, params->multi.noskip_mask, params->multi.base_q,
                 edges HIGHBD_TAIL_SUFFIX);
}

static void wiener_filter_h(uint16_t *dst, const pixel (*left)[4],
                            const pixel *src, const int16_t fh[8],
                            const int w, const enum LrEdgeFlags edges
                            HIGHBD_DECL_SUFFIX)
{
    const int bitdepth = bitdepth_from_max(bitdepth_max);
    const int round_bits_h = 3 + (bitdepth == 12) * 2;
    const int rounding_off_h = 1 << (round_bits_h - 1);
    const int clip_limit = 1 << (bitdepth + 1 + 7 - round_bits_h);

    if (w < 6) {
        // For small widths, do the fully conditional loop with
        // conditions on each access.
        for (int x = 0; x < w; x++) {
            int sum = (1 << (bitdepth + 6));
#if BITDEPTH == 8
            sum += src[x] * 128;
#endif
            for (int i = 0; i < 7; i++) {
                int idx = x + i - 3;
                if (idx < 0) {
                    if (!(edges & LR_HAVE_LEFT))
                        sum += src[0] * fh[i];
                    else if (left)
                        sum += left[0][4 + idx] * fh[i];
                    else
                        sum += src[idx] * fh[i];
                } else if (idx >= w && !(edges & LR_HAVE_RIGHT)) {
                    sum += src[w - 1] * fh[i];
                } else
                    sum += src[idx] * fh[i];
            }
            sum = iclip((sum + rounding_off_h) >> round_bits_h, 0, clip_limit - 1);
            dst[x] = sum;
        }

        return;
    }

    // For larger widths, do separate loops with less conditions; first
    // handle the start of the row.
    int start = 3;
    if (!(edges & LR_HAVE_LEFT)) {
        // If there's no left edge, pad using the leftmost pixel.
        for (int x = 0; x < 3; x++) {
            int sum = (1 << (bitdepth + 6));
#if BITDEPTH == 8
            sum += src[x] * 128;
#endif
            for (int i = 0; i < 7; i++) {
                int idx = x + i - 3;
                if (idx < 0)
                    sum += src[0] * fh[i];
                else
                    sum += src[idx] * fh[i];
            }
            sum = iclip((sum + rounding_off_h) >> round_bits_h, 0, clip_limit - 1);
            dst[x] = sum;
        }
    } else if (left) {
        // If we have the left edge and a separate left buffer, pad using that.
        for (int x = 0; x < 3; x++) {
            int sum = (1 << (bitdepth + 6));
#if BITDEPTH == 8
            sum += src[x] * 128;
#endif
            for (int i = 0; i < 7; i++) {
                int idx = x + i - 3;
                if (idx < 0)
                    sum += left[0][4 + idx] * fh[i];
                else
                    sum += src[idx] * fh[i];
            }
            sum = iclip((sum + rounding_off_h) >> round_bits_h, 0, clip_limit - 1);
            dst[x] = sum;
        }
    } else {
        // If we have the left edge, but no separate left buffer, we're in the
        // top/bottom area (lpf) with the left edge existing in the same
        // buffer; just do the regular loop from the start.
        start = 0;
    }
    int end = w - 3;
    if (edges & LR_HAVE_RIGHT)
        end = w;

    // Do a condititon free loop for the bulk of the row.
    for (int x = start; x < end; x++) {
        int sum = (1 << (bitdepth + 6));
#if BITDEPTH == 8
        sum += src[x] * 128;
#endif
        for (int i = 0; i < 7; i++) {
            int idx = x + i - 3;
            sum += src[idx] * fh[i];
        }
        sum = iclip((sum + rounding_off_h) >> round_bits_h, 0, clip_limit - 1);
        dst[x] = sum;
    }

    // If we need to, calculate the end of the row with a condition for
    // right edge padding.
    for (int x = end; x < w; x++) {
        int sum = (1 << (bitdepth + 6));
#if BITDEPTH == 8
        sum += src[x] * 128;
#endif
        for (int i = 0; i < 7; i++) {
            int idx = x + i - 3;
            if (idx >= w)
                sum += src[w - 1] * fh[i];
            else
                sum += src[idx] * fh[i];
        }
        sum = iclip((sum + rounding_off_h) >> round_bits_h, 0, clip_limit - 1);
        dst[x] = sum;
    }
}

static void wiener_filter_v(pixel *p, uint16_t **ptrs, const int16_t fv[8],
                            const int w HIGHBD_DECL_SUFFIX)
{
    const int bitdepth = bitdepth_from_max(bitdepth_max);

    const int round_bits_v = 11 - (bitdepth == 12) * 2;
    const int rounding_off_v = 1 << (round_bits_v - 1);
    const int round_offset = 1 << (bitdepth + (round_bits_v - 1));

    for (int i = 0; i < w; i++) {
        int sum = -round_offset;

        // Only filter using 6 input rows. The 7th row is assumed to be
        // identical to the last one.
        //
        // This function is assumed to only be called at the end, when doing
        // padding at the bottom.
        for (int k = 0; k < 6; k++)
            sum += ptrs[k][i] * fv[k];
        sum += ptrs[5][i] * fv[6];

        p[i] = iclip_pixel((sum + rounding_off_v) >> round_bits_v);
    }

    // Shift the pointers, but only update the first 5; the 6th pointer is kept
    // as it was before (and the 7th is implicitly identical to the 6th).
    for (int i = 0; i < 5; i++)
        ptrs[i] = ptrs[i + 1];
}

static void wiener_filter_hv(pixel *p, uint16_t **ptrs, const pixel (*left)[4],
                             const pixel *src, const int16_t filter[2][8],
                             const int w, const enum LrEdgeFlags edges
                             HIGHBD_DECL_SUFFIX)
{
    const int bitdepth = bitdepth_from_max(bitdepth_max);

    const int round_bits_v = 11 - (bitdepth == 12) * 2;
    const int rounding_off_v = 1 << (round_bits_v - 1);
    const int round_offset = 1 << (bitdepth + (round_bits_v - 1));

    const int16_t *fh = filter[0];
    const int16_t *fv = filter[1];

    // Do combined horziontal and vertical filtering; doing horizontal
    // filtering of one row, combined with vertical filtering of 6
    // preexisting rows and the newly filtered row.

    // For simplicity in the C implementation, just do a separate call
    // of the horizontal filter, into a temporary buffer.
    uint16_t tmp[REST_UNIT_STRIDE];
    wiener_filter_h(tmp, left, src, fh, w, edges HIGHBD_TAIL_SUFFIX);

    for (int i = 0; i < w; i++) {
        int sum = -round_offset;

        // Filter using the 6 stored preexisting rows, and the newly
        // filtered one in tmp[].
        for (int k = 0; k < 6; k++)
            sum += ptrs[k][i] * fv[k];
        sum += tmp[i] * fv[6];
        // At this point, after having read all inputs at point [i], we
        // could overwrite [i] with the newly filtered data.

        p[i] = iclip_pixel((sum + rounding_off_v) >> round_bits_v);
    }

    // For simplicity in the C implementation, just memcpy the newly
    // filtered row into ptrs[6]. Normally, in steady state filtering,
    // this output row, ptrs[6], is equal to ptrs[0]. However at startup,
    // at the top of the filtered area, we may have ptrs[0] equal to ptrs[1],
    // so we can't assume we can write into ptrs[0] but we need to keep
    // a separate pointer for the next row to write into.
    memcpy(ptrs[6], tmp, sizeof(uint16_t) * REST_UNIT_STRIDE);

    // Rotate the window of pointers. Shift the 6 pointers downwards one step.
    for (int i = 0; i < 6; i++)
        ptrs[i] = ptrs[i + 1];
    // The topmost pointer, ptrs[6], which isn't used as input, is set to
    // ptrs[0], which will be used as output for the next _hv call.
    // At the start of the filtering, the caller may set ptrs[6] to the
    // right next buffer to fill in, instead.
    ptrs[6] = ptrs[0];
}

// FIXME Could split into luma and chroma specific functions,
// (since first and last tops are always 0 for chroma)
static void wiener_c(pixel *p, const ptrdiff_t stride,
                     const pixel (*left)[4],
                     const pixel *lpf, const int w, int h,
                     const LooprestorationParams *const params,
                     const enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
    // Values stored between horizontal and vertical filtering don't
    // fit in a uint8_t.
    uint16_t hor[6 * REST_UNIT_STRIDE];
    uint16_t *ptrs[7], *rows[6];
    for (int i = 0; i < 6; i++)
        rows[i] = &hor[i * REST_UNIT_STRIDE];
    const int16_t (*const filter)[8] = params->filter;
    const int16_t *fh = params->filter[0];
    const int16_t *fv = params->filter[1];
    const pixel *lpf_bottom = lpf + 6*PXSTRIDE(stride);

    const pixel *src = p;
    if (edges & LR_HAVE_TOP) {
        ptrs[0] = rows[0];
        ptrs[1] = rows[0];
        ptrs[2] = rows[1];
        ptrs[3] = rows[2];
        ptrs[4] = rows[2];
        ptrs[5] = rows[2];

        wiener_filter_h(rows[0], NULL, lpf, fh, w, edges HIGHBD_TAIL_SUFFIX);
        lpf += PXSTRIDE(stride);
        wiener_filter_h(rows[1], NULL, lpf, fh, w, edges HIGHBD_TAIL_SUFFIX);

        wiener_filter_h(rows[2], left, src, fh, w, edges HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto v1;

        ptrs[4] = ptrs[5] = rows[3];
        wiener_filter_h(rows[3], left, src, fh, w, edges HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto v2;

        ptrs[5] = rows[4];
        wiener_filter_h(rows[4], left, src, fh, w, edges HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto v3;
    } else {
        ptrs[0] = rows[0];
        ptrs[1] = rows[0];
        ptrs[2] = rows[0];
        ptrs[3] = rows[0];
        ptrs[4] = rows[0];
        ptrs[5] = rows[0];

        wiener_filter_h(rows[0], left, src, fh, w, edges HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto v1;

        ptrs[4] = ptrs[5] = rows[1];
        wiener_filter_h(rows[1], left, src, fh, w, edges HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto v2;

        ptrs[5] = rows[2];
        wiener_filter_h(rows[2], left, src, fh, w, edges HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto v3;

        ptrs[6] = rows[3];
        wiener_filter_hv(p, ptrs, left, src, filter, w, edges
                         HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);
        p += PXSTRIDE(stride);

        if (--h <= 0)
            goto v3;

        ptrs[6] = rows[4];
        wiener_filter_hv(p, ptrs, left, src, filter, w, edges
                         HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);
        p += PXSTRIDE(stride);

        if (--h <= 0)
            goto v3;
    }

    ptrs[6] = ptrs[5] + REST_UNIT_STRIDE;
    do {
        wiener_filter_hv(p, ptrs, left, src, filter, w, edges
                         HIGHBD_TAIL_SUFFIX);
        left++;
        src += PXSTRIDE(stride);
        p += PXSTRIDE(stride);
    } while (--h > 0);

    if (!(edges & LR_HAVE_BOTTOM))
        goto v3;

    wiener_filter_hv(p, ptrs, NULL, lpf_bottom, filter, w, edges
                     HIGHBD_TAIL_SUFFIX);
    lpf_bottom += PXSTRIDE(stride);
    p += PXSTRIDE(stride);

    wiener_filter_hv(p, ptrs, NULL, lpf_bottom, filter, w, edges
                     HIGHBD_TAIL_SUFFIX);
    p += PXSTRIDE(stride);
v1:
    wiener_filter_v(p, ptrs, fv, w HIGHBD_TAIL_SUFFIX);

    return;

v3:
    wiener_filter_v(p, ptrs, fv, w HIGHBD_TAIL_SUFFIX);
    p += PXSTRIDE(stride);
v2:
    wiener_filter_v(p, ptrs, fv, w HIGHBD_TAIL_SUFFIX);
    p += PXSTRIDE(stride);
    goto v1;
}

// SGR
static NOINLINE void rotate(int32_t **sumsq_ptrs, coef **sum_ptrs, int n)
{
    int32_t *tmp32 = sumsq_ptrs[0];
    coef *tmpc = sum_ptrs[0];
    for (int i = 0; i < n - 1; i++) {
        sumsq_ptrs[i] = sumsq_ptrs[i + 1];
        sum_ptrs[i] = sum_ptrs[i + 1];
    }
    sumsq_ptrs[n - 1] = tmp32;
    sum_ptrs[n - 1] = tmpc;
}

static NOINLINE void rotate5_x2(int32_t **sumsq_ptrs, coef **sum_ptrs)
{
    int32_t *tmp32[2];
    coef *tmpc[2];
    for (int i = 0; i < 2; i++) {
        tmp32[i] = sumsq_ptrs[i];
        tmpc[i] = sum_ptrs[i];
    }
    for (int i = 0; i < 3; i++) {
        sumsq_ptrs[i] = sumsq_ptrs[i + 2];
        sum_ptrs[i] = sum_ptrs[i + 2];
    }
    for (int i = 0; i < 2; i++) {
        sumsq_ptrs[3 + i] = tmp32[i];
        sum_ptrs[3 + i] = tmpc[i];
    }
}

static NOINLINE void sgr_box3_row_h(int32_t *sumsq, coef *sum,
                                    const pixel (*left)[4],
                                    const pixel *src, const int w,
                                    const enum LrEdgeFlags edges)
{
    sumsq++;
    sum++;
    int a = edges & LR_HAVE_LEFT ? (left ? left[0][2] : src[-2]) : src[0];
    int b = edges & LR_HAVE_LEFT ? (left ? left[0][3] : src[-1]) : src[0];
    for (int x = -1; x < w + 1; x++) {
        int c = (x + 1 < w || (edges & LR_HAVE_RIGHT)) ? src[x + 1] : src[w - 1];
        sum[x] = a + b + c;
        sumsq[x] = a * a + b * b + c * c;
        a = b;
        b = c;
    }
}

static NOINLINE void sgr_box5_row_h(int32_t *sumsq, coef *sum,
                                    const pixel (*left)[4],
                                    const pixel *src, const int w,
                                    const enum LrEdgeFlags edges)
{
    sumsq++;
    sum++;
    int a = edges & LR_HAVE_LEFT ? (left ? left[0][1] : src[-3]) : src[0];
    int b = edges & LR_HAVE_LEFT ? (left ? left[0][2] : src[-2]) : src[0];
    int c = edges & LR_HAVE_LEFT ? (left ? left[0][3] : src[-1]) : src[0];
    int d = src[0];
    for (int x = -1; x < w + 1; x++) {
        int e = (x + 2 < w || (edges & LR_HAVE_RIGHT)) ? src[x + 2] : src[w - 1];
        sum[x] = a + b + c + d + e;
        sumsq[x] = a * a + b * b + c * c + d * d + e * e;
        a = b;
        b = c;
        c = d;
        d = e;
    }
}

static void sgr_box35_row_h(int32_t *sumsq3, coef *sum3,
                            int32_t *sumsq5, coef *sum5,
                            const pixel (*left)[4],
                            const pixel *src, const int w,
                            const enum LrEdgeFlags edges)
{
    sgr_box3_row_h(sumsq3, sum3, left, src, w, edges);
    sgr_box5_row_h(sumsq5, sum5, left, src, w, edges);
}

static NOINLINE void sgr_box3_row_v(int32_t **sumsq, coef **sum,
                                    int32_t *sumsq_out, coef *sum_out,
                                    const int w)
{
    for (int x = 0; x < w + 2; x++) {
        int sq_a = sumsq[0][x];
        int sq_b = sumsq[1][x];
        int sq_c = sumsq[2][x];
        int s_a = sum[0][x];
        int s_b = sum[1][x];
        int s_c = sum[2][x];
        sumsq_out[x] = sq_a + sq_b + sq_c;
        sum_out[x] = s_a + s_b + s_c;
    }
}

static NOINLINE void sgr_box5_row_v(int32_t **sumsq, coef **sum,
                                    int32_t *sumsq_out, coef *sum_out,
                                    const int w)
{
    for (int x = 0; x < w + 2; x++) {
        int sq_a = sumsq[0][x];
        int sq_b = sumsq[1][x];
        int sq_c = sumsq[2][x];
        int sq_d = sumsq[3][x];
        int sq_e = sumsq[4][x];
        int s_a = sum[0][x];
        int s_b = sum[1][x];
        int s_c = sum[2][x];
        int s_d = sum[3][x];
        int s_e = sum[4][x];
        sumsq_out[x] = sq_a + sq_b + sq_c + sq_d + sq_e;
        sum_out[x] = s_a + s_b + s_c + s_d + s_e;
    }
}

static NOINLINE void sgr_calc_row_ab(int32_t *AA, coef *BB, int w, int s,
                                     int bitdepth_max, int n, int sgr_one_by_x)
{
    const int bitdepth_min_8 = bitdepth_from_max(bitdepth_max) - 8;
    for (int i = 0; i < w + 2; i++) {
        const int a =
            (AA[i] + ((1 << (2 * bitdepth_min_8)) >> 1)) >> (2 * bitdepth_min_8);
        const int b =
            (BB[i] + ((1 << bitdepth_min_8) >> 1)) >> bitdepth_min_8;

        const unsigned p = imax(a * n - b * b, 0);
        const unsigned z = (p * s + (1 << 19)) >> 20;
        const unsigned x = dav2d_sgr_x_by_x[umin(z, 255)];

        // This is where we invert A and B, so that B is of size coef.
        AA[i] = (x * BB[i] * sgr_one_by_x + (1 << 11)) >> 12;
        BB[i] = x;
    }
}

static void sgr_box3_vert(int32_t **sumsq, coef **sum,
                          int32_t *sumsq_out, coef *sum_out,
                          const int w, const int s, const int bitdepth_max)
{
    sgr_box3_row_v(sumsq, sum, sumsq_out, sum_out, w);
    sgr_calc_row_ab(sumsq_out, sum_out, w, s, bitdepth_max, 9, 455);
    rotate(sumsq, sum, 3);
}

static void sgr_box5_vert(int32_t **sumsq, coef **sum,
                          int32_t *sumsq_out, coef *sum_out,
                          const int w, const int s, const int bitdepth_max)
{
    sgr_box5_row_v(sumsq, sum, sumsq_out, sum_out, w);
    sgr_calc_row_ab(sumsq_out, sum_out, w, s, bitdepth_max, 25, 164);
    rotate5_x2(sumsq, sum);
}

static void sgr_box3_hv(int32_t **sumsq, coef **sum,
                        int32_t *AA, coef *BB,
                        const pixel (*left)[4],
                        const pixel *src, const int w,
                        const int s,
                        const enum LrEdgeFlags edges,
                        const int bitdepth_max)
{
    sgr_box3_row_h(sumsq[2], sum[2], left, src, w, edges);
    sgr_box3_vert(sumsq, sum, AA, BB, w, s, bitdepth_max);
}

static NOINLINE void sgr_finish_filter_row1(coef *tmp,
                                            const pixel *src,
                                            int32_t **A_ptrs, coef **B_ptrs,
                                            const int w)
{
#define EIGHT_NEIGHBORS(P, i)\
    ((P[1][i] + P[1][i - 1] + P[1][i + 1] + P[0][i] + P[2][i]) * 4 + \
     (P[0][i - 1] + P[2][i - 1] +                           \
      P[0][i + 1] + P[2][i + 1]) * 3)
    for (int i = 0; i < w; i++) {
        const int a = EIGHT_NEIGHBORS(B_ptrs, i + 1);
        const int b = EIGHT_NEIGHBORS(A_ptrs, i + 1);
        tmp[i] = (b - a * src[i] + (1 << 8)) >> 9;
    }
#undef EIGHT_NEIGHBORS
}

#define FILTER_OUT_STRIDE (384)

static NOINLINE void sgr_finish_filter2(coef *tmp,
                                        const pixel *src,
                                        const ptrdiff_t src_stride,
                                        int32_t **A_ptrs, coef **B_ptrs,
                                        const int w, const int h)
{
#define SIX_NEIGHBORS(P, i)\
    ((P[0][i]     + P[1][i]) * 6 +   \
     (P[0][i - 1] + P[1][i - 1] +    \
      P[0][i + 1] + P[1][i + 1]) * 5)
    for (int i = 0; i < w; i++) {
        const int a = SIX_NEIGHBORS(B_ptrs, i + 1);
        const int b = SIX_NEIGHBORS(A_ptrs, i + 1);
        tmp[i] = (b - a * src[i] + (1 << 8)) >> 9;
    }
    if (h <= 1)
        return;
    tmp += FILTER_OUT_STRIDE;
    src += PXSTRIDE(src_stride);
    const int32_t *A = &A_ptrs[1][1];
    const coef *B = &B_ptrs[1][1];
    for (int i = 0; i < w; i++) {
        const int a = B[i] * 6 + (B[i - 1] + B[i + 1]) * 5;
        const int b = A[i] * 6 + (A[i - 1] + A[i + 1]) * 5;
        tmp[i] = (b - a * src[i] + (1 << 7)) >> 8;
    }
#undef SIX_NEIGHBORS
}

static NOINLINE void sgr_weighted_row1(pixel *dst, const coef *t1,
                                       const int w, const int w1 HIGHBD_DECL_SUFFIX)
{
    for (int i = 0; i < w; i++) {
        const int v = w1 * t1[i];
        dst[i] = iclip_pixel(dst[i] + ((v + (1 << 10)) >> 11));
    }
}

static NOINLINE void sgr_weighted2(pixel *dst, const ptrdiff_t dst_stride,
                                   const coef *t1, const coef *t2,
                                   const int w, const int h,
                                   const int w0, const int w1 HIGHBD_DECL_SUFFIX)
{
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            const int v = w0 * t1[i] + w1 * t2[i];
            dst[i] = iclip_pixel(dst[i] + ((v + (1 << 10)) >> 11));
        }
        dst += PXSTRIDE(dst_stride);
        t1 += FILTER_OUT_STRIDE;
        t2 += FILTER_OUT_STRIDE;
    }
}

static NOINLINE void sgr_finish1(pixel **dst, const ptrdiff_t stride,
                                 int32_t **A_ptrs, coef **B_ptrs, const int w,
                                 const int w1 HIGHBD_DECL_SUFFIX)
{
    // Only one single row, no stride needed
    ALIGN_STK_16(coef, tmp, 384,);

    sgr_finish_filter_row1(tmp, *dst, A_ptrs, B_ptrs, w);
    sgr_weighted_row1(*dst, tmp, w, w1 HIGHBD_TAIL_SUFFIX);
    *dst += PXSTRIDE(stride);
    rotate(A_ptrs, B_ptrs, 3);
}

static NOINLINE void sgr_finish2(pixel **dst, const ptrdiff_t stride,
                                 int32_t **A_ptrs, coef **B_ptrs,
                                 const int w, const int h, const int w1
                                 HIGHBD_DECL_SUFFIX)
{
    ALIGN_STK_16(coef, tmp, 2*FILTER_OUT_STRIDE,);

    sgr_finish_filter2(tmp, *dst, stride, A_ptrs, B_ptrs, w, h);
    sgr_weighted_row1(*dst, tmp, w, w1 HIGHBD_TAIL_SUFFIX);
    *dst += PXSTRIDE(stride);
    if (h > 1) {
        sgr_weighted_row1(*dst, tmp + FILTER_OUT_STRIDE, w, w1 HIGHBD_TAIL_SUFFIX);
        *dst += PXSTRIDE(stride);
    }
    rotate(A_ptrs, B_ptrs, 2);
}

static NOINLINE void sgr_finish_mix(pixel **dst, const ptrdiff_t stride,
                                    int32_t **A5_ptrs, coef **B5_ptrs,
                                    int32_t **A3_ptrs, coef **B3_ptrs,
                                    const int w, const int h,
                                    const int w0, const int w1 HIGHBD_DECL_SUFFIX)
{
    ALIGN_STK_16(coef, tmp5, 2*FILTER_OUT_STRIDE,);
    ALIGN_STK_16(coef, tmp3, 2*FILTER_OUT_STRIDE,);

    sgr_finish_filter2(tmp5, *dst, stride, A5_ptrs, B5_ptrs, w, h);
    sgr_finish_filter_row1(tmp3, *dst, A3_ptrs, B3_ptrs, w);
    if (h > 1)
        sgr_finish_filter_row1(tmp3 + FILTER_OUT_STRIDE, *dst + PXSTRIDE(stride),
                               &A3_ptrs[1], &B3_ptrs[1], w);
    sgr_weighted2(*dst, stride, tmp5, tmp3, w, h, w0, w1 HIGHBD_TAIL_SUFFIX);
    *dst += h*PXSTRIDE(stride);
    rotate(A5_ptrs, B5_ptrs, 2);
    rotate(A3_ptrs, B3_ptrs, 4);
}


static void sgr_3x3_c(pixel *dst, const ptrdiff_t stride,
                      const pixel (*left)[4], const pixel *lpf,
                      const int w, int h,
                      const LooprestorationParams *const params,
                      const enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
#define BUF_STRIDE (384 + 16)
    ALIGN_STK_16(int32_t, sumsq_buf, BUF_STRIDE * 3 + 16,);
    ALIGN_STK_16(coef, sum_buf, BUF_STRIDE * 3 + 16,);
    int32_t *sumsq_ptrs[3], *sumsq_rows[3];
    coef *sum_ptrs[3], *sum_rows[3];
    for (int i = 0; i < 3; i++) {
        sumsq_rows[i] = &sumsq_buf[i * BUF_STRIDE];
        sum_rows[i] = &sum_buf[i * BUF_STRIDE];
    }

    ALIGN_STK_16(int32_t, A_buf, BUF_STRIDE * 3 + 16,);
    ALIGN_STK_16(coef, B_buf, BUF_STRIDE * 3 + 16,);
    int32_t *A_ptrs[3];
    coef *B_ptrs[3];
    for (int i = 0; i < 3; i++) {
        A_ptrs[i] = &A_buf[i * BUF_STRIDE];
        B_ptrs[i] = &B_buf[i * BUF_STRIDE];
    }
    const pixel *src = dst;
    const pixel *lpf_bottom = lpf + 6*PXSTRIDE(stride);

    if (edges & LR_HAVE_TOP) {
        sumsq_ptrs[0] = sumsq_rows[0];
        sumsq_ptrs[1] = sumsq_rows[1];
        sumsq_ptrs[2] = sumsq_rows[2];
        sum_ptrs[0] = sum_rows[0];
        sum_ptrs[1] = sum_rows[1];
        sum_ptrs[2] = sum_rows[2];

        sgr_box3_row_h(sumsq_rows[0], sum_rows[0], NULL, lpf, w, edges);
        lpf += PXSTRIDE(stride);
        sgr_box3_row_h(sumsq_rows[1], sum_rows[1], NULL, lpf, w, edges);

        sgr_box3_hv(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                    left, src, w, params->sgr.s1, edges, BITDEPTH_MAX);
        left++;
        src += PXSTRIDE(stride);
        rotate(A_ptrs, B_ptrs, 3);

        if (--h <= 0)
            goto vert_1;

        sgr_box3_hv(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                    left, src, w, params->sgr.s1, edges, BITDEPTH_MAX);
        left++;
        src += PXSTRIDE(stride);
        rotate(A_ptrs, B_ptrs, 3);

        if (--h <= 0)
            goto vert_2;
    } else {
        sumsq_ptrs[0] = sumsq_rows[0];
        sumsq_ptrs[1] = sumsq_rows[0];
        sumsq_ptrs[2] = sumsq_rows[0];
        sum_ptrs[0] = sum_rows[0];
        sum_ptrs[1] = sum_rows[0];
        sum_ptrs[2] = sum_rows[0];

        sgr_box3_row_h(sumsq_rows[0], sum_rows[0], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box3_vert(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                      w, params->sgr.s1, BITDEPTH_MAX);
        rotate(A_ptrs, B_ptrs, 3);

        if (--h <= 0)
            goto vert_1;

        sumsq_ptrs[2] = sumsq_rows[1];
        sum_ptrs[2] = sum_rows[1];

        sgr_box3_hv(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                    left, src, w, params->sgr.s1, edges, BITDEPTH_MAX);
        left++;
        src += PXSTRIDE(stride);
        rotate(A_ptrs, B_ptrs, 3);

        if (--h <= 0)
            goto vert_2;

        sumsq_ptrs[2] = sumsq_rows[2];
        sum_ptrs[2] = sum_rows[2];
    }

    do {
        sgr_box3_hv(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                    left, src, w, params->sgr.s1, edges, BITDEPTH_MAX);
        left++;
        src += PXSTRIDE(stride);

        sgr_finish1(&dst, stride, A_ptrs, B_ptrs,
                    w, params->sgr.w1 HIGHBD_TAIL_SUFFIX);
    } while (--h > 0);

    if (!(edges & LR_HAVE_BOTTOM))
        goto vert_2;

    sgr_box3_hv(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                NULL, lpf_bottom, w, params->sgr.s1, edges, BITDEPTH_MAX);
    lpf_bottom += PXSTRIDE(stride);

    sgr_finish1(&dst, stride, A_ptrs, B_ptrs,
                w, params->sgr.w1 HIGHBD_TAIL_SUFFIX);

    sgr_box3_hv(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                NULL, lpf_bottom, w, params->sgr.s1, edges, BITDEPTH_MAX);

    sgr_finish1(&dst, stride, A_ptrs, B_ptrs,
                w, params->sgr.w1 HIGHBD_TAIL_SUFFIX);
    return;

vert_2:
    sumsq_ptrs[2] = sumsq_ptrs[1];
    sum_ptrs[2] = sum_ptrs[1];
    sgr_box3_vert(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                  w, params->sgr.s1, BITDEPTH_MAX);

    sgr_finish1(&dst, stride, A_ptrs, B_ptrs,
                w, params->sgr.w1 HIGHBD_TAIL_SUFFIX);

output_1:
    sumsq_ptrs[2] = sumsq_ptrs[1];
    sum_ptrs[2] = sum_ptrs[1];
    sgr_box3_vert(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                  w, params->sgr.s1, BITDEPTH_MAX);

    sgr_finish1(&dst, stride, A_ptrs, B_ptrs,
                w, params->sgr.w1 HIGHBD_TAIL_SUFFIX);
    return;

vert_1:
    sumsq_ptrs[2] = sumsq_ptrs[1];
    sum_ptrs[2] = sum_ptrs[1];
    sgr_box3_vert(sumsq_ptrs, sum_ptrs, A_ptrs[2], B_ptrs[2],
                  w, params->sgr.s1, BITDEPTH_MAX);
    rotate(A_ptrs, B_ptrs, 3);
    goto output_1;
}

static void sgr_5x5_c(pixel *dst, const ptrdiff_t stride,
                      const pixel (*left)[4], const pixel *lpf,
                      const int w, int h,
                      const LooprestorationParams *const params,
                      const enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
    ALIGN_STK_16(int32_t, sumsq_buf, BUF_STRIDE * 5 + 16,);
    ALIGN_STK_16(coef, sum_buf, BUF_STRIDE * 5 + 16,);
    int32_t *sumsq_ptrs[5], *sumsq_rows[5];
    coef *sum_ptrs[5], *sum_rows[5];
    for (int i = 0; i < 5; i++) {
        sumsq_rows[i] = &sumsq_buf[i * BUF_STRIDE];
        sum_rows[i] = &sum_buf[i * BUF_STRIDE];
    }

    ALIGN_STK_16(int32_t, A_buf, BUF_STRIDE * 2 + 16,);
    ALIGN_STK_16(coef, B_buf, BUF_STRIDE * 2 + 16,);
    int32_t *A_ptrs[2];
    coef *B_ptrs[2];
    for (int i = 0; i < 2; i++) {
        A_ptrs[i] = &A_buf[i * BUF_STRIDE];
        B_ptrs[i] = &B_buf[i * BUF_STRIDE];
    }
    const pixel *src = dst;
    const pixel *lpf_bottom = lpf + 6*PXSTRIDE(stride);

    if (edges & LR_HAVE_TOP) {
        sumsq_ptrs[0] = sumsq_rows[0];
        sumsq_ptrs[1] = sumsq_rows[0];
        sumsq_ptrs[2] = sumsq_rows[1];
        sumsq_ptrs[3] = sumsq_rows[2];
        sumsq_ptrs[4] = sumsq_rows[3];
        sum_ptrs[0] = sum_rows[0];
        sum_ptrs[1] = sum_rows[0];
        sum_ptrs[2] = sum_rows[1];
        sum_ptrs[3] = sum_rows[2];
        sum_ptrs[4] = sum_rows[3];

        sgr_box5_row_h(sumsq_rows[0], sum_rows[0], NULL, lpf, w, edges);
        lpf += PXSTRIDE(stride);
        sgr_box5_row_h(sumsq_rows[1], sum_rows[1], NULL, lpf, w, edges);

        sgr_box5_row_h(sumsq_rows[2], sum_rows[2], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto vert_1;

        sgr_box5_row_h(sumsq_rows[3], sum_rows[3], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);
        sgr_box5_vert(sumsq_ptrs, sum_ptrs, A_ptrs[1], B_ptrs[1],
                      w, params->sgr.s0, BITDEPTH_MAX);
        rotate(A_ptrs, B_ptrs, 2);

        if (--h <= 0)
            goto vert_2;

        // ptrs are rotated by 2; both [3] and [4] now point at rows[0]; set
        // one of them to point at the previously unused rows[4].
        sumsq_ptrs[3] = sumsq_rows[4];
        sum_ptrs[3] = sum_rows[4];
    } else {
        sumsq_ptrs[0] = sumsq_rows[0];
        sumsq_ptrs[1] = sumsq_rows[0];
        sumsq_ptrs[2] = sumsq_rows[0];
        sumsq_ptrs[3] = sumsq_rows[0];
        sumsq_ptrs[4] = sumsq_rows[0];
        sum_ptrs[0] = sum_rows[0];
        sum_ptrs[1] = sum_rows[0];
        sum_ptrs[2] = sum_rows[0];
        sum_ptrs[3] = sum_rows[0];
        sum_ptrs[4] = sum_rows[0];

        sgr_box5_row_h(sumsq_rows[0], sum_rows[0], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto vert_1;

        sumsq_ptrs[4] = sumsq_rows[1];
        sum_ptrs[4] = sum_rows[1];

        sgr_box5_row_h(sumsq_rows[1], sum_rows[1], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box5_vert(sumsq_ptrs, sum_ptrs, A_ptrs[1], B_ptrs[1],
                      w, params->sgr.s0, BITDEPTH_MAX);
        rotate(A_ptrs, B_ptrs, 2);

        if (--h <= 0)
            goto vert_2;

        sumsq_ptrs[3] = sumsq_rows[2];
        sumsq_ptrs[4] = sumsq_rows[3];
        sum_ptrs[3] = sum_rows[2];
        sum_ptrs[4] = sum_rows[3];

        sgr_box5_row_h(sumsq_rows[2], sum_rows[2], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto odd;

        sgr_box5_row_h(sumsq_rows[3], sum_rows[3], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box5_vert(sumsq_ptrs, sum_ptrs, A_ptrs[1], B_ptrs[1],
                      w, params->sgr.s0, BITDEPTH_MAX);
        sgr_finish2(&dst, stride, A_ptrs, B_ptrs,
                    w, 2, params->sgr.w0 HIGHBD_TAIL_SUFFIX);

        if (--h <= 0)
            goto vert_2;

        // ptrs are rotated by 2; both [3] and [4] now point at rows[0]; set
        // one of them to point at the previously unused rows[4].
        sumsq_ptrs[3] = sumsq_rows[4];
        sum_ptrs[3] = sum_rows[4];
    }

    do {
        sgr_box5_row_h(sumsq_ptrs[3], sum_ptrs[3], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        if (--h <= 0)
            goto odd;

        sgr_box5_row_h(sumsq_ptrs[4], sum_ptrs[4], left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box5_vert(sumsq_ptrs, sum_ptrs, A_ptrs[1], B_ptrs[1],
                      w, params->sgr.s0, BITDEPTH_MAX);
        sgr_finish2(&dst, stride, A_ptrs, B_ptrs,
                    w, 2, params->sgr.w0 HIGHBD_TAIL_SUFFIX);
    } while (--h > 0);

    if (!(edges & LR_HAVE_BOTTOM))
        goto vert_2;

    sgr_box5_row_h(sumsq_ptrs[3], sum_ptrs[3], NULL, lpf_bottom, w, edges);
    lpf_bottom += PXSTRIDE(stride);
    sgr_box5_row_h(sumsq_ptrs[4], sum_ptrs[4], NULL, lpf_bottom, w, edges);

output_2:
    sgr_box5_vert(sumsq_ptrs, sum_ptrs, A_ptrs[1], B_ptrs[1],
                  w, params->sgr.s0, BITDEPTH_MAX);
    sgr_finish2(&dst, stride, A_ptrs, B_ptrs,
                w, 2, params->sgr.w0 HIGHBD_TAIL_SUFFIX);
    return;

vert_2:
    // Duplicate the last row twice more
    sumsq_ptrs[3] = sumsq_ptrs[2];
    sumsq_ptrs[4] = sumsq_ptrs[2];
    sum_ptrs[3] = sum_ptrs[2];
    sum_ptrs[4] = sum_ptrs[2];
    goto output_2;

odd:
    // Copy the last row as padding once
    sumsq_ptrs[4] = sumsq_ptrs[3];
    sum_ptrs[4] = sum_ptrs[3];

    sgr_box5_vert(sumsq_ptrs, sum_ptrs, A_ptrs[1], B_ptrs[1],
                  w, params->sgr.s0, BITDEPTH_MAX);
    sgr_finish2(&dst, stride, A_ptrs, B_ptrs,
                w, 2, params->sgr.w0 HIGHBD_TAIL_SUFFIX);

output_1:
    // Duplicate the last row twice more
    sumsq_ptrs[3] = sumsq_ptrs[2];
    sumsq_ptrs[4] = sumsq_ptrs[2];
    sum_ptrs[3] = sum_ptrs[2];
    sum_ptrs[4] = sum_ptrs[2];

    sgr_box5_vert(sumsq_ptrs, sum_ptrs, A_ptrs[1], B_ptrs[1],
                  w, params->sgr.s0, BITDEPTH_MAX);
    // Output only one row
    sgr_finish2(&dst, stride, A_ptrs, B_ptrs,
                w, 1, params->sgr.w0 HIGHBD_TAIL_SUFFIX);
    return;

vert_1:
    // Copy the last row as padding once
    sumsq_ptrs[4] = sumsq_ptrs[3];
    sum_ptrs[4] = sum_ptrs[3];

    sgr_box5_vert(sumsq_ptrs, sum_ptrs, A_ptrs[1], B_ptrs[1],
                  w, params->sgr.s0, BITDEPTH_MAX);
    rotate(A_ptrs, B_ptrs, 2);

    goto output_1;
}

static void sgr_mix_c(pixel *dst, const ptrdiff_t stride,
                      const pixel (*left)[4], const pixel *lpf,
                      const int w, int h,
                      const LooprestorationParams *const params,
                      const enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
    ALIGN_STK_16(int32_t, sumsq5_buf, BUF_STRIDE * 5 + 16,);
    ALIGN_STK_16(coef, sum5_buf, BUF_STRIDE * 5 + 16,);
    int32_t *sumsq5_ptrs[5], *sumsq5_rows[5];
    coef *sum5_ptrs[5], *sum5_rows[5];
    for (int i = 0; i < 5; i++) {
        sumsq5_rows[i] = &sumsq5_buf[i * BUF_STRIDE];
        sum5_rows[i] = &sum5_buf[i * BUF_STRIDE];
    }
    ALIGN_STK_16(int32_t, sumsq3_buf, BUF_STRIDE * 3 + 16,);
    ALIGN_STK_16(coef, sum3_buf, BUF_STRIDE * 3 + 16,);
    int32_t *sumsq3_ptrs[3], *sumsq3_rows[3];
    coef *sum3_ptrs[3], *sum3_rows[3];
    for (int i = 0; i < 3; i++) {
        sumsq3_rows[i] = &sumsq3_buf[i * BUF_STRIDE];
        sum3_rows[i] = &sum3_buf[i * BUF_STRIDE];
    }

    ALIGN_STK_16(int32_t, A5_buf, BUF_STRIDE * 2 + 16,);
    ALIGN_STK_16(coef, B5_buf, BUF_STRIDE * 2 + 16,);
    int32_t *A5_ptrs[2];
    coef *B5_ptrs[2];
    for (int i = 0; i < 2; i++) {
        A5_ptrs[i] = &A5_buf[i * BUF_STRIDE];
        B5_ptrs[i] = &B5_buf[i * BUF_STRIDE];
    }
    ALIGN_STK_16(int32_t, A3_buf, BUF_STRIDE * 4 + 16,);
    ALIGN_STK_16(coef, B3_buf, BUF_STRIDE * 4 + 16,);
    int32_t *A3_ptrs[4];
    coef *B3_ptrs[4];
    for (int i = 0; i < 4; i++) {
        A3_ptrs[i] = &A3_buf[i * BUF_STRIDE];
        B3_ptrs[i] = &B3_buf[i * BUF_STRIDE];
    }
    const pixel *src = dst;
    const pixel *lpf_bottom = lpf + 6*PXSTRIDE(stride);

    if (edges & LR_HAVE_TOP) {
        sumsq5_ptrs[0] = sumsq5_rows[0];
        sumsq5_ptrs[1] = sumsq5_rows[0];
        sumsq5_ptrs[2] = sumsq5_rows[1];
        sumsq5_ptrs[3] = sumsq5_rows[2];
        sumsq5_ptrs[4] = sumsq5_rows[3];
        sum5_ptrs[0] = sum5_rows[0];
        sum5_ptrs[1] = sum5_rows[0];
        sum5_ptrs[2] = sum5_rows[1];
        sum5_ptrs[3] = sum5_rows[2];
        sum5_ptrs[4] = sum5_rows[3];

        sumsq3_ptrs[0] = sumsq3_rows[0];
        sumsq3_ptrs[1] = sumsq3_rows[1];
        sumsq3_ptrs[2] = sumsq3_rows[2];
        sum3_ptrs[0] = sum3_rows[0];
        sum3_ptrs[1] = sum3_rows[1];
        sum3_ptrs[2] = sum3_rows[2];

        sgr_box35_row_h(sumsq3_rows[0], sum3_rows[0],
                        sumsq5_rows[0], sum5_rows[0],
                        NULL, lpf, w, edges);
        lpf += PXSTRIDE(stride);
        sgr_box35_row_h(sumsq3_rows[1], sum3_rows[1],
                        sumsq5_rows[1], sum5_rows[1],
                        NULL, lpf, w, edges);

        sgr_box35_row_h(sumsq3_rows[2], sum3_rows[2],
                        sumsq5_rows[2], sum5_rows[2],
                        left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                      w, params->sgr.s1, BITDEPTH_MAX);
        rotate(A3_ptrs, B3_ptrs, 4);

        if (--h <= 0)
            goto vert_1;

        sgr_box35_row_h(sumsq3_ptrs[2], sum3_ptrs[2],
                        sumsq5_rows[3], sum5_rows[3],
                        left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);
        sgr_box5_vert(sumsq5_ptrs, sum5_ptrs, A5_ptrs[1], B5_ptrs[1],
                      w, params->sgr.s0, BITDEPTH_MAX);
        rotate(A5_ptrs, B5_ptrs, 2);
        sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                      w, params->sgr.s1, BITDEPTH_MAX);
        rotate(A3_ptrs, B3_ptrs, 4);

        if (--h <= 0)
            goto vert_2;

        // ptrs are rotated by 2; both [3] and [4] now point at rows[0]; set
        // one of them to point at the previously unused rows[4].
        sumsq5_ptrs[3] = sumsq5_rows[4];
        sum5_ptrs[3] = sum5_rows[4];
    } else {
        sumsq5_ptrs[0] = sumsq5_rows[0];
        sumsq5_ptrs[1] = sumsq5_rows[0];
        sumsq5_ptrs[2] = sumsq5_rows[0];
        sumsq5_ptrs[3] = sumsq5_rows[0];
        sumsq5_ptrs[4] = sumsq5_rows[0];
        sum5_ptrs[0] = sum5_rows[0];
        sum5_ptrs[1] = sum5_rows[0];
        sum5_ptrs[2] = sum5_rows[0];
        sum5_ptrs[3] = sum5_rows[0];
        sum5_ptrs[4] = sum5_rows[0];

        sumsq3_ptrs[0] = sumsq3_rows[0];
        sumsq3_ptrs[1] = sumsq3_rows[0];
        sumsq3_ptrs[2] = sumsq3_rows[0];
        sum3_ptrs[0] = sum3_rows[0];
        sum3_ptrs[1] = sum3_rows[0];
        sum3_ptrs[2] = sum3_rows[0];

        sgr_box35_row_h(sumsq3_rows[0], sum3_rows[0],
                        sumsq5_rows[0], sum5_rows[0],
                        left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                      w, params->sgr.s1, BITDEPTH_MAX);
        rotate(A3_ptrs, B3_ptrs, 4);

        if (--h <= 0)
            goto vert_1;

        sumsq5_ptrs[4] = sumsq5_rows[1];
        sum5_ptrs[4] = sum5_rows[1];

        sumsq3_ptrs[2] = sumsq3_rows[1];
        sum3_ptrs[2] = sum3_rows[1];

        sgr_box35_row_h(sumsq3_rows[1], sum3_rows[1],
                        sumsq5_rows[1], sum5_rows[1],
                        left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box5_vert(sumsq5_ptrs, sum5_ptrs, A5_ptrs[1], B5_ptrs[1],
                      w, params->sgr.s0, BITDEPTH_MAX);
        rotate(A5_ptrs, B5_ptrs, 2);
        sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                      w, params->sgr.s1, BITDEPTH_MAX);
        rotate(A3_ptrs, B3_ptrs, 4);

        if (--h <= 0)
            goto vert_2;

        sumsq5_ptrs[3] = sumsq5_rows[2];
        sumsq5_ptrs[4] = sumsq5_rows[3];
        sum5_ptrs[3] = sum5_rows[2];
        sum5_ptrs[4] = sum5_rows[3];

        sumsq3_ptrs[2] = sumsq3_rows[2];
        sum3_ptrs[2] = sum3_rows[2];

        sgr_box35_row_h(sumsq3_rows[2], sum3_rows[2],
                        sumsq5_rows[2], sum5_rows[2],
                        left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                      w, params->sgr.s1, BITDEPTH_MAX);
        rotate(A3_ptrs, B3_ptrs, 4);

        if (--h <= 0)
            goto odd;

        sgr_box35_row_h(sumsq3_ptrs[2], sum3_ptrs[2],
                        sumsq5_rows[3], sum5_rows[3],
                        left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box5_vert(sumsq5_ptrs, sum5_ptrs, A5_ptrs[1], B5_ptrs[1],
                      w, params->sgr.s0, BITDEPTH_MAX);
        sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                      w, params->sgr.s1, BITDEPTH_MAX);
        sgr_finish_mix(&dst, stride, A5_ptrs, B5_ptrs, A3_ptrs, B3_ptrs,
                       w, 2, params->sgr.w0, params->sgr.w1
                       HIGHBD_TAIL_SUFFIX);

        if (--h <= 0)
            goto vert_2;

        // ptrs are rotated by 2; both [3] and [4] now point at rows[0]; set
        // one of them to point at the previously unused rows[4].
        sumsq5_ptrs[3] = sumsq5_rows[4];
        sum5_ptrs[3] = sum5_rows[4];
    }

    do {
        sgr_box35_row_h(sumsq3_ptrs[2], sum3_ptrs[2],
                        sumsq5_ptrs[3], sum5_ptrs[3],
                        left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                      w, params->sgr.s1, BITDEPTH_MAX);
        rotate(A3_ptrs, B3_ptrs, 4);

        if (--h <= 0)
            goto odd;

        sgr_box35_row_h(sumsq3_ptrs[2], sum3_ptrs[2],
                        sumsq5_ptrs[4], sum5_ptrs[4],
                        left, src, w, edges);
        left++;
        src += PXSTRIDE(stride);

        sgr_box5_vert(sumsq5_ptrs, sum5_ptrs, A5_ptrs[1], B5_ptrs[1],
                      w, params->sgr.s0, BITDEPTH_MAX);
        sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                      w, params->sgr.s1, BITDEPTH_MAX);
        sgr_finish_mix(&dst, stride, A5_ptrs, B5_ptrs, A3_ptrs, B3_ptrs,
                       w, 2, params->sgr.w0, params->sgr.w1
                       HIGHBD_TAIL_SUFFIX);
    } while (--h > 0);

    if (!(edges & LR_HAVE_BOTTOM))
        goto vert_2;

    sgr_box35_row_h(sumsq3_ptrs[2], sum3_ptrs[2],
                    sumsq5_ptrs[3], sum5_ptrs[3],
                    NULL, lpf_bottom, w, edges);
    lpf_bottom += PXSTRIDE(stride);
    sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                  w, params->sgr.s1, BITDEPTH_MAX);
    rotate(A3_ptrs, B3_ptrs, 4);

    sgr_box35_row_h(sumsq3_ptrs[2], sum3_ptrs[2],
                    sumsq5_ptrs[4], sum5_ptrs[4],
                    NULL, lpf_bottom, w, edges);

output_2:
    sgr_box5_vert(sumsq5_ptrs, sum5_ptrs, A5_ptrs[1], B5_ptrs[1],
                  w, params->sgr.s0, BITDEPTH_MAX);
    sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                  w, params->sgr.s1, BITDEPTH_MAX);
    sgr_finish_mix(&dst, stride, A5_ptrs, B5_ptrs, A3_ptrs, B3_ptrs,
                   w, 2, params->sgr.w0, params->sgr.w1
                   HIGHBD_TAIL_SUFFIX);
    return;

vert_2:
    // Duplicate the last row twice more
    sumsq5_ptrs[3] = sumsq5_ptrs[2];
    sumsq5_ptrs[4] = sumsq5_ptrs[2];
    sum5_ptrs[3] = sum5_ptrs[2];
    sum5_ptrs[4] = sum5_ptrs[2];

    sumsq3_ptrs[2] = sumsq3_ptrs[1];
    sum3_ptrs[2] = sum3_ptrs[1];
    sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                  w, params->sgr.s1, BITDEPTH_MAX);
    rotate(A3_ptrs, B3_ptrs, 4);

    sumsq3_ptrs[2] = sumsq3_ptrs[1];
    sum3_ptrs[2] = sum3_ptrs[1];

    goto output_2;

odd:
    // Copy the last row as padding once
    sumsq5_ptrs[4] = sumsq5_ptrs[3];
    sum5_ptrs[4] = sum5_ptrs[3];

    sumsq3_ptrs[2] = sumsq3_ptrs[1];
    sum3_ptrs[2] = sum3_ptrs[1];

    sgr_box5_vert(sumsq5_ptrs, sum5_ptrs, A5_ptrs[1], B5_ptrs[1],
                  w, params->sgr.s0, BITDEPTH_MAX);
    sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                  w, params->sgr.s1, BITDEPTH_MAX);
    sgr_finish_mix(&dst, stride, A5_ptrs, B5_ptrs, A3_ptrs, B3_ptrs,
                   w, 2, params->sgr.w0, params->sgr.w1
                   HIGHBD_TAIL_SUFFIX);

output_1:
    // Duplicate the last row twice more
    sumsq5_ptrs[3] = sumsq5_ptrs[2];
    sumsq5_ptrs[4] = sumsq5_ptrs[2];
    sum5_ptrs[3] = sum5_ptrs[2];
    sum5_ptrs[4] = sum5_ptrs[2];

    sumsq3_ptrs[2] = sumsq3_ptrs[1];
    sum3_ptrs[2] = sum3_ptrs[1];

    sgr_box5_vert(sumsq5_ptrs, sum5_ptrs, A5_ptrs[1], B5_ptrs[1],
                  w, params->sgr.s0, BITDEPTH_MAX);
    sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                  w, params->sgr.s1, BITDEPTH_MAX);
    rotate(A3_ptrs, B3_ptrs, 4);
    // Output only one row
    sgr_finish_mix(&dst, stride, A5_ptrs, B5_ptrs, A3_ptrs, B3_ptrs,
                   w, 1, params->sgr.w0, params->sgr.w1
                   HIGHBD_TAIL_SUFFIX);
    return;

vert_1:
    // Copy the last row as padding once
    sumsq5_ptrs[4] = sumsq5_ptrs[3];
    sum5_ptrs[4] = sum5_ptrs[3];

    sumsq3_ptrs[2] = sumsq3_ptrs[1];
    sum3_ptrs[2] = sum3_ptrs[1];

    sgr_box5_vert(sumsq5_ptrs, sum5_ptrs, A5_ptrs[1], B5_ptrs[1],
                  w, params->sgr.s0, BITDEPTH_MAX);
    rotate(A5_ptrs, B5_ptrs, 2);
    sgr_box3_vert(sumsq3_ptrs, sum3_ptrs, A3_ptrs[3], B3_ptrs[3],
                  w, params->sgr.s1, BITDEPTH_MAX);
    rotate(A3_ptrs, B3_ptrs, 4);

    goto output_1;
}

#if HAVE_ASM
#if ARCH_AARCH64 || ARCH_ARM
#include "src/arm/looprestoration.h"
#elif ARCH_LOONGARCH64
#include "src/loongarch/looprestoration.h"
#elif ARCH_PPC64LE
#include "src/ppc/looprestoration.h"
#elif ARCH_X86
#include "src/x86/looprestoration.h"
#endif
#endif

COLD void bitfn(dav2d_loop_restoration_dsp_init)(Dav2dLoopRestorationDSPContext *const c,
                                                 const int bpc)
{
    c->ns_wiener_single = ns_wiener_single_y_c;
    c->ns_wiener_multi = ns_wiener_multi_c;
    c->pc_wiener = pc_wiener_c;

    c->wiener[0] = c->wiener[1] = wiener_c;
    c->sgr[0] = sgr_5x5_c;
    c->sgr[1] = sgr_3x3_c;
    c->sgr[2] = sgr_mix_c;

#if HAVE_ASM
#if ARCH_AARCH64 || ARCH_ARM
    loop_restoration_dsp_init_arm(c, bpc);
#elif ARCH_LOONGARCH64
    loop_restoration_dsp_init_loongarch(c, bpc);
#elif ARCH_PPC64LE
    loop_restoration_dsp_init_ppc(c, bpc);
#elif ARCH_X86
    loop_restoration_dsp_init_x86(c, bpc);
#endif
#endif
}
