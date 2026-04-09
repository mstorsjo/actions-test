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
#include "src/gdf_tables.h"

// 64 + 6 + 6
#define REST_UNIT_STRIDE (76)
// 64 / 4
#define CLASS_BUF_SIZE (16)
// 64 / 2 + 1
#define GRADIENT_BUF_STRIDE (33)

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

static const int8_t wiener_ns_config_uv[6][2] = {
    { 1, 0 }, { 0, 1 }, { 1, 1 }, { -1, 1 }, { 2, 0 }, { 0, 2 },
};

static const int8_t wiener_ns_config_uv_from_y[12][3] = {
    { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 }, { 1, 1 }, { -1, -1 },
    { -1, 1 }, { 1, -1 }, { 2, 0 }, { -2, 0 }, { 0, 2 }, { 0, -2 },
};

static const int8_t pc_wiener_config[25][2] = {
    {  1,  0 }, { -1,  0 }, {  0,  1 }, {  0, -1 }, {  2,  0 },
    { -2,  0 }, {  0,  2 }, {  0, -2 }, {  1,  1 }, { -1, -1 },
    { -1,  1 }, {  1, -1 }, {  2,  1 }, { -2, -1 }, {  2, -1 },
    { -2,  1 }, {  1,  2 }, { -1, -2 }, {  1, -2 }, { -1,  2 },
    {  3,  0 }, { -3,  0 }, {  0,  3 }, {  0, -3 }, {  0,  0 }
};

static const int8_t gdf_coords[18][2] = {
    { 6,  0 }, { 5,  0 }, { 4,  0 }, { 3,  0 }, { 2,  1 }, { 2,  0 },
    { 2, -1 }, { 1,  2 }, { 1,  1 }, { 1,  0 }, { 1, -1 }, { 1, -2 },
    { 0,  6 }, { 0,  5 }, { 0,  4 }, { 0,  3 }, { 0,  2 }, { 0,  1 }
};

static const uint16_t pc_wiener_normalizer[ 4 ] = { 3739, 3273, 3074, 7 };

static const int16_t mode_weights[ 4 ][ 3 ] = {
    { -527, 15325, 321 }, { 26436, -17705, 17905 }, { 366, -147, -194 }, { 202, -267, -179 }
};

static const int16_t mode_offsets[ 4 ] = { -547, -21565, -573, -680 };

static void backup_row(pixel *dst, const pixel *src, const pixel *left, const int w,
                       const int edge_len, const enum LrEdgeFlags edges)
{
    if (edges & LR_HAVE_LEFT)
        for (int x = -edge_len; x < 0; x++)
            dst[x] = left[x + 6];
    else
        for (int x = -edge_len; x < 0; x++)
            dst[x] = src[0];

    for (int x = 0; x < w; x++)
        dst[x] = src[x];

    if (edges & LR_HAVE_RIGHT)
        for (int x = w; x < w + edge_len; x++)
            dst[x] = src[x];
    else
        for (int x = w; x < w + edge_len; x++)
            dst[x] = src[w - 1];
}

static void backup_row_lpf(pixel *dst, const pixel *src, const int w,
                           const int edge_len, const enum LrEdgeFlags edges)
{
    if (edges & LR_HAVE_LEFT) {
        for (int x = -edge_len; x < 0; x++)
            dst[x] = src[x];
    } else {
        for (int x = -edge_len; x < 0; x++)
            dst[x] = src[0];
    }

    for (int x = 0; x < w; x++)
        dst[x] = src[x];

    if (edges & LR_HAVE_RIGHT) {
        for (int x = w; x < w + edge_len; x++)
            dst[x] = src[x];
    } else {
        for (int x = w; x < w + edge_len; x++)
            dst[x] = src[w - 1];
    }
}

static void backup_row_luma(pixel *dst, const pixel *src, const ptrdiff_t src_stride,
                            const int w, const enum LrEdgeFlags edges,
                            const int ss_hor, const int ss_ver,
                            const int cfl_ds_flt)
{
    if (!ss_ver) {
        backup_row_lpf(dst, src, w, 4, edges);
        return;
    }

    const pixel *src2 = &src[PXSTRIDE(src_stride)];
    switch (cfl_ds_flt) {
    case 0:
        for (int x = 0; x < w; x += 1 + ss_hor)
            dst[x] = (src[x] + src2[x] + src[x + 1] + src2[x + 1]) >> 2;
        break;
    case 1:
        for (int x = 0; x < w; x++)
            dst[x] = (src[x] + src2[x]) >> 1;
        break;
    default:
        assert(0);
        // fall-through in non-debug mode
    case 2:
        for (int x = 0; x < w; x++)
            dst[x] = src[x];
        break;
    }

    if (edges & LR_HAVE_LEFT) {
        switch (cfl_ds_flt) {
        case 0:
            for (int x = -4; x < 0; x++)
                dst[x] = (src[x] + src2[x] + src[x + 1] + src2[x + 1]) >> 2;
            break;
        case 1:
            for (int x = -4; x < 0; x++)
                dst[x] = (src[x] + src2[x]) >> 1;
            break;
        default:
            assert(0);
            // fall-through in non-debug mode
        case 2:
            for (int x = -4; x < 0; x++)
                dst[x] = src[x];
            break;
        }
    } else {
        for (int x = -4; x < 0; x++)
            dst[x] = dst[0];
    }

    if (edges & LR_HAVE_RIGHT) {
        switch (cfl_ds_flt) {
        case 0:
            for (int x = w; x < w + 4; x++)
                dst[x] = (src[x] + src2[x] + src[x + 1] + src2[x + 1]) >> 2;
            break;
        case 1:
            for (int x = w; x < w + 4; x++)
                dst[x] = (src[x] + src2[x]) >> 1;
            break;
        default:
            assert(0);
            // fall-through in non-debug mode
        case 2:
            for (int x = w; x < w + 4; x++)
                dst[x] = src[x];
            break;
        }
    } else {
        for (int x = w; x < w + 4; x++)
            dst[x] = dst[w - 2];
    }
}

static void ns_wiener_single_y_c(pixel *p, const ptrdiff_t stride,
                                 const pixel (*left)[6],
                                 const pixel *lpf, const pixel *lpf_bottom,
                                 const int w, int h, const WienerParams *params,
                                 const enum LrEdgeFlags edges,
                                 const uint16_t (*ll_mask)[4] HIGHBD_DECL_SUFFIX)
{
    const int8_t *filter = params->single.filter;
    pixel row_buffers[9][REST_UNIT_STRIDE];
    pixel *bak_rows[9];
    const pixel *ptrs[9];

    for (int i = 0; i < 9; i++)
        bak_rows[i] = row_buffers[i] + 6;

    backup_row(bak_rows[4], p, left[0], w, 4, edges);
    ptrs[4] = bak_rows[4];
    if (edges & LR_HAVE_TOP_INTEGRATED) {
        for (int i = 0; i < 4; i++) {
            backup_row_lpf(bak_rows[i], lpf, w, 4, edges);
            lpf += PXSTRIDE(stride);
            ptrs[i] = bak_rows[i];
        }
    } else if (edges & LR_HAVE_TOP) {
        // y = -2,-1
        backup_row_lpf(bak_rows[2], lpf, w, 4, edges);
        ptrs[2] = bak_rows[2];
        backup_row_lpf(bak_rows[3], lpf + PXSTRIDE(stride), w, 4, edges);
        ptrs[3] = bak_rows[3];

        // y = -3,-4
        ptrs[0] = ptrs[1] = ptrs[2];
    } else {
        ptrs[0] = ptrs[1] = ptrs[2] = ptrs[3] = ptrs[4];
    }

    backup_row(bak_rows[5], p + PXSTRIDE(stride), left[1], w, 4, edges);
    ptrs[5] = bak_rows[5];
    backup_row(bak_rows[6], p + 2*PXSTRIDE(stride), left[2], w, 4, edges);
    ptrs[6] = bak_rows[6];
    backup_row(bak_rows[7], p + 3*PXSTRIDE(stride), left[3], w, 4, edges);
    ptrs[7] = bak_rows[7];
    int bak_idx = 8;

    for (int y = 0; y < h; y++) {
        if (y + 4 < h) {
            backup_row(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), left[y + 4], w, 4, edges);
            ptrs[8] = bak_rows[bak_idx];
        } else if (edges & LR_HAVE_BOTTOM_INTEGRATED) {
            backup_row_lpf(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), w, 4, edges);
            ptrs[8] = bak_rows[bak_idx];
        } else if (y + 2 < h && edges & LR_HAVE_BOTTOM) {
            int offset_y = y + 4 - h;
            assert(offset_y < 2);
            backup_row_lpf(bak_rows[bak_idx], lpf_bottom + offset_y * PXSTRIDE(stride), w, 4, edges);
            ptrs[8] = bak_rows[bak_idx];
        } else {
            ptrs[8] = ptrs[7];
        }
        if (++bak_idx == 9) bak_idx = 0;

        for (int bx = 0; bx < (w >> 2); bx++) {
            if (ll_mask[y >> 2][0] & (1 << bx)) continue;
            for (int x = bx * 4; x < bx * 4 + 4; x++) {
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
        }

        for (int r = 0; r < 8; r++) ptrs[r] = ptrs[r+1];
        p += PXSTRIDE(stride);
    }
}

static void ns_wiener_single_uv_c(pixel *p, const ptrdiff_t stride,
                                  const pixel (*left)[6], // FIXME this can be 2
                                  const pixel *lpf, const pixel *lpf_bottom,
                                  const int w, int h, const WienerParams *params,
                                  const enum LrEdgeFlags edges,
                                  const uint16_t (*ll_mask)[4] HIGHBD_DECL_SUFFIX)
{
    const int8_t *filter = params->single.filter;
    pixel row_buffers_c[5][REST_UNIT_STRIDE];
    pixel row_buffers_l[5][REST_UNIT_STRIDE + 64];
    pixel *bak_rows[2][5];
    const pixel *ptrs[2][5];

    for (int i = 0; i < 5; i++) {
        bak_rows[0][i] = row_buffers_c[i] + 6;
        bak_rows[1][i] = row_buffers_l[i] + 6;
    }

    // FIXME we only need 2px edges here (left/right), not 4
    backup_row(bak_rows[0][2], p, left[0], w, 2, edges);
    ptrs[0][2] = bak_rows[0][2];
    if (edges & (LR_HAVE_TOP_INTEGRATED | LR_HAVE_TOP)) {
        // y = -2,-1
        for (int i = 0; i < 2; i++) {
            backup_row_lpf(bak_rows[0][i], lpf, w, 2, edges);
            ptrs[0][i] = bak_rows[0][i];
            lpf += PXSTRIDE(stride);
        }
    } else {
        ptrs[0][0] = ptrs[0][1] = ptrs[0][2];
    }

    backup_row(bak_rows[0][3], p + PXSTRIDE(stride), left[1], w, 2, edges);
    ptrs[0][3] = bak_rows[0][3];
    int bak_idx = 4;

    const pixel *luma = params->single.luma;
    const ptrdiff_t lstride = params->single.stride;
    const int ss_hor = params->single.ss_hor, ss_ver = params->single.ss_ver;
    backup_row_luma(bak_rows[1][2], luma, lstride, w << ss_hor, edges,
                    ss_hor, ss_ver, params->single.ds_flt);
    ptrs[1][2] = bak_rows[1][2];
    if (edges & LR_HAVE_TOP_INTEGRATED) {
        backup_row_luma(bak_rows[1][0], params->single.luma - 4 * PXSTRIDE(lstride),
                        lstride, w << ss_hor, edges, ss_hor, ss_ver, params->single.ds_flt);
        ptrs[1][0] = bak_rows[1][0];
        backup_row_luma(bak_rows[1][1], params->single.luma - 2 * PXSTRIDE(lstride),
                        lstride, w << ss_hor, edges, ss_hor, ss_ver, params->single.ds_flt);
        ptrs[1][1] = bak_rows[1][1];
    } else if (edges & LR_HAVE_TOP) {
        backup_row_luma(bak_rows[1][0], params->single.luma_top,
                        0, w << ss_hor, edges, ss_hor, ss_ver, params->single.ds_flt);
        ptrs[1][0] = bak_rows[1][0];
        backup_row_luma(bak_rows[1][1], params->single.luma_top,
                        lstride, w << ss_hor, edges, ss_hor, ss_ver, params->single.ds_flt);
        ptrs[1][1] = bak_rows[1][1];
    } else {
        ptrs[1][0] = ptrs[1][1] = ptrs[1][2];
    }
    backup_row_luma(bak_rows[1][3], luma + (1 << ss_ver) * PXSTRIDE(lstride),
                    lstride, w << ss_hor, edges, ss_hor, ss_ver, params->single.ds_flt);
    ptrs[1][3] = bak_rows[1][3];
    int lbak_idx = 4;

    for (int y = 0; y < h; y++) {
        if (y + 2 < h) {
            backup_row(bak_rows[0][bak_idx], p + 2*PXSTRIDE(stride), left[y + 2], w, 2, edges);
            ptrs[0][4] = bak_rows[0][bak_idx];
        } else if (edges & LR_HAVE_BOTTOM_INTEGRATED) {
            backup_row_lpf(bak_rows[0][bak_idx], p + 2*PXSTRIDE(stride), w, 2, edges);
            ptrs[0][4] = bak_rows[0][bak_idx];
        } else if (edges & LR_HAVE_BOTTOM) {
            int offset_y = y + 2 - h;
            assert(offset_y < 2);
            backup_row_lpf(bak_rows[0][bak_idx], lpf_bottom + offset_y * PXSTRIDE(stride), w, 2, edges);
            ptrs[0][4] = bak_rows[0][bak_idx];
        } else {
            ptrs[0][4] = ptrs[0][3];
        }
        if (++bak_idx == 5) bak_idx = 0;

        if (ptrs[0][4] == ptrs[0][3]) {
            ptrs[1][4] = ptrs[1][3];
        } else if (y + 2 == h && !(edges & LR_HAVE_BOTTOM_INTEGRATED)) {
            backup_row_luma(bak_rows[1][lbak_idx], params->single.luma_bottom,
                            lstride, w << ss_hor, edges, ss_hor, ss_ver, params->single.ds_flt);
            ptrs[1][4] = bak_rows[1][lbak_idx];
        } else if (y + 1 == h && !(edges & LR_HAVE_BOTTOM_INTEGRATED)) {
            backup_row_luma(bak_rows[1][lbak_idx], params->single.luma_bottom + PXSTRIDE(lstride),
                            0, w << ss_hor, edges, ss_hor, ss_ver, params->single.ds_flt);
            ptrs[1][4] = bak_rows[1][lbak_idx];
        } else {
            backup_row_luma(bak_rows[1][lbak_idx], luma + (2 << ss_ver) * PXSTRIDE(lstride),
                            lstride, w << ss_hor, edges, ss_hor, ss_ver, params->single.ds_flt);
            ptrs[1][4] = bak_rows[1][lbak_idx];
        }
        if (++lbak_idx == 5) lbak_idx = 0;

        for (int bx = 0; bx < (w >> 2); bx++) {
            if (ll_mask[y >> 2][0] & (1 << bx)) continue;
            for (int x = bx * 4; x < bx * 4 + 4; x++) {
                const int m = ptrs[0][2][x];
                int s = m << 7;
                for (int i = 0; i < 6; i++) {
                    const int dy = wiener_ns_config_uv[i][0];
                    const int dx = wiener_ns_config_uv[i][1];
                    const int diff = ptrs[0][2 + dy][x + dx] + ptrs[0][2 - dy][x - dx] - 2 * m;
                    s += diff * filter[i];
                }
                const int l = ptrs[1][2][x << ss_hor];
                for (int i = 0; i < 12; i++) {
                    const int dy = wiener_ns_config_uv_from_y[i][0];
                    const int dx = wiener_ns_config_uv_from_y[i][1];
                    const int diff = ptrs[1][2 + dy][(x + dx) << ss_hor] - l;
                    s += diff * filter[6 + i];
                }
                const int v = (s + 64) >> 7;
                p[x] = iclip_pixel(v);
            }
        }

        for (int r = 0; r < 4; r++) ptrs[0][r] = ptrs[0][r + 1];
        for (int r = 0; r < 4; r++) ptrs[1][r] = ptrs[1][r + 1];
        p += PXSTRIDE(stride);
        luma += PXSTRIDE(lstride) << ss_ver;
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

static int get_class_lut_idx(const pixel *ptrs[10], const uint16_t *noskip_mask, const int base_q,
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
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            const int edge = !!dy + !!dx;
            const int fx = iclip((bx & 15) + dx, 0, 15);
            const int fy = iclip(by + dy, 0, bh - 1);
            s += num_pixels[edge] * !((noskip_mask[fy] >> fx) & 1);
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
                         const pixel (*left)[6],
                         const pixel *lpf, const pixel *lpf_bottom,
                         const int w, int h,
                         const int8_t (*filters_user)[18],
                         const int16_t (*filters_pretrained)[13],
                         const uint8_t *subclass_lut,
                         const uint16_t *noskip_mask,
                         const int base_q, const enum LrEdgeFlags edges,
                         const uint16_t (*ll_mask)[4] HIGHBD_DECL_SUFFIX)
{
    const int bitdepth_min_8 = bitdepth_from_max(bitdepth_max) - 8;

    uint8_t classes[CLASS_BUF_SIZE];
    pixel row_buffers[10][REST_UNIT_STRIDE];
    pixel *bak_rows[10];
    const pixel *ptrs[10];

    for (int i = 0; i < 10; i++)
        bak_rows[i] = row_buffers[i] + 6;

    backup_row(bak_rows[4], p, left[0], w, 4, edges);
    ptrs[4] = bak_rows[4];
    if (edges & LR_HAVE_TOP_INTEGRATED) {
        for (int i = 0; i < 4; i++) {
            backup_row_lpf(bak_rows[i], lpf, w, 4, edges);
            lpf += PXSTRIDE(stride);
            ptrs[i] = bak_rows[i];
        }
    } else if (edges & LR_HAVE_TOP) {
        // y = -2,-1
        backup_row_lpf(bak_rows[2], lpf, w, 4, edges);
        ptrs[2] = bak_rows[2];
        backup_row_lpf(bak_rows[3], lpf + PXSTRIDE(stride), w, 4, edges);
        ptrs[3] = bak_rows[3];

        // y = -3,-4
        ptrs[0] = ptrs[1] = ptrs[2];
    } else {
        ptrs[0] = ptrs[1] = ptrs[2] = ptrs[3] = ptrs[4];
    }

    backup_row(bak_rows[5], p + PXSTRIDE(stride), left[1], w, 4, edges);
    ptrs[5] = bak_rows[5];
    backup_row(bak_rows[6], p + 2*PXSTRIDE(stride), left[2], w, 4, edges);
    ptrs[6] = bak_rows[6];
    backup_row(bak_rows[7], p + 3*PXSTRIDE(stride), left[3], w, 4, edges);
    ptrs[7] = bak_rows[7];
    int bak_idx = 8;

    const int bh = h >> 2;
    const int bw = w >> 2;
    for (int by = 0; by < bh; by++) {
        // Backup an extra row to compute class
        if (by + 1 < bh) {
            // TODO: don't backup lines twice
            backup_row(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), left[(by << 2) + 4], w, 4, edges);
            ptrs[8] = bak_rows[bak_idx];
            backup_row(bak_rows[9], p + 5*PXSTRIDE(stride), left[(by << 2) + 5], w, 4, edges);
            ptrs[9] = bak_rows[9];
        } else if (edges & LR_HAVE_BOTTOM_INTEGRATED) {
            backup_row_lpf(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), w, 4, edges);
            ptrs[8] = bak_rows[bak_idx];
            backup_row_lpf(bak_rows[9], p + 5*PXSTRIDE(stride), w, 4, edges);
            ptrs[9] = bak_rows[9];
        } else if (edges & LR_HAVE_BOTTOM) {
            backup_row_lpf(bak_rows[bak_idx], lpf_bottom + 0 * PXSTRIDE(stride), w, 4, edges);
            ptrs[8] = bak_rows[bak_idx];
            backup_row_lpf(bak_rows[9], lpf_bottom + 1 * PXSTRIDE(stride), w, 4, edges);
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
                backup_row(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), left[y + 4], w, 4, edges);
                ptrs[8] = bak_rows[bak_idx];
            } else if (edges & LR_HAVE_BOTTOM_INTEGRATED) {
                backup_row_lpf(bak_rows[bak_idx], p + 4*PXSTRIDE(stride), w, 4, edges);
                ptrs[8] = bak_rows[bak_idx];
            } else if (y + 2 < h && edges & LR_HAVE_BOTTOM) {
                int offset_y = y + 4 - h;
                assert(offset_y < 2);
                backup_row_lpf(bak_rows[bak_idx], lpf_bottom + offset_y * PXSTRIDE(stride), w, 4, edges);
                ptrs[8] = bak_rows[bak_idx];
            } else {
                ptrs[8] = ptrs[7];
            }
            if (++bak_idx == 9) bak_idx = 0;

            for (int bx = 0; bx < bw; bx++) {
                if (ll_mask[y >> 2][0] & (1 << bx)) continue;
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
                              const pixel (*left)[6],
                              const pixel *lpf, const pixel *lpf_bottom,
                              const int w, int h, const WienerParams *params,
                              const enum LrEdgeFlags edges,
                              const uint16_t (*ll_mask)[4] HIGHBD_DECL_SUFFIX)
{
    wiener_multi(p, stride, left, lpf, lpf_bottom, w, h,
                 params->multi.filters.user, NULL,
                 params->multi.subclass_lut, params->multi.noskip_mask,
                 params->multi.base_q, edges, ll_mask HIGHBD_TAIL_SUFFIX);
}

static void pc_wiener_c(pixel *p, const ptrdiff_t stride,
                        const pixel (*left)[6],
                        const pixel *lpf, const pixel *lpf_bottom,
                        const int w, int h, const WienerParams *params,
                        const enum LrEdgeFlags edges,
                        const uint16_t (*ll_mask)[4] HIGHBD_DECL_SUFFIX)
{
    wiener_multi(p, stride, left, lpf, lpf_bottom, w, h,
                 NULL, params->multi.filters.pretrained,
                 params->multi.subclass_lut, params->multi.noskip_mask,
                 params->multi.base_q, edges, ll_mask HIGHBD_TAIL_SUFFIX);
}

// Sum 2x2 rows of gradients and store in dst
static void compute_gradient_row(uint16_t (*dst)[4], const pixel **src,
                                 const int w, const int shift)
{
    for (int x1 = 0; x1 < w + 2; x1 += 2) {
        const int8_t offs[4][2] = { { 1, 0 }, { 0, 1 }, { 1, 1 }, { -1, 1 } };
        for (int d = 0; d < 4; d++) {
            int grad = 0;
            for (int x2 = 0; x2 < 2; x2++) {
                int x = x1 + x2;
                for (int y = 0; y < 2; y++) {
                    int dy = offs[d][0];
                    int dx = offs[d][1];
                    int a = src[y - 1 - dy][x - 1 - dx] >> shift;
                    int b = src[y - 1][x - 1] >> shift;
                    int c = src[y - 1 + dy][x - 1 + dx] >> shift;
                    grad += abs(b * 2 - a - c);
                }
            }
            dst[x1 >> 1][d] = grad;
        }
    }
}

static void gdf_prep_c(int8_t *dst, const ptrdiff_t dst_stride,
                       const pixel *p, const ptrdiff_t stride,
                       const pixel (*left)[6], const pixel *lpf,
                       const pixel *lpf_bottom, const int w, const int h,
                       const int ref_dst_idx, const int qp_idx,
                       enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
{
    const int bitdepth = bitdepth_from_max(bitdepth_max);
    const int down_shift = bitdepth == 12 ? 2 : 0;
    const int up_shift = bitdepth == 8 ? 2 : 0;

    uint16_t grad[2][GRADIENT_BUF_STRIDE][4];
    pixel row_buffers[13][REST_UNIT_STRIDE];
    pixel *bak_rows[13];
    const pixel *ptrs[13];

    for (int i = 0; i < 13; i++)
        bak_rows[i] = row_buffers[i] + 6;

    backup_row(bak_rows[6], p, left[0], w, 6, edges);
    ptrs[6] = bak_rows[6];
    if (edges & LR_HAVE_TOP_INTEGRATED) {
        for (int n = 0; n < 6; n++) {
            backup_row_lpf(bak_rows[n], lpf + n * PXSTRIDE(stride), w, 6, edges);
            ptrs[n] = bak_rows[n];
        }
    } else if (edges & LR_HAVE_TOP) {
        // y = -2,-1
        backup_row_lpf(bak_rows[4], lpf, w, 6, edges);
        ptrs[4] = bak_rows[4];
        backup_row_lpf(bak_rows[5], lpf + PXSTRIDE(stride), w, 6, edges);
        ptrs[5] = bak_rows[5];

        // y = -3,-4,-5,-6
        ptrs[0] = ptrs[1] = ptrs[2] = ptrs[3] = ptrs[4];
    } else {
        ptrs[0] = ptrs[1] = ptrs[2] = ptrs[3] = ptrs[4] = ptrs[5] = ptrs[6];
    }

    int bak_idx = 7;
    for (int y = 1; y < 6; y++, bak_idx++) {
        backup_row(bak_rows[bak_idx], p + y * PXSTRIDE(stride), left[y], w, 6, edges);
        ptrs[bak_idx] = bak_rows[bak_idx];
    }

    const int8_t *error_lut;
    int scale;
    compute_gradient_row(grad[0], &ptrs[6], w, down_shift);
    int grad_bit = 1;

    if (ref_dst_idx == 0) {
        error_lut = dav2d_gdf_intra_error[qp_idx];
        scale = 8;
    } else {
        error_lut = dav2d_gdf_inter_error[ref_dst_idx - 1][qp_idx];
        scale = 5;
    }

    for (int y = 0; y < h; y++) {
        if (y + 6 < h) {
            backup_row(bak_rows[bak_idx], p + 6*PXSTRIDE(stride), left[y + 6], w, 6, edges);
            ptrs[12] = bak_rows[bak_idx];
        } else if (edges & LR_HAVE_BOTTOM_INTEGRATED) {
            backup_row_lpf(bak_rows[bak_idx], p + 6*PXSTRIDE(stride), w, 6, edges);
            ptrs[12] = bak_rows[bak_idx];
        } else if (y + 4 < h && edges & LR_HAVE_BOTTOM) {
            int offset_y = y + 6 - h;
            assert(offset_y < 2);
            backup_row_lpf(bak_rows[bak_idx], lpf_bottom + offset_y * PXSTRIDE(stride), w, 6, edges);
            ptrs[12] = bak_rows[bak_idx];
        } else {
            ptrs[12] = ptrs[11];
        }
        if (++bak_idx == 13) bak_idx = 0;
        if ((y & 1) == 0) {
            compute_gradient_row(grad[grad_bit], &ptrs[8], w, down_shift);
            grad_bit ^= 1;
        }

        for (int x1 = 0; x1 < w; x1 += 2) {
            // TODO: Don't recompute the same grad_sum/shared_vals on odd rows.
            int grad_sums[4] = { 0, 0, 0, 0 };
            int shared_vals[3];
            for (int d = 0; d < 4; d++) {
                int hx = x1 >> 1;
                // Compute gradients over a 4x4 region
                grad_sums[d] = grad[0][hx][d] + grad[0][hx + 1][d] +
                               grad[1][hx][d] + grad[1][hx + 1][d];
            }
            int cls = (grad_sums[0] <= grad_sums[1]) | ((grad_sums[2] <= grad_sums[3]) << 1);

            for (int idx = 0; idx < 3; idx++)
                shared_vals[idx] = dav2d_gdf_bias[ref_dst_idx][qp_idx][idx];
            for (int d = 0; d < 4; d++) {
                const int k = d + 18;
                const int alpha = dav2d_gdf_alpha[ref_dst_idx][qp_idx][k][cls];
                const int v = imin(grad_sums[d] >> (4 - up_shift), alpha);
                for (int idx = 0; idx < 3; idx++)
                    shared_vals[idx] += v * dav2d_gdf_weight[ref_dst_idx][qp_idx][idx][k][cls];
            }

            for (int x2 = 0; x2 < 2; x2++) {
                int x = x1 + x2;
                int idx_vals[3];
                int m = ptrs[6][x] >> down_shift;
                for (int idx = 0; idx < 3; idx++)
                    idx_vals[idx] = shared_vals[idx];
                for (int k = 0; k < 18; k++) {
                    const int alpha = dav2d_gdf_alpha[ref_dst_idx][qp_idx][k][cls];
                    const int dy = gdf_coords[k][0];
                    const int dx = gdf_coords[k][1];
                    const int a = ptrs[6 - dy][x - dx] >> down_shift;
                    const int b = ptrs[6 + dy][x + dx] >> down_shift;
                    const int above = iclip((a - m) << up_shift, -alpha, alpha);
                    const int below = iclip((b - m) << up_shift, -alpha, alpha);
                    const int v = iclip(above + below, -512, 511);
                    for (int idx = 0; idx < 3; idx++)
                        idx_vals[idx] += v * dav2d_gdf_weight[ref_dst_idx][qp_idx][idx][k][cls];
                }

                int full_idx = 0;
                for (int idx = 0; idx < 3; idx++) {
                    int v = idx_vals[idx] * scale;
                    v = apply_sign((abs(v) + (1 << 14)) >> 15, v);
                    int sub_idx = iclip(v, -scale, scale - 1) + scale;
                    full_idx = full_idx * scale * 2 + sub_idx;
                }
                dst[x] = error_lut[full_idx];
            }
        }

        for (int r = 0; r < 12; r++) ptrs[r] = ptrs[r+1];
        dst += dst_stride;
        p += PXSTRIDE(stride);
    }
}

static void gdf_add_c(pixel *p_line, const ptrdiff_t stride,
                      const int8_t *err_line, const ptrdiff_t err_stride,
                      const int w, const int h, const int scale,
                      const uint16_t (*ll_mask)[4] HIGHBD_DECL_SUFFIX)
{
    const int shift = 12 - bitdepth_from_max(bitdepth_max);
    const int rnd = 1 << shift >> 1;
    for (int by = 0; by < h >> 2; by++) {
        for (int bx = 0; bx < w >> 2; bx++) {
            if (ll_mask[by][0] & (1 << bx)) continue;
            pixel *p = p_line;
            const int8_t *err = err_line;
            for (int y = by * 4; y < by * 4 + 4; y++) {
                for (int x = bx * 4; x < bx * 4 + 4; x++) {
                    int diff = err[x] * scale;
                    p[x] = iclip_pixel(p[x] + apply_sign((abs(diff) + rnd) >> shift, diff));
                }
                p += PXSTRIDE(stride);
                err += err_stride;
            }
        }
        p_line += PXSTRIDE(stride) * 4;
        err_line += err_stride * 4;
    }
}

#if HAVE_ASM && 0
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
    c->ns_wiener_single[0] = ns_wiener_single_y_c;
    c->ns_wiener_single[1] = ns_wiener_single_uv_c;
    c->ns_wiener_multi = ns_wiener_multi_c;
    c->pc_wiener = pc_wiener_c;

    c->gdf_add = gdf_add_c;
    c->gdf_prep = gdf_prep_c;

#if HAVE_ASM && 0
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
