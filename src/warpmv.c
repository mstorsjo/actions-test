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

#include "common/intops.h"

#include "src/tables.h"
#include "src/warpmv.h"

static inline int iclip_wmp(const int v) {
    return iclip((v + 0x20 - (v < 0)) & ~0x3f, -0x8000, 0x7fc0);
}

int dav2d_resolve_divisor_32(const unsigned d, int *const shift) {
    *shift = ulog2(d);
    const int e = d - (1 << *shift);
    const int f = *shift > 7 ? (e + (1 << (*shift - 8))) >> (*shift - 7) :
                               e << (7 - *shift);
    assert(f <= 128);
    *shift += 9;
    // Use f as lookup into the precomputed table of multipliers
    return dav2d_div_recip[f];
}

int dav2d_get_shear_params(Dav2dWarpedMotionParams *const wm) {
    const int32_t *const mat = wm->matrix;

    if (mat[2] <= 0) return 1;

    wm->u.p.alpha = iclip_wmp(mat[2] - 0x10000);
    wm->u.p.beta = iclip_wmp(mat[3]);

    int shift;
    const int y = apply_sign(dav2d_resolve_divisor_32(abs(mat[2]), &shift), mat[2]);
    const int64_t v1 = ((int64_t) mat[4] * 0x10000) * y;
    const int rnd = (1 << shift) >> 1;
    wm->u.p.gamma = iclip_wmp(apply_sign64((int) ((llabs(v1) + rnd) >> shift), v1));
    const int64_t v2 = ((int64_t) mat[3] * mat[4]) * y;
    wm->u.p.delta = iclip_wmp(mat[5] -
                          apply_sign64((int) ((llabs(v2) + rnd) >> shift), v2) -
                          0x10000);

    wm->affine = (4 * abs(wm->u.p.alpha) + 7 * abs(wm->u.p.beta) < 0x30000) &&
                 (4 * abs(wm->u.p.gamma) + 4 * abs(wm->u.p.delta) < 0x30000);
    return 0;
}

static int resolve_divisor_64(const uint64_t d, int *const shift) {
    *shift = u64log2(d);
    const int64_t e = d - (1LL << *shift);
    const int64_t f = *shift > 7 ? (e + (1LL << (*shift - 8))) >> (*shift - 7) :
                                   e << (7 - *shift);
    assert(f <= 128);
    *shift += 9;
    // Use f as lookup into the precomputed table of multipliers
    return dav2d_div_recip[f];
}

static int get_mult_shift_ndiag(const int64_t px, const int idet,
                                const int64_t rnd, const int sh)
{
    const int64_t v1 = px * idet;
    const int v2 = (int) ((v1 + rnd - (v1 < 0)) >> sh);
    const int v3 = (v2 + 0x20 - (v2 < 0)) & ~0x3f;
    return iclip(v3, -0x7fc0, 0x7fc0);
}

static int get_mult_shift_diag(const int64_t px, const int idet,
                               const int64_t rnd, const int sh)
{
    const int64_t v1 = px * idet;
    const int v2 = (int) ((v1 + rnd - (v1 < 0)) >> sh);
    const int v3 = (v2 + 0x20 - (v2 < 0x10000)) & ~0x3f;
    return iclip(v3, 0x8040, 0x17fc0);
}

void dav2d_set_affine_mv2d(const int bw4, const int bh4,
                           const mv mv, Dav2dWarpedMotionParams *const wm,
                           const int bx4, const int by4)
{
    int32_t *const mat = wm->matrix;
    const int rsuy = 2 * bh4 - 1;
    const int rsux = 2 * bw4 - 1;
    const int isuy = by4 * 4 + rsuy;
    const int isux = bx4 * 4 + rsux;

    mat[0] = iclip64to32(mv.x * 0x2000LL - (int64_t) isux * (mat[2] - 0x10000) -
                         (int64_t) isuy * mat[3], -0x8000000, 0x7ffffc0);
    mat[1] = iclip64to32(mv.y * 0x2000LL - (int64_t) isux * mat[4] -
                         (int64_t) isuy * (mat[5] - 0x10000), -0x8000000, 0x7ffffc0);
}

int dav2d_find_affine_int(const int (*pts)[2][2], const int np,
                          const int bw4, const int bh4,
                          const mv mv, Dav2dWarpedMotionParams *const wm,
                          const int bx4, const int by4)
{
    int32_t *const mat = wm->matrix;
    int a[2][2] = { { 0, 0 }, { 0, 0 } };
    int bx[2] = { 0, 0 };
    int by[2] = { 0, 0 };
    const int rsuy = 2 * bh4 - 1;
    const int rsux = 2 * bw4 - 1;
    const int suy = rsuy * 8;
    const int sux = rsux * 8;
    const int duy = suy + mv.y;
    const int dux = sux + mv.x;

    for (int i = 0; i < np; i++) {
        const int dx = pts[i][1][0] - dux;
        const int dy = pts[i][1][1] - duy;
        const int sx = pts[i][0][0] - sux;
        const int sy = pts[i][0][1] - suy;
        if (abs(sx - dx) < 256 && abs(sy - dy) < 256) {
            a[0][0] += ((sx * sx) >> 2) + sx * 2 + 8;
            a[0][1] += ((sx * sy) >> 2) + sx + sy + 4;
            a[1][1] += ((sy * sy) >> 2) + sy * 2 + 8;
            bx[0] += ((sx * dx) >> 2) + sx + dx + 8;
            bx[1] += ((sy * dx) >> 2) + sy + dx + 4;
            by[0] += ((sx * dy) >> 2) + sx + dy + 4;
            by[1] += ((sy * dy) >> 2) + sy + dy + 8;
        }
    }

    // compute determinant of a
    const int64_t det = (int64_t) a[0][0] * a[1][1] - (int64_t) a[0][1] * a[0][1];
    if (det == 0) {
        mat[2] = mat[5] = 0x10000;
        mat[3] = mat[4] = 0;
        dav2d_set_affine_mv2d(bw4, bh4, mv, wm, bx4, by4);
        return 0;
    }
    int shift, idet = apply_sign64(resolve_divisor_64(llabs(det), &shift), det);
    shift -= 16;
    if (shift < 0) {
        idet <<= -shift;
        shift = 0;
    }

    // solve the least-squares
    const int64_t r = (1LL << shift) >> 1;
    mat[2] = get_mult_shift_diag((int64_t) a[1][1] * bx[0] -
                                 (int64_t) a[0][1] * bx[1], idet, r, shift);
    mat[3] = get_mult_shift_ndiag((int64_t) a[0][0] * bx[1] -
                                  (int64_t) a[0][1] * bx[0], idet, r, shift);
    mat[4] = get_mult_shift_ndiag((int64_t) a[1][1] * by[0] -
                                  (int64_t) a[0][1] * by[1], idet, r, shift);
    mat[5] = get_mult_shift_diag((int64_t) a[0][0] * by[1] -
                                 (int64_t) a[0][1] * by[0], idet, r, shift);

    dav2d_set_affine_mv2d(bw4, bh4, mv, wm, bx4, by4);

    return 0;
}
