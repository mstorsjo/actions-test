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
#include <string.h>

#include "common/dump.h"
#include "common/intops.h"

#include "src/debug.h"
#include "src/ipred_prepare.h"

static const uint8_t mode_conv[2 /* is_paeth */][2 /* have_left */][2 /* have_top */] =
{
    [0 /*DC_PRED*/]    = { { DC_128_PRED,  TOP_DC_PRED },
                           { LEFT_DC_PRED, DC_PRED     } },
    [1 /*PAETH_PRED*/] = { { DC_128_PRED,  VERT_PRED   },
                           { HOR_PRED,     PAETH_PRED  } },
};

typedef struct EdgeMask {
    uint8_t needs_left:1;
    uint8_t needs_top:1;
    uint8_t needs_topleft:1;
    uint8_t needs_topright:1;
    uint8_t needs_bottomleft:1;
} EdgeMask;

static const EdgeMask intra_prediction_edges[N_IMPL_INTRA_PRED_MODES] = {
    [DC_PRED]       = { .needs_top  = 1, .needs_left = 1 },
    [VERT_PRED]     = { .needs_top  = 1 },
    [HOR_PRED]      = { .needs_left = 1 },
    [LEFT_DC_PRED]  = { .needs_left = 1 },
    [TOP_DC_PRED]   = { .needs_top  = 1 },
    [DC_128_PRED]   = { 0 },
    [Z1_PRED]       = { .needs_top = 1, .needs_topright = 1,
                        .needs_topleft = 1 },
    [Z2_PRED]       = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1 },
    [Z3_PRED]       = { .needs_left = 1, .needs_bottomleft = 1,
                        .needs_topleft = 1 },
    [SMOOTH_PRED]   = { .needs_left = 1, .needs_top = 1,
                        .needs_topright = 1, .needs_bottomleft = 1 },
    [SMOOTH_V_PRED] = { .needs_top = 1, .needs_bottomleft = 1 },
    [SMOOTH_H_PRED] = { .needs_left = 1, .needs_topright = 1 },
    [PAETH_PRED]    = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1 },
    [DIP_PRED]      = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1,
                        .needs_topright = 1, .needs_bottomleft= 1 }
};

enum IntraPredMode
bytefn(dav2d_prepare_intra_edges)(DB_ONLY(const int print_dbg)
                                  const int x, const int y,
                                  const int w, const int h,
                                  const int n_tr, const int n_bl,
                                  const pixel *const dst,
                                  const ptrdiff_t stride,
                                  const pixel *prefilter_toplevel_sb_edge,
                                  enum IntraPredMode mode,
                                  const int tw4, const int th4,
                                  const int intra_flags,
                                  pixel *const topleft_out HIGHBD_DECL_SUFFIX)
{
    const int bitdepth = bitdepth_from_max(bitdepth_max);
    assert(y < h && x < w);
    int is_dir = 0;
    const int enable_edge_filter = !!(intra_flags & ANGLE_USE_EDGE_FILTER_FLAG);
    const int angle = intra_flags & 511;
    const int apply_dip = !!(intra_flags & ANGLE_DIP_FLAG);
    const int apply_ibp = !!(intra_flags & ANGLE_IBP_FLAG);
    const int mrl_idx =
        (intra_flags & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int mrl_mul = !!(intra_flags & ANGLE_MULTI_MRL_FLAG);
    const int have_left = !!(intra_flags & ANGLE_HAS_LEFT_FLAG);
    const int have_top = !!(intra_flags & ANGLE_HAS_TOP_FLAG);

    int tl_filter = 0;
    switch (mode) {
    case VERT_PRED:
    case HOR_PRED:
    case DIAG_DOWN_LEFT_PRED:
    case DIAG_DOWN_RIGHT_PRED:
    case VERT_RIGHT_PRED:
    case HOR_DOWN_PRED:
    case HOR_UP_PRED:
    case VERT_LEFT_PRED: {
        is_dir = 1;
        if (angle <= 90)
            mode = angle < 90 && (have_top || apply_ibp) ? Z1_PRED : VERT_PRED;
        else if (angle < 180)
            mode = Z2_PRED;
        else
            mode = angle > 180 && (have_left || apply_ibp) ? Z3_PRED : HOR_PRED;
        tl_filter = (unsigned) mode - Z1_PRED <= 2U && have_left && have_top &&
                    !mrl_idx && enable_edge_filter && tw4 + th4 >= 6;
        break;
    }
    case DC_PRED:
        mode = apply_dip ? DIP_PRED : mode_conv[0][have_left][have_top];
        break;
    case PAETH_PRED:
        assert(!apply_dip);
        mode = mode_conv[1][have_left][have_top];
        break;
    default:
        break;
    }
    assert(!mrl_idx || is_dir);

    EdgeMask e = intra_prediction_edges[mode];
    if ((mode == Z1_PRED || mode == Z3_PRED) && apply_ibp)
        e = intra_prediction_edges[DIP_PRED]; // all edges

    const pixel *dst_top, *dst_top2;
    ptrdiff_t top_stride;
    if (have_top && ((e.needs_top | e.needs_topleft | e.needs_topright) ||
                     ((e.needs_left | e.needs_bottomleft) && !have_left)))
    {
        if (prefilter_toplevel_sb_edge) {
            dst_top = dst_top2 = &prefilter_toplevel_sb_edge[x * 4];
            top_stride = 0;
        } else {
            dst_top = &dst[-((mrl_idx + 1) * PXSTRIDE(stride))];
            dst_top2 = &dst[-PXSTRIDE(stride)];
            top_stride = stride;
        }
    }

    const int tw = tw4 << 2, th = th4 << 2;
    // in case of multi-mrl: ptr1 will point to the mrl_idx line, which
    // contains one topleft pixel, mrl_idx pixels between top/left and
    // top-most left pixel, then width pixels above, followed by height
    // pixels and 2*mrl_idx pixels top/right. Then the left edge of the
    // adjacent (non-mrl_idx) line, which contains width+height pixels.
    const int diag_mrl_idx = (unsigned) mode - Z1_PRED <= 2U ? mrl_idx : 0;
    const int e_stride = (tw + th) * 2 + diag_mrl_idx * 3 + 1;
    if (e.needs_left || tl_filter) {
        int sz = e.needs_left ? th : 1, sz2 = th;
        if (e.needs_bottomleft) {
            sz += apply_dip ? th >> 2 : is_dir ? tw + 2 * diag_mrl_idx : 1 /* smooth */;
            sz2 = sz - 2 * diag_mrl_idx;
        }
        pixel *const left = &topleft_out[-(diag_mrl_idx + 1)];
        pixel *const left2 = &topleft_out[e_stride - 1];

        if (have_left) {
            int px_have = e.needs_left ? imin(th, (h - y) << 2) : 1;
            int i;
            for (i = 0; i < px_have; i++)
                left[-i] = dst[PXSTRIDE(stride) * i - 1 - mrl_idx];
            if (e.needs_bottomleft && n_bl > 0) {
                px_have += imin(n_bl << 2, sz - th);
                for (; i < px_have; i++)
                    left[-i] = dst[PXSTRIDE(stride) * i - 1 - mrl_idx];
            }
            if (px_have < sz)
                pixel_set(&left[1 - sz], left[1 - i], sz - px_have);
            if (mrl_mul) {
                px_have = imin(px_have, sz2);
                for (int i = 0; i < px_have; i++)
                    left2[-i] = dst[PXSTRIDE(stride) * i - 1];
                if (px_have < sz2)
                    pixel_set(&left2[1 - sz2], left2[1 - i], sz2 - px_have);
            }
        } else {
            pixel_set(&left[1 - sz], have_top ? *dst_top : ((1 << bitdepth) >> 1) + 1, sz);
            if (mrl_mul)
                pixel_set(&left2[1 - sz2], have_top ? *dst_top2 :
                                 ((1 << bitdepth) >> 1) + 1, sz2);
        }

#if DEBUG_BLOCK_INFO
        if (print_dbg) {
            hex_dump(&left[1 - sz], 0, sz, 1, "l");
            if (mrl_mul)
                hex_dump(&left2[1 - sz2], 0, sz2, 1, "l2");
        }
#endif
    } else if (e.needs_bottomleft) {
        assert(mode == SMOOTH_V_PRED);
        pixel *const bottom_left = &topleft_out[-(1 + th)];

        if (!have_left) {
            *bottom_left = have_top ? *dst_top : ((1 << bitdepth) >> 1) + 1;
        } else if (n_bl <= 0) {
            *bottom_left = dst[PXSTRIDE(stride) * (imin(th, (h - y) << 2) - 1) - 1];
        } else {
            *bottom_left = dst[PXSTRIDE(stride) * th - 1];
        }

#if DEBUG_BLOCK_INFO
        if (print_dbg)
            hex_dump(bottom_left, 0, 1, 1, "bl");
#endif
    }

    if (e.needs_top || tl_filter) {
        int sz = e.needs_top ? tw : 1, sz2 = tw;
        if (e.needs_topright) {
            sz += apply_dip ? tw >> 2 : is_dir ? th + 2 * diag_mrl_idx : 1 /* smooth */;
            sz2 = sz - 2 * diag_mrl_idx;
        }
        pixel *const top = &topleft_out[diag_mrl_idx + 1];
        pixel *const top2 = &topleft_out[e_stride + 1];

        if (have_top) {
            int px_have = e.needs_top ? imin(tw, (w - x) << 2) : 1;
            pixel_copy(top, dst_top, px_have);
            if (e.needs_topright && n_tr > 0) {
                px_have += imin(n_tr << 2, sz - tw);
                pixel_copy(top + tw, dst_top + tw, px_have - tw);
            }
            if (px_have < sz)
                pixel_set(top + px_have, top[px_have - 1], sz - px_have);
            if (mrl_mul) {
                px_have = imin(px_have, sz2);
                pixel_copy(top2, dst_top2, px_have);
                if (px_have < sz2)
                    pixel_set(top2 + px_have, top2[px_have - 1], sz2 - px_have);
            }
        } else {
            pixel_set(top, have_left ? dst[-(1 + mrl_idx)] :
                      ((1 << bitdepth) >> 1) - 1, sz);
            if (mrl_mul)
                pixel_set(top2, have_left ? dst[-1] :
                          ((1 << bitdepth) >> 1) - 1, sz2);
        }

#if DEBUG_BLOCK_INFO
        if (print_dbg) {
            hex_dump(top, 0, sz, 1, "t");
            if (mrl_mul)
                hex_dump(top2, 0, sz2, 1, "t2");
        }
#endif
    } else if (e.needs_topright) {
        assert(mode == SMOOTH_H_PRED);
        pixel *const top_right = &topleft_out[1 + tw];

        if (!have_top) {
            *top_right = have_left ? dst[-1] : ((1 << bitdepth) >> 1) - 1;
        } else if (n_tr <= 0) {
            *top_right = dst_top[imin(tw, (w - x) << 2) - 1];
        } else {
            *top_right = dst_top[tw];
        }

#if DEBUG_BLOCK_INFO
        if (print_dbg)
            hex_dump(top_right, 0, 1, 1, "tr");
#endif
    }

    if (e.needs_topleft) {
        assert(diag_mrl_idx == mrl_idx);
        if (have_top && have_left) {
            for (int i = -mrl_idx; i < 0; i++)
                topleft_out[i] = dst_top[-(mrl_idx + 1) + (-i) *
                                          PXSTRIDE(top_stride)];
            for (int i = 0; i <= mrl_idx; i++)
                topleft_out[i] = dst_top[-(mrl_idx + 1 - i)];
        } else {
            int v;
            if (have_left)
                v = dst[-(1 + mrl_idx)];
            else
                v = have_top ? *dst_top : (1 << bitdepth) >> 1;
            pixel_set(&topleft_out[-mrl_idx], v, 2 * mrl_idx + 1);
        }
        topleft_out[e_stride] =
            have_left ? have_top ? dst_top2[-1] : dst[-1] :
                        have_top ? *dst_top2 : (1 << bitdepth) >> 1;

#if DEBUG_BLOCK_INFO
        if (print_dbg) {
            hex_dump(&topleft_out[-mrl_idx], 0, 2 * mrl_idx + 1, 1, "tl");
            if (mrl_mul)
                hex_dump(&topleft_out[e_stride], 0, 1, 1, "tl2");
        }
#endif

        if (tl_filter) {
            const int c = topleft_out[0] +
                (topleft_out[-1] + topleft_out[0] + topleft_out[1]) * 5;
            topleft_out[0] = (c + 8) >> 4;
        }
    }

    return mode;
}
