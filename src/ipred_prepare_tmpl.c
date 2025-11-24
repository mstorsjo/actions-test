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

#include <stdint.h>
#include <string.h>

#include "common/dump.h"
#include "common/intops.h"

#include "src/debug.h"
#include "src/ipred_prepare.h"

static const uint8_t mode_conv[N_INTRA_PRED_MODES]
                              [2 /* have_left */][2 /* have_top */] =
{
    [DC_PRED]    = { { DC_128_PRED,  TOP_DC_PRED },
                     { LEFT_DC_PRED, DC_PRED     } },
    [PAETH_PRED] = { { DC_128_PRED,  VERT_PRED   },
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
    [DC_PRED]       = { .needs_top  = 1, .needs_left = 1, .needs_topleft = 1 },
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
    [SMOOTH_PRED]   = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1,
                        .needs_topright = 1, .needs_bottomleft= 1 },
    [SMOOTH_V_PRED] = { .needs_left = 1, .needs_top = 1, .needs_bottomleft= 1 },
    [SMOOTH_H_PRED] = { .needs_left = 1, .needs_top = 1, .needs_topright = 1 },
    [PAETH_PRED]    = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1 },
    [DIP_PRED]      = { .needs_left = 1, .needs_top = 1, .needs_topleft = 1,
                        .needs_topright = 1, .needs_bottomleft= 1 }
};

enum IntraPredMode
bytefn(dav1d_prepare_intra_edges)(DB_ONLY(const int print_dbg)
                                  const int x, const int y,
                                  const int w, const int h,
                                  const int n_tr, const int n_bl,
                                  const pixel *const dst,
                                  const ptrdiff_t stride,
                                  const pixel *prefilter_toplevel_sb_edge,
                                  enum IntraPredMode mode, int *const angle,
                                  const int tw4, const int th4,
                                  const int intra_flags,
                                  pixel *const topleft_out HIGHBD_DECL_SUFFIX)
{
    const int bitdepth = bitdepth_from_max(bitdepth_max);
    assert(y < h && x < w);
    int is_dir = 0;
    const int enable_edge_filter = !!(intra_flags & ANGLE_USE_EDGE_FILTER_FLAG);
    const int apply_dip = !!(intra_flags & ANGLE_DIP_FLAG);
    const int apply_ibp = !!(intra_flags & ANGLE_IBP_FLAG);
    const int mrl_idx =
        (intra_flags & ANGLE_MRL_IDX_MASK) >> ANGLE_MRL_IDX_SHIFT;
    const int mrl_mul = !!(intra_flags & ANGLE_MULTI_MRL_FLAG);
    const int have_left = !!(intra_flags & ANGLE_HAS_LEFT_FLAG);
    const int have_top = !!(intra_flags & ANGLE_HAS_TOP_FLAG);

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
        if (*angle <= 90)
            mode = *angle < 90 && (have_top || apply_ibp) ? Z1_PRED : VERT_PRED;
        else if (*angle < 180)
            mode = Z2_PRED;
        else
            mode = *angle > 180 && (have_left || apply_ibp) ? Z3_PRED : HOR_PRED;
        break;
    }
    case DC_PRED:
        mode = apply_dip ? DIP_PRED : mode_conv[mode][have_left][have_top];
        break;
    default:
        break;
    }

    // FIXME SMOOTH predictors don't need all of the edge pixels
    EdgeMask e = intra_prediction_edges[mode];
    if (is_dir && apply_ibp) {
        e.needs_top = 1;
        e.needs_left = 1;
        e.needs_topleft = 1;
        e.needs_bottomleft |= *angle < 90;
    }

    const pixel *dst_top, *dst_top2;
    if (have_top &&
        (e.needs_top || e.needs_topleft || (e.needs_left && !have_left)))
    {
        if (prefilter_toplevel_sb_edge) {
            dst_top = dst_top2 = &prefilter_toplevel_sb_edge[x * 4];
        } else {
            dst_top = &dst[-((mrl_idx + 1) * PXSTRIDE(stride))];
            dst_top2 = &dst[-PXSTRIDE(stride)];
        }
    }

    const int tw = tw4 << 2, th = th4 << 2;
    // Safe maximum size for edge buffers
    const int e_stride = (tw + th + (mrl_idx << 1) + 3) * 2;
    if (e.needs_left) {
        const int sz = th + (apply_dip ? (th >> 2) :
            ((e.needs_bottomleft ? tw : 3) + (mrl_idx << 1)));
        pixel *const left = &topleft_out[-(mrl_idx + sz)];
        pixel *const left2 = &topleft_out[-(sz + e_stride)];

        if (have_left) {
            int px_have = imin(th, (h - y) << 2);
            int i;
            for (i = 0; i < px_have; i++)
                left[sz - 1 - i] = dst[PXSTRIDE(stride) * i - 1 - mrl_idx];
            if (e.needs_bottomleft) {
                px_have += n_bl << 2;
                for (; i < px_have; i++)
                    left[sz - 1 - i] = dst[PXSTRIDE(stride) * i - 1 - mrl_idx];
            }
            if (px_have < sz)
                pixel_set(left, left[sz - px_have], sz - px_have);
            if (mrl_mul) {
                for (int i = 0; i < px_have; i++)
                    left2[sz - 1 - i] = dst[PXSTRIDE(stride) * i - 1];
                if (px_have < sz)
                    pixel_set(left2, left2[sz - px_have], sz - px_have);
            }
        } else {
            pixel_set(left, have_top ? *dst_top : ((1 << bitdepth) >> 1) + 1, sz);
        }

#if 0
        if (e.needs_bottomleft) {
            const int have_bottomleft = (!have_left || y + th >= h) ? 0 :
                                        (edge_flags & EDGE_I444_LEFT_HAS_BOTTOM);

            if (have_bottomleft) {
                const int px_have = imin(sz, (h - y - th) << 2);

                for (int i = 0; i < px_have; i++)
                    left[-(i + 1)] = dst[(sz + i) * PXSTRIDE(stride) - 1];
                if (px_have < sz)
                    pixel_set(left - sz, left[-px_have], sz - px_have);
            } else {
                pixel_set(left - sz, left[0], sz);
            }
        }
#endif
#if DEBUG_BLOCK_INFO
        if (print_dbg) {
            hex_dump(left, sz, sz, 1, "l");
            if (mrl_mul) {
                hex_dump(left2, sz, sz, 1, "l2");
            }
        }
#endif
    }

    if (e.needs_top) {
        if (is_dir) {
            e.needs_topright = apply_ibp ? *angle < 90 || *angle > 180 : *angle < 90;
        }
        const int sz = tw + (apply_dip ? (tw >> 2) :
            ((e.needs_topright ? th : 0) + (mrl_idx << 1)));
        pixel *const top = &topleft_out[mrl_idx + 1];
        pixel *const top2 = &topleft_out[1 - e_stride];

        if (have_top) {
            int px_have = imin(tw, (w - x) << 2);
            pixel_copy(top, dst_top, px_have);
            if (e.needs_topright && n_tr) {
                px_have += n_tr << 2;
                pixel_copy(top + tw, dst_top + tw, n_tr << 2);
            }
            if (px_have < sz)
                pixel_set(top + px_have, top[px_have - 1], sz - px_have);
            if (mrl_mul) {
                pixel_copy(top2, dst_top2, px_have);
                if (px_have < sz)
                    pixel_set(top2 + px_have, top2[px_have - 1], sz - px_have);
            }
        } else {
            pixel_set(top, have_left ? dst[-1] : ((1 << bitdepth) >> 1) - 1, sz);
        }

#if 0
        if (e.needs_topright) {
            const int have_topright = (!have_top || x + tw >= w) ? 0 :
                                      (edge_flags & EDGE_I444_TOP_HAS_RIGHT);

            if (have_topright) {
                const int px_have = imin(sz, (w - x - tw) << 2);

                pixel_copy(top + sz, &dst_top[sz], px_have);
                if (px_have < sz)
                    pixel_set(top + sz + px_have, top[sz + px_have - 1],
                              sz - px_have);
            } else {
                pixel_set(top + sz, top[sz - 1], sz);
            }
        }
#endif
#if DEBUG_BLOCK_INFO
        if (print_dbg) {
            hex_dump(top, sz, sz, 1, "t");
            if (mrl_mul) {
                hex_dump(top2, sz, sz, 1, "t2");
            }
        }
#endif
    }

    if (e.needs_topleft) {
        if (have_top && have_left) {
            for (int i = -mrl_idx; i < 0; i++)
                topleft_out[i] = dst_top[-(mrl_idx + 1) + (-i) * PXSTRIDE(stride)];
            for (int i = 0; i <= mrl_idx; i++)
                topleft_out[i] = dst_top[-(mrl_idx + 1 - i)];
        } else {
            int v;
            if (have_left)
                v = have_top ? dst_top[-1] : dst[-1];
            else
                v = have_top ? *dst_top : (1 << bitdepth) >> 1;
            pixel_set(&topleft_out[-mrl_idx], v, 2 * mrl_idx + 1);
        }
        topleft_out[-e_stride] =
            have_left ? have_top ? dst_top2[-1] : dst[-1] :
                        have_top ? *dst_top2 : (1 << bitdepth) >> 1;

#if DEBUG_BLOCK_INFO
        if (print_dbg) {
            hex_dump(&topleft_out[-mrl_idx], 0, 2 * mrl_idx + 1, 1, "tl");
            if (mrl_mul) {
                hex_dump(&topleft_out[-e_stride], 0, 1, 1, "tl2");
            }
        }
#endif

        if (is_dir && *angle != 90 && *angle != 180 && e.needs_top &&
            e.needs_left && !mrl_idx && enable_edge_filter && tw + th >= 24)
        {
            const int c = topleft_out[0] +
                (topleft_out[-1] + topleft_out[0] + topleft_out[1]) * 5;
            topleft_out[0] = (c + 8) >> 4;
        }
    }

    if (apply_ibp) {
        const int max_base = tw + th;
        if (*angle <= 90) {
            topleft_out[-(max_base + 1)] = topleft_out[-max_base];
            topleft_out[-(max_base + 2)] = topleft_out[-max_base];
        } else if (*angle > 180) {
            topleft_out[max_base + 1] = topleft_out[max_base];
            topleft_out[max_base + 2] = topleft_out[max_base];
        }
    }

    return mode;
}
