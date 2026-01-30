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

#ifndef DAV2D_SRC_LOOPRESTORATION_H
#define DAV2D_SRC_LOOPRESTORATION_H

#include <stdint.h>
#include <stddef.h>

#include "common/bitdepth.h"

enum LrEdgeFlags {
    LR_HAVE_LEFT = 1 << 0,
    LR_HAVE_RIGHT = 1 << 1,
    LR_HAVE_TOP = 1 << 2,
    LR_HAVE_BOTTOM = 1 << 3,
};

#ifdef BITDEPTH
typedef const pixel (*const_left_pixel_row)[4];
#else
typedef const void *const_left_pixel_row;
#endif

typedef union LooprestorationParams {
    ALIGN(int16_t filter[2][8], 16);
    struct {
        uint32_t s0, s1;
        int16_t w0, w1;
    } sgr;
} LooprestorationParams;

typedef union WienerParams {
    struct {
        const int8_t *filter;
    } single;
    struct {
        union {
            const int8_t (*user)[18];
            const int16_t (*pretrained)[13];
        } filters;
        const uint8_t *subclass_lut;
        const uint16_t (*noskip_mask)[12];
        int base_q;
    } multi;
} WienerParams;

// Although the spec applies restoration filters over 4x4 blocks,
// they can be applied to a bigger surface.
//    * w is constrained by the restoration unit size (w <= 256)
//    * h is constrained by the stripe height (h <= 64)
// The filter functions are allowed to do aligned writes past the right
// edge of the buffer, aligned up to the minimum loop restoration unit size
// (which is 32 pixels for subsampled chroma and 64 pixels for luma).
#define decl_lr_filter_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const_left_pixel_row left, \
            const pixel *lpf, int w, int h, \
            const LooprestorationParams *params, \
            enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
typedef decl_lr_filter_fn(*looprestorationfilter_fn);

#define decl_wiener_filter_fn(name) \
void (name)(pixel *dst, ptrdiff_t dst_stride, \
            const_left_pixel_row left, \
            const pixel *lpf, int w, int h, \
            const WienerParams *params, \
            enum LrEdgeFlags edges HIGHBD_DECL_SUFFIX)
typedef decl_wiener_filter_fn(*wienerfilter_fn);

typedef struct Dav2dLoopRestorationDSPContext {
    wienerfilter_fn ns_wiener_single;
    wienerfilter_fn ns_wiener_multi;
    wienerfilter_fn pc_wiener;

    looprestorationfilter_fn wiener[2]; /* 7-tap, 5-tap */
    looprestorationfilter_fn sgr[3]; /* 5x5, 3x3, mix */
} Dav2dLoopRestorationDSPContext;

bitfn_decls(void dav2d_loop_restoration_dsp_init, Dav2dLoopRestorationDSPContext *c, int bpc);

#endif /* DAV2D_SRC_LOOPRESTORATION_H */
