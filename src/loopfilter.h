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

#ifndef DAV2D_SRC_LOOPFILTER_H
#define DAV2D_SRC_LOOPFILTER_H

#include <stdint.h>
#include <stddef.h>

#include "common/bitdepth.h"

#include "src/levels.h"
#include "src/lf_mask.h"

// TODO: * Compute q_thr/side_thr from seg_ids in filter.
//       * Add a flag to shift down q_thr/side_thr for sub_pu_edge
#define decl_loopfilter_sb_fn(name) \
void (name)(pixel *dst, ptrdiff_t stride, const uint64_t *mask, \
            unsigned q_thr, unsigned side_thr, int edge, \
            const Av2FilterLUT *lut, int w HIGHBD_DECL_SUFFIX)
typedef decl_loopfilter_sb_fn(*loopfilter_sb_fn);

typedef struct Dav2dLoopFilterDSPContext {
    /*
     * dimension 1: plane (0=luma, 1=chroma)
     * dimension 2: 0=col-edge filter (h), 1=row-edge filter (v)
     *
     * dst/stride are aligned by 32
     */
    loopfilter_sb_fn loop_filter_sb[2][2];
} Dav2dLoopFilterDSPContext;

bitfn_decls(void dav2d_loop_filter_dsp_init, Dav2dLoopFilterDSPContext *c);

#endif /* DAV2D_SRC_LOOPFILTER_H */
