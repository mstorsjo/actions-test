/*
 * Copyright © 2023, VideoLAN and dav2d authors
 * Copyright © 2023, Loongson Technology Corporation Limited
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

#ifndef DAV2D_SRC_LOONGARCH_REFMVS_H
#define DAV2D_SRC_LOONGARCH_REFMVS_H

#include "src/cpu.h"
#include "src/refmvs.h"

decl_splat_mv_fn(dav2d_splat_mv_lsx);
decl_load_tmvs_fn(dav2d_load_tmvs_lsx);
decl_save_tmvs_fn(dav2d_save_tmvs_lsx);

static ALWAYS_INLINE void refmvs_dsp_init_loongarch(Dav2dRefmvsDSPContext *const c) {
    const unsigned flags = dav2d_get_cpu_flags();

    if (!(flags & DAV2D_LOONGARCH_CPU_FLAG_LSX)) return;

    c->splat_mv = dav2d_splat_mv_lsx;
    c->load_tmvs = dav2d_load_tmvs_lsx;
    c->save_tmvs = dav2d_save_tmvs_lsx;
}

#endif /* DAV2D_SRC_LOONGARCH_REFMVS_H */
