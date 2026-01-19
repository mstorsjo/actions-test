/*
 * Copyright © 2018-2021, VideoLAN and dav1d authors
 * Copyright © 2018-2021, Two Orioles, LLC
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

#include "src/cpu.h"
#include "src/mc.h"

#define decl_fn(type, name) \
    decl_##type##_fn(BF(name, ssse3)); \
    decl_##type##_fn(BF(name, avx2)); \
    decl_##type##_fn(BF(name, avx512icl));
#define init_mc_fn(type, name, suffix) \
    c->mc[type] = BF(dav1d_put_##name, suffix)
#define init_mct_fn(type, name, suffix) \
    c->mct[type] = BF(dav1d_prep_##name, suffix)
#define init_mc_scaled_fn(type, name, suffix) \
    c->mc_scaled[type] = BF(dav1d_put_##name, suffix)
#define init_mct_scaled_fn(type, name, suffix) \
    c->mct_scaled[type] = BF(dav1d_prep_##name, suffix)

decl_8tap_fns(ssse3);
decl_8tap_fns(avx2);
decl_8tap_fns(avx512icl);

decl_fn(mc, dav1d_put_bilin);
decl_fn(mct, dav1d_prep_bilin);

static ALWAYS_INLINE void mc_dsp_init_x86(Dav1dMCDSPContext *const c) {
    const unsigned flags = dav1d_get_cpu_flags();

    if (!(flags & DAV1D_X86_CPU_FLAG_AVX2))
        return;

    init_8tap_fns(avx2);

    init_mc_fn(DAV1D_FILTER_BILINEAR,  bilin, avx2);
    init_mct_fn(DAV1D_FILTER_BILINEAR, bilin, avx2);
}
