/*
 * Copyright © 2019-2026, VideoLAN and dav2d authors
 * Copyright © 2019-2026, Two Orioles, LLC
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

#ifndef DAV2D_SRC_X86_MSAC_H
#define DAV2D_SRC_X86_MSAC_H

#include "src/cpu.h"

unsigned dav2d_msac_decode_symbol_adapt4_sse2(MsacContext *s, uint16_t *cdf,
                                              size_t n_symbols);
unsigned dav2d_msac_decode_symbol_adapt8_sse2(MsacContext *s, uint16_t *cdf,
                                              size_t n_symbols);
unsigned dav2d_msac_decode_bool_adapt_sse2(MsacContext *s, uint16_t *cdf);
unsigned dav2d_msac_decode_bool_bypass_sse2(MsacContext *s);
unsigned dav2d_msac_decode_bools_bypass_sse2(MsacContext *s, unsigned n_bits);
unsigned dav2d_msac_decode_unary_bypass6_avx2(MsacContext *s, unsigned max_bits);
unsigned dav2d_msac_decode_unary_bypass21_avx2(MsacContext *s);
unsigned dav2d_msac_decode_unary_bypass21_avx512icl(MsacContext *s);

#define dav2d_msac_decode_symbol_adapt4 dav2d_msac_decode_symbol_adapt4_sse2
#define dav2d_msac_decode_symbol_adapt8 dav2d_msac_decode_symbol_adapt8_sse2
#define dav2d_msac_decode_bool_adapt    dav2d_msac_decode_bool_adapt_sse2
#define dav2d_msac_decode_bool_bypass   dav2d_msac_decode_bool_bypass_sse2
#define dav2d_msac_decode_bools_bypass  dav2d_msac_decode_bools_bypass_sse2

#define dav2d_msac_decode_unary_bypass6(s, n) ((s)->unary_bypass6(s, n))
#define dav2d_msac_decode_unary_bypass21(s) ((s)->unary_bypass21(s))

static ALWAYS_INLINE void msac_dsp_init_x86(MsacContext *const s) {
    const unsigned flags = dav2d_get_cpu_flags();

    if (!(flags & DAV2D_X86_CPU_FLAG_AVX2)) return;

    s->unary_bypass6  = dav2d_msac_decode_unary_bypass6_avx2;
    s->unary_bypass21 = dav2d_msac_decode_unary_bypass21_avx2;

    if (!(flags & DAV2D_X86_CPU_FLAG_AVX512ICL)) return;

    s->unary_bypass21 = dav2d_msac_decode_unary_bypass21_avx512icl;
}

#endif /* DAV2D_SRC_X86_MSAC_H */
