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

#ifndef DAV2D_SRC_CPU_H
#define DAV2D_SRC_CPU_H

#include "config.h"

#include "common/attributes.h"

#include "dav2d/common.h"
#include "dav2d/dav2d.h"

#if ARCH_AARCH64 || ARCH_ARM
#include "src/arm/cpu.h"
#elif ARCH_LOONGARCH
#include "src/loongarch/cpu.h"
#elif ARCH_PPC64LE
#include "src/ppc/cpu.h"
#elif ARCH_RISCV
#include "src/riscv/cpu.h"
#elif ARCH_X86
#include "src/x86/cpu.h"
#endif

EXTERN unsigned dav2d_cpu_flags;
EXTERN unsigned dav2d_cpu_flags_mask;

void dav2d_init_cpu(void);
DAV2D_API void dav2d_set_cpu_flags_mask(unsigned mask);
int dav2d_num_logical_processors(Dav2dContext *c);
unsigned long dav2d_getauxval(unsigned long);

static ALWAYS_INLINE unsigned dav2d_get_default_cpu_flags(void) {
    unsigned flags = 0;

#if ARCH_AARCH64 || ARCH_ARM
#if defined(__ARM_NEON) || defined(__APPLE__) || defined(_WIN32) || ARCH_AARCH64
    flags |= DAV2D_ARM_CPU_FLAG_NEON;
#endif
#ifdef __ARM_FEATURE_DOTPROD
    flags |= DAV2D_ARM_CPU_FLAG_DOTPROD;
#endif
#ifdef __ARM_FEATURE_MATMUL_INT8
    flags |= DAV2D_ARM_CPU_FLAG_I8MM;
#endif
#if ARCH_AARCH64
#ifdef __ARM_FEATURE_SVE
    flags |= DAV2D_ARM_CPU_FLAG_SVE;
#endif
#ifdef __ARM_FEATURE_SVE2
    flags |= DAV2D_ARM_CPU_FLAG_SVE2;
#endif
#endif /* ARCH_AARCH64 */
#elif ARCH_PPC64LE
#if defined(__VSX__)
    flags |= DAV2D_PPC_CPU_FLAG_VSX;
#endif
#if defined(__POWER9_VECTOR__)
    flags |= DAV2D_PPC_CPU_FLAG_PWR9;
#endif
#elif ARCH_RISCV
#if defined(__riscv_v)
    flags |= DAV2D_RISCV_CPU_FLAG_V;
#endif
#elif ARCH_X86
#if defined(__AVX512F__) && defined(__AVX512CD__) && \
    defined(__AVX512BW__) && defined(__AVX512DQ__) && \
    defined(__AVX512VL__) && defined(__AVX512VNNI__) && \
    defined(__AVX512IFMA__) && defined(__AVX512VBMI__) && \
    defined(__AVX512VBMI2__) && defined(__AVX512VPOPCNTDQ__) && \
    defined(__AVX512BITALG__) && defined(__GFNI__) && \
    defined(__VAES__) && defined(__VPCLMULQDQ__)
    flags |= DAV2D_X86_CPU_FLAG_AVX512ICL |
             DAV2D_X86_CPU_FLAG_AVX2 |
             DAV2D_X86_CPU_FLAG_SSE41 |
             DAV2D_X86_CPU_FLAG_SSSE3 |
             DAV2D_X86_CPU_FLAG_SSE2;
#elif defined(__AVX2__)
    flags |= DAV2D_X86_CPU_FLAG_AVX2 |
             DAV2D_X86_CPU_FLAG_SSE41 |
             DAV2D_X86_CPU_FLAG_SSSE3 |
             DAV2D_X86_CPU_FLAG_SSE2;
#elif defined(__SSE4_1__) || defined(__AVX__)
    flags |= DAV2D_X86_CPU_FLAG_SSE41 |
             DAV2D_X86_CPU_FLAG_SSSE3 |
             DAV2D_X86_CPU_FLAG_SSE2;
#elif defined(__SSSE3__)
    flags |= DAV2D_X86_CPU_FLAG_SSSE3 |
             DAV2D_X86_CPU_FLAG_SSE2;
#elif ARCH_X86_64 || defined(__SSE2__) || \
      (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
    flags |= DAV2D_X86_CPU_FLAG_SSE2;
#endif
#endif

    return flags;
}

static ALWAYS_INLINE unsigned dav2d_get_cpu_flags(void) {
    unsigned flags = dav2d_cpu_flags & dav2d_cpu_flags_mask;

#if TRIM_DSP_FUNCTIONS
/* Since this function is inlined, unconditionally setting a flag here will
 * enable dead code elimination in the calling function. */
    flags |= dav2d_get_default_cpu_flags();
#endif

    return flags;
}

#endif /* DAV2D_SRC_CPU_H */
