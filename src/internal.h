/*
 * Copyright © 2025, Niklas Haas
 * Copyright © 2018, VideoLAN and dav1d authors
 * Copyright © 2018, Two Orioles, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CHECKASM_INTERNAL_H
#define CHECKASM_INTERNAL_H

#include <stdint.h>
#include <stdio.h>

#include "checkasm/attributes.h"
#include "checkasm/test.h"

#ifdef __GNUC__
  #define COLD __attribute__((cold))
#else
  #define COLD
#endif

#ifndef __has_attribute
  #define __has_attribute(x) 0
#endif

#ifdef _MSC_VER
  #define NOINLINE __declspec(noinline)
#elif __has_attribute(noclone)
  #define NOINLINE __attribute__((noinline, noclone))
#else
  #define NOINLINE __attribute__((noinline))
#endif

#ifdef _MSC_VER
  #define ALWAYS_INLINE __forceinline
#else
  #define ALWAYS_INLINE inline __attribute__((always_inline))
#endif

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

void checkasm_srand(unsigned seed);

/* Hidden alias without public visibility, for use in asm */
int checkasm_fail_internal(const char *msg, ...) ATTR_FORMAT_PRINTF(1, 2);

#define COLOR_DEFAULT -1
#define COLOR_RED     31
#define COLOR_GREEN   32
#define COLOR_YELLOW  33
#define COLOR_BLUE    34
#define COLOR_MAGENTA 35
#define COLOR_CYAN    36
#define COLOR_WHITE   37

/* Colored variant of fprintf for terminals that support it */
void checkasm_setup_fprintf(FILE *const f);
void checkasm_fprintf(FILE *const f, const int color, const char *const fmt, ...)
    ATTR_FORMAT_PRINTF(3, 4);

/* Platform specific signal handling */
void checkasm_set_signal_handlers(void);

typedef struct CheckasmVar {
    double lmean, lvar; /* log mean and variance */
} CheckasmVar;

/* Platform specific timing code */
int checkasm_perf_init(void);
int checkasm_perf_init_linux(CheckasmPerf *perf);
int checkasm_perf_init_macos(CheckasmPerf *perf);

CheckasmVar checkasm_measure_nop_cycles(void); /* cycles per iter */
CheckasmVar checkasm_measure_perf_scale(void); /* ns per cycle */
uint64_t    checkasm_gettime_nsec(void);
uint64_t    checkasm_gettime_nsec_diff(uint64_t t); /* subtracts t */
void        checkasm_noop(void *);

/* Miscellaneous helpers */
static inline int imax(const int a, const int b)
{
    return a > b ? a : b;
}

static inline int imin(const int a, const int b)
{
    return a < b ? a : b;
}

#endif /* CHECKASM_INTERNAL_H */
