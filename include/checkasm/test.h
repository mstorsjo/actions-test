/*
 * Copyright © 2025, Niklas Haas
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

#ifndef CHECKASM_TEST_H
#define CHECKASM_TEST_H

#include <stdint.h>

#include "checkasm/attributes.h"
#include "checkasm/longjmp.h"
#include "checkasm/perf.h"
#include "checkasm/platform.h"

/********************************************
 * Internal checkasm API. Used inside tests *
 ********************************************/

CHECKASM_API void *checkasm_check_func(void *func, const char *name, ...)
    CHECKASM_PRINTF(2, 3);
CHECKASM_API int  checkasm_fail_func(const char *msg, ...) CHECKASM_PRINTF(1, 2);
CHECKASM_API void checkasm_report(const char *name, ...) CHECKASM_PRINTF(1, 2);
CHECKASM_API void checkasm_set_signal_handler_state(int enabled);
CHECKASM_API void checkasm_handle_signal(void);
CHECKASM_API checkasm_jmp_buf *checkasm_get_context(void);

/* Mark a block of tests as expected to fail. If this is set, all tested
 * functions must must be marked as failed, otherwise the whole test will
 * be marked as failed. This state does not persist between tests.
 *
 * Returns a nonzero value if tests failure tests are expected to work,
 * zero if such tests should be skipped (if signal handling isn't
 * supported). */
CHECKASM_API int checkasm_should_fail(int);

/* Decide whether or not the specified function needs to be tested */
#define check_func(func, ...)                                                            \
    (func_ref = (func_type *) checkasm_check_func((func_new = func), __VA_ARGS__))

/* Declare the function prototype. The first argument is the return value,
 * the remaining arguments are the function parameters. Naming parameters
 * is optional. */
#define declare_func(ret, ...)                                                           \
    declare_new(ret, __VA_ARGS__);                                                       \
    typedef ret func_type(__VA_ARGS__);                                                  \
    func_type  *func_ref, *func_new;                                                     \
    if (checkasm_save_context(checkasm_get_context()))                                   \
        checkasm_handle_signal();

#ifndef declare_func_emms
  #define declare_func_emms(cpu_flags, ret, ...) declare_func(ret, __VA_ARGS__)
#endif

/* Indicate that the current test has failed */
#define fail() checkasm_fail_func("%s:%d", __FILE__, __LINE__)

/* Print the test outcome */
#define report checkasm_report

/* Call the reference function */
#define call_ref(...)                                                                    \
    (checkasm_set_signal_handler_state(1), func_ref(__VA_ARGS__));                       \
    checkasm_set_signal_handler_state(0)

/* Verifies that clobbered callee-saved registers
 * are properly saved and restored */
CHECKASM_API void checkasm_checked_call(void *func, ...);

/* Clears any processor state possible left over after running a function */
#ifndef checkasm_clear_cpu_state
  #define checkasm_clear_cpu_state()                                                     \
      do {                                                                               \
      } while (0)
#endif

typedef struct CheckasmPerf {
    /* Start/stop callbacks, used whenever not using inline ASM */
    uint64_t (*start)(void);
    uint64_t (*stop)(uint64_t);
    const char *name, *unit;

#ifdef CHECKASM_PERF_ASM
    int asm_usable; /* Whether the ASM instruction is usable */
#endif
} CheckasmPerf;

CHECKASM_API const CheckasmPerf *checkasm_get_perf(void);

#define CHECKASM_PERF_CALL4(...)                                                         \
    do {                                                                                 \
        int tidx = 0;                                                                    \
        func_new(__VA_ARGS__);                                                           \
        tidx = 1;                                                                        \
        func_new(__VA_ARGS__);                                                           \
        tidx = 2;                                                                        \
        func_new(__VA_ARGS__);                                                           \
        tidx = 3;                                                                        \
        func_new(__VA_ARGS__);                                                           \
        (void) tidx;                                                                     \
    } while (0)

#define CHECKASM_PERF_CALL16(...)                                                        \
    do {                                                                                 \
        CHECKASM_PERF_CALL4(__VA_ARGS__);                                                \
        CHECKASM_PERF_CALL4(__VA_ARGS__);                                                \
        CHECKASM_PERF_CALL4(__VA_ARGS__);                                                \
        CHECKASM_PERF_CALL4(__VA_ARGS__);                                                \
    } while (0)

/* Naive loop; used when perf.start/stop() is expected to be slow or imprecise, or when
 * we have no ASM cycle counters, or when the number of iterations is low */
#define CHECKASM_PERF_BENCH_SIMPLE(count, time, ...)                                     \
    do {                                                                                 \
        time = perf.start();                                                             \
        for (int tidx = 0; tidx < count; tidx++)                                         \
            func_new(__VA_ARGS__);                                                       \
        time = perf.stop(time);                                                          \
    } while (0)

/* Unrolled loop with inline outlier rejection; used when we have asm cycle counters */
#define CHECKASM_PERF_BENCH_ASM(total_count, time, ...)                                  \
    do {                                                                                 \
        int      tcount_trim = 0;                                                        \
        uint64_t tsum_trim   = 0;                                                        \
        for (int titer = 0; titer < total_count; titer += 32) {                          \
            uint64_t t = CHECKASM_PERF_ASM();                                            \
            CHECKASM_PERF_CALL16(__VA_ARGS__);                                           \
            CHECKASM_PERF_CALL16(__VA_ARGS__);                                           \
            t = CHECKASM_PERF_ASM() - t;                                                 \
            if (t * tcount_trim <= tsum_trim * 4 && (titer > 0 || total_count < 1000)) { \
                tsum_trim += t;                                                          \
                tcount_trim++;                                                           \
            }                                                                            \
        }                                                                                \
        time        = tsum_trim;                                                         \
        total_count = tcount_trim << 5;                                                  \
    } while (0)

/* Select the best benchmarking method at runtime */
#ifdef CHECKASM_PERF_ASM
  #ifndef CHECKASM_PERF_ASM_USABLE
    #define CHECKASM_PERF_ASM_USABLE perf.asm_usable
  #endif
  #define CHECKASM_PERF_BENCH(count, time, ...)                                          \
      do {                                                                               \
          const CheckasmPerf perf = *checkasm_get_perf();                                \
          if (CHECKASM_PERF_ASM_USABLE && count >= 128) {                                \
              CHECKASM_PERF_BENCH_ASM(count, time, __VA_ARGS__);                         \
          } else {                                                                       \
              CHECKASM_PERF_BENCH_SIMPLE(count, time, __VA_ARGS__);                      \
          }                                                                              \
      } while (0)
#else /* !CHECKASM_PERF_ASM */
  #define CHECKASM_PERF_BENCH(count, time, ...)                                          \
      do {                                                                               \
          const CheckasmPerf perf = *checkasm_get_perf();                                \
          CHECKASM_PERF_BENCH_SIMPLE(count, time, __VA_ARGS__);                          \
      } while (0)
#endif

/* Benchmark the function */
CHECKASM_API int  checkasm_bench_func(void);
CHECKASM_API int  checkasm_bench_runs(void);
CHECKASM_API void checkasm_bench_update(int iterations, uint64_t cycles);
CHECKASM_API void checkasm_bench_finish(void);

#define bench_new(...)                                                                   \
    do {                                                                                 \
        if (checkasm_bench_func()) {                                                     \
            checkasm_set_signal_handler_state(1);                                        \
            for (int truns; (truns = checkasm_bench_runs());) {                          \
                uint64_t time;                                                           \
                CHECKASM_PERF_BENCH(truns, time, __VA_ARGS__);                           \
                checkasm_clear_cpu_state();                                              \
                checkasm_bench_update(truns, time);                                      \
            }                                                                            \
            checkasm_set_signal_handler_state(0);                                        \
            checkasm_bench_finish();                                                     \
        } else {                                                                         \
            const int tidx = 0;                                                          \
            (void) tidx;                                                                 \
            call_new(__VA_ARGS__);                                                       \
        }                                                                                \
    } while (0)

/* Alternates between two pointers. Intended to be used within bench_new()
 * calls for functions which modifies their input buffer(s) to ensure that
 * throughput, and not latency, is measured. */
#define alternate(a, b) ((tidx & 1) ? (b) : (a))

#endif /* CHECKASM_TEST_H */
