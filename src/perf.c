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

#include <stdio.h>

#include "checkasm/perf.h"
#include "checkasm/test.h"
#include "internal.h"
#include "stats.h"

#ifdef CHECKASM_PERF_ASM
static uint64_t perf_start_asm(void)
{
    return CHECKASM_PERF_ASM();
}

static uint64_t perf_stop_asm(uint64_t t)
{
    return CHECKASM_PERF_ASM() - t;
}
#endif

CheckasmPerf checkasm_perf;

COLD int checkasm_perf_init(void)
{
#ifdef CHECKASM_PERF_ASM
    if (!checkasm_save_context(checkasm_context)) {
        /* Try calling the asm timer to see if it works */
        checkasm_set_signal_handler_state(1);
        CHECKASM_PERF_ASM();
        checkasm_set_signal_handler_state(0);

        checkasm_perf.start      = perf_start_asm;
        checkasm_perf.stop       = perf_stop_asm;
        checkasm_perf.name       = CHECKASM_PERF_ASM_NAME;
        checkasm_perf.unit       = CHECKASM_PERF_ASM_UNIT;
        checkasm_perf.asm_usable = 1;
        return 0;
    } else {
        fprintf(stderr, "checkasm: unable to access cycle counter\n");
        checkasm_perf.asm_usable = 0;
    }
#endif

#if HAVE_LINUX_PERF
    if (!checkasm_perf_init_linux(&checkasm_perf))
        return 0;
#endif

#if HAVE_MACOS_KPERF
    if (!checkasm_perf_init_macos(&checkasm_perf))
        return 0;
#endif

    /* Generic fallback to gettime() */
    if (checkasm_gettime_nsec() == (uint64_t) -1) {
        fprintf(stderr, "checkasm: checkasm_gettime_nsec() returned nonsense\n");
        return 1;
    }

    checkasm_perf.start = checkasm_gettime_nsec;
    checkasm_perf.stop  = checkasm_gettime_nsec_diff;
    checkasm_perf.name  = "gettime";
    checkasm_perf.unit  = "nsec";
    return 0;
}

/* Measure the overhead of the timing code */
COLD CheckasmVar checkasm_measure_nop_cycles(void)
{
    const uint64_t target_nsec = 10000000; /* 10 ms */

    CheckasmStats stats;
    checkasm_stats_reset(&stats);

    void (*const func_new)(void *) = checkasm_noop;
    void *const ptr0 = (void *) 0x1000, *const ptr1 = (void *) 0x2000;

    const CheckasmPerf perf = checkasm_perf;
    (void) perf;

    for (uint64_t total_nsec = 0; total_nsec < target_nsec;) {
        int      count  = stats.next_count;
        uint64_t cycles = 0;

        /* Measure the overhead of the timing code (in cycles) */
        uint64_t nsec = checkasm_gettime_nsec();
        CHECKASM_PERF_BENCH(count, cycles, alternate(ptr0, ptr1));
        nsec = checkasm_gettime_nsec() - nsec;
        checkasm_stats_add(&stats, (CheckasmSample) { cycles, count });

        total_nsec += nsec;
        if (nsec < target_nsec >> 7)
            checkasm_stats_count_grow(&stats);
    }

    return checkasm_stats_estimate(&stats);
}

COLD CheckasmVar checkasm_measure_perf_scale(void)
{
    const CheckasmPerf perf = checkasm_perf;

    /* Try to make the loop long enough to be measurable, but not too long
     * to avoid being affected by CPU frequency scaling or preemption */
    const uint64_t target_nsec = 100000 / 2; /* 100 us */

    /* Estimate the time per loop iteration in two different ways */
    CheckasmStats stats_cycles;
    CheckasmStats stats_nsec;
    checkasm_stats_reset(&stats_cycles);
    checkasm_stats_reset(&stats_nsec);

    while (stats_cycles.nb_samples < (int) ARRAY_SIZE(stats_cycles.samples)) {
        const int iters = stats_cycles.next_count;

        /* Warm up the CPU a tiny bit */
        for (int i = 0; i < 100; i++)
            checkasm_noop(NULL);

        uint64_t cycles;
        cycles = perf.start();
        for (int i = 0; i < iters; i++)
            checkasm_noop(NULL);
        cycles = perf.stop(cycles);

        /* Measure the same loop with wallclock time instead of cycles */
        uint64_t nsec = checkasm_gettime_nsec();
        for (int i = 0; i < iters; i++)
            checkasm_noop(NULL);
        nsec = checkasm_gettime_nsec() - nsec;

        checkasm_stats_add(&stats_cycles, (CheckasmSample) { cycles, iters });
        checkasm_stats_add(&stats_nsec, (CheckasmSample) { nsec, iters });

        if (nsec < target_nsec)
            checkasm_stats_count_grow(&stats_cycles);
    }

    CheckasmVar est_cycles = checkasm_stats_estimate(&stats_cycles);
    CheckasmVar est_nsec   = checkasm_stats_estimate(&stats_nsec);
    return checkasm_var_div(est_nsec, est_cycles);
}
