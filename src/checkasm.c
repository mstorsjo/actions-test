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

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

#include "checkasm/checkasm.h"
#include "checkasm/test.h"
#include "cpu.h"
#include "internal.h"
#include "stats.h"

#ifndef _WIN32
  #if HAVE_PTHREAD_SETAFFINITY_NP
    #include <pthread.h>
    #if HAVE_PTHREAD_NP_H
      #include <pthread_np.h>
    #endif
  #endif
#endif

#ifdef _WIN32
  #include <windows.h>
#endif

typedef struct CheckasmFuncVersion {
    struct CheckasmFuncVersion *next;
    const CheckasmCpuInfo      *cpu;

    void       *func;
    CheckasmVar cycles_prod;
    int         nb_bench;
    int         ok;
} CheckasmFuncVersion;

/* Binary search tree node */
typedef struct CheckasmFunc {
    struct CheckasmFunc *child[2];
    CheckasmFuncVersion  versions;
    uint8_t              color; /* 0 = red, 1 = black */
    char                 name[];
} CheckasmFunc;

/* Internal state */
static CheckasmConfig cfg;
static struct {
    /* Current function/test state */
    CheckasmFunc          *funcs;
    CheckasmFunc          *current_func;
    CheckasmFuncVersion   *current_func_ver;
    const CheckasmCpuInfo *cpu;
    CheckasmCpu            cpu_flags;
    const char            *current_test_name;
    CheckasmStats          stats;
    uint64_t               total_cycles;

    /* Miscellaneous state */
    int num_checked;
    int num_skipped;
    int num_failed;
    int suffix_length;
    int cpu_name_printed;
    int max_function_name_length;
    int should_fail;

    /* Runtime constants */
    CheckasmVar nop_cycles;
    CheckasmVar perf_scale;
    uint64_t    target_cycles;
    int         skip_tests;
} state;

CheckasmCpu checkasm_get_cpu_flags(void)
{
    return state.cpu_flags;
}

/* Deallocate a tree */
static void destroy_func_tree(CheckasmFunc *const f)
{
    if (f) {
        CheckasmFuncVersion *v = f->versions.next;
        while (v) {
            CheckasmFuncVersion *next = v->next;
            free(v);
            v = next;
        }

        destroy_func_tree(f->child[0]);
        destroy_func_tree(f->child[1]);
        free(f);
    }
}

/* Allocate a zero-initialized block, clean up and exit on failure */
static void *checkasm_malloc(const size_t size)
{
    void *const ptr = calloc(1, size);
    if (!ptr) {
        fprintf(stderr, "checkasm: malloc failed\n");
        destroy_func_tree(state.funcs);
        exit(1);
    }
    return ptr;
}

/* Get the suffix of the specified cpu flag */
static const char *cpu_suffix(const CheckasmCpuInfo *cpu)
{
    return cpu ? cpu->suffix : "c";
}

static CheckasmVar get_cycles(const CheckasmFuncVersion *const v)
{
    /* Gives the geometric mean across all bench_new() invocations */
    return v->nb_bench ? checkasm_var_pow(v->cycles_prod, 1.0 / v->nb_bench)
                       : checkasm_var_const(0.0);
}

/* Print benchmark results */
static void print_benchs(const CheckasmFunc *const f)
{
    if (f) {
        print_benchs(f->child[0]);

        const CheckasmFuncVersion *ref = &f->versions;
        const CheckasmFuncVersion *v   = ref;

        do {
            if (v->nb_bench) {
                const CheckasmVar cycles     = get_cycles(v);
                const CheckasmVar cycles_ref = get_cycles(ref);
                const CheckasmVar ratio      = checkasm_var_div(cycles_ref, cycles);
                const CheckasmVar time       = checkasm_var_mul(cycles, state.perf_scale);

                if (cfg.separator) {
                    printf("%s%c%s%c%.4f%c%.5f%c%.4f\n", f->name, cfg.separator,
                           cpu_suffix(v->cpu), cfg.separator, checkasm_mean(cycles),
                           cfg.separator, checkasm_stddev(cycles), cfg.separator,
                           checkasm_mean(time));
                } else {
                    const int pad = 12 + state.max_function_name_length
                                  - printf("  %s_%s:", f->name, cpu_suffix(v->cpu));
                    printf("%*.1f", imax(pad, 0), checkasm_mean(cycles));
                    if (cfg.verbose) {
                        printf(" +/- %-7.1f %11.1f ns +/- %-6.1f",
                               checkasm_stddev(cycles), checkasm_mean(time),
                               checkasm_stddev(time));
                    }
                    if (v != ref && ref->nb_bench) {
                        const double ratio_lo = checkasm_sample(ratio, -1.0);
                        const double ratio_hi = checkasm_sample(ratio, 1.0);
                        const int    color    = ratio_lo >= 10.0 ? COLOR_GREEN
                                              : ratio_hi >= 1.1 && ratio_lo >= 1.0
                                                  ? COLOR_DEFAULT
                                              : ratio_hi >= 1.0 ? COLOR_YELLOW
                                                                : COLOR_RED;
                        printf(" (");
                        checkasm_fprintf(stdout, color, "%5.2fx", checkasm_mean(ratio));
                        printf(")");
                    }
                    printf("\n");
                }
            }
        } while ((v = v->next));

        print_benchs(f->child[1]);
    }
}

#define is_digit(x) ((x) >= '0' && (x) <= '9')

/* ASCIIbetical sort except preserving natural order for numbers */
static int cmp_func_names(const char *a, const char *b)
{
    const char *const start = a;

    int ascii_diff, digit_diff;
    for (; !(ascii_diff = *(const unsigned char *) a - *(const unsigned char *) b) && *a;
         a++, b++)
        ;
    for (; is_digit(*a) && is_digit(*b); a++, b++)
        ;

    if (a > start && is_digit(a[-1]) && (digit_diff = is_digit(*a) - is_digit(*b)))
        return digit_diff;

    return ascii_diff;
}

/* Perform a tree rotation in the specified direction and return the new root */
static CheckasmFunc *rotate_tree(CheckasmFunc *const f, const int dir)
{
    CheckasmFunc *const r = f->child[dir ^ 1];

    f->child[dir ^ 1] = r->child[dir];
    r->child[dir]     = f;
    r->color          = f->color;
    f->color          = 0;
    return r;
}

#define is_red(f) ((f) && !(f)->color)

/* Balance a left-leaning red-black tree at the specified node */
static void balance_tree(CheckasmFunc **const root)
{
    CheckasmFunc *const f = *root;

    if (is_red(f->child[0]) && is_red(f->child[1])) {
        f->color ^= 1;
        f->child[0]->color = f->child[1]->color = 1;
    } else if (!is_red(f->child[0]) && is_red(f->child[1]))
        *root = rotate_tree(f, 0); /* Rotate left */
    else if (is_red(f->child[0]) && is_red(f->child[0]->child[0]))
        *root = rotate_tree(f, 1); /* Rotate right */
}

/* Get a node with the specified name, creating it if it doesn't exist */
static CheckasmFunc *get_func(CheckasmFunc **const root, const char *const name)
{
    CheckasmFunc *f = *root;

    if (f) {
        /* Search the tree for a matching node */
        const int cmp = cmp_func_names(name, f->name);
        if (cmp) {
            f = get_func(&f->child[cmp > 0], name);

            /* Rebalance the tree on the way up if a new node was inserted */
            if (!f->versions.func)
                balance_tree(root);
        }
    } else {
        /* Allocate and insert a new node into the tree */
        const size_t name_length = strlen(name) + 1;
        f = *root = checkasm_malloc(offsetof(CheckasmFunc, name) + name_length);
        memcpy(f->name, name, name_length);
    }

    return f;
}

/* Decide whether or not the current function needs to be benchmarked */
int checkasm_bench_func(void)
{
    return !state.num_failed && cfg.bench;
}

int checkasm_bench_runs(void)
{
    /* This limit should be impossible to hit in practice */
    if (state.stats.nb_samples == ARRAY_SIZE(state.stats.samples))
        return 0;

    /* Try and gather at least 30 samples for statistical validity, even if
     * it means exceeding the time budget */
    if (state.total_cycles < state.target_cycles || state.stats.nb_samples < 30)
        return state.stats.next_count;
    else
        return 0;
}

/* Update benchmark results of the current function */
void checkasm_bench_update(const int iterations, const uint64_t cycles)
{
    checkasm_stats_add(&state.stats, (CheckasmSample) { cycles, iterations });
    state.total_cycles += cycles;

    /* Try and record at least 100 data points for each function */
    if (cycles < state.target_cycles >> 7)
        checkasm_stats_count_grow(&state.stats);
}

void checkasm_bench_finish(void)
{
    CheckasmFuncVersion *const v = state.current_func_ver;
    if (v && state.total_cycles) {
        const CheckasmVar est_raw = checkasm_stats_estimate(&state.stats);
        const CheckasmVar cycles  = checkasm_var_sub(est_raw, state.nop_cycles);

        /* Accumulate multiple bench_new() calls */
        v->cycles_prod = checkasm_var_mul(v->cycles_prod, cycles);
        v->nb_bench++;
    }

    checkasm_stats_reset(&state.stats);
    state.total_cycles = 0;
}

/* Compares a string with a wildcard pattern. */
static int wildstrcmp(const char *str, const char *pattern)
{
    const char *wild = strchr(pattern, '*');
    if (wild) {
        const size_t len = wild - pattern;
        if (strncmp(str, pattern, len))
            return 1;
        while (*++wild == '*')
            ;
        if (!*wild)
            return 0;
        str += len;
        while (*str && wildstrcmp(str, wild))
            str++;
        return !*str;
    }
    return strcmp(str, pattern);
}

/* Perform tests and benchmarks for the specified
 * cpu flag if supported by the host */
static void check_cpu_flag(const CheckasmCpuInfo *cpu)
{
    const CheckasmCpu prev_cpu_flags = state.cpu_flags;
    if (cpu)
        state.cpu_flags |= cpu->flag & cfg.cpu;

    if (!cpu || state.cpu_flags != prev_cpu_flags) {
        state.cpu              = cpu;
        state.cpu_name_printed = 0;
        state.suffix_length    = (int) strlen(cpu_suffix(cpu)) + 1;
        if (cfg.set_cpu_flags)
            cfg.set_cpu_flags(state.cpu_flags);

        for (int i = 0; i < cfg.nb_tests; i++) {
            if (cfg.test_pattern && wildstrcmp(cfg.tests[i].name, cfg.test_pattern))
                continue;
            checkasm_srand(cfg.seed);
            state.current_test_name = cfg.tests[i].name;
            state.should_fail       = 0; // reset between tests
            cfg.tests[i].func();
        }
    }
}

/* Print the name of the current CPU flag, but only do it once */
static void print_cpu_name(void)
{
    if (!state.cpu_name_printed) {
        checkasm_fprintf(stderr, COLOR_YELLOW, "%s:\n",
                         state.cpu ? state.cpu->name : "C");
        state.cpu_name_printed = 1;
    }
}

static int set_cpu_affinity(const uint64_t affinity)
{
    int affinity_err;
    if (!affinity)
        return 0;

#ifdef _WIN32
    HANDLE process = GetCurrentProcess();
  #if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)
    BOOL(WINAPI * spdcs)(HANDLE, const ULONG *, ULONG) = (void *) GetProcAddress(
        GetModuleHandleW(L"kernel32.dll"), "SetProcessDefaultCpuSets");
    if (spdcs)
        affinity_err = !spdcs(process, (ULONG[]) { affinity + 256 }, 1);
    else
  #endif
    {
        if (affinity < sizeof(DWORD_PTR) * 8)
            affinity_err = !SetProcessAffinityMask(process, (DWORD_PTR) 1 << affinity);
        else
            affinity_err = 1;
    }
#elif HAVE_PTHREAD_SETAFFINITY_NP && defined(CPU_SET)
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(affinity, &set);
    affinity_err = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
#else
    (void) affinity;
    (void) affinity_err;
    fprintf(stderr, "checkasm: --affinity is not supported on your system\n");
    return 1;
#endif

    if (affinity_err) {
        fprintf(stderr, "checkasm: invalid cpu affinity (%" PRIx64 ")\n", affinity);
        return 1;
    } else {
        fprintf(stderr, "checkasm: running on cpu %" PRIx64 "\n", affinity);
        return 0;
    }
}

void checkasm_list_cpu_flags(const CheckasmConfig *cfg)
{
    const int last_flag = cfg->nb_cpu_flags - 1;
    checkasm_setup_fprintf(stdout);

    for (int i = 0; i < cfg->nb_cpu_flags; i++) {
        const CheckasmCpu flag = cfg->cpu_flags[i].flag;
        if ((cfg->cpu & flag) == flag)
            checkasm_fprintf(stdout, COLOR_GREEN, "%s", cfg->cpu_flags[i].suffix);
        else
            checkasm_fprintf(stdout, COLOR_RED, "~%s", cfg->cpu_flags[i].suffix);
        printf(i == last_flag ? "\n" : ", ");
    }
}

void checkasm_list_tests(const CheckasmConfig *config)
{
    for (int i = 0; i < config->nb_tests; i++)
        printf("%s\n", config->tests[i].name);
}

static void print_functions(const CheckasmFunc *const f)
{
    if (f) {
        print_functions(f->child[0]);
        const CheckasmFuncVersion *v = &f->versions;
        printf("%s (%s", f->name, cpu_suffix(v->cpu));
        while ((v = v->next))
            printf(", %s", cpu_suffix(v->cpu));
        printf(")\n");
        print_functions(f->child[1]);
    }
}

/* Also include any CPU flags not related to the CPU flags list */
static CheckasmCpu initial_cpu_flags(const CheckasmConfig *config)
{
    CheckasmCpu cpu = config->cpu;
    for (int i = 0; i < config->nb_cpu_flags; i++)
        cpu &= ~config->cpu_flags[i].flag;
    return cpu;
}

void checkasm_list_functions(const CheckasmConfig *config)
{
    memset(&state, 0, sizeof(state));
    state.cpu_flags  = initial_cpu_flags(config);
    state.skip_tests = 1;
    cfg              = *config;

    check_cpu_flag(NULL);
    for (int i = 0; i < cfg.nb_cpu_flags; i++)
        check_cpu_flag(&cfg.cpu_flags[i]);

    print_functions(state.funcs);
    destroy_func_tree(state.funcs);
}

int checkasm_run(const CheckasmConfig *config)
{
    memset(&state, 0, sizeof(state));
    state.cpu_flags = initial_cpu_flags(config);
    cfg             = *config;

    checkasm_set_signal_handlers();
    set_cpu_affinity(cfg.cpu_affinity);
    checkasm_setup_fprintf(stderr);

    if (!cfg.seed)
        cfg.seed = (unsigned) checkasm_gettime_nsec();
    if (!cfg.bench_usec)
        cfg.bench_usec = 2000;

    if (cfg.bench) {
        if (checkasm_perf_init())
            return 1;
        state.nop_cycles = checkasm_measure_nop_cycles();
        state.perf_scale = checkasm_measure_perf_scale();
        /* Use the low estimate to compute the number of target cycles, to
         * ensure we reach the required number of cycles with confidence */
        const double low_estimate = checkasm_sample(state.perf_scale, -1.0);
        state.target_cycles       = 1e3 * cfg.bench_usec / low_estimate;
        checkasm_stats_reset(&state.stats);
    }

#if ARCH_ARM
    if (checkasm_has_neon())
        checkasm_checked_call_ptr = checkasm_checked_call_vfp;
    else
        checkasm_checked_call_ptr = checkasm_checked_call_novfp;
#endif

    checkasm_fprintf(stderr, COLOR_YELLOW, "checkasm:\n");
#if ARCH_X86
    char           name[48];
    const unsigned cpuid = checkasm_init_x86(name);
    for (size_t len = strlen(name); len && name[len - 1] == ' '; len--)
        name[len - 1] = '\0'; /* trim trailing whitespace */
    fprintf(stderr, " - CPU: %s (%08X)\n", name, cpuid);
#elif ARCH_RV64
    const unsigned vlenb = checkasm_init_riscv();
    if (vlenb)
        fprintf(stderr, " - CPU: VLEN = %d bits\n", vlenb * 8);
#elif ARCH_AARCH64 && HAVE_SVE
    const unsigned sve_len = checkasm_sve_length();
    if (sve_len)
        fprintf(stderr, " - CPU: SVE = %d bits\n", sve_len);
#endif
    if (cfg.bench) {
        fprintf(stderr, " - Timing source: %s\n", checkasm_perf.name);
        if (cfg.verbose) {
            const CheckasmVar mhz
                = checkasm_var_div(checkasm_var_const(1e3), state.perf_scale);
            fprintf(stderr,
                    " - Timing resolution: %.4f +/- %.3f ns/%s (%.0f +/- %.1f "
                    "MHz)\n",
                    checkasm_mean(state.perf_scale), checkasm_stddev(state.perf_scale),
                    checkasm_perf.unit, checkasm_mean(mhz), checkasm_stddev(mhz));

            fprintf(stderr, " - No-op overhead: %.2f +/- %.3f %ss per call\n",
                    checkasm_mean(state.nop_cycles), checkasm_stddev(state.nop_cycles),
                    checkasm_perf.unit);
        }
        fprintf(stderr, " - Bench duration: %d µs per function (%" PRIu64 " %ss)\n",
                cfg.bench_usec, state.target_cycles, checkasm_perf.unit);
    }
    fprintf(stderr, " - Random seed: %u\n", cfg.seed);

    check_cpu_flag(NULL);
    for (int i = 0; i < cfg.nb_cpu_flags; i++)
        check_cpu_flag(&cfg.cpu_flags[i]);

    int  ret         = 0;
    char skipped[32] = "";
    if (state.num_skipped)
        snprintf(skipped, sizeof(skipped), " (%d skipped)", state.num_skipped);
    if (state.num_failed) {
        fprintf(stderr, "checkasm: %d of %d tests failed%s\n", state.num_failed,
                state.num_checked, skipped);
        ret = 1;
    } else {
        if (state.num_checked)
            fprintf(stderr, "checkasm: all %d tests passed%s\n", state.num_checked,
                    skipped);
        else
            fprintf(stderr, "checkasm: no tests to perform%s\n", skipped);

        if (cfg.bench && state.max_function_name_length) {
            if (cfg.separator && cfg.verbose) {
                printf("name%csuffix%c%ss%cstddev%cnanoseconds\n", cfg.separator,
                       cfg.separator, checkasm_perf.unit, cfg.separator, cfg.separator);
            } else if (!cfg.separator) {
                checkasm_fprintf(stdout, COLOR_YELLOW, "Benchmark results:\n");
                checkasm_fprintf(stdout, COLOR_GREEN, "  name%*ss",
                                 5 + state.max_function_name_length, checkasm_perf.unit);
                if (cfg.verbose) {
                    checkasm_fprintf(stdout, COLOR_GREEN, " +/- stddev %*s", 26,
                                     "time (nanoseconds)");
                }
                checkasm_fprintf(stdout, COLOR_GREEN, " (vs ref)\n");
            }
            print_benchs(state.funcs);
        }
    }

    destroy_func_tree(state.funcs);
    return ret;
}

/* Decide whether or not the specified function needs to be tested and
 * allocate/initialize data structures if needed. Returns a pointer to a
 * reference function if the function should be tested, otherwise NULL */
void *checkasm_check_func(void *const func, const char *const name, ...)
{
    char    name_buf[256];
    va_list arg;

    va_start(arg, name);
    int name_length = vsnprintf(name_buf, sizeof(name_buf), name, arg);
    va_end(arg);

    if (!func || name_length <= 0 || (size_t) name_length >= sizeof(name_buf)
        || (cfg.function_pattern && wildstrcmp(name_buf, cfg.function_pattern))) {
        return NULL;
    }

    state.current_func = get_func(&state.funcs, name_buf);

    state.funcs->color       = 1;
    CheckasmFuncVersion *v   = &state.current_func->versions;
    void                *ref = func;

    if (v->func) {
        CheckasmFuncVersion *prev;
        do {
            /* Only test functions that haven't already been tested */
            if (v->func == func || (!v->cpu && !v->ok))
                return NULL;

            if (v->ok)
                ref = v->func;

            prev = v;
        } while ((v = v->next));

        v = prev->next = checkasm_malloc(sizeof(CheckasmFuncVersion));
    }

    name_length += state.suffix_length;
    if (name_length > state.max_function_name_length)
        state.max_function_name_length = name_length;

    v->func = func;
    v->ok   = 1;
    v->cpu  = state.cpu;

    state.current_func_ver = v;
    if (state.skip_tests)
        return NULL;

    checkasm_srand(cfg.seed);

    if (state.cpu)
        state.num_checked++;

    if (cfg.bench)
        v->cycles_prod = checkasm_var_const(1.0);
    return ref;
}

/* Indicate that the current test has failed, return whether verbose printing
 * is requested. */
#define DEF_FAIL_FUNC(funcname)                                                          \
    int funcname(const char *const msg, ...)                                             \
    {                                                                                    \
        CheckasmFuncVersion *const v = state.current_func_ver;                           \
        if (v && v->ok) {                                                                \
            va_list arg;                                                                 \
                                                                                         \
            if (!state.should_fail) {                                                    \
                print_cpu_name();                                                        \
                checkasm_fprintf(stderr, COLOR_RED, "FAILURE:");                         \
                fprintf(stderr, " %s_%s (", state.current_func->name,                    \
                        cpu_suffix(v->cpu));                                             \
                va_start(arg, msg);                                                      \
                vfprintf(stderr, msg, arg);                                              \
                va_end(arg);                                                             \
                fprintf(stderr, ")\n");                                                  \
            }                                                                            \
                                                                                         \
            v->ok = 0;                                                                   \
            if (v->cpu)                                                                  \
                state.num_failed++;                                                      \
            else                                                                         \
                state.num_skipped++;                                                     \
        }                                                                                \
        return cfg.verbose && !state.should_fail;                                        \
    }

/* We need to define two versions of this function, one to export and one for
 * asm routines to call internally (with local linking) */
DEF_FAIL_FUNC(checkasm_fail_func);
DEF_FAIL_FUNC(checkasm_fail_internal);

void checkasm_should_fail(int s)
{
    state.should_fail = !!s;
}

/* Print the outcome of all tests performed since
 * the last time this function was called */
void checkasm_report(const char *const name, ...)
{
    static int    prev_checked, prev_failed;
    static size_t max_length;

    const int new_checked = state.num_checked - prev_checked;
    if (new_checked) {
        int     pad_length = (int) max_length + 4;
        va_list arg;
        assert(!state.skip_tests);

        print_cpu_name();
        pad_length -= fprintf(stderr, " - %s.", state.current_test_name);
        va_start(arg, name);
        pad_length -= vfprintf(stderr, name, arg);
        va_end(arg);
        fprintf(stderr, "%*c", imax(pad_length, 0) + 2, '[');

        if (state.should_fail) {
            /* Pass the test as a whole if *any* failure was recorded since the
             * last time checkasm_report() was called; do this instead of
             * requiring a failure for every single test, since not all checked
             * functions in each block may actually trigger the failure,
             * dependent on register sizes etc. */
            state.num_failed
                = prev_failed + (state.num_failed == prev_failed) * new_checked;
        }

        if (state.num_failed == prev_failed)
            checkasm_fprintf(stderr, COLOR_GREEN, state.should_fail ? "EXPECTED" : "OK");
        else
            checkasm_fprintf(stderr, COLOR_RED,
                             state.should_fail ? "SHOULD FAIL" : "FAILED");
        fprintf(stderr, "]\n");

        prev_checked = state.num_checked;
        prev_failed  = state.num_failed;
    } else if (!state.cpu) {
        /* Calculate the amount of padding required
         * to make the output vertically aligned */
        size_t  length = strlen(state.current_test_name);
        va_list arg;

        va_start(arg, name);
        length += vsnprintf(NULL, 0, name, arg);
        va_end(arg);

        if (length > max_length)
            max_length = length;
    }
}

#if ARCH_ARM
void (*checkasm_checked_call_ptr)(void *func, int dummy, ...);
#endif

static void print_usage(const char *const progname)
{
    fprintf(stderr,
            "Usage: %s [options...] <random seed>\n"
            "    <random seed>              Use fixed value to seed the PRNG\n"
            "Options:\n"
            "    --affinity=<cpu>           Run the process on CPU <cpu>\n"
            "    --bench -b                 Benchmark the tested functions\n"
            "    --csv, --tsv               Output results in rows of comma or tab "
            "separated values.\n"
            "    --function=<pattern> -f    Test only the functions matching "
            "<pattern>\n"
            "    --help -h                  Print this usage info\n"
            "    --list-cpu-flags           List available cpu flags\n"
            "    --list-functions           List available functions\n"
            "    --list-tests               List available tests\n"
            "    --duration=<μs>            Benchmark duration (per function) in "
            "μs\n"
            "    --test=<pattern> -t        Test only <pattern>\n"
            "    --verbose -v               Print verbose timing info and failure "
            "data\n",
            progname);
}

static int parseu(unsigned *const dst, const char *const str, const int base)
{
    unsigned long val;
    char         *end;
    errno = 0;
    val   = strtoul(str, &end, base);
    if (errno || end == str || *end || val > (unsigned) -1)
        return 0;
    *dst = val;
    return 1;
}

int checkasm_main(CheckasmConfig *config, int argc, const char *argv[])
{
    while (argc > 1) {
        if (!strncmp(argv[1], "--help", 6) || !strcmp(argv[1], "-h")) {
            print_usage(argv[0]);
            return 0;
        } else if (!strcmp(argv[1], "--list-cpu-flags")
                   || !strcmp(argv[1], "--list-cpuflags")) {
            checkasm_list_cpu_flags(config);
            return 0;
        } else if (!strcmp(argv[1], "--list-tests")) {
            checkasm_list_tests(config);
            return 0;
        } else if (!strcmp(argv[1], "--list-functions")) {
            checkasm_list_functions(config);
            return 0;
        } else if (!strcmp(argv[1], "--bench") || !strcmp(argv[1], "-b")) {
            config->bench = 1;
        } else if (!strcmp(argv[1], "--csv")) {
            config->separator = ',';
        } else if (!strcmp(argv[1], "--tsv")) {
            config->separator = '\t';
        } else if (!strncmp(argv[1], "--duration=", 11)) {
            const char *const s = argv[1] + 11;
            if (!parseu(&config->bench_usec, s, 10)) {
                fprintf(stderr, "checkasm: invalid duration (%s)\n", s);
                print_usage(argv[0]);
                return 1;
            }
        } else if (!strncmp(argv[1], "--test=", 7)) {
            config->test_pattern = argv[1] + 7;
        } else if (!strcmp(argv[1], "-t")) {
            config->test_pattern = argc > 1 ? argv[2] : "";
            argc--;
            argv++;
        } else if (!strncmp(argv[1], "--function=", 11)) {
            config->function_pattern = argv[1] + 11;
        } else if (!strcmp(argv[1], "-f")) {
            config->function_pattern = argc > 1 ? argv[2] : "";
            argc--;
            argv++;
        } else if (!strcmp(argv[1], "--verbose") || !strcmp(argv[1], "-v")) {
            config->verbose = 1;
        } else if (!strncmp(argv[1], "--affinity=", 11)) {
            const char *const s = argv[1] + 11;
            if (!parseu(&config->cpu_affinity, s, 16)) {
                fprintf(stderr, "checkasm: invalid cpu affinity (%s)\n", s);
                print_usage(argv[0]);
                return 1;
            }
        } else {
            if (!parseu(&config->seed, argv[1], 10)) {
                fprintf(stderr, "checkasm: unknown option (%s)\n", argv[1]);
                print_usage(argv[0]);
                return 1;
            }
        }

        argc--;
        argv++;
    }

    return checkasm_run(config);
}
