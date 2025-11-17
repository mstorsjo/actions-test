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
#include "html_data.h"
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

#if ARCH_ARM
static checkasm_checked_call_func checkasm_checked_call_ptr;

checkasm_checked_call_func checkasm_get_checked_call_ptr(void)
{
    return checkasm_checked_call_ptr;
}
#endif

typedef struct CheckasmFuncVersion {
    struct CheckasmFuncVersion *next;
    const CheckasmCpuInfo      *cpu;
    CheckasmMeasurement         cycles;

    void *func;
    int   ok;
} CheckasmFuncVersion;

/* Binary search tree node */
typedef struct CheckasmFunc {
    struct CheckasmFunc *child[2];
    struct CheckasmFunc *prev; /* previous function in current report group */
    CheckasmFuncVersion  versions;
    uint8_t              color; /* 0 = red, 1 = black */
    const char          *test_name;
    char                *report_name;
    char                 name[];
} CheckasmFunc;

/* Internal state */
static CheckasmConfig cfg;
static struct {
    /* Current function/test state, reset after each test run */
    struct {
        CheckasmFunc          *funcs;
        CheckasmFunc          *func;
        CheckasmFuncVersion   *func_ver;
        const CheckasmCpuInfo *cpu;
        int                    cpu_name_printed;
        CheckasmCpu            cpu_flags;
        const char            *test_name;
        uint64_t               cycles;
    } current;

    /* Overall stats, kept between test runs */
    int num_checked;
    int num_skipped;
    int num_failed;
    int num_benched;
    int prev_checked, prev_failed;

    /* Miscellaneous state */
    CheckasmStats stats;
    int           suffix_length;
    int           max_function_name_length;
    int           max_report_name_length;
    int           should_fail;
    double        var_sum, var_max;

    /* Timing code measurements (aggregated over multiple trials) */
    CheckasmMeasurement nop_cycles;
    CheckasmMeasurement perf_scale;

    /* Runtime constants */
    uint64_t target_cycles;
    int      skip_tests;
} state;

CheckasmCpu checkasm_get_cpu_flags(void)
{
    return state.current.cpu_flags;
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
        free(f->report_name);
        free(f);
    }
}

/* Allocate a zero-initialized block, clean up and exit on failure */
static void *checkasm_malloc(const size_t size)
{
    void *const ptr = calloc(1, size);
    if (!ptr) {
        fprintf(stderr, "checkasm: malloc failed\n");
        destroy_func_tree(state.current.funcs);
        exit(1);
    }
    return ptr;
}

/* Get the suffix of the specified cpu flag */
static const char *cpu_suffix(const CheckasmCpuInfo *cpu)
{
    return cpu ? cpu->suffix : "c";
}

/* Returns the coefficient of variation (CV) */
static double relative_error(double lvar)
{
    return sqrt(exp(lvar) - 1.0);
}

static inline char separator(CheckasmFormat format)
{
    switch (format) {
    case CHECKASM_FORMAT_CSV: return ',';
    case CHECKASM_FORMAT_TSV: return '\t';
    default:                  return 0;
    }
}

static void json_var(CheckasmJson *json, const char *key, const char *unit,
                     const CheckasmVar var)
{
    if (key)
        checkasm_json_push(json, key, '{');
    if (unit)
        checkasm_json_str(json, "unit", unit);
    checkasm_json(json, "mode", "%g", checkasm_mode(var));
    checkasm_json(json, "median", "%g", checkasm_median(var));
    checkasm_json(json, "mean", "%g", checkasm_mean(var));
    checkasm_json(json, "lowerCI", "%g", checkasm_sample(var, -1.96));
    checkasm_json(json, "upperCI", "%g", checkasm_sample(var, 1.96));
    checkasm_json(json, "stdDev", "%g", checkasm_stddev(var));
    checkasm_json(json, "logMean", "%g", var.lmean);
    checkasm_json(json, "logVar", "%g", var.lvar);
    if (key)
        checkasm_json_pop(json, '}');
}

static void json_measurement(CheckasmJson *json, const char *key, const char *unit,
                             const CheckasmMeasurement measurement)
{
    const CheckasmVar result = checkasm_measurement_result(measurement);
    if (key)
        checkasm_json_push(json, key, '{');
    json_var(json, NULL, unit, result);
    checkasm_json(json, "numMeasurements", "%d", measurement.nb_measurements);

    if (measurement.stats.nb_samples) {
        json_var(json, "regressionSlope", unit,
                 checkasm_stats_estimate(&measurement.stats));
        checkasm_json_push(json, "rawData", '[');
        for (int i = 0; i < measurement.stats.nb_samples; i++) {
            const CheckasmSample s = measurement.stats.samples[i];
            checkasm_json(json, NULL, "{ \"iters\": %d, \"cycles\": %" PRIu64 " }",
                          s.count, s.sum);
        }
        checkasm_json_pop(json, ']');
    }

    if (key)
        checkasm_json_pop(json, '}');
}

struct IterState {
    const char  *test;
    const char  *report;
    CheckasmJson json;
};

static void print_bench_header(struct IterState *const iter)
{
    const CheckasmVar   nop_cycles = checkasm_measurement_result(state.nop_cycles);
    const CheckasmVar   perf_scale = checkasm_measurement_result(state.perf_scale);
    const CheckasmVar   nop_time   = checkasm_var_mul(nop_cycles, perf_scale);
    CheckasmJson *const json       = &iter->json;

    switch (cfg.format) {
    case CHECKASM_FORMAT_TSV:
    case CHECKASM_FORMAT_CSV:
        if (cfg.verbose) {
            const char sep = separator(cfg.format);
            printf("name%csuffix%c%ss%cstddev%cnanoseconds\n", sep, sep,
                   checkasm_perf.unit, sep, sep);
            printf("nop%c%c%.4f%c%.5f%c%.4f\n", sep, sep, checkasm_mode(nop_cycles), sep,
                   checkasm_stddev(nop_cycles), sep, checkasm_mode(nop_time));
        }
        break;
    case CHECKASM_FORMAT_HTML:
        printf("<!doctype html>\n"
               "<html>\n"
               "<head>\n"
               "  <meta charset=\"utf-8\"/>\n"
               "  <title>checkasm report</title>\n"
               "  <script type=\"module\">\n"
               "    %s"
               "    %s"
               "  </script>\n"
               "  <style>\n"
               "    %s"
               "  </style>\n"
               "  <script type=\"application/json\" id=\"report-data\">\n",
               checkasm_chart_js, checkasm_js, checkasm_css);
        /* fall through */
    case CHECKASM_FORMAT_JSON:
        checkasm_json_push(json, NULL, '{');
        checkasm_json_str(json, "checkasmVersion", CHECKASM_VERSION);
        checkasm_json(json, "numChecked", "%d", state.num_checked);
        checkasm_json(json, "numFailed", "%d", state.num_failed);
        checkasm_json(json, "numSkipped", "%d", state.num_skipped);
        checkasm_json(json, "targetCycles", "%" PRIu64, state.target_cycles);
        checkasm_json(json, "numBenchmarks", "%d", state.num_benched);
        checkasm_json_push(json, "cpuFlags", '{');
        for (int i = 0; i < cfg.nb_cpu_flags; i++) {
            const CheckasmCpu flag = cfg.cpu_flags[i].flag;
            checkasm_json_push(json, cfg.cpu_flags[i].suffix, '{');
            checkasm_json_str(json, "name", cfg.cpu_flags[i].name);
            checkasm_json(json, "available", (cfg.cpu & flag) == flag ? "true" : "false");
            checkasm_json_pop(json, '}');
        }
        checkasm_json_pop(json, '}'); /* close cpuFlags */
        checkasm_json_push(json, "tests", '[');
        for (int i = 0; i < cfg.nb_tests; i++)
            checkasm_json_str(json, NULL, cfg.tests[i].name);
        checkasm_json_pop(json, ']'); /* close tests */
        char perf_scale_unit[32];
        snprintf(perf_scale_unit, sizeof(perf_scale_unit), "nsec/%s", checkasm_perf.unit);
        json_measurement(json, "nopCycles", checkasm_perf.unit, state.nop_cycles);
        json_measurement(json, "timerScale", perf_scale_unit, state.perf_scale);
        json_var(json, "nopTime", checkasm_perf.unit, nop_time);
        checkasm_json_push(json, "functions", '{');
        break;
    case CHECKASM_FORMAT_PRETTY:
        checkasm_fprintf(stdout, COLOR_YELLOW, "Benchmark results:\n");
        checkasm_fprintf(stdout, COLOR_GREEN, "  name%*ss",
                         5 + state.max_function_name_length, checkasm_perf.unit);
        if (cfg.verbose) {
            checkasm_fprintf(stdout, COLOR_GREEN, " +/- stddev %*s", 26,
                             "time (nanoseconds)");
        }
        checkasm_fprintf(stdout, COLOR_GREEN, " (vs ref)\n");
        if (cfg.verbose) {
            printf("  nop:%*.1f +/- %-7.1f %11.1f ns +/- %-6.1f\n",
                   6 + state.max_function_name_length, checkasm_mode(nop_cycles),
                   checkasm_stddev(nop_cycles), checkasm_mode(nop_time),
                   checkasm_stddev(nop_time));
        }
        break;
    }
}

static void print_bench_footer(struct IterState *const iter)
{
    const double        err_rel = relative_error(state.var_sum / state.num_benched);
    const double        err_max = relative_error(state.var_max);
    CheckasmJson *const json    = &iter->json;

    switch (cfg.format) {
    case CHECKASM_FORMAT_TSV:
    case CHECKASM_FORMAT_CSV: break;
    case CHECKASM_FORMAT_PRETTY:
        if (cfg.verbose) {
            printf(" - average timing error: %.3f%% across %d benchmarks "
                   "(maximum %.3f%%)\n",
                   100.0 * err_rel, state.num_benched, err_max);
        }
        break;
    case CHECKASM_FORMAT_HTML:
    case CHECKASM_FORMAT_JSON:
        checkasm_json_pop(json, '}'); /* close functions */
        checkasm_json(json, "averageError", "%g", err_rel);
        checkasm_json(json, "maximumError", "%g", err_max);
        checkasm_json_pop(json, '}'); /* close root */

        if (cfg.format == CHECKASM_FORMAT_HTML) {
            printf("  </script>\n"
                   "  <meta name=\"viewport\" content=\"width=device-width, "
                   "initial-scale=1\">\n"
                   "</head>\n"
                   "%s"
                   "</html>\n",
                   checkasm_html_body);
        }
        break;
    }
}

static void print_bench_iter(const CheckasmFunc *const f, struct IterState *const iter)
{
    CheckasmJson *const json = &iter->json;
    const char          sep  = separator(cfg.format);
    if (!f)
        return;

    print_bench_iter(f->child[0], iter);

    const CheckasmFuncVersion *ref        = &f->versions;
    const CheckasmFuncVersion *v          = ref;
    const CheckasmVar          nop_cycles = checkasm_measurement_result(state.nop_cycles);
    const CheckasmVar          perf_scale = checkasm_measurement_result(state.perf_scale);

    /* Defer pushing the function header until we know that we have at least one
     * benchmark to report */
    int json_func_pushed = 0;

    do {
        if (v->cycles.nb_measurements) {
            const CheckasmVar raw     = checkasm_measurement_result(v->cycles);
            const CheckasmVar raw_ref = checkasm_measurement_result(ref->cycles);

            const CheckasmVar cycles     = checkasm_var_sub(raw, nop_cycles);
            const CheckasmVar cycles_ref = checkasm_var_sub(raw_ref, nop_cycles);
            const CheckasmVar ratio      = checkasm_var_div(cycles_ref, cycles);
            const CheckasmVar raw_time   = checkasm_var_mul(raw, perf_scale);
            const CheckasmVar time       = checkasm_var_mul(cycles, perf_scale);

            switch (cfg.format) {
            case CHECKASM_FORMAT_HTML:
            case CHECKASM_FORMAT_JSON:
                if (!json_func_pushed) {
                    checkasm_json_push(json, f->name, '{');
                    checkasm_json_str(json, "testName", f->test_name);
                    checkasm_json_str(json, "reportName",
                                      f->report_name ? f->report_name : "unknown");
                    checkasm_json_push(json, "versions", '{');
                    json_func_pushed = 1;
                }

                checkasm_json_push(json, cpu_suffix(v->cpu), '{');
                json_measurement(json, "rawCycles", checkasm_perf.unit, v->cycles);
                json_var(json, "rawTime", "nsec", raw_time);
                json_var(json, "adjustedCycles", checkasm_perf.unit, cycles);
                json_var(json, "adjustedTime", "nsec", time);
                if (v != ref && ref->cycles.nb_measurements)
                    json_var(json, "ratio", NULL, checkasm_var_div(cycles_ref, cycles));
                checkasm_json_pop(json, '}'); /* close version */
                break;
            case CHECKASM_FORMAT_TSV:
            case CHECKASM_FORMAT_CSV:
                printf("%s%c%s%c%.4f%c%.5f%c%.4f\n", f->name, sep, cpu_suffix(v->cpu),
                       sep, checkasm_mode(cycles), sep, checkasm_stddev(cycles), sep,
                       checkasm_mode(time));
                break;
            case CHECKASM_FORMAT_PRETTY:;
                const int pad = 12 + state.max_function_name_length
                              - printf("  %s_%s:", f->name, cpu_suffix(v->cpu));
                printf("%*.1f", imax(pad, 0), checkasm_mode(cycles));
                if (cfg.verbose) {
                    printf(" +/- %-7.1f %11.1f ns +/- %-6.1f", checkasm_stddev(cycles),
                           checkasm_mode(time), checkasm_stddev(time));
                }
                if (v != ref && ref->cycles.nb_measurements) {
                    const double ratio_lo = checkasm_sample(ratio, -1.0);
                    const double ratio_hi = checkasm_sample(ratio, 1.0);
                    const int    color    = ratio_lo >= 10.0 ? COLOR_GREEN
                                          : ratio_hi >= 1.1 && ratio_lo >= 1.0 ? COLOR_DEFAULT
                                          : ratio_hi >= 1.0 ? COLOR_YELLOW
                                                            : COLOR_RED;
                    printf(" (");
                    checkasm_fprintf(stdout, color, "%5.2fx", checkasm_mode(ratio));
                    printf(")");
                }
                printf("\n");
                break;
            }
        }
    } while ((v = v->next));

    if (json_func_pushed) {
        checkasm_json_pop(json, '}'); /* close versions */
        checkasm_json_pop(json, '}'); /* close function */
    }

    print_bench_iter(f->child[1], iter);
}

static void print_benchmarks(void)
{
    struct IterState iter = { .json.file = stdout };
    print_bench_header(&iter);
    print_bench_iter(state.current.funcs, &iter);
    print_bench_footer(&iter);
    assert(iter.json.level == 0);
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
        /* Associate this function with each other function that was last used
         * as part of the same report group */
        f->prev      = state.current.func;
        f->test_name = state.current.test_name;
        memcpy(f->name, name, name_length);
    }

    return f;
}

/* Decide whether or not the current function needs to be benchmarked */
int checkasm_bench_func(void)
{
    return !state.num_failed && cfg.bench && !checkasm_interrupted;
}

int checkasm_bench_runs(void)
{
    if (checkasm_interrupted)
        return 0;

    /* This limit should be impossible to hit in practice */
    if (state.stats.nb_samples == CHECKASM_STATS_SAMPLES)
        return 0;

    /* Try and gather at least 30 samples for statistical validity, even if
     * it means exceeding the time budget */
    if (state.current.cycles < state.target_cycles || state.stats.nb_samples < 30)
        return state.stats.next_count;
    else
        return 0;
}

/* Update benchmark results of the current function */
void checkasm_bench_update(const int iterations, const uint64_t cycles)
{
    checkasm_stats_add(&state.stats, (CheckasmSample) { cycles, iterations });
    checkasm_stats_count_grow(&state.stats, cycles, state.target_cycles);
    state.current.cycles += cycles;
}

void checkasm_bench_finish(void)
{
    CheckasmFuncVersion *const v = state.current.func_ver;
    if (v && state.current.cycles) {
        const CheckasmVar cycles = checkasm_stats_estimate(&state.stats);

        /* Accumulate multiple bench_new() calls */
        checkasm_measurement_update(&v->cycles, state.stats);

        /* Keep track of min/max/avg (log) variance */
        state.var_sum += cycles.lvar;
        state.var_max = fmax(state.var_max, cycles.lvar);
        state.num_benched++;
    }

    checkasm_stats_reset(&state.stats);
    state.current.cycles = 0;
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

static void handle_interrupt(void);

/* Perform tests and benchmarks for the specified
 * cpu flag if supported by the host */
static void check_cpu_flag(const CheckasmCpuInfo *cpu)
{
    const CheckasmCpu prev_cpu_flags = state.current.cpu_flags;
    if (cpu) {
        state.current.cpu_flags |= cpu->flag & cfg.cpu;
    } else {
        /* Also include any CPU flags not related to the CPU flags list */
        state.current.cpu_flags = cfg.cpu;
        for (int i = 0; i < cfg.nb_cpu_flags; i++)
            state.current.cpu_flags &= ~cfg.cpu_flags[i].flag;
    }

    if (!cpu || state.current.cpu_flags != prev_cpu_flags) {
        state.current.cpu              = cpu;
        state.current.cpu_name_printed = 0;
        state.suffix_length            = (int) strlen(cpu_suffix(cpu)) + 1;
        if (cfg.set_cpu_flags)
            cfg.set_cpu_flags(state.current.cpu_flags);

        for (int i = 0; i < cfg.nb_tests; i++) {
            if (cfg.test_pattern && wildstrcmp(cfg.tests[i].name, cfg.test_pattern))
                continue;
            checkasm_srand(cfg.seed);
            state.current.test_name = cfg.tests[i].name;
            state.should_fail       = 0; // reset between tests
            handle_interrupt();
            cfg.tests[i].func();

            if (cfg.bench) {
                /* Measure NOP and perf scale after each test+CPU flag configuration */
                handle_interrupt();
                const CheckasmStats nop_cycles
                    = checkasm_measure_nop_cycles(state.target_cycles);
                checkasm_measurement_update(&state.nop_cycles, nop_cycles);

                handle_interrupt();
                const CheckasmStats perf_scale = checkasm_measure_perf_scale();
                checkasm_measurement_update(&state.perf_scale, perf_scale);
            }
        }
    }
}

/* Print the name of the current CPU flag, but only do it once */
static void print_cpu_name(void)
{
    if (!state.current.cpu_name_printed) {
        checkasm_fprintf(stderr, COLOR_YELLOW, "%s:\n",
                         state.current.cpu ? state.current.cpu->name : "C");
        state.current.cpu_name_printed = 1;
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
        affinity_err = !spdcs(process, (ULONG[]) { (ULONG)affinity + 256 }, 1);
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

void checkasm_list_functions(const CheckasmConfig *config)
{
    memset(&state, 0, sizeof(state));
    state.skip_tests = 1;
    cfg              = *config;

    check_cpu_flag(NULL);
    for (int i = 0; i < cfg.nb_cpu_flags; i++)
        check_cpu_flag(&cfg.cpu_flags[i]);

    print_functions(state.current.funcs);
    destroy_func_tree(state.current.funcs);
}

static void print_info(void)
{
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
    if (checkasm_has_sve()) {
        const unsigned sve_len = checkasm_sve_length();
        fprintf(stderr, " - CPU: SVE = %d bits\n", sve_len);
    }
#endif
    if (cfg.bench) {
        fprintf(stderr, " - Timing source: %s\n", checkasm_perf.name);
        if (cfg.verbose) {
            const CheckasmVar perf_scale = checkasm_measurement_result(state.perf_scale);
            const CheckasmVar nop_cycles = checkasm_measurement_result(state.nop_cycles);
            const CheckasmVar mhz = checkasm_var_div(checkasm_var_const(1e3), perf_scale);
            fprintf(stderr,
                    " - Timing resolution: %.4f +/- %.3f ns/%s (%.0f +/- %.1f "
                    "MHz) (provisional)\n",
                    checkasm_mode(perf_scale), checkasm_stddev(perf_scale),
                    checkasm_perf.unit, checkasm_mode(mhz), checkasm_stddev(mhz));

            fprintf(stderr,
                    " - No-op overhead: %.2f +/- %.3f %ss per call (provisional)\n",
                    checkasm_mode(nop_cycles), checkasm_stddev(nop_cycles),
                    checkasm_perf.unit);
        }
        fprintf(stderr, " - Bench duration: %d µs per function (%" PRIu64 " %ss)\n",
                cfg.bench_usec, state.target_cycles, checkasm_perf.unit);
    }
    fprintf(stderr, " - Random seed: %u\n", cfg.seed);
}

static int print_summary(void)
{
    char skipped[32] = "";
    if (state.num_skipped)
        snprintf(skipped, sizeof(skipped), " (%d skipped)", state.num_skipped);

    if (state.num_failed) {
        fprintf(stderr, "checkasm: %d of %d tests failed%s\n", state.num_failed,
                state.num_checked, skipped);
    } else if (state.num_checked) {
        fprintf(stderr, "checkasm: all %d tests passed%s\n", state.num_checked, skipped);
    } else {
        fprintf(stderr, "checkasm: no tests to perform%s\n", skipped);
    }

    if (state.num_benched && !state.num_failed)
        print_benchmarks();

    return state.num_failed || state.num_skipped;
}

static void handle_interrupt(void)
{
    if (checkasm_interrupted) {
        fprintf(stderr, "checkasm: interrupted\n");
        exit(print_summary());
    }
}

int checkasm_run(const CheckasmConfig *config)
{
#if !HAVE_HTML_DATA
    if (cfg.format == CHECKASM_FORMAT_HTML) {
        fprintf(stderr, "checkasm: built without HTML support\n");
        return 1;
    }
#endif

    memset(&state, 0, sizeof(state));
    cfg = *config;

    checkasm_set_signal_handlers();
    set_cpu_affinity(cfg.cpu_affinity);
    checkasm_setup_fprintf(stderr);

    if (!cfg.seed)
        cfg.seed = (unsigned) checkasm_gettime_nsec();
    if (!cfg.repeat)
        cfg.repeat = 1;
    if (!cfg.bench_usec)
        cfg.bench_usec = 1000;

    if (cfg.bench) {
        if (checkasm_perf_init())
            return 1;

        checkasm_measurement_init(&state.nop_cycles);
        checkasm_measurement_init(&state.perf_scale);
        checkasm_stats_reset(&state.stats);

        const CheckasmStats perf_stats = checkasm_measure_perf_scale();
        const CheckasmVar   perf_scale = checkasm_stats_estimate(&perf_stats);
        checkasm_measurement_update(&state.perf_scale, perf_stats);
        /* Use the low estimate to compute the number of target cycles, to
         * ensure we reach the required number of cycles with confidence */
        const double low_estimate = checkasm_sample(perf_scale, -1.0);
        if (low_estimate <= 0.0) {
            fprintf(stderr,
                    "checkasm: cycle counter seems to be non-functional "
                    "(invalid timer scale: %.4f %ss/nsec)\n",
                    checkasm_mode(perf_scale), checkasm_perf.unit);
            return 1;
        }

        state.target_cycles            = 1e3 * cfg.bench_usec / low_estimate;
        const CheckasmStats nop_cycles = checkasm_measure_nop_cycles(state.target_cycles);
        checkasm_measurement_update(&state.nop_cycles, nop_cycles);
    }

#if ARCH_ARM
    if (checkasm_has_neon() || checkasm_has_vfp())
        checkasm_checked_call_ptr = checkasm_checked_call_vfp;
    else
        checkasm_checked_call_ptr = checkasm_checked_call_novfp;
#endif

    print_info();

    for (unsigned i = 0; i < cfg.repeat; i++) {
        if (i > 0) {
            checkasm_fprintf(stderr, COLOR_YELLOW, "\nTest #%d:\n", i + 1);
            fprintf(stderr, " - Random seed: %u\n", cfg.seed);
        }

        check_cpu_flag(NULL);
        for (int i = 0; i < cfg.nb_cpu_flags; i++)
            check_cpu_flag(&cfg.cpu_flags[i]);

        int res = print_summary();
        destroy_func_tree(state.current.funcs);
        if (res)
            return res;

        memset(&state.current, 0, sizeof(state.current));
        cfg.seed++;
    }

    return 0;
}

/* Decide whether or not the specified function needs to be tested and
 * allocate/initialize data structures if needed. Returns a pointer to a
 * reference function if the function should be tested, otherwise NULL */
void *checkasm_check_func(void *const func, const char *const name, ...)
{
    char    name_buf[256];
    va_list arg;

    if (checkasm_interrupted)
        return NULL;

    va_start(arg, name);
    int name_length = vsnprintf(name_buf, sizeof(name_buf), name, arg);
    va_end(arg);

    if (!func || name_length <= 0 || (size_t) name_length >= sizeof(name_buf)
        || (cfg.function_pattern && wildstrcmp(name_buf, cfg.function_pattern))) {
        return NULL;
    }

    state.current.func = get_func(&state.current.funcs, name_buf);

    state.current.funcs->color = 1;
    CheckasmFuncVersion *v     = &state.current.func->versions;
    void                *ref   = func;

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
    v->cpu  = state.current.cpu;

    state.current.func_ver = v;
    if (state.skip_tests)
        return NULL;

    checkasm_srand(cfg.seed);

    if (state.current.cpu)
        state.num_checked++;

    if (cfg.bench)
        checkasm_measurement_init(&v->cycles);
    return ref;
}

/* Indicate that the current test has failed, return whether verbose printing
 * is requested. */
#define DEF_FAIL_FUNC(funcname)                                                          \
    int funcname(const char *const msg, ...)                                             \
    {                                                                                    \
        CheckasmFuncVersion *const v = state.current.func_ver;                           \
        if (v && v->ok) {                                                                \
            va_list arg;                                                                 \
                                                                                         \
            if (!state.should_fail) {                                                    \
                print_cpu_name();                                                        \
                checkasm_fprintf(stderr, COLOR_RED, "FAILURE:");                         \
                fprintf(stderr, " %s_%s (", state.current.func->name,                    \
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

int checkasm_should_fail(int s)
{
    state.should_fail = !!s;
#if !CHECKASM_WORKING_SIGNAL_HANDLER
    /* If our signal handler isn't working, we shouldn't run tests that
     * are expected to fail, as they may rely on the signal handler. */
    if (s)
        return 0;
#endif
    return 1;
}

/* Print the outcome of all tests performed since
 * the last time this function was called */
void checkasm_report(const char *const name, ...)
{
    char report_name[256];

    va_list arg;
    va_start(arg, name);
    vsnprintf(report_name, sizeof(report_name), name, arg);
    va_end(arg);

    const int new_checked = state.num_checked - state.prev_checked;
    if (new_checked) {
        int pad_length = (int) state.max_report_name_length + 4;
        assert(!state.skip_tests);

        print_cpu_name();
        pad_length -= fprintf(stderr, " - %s.%s", state.current.test_name, report_name);
        fprintf(stderr, "%*c", imax(pad_length, 0) + 2, '[');

        int fails = state.num_failed - state.prev_failed;
        if (state.should_fail) {
            state.num_failed = state.prev_failed + (new_checked - fails);
        }

        if (state.num_failed == state.prev_failed)
            checkasm_fprintf(stderr, COLOR_GREEN, state.should_fail ? "EXPECTED" : "OK");
        else if (!state.should_fail)
            checkasm_fprintf(stderr, COLOR_RED, "FAILED");
        else
            checkasm_fprintf(stderr, COLOR_RED, "%d/%d EXPECTED", fails, new_checked);
        fprintf(stderr, "]\n");

        state.prev_checked = state.num_checked;
        state.prev_failed  = state.num_failed;
    } else if (!state.current.cpu) {
        /* Calculate the amount of padding required
         * to make the output vertically aligned */
        int length = (int) (strlen(state.current.test_name) + strlen(report_name));
        if (length > state.max_report_name_length)
            state.max_report_name_length = length;
    }

    /* Store the report name with each function in this report group */
    CheckasmFunc *func = state.current.func;
    while (func) {
        if (!func->report_name)
            func->report_name = strdup(report_name);
        func = func->prev;
    }

    state.current.func = NULL; /* reset current function for new report */
    handle_interrupt();
}

static void print_usage(const char *const progname)
{
    fprintf(stderr,
            "Usage: %s [options...] <random seed>\n"
            "    <random seed>              Use fixed value to seed the PRNG\n"
            "Options:\n"
            "    --affinity=<cpu>           Run the process on CPU <cpu>\n"
            "    --bench -b                 Benchmark the tested functions\n"
            "    --csv, --tsv, --json,      Choose output format for benchmarks\n"
            "    --html\n"
            "separated values.\n"
            "    --function=<pattern> -f    Test only the functions matching "
            "<pattern>\n"
            "    --help -h                  Print this usage info\n"
            "    --list-cpu-flags           List available cpu flags\n"
            "    --list-functions           List available functions\n"
            "    --list-tests               List available tests\n"
            "    --duration=<μs>            Benchmark duration (per function) in "
            "μs\n"
            "    --repeat[=<N>]             Repeat tests N times, on successive seeds\n"
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
    *dst = (unsigned) val;
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
            config->format = CHECKASM_FORMAT_CSV;
        } else if (!strcmp(argv[1], "--tsv")) {
            config->format = CHECKASM_FORMAT_TSV;
        } else if (!strcmp(argv[1], "--json")) {
            config->format = CHECKASM_FORMAT_JSON;
        } else if (!strcmp(argv[1], "--html")) {
#if HAVE_HTML_DATA
            config->format = CHECKASM_FORMAT_HTML;
#else
            fprintf(stderr, "checkasm: built without HTML support\n");
            return 1;
#endif
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
        } else if (!strncmp(argv[1], "--repeat=", 9)) {
            const char *const s = argv[1] + 9;
            if (!parseu(&config->repeat, s, 10)) {
                fprintf(stderr, "checkasm: invalid number of repetitions (%s)\n", s);
                print_usage(argv[0]);
                return 1;
            }
        } else if (!strcmp(argv[1], "--repeat")) {
            config->repeat = UINT_MAX;
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
