/*
 * Copyright © 2024, Marvin Scholz
 * Copyright © 2019, VideoLAN and dav1d authors
 * Copyright © 2019, Two Orioles, LLC
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

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tests.h"

typedef uint8_t pixel;
#define PIXEL_TYPE uint8_t

void checkasm_test_copy(copy_func fun, const char *name, const int min_width)
{
#define WIDTH 256
    PIXEL_RECT(c_dst, WIDTH, 1);
    PIXEL_RECT(a_dst, WIDTH, 1);

    CHECKASM_ALIGN(uint8_t src[WIDTH]);
    INITIALIZE_BUF(src);

    declare_func(void, uint8_t *dest, const uint8_t *src, size_t n);

    for (int w = min_width; w <= WIDTH; w *= 2) {
        if (check_func(fun, "%s_%d", name, w)) {
            CLEAR_PIXEL_RECT(c_dst);
            CLEAR_PIXEL_RECT(a_dst);

            /* Make sure that the destination buffer actually differs from
             * the source buffer, to make sure that a skipped write does
             * trigger a failure. */
            for (int i = 0; i < w; i++)
                c_dst[i] = a_dst[i] = ~src[i];
            call_ref(c_dst, src, w);
            call_new(a_dst, src, w);
            checkasm_check_pixel_padded(c_dst, c_dst_stride, a_dst, a_dst_stride, w, 1,
                                        "dst data");

            bench_new(a_dst, src, w);
        }
    }

    report("%s", name);
}

void checkasm_test_noop(noop_func fun, const char *name)
{
    declare_func(void, int);

    if (check_func(fun, "%s", name)) {
        /* don't call unchecked because some of these functions are designed to
         * e.g. intentionally corrupt the stack */
        (void) func_ref;
        call_new(0);
    }

    report("%s", name);
}

void checkasm_test_float(float_func fun, const char *name, const float input)
{
    declare_func(float, float);

    if (check_func(fun, "%s", name)) {
        float x = call_ref(input);
        float y = call_new(input);
        if (!float_near_abs_eps(x, y, FLT_EPSILON)) {
            if (fail())
                fprintf(stderr, "expected %f, got %f\n", x, y);
        }
    }

    report("%s", name);
}

static DEF_COPY_FUNC(memset)
{
    memset(dst, 0xAC, size);
}

static DEF_COPY_FUNC(overwrite_left)
{
    memcpy(dst, src, size);
    dst[-1] = dst[-2] = dst[-3] = dst[-4] = 0xAC;
}

static DEF_COPY_FUNC(overwrite_right)
{
    memcpy(dst, src, size);
    dst[size] = dst[size + 1] = dst[size + 2] = dst[size + 3] = 0xAC;
}

static DEF_COPY_FUNC(underwrite)
{
    if (size < 4)
        return;
    memcpy(dst, src, size - 4);
}

static DEF_NOOP_FUNC(segfault)
{
    volatile int *bad = NULL;
    *bad              = 0;
}

static DEF_FLOAT_FUNC(sqrt)
{
    return sqrtf(input);
}

static int identity_ref(const int x)
{
    return x;
}

/* Just make this a separate function to test the checked wrappers */
static int identity_new(const int x)
{
    return x;
}

static void checkasm_test_retval(void)
{
    const uint64_t flags = checkasm_get_cpu_flags();

    declare_func(int, int);

    if (check_func(flags ? identity_new : identity_ref, "identity")) {
        for (int i = 0; i < 10; i++) {
            int x = call_ref(i);
            int y = call_new(i);
            if (x != y) {
                if (fail())
                    fprintf(stderr, "expected %d, got %d\n", x, y);
            }
        }
    }

    report("identity");
}

static int truncate_c(const float x)
{
    return (int) x;
}

static void checkasm_test_float_arg(void)
{
    declare_func(int, float);

    if (check_func(truncate_c, "truncate")) {
        for (float f = 0.0f; f <= 10.0f; f += 0.5f) {
            int x = call_ref(f);
            int y = call_new(f);
            if (x != y) {
                if (fail())
                    fprintf(stderr, "expected %d, got %d\n", x, y);
            }
        }
    }

    report("truncate");
}

DEF_COPY_GETTER(CHECKASM_CPU_FLAG_BAD_C, memset)
DEF_COPY_GETTER(CHECKASM_CPU_FLAG_BAD_C, overwrite_left)
DEF_COPY_GETTER(CHECKASM_CPU_FLAG_BAD_C, overwrite_right)
DEF_COPY_GETTER(CHECKASM_CPU_FLAG_BAD_C, underwrite)
DEF_NOOP_GETTER(CHECKASM_CPU_FLAG_BAD_C, segfault)

void checkasm_check_generic(void)
{
    checkasm_test_copy(checkasm_copy_c, "copy_generic", 1);
    checkasm_test_float(checkasm_sqrt, "sqrt_generic", 2.0f);
    checkasm_test_float_arg();
    checkasm_test_retval();

    if (!checkasm_should_fail(1))
        return;

    checkasm_test_copy(get_memset(), "memset", 1);
    checkasm_test_copy(get_overwrite_left(), "overwrite_left", 1);
    checkasm_test_copy(get_overwrite_right(), "overwrite_right", 1);
    checkasm_test_copy(get_underwrite(), "underwrite", 1);
    checkasm_test_noop(get_segfault(), "segfault");
}
