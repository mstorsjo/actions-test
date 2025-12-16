/*
 * Copyright © 2018-2025, VideoLAN and dav1d authors
 * Copyright © 2018-2025, Two Orioles, LLC
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

#include "config.h"

#include <stddef.h>
#include <stdint.h>

#include "common/intops.h"

#include "src/itx_1d.h"

static const int8_t dct8_kernel[4 * 4] = {
     89,  75,  50,  18,
     75, -18, -89, -50,
     50, -89,  18,  75,
     18, -50,  75, -89,
};

static const int8_t dct16_kernel[8 * 8] = {
     90,  87,  80,  70,  57,  43,  26,   9,
     87,  57,   9, -43, -80, -90, -70, -26,
     80,   9, -70, -87, -26,  57,  90,  43,
     70, -43, -87,   9,  90,  26, -80, -57,
     57, -80, -26,  90,  -9, -87,  43,  70,
     43, -90,  57,  26, -87,  70,   9, -80,
     26, -70,  90, -80,  43,   9, -57,  87,
      9, -26,  43, -57,  70, -80,  87, -90,
};

static const int8_t dct32_kernel[16 * 16] = {
     90,  90,  88,  85,  82,  78,  73,  67,  61,  54,  47,  39,  30,  22,  13,   4,
     90,  82,  67,  47,  22,  -4, -30, -54, -73, -85, -90, -88, -78, -61, -39, -13,
     88,  67,  30, -13, -54, -82, -90, -78, -47,  -4,  39,  73,  90,  85,  61,  22,
     85,  47, -13, -67, -90, -73, -22,  39,  82,  88,  54,  -4, -61, -90, -78, -30,
     82,  22, -54, -90, -61,  13,  78,  85,  30, -47, -90, -67,   4,  73,  88,  39,
     78,  -4, -82, -73,  13,  85,  67, -22, -88, -61,  30,  90,  54, -39, -90, -47,
     73, -30, -90, -22,  78,  67, -39, -90, -13,  82,  61, -47, -88,  -4,  85,  54,
     67, -54, -78,  39,  85, -22, -90,   4,  90,  13, -88, -30,  82,  47, -73, -61,
     61, -73, -47,  82,  30, -88, -13,  90,  -4, -90,  22,  85, -39, -78,  54,  67,
     54, -85,  -4,  88, -47, -61,  82,  13, -90,  39,  67, -78, -22,  90, -30, -73,
     47, -90,  39,  54, -90,  30,  61, -88,  22,  67, -85,  13,  73, -82,   4,  78,
     39, -88,  73,  -4, -67,  90, -47, -30,  85, -78,  13,  61, -90,  54,  22, -82,
     30, -78,  90, -61,   4,  54, -88,  82, -39, -22,  73, -90,  67, -13, -47,  85,
     22, -61,  85, -90,  73, -39,  -4,  47, -78,  90, -82,  54, -13, -30,  67, -88,
     13, -39,  61, -78,  88, -90,  85, -73,  54, -30,   4,  22, -47,  67, -82,  90,
      4, -13,  22, -30,  39, -47,  54, -61,  67, -73,  78, -82,  85, -88,  90, -90,
};

static const int8_t adst_kernel_sz4[4][4] = {
    {  18,  50,  75,  89 },
    {  50,  89,  18, -75 },
    {  75,  18, -89,  50 },
    {  89, -75,  50, -18 },
};

static const int8_t adst_kernel_sz8[8][8] = {
    {  11,  28,  44,  58,  70,  79,  86,  89 },
    {  34,  74,  89,  76,  39, -12, -58, -86 },
    {  54,  89,  48, -34, -87, -66,  12,  79 },
    {  71,  68, -41, -86,   1,  87,  38, -70 },
    {  84,  17, -89,  10,  86, -35, -75,  58 },
    {  88, -44, -44,  88, -44, -44,  88, -44 },
    {  79, -83,  50,   6, -59,  86, -74,  29 },
    {  50, -69,  81, -84,  78, -62,  40, -14 },
};

static const int8_t adst_kernel_sz16[16][16] = {
    {   8,  17,  25,  33,  41,  48,  55,  62,  67,  73,  77,  81,  84,  87,  88,  89 },
    {  25,  48,  67,  81,  88,  88,  81,  67,  48,  25,   0, -25, -48, -67, -81, -88 },
    {  41,  73,  88,  84,  62,  25, -17, -55, -81, -89, -77, -48,  -8,  33,  67,  87 },
    {  55,  87,  81,  41, -17, -67, -89, -73, -25,  33,  77,  88,  62,   8, -48, -84 },
    {  67,  88,  48, -25, -81, -81, -25,  48,  88,  67,   0, -67, -88, -48,  25,  81 },
    {  77,  77,   0, -77, -77,   0,  77,  77,   0, -77, -77,   0,  77,  77,   0, -77 },
    {  84,  55, -48, -87,  -8,  81,  62, -41, -88, -17,  77,  67, -33, -89, -25,  73 },
    {  88,  25, -81, -48,  67,  67, -48, -81,  25,  88,   0, -88, -25,  81,  48, -67 },
    {  89,  -8, -88,  17,  87, -25, -84,  33,  81, -41, -77,  48,  73, -55, -67,  62 },
    {  87, -41, -67,  73,  33, -88,   8,  84, -48, -62,  77,  25, -89,  17,  81, -55 },
    {  81, -67, -25,  88, -48, -48,  88, -25, -67,  81,   0, -81,  67,  25, -88,  48 },
    {  73, -84,  25,  55, -89,  48,  33, -87,  67,   8, -77,  81, -17, -62,  88, -41 },
    {  62, -89,  67,  -8, -55,  88, -73,  17,  48, -87,  77, -25, -41,  84, -81,  33 },
    {  48, -81,  88, -67,  25,  25, -67,  88, -81,  48,   0, -48,  81, -88,  67, -25 },
    {  33, -62,  81, -89,  84, -67,  41,  -8, -25,  55, -77,  88, -87,  73, -48,  17 },
    {  17, -33,  48, -62,  73, -81,  87, -89,  88, -84,  77, -67,  55, -41,  25,  -8 },
};

static const int8_t flipadst_kernel_sz4[4][4] = {
    {  89,  75,  50,  18 },
    {  75, -18, -89, -50 },
    {  50, -89,  18,  75 },
    {  18, -50,  75, -89 },
};

static const int8_t flipadst_kernel_sz8[8][8] = {
    {  89,  86,  79,  70,  58,  44,  28,  11 },
    { -86, -58, -12,  39,  76,  89,  74,  34 },
    {  79,  12, -66, -87, -34,  48,  89,  54 },
    { -70,  38,  87,   1, -86, -41,  68,  71 },
    {  58, -75, -35,  86,  10, -89,  17,  84 },
    { -44,  88, -44, -44,  88, -44, -44,  88 },
    {  29, -74,  86, -59,   6,  50, -83,  79 },
    { -14,  40, -62,  78, -84,  81, -69,  50 },
};

static const int8_t flipadst_kernel_sz16[16][16] = {
    {  89,  88,  87,  84,  81,  77,  73,  67,  62,  55,  48,  41,  33,  25,  17,   8 },
    {  88,  81,  67,  48,  25,   0, -25, -48, -67, -81, -88, -88, -81, -67, -48, -25 },
    {  87,  67,  33,  -8, -48, -77, -89, -81, -55, -17,  25,  62,  84,  88,  73,  41 },
    {  84,  48,  -8, -62, -88, -77, -33,  25,  73,  89,  67,  17, -41, -81, -87, -55 },
    {  81,  25, -48, -88, -67,   0,  67,  88,  48, -25, -81, -81, -25,  48,  88,  67 },
    {  77,   0, -77, -77,   0,  77,  77,   0, -77, -77,   0,  77,  77,   0, -77, -77 },
    {  73, -25, -89, -33,  67,  77, -17, -88, -41,  62,  81,  -8, -87, -48,  55,  84 },
    {  67, -48, -81,  25,  88,   0, -88, -25,  81,  48, -67, -67,  48,  81, -25, -88 },
    {  62, -67, -55,  73,  48, -77, -41,  81,  33, -84, -25,  87,  17, -88,  -8,  89 },
    {  55, -81, -17,  89, -25, -77,  62,  48, -84,  -8,  88, -33, -73,  67,  41, -87 },
    {  48, -88,  25,  67, -81,   0,  81, -67, -25,  88, -48, -48,  88, -25, -67,  81 },
    {  41, -88,  62,  17, -81,  77,  -8, -67,  87, -33, -48,  89, -55, -25,  84, -73 },
    {  33, -81,  84, -41, -25,  77, -87,  48,  17, -73,  88, -55, -8,   67, -89,  62 },
    {  25, -67,  88, -81,  48,   0, -48,  81, -88,  67, -25, -25,  67, -88,  81, -48 },
    {  17, -48,  73, -87,  88, -77,  55, -25,  -8,  41, -67,  84, -89,  81, -62,  33 },
    {   8, -25,  41, -55,  67, -77,  84, -88,  89, -87,  81, -73,  62, -48,  33, -17 },
};

static const int8_t ddt_kernel_sz8[8][8] = {
    {   4,   7,  15,  33,  65,  98, 106,  80 },
    {   6,  14,  36,  77, 100,  45, -57, -98 },
    {  22,  48,  85,  88,   0, -86, -23,  82 },
    {  57,  94,  76, -26, -73,  34,  54, -66 },
    {  96,  73, -43, -69,  55,  20, -71,  53 },
    { 103, -17, -80,  56,  15, -66,  75, -41 },
    {  78, -79,   7,  56, -82,  79, -56,  26 },
    {  56, -96,  98, -77,  54, -33,  19,  -6 },
};

static const int8_t fddt_kernel_sz8[8][8] = {
    {  80, 106,  98,  65,  33,  15,   7,   4 },
    { -98, -57,  45, 100,  77,  36,  14,   6 },
    {  82, -23, -86,   0,  88,  85,  48,  22 },
    { -66,  54,  34, -73, -26,  76,  94,  57 },
    {  53, -71,  20,  55, -69, -43,  73,  96 },
    { -41,  75, -66,  15,  56, -80, -17, 103 },
    {  26, -56,  79, -82,  56,   7, -79,  78 },
    {  -6,  19, -33,  54, -77,  98, -96,  56 },
};

static const int8_t ddt_kernel_sz16[16][16] = {
    {  12,  15,  19,   23,  30,  39,   51,  66,
       78,  88,  94,   97,  93,  83,   68,  50 },
    {  17,  23,  30,   38,  48,  61,   76,  87,
       83,  59,  19,  -30, -73, -99,  -99, -76 },
    {  37,  49,  60,   69,  75,  75,   61,  29,
      -18, -67, -96,  -83, -28,  40,   84,  83 },
    {  45,  60,  69,   73,  66,  40,   -8, -65,
      -91, -57,  21,   86,  81,  8,   -69, -90 },
    {  47,  60,  61,   49,  19, -29,  -77, -83,
      -16,  75,  93,    3, -92, -74,   32,  97 },
    {  60,  74,  64,   28, -31, -87,  -82,   4,
       88,  54, -55,  -77,  29,  88,    3, -86 },
    {  64,  70,  40,  -19, -79, -78,   11,  92,
       28, -85, -41,   82,  39, -83,  -37,  83 },
    {  82,  73,   3,  -80, -91,  10,   94,  18,
      -84,  -5,  80,  -17, -70,  47,   55, -68 },
    {  89,  48, -53,  -96,  -5,  89,   16, -83,
       12,  75, -51,  -43,  81, -14,  -75,  67 },
    { 100,   9, -99,  -45,  84,  36,  -81,   4,
       73, -60, -17,   76, -55, -21,   81, -56 },
    {  92, -35, -91,   42,  71, -69,  -22,  85,
      -60, -17,  77,  -70,  11,  56,  -83,  49 },
    {  84, -71, -46,   88, -16, -67,   79, -22,
      -46,  84, -68,   15,  46, -83,   82, -40 },
    {  69, -83,   2,   75, -78,  18,   50, -85,
       81, -43,  -6,   53, -81,  88,  -69,  32 },
    {  50, -79,  47,   14, -60,  67,  -37,  -6,
       49, -80,  98,  -99,  90, -71,   48, -19 },
    {  51, -89,  73,  -17, -45,  89, -103,  97,
      -83,  71, -56,   44, -31,  22,  -11,   5 },
    {  44, -95, 124, -126, 108, -81,   54, -30,
       16,  -6,   1,    3,  -4,   5,   -3,   2 },
};

static const int8_t fddt_kernel_sz16[16][16] = {
    {  50,   68,  83,  93,   97,  94,  88,  78,
       66,   51,  39,  30,   23,  19,  15,  12 },
    { -76,  -99, -99, -73,  -30,  19,  59,  83,
       87,   76,  61,  48,   38,  30,  23,  17 },
    {  83,   84,  40, -28,  -83, -96, -67, -18,
       29,   61,  75,  75,   69,  60,  49,  37 },
    { -90,  -69,   8,  81,   86,  21, -57, -91,
      -65,   -8,  40,  66,   73,  69,  60,  45 },
    {  97,   32, -74, -92,    3,  93,  75, -16,
      -83,  -77, -29,  19,   49,  61,  60,  47 },
    { -86,    3,  88,  29,  -77, -55,  54,  88,
        4,  -82, -87, -31,   28,  64,  74,  60 },
    {  83,  -37, -83,  39,   82, -41, -85,  28,
       92,   11, -78, -79,  -19,  40,  70,  64 },
    { -68,   55,  47, -70,  -17,  80,  -5, -84,
       18,   94,  10, -91,  -80,   3,  73,  82 },
    {  67,  -75, -14,  81,  -43, -51,  75,  12,
      -83,   16,  89,  -5,  -96, -53,  48,  89 },
    { -56,   81, -21, -55,   76, -17, -60,  73,
        4,  -81,  36,  84,  -45, -99,   9, 100 },
    {  49,  -83,  56,  11,  -70,  77, -17, -60,
       85,  -22, -69,  71,   42, -91, -35,  92 },
    { -40,   82, -83,  46,   15, -68,  84, -46,
      -22,   79, -67, -16,   88, -46, -71,  84 },
    {  32,  -69,  88, -81,   53,  -6, -43,  81,
      -85,   50,  18, -78,   75,   2, -83,  69 },
    { -19,   48, -71,  90,  -99,  98, -80,  49,
       -6,  -37,  67, -60,   14,  47, -79,  50 },
    {   5,  -11,  22, -31,   44, -56,  71, -83,
       97, -103,  89, -45,  -17,  73, -89,  51 },
    {   2,   -3,   5,  -4,    3,   1,  -6,  16,
      -30,   54, -81, 108, -126, 124, -95,  44 },
};

static NOINLINE void inv_dct_1d_c(int32_t *const c, const ptrdiff_t stride,
                                  const int8_t *mat, const int n) {
    int32_t a[16], b[16];
    const int k = n * 2 - 1;
    assert(stride > 0);

    for (int i = 0; i < n; i++) {
        int sum = 0;
        for (int j = 1; j <= k; j += 2)
            sum += *mat++ * c[j * stride];
        a[i] = c[i * 2 * stride];
        b[i] = sum;
    }

    for (int i = 0; i < n; i++) {
        c[(i    ) * stride] = a[i] + b[i];
        c[(k - i) * stride] = a[i] - b[i];
    }
}

static NOINLINE void inv_dct4_1d_c(int32_t *const c, const ptrdiff_t stride) {
    const int a0 = c[0 * stride] * 64 + c[2 * stride] * 64;
    const int a1 = c[0 * stride] * 64 - c[2 * stride] * 64;
    const int b0 = c[1 * stride] * 83 + c[3 * stride] * 35;
    const int b1 = c[1 * stride] * 35 - c[3 * stride] * 83;

    c[0 * stride] = a0 + b0;
    c[1 * stride] = a1 + b1;
    c[2 * stride] = a1 - b1;
    c[3 * stride] = a0 - b0;
}

static NOINLINE void inv_dct8_1d_c(int32_t *const c, const ptrdiff_t stride) {
    inv_dct4_1d_c(c, 2 * stride);
    inv_dct_1d_c(c, stride, dct8_kernel, 4);
}

static NOINLINE void inv_dct16_1d_c(int32_t *const c, const ptrdiff_t stride) {
    inv_dct8_1d_c(c, 2 * stride);
    inv_dct_1d_c(c, stride, dct16_kernel, 8);
}

static void inv_dct32_1d_c(int32_t *const c, const ptrdiff_t stride) {
    inv_dct16_1d_c(c, 2 * stride);
    inv_dct_1d_c(c, stride, dct32_kernel, 16);
}

static NOINLINE void inv_dst4_1d(int32_t *const c, const ptrdiff_t stride,
                                 const int8_t (*const mat)[4])
{
    int32_t sums[4];
    assert(stride > 0);

    for (int i = 0; i < 4; i++) {
        int sum = 0;
        for (int j = 0; j < 4; j++) {
            sum += mat[j][i] * c[j * stride];
        }
        sums[i] = sum;
    }

    for (int i = 0; i < 4; i++) {
        c[i * stride] = sums[i];
    }
}

static NOINLINE void inv_dst8_1d(int32_t *const c, const ptrdiff_t stride,
                                 const int8_t (*const mat)[8])
{
    int32_t sums[8];
    assert(stride > 0);

    for (int i = 0; i < 8; i++) {
        int sum = 0;
        for (int j = 0; j < 8; j++) {
            sum += mat[j][i] * c[j * stride];
        }
        sums[i] = sum;
    }

    for (int i = 0; i < 8; i++) {
        c[i * stride] = sums[i];
    }
}

static NOINLINE void inv_dst16_1d(int32_t *const c, const ptrdiff_t stride,
                                  const int8_t (*const mat)[16])
{
    int32_t sums[16];
    assert(stride > 0);

    for (int i = 0; i < 16; i++) {
        int sum = 0;
        for (int j = 0; j < 16; j++) {
            sum += mat[j][i] * c[j * stride];
        }
        sums[i] = sum;
    }

    for (int i = 0; i < 16; i++) {
        c[i * stride] = sums[i];
    }
}

#define inv_adst_1d(sz) \
static void inv_adst##sz##_1d_c(int32_t *const c, const ptrdiff_t stride) { \
    inv_dst##sz##_1d(c, stride, adst_kernel_sz##sz); \
} \
static void inv_flipadst##sz##_1d_c(int32_t *const c, const ptrdiff_t stride) { \
    inv_dst##sz##_1d(c, stride, flipadst_kernel_sz##sz); \
}

inv_adst_1d( 4)
inv_adst_1d( 8)
inv_adst_1d(16)

#undef inv_adst_1d

#define inv_ddt_1d(sz) \
static void inv_ddtx##sz##_1d_c(int32_t *const c, const ptrdiff_t stride) { \
    inv_dst##sz##_1d(c, stride, ddt_kernel_sz##sz); \
} \
static void inv_fddt##sz##_1d_c(int32_t *const c, const ptrdiff_t stride) { \
    inv_dst##sz##_1d(c, stride, fddt_kernel_sz##sz); \
}

inv_ddt_1d( 8)
inv_ddt_1d(16)

#undef inv_ddt_1d

static void inv_identity4_1d_c(int32_t *const c, const ptrdiff_t stride) {
    assert(stride > 0);
    for (int i = 0; i < 4; i++) {
        c[stride * i] *= 128;
    }
}

static void inv_identity8_1d_c(int32_t *const c, const ptrdiff_t stride) {
    assert(stride > 0);
    for (int i = 0; i < 8; i++) {
        c[stride * i] *= 181;
    }
}

static void inv_identity16_1d_c(int32_t *const c, const ptrdiff_t stride) {
    assert(stride > 0);
    for (int i = 0; i < 16; i++) {
        c[stride * i] *= 256;
    }
}

static void inv_identity32_1d_c(int32_t *const c, const ptrdiff_t stride) {
    assert(stride > 0);
    for (int i = 0; i < 32; i++)
        c[stride * i] *= 362;
}

const itx_1d_fn dav1d_tx1d_fns[N_TX_SIZES][N_TX_1D_TYPES] = {
    [TX_4X4] = {
        [DCT] = inv_dct4_1d_c,
        [ADST] = inv_adst4_1d_c,
        [FLIPADST] = inv_flipadst4_1d_c,
        [IDENTITY] = inv_identity4_1d_c,
    }, [TX_8X8] = {
        [DCT] = inv_dct8_1d_c,
        [ADST] = inv_adst8_1d_c,
        [FLIPADST] = inv_flipadst8_1d_c,
        [IDENTITY] = inv_identity8_1d_c,
        [DDT] = inv_ddtx8_1d_c,
        [FDDT] = inv_fddt8_1d_c,
    }, [TX_16X16] = {
        [DCT] = inv_dct16_1d_c,
        [ADST] = inv_adst16_1d_c,
        [FLIPADST] = inv_flipadst16_1d_c,
        [IDENTITY] = inv_identity16_1d_c,
        [DDT] = inv_ddtx16_1d_c,
        [FDDT] = inv_fddt16_1d_c,
    }, [TX_32X32] = {
        [DCT] = inv_dct32_1d_c,
        [IDENTITY] = inv_identity32_1d_c,
    }, [TX_64X64] = {
        [DCT] = inv_dct32_1d_c,
    },
};

void dav1d_inv_wht4_1d_c(int32_t *const c, const ptrdiff_t stride) {
    assert(stride > 0);
    const int in0 = c[0 * stride], in1 = c[1 * stride];
    const int in2 = c[2 * stride], in3 = c[3 * stride];

    const int t0 = in0 + in1;
    const int t2 = in2 - in3;
    const int t4 = (t0 - t2) >> 1;
    const int t3 = t4 - in3;
    const int t1 = t4 - in1;

    c[0 * stride] = t0 - t3;
    c[1 * stride] = t3;
    c[2 * stride] = t1;
    c[3 * stride] = t2 + t1;
}
