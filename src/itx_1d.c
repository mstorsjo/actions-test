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

static const int8_t tx_kernel_dct2_size4[4][4] = {
    {  64,  83,  64,  35 },
    {  64,  35, -64, -83 },
    {  64, -35, -64,  83 },
    {  64, -83,  64, -35 },
};

static const int8_t tx_kernel_dct2_size8[8][8] = {
    {  64,  89,  83,  75,  64,  50,  35,  18 },
    {  64,  75,  35, -18, -64, -89, -83, -50 },
    {  64,  50, -35, -89, -64,  18,  83,  75 },
    {  64,  18, -83, -50,  64,  75, -35, -89 },
    {  64, -18, -83,  50,  64, -75, -35,  89 },
    {  64, -50, -35,  89, -64, -18,  83, -75 },
    {  64, -75,  35,  18, -64,  89, -83,  50 },
    {  64, -89,  83, -75,  64, -50,  35, -18 },
};

static const int8_t tx_kernel_dct2_size16[16][16] = {
    {  64,  90,  89,  87,  83,  80,  75,  70,  64,  57,  50,  43,  35,  26,  18,   9 },
    {  64,  87,  75,  57,  35,   9, -18, -43, -64, -80, -89, -90, -83, -70, -50, -26 },
    {  64,  80,  50,   9, -35, -70, -89, -87, -64, -26,  18,  57,  83,  90,  75,  43 },
    {  64,  70,  18, -43, -83, -87, -50,   9,  64,  90,  75,  26, -35, -80, -89, -57 },
    {  64,  57, -18, -80, -83, -26,  50,  90,  64,  -9, -75, -87, -35,  43,  89,  70 },
    {  64,  43, -50, -90, -35,  57,  89,  26, -64, -87, -18,  70,  83,   9, -75, -80 },
    {  64,  26, -75, -70,  35,  90,  18, -80, -64,  43,  89,   9, -83, -57,  50,  87 },
    {  64,   9, -89, -26,  83,  43, -75, -57,  64,  70, -50, -80,  35,  87, -18, -90 },
    {  64,  -9, -89,  26,  83, -43, -75,  57,  64, -70, -50,  80,  35, -87, -18,  90 },
    {  64, -26, -75,  70,  35, -90,  18,  80, -64, -43,  89,  -9, -83,  57,  50, -87 },
    {  64, -43, -50,  90, -35, -57,  89, -26, -64,  87, -18, -70,  83,  -9, -75,  80 },
    {  64, -57, -18,  80, -83,  26,  50, -90,  64,   9, -75,  87, -35, -43,  89, -70 },
    {  64, -70,  18,  43, -83,  87, -50,  -9,  64, -90,  75, -26, -35,  80, -89,  57 },
    {  64, -80,  50,  -9, -35,  70, -89,  87, -64,  26,  18, -57,  83, -90,  75, -43 },
    {  64, -87,  75, -57,  35,  -9, -18,  43, -64,  80, -89,  90, -83,  70, -50,  26 },
    {  64, -90,  89, -87,  83, -80,  75, -70,  64, -57,  50, -43,  35, -26,  18,  -9 },
};

static const int8_t tx_kernel_dct2_size32[32][32] = {
    {  64,  90,  90,  90,  89,  88,  87,  85,  83,  82,  80,  78,  75,  73,  70,  67,
       64,  61,  57,  54,  50,  47,  43,  39,  35,  30,  26,  22,  18,  13,   9,   4 },
    {  64,  90,  87,  82,  75,  67,  57,  47,  35,  22,   9,  -4, -18, -30, -43, -54,
      -64, -73, -80, -85, -89, -90, -90, -88, -83, -78, -70, -61, -50, -39, -26, -13 },
    {  64,  88,  80,  67,  50,  30,   9, -13, -35, -54, -70, -82, -89, -90, -87, -78,
      -64, -47, -26,  -4,  18,  39,  57,  73,  83,  90,  90,  85,  75,  61,  43,  22 },
    {  64,  85,  70,  47,  18, -13, -43, -67, -83, -90, -87, -73, -50, -22,   9,  39,
       64,  82,  90,  88,  75,  54,  26,  -4, -35, -61, -80, -90, -89, -78, -57, -30 },
    {  64,  82,  57,  22, -18, -54, -80, -90, -83, -61, -26,  13,  50,  78,  90,  85,
       64,  30,  -9, -47, -75, -90, -87, -67, -35,   4,  43,  73,  89,  88,  70,  39 },
    {  64,  78,  43,  -4, -50, -82, -90, -73, -35,  13,  57,  85,  89,  67,  26, -22,
      -64, -88, -87, -61, -18,  30,  70,  90,  83,  54,   9, -39, -75, -90, -80, -47 },
    {  64,  73,  26, -30, -75, -90, -70, -22,  35,  78,  90,  67,  18, -39, -80, -90,
      -64, -13,  43,  82,  89,  61,   9, -47, -83, -88, -57,  -4,  50,  85,  87,  54 },
    {  64,  67,   9, -54, -89, -78, -26,  39,  83,  85,  43, -22, -75, -90, -57,   4,
       64,  90,  70,  13, -50, -88, -80, -30,  35,  82,  87,  47, -18, -73, -90, -61 },
    {  64,  61,  -9, -73, -89, -47,  26,  82,  83,  30, -43, -88, -75, -13,  57,  90,
       64,  -4, -70, -90, -50,  22,  80,  85,  35, -39, -87, -78, -18,  54,  90,  67 },
    {  64,  54, -26, -85, -75,  -4,  70,  88,  35, -47, -90, -61,  18,  82,  80,  13,
      -64, -90, -43,  39,  89,  67,  -9, -78, -83, -22,  57,  90,  50, -30, -87, -73 },
    {  64,  47, -43, -90, -50,  39,  90,  54, -35, -90, -57,  30,  89,  61, -26, -88,
      -64,  22,  87,  67, -18, -85, -70,  13,  83,  73,  -9, -82, -75,   4,  80,  78 },
    {  64,  39, -57, -88, -18,  73,  80,  -4, -83, -67,  26,  90,  50, -47, -90, -30,
       64,  85,   9, -78, -75,  13,  87,  61, -35, -90, -43,  54,  89,  22, -70, -82 },
    {  64,  30, -70, -78,  18,  90,  43, -61, -83,   4,  87,  54, -50, -88,  -9,  82,
       64, -39, -90, -22,  75,  73, -26, -90, -35,  67,  80, -13, -89, -47,  57,  85 },
    {  64,  22, -80, -61,  50,  85,  -9, -90, -35,  73,  70, -39, -89,  -4,  87,  47,
      -64, -78,  26,  90,  18, -82, -57,  54,  83, -13, -90, -30,  75,  67, -43, -88 },
    {  64,  13, -87, -39,  75,  61, -57, -78,  35,  88,  -9, -90, -18,  85,  43, -73,
      -64,  54,  80, -30, -89,   4,  90,  22, -83, -47,  70,  67, -50, -82,  26,  90 },
    {  64,   4, -90, -13,  89,  22, -87, -30,  83,  39, -80, -47,  75,  54, -70, -61,
       64,  67, -57, -73,  50,  78, -43, -82,  35,  85, -26, -88,  18,  90,  -9, -90 },
    {  64,  -4, -90,  13,  89, -22, -87,  30,  83, -39, -80,  47,  75, -54, -70,  61,
       64, -67, -57,  73,  50, -78, -43,  82,  35, -85, -26,  88,  18, -90,  -9,  90 },
    {  64, -13, -87,  39,  75, -61, -57,  78,  35, -88,  -9,  90, -18, -85,  43,  73,
      -64, -54,  80,  30, -89,  -4,  90, -22, -83,  47,  70, -67, -50,  82,  26, -90 },
    {  64, -22, -80,  61,  50, -85,  -9,  90, -35, -73,  70,  39, -89,   4,  87, -47,
      -64,  78,  26, -90,  18,  82, -57, -54,  83,  13, -90,  30,  75, -67, -43,  88 },
    {  64, -30, -70,  78,  18, -90,  43,  61, -83,  -4,  87, -54, -50,  88,  -9, -82,
       64,  39, -90,  22,  75, -73, -26,  90, -35, -67,  80,  13, -89,  47,  57, -85 },
    {  64, -39, -57,  88, -18, -73,  80,   4, -83,  67,  26, -90,  50,  47, -90,  30,
       64, -85,   9,  78, -75, -13,  87, -61, -35,  90, -43, -54,  89, -22, -70,  82 },
    {  64, -47, -43,  90, -50, -39,  90, -54, -35,  90, -57, -30,  89, -61, -26,  88,
      -64, -22,  87, -67, -18,  85, -70, -13,  83, -73,  -9,  82, -75,  -4,  80, -78 },
    {  64, -54, -26,  85, -75,   4,  70, -88,  35,  47, -90,  61,  18, -82,  80, -13,
      -64,  90, -43, -39,  89, -67,  -9,  78, -83,  22,  57, -90,  50,  30, -87,  73 },
    {  64, -61,  -9,  73, -89,  47,  26, -82,  83, -30, -43,  88, -75,  13,  57, -90,
       64,   4, -70,  90, -50, -22,  80, -85,  35,  39, -87,  78, -18, -54,  90, -67 },
    {  64, -67,   9,  54, -89,  78, -26, -39,  83, -85,  43,  22, -75,  90, -57,  -4,
       64, -90,  70, -13, -50,  88, -80,  30,  35, -82,  87, -47, -18,  73, -90,  61 },
    {  64, -73,  26,  30, -75,  90, -70,  22,  35, -78,  90, -67,  18,  39, -80,  90,
      -64,  13,  43, -82,  89, -61,   9,  47, -83,  88, -57,   4,  50, -85,  87, -54 },
    {  64, -78,  43,   4, -50,  82, -90,  73, -35, -13,  57, -85,  89, -67,  26,  22,
      -64,  88, -87,  61, -18, -30,  70, -90,  83, -54,   9,  39, -75,  90, -80,  47 },
    {  64, -82,  57, -22, -18,  54, -80,  90, -83,  61, -26, -13,  50, -78,  90, -85,
       64, -30,  -9,  47, -75,  90, -87,  67, -35,  -4,  43, -73,  89, -88,  70, -39 },
    {  64, -85,  70, -47,  18,  13, -43,  67, -83,  90, -87,  73, -50,  22,   9, -39,
       64, -82,  90, -88,  75, -54,  26,   4, -35,  61, -80,  90, -89,  78, -57,  30 },
    {  64, -88,  80, -67,  50, -30,   9,  13, -35,  54, -70,  82, -89,  90, -87,  78,
      -64,  47, -26,   4,  18, -39,  57, -73,  83, -90,  90, -85,  75, -61,  43, -22 },
    {  64, -90,  87, -82,  75, -67,  57, -47,  35, -22,   9,   4, -18,  30, -43,  54,
      -64,  73, -80,  85, -89,  90, -90,  88, -83,  78, -70,  61, -50,  39, -26,  13 },
    {  64, -90,  90, -90,  89, -88,  87, -85,  83, -82,  80, -78,  75, -73,  70, -67,
       64, -61,  57, -54,  50, -47,  43, -39,  35, -30,  26, -22,  18, -13,   9,  -4 },
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

static void inv_dct4_1d_c(int32_t *const c, const ptrdiff_t stride) {
    int32_t odd[2], stage1[2];
    const int8_t (*const mat)[4] = tx_kernel_dct2_size4;
    assert(stride > 0);

    odd[0]    = mat[0][1] * c[1 * stride] + mat[0][3] * c[3 * stride];
    odd[1]    = mat[1][1] * c[1 * stride] + mat[1][3] * c[3 * stride];
    stage1[0] = mat[0][0] * c[0 * stride] + mat[0][2] * c[2 * stride];
    stage1[1] = mat[1][0] * c[0 * stride] + mat[1][2] * c[2 * stride];

    c[0 * stride] = stage1[0] + odd[0];
    c[1 * stride] = stage1[1] + odd[1];
    c[2 * stride] = stage1[1] - odd[1];
    c[3 * stride] = stage1[0] - odd[0];
}

static void inv_dct8_1d_c(int32_t *const c, const ptrdiff_t stride) {
    int32_t odd[4], quarter[2], stage1[4], dc_nyquist[2];
    const int8_t (*const mat)[8] = tx_kernel_dct2_size8;
    assert(stride > 0);

    for (int i = 0; i < 4; i++) {
        int sum = 0;
        for (int j = 1; j < 8; j += 2) {
            sum += mat[i][j] * c[j * stride];
        }
        odd[i] = sum;
    }

    quarter[0]    = mat[0][2] * c[2 * stride] + mat[0][6] * c[6 * stride];
    quarter[1]    = mat[1][2] * c[2 * stride] + mat[1][6] * c[6 * stride];
    dc_nyquist[0] = mat[0][0] * c[0 * stride] + mat[0][4] * c[4 * stride];
    dc_nyquist[1] = mat[1][0] * c[0 * stride] + mat[1][4] * c[4 * stride];

    stage1[0] = dc_nyquist[0] + quarter[0];
    stage1[1] = dc_nyquist[1] + quarter[1];
    stage1[2] = dc_nyquist[1] - quarter[1];
    stage1[3] = dc_nyquist[0] - quarter[0];

    for (int i = 0; i < 4; i++) {
        c[(i    ) * stride] = stage1[i] + odd[i];
        c[(7 - i) * stride] = stage1[i] - odd[i];
    }
}

static void inv_dct16_1d_c(int32_t *const c, const ptrdiff_t stride) {
    int32_t odd[8], quarter[4], eighth[2];
    int32_t stage2[8], stage1[4], dc_nyquist[2];
    const int8_t (*const mat)[16] = tx_kernel_dct2_size16;
    assert(stride > 0);

    for (int i = 0; i < 8; i++) {
        int sum = 0;
        for (int j = 1; j < 16; j += 2) {
            sum += mat[i][j] * c[j * stride];
        }
        odd[i] = sum;
    }

    for (int i = 0; i < 4; i++) {
        int sum = 0;
        for (int j = 2; j < 16; j += 4) {
            sum += mat[i][j] * c[j * stride];
        }
        quarter[i] = sum;
    }

    eighth[0]     = mat[0][4] * c[4 * stride] + mat[0][12] * c[12 * stride];
    eighth[1]     = mat[1][4] * c[4 * stride] + mat[1][12] * c[12 * stride];
    dc_nyquist[0] = mat[0][0] * c[0 * stride] + mat[0][ 8] * c[ 8 * stride];
    dc_nyquist[1] = mat[1][0] * c[0 * stride] + mat[1][ 8] * c[ 8 * stride];

    stage1[0] = dc_nyquist[0] + eighth[0];
    stage1[1] = dc_nyquist[1] + eighth[1];
    stage1[2] = dc_nyquist[1] - eighth[1];
    stage1[3] = dc_nyquist[0] - eighth[0];

    for (int i = 0; i < 4; i++) {
        stage2[i    ] = stage1[i] + quarter[i];
        stage2[7 - i] = stage1[i] - quarter[i];
    }

    for (int i = 0; i < 8; i++) {
        c[(i     ) * stride] = stage2[i] + odd[i];
        c[(15 - i) * stride] = stage2[i] - odd[i];
    }
}

static NOINLINE void idct32_1d(const int32_t *const c, const ptrdiff_t stride,
                               int32_t odd[16], int32_t stage3[16])
{
    int32_t quarter[8], eighth[4], sixteenth[2];
    int32_t stage2[8], stage1[4], dc_nyquist[2];
    const int8_t (*const mat)[32] = tx_kernel_dct2_size32;

    for (int i = 0; i < 16; i++) {
        int sum = 0;
        for (int j = 1; j < 32; j += 2) {
            sum += mat[i][j] * c[j * stride];
        }
        odd[i] = sum;
    }

    for (int i = 0; i < 8; i++) {
        int sum = 0;
        for (int j = 2; j < 32; j += 4) {
            sum += mat[i][j] * c[j * stride];
        }
        quarter[i] = sum;
    }

    for (int i = 0; i < 4; i++) {
        int sum = 0;
        for (int j = 4; j < 32; j += 8) {
            sum += mat[i][j] * c[j * stride];
        }
        eighth[i] = sum;
    }

    sixteenth[0]  = mat[0][8] * c[8 * stride] + mat[0][24] * c[24 * stride];
    sixteenth[1]  = mat[1][8] * c[8 * stride] + mat[1][24] * c[24 * stride];
    dc_nyquist[0] = mat[0][0] * c[0 * stride] + mat[0][16] * c[16 * stride];
    dc_nyquist[1] = mat[1][0] * c[0 * stride] + mat[1][16] * c[16 * stride];

    stage1[0] = dc_nyquist[0] + sixteenth[0];
    stage1[1] = dc_nyquist[1] + sixteenth[1];
    stage1[2] = dc_nyquist[1] - sixteenth[1];
    stage1[3] = dc_nyquist[0] - sixteenth[0];

    for (int i = 0; i < 4; i++) {
        stage2[i    ] = stage1[i] + eighth[i];
        stage2[7 - i] = stage1[i] - eighth[i];
    }

    for (int i = 0; i < 8; i++) {
        stage3[i     ] = stage2[i] + quarter[i];
        stage3[15 - i] = stage2[i] - quarter[i];
    }
}

static void inv_dct32_1d_c(int32_t *const c, const ptrdiff_t stride) {
    int32_t odd[16], stage3[16];
    assert(stride > 0);

    idct32_1d(c, stride, odd, stage3);

    for (int i = 0; i < 16; i++) {
        c[(i     ) * stride] = stage3[i] + odd[i];
        c[(31 - i) * stride] = stage3[i] - odd[i];
    }
}

static void inv_dct64_1d_c(int32_t *const c, const ptrdiff_t stride) {
    int32_t odd[16], stage3[16];
    assert(stride > 0);

    idct32_1d(c, stride, odd, stage3);

    for (int i = 0; i < 16; i++) {
        const int ii = i + i;
        c[(ii +  0) * stride] =
        c[(ii +  1) * stride] = stage3[i] + odd[i];
        c[(63 - ii) * stride] =
        c[(62 - ii) * stride] = stage3[i] - odd[i];
    }
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
    }, [TX_16X16] = {
        [DCT] = inv_dct16_1d_c,
        [ADST] = inv_adst16_1d_c,
        [FLIPADST] = inv_flipadst16_1d_c,
        [IDENTITY] = inv_identity16_1d_c,
    }, [TX_32X32] = {
        [DCT] = inv_dct32_1d_c,
        [IDENTITY] = inv_identity32_1d_c,
    }, [TX_64X64] = {
        [DCT] = inv_dct64_1d_c,
    },
};

const uint8_t /* enum Tx1dType */ dav1d_tx1d_types[N_TX_TYPES][2] = {
    [DCT_DCT]           = { DCT, DCT },
    [ADST_DCT]          = { ADST, DCT },
    [DCT_ADST]          = { DCT, ADST },
    [ADST_ADST]         = { ADST, ADST },
    [FLIPADST_DCT]      = { FLIPADST, DCT },
    [DCT_FLIPADST]      = { DCT, FLIPADST },
    [FLIPADST_FLIPADST] = { FLIPADST, FLIPADST },
    [ADST_FLIPADST]     = { ADST, FLIPADST },
    [FLIPADST_ADST]     = { FLIPADST, ADST },
    [IDTX]              = { IDENTITY, IDENTITY },
    [V_DCT]             = { DCT, IDENTITY },
    [H_DCT]             = { IDENTITY, DCT },
    [V_ADST]            = { ADST, IDENTITY },
    [H_ADST]            = { IDENTITY, ADST },
    [V_FLIPADST]        = { FLIPADST, IDENTITY },
    [H_FLIPADST]        = { IDENTITY, FLIPADST },
};

#if !(HAVE_ASM && TRIM_DSP_FUNCTIONS && ( \
  ARCH_AARCH64 || \
  (ARCH_ARM && (defined(__ARM_NEON) || defined(__APPLE__) || defined(_WIN32))) \
))
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
#endif
