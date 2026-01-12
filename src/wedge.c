/*
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

#include "config.h"

#include <stdint.h>
#include <string.h>

#include "common/intops.h"

#include "src/tables.h"
#include "src/wedge.h"

Dav1dMasks dav1d_masks;

enum WedgeDirectionType {
    WEDGE_0,
    WEDGE_14,
    WEDGE_27,
    WEDGE_45,
    WEDGE_63,
    WEDGE_90,
    WEDGE_117,
    WEDGE_135,
    WEDGE_153,
    WEDGE_166,
    WEDGE_180,
    WEDGE_194,
    WEDGE_207,
    WEDGE_225,
    WEDGE_243,
    WEDGE_270,
    WEDGE_297,
    WEDGE_315,
    WEDGE_333,
    WEDGE_346,
    N_WEDGE_DIRECTIONS,
};

typedef struct {
    uint8_t /* enum WedgeDirectionType */ direction;
    uint8_t x_offset;
    uint8_t y_offset;
} wedge_code_type;

static const wedge_code_type wedge_codebook_16[68] = {
    { WEDGE_0, 5, 4 },   { WEDGE_0, 6, 4 },   { WEDGE_0, 7, 4 },
    { WEDGE_14, 4, 4 },  { WEDGE_14, 5, 4 },  { WEDGE_14, 6, 4 },
    { WEDGE_14, 7, 4 },  { WEDGE_27, 4, 4 },  { WEDGE_27, 5, 4 },
    { WEDGE_27, 6, 4 },  { WEDGE_27, 7, 4 },  { WEDGE_45, 4, 4 },
    { WEDGE_45, 5, 4 },  { WEDGE_45, 6, 4 },  { WEDGE_45, 7, 4 },
    { WEDGE_63, 4, 4 },  { WEDGE_63, 4, 3 },  { WEDGE_63, 4, 2 },
    { WEDGE_63, 4, 1 },  { WEDGE_90, 4, 3 },  { WEDGE_90, 4, 2 },
    { WEDGE_90, 4, 1 },  { WEDGE_117, 4, 4 }, { WEDGE_117, 4, 3 },
    { WEDGE_117, 4, 2 }, { WEDGE_117, 4, 1 }, { WEDGE_135, 4, 4 },
    { WEDGE_135, 3, 4 }, { WEDGE_135, 2, 4 }, { WEDGE_135, 1, 4 },
    { WEDGE_153, 4, 4 }, { WEDGE_153, 3, 4 }, { WEDGE_153, 2, 4 },
    { WEDGE_153, 1, 4 }, { WEDGE_166, 4, 4 }, { WEDGE_166, 3, 4 },
    { WEDGE_166, 2, 4 }, { WEDGE_166, 1, 4 }, { WEDGE_180, 3, 4 },
    { WEDGE_180, 2, 4 }, { WEDGE_180, 1, 4 }, { WEDGE_194, 3, 4 },
    { WEDGE_194, 2, 4 }, { WEDGE_194, 1, 4 }, { WEDGE_207, 3, 4 },
    { WEDGE_207, 2, 4 }, { WEDGE_207, 1, 4 }, { WEDGE_225, 3, 4 },
    { WEDGE_225, 2, 4 }, { WEDGE_225, 1, 4 }, { WEDGE_243, 4, 5 },
    { WEDGE_243, 4, 6 }, { WEDGE_243, 4, 7 }, { WEDGE_270, 4, 5 },
    { WEDGE_270, 4, 6 }, { WEDGE_270, 4, 7 }, { WEDGE_297, 4, 5 },
    { WEDGE_297, 4, 6 }, { WEDGE_297, 4, 7 }, { WEDGE_315, 5, 4 },
    { WEDGE_315, 6, 4 }, { WEDGE_315, 7, 4 }, { WEDGE_333, 5, 4 },
    { WEDGE_333, 6, 4 }, { WEDGE_333, 7, 4 }, { WEDGE_346, 5, 4 },
    { WEDGE_346, 6, 4 }, { WEDGE_346, 7, 4 },
};

static void copy2d(uint8_t *dst, const uint8_t *src,
                   const int w8, const int h8,
                   const int x_off, const int y_off)
{
    src += (64 - y_off * h8) * 128 + (64 - x_off * w8);
    for (int y = 0; y < h8 * 8; y++) {
        memcpy(dst, src, w8 * 8);
        src += 128;
        dst += w8 * 8;
    }
}

static void fill_tmvp(uint8_t *dst, const uint8_t *src,
                      const int w8, const int h8)
{
    for (int y = 0; y < h8; y++) {
        for (int x = 0; x < w8; x++) {
            int score[2] = { 0 };
            const uint8_t *sptr = src;
            for (int yy = y * 8; yy < y * 8 + 8; yy++) {
                for (int xx = x * 8; xx < x * 8 + 8; xx++) {
                    score[0] += sptr[xx] < 4;
                    score[1] += sptr[xx] > 60;
                }
                sptr += w8 * 8;
            }
            dst[x] = score[0] >= 60 ? 0 : score[1] >= 60 ? 1 : 2;
        }
        dst += w8;
        src += w8 * 8 * 8;
    }
}

static void gen_master(uint8_t *master, const int mul,
                       const enum WedgeDirectionType wd)
{
    static const int8_t cos_lut[N_WEDGE_DIRECTIONS] = {
        4, 4, 4, 2, 2, 0, -2, -2, -4, -4, -4, -4, -4, -2, -2, 0, 2, 2, 4, 4
    }, sin_lut[N_WEDGE_DIRECTIONS] = {
        0, -1, -2, -2, -4, -4, -4, -2, -2, -1, 0, 1, 2, 2, 4, 4, 4, 2, 2, 1
    }, weight[29] = {
        8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2,
        2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0
    };
    const int s = sin_lut[wd] * mul, c = cos_lut[wd] * mul;
    for (int y = 0; y < 128; y++) {
        const int dy = (2 * y - 127) * s;
        for (int x = 0; x < 128; x++) {
            const int d = iclip((2 * x - 127) * c + dy, -28, 28);
            master[x] = 4 * (d >= 0 ? 16 - weight[d] : weight[-d]);
        }
        master += 128;
    }
}

static COLD void init_wedge_masks(void) {
    int o = 0;
    for (enum BlockSize bs = BS_64x64; bs < N_BS_SIZES; bs++) {
        const uint8_t *const b_dim = dav1d_block_dimensions[bs];
        if (b_dim[0] == 1 || b_dim[1] == 1) continue;
        dav1d_masks.offsets.wedge[bs - BS_64x64] = o;
        o += b_dim[0] * b_dim[1] >> 2;
    }
    assert(o * 0x1100 == sizeof(dav1d_masks.wedge) && o < 256);

    uint8_t master[128 * 128];
    enum WedgeDirectionType wd = N_WEDGE_DIRECTIONS;
    for (int widx = 0; widx < 68; widx++) {
        const wedge_code_type *const cb = &wedge_codebook_16[widx];
        if (cb->direction != wd) {
            gen_master(master, 2 /* sharp edge */, cb->direction);
            wd = cb->direction;
        }
#define fill(w8, h8, sz) do { \
        uint8_t *const wm = WEDGE_MASK(BS_##sz, w8 * 2, h8 * 2, widx); \
        copy2d(wm, master, w8, h8, cb->x_offset, cb->y_offset); \
        fill_tmvp(WEDGE_TMVP(BS_##sz, w8 * 2, h8 * 2, widx), wm, w8, h8); \
    } while (0)
        fill(1, 1, 8x8);
        fill(1, 2, 8x16);
        fill(2, 1, 16x8);
        fill(2, 2, 16x16);
    }
    for (int widx = 0; widx < 68; widx++) {
        const wedge_code_type *const cb = &wedge_codebook_16[widx];
        if (cb->direction != wd) {
            gen_master(master, 1 /* soft edge */, cb->direction);
            wd = cb->direction;
        }
        fill(1, 4, 8x32);
        fill(1, 8, 8x64);
        fill(2, 4, 16x32);
        fill(2, 8, 16x64);
        fill(4, 1, 32x8);
        fill(4, 2, 32x16);
        fill(4, 4, 32x32);
        fill(4, 8, 32x64);
        fill(8, 1, 64x8);
        fill(8, 2, 64x16);
        fill(8, 4, 64x32);
        fill(8, 8, 64x64);
#undef fill
    }
}

static COLD void build_nondc_ii_masks(uint8_t *const mask_v, const int w,
                                      const int h, const int step)
{
    static const uint8_t ii_weights_1d[64] = {
         60, 56, 52, 48, 45, 42, 39, 37, 34, 32, 30, 28, 26, 24, 22, 21,
         19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 10,  9,  8,  8,  7,  7,
          6,  6,  6,  5,  5,  4,  4,  4,  4,  3,  3,  3,  3,  3,  2,  2,
          2,  2,  2,  2,  2,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
    };

    uint8_t *const mask_h  = &mask_v[w * h];
    uint8_t *const mask_sm = &mask_h[w * h];
    for (int y = 0, off = 0; y < h; y++, off += w) {
        memset(&mask_v[off], ii_weights_1d[y * step], w);
        for (int x = 0; x < w; x++) {
            mask_sm[off + x] = ii_weights_1d[imin(x, y) * step];
            mask_h[off + x] = ii_weights_1d[x * step];
        }
    }
}

static COLD void init_ii_masks(void) {
    memset(dav1d_masks.ii_dc, 32, 64 * 64);

    int o = 0;
    for (enum BlockSize bs = BS_64x64; bs < N_BS_SIZES; bs++) {
        const uint8_t *const b_dim = dav1d_block_dimensions[bs];
        if (b_dim[0] * b_dim[1] <= 2) continue;
        dav1d_masks.offsets.ii_nondc[bs - BS_64x64] = o;
        o += b_dim[0] * b_dim[1] >> 2;
    }
    assert(o * 0xc0 == sizeof(dav1d_masks.ii_nondc) && o < 256);

#define fill(w, h, s) \
    build_nondc_ii_masks(II_MASK(BS_##w##x##h, 0, 0, 1), w, h, s)
    fill( 4, 16, 4);
    fill( 4, 32, 2);
    fill( 4, 64, 1);
    fill( 8,  8, 8);
    fill( 8, 16, 4);
    fill( 8, 32, 2);
    fill( 8, 64, 1);
    fill(16,  4, 4);
    fill(16,  8, 4);
    fill(16, 16, 4);
    fill(16, 32, 2);
    fill(16, 64, 1);
    fill(32,  4, 2);
    fill(32,  8, 2);
    fill(32, 16, 2);
    fill(32, 32, 2);
    fill(32, 64, 1);
    fill(64,  4, 1);
    fill(64,  8, 1);
    fill(64, 16, 1);
    fill(64, 32, 1);
    fill(64, 64, 1);
#undef fill
}

COLD void dav1d_init_ii_wedge_masks(void) {
    // This function is guaranteed to be called only once
    init_wedge_masks();
    init_ii_masks();
}
