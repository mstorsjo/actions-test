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

#ifndef DAV2D_COMMON_INTOPS_H
#define DAV2D_COMMON_INTOPS_H

#include <stdint.h>

#include "common/attributes.h"

static inline int imax(const int a, const int b) {
    return a > b ? a : b;
}

static inline int imin(const int a, const int b) {
    return a < b ? a : b;
}

static inline unsigned umax(const unsigned a, const unsigned b) {
    return a > b ? a : b;
}

static inline unsigned umin(const unsigned a, const unsigned b) {
    return a < b ? a : b;
}

static inline int iclip64to32(int64_t v, const int min, const int max) {
    return v < min ? min : v > max ? max : (int) v;
}

static inline int iclip(const int v, const int min, const int max) {
    return v < min ? min : v > max ? max : v;
}

static inline int iclip_u8(const int v) {
    return iclip(v, 0, 255);
}

static inline int apply_sign(const int v, const int s) {
    return s < 0 ? -v : v;
}

static inline int apply_sign64(const int64_t v, const int64_t s) {
    return s < 0 ? -(int)v : (int)v;
}

static inline int ulog2(const unsigned v) {
    return 31 ^ clz(v);
}

static inline int u64log2(const uint64_t v) {
    return 63 ^ clzll(v);
}

static inline unsigned inv_recenter(const unsigned r, const unsigned v) {
    if (v > (r << 1))
        return v;
    else if ((v & 1) == 0)
        return (v >> 1) + r;
    else
        return r - ((v + 1) >> 1);
}

static inline unsigned popcnt(const unsigned x) {
#ifdef _MSC_VER
#if defined(_M_ARM64) || defined(__AVX__)
    // On x86/x64, this intrinsic produces a raw "popcnt" instruction, which
    // only works if running on SSE4.2 or newer, and would require us to
    // do runtime checking for the feature.
    //
    // Unconditionally use it, if we allow the compiler to use AVX anywhere
    // (if built with /arch:AVX). Also use it on ARM64, where the corresponding
    // instructions always are available.
    return __popcnt(x);
#else
    static const uint8_t table[256] = {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
    };
    return table[x & 0xff]         + table[(x >> 8) & 0xff] +
           table[(x >> 16) & 0xff] + table[(x >> 24) & 0xff];
#endif
#else
    return __builtin_popcount(x);
#endif
}

#endif /* DAV2D_COMMON_INTOPS_H */
