/*
 * Copyright © 2020, VideoLAN and dav2d authors
 * Copyright © 2020, Two Orioles, LLC
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

#ifndef DAV2D_SRC_REF_MVS_H
#define DAV2D_SRC_REF_MVS_H

#include <stdint.h>

#include "dav2d/headers.h"

#include "common/intops.h"

#include "src/debug.h"
#include "src/intra_edge.h"
#include "src/tables.h"

#define INVALID_REF2CUR (-32)
#define INVALID_TRAJ 0x8080

typedef union refmvs_traj_map {
    struct {
        int8_t y, x;
    };
    uint16_t n;
} refmvs_traj_map;

PACKED(typedef struct refmvs_sngl_mv_block {
    mv mv;
    uint8_t ref;
}) refmvs_sngl_mv_block;
CHECK_SIZE(refmvs_sngl_mv_block, 9);

PACKED(typedef union refmvs_refpair {
    int8_t ref[2]; // [0] = 0: intra=1, [1] = -1: comp=0
    uint16_t pair;
}) ALIGN(refmvs_refpair, 2);
CHECK_SIZE(refmvs_refpair, 2);

PACKED(typedef struct refmvs_temporal_block {
    union {
        union qmv {
            struct {
                int8_t y, x;
            };
            uint16_t n;
        } mv[2];
        uint32_t n;
    } mv;
    refmvs_refpair ref;
}) refmvs_temporal_block;
CHECK_SIZE(refmvs_temporal_block, 6);

// FIXME the size of this array can be reduced if we generate mv on-the-fly
// from the (separately stored) warp matrix.
// bx4/by4 can be stored in one-byte elements as relative offset to start of
// the block (see how it's used in decode.c:derive_warpmv()
PACKED(typedef struct refmvs_block {
    union mv mv[2];
    refmvs_refpair ref;
    uint8_t bs;
    int8_t mf; // bits: 0: globalmv, 1: warp[not gmv], 2-7: cwp_idx
    uint16_t bx4, by4; // top/left coordinates (in 4px units) of this block
    union mv lmv[2]; // 2dmv for warp blocks (see #1146; mf & 2)
    int32_t m[7]; // warp matrix
}) ALIGN(refmvs_block, 4);
CHECK_SIZE(refmvs_block, 68);

typedef struct refmvs_frame {
    const Dav2dSequenceHeader *seq_hdr;
    const Dav2dFrameHeader *frm_hdr;
    int iw4, ih4, iw8, ih8;
    int sbsz /* in 4px units */;
    int mfmv_sbsz8, mfmv_edge, mfmv_k_shift;
    int use_ref_frame_mvs;
    int32_t tip_sf[2];
    int8_t tip_delta;
    uint8_t ref_sign[7];
    int8_t pocdiff[7];
    uint64_t ref_flip;
    uint8_t abspocdiff[7];
    struct MfmvRef {
        uint8_t ref;
        int8_t tgt;
        uint8_t dir;
    } mfmv[4];
    int8_t mfmv_ref2cur[4];
    int8_t mfmv_ref2ref[4][7];
    int8_t mfmv_ref2idx[4][7];
    int32_t mfmv_ref2sf[4][7][2];
    int n_mfmvs;

    int n_blocks;
    refmvs_temporal_block *rp;
    ptrdiff_t rp_stride;
    /*const*/ refmvs_temporal_block *const *rp_ref;
    refmvs_sngl_mv_block *rp_proj;
    mv *rp_traj[7]; // FIXME we may not need 7?
    refmvs_traj_map *rp_map[3][7];
    refmvs_block *ra;
    int32_t (*ram)[7];
#if 0
    int n_frame_threads;
#endif
    int n_tile_threads;
} refmvs_frame;

typedef struct refmvs_tile {
    const refmvs_frame *rf;
    refmvs_sngl_mv_block *rp_proj;
    mv *rp_traj[7];
    refmvs_block *ra, ra_tl;
    refmvs_block r[64 * 64 * 2]; // one sb may be enough? smaller sb sizes than 256x256?
    struct {
        int start, end;
    } tile_col, tile_row;
    struct {
        union mv mv[9][4][2];
        int8_t cwp_idx[3 /* class-6 */][4];
        refmvs_refpair ref[4];
        uint8_t size[9], idx[9];
        uint8_t hits[2 /* sb, b */], avail;
    } bank;
    struct {
        int32_t mat[7][4][7 /* see #834 */];
        uint8_t hits, size[7], idx[7];
    } warp;
} refmvs_tile;

typedef struct refmvs_candidate {
    union mv mv[2];
    uint8_t weight;
    int8_t cwp_idx;
    int8_t y_off, x_off;
} refmvs_candidate;

#define decl_splat_mv_fn(name) \
void (name)(refmvs_block *s_dst, refmvs_block *s_src, \
            refmvs_temporal_block *t_dst, ptrdiff_t t_stride, \
            refmvs_temporal_block *t_src, int bw4, int bh4)
typedef decl_splat_mv_fn(*splat_mv_fn);

#define decl_splat_warpmv_fn(name) \
void (name)(refmvs_block *s_dst, refmvs_block *s_src, \
            refmvs_temporal_block *t_dst, ptrdiff_t t_stride, \
            refmvs_temporal_block *t_src, int64_t mvy, int64_t mvx, \
            const Dav2dWarpedMotionParams *const matrix, int bw4, int bh4)
typedef decl_splat_warpmv_fn(*splat_warpmv_fn);

#define decl_splat_comp_warpmv_fn(name) \
void (name)(refmvs_block *s_dst, refmvs_block *s_src, \
            refmvs_temporal_block *t_dst, ptrdiff_t t_stride, \
            refmvs_temporal_block *t_src, \
            int64_t mvy1, int64_t mvx1, int64_t mvy2, int64_t mvx2, \
            const Dav2dWarpedMotionParams *const matrix, \
            int bw4, int bh4, int t_swap, const uint8_t *wedge, int w_mask)
typedef decl_splat_comp_warpmv_fn(*splat_comp_warpmv_fn);

#define decl_splat_comp_wedgemv_fn(name) \
void (name)(refmvs_block *s_dst, refmvs_block *s_src, \
            refmvs_temporal_block *t_dst, ptrdiff_t t_stride, \
            refmvs_temporal_block *t_src, int bw4, int bh4, \
            const uint8_t *wedge, int w_mask)
typedef decl_splat_comp_wedgemv_fn(*splat_comp_wedgemv_fn);

typedef struct Dav2dRefmvsDSPContext {
    splat_mv_fn splat_mv;
    splat_warpmv_fn splat_warpmv;
    splat_comp_warpmv_fn splat_comp_warpmv;
    splat_comp_wedgemv_fn splat_comp_wedgemv;
} Dav2dRefmvsDSPContext;

// call once per frame
int dav2d_refmvs_init_frame(refmvs_frame *rf,
                            const Dav2dSequenceHeader *seq_hdr,
                            const Dav2dFrameHeader *frm_hdr,
                            const uint8_t ref_poc[7],
                            refmvs_temporal_block *rp,
                            const uint8_t ref_ref_poc[7][7],
                            const uint8_t refcnt[7],
                            /*const*/ refmvs_temporal_block *const rp_ref[7],
                            int n_tile_threads, int n_frame_threads);

// cache the current superblock's bottom spatial values into into a "top"
// buffer to act as "top" across superblock boundaries for the next sbrow
void dav2d_refmvs_save_tmvs(const Dav2dRefmvsDSPContext *dsp,
                            refmvs_tile *rt,
                            int col_start8, int col_end8,
                            int row_start8, int row_end8);

// load temporal MVs for current tile or frame's superblock-row
void dav2d_refmvs_load_tmvs(const refmvs_frame *const rf, int tile_row_idx,
                            const int col_start8, const int col_end8,
                            const int row_start8, int row_end8);

mv dav2d_mv_projection(mv in, int num, int den, int min, int max);
static ALWAYS_INLINE mv scale_mv(const mv in, const int sf) {
    const int64_t y = in.y * (int64_t) sf, x = in.x * (int64_t) sf;
    return (mv) {
        .y = iclip((int)((y + 0x2000 - (y < 0)) >> 14), -0xffff, 0xffff),
        .x = iclip((int)((x + 0x2000 - (x < 0)) >> 14), -0xffff, 0xffff),
    };
}

static ALWAYS_INLINE unsigned quantize_mv_comp(const unsigned absv) {
    assert(absv < 2048);
    if (!absv) return 0;
    const int nbits = iclip(ulog2(absv) - 4, 0, 6);
    int res = (absv - (16 * !!nbits << nbits)) >> nbits;
    res += (nbits + !!nbits) * 16;
    return res;
}

static ALWAYS_INLINE union qmv quantize_mv(const union mv mv) {
    const int absy = abs(mv.y), absx = abs(mv.x);
    if (imax(absx, absy) >= 2048) return (union qmv) { .n = INVALID_TRAJ };
    return (union qmv) {
        .y = apply_sign(quantize_mv_comp(absy), mv.y),
        .x = apply_sign(quantize_mv_comp(absx), mv.x),
    };
}

// initialize tile boundaries and refmvs_block pointers for one tile/sbrow
void dav2d_refmvs_tile_sbrow_init(refmvs_tile *rt, const refmvs_frame *rf,
                                  int tile_col_start4, int tile_col_end4,
                                  int tile_row_start4, int tile_row_end4,
                                  int sby, int tile_row_idx, int pass);
void dav2d_refmvs_reset_sb(refmvs_tile *rt, int by, int bx);
void dav2d_refmvs_bank_update(refmvs_tile *rt, enum BlockSize bs, int by, int bx);
void dav2d_refmvs_bank_add(refmvs_tile *rt, enum BlockSize bs, int by, int bx,
                           const Av1Block *b);
int dav2d_refmvs_warp_add(refmvs_tile *rt, const Dav2dWarpedMotionParams *const m,
                          DB_ONLY(int by4, int bx4) int ref);

// call for each block
void dav2d_refmvs_find(const refmvs_tile *rt, refmvs_candidate mvstack[6],
                       int32_t (*warp)[7], int *cnt, const refmvs_refpair ref,
                       enum BlockSize bs, int skip_mode, int by4, int bx4);

void dav2d_refmvs_dsp_init(Dav2dRefmvsDSPContext *dsp);
void dav2d_refmvs_dsp_init_arm(Dav2dRefmvsDSPContext *dsp);
void dav2d_refmvs_dsp_init_loongarch(Dav2dRefmvsDSPContext *dsp);
void dav2d_refmvs_dsp_init_x86(Dav2dRefmvsDSPContext *dsp);

#endif /* DAV2D_SRC_REF_MVS_H */
