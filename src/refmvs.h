/*
 * Copyright © 2020, VideoLAN and dav1d authors
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

#ifndef DAV1D_SRC_REF_MVS_H
#define DAV1D_SRC_REF_MVS_H

#include <stdint.h>

#include "dav1d/headers.h"

#include "common/intops.h"

#include "src/debug.h"
#include "src/intra_edge.h"
#include "src/tables.h"

#define INVALID_MV 0x80008000
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
CHECK_SIZE(refmvs_sngl_mv_block, 5);

PACKED(typedef union refmvs_refpair {
    int8_t ref[2]; // [0] = 0: intra=1, [1] = -1: comp=0
    uint16_t pair;
}) ALIGN(refmvs_refpair, 2);
CHECK_SIZE(refmvs_refpair, 2);

typedef union refmvs_mvpair {
    mv mv[2];
    uint64_t n;
} refmvs_mvpair;
CHECK_SIZE(refmvs_mvpair, 8);

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
PACKED(typedef struct refmvs_block {
    refmvs_mvpair mv, lmv; // for non-warp blocks, lmv==mv (see #1146)
    refmvs_refpair ref;
    uint8_t bs;
    int8_t mf; // 1 = globalmv+affine, 2 = warp[not gmv], 3-7: cwp-idx
    uint16_t bx4, by4; // top/left coordinates (in 4px units) of this block
}) ALIGN(refmvs_block, 4);
CHECK_SIZE(refmvs_block, 24);

typedef struct refmvs_frame {
    const Dav1dSequenceHeader *seq_hdr;
    const Dav1dFrameHeader *frm_hdr;
    int iw4, ih4, iw8, ih8;
    int sbsz /* in 4px units */;
    int mfmv_sbsz8, mfmv_edge;
    int use_ref_frame_mvs;
    int32_t tip_sf[2];
    int8_t tip_delta;
    uint8_t ref_sign[7];
    int8_t pocdiff[7];
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
    int32_t m[64 * 64 * 2][7]; // FIXME cross-sb top edge? 8x8 res? comp-ref?
    struct {
        int start, end;
    } tile_col, tile_row;
    struct {
        refmvs_mvpair mv[9][4];
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
    refmvs_mvpair mv;
    uint8_t weight;
    int8_t cwp_idx;
    int8_t y_off, x_off;
} refmvs_candidate;

#define decl_save_tmvs_fn(name) \
void (name)(refmvs_temporal_block *rp, const ptrdiff_t stride, \
            refmvs_block *ra, refmvs_block *ra_tl, \
            const refmvs_block *rr, const refmvs_sngl_mv_block *rp_proj, \
            const int32_t tip_sf[2], const uint8_t tip_ref[2], \
            int col_end8, int row_end8, int col_start8, int row_start8)
typedef decl_save_tmvs_fn(*save_tmvs_fn);

#define decl_splat_mv_fn(name) \
void (name)(refmvs_block *r, refmvs_block *rmv, int bw4, int bh4)
typedef decl_splat_mv_fn(*splat_mv_fn);

#define decl_splat_warpmv_fn(name) \
void (name)(refmvs_block *r, int32_t (*m)[7], refmvs_block *rmv, \
            int64_t mvy, int64_t mvx, const Dav1dWarpedMotionParams *const matrix, \
            int bw4, int bh4)
typedef decl_splat_warpmv_fn(*splat_warpmv_fn);

typedef struct Dav1dRefmvsDSPContext {
    save_tmvs_fn save_tmvs;
    splat_mv_fn splat_mv;
    splat_warpmv_fn splat_warpmv;
} Dav1dRefmvsDSPContext;

// call once per frame
int dav1d_refmvs_init_frame(refmvs_frame *rf,
                            const Dav1dSequenceHeader *seq_hdr,
                            const Dav1dFrameHeader *frm_hdr,
                            const uint8_t ref_poc[7],
                            refmvs_temporal_block *rp,
                            const uint8_t ref_ref_poc[7][7],
                            /*const*/ refmvs_temporal_block *const rp_ref[7],
                            int n_tile_threads, int n_frame_threads);

// cache the current superblock's projectable motion vectors
// into buffers for use in future frame's temporal MV prediction
void dav1d_refmvs_save_tmvs(const Dav1dRefmvsDSPContext *dsp,
                            refmvs_tile *rt,
                            int col_start8, int col_end8,
                            int row_start8, int row_end8);

// load temporal MVs for current tile or frame's superblock-row
void dav1d_refmvs_load_tmvs(const refmvs_frame *const rf, int tile_row_idx,
                            const int col_start8, const int col_end8,
                            const int row_start8, int row_end8);

mv mv_projection(mv in, int num, int den);
mv scale_mv(mv in, int sf);

// initialize tile boundaries and refmvs_block pointers for one tile/sbrow
void dav1d_refmvs_tile_sbrow_init(refmvs_tile *rt, const refmvs_frame *rf,
                                  int tile_col_start4, int tile_col_end4,
                                  int tile_row_start4, int tile_row_end4,
                                  int sby, int tile_row_idx, int pass);
void dav1d_refmvs_reset_sb(refmvs_tile *rt, int by, int bx);
void dav1d_refmvs_bank_update(refmvs_tile *rt, enum BlockSize bs, int by, int bx);
void dav1d_refmvs_bank_add(refmvs_tile *rt, enum BlockSize bs, int by, int bx,
                           const Av1Block *b);
void dav1d_refmvs_warp_add(refmvs_tile *rt, const Dav1dWarpedMotionParams *const m,
                           DB_ONLY(int by4, int bx4) int ref);

// call for each block
void dav1d_refmvs_find(const refmvs_tile *rt, refmvs_candidate mvstack[6],
                       int32_t (*warp)[7], int *cnt, const refmvs_refpair ref,
                       enum BlockSize bs, int by4, int bx4);

void dav1d_refmvs_dsp_init(Dav1dRefmvsDSPContext *dsp);
void dav1d_refmvs_dsp_init_arm(Dav1dRefmvsDSPContext *dsp);
void dav1d_refmvs_dsp_init_loongarch(Dav1dRefmvsDSPContext *dsp);
void dav1d_refmvs_dsp_init_x86(Dav1dRefmvsDSPContext *dsp);

#endif /* DAV1D_SRC_REF_MVS_H */
