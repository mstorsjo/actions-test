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

#ifndef DAV1D_SRC_CDF_H
#define DAV1D_SRC_CDF_H

#include <stdint.h>

#include "src/levels.h"
#include "src/ref.h"
#include "src/thread_data.h"

/* Buffers padded to [4]/[8]/[16] for SIMD where needed. */

typedef struct CdfModeContext {
    uint16_t rst_switchable[2][3];
    uint16_t rst_pc_wiener[3];
    uint16_t rst_ns_wiener[3];
    uint16_t wiener_ns_len[2][3];
    uint16_t wiener_ns_sym[3];
    uint16_t wiener_ns_cf[5];
    uint16_t part_split[2][64][3];
    uint16_t part_square[8][3];
    uint16_t part_dir[2][64][3];
    uint16_t part_ext[2][64][3];
    uint16_t part_4way[2][64][3];
    uint16_t intrabc[3][3];
    uint16_t gdf[3];
    uint16_t cdef_idx0[4][3];
    uint16_t cdef_idx[6][8];
    uint16_t ccso[3][4][3];
    uint16_t skip_txfm[6][3];
    uint16_t intra_y_set[5];
    uint16_t intra_y_idx0[3][9];
    uint16_t intra_y_idx1[3][7];
    uint16_t fsc[4][6][3];
    uint16_t mrl_index[3][5];
    uint16_t multi_mrl[3][3];
    uint16_t dip_mode[7];
    uint16_t cfl[3][3];
    uint16_t intra_uv_mode[2][9];
    uint16_t mhccp[3];
    uint16_t mhccp_filter_dir[4][4];
    uint16_t cfl_type[3];
    uint16_t cfl_sign[9];
    uint16_t cfl_alpha[6][9];
    uint16_t intrabc_mode[3];
    uint16_t intrabc_precision[3];
    uint16_t tx_split[2][2][9][3];
    uint16_t tx_part_2d[2][2][14][8];
    uint16_t tx_part_1d[2][2][2][3];
    uint16_t txtp_long32_dct[2][3];
    uint16_t txtp_intra_short_1d[4][5];
    uint16_t txtp_inter_short_1d[3][4][5];
    uint16_t txtp_ext[4][8];
    uint16_t txtp_ext_reduced[4][3];
    uint16_t txtp_inter_tx_set[2][3][4][3];
    uint16_t txtp_inter_set0[2][3][9];
    uint16_t txtp_inter_set1[3][9];
    uint16_t txtp_inter_set2[3][5];
    uint16_t stx[2][5][5];
    uint16_t stx_set_adst[5];
    uint16_t stx_set[8];
    uint16_t cctx[8];
    ALIGN(uint16_t seg_id[3][DAV1D_MAX_SEGMENTS], 16);
    ALIGN(uint16_t pal_sz[2][7][7 + 1], 16);
    ALIGN(uint16_t color_map[2][7][5][8], 16);
    ALIGN(uint16_t delta_q[4], 8);
    ALIGN(uint16_t delta_lf[5][4], 8);
    ALIGN(uint16_t txpart[7][3][2], 4);
    ALIGN(uint16_t pal_y[7][3][2], 4);
    ALIGN(uint16_t pal_uv[2][2], 4);

    /* inter/switch */
    ALIGN(uint16_t wedge_idx[9][16], 32);
    ALIGN(uint16_t comp_inter_mode[8][N_COMP_INTER_PRED_MODES], 16);
    ALIGN(uint16_t filter[2][8][DAV1D_N_SWITCHABLE_FILTERS + 1], 8);
    ALIGN(uint16_t interintra_mode[4][4], 8);
    ALIGN(uint16_t motion_mode[N_BS_SIZES][3 + 1], 8);
    ALIGN(uint16_t skip_mode[3][2], 4);
    ALIGN(uint16_t newmv_mode[6][2], 4);
    ALIGN(uint16_t globalmv_mode[2][2], 4);
    ALIGN(uint16_t refmv_mode[6][2], 4);
    ALIGN(uint16_t drl_bit[3][2], 4);
    ALIGN(uint16_t intra[4][2], 4);
    ALIGN(uint16_t comp[5][2], 4);
    ALIGN(uint16_t comp_dir[5][2], 4);
    ALIGN(uint16_t jnt_comp[6][2], 4);
    ALIGN(uint16_t mask_comp[6][2], 4);
    ALIGN(uint16_t wedge_comp[9][2], 4);
    ALIGN(uint16_t ref[6][3][2], 4);
    ALIGN(uint16_t comp_fwd_ref[3][3][2], 4);
    ALIGN(uint16_t comp_bwd_ref[2][3][2], 4);
    ALIGN(uint16_t comp_uni_ref[3][3][2], 4);
    ALIGN(uint16_t seg_pred[3][2], 4);
    ALIGN(uint16_t interintra[7][2], 4);
    ALIGN(uint16_t interintra_wedge[7][2], 4);
    ALIGN(uint16_t obmc[N_BS_SIZES][2], 4);
} CdfModeContext;

typedef struct CdfCoefContext {
    uint16_t skip[2][5][20][3];
    uint16_t eob_bin_16[3][6];
    uint16_t eob_bin_32[3][7];
    uint16_t eob_bin_64[3][8];
    uint16_t eob_bin_128[3][9];
    uint16_t eob_bin_256[3][9];
    uint16_t eob_bin_512[3][9];
    uint16_t eob_bin_1024[3][9];
    uint16_t eob_hi_bit[3];
    uint16_t eob_base_y_tok_hf[5][4][4];
    uint16_t base_y_tok_hf[5][20][2][5];
    uint16_t br_y_tok_hf[7][5];
    uint16_t eob_base_y_tok_lf[5][4][6];
    uint16_t base_y_tok_lf[5][33][2][7];
    uint16_t br_y_tok_lf[14][5];
    uint16_t dc_sign[2][2][3][3];
    uint16_t bob_base_y_tok[3][3][4];
    uint16_t br_y_tok_idtx[3][7][5];
    uint16_t base_y_tok_idtx[3][7][5];
    uint16_t sign_idtx[3][9][3];
    uint16_t dip[3][3];
    uint16_t skip_v[12][3];
    uint16_t eob_base_uv_tok_hf[4][4];
    uint16_t base_uv_tok_hf[12][5];
    uint16_t br_uv_tok_hf[4][5];
    uint16_t eob_base_uv_tok_lf[4][6];
    uint16_t base_uv_tok_lf[12][7];
} CdfCoefContext;

typedef struct CdfMvComponent {
    ALIGN(uint16_t classes[11 + 5], 32);
    ALIGN(uint16_t sign[2], 4);
    ALIGN(uint16_t class0[2], 4);
    ALIGN(uint16_t class0_fp[2][4], 8);
    ALIGN(uint16_t class0_hp[2], 4);
    ALIGN(uint16_t classN[10][2], 4);
    ALIGN(uint16_t classN_fp[4], 8);
    ALIGN(uint16_t classN_hp[2], 4);
} CdfMvComponent;

typedef struct CdfMvContext {
    CdfMvComponent comp[2];
    ALIGN(uint16_t joint[N_MV_JOINTS], 8);
    uint16_t shell_set[3];
    uint16_t shell_lower[7][9];
    uint16_t shell_upper[7][9];
    uint16_t shell_offset_low[2][3];
    uint16_t shell_offset_cl2[3];
    uint16_t shell_offset_hi[16][3];
    uint16_t col_component[2][3];
    uint16_t col_index[4][3];
} CdfMvContext;

typedef struct CdfContext {
    CdfCoefContext coef;
    CdfModeContext m;
    CdfMvContext mv;
    ALIGN(uint16_t kfym[5][5][N_INTRA_PRED_MODES + 3], 32);
} CdfContext;

typedef struct CdfThreadContext {
    Dav1dRef *ref; ///< allocation origin
    union {
        CdfContext *cdf; // if ref != NULL
        unsigned qcat; // if ref == NULL, from static CDF tables
    } data;
    atomic_uint *progress;
} CdfThreadContext;

void dav1d_cdf_thread_init_static(CdfThreadContext *cdf, unsigned qidx);
int dav1d_cdf_thread_alloc(Dav1dContext *c, CdfThreadContext *cdf,
                           const int have_frame_mt);
void dav1d_cdf_thread_copy(CdfContext *dst, const CdfThreadContext *src);
void dav1d_cdf_thread_ref(CdfThreadContext *dst, CdfThreadContext *src);
void dav1d_cdf_thread_unref(CdfThreadContext *cdf);
void dav1d_cdf_thread_update(const Dav1dFrameHeader *hdr, CdfContext *dst,
                             const CdfContext *src);

#endif /* DAV1D_SRC_CDF_H */
