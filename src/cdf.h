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

#ifndef DAV2D_SRC_CDF_H
#define DAV2D_SRC_CDF_H

#include <stdint.h>

#include "src/levels.h"
#include "src/ref.h"
#include "src/thread_data.h"

/* Buffers padded to [4]/[8] for SIMD where needed. */

/* TODO: Reorganize structs to minimize alignment padding. */
typedef struct CdfModeContext {
    ALIGN(uint16_t rst_switchable[2][2], 4);
    ALIGN(uint16_t rst_pc_wiener[2], 4);
    ALIGN(uint16_t rst_ns_wiener[2], 4);
    ALIGN(uint16_t wiener_ns_len[2][2], 4);
    ALIGN(uint16_t wiener_ns_sym[2], 4);
    ALIGN(uint16_t wiener_ns_cf[4], 8);
    ALIGN(uint16_t part_split[2][64][2], 4);
    ALIGN(uint16_t part_square[8][2], 4);
    ALIGN(uint16_t part_dir[2][64][2], 4);
    ALIGN(uint16_t part_ext[2][64][2], 4);
    ALIGN(uint16_t part_4way[2][64][2], 4);
    ALIGN(uint16_t region_type[4][2], 4);
    ALIGN(uint16_t intrabc[3][2], 4);
    ALIGN(uint16_t gdf[2], 4);
    ALIGN(uint16_t cdef_idx0[4][2], 4);
    ALIGN(uint16_t cdef_idx[6][7+1], 16);
    ALIGN(uint16_t ccso[3][4][2], 4);
    ALIGN(uint16_t skip_txfm[6][2], 4);
    ALIGN(uint16_t dpcm[2][2], 4);
    ALIGN(uint16_t dpcm_dir[2][2], 4);
    ALIGN(uint16_t intra_y_set[4], 8);
    ALIGN(uint16_t intra_y_idx0[3][8], 16);
    ALIGN(uint16_t intra_y_idx1[3][6+2], 16);
    ALIGN(uint16_t fsc[4][6][2], 4);
    ALIGN(uint16_t mrl_index[3][4], 8);
    ALIGN(uint16_t multi_mrl[3][2], 4);
    ALIGN(uint16_t pal_y[2], 4);
    ALIGN(uint16_t pal_sz[7+1], 16);
    ALIGN(uint16_t dip[3][2], 4);
    ALIGN(uint16_t dip_mode[6+2], 16);
    ALIGN(uint16_t cfl[3][2], 4);
    ALIGN(uint16_t intra_uv_mode[2][8], 16);
    ALIGN(uint16_t mhccp[2], 4);
    ALIGN(uint16_t mhccp_filter_dir[4][3+1], 8);
    ALIGN(uint16_t cfl_type[2], 4);
    ALIGN(uint16_t cfl_sign[8], 16);
    ALIGN(uint16_t cfl_alpha[6][8], 16);
    ALIGN(uint16_t pal_idx_identity[4][3+1], 8);
    ALIGN(uint16_t pal_idx[7][5][8], 16);
    ALIGN(uint16_t intrabc_mode[2], 4);
    ALIGN(uint16_t intrabc_precision[2], 4);
    ALIGN(uint16_t morph_pred[3][2], 4);
    ALIGN(uint16_t txsz_lossless[4][2][2], 4);
    ALIGN(uint16_t tx_split[2][2][9][2], 4);
    ALIGN(uint16_t tx_part_2d[2][2][14][7+1], 16);
    ALIGN(uint16_t tx_part_1d[2][2][2][2], 4);
    ALIGN(uint16_t txtp_lossless[2], 4);
    ALIGN(uint16_t txtp_long32_dct[2][2], 4);
    ALIGN(uint16_t txtp_intra_short_1d[4][4], 8);
    ALIGN(uint16_t txtp_inter_short_1d[3][4][4], 8);
    ALIGN(uint16_t txtp_ext[4][7+1], 16);
    ALIGN(uint16_t txtp_ext_reduced[4][2], 4);
    ALIGN(uint16_t txtp_inter_tx_set[2][3][4][2], 4);
    ALIGN(uint16_t txtp_inter_set0[2][3][8], 16);
    ALIGN(uint16_t txtp_inter_set1[3][8], 16);
    ALIGN(uint16_t txtp_inter_set2[3][4], 8);
    ALIGN(uint16_t txtp_inter_dct_idtx[3][4][2], 4);
    ALIGN(uint16_t txtp_inter_dct_idtx_iddct[3][4][4], 4);
    ALIGN(uint16_t stx[2][5][4], 8);
    ALIGN(uint16_t stx_set_adst[4], 8);
    ALIGN(uint16_t stx_set[7+1], 16);
    ALIGN(uint16_t cctx[7+1], 16);
    ALIGN(uint16_t seg_id_ext[3][2], 2);
    ALIGN(uint16_t seg_id[2][3][8], 8);
    ALIGN(uint16_t delta_q[8], 8);

    /* inter/switch */
    ALIGN(uint16_t skip_mode[3][2], 4);
    ALIGN(uint16_t skip_mode_drl_idx[3][2], 4);
    ALIGN(uint16_t intra[4][2], 4);
    ALIGN(uint16_t tip[3][2], 4);
    ALIGN(uint16_t comp[5][2], 4);
    ALIGN(uint16_t single_ref[3][6][2], 4);
    ALIGN(uint16_t comp0_ref[3][6][2], 4);
    ALIGN(uint16_t comp1_ref[3][2][6][2], 4);
    ALIGN(uint16_t tip_mode[2], 4);
    ALIGN(uint16_t warp[5][2], 4);
    ALIGN(uint16_t warp_newmv[2], 4);
    ALIGN(uint16_t inter_mode[5][3+1], 8);
    ALIGN(uint16_t amvd[9][3][2], 4);
    ALIGN(uint16_t bawp[2][2], 4);
    ALIGN(uint16_t bawp_explicit[3][2], 4);
    ALIGN(uint16_t bawp_explicit_scale[2], 4);
    ALIGN(uint16_t warp_extend[3][2], 4);
    ALIGN(uint16_t warp_causal[4][2], 4);
    ALIGN(uint16_t interintra[4][2], 4);
    ALIGN(uint16_t interintra_mode[4][4], 8);
    ALIGN(uint16_t interintra_wedge[2], 4);
    ALIGN(uint16_t wedge_quad[4], 8);
    ALIGN(uint16_t wedge_angle[4][5+3], 16);
    ALIGN(uint16_t wedge_dist2[3+1], 8);
    ALIGN(uint16_t wedge_dist[4], 8);
    ALIGN(uint16_t tip_drl_idx[3][2], 4);
    ALIGN(uint16_t jmvd_amvd_scale_mode[3+1], 8);
    ALIGN(uint16_t jmvd_scale_mode[5+3], 16);
    ALIGN(uint16_t drl_idx[3][5][2], 4);
    ALIGN(uint16_t mvprec_def[3][2], 4);
    ALIGN(uint16_t mvprec_rem[2][3][3+1], 8);
    ALIGN(uint16_t warp_ref_idx[3][2], 4);
    ALIGN(uint16_t amvd_joint[4], 8);
    ALIGN(uint16_t amvd_index[2][8], 16);
    ALIGN(uint16_t warpmv_with_mvd[2], 4);
    ALIGN(uint16_t warp_delta_prec[N_BS_SIZES][2], 4);
    ALIGN(uint16_t warp_delta_param[2][2][8], 16);
    ALIGN(uint16_t warp_delta_sign[2], 4);
    ALIGN(uint16_t warp_interintra[4][2], 4);
    ALIGN(uint16_t comp_mode_sameref[5][4], 8);
    ALIGN(uint16_t comp_mode_joint[2][2], 4);
    ALIGN(uint16_t comp_mode[5][5+3], 16);
    ALIGN(uint16_t opfl[2][2], 4);
    ALIGN(uint16_t refine_mv[11][2], 4);
    ALIGN(uint16_t comp_type_masked[12][2], 4);
    ALIGN(uint16_t comp_type_weighted[2], 4);
    ALIGN(uint16_t cwp_idx[4][2], 4);
    ALIGN(uint16_t filter[8][4], 8);
    ALIGN(uint16_t seg_pred[3][2], 4);
} CdfModeContext;

typedef struct CdfCoefContext {
    ALIGN(uint16_t skip[2][5][10][2], 4);
    ALIGN(uint16_t eob_bin_16[3][5+3], 16);
    ALIGN(uint16_t eob_bin_32[3][6+2], 16);
    ALIGN(uint16_t eob_bin_64[3][7+1], 16);
    ALIGN(uint16_t eob_bin_128[3][8], 16);
    ALIGN(uint16_t eob_bin_256[3][8], 16);
    ALIGN(uint16_t eob_bin_512[3][8], 16);
    ALIGN(uint16_t eob_bin_1024[3][8], 16);
    ALIGN(uint16_t eob_hi_bit[2], 4);
    ALIGN(uint16_t eob_base_y_tok_hf[5][4][3+1], 8);
    ALIGN(uint16_t base_y_tok_hf[5][20][2][4], 8);
    ALIGN(uint16_t br_y_tok_hf[7][4], 8);
    ALIGN(uint16_t eob_base_y_tok_lf[5][4][5+3], 16);
    ALIGN(uint16_t base_y_tok_lf[5][33][2][6+2], 16);
    ALIGN(uint16_t br_y_tok_lf[14][4], 8);
    ALIGN(uint16_t dc_sign[2][2][3][2], 4);
    ALIGN(uint16_t bob_base_y_tok[3][3][3+1], 8);
    ALIGN(uint16_t br_y_tok_idtx[3][7][4], 8);
    ALIGN(uint16_t base_y_tok_idtx[3][7][4], 8);
    ALIGN(uint16_t sign_idtx[3][9][2], 4);
    ALIGN(uint16_t skip_v[12][2], 4);
    ALIGN(uint16_t eob_base_uv_tok_hf[4][3+1], 8);
    ALIGN(uint16_t base_uv_tok_hf[12][4], 8);
    ALIGN(uint16_t br_uv_tok_hf[4][4], 8);
    ALIGN(uint16_t eob_base_uv_tok_lf[4][5+3], 16);
    ALIGN(uint16_t base_uv_tok_lf[12][6+2], 16);
} CdfCoefContext;

typedef struct CdfMvContext {
    ALIGN(uint16_t shell_lower[7][8], 16);
    ALIGN(uint16_t shell_upper[7][8], 16);
    ALIGN(uint16_t shell_set[2], 4);
    ALIGN(uint16_t shell_tip[2], 4);
    ALIGN(uint16_t shell_offset_low[2][2], 4);
    ALIGN(uint16_t shell_offset_cl2[2], 4);
    ALIGN(uint16_t shell_offset_hi[16][2], 4);
    ALIGN(uint16_t col_component[2][2], 4);
    ALIGN(uint16_t col_index[4][2], 4);
} CdfMvContext;

typedef struct CdfContext {
    CdfCoefContext coef;
    CdfModeContext m;
    CdfMvContext mv, dmv;
} CdfContext;

typedef struct CdfThreadContext {
    Dav2dRef *ref; ///< allocation origin
    union {
        CdfContext *cdf; // if ref != NULL
        unsigned qcat; // if ref == NULL, from static CDF tables
    } data;
    atomic_uint *progress;
} CdfThreadContext;

void dav2d_cdf_reset_count(const Dav2dFrameHeader *hdr, CdfContext *dst);
void dav2d_cdf_shift(CdfContext *dst, const CdfContext *src, int n_tiles_log2);
void dav2d_cdf_shift_accumulate(CdfContext *dst, const CdfContext *src,
                                int n_tiles_log2);
void dav2d_cdf_pri_sec_average(CdfContext *dst, const CdfThreadContext *src1,
                               const CdfThreadContext *src2);
void dav2d_cdf_thread_init_static(CdfThreadContext *cdf, unsigned qidx);
int dav2d_cdf_thread_alloc(Dav2dContext *c, CdfThreadContext *cdf,
                           const int have_frame_mt);
void dav2d_cdf_thread_copy(CdfContext *dst, const CdfThreadContext *src);
void dav2d_cdf_thread_ref(CdfThreadContext *dst, CdfThreadContext *src);
void dav2d_cdf_thread_unref(CdfThreadContext *cdf);

#endif /* DAV2D_SRC_CDF_H */
