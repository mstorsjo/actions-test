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

#ifndef DAV1D_SRC_TABLES_H
#define DAV1D_SRC_TABLES_H

#include <stdint.h>

#include "common/intops.h"

#include "src/levels.h"

// width, height (in 4px blocks), log2 versions of these two
EXTERN const uint8_t dav1d_block_dimensions[N_BS_SIZES][4];
typedef struct TxfmInfo {
    // width, height (in 4px blocks), log2 of them, min/max of log2, sub, pad
    uint8_t w, h, lw, lh, min, max, sub, ctx;
} TxfmInfo;
EXTERN const TxfmInfo dav1d_txfm_dimensions[N_RECT_TX_SIZES];
EXTERN const uint8_t dav1d_tx_shift[N_RECT_TX_SIZES][2];
EXTERN const uint8_t dav1d_tx_ddt_mask[N_RECT_TX_SIZES];
EXTERN const uint8_t /* enum (Rect)TxfmSize */
                     dav1d_max_txfm_size_for_bs[N_BS_SIZES][4 /* y, 420, 422, 444 */];

EXTERN const char *const dav1d_tx1d_names[N_TX_1D_TYPES];

// order: split, horz, vert, horz4, vert4, horz5[small], ver5[small]
// the big transforms in horz5 and vert5 are identical to horz or vert
EXTERN const int8_t dav1d_tx_part_tbl[N_BS_SIZES][8];

EXTERN const uint8_t /* enum TxfmType */
                     dav1d_txtp_from_uvmode[N_UV_INTRA_PRED_MODES];

EXTERN const uint8_t dav1d_mode_to_angle_map[8];

EXTERN const uint8_t /* enum InterPredMode */
                     dav1d_comp_inter_pred_modes[][2];

EXTERN const Dav1dWarpedMotionParams dav1d_default_wm_params;

EXTERN const int16_t dav1d_deblock_side_thresholds[296];

EXTERN const int8_t dav1d_cdef_directions[12][2];

EXTERN const uint16_t dav1d_ccso_quant_sz[4][4];

EXTERN const unsigned dav1d_subset_masks_y[4];
EXTERN const unsigned dav1d_subset_masks_uv[3];
EXTERN const int8_t dav1d_wiener_ns_filters[64][16];
EXTERN const int8_t dav1d_ns_wiener_coef_range_y[16][2];
EXTERN const int8_t dav1d_ns_wiener_coef_range_uv[18][2];
EXTERN const uint16_t dav1d_sgr_params[16][2];
EXTERN const uint8_t dav1d_sgr_x_by_x[256];

EXTERN const int8_t dav1d_mc_subpel_filters[6][15][8];
EXTERN const int8_t dav1d_ext_warp_filter[63][8];
EXTERN const int8_t dav1d_mc_warp_filter[7*64+1][8];
EXTERN const int8_t dav1d_resize_filter[64][8];

EXTERN const uint8_t dav1d_avm_sm_weights[3][64];
EXTERN const uint16_t dav1d_dr_intra_derivative[90];
EXTERN const uint16_t dav1d_div_recip[128 + 1];
EXTERN const uint16_t dav1d_div_scale_sh_offset[8];
EXTERN const uint16_t dav1d_div_scale_sh_bias[8];
EXTERN const uint8_t  dav1d_div_scale_sh_coefw[8];
EXTERN const uint8_t  dav1d_div_scale_sh_coefq[8];
EXTERN const int8_t dav1d_filter_intra_taps[5][64];

EXTERN const int16_t dav1d_gaussian_sequence[2048]; // for fgs

#endif /* DAV1D_SRC_TABLES_H */
