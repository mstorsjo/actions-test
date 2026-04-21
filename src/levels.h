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

#ifndef DAV2D_SRC_LEVELS_H
#define DAV2D_SRC_LEVELS_H

#include <stdint.h>

#include "dav2d/headers.h"
#include "common/attributes.h"

enum ObuMetaType {
    OBU_META_HDR_CLL     = 1,
    OBU_META_HDR_MDCV    = 2,
    OBU_META_SCALABILITY = 3,
    OBU_META_ITUT_T35    = 4,
    OBU_META_TIMECODE    = 5,
};

enum TxfmSize {
    TX_4X4,
    TX_8X8,
    TX_16X16,
    TX_32X32,
    TX_64X64,
    N_TX_SIZES,
};

enum RectTxfmSize {
    RTX_4X8 = N_TX_SIZES,
    RTX_8X4,
    RTX_8X16,
    RTX_16X8,
    RTX_16X32,
    RTX_32X16,
    RTX_32X64,
    RTX_64X32,
    RTX_4X16,
    RTX_16X4,
    RTX_8X32,
    RTX_32X8,
    RTX_16X64,
    RTX_64X16,
    RTX_4X32,
    RTX_32X4,
    RTX_8X64,
    RTX_64X8,
    RTX_4X64,
    RTX_64X4,
    N_RECT_TX_SIZES
};

enum Tx1dType {
    DCT,
    IDENTITY,
    ADST,
    FLIPADST,
    DDT,
    FLIPDDT,
    WHT,
    N_TX_1D_TYPES,
};

enum TxClass {
    TX_CLASS_2D,
    TX_CLASS_2D_INV, /* inverse coefficient order */
    TX_CLASS_H,
    TX_CLASS_V,
};

#define TX_TYPE_ENUM(NAME, HOR_1D, VER_1D, CLASS) \
    NAME = (HOR_1D) | (TX_CLASS_##CLASS << 3) | (VER_1D << 5)
#define TX_TYPE_ENUM_2D(HOR_1D, VER_1D) \
    TX_TYPE_ENUM(VER_1D##_##HOR_1D, HOR_1D, VER_1D, 2D)
enum TxfmType {
    TX_TYPE_ENUM_2D(DCT, DCT),
    TX_TYPE_ENUM_2D(DCT, ADST),
    TX_TYPE_ENUM_2D(ADST, DCT),
    TX_TYPE_ENUM_2D(ADST, ADST),
    TX_TYPE_ENUM_2D(DCT, FLIPADST),
    TX_TYPE_ENUM_2D(FLIPADST, DCT),
    TX_TYPE_ENUM_2D(FLIPADST, FLIPADST),
    TX_TYPE_ENUM_2D(FLIPADST, ADST),
    TX_TYPE_ENUM_2D(ADST, FLIPADST),
    TX_TYPE_ENUM(IDTX, IDENTITY, IDENTITY, 2D),
    TX_TYPE_ENUM(IDTX_INV, IDENTITY, IDENTITY, 2D_INV),
    TX_TYPE_ENUM(V_DCT, IDENTITY, DCT, V),
    TX_TYPE_ENUM(H_DCT, DCT, IDENTITY, H),
    TX_TYPE_ENUM(V_ADST, IDENTITY, ADST, V),
    TX_TYPE_ENUM(H_ADST, ADST, IDENTITY, H),
    TX_TYPE_ENUM(V_FLIPADST, IDENTITY, FLIPADST, V),
    TX_TYPE_ENUM(H_FLIPADST, FLIPADST, IDENTITY, H),
    TX_TYPE_ENUM_2D(WHT, WHT),

    // when the ddt sequence header bit is enabled
    TX_TYPE_ENUM_2D(DCT, DDT),
    TX_TYPE_ENUM_2D(DCT, FLIPDDT),
    TX_TYPE_ENUM_2D(IDENTITY, DDT),
    TX_TYPE_ENUM_2D(IDENTITY, FLIPDDT),
    TX_TYPE_ENUM_2D(DDT, DDT),
    TX_TYPE_ENUM_2D(DDT, FLIPDDT),
    TX_TYPE_ENUM_2D(DDT, DCT),
    TX_TYPE_ENUM_2D(DDT, IDENTITY),
    TX_TYPE_ENUM_2D(FLIPDDT, DDT),
    TX_TYPE_ENUM_2D(FLIPDDT, FLIPDDT),
    TX_TYPE_ENUM_2D(FLIPDDT, DCT),
    TX_TYPE_ENUM_2D(FLIPDDT, IDENTITY),

    // when one side is 4-point, we can mix adst/ddt
    TX_TYPE_ENUM_2D(ADST, DDT),
    TX_TYPE_ENUM_2D(ADST, FLIPDDT),
    TX_TYPE_ENUM_2D(FLIPADST, DDT),
    TX_TYPE_ENUM_2D(FLIPADST, FLIPDDT),
    TX_TYPE_ENUM_2D(DDT, ADST),
    TX_TYPE_ENUM_2D(FLIPDDT, ADST),
    TX_TYPE_ENUM_2D(DDT, FLIPADST),
    TX_TYPE_ENUM_2D(FLIPDDT, FLIPADST),
};
#undef TX_TYPE_ENUM_2D
#undef TX_TYPE_ENUM

enum IntraPredMode {
    DC_PRED,
    VERT_PRED,
    HOR_PRED,
    DIAG_DOWN_LEFT_PRED,
    DIAG_DOWN_RIGHT_PRED,
    VERT_RIGHT_PRED,
    HOR_DOWN_PRED,
    HOR_UP_PRED,
    VERT_LEFT_PRED,
    SMOOTH_PRED,
    SMOOTH_V_PRED,
    SMOOTH_H_PRED,
    PAETH_PRED,
    N_INTRA_PRED_MODES,
    CFL_PRED = N_INTRA_PRED_MODES,
    N_UV_INTRA_PRED_MODES,
    N_IMPL_INTRA_PRED_MODES = N_UV_INTRA_PRED_MODES,
    LEFT_DC_PRED = DIAG_DOWN_LEFT_PRED,
    TOP_DC_PRED,
    DC_128_PRED,
    Z1_PRED,
    Z2_PRED,
    Z3_PRED,
    DIP_PRED = N_INTRA_PRED_MODES,
};

enum InterIntraPredMode {
    II_DC_PRED,
    II_VERT_PRED,
    II_HOR_PRED,
    II_SMOOTH_PRED,
    N_INTER_INTRA_PRED_MODES,
};

enum BlockPartition {
    PARTITION_INVALID = -1,
    PARTITION_NONE,     // [ ]
    PARTITION_H,        // [-]
    PARTITION_V,        // [|]
    PARTITION_H3,       // 4x4 -> 4x1[top], 2x2 [left], 2x2 [right], 4x1[bottom]
    PARTITION_V3,       // transpose of H3
    PARTITION_H4A,      // Nx8 -> Nx1,Nx2,Nx4,Nx1
    PARTITION_H4B,      // Nx8 -> Nx1,Nx4,Nx2,Nx1
    PARTITION_V4A,      // transpose of H4A
    PARTITION_V4B,      // transpose of H4B
    PARTITION_SPLIT,    // [+]
    N_PARTITIONS,
};

enum TxPartition {
    TX_PARTITION_NONE,
    TX_PARTITION_SPLIT,
    TX_PARTITION_H,
    TX_PARTITION_V,
    TX_PARTITION_H4,
    TX_PARTITION_V4,
    TX_PARTITION_H5,
    TX_PARTITION_V5,
};

enum BlockSize {
    BS_INVALID = -1,
    BS_256x256,
    BS_256x128,
    BS_128x256,
    BS_128x128,
    BS_128x64,
    BS_64x128,
    BS_64x64,
    BS_64x32,
    BS_64x16,
    BS_64x8,
    BS_64x4,
    BS_32x64,
    BS_32x32,
    BS_32x16,
    BS_32x8,
    BS_32x4,
    BS_16x64,
    BS_16x32,
    BS_16x16,
    BS_16x8,
    BS_16x4,
    BS_8x64,
    BS_8x32,
    BS_8x16,
    BS_8x8,
    BS_8x4,
    BS_4x64,
    BS_4x32,
    BS_4x16,
    BS_4x8,
    BS_4x4,
    N_BS_SIZES,
};

enum InterPredMode {
    NEARMV = 13,
    GLOBALMV,
    NEWMV,
    WARPMV,
    WARPNEWMV,
};

enum CompInterPredMode {
    NEARMV_NEARMV = 18,
    NEARMV_NEWMV,
    NEWMV_NEARMV,
    GLOBALMV_GLOBALMV,
    NEWMV_NEWMV,
    JOINT_NEWMV,
    OPFL_NEARMV_NEARMV,
    OPFL_NEARMV_NEWMV,
    OPFL_NEWMV_NEARMV,
    OPFL_NEWMV_NEWMV,
    OPFL_JOINT_NEWMV,
};

enum CompInterType {
    COMP_INTER_NONE,
    COMP_INTER_AVG,
    COMP_INTER_WEDGE,
    COMP_INTER_SEG,
};

typedef union mv {
    struct {
        int32_t y, x;
    };
    uint64_t n;
} mv;
CHECK_SIZE(mv, 8);
#define INVALID_MV 0x200000 // applied to mv.y
#define COPY2MV(dst, src) memcpy(dst, src, 2 * sizeof(union mv))
#define CMP2MV(src1, src2) memcmp(src1, src2, 2 * sizeof(union mv))
#define ZERO2MV(dst) memset(dst, 0, 2 * sizeof(union mv))

PACKED(typedef union refpair {
    int8_t ref[2];
    int16_t pair;
}) ALIGN(refpair, 2);
CHECK_SIZE(refpair, 2);

enum MotionMode {
    MM_TRANSLATION,
    MM_INTERINTRA,
    MM_WARP_CAUSAL,
    MM_WARP_DELTA,
    MM_WARP_EXTEND,
};

enum CflType {
    CFL_EXPLICIT,
    CFL_IMPLICIT,
    CFL_MHCCP,
};

enum CflMhDir {
    CFL_DIR_CENTER,
    CFL_DIR_TOP,
    CFL_DIR_LEFT,
    CFL_DIR_ALL,
};

#define TIP_FRAME 7

typedef struct Av2Block {
    int8_t bs, cbs;
    uint8_t intra, intrabc, seg_id, skip_mode, skip_txfm, tx_part, fsc, tx_size_ll;
    union refpair ref;
    union {
        struct {
            // it's also possible to access this using mv[0]
            union mv intrabc_mv;
            uint8_t dpcm[2], y_mode, mrl_index, multi_mrl, dip;
            uint8_t morph_pred, is_refmv, is_qpel; // for intrabc
            uint8_t uv_mode, pal_sz;
            int8_t y_angle, uv_angle, cfl_type;
            union {
                int8_t cfl_alpha[2];
                uint8_t cfl_mh_dir; // enum CflMhDir
            };
            struct {
                int a, l;
            } is_sm[2 /* luma, chroma */];
        }; // intra
        struct {
            union mv mv[2];
            int8_t wedge_idx, wedge_sign; // -1 for no wedge
            uint8_t mask_sign, interintra_mode;
            int8_t matrix[4];
            uint8_t drl_idx[2];
            uint8_t warp_ref_idx, warpmv_with_mvd;
            uint8_t comp_type, inter_mode, motion_mode, warp_ii;
            int8_t cwp_idx, mv_prec, amvd;
            uint8_t bawp[2], filter;
            uint8_t refine_mv; // 1 = enabled, 2 = implicitly enabled
            int32_t mtxbak[6]; // for frame-mt only
        }; // inter
    };
} Av2Block;

#endif /* DAV2D_SRC_LEVELS_H */
