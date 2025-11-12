/*
 * Copyright © 2018-2021, VideoLAN and dav1d authors
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

#include <errno.h>
#include <limits.h>
#include <stdio.h>

#include "dav1d/data.h"

#include "common/frame.h"
#include "common/intops.h"
#include "common/validate.h"

#include "src/decode.h"
#include "src/getbits.h"
#include "src/levels.h"
#include "src/log.h"
#include "src/obu.h"
#include "src/ref.h"
#include "src/thread_task.h"

static int check_trailing_bits(GetBits *const gb,
                               const int strict_std_compliance)
{
    const int trailing_one_bit = dav1d_get_bit(gb);

    if (gb->error)
        return DAV1D_ERR(EINVAL);

    if (!strict_std_compliance)
        return 0;

    if (!trailing_one_bit || gb->state)
        return DAV1D_ERR(EINVAL);

    ptrdiff_t size = gb->ptr_end - gb->ptr;
    while (size > 0 && gb->ptr[size - 1] == 0)
        size--;

    if (size)
        return DAV1D_ERR(EINVAL);

    return 0;
}

static inline int tile_log2(const int sz, const int tgt) {
    int k;
    for (k = 0; (sz << k) < tgt; k++) ;
    return k;
}

static NOINLINE void parse_tile_info(struct Dav1dTileInfo *const thdr,
                                     GetBits *const gb, const int sbmul,
                                     const int sb128, const int seq_sb128,
                                     const int w, const int h)
{
    thdr->uniform = dav1d_get_bit(gb);

    // the limits are calculated based on a frame's sb128, and rounded-up
    // width/height variables (aligned to sbsz)
    const int sbsz_min1 = (64 << sb128) - 1;
    const int sbsz_log2 = 6 + sb128;
    const int sbw = (w + sbsz_min1) >> sbsz_log2;
    const int sbh = (h + sbsz_min1) >> sbsz_log2;
    const int max_tile_width_sb = 4096 >> sbsz_log2;
    const int max_tile_area_sb = 4096 * 2304 >> (2 * sbsz_log2);
    thdr->min_log2_cols = tile_log2(max_tile_width_sb, sbw);
    thdr->max_log2_cols = tile_log2(1, imin(sbw, DAV1D_MAX_TILE_COLS));
    thdr->max_log2_rows = tile_log2(1, imin(sbh, DAV1D_MAX_TILE_ROWS));
    const int min_log2_tiles = imax(tile_log2(max_tile_area_sb, sbw * sbh),
                                    thdr->min_log2_cols);

    if (thdr->uniform) {
        // but the (uniform) tile distribution is done based on "full" SBs only,
        // which can be less than the rounded-up versions above. Also, this is
        // done based on the sequence header's sb128 (not the frame's), which
        // can be different for keyframes
        const int seq_sbsz_log2 = 6 + seq_sb128;
        const int fsbw = imax(1, (w + 7) >> seq_sbsz_log2);
        const int fsbh = imax(1, (h + 7) >> seq_sbsz_log2);

        for (thdr->log2_cols = thdr->min_log2_cols;
             thdr->log2_cols < thdr->max_log2_cols && dav1d_get_bit(gb);
             thdr->log2_cols++) ;
        const int tile_w = imax(1, fsbw >> thdr->log2_cols);
        int extra = imax(0, fsbw - (tile_w << thdr->log2_cols));
        thdr->cols = 0;
        for (int sbx = 0; sbx < fsbw;
             sbx += tile_w + (extra > 0), thdr->cols++, extra--)
        {
            thdr->col_start_sb[thdr->cols] = sbx * sbmul;
        }
        thdr->min_log2_rows =
            imax(min_log2_tiles - thdr->log2_cols, 0);

        for (thdr->log2_rows = thdr->min_log2_rows;
             thdr->log2_rows < thdr->max_log2_rows && dav1d_get_bit(gb);
             thdr->log2_rows++) ;
        const int tile_h = imax(1, fsbh >> thdr->log2_rows);
        extra = imax(0, fsbh - (tile_h << thdr->log2_rows));
        thdr->rows = 0;
        for (int sby = 0; sby < fsbh;
             sby += tile_h + (extra > 0), thdr->rows++, extra--)
        {
            thdr->row_start_sb[thdr->rows] = sby * sbmul;
        }
    } else {
        thdr->cols = 0;
        int widest_tile = 0, max_tile_area_sb = sbw * sbh;
        for (int sbx = 0; sbx < sbw && thdr->cols < DAV1D_MAX_TILE_COLS; thdr->cols++) {
            const int tile_width_sb = imin(sbw - sbx, max_tile_width_sb);
            const int tile_w = (tile_width_sb > 1) ? 1 + dav1d_get_uniform(gb, tile_width_sb) : 1;
            thdr->col_start_sb[thdr->cols] = sbx;
            sbx += tile_w;
            widest_tile = imax(widest_tile, tile_w);
        }
        thdr->log2_cols = tile_log2(1, thdr->cols);
        if (min_log2_tiles) max_tile_area_sb >>= min_log2_tiles + 1;
        const int max_tile_height_sb = imax(max_tile_area_sb / widest_tile, 1);

        thdr->rows = 0;
        for (int sby = 0; sby < sbh && thdr->rows < DAV1D_MAX_TILE_ROWS; thdr->rows++) {
            const int tile_height_sb = imin(sbh - sby, max_tile_height_sb);
            const int tile_h = (tile_height_sb > 1) ? 1 + dav1d_get_uniform(gb, tile_height_sb) : 1;
            thdr->row_start_sb[thdr->rows] = sby;
            sby += tile_h;
        }
        thdr->log2_rows = tile_log2(1, thdr->rows);
    }
    thdr->col_start_sb[thdr->cols] = sbw;
    thdr->row_start_sb[thdr->rows] = sbh;
}

static NOINLINE int parse_seq_hdr(Dav1dSequenceHeader *const hdr,
                                  GetBits *const gb,
                                  const int strict_std_compliance)
{
#define DEBUG_SEQ_HDR 0

#if DEBUG_SEQ_HDR
    const unsigned init_bit_pos = dav1d_get_bits_pos(gb);
#endif

    memset(hdr, 0, sizeof(*hdr));
    hdr->id = dav1d_get_vlc(gb);
    hdr->profile = dav1d_get_bits(gb, 3);
    if (hdr->profile > 2) goto error;
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-profile[%d]: off=%u\n",
           hdr->profile, dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->width_n_bits = dav1d_get_bits(gb, 4) + 1;
    hdr->height_n_bits = dav1d_get_bits(gb, 4) + 1;
    hdr->max_width = dav1d_get_bits(gb, hdr->width_n_bits) + 1;
    hdr->max_height = dav1d_get_bits(gb, hdr->height_n_bits) + 1;
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-size[bits:%dx%d,max:%dx%d]: off=%u\n",
           hdr->width_n_bits, hdr->height_n_bits,
           hdr->max_width, hdr->max_height,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->crop.enabled = dav1d_get_bit(gb);
    if (hdr->crop.enabled) {
        hdr->crop.left = dav1d_get_vlc(gb);
        hdr->crop.right = dav1d_get_vlc(gb);
        hdr->crop.top = dav1d_get_vlc(gb);
        hdr->crop.bottom = dav1d_get_vlc(gb);
    }
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-cropwindow[%d,l:%d,r:%d,t:%d,b:%d]: off=%u\n",
           hdr->crop.enabled,
           hdr->crop.left, hdr->crop.right,
           hdr->crop.top, hdr->crop.bottom,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->layout = dav1d_get_vlc(gb);
    if (hdr->layout > 3) goto error;
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-layout[%d]: off=%u\n",
           hdr->layout, dav1d_get_bits_pos(gb) - init_bit_pos);
#endif
    hdr->layout = (const uint8_t[]) {
        DAV1D_PIXEL_LAYOUT_I420, DAV1D_PIXEL_LAYOUT_I400,
        DAV1D_PIXEL_LAYOUT_I444, DAV1D_PIXEL_LAYOUT_I422 }[hdr->layout];

    hdr->hbd = dav1d_get_vlc(gb);
    if (hdr->hbd > 2) goto error;
    if (hdr->hbd < 2) hdr->hbd ^= 1;
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-bitdepth[%d]: off=%u\n",
           8 + 2 * hdr->hbd,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->color_description_present = dav1d_get_bit(gb);
    if (hdr->color_description_present) {
        hdr->pri = dav1d_get_bits(gb, 8);
        hdr->trc = dav1d_get_bits(gb, 8);
        hdr->mtrx = dav1d_get_bits(gb, 8);
    } else {
        hdr->pri = DAV1D_COLOR_PRI_UNKNOWN;
        hdr->trc = DAV1D_TRC_UNKNOWN;
        hdr->mtrx = DAV1D_MC_UNKNOWN;
    }
    hdr->chr = DAV1D_CHR_UNKNOWN;
    if (hdr->layout == DAV1D_PIXEL_LAYOUT_I400) {
        hdr->color_range = dav1d_get_bit(gb);
        hdr->ss_hor = hdr->ss_ver = 1;
    } else if (hdr->pri == DAV1D_COLOR_PRI_BT709 &&
               hdr->trc == DAV1D_TRC_SRGB &&
               hdr->mtrx == DAV1D_MC_IDENTITY)
    {
        hdr->layout = DAV1D_PIXEL_LAYOUT_I444;
        hdr->ss_hor = hdr->ss_ver = 0;
        hdr->color_range = 1;
        if (hdr->profile != 1 && !(hdr->profile == 2 && hdr->hbd == 2))
            goto error;
    } else {
        hdr->color_range = dav1d_get_bit(gb);
        switch (hdr->layout) {
        case DAV1D_PIXEL_LAYOUT_I420:
            if (dav1d_get_bit(gb)) {
                hdr->chr = dav1d_get_bits(gb, 3);
                if (hdr->chr >= DAV1D_CHR_UNKNOWN)
                    goto error;
            }
            // fall-through
        case DAV1D_PIXEL_LAYOUT_I400:
            hdr->ss_hor = hdr->ss_ver = 1;
            break;
        case DAV1D_PIXEL_LAYOUT_I422:
            hdr->ss_hor = 1;
            hdr->ss_ver = 1;
            if (dav1d_get_bit(gb))
                hdr->chr = dav1d_get_bit(gb);
            break;
        default: break;
        }
    }
    if (strict_std_compliance &&
        hdr->mtrx == DAV1D_MC_IDENTITY && hdr->layout != DAV1D_PIXEL_LAYOUT_I444)
    {
        goto error;
    }
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-colorinfo[ss=h:%d,v:%d]: off=%u\n",
           hdr->ss_hor, hdr->ss_ver,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->still_picture = dav1d_get_bit(gb);
    hdr->reduced_still_picture_header = dav1d_get_bit(gb);
    if (hdr->reduced_still_picture_header && !hdr->still_picture) goto error;
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-stillpicture_flags[%d,reduced_hdr:%d]: off=%u\n",
           hdr->still_picture, hdr->reduced_still_picture_header,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    if (hdr->reduced_still_picture_header) {
        hdr->num_operating_points = 1;
        hdr->operating_points[0].level = dav1d_get_bits(gb, 5);
        hdr->operating_points[0].initial_display_delay = 10;
    } else {
        hdr->timing_info_present = dav1d_get_bit(gb);
        if (hdr->timing_info_present) {
            hdr->num_units_in_tick = dav1d_get_bits(gb, 32);
            hdr->time_scale = dav1d_get_bits(gb, 32);
            if (strict_std_compliance && (!hdr->num_units_in_tick || !hdr->time_scale))
                goto error;
            hdr->equal_picture_interval = dav1d_get_bit(gb);
            if (hdr->equal_picture_interval) {
                const unsigned num_ticks_per_picture = dav1d_get_vlc(gb);
                if (num_ticks_per_picture == UINT32_MAX)
                    goto error;
                hdr->num_ticks_per_picture = num_ticks_per_picture + 1;
            }

            hdr->decoder_model_info_present = dav1d_get_bit(gb);
            if (hdr->decoder_model_info_present) {
                hdr->encoder_decoder_buffer_delay_length = dav1d_get_bits(gb, 5) + 1;
                hdr->num_units_in_decoding_tick = dav1d_get_bits(gb, 32);
                if (strict_std_compliance && !hdr->num_units_in_decoding_tick)
                    goto error;
                hdr->buffer_removal_delay_length = dav1d_get_bits(gb, 5) + 1;
                hdr->frame_presentation_delay_length = dav1d_get_bits(gb, 5) + 1;
            }
        }
#if DEBUG_SEQ_HDR
        printf("SEQHDR: post-timinginfo[%d]: off=%u\n",
               hdr->timing_info_present,
               dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

        hdr->display_model_info_present = dav1d_get_bit(gb);
        hdr->num_operating_points = dav1d_get_bits(gb, 5) + 1;
        for (int i = 0; i < hdr->num_operating_points; i++) {
            struct Dav1dSequenceHeaderOperatingPoint *const op =
                &hdr->operating_points[i];
            op->idc = dav1d_get_bits(gb, 12);
            if (op->idc && (!(op->idc & 0xff) || !(op->idc & 0xf00)))
                goto error;
            op->level = dav1d_get_bits(gb, 5);
            if (op->level >= 8)
                op->tier = dav1d_get_bit(gb);
            if (hdr->decoder_model_info_present) {
                op->decoder_model_param_present = dav1d_get_bit(gb);
                if (op->decoder_model_param_present) {
                    struct Dav1dSequenceHeaderOperatingParameterInfo *const opi =
                        &hdr->operating_parameter_info[i];
                    opi->decoder_buffer_delay =
                        dav1d_get_bits(gb, hdr->encoder_decoder_buffer_delay_length);
                    opi->encoder_buffer_delay =
                        dav1d_get_bits(gb, hdr->encoder_decoder_buffer_delay_length);
                    opi->low_delay_mode = dav1d_get_bit(gb);
                }
            }
            if (hdr->display_model_info_present)
                op->display_model_param_present = dav1d_get_bit(gb);
            op->initial_display_delay =
                op->display_model_param_present ? dav1d_get_bits(gb, 4) + 1 : 10;
        }
#if DEBUG_SEQ_HDR
        printf("SEQHDR: post-operatingpoints[%d]: off=%u\n",
               hdr->num_operating_points,
               dav1d_get_bits_pos(gb) - init_bit_pos);
#endif
    }

    // here goes multi-layer HLS info
    if (!hdr->reduced_still_picture_header) {
        hdr->max_tlayer_id = dav1d_get_bits(gb, 2);
        hdr->max_mlayer_id = dav1d_get_bits(gb, 3);
    }

#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-maxlayerid[t:%d,m:%d]: off=%u\n",
           hdr->max_tlayer_id, hdr->max_mlayer_id,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    if (hdr->max_tlayer_id) {
        hdr->tlayer_dependency_present = dav1d_get_bit(gb);
        if (hdr->tlayer_dependency_present) {
            for (unsigned n = 1; n < hdr->max_tlayer_id; n++)
                hdr->tlayer_dependencies[n] = dav1d_get_bits(gb, n);
        } else {
            for (unsigned n = 1, mask = ~0U; n < hdr->max_tlayer_id; n++, mask <<= 1)
                hdr->tlayer_dependencies[n] = ~mask;
        }
    }

    if (hdr->max_mlayer_id) {
        hdr->mlayer_dependency_present = dav1d_get_bit(gb);
        if (hdr->mlayer_dependency_present) {
            for (unsigned n = 1; n < hdr->max_mlayer_id; n++)
                hdr->mlayer_dependencies[n] = dav1d_get_bits(gb, n);
        } else {
            for (unsigned n = 1, mask = ~0U; n < hdr->max_mlayer_id; n++, mask <<= 1)
                hdr->mlayer_dependencies[n] = ~mask;
        }
    }

#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-layerdesc[%d]: off=%u\n",
           hdr->mlayer_dependency_present,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->sb128 = dav1d_get_bit(gb) ? 2 : dav1d_get_bit(gb);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-sbsz[%dx%d]: off=%u\n",
           64 << hdr->sb128, 64 << hdr->sb128,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->intra_dip = dav1d_get_bit(gb); // data-driven intra prediction
    hdr->intra_edge_filter = dav1d_get_bit(gb);
    hdr->mrls = dav1d_get_bit(gb);
    hdr->cfl = dav1d_get_bit(gb);
    hdr->mhccp = dav1d_get_bit(gb);
    hdr->orip = dav1d_get_bit(gb);
    hdr->ibp = dav1d_get_bit(gb);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-intratools[dip:%d,edgefilter:%d,mrl:%d,cfl:%d,"
           "mhccp:%d,orip:%d,ibp:%d]: off=%u\n",
           hdr->intra_dip,
           hdr->intra_edge_filter,
           hdr->mrls,
           hdr->cfl,
           hdr->mhccp,
           hdr->orip,
           hdr->ibp,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    if (hdr->reduced_still_picture_header) {
        hdr->motion_modes = 1 << MM_TRANSLATION;
    } else {
        hdr->motion_modes = (1 << MM_TRANSLATION) + (dav1d_get_bits(gb, 4) << 1);
        hdr->frame_motion_modes_present = dav1d_get_bit(gb);
        hdr->six_param_warp_delta = dav1d_get_bit(gb);
        hdr->masked_compound = dav1d_get_bit(gb);
        hdr->ref_frame_mvs = dav1d_get_bit(gb);
        hdr->reduced_ref_frame_mvs_mode = dav1d_get_bit(gb);
        hdr->order_hint_n_bits = dav1d_get_bits(gb, 3) + 1;
#if DEBUG_SEQ_HDR
        printf("SEQHDR: post-interframetools[mm:%x,fmm:%d,6pwarp:%d,"
               "maskcomp:%d,refmvs:%d,redrefmvs:%d,pocbits:%d]: off=%u\n",
               hdr->motion_modes,
               hdr->frame_motion_modes_present,
               hdr->six_param_warp_delta,
               hdr->masked_compound,
               hdr->ref_frame_mvs,
               hdr->reduced_ref_frame_mvs_mode,
               hdr->order_hint_n_bits,
               dav1d_get_bits_pos(gb) - init_bit_pos);
#endif
    }

    hdr->refmv_bank = dav1d_get_bit(gb);
    hdr->drl_reorder = dav1d_get_bit(gb) ? 0 : 2 - dav1d_get_bit(gb);
    hdr->explicit_ref_frame_map = dav1d_get_bit(gb);
    hdr->ref_frames = dav1d_get_bit(gb) ? dav1d_get_bits(gb, 4) + 1 : 8;
    hdr->ref_frames_log2 = hdr->ref_frames <= 2 ? hdr->ref_frames - 1 :
                           1 + ulog2(hdr->ref_frames - 1);

    hdr->def_max_drl_bits = dav1d_get_uniform(gb, 5) + 1;
    hdr->allow_frame_max_drl_bits = dav1d_get_bit(gb);
    hdr->def_max_bvp_drl_bits = dav1d_get_uniform(gb, 3) + 1;
    hdr->allow_max_bvp_drl_bits = dav1d_get_bit(gb);
    hdr->num_same_ref_comp = dav1d_get_bits(gb, 2);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-refs[bank:%d,drlreorder:%d,explrefmap:%d,"
       "nrefs:%d,drlbits:%d,fdrlbits:%d,bvpdrlbits:%d,fbvpdrlbits:%d,"
       "numsamerefcomp:%d]: off=%u\n",
           hdr->refmv_bank,
           hdr->drl_reorder,
           hdr->explicit_ref_frame_map,
           hdr->ref_frames,
           hdr->def_max_drl_bits,
           hdr->allow_frame_max_drl_bits,
           hdr->def_max_bvp_drl_bits,
           hdr->allow_max_bvp_drl_bits,
           hdr->num_same_ref_comp,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->tip = dav1d_get_bit(gb) ? 1 + dav1d_get_bit(gb) : 0;
    if (hdr->tip)
        hdr->tip_hole_fill = dav1d_get_bit(gb);
    hdr->mv_traj = dav1d_get_bit(gb);
    hdr->bawp = dav1d_get_bit(gb);
    hdr->cwp = dav1d_get_bit(gb);
    hdr->imp_msk_bld = dav1d_get_bit(gb);
    hdr->fsc = dav1d_get_bit(gb);
    hdr->idtx_intra = hdr->fsc || dav1d_get_bit(gb);
    hdr->lf_sub_pu = dav1d_get_bit(gb);
    if (hdr->tip == 1 && hdr->lf_sub_pu)
        hdr->tip_explicit_qp = dav1d_get_bit(gb);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-intertools1[tip:%d,tipholefill:%d,mvtraj:%d,bawp:%d,"
           "cwp:%d,impmskbld:%d,fsc:%d,idtxintra:%d,lfsubpu:%d,tipqp:%d]: off=%u\n",
           hdr->tip,
           hdr->tip_hole_fill,
           hdr->mv_traj,
           hdr->bawp,
           hdr->cwp,
           hdr->imp_msk_bld,
           hdr->fsc,
           hdr->idtx_intra,
           hdr->lf_sub_pu,
           hdr->tip_explicit_qp,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->opfl_refine = dav1d_get_bits(gb, 2);
    hdr->adaptive_mvd = dav1d_get_bit(gb);
    hdr->refine_mv = dav1d_get_bit(gb);
    if (hdr->tip && (hdr->opfl_refine || hdr->refine_mv))
        hdr->tip_refine_mv = dav1d_get_bit(gb);
    hdr->bru = dav1d_get_bit(gb);
    hdr->mvd_sign_derive = dav1d_get_bit(gb);
    hdr->flex_mvres = dav1d_get_bit(gb);
    if (!hdr->reduced_still_picture_header)
        hdr->global_motion = dav1d_get_bit(gb);
    hdr->short_refresh_frame_flags = dav1d_get_bit(gb);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-intertools2[opflrefine:%d,adaptivemvd:%d,refinemv:%d,"
           "tiprefinemv:%d,bru:%d,mvdsignderive:%d,flexmvres:%d,gmv:%d,"
           "shortrefeshmsk:%d]: off=%u\n",
           hdr->opfl_refine,
           hdr->adaptive_mvd,
           hdr->refine_mv,
           hdr->tip_refine_mv,
           hdr->bru,
           hdr->mvd_sign_derive,
           hdr->flex_mvres,
           hdr->global_motion,
           hdr->short_refresh_frame_flags,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    if (hdr->reduced_still_picture_header) {
        hdr->screen_content_tools = DAV1D_ADAPTIVE;
        hdr->force_integer_mv = DAV1D_ADAPTIVE;
    } else {
        hdr->screen_content_tools =
            dav1d_get_bit(gb) ? DAV1D_ADAPTIVE : dav1d_get_bit(gb);
        hdr->force_integer_mv = hdr->screen_content_tools ?
                                dav1d_get_bit(gb) ? DAV1D_ADAPTIVE :
                                dav1d_get_bit(gb) : DAV1D_ADAPTIVE;
#if DEBUG_SEQ_HDR
        printf("SEQHDR: post-screentools[scc:%d,forceintmv:%d]: off=%u\n",
               hdr->screen_content_tools, hdr->force_integer_mv,
               dav1d_get_bits_pos(gb) - init_bit_pos);
#endif
    }

    hdr->disable_loopfilters_across_tiles = dav1d_get_bit(gb);
    hdr->cdef = dav1d_get_bit(gb);
    hdr->gdf = dav1d_get_bit(gb);
    hdr->restoration = dav1d_get_bit(gb);

    if (hdr->restoration) {
        hdr->rst_disable_mask[0] = dav1d_get_bits(gb, 2);
        if (dav1d_get_bit(gb)) {
            hdr->rst_disable_mask[1] = (dav1d_get_bit(gb) << 1) | 1;
        } else {
            hdr->rst_disable_mask[1] = hdr->rst_disable_mask[0] | 1;
        }
    }
    hdr->ccso = dav1d_get_bit(gb);
    if (hdr->layout != DAV1D_PIXEL_LAYOUT_I400)
        hdr->cfl_ds_filter_index = dav1d_get_bits(gb, 2);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-inloopfilters[disablelfacrosstiles:%d,cdef:%d,gdf:%d,"
           "rst:%d,lrdisablemsk:%x,%x,cccso:%d,cfldsfltidx:%d]: off=%u\n",
           hdr->disable_loopfilters_across_tiles,
           hdr->cdef,
           hdr->gdf,
           hdr->restoration,
           hdr->rst_disable_mask[0] << 1,
           hdr->rst_disable_mask[1] << 1,
           hdr->ccso,
           hdr->cfl_ds_filter_index,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->tcq = dav1d_get_bit(gb) ? 1 + dav1d_get_bit(gb) : 0;
    if (hdr->tcq != 1)
        hdr->parity_hiding = dav1d_get_bit(gb);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-coef[tcq:%d,parityhiding:%d]: off=%u\n",
           hdr->tcq,
           hdr->parity_hiding,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->ext_partitions = dav1d_get_bit(gb);
    if (hdr->ext_partitions)
        hdr->uneven_4way_partitions = dav1d_get_bit(gb);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-partition[ext:%d,uneven4way:%d]: off=%u\n",
           hdr->ext_partitions,
           hdr->uneven_4way_partitions,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    if (hdr->layout != DAV1D_PIXEL_LAYOUT_I400)
        hdr->sdp = dav1d_get_bit(gb);
    if (hdr->sdp)
        hdr->ext_sdp = dav1d_get_bit(gb);
    hdr->ist[0] = dav1d_get_bit(gb);
    hdr->ist[1] = dav1d_get_bit(gb);
    if (hdr->layout != DAV1D_PIXEL_LAYOUT_I400)
        hdr->chroma_dctonly = dav1d_get_bit(gb);
    hdr->inter_ddt = dav1d_get_bit(gb);
    hdr->reduced_tx_part_set = dav1d_get_bit(gb);
    if (hdr->layout != DAV1D_PIXEL_LAYOUT_I400)
        hdr->cctx = dav1d_get_bit(gb);
    hdr->number_of_bits_for_lt_frame_id = dav1d_get_bits(gb, 3);
    hdr->ext_seg = dav1d_get_bit(gb);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-txgrptools[sdp:%d,extsdp:%d,ist:%d,interist:%d,"
           "chromadctonly:%d,interddt:%d,reducedtxtpset:%d,cctx:%d,"
           "ltfidnbits:%d,extseg:%d]: off=%u\n",
           hdr->sdp,
           hdr->ext_sdp,
           hdr->ist[0],
           hdr->ist[1],
           hdr->chroma_dctonly,
           hdr->inter_ddt,
           hdr->reduced_tx_part_set,
           hdr->cctx,
           hdr->number_of_bits_for_lt_frame_id,
           hdr->ext_seg,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    if (hdr->layout != DAV1D_PIXEL_LAYOUT_I400)
        hdr->separate_uv_delta_q = dav1d_get_bit(gb);
    hdr->equal_ac_dc_q = dav1d_get_bit(gb);
    if (!hdr->equal_ac_dc_q) {
        hdr->base_ydc_dq = dav1d_get_bits(gb, 5) - 23;
        hdr->ydc_dq_enabled = dav1d_get_bit(gb);
    }
    if (hdr->layout != DAV1D_PIXEL_LAYOUT_I400) {
        if (!hdr->equal_ac_dc_q) {
            hdr->base_uvdc_dq = dav1d_get_bits(gb, 5) - 23;
            hdr->uvdc_dq_enabled = dav1d_get_bit(gb);
        }
        hdr->base_uvac_dq = dav1d_get_bits(gb, 5) - 23;
        hdr->uvac_dq_enabled = dav1d_get_bit(gb);
        if (hdr->equal_ac_dc_q)
            hdr->base_uvdc_dq = hdr->base_uvac_dq;
    }
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-quantflags[sepuvdq:%d,aceqdc:%d,ydcdq:%d,"
           "fydcdq:%d,uvdcdq:%d,fuvdcdq:%d,uvacdq:%d,fuvacdq:%d]: off=%u\n",
           hdr->separate_uv_delta_q,
           hdr->equal_ac_dc_q,
           hdr->base_ydc_dq,
           hdr->ydc_dq_enabled,
           hdr->base_uvdc_dq,
           hdr->uvdc_dq_enabled,
           hdr->base_uvac_dq,
           hdr->uvac_dq_enabled,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->tiling.present = dav1d_get_bit(gb);
    if (hdr->tiling.present) {
        hdr->tiling.present += dav1d_get_bit(gb);
        parse_tile_info(&hdr->tiling.t, gb, 1, hdr->sb128, hdr->sb128,
                        hdr->max_width, hdr->max_height);
    }
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-tileinfo[%d,%dx%d]: off=%u\n",
           hdr->tiling.present,
           hdr->tiling.t.cols,
           hdr->tiling.t.rows,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->film_grain_present = dav1d_get_bit(gb);
#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-filmgrain[%d]: off=%u\n",
           hdr->film_grain_present,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->cdef_on_skiptx = dav1d_get_bit(gb) ? 1 :
                          dav1d_get_bit(gb) ? 0 : DAV1D_ADAPTIVE;
    hdr->avg_cdf = dav1d_get_bit(gb);
    if (hdr->avg_cdf)
        hdr->avg_cdf_type = dav1d_get_bit(gb);

#if DEBUG_SEQ_HDR
    printf("SEQHDR: post-cdefcdfbits[cdefskiptx:%d,avgcdf:%d,%d]: off=%u\n",
           hdr->cdef_on_skiptx,
           hdr->avg_cdf, hdr->avg_cdf_type,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    hdr->max_pb_aspect_ratio_log2 = dav1d_get_bit(gb) ? 1 + dav1d_get_bit(gb) : 3;
    hdr->df_par_bits = 2 + dav1d_get_bits(gb, 2);
    hdr->user_defined_qmatrix = dav1d_get_bit(gb);
    if (hdr->user_defined_qmatrix) {
        //.. FIXME
    }

#if DEBUG_SEQ_HDR
    printf("SEQHDR: end of seqhdr[maxpbaspect:%d,dfparbits:%d,qm:%d]: off=%u\n",
           hdr->max_pb_aspect_ratio_log2,
           hdr->df_par_bits,
           hdr->user_defined_qmatrix,
           dav1d_get_bits_pos(gb) - init_bit_pos);
#endif

    // We needn't bother flushing the OBU here: we'll check we didn't
    // overrun in the caller and will then discard gb, so there's no
    // point in setting its position properly.

    return check_trailing_bits(gb, strict_std_compliance);

error:
    return DAV1D_ERR(EINVAL);
}

int dav1d_parse_sequence_header(Dav1dSequenceHeader *const out,
                                const uint8_t *const ptr, const size_t sz)
{
    validate_input_or_ret(out != NULL, DAV1D_ERR(EINVAL));
    validate_input_or_ret(ptr != NULL, DAV1D_ERR(EINVAL));
    validate_input_or_ret(sz > 0 && sz <= SIZE_MAX / 2, DAV1D_ERR(EINVAL));

    GetBits gb;
    dav1d_init_get_bits(&gb, ptr, sz);
    int res = DAV1D_ERR(ENOENT);

    do {
        dav1d_get_bit(&gb); // obu_forbidden_bit
        const enum Dav1dObuType type = dav1d_get_bits(&gb, 4);
        const int has_extension = dav1d_get_bit(&gb);
        const int has_length_field = dav1d_get_bit(&gb);
        dav1d_get_bits(&gb, 1 + 8 * has_extension); // ignore

        const uint8_t *obu_end = gb.ptr_end;
        if (has_length_field) {
            const size_t len = dav1d_get_uleb128(&gb);
            if (len > (size_t)(obu_end - gb.ptr)) return DAV1D_ERR(EINVAL);
            obu_end = gb.ptr + len;
        }

        if (type == DAV1D_OBU_SEQ_HDR) {
            if ((res = parse_seq_hdr(out, &gb, 0)) < 0) return res;
            if (gb.ptr > obu_end) return DAV1D_ERR(EINVAL);
            dav1d_bytealign_get_bits(&gb);
        }

        if (gb.error) return DAV1D_ERR(EINVAL);
        assert(gb.state == 0 && gb.bits_left == 0);
        gb.ptr = obu_end;
    } while (gb.ptr < gb.ptr_end);

    return res;
}

static int read_frame_size(Dav1dContext *const c, GetBits *const gb) {
    Dav1dFrameHeader *const hdr = c->frame_hdr;

    if (hdr->frame_size_override && IS_INTER_OR_SWITCH(hdr)) {
        for (int i = 0; i < hdr->n_ref_frames; i++) {
            if (dav1d_get_bit(gb)) {
                const Dav1dThreadPicture *const ref = &c->refs[hdr->refidx[i]].p;
                if (!ref->p.frame_hdr) return -1;
                const Dav1dFrameHeader *const refhdr = ref->p.frame_hdr;
                hdr->width = refhdr->width;
                hdr->height = refhdr->height;
                return 0;
            }
        }
    }

    const Dav1dSequenceHeader *const seqhdr = c->seq_hdr;
    if (hdr->frame_size_override) {
        hdr->width = dav1d_get_bits(gb, seqhdr->width_n_bits) + 1;
        hdr->height = dav1d_get_bits(gb, seqhdr->height_n_bits) + 1;
    } else {
        hdr->width = seqhdr->max_width;
        hdr->height = seqhdr->max_height;
    }
    return 0;
}

static int get_ref_frames(Dav1dContext *const c, const int have_resolution) {
    const Dav1dSequenceHeader *const seqhdr = c->seq_hdr;
    Dav1dFrameHeader *const hdr = c->frame_hdr;
    struct Score {
        int score;
        uint8_t poc;
        int8_t pocdiff;
        uint16_t qidx;
        uint8_t mlayer;
        int8_t res_ratio_log2;
    } ref_info[8];
    uint8_t sort_idx[8];
    int n_refs = 0, have_fwd_refs = 0;
    const unsigned poc = hdr->frame_offset;
    for (int n = 0; n < 8 && !have_fwd_refs; n++) {
        if (!c->refs[n].p.p.frame_hdr) continue;
        have_fwd_refs = get_poc_diff(seqhdr->order_hint_n_bits, poc,
                                     c->refs[n].p.p.frame_hdr->frame_offset) < 0;
    }
    const int mlayer = hdr->mlayer_id, tlayer = hdr->tlayer_id;
    const int w = hdr->width, h = hdr->height;
    int minq = 512, maxq = -1;
    const Dav1dFrameHeader *last_refhdr = NULL;
    for (int n = 0; n < 8; n++) {
        struct Score *const r = &ref_info[n];
        const Dav1dFrameHeader *const refhdr = c->refs[n].p.p.frame_hdr;
        if (!refhdr || refhdr == last_refhdr) continue;
        if (seqhdr->tlayer_dependency_present) {
            if (!(seqhdr->tlayer_dependencies[tlayer] & (1 << refhdr->tlayer_id)))
                continue;
        } else {
            if (tlayer < refhdr->tlayer_id) continue;
        }
        r->mlayer = refhdr->mlayer_id;
        if (seqhdr->mlayer_dependency_present) {
            if (!(seqhdr->mlayer_dependencies[mlayer] & (1 << r->mlayer)))
                continue;
        } else {
            if (mlayer < r->mlayer) continue;
        }
        if (have_resolution &&
            (2 * w < refhdr->width || 2 * h < refhdr->height ||
             w > 16 * refhdr->width || h > 16 * refhdr->height))
        {
            continue;
        }
        r->res_ratio_log2 = -ulog2(refhdr->width * refhdr->height);
        r->poc = refhdr->frame_offset;
        r->pocdiff = get_poc_diff(seqhdr->order_hint_n_bits, poc, r->poc);
        r->qidx = refhdr->quant.yac;
        maxq = imax(r->qidx, maxq);
        minq = imin(r->qidx, minq);
        const unsigned tdist = abs(r->pocdiff) + mlayer - r->mlayer;
        r->score = have_fwd_refs ? (tdist << 6) :
                   128 - (128 >> (imin(tdist, 6))) + imax(tdist - 6, 0);
        r->score += r->res_ratio_log2 * (1 << 5) + r->qidx;
        int m;
        for (m = 0; m < n_refs; m++) {
            const struct Score *const r2 = &ref_info[sort_idx[m]];
            if (r->score == r2->score && r->poc == r2->poc &&
                r->mlayer == r2->mlayer)
            {
                break;
            }
        }
        if (m < n_refs) continue; // ref already exists
        for (; m > 0; m--) {
            const int idx = sort_idx[m - 1];
            const struct Score *const r2 = &ref_info[idx];
            if (r2->score <= r->score) break;
            sort_idx[m] = idx;
        }
        sort_idx[m] = n;
        n_refs++;
        last_refhdr = refhdr;
    }

    if (n_refs == 8) {
        const int q_thr = (maxq + minq + 1) >> 1;
        int maxpocdiff[2] = { 0, 0 }, num[2] = { 0, 0 }, furthest_idx[2];
        for (int n = 0; n < 8; n++) {
            const struct Score *const r = &ref_info[sort_idx[n]];
            if (r->qidx < q_thr) continue;
            if (r->pocdiff > 0) {
                if (r->pocdiff > maxpocdiff[0]) {
                    maxpocdiff[0] = r->pocdiff;
                    furthest_idx[0] = n;
                }
                num[0]++;
            } else if (r->pocdiff < 0) {
                if (r->pocdiff < maxpocdiff[1]) {
                    maxpocdiff[1] = r->pocdiff;
                    furthest_idx[1] = n;
                }
                num[1]++;
            }
        }
        const int idx = num[0] > num[1] ? furthest_idx[0] :
                        num[0] < num[1] ? furthest_idx[1] :
                        furthest_idx[maxpocdiff[0] < maxpocdiff[1]];
        if (idx < 7) {
            memmove(&sort_idx[idx], &sort_idx[idx + 1], 7 - idx);
            sort_idx[7] = idx;
        }
    }

    for (int n = 0; n < 7; n++)
        hdr->refidx[n] = sort_idx[n < n_refs ? n : 0];

    return imin(7, n_refs);
}

static void derive_pri_sec_ref(const Dav1dContext *const c, int refs[2]) {
    const Dav1dSequenceHeader *const seqhdr = c->seq_hdr;
    const Dav1dFrameHeader *const hdr = c->frame_hdr;
    refs[0] = DAV1D_PRIMARY_REF_NONE;
    int best_qdiff[2], best_pocdiff[2], best = 0;
    const int qidx = hdr->quant.yac, poc = hdr->frame_offset;
    for (int i = 0; i < hdr->n_ref_frames; i++) {
        const Dav1dFrameHeader *const refhdr = c->refs[hdr->refidx[i]].p.p.frame_hdr;
        if (!refhdr || IS_KEY_OR_INTRA(refhdr)) continue;
        const int ref_qidx = refhdr->quant.yac, qdiff = abs(ref_qidx - qidx);
        const int ref_poc = refhdr->frame_offset, pocdiff =
            abs(get_poc_diff(seqhdr->order_hint_n_bits, poc, ref_poc));
        for (int n = 0, m = best; n < 2; n++, m = !m) {
            if (refs[m] == DAV1D_PRIMARY_REF_NONE || qdiff < best_qdiff[m] ||
                (qdiff == best_qdiff[m] && pocdiff < best_pocdiff[m]))
            {
                refs[!best] = i;
                best_pocdiff[!best] = pocdiff;
                best_qdiff[!best] = qdiff;
                if (!n) best = !best;
                break;
            }
        }
    }
    if (best) {
        const int tmp = refs[0];
        refs[0] = refs[1];
        refs[1] = tmp;
    }
}

static int parse_frame_hdr(Dav1dContext *const c, GetBits *const gb,
                           const enum Dav1dObuType obu_type)
{
#define DEBUG_FRAME_HDR 0

#if DEBUG_FRAME_HDR
    const uint8_t *const init_ptr = &gb->ptr[-1];
#endif
    const Dav1dSequenceHeader *const seqhdr = c->seq_hdr;
    Dav1dFrameHeader *const hdr = c->frame_hdr;

    hdr->id = dav1d_get_vlc(gb);
    if (hdr->id) goto error;
    const int seqhdr_idx = dav1d_get_vlc(gb);
    if (seqhdr_idx != seqhdr->id) goto error;
#if DEBUG_FRAME_HDR
    printf("HDR: post-ids[f=%d,s=%d]: off=%td\n",
           hdr->id, seqhdr->id,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    hdr->show_existing_frame = obu_type == DAV1D_OBU_SEF;
    if (hdr->show_existing_frame) {
        hdr->existing_frame_idx = dav1d_get_bits(gb, seqhdr->ref_frames_log2);
        if (hdr->existing_frame_idx >= seqhdr->ref_frames) goto error;
        if (seqhdr->decoder_model_info_present && !seqhdr->equal_picture_interval)
            hdr->frame_presentation_delay = dav1d_get_bits(gb, seqhdr->frame_presentation_delay_length);
#if DEBUG_FRAME_HDR
        printf("HDR: post-existing_frame_idx[%d]: off=%td\n",
               hdr->existing_frame_idx,
               (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
        return 0;
    }

    if (seqhdr->reduced_still_picture_header) {
        hdr->frame_type = DAV1D_FRAME_TYPE_KEY;
        hdr->show_frame = 1;
    } else {
        hdr->frame_type = obu_type == DAV1D_OBU_SWITCH ||
                          obu_type == DAV1D_OBU_RAS ? DAV1D_FRAME_TYPE_SWITCH :
                          obu_type == DAV1D_OBU_TIP ||
                          obu_type == DAV1D_OBU_BRIDGE ? DAV1D_FRAME_TYPE_INTER :
                          dav1d_get_bit(gb) ? DAV1D_FRAME_TYPE_INTER :
                          dav1d_get_bit(gb) ? DAV1D_FRAME_TYPE_KEY :
                                              DAV1D_FRAME_TYPE_INTRA;
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-frametype[%d]: off=%td\n",
           hdr->frame_type,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    if (hdr->frame_type == DAV1D_FRAME_TYPE_KEY) {
        hdr->ltr_id = dav1d_get_bits(gb, seqhdr->number_of_bits_for_lt_frame_id);
#if DEBUG_FRAME_HDR
        printf("HDR: post-ltr_id[%d]: off=%td\n",
               hdr->ltr_id,
               (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    } else if (obu_type == DAV1D_OBU_RAS) {
        hdr->n_ref_frames = dav1d_get_bits(gb, 3);
        for (int n = 0; n < hdr->n_ref_frames; n++)
            hdr->refidx[n] =
                dav1d_get_bits(gb, seqhdr->number_of_bits_for_lt_frame_id);
#if DEBUG_FRAME_HDR
        printf("HDR: post-ltr_setup[%d]: off=%td\n",
               hdr->n_ref_frames,
               (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    }
    if (obu_type != DAV1D_OBU_BRIDGE) {
        if (!seqhdr->reduced_still_picture_header)
            hdr->show_frame = dav1d_get_bit(gb);
        if (hdr->show_frame) {
            hdr->showable_frame = hdr->frame_type != DAV1D_FRAME_TYPE_KEY;
        } else {
            hdr->showable_frame = dav1d_get_bit(gb);
        }
        if ((hdr->show_frame || hdr->showable_frame) &&
            seqhdr->decoder_model_info_present && !seqhdr->equal_picture_interval)
        {
            hdr->frame_presentation_delay = dav1d_get_bits(gb, seqhdr->frame_presentation_delay_length);
        }
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-frametype_bits[%d,show=%d|%d]: off=%td\n",
           hdr->frame_type, hdr->show_frame, hdr->showable_frame,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    hdr->primary_ref_frame = DAV1D_PRIMARY_REF_NONE;
    if (!seqhdr->reduced_still_picture_header) {
        hdr->frame_size_override = hdr->frame_type == DAV1D_FRAME_TYPE_SWITCH ?
                                   1 : dav1d_get_bit(gb);
        hdr->frame_offset = dav1d_get_bits(gb, seqhdr->order_hint_n_bits);
        int did_signal_pri_ref = -1;
        if (IS_INTER_OR_SWITCH(hdr) &&
            (did_signal_pri_ref = dav1d_get_bit(gb)))
        {
            hdr->primary_ref_frame = dav1d_get_bits(gb, 3);
            hdr->primary_ref_signaled = 1;
        }
#if DEBUG_FRAME_HDR
        printf("HDR: post-frame_size_override_flag[%d,poc=%d,p_ref=%d|%d]: off=%td\n",
               hdr->frame_size_override, hdr->frame_offset, did_signal_pri_ref,
               hdr->primary_ref_frame, (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    }

    if (seqhdr->decoder_model_info_present) {
        hdr->buffer_removal_time_present = dav1d_get_bit(gb);
        if (hdr->buffer_removal_time_present) {
            for (int i = 0; i < c->seq_hdr->num_operating_points; i++) {
                const struct Dav1dSequenceHeaderOperatingPoint *const seqop = &seqhdr->operating_points[i];
                struct Dav1dFrameHeaderOperatingPoint *const op = &hdr->operating_points[i];
                if (seqop->decoder_model_param_present) {
                    int in_temporal_layer = (seqop->idc >> hdr->tlayer_id) & 1;
                    int in_spatial_layer  = (seqop->idc >> (hdr->mlayer_id + 8)) & 1;
                    if (!seqop->idc || (in_temporal_layer && in_spatial_layer))
                        op->buffer_removal_time = dav1d_get_bits(gb, seqhdr->buffer_removal_delay_length);
                }
            }
        }
#if DEBUG_FRAME_HDR
        printf("HDR: post-buffer_removal_timings[%d]: off=%td\n",
               hdr->buffer_removal_time_present,
               (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    }

    if (hdr->frame_type == DAV1D_FRAME_TYPE_SWITCH ||
        (hdr->frame_type == DAV1D_FRAME_TYPE_KEY && hdr->show_frame))
    {
        hdr->refresh_frame_flags = (1 << seqhdr->ref_frames) - 1;
    } else if (seqhdr->short_refresh_frame_flags) {
        const int refresh = dav1d_get_bit(gb);
        if (refresh) {
            const int refresh_idx = dav1d_get_bits(gb, seqhdr->ref_frames_log2);
            if (refresh_idx >= seqhdr->ref_frames) goto error;
            hdr->refresh_frame_flags = 1 << refresh_idx;
        }
    } else {
        hdr->refresh_frame_flags = dav1d_get_bits(gb, seqhdr->ref_frames);
        if (c->strict_std_compliance && hdr->frame_type == DAV1D_FRAME_TYPE_INTRA &&
            hdr->refresh_frame_flags == (1 << seqhdr->ref_frames) - 1)
        {
            goto error;
        }
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-refresh_frame_flags[%x]: off=%td\n",
           hdr->refresh_frame_flags,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    if (IS_INTER_OR_SWITCH(hdr)) {
        if (hdr->frame_type == DAV1D_FRAME_TYPE_SWITCH ||
            seqhdr->explicit_ref_frame_map)
        {
            // explicit ref frame signaling
            hdr->n_ref_frames = dav1d_get_bits(gb, 3);
            if (hdr->n_ref_frames > imin(7, seqhdr->ref_frames)) goto error;
            for (int n = 0; n < hdr->n_ref_frames; n++) {
                hdr->refidx[n] = dav1d_get_bits(gb, seqhdr->ref_frames_log2);
                if (hdr->refidx[n] >= seqhdr->ref_frames) goto error;
            }
        } else {
            // implicit ref frame scoring (this will fill hdr->refidx[])
            hdr->n_ref_frames = get_ref_frames(c, 0);
        }
    }

    if (read_frame_size(c, gb) < 0) goto error;
#if DEBUG_FRAME_HDR
    printf("HDR: post-framesize[%dx%d]: off=%td\n",
           hdr->width, hdr->height,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    if (IS_INTER_OR_SWITCH(hdr)) {
        if (hdr->frame_type == DAV1D_FRAME_TYPE_INTER &&
            !seqhdr->explicit_ref_frame_map)
        {
            // include resolution constraints
            hdr->n_ref_frames = get_ref_frames(c, 1);
        }

        // FIXME bru

        if (seqhdr->ref_frame_mvs)
            hdr->use_ref_frame_mvs = dav1d_get_bit(gb);
        hdr->tmvp_sample_step = 1 +
            (hdr->use_ref_frame_mvs && hdr->n_ref_frames > 1 && dav1d_get_bit(gb));
        if (seqhdr->lf_sub_pu)
            hdr->loopfilter.lf_sub_pu = dav1d_get_bit(gb);
        if (seqhdr->tip && hdr->n_ref_frames > 1 && hdr->use_ref_frame_mvs) {
            if (obu_type == DAV1D_OBU_TIP) {
                hdr->tip.frame_mode = 2; // output
                hdr->opfl_refine_type =
                    2 * (seqhdr->opfl_refine && seqhdr->tip_refine_mv);
            } else {
                hdr->tip.frame_mode = dav1d_get_bit(gb); // 1: ref, or 0: disabled
                hdr->opfl_refine_type =
                    seqhdr->opfl_refine < 3 /* auto */ ? seqhdr->opfl_refine :
                    dav1d_get_bit(gb) ? 1 /* switchable */ :
                                        2 * dav1d_get_bit(gb) /* all or none */;
            }
            if (hdr->tip.frame_mode) {
                if (seqhdr->tip_hole_fill)
                    hdr->tip.hole_fill = dav1d_get_bit(gb);
                if (/* do not have both-sides-refs || */
                    (!seqhdr->opfl_refine && !seqhdr->refine_mv))
                {
                    hdr->tip.global_wtd_idx = dav1d_get_bits(gb, 3);
                }
                if (hdr->tip.frame_mode == 2) {
                    if (hdr->loopfilter.lf_sub_pu)
                        hdr->tip.filter_level = dav1d_get_bit(gb);
                    if (!dav1d_get_bit(gb)) {
                        hdr->tip.gmv.y = dav1d_get_bits(gb, 4);
                        hdr->tip.gmv.x = dav1d_get_bits(gb, 4);
                        if (hdr->tip.gmv.y && dav1d_get_bit(gb))
                            hdr->tip.gmv.y = -hdr->tip.gmv.y;
                        if (hdr->tip.gmv.x && dav1d_get_bit(gb))
                            hdr->tip.gmv.x = -hdr->tip.gmv.x;
                    }
                    hdr->tip.subpel_filter =
                        dav1d_get_bit(gb) ? DAV1D_FILTER_8TAP_SHARP :
                        dav1d_get_bit(gb) ? DAV1D_FILTER_8TAP_REGULAR :
                                            DAV1D_FILTER_8TAP_SMOOTH;
                }
            }
        }
#if DEBUG_FRAME_HDR
        printf("HDR: post-tip[refmvs:%d,tmvp:%d,lfsubpu:%d,tip:%d]: off=%td\n",
               hdr->use_ref_frame_mvs, hdr->tmvp_sample_step,
               hdr->loopfilter.lf_sub_pu, hdr->tip.frame_mode,
               (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    }

    hdr->allow_screen_content_tools =
        seqhdr->screen_content_tools == DAV1D_ADAPTIVE ?
        dav1d_get_bit(gb) : seqhdr->screen_content_tools;
    if (hdr->allow_screen_content_tools)
        hdr->force_integer_mv = seqhdr->force_integer_mv == DAV1D_ADAPTIVE ?
                                dav1d_get_bit(gb) : seqhdr->force_integer_mv;
    hdr->allow_intrabc = dav1d_get_bit(gb);
    if (hdr->allow_intrabc) {
        if (IS_KEY_OR_INTRA(hdr))
            hdr->allow_global_intrabc = dav1d_get_bit(gb);
        hdr->allow_local_intrabc = !hdr->allow_global_intrabc || dav1d_get_bit(gb);
    }
    if (IS_INTER_OR_SWITCH(hdr))
        hdr->max_drl_bits = seqhdr->allow_frame_max_drl_bits ?
            dav1d_get_ref_uniform(gb, 3, seqhdr->def_max_drl_bits) + 1 :
            seqhdr->def_max_drl_bits;
    if (hdr->allow_intrabc) {
        hdr->max_bvp_drl_bits = seqhdr->allow_max_bvp_drl_bits ?
            dav1d_get_ref_uniform(gb, 3, seqhdr->def_max_bvp_drl_bits) + 1 :
            seqhdr->def_max_bvp_drl_bits;
    }

#if DEBUG_FRAME_HDR
    printf("HDR: post-screencontent_ibc[%d,%d,%d,%d,%d,%d,%d]: off=%td\n",
           hdr->allow_screen_content_tools,
           hdr->force_integer_mv,
           hdr->allow_intrabc,
           hdr->allow_global_intrabc,
           hdr->allow_local_intrabc,
           hdr->max_drl_bits,
           hdr->max_bvp_drl_bits,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    if (IS_INTER_OR_SWITCH(hdr)) {
        if (!hdr->force_integer_mv)
            hdr->mv_precision = dav1d_get_bit(gb) ? 2 : 1 + 2 * dav1d_get_bit(gb);
        hdr->subpel_filter_mode = dav1d_get_bit(gb) ? DAV1D_FILTER_SWITCHABLE :
                                                      dav1d_get_bits(gb, 2);
        if (seqhdr->frame_motion_modes_present) {
            hdr->motion_modes = 1;
            for (int n = 2; n <= 16; n <<= 1)
                if ((seqhdr->motion_modes & n) && dav1d_get_bit(gb))
                    hdr->motion_modes |= n;
        } else {
            hdr->motion_modes = seqhdr->motion_modes;
        }
#if DEBUG_FRAME_HDR
    printf("HDR: post-frametype-specific-bits[mvprec:%d,flt:%d,mm:%d]: off=%td\n",
           hdr->mv_precision, hdr->subpel_filter_mode,
           hdr->motion_modes,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    }

    hdr->disable_cdf_update = dav1d_get_bit(gb);
    if (!seqhdr->reduced_still_picture_header && !hdr->disable_cdf_update)
        hdr->refresh_context = !dav1d_get_bit(gb);

#if DEBUG_FRAME_HDR
    printf("HDR: post-refresh_context[%d,%d]: off=%td\n",
           hdr->disable_cdf_update, hdr->refresh_context,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    // tile data
    hdr->sb128 = IS_INTER_OR_SWITCH(hdr) ? seqhdr->sb128 : !!seqhdr->sb128;
    if (seqhdr->tiling.present == 1 ||
        (seqhdr->tiling.present == DAV1D_ADAPTIVE && dav1d_get_bit(gb)))
    {
        hdr->tiling.t = seqhdr->tiling.t;
        if (hdr->sb128 != seqhdr->sb128) {
            assert(hdr->sb128 == 1 && seqhdr->sb128 == 2 &&
                   IS_KEY_OR_INTRA(hdr));
            int n;
            for (n = 0; n < hdr->tiling.t.rows; n++)
                hdr->tiling.t.row_start_sb[n] *= 2;
            hdr->tiling.t.row_start_sb[n] = imin(hdr->tiling.t.row_start_sb[n] * 2,
                                                 (hdr->height + 127) >> 7);
            for (n = 0; n < hdr->tiling.t.cols; n++)
                hdr->tiling.t.col_start_sb[n] *= 2;
            hdr->tiling.t.col_start_sb[n] = imin(hdr->tiling.t.col_start_sb[n] * 2,
                                                 (hdr->width + 127) >> 7);
        }
    } else {
        const int sbmul = seqhdr->sb128 == 2 && IS_KEY_OR_INTRA(hdr) ? 2 : 1;
        parse_tile_info(&hdr->tiling.t, gb, sbmul, hdr->sb128, seqhdr->sb128,
                        hdr->width, hdr->height);
        if (sbmul == 2) {
            hdr->tiling.t.row_start_sb[hdr->tiling.t.rows] =
                imin(hdr->tiling.t.row_start_sb[hdr->tiling.t.rows] * 2,
                     (hdr->height + 127) >> 7);
            hdr->tiling.t.col_start_sb[hdr->tiling.t.cols] =
                imin(hdr->tiling.t.col_start_sb[hdr->tiling.t.cols] * 2,
                     (hdr->width + 127) >> 7);
        }
    }
    if (hdr->tiling.t.log2_cols || hdr->tiling.t.log2_rows) {
        if (!seqhdr->avg_cdf_type)
            hdr->tiling.update = dav1d_get_bits(gb, hdr->tiling.t.log2_cols +
                                                    hdr->tiling.t.log2_rows);
        if (hdr->tiling.update >= hdr->tiling.t.cols * hdr->tiling.t.rows)
            goto error;
        hdr->tiling.n_bytes = dav1d_get_bits(gb, 2) + 1;
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-tiling[%dx%dtiles,%dbytes]: off=%td\n",
           hdr->tiling.t.cols, hdr->tiling.t.rows, hdr->tiling.n_bytes,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    // quant data
    hdr->quant.yac = dav1d_get_bits(gb, 8 + !!seqhdr->hbd);
    if (seqhdr->ydc_dq_enabled && dav1d_get_bit(gb))
        hdr->quant.ydc_delta = dav1d_get_sbits(gb, 7);
    if (seqhdr->layout != DAV1D_PIXEL_LAYOUT_I400 && (seqhdr->uvdc_dq_enabled ||
                                                      seqhdr->uvac_dq_enabled))
    {
        // If the sequence header says that delta_q might be different
        // for U, V, we must check whether it actually is for this
        // frame.
        const int diff_uv_delta = seqhdr->separate_uv_delta_q ? dav1d_get_bit(gb) : 0;
        if (seqhdr->uvdc_dq_enabled && dav1d_get_bit(gb))
            hdr->quant.udc_delta = dav1d_get_sbits(gb, 7);
        if (seqhdr->uvac_dq_enabled && dav1d_get_bit(gb))
            hdr->quant.uac_delta = dav1d_get_sbits(gb, 7);
        if (diff_uv_delta) {
            if (seqhdr->uvdc_dq_enabled && dav1d_get_bit(gb))
                hdr->quant.vdc_delta = dav1d_get_sbits(gb, 7);
            if (seqhdr->uvac_dq_enabled && dav1d_get_bit(gb))
                hdr->quant.vac_delta = dav1d_get_sbits(gb, 7);
        } else {
            hdr->quant.vdc_delta = hdr->quant.udc_delta;
            hdr->quant.vac_delta = hdr->quant.uac_delta;
        }
    }

    hdr->secondary_ref_frame = DAV1D_PRIMARY_REF_NONE;
    if (IS_INTER_OR_SWITCH(hdr)) {
        int refs[2];
        derive_pri_sec_ref(c, refs);
        if (!hdr->primary_ref_signaled)
            hdr->primary_ref_frame = refs[0];
        if (hdr->primary_ref_frame != DAV1D_PRIMARY_REF_NONE)
            hdr->secondary_ref_frame = refs[refs[0] == hdr->primary_ref_frame];
    }

#if DEBUG_FRAME_HDR
    printf("HDR: post-quant[yac:%d,deltas=ydc:%d,uac:%d/dc:%d,vac:%d/dc:%d]: off=%td\n",
           hdr->quant.yac, hdr->quant.ydc_delta,
           hdr->quant.uac_delta, hdr->quant.udc_delta,
           hdr->quant.vac_delta, hdr->quant.vdc_delta,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    // segmentation data
    hdr->segmentation.enabled = dav1d_get_bit(gb);
    if (hdr->segmentation.enabled) {
        if (hdr->primary_ref_frame == DAV1D_PRIMARY_REF_NONE) {
            hdr->segmentation.update_map = 1;
            hdr->segmentation.update_data = 1;
        } else {
            hdr->segmentation.update_map = dav1d_get_bit(gb);
            if (hdr->segmentation.update_map)
                hdr->segmentation.temporal = dav1d_get_bit(gb);
            hdr->segmentation.update_data = dav1d_get_bit(gb);
        }

        if (hdr->segmentation.update_data) {
            hdr->segmentation.seg_data.last_active_segid = -1;
            const int n_segments = 8 << seqhdr->ext_seg;
            for (int i = 0; i < n_segments; i++) {
                Dav1dSegmentationData *const seg =
                    &hdr->segmentation.seg_data.d[i];
                if (dav1d_get_bit(gb)) {
                    seg->delta_q = dav1d_get_sbits(gb, 9);
                    hdr->segmentation.seg_data.last_active_segid = i;
                }
                if (dav1d_get_bit(gb)) {
                    seg->delta_lf_y_v = dav1d_get_sbits(gb, 7);
                    hdr->segmentation.seg_data.last_active_segid = i;
                }
                if (dav1d_get_bit(gb)) {
                    seg->delta_lf_y_h = dav1d_get_sbits(gb, 7);
                    hdr->segmentation.seg_data.last_active_segid = i;
                }
                if (dav1d_get_bit(gb)) {
                    seg->delta_lf_u = dav1d_get_sbits(gb, 7);
                    hdr->segmentation.seg_data.last_active_segid = i;
                }
                if (dav1d_get_bit(gb)) {
                    seg->delta_lf_v = dav1d_get_sbits(gb, 7);
                    hdr->segmentation.seg_data.last_active_segid = i;
                }
                if (dav1d_get_bit(gb)) {
                    seg->ref = dav1d_get_bits(gb, 3);
                    hdr->segmentation.seg_data.last_active_segid = i;
                    hdr->segmentation.seg_data.preskip = 1;
                } else {
                    seg->ref = -1;
                }
                if ((seg->skip = dav1d_get_bit(gb))) {
                    hdr->segmentation.seg_data.last_active_segid = i;
                    hdr->segmentation.seg_data.preskip = 1;
                }
                if ((seg->globalmv = dav1d_get_bit(gb))) {
                    hdr->segmentation.seg_data.last_active_segid = i;
                    hdr->segmentation.seg_data.preskip = 1;
                }
            }
        } else {
            // segmentation.update_data was false so we should copy
            // segmentation data from the reference frame.
            assert(hdr->primary_ref_frame != DAV1D_PRIMARY_REF_NONE);
            const int pri_ref = hdr->refidx[hdr->primary_ref_frame];
            if (!c->refs[pri_ref].p.p.frame_hdr) goto error;
            hdr->segmentation.seg_data =
                c->refs[pri_ref].p.p.frame_hdr->segmentation.seg_data;
        }
    } else {
        for (int i = 0; i < DAV1D_MAX_SEGMENTS; i++)
            hdr->segmentation.seg_data.d[i].ref = -1;
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-segmentation[%d]: off=%td\n",
           hdr->segmentation.enabled,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    hdr->quant.qm.enabled = dav1d_get_bit(gb);
    if (hdr->quant.qm.enabled) {
        hdr->quant.qm.num = hdr->segmentation.enabled ?
                            dav1d_get_bits(gb, 2) + 1 : 1;
        for (int n = 0; n < hdr->quant.qm.num; n++) {
            hdr->quant.qm.y[n] = dav1d_get_bits(gb, 4);
            if (seqhdr->layout != DAV1D_PIXEL_LAYOUT_I400) {
                if (dav1d_get_bit(gb)) {
                    hdr->quant.qm.u[n] = hdr->quant.qm.v[n] = hdr->quant.qm.y[n];
                } else {
                    hdr->quant.qm.u[n] = dav1d_get_bits(gb, 4);
                    hdr->quant.qm.v[n] = seqhdr->separate_uv_delta_q ?
                                         dav1d_get_bits(gb, 4) :
                                         hdr->quant.qm.u[n];
                }
            }
        }
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-qm[%d]: off=%td\n",
           hdr->quant.qm.enabled,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    // delta q
    if (hdr->quant.yac) {
        hdr->delta.q.present = dav1d_get_bit(gb);
        if (hdr->delta.q.present) {
            hdr->delta.q.res_log2 = dav1d_get_bits(gb, 2);
            if (!hdr->allow_intrabc) {
                hdr->delta.lf.present = dav1d_get_bit(gb);
                if (hdr->delta.lf.present) {
                    hdr->delta.lf.res_log2 = dav1d_get_bits(gb, 2);
                    hdr->delta.lf.multi = dav1d_get_bit(gb);
                }
            }
        }
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-delta_q_lf[q:%d,lf:%d]: off=%td\n",
           hdr->delta.q.present, hdr->delta.lf.present,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    // derive lossless flags
    const int delta_lossless = !hdr->quant.ydc_delta && !hdr->quant.udc_delta &&
        !hdr->quant.uac_delta && !hdr->quant.vdc_delta && !hdr->quant.vac_delta;
    hdr->all_lossless = 1;
    for (int i = 0; i < DAV1D_MAX_SEGMENTS; i++) {
        hdr->segmentation.qidx[i] = hdr->segmentation.enabled ?
            iclip_u8(hdr->quant.yac + hdr->segmentation.seg_data.d[i].delta_q) :
            hdr->quant.yac;
        hdr->segmentation.lossless[i] =
            !hdr->segmentation.qidx[i] && delta_lossless;
        hdr->all_lossless &= hdr->segmentation.lossless[i];

        // FIXME when using qm & segmentaiton, there are also some
        // bits here which qm to use per seg
    }

    if (!hdr->all_lossless)
        hdr->tcq = seqhdr->tcq == DAV1D_ADAPTIVE ? dav1d_get_bit(gb) : seqhdr->tcq;
    if (!hdr->all_lossless && !hdr->tcq && seqhdr->parity_hiding)
        hdr->parity_hiding = dav1d_get_bit(gb);

#if DEBUG_FRAME_HDR
    printf("HDR: post-tcq_parity[tcq:%d,par:%d]: off=%td\n",
           hdr->tcq, hdr->parity_hiding,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    // loopfilter
    if (!hdr->all_lossless) {
        hdr->loopfilter.level_y[0] = dav1d_get_bit(gb);
        hdr->loopfilter.level_y[1] = dav1d_get_bit(gb);
        if (seqhdr->layout != DAV1D_PIXEL_LAYOUT_I400 &&
            (hdr->loopfilter.level_y[0] || hdr->loopfilter.level_y[1]))
        {
            hdr->loopfilter.level_u = dav1d_get_bit(gb);
            hdr->loopfilter.level_v = dav1d_get_bit(gb);
        }
        const int bits = seqhdr->df_par_bits, off = 1 << (bits - 1);
        if (hdr->loopfilter.level_y[0] && dav1d_get_bit(gb))
            hdr->loopfilter.delta_q_y[0] = dav1d_get_bits(gb, bits) - off;
        if (hdr->loopfilter.level_y[1])
            hdr->loopfilter.delta_q_y[1] = dav1d_get_bit(gb) ?
                                           dav1d_get_bits(gb, bits) - off :
                                           hdr->loopfilter.delta_q_y[0];
        if (hdr->loopfilter.level_u && dav1d_get_bit(gb))
            hdr->loopfilter.delta_q_u = dav1d_get_bits(gb, bits) - off;
        if (hdr->loopfilter.level_v && dav1d_get_bit(gb))
            hdr->loopfilter.delta_q_v = dav1d_get_bits(gb, bits) - off;
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-deblock[y:%d|%d,u:%d,v:%d]: off=%td\n",
           hdr->loopfilter.level_y[0], hdr->loopfilter.level_y[1],
           hdr->loopfilter.level_u, hdr->loopfilter.level_v,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    if (!hdr->all_lossless && seqhdr->gdf /* && not large-scale tiles */) {
        const int gdf_bs = 128 << (hdr->sb128 == 2);
        hdr->gdf.enabled = dav1d_get_bit(gb);
        if (hdr->gdf.enabled) {
            if (imax(hdr->width, hdr->height) > gdf_bs)
                hdr->gdf.enabled += dav1d_get_bit(gb);
            hdr->gdf.qp_idx = dav1d_get_bits(gb, 2);
            hdr->gdf.scale_idx = dav1d_get_bits(gb, 2);
        }
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-gdf[%d]: off=%td\n",
           hdr->gdf.enabled,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    // cdef
    if (!hdr->all_lossless && seqhdr->cdef) {
        hdr->cdef.enabled = dav1d_get_bit(gb);
        if (hdr->cdef.enabled) {
            hdr->cdef.damping = dav1d_get_bits(gb, 2) + 3;
            hdr->cdef.n_strengths = dav1d_get_bits(gb, 3) + 1;
            hdr->cdef.on_skiptx = seqhdr->cdef_on_skiptx == DAV1D_ADAPTIVE ?
                                  dav1d_get_bit(gb) : seqhdr->cdef_on_skiptx;
            for (int i = 0; i < hdr->cdef.n_strengths; i++) {
                hdr->cdef.y_strength[i] = dav1d_get_bits(gb, 6 - 4 * dav1d_get_bit(gb));
                if (seqhdr->layout != DAV1D_PIXEL_LAYOUT_I400)
                    hdr->cdef.uv_strength[i] = dav1d_get_bits(gb, 6 - 4 * dav1d_get_bit(gb));
            }
        }
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-cdef[%d]: off=%td\n",
           hdr->cdef.enabled,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    // restoration
    if (!hdr->all_lossless && seqhdr->restoration) {
        for (int p = 0; p < 3; p++) {
            const unsigned disable_mask = seqhdr->rst_disable_mask[!!p];
            if (disable_mask == 0) {
                hdr->restoration.p[p].type = dav1d_get_bits(gb, 2);
            } else if (disable_mask == 3) {
                hdr->restoration.p[p].type = DAV1D_RESTORATION_NONE;
            } else {
                hdr->restoration.p[p].type = dav1d_get_bit(gb) * (3 - disable_mask);
            }

            if (hdr->restoration.p[p].type >= DAV1D_RESTORATION_NS_WIENER) {
                hdr->restoration.p[p].ns.frame_filters_on = dav1d_get_bit(gb);
                if (hdr->restoration.p[p].ns.frame_filters_on) {
                    if (IS_INTER_OR_SWITCH(hdr))
                        hdr->restoration.p[p].ns.temporal = dav1d_get_bit(gb);
                    if (hdr->restoration.p[p].ns.temporal) {
                        // FIXME find refidx
                    } else {
                        const int val = dav1d_get_bits(gb, 3);
                        hdr->restoration.p[p].ns.num_classes =
                            1 + val + imax(val - 3, 0) + imax(val - 5, 0) * 2;
                    }
                } else {
                    hdr->restoration.p[p].ns.num_classes = 1;
                }
            }
        }

        hdr->restoration.unit_size[0] = 9;
        if (hdr->restoration.p[0].type) {
            if (dav1d_get_bit(gb)) {
                hdr->restoration.unit_size[0]--;
            } else if (hdr->sb128 < 2 && !dav1d_get_bit(gb)) {
                hdr->restoration.unit_size[0] -=
                    2 + (!hdr->sb128 && !dav1d_get_bit(gb));
            }
        }

        const int ss = seqhdr->layout != DAV1D_PIXEL_LAYOUT_I444;
        hdr->restoration.unit_size[1] = 9 - ss;
        if (hdr->restoration.p[1].type || hdr->restoration.p[2].type) {
            if (dav1d_get_bit(gb)) {
                hdr->restoration.unit_size[1]--;
            } else if (hdr->sb128 < 2 && !dav1d_get_bit(gb)) {
                hdr->restoration.unit_size[1] -=
                    2 + (!hdr->sb128 && !dav1d_get_bit(gb));
            }
        }

        for (int p = 0; p < 3; p++) {
            int8_t ref_filters[48][18];
            if (!hdr->restoration.p[p].ns.frame_filters_on) continue;
            const int n_feat = 16 + 2 * !!p;
            int i = 0;
            const int n_ref_filters = seqhdr->rst_disable_mask[!!p] & 1 ? 16 :
                48 - hdr->restoration.p[p].ns.num_classes;
            for (int r = 0; r < hdr->n_ref_frames; r++) {
                const Dav1dFrameHeader *const ref_hdr =
                    c->refs[hdr->refidx[r]].p.p.frame_hdr;
                for (int dir = (const int8_t[]){ 0, +1, -1 }[p], p2 = p;;
                     p2 += dir, dir = 0)
                {
                    if (ref_hdr->restoration.p[p2].ns.frame_filters_on) {
                        const int n_classes =
                            imin(n_ref_filters - i,
                                 ref_hdr->restoration.p[p2].ns.num_classes);
                        for (int n = 0; n < n_classes; n++)
                            memcpy(ref_filters[i++],
                                   ref_hdr->restoration.p[p2].ns.filter[n],
                                   n_feat);
                    }
                    if (!dir) break;
                }
            }
            const int n_filters = seqhdr->rst_disable_mask[!!p] & 1 ? 16 : 64;
            const int n_classes = hdr->restoration.p[p].ns.num_classes;
            uint8_t grp_cnt[3], grp_ref_cnt[3] = { 0 };
            assert(n_classes > 0);
            grp_cnt[0] = n_classes;
            grp_cnt[1] = i;
            grp_cnt[2] = n_filters - (grp_cnt[0] + grp_cnt[1]);
            uint8_t filter_refs[64];
            int pred_grp = 2 - (grp_cnt[1] > 2);
            const int nnz_grps = 1 + !!grp_cnt[1] + !!grp_cnt[2];
            for (int n = 0; n < n_classes; n++) {
                int group;
                if (nnz_grps == 1 || !dav1d_get_bit(gb)) {
                    group = pred_grp;
                } else if (nnz_grps == 2) {
                    group = 2 - !grp_cnt[2] - pred_grp;
                } else if (dav1d_get_bit(gb)) {
                    group = 2 - (pred_grp == 2);
                } else {
                    group = pred_grp == 0;
                }
                if (++grp_ref_cnt[group] + (group < pred_grp) > grp_ref_cnt[pred_grp])
                    pred_grp = group;
                const int base = grp_cnt[0] * !!group + grp_cnt[1] * (group == 2);
                const int range = group ? grp_cnt[group] : n + 1;
                filter_refs[n] = base + (range == 1 ? 0 :
                    dav1d_get_bits_subexp_u(gb, range >> 1, range, 4));
            }
            unsigned exact_match_mask = 0;
            // FIXME use dav1d_get_bits()
            for (int n = 0, mask = 1; n < n_classes; n++, mask <<= 1) {
                exact_match_mask |= mask * dav1d_get_bit(gb);
            }
            const unsigned *const masks = p ? dav1d_subset_masks_uv : dav1d_subset_masks_y;
            const int8_t (*const cf_range)[2] = p ? dav1d_ns_wiener_coef_range_uv :
                                                    dav1d_ns_wiener_coef_range_y;
            static const uint8_t shuffled_index[] = {
                16, 7,  58, 21, 12, 61, 26, 38, 18, 30, 50, 45, 23, 49, 43, 62,
                42, 54, 27, 36, 17, 44, 32, 34, 4,  24, 52, 31, 37, 11, 33, 19,
                35, 6,  22, 53, 63, 25, 41, 47, 1,  59, 0,  28, 40, 55, 48, 8,
                5,  51, 9,  46, 56, 60, 15, 2,  13, 14, 57, 29, 3,  20, 39, 10
            };
            static const int8_t zero[18] = { 0 };
            for (int n = 0; n < n_classes; n++, exact_match_mask >>= 1) {
                const int r = filter_refs[n];
                int8_t *const filter = hdr->restoration.p[p].ns.filter[n];
                const int8_t *const ref_filter = !r ? zero :
                    r < n_classes ? hdr->restoration.p[p].ns.filter[r - 1] :
                    r < n_classes + grp_cnt[1] ?
                                    ref_filters[r - n_classes] :
                    dav1d_wiener_ns_filters[shuffled_index[r - n_classes - grp_cnt[1]]];
                if (exact_match_mask & 1) {
                    memcpy(filter, ref_filter, 16 + 2 * !!p);
                    continue;
                }
                memset(filter, 0, 16 + !!p * 2);
                int s;
                for (s = 0; s < 3 - !!p; s++) {
                    const int found = dav1d_get_bit(gb);
                    if (!found) break;
                }
                const unsigned mask = masks[s];
                // FIXME read sym bit (chroma only) if ref filter subset "s" is
                // assymetric and has space
                for (int i = 0, m = mask; i < 16 + !!p * 2; i++, m >>= 1) {
                    if (!(m & 1)) continue;
                    const int nbits = cf_range[i][0];
                    filter[i] = (int) dav1d_get_bits_subexp_u(gb,
                                    ref_filter[i] - cf_range[i][1],
                                    1 << nbits, nbits - 3) +
                                cf_range[i][1];
                    // FIXME if sym is set and this coef is assymetric, insert an
                    // extra coef here
                }
            }
        }
    }
#if DEBUG_FRAME_HDR
    printf("HDR: post-restoration[y:%d,u:%d,v:%d]: off=%td\n",
           hdr->restoration.p[0].type,
           hdr->restoration.p[1].type,
           hdr->restoration.p[2].type,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    if (!hdr->all_lossless && seqhdr->ccso) {
        hdr->ccso.enabled = dav1d_get_bit(gb);
        if (hdr->ccso.enabled) {
            const int n_planes = seqhdr->layout == DAV1D_PIXEL_LAYOUT_I400 ? 1 : 3;
            for (int p = 0; p < n_planes; p++) {
                hdr->ccso.p[p].enabled = dav1d_get_bit(gb);
                if (!hdr->ccso.p[p].enabled) continue;
                if (IS_INTER_OR_SWITCH(hdr)) {
                    hdr->ccso.p[p].reuse = dav1d_get_bit(gb);
                    hdr->ccso.p[p].sb_reuse = dav1d_get_bit(gb);
                    if (hdr->ccso.p[p].reuse || hdr->ccso.p[p].sb_reuse) {
                        int ref = 0;
                        const int n_bits = hdr->n_ref_frames <= 2 ?
                                           hdr->n_ref_frames - 1 :
                                           1 + ulog2(hdr->n_ref_frames - 1);
                        if (n_bits) {
                            hdr->ccso.p[p].refidx = ref =
                                dav1d_get_bits(gb, n_bits);
                            if (hdr->ccso.p[p].refidx >= hdr->n_ref_frames)
                                goto error;
                        }
                        const Dav1dFrameHeader *const refhdr =
                            c->refs[hdr->refidx[ref]].p.p.frame_hdr;
                        if (!refhdr) goto error;
                        if (hdr->ccso.p[p].reuse) {
                            const int w4 = (hdr->width + 3) >> 2;
                            const int h4 = (hdr->height + 3) >> 2;
                            const int rw4 = (refhdr->width + 3) >> 2;
                            const int rh4 = (refhdr->height + 3) >> 2;
                            if (w4 != rw4 || h4 != rh4) goto error;
                        }
                    }
                }
                static const uint16_t quant_sz[4][4] = {
                    { 16, 8, 32, 0 },
                    { 56, 40, 64, 128 },
                    { 48, 24, 96, 192 },
                    { 80, 112, 160, 256 }
                };
                if (!hdr->ccso.p[p].reuse) {
                    hdr->ccso.p[p].bo_only = dav1d_get_bit(gb);
                    const int si = hdr->ccso.p[p].scale_idx = dav1d_get_bits(gb, 2);
                    if (hdr->ccso.p[p].bo_only) {
                        hdr->ccso.p[p].max_band_log2 = dav1d_get_bits(gb, 3);
                    } else {
                        const int qi =
                        hdr->ccso.p[p].quant_idx = dav1d_get_bits(gb, 2);
                        hdr->ccso.p[p].ext_filter_support = dav1d_get_bits(gb, 3);
                        if (hdr->ccso.p[p].ext_filter_support == 7) goto error;
                        if (quant_sz[si][qi])
                            hdr->ccso.p[p].edge_clf = dav1d_get_bit(gb);
                        hdr->ccso.p[p].max_band_log2 = dav1d_get_bits(gb, 2);
                    }
                    const int n_edge_off_intervals = hdr->ccso.p[p].bo_only ? 1 :
                                                     3 - hdr->ccso.p[p].edge_clf;
                    const int max_band = 1 << hdr->ccso.p[p].max_band_log2;
                    for (int n = 0; n < n_edge_off_intervals; n++) {
                        for (int m = 0; m < n_edge_off_intervals; m++) {
                            for (int o = 0; o < max_band; o++) {
                                int off = 0;
                                for (; off < 7; off++)
                                    if (!dav1d_get_bit(gb)) break;
                                static const int8_t ccso_offset[8] = {
                                    0, 1, -1, 3, -3, 7, -7, -10
                                }, ccso_scale[4] = { 1, 2, 3, 4 };
                                hdr->ccso.p[p].filter_off[o][n][m] =
                                    ccso_offset[off] * ccso_scale[si];
                            }
                        }
                    }
                } else {
                    // FIXME copy ccso plane data from reference
                }
            }
        }
#if DEBUG_FRAME_HDR
    printf("HDR: post-ccso[%d]: off=%td\n",
           hdr->ccso.enabled,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    }

    if (!hdr->all_lossless)
        hdr->txfm_mode = dav1d_get_bit(gb) ? DAV1D_TX_SWITCHABLE : DAV1D_TX_LARGEST;

    if (IS_INTER_OR_SWITCH(hdr)) {
        hdr->switchable_comp_refs = dav1d_get_bit(gb);
        hdr->skip_mode_enabled = dav1d_get_bit(gb);

        if (seqhdr->bawp)
            hdr->bawp = dav1d_get_bit(gb);

        if (seqhdr->motion_modes & (1 << MM_WARP_DELTA))
            hdr->warp_motion = dav1d_get_bit(gb);
    }

    hdr->reduced_txtp_set = dav1d_get_bits(gb, 2);
#if DEBUG_FRAME_HDR
    printf("HDR: post-modebits[tx:%d,refmode:%d,skipmode:%d,bawp:%d,warp:%d,redtxset:%d]: off=%td\n",
           hdr->txfm_mode,
           hdr->switchable_comp_refs,
           hdr->skip_mode_enabled,
           hdr->bawp,
           hdr->warp_motion,
           hdr->reduced_txtp_set,
           (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif

    for (int i = 0; i < 7; i++)
        hdr->gmv[i] = dav1d_default_wm_params;

    if (IS_INTER_OR_SWITCH(hdr)) {
        if (seqhdr->global_motion && dav1d_get_bit(gb)) for (int i = 0; i < 7; i++) {
            hdr->gmv[i].type = !dav1d_get_bit(gb) ? DAV1D_WM_TYPE_IDENTITY :
                                dav1d_get_bit(gb) ? DAV1D_WM_TYPE_ROT_ZOOM :
                                dav1d_get_bit(gb) ? DAV1D_WM_TYPE_TRANSLATION :
                                                    DAV1D_WM_TYPE_AFFINE;

            if (hdr->gmv[i].type == DAV1D_WM_TYPE_IDENTITY) continue;

            const Dav1dWarpedMotionParams *ref_gmv;
            if (hdr->primary_ref_frame == DAV1D_PRIMARY_REF_NONE) {
                ref_gmv = &dav1d_default_wm_params;
            } else {
                const int pri_ref = hdr->refidx[hdr->primary_ref_frame];
                if (!c->refs[pri_ref].p.p.frame_hdr) goto error;
                ref_gmv = &c->refs[pri_ref].p.p.frame_hdr->gmv[i];
            }
            int32_t *const mat = hdr->gmv[i].matrix;
            const int32_t *const ref_mat = ref_gmv->matrix;
            int bits, shift;

            if (hdr->gmv[i].type >= DAV1D_WM_TYPE_ROT_ZOOM) {
                mat[2] = (1 << 16) + 2 *
                    dav1d_get_bits_subexp(gb, (ref_mat[2] - (1 << 16)) >> 1, 12);
                mat[3] = 2 * dav1d_get_bits_subexp(gb, ref_mat[3] >> 1, 12);

                bits = 12;
                shift = 10;
            } else {
                bits = 6 + hdr->mv_precision;
                shift = 16 - hdr->mv_precision;
            }

            if (hdr->gmv[i].type == DAV1D_WM_TYPE_AFFINE) {
                mat[4] = 2 * dav1d_get_bits_subexp(gb, ref_mat[4] >> 1, 12);
                mat[5] = (1 << 16) + 2 *
                    dav1d_get_bits_subexp(gb, (ref_mat[5] - (1 << 16)) >> 1, 12);
            } else {
                mat[4] = -mat[3];
                mat[5] = mat[2];
            }

            mat[0] = dav1d_get_bits_subexp(gb, ref_mat[0] >> shift, bits) * (1 << shift);
            mat[1] = dav1d_get_bits_subexp(gb, ref_mat[1] >> shift, bits) * (1 << shift);
        }
#if DEBUG_FRAME_HDR
        printf("HDR: post-gmv: off=%td\n",
               (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    }

    if (seqhdr->film_grain_present && (hdr->show_frame || hdr->showable_frame)) {
        hdr->film_grain.present = dav1d_get_bit(gb);
        if (hdr->film_grain.present) {
            const unsigned seed = dav1d_get_bits(gb, 16);
            hdr->film_grain.update = hdr->frame_type != DAV1D_FRAME_TYPE_INTER || dav1d_get_bit(gb);
            if (!hdr->film_grain.update) {
                const int refidx = dav1d_get_bits(gb, 3);
                int i;
                for (i = 0; i < 7; i++)
                    if (hdr->refidx[i] == refidx)
                        break;
                if (i == 7 || !c->refs[refidx].p.p.frame_hdr) goto error;
                hdr->film_grain.data = c->refs[refidx].p.p.frame_hdr->film_grain.data;
                hdr->film_grain.data.seed = seed;
            } else {
                Dav1dFilmGrainData *const fgd = &hdr->film_grain.data;
                fgd->seed = seed;

                fgd->num_y_points = dav1d_get_bits(gb, 4);
                if (fgd->num_y_points > 14) goto error;
                for (int i = 0; i < fgd->num_y_points; i++) {
                    fgd->y_points[i][0] = dav1d_get_bits(gb, 8);
                    if (i && fgd->y_points[i - 1][0] >= fgd->y_points[i][0])
                        goto error;
                    fgd->y_points[i][1] = dav1d_get_bits(gb, 8);
                }

                if (seqhdr->layout != DAV1D_PIXEL_LAYOUT_I400)
                    fgd->chroma_scaling_from_luma = dav1d_get_bit(gb);
                if (seqhdr->layout == DAV1D_PIXEL_LAYOUT_I400 ||
                    fgd->chroma_scaling_from_luma ||
                    (seqhdr->ss_ver == 1 && seqhdr->ss_hor == 1 && !fgd->num_y_points))
                {
                    fgd->num_uv_points[0] = fgd->num_uv_points[1] = 0;
                } else for (int pl = 0; pl < 2; pl++) {
                    fgd->num_uv_points[pl] = dav1d_get_bits(gb, 4);
                    if (fgd->num_uv_points[pl] > 10) goto error;
                    for (int i = 0; i < fgd->num_uv_points[pl]; i++) {
                        fgd->uv_points[pl][i][0] = dav1d_get_bits(gb, 8);
                        if (i && fgd->uv_points[pl][i - 1][0] >= fgd->uv_points[pl][i][0])
                            goto error;
                        fgd->uv_points[pl][i][1] = dav1d_get_bits(gb, 8);
                    }
                }

                if (seqhdr->ss_hor == 1 && seqhdr->ss_ver == 1 &&
                    !!fgd->num_uv_points[0] != !!fgd->num_uv_points[1])
                {
                    goto error;
                }

                fgd->scaling_shift = dav1d_get_bits(gb, 2) + 8;
                fgd->ar_coeff_lag = dav1d_get_bits(gb, 2);
                const int num_y_pos = 2 * fgd->ar_coeff_lag * (fgd->ar_coeff_lag + 1);
                if (fgd->num_y_points)
                    for (int i = 0; i < num_y_pos; i++)
                        fgd->ar_coeffs_y[i] = dav1d_get_bits(gb, 8) - 128;
                for (int pl = 0; pl < 2; pl++)
                    if (fgd->num_uv_points[pl] || fgd->chroma_scaling_from_luma) {
                        const int num_uv_pos = num_y_pos + !!fgd->num_y_points;
                        for (int i = 0; i < num_uv_pos; i++)
                            fgd->ar_coeffs_uv[pl][i] = dav1d_get_bits(gb, 8) - 128;
                        if (!fgd->num_y_points)
                            fgd->ar_coeffs_uv[pl][num_uv_pos] = 0;
                    }
                fgd->ar_coeff_shift = dav1d_get_bits(gb, 2) + 6;
                fgd->grain_scale_shift = dav1d_get_bits(gb, 2);
                for (int pl = 0; pl < 2; pl++)
                    if (fgd->num_uv_points[pl]) {
                        fgd->uv_mult[pl] = dav1d_get_bits(gb, 8) - 128;
                        fgd->uv_luma_mult[pl] = dav1d_get_bits(gb, 8) - 128;
                        fgd->uv_offset[pl] = dav1d_get_bits(gb, 9) - 256;
                    }
                fgd->overlap_flag = dav1d_get_bit(gb);
                fgd->clip_to_restricted_range = dav1d_get_bit(gb);
            }
        }
#if DEBUG_FRAME_HDR
        printf("HDR: post-filmgrain[%d]: off=%td\n",
               hdr->film_grain.present,
               (gb->ptr - init_ptr) * 8 - gb->bits_left);
#endif
    }

    return 0;

error:
    dav1d_log(c, "Error parsing frame header\n");
    return DAV1D_ERR(EINVAL);
}

static void parse_tile_hdr(Dav1dContext *const c, GetBits *const gb) {
    const int n_tiles = c->frame_hdr->tiling.t.cols * c->frame_hdr->tiling.t.rows;
    const int have_tile_pos = n_tiles > 1 ? dav1d_get_bit(gb) : 0;

    if (have_tile_pos) {
        const int n_bits = c->frame_hdr->tiling.t.log2_cols +
                           c->frame_hdr->tiling.t.log2_rows;
        c->tile[c->n_tile_data].start = dav1d_get_bits(gb, n_bits);
        c->tile[c->n_tile_data].end = dav1d_get_bits(gb, n_bits);
    } else {
        c->tile[c->n_tile_data].start = 0;
        c->tile[c->n_tile_data].end = n_tiles - 1;
    }
}

ptrdiff_t dav1d_parse_obus(Dav1dContext *const c, Dav1dData *const in) {
    GetBits gb;
    int res;

    dav1d_init_get_bits(&gb, in->data, in->sz);

    // length field
    const size_t len = dav1d_get_uleb128(&gb);
    dav1d_bytealign_get_bits(&gb);
    if (len > (size_t)(gb.ptr_end - gb.ptr)) goto error;
    gb.ptr_end = gb.ptr + len;

    const int has_extension = dav1d_get_bit(&gb);
    const enum Dav1dObuType type = dav1d_get_bits(&gb, 5);
    const int tlayer_id = dav1d_get_bits(&gb, 2);

    int mlayer_id = 0, xlayer_id = 0;
    if (has_extension) {
        mlayer_id = dav1d_get_bits(&gb, 3);
        xlayer_id = dav1d_get_bits(&gb, 5);
    }

    if (gb.error) goto error;

    // We must have read a whole number of bytes at this point (1 byte
    // for the header and whole bytes at a time when reading the
    // leb128 length field).
    assert(gb.bits_left == 0);

    // skip obu not belonging to the selected temporal/spatial layer
    if (type != DAV1D_OBU_SEQ_HDR && type != DAV1D_OBU_TD &&
        has_extension && c->operating_point_idc != 0)
    {
        const int in_temporal_layer = 1; //(c->operating_point_idc >> temporal_id) & 1;
        const int in_spatial_layer = 1; //(c->operating_point_idc >> (spatial_id + 8)) & 1;
        if (!in_temporal_layer || !in_spatial_layer)
            return gb.ptr_end - gb.ptr_start;
    }
#define DEBUG_OBU_HDR 0
    if (DEBUG_OBU_HDR)
        printf("OBU type=%d size=%ld\n",
               type, gb.ptr_end - gb.ptr);

    switch (type) {
    case DAV1D_OBU_SEQ_HDR: {
        Dav1dRef *ref = dav1d_ref_create_using_pool(c->seq_hdr_pool,
                                                    sizeof(Dav1dSequenceHeader));
        if (!ref) return DAV1D_ERR(ENOMEM);
        Dav1dSequenceHeader *seq_hdr = ref->data;
        if ((res = parse_seq_hdr(seq_hdr, &gb, c->strict_std_compliance)) < 0) {
            dav1d_log(c, "Error parsing sequence header\n");
            dav1d_ref_dec(&ref);
            goto error;
        }

        const int op_idx =
            c->operating_point < seq_hdr->num_operating_points ? c->operating_point : 0;
        c->operating_point_idc = seq_hdr->operating_points[op_idx].idc;
        const unsigned spatial_mask = c->operating_point_idc >> 8;
        c->max_spatial_id = spatial_mask ? ulog2(spatial_mask) : 0;

        // If we have read a sequence header which is different from
        // the old one, this is a new video sequence and can't use any
        // previous state. Free that state.

        if (!c->seq_hdr) {
            c->frame_hdr = NULL;
            c->frame_flags |= PICTURE_FLAG_NEW_SEQUENCE;
        // see 7.5, operating_parameter_info is allowed to change in
        // sequence headers of a single sequence
        } else if (memcmp(seq_hdr, c->seq_hdr, offsetof(Dav1dSequenceHeader, operating_parameter_info))) {
            c->frame_hdr = NULL;
            c->mastering_display = NULL;
            c->content_light = NULL;
            dav1d_ref_dec(&c->mastering_display_ref);
            dav1d_ref_dec(&c->content_light_ref);
            for (int i = 0; i < 8; i++) {
                if (c->refs[i].p.p.frame_hdr)
                    dav1d_thread_picture_unref(&c->refs[i].p);
                dav1d_ref_dec(&c->refs[i].segmap);
                dav1d_ref_dec(&c->refs[i].refmvs);
                dav1d_cdf_thread_unref(&c->cdf[i]);
            }
            c->frame_flags |= PICTURE_FLAG_NEW_SEQUENCE;
        // If operating_parameter_info changed, signal it
        } else if (memcmp(seq_hdr->operating_parameter_info, c->seq_hdr->operating_parameter_info,
                          sizeof(seq_hdr->operating_parameter_info)))
        {
            c->frame_flags |= PICTURE_FLAG_NEW_OP_PARAMS_INFO;
        }
        dav1d_ref_dec(&c->seq_hdr_ref);
        c->seq_hdr_ref = ref;
        c->seq_hdr = seq_hdr;
        break;
    }
    case DAV1D_OBU_TILE_GRP:
    case DAV1D_OBU_SWITCH:
    case DAV1D_OBU_SEF:
    case DAV1D_OBU_TIP:
    case DAV1D_OBU_BRIDGE:
    case DAV1D_OBU_RAS: {
        if (!c->seq_hdr) goto error;
        if (!c->frame_hdr_ref) {
            c->frame_hdr_ref = dav1d_ref_create_using_pool(c->frame_hdr_pool,
                                                           sizeof(Dav1dFrameHeader));
            if (!c->frame_hdr_ref) return DAV1D_ERR(ENOMEM);
        }
#ifndef NDEBUG
        // ensure that the reference is writable
        assert(dav1d_ref_is_writable(c->frame_hdr_ref));
#endif
        c->frame_hdr = c->frame_hdr_ref->data;
        memset(c->frame_hdr, 0, sizeof(*c->frame_hdr));
        c->frame_hdr->tlayer_id = tlayer_id;
        c->frame_hdr->mlayer_id = mlayer_id;
        c->frame_hdr->xlayer_id = xlayer_id;
        const int first_tile = type == DAV1D_OBU_SEF || type == DAV1D_OBU_TIP ||
                               type == DAV1D_OBU_BRIDGE || dav1d_get_bit(&gb);
        const int has_hdr = first_tile || dav1d_get_bit(&gb);
        // FIXME if not first tile, we can skip re-parsing the header and
        // instead skip the header data and move on to block data directly
        if (has_hdr && (res = parse_frame_hdr(c, &gb, type)) < 0) {
            c->frame_hdr = NULL;
            goto error;
        }
        for (int n = 0; n < c->n_tile_data; n++)
            dav1d_data_unref_internal(&c->tile[n].data);
        c->n_tile_data = 0;
        c->n_tiles = 0;

        if (type == DAV1D_OBU_SEF || type == DAV1D_OBU_TIP ||
            type == DAV1D_OBU_BRIDGE /* || bru frame inactive */)
        {
            // This is actually a frame header OBU so read the
            // trailing bit and check for overrun.
            if (check_trailing_bits(&gb, c->strict_std_compliance) < 0) {
                c->frame_hdr = NULL;
                goto error;
            }
        }

        if (c->frame_size_limit && (int64_t)c->frame_hdr->width *
            c->frame_hdr->height > c->frame_size_limit)
        {
            dav1d_log(c, "Frame size %dx%d exceeds limit %u\n", c->frame_hdr->width,
                      c->frame_hdr->height, c->frame_size_limit);
            c->frame_hdr = NULL;
            return DAV1D_ERR(ERANGE);
        }

        if (type == DAV1D_OBU_SEF || type == DAV1D_OBU_TIP ||
            type == DAV1D_OBU_BRIDGE /* || bru frame inactive */)
        {
            break;
        }

        if (c->n_tile_data_alloc < c->n_tile_data + 1) {
            if ((c->n_tile_data + 1) > INT_MAX / (int)sizeof(*c->tile)) goto error;
            struct Dav1dTileGroup *tile = dav1d_realloc(ALLOC_TILE, c->tile,
                                                        (c->n_tile_data + 1) * sizeof(*c->tile));
            if (!tile) goto error;
            c->tile = tile;
            memset(c->tile + c->n_tile_data, 0, sizeof(*c->tile));
            c->n_tile_data_alloc = c->n_tile_data + 1;
        }
        parse_tile_hdr(c, &gb);
        // Align to the next byte boundary and check for overrun.
        dav1d_bytealign_get_bits(&gb);
        if (gb.error) goto error;

        dav1d_data_ref(&c->tile[c->n_tile_data].data, in);
        c->tile[c->n_tile_data].data.data = gb.ptr;
        c->tile[c->n_tile_data].data.sz = (size_t)(gb.ptr_end - gb.ptr);
        // ensure tile groups are in order and sane, see 6.10.1
        if (c->tile[c->n_tile_data].start > c->tile[c->n_tile_data].end ||
            c->tile[c->n_tile_data].start != c->n_tiles)
        {
            for (int i = 0; i <= c->n_tile_data; i++)
                dav1d_data_unref_internal(&c->tile[i].data);
            c->n_tile_data = 0;
            c->n_tiles = 0;
            goto error;
        }
        c->n_tiles += 1 + c->tile[c->n_tile_data].end -
                          c->tile[c->n_tile_data].start;
        c->n_tile_data++;
        break;
    }
    case DAV1D_OBU_METADATA: {
#define DEBUG_OBU_METADATA 0
#if DEBUG_OBU_METADATA
        const uint8_t *const init_ptr = gb.ptr;
#endif
        // obu metadta type field
        const enum ObuMetaType meta_type = dav1d_get_uleb128(&gb);
        if (gb.error) goto error;

        switch (meta_type) {
        case OBU_META_HDR_CLL: {
            Dav1dRef *ref = dav1d_ref_create(ALLOC_OBU_META,
                                             sizeof(Dav1dContentLightLevel));
            if (!ref) return DAV1D_ERR(ENOMEM);
            Dav1dContentLightLevel *const content_light = ref->data;

            content_light->max_content_light_level = dav1d_get_bits(&gb, 16);
#if DEBUG_OBU_METADATA
            printf("CLLOBU: max-content-light-level: %d [off=%td]\n",
                   content_light->max_content_light_level,
                   (gb.ptr - init_ptr) * 8 - gb.bits_left);
#endif
            content_light->max_frame_average_light_level = dav1d_get_bits(&gb, 16);
#if DEBUG_OBU_METADATA
            printf("CLLOBU: max-frame-average-light-level: %d [off=%td]\n",
                   content_light->max_frame_average_light_level,
                   (gb.ptr - init_ptr) * 8 - gb.bits_left);
#endif

            if (check_trailing_bits(&gb, c->strict_std_compliance) < 0) {
                dav1d_ref_dec(&ref);
                goto error;
            }

            dav1d_ref_dec(&c->content_light_ref);
            c->content_light = content_light;
            c->content_light_ref = ref;
            break;
        }
        case OBU_META_HDR_MDCV: {
            Dav1dRef *ref = dav1d_ref_create(ALLOC_OBU_META,
                                             sizeof(Dav1dMasteringDisplay));
            if (!ref) return DAV1D_ERR(ENOMEM);
            Dav1dMasteringDisplay *const mastering_display = ref->data;

            for (int i = 0; i < 3; i++) {
                mastering_display->primaries[i][0] = dav1d_get_bits(&gb, 16);
                mastering_display->primaries[i][1] = dav1d_get_bits(&gb, 16);
#if DEBUG_OBU_METADATA
                printf("MDCVOBU: primaries[%d]: (%d, %d) [off=%td]\n", i,
                       mastering_display->primaries[i][0],
                       mastering_display->primaries[i][1],
                       (gb.ptr - init_ptr) * 8 - gb.bits_left);
#endif
            }
            mastering_display->white_point[0] = dav1d_get_bits(&gb, 16);
#if DEBUG_OBU_METADATA
            printf("MDCVOBU: white-point-x: %d [off=%td]\n",
                   mastering_display->white_point[0],
                   (gb.ptr - init_ptr) * 8 - gb.bits_left);
#endif
            mastering_display->white_point[1] = dav1d_get_bits(&gb, 16);
#if DEBUG_OBU_METADATA
            printf("MDCVOBU: white-point-y: %d [off=%td]\n",
                   mastering_display->white_point[1],
                   (gb.ptr - init_ptr) * 8 - gb.bits_left);
#endif
            mastering_display->max_luminance = dav1d_get_bits(&gb, 32);
#if DEBUG_OBU_METADATA
            printf("MDCVOBU: max-luminance: %d [off=%td]\n",
                   mastering_display->max_luminance,
                   (gb.ptr - init_ptr) * 8 - gb.bits_left);
#endif
            mastering_display->min_luminance = dav1d_get_bits(&gb, 32);
#if DEBUG_OBU_METADATA
            printf("MDCVOBU: min-luminance: %d [off=%td]\n",
                   mastering_display->min_luminance,
                   (gb.ptr - init_ptr) * 8 - gb.bits_left);
#endif
            if (check_trailing_bits(&gb, c->strict_std_compliance) < 0) {
                dav1d_ref_dec(&ref);
                goto error;
            }

            dav1d_ref_dec(&c->mastering_display_ref);
            c->mastering_display = mastering_display;
            c->mastering_display_ref = ref;
            break;
        }
        case OBU_META_ITUT_T35: {
            ptrdiff_t payload_size = gb.ptr_end - gb.ptr;
            // Don't take into account all the trailing bits for payload_size
            while (payload_size > 0 && !gb.ptr[payload_size - 1])
                payload_size--; // trailing_zero_bit x 8
            payload_size--; // trailing_one_bit + trailing_zero_bit x 7

            int country_code_extension_byte = 0;
            const int country_code = dav1d_get_bits(&gb, 8);
            payload_size--;
            if (country_code == 0xFF) {
                country_code_extension_byte = dav1d_get_bits(&gb, 8);
                payload_size--;
            }

            if (payload_size <= 0 || gb.ptr[payload_size] != 0x80) {
                dav1d_log(c, "Malformed ITU-T T.35 metadata message format\n");
                break;
            }

            if ((c->n_itut_t35 + 1) > INT_MAX / (int)sizeof(*c->itut_t35)) goto error;
            struct Dav1dITUTT35 *itut_t35 = dav1d_realloc(ALLOC_OBU_META, c->itut_t35,
                                                          (c->n_itut_t35 + 1) * sizeof(*c->itut_t35));
            if (!itut_t35) goto error;
            c->itut_t35 = itut_t35;
            memset(c->itut_t35 + c->n_itut_t35, 0, sizeof(*c->itut_t35));

            struct itut_t35_ctx_context *itut_t35_ctx;
            if (!c->n_itut_t35) {
                assert(!c->itut_t35_ref);
                itut_t35_ctx = dav1d_malloc(ALLOC_OBU_META, sizeof(struct itut_t35_ctx_context));
                if (!itut_t35_ctx) goto error;
                c->itut_t35_ref = dav1d_ref_init(&itut_t35_ctx->ref, c->itut_t35,
                                                 dav1d_picture_free_itut_t35, itut_t35_ctx, 0);
            } else {
                assert(c->itut_t35_ref && atomic_load(&c->itut_t35_ref->ref_cnt) == 1);
                itut_t35_ctx = c->itut_t35_ref->user_data;
                c->itut_t35_ref->const_data = (uint8_t *)c->itut_t35;
            }
            itut_t35_ctx->itut_t35 = c->itut_t35;
            itut_t35_ctx->n_itut_t35 = c->n_itut_t35 + 1;

            Dav1dITUTT35 *const itut_t35_metadata = &c->itut_t35[c->n_itut_t35];
            itut_t35_metadata->payload = dav1d_malloc(ALLOC_OBU_META, payload_size);
            if (!itut_t35_metadata->payload) goto error;

            itut_t35_metadata->country_code = country_code;
            itut_t35_metadata->country_code_extension_byte = country_code_extension_byte;
            itut_t35_metadata->payload_size = payload_size;

            // We know that we've read a whole number of bytes and that the
            // payload is within the OBU boundaries, so just use memcpy()
            assert(gb.bits_left == 0);
            memcpy(itut_t35_metadata->payload, gb.ptr, payload_size);

            c->n_itut_t35++;
            break;
        }
        case OBU_META_SCALABILITY:
        case OBU_META_TIMECODE:
            // ignore metadata OBUs we don't care about
            break;
        default:
            // print a warning but don't fail for unknown types
            if (meta_type > 31) // Types 6 to 31 are "Unregistered user private", so ignore them.
                dav1d_log(c, "Unknown Metadata OBU type %d\n", meta_type);
            break;
        }

        break;
    }
    case DAV1D_OBU_TD:
        c->frame_flags |= PICTURE_FLAG_NEW_TEMPORAL_UNIT;
        break;
    case DAV1D_OBU_PADDING:
        // ignore OBUs we don't care about
        break;
    default:
        // print a warning but don't fail for unknown types
        dav1d_log(c, "Unknown OBU type %d of size %td\n", type, gb.ptr_end - gb.ptr);
        break;
    }

    if (c->seq_hdr && c->frame_hdr) {
        if (c->frame_hdr->show_existing_frame) {
            if (!c->refs[c->frame_hdr->existing_frame_idx].p.p.frame_hdr) goto error;
            switch (c->refs[c->frame_hdr->existing_frame_idx].p.p.frame_hdr->frame_type) {
            case DAV1D_FRAME_TYPE_INTER:
            case DAV1D_FRAME_TYPE_SWITCH:
                if (c->decode_frame_type > DAV1D_DECODEFRAMETYPE_REFERENCE)
                    goto skip;
                break;
            case DAV1D_FRAME_TYPE_INTRA:
                if (c->decode_frame_type > DAV1D_DECODEFRAMETYPE_INTRA)
                    goto skip;
                // fall-through
            default:
                break;
            }
            if (!c->refs[c->frame_hdr->existing_frame_idx].p.p.data[0]) goto error;
            if (c->strict_std_compliance &&
                !c->refs[c->frame_hdr->existing_frame_idx].p.showable)
            {
                goto error;
            }
            if (c->n_fc == 1) {
                dav1d_thread_picture_ref(&c->out,
                                         &c->refs[c->frame_hdr->existing_frame_idx].p);
                dav1d_picture_copy_props(&c->out.p,
                                         c->content_light, c->content_light_ref,
                                         c->mastering_display, c->mastering_display_ref,
                                         c->itut_t35, c->itut_t35_ref, c->n_itut_t35,
                                         &in->m);
                // Must be removed from the context after being attached to the frame
                dav1d_ref_dec(&c->itut_t35_ref);
                c->itut_t35 = NULL;
                c->n_itut_t35 = 0;
                c->event_flags |= dav1d_picture_get_event_flags(&c->refs[c->frame_hdr->existing_frame_idx].p);
            } else {
                pthread_mutex_lock(&c->task_thread.lock);
                // need to append this to the frame output queue
                const unsigned next = c->frame_thread.next++;
                if (c->frame_thread.next == c->n_fc)
                    c->frame_thread.next = 0;

                Dav1dFrameContext *const f = &c->fc[next];
                while (f->n_tile_data > 0)
                    pthread_cond_wait(&f->task_thread.cond,
                                      &f->task_thread.ttd->lock);
                Dav1dThreadPicture *const out_delayed =
                    &c->frame_thread.out_delayed[next];
                if (out_delayed->p.data[0] || atomic_load(&f->task_thread.error)) {
                    unsigned first = atomic_load(&c->task_thread.first);
                    if (first + 1U < c->n_fc)
                        atomic_fetch_add(&c->task_thread.first, 1U);
                    else
                        atomic_store(&c->task_thread.first, 0);
                    atomic_compare_exchange_strong(&c->task_thread.reset_task_cur,
                                                   &first, UINT_MAX);
                    if (c->task_thread.cur && c->task_thread.cur < c->n_fc)
                        c->task_thread.cur--;
                }
                const int error = f->task_thread.retval;
                if (error) {
                    c->cached_error = error;
                    f->task_thread.retval = 0;
                    dav1d_data_props_copy(&c->cached_error_props, &out_delayed->p.m);
                    dav1d_thread_picture_unref(out_delayed);
                } else if (out_delayed->p.data[0]) {
                    const unsigned progress = atomic_load_explicit(&out_delayed->progress[1],
                                                                   memory_order_relaxed);
                    if ((out_delayed->visible || c->output_invisible_frames) &&
                        progress != FRAME_ERROR)
                    {
                        dav1d_thread_picture_ref(&c->out, out_delayed);
                        c->event_flags |= dav1d_picture_get_event_flags(out_delayed);
                    }
                    dav1d_thread_picture_unref(out_delayed);
                }
                dav1d_thread_picture_ref(out_delayed,
                                         &c->refs[c->frame_hdr->existing_frame_idx].p);
                out_delayed->visible = 1;
                dav1d_picture_copy_props(&out_delayed->p,
                                         c->content_light, c->content_light_ref,
                                         c->mastering_display, c->mastering_display_ref,
                                         c->itut_t35, c->itut_t35_ref, c->n_itut_t35,
                                         &in->m);
                // Must be removed from the context after being attached to the frame
                dav1d_ref_dec(&c->itut_t35_ref);
                c->itut_t35 = NULL;
                c->n_itut_t35 = 0;

                pthread_mutex_unlock(&c->task_thread.lock);
            }
            if (c->refs[c->frame_hdr->existing_frame_idx].p.p.frame_hdr->frame_type == DAV1D_FRAME_TYPE_KEY) {
                const int r = c->frame_hdr->existing_frame_idx;
                c->refs[r].p.showable = 0;
                for (int i = 0; i < 8; i++) {
                    if (i == r) continue;

                    if (c->refs[i].p.p.frame_hdr)
                        dav1d_thread_picture_unref(&c->refs[i].p);
                    dav1d_thread_picture_ref(&c->refs[i].p, &c->refs[r].p);

                    dav1d_cdf_thread_unref(&c->cdf[i]);
                    dav1d_cdf_thread_ref(&c->cdf[i], &c->cdf[r]);

                    dav1d_ref_dec(&c->refs[i].segmap);
                    c->refs[i].segmap = c->refs[r].segmap;
                    if (c->refs[r].segmap)
                        dav1d_ref_inc(c->refs[r].segmap);
                    dav1d_ref_dec(&c->refs[i].refmvs);
                }
            }
            c->frame_hdr = NULL;
        } else if (c->n_tiles == c->frame_hdr->tiling.t.cols *
                                 c->frame_hdr->tiling.t.rows)
        {
            switch (c->frame_hdr->frame_type) {
            case DAV1D_FRAME_TYPE_INTER:
            case DAV1D_FRAME_TYPE_SWITCH:
                if (c->decode_frame_type > DAV1D_DECODEFRAMETYPE_REFERENCE ||
                    (c->decode_frame_type == DAV1D_DECODEFRAMETYPE_REFERENCE &&
                     !c->frame_hdr->refresh_frame_flags))
                    goto skip;
                break;
            case DAV1D_FRAME_TYPE_INTRA:
                if (c->decode_frame_type > DAV1D_DECODEFRAMETYPE_INTRA ||
                    (c->decode_frame_type == DAV1D_DECODEFRAMETYPE_REFERENCE &&
                     !c->frame_hdr->refresh_frame_flags))
                    goto skip;
                // fall-through
            default:
                break;
            }
            if (!c->n_tile_data)
                goto error;
            if ((res = dav1d_submit_frame(c)) < 0)
                return res;
            assert(!c->n_tile_data);
            c->frame_hdr = NULL;
            c->n_tiles = 0;
        }
    }

    return gb.ptr_end - gb.ptr_start;

skip:
    // update refs with only the headers in case we skip the frame
    for (int i = 0; i < 8; i++) {
        if (c->frame_hdr->refresh_frame_flags & (1 << i)) {
            dav1d_thread_picture_unref(&c->refs[i].p);
            c->refs[i].p.p.frame_hdr = c->frame_hdr;
            c->refs[i].p.p.seq_hdr = c->seq_hdr;
            c->refs[i].p.p.frame_hdr_ref = c->frame_hdr_ref;
            c->refs[i].p.p.seq_hdr_ref = c->seq_hdr_ref;
            dav1d_ref_inc(c->frame_hdr_ref);
            dav1d_ref_inc(c->seq_hdr_ref);
        }
    }

    dav1d_ref_dec(&c->frame_hdr_ref);
    c->frame_hdr = NULL;
    c->n_tiles = 0;

    return gb.ptr_end - gb.ptr_start;

error:
    dav1d_data_props_copy(&c->cached_error_props, &in->m);
    dav1d_log(c, gb.error ? "Overrun in OBU bit buffer\n" :
                            "Error parsing OBU data\n");
    return DAV1D_ERR(EINVAL);
}
