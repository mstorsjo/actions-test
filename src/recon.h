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

#ifndef DAV2D_SRC_RECON_H
#define DAV2D_SRC_RECON_H

#include "src/debug.h"
#include "src/internal.h"
#include "src/levels.h"

#define decl_recon_b_fn(name) \
int (name)(Dav2dTaskContext *t, DB_ONLY(int depth) \
           enum BlockSize bs, const enum BlockSize cbs[2], Av2Block *b)
typedef decl_recon_b_fn(*recon_b_fn);

#define decl_filter_sbrow_fn(name) \
void (name)(Dav2dFrameContext *f, int sby)
typedef decl_filter_sbrow_fn(*filter_sbrow_fn);

#define decl_backup_prefilter_data_fn(name) \
void (name)(Dav2dTaskContext *t)
typedef decl_backup_prefilter_data_fn(*backup_prefilter_data_fn);

#define decl_read_coef_blocks_fn(name) \
int (name)(Dav2dTaskContext *t, DB_ONLY(int depth) \
           enum BlockSize lbs, enum BlockSize cbs, Av2Block *b)
typedef decl_read_coef_blocks_fn(*read_coef_blocks_fn);

#define decl_copy_pal_block_fn(name) \
void (name)(Dav2dTaskContext *t, int bx4, int by4, int bw4, int bh4)
typedef decl_copy_pal_block_fn(*copy_pal_block_fn);

#define decl_read_pal_plane_fn(name) \
void (name)(DB_ONLY(const int depth) Dav2dTaskContext *t, \
            Av2Block *b, int bx4, int by4)
typedef decl_read_pal_plane_fn(*read_pal_plane_fn);

decl_recon_b_fn(dav2d_recon_b_8bpc);
decl_recon_b_fn(dav2d_recon_b_16bpc);

decl_filter_sbrow_fn(dav2d_filter_sbrow_8bpc);
decl_filter_sbrow_fn(dav2d_filter_sbrow_16bpc);
decl_filter_sbrow_fn(dav2d_filter_sbrow_deblock_cols_8bpc);
decl_filter_sbrow_fn(dav2d_filter_sbrow_deblock_cols_16bpc);
decl_filter_sbrow_fn(dav2d_filter_sbrow_deblock_rows_8bpc);
decl_filter_sbrow_fn(dav2d_filter_sbrow_deblock_rows_16bpc);
void dav2d_filter_sbrow_cdef_8bpc(Dav2dTaskContext *tc, int sby);
void dav2d_filter_sbrow_cdef_16bpc(Dav2dTaskContext *tc, int sby);
decl_filter_sbrow_fn(dav2d_filter_sbrow_lr_8bpc);
decl_filter_sbrow_fn(dav2d_filter_sbrow_lr_16bpc);

decl_backup_prefilter_data_fn(dav2d_backup_prefilter_data_8bpc);
decl_backup_prefilter_data_fn(dav2d_backup_prefilter_data_16bpc);

decl_read_coef_blocks_fn(dav2d_read_coef_blocks_8bpc);
decl_read_coef_blocks_fn(dav2d_read_coef_blocks_16bpc);

decl_copy_pal_block_fn(dav2d_copy_pal_block_y_8bpc);
decl_copy_pal_block_fn(dav2d_copy_pal_block_y_16bpc);
decl_read_pal_plane_fn(dav2d_read_pal_plane_8bpc);
decl_read_pal_plane_fn(dav2d_read_pal_plane_16bpc);

#endif /* DAV2D_SRC_RECON_H */
