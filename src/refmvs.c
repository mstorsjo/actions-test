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

#include "config.h"

#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#include "dav1d/common.h"

#include "common/frame.h"
#include "common/intops.h"

#include "src/env.h"
#include "src/mem.h"
#include "src/refmvs.h"

#define DEBUG_REFMV 0

#if DEBUG_BLOCK_INFO && DEBUG_REFMV
#define RDB_ONLY(x...) x
#define DB_ARGS(x...) x,
#define DEBUG_REFMV_printf(fmt...) \
    if (DEBUG_REFMV && BLOCK_TO_DEBUG_S(rf->frm_hdr->frame_offset, by4, bx4)) { \
        printf(fmt); \
    }
#else
#define RDB_ONLY(x...)
#define DB_ARGS(x...)
#define DEBUG_REFMV_printf(fmt...) do {} while (0)
#endif

static int add_candidate_sngl(DB_ARGS(const refmvs_frame *const rf,
                                      const int by4, const int bx4,
                                      const int y_off, const int x_off,
                                      const char *const tag, const int refidx)
                              refmvs_candidate *const mvstack,
                              int *const cnt, const int max_cnt,
                              const int weight, const mv cand_mv,
                              const int y_off_s, const int x_off_s,
                              int *const iter_cntr, const int max_iter)
{
    const int last = *cnt;
    RDB_ONLY(int did_check = 0);
    if (iter_cntr[0] < max_iter) {
        for (int m = 0; m < last; m++)
            if (mvstack[m].mv.mv[0].n == cand_mv.n) {
                iter_cntr[0] += m + 1;
                mvstack[m].weight += weight;
                DEBUG_REFMV_printf("%s[%d:%d]: increasing[%d] y=%d,x=%d,w+=%d "
                                   "at offset y=%d,x=%d\n",
                                   tag, refidx, iter_cntr[0], m, cand_mv.y,
                                   cand_mv.x, weight, y_off, x_off);
                return 0;
            }
        RDB_ONLY(did_check = 1);
        iter_cntr[0] += last;
    }

    if (last >= max_cnt) return 0;

    mvstack[last].mv.mv[0] = cand_mv;
    mvstack[last].weight = weight;
    mvstack[last].y_off = y_off_s;
    mvstack[last].x_off = x_off_s;
    DEBUG_REFMV_printf("%s[%d:%d]: %s[%d] y=%d,x=%d,w=%d,y_off=%d,x_off=%d at "
                       "offset y=%d,x=%d\n",
                       tag, refidx, iter_cntr[0], did_check ? "adding" : "tailing",
                       *cnt, cand_mv.y, cand_mv.x, weight,
                       y_off_s, x_off_s, y_off, x_off);
    *cnt = last + 1;
    return 1;
}

static void add_candidate_c2s(DB_ARGS(const refmvs_frame *const rf,
                                      const int by4, const int bx4,
                                      const int y_off, const int x_off,
                                      const char *const tag, const int refidx)
                              refmvs_sngl_mv_block *const mvstack,
                              int *const cnt, const int max_cnt,
                              const int ref, const mv cand_mv,
                              int *const iter_cntr, const int max_iter)
{
    const int last = *cnt;
    RDB_ONLY(int did_check = 0);
    if (*iter_cntr < max_iter) {
        for (int m = 0; m < last; m++)
            if (mvstack[m].mv.n == cand_mv.n && mvstack[m].ref == ref) {
                *iter_cntr += m + 1;
                DEBUG_REFMV_printf("%s[%d:%d]: skipping[%d] y=%d,x=%d,r=%d "
                                   "at offset y=%d,x=%d\n",
                                   tag, refidx, *iter_cntr, m, cand_mv.y,
                                   cand_mv.x, ref, y_off, x_off);
                return;
            }
        RDB_ONLY(did_check = 1);
        *iter_cntr += last;
    }

    if (last >= max_cnt) return;

    mvstack[last].mv = cand_mv;
    mvstack[last].ref = ref;
    DEBUG_REFMV_printf("%s[%d:%d]: %s[%d] y=%d,x=%d,r=%d at offset y=%d,x=%d\n",
                       tag, refidx, *iter_cntr, did_check ? "adding" : "tailing",
                       *cnt, cand_mv.y, cand_mv.x, ref, y_off, x_off);
    *cnt = last + 1;
}

static int add_candidate_comp(DB_ARGS(const refmvs_frame *const rf,
                                      const int by4, const int bx4,
                                      const int y_off, const int x_off,
                                      const char *const tag)
                              refmvs_candidate *const mvstack,
                              int *const cnt, const int max_cnt,
                              const int weight, const int cwp_idx,
                              const refmvs_mvpair cand_mv,
                              int *const iter_cntr, const int max_iter)
{
    const int last = *cnt;
    RDB_ONLY(int did_check = 0);
    if (iter_cntr[0] < max_iter) {
        for (int n = 0; n < last; n++)
            if (mvstack[n].mv.n == cand_mv.n) {
                iter_cntr[0] += n + 1;
                mvstack[n].weight += weight;
                DEBUG_REFMV_printf("%s-c[%d]: increasing[%d] y=%d,x=%d,"
                                   "y2=%d,x2=%d,w+=%d at offset y=%d,x=%d\n",
                                   tag, iter_cntr[0], n, cand_mv.mv[0].y,
                                   cand_mv.mv[0].x, cand_mv.mv[1].y,
                                   cand_mv.mv[1].x, weight, y_off, x_off);
                return 0;
            }
        RDB_ONLY(did_check = 1);
        iter_cntr[0] += last;
    }

    if (last >= max_cnt) return 0;

    mvstack[last].mv = cand_mv;
    mvstack[last].weight = weight;
    mvstack[last].cwp_idx = cwp_idx;
    DEBUG_REFMV_printf("%s-c[%d]: %s[%d] y=%d,x=%d,y2=%d,x2=%d,w=%d at offset "
                       "y=%d,x=%d\n",
                       tag, iter_cntr[0], did_check ? "adding" : "tailing",
                       *cnt, cand_mv.mv[0].y, cand_mv.mv[0].x,
                       cand_mv.mv[1].y, cand_mv.mv[1].x, weight, y_off, x_off);
    *cnt = last + 1;
    return 1;
}

mv scale_mv(const mv in, const int sf) {
    const int64_t y = in.y * (int64_t) sf, x = in.x * (int64_t) sf;
    return (mv) {
        .y = iclip((int)((y + 0x2000 - (y < 0)) >> 14), -0xffff, 0xffff),
        .x = iclip((int)((x + 0x2000 - (x < 0)) >> 14), -0xffff, 0xffff),
    };
}

struct refmvs_state {
    refmvs_candidate dr[4], *mv;
    refmvs_sngl_mv_block sngl[4];
    int drvd_cnt, sngl_cnt, *cnt;
    int drvd_iter_cntr, sngl_iter_cntr, iter_cntr;
    ptrdiff_t b8x8;
#if DEBUG_BLOCK_INFO && DEBUG_REFMV
    int bx4, by4;
#endif
};

static void add_spatial_candidate(const int y_off, const int x_off,
                                  const refmvs_tile *const rt,
                                  struct refmvs_state *const st,
                                  const int weight, const refmvs_block *const b,
                                  const ptrdiff_t off_8x8,
                                  const union refmvs_refpair ref, const mv gmv[2])
{
    if (*st->cnt >= 6) return;
    if (b->mv.mv[0].n == INVALID_MV) return; // intra block, no intrabc

    const refmvs_frame *const rf = rt->rf;
    if (ref.ref[1] == -1) {
        for (int n = 0; n < 2; n++) {
            if (b->ref.ref[n] == ref.ref[0]) {
                const mv cand_mv = ((b->mf & 1) && gmv[0].n != INVALID_MV) ?
                                   gmv[0] : b->mv.mv[n];
                add_candidate_sngl(DB_ARGS(rf, st->by4, st->bx4,
                                           y_off, x_off, "spc", n)
                                   st->mv, st->cnt, 6, weight, cand_mv,
                                   y_off, x_off, &st->iter_cntr, 16);
            } else if (b->ref.ref[0] - 1 == TIP_FRAME &&
                       rf->frm_hdr->tip.refs[n] == ref.ref[0] - 1)
            {
                const mv tipmv = scale_mv(rt->rp_proj[off_8x8].mv, rf->tip_sf[n]);
                const mv cand_mv = (mv) {
                    .y = iclip(tipmv.y + b->mv.mv[0].y, -0xffff, 0xffff),
                    .x = iclip(tipmv.x + b->mv.mv[0].x, -0xffff, 0xffff),
                };
                add_candidate_sngl(DB_ARGS(rf, st->by4, st->bx4,
                                           y_off, x_off, "tip-spc", n)
                                   st->mv, st->cnt, 6, weight, cand_mv,
                                   y_off, x_off, &st->iter_cntr, 16);
            } else if (ref.ref[0] - 1 == TIP_FRAME &&
                       b->ref.ref[0] - 1 == rf->frm_hdr->tip.refs[0] &&
                       b->ref.ref[1] - 1 == rf->frm_hdr->tip.refs[1])
            {
                const mv in_delta = (mv) {
                    .y = b->mv.mv[0].y - b->mv.mv[1].y,
                    .x = b->mv.mv[0].x - b->mv.mv[1].x,
                };
                const mv out_delta = scale_mv(in_delta, rf->tip_sf[0]);
                const mv cand_mv = (mv) {
                    .y = iclip(b->mv.mv[0].y - out_delta.y, -0xffff, 0xffff),
                    .x = iclip(b->mv.mv[0].x - out_delta.x, -0xffff, 0xffff),
                };
                add_candidate_sngl(DB_ARGS(rf, st->by4, st->bx4,
                                           y_off, x_off, "tip2-spc", n)
                                   st->dr, &st->drvd_cnt, 4, weight, cand_mv,
                                   0, 0, &st->drvd_iter_cntr, 2);
                break;
            } else if (rf->seq_hdr->mv_traj && rf->frm_hdr->use_ref_frame_mvs &&
                       ref.ref[0] - 1U < TIP_FRAME &&
                       (b->ref.ref[0] - 1U == TIP_FRAME ||
                        b->ref.ref[n] - 1U < TIP_FRAME) &&
                       rt->rp_traj[ref.ref[0] - 1][st->b8x8].n != INVALID_MV &&
                       rt->rp_traj[b->ref.ref[0] - 1 == TIP_FRAME ?
                                   rf->frm_hdr->tip.refs[n] :
                                   b->ref.ref[n] - 1][st->b8x8].n != INVALID_MV)
            {
                mv a_mv, b_mv;
                if (b->ref.ref[0] - 1 == TIP_FRAME) {
                    a_mv = rt->rp_traj[rf->frm_hdr->tip.refs[n]][st->b8x8];
                    const mv tipmv = scale_mv(rt->rp_proj[off_8x8].mv,
                                              rf->tip_sf[n]);
                    b_mv = (mv) {
                        .y = iclip(tipmv.y + b->mv.mv[0].y, -0xffff, 0xffff),
                        .x = iclip(tipmv.x + b->mv.mv[0].x, -0xffff, 0xffff),
                    };
                } else {
                    a_mv = rt->rp_traj[b->ref.ref[n] - 1][st->b8x8];
                    b_mv = b->mv.mv[n];
                }
                const mv c_mv = rt->rp_traj[ref.ref[0] - 1][st->b8x8];
                const mv cand_mv = (mv) {
                    .y = iclip(b_mv.y + c_mv.y - a_mv.y, -0xffff, 0xffff),
                    .x = iclip(b_mv.x + c_mv.x - a_mv.x, -0xffff, 0xffff),
                };
                add_candidate_sngl(DB_ARGS(rf, st->by4, st->bx4,
                                           y_off, x_off, "mvtj-spc", n)
                                   st->dr, &st->drvd_cnt, 4, weight, cand_mv,
                                   0, 0, &st->drvd_iter_cntr, 2);
            } else if (ref.ref[0] > 0 && b->ref.ref[0] > 0 &&
                       rf->ref_sign[ref.ref[0] - 1] ==
                           rf->ref_sign[b->ref.ref[0] - 1U == TIP_FRAME ?
                                            rf->frm_hdr->tip.refs[n] :
                                            b->ref.ref[n] - 1])
            {
                mv cand_mv;
                int den;
                if (b->ref.ref[0] - 1U == TIP_FRAME) {
                    const mv tipmv = scale_mv(rt->rp_proj[off_8x8].mv,
                                              rf->tip_sf[n]);
                    cand_mv = (mv) {
                        .y = iclip(tipmv.y + b->mv.mv[0].y, -0xffff, 0xffff),
                        .x = iclip(tipmv.x + b->mv.mv[0].x, -0xffff, 0xffff),
                    };
                    den = rf->abspocdiff[rf->frm_hdr->tip.refs[n]];
                } else {
                    cand_mv = b->mv.mv[n];
                    den = rf->abspocdiff[b->ref.ref[n] - 1];
                }
                cand_mv = mv_projection(cand_mv, rf->pocdiff[ref.ref[0] - 1], den);
                add_candidate_sngl(DB_ARGS(rf, st->by4, st->bx4,
                                           y_off, x_off, "lnr-spc", n)
                                   st->dr, &st->drvd_cnt, 4, weight, cand_mv,
                                   0, 0, &st->drvd_iter_cntr, 2);
            }
            if (b->ref.ref[1] < 0 && b->ref.ref[0] - 1 != TIP_FRAME) break;
        }
    } else if (b->ref.ref[0] - 1 == TIP_FRAME &&
               ref.ref[0] - 1 == rf->frm_hdr->tip.refs[0] &&
               ref.ref[1] - 1 == rf->frm_hdr->tip.refs[1])
    {
        const mv tmv = rt->rp_proj[off_8x8].mv;
        const mv tip0mv = scale_mv(tmv, rf->tip_sf[0]);
        const mv tip1mv = scale_mv(tmv, rf->tip_sf[1]);
        const refmvs_mvpair cand_mv = { .mv = {
            [0] = {
                .y = iclip(tip0mv.y + b->mv.mv[0].y, -0xffff, 0xffff),
                .x = iclip(tip0mv.x + b->mv.mv[0].x, -0xffff, 0xffff),
            }, [1] = {
                .y = iclip(tip1mv.y + b->mv.mv[0].y, -0xffff, 0xffff),
                .x = iclip(tip1mv.x + b->mv.mv[0].x, -0xffff, 0xffff),
            }
        }};
        add_candidate_comp(DB_ARGS(rf, st->by4, st->bx4,
                                   y_off, x_off, "tip-spc")
                           st->mv, st->cnt, 6, weight, 8, cand_mv,
                           &st->iter_cntr, 16);
    } else if (b->ref.pair == ref.pair) {
        const refmvs_mvpair cand_mv = { .mv = {
            [0] = ((b->mf & 1) && gmv[0].n != INVALID_MV) ? gmv[0] : b->mv.mv[0],
            [1] = ((b->mf & 1) && gmv[1].n != INVALID_MV) ? gmv[1] : b->mv.mv[1],
        }};
        add_candidate_comp(DB_ARGS(rf, st->by4, st->bx4,
                                   y_off, x_off, "spc")
                           st->mv, st->cnt, 6, weight, (b->mf >> 3) - 4,
                           cand_mv, &st->iter_cntr, 16);
    } else {
        if (rf->seq_hdr->mv_traj && rf->frm_hdr->use_ref_frame_mvs &&
            b->ref.ref[0] - 1 != TIP_FRAME && ref.ref[0] != ref.ref[1] &&
            rt->rp_traj[ref.ref[0] - 1][st->b8x8].n != INVALID_MV &&
            rt->rp_traj[ref.ref[1] - 1][st->b8x8].n != INVALID_MV)
        {
            const mv b1_mv = rt->rp_traj[ref.ref[0] - 1][st->b8x8];
            const mv b2_mv = rt->rp_traj[ref.ref[1] - 1][st->b8x8];
            for (int n = 0; n < 2 && b->ref.ref[n] > 0; n++) {
                const mv a_mv = rt->rp_traj[b->ref.ref[n] - 1][st->b8x8];
                if (a_mv.n == INVALID_MV) continue;
                const refmvs_mvpair cand_mv = (refmvs_mvpair) { .mv = {
                    [0] = {
                        .y = iclip(b->mv.mv[n].y + b1_mv.y - a_mv.y,
                                   -0xffff, 0xffff),
                        .x = iclip(b->mv.mv[n].x + b1_mv.x - a_mv.x,
                                   -0xffff, 0xffff),
                    }, [1] = {
                        .y = iclip(b->mv.mv[n].y + b2_mv.y - a_mv.y,
                                   -0xffff, 0xffff),
                        .x = iclip(b->mv.mv[n].x + b2_mv.x - a_mv.x,
                                   -0xffff, 0xffff),
                    },
                }};
                add_candidate_comp(DB_ARGS(rf, st->by4, st->bx4,
                                           y_off, x_off, "mvtj-spc")
                                   st->dr, &st->drvd_cnt, 4, weight, 8,
                                   cand_mv, &st->drvd_iter_cntr, 2);
                if (b->ref.ref[1] <= 0) break;
            }
        }

        int ns = 1;
        if (ref.ref[0] == b->ref.ref[0] || ref.ref[0] == b->ref.ref[1]) {
            ns = 0;
        } else if (ref.ref[1] != b->ref.ref[0] && ref.ref[1] != b->ref.ref[1])
            return;
        const int nc = ref.ref[ns] != b->ref.ref[0];
        int oidx;
        for (oidx = 0; oidx < st->sngl_cnt; oidx++)
            if (ref.ref[!ns] == st->sngl[oidx].ref)
                break;
        if (oidx < st->sngl_cnt) {
            refmvs_mvpair cand_mv;
            cand_mv.mv[ns] = b->mv.mv[nc];
            cand_mv.mv[!ns] = st->sngl[oidx].mv;
            add_candidate_comp(DB_ARGS(rf, st->by4, st->bx4,
                                       y_off, x_off, "mvxp-spc")
                               st->dr, &st->drvd_cnt, 4, weight, 8,
                               cand_mv, &st->drvd_iter_cntr, 2);
        }
        add_candidate_c2s(DB_ARGS(rf, st->by4, st->bx4,
                                  y_off, x_off, "sngl-c", ns)
                          st->sngl, &st->sngl_cnt, 4, b->ref.ref[nc],
                          b->mv.mv[nc], &st->sngl_iter_cntr, 2);
    }
}

static void add_derived(DB_ARGS(const refmvs_frame *const rf)
                        struct refmvs_state *const st,
                        const int lim, const int comp)
{
    if (*st->cnt >= lim) return;
    for (int n = 0; n < st->drvd_cnt; n++)
        if (comp) {
            add_candidate_comp(DB_ARGS(rf, st->by4, st->bx4,
                                       0, 0, "derived")
                               st->mv, st->cnt, lim, 0, 8,
                               st->dr[n].mv, &st->iter_cntr, 16);
        } else {
            add_candidate_sngl(DB_ARGS(rf, st->by4, st->bx4,
                                       0, 0, "derived", n)
                               st->mv, st->cnt, lim, 0, st->dr[n].mv.mv[0],
                               0, 0, &st->iter_cntr, 16);
        }
}

static const uint16_t div_mult[32] = {
       0, 16384, 8192, 5461, 4096, 3276, 2730, 2340,
    2048,  1820, 1638, 1489, 1365, 1260, 1170, 1092,
    1024,   963,  910,  862,  819,  780,  744,  712,
     682,   655,  630,  606,  585,  564,  546,  528
};

mv mv_projection(const union mv mv, const int num, const int den) {
    assert(den > 0 && den < 32);
    assert(num > -32 && num < 32);
    const int frac = num * div_mult[den];
    const int y = mv.y * frac, x = mv.x * frac;
    // Round and clip according to AV1 spec section 7.9.3
    return (union mv) { // 0x3fff == (1 << 14) - 1
        .y = iclip((y + 8192 + (y >> 31)) >> 14, -0x3fff, 0x3fff),
        .x = iclip((x + 8192 + (x >> 31)) >> 14, -0x3fff, 0x3fff)
    };
}

static int add_temporal_candidate(const refmvs_tile *const rt,
                                  struct refmvs_state *const st,
                                  const ptrdiff_t off_8x8,
                                  const union refmvs_refpair ref)
{
    const refmvs_frame *const rf = rt->rf;

    if (ref.ref[0] - 1 == TIP_FRAME || !ref.ref[0] /* intrabc */) return 0;
    union mv mv = rt->rp_traj[ref.ref[0] - 1][off_8x8];
    if (mv.n == INVALID_MV) {
        mv = rt->rp_proj[off_8x8].mv;
        if (mv.n == INVALID_MV) return 0;
        mv = mv_projection(mv, rf->pocdiff[ref.ref[0] - 1],
                           rf->frm_hdr->tip.frame_mode ?
                               rf->tip_delta : rt->rp_proj[off_8x8].ref);
    }

    if (ref.ref[1] == -1) {
        const int weight = 1 + (rf->abspocdiff[ref.ref[0] - 1] <= 2);
        return add_candidate_sngl(DB_ARGS(rf, st->by4, st->bx4,
                                          (int) (off_8x8 / rf->rp_stride),
                                          (int) (off_8x8 % rf->rp_stride),
                                          "tpl", 0)
                                  st->mv, st->cnt, 6, weight, mv, 0, 0,
                                  &st->iter_cntr, 16);
    }
    union mv mv2 = rt->rp_traj[ref.ref[1] - 1][off_8x8];
    if (mv2.n == INVALID_MV) {
        mv2 = rt->rp_proj[off_8x8].mv;
        if (mv2.n == INVALID_MV) return 0;
        mv2 = mv_projection(mv2, rf->pocdiff[ref.ref[1] - 1],
                            rf->frm_hdr->tip.frame_mode ?
                                rf->tip_delta : rt->rp_proj[off_8x8].ref);
    }
    const refmvs_mvpair mvp = { .mv = {
        [0] = mv,
        [1] = mv2,
    }};
    return add_candidate_comp(DB_ARGS(rf, st->by4, st->bx4,
                                      (int) (off_8x8 / rf->rp_stride),
                                      (int) (off_8x8 % rf->rp_stride), "tpl")
                              st->mv, st->cnt, 6, 1, 8, mvp, &st->iter_cntr, 16);
}

static int model_from_corners(DB_ARGS(const int idx)
                              int32_t *const mat, const mv topleft_mv,
                              const mv topright_mv, const mv bottomleft_mv,
                              const int xpos, const int ypos,
                              const uint8_t *const b_dim)
{
    if (topright_mv.n == topleft_mv.n && bottomleft_mv.n == topleft_mv.n)
        return 0;
    if (imin(imin(topleft_mv.x, bottomleft_mv.x), topright_mv.x + b_dim[0] * 32) < -xpos * 8)
        return 0;
    if (imin(imin(topleft_mv.y, topright_mv.y), bottomleft_mv.y + b_dim[1] * 32) < -ypos * 8)
        return 0;

    mat[2] = ((topright_mv.x - topleft_mv.x) * (1 << 11)) >> b_dim[2];
    mat[4] = ((topright_mv.y - topleft_mv.y) * (1 << 11)) >> b_dim[2];
    mat[3] = ((bottomleft_mv.x - topleft_mv.x) * (1 << 11)) >> b_dim[3];
    mat[5] = ((bottomleft_mv.y - topleft_mv.y) * (1 << 11)) >> b_dim[3];
    mat[0] = topleft_mv.x * (1 << 13) - xpos * mat[2] - ypos * mat[3];
    mat[1] = topleft_mv.y * (1 << 13) - xpos * mat[4] - ypos * mat[5];
#define reduce(i) \
    mat[i] = iclip(mat[i], -0x7fc0, 0x7fc0); \
    mat[i] += 0x20 - (mat[i] > 0); \
    mat[i] &= ~0x3f
    reduce(2);
    reduce(3);
    reduce(4);
    reduce(5);
#undef reduce
    mat[2] += 0x10000;
    mat[5] += 0x10000;
    mat[6] = DAV1D_WM_TYPE_AFFINE;

    return 1;
}

/*
 * refmvs_frame allocates memory for one sbrow (32 blocks high, whole frame
 * wide) of 4x4-resolution refmvs_block entries for spatial MV referencing.
 * mvrefs_tile[] keeps a list of 35 (32 + 3 above) pointers into this memory,
 * and each sbrow, the bottom entries (y=27/29/31) are exchanged with the top
 * (-5/-3/-1) pointers by calling dav1d_refmvs_tile_sbrow_init() at the start
 * of each tile/sbrow.
 *
 * For temporal MV referencing, we call dav1d_refmvs_save_tmvs() at the end of
 * each tile/sbrow (when tile column threading is enabled), or at the start of
 * each interleaved sbrow (i.e. once for all tile columns together, when tile
 * column threading is disabled). This will copy the 4x4-resolution spatial MVs
 * into 8x8-resolution refmvs_temporal_block structures. Then, for subsequent
 * frames, at the start of each tile/sbrow (when tile column threading is
 * enabled) or at the start of each interleaved sbrow (when tile column
 * threading is disabled), we call load_tmvs(), which will project the MVs to
 * their respective position in the current frame.
 */

void dav1d_refmvs_find(const refmvs_tile *const rt,
                       refmvs_candidate mvstack[6], int32_t (*const warp)[7],
                       int *const cnt, const union refmvs_refpair ref,
                       const enum BlockSize bs, const int by4, const int bx4)
{
    const refmvs_frame *const rf = rt->rf;
    const uint8_t *const b_dim = dav1d_block_dimensions[bs];
    const int bw4 = b_dim[0], w4 = imin(bw4, rt->tile_col.end - bx4);
    const int bh4 = b_dim[1], h4 = imin(bh4, rt->tile_row.end - by4);
    mv gmv[2];
    const int comp = ref.ref[1] > 0;

    DEBUG_REFMV_printf("setup_ref_mv_list(%d,%d) for y=%d,x=%d\n",
                       ref.ref[0] - 1, ref.ref[1] - (ref.ref[1] >= 0), by4, bx4);

    *cnt = 0;
    if (warp) cnt[1] = 0;
    assert(ref.ref[0] >=  0 && ref.ref[0] <= 8 &&
           ref.ref[1] >= -1 && ref.ref[1] <= 7);
    gmv[0] = ref.ref[0] <= 0 ? (mv) { .n = 0 } :
             get_gmv_2d(&rf->frm_hdr->gmv[ref.ref[0] - 1],
                        bx4, by4, bw4, bh4, rf->frm_hdr);
    if (comp) {
        gmv[1] = get_gmv_2d(&rf->frm_hdr->gmv[ref.ref[1] - 1],
                            bx4, by4, bw4, bh4, rf->frm_hdr);
        DEBUG_REFMV_printf("Gmv2d: y=%d,x=%d, y2=%d,x2=%d\n",
                           gmv[0].y, gmv[0].x, gmv[1].y, gmv[1].x);
    } else {
        gmv[1].n = 0;
        if (ref.ref[0] > 0)
            DEBUG_REFMV_printf("Gmv2d: y=%d,x=%d\n", gmv[0].y, gmv[0].x);
    }

    const int is_sb_boundary = !(by4 & (rf->sbsz - 1));
    const int have_left = bx4 > rt->tile_col.start;
    const refmvs_block *const bml = have_left && bh4 == h4 ?
        &rt->r[((by4 + bh4 - 1) & 63) * 128 + ((bx4 - 1) & 127)] : NULL;
    const int have_top = by4 > rt->tile_row.start;
    int x_off, abw4;
    const refmvs_block *tl = NULL, *lmt = NULL, *rmt = NULL, *tr = NULL;
    if (have_top) {
        if (is_sb_boundary) {
            x_off = bx4 & 1;
            abw4 = (bw4 + 1) & ~1;
            if (bx4 - x_off - 2 >= rt->tile_col.start)
                tl = bx4 & (rf->sbsz - 2) ? &rt->ra[(bx4 >> 1) - 1] : &rt->ra_tl;
            if (bw4 > 2) lmt = &rt->ra[bx4 >> 1];
            if (bw4 == w4) rmt = &rt->ra[(bx4 >> 1) + (abw4 >> 1) - 1];
            if (bx4 - x_off + abw4 < rt->tile_col.end && bw4 <= 16)
                    tr = &rt->ra[(bx4 >> 1) + (abw4 >> 1)];
        } else {
            x_off = 0;
            abw4 = bw4;
            if (have_left) tl = &rt->r[((by4 - 1) & 63) * 128 + ((bx4 - 1) & 127)];
            if (bw4 > 1) lmt = &rt->r[((by4 - 1) & 63) * 128 + (bx4 & 127)];
            if (bw4 == w4) rmt = &rt->r[((by4 - 1) & 63) * 128 + ((bx4 + bw4 - 1) & 127)];
            if ((bx4 + bw4) & (rf->sbsz - 1) && bx4 + bw4 < rt->tile_col.end && bw4 <= 16)
                tr = &rt->r[((by4 - 1) & 63) * 128 + ((bx4 + bw4) & 127)];
        }
    }
    if (warp) {
        DEBUG_REFMV_printf("Warp corners [%d|%d]\n", *cnt, cnt[1]);
        int bl_ref_idx;
        if (bml && (!(bl_ref_idx = (bml->ref.ref[0] != ref.ref[0])) ||
                    (bml->ref.ref[1] == ref.ref[0] && !(bml->mf & 2))))
        {
            int tl_ref_idx, tr_ref_idx;
            const mv bl_mv = !(bml->mf & 2) ? bml->mv.mv[bl_ref_idx] :
                get_warpmv_proj(bml->m, bx4 * 4, (by4 + bh4) * 4);
            if (tl && (!(tl_ref_idx = (tl->ref.ref[0] != ref.ref[0])) ||
                       (tl->ref.ref[1] == ref.ref[0] && !(tl->mf & 2))) &&
                rmt && (!(tr_ref_idx = (rmt->ref.ref[0] != ref.ref[0])) ||
                        (rmt->ref.ref[1] == ref.ref[0] && !(rmt->mf & 2))))
            {
                const mv tl_mv = !(tl->mf & 2) ? tl->mv.mv[tl_ref_idx] :
                    get_warpmv_proj(tl->m, bx4 * 4, by4 * 4);
                const mv tr_mv = !(rmt->mf & 2) ? rmt->mv.mv[tr_ref_idx] :
                    get_warpmv_proj(rmt->m, (bx4 + bw4) * 4, by4 * 4);
                cnt[1] = model_from_corners(DB_ARGS(0)
                                            warp[0], tl_mv, tr_mv, bl_mv,
                                            bx4 * 4, by4 * 4, b_dim);
            }
            if (!cnt[1] &&
                lmt && (!(tl_ref_idx = (lmt->ref.ref[0] != ref.ref[0])) ||
                        (lmt->ref.ref[1] == ref.ref[0] && !(lmt->mf & 2))) &&
                tr && (!(tr_ref_idx = (tr->ref.ref[0] != ref.ref[0])) ||
                        (tr->ref.ref[1] == ref.ref[0] && !(tr->mf & 2))))
            {
                const mv tl_mv = !(lmt->mf & 2) ? lmt->mv.mv[tl_ref_idx] :
                    get_warpmv_proj(lmt->m, bx4 * 4, by4 * 4);
                const mv tr_mv = !(tr->mf & 2) ? tr->mv.mv[tr_ref_idx] :
                    get_warpmv_proj(tr->m, (bx4 + bw4) * 4, by4 * 4);
                cnt[1] = model_from_corners(DB_ARGS(1)
                                            warp[0], tl_mv, tr_mv, bl_mv,
                                            bx4 * 4, by4 * 4, b_dim);
            }
        }
    }

    const ptrdiff_t stride = rf->rp_stride;
    const ptrdiff_t tms_8x8y = ((by4 & (rf->sbsz - 1)) >> 1) * stride;
    const ptrdiff_t lms_8x8x = bx4 >> 1;
    struct refmvs_state st = {
        .mv = mvstack,
        .cnt = cnt,
        .drvd_cnt = 0,
        .sngl_cnt = 0,
        .iter_cntr = 0,
        .drvd_iter_cntr = 0,
        .sngl_iter_cntr = 0,
        .b8x8 = lms_8x8x + tms_8x8y,
#if DEBUG_BLOCK_INFO && DEBUG_REFMV
        .by4 = by4,
        .bx4 = bx4,
#endif
    };

    // FIXME high-priority TMVP

    DEBUG_REFMV_printf("Spatial MVP [%d|%d]\n", *cnt, warp ? cnt[1] : 0);

    // bottom-most left
    const ptrdiff_t bms_8x8y =
        (((by4 + bh4 - 1) & (rf->sbsz - 1)) >> 1) * stride;
    const ptrdiff_t left_8x8x = (bx4 - 1) >> 1;
    if (bml) {
        add_spatial_candidate(bh4 - 1, -1,
                              rt, &st, 1, bml, bms_8x8y + left_8x8x,
                              ref, gmv);
        if (warp && bml->mf & 2 && bml->ref.ref[0] == ref.ref[0])
            memcpy(warp[cnt[1]++], bml->m, sizeof(int32_t) * 7);
    }

    // right-most top
    const ptrdiff_t top_8x8y = (((by4 - 1) & (rf->sbsz - 1)) >> 1) * stride;
    if (rmt) {
        const int xpos = abw4 - (1 << is_sb_boundary) - x_off;
        add_spatial_candidate(-1, xpos,
                              rt, &st, xpos >= 0, rmt,
                              top_8x8y + ((bx4 + bw4 - 1) >> 1), ref, gmv);
        if (warp && rmt->mf & 2 && rmt->ref.ref[0] == ref.ref[0]) {
            memcpy(warp[cnt[1]++], rmt->m, sizeof(int32_t) * 7);
        }
    }

    // top-most left
    const refmvs_block *tml = NULL;
    if (have_left && bh4 > 1) {
        tml = &rt->r[(by4 & 63) * 128 + ((bx4 - 1) & 127)];
        add_spatial_candidate(0, -1,
                              rt, &st, 1, tml, tms_8x8y + left_8x8x, ref, gmv);
        if (warp && tml->mf & 2 && tml->ref.ref[0] == ref.ref[0])
            memcpy(warp[cnt[1]++], tml->m, sizeof(int32_t) * 7);
    }

    // left-most top
    if (lmt) {
        add_spatial_candidate(-1, -x_off,
                              rt, &st, !x_off, lmt, top_8x8y + lms_8x8x,
                              ref, gmv);
        if (warp && cnt[1] < 4 && lmt->mf & 2 && lmt->ref.ref[0] == ref.ref[0]) {
            memcpy(warp[cnt[1]++], lmt->m, sizeof(int32_t) * 7);
        }
    }

    // bottom-left
    if (have_left && (by4 + bh4) & (rf->sbsz - 1) &&
        by4 + bh4 < rt->tile_row.end)
    {
        const refmvs_block *const bl = &rt->r[((by4 + bh4) & 63) * 128 + ((bx4 - 1) & 127)];
        add_spatial_candidate(bh4, -1,
                              rt, &st, 1, bl, left_8x8x +
                              (((by4 + bh4) & (rf->sbsz - 1)) >> 1) * stride,
                              ref, gmv);
        if (warp && cnt[1] < 4 && bl->mf & 2 && bl->ref.ref[0] == ref.ref[0])
            memcpy(warp[cnt[1]++], bl->m, sizeof(int32_t) * 7);
    }

    // top-right
    if (tr && tr->mv.mv[0].n != INVALID_MV) {
        add_spatial_candidate(-1, abw4 - x_off,
                              rt, &st, 1, tr, top_8x8y + ((bx4 + bw4) >> 1),
                              ref, gmv);
        if (warp && cnt[1] < 4 && tr->mf & 2 && tr->ref.ref[0] == ref.ref[0]) {
            memcpy(warp[cnt[1]++], tr->m, sizeof(int32_t) * 7);
        }
    }

    // normal priority TMVP
    DEBUG_REFMV_printf("Low-priority TMVP [%d|%d]\n", *cnt, warp ? cnt[1] : 0);
    if (rf->use_ref_frame_mvs && ref.ref[0] != ref.ref[1]) {
        const int bw8 = imin(bw4 >> 1, 8), bh8 = imin(bh4 >> 1, 8);
        const int step_h = bw4 >= 16 ? 2 : 1, step_v = bh4 >= 16 ? 2 : 1;
        const int first = (unsigned) 2 * bw8 - 2 * step_h <= (unsigned) w4 &&
                          (unsigned) 2 * bh8 - 2 * step_v <= (unsigned) h4 &&
            add_temporal_candidate(rt, &st,
                                   (((by4 + 2 * bh8 - 2 * step_v) &
                                     (rf->sbsz - 1)) >> 1) * stride +
                                   ((bx4 + 2 * bw8 - 2 * step_h) >> 1),
                                   ref);
        if (!first && (bw4 > 4 || bh4 > 4)) {
            add_temporal_candidate(rt, &st,
                                   (((by4 + bh8) & (rf->sbsz - 1)) >> 1) *
                                       stride + ((bx4 + bw8) >> 1),
                                   ref);
        }
    }

    // top-left
    DEBUG_REFMV_printf("Extra Spatial MVP [%d|%d]\n", *cnt, warp ? cnt[1] : 0);
    if (tl) {
        add_spatial_candidate(-1, -(1 << is_sb_boundary) - x_off,
                              rt, &st, 0, tl, top_8x8y + left_8x8x, ref, gmv);
        if (warp && cnt[1] < 4 && tl->mf & 2 && tl->ref.ref[0] == ref.ref[0]) {
            memcpy(warp[cnt[1]++], tl->m, sizeof(int32_t) * 7);
        }
    }

    const int nearest_refmv_count = *cnt;

    if (have_left) {
        DEBUG_REFMV_printf("Spatial Ext [left] MVP [%d|%d]\n",
                           *cnt, warp ? cnt[1] : 0);
        const int adj = 3 - (bx4 & (bw4 == 1));
        if (bx4 - adj >= rt->tile_col.start) {
            if (bh4 == h4) {
                const int pos = ((by4 + bh4 - 1) & 63) * 128 + ((bx4 - adj) & 127);
                const refmvs_block *const cand_b = &rt->r[pos];
                assert(bml);
                if (dav1d_block_dimensions[cand_b->bs][0] < adj ||
                    cand_b->bs != bml->bs)
                {
                    add_spatial_candidate(bh4 - 1, -adj,
                                          rt, &st, 0, cand_b,
                                          bms_8x8y + ((bx4 - adj) >> 1),
                                          ref, gmv);
                    if (warp && cnt[1] < 4 && cand_b->mf & 2 &&
                        cand_b->ref.ref[0] == ref.ref[0])
                    {
                        memcpy(warp[cnt[1]++], cand_b->m, sizeof(int32_t) * 7);
                    }
                }
            }

            if (bh4 > 1) {
                const int pos = (by4 & 63) * 128 + ((bx4 - adj) & 127);
                const refmvs_block *const cand_b = &rt->r[pos];
                assert(tml);
                if (dav1d_block_dimensions[cand_b->bs][0] < adj ||
                    cand_b->bs != tml->bs)
                {
                    add_spatial_candidate(0, -adj,
                                          rt, &st, 0, cand_b,
                                          tms_8x8y + ((bx4 - adj) >> 1),
                                          ref, gmv);
                    if (warp && cnt[1] < 4 && cand_b->mf & 2 &&
                        cand_b->ref.ref[0] == ref.ref[0])
                    {
                        memcpy(warp[cnt[1]++], cand_b->m, sizeof(int32_t) * 7);
                    }
            }
            }
        }
    }

    // sort
    if ((rf->seq_hdr->drl_reorder == 2 /* always */ &&
         nearest_refmv_count >= 2) ||
        (rf->seq_hdr->drl_reorder == 1 /* constraint */ &&
         (/*!is_tmvp_high_priority &&*/ nearest_refmv_count >= 4)))
    {
        int maxwidx = 0, maxw = mvstack[0].weight;
        for (int n = 1; n < nearest_refmv_count; n++) {
            const int w = mvstack[n].weight;
            if (w > maxw) {
                maxw = w;
                maxwidx = n;
            }
        }
        if (maxwidx) {
            refmvs_candidate tmp = mvstack[maxwidx];
            mvstack[maxwidx] = mvstack[0];
            mvstack[0] = tmp;
        }
     }

    DEBUG_REFMV_printf("Derived Spatial MVP & refbank [%d|%d]\n",
                       *cnt, warp ? cnt[1] : 0);
    const int lim = 1 + (ref.ref[0] ? rf->frm_hdr->max_drl_bits :
                                      rf->frm_hdr->max_bvp_drl_bits);
    if (ref.ref[1] != -1) add_derived(DB_ARGS(rf) &st, lim, 1);
    if (rf->seq_hdr->refmv_bank) {
        const int c = ref.ref[1] == -1 ? (ref.ref[0] - 1U <= 5U ? ref.ref[0] - 1 : 8) :
                      (ref.ref[0] == 1 && ref.ref[1] <= 2) ? 5 + ref.ref[1] : 8;
        const int sz = rt->bank.size[c], idx = rt->bank.idx[c];
        const int start = sz + idx - 1;
        for (int n = 0; n < sz && *cnt < lim; n++) {
            const int bank_idx = (start - n) & 3;
            if (c == 8 && rt->bank.ref[bank_idx].pair != ref.pair) continue;
            const refmvs_mvpair *const mv = &rt->bank.mv[c][bank_idx];
            const int last = *cnt;
            RDB_ONLY(int did_check = 0);
            if (st.iter_cntr < 16) {
                for (int m = 0; m < last; m++)
                    if (mvstack[m].mv.mv[0].n == mv->mv[0].n &&
                        mvstack[m].mv.mv[comp].n == mv->mv[comp].n)
                    {
                        st.iter_cntr += m + 1;
                        DEBUG_REFMV_printf("insert_bank[%d/%d:%d]: skipping[%d] y=%d,x=%d,y2=%d,x2=%d\n",
                                           n, sz, st.iter_cntr,
                                           m, mv->mv[0].y, mv->mv[0].x, comp ? mv->mv[1].y : 0,
                                           comp ? mv->mv[1].x : 0);
                        goto end;
                    }
                RDB_ONLY(did_check = 1);
                st.iter_cntr += last;
            }
            int i;
            for (i = 0; i <= comp; i++) {
                const int rx = bx4 * 4 + apply_sign(abs(mv->mv[i].x) >> 3,
                                                    mv->mv[i].x);
                const int ry = by4 * 4 + apply_sign(abs(mv->mv[i].y) >> 3,
                                                    mv->mv[i].y);
                if (rx <= -bw4 * 4 || ry <= -bh4 * 4 ||
                    rx >= rf->iw8 * 8 || ry >= rf->ih8 * 8)
                {
                    break;
                }
            }
            if (i <= comp) continue;
            DEBUG_REFMV_printf("insert_bank[%d/%d:%d]: %s[%d] y=%d,x=%d,y2=%d,x2=%d,w=%d\n",
                               n, sz, st.iter_cntr, did_check ? "adding" : "tailing",
                               last, mv->mv[0].y, mv->mv[0].x, comp ? mv->mv[1].y : 0,
                               comp ? mv->mv[1].x : 0, 0);
            mvstack[last].mv.mv[0].n = mv->mv[0].n;
            mvstack[last].mv.mv[1].n = mv->mv[1].n;
            mvstack[last].weight = 0;
            if (ref.ref[1] > 0)
                mvstack[last].cwp_idx = rt->bank.cwp_idx[c - 6][bank_idx];
            mvstack[last].y_off = mvstack[last].x_off = 0;
            *cnt = last + 1;
        end: {}
        }
    }
    if (ref.ref[1] == -1) add_derived(DB_ARGS(rf) &st, lim, 0);

    DEBUG_REFMV_printf("GMVs [%d|%d]\n", *cnt, warp ? cnt[1] : 0);
    if (*cnt < 6 && ref.ref[0] > 0) {
        int last = *cnt;
        RDB_ONLY(int did_check = 0;)
        if (st.iter_cntr < 16) {
            for (int n = 0; n < last; n++)
                if (mvstack[n].mv.mv[0].n == gmv[0].n &&
                    mvstack[n].mv.mv[comp].n == gmv[comp].n)
                {
                    st.iter_cntr += n + 1;
                    DEBUG_REFMV_printf("gmv_add[%d]: skipping[%d] y=%d,x=%d,"
                                       "y2=%d,x2=%d from GMV\n",
                                       st.iter_cntr, n,
                                       gmv[0].y, gmv[0].x, gmv[1].y, gmv[1].x);
                    goto end_gmv;
                }
            RDB_ONLY(did_check = 1);
            st.iter_cntr += last;
        }
        mvstack[last].mv.mv[0] = gmv[0];
        mvstack[last].mv.mv[1] = gmv[1];
        mvstack[last].weight = 0;
        mvstack[last].cwp_idx = 8;
        mvstack[last].y_off = mvstack[last].x_off = 0;
        DEBUG_REFMV_printf("gmv_add[%d]: %s[%d] y=%d,x=%d,y2=%d,x2=%d,w=%d from GMV\n",
                           st.iter_cntr, did_check ? "adding" : "tailing",
                           last, gmv[0].y, gmv[0].x, gmv[1].y, gmv[1].x, 0);
        *cnt = last + 1;
    end_gmv: {}
    }

    if (warp && cnt[1] < 4) {
        assert(ref.ref[0] > 0 && ref.ref[0] <= 7 && ref.ref[1] == -1);

        DEBUG_REFMV_printf("Warp bank [%d|%d]\n", *cnt, cnt[1]);
        const int sz = rt->warp.size[ref.ref[0] - 1];
        const int idx = rt->warp.idx[ref.ref[0] - 1];
        const int start = sz + idx - 1;
        for (int n = 0; n < sz && cnt[1] < 4; n++) {
            const int32_t *const mat =
                rt->warp.mat[ref.ref[0] - 1][(start - n) & 3];
            memcpy(warp[cnt[1]++], mat, sizeof(int32_t) * 7);
        }

        DEBUG_REFMV_printf("Warp gmv [%d|%d]\n", *cnt, cnt[1]);
        if (cnt[1] < 4) {
            const int32_t *const mat = rf->frm_hdr->gmv[ref.ref[0] - 1].matrix;
            warp[cnt[1]][6] = rf->frm_hdr->gmv[ref.ref[0] - 1].type;
            memcpy(warp[cnt[1]++], mat, sizeof(int32_t) * 6);
        }

        DEBUG_REFMV_printf("Warp defaults [%d|%d]\n", *cnt, cnt[1]);
        for (int n = 0; n < 2; n++) {
            if (cnt[1] >= 4) break;
            warp[cnt[1]][6] = dav1d_default_wm_params.type;
            memcpy(warp[cnt[1]++], dav1d_default_wm_params.matrix,
                   sizeof(int32_t) * 6);
        }
    }
    assert(*cnt <= 6);

    // default intrabc refs
    int n_refmvs = *cnt;
    if (!ref.ref[0]) {
        DEBUG_REFMV_printf("Intrabc defaults [%d|%d]\n", *cnt, warp ? cnt[1] : 0);

        if (n_refmvs < rt->rf->frm_hdr->max_bvp_drl_bits + 1) {
            const int sbsz = 64 << rt->rf->frm_hdr->sb128;
            mvstack[n_refmvs].mv.mv[0].x = 0;
            mvstack[n_refmvs].mv.mv[0].y = -(sbsz * 8);
            mvstack[n_refmvs].weight = 0;
            *cnt = ++n_refmvs;
            if (n_refmvs < rt->rf->frm_hdr->max_bvp_drl_bits + 1) {
                mvstack[n_refmvs].mv.mv[0].x = -(8 * (sbsz + 256));
                mvstack[n_refmvs].mv.mv[0].y = 0;
                mvstack[n_refmvs].weight = 0;
                *cnt = ++n_refmvs;
                if (n_refmvs < rt->rf->frm_hdr->max_bvp_drl_bits + 1) {
                    mvstack[n_refmvs].mv.mv[0].x = 0;
                    mvstack[n_refmvs].mv.mv[0].y = -(bh4 * 32);
                    mvstack[n_refmvs].weight = 0;
                    *cnt = ++n_refmvs;
                    if (n_refmvs < rt->rf->frm_hdr->max_bvp_drl_bits + 1) {
                        mvstack[n_refmvs].mv.mv[0].x = -(bw4 * 32);
                        mvstack[n_refmvs].mv.mv[0].y = 0;
                        mvstack[n_refmvs].weight = 0;
                        *cnt = ++n_refmvs;
                    }
                }
            }
        }
    }

    DEBUG_REFMV_printf("Final [%d|%d]\n", *cnt, warp ? cnt[1] : 0);
}

void dav1d_refmvs_tile_sbrow_init(refmvs_tile *const rt,
                                  const refmvs_frame *const rf,
                                  const int tile_col_start4, const int tile_col_end4,
                                  const int tile_row_start4, const int tile_row_end4,
                                  const int sby, int tile_row_idx, const int pass)
{
    if (rf->n_tile_threads == 1) tile_row_idx = 0;
    const ptrdiff_t off1 = rf->rp_stride * tile_row_idx;
    const ptrdiff_t off2 = (rf->sbsz >> 1) * off1;
    rt->rp_proj = &rf->rp_proj[off2];
    for (int n = 0; n < 7; n++)
        rt->rp_traj[n] = &rf->rp_traj[n][off2];
    rt->ra = &rf->ra[off1];
    rt->rf = rf;
    rt->tile_row.start = tile_row_start4;
    rt->tile_row.end = imin(tile_row_end4, rf->ih4);
    rt->tile_col.start = tile_col_start4;
    rt->tile_col.end = imin(tile_col_end4, rf->iw4);
    memset(rt->bank.size, 0, sizeof(rt->bank.size));
    memset(rt->bank.idx, 0, sizeof(rt->bank.idx));
    memset(rt->warp.size, 0, sizeof(rt->warp.size));
    memset(rt->warp.idx, 0, sizeof(rt->warp.idx));
}

void dav1d_refmvs_bank_update(refmvs_tile *const rt, const enum BlockSize bs,
                              const int by4, const int bx4)
{
    const refmvs_frame *const rf = rt->rf;
    const int bsh = 1 + rf->frm_hdr->sb128, bsz = 1 << bsh;
    if (!((by4 | bx4) & (rf->sbsz - 1))) {
        const uint8_t *const b_dim = dav1d_block_dimensions[bs];
        const int w = imax(1, b_dim[0] >> bsh) * imax(1, b_dim[1] >> bsh);
        rt->bank.hits[1] = 0;
        rt->bank.avail = imax(w, 4);
        DEBUG_REFMV_printf("Resetting refbank: remain=%d|hits=%d|%d\n",
                           rt->bank.avail, rt->bank.hits[1], rt->bank.hits[0]);
    } else if (!((by4 | bx4) & (bsz - 1))) {
        const uint8_t *const b_dim = dav1d_block_dimensions[bs];
        const int w = imax(1, b_dim[0] >> bsh) * imax(1, b_dim[1] >> bsh);
        rt->bank.hits[1] = 0;
        rt->bank.avail += w;
        DEBUG_REFMV_printf("Updating refbank availability: remain=%d|hits=%d|%d\n",
                           rt->bank.avail, rt->bank.hits[1], rt->bank.hits[0]);
    }
}

#if DEBUG_BLOCK_INFO && DEBUG_REFMV
static void debug_warpbank(const refmvs_tile *const rt, const int ref,
                           const int by4, const int bx4)
{
    const refmvs_frame *const rf = rt->rf;
    const int sz = rt->warp.size[ref], idx = rt->warp.idx[ref];
    const int start = idx + sz - 1;
    for (int n = 0; n < sz; n++) {
        const int32_t *const m = rt->warp.mat[ref][(start - n) & 3];
        DEBUG_REFMV_printf("refbank[%d/%d,r=%d]: %d,%d,%d,%d,%d,%d,t=%d\n",
                           n, sz, ref, m[0], m[1], m[2], m[3], m[4], m[5], m[6]);
    }
}
#else
#define debug_warpbank(...)
#endif

void dav1d_refmvs_warp_add(refmvs_tile *const rt,
                           const Dav1dWarpedMotionParams *const mat,
                           DB_ONLY(const int by4, const int bx4)
                           const int ref)
{
    RDB_ONLY(const refmvs_frame *const rf = rt->rf);
    if (rt->warp.hits >= 64) {
        DEBUG_REFMV_printf("warprefbank: ignoring further action, hits=%d\n",
                           rt->warp.hits);
        return;
    }
    rt->warp.hits++;
    const int sz = rt->warp.size[ref], idx = rt->warp.idx[ref];
    int n;
    for (n = 0; n < sz; n++) {
        const int32_t *const m = &rt->warp.mat[ref][(idx + n) & 3][2];
        if (!memcmp(m, &mat->matrix[2], sizeof(int32_t) * 4) &&
            m[4] == (int) mat->type /* see #834 */)
        {
            break;
        }
    }
    if (n < sz) {
        const int to = sz == 4 ? (idx - 1) & 3 : sz - 1, from = (idx + n) & 3;
        DEBUG_REFMV_printf("warprefbank: reordering %d to %d [%d,%d,%d,%d,%d,%d]\n",
                           from, to,
                           mat->matrix[0], mat->matrix[1], mat->matrix[2],
                           mat->matrix[3], mat->matrix[4], mat->matrix[5]);
        if (from != to) {
            int32_t bak[7];
            memcpy(bak, rt->warp.mat[ref][from], sizeof(int32_t) * 7);
            for (int n1 = from, n2 = (n1 + 1) & 3; n1 != to;
                 n1 = n2, n2 = (n2 + 1) & 3)
            {
                memcpy(rt->warp.mat[ref][n1], rt->warp.mat[ref][n2],
                       sizeof(int32_t) * 7);
            }
            memcpy(rt->warp.mat[ref][to], bak, sizeof(int32_t) * 7);
        }
        debug_warpbank(rt, ref, by4, bx4);
        return;
    }
    const int tgt = sz == 4 ? rt->warp.idx[ref]++ & 3 : rt->warp.size[ref]++;
    memcpy(rt->warp.mat[ref][tgt], mat->matrix, sizeof(int32_t) * 6);
    rt->warp.mat[ref][tgt][6] = mat->type;
    DEBUG_REFMV_printf("warprefbank: adding at %d|%d [%d,%d,%d,%d,%d,%d]\n",
                       rt->warp.size[ref], rt->warp.idx[ref],
                       mat->matrix[0], mat->matrix[1], mat->matrix[2],
                       mat->matrix[3], mat->matrix[4], mat->matrix[5]);
    debug_warpbank(rt, ref, by4, bx4);
}

#if DEBUG_BLOCK_INFO && DEBUG_REFMV
static void debug_refbank(const refmvs_tile *const rt, const int c,
                          const int by4, const int bx4)
{
    const refmvs_frame *const rf = rt->rf;
    const int sz = rt->bank.size[c], idx = rt->bank.idx[c];
    const int start = idx + sz - 1;
    for (int n = 0; n < sz; n++) {
        const int idx = (start - n) & 3;
        const refmvs_refpair ref = rt->bank.ref[idx];
        const int comp = c - 6U < 2U || (c == 8 && ref.ref[1] != -1);
        DEBUG_REFMV_printf("refbank[%d/%d,c=%d]: mv=y:%d,x:%d,y2=%d,x2=%d,r=%d,%d\n",
                           n, sz, c,
                           rt->bank.mv[c][idx].mv[0].y,
                           rt->bank.mv[c][idx].mv[0].x,
                           comp ? rt->bank.mv[c][idx].mv[1].y : 0,
                           comp ? rt->bank.mv[c][idx].mv[1].x : 0,
                           c < 6 ? c : c < 8 ? 0 : ref.ref[0] - 1,
                           c < 6 ? -1 : c < 8 ? c - 6 : ref.ref[1] - (ref.ref[1] > 0));
    }
}
#else
#define debug_refbank(...)
#endif

static void refmvs_bank_add(refmvs_tile *const rt,
                            DB_ONLY(const int by4, const int bx4)
                            const int8_t ref[2], const union mv mv[2],
                            const int cwp_idx)
{
    RDB_ONLY(const refmvs_frame *const rf = rt->rf);

    rt->bank.hits[0]++;

    const int c = ref[1] == -1 ? ((unsigned) ref[0] <= 5U ? ref[0] : 8) :
                  (!ref[0] && ref[1] <= 1) ? 6 + ref[1] : 8;
    const int sz = rt->bank.size[c], idx = rt->bank.idx[c];
    const int comp = ref[1] != -1;
    int n;
    for (n = 0; n < sz; n++) {
        const int i = (idx + n) & 3;
        if (mv[0].n == rt->bank.mv[c][i].mv[0].n &&
            mv[comp].n == rt->bank.mv[c][i].mv[comp].n &&
            (c < 8 || (ref[0] + 1 == rt->bank.ref[i].ref[0] &&
                       ref[comp] + 1 == rt->bank.ref[i].ref[comp])))
        {
            break;
        }
    }
    if (n < sz) {
        const int to = sz == 4 ? (idx - 1) & 3 : sz - 1, from = (idx + n) & 3;
        DEBUG_REFMV_printf("Moving refbank entry %d to tail %d | remain=%d|hits=%d|%d\n",
                           from, to,
                           rt->bank.avail,
                           rt->bank.hits[1],
                           rt->bank.hits[0]);
        if (from != to) {
            refmvs_mvpair mv_bak;
            refmvs_refpair ref_bak;
            mv_bak.n = rt->bank.mv[c][from].n;
            ref_bak.pair = rt->bank.ref[from].pair;
            for (int n1 = from, n2 = (n1 + 1) & 3; n1 != to;
                 n1 = n2, n2 = (n2 + 1) & 3)
            {
                rt->bank.mv[c][n1].n = rt->bank.mv[c][n2].n;
                if (c == 8)
                    rt->bank.ref[n1].pair = rt->bank.ref[n2].pair;
                if (c >= 6)
                    rt->bank.cwp_idx[c - 6][n1] = rt->bank.cwp_idx[c - 6][n2];
            }
            rt->bank.mv[c][to].n = mv_bak.n;
            if (c == 8)
                rt->bank.ref[to].pair = ref_bak.pair;
        }
        debug_refbank(rt, c, by4, bx4);
        return;
    }

    const int tgt = sz == 4 ? rt->bank.idx[c]++ & 3 : rt->bank.size[c]++;
    rt->bank.mv[c][tgt].mv[0] = mv[0];
    rt->bank.mv[c][tgt].mv[1] = mv[1];
    if (c == 8) {
        rt->bank.ref[tgt].ref[0] = ref[0] + 1;
        rt->bank.ref[tgt].ref[1] = ref[1] + (ref[1] >= 0);
    }
    if (ref[1] != -1)
        rt->bank.cwp_idx[c - 6][tgt] = cwp_idx;
    DEBUG_REFMV_printf("Adding new refbank entry in %d | remain=%d|hits=%d|%d\n",
                       tgt, rt->bank.avail, rt->bank.hits[1], rt->bank.hits[0]);
    debug_refbank(rt, c, by4, bx4);
}

void dav1d_refmvs_bank_add(refmvs_tile *const rt, const enum BlockSize bs,
                           const int by4, const int bx4, const Av1Block *const b)
{
    RDB_ONLY(const refmvs_frame *const rf = rt->rf);

    assert(rt->rf->seq_hdr->refmv_bank);
    assert(!b->intra || b->intrabc);
    dav1d_refmvs_bank_update(rt, bs, by4, bx4);
    if (rt->bank.hits[0] >= 64 ||
        rt->bank.hits[1] >= 16 ||
        !rt->bank.avail)
    {
        DEBUG_REFMV_printf("Refbank is full: remain=%d|hits=%d|%d\n",
                           rt->bank.avail,
                           rt->bank.hits[1],
                           rt->bank.hits[0]);
        return;
    }
    rt->bank.hits[1]++;
    rt->bank.avail--;
    refmvs_bank_add(rt, DB_ONLY(by4, bx4) b->ref, b->mv,
                    b->ref[1] == -1 ? 0 : b->cwp_idx);
}

void dav1d_refmvs_reset_sb(refmvs_tile *const rt, const int by, const int bx) {
    // FIXME should (eventually) be able to re-use is_coded
    for (int y = by & 63; y < (by & 63) + rt->rf->sbsz; y++) {
        for (int x = bx & 127; x < (bx & 127) + rt->rf->sbsz; x++) {
            rt->r[y * 128 + x].mv.mv[0].n = INVALID_MV;
        }
    }

    const refmvs_frame *const rf = rt->rf;
    if (rf->seq_hdr->refmv_bank) {
        rt->bank.hits[0] = 0;
        rt->bank.hits[1] = 0;
        rt->bank.avail = 0;
    }

    rt->warp.hits = 0;

    if (by == rt->tile_row.start || IS_KEY_OR_INTRA(rf->frm_hdr)) return;

    const int end_x4 = imin(bx + rf->sbsz, rt->tile_col.end);
    for (int x = bx, sz4, hits = 0; x < end_x4; x += sz4) {
        const refmvs_block *const r = &rt->ra[x >> 1];
        sz4 = dav1d_block_dimensions[r->bs][0];
        if (r->ref.ref[0] == -1) continue;
        if (rf->seq_hdr->refmv_bank) {
            int8_t ref[2];
            ref[0] = r->ref.ref[0] - 1;
            ref[1] = r->ref.ref[1] - (r->ref.ref[1] > 0);
            refmvs_bank_add(rt, DB_ONLY(by, x) ref,
                            r->mf & 2 ? r->lmv.mv : r->mv.mv, (r->mf >> 3) - 4);
        }
        if (r->mf & 2) {
            Dav1dWarpedMotionParams wmp;
            wmp.type = r->m[6];
            memcpy(wmp.matrix, r->m, sizeof(int32_t) * 6);
            dav1d_refmvs_warp_add(rt, &wmp, DB_ONLY(by, x) r->ref.ref[0] - 1);
        }
        if (++hits == 4) break;
    }
}

static inline int dequantize_mv_comp(const int v) {
    const unsigned absv = abs(v);
    assert(v < 0x80);
    const int nbits = (absv >> 4) - (absv >= 16);
    int res = (absv - (nbits + !!nbits) * 16) << nbits;
    res += 16 * !!nbits << nbits;
    return v < 0 ? -res : res;
}

static inline mv dequantize_mv(const union qmv mv) {
    if (mv.n == INVALID_TRAJ) return (union mv) { .n = INVALID_MV };
    return (union mv) {
        .y = dequantize_mv_comp(mv.y),
        .x = dequantize_mv_comp(mv.x),
    };
}

static void fill_holes(refmvs_sngl_mv_block *const rp_proj, const ptrdiff_t stride,
                       const int col_start8, const int col_end8,
                       const int row_start8, int row_end8,
                       const int mfmv_sbsz8, const int sbsz8,
                       const int tmvp_sample_step)
{
    for (int sx = col_start8; sx < col_end8; sx += mfmv_sbsz8) {
        const int xend = imin(col_end8, sx + mfmv_sbsz8);
        for (int y = row_start8; y < row_end8; y++) {
            const int ystart = y & ~(mfmv_sbsz8 - 1);
            const int yend = imin(ystart + mfmv_sbsz8, row_end8);
            const ptrdiff_t pos_base = (y & (sbsz8 - 1)) * stride;
            for (int x = sx; x < xend; x++) {
                const ptrdiff_t pos = pos_base + x;
                const union mv mv = rp_proj[pos].mv; \
                if (mv.n == INVALID_MV) continue;
#define copy(off) do { \
                if (rp_proj[pos + off].mv.n == INVALID_MV) \
                    rp_proj[pos + off].mv = mv; \
} while (0)
                if (x - tmvp_sample_step >= sx)
                    copy(-tmvp_sample_step);
                if (x + tmvp_sample_step < xend)
                    copy(+tmvp_sample_step);
                if (y - tmvp_sample_step >= ystart)
                    copy(-tmvp_sample_step * stride);
                if (y + tmvp_sample_step < yend)
                    copy(+tmvp_sample_step * stride);
#undef copy
            }
        }
    }
}

static void smoothen(refmvs_sngl_mv_block *const rp_proj, const ptrdiff_t stride,
                     const int col_start8, const int col_end8,
                     const int row_start8, int row_end8,
                     const int mfmv_sbsz8, const int sbsz8,
                     const int tmvp_sample_step)
{
    static const unsigned idiv[] = {
        65536, 32768, 21845, 16384, 13107
    };
    union mv mv_line[32];
    for (int sx = col_start8; sx < col_end8; sx += mfmv_sbsz8) {
        const int xend = imin(col_end8, sx + mfmv_sbsz8);
        int first_line = 1, y;
        for (y = row_start8; y < row_end8; y++, first_line = 0) {
            const int ystart = y & ~(mfmv_sbsz8 - 1);
            const int yend = imin(ystart + mfmv_sbsz8, row_end8);
            const ptrdiff_t pos_base = (y & (sbsz8 - 1)) * stride;
            for (int x = sx; x < xend; x++) {
                const ptrdiff_t pos = pos_base + x;
                int sum_x = 0, sum_y = 0, sum_n = 0;
#define add(p, s) do { \
                if (rp_proj[p].mv.n != INVALID_MV) { \
                    sum_x += rp_proj[p].mv.x; \
                    sum_y += rp_proj[p].mv.y; \
                    sum_n++; \
                } \
            } while (0)
                add(pos,"self");
                if (x - tmvp_sample_step >= sx)
                    add(pos - tmvp_sample_step,"left");
                if (x + tmvp_sample_step < xend)
                    add(pos + tmvp_sample_step,"right");
                if (y - tmvp_sample_step >= ystart)
                    add(pos - tmvp_sample_step * stride,"up");
                if (y + tmvp_sample_step < yend)
                    add(pos + tmvp_sample_step * stride,"bottom");
#undef add
                if (!first_line) {
                    rp_proj[pos - stride].mv.n = mv_line[x - sx].n;
                }
                if (sum_n) {
                    mv_line[x - sx].y = (sum_y * idiv[sum_n - 1] + 0x8000 -
                                         (sum_y < 0)) >> 16;
                    mv_line[x - sx].x = (sum_x * idiv[sum_n - 1] + 0x8000 -
                                         (sum_x < 0)) >> 16;
                } else {
                    mv_line[x - sx].n = INVALID_MV;
                }
            }
        }
        if (!first_line) {
            const ptrdiff_t pos_base = ((y - 1) & (sbsz8 - 1)) * stride;
            for (int x = sx; x < xend; x++) {
                rp_proj[pos_base + x].mv.n = mv_line[x - sx].n;
            }
        }
    }
}

static void check_traj_intersect(const refmvs_frame *const rf,
                                 mv *rp_traj[7],
                                 refmvs_traj_map *map[3][7],
                                 const int ref1 /* src */,
                                 const int ref2 /* dst */,
                                 const int y, const int x,
                                 const union mv mv_in)
{
    assert(ref2 != -1);
    const unsigned sbsz8 = rf->sbsz >> 1;
    const int mfmv_sbsz8 = rf->mfmv_sbsz8;
    const int mfmv_edge = rf->mfmv_edge;
    const int shift = rf->mfmv_k_shift, mask = ~(rf->frm_hdr->tmvp_sample_step - 1);
    const ptrdiff_t stride = rf->rp_stride;
    const ptrdiff_t pos = (y & (sbsz8 - 1)) * stride + x;
    for (int k = 0; k < 3; k++) {
        refmvs_traj_map *const map1 = &map[k][ref1][pos];
        if (map1->n == INVALID_TRAJ) continue;
        const int x1 = x + map1->x;
        if ((x1 >> shift) % 3 != k) continue;
        const int x_sb_align = x1 & ~(mfmv_sbsz8 - 1);
        const int x_proj_start = imax(x_sb_align - mfmv_edge, 0);
        const int x_proj_end =
            imin(x_sb_align + mfmv_sbsz8 + rf->mfmv_edge, rf->iw8);
        if (x < x_proj_start || x >= x_proj_end) continue;
        const int y1 = y + map1->y;
        const int y_proj_start = y1 & ~(mfmv_sbsz8 - 1);
        const int y_proj_end = imin(y_proj_start + mfmv_sbsz8, rf->ih8);
        if (y < y_proj_start || y >= y_proj_end) continue;
        const ptrdiff_t pos1 = (y1 & (sbsz8 - 1)) * stride + x1;
        mv *const mv_dst = &rp_traj[ref2][pos1];
        if (mv_dst->n != INVALID_MV) continue;
        const mv *const mv_src = &rp_traj[ref1][pos1];
        const int py = mv_dst->y = iclip(mv_src->y + mv_in.y, -0xffff, 0xffff);
        const int px = mv_dst->x = iclip(mv_src->x + mv_in.x, -0xffff, 0xffff);
        const int y2 = (y1 + apply_sign(abs(py) >> 6, py)) & mask;
        const int x2 = (x1 + apply_sign(abs(px) >> 6, px)) & mask;
        if (x2 < x_proj_start || x2 >= x_proj_end) continue;
        if (y2 < y_proj_start || y2 >= y_proj_end) continue;
        const ptrdiff_t pos2 = (y2 & (sbsz8 - 1)) * stride + x2;
        refmvs_traj_map *const map2 = &map[k][ref2][pos2];
        map2->y = y1 - y2;
        map2->x = x1 - x2;
    }

    const int y1 = (y + apply_sign(abs(mv_in.y) >> 6, mv_in.y)) & mask;
    const int x1 = (x + apply_sign(abs(mv_in.x) >> 6, mv_in.x)) & mask;
    if (imin(y1, x1) < 0 || y1 >= rf->ih8 || x1 >= rf->iw8) return;
    for (int k = 0; k < 3; k++) {
        const ptrdiff_t pos1 = (y1 & (sbsz8 - 1)) * stride + x1;
        refmvs_traj_map *const map1 = &map[k][ref2][pos1];
        if (map1->n == INVALID_TRAJ) continue;
        const int x2 = x1 + map1->x;
        if ((x2 >> shift) % 3 != k) continue;
        const int x_sb_align = x2 & ~(mfmv_sbsz8 - 1);
        const int x_proj_start = imax(x_sb_align - mfmv_edge, 0);
        const int x_proj_end =
            imin(x_sb_align + mfmv_sbsz8 + rf->mfmv_edge, rf->iw8);
        if (x < x_proj_start || x >= x_proj_end) continue;
        if (x1 < x_proj_start || x1 >= x_proj_end) continue;
        const int y2 = y1 + map1->y;
        const int y_proj_start = y2 & ~(mfmv_sbsz8 - 1);
        const int y_proj_end = imin(y_proj_start + mfmv_sbsz8, rf->ih8);
        if (y < y_proj_start || y >= y_proj_end ||
            y1 < y_proj_start || y1 >= y_proj_end)
        {
            continue;
        }
        const ptrdiff_t pos2 = (y2 & (sbsz8 - 1)) * stride + x2;
        mv *const mv_dst = &rp_traj[ref1][pos2];
        if (mv_dst->n != INVALID_MV) continue;
        const mv *const mv_src = &rp_traj[ref2][pos2];
        const int py = mv_dst->y = iclip(mv_src->y - mv_in.y, -0xffff, 0xffff);
        const int px = mv_dst->x = iclip(mv_src->x - mv_in.x, -0xffff, 0xffff);
        const int y3 = (y2 + apply_sign(abs(py) >> 6, py)) & mask;
        const int x3 = (x2 + apply_sign(abs(px) >> 6, px)) & mask;
        if (x3 < x_proj_start || x3 >= x_proj_end) continue;
        if (y3 < y_proj_start || y3 >= y_proj_end) continue;
        const ptrdiff_t pos3 = (y3 & (sbsz8 - 1)) * stride + x3;
        refmvs_traj_map *const map2 = &map[k][ref1][pos3];
        map2->y = y2 - y3;
        map2->x = x2 - x3;
    }
}

// FIXME split this up in smaller DSP'able functions
void dav1d_refmvs_load_tmvs(const refmvs_frame *const rf, int tile_row_idx,
                            const int col_start8, const int col_end8,
                            const int row_start8, int row_end8)
{
    if (rf->n_tile_threads == 1) tile_row_idx = 0;
    assert(row_start8 >= 0);
    const unsigned sbsz8 = rf->sbsz >> 1;
    const int mfmv_sbsz8 = rf->mfmv_sbsz8;
    const int mfmv_edge = rf->mfmv_edge;
    assert((unsigned) (row_end8 - row_start8) <= sbsz8);
    assert(!(row_start8 & (sbsz8 - 1)));
    row_end8 = imin(row_end8, rf->ih8);
    const int col_start8i = imax(col_start8 - mfmv_edge, 0);
    const int col_end8i = imin(col_end8 + mfmv_edge, rf->iw8);

    const ptrdiff_t stride = rf->rp_stride;
    const ptrdiff_t offset = sbsz8 * stride * tile_row_idx;
    refmvs_sngl_mv_block *rp_proj = &rf->rp_proj[offset];
    for (int y = row_start8; y < row_end8; y++) {
        for (int x = col_start8; x < col_end8; x++)
            rp_proj[x].mv.n = INVALID_MV;
        rp_proj += stride;
    }
    mv *rp_traj[7];
    refmvs_traj_map *rp_map[3][7];
    if (rf->seq_hdr->mv_traj) {
        for (int n = 0; n < 7 /*rf->frm_hdr->n_ref_frames*/; n++) {
            mv *tj = rp_traj[n] = &rf->rp_traj[n][offset];
            for (int y = row_start8; y < row_end8; y++) {
                for (int x = col_start8; x < col_end8; x++)
                    tj[x].n = INVALID_MV;
                tj += stride;
            }
            for (int m = 0; m < 3; m++) {
                refmvs_traj_map *map = rp_map[m][n] = &rf->rp_map[m][n][offset];
                for (int y = row_start8; y < row_end8; y++) {
                    for (int x = col_start8; x < col_end8; x++)
                        map[x].n = INVALID_TRAJ;
                    map += stride;
                }
            }
        }
    }

    rp_proj = &rf->rp_proj[offset];
    const int shift = rf->mfmv_k_shift, mask = ~(rf->frm_hdr->tmvp_sample_step - 1);
    for (int n = 0; n < rf->n_mfmvs; n++) {
        const int ref2cur = rf->mfmv_ref2cur[n];
        if (ref2cur == INVALID_REF2CUR) continue;

        const int ref = rf->mfmv[n].ref;
        const int tgt = rf->mfmv[n].tgt;
        const int ref_sign = rf->mfmv[n].dir;
        const refmvs_temporal_block *r = &rf->rp_ref[ref][row_start8 * stride];
        for (int y = row_start8; y < row_end8; y++) {
            for (int x = col_start8i; x < col_end8i; x++) {
                const ptrdiff_t pos = (y & (sbsz8 - 1)) * stride + x;
                const refmvs_temporal_block *rb = &r[pos];
                const int b_ref = rb->ref.ref[ref_sign];
                if (!b_ref) continue;
                const int ref2idx = rf->mfmv_ref2idx[n][b_ref - 1];
                mv b_mv = dequantize_mv(rb->mv.mv[ref_sign]);
                if (b_mv.n == INVALID_MV) continue;
                if (rf->seq_hdr->mv_traj && ref2idx != -1)
                    check_traj_intersect(rf, rp_traj, rp_map,
                                         ref, ref2idx, y, x, b_mv);
                int ref2ref = rf->mfmv_ref2ref[n][b_ref - 1];
                if (!ref2ref || (ref2ref < 0) != ref_sign) continue;
                const mv mv1 = scale_mv(b_mv, -rf->mfmv_ref2sf[n][b_ref - 1][0]);
                const int y1 = (y - apply_sign(abs(mv1.y) >> 6, mv1.y)) & mask;
                if (y1 < 0 || y1 >= rf->ih8) continue;
                const int x1 = (x - apply_sign(abs(mv1.x) >> 6, mv1.x)) & mask;
                if (x1 < 0 || x1 >= rf->iw8) continue;
                const int y_proj_start = y1 & ~(mfmv_sbsz8 - 1);
                const int y_proj_end = imin(y_proj_start + mfmv_sbsz8, row_end8);
                if (y < y_proj_start || y >= y_proj_end) continue;
                const int x_sb_align = x1 & ~(mfmv_sbsz8 - 1);
                const int x_proj_start = imax(x_sb_align - mfmv_edge, col_start8);
                const int x_proj_end =
                    imin(x_sb_align + mfmv_sbsz8 + rf->mfmv_edge, col_end8);
                if (x < x_proj_start || x >= x_proj_end) continue;
                const ptrdiff_t pos1 = (y1 & (sbsz8 - 1)) * stride + x1;
                if (rp_proj[pos1].mv.n != INVALID_MV &&
                    (tgt == -1 || ref2idx != tgt ||
                     rp_proj[pos1].ref == abs(ref2ref)))
                {
                    continue;
                }
                if (rf->seq_hdr->mv_traj) {
                    const int k = (x1 >> shift) % 3;
                    rp_traj[ref][pos1] = mv1;
                    rp_map[k][ref][pos].y = y1 - y;
                    rp_map[k][ref][pos].x = x1 - x;
                    do /* so we can "break" out of it, saves indentation */ {
                        if (ref2idx < 0) break;
                        const mv mv2 =
                            scale_mv(b_mv, rf->mfmv_ref2sf[n][b_ref - 1][1]);
                        rp_traj[ref2idx][pos1] = mv2;
                        const int y2 = (y1 + apply_sign(abs(mv2.y) >> 6,
                                                        mv2.y)) & mask;
                        if (y2 < y_proj_start || y2 >= y_proj_end) break;
                        const int x2 = (x1 + apply_sign(abs(mv2.x) >> 6,
                                                        mv2.x)) & mask;
                        if (x2 < x_proj_start || x2 >= x_proj_end) break;
                        const ptrdiff_t pos2 = (y2 & (sbsz8 - 1)) * stride + x2;
                        rp_map[k][ref2idx][pos2].y = y1 - y2;
                        rp_map[k][ref2idx][pos2].x = x1 - x2;
                    } while (0);
                }
                if (ref2ref < 0) {
                    b_mv.y = -b_mv.y;
                    b_mv.x = -b_mv.x;
                }
                if (rf->frm_hdr->tip.frame_mode) {
                    rp_proj[pos1].mv =
                        mv_projection(b_mv, rf->tip_delta, abs(ref2ref));
                } else {
                    rp_proj[pos1].mv = b_mv;
                }
                rp_proj[pos1].ref = abs(ref2ref);
            }
        }
    }

    if (!rf->frm_hdr->tip.frame_mode) return;

    if (rf->seq_hdr->tip_hole_fill) {
        fill_holes(rp_proj, stride,
                   col_start8, col_end8, row_start8, row_end8, mfmv_sbsz8, sbsz8,
                   rf->frm_hdr->tmvp_sample_step);
        smoothen(rp_proj, stride,
                 col_start8, col_end8, row_start8, row_end8, mfmv_sbsz8, sbsz8,
                 rf->frm_hdr->tmvp_sample_step);
    }
    // FIXME fill_gap()
}

static inline unsigned quantize_mv_comp(const unsigned absv) {
    assert(absv < 2048);
    if (!absv) return 0;
    const int nbits = iclip(ulog2(absv) - 4, 0, 6);
    int res = (absv - (16 * !!nbits << nbits)) >> nbits;
    res += (nbits + !!nbits) * 16;
    return res;
}

static inline union qmv quantize_mv(const union mv mv) {
    const int absy = abs(mv.y), absx = abs(mv.x);
    if (imax(absx, absy) >= 2048) return (union qmv) { .n = INVALID_TRAJ };
    return (union qmv) {
        .y = apply_sign(quantize_mv_comp(absy), mv.y),
        .x = apply_sign(quantize_mv_comp(absx), mv.x),
    };
}

static void save_tmvs_c(refmvs_temporal_block *rp, const ptrdiff_t stride,
                        const refmvs_block *const rr,
                        const refmvs_sngl_mv_block *rp_proj,
                        const int32_t tip_sf[2], const uint8_t tip_ref[2],
                        const int col_end8, const int row_end8,
                        const int col_start8, const int row_start8,
                        const uint64_t flipmask)
{
    for (int y = row_start8; y < row_end8; y++) {
        const refmvs_block *const b = &rr[((y & 31) * 2 + 1) * 128];
        for (int x = col_start8; x < col_end8; x++) {
            const refmvs_block *const cand_b = &b[((x * 2) & 127) + 1];
            const union mv *const cand_mv =
                cand_b->mf & 4 ? cand_b->lmv.mv : cand_b->mv.mv;

            // FIXME opfl, refinemv
            if (cand_b->ref.ref[0] - 1 == TIP_FRAME) {
                const union mv tmv = rp_proj[x].mv;
                const union mv tip0mv = scale_mv(tmv, tip_sf[0]);
                const union mv tip1mv = scale_mv(tmv, tip_sf[1]);
                rp[x].mv.mv[0] = quantize_mv((union mv) {
                    .y = iclip(tip0mv.y + cand_mv[0].y, -0xffff, 0xffff),
                    .x = iclip(tip0mv.x + cand_mv[0].x, -0xffff, 0xffff),
                });
                const int i = (cand_b->mf & 4) >> 2;
                rp[x].mv.mv[1] = quantize_mv((union mv) {
                    .y = iclip(tip1mv.y + cand_mv[i].y, -0xffff, 0xffff),
                    .x = iclip(tip1mv.x + cand_mv[i].x, -0xffff, 0xffff),
                });
                rp[x].ref.ref[0] = tip_ref[0] + 1;
                rp[x].ref.ref[1] = tip_ref[1] + 1;
            } else {
                if (cand_mv[0].n == INVALID_MV) {
                    if (cand_mv[1].n == INVALID_MV) {
                        rp[x].mv.n = INVALID_TRAJ * 0x10001U;
                        rp[x].ref.pair = 0;
                    } else {
                        rp[x].mv.n = quantize_mv(cand_mv[1]).n * 0x10001U;
                        rp[x].ref.pair = cand_b->ref.ref[1] * 0x101U;
                    }
                } else {
                    if (cand_mv[1].n == INVALID_MV) {
                        rp[x].mv.n = quantize_mv(cand_mv[0]).n * 0x10001U;
                        rp[x].ref.pair = cand_b->ref.ref[0] * 0x101U;
                    } else {
                        const int r0 = cand_b->ref.ref[0] - 1;
                        const int r1 = cand_b->ref.ref[1] - 1;
                        const int ridx = r0 * 8 + r1;
                        const int f = !!(flipmask & (1ULL << ridx));
                        rp[x].mv.mv[0] = quantize_mv(cand_mv[ f]);
                        rp[x].mv.mv[1] = quantize_mv(cand_mv[!f]);
                        rp[x].ref.ref[0] = cand_b->ref.ref[ f];
                        rp[x].ref.ref[1] = cand_b->ref.ref[!f];
                    }
                }
            }
        }
        rp += stride;
        rp_proj += stride;
    }
}

// cache the current tile/sbrow (or frame/sbrow)'s projectable motion vectors
// into buffers for use in future frame's temporal MV prediction
void dav1d_refmvs_save_tmvs(const Dav1dRefmvsDSPContext *const dsp,
                            refmvs_tile *const rt,
                            const int col_start8, int col_end8,
                            const int row_start8, int row_end8)
{
    const refmvs_frame *const rf = rt->rf;

    assert(row_start8 >= 0);
    assert((unsigned) (row_end8 - row_start8) <= 32U &&
           (unsigned) (col_end8 - col_start8) <= 32U);
    row_end8 = imin(row_end8, rf->ih8);
    col_end8 = imin(col_end8, rf->iw8);

    const ptrdiff_t stride = rf->rp_stride;
    refmvs_temporal_block *rp =
        (rf->seq_hdr->ref_frame_mvs && IS_INTER_OR_SWITCH(rf->frm_hdr)) ?
        &rf->rp[row_start8 * stride] : NULL;

    if (rp)
        dsp->save_tmvs(rp, stride, rt->r, rt->rp_proj,
                       rf->tip_sf, rf->frm_hdr->tip.refs,
                       col_end8, row_end8, col_start8, row_start8,
                       rf->ref_flip);

    // keep a backup of top (at 8x8 resolution) for next sbrow
    const refmvs_block *const b = &rt->r[(((row_end8 - 1) & 31) * 2 + 1) * 128];
    rt->ra_tl = rt->ra[col_end8 - 1];
    for (int x = col_start8; x < col_end8; x++) {
        const refmvs_block *const cand_b = &b[((x * 2) & 127) + 0];
        rt->ra[x] = *cand_b;
    }
}

static unsigned abs_closest_ref(const int8_t *const ref2ref,
                                const int8_t *const cur2ref, const int dir)
{
    int b = 0xff;
    for (int n = 0; n < 7; n++) {
        const int a = abs(ref2ref[n]);
        if (((cur2ref[n] > 0 && ref2ref[n] > 0 && dir) ||
             (cur2ref[n] < 0 && ref2ref[n] < 0 && !dir)) && a < b)
        {
            b = a;
        }
    }
    return b;
}

static int topo_insert(int cnt, const int idx, int8_t *const order,
                       int8_t *const rev_order, const int8_t (*const cnv)[7])
{
    if (rev_order[idx] != -1) return cnt;
    rev_order[idx] = 0; // dummy
    for (int n = 0; n < 7; n++) {
        const int r_idx = cnv[idx][n];
        if (r_idx == -1) continue;
        cnt = topo_insert(cnt, r_idx, order, rev_order, cnv);
    }
    order[cnt] = idx;
    rev_order[idx] = cnt;
    return cnt + 1;
}

int dav1d_refmvs_init_frame(refmvs_frame *const rf,
                            const Dav1dSequenceHeader *const seq_hdr,
                            const Dav1dFrameHeader *const frm_hdr,
                            const uint8_t ref_poc[7],
                            refmvs_temporal_block *const rp,
                            const uint8_t ref_ref_poc[7][7],
                            /*const*/ refmvs_temporal_block *const rp_ref[7],
                            const int n_tile_threads, const int n_frame_threads)
{
    const int rp_stride = ((frm_hdr->width + 255) & ~255) >> 3;
    const int n_tile_rows = n_tile_threads > 1 ? frm_hdr->tiling.t.rows : 1;
    const int n_blocks = rp_stride * n_tile_rows;

    rf->sbsz = 16 << frm_hdr->sb128;
    const int mfmv_sb128 = frm_hdr->sb128 && frm_hdr->tmvp_sample_step > 1;
    rf->mfmv_k_shift = 3 + mfmv_sb128;
    rf->mfmv_sbsz8 = 8 << mfmv_sb128;
    rf->mfmv_edge = rf->mfmv_sbsz8 >> (frm_hdr->tmvp_sample_step == 1);
    rf->seq_hdr = seq_hdr;
    rf->frm_hdr = frm_hdr;
    rf->iw8 = (frm_hdr->width + 7) >> 3;
    rf->ih8 = (frm_hdr->height + 7) >> 3;
    rf->iw4 = rf->iw8 << 1;
    rf->ih4 = rf->ih8 << 1;
    rf->rp = rp;
    rf->rp_stride = rp_stride;
    rf->n_tile_threads = n_tile_threads;
#if 0
    rf->n_frame_threads = n_frame_threads;
#endif
    if (n_blocks * rf->sbsz > rf->n_blocks) {
        const int sbsz8 = rf->sbsz >> 1;
        const size_t rp_proj_sz = sizeof(*rf->rp_proj) * sbsz8 * n_blocks;
        const size_t rp_traj_sz = sizeof(mv) * sbsz8 * n_blocks;
        const size_t rp_map_sz = sizeof(**rf->rp_map) * sbsz8 * n_blocks;
        const size_t r_above_sz = sizeof(*rf->ra) * n_blocks;
        dav1d_free_aligned(rf->rp_proj);
        uint8_t *mem =
            dav1d_alloc_aligned(ALLOC_REFMVS, 7 * 3 * rp_map_sz +
                                rp_proj_sz + 7 * rp_traj_sz + r_above_sz, 64);
        if (!mem) {
            rf->rp_proj = NULL;
            rf->n_blocks = 0;
            return DAV1D_ERR(ENOMEM);
        }
        rf->rp_proj = (refmvs_sngl_mv_block *) mem;
        mem += rp_proj_sz;
        for (int n = 0; n < 7; n++) {
            rf->rp_traj[n] = (mv *) mem;
            mem += rp_traj_sz;
        }
        for (int n = 0; n < 3; n++)
            for (int m = 0; m < 7; m++) {
                rf->rp_map[n][m] = (refmvs_traj_map *) mem;
                mem += rp_map_sz;
            }
        rf->ra = (refmvs_block *) mem;
        rf->n_blocks = n_blocks * rf->sbsz;
    }
    const int poc = frm_hdr->frame_offset;
    int8_t ref2ref[7][7], ref2cur[7][7], refref2curref_idx[7][7];
    for (int i = 0; i < frm_hdr->n_ref_frames; i++) {
        const int poc_diff = get_poc_diff(seq_hdr->order_hint_n_bits,
                                          ref_poc[i], poc);
        rf->ref_sign[i] = poc_diff < 0;
        rf->pocdiff[i] = iclip(get_poc_diff(seq_hdr->order_hint_n_bits,
                                            poc, ref_poc[i]), -31, 31);
        rf->abspocdiff[i] = abs(rf->pocdiff[i]);
        for (int n = 0; n < 7; n++) {
            ref2ref[i][n] = get_poc_diff(seq_hdr->order_hint_n_bits,
                                         ref_poc[i], ref_ref_poc[i][n]);
            ref2cur[i][n] = get_poc_diff(seq_hdr->order_hint_n_bits,
                                         poc, ref_ref_poc[i][n]);
            int m;
            for (m = 0; m < frm_hdr->n_ref_frames; m++)
                if (ref_ref_poc[i][n] == ref_poc[m]) break;
            refref2curref_idx[i][n] = m == frm_hdr->n_ref_frames ? -1 : m;
        }
    }
    uint64_t flipmask = 0;
    for (int i = 0; i < frm_hdr->n_ref_frames; i++) {
        for (int n = 0; n < frm_hdr->n_ref_frames; n++) {
            const int flip = rf->ref_sign[i] == rf->ref_sign[n] ?
                             get_poc_diff(seq_hdr->order_hint_n_bits,
                                          ref_poc[i], ref_poc[n]) < 0 :
                             rf->ref_sign[n];
            flipmask |= ((uint64_t) flip) << (i * 8 + n);
        }
    }
    rf->ref_flip = flipmask;

    // tip setup
    if (rf->frm_hdr->tip.frame_mode) {
        const unsigned tip0poc = ref_poc[frm_hdr->tip.refs[0]];
        const unsigned tip1poc = ref_poc[frm_hdr->tip.refs[1]];
        const int d2 = get_poc_diff(seq_hdr->order_hint_n_bits,
                                    tip1poc, tip0poc);
        rf->tip_delta = d2;
        const int d1 = rf->pocdiff[frm_hdr->tip.refs[0]];
        const int dv = div_mult[imin(abs(d2), 31)];
        rf->tip_sf[0] = imin(abs(d1), 31) * dv;
        if ((d1 < 0) ^ (d2 < 0)) rf->tip_sf[0] *= -1;
        const int d3 = rf->pocdiff[frm_hdr->tip.refs[1]];
        rf->tip_sf[1] = imin(abs(d3), 31) * dv;
        if ((d3 < 0) ^ (d2 < 0)) rf->tip_sf[1] *= -1;
    }

    // temporal MV setup
    rf->n_mfmvs = 0;
    rf->rp_ref = rp_ref;
    if (frm_hdr->use_ref_frame_mvs && seq_hdr->order_hint_n_bits) {
        // sort refs
        uint8_t order[7];
        for (int n = 0; n < frm_hdr->n_ref_frames; n++) {
            const int pocdiff = rf->pocdiff[n];
            int m;
            for (m = n; m > 0 && pocdiff > rf->pocdiff[order[m - 1]]; m--)
                order[m] = order[m - 1];
            order[m] = n;
        }
        // find bwd/fwd ref split point
        int first_fut;
        for (first_fut = 0; first_fut < frm_hdr->n_ref_frames &&
             rf->ref_sign[order[first_fut]]; first_fut++)
        {
            /* empty */
        }
        // dependency ordering
        int8_t topo_order[7], rev_topo_order[7];
        memset(rev_topo_order, -1, sizeof(rev_topo_order));
        int topo_cnt = 0;
        for (int n = 0; n < frm_hdr->n_ref_frames; n++)
            topo_cnt = topo_insert(topo_cnt, n, topo_order, rev_topo_order,
                                   refref2curref_idx);
        if (topo_cnt <= 1) goto end;

        uint8_t ref_done[7][2] = {{ 0 }};
        // mark keyframes as done since they don't have MVs
        for (int n = 0; n < frm_hdr->n_ref_frames; n++)
            if (!rp_ref[n]) ref_done[n][0] = ref_done[n][1] = 1;

        if (seq_hdr->tip) {
            const int o = rev_topo_order[frm_hdr->tip.refs[0]] >
                              rev_topo_order[frm_hdr->tip.refs[1]];
            rf->mfmv[rf->n_mfmvs++] = (struct MfmvRef) {
                .ref = frm_hdr->tip.refs[!o],
                .tgt = frm_hdr->tip.refs[o],
                .dir = o,
            };
            ref_done[frm_hdr->tip.refs[!o]][o] = 1;
        }
        // adjacent refs
        for (int n = 0; n < 2; n++) {
            const int ref1 = first_fut - n > 0 ? order[first_fut - n - 1] : -1;
            const int ref2 = first_fut + n < frm_hdr->n_ref_frames ?
                             order[first_fut + n] : -1;
            int order = 0;
            if (ref1 >= 0 && ref2 >= 0) {
                uint8_t acr1 = abs_closest_ref(ref2ref[ref1], ref2cur[ref1], 0);
                uint8_t acr2 = abs_closest_ref(ref2ref[ref2], ref2cur[ref2], 1);
                order = acr1 < acr2;
            }
            if (order && !ref_done[ref1][1]) {
                assert(ref1 >= 0);
                rf->mfmv[rf->n_mfmvs++] = (struct MfmvRef) {
                    .ref = ref1,
                    .tgt = -1,
                    .dir = 1,
                };
                ref_done[ref1][1] = 1;
                if (rf->n_mfmvs == 3) break;
            }
            if (ref2 >= 0 && !ref_done[ref2][0]) {
                rf->mfmv[rf->n_mfmvs++] = (struct MfmvRef) {
                    .ref = ref2,
                    .tgt = -1,
                    .dir = 0,
                };
                ref_done[ref2][0] = 1;
                if (rf->n_mfmvs == 3) break;
            }
            if (!order && ref1 >= 0 && !ref_done[ref1][1]) {
                rf->mfmv[rf->n_mfmvs++] = (struct MfmvRef) {
                    .ref = ref1,
                    .tgt = -1,
                    .dir = 1,
                };
                ref_done[ref1][1] = 1;
                if (rf->n_mfmvs == 3) break;
            }
        }
        // bwd adjacent refs, opposite direction
        if (rf->n_mfmvs < 3 && first_fut > 0) {
            const int ref = order[first_fut - 1];
            if (!ref_done[ref][0]) {
                rf->mfmv[rf->n_mfmvs++] = (struct MfmvRef) {
                    .ref = ref,
                    .tgt = -1,
                    .dir = 0,
                };
                ref_done[ref][0] = 1;
            }
            if (rf->n_mfmvs < 3 && first_fut > 1) {
                const int ref2 = order[first_fut - 2];
                if (!ref_done[ref2][0]) {
                    rf->mfmv[rf->n_mfmvs++] = (struct MfmvRef) {
                        .ref = ref2,
                        .tgt = -1,
                        .dir = 1,
                    };
                    ref_done[ref2][0] = 1;
                }
            }
        }
        for (int n = topo_cnt - 1; n >= 0; n--) {
            const int ref = topo_order[n];
            const int dir = rf->pocdiff[ref] < 0;
            if (!ref_done[ref][dir]) {
                rf->mfmv[rf->n_mfmvs++] = (struct MfmvRef) {
                    .ref = ref,
                    .tgt = -1,
                    .dir = dir,
                };
                ref_done[ref][dir] = 1;
                if (rf->n_mfmvs == 4) break;
            }
            if (!ref_done[ref][!dir]) {
                rf->mfmv[rf->n_mfmvs++] = (struct MfmvRef) {
                    .ref = ref,
                    .tgt = -1,
                    .dir = !dir,
                };
                ref_done[ref][!dir] = 1;
                if (rf->n_mfmvs == 4) break;
            }
        }

        for (int n = 0; n < rf->n_mfmvs; n++) {
            const int rpoc = ref_poc[rf->mfmv[n].ref];
            const int diff1 = get_poc_diff(seq_hdr->order_hint_n_bits,
                                           rpoc, frm_hdr->frame_offset);
            if (abs(diff1) > 31) {
                rf->mfmv_ref2cur[n] = INVALID_REF2CUR;
            } else {
                rf->mfmv_ref2cur[n] = rf->mfmv[n].ref < 4 ? -diff1 : diff1;
                for (int m = 0; m < 7; m++) {
                    const int rrpoc = ref_ref_poc[rf->mfmv[n].ref][m];
                    const int diff2 = get_poc_diff(seq_hdr->order_hint_n_bits,
                                                   rpoc, rrpoc);
                    rf->mfmv_ref2ref[n][m] = diff2 + 31U < 63U ? diff2 : 0;
                    int l;
                    for (l = 0; l < 7; l++)
                        if (rrpoc == ref_poc[l])
                            break;
                    rf->mfmv_ref2idx[n][m] = l == 7 ? -1 : l;
                    const int d1 = -rf->mfmv_ref2cur[n];
                    const int d2 = rf->mfmv_ref2ref[n][m];
                    const int dv = div_mult[imin(abs(d2), 31)];
                    rf->mfmv_ref2sf[n][m][0] = imin(abs(d1), 31) * dv;
                    if ((d1 < 0) ^ (d2 < 0)) rf->mfmv_ref2sf[n][m][0] *= -1;
                    const int d3 = d1 - d2;
                    rf->mfmv_ref2sf[n][m][1] = imin(abs(d3), 31) * dv;
                    if ((d3 < 0) ^ (d2 > 0)) rf->mfmv_ref2sf[n][m][1] *= -1;
                }
            }
        }
    }
end:
    rf->use_ref_frame_mvs = rf->n_mfmvs > 0;

    return 0;
}

static void splat_mv_c(refmvs_block *r, refmvs_block *const rmv,
                       const int bw4, int bh4)
{
    do {
        for (int x = 0; x < bw4; x++) {
            memcpy(&r[x], rmv, offsetof(refmvs_block, lmv));
        }
        r += 128;
    } while (--bh4);
}

static void splat_warpmv_c(refmvs_block *r,
                           refmvs_block *const rmv, int64_t mvy, int64_t mvx,
                           const Dav1dWarpedMotionParams *const mat,
                           const int bw4, int bh4)
{
    assert(bw4 > 1);
    rmv->lmv = rmv->mv;
    memcpy(rmv->m, mat->matrix, sizeof(int32_t) * 6);
    rmv->m[6] = mat->type;
    do {
        int64_t mvxi = mvx, mvyi = mvy;
        for (int x = 0; x < bw4; x += 2) {
            rmv->mv.mv[0].y = iclip(apply_sign64((llabs(mvyi) + 4096) >> 13, mvyi),
                                    -0xffff, 0xffff);
            rmv->mv.mv[0].x = iclip(apply_sign64((llabs(mvxi) + 4096) >> 13, mvxi),
                                    -0xffff, 0xffff);
            r[x] = r[x + 1] = *rmv;
            if (bh4 > 1) {
                r[x + 128] = r[x + 128 + 1] = *rmv;
            }
            mvxi += (mat->matrix[2] - 0x10000) * 8;
            mvyi += mat->matrix[4] * 8;
        }
        mvx += mat->matrix[3] * 8;
        mvy += (mat->matrix[5] - 0x10000) * 8;
        r += 2 * 128;
        bh4 -= 2;
    } while (bh4);
}

static void splat_comp_warpmv_c(refmvs_block *r,
                                refmvs_block *const rmv, int64_t mvy1, int64_t mvx1,
                                int64_t mvy2, int64_t mvx2,
                                const Dav1dWarpedMotionParams *const mat,
                                const int bw4, int bh4)
{
    assert(bw4 > 1);
    rmv->lmv = rmv->mv;
    // FIXME for compound-warp_causal-newmv^2, do we need a 2nd matrix?
    memcpy(rmv->m, mat->matrix, sizeof(int32_t) * 6);
    rmv->m[6] = mat->type;
    do {
        int64_t mvxi1 = mvx1, mvyi1 = mvy1, mvxi2 = mvx2, mvyi2 = mvy2;
        for (int x = 0; x < bw4; x += 2) {
            rmv->mv.mv[0].y = iclip(apply_sign64((llabs(mvyi1) + 4096) >> 13, mvyi1),
                                    -0xffff, 0xffff);
            rmv->mv.mv[0].x = iclip(apply_sign64((llabs(mvxi1) + 4096) >> 13, mvxi1),
                                    -0xffff, 0xffff);
            rmv->mv.mv[1].y = iclip(apply_sign64((llabs(mvyi2) + 4096) >> 13, mvyi2),
                                    -0xffff, 0xffff);
            rmv->mv.mv[1].x = iclip(apply_sign64((llabs(mvxi2) + 4096) >> 13, mvxi2),
                                    -0xffff, 0xffff);
            r[x] = r[x + 1] = *rmv;
            if (bh4 > 1) {
                r[x + 128] = r[x + 128 + 1] = *rmv;
            }
            mvxi1 += (mat[0].matrix[2] - 0x10000) * 8;
            mvyi1 += mat[0].matrix[4] * 8;
            mvxi2 += (mat[1].matrix[2] - 0x10000) * 8;
            mvyi2 += mat[1].matrix[4] * 8;
        }
        mvx1 += mat[0].matrix[3] * 8;
        mvy1 += (mat[0].matrix[5] - 0x10000) * 8;
        mvx2 += mat[1].matrix[3] * 8;
        mvy2 += (mat[1].matrix[5] - 0x10000) * 8;
        r += 2 * 128;
        bh4 -= 2;
    } while (bh4);
}

#if HAVE_ASM
#if ARCH_AARCH64 || ARCH_ARM
#include "src/arm/refmvs.h"
#elif ARCH_LOONGARCH64
#include "src/loongarch/refmvs.h"
#elif ARCH_X86
#include "src/x86/refmvs.h"
#endif
#endif

COLD void dav1d_refmvs_dsp_init(Dav1dRefmvsDSPContext *const c)
{
    c->save_tmvs = save_tmvs_c;
    c->splat_mv = splat_mv_c;
    c->splat_warpmv = splat_warpmv_c;
    c->splat_comp_warpmv = splat_comp_warpmv_c;

#if HAVE_ASM && 0
#if ARCH_AARCH64 || ARCH_ARM
    refmvs_dsp_init_arm(c);
#elif ARCH_LOONGARCH64
    refmvs_dsp_init_loongarch(c);
#elif ARCH_X86
    refmvs_dsp_init_x86(c);
#endif
#endif
}
