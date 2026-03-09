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

#include "config.h"
#include "vcs_version.h"

#include <errno.h>
#include <string.h>

#if defined(__linux__) && HAVE_DLSYM
#include <dlfcn.h>
#endif

#include "dav2d/dav2d.h"
#include "dav2d/data.h"

#include "common/validate.h"

#include "src/cpu.h"
#include "src/fg_apply.h"
#include "src/ibp.h"
#include "src/internal.h"
#include "src/log.h"
#include "src/obu.h"
#include "src/qm.h"
#include "src/ref.h"
#include "src/thread_task.h"
#include "src/wedge.h"

static COLD void init_internal(void) {
    dav2d_init_cpu();
    dav2d_init_ibp_weights();
    dav2d_init_ii_wedge_masks();
    dav2d_init_qm_tables();
    dav2d_init_thread();
}

COLD const char *dav2d_version(void) {
    return DAV2D_VERSION;
}

COLD unsigned dav2d_version_api(void) {
    return (DAV2D_API_VERSION_MAJOR << 16) |
           (DAV2D_API_VERSION_MINOR <<  8) |
           (DAV2D_API_VERSION_PATCH <<  0);
}

COLD void dav2d_default_settings(Dav2dSettings *const s) {
    s->n_threads = 0;
    s->max_frame_delay = 0;
    s->apply_grain = 1;
    s->allocator.cookie = NULL;
    s->allocator.alloc_picture_callback = dav2d_default_picture_alloc;
    s->allocator.release_picture_callback = dav2d_default_picture_release;
    s->logger.cookie = NULL;
    s->logger.callback = dav2d_log_default_callback;
    s->operating_point = 0;
    s->all_layers = 1; // just until the tests are adjusted
    s->frame_size_limit = 0;
    s->strict_std_compliance = 0;
    s->output_invisible_frames = 0;
    s->inloop_filters = DAV2D_INLOOPFILTER_ALL;
    s->decode_frame_type = DAV2D_DECODEFRAMETYPE_ALL;
}

static void close_internal(Dav2dContext **const c_out, int flush);

NO_SANITIZE("cfi-icall") // CFI is broken with dlsym()
static COLD size_t get_stack_size_internal(const pthread_attr_t *const thread_attr) {
#if defined(__linux__) && HAVE_DLSYM && defined(__GLIBC__)
    /* glibc has an issue where the size of the TLS is subtracted from the stack
     * size instead of allocated separately. As a result the specified stack
     * size may be insufficient when used in an application with large amounts
     * of TLS data. The following is a workaround to compensate for that.
     * See https://sourceware.org/bugzilla/show_bug.cgi?id=11787 */
    size_t (*const get_minstack)(const pthread_attr_t*) =
        dlsym(RTLD_DEFAULT, "__pthread_get_minstack");
    if (get_minstack)
        return get_minstack(thread_attr) - PTHREAD_STACK_MIN;
#endif
    return 0;
}

static COLD void get_num_threads(Dav2dContext *const c, const Dav2dSettings *const s,
                                 unsigned *n_tc, unsigned *n_fc)
{
#if 0
    /* ceil(sqrt(n)) */
    static const uint8_t fc_lut[49] = {
        1,                                     /*     1 */
        2, 2, 2,                               /*  2- 4 */
        3, 3, 3, 3, 3,                         /*  5- 9 */
        4, 4, 4, 4, 4, 4, 4,                   /* 10-16 */
        5, 5, 5, 5, 5, 5, 5, 5, 5,             /* 17-25 */
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,       /* 26-36 */
        7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, /* 37-49 */
    };
    *n_tc = s->n_threads ? s->n_threads :
        iclip(dav2d_num_logical_processors(c), 1, DAV2D_MAX_THREADS);
    *n_fc = s->max_frame_delay ? umin(s->max_frame_delay, *n_tc) :
            *n_tc < 50 ? fc_lut[*n_tc - 1] : 8; // min(8, ceil(sqrt(n)))
#endif
    // FIXME re-enable threading
    *n_tc = *n_fc = 1;
}

COLD int dav2d_get_frame_delay(const Dav2dSettings *const s) {
    unsigned n_tc, n_fc;
    validate_input_or_ret(s != NULL, DAV2D_ERR(EINVAL));
    validate_input_or_ret(s->n_threads >= 0 &&
                          s->n_threads <= DAV2D_MAX_THREADS, DAV2D_ERR(EINVAL));
    validate_input_or_ret(s->max_frame_delay >= 0 &&
                          s->max_frame_delay <= DAV2D_MAX_FRAME_DELAY, DAV2D_ERR(EINVAL));

    get_num_threads(NULL, s, &n_tc, &n_fc);
    return n_fc;
}

COLD int dav2d_open(Dav2dContext **const c_out, const Dav2dSettings *const s) {
    static pthread_once_t initted = PTHREAD_ONCE_INIT;
    pthread_once(&initted, init_internal);

    validate_input_or_ret(c_out != NULL, DAV2D_ERR(EINVAL));
    validate_input_or_ret(s != NULL, DAV2D_ERR(EINVAL));
    validate_input_or_ret(s->n_threads >= 0 &&
                          s->n_threads <= DAV2D_MAX_THREADS, DAV2D_ERR(EINVAL));
    validate_input_or_ret(s->max_frame_delay >= 0 &&
                          s->max_frame_delay <= DAV2D_MAX_FRAME_DELAY, DAV2D_ERR(EINVAL));
    validate_input_or_ret(s->allocator.alloc_picture_callback != NULL,
                          DAV2D_ERR(EINVAL));
    validate_input_or_ret(s->allocator.release_picture_callback != NULL,
                          DAV2D_ERR(EINVAL));
    validate_input_or_ret(s->operating_point >= 0 &&
                          s->operating_point <= 31, DAV2D_ERR(EINVAL));
    validate_input_or_ret(s->decode_frame_type >= DAV2D_DECODEFRAMETYPE_ALL &&
                          s->decode_frame_type <= DAV2D_DECODEFRAMETYPE_KEY, DAV2D_ERR(EINVAL));

    pthread_attr_t thread_attr;
    if (pthread_attr_init(&thread_attr)) return DAV2D_ERR(ENOMEM);
    size_t stack_size = 1024 * 1024 + get_stack_size_internal(&thread_attr);

    pthread_attr_setstacksize(&thread_attr, stack_size);

    Dav2dContext *const c = *c_out = dav2d_alloc_aligned(ALLOC_COMMON_CTX, sizeof(*c), 64);
    if (!c) goto error;
    memset(c, 0, sizeof(*c));

    c->allocator = s->allocator;
    c->logger = s->logger;
    c->apply_grain = s->apply_grain;
    c->operating_point = s->operating_point;
    c->all_layers = s->all_layers;
    c->frame_size_limit = s->frame_size_limit;
    c->strict_std_compliance = s->strict_std_compliance;
    c->output_invisible_frames = s->output_invisible_frames;
    c->inloop_filters = s->inloop_filters;
    c->decode_frame_type = s->decode_frame_type;

#if 0
    dav2d_data_props_set_defaults(&c->cached_error_props);
#endif

    if (dav2d_mem_pool_init(ALLOC_OBU_HDR, &c->seq_hdr_pool) ||
        dav2d_mem_pool_init(ALLOC_OBU_HDR, &c->frame_hdr_pool) ||
        dav2d_mem_pool_init(ALLOC_SEGMAP, &c->segmap_pool) ||
        dav2d_mem_pool_init(ALLOC_REFMVS, &c->refmvs_pool) ||
        dav2d_mem_pool_init(ALLOC_CCSOMAP, &c->ccsomap_pool) ||
        dav2d_mem_pool_init(ALLOC_PIC_CTX, &c->pic_ctx_pool) ||
        dav2d_mem_pool_init(ALLOC_CDF, &c->cdf_pool) ||
        dav2d_mem_pool_init(ALLOC_CDF, &c->fgm_pool))
    {
        goto error;
    }

    if (c->allocator.alloc_picture_callback   == dav2d_default_picture_alloc &&
        c->allocator.release_picture_callback == dav2d_default_picture_release)
    {
        if (c->allocator.cookie) goto error;
        if (dav2d_mem_pool_init(ALLOC_PIC, &c->picture_pool)) goto error;
        c->allocator.cookie = c->picture_pool;
    } else if (c->allocator.alloc_picture_callback   == dav2d_default_picture_alloc ||
               c->allocator.release_picture_callback == dav2d_default_picture_release)
    {
        goto error;
    }

    /* On 32-bit systems extremely large frame sizes can cause overflows in
     * dav2d_decode_frame() malloc size calculations. Prevent that from occuring
     * by enforcing a maximum frame size limit, chosen to roughly correspond to
     * the largest size possible to decode without exhausting virtual memory. */
    if (sizeof(size_t) < 8 && s->frame_size_limit - 1 >= 8192 * 8192) {
        c->frame_size_limit = 8192 * 8192;
        if (s->frame_size_limit)
            dav2d_log(c, "Frame size limit reduced from %u to %u.\n",
                      s->frame_size_limit, c->frame_size_limit);
    }

    c->flush = &c->flush_mem;
    atomic_init(c->flush, 0);

    get_num_threads(c, s, &c->n_tc, &c->n_fc);

    c->fc = dav2d_alloc_aligned(ALLOC_THREAD_CTX, sizeof(*c->fc) * c->n_fc, 32);
    if (!c->fc) goto error;
    memset(c->fc, 0, sizeof(*c->fc) * c->n_fc);

    c->tc = dav2d_alloc_aligned(ALLOC_THREAD_CTX, sizeof(*c->tc) * c->n_tc, 64);
    if (!c->tc) goto error;
    memset(c->tc, 0, sizeof(*c->tc) * c->n_tc);
    if (c->n_tc > 1) {
        if (pthread_mutex_init(&c->task_thread.lock, NULL)) goto error;
        if (pthread_cond_init(&c->task_thread.cond, NULL)) {
            pthread_mutex_destroy(&c->task_thread.lock);
            goto error;
        }
        if (pthread_cond_init(&c->task_thread.delayed_fg.cond, NULL)) {
            pthread_cond_destroy(&c->task_thread.cond);
            pthread_mutex_destroy(&c->task_thread.lock);
            goto error;
        }
        c->task_thread.cur = c->n_fc;
        atomic_init(&c->task_thread.reset_task_cur, UINT_MAX);
        atomic_init(&c->task_thread.cond_signaled, 0);
        c->task_thread.inited = 1;
    }

#if 0
    if (c->n_fc > 1) {
        const size_t out_delayed_sz = sizeof(*c->frame_thread.out_delayed) * c->n_fc;
        c->frame_thread.out_delayed =
            dav2d_malloc(ALLOC_THREAD_CTX, out_delayed_sz);
        if (!c->frame_thread.out_delayed) goto error;
        memset(c->frame_thread.out_delayed, 0, out_delayed_sz);
    }
#endif
    c->dpb_sz = c->n_fc + 16;
    c->dpb = dav2d_malloc(ALLOC_THREAD_CTX, sizeof(*c->dpb) * c->dpb_sz);
    if (!c->dpb) goto error;
    memset(c->dpb, 0, sizeof(*c->dpb) * c->dpb_sz);
    for (unsigned n = 0; n < c->n_fc; n++) {
        Dav2dFrameContext *const f = &c->fc[n];
        if (c->n_tc > 1) {
            if (pthread_mutex_init(&f->task_thread.lock, NULL)) goto error;
            if (pthread_cond_init(&f->task_thread.cond, NULL)) {
                pthread_mutex_destroy(&f->task_thread.lock);
                goto error;
            }
            if (pthread_mutex_init(&f->task_thread.pending_tasks.lock, NULL)) {
                pthread_cond_destroy(&f->task_thread.cond);
                pthread_mutex_destroy(&f->task_thread.lock);
                goto error;
            }
        }
        f->c = c;
        f->task_thread.ttd = &c->task_thread;
        f->refdir_intra = -1;
        f->refdir[TIP_FRAME] = 1;
    }

    for (unsigned m = 0; m < c->n_tc; m++) {
        Dav2dTaskContext *const t = &c->tc[m];
        t->f = &c->fc[0];
        t->task_thread.ttd = &c->task_thread;
        t->c = c;
        memset(t->cf_y_16bpc, 0, sizeof(t->cf_y_16bpc));
        memset(t->cf_uv_16bpc, 0, sizeof(t->cf_uv_16bpc));
        if (c->n_tc > 1) {
            if (pthread_mutex_init(&t->task_thread.td.lock, NULL)) goto error;
            if (pthread_cond_init(&t->task_thread.td.cond, NULL)) {
                pthread_mutex_destroy(&t->task_thread.td.lock);
                goto error;
            }
            if (pthread_create(&t->task_thread.td.thread, &thread_attr, dav2d_worker_task, t)) {
                pthread_cond_destroy(&t->task_thread.td.cond);
                pthread_mutex_destroy(&t->task_thread.td.lock);
                goto error;
            }
            t->task_thread.td.inited = 1;
        }
    }
    dav2d_pal_dsp_init(&c->pal_dsp);
    dav2d_refmvs_dsp_init(&c->refmvs_dsp);

    pthread_attr_destroy(&thread_attr);

    return 0;

error:
    if (c) close_internal(c_out, 0);
    pthread_attr_destroy(&thread_attr);
    return DAV2D_ERR(ENOMEM);
}

static struct OutputQueue *queue_append(Dav2dContext *const c,
                                        Dav2dThreadPicture *const p)
{
    struct OutputQueue *const q = &c->dpb[c->dpb_in++];
    dav2d_thread_picture_ref(&q->p, p);
    q->res = 0;
    if (c->dpb_in == c->dpb_sz) c->dpb_in = 0;
    assert(c->dpb_in != c->dpb_out);
    assert(!c->dpb_poc || c->dpb_poc != p->p.frame_hdr->frame_offset);
    c->dpb_poc = p->p.frame_hdr->frame_offset;
    return q;
}

static void queue_flush(Dav2dContext *const c) {
    if (!c->seq_hdr) return;

    const int nb = c->seq_hdr->order_hint_n_bits;
    int mask = 0;
    for (;;) {
        int cand_n = -1, cand_poc;
        for (int n = 0, m = 1; n < 8; n++, m <<= 1) {
            if (mask & m) continue;
            if (!c->refs[n].p.p.data[0]) continue;
            const Dav2dFrameHeader *const hdr = c->refs[n].p.p.frame_hdr;
            assert(hdr);
            if (hdr->show_frame || !hdr->showable_frame) continue;
            const int ipoc = hdr->frame_offset;
            if (get_poc_diff(nb, ipoc, c->dpb_poc) > 0 &&
                (cand_n == -1 || get_poc_diff(nb, ipoc, cand_poc) < 0))
            {
                cand_n = n;
                cand_poc = ipoc;
            }
        }
        if (cand_n == -1) break;
        queue_append(c, &c->refs[cand_n].p);
        mask |= 1 << cand_n;
    }
}

struct OutputQueue *dav2d_queue_output(Dav2dContext *const c,
                                       Dav2dThreadPicture *const p)
{
    assert(c->seq_hdr);

    if (c->output_invisible_frames) return queue_append(c, p);

    // FIXME the remainder of this code is not multi-layer-compatible yet
    const int nb = c->seq_hdr->order_hint_n_bits;
    const int poc = p->p.frame_hdr->frame_offset;
    unsigned mask = 0;

    for (;;) {
        int cand_n = -1, cand_poc = poc;
        for (int n = 0, m = 1; n < 8; n++, m <<= 1) {
            if (mask & m) continue;
            if (!c->refs[n].p.p.data[0]) continue;
            const Dav2dFrameHeader *const hdr = c->refs[n].p.p.frame_hdr;
            assert(hdr);
            if (hdr->show_frame || !hdr->showable_frame) continue;
            const int ipoc = hdr->frame_offset;
            if (get_poc_diff(nb, ipoc, c->dpb_poc) > 0 &&
                get_poc_diff(nb, ipoc, cand_poc) < 0)
            {
                cand_n = n;
                cand_poc = ipoc;
            }
        }
        if (cand_n == -1) break;
        queue_append(c, &c->refs[cand_n].p);
        mask |= 1 << cand_n;
    }

    struct OutputQueue *const q = queue_append(c, p);

    // immediately-adjacent future refs after the trigger frame
    for (;;) {
        int n, m;
        for (n = 0, m = 1; n < 8; n++, m <<= 1) {
            if (mask & m) continue;
            if (!c->refs[n].p.p.data[0]) continue;
            const Dav2dFrameHeader *const hdr = c->refs[n].p.p.frame_hdr;
            assert(hdr);
            if (hdr->show_frame || !hdr->showable_frame) continue;
            const int ipoc = hdr->frame_offset;
            if (get_poc_diff(nb, ipoc, c->dpb_poc) == 1) break;
        }
        if (n == 8) break;
        queue_append(c, &c->refs[n].p);
        mask |= 1 << n;
    }

    return q;
}

static int has_grain(const Dav2dPicture *const pic) {
    const Dav2dFilmGrainData *fgdata = pic->fgm;
    return fgdata &&
           (fgdata->num_points[0] || fgdata->num_points[1] ||
            fgdata->num_points[2] || (fgdata->clip_to_restricted_range &&
                                      fgdata->chroma_scaling_from_luma));
}

static int output_image(Dav2dContext *const c, Dav2dPicture *const out) {
    if (c->dpb_in == c->dpb_out) {
        if (!c->drain) return DAV2D_ERR(EAGAIN);
        c->drain = 0;
        return DAV2D_EOF;
    }
    struct OutputQueue *const q = &c->dpb[c->dpb_out++];
    if (c->dpb_out == c->dpb_sz) c->dpb_out = 0;

    const int res = dav2d_apply_grain(c, out, &q->p.p);
    dav2d_thread_picture_unref(&q->p);
    return res;
}

static int output_picture_ready(Dav2dContext *const c) {
    return c->dpb_out != c->dpb_in;
}

static int gen_picture(Dav2dContext *const c) {
    Dav2dData *const in = &c->in;

    if (output_picture_ready(c))
        return 0;

    while (in->sz > 0) {
        const ptrdiff_t res = dav2d_parse_obus(c, in);
        if (res < 0) {
            dav2d_data_unref_internal(in);
        } else {
            assert((size_t)res <= in->sz);
            in->sz -= res;
            in->data += res;
            if (!in->sz) dav2d_data_unref_internal(in);
        }
        if (output_picture_ready(c))
            break;
        if (res < 0)
            return (int)res;
    }

    return 0;
}

int dav2d_send_data(Dav2dContext *const c, Dav2dData *const in) {
    validate_input_or_ret(c != NULL, DAV2D_ERR(EINVAL));

    if (!in) {
        c->drain = 1;
        return 0;
    } else if (c->drain) {
        return DAV2D_EOF;
    }

    validate_input_or_ret(in->sz > 0 && in->sz <= SIZE_MAX / 2, DAV2D_ERR(EINVAL));

    if (c->in.data)
        return DAV2D_ERR(EAGAIN);
    dav2d_data_ref(&c->in, in);
    dav2d_data_unref(in);

    return 0;
}

int dav2d_get_picture(Dav2dContext *const c, Dav2dPicture *const out) {
    validate_input_or_ret(c != NULL, DAV2D_ERR(EINVAL));
    validate_input_or_ret(out != NULL, DAV2D_ERR(EINVAL));

    int res = gen_picture(c);
    if (res < 0)
        return res;

    if (c->drain)
        queue_flush(c);

    return output_image(c, out);
}

int dav2d_apply_grain(Dav2dContext *const c, Dav2dPicture *const out,
                      const Dav2dPicture *const in)
{
    validate_input_or_ret(c != NULL, DAV2D_ERR(EINVAL));
    validate_input_or_ret(out != NULL, DAV2D_ERR(EINVAL));
    validate_input_or_ret(in != NULL, DAV2D_ERR(EINVAL));

    if (!has_grain(in) || !c->apply_grain) {
        dav2d_picture_ref(out, in);
        return 0;
    }

    int res = dav2d_picture_alloc_copy(c, out, in);
    if (res < 0) goto error;

    if (c->n_tc > 1) {
        dav2d_task_delayed_fg(c, out, in);
    } else {
        switch (out->p.bpc) {
#if CONFIG_8BPC
        case 8:
            dav2d_apply_grain_8bpc(&c->dsp[0].fg, out, in);
            break;
#endif
#if CONFIG_16BPC
        case 10:
        case 12:
            dav2d_apply_grain_16bpc(&c->dsp[(out->p.bpc >> 1) - 4].fg, out, in);
            break;
#endif
        default: abort();
        }
    }

    return 0;

error:
    dav2d_picture_unref_internal(out);
    return res;
}

void dav2d_flush(Dav2dContext *const c) {
    dav2d_data_unref_internal(&c->in);
    for (int n = 0; n < c->dpb_sz; n++)
        if (c->dpb[n].p.p.data[0])
            dav2d_thread_picture_unref(&c->dpb[n].p);
    c->dpb_in = c->dpb_out = c->drain = 0;

    for (int i = 0; i < 8; i++) {
        if (c->refs[i].p.p.frame_hdr)
            dav2d_thread_picture_unref(&c->refs[i].p);
        dav2d_ref_dec(&c->refs[i].segmap);
        dav2d_ref_dec(&c->refs[i].refmvs);
        dav2d_cdf_thread_unref(&c->cdf[i]);
        dav2d_ref_dec(&c->fgm[i]);
    }
    c->frame_hdr = NULL;
    c->seq_hdr = NULL;
    dav2d_ref_dec(&c->seq_hdr_ref);

    c->mastering_display = NULL;
    c->content_light = NULL;
    c->itut_t35 = NULL;
    c->n_itut_t35 = 0;
    dav2d_ref_dec(&c->mastering_display_ref);
    dav2d_ref_dec(&c->content_light_ref);
    dav2d_ref_dec(&c->itut_t35_ref);

    if (c->n_fc == 1 && c->n_tc == 1) return;
    atomic_store(c->flush, 1);

    if (c->n_tc > 1) {
        pthread_mutex_lock(&c->task_thread.lock);
        // stop running tasks in worker threads
        for (unsigned i = 0; i < c->n_tc; i++) {
            Dav2dTaskContext *const tc = &c->tc[i];
            while (!tc->task_thread.flushed) {
                pthread_cond_wait(&tc->task_thread.td.cond, &c->task_thread.lock);
            }
        }
        for (unsigned i = 0; i < c->n_fc; i++) {
            c->fc[i].task_thread.task_head = NULL;
            c->fc[i].task_thread.task_tail = NULL;
            c->fc[i].task_thread.task_cur_prev = NULL;
            c->fc[i].task_thread.pending_tasks.head = NULL;
            c->fc[i].task_thread.pending_tasks.tail = NULL;
            atomic_init(&c->fc[i].task_thread.pending_tasks.merge, 0);
        }
        atomic_init(&c->task_thread.first, 0);
        c->task_thread.cur = c->n_fc;
        atomic_store(&c->task_thread.reset_task_cur, UINT_MAX);
        atomic_store(&c->task_thread.cond_signaled, 0);
        pthread_mutex_unlock(&c->task_thread.lock);
    }

#if 0
    if (c->n_fc > 1) {
        for (unsigned n = 0, next = c->frame_thread.next; n < c->n_fc; n++, next++) {
            if (next == c->n_fc) next = 0;
            Dav2dFrameContext *const f = &c->fc[next];
            dav2d_decode_frame_exit(f, -1);
            f->n_tile_data = 0;
            f->task_thread.retval = 0;
            f->task_thread.error = 0;
            Dav2dThreadPicture *out_delayed = &c->frame_thread.out_delayed[next];
            if (out_delayed->p.frame_hdr) {
                dav2d_thread_picture_unref(out_delayed);
            }
        }
        c->frame_thread.next = 0;
    }
#endif
    atomic_store(c->flush, 0);
}

COLD void dav2d_close(Dav2dContext **const c_out) {
    validate_input(c_out != NULL);
#if TRACK_HEAP_ALLOCATIONS
    dav2d_log_alloc_stats(*c_out);
#endif
    close_internal(c_out, 1);
}

static COLD void close_internal(Dav2dContext **const c_out, int flush) {
    Dav2dContext *const c = *c_out;
    if (!c) return;

    if (flush) dav2d_flush(c);

    if (c->tc) {
        struct TaskThreadData *ttd = &c->task_thread;
        if (ttd->inited) {
            pthread_mutex_lock(&ttd->lock);
            for (unsigned n = 0; n < c->n_tc && c->tc[n].task_thread.td.inited; n++)
                c->tc[n].task_thread.die = 1;
            pthread_cond_broadcast(&ttd->cond);
            pthread_mutex_unlock(&ttd->lock);
            for (unsigned n = 0; n < c->n_tc; n++) {
                Dav2dTaskContext *const pf = &c->tc[n];
                if (!pf->task_thread.td.inited) break;
                pthread_join(pf->task_thread.td.thread, NULL);
                pthread_cond_destroy(&pf->task_thread.td.cond);
                pthread_mutex_destroy(&pf->task_thread.td.lock);
            }
            pthread_cond_destroy(&ttd->delayed_fg.cond);
            pthread_cond_destroy(&ttd->cond);
            pthread_mutex_destroy(&ttd->lock);
        }
        dav2d_free_aligned(c->tc);
    }

    for (unsigned n = 0; c->fc && n < c->n_fc; n++) {
        Dav2dFrameContext *const f = &c->fc[n];

        // clean-up threading stuff
        if (c->n_fc > 1) {
            dav2d_free(f->tile_thread.lowest_pixel_mem);
            dav2d_free(f->frame_thread.b);
            dav2d_free_aligned(f->frame_thread.cbi);
            dav2d_free_aligned(f->frame_thread.pal_idx);
            dav2d_free_aligned(f->frame_thread.cf);
            dav2d_free(f->frame_thread.tile_start_off);
            dav2d_free_aligned(f->frame_thread.pal);
        }
        if (c->n_tc > 1) {
            pthread_mutex_destroy(&f->task_thread.pending_tasks.lock);
            pthread_cond_destroy(&f->task_thread.cond);
            pthread_mutex_destroy(&f->task_thread.lock);
        }
        dav2d_free(f->frame_thread.frame_progress);
        dav2d_free(f->task_thread.tasks);
        dav2d_free(f->task_thread.tile_tasks[0]);
        dav2d_free_aligned(f->ts);
        dav2d_free_aligned(f->ipred_edge[0]);
        dav2d_free(f->a);
        dav2d_free(f->tile);
        dav2d_free(f->lf.mask);
        dav2d_free(f->lf.lr_mask);
        dav2d_free(f->lf.tx_db_right_edge[0]);
        dav2d_free(f->lf.start_of_tile_row);
        dav2d_free_aligned(f->rf.rp_proj);
        dav2d_free_aligned(f->lf.cdef_line_buf);
        dav2d_free_aligned(f->lf.lr_line_buf);
    }
    dav2d_free_aligned(c->fc);
#if 0
    if (c->n_fc > 1 && c->frame_thread.out_delayed) {
        for (unsigned n = 0; n < c->n_fc; n++)
            if (c->frame_thread.out_delayed[n].p.frame_hdr)
                dav2d_thread_picture_unref(&c->frame_thread.out_delayed[n]);
        dav2d_free(c->frame_thread.out_delayed);
    }
#endif
    for (int n = 0; n < c->n_tile_data; n++)
        dav2d_data_unref_internal(&c->tile[n].data);
    dav2d_free(c->tile);
    for (int n = 0; n < 8; n++) {
        dav2d_cdf_thread_unref(&c->cdf[n]);
        if (c->refs[n].p.p.frame_hdr)
            dav2d_thread_picture_unref(&c->refs[n].p);
        dav2d_ref_dec(&c->refs[n].refmvs);
        dav2d_ref_dec(&c->refs[n].segmap);
    }
    dav2d_ref_dec(&c->seq_hdr_ref);
    dav2d_ref_dec(&c->frame_hdr_ref);

    dav2d_ref_dec(&c->mastering_display_ref);
    dav2d_ref_dec(&c->content_light_ref);
    dav2d_ref_dec(&c->itut_t35_ref);

    dav2d_mem_pool_end(c->seq_hdr_pool);
    dav2d_mem_pool_end(c->frame_hdr_pool);
    dav2d_mem_pool_end(c->segmap_pool);
    dav2d_mem_pool_end(c->refmvs_pool);
    dav2d_mem_pool_end(c->ccsomap_pool);
    dav2d_mem_pool_end(c->cdf_pool);
    dav2d_mem_pool_end(c->fgm_pool);
    dav2d_mem_pool_end(c->picture_pool);
    dav2d_mem_pool_end(c->pic_ctx_pool);

    dav2d_freep_aligned(c_out);
}

#if 0
int dav2d_get_event_flags(Dav2dContext *const c, enum Dav2dEventFlags *const flags) {
    validate_input_or_ret(c != NULL, DAV2D_ERR(EINVAL));
    validate_input_or_ret(flags != NULL, DAV2D_ERR(EINVAL));

    *flags = c->event_flags;
    c->event_flags = 0;
    return 0;
}

int dav2d_get_decode_error_data_props(Dav2dContext *const c, Dav2dDataProps *const out) {
    validate_input_or_ret(c != NULL, DAV2D_ERR(EINVAL));
    validate_input_or_ret(out != NULL, DAV2D_ERR(EINVAL));

    dav2d_data_props_unref_internal(out);
    *out = c->cached_error_props;
    dav2d_data_props_set_defaults(&c->cached_error_props);

    return 0;
}
#endif

void dav2d_picture_unref(Dav2dPicture *const p) {
    dav2d_picture_unref_internal(p);
}

uint8_t *dav2d_data_create(Dav2dData *const buf, const size_t sz) {
    return dav2d_data_create_internal(buf, sz);
}

int dav2d_data_wrap(Dav2dData *const buf, const uint8_t *const ptr,
                    const size_t sz,
                    void (*const free_callback)(const uint8_t *data,
                                                void *user_data),
                    void *const user_data)
{
    return dav2d_data_wrap_internal(buf, ptr, sz, free_callback, user_data);
}

int dav2d_data_wrap_user_data(Dav2dData *const buf,
                              const uint8_t *const user_data,
                              void (*const free_callback)(const uint8_t *user_data,
                                                          void *cookie),
                              void *const cookie)
{
    return dav2d_data_wrap_user_data_internal(buf,
                                              user_data,
                                              free_callback,
                                              cookie);
}

void dav2d_data_unref(Dav2dData *const buf) {
    dav2d_data_unref_internal(buf);
}

void dav2d_data_props_unref(Dav2dDataProps *const props) {
    dav2d_data_props_unref_internal(props);
}
