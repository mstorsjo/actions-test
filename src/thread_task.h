/*
 * Copyright © 2018-2021, VideoLAN and dav2d authors
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

#ifndef DAV2D_SRC_THREAD_TASK_H
#define DAV2D_SRC_THREAD_TASK_H

#include <limits.h>

#include "src/internal.h"

#define FRAME_ERROR (UINT_MAX - 1)
#define TILE_ERROR (INT_MAX - 1)

// these functions assume the task scheduling lock is already taken
int dav2d_task_create_tile_sbrow(Dav2dFrameContext *f, int pass, int cond_signal);
void dav2d_task_frame_init(Dav2dFrameContext *f);

void dav2d_task_delayed_fg(Dav2dContext *c, Dav2dPicture *out, const Dav2dPicture *in);

void *dav2d_worker_task(void *data);

int dav2d_decode_frame_init(Dav2dFrameContext *f);
int dav2d_decode_frame_init_cdf(Dav2dFrameContext *f);
int dav2d_decode_frame_main(Dav2dFrameContext *f);
void dav2d_decode_frame_exit(Dav2dFrameContext *f, int retval);
int dav2d_decode_frame(Dav2dFrameContext *f);
int dav2d_decode_tile_sbrow(Dav2dTaskContext *t);

#endif /* DAV2D_SRC_THREAD_TASK_H */
