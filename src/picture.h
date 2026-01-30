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

#ifndef DAV2D_SRC_PICTURE_H
#define DAV2D_SRC_PICTURE_H

#include <stdatomic.h>

#include "src/thread.h"
#include "dav2d/picture.h"

#include "src/thread_data.h"
#include "src/ref.h"

enum PlaneType {
    PLANE_TYPE_Y,
    PLANE_TYPE_UV,
    PLANE_TYPE_BLOCK,
    PLANE_TYPE_ALL,
};

enum PictureFlags {
    PICTURE_FLAG_NEW_SEQUENCE =       1 << 0,
    PICTURE_FLAG_NEW_OP_PARAMS_INFO = 1 << 1,
    PICTURE_FLAG_NEW_TEMPORAL_UNIT  = 1 << 2,
};

typedef struct Dav2dThreadPicture {
    Dav2dPicture p;
    int visible;
    // This can be set for inter frames, non-key intra frames, or for invisible
    // keyframes that have not yet been made visible using the show-existing-frame
    // mechanism.
    int showable;
    enum PictureFlags flags;
    // [0] block data (including segmentation map and motion vectors)
    // [1] pixel data
    atomic_uint *progress;
} Dav2dThreadPicture;

typedef struct Dav2dPictureBuffer {
    void *data;
    struct Dav2dPictureBuffer *next;
} Dav2dPictureBuffer;

/*
 * Allocate a picture with custom border size.
 */
int dav2d_thread_picture_alloc(Dav2dContext *c, Dav2dFrameContext *f, const int bpc);

/**
 * Allocate a picture with identical metadata to an existing picture.
 */
int dav2d_picture_alloc_copy(Dav2dContext *c, Dav2dPicture *dst,
                             const Dav2dPicture *src);

/**
 * Create a copy of a picture.
 */
void dav2d_picture_ref(Dav2dPicture *dst, const Dav2dPicture *src);
void dav2d_thread_picture_ref(Dav2dThreadPicture *dst,
                              const Dav2dThreadPicture *src);
void dav2d_thread_picture_move_ref(Dav2dThreadPicture *dst,
                                   Dav2dThreadPicture *src);
void dav2d_thread_picture_unref(Dav2dThreadPicture *p);

/**
 * Move a picture reference.
 */
void dav2d_picture_move_ref(Dav2dPicture *dst, Dav2dPicture *src);

int dav2d_default_picture_alloc(Dav2dPicture *p, void *cookie);
void dav2d_default_picture_release(Dav2dPicture *p, void *cookie);
void dav2d_picture_unref_internal(Dav2dPicture *p);

struct itut_t35_ctx_context {
    Dav2dITUTT35 *itut_t35;
    size_t n_itut_t35;
    Dav2dRef ref;
};

void dav2d_picture_free_itut_t35(const uint8_t *data, void *user_data);
void dav2d_picture_copy_props(Dav2dPicture *p,
                              Dav2dContentLightLevel *content_light, Dav2dRef *content_light_ref,
                              Dav2dMasteringDisplay *mastering_display, Dav2dRef *mastering_display_ref,
                              Dav2dITUTT35 *itut_t35, Dav2dRef *itut_t35_ref, size_t n_itut_t35,
                              const Dav2dDataProps *props);

/**
 * Get event flags from picture flags.
 */
enum Dav2dEventFlags dav2d_picture_get_event_flags(const Dav2dThreadPicture *p);

#endif /* DAV2D_SRC_PICTURE_H */
