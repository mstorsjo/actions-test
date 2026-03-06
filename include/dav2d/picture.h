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

#ifndef DAV2D_PICTURE_H
#define DAV2D_PICTURE_H

#include <stddef.h>
#include <stdint.h>

#include "common.h"
#include "headers.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Number of bytes to align AND pad picture memory buffers by, so that SIMD
 * implementations can over-read by a few bytes, and use aligned read/write
 * instructions. */
#define DAV2D_PICTURE_ALIGNMENT 64

typedef struct Dav2dPictureParameters {
    int w; ///< width (in pixels)
    int h; ///< height (in pixels)
    enum Dav2dPixelLayout layout; ///< format of the picture
    int bpc; ///< bits per pixel component (8 or 10)
} Dav2dPictureParameters;

typedef struct Dav2dPicture {
    Dav2dSequenceHeader *seq_hdr;
    Dav2dFrameHeader *frame_hdr;

    /**
     * Pointers to planar image data (Y is [0], U is [1], V is [2]). The data
     * should be bytes (for 8 bpc) or words (for 10 bpc). In case of words
     * containing 10 bpc image data, the pixels should be located in the LSB
     * bits, so that values range between [0, 1023]; the upper bits should be
     * zero'ed out.
     */
    void *data[3];

    /**
     * Number of bytes between 2 lines in data[] for luma [0] or chroma [1].
     */
    ptrdiff_t stride[2];

    Dav2dPictureParameters p;
    Dav2dDataProps m;

    /**
     * High Dynamic Range Content Light Level metadata applying to this picture,
     * as defined in section 5.8.3 and 6.7.3
     */
    Dav2dContentLightLevel *content_light;
    /**
     * High Dynamic Range Mastering Display Color Volume metadata applying to
     * this picture, as defined in section 5.8.4 and 6.7.4
     */
    Dav2dMasteringDisplay *mastering_display;
    /**
     * Array of ITU-T T.35 metadata as defined in section 5.8.2 and 6.7.2
     */
    Dav2dITUTT35 *itut_t35;
    Dav2dFilmGrainData *fgm;

    /**
     * Number of ITU-T T35 metadata entries in the array
     */
    size_t n_itut_t35;

    uintptr_t reserved[4]; ///< reserved for future use

    struct Dav2dRef *frame_hdr_ref; ///< Dav2dFrameHeader allocation origin
    struct Dav2dRef *seq_hdr_ref; ///< Dav2dSequenceHeader allocation origin
    struct Dav2dRef *content_light_ref; ///< Dav2dContentLightLevel allocation origin
    struct Dav2dRef *mastering_display_ref; ///< Dav2dMasteringDisplay allocation origin
    struct Dav2dRef *itut_t35_ref; ///< Dav2dITUTT35 allocation origin
    struct Dav2dRef *fgm_ref;
    uintptr_t reserved_ref[4]; ///< reserved for future use
    struct Dav2dRef *ref; ///< Frame data allocation origin

    void *allocator_data; ///< pointer managed by the allocator
} Dav2dPicture;

typedef struct Dav2dPicAllocator {
    void *cookie; ///< custom data to pass to the allocator callbacks.
    /**
     * Allocate the picture buffer based on the Dav2dPictureParameters.
     *
     * The data[0], data[1] and data[2] must be DAV2D_PICTURE_ALIGNMENT byte
     * aligned and with a pixel width/height multiple of 128 pixels. Any
     * allocated memory area should also be padded by DAV2D_PICTURE_ALIGNMENT
     * bytes.
     * data[1] and data[2] must share the same stride[1].
     *
     * This function will be called on the main thread (the thread which calls
     * dav2d_get_picture()).
     *
     * @param  pic The picture to allocate the buffer for. The callback needs to
     *             fill the picture data[0], data[1], data[2], stride[0] and
     *             stride[1].
     *             The allocator can fill the pic allocator_data pointer with
     *             a custom pointer that will be passed to
     *             release_picture_callback().
     * @param cookie Custom pointer passed to all calls.
     *
     * @note No fields other than data, stride and allocator_data must be filled
     *       by this callback.
     * @return 0 on success. A negative DAV2D_ERR value on error.
     */
    int (*alloc_picture_callback)(Dav2dPicture *pic, void *cookie);
    /**
     * Release the picture buffer.
     *
     * If frame threading is used, this function may be called by the main
     * thread (the thread which calls dav2d_get_picture()) or any of the frame
     * threads and thus must be thread-safe. If frame threading is not used,
     * this function will only be called on the main thread.
     *
     * @param pic    The picture that was filled by alloc_picture_callback().
     * @param cookie Custom pointer passed to all calls.
     */
    void (*release_picture_callback)(Dav2dPicture *pic, void *cookie);
} Dav2dPicAllocator;

/**
 * Release reference to a picture.
 */
DAV2D_API void dav2d_picture_unref(Dav2dPicture *p);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DAV2D_PICTURE_H */
