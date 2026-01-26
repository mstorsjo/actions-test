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

#ifndef DAV2D_H
#define DAV2D_H

#include <errno.h>
#include <stdarg.h>

#include "common.h"
#include "picture.h"
#include "data.h"
#include "version.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Dav2dContext Dav2dContext;
typedef struct Dav2dRef Dav2dRef;

#define DAV2D_MAX_THREADS 256
#define DAV2D_MAX_FRAME_DELAY 256

typedef struct Dav2dLogger {
    void *cookie; ///< Custom data to pass to the callback.
    /**
     * Logger callback. May be NULL to disable logging.
     *
     * @param cookie Custom pointer passed to all calls.
     * @param format The vprintf compatible format string.
     * @param     ap List of arguments referenced by the format string.
     */
    void (*callback)(void *cookie, const char *format, va_list ap);
} Dav2dLogger;

enum Dav2dInloopFilterType {
    DAV2D_INLOOPFILTER_DEBLOCK     = 1 << 0,
    DAV2D_INLOOPFILTER_CDEF        = 1 << 1,
    DAV2D_INLOOPFILTER_CCSO        = 1 << 2,
    DAV2D_INLOOPFILTER_RESTORATION = 1 << 3,
    DAV2D_INLOOPFILTER_ALL = DAV2D_INLOOPFILTER_DEBLOCK |
                             DAV2D_INLOOPFILTER_CDEF |
                             DAV2D_INLOOPFILTER_CCSO |
                             DAV2D_INLOOPFILTER_RESTORATION,
};

enum Dav2dDecodeFrameType {
    DAV2D_DECODEFRAMETYPE_ALL   = 0, ///< decode and return all frames
    DAV2D_DECODEFRAMETYPE_REFERENCE = 1,///< decode and return frames referenced by other frames only
    DAV2D_DECODEFRAMETYPE_INTRA = 2, ///< decode and return intra frames only (includes keyframes)
    DAV2D_DECODEFRAMETYPE_KEY   = 3, ///< decode and return keyframes only
};

typedef struct Dav2dSettings {
    int n_threads; ///< number of threads (0 = number of logical cores in host system, default 0)
    int max_frame_delay; ///< Set to 1 for low-latency decoding (0 = ceil(sqrt(n_threads)), default 0)
    int apply_grain; ///< whether to apply film grain on output frames (default 1)
    int operating_point; ///< select an operating point for scalable AV2 bitstreams (0 - 31, default 0)
    int all_layers; ///< output all spatial layers of a scalable AV2 biststream (default 1)
    unsigned frame_size_limit; ///< maximum frame size, in pixels (0 = unlimited, default 0)
    Dav2dPicAllocator allocator; ///< Picture allocator callback.
    Dav2dLogger logger; ///< Logger callback.
    int strict_std_compliance; ///< whether to abort decoding on standard compliance violations
                               ///< that don't affect actual bitstream decoding (e.g. inconsistent
                               ///< or invalid metadata, default 0)
    int output_invisible_frames; ///< output invisibly coded frames (in coding order) in addition
                                 ///< to all visible frames. Because of show-existing-frame, this
                                 ///< means some frames may appear twice (once when coded,
                                 ///< once when shown, default 0)
    enum Dav2dInloopFilterType inloop_filters; ///< postfilters to enable during decoding (default
                                               ///< DAV2D_INLOOPFILTER_ALL)
    enum Dav2dDecodeFrameType decode_frame_type; ///< frame types to decode (default
                                                 ///< DAV2D_DECODEFRAMETYPE_ALL)
    uint8_t reserved[16]; ///< reserved for future use
} Dav2dSettings;

/**
 * Get library version.
 */
DAV2D_API const char *dav2d_version(void);

/**
 * Get library API version.
 *
 * @return A value in the format 0x00XXYYZZ, where XX is the major version,
 *         YY the minor version, and ZZ the patch version.
 * @see DAV2D_API_MAJOR, DAV2D_API_MINOR, DAV2D_API_PATCH
 */
DAV2D_API unsigned dav2d_version_api(void);

/**
 * Initialize settings to default values.
 *
 * @param s Input settings context.
 */
DAV2D_API void dav2d_default_settings(Dav2dSettings *s);

/**
 * Allocate and open a decoder instance.
 *
 * @param c_out The decoder instance to open. *c_out will be set to the
 *              allocated context.
 * @param     s Input settings context.
 *
 * @note The context must be freed using dav2d_close() when decoding is
 *       finished.
 *
 * @return 0 on success, or < 0 (a negative DAV2D_ERR code) on error.
 */
DAV2D_API int dav2d_open(Dav2dContext **c_out, const Dav2dSettings *s);

/**
 * Parse a Sequence Header OBU from bitstream data.
 *
 * @param out Output Sequence Header.
 * @param buf The data to be parser.
 * @param sz  Size of the data.
 *
 * @return
 *                  0: Success, and out is filled with the parsed Sequence Header
 *                     OBU parameters.
 *  DAV2D_ERR(ENOENT): No Sequence Header OBUs were found in the buffer.
 *  Other negative DAV2D_ERR codes: Invalid data in the buffer, invalid passed-in
 *                                  arguments, and other errors during parsing.
 *
 * @note It is safe to feed this function data containing other OBUs than a
 *       Sequence Header, as they will simply be ignored. If there is more than
 *       one Sequence Header OBU present, only the last will be returned.
 */
DAV2D_API int dav2d_parse_sequence_header(Dav2dSequenceHeader *out,
                                          const uint8_t *buf, const size_t sz);

/**
 * Feed bitstream data to the decoder, in the form of one or multiple AV2
 * Open Bitstream Units (OBUs).
 *
 * @param   c Input decoder instance.
 * @param  in Input bitstream data. On success, ownership of the reference is
 *            passed to the library. May be NULL, in which case the decoder will
 *            enter draining mode.
 *
 * @return
 *         0: Success, and the data was consumed.
 *  DAV2D_ERR(EAGAIN): The data can't be consumed. dav2d_get_picture() should
 *                     be called to get one or more frames before the function
 *                     can consume new data.
 *  DAV2D_EOF:         The decoder is in draining mode. No further data can be
 *                     consumed. Use dav2d_flush() to force out of draining
 *                     mode, or call get_picture() until it returns DAV2D_EOF.
 *  Other negative DAV2D_ERR codes: Error during decoding or because of invalid
 *                                  passed-in arguments. The reference remains
 *                                  owned by the caller.
 */
DAV2D_API int dav2d_send_data(Dav2dContext *c, Dav2dData *in);

/**
 * Return a decoded picture.
 *
 * @param   c Input decoder instance.
 * @param out Output frame. The caller assumes ownership of the returned
 *            reference.
 *
 * @return
 *         0: Success, and a frame is returned.
 *  DAV2D_ERR(EAGAIN): Not enough data to output a frame. dav2d_send_data()
 *                     should be called with new input.
 *  DAV2D_EOF:         The decoder has been drained. No more frame will be
 *                     output.
 *
 *  Other negative DAV2D_ERR codes: Error during decoding or because of invalid
 *                                  passed-in arguments.
 *
 * @code{.c}
 *  Dav2dData data = { 0 };
 *  Dav2dPicture p = { 0 };
 *  int res;
 *
 *  read_data(&data);
 *  do {
 *      res = dav2d_send_data(c, &data);
 *      // Given we fetch all available frames in the loop below,
 *      // DAV2D_ERR(EAGAIN) should not happen.
 *      assert(res != DAV2D_ERR(EGAIN));
 *      if (res < 0)
 *          free_and_abort();
 *      for (;;) {
 *          res = dav2d_get_picture(c, &p);
 *          if (res < 0) {
 *              if (res != DAV2D_ERR(EAGAIN))
 *                  free_and_abort();
 *              break;
 *          } else
 *              output_and_unref_picture(&p);
 *      // Stay in the loop until no more frames are available.
 *      }
 *  // Stay in the loop as long as there's data to consume.
 *  } while (data.sz || read_data(&data) == SUCCESS);
 *
 *  // Handle EOS by draining all buffered frames.
 *  dav2d_send_data(c, NULL);
 *  do {
 *      res = dav2d_get_picture(c, &p);
 *      if (res < 0) {
 *          if (res != DAV2D_EOF)
 *              free_and_abort();
 *      } else
 *          output_and_unref_picture(&p);
 *  } while (res == 0);
 * @endcode
 */
DAV2D_API int dav2d_get_picture(Dav2dContext *c, Dav2dPicture *out);

/**
 * Apply film grain to a previously decoded picture. If the picture contains no
 * film grain metadata, then this function merely returns a new reference.
 *
 * @param   c Input decoder instance.
 * @param out Output frame. The caller assumes ownership of the returned
 *            reference.
 * @param  in Input frame. No ownership is transferred.
 *
 * @return
 *         0: Success, and a frame is returned.
 *  Other negative DAV2D_ERR codes: Error due to lack of memory or because of
 *                                  invalid passed-in arguments.
 *
 * @note If `Dav2dSettings.apply_grain` is true, film grain was already applied
 *       by `dav2d_get_picture`, and so calling this function leads to double
 *       application of film grain. Users should only call this when needed.
 */
DAV2D_API int dav2d_apply_grain(Dav2dContext *c, Dav2dPicture *out,
                                const Dav2dPicture *in);

/**
 * Close a decoder instance and free all associated memory.
 *
 * @param c_out The decoder instance to close. *c_out will be set to NULL.
 */
DAV2D_API void dav2d_close(Dav2dContext **c_out);

/**
 * Flush all delayed frames in decoder and clear internal decoder state,
 * to be used when seeking.
 *
 * @param c Input decoder instance.
 *
 * @note Decoding will start only after a valid sequence header OBU is
 *       delivered to dav2d_send_data().
 *
 */
DAV2D_API void dav2d_flush(Dav2dContext *c);

enum Dav2dEventFlags {
    /**
     * The last returned picture contains a reference to a new Sequence Header,
     * either because it's the start of a new coded sequence, or the decoder was
     * flushed before it was generated.
     */
    DAV2D_EVENT_FLAG_NEW_SEQUENCE =       1 << 0,
    /**
     * The last returned picture contains a reference to a Sequence Header with
     * new operating parameters information for the current coded sequence.
     */
    DAV2D_EVENT_FLAG_NEW_OP_PARAMS_INFO = 1 << 1,
};

/**
 * Fetch a combination of DAV2D_EVENT_FLAG_* event flags generated by the decoding
 * process.
 *
 * @param c Input decoder instance.
 * @param flags Where to write the flags.
 *
 * @return 0 on success, or < 0 (a negative DAV2D_ERR code) on error.
 *
 * @note Calling this function will clear all the event flags currently stored in
 *       the decoder.
 *
 */
DAV2D_API int dav2d_get_event_flags(Dav2dContext *c, enum Dav2dEventFlags *flags);

/**
 * Retrieve the user-provided metadata associated with the input data packet
 * for the last decoding error reported to the user, i.e. a negative return
 * value (not EAGAIN) from dav2d_send_data() or dav2d_get_picture().
 *
 * @param   c Input decoder instance.
 * @param out Output Dav2dDataProps. On success, the caller assumes ownership of
 *            the returned reference.
 *
 * @return 0 on success, or < 0 (a negative DAV2D_ERR code) on error.
 */
DAV2D_API int dav2d_get_decode_error_data_props(Dav2dContext *c, Dav2dDataProps *out);

/**
 * Get the decoder delay, which is the number of internally buffered frames, not
 * including reference frames.
 * This value is guaranteed to be >= 1 and <= max_frame_delay.
 *
 * @param s Input settings context.
 *
 * @return Decoder frame delay on success, or < 0 (a negative DAV2D_ERR code) on
 *         error.
 *
 * @note The returned delay is valid only for a Dav2dContext initialized with the
 *       provided Dav2dSettings.
 */
DAV2D_API int dav2d_get_frame_delay(const Dav2dSettings *s);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DAV2D_H */
