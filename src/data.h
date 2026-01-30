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

#ifndef DAV2D_SRC_DATA_H
#define DAV2D_SRC_DATA_H

#include "dav2d/data.h"

void dav2d_data_ref(Dav2dData *dst, const Dav2dData *src);

/**
 * Copy the source properties to the destination and increase the
 * user_data's reference count (if it's not NULL).
 */
void dav2d_data_props_copy(Dav2dDataProps *dst, const Dav2dDataProps *src);

void dav2d_data_props_set_defaults(Dav2dDataProps *props);

uint8_t *dav2d_data_create_internal(Dav2dData *buf, size_t sz);
int dav2d_data_wrap_internal(Dav2dData *buf, const uint8_t *ptr, size_t sz,
                             void (*free_callback)(const uint8_t *data,
                                                   void *user_data),
                             void *user_data);
int dav2d_data_wrap_user_data_internal(Dav2dData *buf,
                                       const uint8_t *user_data,
                                       void (*free_callback)(const uint8_t *user_data,
                                                             void *cookie),
                                       void *cookie);
void dav2d_data_unref_internal(Dav2dData *buf);
void dav2d_data_props_unref_internal(Dav2dDataProps *props);

#endif /* DAV2D_SRC_DATA_H */
