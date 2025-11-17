/*
 * Copyright © 2025, Niklas Haas
 * Copyright © 2018, VideoLAN and dav1d authors
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

#ifndef CHECKASM_SIGNAL_WIN64_H
#define CHECKASM_SIGNAL_WIN64_H

#include <windows.h>

#if WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP)

/* setjmp/longjmp on Windows on architectures using SEH (all except
 * x86_32) will try to use SEH to unwind the stack, which doesn't work
 * for assembly functions without unwind information. */
typedef struct {
    CONTEXT c;
    int     status;
} checkasm_jmp_buf;

  #define checkasm_save_context(ctx)                                                     \
      ((ctx)->status = 0, RtlCaptureContext(&(ctx)->c), (ctx)->status)
  #define checkasm_load_context(ctx)                                                     \
      ((ctx)->status = 1, RtlRestoreContext(&(ctx)->c, NULL))

  #define CHECKASM_WORKING_SIGNAL_HANDLER 1

#else /* !WINAPI_FAMILY_PARTITION(WINAPI_PARTITION_DESKTOP) */
typedef int checkasm_jmp_buf;
  #define checkasm_save_context(ctx) 0
  #define checkasm_load_context(ctx)

  #define CHECKASM_WORKING_SIGNAL_HANDLER 0

#endif

#endif /* CHECKASM_SIGNAL_WIN64_H */
