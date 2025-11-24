; Copyright © 2019, VideoLAN and dav1d authors
; Copyright © 2019, Two Orioles, LLC
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions are met:
;
; 1. Redistributions of source code must retain the above copyright notice, this
;    list of conditions and the following disclaimer.
;
; 2. Redistributions in binary form must reproduce the above copyright notice,
;    this list of conditions and the following disclaimer in the documentation
;    and/or other materials provided with the distribution.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
; ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
; ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

%include "config.asm"
%include "ext/x86/x86inc.asm"

SECTION_RODATA 16

pw_127: times 8 dw 127

struc msac
    .buf:        resq 1
    .end:        resq 1
    .dif:        resq 1
    .rng:        resd 1
    .cnt:        resd 1
    .update_cdf: resd 1
endstruc

cextern msac_rate
cextern msac_min_prob

%define m(x) mangle(private_prefix %+ _ %+ x %+ SUFFIX)

SECTION .text

%if WIN64
DECLARE_REG_TMP 3
%define buf rsp+stack_offset+8 ; shadow space
%else
DECLARE_REG_TMP 0
%define buf rsp-40 ; red zone
%endif

%define base rax-$$

%macro DECODE_SYMBOL_ADAPT 2 ; n, sz
cglobal msac_decode_symbol_adapt%1, 3, 7, 6, s, cdf, ns
    movd           m2, [sq+msac.rng]
    lea           rax, [$$]
    mov%2          m0, [cdfq]
    add           nsd, nsd
    movq           m3, [sq+msac.dif]
    mov           r3d, [sq+msac.update_cdf]
    pshuflw        m2, m2, q0000
    movd     [buf+12], m2
    por            m1, m0, [pw_127]
    psrlw          m2, 8
    psubusw        m1, [base+msac_min_prob-16+r2*8]
    psllw          m2, 6
%if %1 == 8
    punpcklqdq     m2, m2
%endif
    pmulhuw        m1, m2
    pshuflw        m3, m3, q3333
    pxor           m2, m2
    psllw          m1, 3
%if %1 == 8
    punpcklqdq     m3, m3
%endif
    mova     [buf+16], m1
    psubusw        m1, m3
    pcmpeqw        m1, m2 ; c >= v
    test          r3d, r3d
    jz m(msac_decode_symbol_adapt4).renorm ; !allow_update_cdf

; update_cdf:
    movzx         r3d, word [cdfq+nsq]
    pcmpeqw        m2, m2
    lea           r4d, [r3*3]
    movzx         r5d, r3b
    shr           r4d, 8  ; para * sizeof(*msac_rate)
    shr           r5d, 4  ; count >> 4
    add           r4d, r5d
    movzx         eax, byte [base+msac_rate+r4]
    cmp           nsd, 3*2
    sbb           eax, -1 ; rate + (n_symbols > 2)
    cmp           r3b, 32
    adc           r3d, 0  ; count + (count < 32)
    movd           m3, eax
    pavgw          m2, m1 ; i >= val ? -1 : 32768
    psubw          m2, m0 ; for (i = 0; i < val; i++)
    psubw          m0, m1 ;     cdf[i] += (32768 - cdf[i]) >> rate;
    psraw          m2, m3 ; for (; i < n_symbols; i++)
    paddw          m0, m2 ;     cdf[i] += ((  -1 - cdf[i]) >> rate) + 1;
    mov%2      [cdfq], m0
    mov    [cdfq+nsq], r3w
%if %1 == 8
    jmp m(msac_decode_symbol_adapt4).renorm
%else
.renorm:
    pmovmskb      eax, m1
    mov            r4, [r0+msac.dif]
    tzcnt         eax, eax
    movzx         r1d, word [buf+rax+16] ; v
    movzx         r2d, word [buf+rax+14] ; u
    shr           eax, 1
    not            r4
    sub           r2d, r1d ; rng
    shl            r1, gprsize*8-16
    add            r4, r1  ; ~dif
.renorm2:
    mov           r1d, [r0+msac.cnt]
    movifnidn      t0, r0
.renorm3:
    bsr           ecx, r2d
    xor           ecx, 15  ; d
    shl           r2d, cl
    shl            r4, cl
    mov [t0+msac.rng], r2d
    not            r4
    sub           r1d, ecx
%if 1 ; FIXME: Determine exact constraints and adjust cnt offset
    cmp           r1d, 8
    jge .end
%else
    jae .end ; no refill required
%endif

; refill:
    mov            r2, [t0+msac.buf]
    mov           rcx, [t0+msac.end]
    lea            r5, [r2+gprsize]
    cmp            r5, rcx
    ja .refill_eob
    mov            r2, [r2]
    lea           ecx, [r1+23]
    add           r1d, 16
    shr           ecx, 3   ; shift_bytes
    bswap          r2
    sub            r5, rcx
    shl           ecx, 3   ; shift_bits
    shr            r2, cl
    sub           ecx, r1d ; shift_bits - 16 - cnt
    mov           r1d, gprsize*8-16
    shl            r2, cl
    mov [t0+msac.buf], r5
    sub           r1d, ecx ; cnt + gprsize*8 - shift_bits
    xor            r4, r2
.end:
    mov [t0+msac.cnt], r1d
    mov [t0+msac.dif], r4
    RET
.refill_eob: ; avoid overreading the input buffer
    mov            r5, rcx
    mov           ecx, gprsize*8-24
    sub           ecx, r1d ; c
.refill_eob_loop:
    cmp            r2, r5
    jae .refill_eob_end    ; eob reached
    movzx         r1d, byte [r2]
    inc            r2
    shl            r1, cl
    xor            r4, r1
    sub           ecx, 8
    jge .refill_eob_loop
.refill_eob_end:
    mov           r1d, gprsize*8-24
    sub           r1d, ecx
    mov [t0+msac.buf], r2
    mov [t0+msac.dif], r4
    mov [t0+msac.cnt], r1d
    RET
%endif
%endmacro

INIT_XMM sse2
DECODE_SYMBOL_ADAPT 4, q
DECODE_SYMBOL_ADAPT 8, a

cglobal msac_decode_bool_adapt, 2, 7, 0
    movzx         eax, word [r1]
    movzx         r3d, byte [r0+msac.rng+1]
    mov            r4, [r0+msac.dif]
    mov           r2d, [r0+msac.rng]
    shr           eax, 7
    imul          eax, r3d
    shr           r3d, 1
    mov            r5, r4
    add           eax, r3d
    and           eax, ~7  ; v
    mov           r3d, eax
    shl           rax, 48  ; vw
    sub           r2d, r3d ; r - v
    sub            r4, rax ; dif - vw
    setb           al
    cmovb         r2d, r3d
    mov           r3d, [r0+msac.update_cdf]
    cmovb          r4, r5
    not            r4
    test          r3d, r3d
    jz m(msac_decode_symbol_adapt4).renorm2
    movzx         r5d, word [r1+2]
%if WIN64
    push           r7
    movifnidn      t0, r0
%endif
    lea           ecx, [r5*3]
    movzx         r7d, r5b
    shr           ecx, 8  ; para * sizeof(*msac_rate)
    shr           r7d, 4  ; count >> 4
    cmp           r5b, 32
    adc           r5d, 0
    mov        [r1+2], r5w
    lea            r5, [msac_rate]
    add           ecx, r7d
    movzx         r7d, word [r1]
    movzx         ecx, byte [r5+rcx]
    imul          r5d, eax, -32769
    add           r5d, r7d ; if (bit)
    sub           r7d, eax ;     cdf[0] -= ((cdf[0] - 32769) >> rate) + 1;
    sar           r5d, cl  ; else
    sub           r7d, r5d ;     cdf[0] -= cdf[0] >> rate;
    mov          [r1], r7w
%if WIN64
    mov           r1d, [t0+msac.cnt]
    pop            r7
    jmp m(msac_decode_symbol_adapt4).renorm3
%else
    jmp m(msac_decode_symbol_adapt4).renorm2
%endif
