; Copyright © 2019, VideoLAN and dav2d authors
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

SECTION_RODATA 64

unary_mul32: dd 0x8000, 0xc000, 0xe000, 0xf000, 0xf800, 0xfc00, 0xfe00, 0xff00
             dd 0xff80, 0xffc0, 0xffe0, 0xfff0, 0xfff8, 0xfffc, 0xfffe, 0xffff
unary_mul64: dq 0xffff8000, 0xffffc000, 0xffffe000, 0xfffff000, 0xfffff800, 0

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

%macro REFILL 3 ; cnt, tmp, is_early_refill
    mov            %2, [t0+msac.buf]
    mov           rcx, [t0+msac.end]
    lea            r5, [%2+8]
    cmp            r5, rcx
    ja %%refill_eob
    mov            %2, [%2]
    lea           ecx, [%1+23]
    add           %1d, 16
    shr           ecx, 3   ; shift_bytes
    bswap          %2
    sub            r5, rcx
    shl           ecx, 3   ; shift_bits
    shr            %2, cl
    sub           ecx, %1d ; shift_bits - 16 - cnt
    mov           %1d, 48
    shl            %2, cl
    mov [t0+msac.buf], r5
    sub           %1d, ecx ; cnt + 64 - shift_bits
    xor            r4, %2
%if %3
    ret
%else
.end:
    mov [t0+msac.cnt], %1d
    mov [t0+msac.dif], r4
    RET
%endif
%%refill_eob: ; avoid overreading the input buffer
    mov            r5, rcx
    mov           ecx, 40
    sub           ecx, %1d ; c
%%refill_eob_loop:
    cmp            %2, r5
    jae %%refill_eob_end   ; eob reached
    movzx         %1d, byte [%2]
    inc            %2
    shl            %1, cl
    xor            r4, %1
    sub           ecx, 8
    jge %%refill_eob_loop
%%refill_eob_end:
    mov           %1d, 40
    sub           %1d, ecx
    mov [t0+msac.buf], %2
%if %3
    ret
%else
    mov [t0+msac.dif], r4
    mov [t0+msac.cnt], %1d
    RET
%endif
%endmacro

%macro DECODE_SYMBOL_ADAPT 2 ; n, sz
cglobal msac_decode_symbol_adapt%1, 3, 7, 4, s, cdf, ns
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
    mov            r4, [sq+msac.dif]
    tzcnt         eax, eax
    movzx         r1d, word [buf+rax+16] ; v
    movzx         r2d, word [buf+rax+14] ; u
    shr           eax, 1
    not            r4
    sub           r2d, r1d ; rng
    shl            r1, gprsize*8-16
    add            r4, r1  ; ~dif
.renorm2:
    mov           r1d, [sq+msac.cnt]
    movifnidn      t0, sq
.renorm3:
    bsr           ecx, r2d
    xor           ecx, 15  ; d
    shl           r2d, cl
    shl            r4, cl
    mov [t0+msac.rng], r2d
    not            r4
    sub           r1d, ecx
    jae .end ; no refill required
.refill:
    REFILL         r1, r2, 0
%endif
%endmacro

INIT_XMM sse2
DECODE_SYMBOL_ADAPT 4, q
DECODE_SYMBOL_ADAPT 8, a

cglobal msac_decode_bool_adapt, 2, 7, 0, s, cdf
    movzx         eax, word [cdfq]
    movzx         r3d, byte [sq+msac.rng+1]
    mov            r4, [sq+msac.dif]
    mov           r2d, [sq+msac.rng]
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
    mov           r3d, [sq+msac.update_cdf]
    cmovb          r4, r5
    not            r4
    test          r3d, r3d
    jz m(msac_decode_symbol_adapt4).renorm2
    movzx         r5d, word [cdfq+2]
%if WIN64
    push           r7
    mov            t0, sq
%endif
    lea           ecx, [r5*3]
    movzx         r7d, r5b
    shr           ecx, 8  ; para * sizeof(*msac_rate)
    shr           r7d, 4  ; count >> 4
    cmp           r5b, 32
    adc           r5d, 0
    mov      [cdfq+2], r5w
    lea            r5, [msac_rate]
    add           ecx, r7d
    movzx         r7d, word [r1]
    movzx         ecx, byte [r5+rcx]
    imul          r5d, eax, -32769
    add           r5d, r7d ; if (bit)
    sub           r7d, eax ;     cdf[0] -= ((cdf[0] - 32769) >> rate) + 1;
    sar           r5d, cl  ; else
    sub           r7d, r5d ;     cdf[0] -= cdf[0] >> rate;
    mov        [cdfq], r7w
%if WIN64
    mov           r1d, [t0+msac.cnt]
    pop            r7
    jmp m(msac_decode_symbol_adapt4).renorm3
%else
    jmp m(msac_decode_symbol_adapt4).renorm2
%endif

cglobal msac_decode_bool_bypass, 1, 7, 0, s
    mov           eax, [sq+msac.rng]
    mov            r4, [sq+msac.dif]
    mov           r1d, [sq+msac.cnt]
    shl           rax, 47
    mov            r2, r4
    sub            r4, rax      ; dif - vw
    cmovb          r4, r2
    setb           al
    movifnidn      t0, sq
    lea            r4, [r4*2+1] ; dif
    sub           r1d, 1        ; cnt
    jb m(msac_decode_symbol_adapt4).refill
    mov [sq+msac.cnt], r1d
    mov [sq+msac.dif], r4
    RET

cglobal msac_decode_bools_bypass, 2, 7, 0, s, n
    mov           r2d, [sq+msac.cnt]
    mov            r4, [sq+msac.dif]
    movifnidn      t0, sq
    cmp           r2d, nd
    jae .main
    call .refill
.main:
    mov           r5d, [t0+msac.rng]
    not            r4
    sub           r2d, nd
    shl            r5, 47
    mov [t0+msac.cnt], r2d
    xor           eax, eax
    mov           ecx, nd
.loop:
    mov            r2, r4
    add            r4, r5  ; dif - vw
    cmovb          r4, r2
    adc           eax, eax ; ret = (ret << 1) + (dif < vw)
    shr            r5, 1
    dec            nd
    jg .loop
    shl            r4, cl
    not            r4
    mov [t0+msac.dif], r4
    RET
.refill:
    REFILL         r2, r6, 1

INIT_YMM avx2
cglobal msac_decode_unary_bypass6, 2, 7, 5, s, n
    vpbroadcastd   m0, [sq+msac.rng]
    psrld          m0, 1
    pmulld         m0, [unary_mul32]
    mov           r2d, [sq+msac.cnt]
    mov            r4, [sq+msac.dif]
    movifnidn      t0, sq
    cmp           r2d, nd
    jb .refill
    vpbroadcastd   m1, [t0+msac.dif+4]
    psrld          m1, 1
.main:
    mov           r5d, [t0+msac.rng]
    psubd          m1, m0
    movmskps      eax, m1
    rorx          ecx, nd, 32-5
    or            eax, ecx ; clip to max_bits (5 or 6)
.end:
    vzeroupper
.end2:
    tzcnt         eax, eax
    mov           ecx, -1
    shl           r5d, 16
    shrx          ecx, ecx, eax
    not            r4
    not           ecx
    imul           r5, rcx ; vw_sum
    xor           ecx, ecx
    add            r4, r5  ; dif - vw_sum
    cmp           eax, nd
    adc           ecx, eax ; bit = ret + (ret < max_bits)
    shlx           r4, r4, rcx
    sub           r2d, ecx
    not            r4
    mov [t0+msac.cnt], r2d
    mov [t0+msac.dif], r4
    ret ; no epilogue (vzeroupper already performed)
.refill:
    call mangle(private_prefix %+ _msac_decode_bools_bypass_sse2).refill
    movq          xm1, r4
    psrlq         xm1, 33
    vpbroadcastd   m1, xm1
    jmp .main

cglobal msac_decode_unary_bypass21, 1, 7, 5, s, n
    vpbroadcastd   m2, [sq+msac.rng]
    psrld          m2, 1
    pmuludq        m0, m2, [unary_mul64]
    pmulld         m1, m2, [unary_mul32+32*1]
    pmulld         m2, [unary_mul32+32*0]
    mov           r2d, [sq+msac.cnt]
    mov            nd, 21
    mov            r4, [sq+msac.dif]
    movifnidn      t0, sq
    cmp           r2d, nd
    jb .refill
    vpbroadcastq   m3, [t0+msac.dif]
    vpbroadcastd   m4, [t0+msac.dif+4]
    psrlq          m3, 17
    psrld          m4, 1
.main:
    mov           r5d, [t0+msac.rng]
    mov           rcx, 0xfffff8000000
    psubq          m0, m3, m0
    imul          rcx, r5 ; vw[20]
    psubd          m1, m4, m1
    movmskpd      eax, m0
    psubd          m2, m4, m2
    cmp            r4, rcx
    lea           ecx, [rax+16]
    cmovb         eax, ecx
    movmskps      ecx, m1
    shl           eax, 16
    mov            ah, cl
    movmskps      ecx, m2
    lea           eax, [rax+rcx+(1<<21)]
    jmp mangle(private_prefix %+ _msac_decode_unary_bypass6_avx2).end
.refill:
    call mangle(private_prefix %+ _msac_decode_bools_bypass_sse2).refill
    movq          xm4, r4
    psrlq         xm3, xm4, 17
    psrlq         xm4, 33
    vpbroadcastq   m3, xm3
    vpbroadcastd   m4, xm4
    jmp .main

INIT_ZMM avx512icl
cglobal msac_decode_unary_bypass21, 1, 7, 5, s, n
    vpbroadcastd   m1, [sq+msac.rng]
    pmulld         m0, m1, [unary_mul32]
    pmuludq        m1, [unary_mul64]
    mov           r2d, [sq+msac.cnt]
    mov            nd, 21
    mov            r4, [sq+msac.dif]
    movifnidn      t0, sq
    cmp           r2d, nd
    jb .refill
    vpbroadcastq   m3, [sq+msac.dif]
    vpbroadcastd   m2, [sq+msac.dif+4]
.main:
    mov           r5d, [t0+msac.rng]
    psrlq          m3, 16
    vpcmpud        k1, m0, m2, 6 ; vw > dif
    vpcmpuq        k2, m1, m3, 6
    kunpckwd       k1, k2, k1
    kmovd         eax, k1
    or            eax, 1<<21
    jmp mangle(private_prefix %+ _msac_decode_unary_bypass6_avx2).end2
.refill:
    call mangle(private_prefix %+ _msac_decode_bools_bypass_sse2).refill
    vpbroadcastq   m3, r4
    pshufd         m2, m3, q3311
    jmp .main
