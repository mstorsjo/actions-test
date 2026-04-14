; Copyright © 2026, VideoLAN and dav2d authors
; Copyright © 2026, Two Orioles, LLC
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

SECTION_RODATA

adst4_mat:      dw  18,  75,  50,  18,  75, -89;  89,  50
flipadst4_mat:  dw  89,  50,  75, -89,  50,  18,  18,  75

pw_4096:        times 2 dw 4096
pd_64:          dd 64
pd_512:         dd 512

pw_64_64:       dw  64,  64
pw_64_m64:      dw  64, -64
pw_35_83:       dw  35,  83
pw_m83_35:      dw -83,  35

%define o_base (adst4_mat + 128)
%define o(x) (r6 - o_base + (x))
%define m(x) mangle(private_prefix %+ _ %+ x %+ _avx2)

%macro ITX_JMP_TABLE 2 ; w, h
    %if %1 <= 16
        itx_%1x%2_table_p1:
        dd m(inv_txfm_add_%1x%2_8bpc).pass1_dct - o_base
        dd m(inv_txfm_add_%1x%2_8bpc).pass1_identity - o_base
        dd m(inv_txfm_add_%1x%2_8bpc).pass1_adst - o_base
        dd m(inv_txfm_add_%1x%2_8bpc).pass1_flipadst - o_base
        %if %1 >= 8
            dd m(inv_txfm_add_%1x%2_8bpc).pass1_ddt - o_base
            dd m(inv_txfm_add_%1x%2_8bpc).pass1_flipddt - o_base
        %elif %2 == 4 ; wht
            pw_17: times 2 dw 17 ; There are two unused entries, make use
            pw_33: times 2 dw 33 ; of the space to store some constants
            dd m(inv_txfm_add_%1x%2_8bpc).wht - o_base
        %endif
    %endif
    %if %2 <= 16
        itx_%1x%2_table_p2:
        dd m(inv_txfm_add_%1x%2_8bpc).pass2_dct - o_base
        dd m(inv_txfm_add_%1x%2_8bpc).pass2_identity - o_base
        dd m(inv_txfm_add_%1x%2_8bpc).pass2_adst - o_base
        dd m(inv_txfm_add_%1x%2_8bpc).pass2_flipadst - o_base
        %if %2 >= 8
            dd m(inv_txfm_add_%1x%2_8bpc).pass2_ddt - o_base
            dd m(inv_txfm_add_%1x%2_8bpc).pass2_flipddt - o_base
        %endif
    %endif
%endmacro

ITX_JMP_TABLE  4,  4

SECTION .text

%macro INV_TXFM_FN 2 ; w, h
cglobal inv_txfm_add_%1x%2_8bpc, 4, 7, 0, dst, ds, cf, tx1, eob, tx2
    lea                  r6, [o_base]
%if WIN64
    rorx               eobd, eobm, 16
%else
    shl                eobd, 16
%endif
    add                eobd, tx1d
    jz .dconly
%if %1 < 32 && %2 < 32
    movzx              tx2d, tx1b
    and                tx1d, 7
    shr                tx2d, 5
    movsxd             tx1q, [o(itx_%1x%2_table_p1)+tx1q*4]
    movsxd             tx2q, [o(itx_%1x%2_table_p2)+tx2q*4]
    add                tx1q, r6
    add                tx2q, r6
%elif %1 < 32
    and                tx1d, 7
    movsxd             tx1q, [o(itx_%1x%2_table_p1)+tx1q*4]
    add                tx1q, r6
%elif %2 < 32
    movzx              tx2d, tx1b
    shr                tx2d, 5
    movsxd             tx2q, [o(itx_%1x%2_table_p2)+tx2q*4]
    add                tx2q, r6
%endif
%endmacro

%macro IDCT4_1D_PACKED 1 ; rnd
    vpbroadcastd         m3, [o(pw_64_64)]
    punpckhwd            m4, m1, m0 ; 3 1
    vpbroadcastd         m2, [o(pw_64_m64)]
    punpcklwd            m0, m1     ; 0 2
    vpbroadcastd         m1, [o(pw_35_83)]
    pmaddwd              m3, m0     ; a0
    pmaddwd              m2, m0     ; a1
    vpbroadcastd         m0, [o(pw_m83_35)]
    pmaddwd              m1, m4     ; b0
    pmaddwd              m4, m0     ; b1
    paddd                m3, %1
    paddd                m2, %1
    paddd                m0, m3, m1 ; out0
    psubd                m3, m1     ; out3
    paddd                m1, m2, m4 ; out1
    psubd                m2, m4     ; out2
    ret
%endmacro

%macro IDST4_1D_PACKED 0
%if WIN64 && xmm_regs_used <= 6 ; avoid xmm spills
    vpbroadcastd         m2, [r3+4*0]
    punpcklwd            m4, m0, m1
    vpbroadcastd         m3, [r3+4*3]
    punpckhwd            m5, m1, m0
    pmaddwd              m0, m2, m4
    pmaddwd              m1, m3, m5
    pmaddwd              m3, m4
    pmaddwd              m2, m5
    paddd                m0, m1
    vpbroadcastd         m1, [r3+4*1]
    psubd                m3, m2
    vpbroadcastd         m2, [r3+4*2]
    pmaddwd              m1, m4
    pmaddwd              m2, m4
    vpbroadcastd         m4, [r3+4*2]
    pmaddwd              m4, m5
    psubd                m1, m4
    vpbroadcastd         m4, [r3+4*1]
    pmaddwd              m4, m5
    paddd                m2, m4
%else
    vpbroadcastd         m4, [r3+4*0]
    punpcklwd            m3, m0, m1
    vpbroadcastd         m5, [r3+4*1]
    punpckhwd            m8, m1, m0
    vpbroadcastd         m6, [r3+4*2]
    pmaddwd              m0, m4, m3
    vpbroadcastd         m7, [r3+4*3]
    pmaddwd              m1, m5, m3
    pmaddwd              m2, m6, m3
    pmaddwd              m3, m7
    REPX    {pmaddwd x, m8}, m7, m6, m5, m4
    paddd                m0, m7
    psubd                m1, m6
    paddd                m2, m5
    psubd                m3, m4
%endif
    ret
%endmacro

%macro IWHT4_1D_PACKED 0
    punpckhqdq           m3, m0, m1 ; in1 in3
    punpcklqdq           m0, m1     ; in0 in2
    psubw                m2, m0, m3
    paddw                m0, m3
    punpckhqdq           m2, m2     ; t2 t2
    punpcklqdq           m0, m0     ; t0 t0
    psubw                m1, m0, m2
    psraw                m1, 1
    psubw                m1, m3     ; t1 t3
    psubw                m0, m1     ; ____ out0
    paddw                m2, m1     ; out3 ____
%endmacro

INIT_XMM avx2
INV_TXFM_FN 4, 4
    mova                 m0, [cfq+16*0]
    mova                 m1, [cfq+16*1]
    jmp                tx1q

.dconly:
    vpbroadcastw         m3, [cfq]
.dconly2:
    vpbroadcastd         m2, [o(pw_17)]
    paddsw               m3, m2
    psraw                m3, 5
    pxor                 m2, m2
    lea                  r3, [dsq*3]
    mova              [cfq], m2
.dconly_loop:
    movd                 m0, [dstq+dsq*0]
    pinsrd               m0, [dstq+dsq*1], 1
    movd                 m1, [dstq+dsq*2]
    pinsrd               m1, [dstq+r3   ], 1
    punpcklbw            m0, m2
    punpcklbw            m1, m2
    paddw                m0, m3
    paddw                m1, m3
    packuswb             m0, m1
    movd       [dstq+dsq*0], m0
    pextrd     [dstq+dsq*1], m0, 1
    pextrd     [dstq+dsq*2], m0, 2
    pextrd     [dstq+r3   ], m0, 3
    lea                dstq, [dstq+dsq*4]
    dec                 r4d
    jg .dconly_loop
    ret

.pass1_dct:
    vpbroadcastd         m5, [o(pd_64)]
    call .dct4
.pass1_end:
    REPX       {psrad x, 7}, m0, m1, m2, m3
    packssdw             m0, m1
    packssdw             m1, m2, m3
.pass1_identity:
    punpckhwd            m2, m0, m1
    punpcklwd            m0, m1
    punpckhwd            m1, m0, m2
    punpcklwd            m0, m2
    jmp                tx2q
.pass2_dct:
    vpbroadcastd         m5, [o(pd_512)]
    call .dct4
    REPX      {psrad x, 10}, m0, m1, m2, m3
    packssdw             m0, m1
    packssdw             m1, m2, m3
    jmp .pass2_end
ALIGN function_align
.dct4:
    IDCT4_1D_PACKED      m5

.wht:
    psraw                m0, 3
    psraw                m1, 3
    IWHT4_1D_PACKED
    punpckhwd            m0, m1
    punpcklwd            m3, m1, m2
    punpckhdq            m1, m0, m3
    punpckldq            m0, m3
    IWHT4_1D_PACKED
    punpckhqdq           m0, m1
    punpcklqdq           m1, m2
    jmp .pass2_identity_end

.pass2_identity:
    vpbroadcastd         m2, [o(pw_4096)]
    pmulhrsw             m0, m2
    pmulhrsw             m1, m2
.pass2_identity_end:
    test               eobd, 0x300
    jz .pass2_end
    test               eobd, 0x200
    jnz .vdpcm
.hdpcm:
    psllq                m2, m0, 16
    psllq                m3, m1, 16
    paddw                m0, m2
    paddw                m1, m3
    psllq                m2, m0, 32
    psllq                m3, m1, 32
    jmp .dpcm_end
.vdpcm:
    pslldq               m2, m0, 8
    paddw                m1, m0
    shufpd               m3, m0, m1, 0x01
.dpcm_end:
    paddw                m0, m2
    paddw                m1, m3
.pass2_end:
    lea                  r6, [dstq+dsq*2]
    movd                 m2, [dstq+dsq*0]
    pinsrd               m2, [dstq+dsq*1], 1
    movd                 m3, [r6  +dsq*0]
    pinsrd               m3, [r6  +dsq*1], 1
    pxor                 m4, m4
    mova         [cfq+16*0], m4
    mova         [cfq+16*1], m4
    punpcklbw            m2, m4
    punpcklbw            m3, m4
    paddw                m0, m2
    paddw                m1, m3
    packuswb             m0, m1
    movd       [dstq+dsq*0], m0
    pextrd     [dstq+dsq*1], m0, 1
    pextrd     [r6  +dsq*0], m0, 2
    pextrd     [r6  +dsq*1], m0, 3
    RET

.pass1_flipadst:
    lea                  r3, [o(flipadst4_mat)]
    jmp .pass1_dst
.pass1_adst:
    lea                  r3, [o(adst4_mat)]
.pass1_dst:
    call .dst4
    vpbroadcastd         m4, [o(pd_64)]
    REPX      {paddd x, m4}, m0, m1, m2, m3
    jmp .pass1_end
.pass2_flipadst:
    lea                  r3, [o(flipadst4_mat)]
    jmp .pass2_dst
.pass2_adst:
    lea                  r3, [o(adst4_mat)]
.pass2_dst:
    call .dst4
    vpbroadcastd         m4, [o(pw_4096)]
    REPX       {psrad x, 7}, m0, m1, m2, m3
    packssdw             m0, m1
    packssdw             m1, m2, m3
    pmulhrsw             m0, m4
    pmulhrsw             m1, m4
    jmp .pass2_end
ALIGN function_align
.dst4:
    IDST4_1D_PACKED
