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

adst8_mat:      dw  11,  54,  34,  71,  84,  79,  88,  50
                dw  28,  89,  74,  68,  17, -83, -44, -69
                dw  44,  48,  89, -41, -89,  50, -44,  81
                dw  58, -34,  76, -86,  10,   6,  88, -84
                dw  70, -87,  39,   1,  86, -59, -44,  78
                dw  79, -66, -12,  87, -35,  86, -44, -62
                dw  86,  12, -58,  38, -75, -74,  88,  40
flipadst8_mat:  dw  89,  79, -86, -70,  58,  29, -44, -14
                dw  86,  12, -58,  38, -75, -74,  88,  40
                dw  79, -66, -12,  87, -35,  86, -44, -62
                dw  70, -87,  39,   1,  86, -59, -44,  78
                dw  58, -34,  76, -86,  10,   6,  88, -84
                dw  44,  48,  89, -41, -89,  50, -44,  81
                dw  28,  89,  74,  68,  17, -83, -44, -69
                dw  11,  54,  34,  71,  84,  79,  88,  50
ddt8_mat:       dw   4,  22,   6,  57,  96,  78, 103,  56
                dw   7,  48,  14,  94,  73, -79, -17, -96
                dw  15,  85,  36,  76, -43,   7, -80,  98
                dw  33,  88,  77, -26, -69,  56,  56, -77
                dw  65,   0, 100, -73,  55, -82,  15,  54
                dw  98, -86,  45,  34,  20,  79, -66, -33
                dw 106, -23, -57,  54, -71, -56,  75,  19
flipddt8_mat:   dw  80,  82, -98, -66,  53,  26, -41,  -6
                dw 106, -23, -57,  54, -71, -56,  75,  19
                dw  98, -86,  45,  34,  20,  79, -66, -33
                dw  65,   0, 100, -73,  55, -82,  15,  54
                dw  33,  88,  77, -26, -69,  56,  56, -77
                dw  15,  85,  36,  76, -43,   7, -80,  98
                dw   7,  48,  14,  94,  73, -79, -17, -96
                dw   4,  22,   6,  57,  96,  78, 103,  56
adst4_mat:      dw  18,  75,  50,  18,  75, -89;  89,  50
flipadst4_mat:  dw  89,  50,  75, -89,  50,  18,  18,  75

pw_4096:        times 2 dw 4096
pw_53x256:      times 2 dw 53*256
pw_181x32:      times 2 dw 181*32
pw_181x128:     times 2 dw 181*128
pd_64:          dd 64
pd_512:         dd 512

pw_64_64:       dw  64,  64
pw_64_m64:      dw  64, -64
pw_35_83:       dw  35,  83
pw_m83_35:      dw -83,  35

%define pw_18_75  (adst4_mat+4*0)
%define pw_50_18  (adst4_mat+4*1)
%define pw_75_m89 (adst4_mat+4*2)
%define pw_89_50  (adst4_mat+4*3)

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
ITX_JMP_TABLE  4,  8
ITX_JMP_TABLE  8,  4

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

%macro WRAP_XMM 1+
    INIT_XMM cpuname
    %1
    INIT_YMM cpuname
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

%macro IDCT8_1D_PACKED 0
    punpckhwd            m7, m0, m2 ; 1 5
    punpcklwd            m0, m2     ; 0 4
    punpckhwd            m2, m3, m1 ; 7 3
    punpcklwd            m1, m3, m1 ; 2 6
.dct8b:
    vpbroadcastd         m6, [o(pw_18_75)]
    vpbroadcastd         m4, [o(pw_75_m89)]
    pmaddwd              m8, m6, m7
    pmaddwd              m6, m2
    vpbroadcastd         m9, [o(pw_89_50)]
    pmaddwd              m5, m4, m7
    pmaddwd              m4, m2
    pmaddwd              m3, m9, m2
    pmaddwd              m9, m7
    psubd                m8, m3     ; b3
    vpbroadcastd         m3, [o(pw_50_18)]
    paddd                m6, m9     ; b0
    pmaddwd              m2, m3
    pmaddwd              m7, m3
    psubd                m5, m2     ; b1
    vpbroadcastd         m3, [o(pw_64_64)]
    paddd                m4, m7     ; b2
    vpbroadcastd         m2, [o(pw_64_m64)]
    pmaddwd              m3, m0     ; dct4 a0
    vpbroadcastd         m7, [o(pw_35_83)]
    pmaddwd              m2, m0     ; dct4 a1
    vpbroadcastd         m0, [o(pw_m83_35)]
    pmaddwd              m7, m1     ; dct4 b0
    pmaddwd              m0, m1     ; dct4 b1
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

%macro IDST8_1D_PACKED 0
    SWAP                  7, 8      ; reduces code size by avoiding C4 prefixes
    punpcklwd            m8, m0, m1 ; 0 2
    punpckhwd            m9, m0, m1 ; 1 3
    vpbroadcastd         m0, [r3+16*0]
    vpbroadcastd         m1, [r3+16*1]
    punpcklwd           m10, m2, m3 ; 4 6
    punpckhwd           m11, m2, m3 ; 5 7
    vpbroadcastd         m2, [r3+16*2]
    vpbroadcastd         m3, [r3+16*3]
    vpbroadcastd         m4, [r3+16*4]
    vpbroadcastd         m5, [r3+16*5]
    vpbroadcastd         m6, [r3+16*6]
    vpbroadcastd         m7, [r3+16*7]
    REPX    {pmaddwd x, m8}, m0, m1, m2, m3, m4, m5, m6, m7
%assign %%i 1
%rep 3
    %assign %%j 0
    %assign %%k %%i+8
    %rep 8
        vpbroadcastd     m8, [r3+%%i*4+%%j*16]
        pmaddwd          m8, m%+%%k
        %if %%i == 3 && %%j == 7
            SWAP          7, 8 ; swap back on the last instruction
        %endif
        paddd        m%+%%j, m8
        %assign %%j %%j+1
    %endrep
    %assign %%i %%i+1
%endrep
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

INIT_YMM avx2
INV_TXFM_FN 4, 8
    vpbroadcastd         m1, [o(pw_181x128)]
    pmulhrsw             m0, m1, [cfq+32*0]
    pmulhrsw             m1, [cfq+32*1]
    WIN64_SPILL_XMM      12
    jmp                tx1q

.dconly:
    vpbroadcastw        xm3, [cfq]
    vpbroadcastd        xm2, [o(pw_181x128)]
    or                  r4d, 2
    pmulhrsw            xm3, xm2
    jmp m(inv_txfm_add_4x4_8bpc).dconly2

.pass1_dct:
    vpbroadcastd        m10, [o(pd_64)]
    vpermq               m0, m0, q3120
    vpermq               m1, m1, q3120
    call m(inv_txfm_add_8x4_8bpc).dct4
.pass1_end:
    REPX       {psrad x, 7}, m0, m1, m2, m3
    packssdw             m0, m1
    packssdw             m1, m2, m3
    punpckhwd            m2, m0, m1
    punpcklwd            m0, m1
    punpckhwd            m1, m0, m2
    punpcklwd            m0, m2
    vextracti128        xm2, m0, 1
    vextracti128        xm3, m1, 1
    jmp                tx2q
.pass2_dct:
    vpbroadcastd        m10, [o(pd_512)]
    call .dct8
.pass2_end:
    psrad                m0, 10
    psrad                m1, 10
    call .write_4x4_dct_start
    psrad                m0, m2, 10
    psrad                m1, m3, 10
    call .write_4x4_dct
    jmp m(inv_txfm_add_8x4_8bpc).pass2_end3
ALIGN function_align
.write_4x4_dct_start:
    lea                  r6, [dsq*3]
    pxor                m10, m10
.write_4x4_dct:
    movd                xm8, [dstq+dsq*0]
    vpbroadcastd         m9, [dstq+dsq*1]
    packssdw             m0, m1
    movd                xm1, [dstq+r6   ]
    vpblendd             m8, m9, 0x10
    vpbroadcastd         m9, [dstq+dsq*2]
    vpblendd             m1, m9, 0x10
    punpckldq            m8, m1
    punpcklbw            m8, m10
    paddw                m0, m8
    packuswb             m0, m0
    vextracti128        xm1, m0, 1
    movd       [dstq+dsq*0], xm0
    movd       [dstq+dsq*1], xm1
    pextrd     [dstq+dsq*2], xm1, 1
    pextrd     [dstq+r6   ], xm0, 1
    lea                dstq, [dstq+dsq*4]
    ret
ALIGN function_align
.dct8:
    WRAP_XMM IDCT8_1D_PACKED
    vinserti128          m2, m3, xm2, 1
    vinserti128          m0, m7, xm0, 1
    vinserti128          m1, m6, xm5, 1
    vinserti128          m4, m8, xm4, 1
    paddd                m2, m10    ; round
    paddd                m3, m2, m0 ; a0   a1
    psubd                m2, m0     ; a3   a2
    paddd                m0, m3, m1 ; out0 out1
    psubd                m3, m1     ; out7 out6
    paddd                m1, m2, m4 ; out3 out2
    psubd                m2, m4     ; out4 out5
    ret

.pass1_identity:
    punpckhwd            m2, m0, m1
    punpcklwd            m0, m1
    vextracti128        xm3, m0, 1
    vextracti128        xm4, m2, 1
    punpckhwd           xm1, xm0, xm3
    punpcklwd           xm0, xm3
    punpckhwd           xm3, xm2, xm4
    punpcklwd           xm2, xm4
    jmp                tx2q
.pass2_identity:
    vpbroadcastd         m4, [o(pw_181x32)]
    lea                  r6, [dsq*3]
    pxor               xm10, xm10
    test               eobd, 0x100
    jnz .hdpcm
    pmulhrsw            xm0, xm4
    pmulhrsw            xm1, xm4
    test               eobd, 0x200
    jnz .vdpcm
    call .write_4x4
    pmulhrsw            xm0, xm4, xm2
    pmulhrsw            xm1, xm4, xm3
    call .write_4x4
    jmp m(inv_txfm_add_8x4_8bpc).pass2_end3
.hdpcm:
    vinserti128          m0, xm2, 1
    vinserti128          m1, xm3, 1
    pmulhrsw             m0, m4
    pmulhrsw             m1, m4
    call .write_4x8_hdpcm
    jmp m(inv_txfm_add_8x4_8bpc).pass2_end3
.vdpcm:
    pmulhrsw            xm2, xm4
    pmulhrsw            xm3, xm4
    punpcklqdq          xm4, xm10, xm0
    paddw               xm1, xm0
    shufpd              xm5, xm0, xm1, 0x01
    paddw               xm2, xm1
    paddw               xm0, xm4
    shufpd              xm4, xm1, xm2, 0x01
    paddw               xm3, xm2
    paddw               xm1, xm5
    shufpd              xm5, xm2, xm3, 0x01
    call .write_4x4
    paddw               xm0, xm2, xm4
    paddw               xm1, xm3, xm5
    call .write_4x4
    jmp m(inv_txfm_add_8x4_8bpc).pass2_end3
ALIGN function_align
.write_4x8_hdpcm:
    psllq                m8, m0, 16
    psllq                m9, m1, 16
    paddw                m0, m8
    paddw                m1, m9
    psllq                m8, m0, 32
    psllq                m9, m1, 32
    paddw                m0, m8
    paddw                m1, m9
    call .write_4x4
    vextracti128        xm0, m0, 1
    vextracti128        xm1, m1, 1
.write_4x4:
    movd                xm8, [dstq+dsq*0]
    pinsrd              xm8, [dstq+dsq*1], 1
    movd                xm9, [dstq+dsq*2]
    pinsrd              xm9, [dstq+r6   ], 1
    punpcklbw           xm8, xm10
    punpcklbw           xm9, xm10
    paddsw              xm8, xm0
    paddsw              xm9, xm1
    packuswb            xm8, xm9
    movd       [dstq+dsq*0], xm8
    pextrd     [dstq+dsq*1], xm8, 1
    pextrd     [dstq+dsq*2], xm8, 2
    pextrd     [dstq+r6   ], xm8, 3
    lea                dstq, [dstq+dsq*4]
    ret

.pass1_flipadst:
    lea                  r3, [o(flipadst4_mat)]
    jmp .pass1_dst
.pass1_adst:
    lea                  r3, [o(adst4_mat)]
.pass1_dst:
    vpermq               m0, m0, q3120
    vpermq               m1, m1, q3120
    call m(inv_txfm_add_8x4_8bpc).dst4
    vpbroadcastd         m4, [o(pd_64)]
    REPX      {paddd x, m4}, m0, m1, m2, m3
    jmp .pass1_end
.pass2_flipddt:
    lea                  r3, [o(flipddt8_mat)]
    jmp .pass2_dst
.pass2_ddt:
    lea                  r3, [o(ddt8_mat)]
    jmp .pass2_dst
.pass2_flipadst:
    lea                  r3, [o(flipadst8_mat)]
    jmp .pass2_dst
.pass2_adst:
    lea                  r3, [o(adst8_mat)]
.pass2_dst:
    call .dst8
    vpbroadcastd         m8, [o(pd_512)]
    vinserti128          m0, xm1, 1
    vinserti128          m1, m3, xm2, 1
    vinserti128          m2, m4, xm5, 1
    vinserti128          m3, m7, xm6, 1
    REPX      {paddd x, m8}, m0, m1, m2, m3
    jmp .pass2_end
ALIGN function_align
.dst8:
    WRAP_XMM IDST8_1D_PACKED

INV_TXFM_FN 8, 4
    WIN64_SPILL_XMM      12
    jmp                tx1q

.dconly:
    movd                xm5, [o(pw_181x128)]
    pmulhrsw            xm5, [cfq]
    movd                xm4, [o(pw_17)]
    paddw               xm5, xm4
    psraw               xm5, 5
    vpbroadcastw         m5, xm5
    lea                  r6, [dsq*3]
    pxor                xm4, xm4
    mova              [cfq], xm4
.dconly_loop:
    movq                xm0, [dstq+dsq*0]
    movq                xm1, [dstq+dsq*1]
    vpbroadcastq         m2, [dstq+dsq*2]
    vpbroadcastq         m3, [dstq+r6   ]
    vpblendd             m0, m2, 0x30
    vpblendd             m1, m3, 0x30
    punpcklbw            m0, m4
    punpcklbw            m1, m4
    paddw                m0, m5
    paddw                m1, m5
    packuswb             m0, m1
    vextracti128        xm1, m0, 1
    movq       [dstq+dsq*0], xm0
    movhps     [dstq+dsq*1], xm0
    movq       [dstq+dsq*2], xm1
    movhps     [dstq+r6   ], xm1
    lea                dstq, [dstq+dsq*4]
    dec                 r4d
    jg .dconly_loop
    vzeroupper
    ret

.pass1_dct:
    vpbroadcastd        xm3, [o(pw_181x128)]
    pmulhrsw            xm0, xm3, [cfq+16*0]
    pmulhrsw            xm1, xm3, [cfq+16*1]
    pmulhrsw            xm2, xm3, [cfq+16*2]
    pmulhrsw            xm3, [cfq+16*3]
    vpbroadcastd        m10, [o(pd_64)]
    call m(inv_txfm_add_4x8_8bpc).dct8
    REPX       {psrad x, 7}, m0, m1, m2, m3
    packssdw             m0, m2        ; 0 4   1 5
    packssdw             m1, m3        ; 3 7   2 6
    vpermq               m0, m0, q3120 ; 0 1   4 5
    vpermq               m1, m1, q1302 ; 2 3   6 7
.pass1_end:
    punpckhwd            m2, m0, m1
    punpcklwd            m0, m1
    punpckhwd            m1, m0, m2
    punpcklwd            m0, m2
    jmp                tx2q
.pass2_dct:
    vpbroadcastd        m10, [o(pd_512)]
    call .dct4
    REPX      {psrad x, 10}, m0, m1, m2, m3
    packssdw             m0, m1
    packssdw             m1, m2, m3
.pass2_end:
    call .write_8x4_start
.pass2_end3:
    mova         [cfq+32*0], m10
    mova         [cfq+32*1], m10
    RET
ALIGN function_align
.write_8x4_hdpcm_start:
    lea                  r6, [dsq*3]
    pxor                m10, m10
.write_8x4_hdpcm_vpermq:
    vpermq               m0, m0, q3120
    vpermq               m1, m1, q3120
.write_8x4_hdpcm:
    pslldq               m8, m0, 2
    pslldq               m9, m1, 2
    paddw                m0, m8
    paddw                m1, m9
    pslldq               m8, m0, 4
    pslldq               m9, m1, 4
    paddw                m0, m8
    paddw                m1, m9
    punpcklqdq           m8, m10, m0
    punpcklqdq           m9, m10, m1
    paddw                m0, m8
    paddw                m1, m9
    jmp .write_8x4
.write_8x4_start:
    lea                  r6, [dsq*3]
    pxor                m10, m10
.write_8x4_vpermq:
    vpermq               m0, m0, q3120
    vpermq               m1, m1, q3120
.write_8x4:
    movq                xm8, [dstq+dsq*0]
    vpbroadcastq        m11, [dstq+dsq*1]
    movq                xm9, [dstq+dsq*2]
    vpblendd             m8, m11, 0x30
    vpbroadcastq        m11, [dstq+r6   ]
    vpblendd             m9, m11, 0x30
    punpcklbw            m8, m10
    punpcklbw            m9, m10
    paddw                m0, m8
    paddw                m1, m9
    packuswb             m0, m1
    vextracti128        xm1, m0, 1
    movq       [dstq+dsq*0], xm0
    movq       [dstq+dsq*1], xm1
    movhps     [dstq+dsq*2], xm0
    movhps     [dstq+r6   ], xm1
    lea                dstq, [dstq+dsq*4]
    ret
ALIGN function_align
.dct4:
    IDCT4_1D_PACKED     m10

.pass1_identity:
    vpbroadcastd         m2, [o(pw_181x128)]
    vpbroadcastd         m3, [o(pw_53x256)]
    mova                xm0, [cfq+16*0]
    vinserti128          m0, [cfq+16*2], 1
    mova                xm1, [cfq+16*1]
    vinserti128          m1, [cfq+16*3], 1
    pmulhrsw             m0, m2
    pmulhrsw             m1, m2
    pmulhrsw             m2, m3, m0 ; (x * 181 + 64) >> 7
    pmulhrsw             m3, m1     ; = x + ((x * 53 + 64) >> 7)
    paddw                m0, m2
    paddw                m1, m3
    jmp .pass1_end
.pass2_identity:
    vpbroadcastd         m2, [o(pw_4096)]
    pmulhrsw             m0, m2
    pmulhrsw             m1, m2
    test               eobd, 0x300
    jz .pass2_end
    test               eobd, 0x200
    jnz .vdpcm
.hdpcm:
    call .write_8x4_hdpcm_start
    jmp .pass2_end3
.vdpcm:
    pslldq               m2, m0, 8
    paddw                m1, m0
    shufpd               m3, m0, m1, 0x05
    paddw                m0, m2
    paddw                m1, m3
    jmp .pass2_end

.pass1_flipddt:
    lea                  r3, [o(flipddt8_mat)]
    jmp .pass1_dst
.pass1_ddt:
    lea                  r3, [o(ddt8_mat)]
    jmp .pass1_dst
.pass1_flipadst:
    lea                  r3, [o(flipadst8_mat)]
    jmp .pass1_dst
.pass1_adst:
    lea                  r3, [o(adst8_mat)]
.pass1_dst:
    vpbroadcastd        xm3, [o(pw_181x128)]
    pmulhrsw            xm0, xm3, [cfq+16*0]
    pmulhrsw            xm1, xm3, [cfq+16*1]
    pmulhrsw            xm2, xm3, [cfq+16*2]
    pmulhrsw            xm3, [cfq+16*3]
    call m(inv_txfm_add_4x8_8bpc).dst8
    vpbroadcastd         m8, [o(pd_64)]
    vinserti128          m0, xm4, 1
    vinserti128          m1, xm5, 1
    vinserti128          m2, xm6, 1
    vinserti128          m3, xm7, 1
    REPX      {paddd x, m8}, m0, m1, m2, m3
    REPX       {psrad x, 7}, m0, m1, m2, m3
    packssdw             m0, m1
    packssdw             m1, m2, m3
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
