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

adst16_mat:
dw   8,  41,  25,  55,  67,  84,  77,  88,  89,  81,  87,  73,  62,  33,  48,  17
dw  17,  73,  48,  87,  88,  55,  77,  25,  -8, -67, -41, -84, -89, -62, -81, -33
dw  25,  88,  67,  81,  48, -48,   0, -81, -88, -25, -67,  25,  67,  81,  88,  48
dw  33,  84,  81,  41, -25, -87, -77, -48,  17,  88,  73,  55,  -8, -89, -67, -62
dw  41,  62,  88, -17, -81,  -8, -77,  67,  87, -48,  33, -89, -55,  84,  25,  73
dw  48,  25,  88, -67, -81,  81,   0,  67, -25, -48, -88,  48,  88, -67,  25, -81
dw  55, -17,  81, -89, -25,  62,  77, -48, -84,  88,   8,  33, -73,  41, -67,  87
dw  62, -55,  67, -73,  48, -41,  77, -81,  33, -25,  84, -87,  17,  -8,  88, -89
dw  67, -81,  48, -25,  88, -88,   0,  25,  81, -67, -48,  67,  48, -25, -81,  88
dw  73, -89,  25,  33,  67, -17, -77,  88, -41,  81, -62,   8, -87,  55,  48, -84
dw  77, -77,   0,  77,   0,  77, -77,   0, -77,   0,  77, -77,  77, -77,   0,  77
dw  81, -48, -25,  88, -67,  67,   0, -88,  48, -81,  25,  81, -25,  88, -48, -67
dw  84,  -8, -48,  62, -88, -33,  77, -25,  73,  67, -89, -17, -41, -87,  81,  55
dw  87,  33, -67,   8, -48, -89,  77,  81, -55,  25,  17, -62,  84,  73, -88, -41
dw  88,  67, -81, -48,  25, -25,   0,  48, -67, -88,  81,  88, -81, -48,  67,  25
dw  89,  87, -88, -84,  81,  73, -77, -67,  62,  48, -55, -41,  33,  17, -25,  -8
flipadst16_mat:
dw  89,  87,  88,  84,  81,  73,  77,  67,  62,  48,  55,  41,  33,  17,  25,   8
dw  88,  67,  81,  48,  25, -25,   0, -48, -67, -88, -81, -88, -81, -48, -67, -25
dw  87,  33,  67,  -8, -48, -89, -77, -81, -55,  25, -17,  62,  84,  73,  88,  41
dw  84,  -8,  48, -62, -88, -33, -77,  25,  73,  67,  89,  17, -41, -87, -81, -55
dw  81, -48,  25, -88, -67,  67,   0,  88,  48, -81, -25, -81, -25,  88,  48,  67
dw  77, -77,   0, -77,   0,  77,  77,   0, -77,   0, -77,  77,  77, -77,   0, -77
dw  73, -89, -25, -33,  67, -17,  77, -88, -41,  81,  62,  -8, -87,  55, -48,  84
dw  67, -81, -48,  25,  88, -88,   0, -25,  81, -67,  48, -67,  48, -25,  81, -88
dw  62, -55, -67,  73,  48, -41, -77,  81,  33, -25, -84,  87,  17,  -8, -88,  89
dw  55, -17, -81,  89, -25,  62, -77,  48, -84,  88,  -8, -33, -73,  41,  67, -87
dw  48,  25, -88,  67, -81,  81,   0, -67, -25, -48,  88, -48,  88, -67, -25,  81
dw  41,  62, -88,  17, -81,  -8,  77, -67,  87, -48, -33,  89, -55,  84, -25, -73
dw  33,  84, -81, -41, -25, -87,  77,  48,  17,  88, -73, -55,  -8, -89,  67,  62
dw  25,  88, -67, -81,  48, -48,   0,  81, -88, -25,  67, -25,  67,  81, -88, -48
dw  17,  73, -48, -87,  88,  55, -77, -25,  -8, -67,  41,  84, -89, -62,  81,  33
dw   8,  41, -25, -55,  67,  84, -77, -88,  89,  81, -87, -73,  62,  33, -48, -17
ddt16_mat:
dw  12,  37,  17,  45,  47,  64,  60,  82,  89,  92, 100,  84,  69,  51,  50,  44
dw  15,  49,  23,  60,  60,  70,  74,  73,  48, -35,   9, -71, -83, -89, -79, -95
dw  19,  60,  30,  69,  61,  40,  64,   3, -53, -91, -99, -46,   2,  73,  47, 124
dw  23,  69,  38,  73,  49, -19,  28, -80, -96,  42, -45,  88,  75, -17,  14,-126
dw  30,  75,  48,  66,  19, -79, -31, -91,  -5,  71,  84, -16, -78, -45, -60, 108
dw  39,  75,  61,  40, -29, -78, -87,  10,  89, -69,  36, -67,  18,  89,  67, -81
dw  51,  61,  76,  -8, -77,  11, -82,  94,  16, -22, -81,  79,  50,-103, -37,  54
dw  66,  29,  87, -65, -83,  92,   4,  18, -83,  85,   4, -22, -85,  97,  -6, -30
dw  78, -18,  83, -91, -16,  28,  88, -84,  12, -60,  73, -46,  81, -83,  49,  16
dw  88, -67,  59, -57,  75, -85,  54,  -5,  75, -17, -60,  84, -43,  71, -80,  -6
dw  94, -96,  19,  21,  93, -41, -55,  80, -51,  77, -17, -68,  -6, -56,  98,   1
dw  97, -83, -30,  86,   3,  82, -77, -17, -43, -70,  76,  15,  53,  44, -99,   3
dw  93, -28, -73,  81, -92,  39,  29, -70,  81,  11, -55,  46, -81, -31,  90,  -4
dw  83,  40, -99,   8, -74, -83,  88,  47, -14,  56, -21, -83,  88,  22, -71,   5
dw  68,  84, -99, -69,  32, -37,   3,  55, -75, -83,  81,  82, -69, -11,  48,  -3
flipddt16_mat:
dw  50,  83, -76, -90,  97,  83, -86, -68,  67,  49, -56, -40,  32,   5, -19,   2
dw  68,  84, -99, -69,  32, -37,   3,  55, -75, -83,  81,  82, -69, -11,  48,  -3
dw  83,  40, -99,   8, -74, -83,  88,  47, -14,  56, -21, -83,  88,  22, -71,   5
dw  93, -28, -73,  81, -92,  39,  29, -70,  81,  11, -55,  46, -81, -31,  90,  -4
dw  97, -83, -30,  86,   3,  82, -77, -17, -43, -70,  76,  15,  53,  44, -99,   3
dw  94, -96,  19,  21,  93, -41, -55,  80, -51,  77, -17, -68,  -6, -56,  98,   1
dw  88, -67,  59, -57,  75, -85,  54,  -5,  75, -17, -60,  84, -43,  71, -80,  -6
dw  78, -18,  83, -91, -16,  28,  88, -84,  12, -60,  73, -46,  81, -83,  49,  16
dw  66,  29,  87, -65, -83,  92,   4,  18, -83,  85,   4, -22, -85,  97,  -6, -30
dw  51,  61,  76,  -8, -77,  11, -82,  94,  16, -22, -81,  79,  50,-103, -37,  54
dw  39,  75,  61,  40, -29, -78, -87,  10,  89, -69,  36, -67,  18,  89,  67, -81
dw  30,  75,  48,  66,  19, -79, -31, -91,  -5,  71,  84, -16, -78, -45, -60, 108
dw  23,  69,  38,  73,  49, -19,  28, -80, -96,  42, -45,  88,  75, -17,  14,-126
dw  19,  60,  30,  69,  61,  40,  64,   3, -53, -91, -99, -46,   2,  73,  47, 124
dw  15,  49,  23,  60,  60,  70,  74,  73,  48, -35,   9, -71, -83, -89, -79, -95
dw  12,  37,  17,  45,  47,  64,  60,  82,  89,  92, 100,  84,  69,  51,  50,  44

dct16_mat:      dw  90,  80,  87,  70,  26,  57,   9,  43
                dw  87,   9,  57, -43, -70, -80, -26, -90
                dw  80, -70,   9, -87,  90, -26,  43,  57
                dw  70, -87, -43,   9, -80,  90, -57,  26
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

pw_512:         times 2 dw 512
pw_1024:        times 2 dw 1024
pw_2048:        times 2 dw 2048
pw_4096:        times 2 dw 4096
pw_53x256:      times 2 dw 53*256
pw_181x16:      times 2 dw 181*16
pw_181x32:      times 2 dw 181*32
pw_181x128:     times 2 dw 181*128
pd_32:          dd 32
pd_64:          dd 64
pd_512:         dd 512
pd_1024:        dd 1024
pd_2048:        dd 2048

pw_64_64:       dw  64,  64
pw_64_m64:      dw  64, -64
pw_35_83:       dw  35,  83
pw_m83_35:      dw -83,  35
pw_0_83:        dw   0,  83
pw_0_35:        dw   0,  35
pw_89_75:       dw  89,  75
pw_75_m18:      dw  75, -18
pw_50_m89:      dw  50, -89
pw_18_m50:      dw  18, -50

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
ITX_JMP_TABLE  4, 16
ITX_JMP_TABLE  8,  4
ITX_JMP_TABLE  8,  8
ITX_JMP_TABLE 16,  4

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

%macro IDCT16_1D_PACKED 1 ; cfq_offset
    lea                  r3, [o(dct16_mat)]
.dct16b:
    punpcklwd            m8, m0, m4 ;  0  8
    punpckhwd            m0, m2     ;  1  5
    mova [cfq+mmsize*(%1+3)], m8
    punpcklwd            m9, m1, m5 ;  2 10
    punpckhwd            m1, m3     ;  3  7
    mova [cfq+mmsize*(%1+2)], m9
    punpcklwd            m8, m6, m2 ; 12  4
    punpckhwd            m2, m6, m4 ; 13  9
    mova [cfq+mmsize*(%1+5)], m8
    punpcklwd            m6, m7, m3 ; 14  6
    punpckhwd            m3, m7, m5 ; 15 11
    mova [cfq+mmsize*(%1+4)], m6
.dct16c:
    call .dct16d
    add                  r3, 4*8
    mova [cfq+mmsize*(%1+0)], m8  ; b0
    mova [cfq+mmsize*(%1+1)], m9  ; b1
    mova [cfq+mmsize*(%1+6)], m12 ; b7
    mova [cfq+mmsize*(%1+7)], m11 ; b6
    call .dct16d
    mova                 m7, [cfq+mmsize*(%1+2)]
    mova                 m0, [cfq+mmsize*(%1+3)]
    mova                 m2, [cfq+mmsize*(%1+4)]
    mova                 m1, [cfq+mmsize*(%1+5)]
    mova [cfq+mmsize*(%1+2)], m9  ; b3
    mova [cfq+mmsize*(%1+3)], m8  ; b2
    mova [cfq+mmsize*(%1+4)], m11 ; b4
    mova [cfq+mmsize*(%1+5)], m12 ; b5
%if mmsize == 16
    jmp m(inv_txfm_add_4x8_8bpc).dct8b
%else
    jmp m(inv_txfm_add_8x8_8bpc).dct8b
%endif
.dct16d:
    vpbroadcastd         m7, [r3+4*0]
    vpbroadcastd        m12, [r3+4*1]
    vpbroadcastd        m11, [r3+4*4]
    vpbroadcastd         m6, [r3+4*5]
    pmaddwd              m8, m7, m0
    pmaddwd              m4, m12, m1
    pmaddwd              m9, m11, m0
    pmaddwd              m5, m6, m1
    pmaddwd              m7, m3
    pmaddwd             m12, m2
    pmaddwd             m11, m3
    pmaddwd              m6, m2
    paddd                m8, m4
    paddd                m9, m5
    psubd               m12, m7
    vpbroadcastd         m7, [r3+4*2]
    psubd               m11, m6
    vpbroadcastd         m6, [r3+4*6]
    pmaddwd              m4, m7, m2
    pmaddwd              m5, m6, m2
    pmaddwd              m7, m1
    pmaddwd              m6, m1
    paddd                m8, m4
    paddd                m9, m5
    psubd               m12, m7
    vpbroadcastd         m7, [r3+4*3]
    paddd               m11, m6
    vpbroadcastd         m6, [r3+4*7]
    pmaddwd              m4, m7, m3
    pmaddwd              m5, m6, m3
    pmaddwd              m7, m0
    pmaddwd              m6, m0
    paddd                m8, m4 ; b0 / b2
    paddd                m9, m5 ; b1 / b3
    paddd               m12, m7 ; b7 / b5
    psubd               m11, m6 ; b6 / b4
    ret
%endmacro

%macro IDCT16_1D_PACKED_FAST 1 ; cfq_offset
    lea                  r3, [o(dct16_mat)]
.dct16_fast2:
    punpcklwd            m9, m0, m2  ; 0 4
    punpckhwd            m0, m2      ; 1 5
    punpcklwd            m8, m1, m3  ; 2 6
    punpckhwd            m1, m3      ; 3 7
.dct16_fast3:
    call .dct16_fast4
    add                  r3, 4*8
    mova [cfq+mmsize*(%1+0)], m4     ; c0
    mova [cfq+mmsize*(%1+1)], m5     ; c1
    mova [cfq+mmsize*(%1+6)], m7     ; c7
    mova [cfq+mmsize*(%1+7)], m6     ; c6
    call .dct16_fast4
    vpbroadcastd         m2, [o(pd_64)]
    vpbroadcastd         m1, [o(pw_0_83)]
    vpbroadcastd        m11, [o(pw_0_35)]
    mova [cfq+mmsize*(%1+2)], m5     ; c3
    mova [cfq+mmsize*(%1+3)], m4     ; c2
    mova [cfq+mmsize*(%1+4)], m6     ; c4
    mova [cfq+mmsize*(%1+5)], m7     ; c5
    vpbroadcastd         m6, [o(pw_89_75)]
    pmaddwd              m2, m9
    vpbroadcastd         m5, [o(pw_75_m18)]
    pmaddwd              m1, m9
    vpbroadcastd         m4, [o(pw_50_m89)]
    pmaddwd             m11, m9
    vpbroadcastd         m7, [o(pw_18_m50)]
    pmaddwd              m6, m8      ; b0
    pmaddwd              m5, m8      ; b1
    pmaddwd              m4, m8      ; b2
    pmaddwd              m8, m7      ; b3
    paddd                m2, m10     ; rnd
%if mmsize == 16
INIT_YMM cpuname
    vinserti128          m2, xm2, 1
    vinserti128          m1, xm11, 1
    paddd                m3, m2, m1  ; a0 a1
    psubd                m2, m1      ; a3 a2
    vinserti128          m1, m6, xm5, 1
    vinserti128          m4, m8, xm4, 1
    jmp m(inv_txfm_add_4x8_8bpc).dct8c
INIT_XMM cpuname
%else
    paddd                m0, m2, m1  ; a0
    psubd                m3, m2, m1  ; a3
    paddd                m1, m11, m2 ; a1
    psubd                m2, m11     ; a2
    jmp m(inv_txfm_add_8x8_8bpc).dct8c
%endif
.dct16_fast4:
    vpbroadcastd         m4, [r3+4*0]
    vpbroadcastd         m6, [r3+4*1]
    vpbroadcastd         m2, [r3+4*2]
    vpbroadcastd         m7, [r3+4*3]
    pmaddwd              m4, m0
    pmaddwd              m6, m1
    pmaddwd              m2, m1
    pmaddwd              m7, m0
    vpbroadcastd         m5, [r3+4*4]
    vpbroadcastd         m3, [r3+4*5]
    paddd                m4, m6 ; b0 / b2
    vpbroadcastd         m6, [r3+4*6]
    psubd                m7, m2 ; b7 / b5
    vpbroadcastd         m2, [r3+4*7]
    pmaddwd              m5, m0
    pmaddwd              m3, m1
    pmaddwd              m6, m1
    pmaddwd              m2, m0
    paddd                m5, m3 ; b1 / b3
    psubd                m6, m2 ; b6 / b4
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

%macro IDST16_1D_PACKED_1ROW 0
    vpbroadcastd         m0, [r3+4*0]
    vpbroadcastd         m1, [r3+4*1]
    vpbroadcastd         m8, [r3+4*2]
    vpbroadcastd         m9, [r3+4*3]
%if mmsize == 16
    pmaddwd              m0, m13
%else
    pmaddwd              m0, m10
%endif
    pmaddwd              m1, m11
    pmaddwd              m8, m12
    pmaddwd              m9, m2
    paddd                m0, m8
    vpbroadcastd         m8, [r3+4*4]
    paddd                m1, m9
    vpbroadcastd         m9, [r3+4*5]
    pmaddwd              m8, m3
    pmaddwd              m9, m4
    paddd                m0, m8
    vpbroadcastd         m8, [r3+4*6]
    paddd                m1, m9
    vpbroadcastd         m9, [r3+4*7]
    pmaddwd              m8, m5
    pmaddwd              m9, m6
    paddd                m0, m8
    paddd                m1, m9
    paddd                m0, m1
    ret
%endmacro

%macro IDST16_1D_PACKED_2ROWS_FAST 1 ; row_offset
    vpbroadcastd         m0, [r3+4*0]
    vpbroadcastd         m3, [r3+4*1]
    vpbroadcastd         m8, [r3+4*2]
    vpbroadcastd         m9, [r3+4*3]
%if mmsize == 16
    pmaddwd              m0, m13
%else
    pmaddwd              m0, m10
%endif
    pmaddwd              m3, m11
    pmaddwd              m8, m12
    vpbroadcastd         m1, [r3+4*0+32*%1]
    pmaddwd              m9, m2
    vpbroadcastd         m4, [r3+4*1+32*%1]
    paddd                m0, m8
    vpbroadcastd         m8, [r3+4*2+32*%1]
    paddd                m3, m9
    vpbroadcastd         m9, [r3+4*3+32*%1]
%if mmsize == 16
    pmaddwd              m1, m13
%else
    pmaddwd              m1, m10
%endif
    pmaddwd              m4, m11
    pmaddwd              m8, m12
    pmaddwd              m9, m2
    paddd                m1, m8
    paddd                m4, m9
    paddd                m0, m3
    paddd                m1, m4
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
.dconly3:
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
.pass1_end2:
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
.dct8c:
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
.write_4x4_end:
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
.dconly2:
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

INV_TXFM_FN 8, 8
    WIN64_SPILL_XMM      12
    jmp                tx1q

.dconly:
    movd                xm5, [o(pw_33)]
    paddsw              xm5, [cfq]
    or                  r4d, 2
    psraw               xm5, 6
    jmp m(inv_txfm_add_8x4_8bpc).dconly2

.pass1_dct:
    vpermq               m0, [cfq+32*0], q3120
    vpermq               m1, [cfq+32*1], q3120
    vpermq               m2, [cfq+32*2], q3120
    vpermq               m3, [cfq+32*3], q3120
    vpbroadcastd        m10, [o(pd_64)]
    call .dct8
.pass1_end:
    call .shift7_pack
    punpckhwd            m4, m0, m1
    punpcklwd            m0, m1
    punpckhwd            m1, m2, m3
    punpcklwd            m2, m3
    punpckhdq            m3, m0, m2
    punpckldq            m0, m2
    punpckhdq            m2, m4, m1
    punpckldq            m4, m1
    vinserti128          m1, m3, xm2, 1
    vperm2i128           m3, m2, 0x31
    vperm2i128           m2, m0, m4, 0x31
    vinserti128          m0, xm4, 1
    jmp                tx2q
.pass2_dct:
    vpbroadcastd        m10, [o(pd_1024)]
    call .dct8
    REPX      {psrad x, 11}, m0, m1, m2, m3
    packssdw             m0, m1
    packssdw             m1, m2, m3
    REPX      {psrad x, 11}, m4, m5, m6, m7
    packssdw             m2, m4, m5
    packssdw             m3, m6, m7
.pass2_end:
    lea                  r6, [dsq*3]
    pxor                m10, m10
    call .write_8x8
.pass2_end2:
    REPX {mova [cfq+32*x], m10}, 0, 1, 2, 3
    RET
ALIGN function_align
.shift7_pack:
    REPX       {psrad x, 7}, m0, m4, m1, m5
    packssdw             m0, m4
    packssdw             m1, m5
    REPX       {psrad x, 7}, m2, m6, m3, m7
    packssdw             m2, m6
    packssdw             m3, m7
    ret
ALIGN function_align
.dct8:
    IDCT8_1D_PACKED
    paddd                m3, m10    ; rnd
    paddd                m2, m10
    paddd                m1, m2, m0 ; a1
    psubd                m2, m0     ; a2
    paddd                m0, m3, m7 ; a0
    psubd                m3, m7     ; a3
    psubd                m7, m0, m6 ; out7
    paddd                m0, m6     ; out0
    psubd                m6, m1, m5 ; out6
    paddd                m1, m5     ; out1
    psubd                m5, m2, m4 ; out5
    paddd                m2, m4     ; out2
    psubd                m4, m3, m8 ; out4
    paddd                m3, m8     ; out3
    ret

.pass1_identity:
    mova                xm0, [cfq+16*0]
    vinserti128          m0, [cfq+16*4], 1
    mova                xm1, [cfq+16*1]
    vinserti128          m1, [cfq+16*5], 1
    mova                xm2, [cfq+16*2]
    vinserti128          m2, [cfq+16*6], 1
    mova                xm3, [cfq+16*3]
    vinserti128          m3, [cfq+16*7], 1
    vpbroadcastd         m7, [o(pw_53x256)]
    pmulhrsw             m4, m7, m0
    pmulhrsw             m5, m7, m1
    pmulhrsw             m6, m7, m2
    pmulhrsw             m7, m3
    paddsw               m0, m4
    paddsw               m1, m5
    paddsw               m2, m6
    paddsw               m3, m7
    jmp m(inv_txfm_add_4x16_8bpc).pass1_end2
.pass2_identity:
    vpbroadcastd         m4, [o(pw_181x16)]
    REPX   {pmulhrsw x, m4}, m0, m1, m2, m3
    test               eobd, 0x300
    jz .pass2_end
    lea                  r6, [dsq*3]
    pxor                m10, m10
    test               eobd, 0x200
    jnz .vdpcm
.hdpcm:
    call m(inv_txfm_add_8x4_8bpc).write_8x4_hdpcm_vpermq
    vpermq               m0, m2, q3120
    vpermq               m1, m3, q3120
    call m(inv_txfm_add_8x4_8bpc).write_8x4_hdpcm
    jmp .pass2_end2
.vdpcm:
    call .write_8x8_vdpcm
    jmp .pass2_end2
ALIGN function_align
.write_8x8_vdpcm:
    pslldq               m8, m0, 8
    paddw                m1, m0
    shufpd               m9, m0, m1, 0x05
    paddw                m2, m1
    paddw                m0, m8
    shufpd               m8, m1, m2, 0x05
    paddw                m3, m2
    paddw                m1, m9
    shufpd               m9, m2, m3, 0x05
    paddw                m2, m8
    paddw                m3, m9
.write_8x8:
    call m(inv_txfm_add_8x4_8bpc).write_8x4_vpermq
    vpermq               m0, m2, q3120
    vpermq               m1, m3, q3120
    jmp m(inv_txfm_add_8x4_8bpc).write_8x4

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
    vpermq               m0, [cfq+32*0], q3120
    vpermq               m1, [cfq+32*1], q3120
    vpermq               m2, [cfq+32*2], q3120
    vpermq               m3, [cfq+32*3], q3120
    call .dst8
    vpbroadcastd         m8, [o(pd_64)]
    REPX      {paddd x, m8}, m0, m1, m2, m3, m4, m5, m6, m7
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
    vpbroadcastd         m8, [o(pw_4096)]
    REPX       {psrad x, 8}, m0, m1, m2, m3
    packssdw             m0, m1
    packssdw             m1, m2, m3
    REPX       {psrad x, 8}, m4, m5, m6, m7
    packssdw             m2, m4, m5
    packssdw             m3, m6, m7
    REPX   {pmulhrsw x, m8}, m0, m1, m2, m3
    jmp .pass2_end
ALIGN function_align
.dst8:
    IDST8_1D_PACKED

INV_TXFM_FN 4, 16
    WIN64_SPILL_XMM      13
    jmp                tx1q

.dconly:
    vpbroadcastw        xm3, [cfq]
    vpbroadcastd        xm2, [o(pw_512)]
    or                  r4d, 4
    pmulhrsw            xm3, xm2
    jmp m(inv_txfm_add_4x4_8bpc).dconly3

.pass1_dct_fast:
    vbroadcasti128       m0, [cfq+32*0]
    vbroadcasti128       m2, [cfq+32*1]
    vbroadcasti128       m1, [cfq+32*2]
    vbroadcasti128       m3, [cfq+32*3]
    shufpd               m0, m2, 0x0c
    shufpd               m1, m3, 0x0c
    call m(inv_txfm_add_8x4_8bpc).dct4
.pass1_fast_end:
    REPX       {psrad x, 6}, m0, m1, m2, m3
    jmp m(inv_txfm_add_4x8_8bpc).pass1_end2
.pass1_dct:
    vpbroadcastd        m10, [o(pd_32)]
    lea                 r3d, [eobq+(18<<16)]
    test               eobb, 0x10 ; TX_CLASS_H
    cmovnz             eobd, r3d
    sub                eobd, 26<<16
    jl .pass1_dct_fast
    mova                 m0, [cfq+32*0]
    mova                 m1, [cfq+32*1]
    mova                 m2, [cfq+32*2]
    mova                 m3, [cfq+32*3]
    call m(inv_txfm_add_16x4_8bpc).dct4
.pass1_end:
    REPX       {psrad x, 6}, m0, m4, m1, m5
    packssdw             m0, m4
    packssdw             m1, m5
    REPX       {psrad x, 6}, m2, m6, m3, m7
    packssdw             m2, m6
    packssdw             m3, m7
.pass1_end2:
    punpckhwd            m4, m2, m3
    punpcklwd            m2, m3
    punpckhwd            m3, m0, m1
    punpcklwd            m0, m1
    punpckhdq            m1, m0, m2
    punpckldq            m0, m2
    punpckldq            m2, m3, m4
    punpckhdq            m3, m4
    jmp                tx2q
.pass2_dct:
    vpbroadcastd        m10, [o(pd_2048)]
    test               eobd, eobd
    jl .pass2_dct_fast
    vextracti128        xm4, m0, 1
    vextracti128        xm5, m1, 1
    vextracti128        xm6, m2, 1
    vextracti128        xm7, m3, 1
    call .dct16
    jmp .pass2_dct2
.pass2_dct_fast:
    call .dct16_fast
.pass2_dct2:
    mova                 m5, [cfq+32*0]
    mova                 m6, [cfq+32*1]
    psubd                m4, m0, m5 ; out15 out14
    paddd                m0, m5     ; out0  out1
    psubd                m5, m1, m6 ; out12 out13
    paddd                m1, m6     ; out3  out2
    psrad                m0, 12
    psrad                m1, 12
    call m(inv_txfm_add_4x8_8bpc).write_4x4_dct_start
    mova                 m1, [cfq+32*2]
    mova                 m6, [cfq+32*3]
    paddd                m0, m2, m1 ; out4  out5
    psubd                m2, m1     ; out11 out10
    paddd                m1, m3, m6 ; out7  out6
    psubd                m3, m6     ; out8  out9
    psrad                m0, 12
    psrad                m1, 12
    call m(inv_txfm_add_4x8_8bpc).write_4x4_dct
    psrad                m0, m3, 12
    psrad                m1, m2, 12
    call m(inv_txfm_add_4x8_8bpc).write_4x4_dct
    psrad                m0, m5, 12
    psrad                m1, m4, 12
    call m(inv_txfm_add_4x8_8bpc).write_4x4_dct
    jmp m(inv_txfm_add_16x4_8bpc).pass2_end2
ALIGN function_align
.dct16_fast:
    WRAP_XMM IDCT16_1D_PACKED_FAST 0
ALIGN function_align
.dct16:
    WRAP_XMM IDCT16_1D_PACKED 0

.pass1_identity:
    mova                 m0, [cfq+32*0]
    mova                 m1, [cfq+32*1]
    mova                 m2, [cfq+32*2]
    mova                 m3, [cfq+32*3]
    REPX      {paddsw x, x}, m0, m1, m2, m3
    lea                 r3d, [eobq-(32<<16)]
    test               eobb, 0x10 ; TX_CLASS_V
    cmovnz             eobd, r3d
    jmp .pass1_end2
.pass2_identity_fast:
    call m(inv_txfm_add_4x8_8bpc).write_4x4
    pmulhrsw            xm0, xm4, xm2
    pmulhrsw            xm1, xm4, xm3
    call m(inv_txfm_add_4x8_8bpc).write_4x4
    jmp m(inv_txfm_add_16x4_8bpc).pass2_end2
.pass2_identity:
    vpbroadcastd         m4, [o(pw_2048)]
    lea                  r6, [dsq*3]
    pxor                m10, m10
    pmulhrsw             m0, m4
    pmulhrsw             m1, m4
    test               eobd, eobd
    jl .pass2_identity_fast
    pmulhrsw             m2, m4
    pmulhrsw             m3, m4
    test               eobd, 0x100
    jnz .hdpcm
    test               eobd, 0x200
    jnz .vdpcm
    call .write_4x16
    jmp m(inv_txfm_add_16x4_8bpc).pass2_end2
.hdpcm:
    call .write_4x16_hdpcm
    jmp m(inv_txfm_add_16x4_8bpc).pass2_end2
.vdpcm:
    call .write_4x16_vdpcm
    jmp m(inv_txfm_add_16x4_8bpc).pass2_end2
ALIGN function_align
.write_4x16_hdpcm:
    psllq                m8, m0, 16
    psllq                m9, m1, 16
    paddw                m0, m8
    paddw                m1, m9
    psllq                m8, m0, 32
    psllq                m9, m1, 32
    paddw                m0, m8
    paddw                m1, m9
    psllq                m8, m2, 16
    psllq                m9, m3, 16
    paddw                m2, m8
    paddw                m3, m9
    psllq                m8, m2, 32
    psllq                m9, m3, 32
    paddw                m2, m8
    paddw                m3, m9
.write_4x16:
    call .write_4x8
    vextracti128        xm0, m0, 1
    vextracti128        xm1, m1, 1
    vextracti128        xm2, m2, 1
    vextracti128        xm3, m3, 1
.write_4x8:
    call m(inv_txfm_add_4x8_8bpc).write_4x4
    movd                xm8, [dstq+dsq*0]
    pinsrd              xm8, [dstq+dsq*1], 1
    movd                xm9, [dstq+dsq*2]
    pinsrd              xm9, [dstq+r6   ], 1
    pmovzxbw            xm8, xm8
    pmovzxbw            xm9, xm9
    paddsw              xm8, xm2
    paddsw              xm9, xm3
    jmp m(inv_txfm_add_4x8_8bpc).write_4x4_end
.write_4x16_vdpcm:
    pslldq               m8, m0, 8
    paddsw               m1, m0
    shufpd               m9, m0, m1, 0x05
    paddsw               m2, m1
    paddsw               m0, m8
    shufpd               m8, m1, m2, 0x05
    paddsw               m3, m2
    paddsw               m1, m9
    shufpd               m9, m2, m3, 0x05
    paddsw               m2, m8
    paddsw               m3, m9
    call .write_4x8
    punpckhqdq          xm8, xm3, xm3
    vextracti128        xm0, m0, 1
    vextracti128        xm1, m1, 1
    vextracti128        xm2, m2, 1
    vextracti128        xm3, m3, 1
    REPX    {paddsw x, xm8}, xm0, xm1, xm2, xm3
    jmp .write_4x8

.pass1_dst_fast:
    vbroadcasti128       m0, [cfq+32*0]
    vbroadcasti128       m2, [cfq+32*1]
    vbroadcasti128       m1, [cfq+32*2]
    vbroadcasti128       m3, [cfq+32*3]
    shufpd               m0, m2, 0x0c
    shufpd               m1, m3, 0x0c
    call m(inv_txfm_add_8x4_8bpc).dst4
    REPX     {paddd x, m12}, m0, m1, m2, m3
    jmp .pass1_fast_end
.pass1_flipadst:
    lea                  r3, [o(flipadst4_mat)]
    jmp .pass1_dst
.pass1_adst:
    lea                  r3, [o(adst4_mat)]
.pass1_dst:
    vpbroadcastd        m12, [o(pd_32)]
%if WIN64
    push                 r8
%endif
    lea                 r8d, [eobq+(18<<16)]
    test               eobb, 0x10 ; TX_CLASS_H
    cmovnz             eobd, r8d
%if WIN64
    pop                  r8
%endif
    sub                eobd, 26<<16
    jl .pass1_dst_fast
    mova                 m0, [cfq+32*0]
    mova                 m1, [cfq+32*1]
    mova                 m2, [cfq+32*2]
    mova                 m3, [cfq+32*3]
    call m(inv_txfm_add_16x4_8bpc).dst4
    REPX     {paddd x, m12}, m0, m4, m1, m5, m2, m6, m3, m7
    jmp .pass1_end
.pass2_flipddt:
    lea                  r3, [o(flipddt16_mat)]
    jmp .pass2_dst
.pass2_ddt:
    lea                  r3, [o(ddt16_mat)]
    jmp .pass2_dst
.pass2_flipadst:
    lea                  r3, [o(flipadst16_mat)]
    jmp .pass2_dst
.pass2_adst:
    lea                  r3, [o(adst16_mat)]
.pass2_dst:
    vpbroadcastd         m7, [o(pd_2048)]
    lea                  r6, [dsq*3]
    pxor                m10, m10
%if WIN64
    movaps       [cfq+16*3], xm13
%endif
    punpcklwd           m13, m0, m1 ;  0  2
    punpckhwd           m11, m0, m1 ;  1  3
    punpcklwd           m12, m2, m3 ;  4  6
    punpckhwd            m2, m3     ;  5  7
    test               eobd, eobd
    jl .pass2_dst_fast
    vextracti128        xm3, m13, 1 ;  8 10
    vextracti128        xm4, m11, 1 ;  9 11
    vextracti128        xm5, m12, 1 ; 12 14
    vextracti128        xm6, m2, 1  ; 13 15
    call .pass2_dst_4x4
    call .pass2_dst_4x4
    call .pass2_dst_4x4
    call .pass2_dst_4x4
%if WIN64
    movaps             xm13, [cfq+16*3]
%endif
    jmp m(inv_txfm_add_16x4_8bpc).pass2_end2
.pass2_dst_fast:
    call .pass2_dst_4x4_fast
    call .pass2_dst_4x4_fast
    call .pass2_dst_4x4_fast
    call .pass2_dst_4x4_fast
%if WIN64
    movaps             xm13, [cfq+16*3]
%endif
    jmp m(inv_txfm_add_16x4_8bpc).pass2_end2
ALIGN function_align
.pass2_dst_4x4:
    call .dst16x1
    add                  r3, 32
    mova         [cfq+16*0], xm0
    call .dst16x1
    add                  r3, 32
    mova         [cfq+16*1], xm0
    call .dst16x1
    add                  r3, 32
    mova         [cfq+16*2], xm0
    call .dst16x1
    add                  r3, 32
    vinserti128          m1, m0, [cfq+16*2], 1
    paddd                m0, m7, [cfq+32*0]
    jmp .pass2_dst_4x4b
.pass2_dst_4x4_fast:
    call .dst16x2
    vinserti128          m6, m0, xm1, 1
    call .dst16x2
    vinserti128          m1, xm0, 1
    paddd                m0, m7, m6
.pass2_dst_4x4b:
    paddd                m1, m7
    psrad                m0, 12
    psrad                m1, 12
    jmp m(inv_txfm_add_4x8_8bpc).write_4x4_dct
ALIGN function_align
.dst16x1:
    WRAP_XMM IDST16_1D_PACKED_1ROW
ALIGN function_align
.dst16x2:
    WRAP_XMM IDST16_1D_PACKED_2ROWS_FAST 1
    add                  r3, 32*2
    ret

INV_TXFM_FN 16, 4
    mova                xm0, [cfq+16*0]
    mova                xm1, [cfq+16*1]
    mova                xm2, [cfq+16*2]
    mova                xm3, [cfq+16*3]
    WIN64_SPILL_XMM      13
    jmp                tx1q

.dconly:
    movd                xm3, [o(pw_512)]
    or                  r4d, 2
.dconly2:
    pmulhrsw            xm3, [cfq]
.dconly3:
    vpbroadcastw         m3, xm3
    pxor                xm2, xm2
    mova              [cfq], xm2
.dconly_loop:
    movu                xm1, [dstq+dsq*0]
    vinserti128          m1, [dstq+dsq*1], 1
    punpcklbw            m0, m1, m2
    punpckhbw            m1, m2
    paddw                m0, m3
    paddw                m1, m3
    packuswb             m0, m1
    movu         [dstq+dsq*0], xm0
    vextracti128 [dstq+dsq*1], m0, 1
    lea                dstq, [dstq+dsq*2]
    dec                 r4d
    jg .dconly_loop
    vzeroupper
    ret

.pass1_dct:
    vpbroadcastd        m10, [o(pd_32)]
    lea                 r3d, [eobq+(3<<16)]
    test               eobb, 0x10 ; TX_CLASS_H
    cmovz              eobd, r3d
    cmp                eobd, 32<<16
    jl .pass1_dct_fast
    mova                xm4, [cfq+16*4]
    mova                xm5, [cfq+16*5]
    mova                xm6, [cfq+16*6]
    mova                xm7, [cfq+16*7]
    call m(inv_txfm_add_4x16_8bpc).dct16
    jmp .pass1_dct2
.pass1_dct_fast:
    call m(inv_txfm_add_4x16_8bpc).dct16_fast
.pass1_dct2:
    mova                 m6, [cfq+32*0]
    mova                 m5, [cfq+32*1]
    mova                 m4, [cfq+32*2]
    mova                 m8, [cfq+32*3]
    psubd                m7, m0, m6 ; out15 out14
    paddd                m0, m6     ; out0  out1
    psubd                m6, m1, m5 ; out12 out13
    paddd                m1, m5     ; out3  out2
    psubd                m5, m2, m4 ; out11 out10
    paddd                m2, m4     ; out4  out5
    psubd                m4, m3, m8 ; out8  out9
    paddd                m3, m8     ; out7  out6
    REPX       {psrad x, 6}, m0, m4, m1, m5
    packssdw             m0, m4
    packssdw             m1, m5
    REPX       {psrad x, 6}, m6, m2, m7, m3
    packssdw             m2, m6
    packssdw             m3, m7
    vpermq               m0, m0, q3120 ;  0  1    8  9
    vpermq               m1, m1, q1302 ;  2  3   10 11
    vpermq               m2, m2, q3120 ;  4  5   12 13
    vpermq               m3, m3, q1302 ;  6  7   14 15
.pass1_end:
    punpckhwd            m4, m0, m1
    punpcklwd            m0, m1
    punpckhwd            m1, m2, m3
    punpcklwd            m2, m3
    punpckhwd            m3, m0, m4
    punpcklwd            m0, m4
    punpckhwd            m4, m2, m1
    punpcklwd            m2, m1
    punpckhqdq           m1, m0, m2
    punpcklqdq           m0, m2
    punpcklqdq           m2, m3, m4
    punpckhqdq           m3, m4
    jmp                tx2q
.pass2_dct:
    vpbroadcastd        m10, [o(pd_2048)]
    call .dct4
    REPX      {psrad x, 12}, m0, m4, m1, m5
    packssdw             m0, m4
    packssdw             m1, m5
    call .write_16x2
    REPX      {psrad x, 12}, m2, m6, m3, m7
    packssdw             m0, m2, m6
    packssdw             m1, m3, m7
.pass2_end:
    call .write_16x2
.pass2_end2:
    pxor                m10, m10
.pass2_end3:
    REPX {mova [cfq+32*x], m10}, 0, 1, 2, 3
    RET
ALIGN function_align
.write_16x2_hdpcm:
    pslldq               m8, m0, 2
    pslldq               m9, m1, 2
    paddw                m0, m8
    paddw                m1, m9
    pslldq               m8, m0, 4
    pslldq               m9, m1, 4
    paddw                m0, m8
    paddw                m1, m9
    pslldq               m8, m0, 8
    pslldq               m9, m1, 8
    paddw                m0, m8
    paddw                m1, m9
    pshufhw             xm8, xm0, q3333
    pshufhw             xm9, xm1, q3333
    vpermq               m8, m8, q1133
    vpermq               m9, m9, q1133
    paddw                m0, m8
    paddw                m1, m9
.write_16x2:
    pmovzxbw             m8, [dstq+dsq*0]
    pmovzxbw             m9, [dstq+dsq*1]
    paddw                m8, m0
    paddw                m9, m1
    packuswb             m8, m9
    vpermq               m8, m8, q3120
    movu         [dstq+dsq*0], xm8
    vextracti128 [dstq+dsq*1], m8, 1
    lea                dstq, [dstq+dsq*2]
    ret
ALIGN function_align
.dct4:
    punpcklwd            m7, m0, m2 ; 0 2
    vpbroadcastd         m6, [o(pw_64_64)]
    punpckhwd            m0, m2
    vpbroadcastd         m8, [o(pw_64_m64)]
    punpckhwd            m5, m3, m1 ; 3 1
    vpbroadcastd         m4, [o(pw_35_83)]
    punpcklwd            m1, m3, m1
    vpbroadcastd         m3, [o(pw_m83_35)]
    pmaddwd              m2, m6, m7 ; a0
    pmaddwd              m6, m0
    pmaddwd              m7, m8     ; a1
    pmaddwd              m8, m0
    pmaddwd              m0, m4, m1 ; b0
    pmaddwd              m4, m5
    pmaddwd              m1, m3     ; b1
    pmaddwd              m5, m3
    REPX     {paddd x, m10}, m2, m6, m7, m8
    psubd                m3, m2, m0 ; out3a
    paddd                m0, m2     ; out0a
    psubd                m2, m7, m1 ; out2a
    paddd                m1, m7     ; out1a
    psubd                m7, m6, m4 ; out3b
    paddd                m4, m6     ; out0b
    psubd                m6, m8, m5 ; out2b
    paddd                m5, m8     ; out1b
    ret

.pass1_identity:
    vinserti128          m0, [cfq+16*4], 1 ;  0  1    8  9
    vinserti128          m1, [cfq+16*5], 1 ;  2  3   10 11
    vinserti128          m2, [cfq+16*6], 1 ;  4  5   12 13
    vinserti128          m3, [cfq+16*7], 1 ;  6  7   14 15
    REPX      {paddsw x, x}, m0, m1, m2, m3, m0, m1, m2, m3
    jmp .pass1_end
.pass2_identity:
    vpbroadcastd         m4, [o(pw_1024)]
    pmulhrsw             m0, m4
    pmulhrsw             m1, m4
    test               eobd, 0x300
    jz .pass2_dst_end
    test               eobd, 0x200
    jnz .vdpcm
.hdpcm:
    call .write_16x2_hdpcm
    pmulhrsw             m0, m4, m2
    pmulhrsw             m1, m4, m3
    call .write_16x2_hdpcm
    jmp .pass2_end2
.vdpcm:
    pmulhrsw             m2, m4
    pmulhrsw             m3, m4
    paddw                m1, m0
    call .write_16x2
    paddw                m0, m2, m1
    paddw                m1, m3, m0
    jmp .pass2_end

.pass1_flipddt:
    lea                  r3, [o(flipddt16_mat+32*6)]
    jmp .pass1_dst
.pass1_ddt:
    lea                  r3, [o(ddt16_mat+32*6)]
    jmp .pass1_dst
.pass1_flipadst:
    lea                  r3, [o(flipadst16_mat+32*6)]
    jmp .pass1_dst
.pass1_adst:
    lea                  r3, [o(adst16_mat+32*6)]
.pass1_dst:
    vpbroadcastd         m7, [o(pd_32)]
%if WIN64
    movaps             xm10, xm13
    push                 r8
    %define             tmp  rsp+8
%else
    %define             tmp  rsp-24
%endif
    punpcklwd          xm13, xm0, xm1 ;  0  2
    punpckhwd          xm11, xm0, xm1 ;  1  3
    punpcklwd          xm12, xm2, xm3 ;  4  6
    punpckhwd           xm2, xm3      ;  5  7
    lea                 r8d, [eobq+(3<<16)]
    test               eobb, 0x10 ; TX_CLASS_H
    cmovz              eobd, r8d
    mov                 r8d, 32*3
    cmp                eobd, 32<<16
    jl .pass1_dst_fast
    mova                xm4, [cfq+16*4]
    mova                xm5, [cfq+16*5]
    mova                xm6, [cfq+16*6]
    mova                xm0, [cfq+16*7]
    punpcklwd           xm3, xm4, xm5 ;  8 10
    punpckhwd           xm4, xm5      ;  9 11
    punpcklwd           xm5, xm6, xm0 ; 12 14
    punpckhwd           xm6, xm0      ; 13 15
.pass1_dst_loop:
    call m(inv_txfm_add_4x16_8bpc).dst16x1 ;  6
    add                  r3, 32*8
    mova         [cfq+16*0], xm0
    call m(inv_txfm_add_4x16_8bpc).dst16x1 ; 14
    add                  r3, 32*1
    mova         [cfq+16*1], xm0
    call m(inv_txfm_add_4x16_8bpc).dst16x1 ; 15
    sub                  r3, 32*8
    mova              [tmp], xm0
    call m(inv_txfm_add_4x16_8bpc).dst16x1 ;  7
    sub                  r3, 32*3
    vinserti128          m1, m0, [tmp], 1
    paddd                m0, m7, [cfq]
    paddd                m1, m7
    psrad                m0, 6  ;  6 14
    psrad                m1, 6  ;  7 15
    packssdw             m0, m1
    mova           [cfq+r8], m0
    sub                 r8d, 32
    jge .pass1_dst_loop
    jmp .pass1_dst_end
.pass1_dst_fast:
    call .dst16x2_fast
    add                  r3, 32
    vinserti128          m6, m0, xm1, 1
    call .dst16x2_fast
    sub                  r3, 32*3
    vinserti128          m1, m0, xm1, 1
    paddd                m0, m7, m6
    paddd                m1, m7
    psrad                m0, 6  ;  6 14
    psrad                m1, 6  ;  7 15
    packssdw             m0, m1
    mova           [cfq+r8], m0
    sub                 r8d, 32
    jge .pass1_dst_fast
.pass1_dst_end:
    mova                 m1, [cfq+32*1]
    mova                 m2, [cfq+32*2]
    mova                 m3, [cfq+32*3]
%if WIN64
    movaps             xm13, xm10
    pop                  r8
%endif
    jmp .pass1_end
.pass2_flipadst:
    lea                  r3, [o(flipadst4_mat)]
    jmp .pass2_dst
.pass2_adst:
    lea                  r3, [o(adst4_mat)]
.pass2_dst:
    call .dst4
    call m(inv_txfm_add_8x8_8bpc).shift7_pack
    vpbroadcastd         m4, [o(pw_1024)]
    pmulhrsw             m0, m4
    pmulhrsw             m1, m4
.pass2_dst_end:
    call .write_16x2
    pmulhrsw             m0, m4, m2
    pmulhrsw             m1, m4, m3
    jmp .pass2_end
ALIGN function_align
.dst16x2_fast:
    WRAP_XMM IDST16_1D_PACKED_2ROWS_FAST 8
    ret
ALIGN function_align
.dst4:
    vpbroadcastd         m5, [r3+4*0]
    punpcklwd            m8, m0, m2 ; 0 2
    vpbroadcastd         m7, [r3+4*3]
    punpckhwd            m9, m0, m2
    vpbroadcastd        m10, [r3+4*1]
    punpcklwd            m2, m3, m1 ; 3 1
    vpbroadcastd        m11, [r3+4*2]
    punpckhwd            m6, m3, m1
    pmaddwd              m0, m5, m8
    pmaddwd              m3, m7, m2
    pmaddwd              m4, m5, m9
    pmaddwd              m1, m7, m6
    paddd                m0, m3     ; out0a
    paddd                m4, m1     ; out0b
    pmaddwd              m3, m7, m8
    pmaddwd              m1, m5, m2
    pmaddwd              m7, m9
    pmaddwd              m5, m6
    psubd                m3, m1     ; out3a
    psubd                m7, m5     ; out3b
    pmaddwd              m1, m10, m8
    pmaddwd              m5, m11, m2
    pmaddwd              m8, m11
    pmaddwd              m2, m10
    psubd                m1, m5     ; out1a
    paddd                m2, m8     ; out2a
    pmaddwd              m5, m10, m9
    pmaddwd              m9, m11
    pmaddwd             m11, m6
    pmaddwd              m6, m10
    psubd                m5, m11    ; out1b
    paddd                m6, m9     ; out2b
    ret
