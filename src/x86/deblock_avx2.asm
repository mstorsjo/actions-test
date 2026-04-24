; Copyright © 2018-2026, VideoLAN and dav2d authors
; Copyright © 2018-2026, Two Orioles, LLC
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

SECTION_RODATA 32

pd_mask: dd 1, 2, 4, 8, 16, 32, 64, 128

rev_shuf: times 2 db 2, 3, 0, 1, 6, 7, 4, 5, 10, 11, 8, 9, 14, 15, 12, 13
side_shuf_zxbw: db 0, -1, 4, -1, 8, -1, 12, -1, 3, -1, 7, -1, 11, -1, 15, -1

qthr_mul_lut: dw  0, 32, 25, 19, 19, 18, 17,  0

base_mul_lut:        dw 0, 1360,   816,   592,   448,   320,   240, 0
base_mul_lut_y_edge: dw 0, 1360,   816,   592,   448,   320,   320, 0
mul1_lut:            dw 0, 1360, 816*2, 592*3, 448*4, 320*6, 240*8, 0
mul2_lut:            dw 0,    0, 816*1, 592*2, 448*3, 320*5, 240*7, 0
mul3_lut:            dw 0,    0,     0, 592*1, 448*2, 320*4, 240*6, 0
mul4_lut:            dw 0,    0,     0,     0, 448*1, 320*3, 240*5, 0
mul5_lut:            dw 0,    0,     0,     0,     0, 320*2, 240*4, 0
mul6_lut:            dw 0,    0,     0,     0,     0, 320*1, 240*3, 0
mul7_lut:            dw 0,    0,     0,     0,     0,     0, 240*2, 0
mul8_lut:            dw 0,    0,     0,     0,     0,     0, 240*1, 0
mul1_lut_uv_edge:    dw 0, 1360, 816*2, 816*2, 816*2, 816*2, 816*2, 0
mul2_lut_uv_edge:    dw 0,    0, 816*1, 816*1, 816*1, 816*1, 816*1, 0
mul1_lut_y_edge:     dw 0, 1360, 816*2, 592*3, 448*4, 320*6, 320*6, 0

side_shuf: db 0, 4, 8, 12, 3, 7, 11, 15

mul_lut_offset: times 2 db 0, 1

pb_12_m4: times 2 db 12, -4
pb_1_m2: times 2 db 1, -2
pb_m2_1: times 2 db -2, 1
pb_m2_3: times 2 db -2, 3
pb_m3_4: times 2 db -3, 4
pb_m5_6: times 2 db -5, 6
pb_m6_7: times 2 db -6, 7
pb_128: times 4 db 0x80

pw_12288: times 2 dw 12288
pw_24576: times 2 dw 24576

pw_1360: times 2 dw 1360
pw_4_3: dw 4, 3
pw_45: dw 45, 45
pw_45_40: dw 45, 40

SECTION .text

%macro TRANSPOSE_16x2_AND_WRITE_2x32 3
    ; transpose 16x2
    punpcklbw    m%3, m%1, m%2
    punpckhbw    m%1, m%2

    ; write out
    pextrw [dstq+strideq*0-1], xm%3, 0
    pextrw [dstq+strideq*1-1], xm%3, 1
    pextrw [dstq+strideq*2-1], xm%3, 2
    pextrw [dstq+stride3q-1], xm%3, 3
    lea         dstq, [dstq+strideq*4-1]
    pextrw [dstq+strideq*0], xm%3, 4
    pextrw [dstq+strideq*1], xm%3, 5
    pextrw [dstq+strideq*2], xm%3, 6
    pextrw [dstq+stride3q], xm%3, 7
    lea         dstq, [dstq+strideq*4]
    pextrw [dstq+strideq*0], xm%1, 0
    pextrw [dstq+strideq*1], xm%1, 1
    pextrw [dstq+strideq*2], xm%1, 2
    pextrw [dstq+stride3q], xm%1, 3
    lea         dstq, [dstq+strideq*4]
    pextrw [dstq+strideq*0], xm%1, 4
    pextrw [dstq+strideq*1], xm%1, 5
    pextrw [dstq+strideq*2], xm%1, 6
    pextrw [dstq+stride3q], xm%1, 7
    lea         dstq, [dstq+strideq*4]

    vextracti128 xm%3, m%3, 1
    vextracti128 xm%1, m%1, 1
    pextrw [dstq+strideq*0], xm%3, 0
    pextrw [dstq+strideq*1], xm%3, 1
    pextrw [dstq+strideq*2], xm%3, 2
    pextrw [dstq+stride3q], xm%3, 3
    lea         dstq, [dstq+strideq*4]
    pextrw [dstq+strideq*0], xm%3, 4
    pextrw [dstq+strideq*1], xm%3, 5
    pextrw [dstq+strideq*2], xm%3, 6
    pextrw [dstq+stride3q], xm%3, 7
    lea         dstq, [dstq+strideq*4]
    pextrw [dstq+strideq*0], xm%1, 0
    pextrw [dstq+strideq*1], xm%1, 1
    pextrw [dstq+strideq*2], xm%1, 2
    pextrw [dstq+stride3q], xm%1, 3
    lea         dstq, [dstq+strideq*4]
    pextrw [dstq+strideq*0], xm%1, 4
    pextrw [dstq+strideq*1], xm%1, 5
    pextrw [dstq+strideq*2], xm%1, 6
    pextrw [dstq+stride3q], xm%1, 7
    lea         dstq, [dstq+strideq*4+1]
%endmacro

%macro TRANSPOSE_16x8_AND_WRITE_8x32 9
    ; 16x8 transpose
    punpcklbw    m%9, m%1, m%2
    punpckhbw    m%1, m%2
    punpcklbw    m%2, m%3, m%4
    punpckhbw    m%3, m%4
    punpcklbw    m%4, m%5, m%6
    punpckhbw    m%5, m%6
    punpcklbw    m%6, m%7, m%8
    punpckhbw    m%7, m%8

    punpcklwd    m%8, m%9, m%2
    punpckhwd    m%9, m%2
    punpcklwd    m%2, m%1, m%3
    punpckhwd    m%1, m%3
    punpcklwd    m%3, m%4, m%6
    punpckhwd    m%4, m%6
    punpcklwd    m%6, m%5, m%7
    punpckhwd    m%5, m%7

    punpckldq    m%7, m%8, m%3
    punpckhdq    m%8, m%3
    punpckldq    m%3, m%9, m%4
    punpckhdq    m%9, m%4
    punpckldq    m%4, m%2, m%6
    punpckhdq    m%2, m%6
    punpckldq    m%6, m%1, m%5
    punpckhdq    m%1, m%5

    ; write 8x32
    movq   [dstq+strideq*0-4], xm%7
    movhps [dstq+strideq*1-4], xm%7
    movq   [dstq+strideq*2-4], xm%8
    movhps [dstq+stride3q -4], xm%8
    lea         dstq, [dstq+strideq*4-4]
    movq   [dstq+strideq*0], xm%3
    movhps [dstq+strideq*1], xm%3
    movq   [dstq+strideq*2], xm%9
    movhps [dstq+stride3q ], xm%9
    lea         dstq, [dstq+strideq*4]
    movq   [dstq+strideq*0], xm%4
    movhps [dstq+strideq*1], xm%4
    movq   [dstq+strideq*2], xm%2
    movhps [dstq+stride3q ], xm%2
    lea         dstq, [dstq+strideq*4]
    movq   [dstq+strideq*0], xm%6
    movhps [dstq+strideq*1], xm%6
    movq   [dstq+strideq*2], xm%1
    movhps [dstq+stride3q ], xm%1
    lea         dstq, [dstq+strideq*4]

    vextracti128 xm%7, m%7, 1
    vextracti128 xm%8, m%8, 1
    vextracti128 xm%3, m%3, 1
    vextracti128 xm%9, m%9, 1
    vextracti128 xm%4, m%4, 1
    vextracti128 xm%2, m%2, 1
    vextracti128 xm%6, m%6, 1
    vextracti128 xm%1, m%1, 1

    movq   [dstq+strideq*0], xm%7
    movhps [dstq+strideq*1], xm%7
    movq   [dstq+strideq*2], xm%8
    movhps [dstq+stride3q ], xm%8
    lea         dstq, [dstq+strideq*4]
    movq   [dstq+strideq*0], xm%3
    movhps [dstq+strideq*1], xm%3
    movq   [dstq+strideq*2], xm%9
    movhps [dstq+stride3q ], xm%9
    lea         dstq, [dstq+strideq*4]
    movq   [dstq+strideq*0], xm%4
    movhps [dstq+strideq*1], xm%4
    movq   [dstq+strideq*2], xm%2
    movhps [dstq+stride3q ], xm%2
    lea         dstq, [dstq+strideq*4]
    movq   [dstq+strideq*0], xm%6
    movhps [dstq+strideq*1], xm%6
    movq   [dstq+strideq*2], xm%1
    movhps [dstq+stride3q ], xm%1
    lea         dstq, [dstq+strideq*4+4]
%endmacro

%macro TRANSPOSE_16X16B 3 ; in_load_15_from_mem, out_store_0_in_mem, mem
%if %1 == 0
    mova          %3, m15
%endif

    ; input in m0-15
    punpcklbw    m15, m0, m1
    punpckhbw     m0, m1
    punpcklbw     m1, m2, m3
    punpckhbw     m2, m3
    punpcklbw     m3, m4, m5
    punpckhbw     m4, m5
    punpcklbw     m5, m6, m7
    punpckhbw     m6, m7
    punpcklbw     m7, m8, m9
    punpckhbw     m8, m9
    punpcklbw     m9, m10, m11
    punpckhbw    m10, m11
    punpcklbw    m11, m12, m13
    punpckhbw    m12, m13
    mova         m13, %3
    mova          %3, m12
    punpcklbw    m12, m14, m13
    punpckhbw    m13, m14, m13

    ; interleaved in m15,0,1,2,3,4,5,6,7,8,9,10,11,rsp%3,12,13
    punpcklwd    m14, m15, m1
    punpckhwd    m15, m1
    punpcklwd     m1, m0, m2
    punpckhwd     m0, m2
    punpcklwd     m2, m3, m5
    punpckhwd     m3, m5
    punpcklwd     m5, m4, m6
    punpckhwd     m4, m6
    punpcklwd     m6, m7, m9
    punpckhwd     m7, m9
    punpcklwd     m9, m8, m10
    punpckhwd     m8, m10
    punpcklwd    m10, m11, m12
    punpckhwd    m11, m12
    mova         m12, %3
    mova          %3, m11
    punpcklwd    m11, m12, m13
    punpckhwd    m12, m13

    ; interleaved in m14,15,1,0,2,3,5,4,6,7,9,8,10,rsp%3,11,12
    punpckldq    m13, m14, m2
    punpckhdq    m14, m2
    punpckldq     m2, m15, m3
    punpckhdq    m15, m3
    punpckldq     m3, m1, m5
    punpckhdq     m1, m5
    punpckldq     m5, m0, m4
    punpckhdq     m0, m4
    punpckldq     m4, m6, m10
    punpckhdq     m6, m10
    punpckldq    m10, m9, m11
    punpckhdq     m9, m11
    punpckldq    m11, m8, m12
    punpckhdq     m8, m12
    mova         m12, %3
    mova          %3, m8
    punpckldq     m8, m7, m12
    punpckhdq     m7, m12

    ; interleaved in m13,14,2,15,3,1,5,0,4,6,8,7,10,9,11,rsp%3
    punpcklqdq   m12, m13, m4
    punpckhqdq   m13, m4
    punpcklqdq    m4, m14, m6
    punpckhqdq   m14, m6
    punpcklqdq    m6, m2, m8
    punpckhqdq    m2, m8
    punpcklqdq    m8, m15, m7
    punpckhqdq   m15, m7
    punpcklqdq    m7, m3, m10
    punpckhqdq    m3, m10
    punpcklqdq   m10, m1, m9
    punpckhqdq    m1, m9
    punpcklqdq    m9, m5, m11
    punpckhqdq    m5, m11
    mova         m11, %3
    mova          %3, m12
    punpcklqdq   m12, m0, m11
    punpckhqdq    m0, m11
%if %2 == 0
    mova         m11, %3
%endif

    ; interleaved m11,13,4,14,6,2,8,15,7,3,10,1,9,5,12,0
    SWAP          0, 11, 1, 13, 5, 2, 4, 6, 8, 7, 15
    SWAP          3, 14, 12, 9
%endmacro

%macro STORE_16X16 0
    movu [dstq+strideq*0-8], xm0
    movu [dstq+strideq*1-8], xm1
    movu [dstq+strideq*2-8], xm2
    movu [dstq+stride3q -8], xm3
    lea         dstq, [dstq+strideq*4-8]
    movu [dstq+strideq*0], xm4
    movu [dstq+strideq*1], xm5
    movu [dstq+strideq*2], xm6
    movu [dstq+stride3q ], xm7
    lea         dstq, [dstq+strideq*4]
    movu [dstq+strideq*0], xm8
    movu [dstq+strideq*1], xm9
    movu [dstq+strideq*2], xm10
    movu [dstq+stride3q ], xm11
    lea         dstq, [dstq+strideq*4]
    movu [dstq+strideq*0], xm12
    movu [dstq+strideq*1], xm13
    movu [dstq+strideq*2], xm14
    movu [dstq+stride3q ], xm15
    lea         dstq, [dstq+strideq*4]
    vextracti128 [dstq+strideq*0], m0, 1
    vextracti128 [dstq+strideq*1], m1, 1
    vextracti128 [dstq+strideq*2], m2, 1
    vextracti128 [dstq+stride3q ], m3, 1
    lea         dstq, [dstq+strideq*4]
    vextracti128 [dstq+strideq*0], m4, 1
    vextracti128 [dstq+strideq*1], m5, 1
    vextracti128 [dstq+strideq*2], m6, 1
    vextracti128 [dstq+stride3q ], m7, 1
    lea         dstq, [dstq+strideq*4]
    vextracti128 [dstq+strideq*0], m8, 1
    vextracti128 [dstq+strideq*1], m9, 1
    vextracti128 [dstq+strideq*2], m10, 1
    vextracti128 [dstq+stride3q ], m11, 1
    lea         dstq, [dstq+strideq*4]
    vextracti128 [dstq+strideq*0], m12, 1
    vextracti128 [dstq+strideq*1], m13, 1
    vextracti128 [dstq+strideq*2], m14, 1
    vextracti128 [dstq+stride3q ], m15, 1
    lea         dstq, [dstq+strideq*4+8]
%endmacro

; Compute:
;   pos_pix -= diff
;   neg_pix += diff
; except when masked out by the associated lossless mask
; diff_lo/hi should be within [-128, 127]. This can be used for everything,
; but the first line of pixels. This can be verified by plugging in the max
; values of q_thr into (=>) the clamp for delta_m2 => delta_m2 => diff.
; For context, the max q_thr is 120 with bitdepth of 8.
%macro SUB_ADD_DIFF 6 ; pos_pix, neg_pix, diff_lo, diff_hi,
                      ; pos_ll_mask, neg_ll_mask
    packsswb         %3, %4
    pxor             %1, m15
    pxor             %2, m15
    pandn            %4, %5, %3
    pandn            %3, %6, %3
    psubsb           %1, %4
    paddsb           %2, %3
    pxor             %1, m15
    pxor             %2, m15
%endmacro

; Perform the same operation as above, but with support for larger diffs.
; Also, no masking and doesn't modify pixels in place.
%macro SUB_ADD_DIFF_LARGE 7 ; pos_dst, neg_dst, pos_src, neg_src,
                            ; diff_lo, diff_hi, tmp
    punpcklbw        %1, %3, m15
    punpckhbw        %2, %3, m15
    psubw            %1, %5
    psubw            %2, %6
    packuswb         %1, %2

    punpcklbw        %2, %4, m15
    punpckhbw        %7, %4, m15
    paddw            %2, %5
    paddw            %7, %6
    packuswb         %2, %7
%endmacro

%macro FILTER 3 ; width [1/3/4/6/8], dir [h/v], is_edge [0,1]
%assign is_edge %3
%assign is_chroma_edge is_edge && (%1 == 3 || %1 == 4)
    ; load data
%ifidn %2, v
    ; load 6-8 pixels, remainder will be read inline
    lea         tmpq, [dstq+mstrideq*4]
    mova          m0, [dstq+strideq*0] ;  0
    mova          m1, [tmpq+stride3q]  ; -1
    mova          m2, [dstq+strideq*1] ;  1
    mova          m3, [tmpq+strideq*2] ; -2
    mova          m4, [dstq+strideq*2] ;  2
    mova          m5, [tmpq+strideq*1] ; -3
%if %1 >= 3
    mova          m6, [dstq+stride3q]  ;  3
%if !is_chroma_edge
    mova          m7, [tmpq+strideq*0] ; -4
%endif
%endif
%else
    ; load lines
%if %1 <= 3
    ; TODO: for w == 1, we need 6 cols for rows 0,3, 4,7, 8,11 but, we only
    ;       need the 4 cols of rows 1,2, 5,6, 9,10... Optimize the transpose
    ;       for this
%assign off %1 == 1 ? 3 : %1 + 1
    movq         xm3, [dstq+strideq*0-off]
    movq         xm4, [dstq+strideq*1-off]
    movq         xm5, [dstq+strideq*2-off]
    movq         xm6, [dstq+stride3q -off]
    lea         tmpq, [dstq+strideq*8-off]
    movhps       xm3, [tmpq+strideq*0]
    movhps       xm4, [tmpq+strideq*1]
    movhps       xm5, [tmpq+strideq*2]
    movhps       xm6, [tmpq+stride3q ]
    lea         tmpq, [tmpq+strideq*8]
    movq         xm7, [tmpq+strideq*0]
    movq         xm8, [tmpq+strideq*1]
    movq         xm9, [tmpq+strideq*2]
    movq        xm11, [tmpq+stride3q ]
    lea         tmpq, [tmpq+strideq*8]
    movhps       xm7, [tmpq+strideq*0]
    movhps       xm8, [tmpq+strideq*1]
    movhps       xm9, [tmpq+strideq*2]
    movhps      xm11, [tmpq+stride3q ]
    vinserti128   m3, xm7, 1
    vinserti128   m4, xm8, 1
    vinserti128   m5, xm9, 1
    vinserti128   m6, xm11, 1
    lea         tmpq, [dstq+strideq*4-off]
    movq        xm12, [tmpq+strideq*0]
    movq        xm13, [tmpq+strideq*1]
    movq        xm14, [tmpq+strideq*2]
    movq        xm15, [tmpq+stride3q ]
    lea         tmpq, [tmpq+strideq*8]
    movhps      xm12, [tmpq+strideq*0]
    movhps      xm13, [tmpq+strideq*1]
    movhps      xm14, [tmpq+strideq*2]
    movhps      xm15, [tmpq+stride3q ]
    lea         tmpq, [tmpq+strideq*8]
    movq         xm7, [tmpq+strideq*0]
    movq         xm8, [tmpq+strideq*1]
    movq         xm9, [tmpq+strideq*2]
    movq        xm11, [tmpq+stride3q ]
    lea         tmpq, [tmpq+strideq*8]
    movhps       xm7, [tmpq+strideq*0]
    movhps       xm8, [tmpq+strideq*1]
    movhps       xm9, [tmpq+strideq*2]
    movhps      xm11, [tmpq+stride3q ]
    vinserti128  m12, xm7, 1
    vinserti128  m13, xm8, 1
    vinserti128  m14, xm9, 1
    vinserti128  m15, xm11, 1

    ; transpose 8x16
    ; xm3: A-H0,A-H8
    ; xm4: A-H1,A-H9
    ; xm5: A-H2,A-H10
    ; xm6: A-H3,A-H11
    ; xm12: A-H4,A-H12
    ; xm13: A-H5,A-H13
    ; xm14: A-H6,A-H14
    ; xm15: A-H7,A-H15
    punpcklbw    m7, m3, m4
    punpckhbw    m3, m4
    punpcklbw    m4, m5, m6
    punpckhbw    m5, m6
    punpcklbw    m6, m12, m13
    punpckhbw   m12, m13
    punpcklbw   m13, m14, m15
    punpckhbw   m14, m15
    ; xm7: A0-1,B0-1,C0-1,D0-1,E0-1,F0-1,G0-1,H0-1
    ; xm3: A8-9,B8-9,C8-9,D8-9,E8-9,F8-9,G8-9,H8-9
    ; xm4: A2-3,B2-3,C2-3,D2-3,E2-3,F2-3,G2-3,H2-3
    ; xm5: A10-11,B10-11,C10-11,D10-11,E10-11,F10-11,G10-11,H10-11
    ; xm6: A4-5,B4-5,C4-5,D4-5,E4-5,F4-5,G4-5,H4-5
    ; xm12: A12-13,B12-13,C12-13,D12-13,E12-13,F12-13,G12-13,H12-13
    ; xm13: A6-7,B6-7,C6-7,D6-7,E6-7,F6-7,G6-7,H6-7
    ; xm14: A14-15,B14-15,C14-15,D14-15,E14-15,F14-15,G14-15,H14-15
    punpcklwd   m15, m7, m4
    punpckhwd    m7, m4
    punpcklwd    m4, m3, m5
    punpckhwd    m3, m5
    punpcklwd    m5, m6, m13
    punpckhwd    m6, m13
    punpcklwd   m13, m12, m14
    punpckhwd   m12, m14
    ; xm15: A0-3,B0-3,C0-3,D0-3
    ; xm7: E0-3,F0-3,G0-3,H0-3
    ; xm4: A8-11,B8-11,C8-11,D8-11
    ; xm3: E8-11,F8-11,G8-11,H8-11
    ; xm5: A4-7,B4-7,C4-7,D4-7
    ; xm6: E4-7,F4-7,G4-7,H4-7
    ; xm13: A12-15,B12-15,C12-15,D12-15
    ; xm12: E12-15,F12-15,G12-15,H12-15
    punpckldq   m14, m15, m5
    punpckhdq   m15, m5
    punpckldq    m5, m7, m6
%if %1 == 3
    punpckhdq    m7, m6
%endif
    punpckldq    m6, m4, m13
    punpckhdq    m4, m13
    punpckldq   m13, m3, m12
%if %1 == 3
    punpckhdq   m12, m3, m12
%endif
    ; xm14: A0-7,B0-7
    ; xm15: C0-7,D0-7
    ; xm5: E0-7,F0-7
    ; xm7: G0-7,H0-7
    ; xm6: A8-15,B8-15
    ; xm4: C8-15,D8-15
    ; xm13: E8-15,F8-15
    ; xm12: G8-15,H8-15
    punpcklqdq   m3, m14, m6
    punpckhqdq  m14, m6
    punpckhqdq   m6, m15, m4
    punpcklqdq  m15, m4
    punpcklqdq   m4, m5, m13
    punpckhqdq  m13, m5, m13
%if %1 == 3
    punpcklqdq   m5, m7, m12
    punpckhqdq  m12, m7, m12
    ; xm3: A0-15
    ; xm14: B0-15
    ; xm15: C0-15
    ; xm6: D0-15
    ; xm4: E0-15
    ; xm13: F0-15
    ; xm5: G0-15
    ; xm12: H0-15
    SWAP          2, 13
    SWAP          6, 12, 1
    SWAP          3, 15, 7
    SWAP          4, 5, 14, 0
    ; 3,14,15,6,4,13,5,12 -> 7,5,3,1,0,2,4,6
    mova [rsp+7*32], m7
    mova [rsp+6*32], m6
%else
    SWAP          0, 6
    SWAP          1, 15
    SWAP          4, 13, 2
    SWAP          3, 14, 5
    ; 3,14,15,6,4,13 -> 5,3,1,0,2,4
%endif
%else
    ; load and 16x16 transpose. Don't always use all pixels but we'll need the
    ; remainder at the end for the second transpose
    ; TODO: w == 4 only requires 8 pixels for output and could optimize this
    ;       transpose
    movu         xm0, [dstq+strideq*0-8]
    movu         xm1, [dstq+strideq*1-8]
    movu         xm2, [dstq+strideq*2-8]
    movu         xm3, [dstq+stride3q -8]
    lea         tmpq, [dstq+strideq*4-8]
    movu         xm4, [tmpq+strideq*0]
    movu         xm5, [tmpq+strideq*1]
    movu         xm6, [tmpq+strideq*2]
    movu         xm7, [tmpq+stride3q ]
    lea         tmpq, [tmpq+strideq*4]
    movu         xm8, [tmpq+strideq*0]
    movu         xm9, [tmpq+strideq*1]
    movu        xm10, [tmpq+strideq*2]
    movu        xm11, [tmpq+stride3q ]
    lea         tmpq, [tmpq+strideq*4]
    movu        xm12, [tmpq+strideq*0]
    movu        xm13, [tmpq+strideq*1]
    movu        xm14, [tmpq+strideq*2]
    movu        xm15, [tmpq+stride3q ]
    lea         tmpq, [tmpq+strideq*4]
    vinserti128   m0, [tmpq+strideq*0], 1
    vinserti128   m1, [tmpq+strideq*1], 1
    vinserti128   m2, [tmpq+strideq*2], 1
    vinserti128   m3, [tmpq+stride3q ], 1
    lea         tmpq, [tmpq+strideq*4]
    vinserti128   m4, [tmpq+strideq*0], 1
    vinserti128   m5, [tmpq+strideq*1], 1
    vinserti128   m6, [tmpq+strideq*2], 1
    vinserti128   m7, [tmpq+stride3q ], 1
    lea         tmpq, [tmpq+strideq*4]
    vinserti128   m8, [tmpq+strideq*0], 1
    vinserti128   m9, [tmpq+strideq*1], 1
    vinserti128  m10, [tmpq+strideq*2], 1
    vinserti128  m11, [tmpq+stride3q ], 1
    lea         tmpq, [tmpq+strideq*4]
    vinserti128  m12, [tmpq+strideq*0], 1
    vinserti128  m13, [tmpq+strideq*1], 1
    vinserti128  m14, [tmpq+strideq*2], 1
    vinserti128  m15, [tmpq+stride3q ], 1

%if %1 >= 6
    TRANSPOSE_16X16B 0, 1, [rsp+15*32]
    mova  [rsp+13*32], m1
    mova  [rsp+11*32], m2
    mova  [rsp+9*32], m3
    mova  [rsp+7*32], m4
    mova  [rsp+6*32], m11
    mova  [rsp+8*32], m12
    mova  [rsp+10*32], m13
    mova  [rsp+12*32], m14
    mova  [rsp+14*32], m15
%else
    TRANSPOSE_16X16B 0, 0, [rsp+0*32]
    mova  [rsp+9*32], m3
    mova  [rsp+7*32], m4
    mova  [rsp+6*32], m11
    mova  [rsp+8*32], m12
%endif
    ; 4,5,6,7,8,9,10,11 -> 7,5,3,1,0,2,4,6
    SWAP            0, 8
    SWAP            2, 9
    SWAP            6, 11, 3
    SWAP            7, 4, 10, 1
%endif
%endif

    ; Compute derivatives and transition

    ; To select filter length, only the sides of each x4 segment are needed.
    ;   i.e.    0 _ _ 3 4 _ _ 7 8 _ _ 11 12 _ _ 15 | 16 _ _ ...
    vpbroadcastq     m14, [side_shuf]
    vbroadcasti128   m15, [side_shuf_zxbw]
    ; s[0-3][0], t[0-3][0]
    ;   where s and t are the respective sides
    pshufb           m12, m0, m14
    ; s[0-3][1], t[0-3][1]
    pshufb           m11, m2, m14
    ; lo 64-bit: s[0], s[1]
    ; hi 64-bit: t[0], t[1]
    punpcklbw        m12, m11

    ; s[0-3][-1], t[0-3][-1]
    pshufb           m13, m1, m14
    ; s[0-3][-2], t[0-3][-2]
    pshufb           m11, m3, m14
    ; lo 64-bit: s[-1], s[-2]
    ; hi 64-bit: t[-1], t[-2]
    punpcklbw        m13, m11

    ; Compute the second derivative and average the results for each side
    vpbroadcastd     m14, [pb_1_m2]
    pmaddubsw        m10, m12, m14
    pmaddubsw        m14, m13, m14
    pshufb            m8, m4, m15
    pshufb            m9, m5, m15
    paddw             m8, m10
    paddw             m9, m14
    ; lo: abs(s[0]  - 2 * s[1]  + s[2])
    ; hi: abs(t[0]  - 2 * t[1]  + t[2])
    pabsw             m8, m8
    ; lo: abs(s[-1] - 2 * s[-2] + s[-3])
    ; hi: abs(t[-1] - 2 * t[-2] + t[-3])
    pabsw             m9, m9
    punpcklwd        m14, m8, m9
    punpckhwd         m8, m9

    ; second_deriv[1], second_deriv[-2]
    pavgw             m8, m14

%if %1 >= 3
    ; compute transition
    vpbroadcastd     m14, [pb_m2_1]
    pmaddubsw         m9, m12, m14
    pmaddubsw        m14, m13, m14
    pshufb           m11, m1, m15
    pshufb           m10, m0, m15
    paddw             m9, m11
    paddw            m10, m14
    ; lo: abs(s[-1] - 2 * s[0]  + s[1])
    ; hi: abs(t[-1] - 2 * t[0]  + t[1])
    pabsw             m9, m9
    ; lo: abs(s[0]  - 2 * s[-1] + s[-2])
    ; hi: abs(t[0]  - 2 * t[-1] + t[-2])
    pabsw            m10, m10
    punpcklwd        m14, m9, m10
    punpckhwd         m9, m10
    ; second_deriv[0], second_deriv[-1]
    pavgw             m9, m14

    ; transition = second_deriv[-1] + second_deriv[0]
    ; duplicate the results across pairs of words
    pshufb           m14, m9, [rev_shuf]
    paddw             m9, m14

    vpbroadcastd     m14, [pb_m2_3]
    pmaddubsw        m10, m12, m14
    pshufb           m11, m6, m15
    psubw            m10, m11
    pabsw            m10, m10
%if !is_chroma_edge
    pmaddubsw        m14, m13, m14
    pshufb           m11, m7, m15
    psubw            m14, m11
    pabsw            m14, m14
    punpcklwd        m11, m10, m14
    punpckhwd        m10, m14
%else
    ; Skip the negative test by duplicating the positive derivatives
    punpcklwd        m11, m10, m10
    punpckhwd        m10, m10
%endif
    pavgw            m10, m11
%if %1 >= 4
    ; load 4, -5
%ifidn %2, v
    mova              m6, [dstq+strideq*4]
    mova              m7, [tmpq+mstrideq*1]
%else
    mova              m6, [rsp+8*32]
    mova              m7, [rsp+9*32]
%endif
    vpbroadcastd     m14, [pb_m3_4]
    pmaddubsw        m11, m12, m14
    pshufb            m6, m15
    psubw            m11, m6
    pabsw            m11, m11
%if !is_chroma_edge
    pmaddubsw        m14, m13, m14
    pshufb            m7, m15
    psubw            m14, m7
    pabsw            m14, m14
    punpcklwd         m6, m11, m14
    punpckhwd        m11, m14
%else
    punpcklwd         m6, m11, m11
    punpckhwd        m11, m11
%endif
    pavgw             m6, m11

%if %1 >= 6
    ; load 6
%ifidn %2, v
    mova              m7, [dstq+stride3q*2]
%else
    mova              m7, [rsp+12*32]
%endif

    vpbroadcastd     m14, [pb_m5_6]
    pmaddubsw        m11, m12, m14
    pmaddubsw        m14, m13, m14
    pshufb            m7, m15
    psubw            m11, m7
    ; load -7
%ifidn %2, v
    sub             tmpq, strideq
    mova              m7, [tmpq+mstrideq*2]
    add             tmpq, strideq
%else
    mova              m7, [rsp+13*32]
%endif
    pshufb            m7, m15
    psubw            m14, m7
    pabsw            m11, m11
    pabsw            m14, m14
    punpcklwd         m7, m11, m14
    punpckhwd        m11, m14
    pavgw             m7, m11

%if %1 == 8
    vpbroadcastd     m14, [pb_m6_7]
    pmaddubsw        m12, m14
%if !is_edge
    pmaddubsw        m13, m14
%endif

    ; load 7
%ifidn %2, v
    add             dstq, stride3q
    mova             m14, [dstq+strideq*4]
    sub             dstq, stride3q
%else
    mova             m14, [rsp+14*32]
%endif
    pshufb           m14, m15
    psubw            m12, m14
    pabsw            m12, m12
%if !is_edge
    ; load -8
%ifidn %2, v
    mova             m14, [tmpq+mstrideq*4] ; -8
%else
    mova             m14, [rsp+15*32]
%endif
    pshufb           m14, m15
    psubw            m13, m14
    pabsw            m13, m13
    punpcklwd        m11, m12, m13
    punpckhwd        m12, m13
%else
    punpcklwd        m11, m12, m12
    punpckhwd        m12, m12
%endif
    pavgw            m11, m12
%endif
%endif
%endif
%endif

    movq            xm15, [side_thrq]
    punpcklbw       xm15, xm15
    pmovzxbw         m15, xm15

    ; side thr comparisons
    ; duplicate side_thr to compare against both s[] and t[] at the same time
    ; successes have pairs of words [0,0]
    ; failures are [-1,-1], [-1,0], or [0,-1]
%if %1 >= 3
%if %1 >= 6
    vpbroadcastd     m12, [pw_24576]
    pmulhuw          m12, m15             ; (side_thr * 6) >> 4
    pcmpgtw           m7, m12
    psrlw            m12, 1               ; (side_thr * 3) >> 4
%else
    vpbroadcastd     m12, [pw_12288]
    pmulhuw          m12, m15             ; (side_thr * 3) >> 4
%endif
    pcmpgtw          m10, m12
    psrlw            m14, m15, 2          ; side_thr >> 2
    psrlw            m13, m15, 3          ; side_thr >> 3
    pcmpgtw          m12, m8, m14
%if %1 >= 4
    pcmpgtw           m6, m14
%endif
    pcmpgtw          m13, m8, m13
    ; Or together earlier conditions
    ; The second_deriv[1] and second_deriv[-2] compares are subsets of later
    ; tests, so ors are skipped for those.
    por              m13, m10
%if %1 >= 4
    por               m6, m13
%if %1 >= 6
    por               m7, m6
%if %1 == 8
    psrlw            m14, m15, 1          ; (side_thr * 8) >> 4
    pcmpgtw          m11, m14
    por              m11, m7
%endif
%endif
%endif
%endif
    pcmpgtw           m8, m15

    movq            xm14, [q_thrq]
    punpcklbw       xm14, xm14
    pmovzxbw         m14, xm14

    pxor             m15, m15
    ; q_thr comparisons then combine with side_thresh compares
%if %1 >= 3
    vpbroadcastd     m10, [pw_4_3]
    pmullw           m10, m14       ; q_thr * 4, q_thr * 3
    pcmpgtw          m10, m9, m10
    por              m13, m10       ; combine side_thr and q_thr conditions
    pblendw          m10, m15, 0xaa
    por              m12, m10
%if %1 == 8
    ; q_thr * 32 > transition << 4
    paddw            m10, m14, m14
    pcmpgtw          m10, m9, m10
    por              m11, m10
%endif
%if %1 >= 4
    psllw            m9, 4          ; transition <<= 4
%if %1 == 4
    vpbroadcastd     m10, [pw_45]
    pmullw           m10, m14
    pcmpgtw          m10, m9, m10
    por               m6, m10
%else ; %1 >= 6
    vpbroadcastd     m10, [pw_45_40]
    pmullw           m10, m14
    pcmpgtw          m10, m9, m10
    por               m7, m10
    pblendw          m10, m15, 0xaa
    por               m6, m10
%endif
%endif
%endif

    ; For each width, check sets of comparisons worked.
    ; Successes are dword -1's now.
    pcmpeqd           m8, m15
%if %1 >= 3
    pcmpeqd          m12, m15
    pcmpeqd          m13, m15
%if %1 >= 4
    pcmpeqd           m6, m15
%if %1 >= 6
    pcmpeqd           m7, m15
%if %1 == 8
    pcmpeqd          m11, m15
%endif
%endif
%endif
%endif

    ; Contrain width selection by the masks
    mova             m15, [pd_mask]
%if %1 >= 3
%if %1 >= 4
%if %1 == 8
    vpbroadcastb     m10, [maskq+6]
    pand             m11, m15
    pand             m11, m10
    pcmpeqd          m11, m15
%endif
    vpbroadcastb      m9, [maskq+4]
%if %1 == 8
    por               m9, m10
%endif
    pand             m10, m9, m15
    pand              m6, m10
    pcmpeqd           m6, m15
%if %1 >= 6
    pand              m7, m10
    pcmpeqd           m7, m15
%endif
%endif
    vpbroadcastb     m10, [maskq+2]
%if %1 >= 4
    por              m10, m9
%endif
    pand              m9, m10, m15

    pand             m12, m9
    pand             m13, m9
    pcmpeqd          m12, m15
    pcmpeqd          m13, m15
%endif
    vpbroadcastb      m9, [maskq+0]
%if %1 >= 3
    por               m9, m10
%endif
    pand              m8, m9
    pand              m8, m15
    pcmpeqd           m8, m15

    ; Create a lookup table index based on the filter size.
%if %1 >= 3
%if %1 == 8
    paddb             m8, m11
%endif
%if %1 >= 6
    paddb            m12, m7
%endif
%if %1 >= 4
    paddb            m13, m6
%endif
    vpbroadcastd     m11, [mul_lut_offset]
    paddb             m8, m12
    paddb             m8, m13

    paddb             m8, m8
    psubb             m8, m11, m8
%endif

    ; delta_m2 = 4 * (3 * (dst[0] - dst[-1]) - (dst[1] - dst[-2])
    vpbroadcastd     m13, [pb_12_m4]
    punpcklbw         m9, m1, m3
    punpckhbw        m10, m1, m3
    punpcklbw        m11, m0, m2
    punpckhbw        m12, m0, m2
    pmaddubsw         m9, m13
    pmaddubsw        m10, m13
    pmaddubsw        m11, m13
    pmaddubsw        m12, m13
    psubw             m9, m11, m9
    psubw            m10, m12, m10

    ; q_thr_clamp = q_thr * q_thresh_mults[]
%if %1 == 1
    psllw            m11, m14, 5    ; q_thr * q_thresh_mult[0]
%else
    vbroadcasti128   m11, [qthr_mul_lut]
    pshufb           m11, m8
    pmullw           m11, m14
%endif
    punpckhdq        m12, m11, m11
    punpckldq        m11, m11

    ; iclip(delta_m2, -q_thr_clamp, q_thr_clamp)
    pminsw            m9, m11
    pminsw           m10, m12
    pxor             m15, m15
    psubw            m11, m15, m11
    psubw            m12, m15, m12
    pmaxsw            m9, m11
    pmaxsw           m10, m12

%if !is_edge
    ; +0, -1
%if %1 == 1
    vpbroadcastd     m11, [pw_1360]
    pmulhrsw          m6, m9, m11
    pmulhrsw          m7, m10, m11
%else
    vbroadcasti128   m11, [mul1_lut]
    pshufd           m14, m8, q3322
    pshufd            m8, m8, q1100
    pshufb           m12, m11, m14
    pshufb           m11, m8
    pmulhrsw          m6, m9, m11
    pmulhrsw          m7, m10, m12
%endif
    SUB_ADD_DIFF_LARGE m11, m12, m0, m1, m6, m7, m13

    mova             m13, [pd_mask]
    vpbroadcastb      m6, [ll_maskq+2] ; pos
    vpbroadcastb      m7, [ll_maskq]   ; neg
    pand              m6, m13
    pand              m7, m13
    pcmpeqd           m6, m13
    pcmpeqd           m7, m13
%if %1 == 1
    pandn             m6, m8
    pandn             m7, m8
    vpblendvb         m0, m11, m6
    vpblendvb         m1, m12, m7
%else
    vpblendvb         m0, m11, m0, m6
    vpblendvb         m1, m12, m1, m7
%endif
%ifidn %2, v
    mova [dstq+strideq*0], m0
    mova  [tmpq+stride3q], m1
%elif %1 == 1
    TRANSPOSE_16x2_AND_WRITE_2x32 1, 0, 11
%endif

%if %1 >= 3
    ; +1, -2
    vbroadcasti128   m11, [mul2_lut]
    vpbroadcastd     m15, [pb_128]
    pshufb           m12, m11, m14
    pshufb           m11, m8
    pmulhrsw         m11, m9, m11
    pmulhrsw         m12, m10, m12
    SUB_ADD_DIFF m2, m3, m11, m12, m6, m7
%ifidn %2, v
    mova [dstq+strideq*1], m2
    mova [tmpq+strideq*2], m3
%elif %1 == 4
    mova      [rsp+0*32], m0
    mova      [rsp+1*32], m1
%elif %1 >= 6
    mova      [rsp+0*32], m0
    mova      [rsp+1*32], m1
    mova      [rsp+2*32], m2
    mova      [rsp+3*32], m3
%endif

    ; +2, -3
    vbroadcasti128   m11, [mul3_lut]
%ifidn %2, h
    pshufb           m12, m11, m14
    pshufb           m11, m8
    pmulhrsw         m11, m9, m11
    pmulhrsw         m12, m10, m12
%else
    pshufb           m0, m11, m8
    pshufb           m1, m11, m14
    pmulhrsw         m11, m9, m0
    pmulhrsw         m12, m10, m1
%endif
    SUB_ADD_DIFF m4, m5, m11, m12, m6, m7
%ifidn %2, v
    mova [dstq+strideq*2], m4
    mova [tmpq+strideq*1], m5
%elif %1 == 3
    mova             m12, [rsp+6*32]
    mova             m13, [rsp+7*32]
    TRANSPOSE_16x8_AND_WRITE_8x32 13, 5, 3, 1, 0, 2, 4, 12, 11
%elif %1 == 4
    SWAP 0, 4
    SWAP 1, 5
%else
    mova      [rsp+4*32], m4
    mova      [rsp+5*32], m5
%endif

%if %1 >= 6 && %isidn(%2, v)
    ; start at +3, -4
    lea             dstq, [dstq+stride3q]
    vbroadcasti128   m11, [base_mul_lut]
    pshufb            m8, m11, m8
    pshufb           m14, m11, m14

    mov               nd, %1-3
.loop_w%1:
    mova              m4, [dstq]
    mova              m5, [tmpq]
    psubusw           m0, m8
    psubusw           m1, m14

    pmulhrsw         m11, m9, m0
    pmulhrsw         m12, m10, m1
    SUB_ADD_DIFF m4, m5, m11, m12, m6, m7
    mova          [dstq], m4
    mova          [tmpq], m5

    add             dstq, strideq
    sub             tmpq, strideq
    dec               nd
    jg .loop_w%1

%if %1 == 6
    sub             dstq, stride3q
    sub             dstq, stride3q
%else
    lea             dstq, [dstq+8*mstrideq]
%endif
%elif %1 >= 4
    ; +3, -4
%ifidn %2, v
    mova              m4, [dstq+stride3q]
    mova              m5, [tmpq+strideq*0]
%else
    mova              m4, [rsp+6*32]
    mova              m5, [rsp+7*32]
%endif

    vbroadcasti128   m11, [mul4_lut]
    pshufb           m12, m11, m14
    pshufb           m11, m8
    pmulhrsw         m11, m9, m11
    pmulhrsw         m12, m10, m12
    SUB_ADD_DIFF m4, m5, m11, m12, m6, m7
%ifidn %2, v
    mova [dstq+stride3q], m4
    mova [tmpq+strideq*0], m5
%elif %1 == 4
    mova             m12, [rsp+0*32]
    mova             m13, [rsp+1*32]
    TRANSPOSE_16x8_AND_WRITE_8x32 5, 1, 3, 13, 12, 2, 0, 4, 11
%elif %1 == 6
    SWAP 0, 4
    SWAP 1, 5
%else
    mova      [rsp+6*32], m4
    mova      [rsp+7*32], m5
%endif

; Only horizontal deblock from here on out
%if %1 >= 6
    ; +4, -5
    mova              m4, [rsp+8*32]
    mova              m5, [rsp+9*32]

    vbroadcasti128   m11, [mul5_lut]
    pshufb           m12, m11, m14
    pshufb           m11, m8
    pmulhrsw         m11, m9, m11
    pmulhrsw         m12, m10, m12
    SUB_ADD_DIFF m4, m5, m11, m12, m6, m7
%if %1 == 6
    SWAP 2, 4
    SWAP 3, 5
%else
    mova      [rsp+8*32], m4
    mova      [rsp+9*32], m5
%endif

    ; +5, -6
    mova              m4, [rsp+10*32]
    mova              m5, [rsp+11*32]

    vbroadcasti128   m11, [mul6_lut]
    pshufb           m12, m11, m14
    pshufb           m11, m8
    pmulhrsw         m11, m9, m11
    pmulhrsw         m12, m10, m12
    SUB_ADD_DIFF m4, m5, m11, m12, m6, m7
%if %1 == 6
    SWAP              11, 0
    SWAP               2, 5, 12
    SWAP               4, 1, 13
    ; 5,3,1,0,2,4 -> 2,3,4,11,12,13
    mova          m0, [rsp+15*32]
    mova          m1, [rsp+13*32]
    mova          m5, [rsp+5*32]
    mova          m6, [rsp+3*32]
    mova          m7, [rsp+1*32]
    mova          m8, [rsp+0*32]
    mova          m9, [rsp+2*32]
    mova         m10, [rsp+4*32]
    mova         m14, [rsp+12*32]
    TRANSPOSE_16X16B   1, 0, [rsp+14*32]
    STORE_16X16
%else
    SWAP 0, 4
    SWAP 1, 5
%endif

%if %1 == 8
    ; +6, -7
    mova              m4, [rsp+12*32]
    mova              m5, [rsp+13*32]
    vbroadcasti128   m11, [mul7_lut]
    pshufb           m12, m11, m14
    pshufb           m11, m8
    pmulhrsw         m11, m9, m11
    pmulhrsw         m12, m10, m12
    SUB_ADD_DIFF m4, m5, m11, m12, m6, m7
    SWAP 2, 4
    SWAP 3, 5

    ; +7, -8
    mova              m4, [rsp+14*32]
    mova              m5, [rsp+15*32]
    vbroadcasti128   m11, [mul8_lut]
    pshufb           m12, m11, m14
    pshufb           m11, m8
    pmulhrsw         m11, m9, m11
    pmulhrsw         m12, m10, m12
    SUB_ADD_DIFF m4, m5, m11, m12, m6, m7

    SWAP              15, 4
    SWAP               0, 5, 13
    SWAP               2, 1, 3, 14
    ; 5,3,1,0,2,4 -> 0,1,2,13,14,15

    ; TODO: optimize this store away
    mova     [rsp+14*32], m15

    mova              m3, [rsp+9*32]
    mova              m4, [rsp+7*32]
    mova              m5, [rsp+5*32]
    mova              m6, [rsp+3*32]
    mova              m7, [rsp+1*32]
    mova              m8, [rsp+0*32]
    mova              m9, [rsp+2*32]
    mova             m10, [rsp+4*32]
    mova             m11, [rsp+6*32]
    mova             m12, [rsp+8*32]
    TRANSPOSE_16X16B   1, 0, [rsp+14*32]
    STORE_16X16
%endif
%endif
%endif
%endif
%else
    ; edge variant
    ; compute pos and neg sides of the filter seperately

%ifidn %2, h
    ; Move to stack so we can access in a loop
    mova      [rsp+2*32], m2
%if !is_chroma_edge
    mova      [rsp+3*32], m3
%endif
    mova      [rsp+4*32], m4
    mova      [rsp+5*32], m5
%endif

    mova             m13, [pd_mask]
    vpbroadcastb      m6, [ll_maskq+2] ; pos
    vpbroadcastb      m7, [ll_maskq]   ; neg
    pand              m6, m13
    pand              m7, m13
    pcmpeqd           m6, m13
    pcmpeqd           m7, m13
    pshufd           m14, m8, q3322
    pshufd            m8, m8, q1100

%if is_chroma_edge
    vbroadcasti128   m13, [mul1_lut_uv_edge]
%else
    vbroadcasti128   m13, [mul1_lut_y_edge]
%endif
    ; -1
    pshufb            m4, m13, m8
    pshufb            m5, m13, m14
    pmulhrsw         m12, m4, m9
    punpcklbw        m11, m1, m15
    paddw            m11, m12
    pmulhrsw         m13, m5, m10
    punpckhbw        m12, m1, m15
    paddw            m12, m13
    packuswb         m11, m12
    vpblendvb        m11, m1, m7
%ifidn %2, v
    mova [tmpq+stride3q], m11
%else
    mova      [rsp+1*32], m11
%endif

    ; 0
    vbroadcasti128   m13, [mul1_lut]
    pshufb           m11, m13, m8
    pshufb           m12, m13, m14
    pmulhrsw         m13, m11, m9
    punpcklbw         m1, m0, m15
    psubw             m1, m13
    pmulhrsw         m13, m12, m10
    punpckhbw        m15, m0, m15
    psubw            m15, m13
    packuswb          m1, m15
    vpblendvb         m1, m0, m6
%ifidn %2, v
    mova          [dstq], m1
%else
    mova      [rsp+0*32], m1
%endif

    vpbroadcastd     m15, [pb_128]
%if is_chroma_edge
    ; -2
    vbroadcasti128   m13, [mul2_lut_uv_edge]
    pshufb            m4, m13, m8
    pshufb            m5, m13, m14
    pmulhrsw          m4, m9
    pmulhrsw          m5, m10
    packsswb          m4, m5
    pxor              m3, m15
    pandn             m4, m7, m4
    paddsb            m3, m4
    pxor              m3, m15
%ifidn %2, v
    mova [tmpq+strideq*2], m3
%else
    mova      [rsp+3*32], m3
%endif
%else
    ; start at -2
%ifidn %2, v
    lea             tmpq, [tmpq+strideq*2]
%else
    lea             tmpq, [rsp+3*32]
%endif

    vbroadcasti128   m13, [base_mul_lut_y_edge]
    pshufb            m2, m13, m8
    pshufb            m3, m13, m14

    mov               nd, 5
.neg_loop_w%1:
    mova             m13, [tmpq]

    psubusw           m4, m2
    psubusw           m5, m3
    pmulhrsw          m0, m4, m9
    pmulhrsw          m1, m5, m10

    packsswb          m0, m1
    pxor             m13, m15
    pandn             m0, m7, m0
    paddsb           m13, m0
    pxor             m13, m15
    mova          [tmpq], m13

%ifidn %2, v
    sub             tmpq, strideq
%else
    add             tmpq, 32*2
%endif
    dec               nd
    jg .neg_loop_w%1
%endif

    ; Start at 1
%ifidn %2, v
    lea             tmpq, [dstq+strideq]
%else
    lea             tmpq, [rsp+32*2]
%endif
    vbroadcasti128   m13, [base_mul_lut]
    pshufb            m4, m13, m8
    pshufb            m5, m13, m14
    vbroadcasti128   m13, [mul1_lut]
    pshufb            m2, m13, m8
    pshufb            m3, m13, m14
    mov               nd, %1-1

.pos_loop_w%1:
    mova             m13, [tmpq]

    psubusw           m2, m4
    psubusw           m3, m5
    pmulhrsw          m0, m2, m9
    pmulhrsw          m1, m3, m10

    packsswb          m0, m1
    pxor             m13, m15
    pandn             m0, m6, m0
    psubsb           m13, m0
    pxor             m13, m15
    mova          [tmpq], m13

%ifidn %2, v
    add             tmpq, strideq
%else
    add             tmpq, 32*2
%endif
    dec               nd
    jg .pos_loop_w%1

%ifidn %2, h
%if %1 != 8
    mova              m7, [rsp+7*32]
    mova              m5, [rsp+5*32]
    mova              m3, [rsp+3*32]
    mova              m1, [rsp+1*32]
    mova              m0, [rsp+0*32]
    mova              m2, [rsp+2*32]
    mova              m4, [rsp+4*32]
    mova              m6, [rsp+6*32]
    TRANSPOSE_16x8_AND_WRITE_8x32 7, 5, 3, 1, 0, 2, 4, 6, 11
%else
    mova              m0, [rsp+15*32]
    mova              m1, [rsp+13*32]
    mova              m2, [rsp+11*32]
    mova              m3, [rsp+9*32]
    mova              m4, [rsp+7*32]
    mova              m5, [rsp+5*32]
    mova              m6, [rsp+3*32]
    mova              m7, [rsp+1*32]
    mova              m8, [rsp+0*32]
    mova              m9, [rsp+2*32]
    mova             m10, [rsp+4*32]
    mova             m11, [rsp+6*32]
    mova             m12, [rsp+8*32]
    mova             m13, [rsp+10*32]
    mova             m14, [rsp+12*32]
    TRANSPOSE_16X16B   1, 0, [rsp+14*32]
    STORE_16X16
%endif
%endif
%endif
%endmacro

INIT_YMM avx2
cglobal deblock_v_sb_y_8bpc, 6, 12, 16, \
                     dst, stride, mask, ll_mask, q_thr, side_thr, \
                     edge, w, stride3, mstride, tmp, n
    movifnidn  edged, edgem
    movifnidn     wd, wm
    mov     mstrideq, strideq
    neg     mstrideq
    lea     stride3q, [strideq*3]

.loop:
    cmp byte [maskq+6], 0                       ; vmask[3]
    je .v6

    test edged, edged
    jnz .v8_edge

    FILTER         8, v, 0
    jmp .end

.v8_edge:
    FILTER         8, v, 1
    jmp .end

.v6:
    cmp byte [maskq+4], 0                       ; vmask[2]
    je .v3_test

    FILTER         6, v, 0
    jmp .end

.v3_test:
    cmp byte [maskq+2], 0                       ; vmask[1]
    je .v1_test

    call .v3
    jmp .end

.v1_test:
    cmp byte [maskq+0], 0                       ; vmask[0]
    je .end

    call .v1

.end:
    add         dstq, 32
    add        maskq, 1
    add     ll_maskq, 1
    add       q_thrq, 8
    add    side_thrq, 8
    sub           wd, 8
    jg .loop
    RET
ALIGN function_align
.v3:
    FILTER         3, v, 0
    ret
ALIGN function_align
.v1:
    FILTER         1, v, 0
    ret

INIT_YMM avx2
cglobal deblock_h_sb_y_8bpc, 6, 11, 16, 32 * 16, \
                     dst, stride, mask, ll_mask, q_thr, side_thr, \
                     edge, h, stride3, tmp, n
    movifnidn  edged, edgem
    movifnidn     hd, hm
    lea     stride3q, [strideq*3]

.loop:
    cmp byte [maskq+6], 0                       ; hmask[3]
    je .h6

    test edged, edged
    jnz .h8_edge

    ; TODO: test edges inside horz filters to reduce binary size. Edges are
    ;       less common for the horz case, so branches should be almost free.
    FILTER         8, h, 0
    jmp .end

.h8_edge:
    FILTER         8, h, 1
    jmp .end

.h6:
    cmp byte [maskq+4], 0                       ; hmask[2]
    je .h3

    FILTER         6, h, 0
    jmp .end

.h3:
    cmp byte [maskq+2], 0                       ; hmask[1]
    je .h1_test

    FILTER         3, h, 0
    jmp .end

.h1_test:
    cmp byte [maskq+0], 0                       ; hmask[0]
    je .no_filter

    call .h1
    jmp .end

.no_filter:
    lea         dstq, [dstq+stride3q*8]
    lea         dstq, [dstq+strideq*8]
.end:
    add        maskq, 1
    add     ll_maskq, 1
    add       q_thrq, 8
    add    side_thrq, 8
    sub           hd, 8
    jg .loop
    RET
ALIGN function_align
.h1:
    FILTER         1, h, 0
    ret

INIT_YMM avx2
cglobal deblock_v_sb_uv_8bpc, 6, 12, 16, \
                     dst, stride, mask, ll_mask, q_thr, side_thr, \
                     edge, w, stride3, mstride, tmp, n
    movifnidn  edged, edgem
    movifnidn     wd, wm
    mov     mstrideq, strideq
    neg     mstrideq
    lea     stride3q, [strideq*3]

.loop:
    cmp byte [maskq+4], 0                       ; vmask[2]
    je .v3

    test edged, edged
    jnz .v4_edge

    FILTER         4, v, 0
    jmp .end

.v4_edge:
    FILTER         4, v, 1
    jmp .end

.v3:
    cmp byte [maskq+2], 0                       ; vmask[1]
    je .v1_test

    test edged, edged
    jnz .v3_edge

    call mangle(private_prefix %+ _deblock_v_sb_y_8bpc_avx2).v3
    jmp .end

.v3_edge:
    FILTER         3, v, 1
    jmp .end

.v1_test:
    cmp byte [maskq+0], 0                       ; vmask[0]
    je .end

    call mangle(private_prefix %+ _deblock_v_sb_y_8bpc_avx2).v1

.end:
    add         dstq, 32
    add        maskq, 1
    add     ll_maskq, 1
    add       q_thrq, 8
    add    side_thrq, 8
    sub           wd, 8
    jg .loop
    RET

INIT_YMM avx2
cglobal deblock_h_sb_uv_8bpc, 6, 11, 16, 32 * 10, \
                     dst, stride, mask, ll_mask, q_thr, side_thr, \
                     edge, h, stride3, tmp, n
    movifnidn  edged, edgem
    movifnidn     hd, hm
    lea     stride3q, [strideq*3]

.loop:
    cmp byte [maskq+4], 0                       ; vmask[2]
    je .h3

    test edged, edged
    jnz .h4_edge

    FILTER         4, h, 0
    jmp .end

.h4_edge:
    FILTER         4, h, 1
    jmp .end

.h3:
    cmp byte [maskq+2], 0                       ; vmask[1]
    je .h1_test

    test edged, edged
    jnz .h3_edge

    ; TODO: call the luma version instead duplicating
    FILTER         3, h, 0
    jmp .end

.h3_edge:
    FILTER         3, h, 1
    jmp .end

.h1_test:
    cmp byte [maskq+0], 0                       ; vmask[0]
    je .no_filter

    call mangle(private_prefix %+ _deblock_h_sb_y_8bpc_avx2).h1
    jmp .end

.no_filter:
    lea         dstq, [dstq+stride3q*8]
    lea         dstq, [dstq+strideq*8]
.end:
    add        maskq, 1
    add     ll_maskq, 1
    add       q_thrq, 8
    add    side_thrq, 8
    sub           hd, 8
    jg .loop
    RET
