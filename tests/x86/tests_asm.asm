%undef private_prefix
%define private_prefix selftest

%include "src/checkasm_config.asm"
%include "src/x86/x86inc.asm"

SECTION .text

%macro copy_mm 0-1 ; suffix
cglobal copy%1, 3, 3, 1, dst, src, size
    cmp     sizeq, mmsize
    jl copy_x86

    add     dstq, sizeq
    add     srcq, sizeq
    neg     sizeq
.loop:
    mova    m0, [srcq + sizeq]
    mova    [dstq + sizeq], m0
    add     sizeq, mmsize
    jl .loop

    ; emit emms after all MMX functions unless suffix is _noemms
%if mmsize == 8
  %ifnidn %1, _noemms
    emms
  %endif
%endif

    ; skip vzeroupper if suffix is _novzeroupper
%ifidn %1, _novzeroupper
    ret
%else
    RET
%endif
%endmacro

; Generic x86 functions
cglobal copy_x86, 3, 3, 0, dst, src, size
%if ARCH_X86_32 || WIN64
    push    rdi
    push    rsi
    mov     rdi, dstq
    mov     rsi, srcq
%endif
    mov     rcx, sizeq
    rep     movsb
%if ARCH_X86_32 || WIN64
    pop     rsi
    pop     rdi
%endif
    RET

cglobal sigill_x86
    ud2

%macro clobber 2 ; register, suffix
cglobal clobber%2
    xor     %1, %1
    RET
%endmacro

clobber r0,  _r0
clobber r1,  _r1
clobber r2,  _r2
clobber r3,  _r3
clobber r4,  _r4
clobber r5,  _r5
clobber r6,  _r6
%if ARCH_X86_64
clobber r7,  _r7
clobber r8,  _r8
clobber r9,  _r9
clobber r10, _r10
clobber r11, _r11
clobber r12, _r12
clobber r13, _r13
clobber r14, _r14
%endif

cglobal corrupt_stack_x86
    xor rax, rax
%if WIN64
    mov [rsp+32+8], rax ; account for shadow space
%else
    mov [rsp+8], rax
%endif
    RET

cglobal get_stack_pointer_x86
    mov rax, rsp
    RET

; MMX functions
INIT_MMX mmx
copy_mm
copy_mm _noemms

cglobal noemms, 3, 3, 0, dst, src, size
    RET

; SSE2 functions
INIT_XMM sse2
copy_mm

; AVX2 functions
INIT_YMM avx2
copy_mm
copy_mm _novzeroupper

; AVX512 functions
INIT_YMM avx512
copy_mm

;-----------------------------------------
; void ff_fill_block_tab_%1(uint8_t *block, uint8_t value,
;                           ptrdiff_t line_size, int h);
;-----------------------------------------
%macro SPLATW 2-3 0
%if cpuflag(avx2) && %3 == 0
    vpbroadcastw %1, %2
%elif mmsize == 16
    pshuflw    %1, %2, (%3)*0x55
    punpcklqdq %1, %1
%elif cpuflag(mmxext)
    pshufw     %1, %2, (%3)*0x55
%else
    %ifnidn %1, %2
        mova       %1, %2
    %endif
    %if %3 & 2
        punpckhwd  %1, %1
    %else
        punpcklwd  %1, %1
    %endif
    %if %3 & 1
        punpckhwd  %1, %1
    %else
        punpcklwd  %1, %1
    %endif
%endif
%endmacro

%macro SPLATB_REG 3
%if cpuflag(ssse3)
    movd      %1, %2d
    pshufb    %1, %3
%else
    movd      %1, %2d
    punpcklbw %1, %1
    SPLATW    %1, %1, 0
%endif
%endmacro

%macro FILL_BLOCK_TAB 2
cglobal fill_block_tab_%1, 4, 5, 1, block, value, stride, h, stride3
    lea stride3q, [strideq + strideq * 2]
%if cpuflag(avx2)
    movd m0, valued
    vpbroadcastb m0, m0
%else
    SPLATB_REG m0, value, x
%endif
.loop:
    mov%2 [blockq], m0
    mov%2 [blockq + strideq], m0
    mov%2 [blockq + strideq * 2], m0
    mov%2 [blockq + stride3q], m0
    lea blockq, [blockq + strideq * 4]
    sub hd, 4
    jg .loop
    RET
%endmacro

INIT_XMM sse2
FILL_BLOCK_TAB 8, q
FILL_BLOCK_TAB 16, a
INIT_XMM avx2
FILL_BLOCK_TAB 8, q
FILL_BLOCK_TAB 16, a
