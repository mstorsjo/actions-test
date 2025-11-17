#include "tests.h"

/* Re-use from main checkasm library */
typedef struct {
    uint32_t eax, ebx, edx, ecx;
} CpuidRegisters;

void       checkasm_cpu_cpuid(CpuidRegisters *regs, unsigned leaf, unsigned subleaf);
uint64_t   checkasm_cpu_xgetbv(unsigned xcr);
static int has_vzeroupper_check;

uint64_t checkasm_get_cpu_flags_x86(void)
{
    uint64_t       flags = CHECKASM_CPU_FLAG_X86;
    CpuidRegisters r;
    checkasm_cpu_cpuid(&r, 0, 0);
    const uint32_t max_leaf = r.eax;
    if (max_leaf < 1)
        return flags;

    checkasm_cpu_cpuid(&r, 1, 0);
    if (r.edx & 0x00800000) /* MMX */
        flags |= CHECKASM_CPU_FLAG_MMX;
    if (r.edx & 0x02000000) /* SSE2 */
        flags |= CHECKASM_CPU_FLAG_SSE2;
    if (~r.ecx & 0x18000000) /* OSXSAVE/AVX */
        return flags;

    const uint64_t xcr0 = checkasm_cpu_xgetbv(0);
    if (max_leaf < 7 || ~xcr0 & 0x6) /* XMM/YMM */
        return flags;

    checkasm_cpu_cpuid(&r, 7, 0);
    if (r.ebx & 0x00000020) { /* AVX2 */
        flags |= CHECKASM_CPU_FLAG_AVX2;
        has_vzeroupper_check
            = !(checkasm_cpu_xgetbv(1) & 0x04); /* YMM state always dirty */
    }

    if (~xcr0 & 0xe0) /* ZMM/OPMASK */
        return flags;

    if (r.ebx & 0x00000020) /* AVX512F */
        flags |= CHECKASM_CPU_FLAG_AVX512;
    return flags;
}

DEF_COPY_FUNC(copy_x86);
DEF_COPY_FUNC(copy_mmx);
DEF_COPY_FUNC(copy_sse2);
DEF_COPY_FUNC(copy_avx2);
DEF_COPY_FUNC(copy_avx512);

DEF_NOOP_FUNC(clobber_r0);
DEF_NOOP_FUNC(clobber_r1);
DEF_NOOP_FUNC(clobber_r2);
DEF_NOOP_FUNC(clobber_r3);
DEF_NOOP_FUNC(clobber_r4);
DEF_NOOP_FUNC(clobber_r5);
DEF_NOOP_FUNC(clobber_r6);
#if ARCH_X86_64
DEF_NOOP_FUNC(clobber_r7);
DEF_NOOP_FUNC(clobber_r8);
DEF_NOOP_FUNC(clobber_r9);
DEF_NOOP_FUNC(clobber_r10);
DEF_NOOP_FUNC(clobber_r11);
DEF_NOOP_FUNC(clobber_r12);
DEF_NOOP_FUNC(clobber_r13);
DEF_NOOP_FUNC(clobber_r14);
#endif

DEF_NOOP_FUNC(sigill_x86);
DEF_NOOP_FUNC(corrupt_stack_x86);

DEF_COPY_FUNC(copy_noemms_mmx);
DEF_COPY_FUNC(copy_novzeroupper_avx2);

static copy_func *get_copy_x86(void)
{
    const uint64_t flags = checkasm_get_cpu_flags();
#if ARCH_X86
    if (flags & CHECKASM_CPU_FLAG_AVX512)
        return checkasm_copy_avx512;
    if (flags & CHECKASM_CPU_FLAG_AVX2)
        return checkasm_copy_avx2;
    if (flags & CHECKASM_CPU_FLAG_SSE2)
        return checkasm_copy_sse2;
    if (flags & CHECKASM_CPU_FLAG_MMX)
        return checkasm_copy_mmx;
    if (flags & CHECKASM_CPU_FLAG_X86)
        return checkasm_copy_x86;
#endif
    return checkasm_copy_c;
}

#if ARCH_X86_64
  #ifdef _WIN32
    #define NUM_SAFE 7
  #else
    #define NUM_SAFE 9
  #endif
  #define NUM_REGS 15
#else
  #define NUM_SAFE 3
  #define NUM_REGS 7
#endif

static noop_func *get_clobber(int reg)
{
    if (!(checkasm_get_cpu_flags() & CHECKASM_CPU_FLAG_X86))
        return NULL;

    switch (reg) {
    case 0: return checkasm_clobber_r0;
    case 1: return checkasm_clobber_r1;
    case 2: return checkasm_clobber_r2;
    case 3: return checkasm_clobber_r3;
    case 4: return checkasm_clobber_r4;
    case 5: return checkasm_clobber_r5;
    case 6: return checkasm_clobber_r6;
#if ARCH_X86_64
    case 7:  return checkasm_clobber_r7;
    case 8:  return checkasm_clobber_r8;
    case 9:  return checkasm_clobber_r9;
    case 10: return checkasm_clobber_r10;
    case 11: return checkasm_clobber_r11;
    case 12: return checkasm_clobber_r12;
    case 13: return checkasm_clobber_r13;
    case 14: return checkasm_clobber_r14;
#endif
    /* can't clobber rsp without completely crashing the program */
    default: return NULL;
    }
}

DEF_NOOP_GETTER(CHECKASM_CPU_FLAG_X86, sigill_x86)
DEF_NOOP_GETTER(CHECKASM_CPU_FLAG_X86, corrupt_stack_x86)
DEF_COPY_GETTER(CHECKASM_CPU_FLAG_MMX, copy_noemms_mmx)
DEF_COPY_GETTER(CHECKASM_CPU_FLAG_AVX2, copy_novzeroupper_avx2)

static void check_clobber(int from, int to)
{
    declare_func(void, int);

    for (int reg = from; reg < to; reg++) {
        noop_func *clobber = get_clobber(reg);
        if (!clobber)
            break;

        if (check_func(clobber, "clobber_r%d", reg)) {
            call_new(0);
        }
    }

    report("clobber");
}

static void test_copy_emms(copy_func fun, const char *name)
{
    CHECKASM_ALIGN(uint8_t c_dst[256]);
    CHECKASM_ALIGN(uint8_t a_dst[256]);
    CHECKASM_ALIGN(uint8_t src[256]);
    INITIALIZE_BUF(src);

    declare_func_emms(CHECKASM_CPU_FLAG_MMX, void, uint8_t *dest, const uint8_t *src,
                      size_t n);

    for (size_t w = 8; w <= 256; w *= 2) {
        if (check_func(fun, "%s_%zu", name, w)) {
            CLEAR_BUF(c_dst);
            CLEAR_BUF(a_dst);

            call_ref(c_dst, src, w);
            call_new(a_dst, src, w);
            checkasm_check(uint8_t, c_dst, 0, a_dst, 0, 256, 1, "dst data");
            bench_new(a_dst, src, w);
        }
    }

    report("%s", name);
}

void checkasm_check_x86(void)
{
    checkasm_test_copy(get_copy_x86(), "copy", 1);
    test_copy_emms(get_copy_noemms_mmx(), "copy_noemms");
    check_clobber(0, NUM_SAFE);

    if (!checkasm_should_fail(1))
        return;
    checkasm_test_noop(get_sigill_x86(), "sigill");
    checkasm_test_noop(get_corrupt_stack_x86(), "corrupt_stack");
    checkasm_test_copy(get_copy_noemms_mmx(), "noemms", 8);
    check_clobber(NUM_SAFE, NUM_REGS);

    if (has_vzeroupper_check)
        checkasm_test_copy(get_copy_novzeroupper_avx2(), "novzeroupper", 32);
}
