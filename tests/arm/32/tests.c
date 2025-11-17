#include "tests.h"

int checkasm_has_neon(void);
int checkasm_has_vfp(void);

uint64_t checkasm_get_cpu_flags_arm(void)
{
    uint64_t flags = CHECKASM_CPU_FLAG_ARM;
    if (checkasm_has_vfp())
        flags |= CHECKASM_CPU_FLAG_VFP;
    if (checkasm_has_neon())
        flags |= CHECKASM_CPU_FLAG_NEON;
    return flags;
}

DEF_NOOP_FUNC(clobber_r0);
DEF_NOOP_FUNC(clobber_r1);
DEF_NOOP_FUNC(clobber_r2);
DEF_NOOP_FUNC(clobber_r3);
DEF_NOOP_FUNC(clobber_r4);
DEF_NOOP_FUNC(clobber_r5);
DEF_NOOP_FUNC(clobber_r6);
DEF_NOOP_FUNC(clobber_r7);
DEF_NOOP_FUNC(clobber_r8);
DEF_NOOP_FUNC(clobber_r9);
DEF_NOOP_FUNC(clobber_r10);
DEF_NOOP_FUNC(clobber_r11);
DEF_NOOP_FUNC(clobber_r12);
DEF_NOOP_FUNC(clobber_d0);
DEF_NOOP_FUNC(clobber_d1);
DEF_NOOP_FUNC(clobber_d2);
DEF_NOOP_FUNC(clobber_d3);
DEF_NOOP_FUNC(clobber_d4);
DEF_NOOP_FUNC(clobber_d5);
DEF_NOOP_FUNC(clobber_d6);
DEF_NOOP_FUNC(clobber_d7);
DEF_NOOP_FUNC(clobber_d8);
DEF_NOOP_FUNC(clobber_d9);
DEF_NOOP_FUNC(clobber_d10);
DEF_NOOP_FUNC(clobber_d11);
DEF_NOOP_FUNC(clobber_d12);
DEF_NOOP_FUNC(clobber_d13);
DEF_NOOP_FUNC(clobber_d14);
DEF_NOOP_FUNC(clobber_d15);
DEF_NOOP_FUNC(clobber_d16);
DEF_NOOP_FUNC(clobber_d17);
DEF_NOOP_FUNC(clobber_d18);
DEF_NOOP_FUNC(clobber_d19);
DEF_NOOP_FUNC(clobber_d20);
DEF_NOOP_FUNC(clobber_d21);
DEF_NOOP_FUNC(clobber_d22);
DEF_NOOP_FUNC(clobber_d23);
DEF_NOOP_FUNC(clobber_d24);
DEF_NOOP_FUNC(clobber_d25);
DEF_NOOP_FUNC(clobber_d26);
DEF_NOOP_FUNC(clobber_d27);
DEF_NOOP_FUNC(clobber_d28);
DEF_NOOP_FUNC(clobber_d29);
DEF_NOOP_FUNC(clobber_d30);
DEF_NOOP_FUNC(clobber_d31);

DEF_NOOP_FUNC(sigill_arm);
DEF_NOOP_FUNC(clobber_fpscr_vfp);

// A function with 4 parameters in registers and 1 on the stack.
typedef void(many_args_func)(int, int, int, int, int);

#define DEF_MANY_ARGS_FUNC(NAME)         void checkasm_##NAME(int, int, int, int, int)
#define DEF_MANY_ARGS_GETTER(FLAG, NAME) DEF_GETTER(FLAG, NAME, many_args_func, NULL)

DEF_MANY_ARGS_FUNC(clobber_stack_args_arm);
DEF_MANY_ARGS_FUNC(clobber_stack_arm);

static noop_func *get_clobber_r(int reg)
{
    if (!(checkasm_get_cpu_flags() & CHECKASM_CPU_FLAG_ARM))
        return NULL;

    switch (reg) {
    case 0:  return checkasm_clobber_r0;
    case 1:  return checkasm_clobber_r1;
    case 2:  return checkasm_clobber_r2;
    case 3:  return checkasm_clobber_r3;
    case 4:  return checkasm_clobber_r4;
    case 5:  return checkasm_clobber_r5;
    case 6:  return checkasm_clobber_r6;
    case 7:  return checkasm_clobber_r7;
    case 8:  return checkasm_clobber_r8;
    case 9:  return checkasm_clobber_r9;
    case 10: return checkasm_clobber_r10;
    case 11: return checkasm_clobber_r11;
    case 12: return checkasm_clobber_r12;
    case 13: return NULL; /* Shouldn't try to clobber SP */
    case 14: return NULL; /* Shouldn't try to clobber LR */
    case 15: return NULL; /* Shouldn't try to clobber PC */
    default: return NULL;
    }
}

static noop_func *get_clobber_d(int reg)
{
    if (!(checkasm_get_cpu_flags() & CHECKASM_CPU_FLAG_VFP))
        return NULL;
    if (reg >= 16 && !(checkasm_get_cpu_flags() & CHECKASM_CPU_FLAG_NEON))
        return NULL;

    switch (reg) {
    case 0:  return checkasm_clobber_d0;
    case 1:  return checkasm_clobber_d1;
    case 2:  return checkasm_clobber_d2;
    case 3:  return checkasm_clobber_d3;
    case 4:  return checkasm_clobber_d4;
    case 5:  return checkasm_clobber_d5;
    case 6:  return checkasm_clobber_d6;
    case 7:  return checkasm_clobber_d7;
    case 8:  return checkasm_clobber_d8;
    case 9:  return checkasm_clobber_d9;
    case 10: return checkasm_clobber_d10;
    case 11: return checkasm_clobber_d11;
    case 12: return checkasm_clobber_d12;
    case 13: return checkasm_clobber_d13;
    case 14: return checkasm_clobber_d14;
    case 15: return checkasm_clobber_d15;
    case 16: return checkasm_clobber_d16;
    case 17: return checkasm_clobber_d17;
    case 18: return checkasm_clobber_d18;
    case 19: return checkasm_clobber_d19;
    case 20: return checkasm_clobber_d20;
    case 21: return checkasm_clobber_d21;
    case 22: return checkasm_clobber_d22;
    case 23: return checkasm_clobber_d23;
    case 24: return checkasm_clobber_d24;
    case 25: return checkasm_clobber_d25;
    case 26: return checkasm_clobber_d26;
    case 27: return checkasm_clobber_d27;
    case 28: return checkasm_clobber_d28;
    case 29: return checkasm_clobber_d29;
    case 30: return checkasm_clobber_d30;
    case 31: return checkasm_clobber_d31;
    default: return NULL;
    }
}

DEF_NOOP_GETTER(CHECKASM_CPU_FLAG_ARM, sigill_arm)
DEF_NOOP_GETTER(CHECKASM_CPU_FLAG_VFP, clobber_fpscr_vfp)
DEF_MANY_ARGS_GETTER(CHECKASM_CPU_FLAG_ARM, clobber_stack_args_arm)
DEF_MANY_ARGS_GETTER(CHECKASM_CPU_FLAG_ARM, clobber_stack_arm)

static void check_clobber_r(int from, int to)
{
    declare_func(void, int);

    for (int reg = from; reg < to; reg++) {
        noop_func *clobber = get_clobber_r(reg);
        if (!clobber)
            break;

        if (check_func(clobber, "clobber_r%d", reg)) {
            call_new(0);
        }
    }

    report("clobber_r");
}

static void check_clobber_d(int from, int to)
{
    declare_func(void, int);

    for (int reg = from; reg < to; reg++) {
        noop_func *clobber = get_clobber_d(reg);
        if (!clobber)
            break;

        if (check_func(clobber, "clobber_d%d", reg)) {
            call_new(0);
        }
    }

    report("clobber_d");
}

static void checkasm_test_many_args(many_args_func fun, const char *name)
{
    declare_func(void, int, int, int, int, int);

    if (check_func(fun, "%s", name)) {
        /* don't call unchecked because that one is called without wrapping,
         * and we try to cloober the stack here. */
        (void) func_ref;
        call_new(1, 2, 3, 4, 5);
    }

    report("%s", name);
}

void checkasm_check_arm(void)
{
    check_clobber_r(0, 4);
    check_clobber_r(12, 13);
    check_clobber_d(0, 8);
    check_clobber_d(16, 32);
    checkasm_test_many_args(get_clobber_stack_args_arm(), "clobber_stack_args");

    if (!checkasm_should_fail(1))
        return;
    checkasm_test_noop(get_sigill_arm(), "sigill");
    checkasm_test_many_args(get_clobber_stack_arm(), "clobber_stack");
    checkasm_test_noop(get_clobber_fpscr_vfp(), "clobber_fpscr");
    check_clobber_r(4, 9);
#ifndef __APPLE__
    // In the iOS ABI, r9 is a volatile register, so we don't check if it is
    // clobbered in the checked_call wrapper.
    check_clobber_r(9, 10);
#endif
    check_clobber_r(10, 11);
    check_clobber_d(8, 16);
}
