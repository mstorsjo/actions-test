#include "tests.h"

int checkasm_get_vlenb(void);

uint64_t checkasm_get_cpu_flags_riscv(void)
{
    uint64_t flags = CHECKASM_CPU_FLAG_RISCV;
    if (checkasm_get_vlenb())
        flags |= CHECKASM_CPU_FLAG_RVV;
    return flags;
}

DEF_COPY_FUNC(copy_rvv);
DEF_COPY_GETTER(CHECKASM_CPU_FLAG_RVV, copy_rvv)

DEF_NOOP_FUNC(clobber_ra);
DEF_NOOP_FUNC(clobber_sp);
DEF_NOOP_FUNC(clobber_gp);
DEF_NOOP_FUNC(clobber_t0);
DEF_NOOP_FUNC(clobber_t1);
DEF_NOOP_FUNC(clobber_t2);
DEF_NOOP_FUNC(clobber_s0);
DEF_NOOP_FUNC(clobber_s1);
DEF_NOOP_FUNC(clobber_a0);
DEF_NOOP_FUNC(clobber_a1);
DEF_NOOP_FUNC(clobber_a2);
DEF_NOOP_FUNC(clobber_a3);
DEF_NOOP_FUNC(clobber_a4);
DEF_NOOP_FUNC(clobber_a5);
DEF_NOOP_FUNC(clobber_a6);
DEF_NOOP_FUNC(clobber_a7);
DEF_NOOP_FUNC(clobber_s2);
DEF_NOOP_FUNC(clobber_s3);
DEF_NOOP_FUNC(clobber_s4);
DEF_NOOP_FUNC(clobber_s5);
DEF_NOOP_FUNC(clobber_s6);
DEF_NOOP_FUNC(clobber_s7);
DEF_NOOP_FUNC(clobber_s8);
DEF_NOOP_FUNC(clobber_s9);
DEF_NOOP_FUNC(clobber_s10);
DEF_NOOP_FUNC(clobber_s11);
DEF_NOOP_FUNC(clobber_t3);
DEF_NOOP_FUNC(clobber_t4);
DEF_NOOP_FUNC(clobber_t5);
DEF_NOOP_FUNC(clobber_t6);

DEF_NOOP_FUNC(sigill_riscv);
DEF_NOOP_FUNC(corrupt_stack_riscv);
DEF_NOOP_GETTER(CHECKASM_CPU_FLAG_RISCV, sigill_riscv)
DEF_NOOP_GETTER(CHECKASM_CPU_FLAG_RISCV, corrupt_stack_riscv)

typedef struct RiscvRegister {
    const char *name;
    noop_func  *clobber;
} RiscvRegister;

static const RiscvRegister registers_safe[] = {
    { "ra", checkasm_clobber_ra },
    { "t0", checkasm_clobber_t0 },
    { "t1", checkasm_clobber_t1 },
    { "t2", checkasm_clobber_t2 },
    { "t3", checkasm_clobber_t3 },
    { "t4", checkasm_clobber_t4 },
    { "t5", checkasm_clobber_t5 },
    { "t6", checkasm_clobber_t6 },
    { "a0", checkasm_clobber_a0 },
    { "a1", checkasm_clobber_a1 },
    { "a2", checkasm_clobber_a2 },
    { "a3", checkasm_clobber_a3 },
    { "a4", checkasm_clobber_a4 },
    { "a5", checkasm_clobber_a5 },
    { "a6", checkasm_clobber_a6 },
    { "a7", checkasm_clobber_a7 },
    { NULL, NULL                }
};

static const RiscvRegister registers_unsafe[] = {
    { "s0",  checkasm_clobber_s0  },
    { "s1",  checkasm_clobber_s1  },
    { "s2",  checkasm_clobber_s2  },
    { "s3",  checkasm_clobber_s3  },
    { "s4",  checkasm_clobber_s4  },
    { "s5",  checkasm_clobber_s5  },
    { "s6",  checkasm_clobber_s6  },
    { "s7",  checkasm_clobber_s7  },
    { "s8",  checkasm_clobber_s8  },
    { "s9",  checkasm_clobber_s9  },
    { "s10", checkasm_clobber_s10 },
    { "s11", checkasm_clobber_s11 },
    { "sp",  checkasm_clobber_sp  },
    { "gp",  checkasm_clobber_gp  },
    /* Can't clobber tp because checkasm.S saves registers in TLS */
    { NULL,  NULL                 }
};

static void check_clobber(const RiscvRegister *registers)
{
    const uint64_t flag = checkasm_get_cpu_flags() & CHECKASM_CPU_FLAG_RISCV;
    declare_func(void, int);

    for (int i = 0; registers[i].name; i++) {
        noop_func *const func = flag ? registers[i].clobber : NULL;
        if (check_func(func, "clobber_%s", registers[i].name)) {
            call_new(0);
        }
    }

    report("clobber");
}

void checkasm_check_riscv(void)
{
    checkasm_test_copy(get_copy_rvv(), "copy_rvv", 1);
    check_clobber(registers_safe);

#if ARCH_RV64
    if (!checkasm_should_fail(1))
        return;
    checkasm_test_noop(get_sigill_riscv(), "sigill");
    checkasm_test_noop(get_corrupt_stack_riscv(), "corrupt_stack");
    check_clobber(registers_unsafe);
#endif
}
