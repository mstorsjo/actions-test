#include <stdio.h>
#include <windows.h>

#ifndef PF_ARM_V83_LRCPC_INSTRUCTIONS_AVAILABLE
#define PF_ARM_V83_LRCPC_INSTRUCTIONS_AVAILABLE 45
#endif

struct feat {
    const char *name;
    int flag;
} feats[] = {
#define F(x) { #x, x }
    F(PF_ARM_VFP_32_REGISTERS_AVAILABLE),
    F(PF_ARM_NEON_INSTRUCTIONS_AVAILABLE),
    F(PF_ARM_DIVIDE_INSTRUCTION_AVAILABLE),
    F(PF_ARM_64BIT_LOADSTORE_ATOMIC),
    F(PF_ARM_EXTERNAL_CACHE_AVAILABLE),
    F(PF_ARM_FMAC_INSTRUCTIONS_AVAILABLE),
    F(PF_ARM_V8_INSTRUCTIONS_AVAILABLE),
    F(PF_ARM_V8_CRYPTO_INSTRUCTIONS_AVAILABLE),
    F(PF_ARM_V8_CRC32_INSTRUCTIONS_AVAILABLE),
    F(PF_ARM_V81_ATOMIC_INSTRUCTIONS_AVAILABLE),
    F(PF_ARM_V82_DP_INSTRUCTIONS_AVAILABLE),
    F(PF_ARM_V83_JSCVT_INSTRUCTIONS_AVAILABLE),
    F(PF_ARM_V83_LRCPC_INSTRUCTIONS_AVAILABLE),
    { NULL, 0 }
};

int main(int argc, char* argv[]) {
    for (int i = 0; feats[i].name; i++) {
        printf("%s: %d\n", feats[i].name, IsProcessorFeaturePresent(feats[i].flag));
    }
    return 0;
}

