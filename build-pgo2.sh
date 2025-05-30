#1/bin/sh
set -ex
rm -rf $HOME/clang-stage1
COMPILER_LAUNCHER=ccache LLVM_CMAKEFLAGS="-DLLVM_USE_RELATIVE_PATHS_IN_FILES=ON" ./build-all.sh $HOME/clang-stage1 --disable-lldb --disable-clang-tools-extra
time ./build-compiler-rt.sh --native $HOME/clang-stage1

time (export PATH=$HOME/clang-stage1/bin:$PATH; CLEAN=1 ./build-llvm.sh /tmp/clang-profile-dummy --disable-lldb --disable-clang-tools-extra --with-clang --disable-dylib --instrumented=Frontend)

time ./pgo-training.sh llvm-project/llvm/build-instrumented $HOME/clang-stage1

rm -rf $HOME/clang-pgo
time cp -a $HOME/clang-stage1 $HOME/clang-pgo
time (export PATH=$HOME/clang-stage1/bin:$PATH; CLEAN=1 ./build-llvm.sh $HOME/clang-pgo --with-clang --thinlto --profile=profile.profdata; ./build-lldb-mi.sh $HOME/clang-pgo; ./strip-llvm.sh $HOME/clang-pgo)



# ./build-all.sh --stage1 $HOME/clang-stage1
#     --disable-lldb --disable-clang-tools-extra

# ./build-all.sh --profile $HOME/clang-stage1 $HOME/clang-profile
#   build-compiler-rt.sh --native
#   build-llvm (BUILDDIR=...)
#   pgo-training.sh

# ./build-all.sh --pgo $HOME/clang-stage1 $HOME/clang-pgo
#  cp -a stage1
#  --llvm-only

# ./build-all.sh --full-pgo $HOME/clang-stage1 $HOME/clang-pgo
# --stage1, --profile, --full-pgo
# CLEAN=1 for later stages
# unset COMPILER_LAUNCHER after first step

# fold -DLLVM_USE_RELATIVE_PATHS_IN_FILES into build-llvm.sh

