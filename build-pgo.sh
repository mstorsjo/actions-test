#1/bin/sh
set -ex
#./build-llvm-local.sh $HOME/clang-stage1
rm -rf $HOME/clang-stage1
./build-llvm.sh $HOME/clang-stage1 --disable-lldb --disable-clang-tools-extra
time ./build-compiler-rt.sh --native $HOME/clang-stage1
rm -rf $HOME/clang-profile
time (export PATH=$HOME/clang-stage1/bin:$PATH; LLVM_PROFILE_DATA_DIR=/tmp/llvm-profile CLEAN=1 ./build-all.sh $HOME/clang-profile --disable-lldb --disable-clang-tools-extra --stage2 --thinlto --instrumented=Frontend)
rm -rf profile.profdata
time ./pgo-training.sh $HOME/clang-profile
rm -rf $HOME/clang-pgo
time (export PATH=$HOME/clang-stage1/bin:$PATH; LLVM_PROFDATA_FILE=$(pwd)/profile.profdata CLEAN=1 ./build-all.sh $HOME/clang-pgo --stage2 --thinlto)

