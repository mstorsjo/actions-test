FROM ubuntu:24.04

RUN apt-get update -qq && \
    DEBIAN_FRONTEND="noninteractive" apt-get install -qqy --no-install-recommends \
    git wget bzip2 file unzip libtool pkg-config cmake build-essential \
    automake nasm gettext autopoint vim-tiny python3 \
    ninja-build ca-certificates curl less zip && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*


RUN git config --global user.name "LLVM MinGW" && \
    git config --global user.email root@localhost

WORKDIR /build

ENV TOOLCHAIN_PREFIX=/opt/llvm-mingw

ARG TOOLCHAIN_ARCHS="i686 x86_64 armv7 aarch64"

ARG DEFAULT_CRT=ucrt

ARG CFGUARD_ARGS=--enable-cfguard

COPY llvm-stage1.tar.xz ./
RUN mkdir -p /opt && \
    tar -Jxvf llvm-stage1.tar.xz -C /opt && \
    rm llvm-stage1.tar.xz

ENV PATH=/opt/llvm/bin:$PATH

COPY build-compiler-rt.sh build-llvm.sh ./
RUN ./build-compiler-rt.sh --native /opt/llvm

# Build everything that uses the llvm monorepo. We need to build the mingw runtime before the compiler-rt/libunwind/libcxxabi/libcxx runtimes.
COPY build-llvm.sh build-lldb-mi.sh strip-llvm.sh install-wrappers.sh build-mingw-w64.sh build-mingw-w64-tools.sh build-compiler-rt.sh build-libcxx.sh build-mingw-w64-libraries.sh build-openmp.sh ./
COPY wrappers/*.sh wrappers/*.c wrappers/*.h wrappers/*.cfg ./wrappers/
RUN LLVM_PROFILE_DATA_DIR=/tmp/llvm-profiles ./build-llvm.sh $TOOLCHAIN_PREFIX --stage2 --thinlto --instrumented=IR && \
    ./build-lldb-mi.sh $TOOLCHAIN_PREFIX && \
    ./strip-llvm.sh $TOOLCHAIN_PREFIX && \
    ./install-wrappers.sh $TOOLCHAIN_PREFIX && \
    ./build-mingw-w64.sh $TOOLCHAIN_PREFIX --with-default-msvcrt=$DEFAULT_CRT $CFGUARD_ARGS && \
    ./build-mingw-w64-tools.sh $TOOLCHAIN_PREFIX && \
    ./build-compiler-rt.sh $TOOLCHAIN_PREFIX $CFGUARD_ARGS && \
    ./build-libcxx.sh $TOOLCHAIN_PREFIX $CFGUARD_ARGS && \
    ./build-mingw-w64-libraries.sh $TOOLCHAIN_PREFIX $CFGUARD_ARGS && \
    ./build-compiler-rt.sh $TOOLCHAIN_PREFIX --build-sanitizers && \
    ./build-openmp.sh $TOOLCHAIN_PREFIX $CFGUARD_ARGS && \
    rm -rf /build/*

ENV PATH=$TOOLCHAIN_PREFIX/bin:$PATH

COPY test/ ./test/

RUN curl -LO https://sqlite.org/2025/sqlite-amalgamation-3480000.zip && \
    unzip sqlite-*.zip && \
    rm sqlite-*.zip && \
    mv sqlite-* sqlite

RUN rm -rf /tmp/llvm-profiles && \
    for arch in i686 x86_64 armv7 aarch64; do \
        $arch-w64-mingw32-clang -O3 sqlite/sqlite3.c sqlite/shell.c -o sqlite3-$arch.exe && \
        $arch-w64-mingw32-clang++ -O3 test/hello-exception.cpp -o hello-exception-$arch.exe; \
    done && \
    llvm-profdata merge -output profile.profdata /tmp/llvm-profiles/*.profraw && \
    ls -lh profile.profdata
