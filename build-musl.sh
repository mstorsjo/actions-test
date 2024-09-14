#!/bin/sh
#
# Copyright (c) 2023 Martin Storsjo
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

set -e

: ${MUSL_VERSION:=f314e133929b6379eccc632bef32eaebb66a7335}

FLAGS=""
while [ $# -gt 0 ]; do
    case "$1" in
    --headers-only)
        HEADERS_ONLY=1
        ;;
    --disable-shared)
        FLAGS="$FLAGS $1"
        ;;
    --disable-static)
        FLAGS="$FLAGS $1"
        ;;
    *)
        PREFIX="$1"
        ;;
    esac
    shift
done
if [ -z "$CHECKOUT_ONLY" ]; then
    if [ -z "$PREFIX" ]; then
        echo "$0 [--headers-only] dest"
        exit 1
    fi

    mkdir -p "$PREFIX"
    PREFIX="$(cd "$PREFIX" && pwd)"
fi

if [ ! -d musl ]; then
    # Unofficial github mirror at https://github.com/bminor/musl
    #git clone https://git.musl-libc.org/git/musl
    git clone https://github.com/bminor/musl
    CHECKOUT=1
fi

cd musl

if [ -n "$SYNC" ] || [ -n "$CHECKOUT" ]; then
    [ -z "$SYNC" ] || git fetch
    git checkout $MUSL_VERSION
fi

[ -z "$CHECKOUT_ONLY" ] || exit 0

MAKE=make
if command -v gmake >/dev/null; then
    MAKE=gmake
fi

export PATH="$PREFIX/bin:$PATH"

unset CC

: ${CORES:=$(nproc 2>/dev/null)}
: ${CORES:=$(sysctl -n hw.ncpu 2>/dev/null)}
: ${CORES:=4}
: ${ARCHS:=${TOOLCHAIN_ARCHS-i386 x86_64 arm aarch64 powerpc64le riscv64}}

mkdir -p $PREFIX/generic-linux-musl/usr/include
mkdir -p $PREFIX/generic-linux-musl/usr/lib

for arch in $ARCHS; do
    triple=$arch-linux-musl
    multiarch_triple=$arch-linux-gnu
    musl_arch=$arch
    case $arch in
    arm*)
        triple=$arch-linux-musleabihf
        multiarch_triple=$arch-linux-gnueabihf
        musl_arch=armhf
        ;;
    i*86)
        multiarch_triple=i386-linux-gnu
        musl_arch=i386
        ;;
    esac

    [ -z "$CLEAN" ] || rm -rf build-$arch
    mkdir -p build-$arch
    cd build-$arch
    includes="$PREFIX/generic-linux-musl/usr/include"

    mkdir -p $includes/$multiarch_triple/asm
    mkdir -p $includes/$multiarch_triple/bits
    ln -sfn $multiarch_triple/asm "$includes/asm"
    ln -sfn $multiarch_triple/bits "$includes/bits"

    ../configure --target=$triple --prefix="$PREFIX/generic-linux-musl/usr" --libdir="$PREFIX/generic-linux-musl/usr/lib/$multiarch_triple" --syslibdir="$PREFIX/generic-linux-musl/lib" --disable-wrapper $FLAGS
    if [ -n "$HEADERS_ONLY" ]; then
        $MAKE -j$CORES install-headers
    else
        $MAKE -j$CORES
        $MAKE -j$CORES install
        # Convert the ld-musl-*.so.1 symlink from an absolute symlink into
        # a relative one. (Note, this use of readlink is specific to
        # GNU coreutils.)
        ln -fs $(realpath --relative-to=$PREFIX/generic-linux-musl/lib $(readlink $PREFIX/generic-linux-musl/lib/ld-musl-$musl_arch.so.1)) $PREFIX/generic-linux-musl/lib/ld-musl-$musl_arch.so.1
    fi

    rm -f "$includes/asm"
    rm -f "$includes/bits"

    cd ..
done
mkdir -p "$PREFIX/generic-linux-musl/share/musl"
install -m644 COPYRIGHT "$PREFIX/generic-linux-musl/share/musl/COPYRIGHT.txt"
