#!/bin/sh
#
# Copyright (c) 2018 Martin Storsjo
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

unset HOST

while [ $# -gt 0 ]; do
    case "$1" in
    --host=*)
        HOST="${1#*=}"
        ;;
    *)
        PREFIX="$1"
        ;;
    esac
    shift
done
if [ -z "$PREFIX" ]; then
    echo $0 [--host=triple] dest
    exit 1
fi
mkdir -p "$PREFIX"
PREFIX="$(cd "$PREFIX" && pwd)"

: ${ARCHS:=${TOOLCHAIN_ARCHS-i386 x86_64 arm aarch64 powerpc64le riscv64}}

if [ -n "$HOST" ] && [ -z "$CC" ]; then
    CC=$HOST-gcc
fi
: ${CC:=cc}

if [ -n "$HOST" ]; then
    case $HOST in
    *-mingw32)
        EXEEXT=.exe
        ;;
    esac
else
    case $(uname) in
    MINGW*)
        EXEEXT=.exe
        ;;
    esac
fi

if [ -n "$MACOS_REDIST" ]; then
    : ${MACOS_REDIST_ARCHS:=arm64 x86_64}
    : ${MACOS_REDIST_VERSION:=10.9}
    for arch in $MACOS_REDIST_ARCHS; do
        WRAPPER_FLAGS="$WRAPPER_FLAGS -arch $arch"
    done
    WRAPPER_FLAGS="$WRAPPER_FLAGS -mmacosx-version-min=$MACOS_REDIST_VERSION"
fi

if [ -n "$EXEEXT" ]; then
    CLANG_MAJOR=$(basename $(echo $PREFIX/lib/clang/* | awk '{print $NF}') | cut -f 1 -d .)
    WRAPPER_FLAGS="$WRAPPER_FLAGS -municode -DCLANG=\"clang-$CLANG_MAJOR\""
fi

mkdir -p "$PREFIX/bin"
cp wrappers/*-wrapper.sh "$PREFIX/bin"
if [ -n "$HOST" ] && [ -n "$EXEEXT" ]; then
    # TODO: If building natively on msys, pick up the default HOST value from there.
    WRAPPER_FLAGS="$WRAPPER_FLAGS -DDEFAULT_TARGET=\"$HOST\""
    for i in wrappers/*-wrapper.sh; do
        cat $i | sed 's/^DEFAULT_TARGET=.*/DEFAULT_TARGET='$HOST/ > "$PREFIX/bin/$(basename $i)"
    done
fi
$CC wrappers/clang-target-wrapper.c -o "$PREFIX/bin/clang-target-wrapper$EXEEXT" -O2 -Wl,-s $WRAPPER_FLAGS
$CC wrappers/llvm-wrapper.c -o "$PREFIX/bin/llvm-wrapper$EXEEXT" -O2 -Wl,-s $WRAPPER_FLAGS
if [ -n "$EXEEXT" ]; then
    # For Windows, we should prefer the executable wrapper, which also works
    # when invoked from outside of MSYS.
    CTW_SUFFIX=$EXEEXT
    CTW_LINK_SUFFIX=$EXEEXT
else
    CTW_SUFFIX=.sh
fi
cd "$PREFIX/bin"
for arch in $ARCHS; do
    triple=$arch-linux-musl
    case $arch in
    arm*)
        triple=$arch-linux-musleabihf
        ;;
    esac
    for exec in clang clang++ gcc g++ c++ as; do
        ln -sf clang-target-wrapper$CTW_SUFFIX $triple-$exec$CTW_LINK_SUFFIX
    done
    for exec in addr2line ar ranlib nm objcopy objdump readelf size strings strip llvm-ar llvm-ranlib; do
        if [ -n "$EXEEXT" ]; then
            link_target=llvm-wrapper
        else
            case $exec in
            llvm-*)
                link_target=$exec
                ;;
            *)
                link_target=llvm-$exec
                ;;
            esac
        fi
        ln -sf $link_target$EXEEXT $triple-$exec$EXEEXT || true
    done
    for exec in ld; do
        ln -sf $exec-wrapper.sh $triple-$exec
    done
done
if [ -n "$EXEEXT" ]; then
    if [ ! -L clang$EXEEXT ] && [ -f clang$EXEEXT ] && [ ! -f clang-$CLANG_MAJOR$EXEEXT ]; then
        mv clang$EXEEXT clang-$CLANG_MAJOR$EXEEXT
    fi
fi
