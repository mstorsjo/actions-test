#!/bin/sh
#
# Copyright (c) 2025 Martin Storsjo
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

set -ex

: ${SQLITE_VERSION:=3490200}
: ${SQLITE_YEAR:=2025}

: ${LLVM_PROFILE_DATA_DIR:=/tmp/llvm-profile}

if [ $# -lt 1 ]; then
    echo $0 build
    exit 1
fi
PREFIX="$1"
PREFIX="$(cd "$PREFIX" && pwd)"

MAKE=make
if command -v gmake >/dev/null; then
    MAKE=gmake
fi

: ${CORES:=$(nproc 2>/dev/null)}
: ${CORES:=$(sysctl -n hw.ncpu 2>/dev/null)}
: ${CORES:=4}
: ${ARCHS:=${TOOLCHAIN_ARCHS-i686 x86_64 armv7 aarch64}}

download() {
    if command -v curl >/dev/null; then
        curl -LO "$1"
    else
        wget "$1"
    fi
}

SQLITE=sqlite-amalgamation-$SQLITE_VERSION
if [ ! -d $SQLITE ]; then
    download https://sqlite.org/$SQLITE_YEAR/sqlite-amalgamation-$SQLITE_VERSION.zip
    unzip sqlite-amalgamation-$SQLITE_VERSION.zip
fi

rm -rf "$LLVM_PROFILE_DATA_DIR"
$MAKE -f pgo-training.make PREFIX=$PREFIX SQLITE=$SQLITE clean
$MAKE -f pgo-training.make PREFIX=$PREFIX SQLITE=$SQLITE -j$CORES

rm -f profile.profdata
llvm-profdata merge -output profile.profdata $LLVM_PROFILE_DATA_DIR/*.profraw
