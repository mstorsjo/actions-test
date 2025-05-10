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

hello-exception-%.exe: test/hello-exception.cpp
	$*-w64-mingw32-clang++ $+ -o $@

hello-exception-opt-%.exe: test/hello-exception.cpp
	$*-w64-mingw32-clang++ $+ -o $@ -O3

sqlite-%.exe: $(SQLITE)/sqlite3.c $(SQLITE)/shell.c
	$*-w64-mingw32-clang $+ -o $@

sqlite-opt-%.exe: $(SQLITE)/sqlite3.c $(SQLITE)/shell.c
	$*-w64-mingw32-clang $+ -o $@ -O3

LIBCXXTEST = llvm-project/libcxx/test/std/algorithms/alg.sorting/alg.sort/sort/sort.pass.cpp

libcxxtest-%.exe: $(LIBCXXTEST)
	$*-w64-mingw32-clang++ $+ -o $@ -Illvm-project/libcxx/test/support

libcxxtest-opt-%.exe: $(LIBCXXTEST)
	$*-w64-mingw32-clang++ $+ -o $@ -Illvm-project/libcxx/test/support -O3

ARCHS ?= i686 x86_64 armv7 aarch64

TARGETS = hello-exception hello-exception-opt

ifneq ($(SQLITE),)
TARGETS += sqlite sqlite-opt
endif
ifneq ($(wildcard $(LIBCXXTEST)),)
TARGETS += libcxxtest libcxxtest-opt
endif

ALLTARGETS = $(foreach arch, $(ARCHS), $(foreach target, $(TARGETS), $(target)-$(arch).exe))

all: $(ALLTARGETS)

clean:
	rm -f $(ALLTARGETS)
