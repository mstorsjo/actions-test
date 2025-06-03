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

CC = $(PREFIX)/bin/clang
CXX = $(PREFIX)/bin/clang++

all:

hello-exception-opt.exe: test/hello-exception.cpp
	$(CXX) $+ -o $@ -O3

hello-exception.exe: test/hello-exception.cpp
	$(CXX) $+ -o $@

sqlite-opt.exe: $(SQLITE)/sqlite3.c $(SQLITE)/shell.c
	$(CC) $+ -o $@ -O3 -lm

sqlite.exe: $(SQLITE)/sqlite3.c $(SQLITE)/shell.c
	$(CC) $+ -o $@ -lm

LIBCXXTEST = llvm-project/libcxx/test/std/algorithms/alg.sorting/alg.sort/sort/sort.pass.cpp

libcxxtest-opt.exe: $(LIBCXXTEST)
	$(CXX) $+ -o $@ -Illvm-project/libcxx/test/support -O3

libcxxtest.exe: $(LIBCXXTEST)
	$(CXX) $+ -o $@ -Illvm-project/libcxx/test/support

TARGETS = hello-exception hello-exception-opt

ifneq ($(SQLITE),)
TARGETS += sqlite sqlite-opt
endif
ifneq ($(wildcard $(LIBCXXTEST)),)
TARGETS += libcxxtest libcxxtest-opt
endif

ALLTARGETS = $(addsuffix .exe, $(TARGETS))

all: $(ALLTARGETS)

clean:
	rm -f $(ALLTARGETS)
