#!/bin/bash
#
# Copyright (c) 2023 Huang Qinjin
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

. "${0%/*}/test.sh"


cat >test.h <<EOF
EOF

cat >test.c <<EOF
#include "test.h"
 	 #line 5 __FILE__
const char* file = __FILE__;
EOF

cat >test-arm.asm <<EOF
        AREA |.text|, CODE, READONLY, ALIGN=4, CODEALIGN
        ALIGN 4
        EXPORT func
func PROC
        nop
        ENDP
        END
EOF

cat >test-x86.asm <<EOF
_TEXT SEGMENT ALIGN(16) 'CODE'
PUBLIC func
func PROC
        ret
func ENDP
_TEXT ENDS
END
EOF


ARCH=$(. "${BIN}msvcenv.sh" && echo $ARCH)


EXEC "" ${BIN}cl ${TESTS}hello.c


EXEC cl-showIncludes ${BIN}cl /nologo /showIncludes /c test.c
DIFF cl-showIncludes.out - <<EOF
test.c
Note: including file: ${CWD}test.h
EOF


EXEC cl-showIncludes-E-FC ${BIN}cl /nologo /showIncludes /E /FC test.c
DIFF cl-showIncludes-E-FC.out - <<EOF
#line 1 "${CWD}test.c"
#line 1 "${CWD}test.h"
#line 2 "${CWD}test.c"
 	 #line 5 "${CWD}test.c"
const char* file = "Z:${CWD//\//\\\\}test.c";
EOF
DIFF cl-showIncludes-E-FC.err - <<EOF
test.c
Note: including file: ${CWD}test.h
EOF


EXEC cl-P-Fi ${BIN}cl /nologo /P /Ficl-P-Fi.i ./test.c
DIFF cl-P-Fi.i - <<EOF
#line 1 "./test.c"
#line 1 "${CWD}test.h"
#line 2 "./test.c"
 	 #line 5 "./test.c"
const char* file = "./test.c";
EOF

case $ARCH in
x86)
    EXEC "" ${BIN}ml /c /Fo test-$ARCH.obj test-x86.asm
    ;;
x64)
    EXEC "" ${BIN}ml64 /c /Fo test-$ARCH.obj test-x86.asm
    ;;
arm)
    EXEC "" ${BIN}armasm test-arm.asm test-$ARCH.obj
    ;;
arm64)
    EXEC "" ${BIN}armasm64 test-arm.asm test-$ARCH.obj
    ;;
esac

EXIT
