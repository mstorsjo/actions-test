#!/bin/bash

set -e

AVMDEC="$1"
if [ ! -e "$AVMDEC" ]; then
    echo $0 path/to/avmdec
    exit 1
fi

SRC_PATH="$(dirname "$0")/.."

for file in "$SRC_PATH"/media/*.obu; do
    base="$(basename "$file")"
    a_md5=$("$AVMDEC" "$file" --rawvideo --md5 | cut -d' ' -f1)
    echo "$a_md5" > "$file.md5"
done
