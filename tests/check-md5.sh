#!/bin/bash

set -e

DAV2D="$1"
if [ ! -e "$DAV2D" ]; then
    echo $0 path/to/dav2d
    exit 1
fi

SRC_PATH="$(dirname "$0")/.."

for file in "$SRC_PATH"/media/*.obu; do
    base="$(basename "$file")"
    a_md5=$(cat "$file.md5")
    d_md5=$("$DAV2D" -i "$file" -o - --threads=1 --quiet --muxer=md5 --filmgrain=1)
    if [[ "$a_md5" == "$d_md5" ]]; then
        echo $base "[OK] md5:$a_md5"
    else
        echo $base "[FAILED!]"
        diff <(echo "$a_md5") <(echo "$d_md5")
    fi
done
