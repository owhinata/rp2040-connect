#!/bin/sh
set -eu

dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
revision=f9491329a818bc7f36cdda465b4bea53b03f4bda
source_dir="$dir/source"

if [ ! -e "$source_dir" ]; then
    git clone --branch 3.0.2 --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/arduino/nina-fw.git "$source_dir"
    if [ "$(git -C "$source_dir" rev-parse HEAD)" != "$revision" ]; then
        echo "Unexpected upstream revision in $source_dir" >&2
        exit 1
    fi
    git -C "$source_dir" apply --check "$dir/patches/0001-fix-socket-poll-exception-set.patch"
    git -C "$source_dir" apply "$dir/patches/0001-fix-socket-poll-exception-set.patch"
fi

if [ ! -d "$source_dir/.git" ] || [ "$(git -C "$source_dir" rev-parse HEAD)" != "$revision" ]; then
    echo "Expected the pinned source in $source_dir; inspect or move the existing directory." >&2
    exit 1
fi
git -C "$source_dir" apply --reverse --check "$dir/patches/0001-fix-socket-poll-exception-set.patch"

docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$source_dir:/data" -w /data espressif/idf:v4.4.8 \
    -- sh -c 'git config --global --add safe.directory /opt/esp/idf && RELEASE=1 NANO_RP2040_CONNECT=1 make -j2 && python combine.py'
