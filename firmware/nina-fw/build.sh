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
    git -C "$source_dir" apply --check "$dir/patches/0002-report-power-save-result.patch"
    git -C "$source_dir" apply "$dir/patches/0002-report-power-save-result.patch"
fi

if [ ! -d "$source_dir/.git" ] || [ "$(git -C "$source_dir" rev-parse HEAD)" != "$revision" ]; then
    echo "Expected the pinned source in $source_dir; inspect or move the existing directory." >&2
    exit 1
fi
git -C "$source_dir" apply --reverse --check "$dir/patches/0001-fix-socket-poll-exception-set.patch"
git -C "$source_dir" apply --reverse --check "$dir/patches/0002-report-power-save-result.patch"

# Refuse to silently include local experiments in the reproducible firmware.
# Use a temporary index so the source checkout and its real index stay untouched.
index_file=$(mktemp)
trap 'rm -f "$index_file"' EXIT HUP INT TERM
rm -f "$index_file"
GIT_INDEX_FILE="$index_file" git -C "$source_dir" read-tree HEAD
GIT_INDEX_FILE="$index_file" git -C "$source_dir" apply --cached "$dir/patches/0001-fix-socket-poll-exception-set.patch"
GIT_INDEX_FILE="$index_file" git -C "$source_dir" apply --cached "$dir/patches/0002-report-power-save-result.patch"
if ! GIT_INDEX_FILE="$index_file" git -C "$source_dir" diff --quiet --ignore-submodules=none; then
    echo "Unexpected tracked changes in $source_dir; inspect them before building." >&2
    GIT_INDEX_FILE="$index_file" git -C "$source_dir" diff --stat --ignore-submodules=none >&2
    exit 1
fi

docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$source_dir:/data" -w /data espressif/idf:v4.4.8 \
    -- sh -c 'git config --global --add safe.directory /opt/esp/idf && RELEASE=1 NANO_RP2040_CONNECT=1 make -j2 && python combine.py'
