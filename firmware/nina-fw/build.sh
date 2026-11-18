#!/bin/sh
set -eu

dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
revision=f9491329a818bc7f36cdda465b4bea53b03f4bda
source_dir="$dir/source"

# Applied in this order on top of the pinned source; each one can be left
# out together with the ones after it.
patches="
0001-fix-socket-poll-exception-set.patch
0002-report-power-save-result.patch
0003-add-capabilities-command.patch
0004-keep-link-during-scan.patch
0005-asynchronous-scan.patch
0006-socket-events-on-irq-line.patch
"
count=$(echo $patches | wc -w)

if [ ! -e "$source_dir" ]; then
    git clone --branch 3.0.2 --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/arduino/nina-fw.git "$source_dir"
    if [ "$(git -C "$source_dir" rev-parse HEAD)" != "$revision" ]; then
        echo "Unexpected upstream revision in $source_dir" >&2
        exit 1
    fi
fi

if [ ! -d "$source_dir/.git" ] || [ "$(git -C "$source_dir" rev-parse HEAD)" != "$revision" ]; then
    echo "Expected the pinned source in $source_dir; inspect or move the existing directory." >&2
    exit 1
fi

# A temporary index holds the pinned source with the first N patches, so the
# source checkout and its real index stay untouched.
index_file=$(mktemp)
trap 'rm -f "$index_file"' EXIT HUP INT TERM

with_patches() {
    n=$1
    rm -f "$index_file"
    GIT_INDEX_FILE="$index_file" git -C "$source_dir" read-tree HEAD
    i=0
    for p in $patches; do
        [ "$i" -lt "$n" ] || break
        GIT_INDEX_FILE="$index_file" git -C "$source_dir" apply --cached "$dir/patches/$p"
        i=$((i + 1))
    done
}

# Whether the checkout is the pinned source with exactly the first N patches:
# no other change to tracked files, and no file a later patch adds.
is_at() {
    with_patches "$1"
    GIT_INDEX_FILE="$index_file" git -C "$source_dir" diff --quiet --ignore-submodules=none &&
        [ -z "$(GIT_INDEX_FILE="$index_file" git -C "$source_dir" ls-files --others \
            --exclude-standard -- main arduino)" ]
}

# How many patches the checkout has (a first clone has none, a checkout made
# before the later patches existed has the first ones), then the rest.
applied=$count
while [ "$applied" -ge 0 ] && ! is_at "$applied"; do
    applied=$((applied - 1))
done
if [ "$applied" -lt 0 ]; then
    # Refuse to silently include local experiments in the reproducible firmware.
    echo "Unexpected tracked changes in $source_dir; inspect them before building." >&2
    with_patches "$count"
    GIT_INDEX_FILE="$index_file" git -C "$source_dir" diff --stat --ignore-submodules=none >&2
    exit 1
fi
i=0
for p in $patches; do
    if [ "$i" -ge "$applied" ]; then
        git -C "$source_dir" apply --check "$dir/patches/$p"
        git -C "$source_dir" apply "$dir/patches/$p"
        echo "Applied $p"
    fi
    i=$((i + 1))
done
if ! is_at "$count"; then
    echo "The patches did not give the expected source in $source_dir." >&2
    exit 1
fi

docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$source_dir:/data" -w /data espressif/idf:v4.4.8 \
    -- sh -c 'git config --global --add safe.directory /opt/esp/idf && RELEASE=1 NANO_RP2040_CONNECT=1 make -j2 && python combine.py'
