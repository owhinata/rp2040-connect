#!/bin/sh
# Host tests of the firmware's ESP-IDF-free parts, built with the host's cc
# against the patched source checkout (firmware/nina-fw/source, patch 0006).
set -eu

dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
main="$dir/../source/main"
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT HUP INT TERM

if [ ! -f "$main/SocketEvents.c" ]; then
    echo "No $main/SocketEvents.c: build the patched source first (build.sh)." >&2
    exit 1
fi

${CC:-cc} -std=c11 -Wall -Wextra -Werror -I"$main" \
    "$dir/socket_events_test.c" "$main/SocketEvents.c" -o "$out/socket_events_test"
"$out/socket_events_test"
