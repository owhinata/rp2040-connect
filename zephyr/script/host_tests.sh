#!/usr/bin/env bash
# Build and run the host tests. They use fake Zephyr headers and need no board.
# The tests of the sources shared with the NuttX build run first (common/tests/run.sh).
set -euo pipefail
cd "$(dirname "$0")/.."
../common/tests/run.sh
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
app=app/usb-console
flags=(-std=c17 -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
link="-Wl,--gc-sections"

run() {
    local name=$1
    shift
    cc "$@" "$link" -o "$out/$name"
    "$out/$name" >/dev/null
    printf 'ok  %s\n' "$name"
}

# NINA SPI framing of this build's drivers/nina/nina_wifi.c (common/tests/run.sh tests the shared
# common/drivers/nina/nina_spi.c as nina_spi); nina_proto.h is in common/drivers/nina.
run nina_wifi_spi -std=gnu17 -Wall -I"$app/tests/nina_fakes" -I../common/drivers/nina "$app/tests/nina_spi.c"

for f in ../host/imu_host/*.py; do
    python3 -c 'import sys; compile(open(sys.argv[1]).read(), sys.argv[1], "exec")' "$f"
done
printf 'ok  host/imu_host (syntax)\n'
