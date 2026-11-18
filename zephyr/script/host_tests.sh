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

# The shared NINA SPI link (common/drivers/nina/nina_spi.c) on this build's boundary
# (drivers/nina/nano_nina.c: the devicetree's lines and SPI calls; nina_os.c: the kernel's, and
# the READY edge interrupt), on fakes of the Zephyr calls: framing, reset, READY waits.  With
# NINA_OS_WAIT_READY as CMakeLists.txt builds it, and without (sampling).
nina_boundary=(-std=gnu17 -Wall -Wextra -Werror -I"$app/tests/nina_fakes" -I../common/drivers/nina
               "$app/tests/nina_boundary.c" "$app/drivers/nina/nano_nina.c"
               "$app/drivers/nina/nina_os.c")
run nina_boundary_irq -DNINA_OS_WAIT_READY "${nina_boundary[@]}"
run nina_boundary_poll "${nina_boundary[@]}"
# The socket offload (drivers/nina/nina_socket.c) on the shared core (nina_sock.c, nina_dns.c with
# NINA_OS_DNS_OFFLOAD, as CMakeLists.txt builds them), on fakes of zvfs, the kernel and the
# module: descriptors, the wait loops and their sleeps, options, accept, sendmsg, datagrams,
# poll, the resolver's registration.
run nina_socket -std=gnu17 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DNINA_OS_DNS_OFFLOAD \
    -DCONFIG_NANO_NINA_TCP_MAX_IO=3584 -I"$app/tests/socket_fakes" -I../common/drivers/nina \
    "$app/tests/nina_socket.c" "$app/drivers/nina/nina_socket.c" ../common/drivers/nina/nina_sock.c \
    ../common/drivers/nina/nina_dns.c

for f in ../host/imu_host/*.py; do
    python3 -c 'import sys; compile(open(sys.argv[1]).read(), sys.argv[1], "exec")' "$f"
done
printf 'ok  host/imu_host (syntax)\n'
