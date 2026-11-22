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

# The shared NINA SPI link (common/drivers/nina/internal/nina_proto.c) on this build's boundary
# (common/drivers/nina/port/zephyr/nina_bus.c: the devicetree's lines and SPI calls; its
# nina_os.c: the kernel's, the Wi-Fi operations' lock and requests, and the READY and IRQ edge
# interrupts; the socket loops' waits, internal/nina_wait.c), on fakes of the Zephyr
# calls: framing, reset, READY waits (sampled when the READY callback cannot be added, else on the
# edge interrupt), the socket events' waits and their start, failed and successful.  With
# CONFIG_NINA_PASSTHROUGH as well (the build's default), the NINA passthrough mode's gate:
# no bus, no line touched while the mode is active (common/drivers/nina/include/nina_passthrough.h).
nina_boundary=(-std=gnu17 -Wall -Wextra -Werror -I"$app/tests/nina_fakes" -I../common/drivers/nina/include
               -I../common/drivers/nina/internal -I../common/drivers/nina/port
               "$app/tests/nina_boundary.c" ../common/drivers/nina/port/zephyr/nina_bus.c
               ../common/drivers/nina/port/zephyr/nina_os.c ../common/drivers/nina/internal/nina_wait.c)
run nina_boundary_passthrough -DCONFIG_NINA_PASSTHROUGH "${nina_boundary[@]}"
run nina_boundary "${nina_boundary[@]}"
# The same in the bus's configuration of prj.conf: the PL022 driver's DMA mode with its
# threshold (CONFIG_SPI_PL022_DMA_MIN_LEN of the local patch 0007).  nina_bus.c does not build in
# that mode without a threshold, where nina_bus_select()'s transfer of no bytes would start a DMA
# instead of only taking the bus lock.
run nina_boundary_spi_dma -DCONFIG_SPI_PL022_DMA -DCONFIG_SPI_PL022_DMA_MIN_LEN=64 \
    "${nina_boundary[@]}"
if cc -std=gnu17 -fsyntax-only -I"$app/tests/nina_fakes" -I../common/drivers/nina/include \
      -I../common/drivers/nina/internal -I../common/drivers/nina/port \
      -DCONFIG_SPI_PL022_DMA -DCONFIG_SPI_PL022_DMA_MIN_LEN=0 \
      ../common/drivers/nina/port/zephyr/nina_bus.c 2>/dev/null; then
    printf 'nina_bus.c builds in the PL022 DMA mode without a threshold\n' >&2
    exit 1
fi
printf 'ok  %s\n' nina_bus_spi_dma_threshold
# The NINA passthrough mode's Zephyr binding (common/drivers/nina/port/zephyr/nina_passthrough_os.c)
# and "nina passthrough" (common/shell/nina_shell.c), which the test includes, with the shared
# core (common/drivers/nina/nina_passthrough.c) on fakes of the Zephyr calls and of the module's
# side (nina_passthrough_bus.h): the request at the start, the lines and the rate on the USB
# stack's messages and the tick, both data paths with their interrupts and handovers, the way
# out, the command.  The core itself is common/tests/nina_passthrough.c's.
run passthrough -std=gnu17 -Wall -Wextra -Werror -I"$app/tests/passthrough_fakes" \
    -I../common/drivers/nina/include -I../common/drivers/nina/internal -I../common/drivers/nina/port \
    "$app/tests/passthrough.c" ../common/drivers/nina/nina_passthrough.c
# The socket offload (common/drivers/nina/port/zephyr/nina_socket.c) on the shared core (nina_sock.c,
# nina_dns.c with NINA_OS_DNS_OFFLOAD, as CMakeLists.txt builds them), on fakes of zvfs, the kernel and the
# module: descriptors, the wait loops and their sleeps, options, accept, sendmsg, datagrams,
# poll, the resolver's registration.
# The waits between the passes are nina_wait.c's: the poll intervals after a start of the events
# that failed, then the IRQ line's edges once they are started (fakes in the test).
nina_socket=(-std=gnu17 -Wall -Wextra -Werror -D_DEFAULT_SOURCE -DNINA_OS_DNS_OFFLOAD
             -DCONFIG_NINA_TCP_MAX_IO=3584 -I"$app/tests/socket_fakes" -I../common/drivers/nina/include
             -I../common/drivers/nina/internal -I../common/drivers/nina/port
             "$app/tests/nina_socket.c" ../common/drivers/nina/port/zephyr/nina_socket.c
             ../common/drivers/nina/nina_sock.c ../common/drivers/nina/nina_dns.c
             ../common/drivers/nina/internal/nina_wait.c)
run nina_socket "${nina_socket[@]}"
# The core's Zephyr binding (common/drivers/nina/port/zephyr/nina_net.c, which the test includes) on
# fakes of the Zephyr declarations it uses and of the core's Wi-Fi functions: the device and its
# interface's init, Zephyr's scan, connect and status parameters and results, the core's
# operations on the interface and as Zephyr's events, the Wi-Fi profile's hooks, the threads.
# The operations themselves are common/tests/nina_wifi.c's; the binding on the FreeRTOS build's
# compatibility layer with the core, end to end, is freertos/tests/wifi.
run nina_net -std=gnu17 -Wall -Wextra -Werror -I"$app/tests/net_fakes" -I../common/drivers/nina/include \
    -I../common/drivers/nina/internal -I../common/drivers/nina/port -I../common/imu "$app/tests/nina_net.c"

# The local patches (manifest/zephyr/patches.yml, applied by script/setup.sh): every patch file
# is listed once, and the sha256sum of each entry is that of its file ("west patch" refuses the
# patch otherwise).
patch_yml=manifest/zephyr/patches.yml
patch_dir=manifest/zephyr/patches
mapfile -t patch_paths < <(sed -n 's/^  - path: //p' "$patch_yml")
mapfile -t patch_sums < <(sed -n 's/^    sha256sum: //p' "$patch_yml")
if (( ${#patch_paths[@]} != ${#patch_sums[@]} )); then
    printf 'FAIL %s: %d paths, %d sha256sums\n' "$patch_yml" "${#patch_paths[@]}" "${#patch_sums[@]}" >&2
    exit 1
fi
for i in "${!patch_paths[@]}"; do
    sum=$(sha256sum < "$patch_dir/${patch_paths[i]}" | cut -d' ' -f1)
    if [[ "$sum" != "${patch_sums[i]}" ]]; then
        printf 'FAIL %s: sha256sum of %s is %s\n' "$patch_yml" "${patch_paths[i]}" "$sum" >&2
        exit 1
    fi
done
listed=$(printf '%s\n' "${patch_paths[@]}" | sort)
present=$(cd "$patch_dir" && find . -name '*.patch' -printf '%P\n' | sort)
if [[ "$listed" != "$present" ]]; then
    printf 'FAIL %s does not list exactly the files of %s\n' "$patch_yml" "$patch_dir" >&2
    exit 1
fi
printf 'ok  local patches (%d in %s)\n' "${#patch_paths[@]}" "$patch_yml"

for f in ../host/imu_host/*.py; do
    python3 -c 'import sys; compile(open(sys.argv[1]).read(), sys.argv[1], "exec")' "$f"
done
printf 'ok  host/imu_host (syntax)\n'
