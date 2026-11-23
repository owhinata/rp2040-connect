#!/usr/bin/env bash
# Build and run the ThreadX build's host tests:
#
# - The ThreadX part of the Zephyr compatibility layer's kernel
#   (threadx/drivers/compat/zephyr_kernel_os_impl.h, the primitives of the
#   generic kernel part, common/compat/kernel_generic) compiles alone: it
#   reads ThreadX's headers and its port header, and none of the layer's.
# - The test of the layer's kernel part (the generic one, on the ThreadX
#   primitives; common/tests/zephyr_kernel/zephyr_kernel_test.c on its
#   kernel_generic/zephyr_kernel_test_os.h and
#   threadx/tests/compat/zephyr_kernel_test_os_impl.h) on ThreadX's Linux
#   port, which is in ThreadX's tree (ports/linux/gnu; threadx/tests/compat,
#   with its tx_user.h).  Needs threadx/threadx (threadx/script/setup.sh)
#   and the host's cc with POSIX threads.  About 2 s with the build of ThreadX.
#
#   threadx/script/host_tests.sh        run them
#   threadx/script/host_tests.sh -v     also show the tests' output
set -euo pipefail
cd "$(dirname "$0")/.."

verbose=0
case "${1:-}" in
  "") ;;
  -v) verbose=1 ;;
  -h|--help) sed -n '2,16p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
esac

tx=threadx
linux=$tx/ports/linux/gnu
[[ -f $linux/src/tx_thread_schedule.c ]] || {
    printf 'host_tests.sh: %s is missing; run threadx/script/setup.sh\n' "$linux" >&2; exit 1; }

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

# ThreadX's options: the test's tx_user.h; the Linux port keeps the process on one CPU
# (TX_LINUX_MULTI_CORE, with _GNU_SOURCE), as its example build does.
txdefs=(-DTX_INCLUDE_USER_DEFINE_FILE -D_GNU_SOURCE -DTX_LINUX_MULTI_CORE)
txinc=(-Itests/compat -I"$tx/common/inc" -I"$linux/inc")
# The test's port header and tx_user.h first, then the board's priorities_os.h
# (app/usb-console/src), the layer (ThreadX's part, the generic kernel part
# common/compat/kernel_generic, common/compat) and ThreadX.
kgen=../common/compat/kernel_generic
inc=(-Itests/compat -Iapp/usb-console/src -Idrivers/compat -I"$kgen" -I../common/compat
     -I"$tx/common/inc" -I"$linux/inc")
flags=(-std=gnu11 -O2 -g -pthread "${txdefs[@]}")
strict=(-Wall -Wextra -Werror)

# The ThreadX part reads no header of the layer: it compiles with ThreadX's and its own
# include paths only.
printf '#include "zephyr_kernel_os_impl.h"\n' >"$out/impl_alone.c"
cc "${flags[@]}" "${strict[@]}" -Itests/compat -Iapp/usb-console/src -Idrivers/compat \
    -I"$tx/common/inc" -I"$linux/inc" -c "$out/impl_alone.c" -o "$out/impl_alone.o"
if grep -n '#include <zephyr/' drivers/compat/*.h drivers/compat/*.c; then
    printf 'host_tests.sh: the ThreadX part of the layer reads a header of the layer\n' >&2
    exit 1
fi
printf 'ok  zephyr_kernel_os_impl (ThreadX part alone)\n'

# The kernel and its Linux port, as they are (no warnings checked), in an archive: the
# link takes the services used.
mapfile -t txsrcs < <(ls "$tx"/common/src/*.c "$linux"/src/*.c)
printf '%s\n' "${txsrcs[@]}" | xargs -P "$(nproc)" -I{} sh -c \
    'cc "$@" -c "$0" -o "'"$out"'/tx_$(basename "$0" .c).o"' {} "${flags[@]}" -w "${txinc[@]}"
ar rcs "$out/libtx.a" "$out"/tx_*.o

objs=()
for src in ../common/compat/zephyr_init.c ../common/compat/zephyr_work.c \
           ../common/compat/zephyr_rtio_workq.c; do
    obj="$out/common_$(basename "$src" .c).o"
    cc "${flags[@]}" "${strict[@]}" "${inc[@]}" -c "$src" -o "$obj"
    objs+=("$obj")
done
for src in $kgen/zephyr_kernel.c $kgen/zephyr_mutex.c drivers/compat/zephyr_kernel_os.c \
           ../common/tests/zephyr_kernel/zephyr_kernel_test.c tests/compat/main.c; do
    obj="$out/$(basename "$src" .c).o"
    cc "${flags[@]}" "${strict[@]}" "${inc[@]}" -I../common/tests/zephyr_kernel \
        -I../common/tests/zephyr_kernel/kernel_generic -c "$src" -o "$obj"
    objs+=("$obj")
done

cc "${flags[@]}" "${objs[@]}" "$out/libtx.a" -o "$out/zephyr_kernel_test"
if (( verbose )); then
    timeout 120 "$out/zephyr_kernel_test"
else
    timeout 120 "$out/zephyr_kernel_test" >"$out/log" || { cat "$out/log"; exit 1; }
fi
printf 'ok  zephyr_kernel_test (the layer kernel, ThreadX Linux port)\n'
