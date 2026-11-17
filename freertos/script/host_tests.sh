#!/usr/bin/env bash
# Build and run the host tests of the FreeRTOS build.  They need no board.
#
# - The tests of the sources shared with the Zephyr and NuttX builds
#   (common/tests/run.sh).
# - The self-test of the FreeRTOS part of the Zephyr compatibility layer
#   (freertos/drivers/compat/zephyr_selftest.c, the board's "selftest"
#   command) on the FreeRTOS POSIX port, which is in the kernel's tree
#   (freertos/tests/compat).  Needs freertos/FreeRTOS-Kernel
#   (freertos/script/setup.sh) and the host's cc with POSIX threads.
#
#   freertos/script/host_tests.sh        run them
#   freertos/script/host_tests.sh -v     also show the self-test's output
set -euo pipefail
cd "$(dirname "$0")/.."

verbose=0
case "${1:-}" in
  "") ;;
  -v) verbose=1 ;;
  -h|--help) sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
esac

../common/tests/run.sh

kernel=FreeRTOS-Kernel
posix=$kernel/portable/ThirdParty/GCC/Posix
[[ -f $posix/port.c ]] || { printf 'host_tests.sh: %s is missing; run freertos/script/setup.sh\n' "$posix" >&2; exit 1; }

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

# The test's FreeRTOSConfig.h and port header first, then the board's
# priorities_os.h (app/usb-console/src), the layer and the kernel.
inc=(-Itests/compat -Iapp/usb-console/src -Idrivers/compat -Idrivers/include
     -I../common/compat -I../common/log -I"$kernel/include" -I"$posix" -I"$posix/utils")
flags=(-std=gnu11 -O2 -g -pthread)
strict=(-Wall -Wextra -Werror)

# The kernel and its POSIX port, as they are (no warnings checked).
objs=()
for src in "$kernel/tasks.c" "$kernel/list.c" "$kernel/queue.c" "$kernel/portable/MemMang/heap_3.c" \
           "$posix/port.c" "$posix/utils/wait_for_event.c"; do
    obj="$out/kernel_$(basename "$src" .c).o"
    cc "${flags[@]}" -w "${inc[@]}" -c "$src" -o "$obj"
    objs+=("$obj")
done

# The shared sources with their prelude, as the board's build compiles them.
for src in ../common/compat/zephyr_work.c ../common/compat/zephyr_rtio_workq.c; do
    obj="$out/common_$(basename "$src" .c).o"
    cc "${flags[@]}" "${strict[@]}" "${inc[@]}" -include drivers/compat/common_prelude.h \
        -c "$src" -o "$obj"
    objs+=("$obj")
done

for src in drivers/compat/zephyr_kernel.c drivers/compat/zephyr_mutex.c \
           drivers/compat/zephyr_selftest.c tests/compat/main.c; do
    obj="$out/$(basename "$src" .c).o"
    cc "${flags[@]}" "${strict[@]}" "${inc[@]}" -c "$src" -o "$obj"
    objs+=("$obj")
done
cc "${flags[@]}" "${strict[@]}" "${inc[@]}" -DvApplicationTickHook=app_tick_hook \
    -c app/usb-console/src/tick.c -o "$out/tick.o"
objs+=("$out/tick.o")

cc "${flags[@]}" "${objs[@]}" -o "$out/compat_selftest"
if (( verbose )); then
    timeout 120 "$out/compat_selftest"
else
    timeout 120 "$out/compat_selftest" >"$out/log" || { cat "$out/log"; exit 1; }
fi
printf 'ok  compat_selftest (FreeRTOS POSIX port)\n'

# The shell (freertos/tests/shell): the line editor, and two sessions at once on two consoles
# (each task's stdout, the shared zshell.c and subcmd.c, the copies of Zephyr's fnmatch() and
# getopt), on the same kernel and layer objects.
shinc=(-Itests/shell -Idrivers/shell -I../common/shell -I../common/compat/lib/utils/getopt
       -DZ_SHELL_CMD_ROOT_GLOBAL -DCONFIG_GETOPT_LONG=1 -U_FORTIFY_SOURCE -ffunction-sections
       -fdata-sections)
shobjs=()
for src in drivers/shell/cli.c drivers/shell/getopt_state.c drivers/shell/nano_console.c \
           drivers/shell/nano_shell.c drivers/shell/shell_argv.c drivers/libc/stdio_wrap.c \
           tests/shell/main.c ../common/shell/zshell.c ../common/shell/subcmd.c \
           ../common/compat/subsys/portability/posix/c_lib_ext/fnmatch.c \
           ../common/compat/lib/utils/getopt/getopt.c ../common/compat/lib/utils/getopt/getopt_long.c; do
    obj="$out/shell_$(basename "$src" .c).o"
    alloc=()
    case $src in
        *zshell.c|*nano_console.c) alloc=(-Dcalloc=test_calloc -Dmalloc=test_malloc -Dfree=test_free) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${shinc[@]}" "${inc[@]}" "${alloc[@]}" -c "$src" -o "$obj"
    shobjs+=("$obj")
done
wraps=()
for f in printf vprintf puts putchar fputs fwrite fputc putc fprintf vfprintf fflush; do
    wraps+=("-Wl,--wrap=$f")
done
kernobjs=()
for o in "${objs[@]}"; do
    [[ $(basename "$o") == zephyr_selftest.o || $(basename "$o") == main.o ]] || kernobjs+=("$o")
done
cc "${flags[@]}" -Wl,--gc-sections "${wraps[@]}" "${kernobjs[@]}" "${shobjs[@]}" -o "$out/shell"
if (( verbose )); then
    timeout 60 "$out/shell"
else
    timeout 60 "$out/shell" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  shell (two sessions, FreeRTOS POSIX port)\n'
