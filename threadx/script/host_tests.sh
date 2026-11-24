#!/usr/bin/env bash
# Build and run the ThreadX build's host tests:
#
# - The tests of the sources shared with the other builds
#   (common/tests/run.sh).
# - The ThreadX part of the Zephyr compatibility layer's kernel
#   (common/compat/port/threadx/zephyr_kernel_os_impl.h, the primitives of the
#   generic kernel part, common/compat/kernel_generic) compiles alone: it
#   reads ThreadX's headers and its port header, and none of the layer's.
# - The test of the layer's kernel part (the generic one, on the ThreadX
#   primitives; common/tests/zephyr_kernel/zephyr_kernel_test.c on its
#   kernel_generic/zephyr_kernel_test_os.h and
#   threadx/tests/compat/zephyr_kernel_test_os_impl.h) on ThreadX's Linux
#   port, which is in ThreadX's tree (ports/linux/gnu; threadx/tests/compat,
#   with its tx_user.h).
# - On the same port, the shared drivers' ThreadX bindings under the FreeRTOS
#   build's tests of their bindings, which are the same for both kernels:
#   I2C0's and I2C1's (common/drivers/i2c_dw/port/threadx,
#   freertos/tests/i2c_dw/main.c on threadx/tests/i2c_dw) and the NINA's
#   (common/drivers/nina/port/threadx, freertos/tests/nina/main.c on
#   threadx/tests/nina), their interrupt handlers on a Linux thread of the
#   tests (threadx/tests/i2c_dw/tx_test_isr.c).
#
# Needs threadx/threadx (threadx/script/setup.sh) and the host's cc with
# POSIX threads.  About 25 s with common/tests/run.sh (7 s) and the build of
# ThreadX.
#
#   threadx/script/host_tests.sh        run them
#   threadx/script/host_tests.sh -v     also show the tests' output
set -euo pipefail
cd "$(dirname "$0")/.."

verbose=0
case "${1:-}" in
  "") ;;
  -v) verbose=1 ;;
  -h|--help) sed -n '2,29p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
esac

../common/tests/run.sh

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
# The test's port header and tx_user.h first, then the layer (ThreadX's part with the board's
# priorities_os.h, common/compat/port/threadx, the parts for an OS without its own
# <zephyr/sys/clock.h> and <zephyr/sys/__assert.h> with the generic kernel part's zephyr_os.h
# and default zephyr_os_atomic.h, common/compat/port/generic, then the layer's headers,
# common/compat/include, port and internal) and ThreadX.
kgen=../common/compat/kernel_generic
inc=(-Itests/compat -I../common/compat/port/threadx -I../common/compat/port/generic
     -I../common/compat/include -I../common/compat/port -I../common/compat/internal
     -I"$tx/common/inc" -I"$linux/inc")
flags=(-std=gnu11 -O2 -g -pthread "${txdefs[@]}")
strict=(-Wall -Wextra -Werror)

# The ThreadX part reads no header of the layer: it compiles with ThreadX's and its own
# include paths only.
printf '#include "zephyr_kernel_os_impl.h"\n' >"$out/impl_alone.c"
cc "${flags[@]}" "${strict[@]}" -Itests/compat -I../common/compat/port/threadx \
    -I"$tx/common/inc" -I"$linux/inc" -c "$out/impl_alone.c" -o "$out/impl_alone.o"
if grep -n '#include <zephyr/' ../common/compat/port/threadx/*.h ../common/compat/port/threadx/*.c; then
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
for src in $kgen/zephyr_kernel.c $kgen/zephyr_mutex.c ../common/compat/port/threadx/zephyr_kernel_os.c \
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

# The layer's objects for the tests below (not the kernel test's own).
kernobjs=()
for o in "${objs[@]}"; do
    [[ $(basename "$o") == zephyr_kernel_test.o || $(basename "$o") == main.o ]] || kernobjs+=("$o")
done

# I2C0 and I2C1 (threadx/tests/i2c_dw): the shared core's ThreadX binding
# (common/drivers/i2c_dw/port/threadx/i2c_dw_os_impl.h) under the FreeRTOS build's test of the
# binding, freertos/tests/i2c_dw/main.c, which is the same for both kernels (its fakes of the
# Pico SDK and of the controllers' and the DMA's registers, the devices i2c0 and i2c1 of
# port/pico_sdk/i2c_dw_instance.h, the DMA reads' channels, the "i2c" command), on ThreadX's part
# of the test (tests/i2c_dw/i2c_dw_test_os.c: the threads of the layer, the interrupt handlers of
# tests/i2c_dw/tx_test_isr.c, the binding's own checks), with the same options and warnings as
# freertos/script/host_tests.sh builds it.  Twice: i2c0 only, and with CONFIG_I2C_DW_I2C1.
i2cdir=../common/drivers/i2c_dw
frtest=../freertos/tests/i2c_dw
seng=../common/compat/shell_engine
i2cinc=(-I"$frtest/pico_fake" -I"$i2cdir" -I"$i2cdir/include" -I"$i2cdir/internal" -I"$i2cdir/port"
        -I"$i2cdir/port/threadx" -I"$i2cdir/port/pico_sdk")
for variant in i2c0 i2c1; do
    i2cdefs=(-DCONFIG_I2C_DW_I2C0=1)
    [[ $variant == i2c1 ]] && i2cdefs+=(-DCONFIG_I2C_DW_I2C1=1)
    i2cobjs=()
    for src in "$i2cdir/port/pico_sdk/i2c_dw_dma_os_rp2040.c" "$frtest/main.c" \
               tests/i2c_dw/i2c_dw_test_os.c tests/i2c_dw/tx_test_isr.c "$frtest/i2c_dw_core.c" \
               ../common/shell/i2c_shell.c ../common/shell/zshell.c ../common/shell/subcmd.c \
               ../common/compat/zephyr_device.c \
               ../common/compat/subsys/portability/posix/c_lib_ext/fnmatch.c; do
        obj="$out/i2c_${variant}_$(basename "$src" .c).o"
        extra=()
        case $src in
            */i2c_dw_core.c) extra=(-include ../common/compat/port/generic/common_prelude.h -Wno-sign-compare) ;;
            */i2c_shell.c) extra=(-include ../common/compat/port/generic/common_prelude.h -Wno-sign-compare
                                  -Wno-unused-parameter -DCONFIG_I2C_SHELL_BUFFER_SIZE=64
                                  -DCONFIG_LOG_DEFAULT_LEVEL=3) ;;
            */zshell.c) extra=(-Dcalloc=test_calloc -Dfree=test_free) ;;
        esac
        cc "${flags[@]}" "${strict[@]}" "${i2cdefs[@]}" -I"$frtest" -I../common/compat/port/generic -I../common/shell \
            -I../common/compat/include -DZ_SHELL_CMD_ROOT_GLOBAL "${i2cinc[@]}" "${inc[@]}" "${extra[@]}" \
            -c "$src" -o "$obj"
        i2cobjs+=("$obj")
    done
    # The test keeps each write of CHAN_ABORT through the stop of the DMA part (main.c), and
    # gives a semaphore right before a wait's call into ThreadX (i2c_dw_test_os.c).
    cc "${flags[@]}" -Wl,--wrap=i2c_dw_dma_os_stop -Wl,--wrap=_txe_semaphore_get \
        "${kernobjs[@]}" "${i2cobjs[@]}" "$out/libtx.a" -o "$out/i2c_dw_$variant"
    if (( verbose )); then
        timeout 120 "$out/i2c_dw_$variant"
    else
        timeout 120 "$out/i2c_dw_$variant" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
    fi
done
printf 'ok  i2c_dw (I2C0 and I2C1 boundaries and devices, DMA reads, Zephyr I2C API, "i2c" command, ThreadX Linux port)\n'

# The NINA link (threadx/tests/nina): the core's ThreadX binding (common/drivers/nina/port/threadx:
# nina_os.c with its waits on READY's and the IRQ line's edge interrupts, and
# nina_bus_dma_os_impl.h, the wait of the long transfers by DMA of
# port/pico_sdk/nina_bus_dma_rp2040.c) with nina_bus_rp2040.c, under the shared core
# (nina_proto.c and nina_wait.c, with the prelude), under the FreeRTOS build's test of the
# binding, freertos/tests/nina/main.c, which is the same for both kernels (its fakes of the Pico
# SDK, the module, the DMA's engine), on ThreadX's part of the test (tests/nina/nina_test_os.c:
# ThreadX threads, the interrupt handlers of tests/i2c_dw/tx_test_isr.c, the binding's own
# checks), on ThreadX alone (no compatibility layer).  At ThreadX's 1 ms tick (the binding
# accepts no other); the tick count starts 5 s before it wraps (tests/nina/nina_test_os.c) and
# the fake TIMER 4.5 s before its low word wraps.
ninadir=../common/drivers/nina
frnina=../freertos/tests/nina
ninainc=(-I"$frnina/pico_fake" -I"$frnina/log_fake" -I"$frnina" -I"$ninadir/include"
         -I"$ninadir/internal" -I"$ninadir/port" -I"$ninadir/port/threadx" -I"$ninadir/port/pico_sdk"
         -Itests/i2c_dw "${txinc[@]}")
ninadefs=("-DTEST_TIMER_START_US=((UINT64_C(1) << 32) - 4500000)")
ninaobjs=()
for src in "$ninadir/port/pico_sdk/nina_bus_rp2040.c" "$ninadir/port/pico_sdk/nina_bus_dma_rp2040.c" \
           "$ninadir/port/threadx/nina_os.c" "$frnina/main.c" tests/nina/nina_test_os.c \
           tests/i2c_dw/tx_test_isr.c "$ninadir/internal/nina_proto.c" "$ninadir/internal/nina_wait.c"; do
    obj="$out/nina_$(basename "$src" .c).o"
    extra=()
    case $src in
        "$ninadir"/internal/nina_*.c) extra=(-include ../common/compat/port/generic/common_prelude.h) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${ninainc[@]}" "${ninadefs[@]}" "${extra[@]}" \
        -c "$src" -o "$obj"
    ninaobjs+=("$obj")
done
# The windows before the waits' blocks (tests/nina/nina_test_os.c).
cc "${flags[@]}" -Wl,--wrap=_txe_semaphore_get "${ninaobjs[@]}" "$out/libtx.a" -o "$out/nina"
if (( verbose )); then
    timeout 120 "$out/nina"
else
    timeout 120 "$out/nina" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  nina (SPI and line boundary with its DMA, ThreadX binding: READY and event interrupts, locks, sleeps, clock; 1000 Hz tick, ThreadX Linux port)\n'
