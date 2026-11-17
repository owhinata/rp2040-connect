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
# - On the same port: the shell (freertos/tests/shell) and the key-value
#   store with the board's flash boundary on fakes of the Pico SDK's flash
#   functions (freertos/tests/kv), and the NINA link's boundaries
#   (freertos/drivers/nina) with the shared core on fakes of the Pico SDK's
#   GPIO and SPI functions (freertos/tests/nina), and I2C0's boundaries
#   (freertos/drivers/i2c_dw, the layer's <zephyr/drivers/i2c.h>) with the
#   shared core on fakes of the Pico SDK's GPIO, reset, IRQ and sync
#   functions and of I2C0's registers (freertos/tests/i2c_dw), and GPIO bank
#   0's boundary (freertos/drivers/compat/rp2040/nano_gpio.c) under the
#   layer's gpio0 and the LSM6DSO's FIFO interrupt on fakes of the Pico SDK's
#   GPIO and IRQ functions (freertos/tests/gpio), and the IMU service's
#   parts of the build (address conversions, hwinfo, the stubs of the MQTT
#   transport and the time synchronization: freertos/tests/imu), and the
#   sockets: the zsock_*() calls (freertos/drivers/nina/nina_socket.c) on the
#   shared NINA core, a fake module and a fake clock (freertos/tests/socket).
#
#   freertos/script/host_tests.sh        run them
#   freertos/script/host_tests.sh -v     also show the tests' output
set -euo pipefail
cd "$(dirname "$0")/.."

verbose=0
case "${1:-}" in
  "") ;;
  -v) verbose=1 ;;
  -h|--help) sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
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

# The key-value store (freertos/tests/kv): the board's flash boundary (drivers/kv/nano_flash_kv.c)
# on fakes of the Pico SDK's flash functions (tests/kv/pico_fake), under the shared store as the
# board builds it (common/imu/kv_store.c, kv_store_wipe.c, zephyr_flash_kv.c, the copies of
# Zephyr's settings, NVS and hex.c with their options, drivers/kv/kv.cmake), on the same kernel
# and layer objects.  Unused parts (the "kv" command's table) are left out by --gc-sections.
kvinc=(-D_GNU_SOURCE -DPICO_FLASH_ASSUME_CORE1_SAFE=1 -DZ_SHELL_CMD_ROOT_GLOBAL -Itests/kv/pico_fake
       -Idrivers/kv -Idrivers/shell -I../common/imu -I../common/compat/subsys/settings/include
       -I../common/compat/subsys/settings/src -ffunction-sections -fdata-sections)
kvconfig=(-include ../common/compat/zephyr_kv_config.h -Wno-undef)
kvobjs=()
for src in drivers/kv/nano_flash_kv.c tests/kv/main.c ../common/imu/kv_store.c \
           ../common/compat/kv_store_wipe.c ../common/compat/zephyr_flash_kv.c \
           ../common/compat/lib/utils/hex.c ../common/compat/subsys/settings/src/settings.c \
           ../common/compat/subsys/settings/src/settings_store.c \
           ../common/compat/subsys/settings/src/settings_init.c \
           ../common/compat/subsys/settings/src/settings_nvs.c \
           ../common/compat/subsys/crc/crc8_sw.c ../common/compat/subsys/kvss/nvs/nvs.c; do
    obj="$out/kv_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    case $src in
        */subsys/*) extra+=("${kvconfig[@]}") ;;
    esac
    case $src in
        */nvs.c) extra+=(-Wno-format -Wno-sign-compare -Wno-unused-parameter) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${kvinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    kvobjs+=("$obj")
done
cc "${flags[@]}" -Wl,--gc-sections "${kernobjs[@]}" "${kvobjs[@]}" -o "$out/kv"
if (( verbose )); then
    timeout 120 "$out/kv"
else
    timeout 120 "$out/kv" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  kv (flash boundary and store, FreeRTOS POSIX port)\n'

# The NINA link (freertos/tests/nina): the board's boundaries of the shared core
# (drivers/nina/nano_nina.c on fakes of the Pico SDK's GPIO, reset and SPI functions,
# tests/nina/pico_fake; drivers/nina/nina_os.c on the layer) under the shared core
# (common/drivers/nina/nina_spi.c, with the prelude, as drivers/nina/nina.cmake builds it), on
# the same kernel and layer objects.
ninainc=(-Itests/nina/pico_fake -I../common/drivers/nina)
ninaobjs=()
for src in drivers/nina/nano_nina.c drivers/nina/nina_os.c tests/nina/main.c \
           ../common/drivers/nina/nina_spi.c; do
    obj="$out/nina_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${ninainc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    ninaobjs+=("$obj")
done
cc "${flags[@]}" "${kernobjs[@]}" "${ninaobjs[@]}" -o "$out/nina"
if (( verbose )); then
    timeout 60 "$out/nina"
else
    timeout 60 "$out/nina" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  nina (SPI and line boundary, kernel services, FreeRTOS POSIX port)\n'

# I2C0 (freertos/tests/i2c_dw): the board's boundary of the shared core (drivers/i2c_dw:
# i2c_dw_os_impl.h, nano_i2c_instance.h with the device i2c0 and its Zephyr I2C API,
# nano_dma_instance.h, nano_dma_os.c; drivers/compat/zephyr/drivers/i2c.h) on fakes of the Pico
# SDK's GPIO, reset, IRQ, sync and DMA-claim functions and of I2C0's registers
# (tests/i2c_dw/pico_fake), under the shared core (common/drivers/i2c_dw/nano_i2c.c, nano_dma.c,
# i2c_dw_rp2040_commands.c, with the prelude, as drivers/i2c_dw/i2c_dw.cmake builds them), on the
# same kernel and layer objects.  The DMA's register addresses are 32-bit constants
# (common/drivers/i2c_dw/nano_dma.h), which a 64-bit host warns about turning into pointers; the
# test does not use them.  The host's cc reports -Wsign-compare for two MIN()s of nano_i2c.c (an
# int count and a uint32_t one, as in Zephyr's i2c_dw.c; the board's arm-none-eabi-gcc 15.2 with
# the same -Wextra does not).
i2cinc=(-Itests/i2c_dw/pico_fake -Idrivers/i2c_dw -I../common/drivers/i2c_dw -Wno-int-to-pointer-cast)
i2cobjs=()
for src in drivers/i2c_dw/nano_dma_os.c tests/i2c_dw/main.c ../common/drivers/i2c_dw/nano_i2c.c \
           ../common/drivers/i2c_dw/nano_dma.c ../common/drivers/i2c_dw/i2c_dw_rp2040_commands.c; do
    obj="$out/i2c_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    case $src in
        */nano_i2c.c) extra+=(-Wno-sign-compare) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${i2cinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    i2cobjs+=("$obj")
done
cc "${flags[@]}" "${kernobjs[@]}" "${i2cobjs[@]}" -o "$out/i2c_dw"
if (( verbose )); then
    timeout 60 "$out/i2c_dw"
else
    timeout 60 "$out/i2c_dw" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  i2c_dw (I2C0 boundary and device, Zephyr I2C API, FreeRTOS POSIX port)\n'

# GPIO bank 0 (freertos/tests/gpio): the board's boundary of the layer's gpio0
# (drivers/compat/rp2040/nano_gpio.c) on fakes of the Pico SDK's GPIO and IRQ functions
# (tests/gpio/pico_fake), under common/compat/zephyr_gpio.c and zephyr_device.c and the LSM6DSO's
# FIFO interrupt (common/drivers/lsm6dso/lsm6dso_fifo_irq.c), with the prelude and
# CONFIG_NANO_LSM6DSO_STREAM as drivers/lsm6dso/lsm6dso.cmake builds them; the layer's kernel part
# again with that option (zephyr_compat_initialize() starts gpio0), on the same kernel objects.
gpioinc=(-Itests/gpio/pico_fake -I../common/drivers/lsm6dso -DCONFIG_NANO_LSM6DSO_STREAM=1)
gpioobjs=()
for src in drivers/compat/rp2040/nano_gpio.c drivers/compat/zephyr_kernel.c tests/gpio/main.c \
           ../common/compat/zephyr_gpio.c ../common/compat/zephyr_device.c \
           ../common/drivers/lsm6dso/lsm6dso_fifo_irq.c; do
    obj="$out/gpio_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${gpioinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    gpioobjs+=("$obj")
done
gpiokern=()
for o in "${kernobjs[@]}"; do
    [[ $(basename "$o") == zephyr_kernel.o ]] || gpiokern+=("$o")
done
cc "${flags[@]}" "${gpiokern[@]}" "${gpioobjs[@]}" -o "$out/gpio"
if (( verbose )); then
    timeout 60 "$out/gpio"
else
    timeout 60 "$out/gpio" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  gpio (GPIO bank 0 boundary, gpio0, FIFO interrupt, FreeRTOS POSIX port)\n'

# The IMU service's FreeRTOS parts (freertos/tests/imu): the address conversions
# (drivers/libc/inet.c, the layer's <zephyr/net/socket.h> on drivers/libc/include),
# hwinfo_get_device_id() (drivers/compat/rp2040/zephyr_hwinfo.c) on a fake of
# pico_get_unique_board_id(), and the stubs of the MQTT transport and the time synchronization
# (drivers/imu, with common/imu/imu_raw_queue.c and imu_raw_packet.c, with the prelude, as
# drivers/imu/imu.cmake builds them), on the same kernel and layer objects.  tests/imu/fake
# (pico/unique_id.h, and os_clock.h on the test's port header) comes first.
imuinc=(-Itests/imu/fake -Idrivers/libc/include -I../common/imu -I../common/drivers/lsm6dso)
imuobjs=()
for src in drivers/libc/inet.c drivers/compat/rp2040/zephyr_hwinfo.c drivers/imu/imu_raw_mqtt_null.c \
           drivers/imu/time_sync_null.c tests/imu/main.c ../common/imu/imu_raw_queue.c \
           ../common/imu/imu_raw_packet.c; do
    obj="$out/imu_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${imuinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    imuobjs+=("$obj")
done
cc "${flags[@]}" "${kernobjs[@]}" "${imuobjs[@]}" -o "$out/imu"
if (( verbose )); then
    timeout 60 "$out/imu"
else
    timeout 60 "$out/imu" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  imu (address conversions, hwinfo, stubs of the transport and time, FreeRTOS POSIX port)\n'

# The sockets (freertos/tests/socket): the zsock_*() calls of the layer's <zephyr/net/socket.h>
# (drivers/nina/nina_socket.c: the descriptors, their locks, the Zephyr build's wait loops) on
# the shared core (common/drivers/nina/nina_sock.c, nina_dns.c, with the prelude, as
# drivers/nina/nina.cmake builds them) and the address conversions (drivers/libc/inet.c), on a
# fake module and a fake clock (the test's nina_request() and nina_os.h), on the same kernel and
# layer objects.  drivers/libc/include (newlib's missing network headers) comes before the host's.
sockinc=(-Idrivers/libc/include -I../common/drivers/nina)
sockobjs=()
for src in drivers/nina/nina_socket.c drivers/libc/inet.c tests/socket/main.c \
           ../common/drivers/nina/nina_sock.c ../common/drivers/nina/nina_dns.c; do
    obj="$out/socket_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${sockinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    sockobjs+=("$obj")
done
cc "${flags[@]}" "${kernobjs[@]}" "${sockobjs[@]}" -o "$out/socket"
if (( verbose )); then
    timeout 60 "$out/socket"
else
    timeout 60 "$out/socket" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  socket (descriptors, wait loops, options, poll, resolver, FreeRTOS POSIX port)\n'
