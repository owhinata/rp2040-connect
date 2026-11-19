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
#   GPIO, SPI and GPIO interrupt functions (freertos/tests/nina), and I2C0's boundaries
#   (freertos/drivers/i2c_dw, the layer's <zephyr/drivers/i2c.h>) with the
#   shared core on fakes of the Pico SDK's GPIO, reset, IRQ and sync
#   functions and of I2C0's registers (freertos/tests/i2c_dw), and GPIO bank
#   0's boundary (freertos/drivers/compat/rp2040/nano_gpio.c) under the
#   layer's gpio0 and the LSM6DSO's FIFO interrupt on fakes of the Pico SDK's
#   GPIO and IRQ functions (freertos/tests/gpio), and the IMU service's
#   parts of the build (address conversions, hwinfo: freertos/tests/imu),
#   and the MQTT transport (common/imu/imu_raw_mqtt.c with the port's MQTT
#   client and its socket layer, the thread, on fake sockets and a fake
#   broker, about 20 s; then against the host's receiver, host/imu_host's
#   "imu-host check" on 127.0.0.1, about 30 s, skipped without
#   host/.venv: freertos/tests/mqtt), and the time synchronization
#   (common/imu/time_sync.c on the build's realtime clock, TIMER read and
#   thread, with fake sockets and a fake NTP server: freertos/tests/time_sync), and the
#   sockets: the zsock_*() calls (freertos/drivers/nina/nina_socket.c) on the
#   shared NINA core, a fake module and a fake clock (freertos/tests/socket), and Wi-Fi: the
#   interface, the net_mgmt events, the stored credentials (settings "wifi_cred/<n>"), the
#   reconnection, the "wifi" and "ifconfig" commands on a fake module (freertos/tests/wifi), and the
#   Telnet shell (freertos/drivers/telnet) and "zperf" (common/compat/zperf_iperf3.c) on the
#   host's sockets with the NINA's behaviour, driven over TCP on 127.0.0.1 by
#   freertos/tests/telnet/client.py (python3; with the host's iperf3, which is skipped when
#   missing) (freertos/tests/telnet, about 60 s), and the "sensor" command
#   (freertos/drivers/sensor_shell with the shared sensor shell, its processing thread and
#   "imu cal") on the shared LSM6DSO driver and a fake sensor, from two sessions
#   (freertos/tests/sensor_shell; skipped without freertos/stmemsc/lsm6dso_reg.c).
# - The kernel tick's catch-up of ticks lost while interrupts are masked
#   (freertos/app/usb-console/src/tick_catchup.h) on a model of SysTick
#   (freertos/tests/tick, no kernel).
# - The NINA passthrough mode's decisions (common/drivers/nina/
#   nina_passthrough_logic.h: the request in the watchdog's scratch
#   registers, DTR and RTS on the module's lines, the rate, the way out, the
#   buffer from the UART's interrupt, the "nina" subcommands;
#   freertos/tests/passthrough, no kernel).
#
#   freertos/script/host_tests.sh        run them
#   freertos/script/host_tests.sh -v     also show the tests' output
set -euo pipefail
cd "$(dirname "$0")/.."

verbose=0
case "${1:-}" in
  "") ;;
  -v) verbose=1 ;;
  -h|--help) sed -n '2,51p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
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

# The kernel tick's catch-up (freertos/tests/tick): the step of the board's tick hook
# (app/usb-console/src/tick_catchup.h) alone and on a model of SysTick, its pending bit and
# masked interrupts.  No kernel.
cc "${flags[@]}" "${strict[@]}" -Iapp/usb-console/src tests/tick/main.c -o "$out/tick"
if (( verbose )); then
    timeout 60 "$out/tick"
else
    timeout 60 "$out/tick" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  tick (catch-up of lost ticks, model of SysTick)\n'

# The NINA passthrough mode's decisions (freertos/tests/passthrough): the request in the
# watchdog's scratch registers against the fatal error record's (common/log/fault_record.c),
# DTR and RTS on the module's lines, the rate and the way out, the buffer from the UART's
# interrupt (also on two threads), and the "nina" subcommands
# (../common/drivers/nina/nina_passthrough_logic.h).  Not linked with the kernel (its headers
# come in through the layer's <zephyr/shell/shell.h>).
cc "${flags[@]}" "${strict[@]}" -I../common/drivers/nina -Idrivers/shell "${inc[@]}" \
    tests/passthrough/main.c ../common/log/fault_record.c -o "$out/passthrough"
if (( verbose )); then
    timeout 60 "$out/passthrough"
else
    timeout 60 "$out/passthrough" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  passthrough (NINA passthrough: request, lines, rate, buffer, subcommands)\n'

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
# (common/drivers/nina/nina_spi.c and nina_wait.c, with the prelude, as drivers/nina/nina.cmake
# builds them), on the same kernel and layer objects.  The waits for READY on its edge interrupt
# (nina_os.c) go through GPIO bank 0's boundary (drivers/compat/rp2040/nano_gpio.c) and the
# layer's gpio0 (common/compat/zephyr_gpio.c), on fakes of the Pico SDK's GPIO interrupt
# functions (tests/nina/pico_fake).  The layer's gpio_add_callback() does not fail, so
# nina_os_ready_start() succeeds here; the sampling after a failed start is
# common/tests/nina_spi.c's.
ninainc=(-Itests/nina/pico_fake -I../common/drivers/nina)
ninaobjs=()
for src in drivers/nina/nano_nina.c drivers/nina/nina_os.c tests/nina/main.c \
           drivers/compat/rp2040/nano_gpio.c ../common/compat/zephyr_gpio.c \
           ../common/drivers/nina/nina_spi.c ../common/drivers/nina/nina_wait.c; do
    obj="$out/nina_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${ninainc[@]}" "${inc[@]}" \
        "${extra[@]}" -c "$src" -o "$obj"
    ninaobjs+=("$obj")
done
cc "${flags[@]}" "${kernobjs[@]}" "${ninaobjs[@]}" -o "$out/nina"
if (( verbose )); then
    timeout 60 "$out/nina"
else
    timeout 60 "$out/nina" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  nina (SPI and line boundary, READY interrupt, kernel services, FreeRTOS POSIX port)\n'

# I2C0 (freertos/tests/i2c_dw): the board's boundary of the shared core (drivers/i2c_dw:
# i2c_dw_os_impl.h, i2c_dw_instance.h with the device i2c0 and its Zephyr I2C API,
# dma_rpi_pico_instance.h, dma_rpi_pico_os.c; drivers/compat/zephyr/drivers/i2c.h) on fakes of
# the Pico SDK's GPIO, reset, IRQ, sync and DMA-claim functions and of I2C0's registers
# (tests/i2c_dw/pico_fake), under the shared core (common/drivers/i2c_dw/i2c_dw.c, through
# tests/i2c_dw/i2c_dw_core.c, which gives the test the core's static command words, and
# dma_rpi_pico.c, with the prelude, as drivers/i2c_dw/i2c_dw.cmake builds them), on the same
# kernel and layer objects.  The DMA's register addresses are 32-bit
# constants (common/drivers/i2c_dw/dma_rpi_pico.h), which a 64-bit host warns about turning into
# pointers; the test does not use them.  The host's cc reports -Wsign-compare for two MIN()s of
# i2c_dw.c (an int count and a uint32_t one, as in Zephyr's i2c_dw.c; the board's
# arm-none-eabi-gcc 15.2 with the same -Wextra does not).
i2cinc=(-Itests/i2c_dw/pico_fake -Idrivers/i2c_dw -I../common/drivers/i2c_dw -Wno-int-to-pointer-cast)
i2cobjs=()
for src in drivers/i2c_dw/dma_rpi_pico_os.c tests/i2c_dw/main.c tests/i2c_dw/i2c_dw_core.c \
           ../common/drivers/i2c_dw/dma_rpi_pico.c; do
    obj="$out/i2c_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*|*/i2c_dw_core.c) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    case $src in
        */i2c_dw_core.c) extra+=(-Wno-sign-compare) ;;
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
# (drivers/libc/inet.c, the layer's <zephyr/net/socket.h> on drivers/libc/include) and
# hwinfo_get_device_id() (drivers/compat/rp2040/zephyr_hwinfo.c) on a fake of
# pico_get_unique_board_id(), on the same kernel and layer objects.  tests/imu/fake
# (pico/unique_id.h) comes first.
imuinc=(-Itests/imu/fake -Idrivers/libc/include)
imuobjs=()
for src in drivers/libc/inet.c drivers/compat/rp2040/zephyr_hwinfo.c tests/imu/main.c; do
    obj="$out/imu_$(basename "$src" .c).o"
    cc "${flags[@]}" "${strict[@]}" "${imuinc[@]}" "${inc[@]}" -c "$src" -o "$obj"
    imuobjs+=("$obj")
done
cc "${flags[@]}" "${kernobjs[@]}" "${imuobjs[@]}" -o "$out/imu"
if (( verbose )); then
    timeout 60 "$out/imu"
else
    timeout 60 "$out/imu" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  imu (address conversions, hwinfo, FreeRTOS POSIX port)\n'

# The MQTT transport (freertos/tests/mqtt): the shared common/imu/imu_raw_mqtt.c with its JSON
# messages (imu_raw_status.c), the port's MQTT client (mqtt_min.c, with -Wno-sign-compare) and
# its socket layer (mqtt_min_sock.c), the queue and the packets (imu_raw_queue.c,
# imu_raw_packet.c), with the prelude, as drivers/imu/imu.cmake builds them, on the build's
# interface (drivers/compat/zephyr_net_if.c), fake zsock_*() calls with the NINA's waits and a
# fake broker (tests/mqtt/main.c), on the same kernel and layer objects: the thread, the
# connection, the messages, the intervals, congestion, failures, CONNACK, the stop.  It waits
# for the real intervals: about 20 s.
mqttsrcs=(drivers/compat/zephyr_net_if.c ../common/imu/imu_raw_mqtt.c ../common/imu/imu_raw_status.c
          ../common/imu/mqtt_min.c ../common/imu/mqtt_min_sock.c ../common/imu/imu_raw_queue.c
          ../common/imu/imu_raw_packet.c)
mqttinc=(-Idrivers/libc/include -I../common/imu -I../common/drivers/lsm6dso)
mqttobjs=()
for src in tests/mqtt/main.c drivers/libc/inet.c "${mqttsrcs[@]}"; do
    obj="$out/mqtt_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    case $src in
        */mqtt_min.c) extra+=(-Wno-sign-compare) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${mqttinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    mqttobjs+=("$obj")
done
cc "${flags[@]}" "${kernobjs[@]}" "${mqttobjs[@]}" -o "$out/mqtt"
if (( verbose )); then
    timeout 90 "$out/mqtt"
else
    timeout 90 "$out/mqtt" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  mqtt (thread, connection, messages, congestion, failures, stop, FreeRTOS POSIX port)\n'

# The same transport against the host's receiver (tests/mqtt/live.c, live.py): the sources
# above on the zsock_*() calls of tests/telnet/host_sock.c (the host's sockets with the NINA's
# limits and waits) and the host's network headers, sending to "imu-host check"
# (host/.venv/bin/imu-host, host/README.md) on 127.0.0.1 and a free port other than 18830: all
# batches, a receiver that stops reading (the board's queue fills and drops, the receiver's
# missing batches are the dropped ones), a receiver that drops the connection (reconnection).
# About 30 s.  Skipped without host/.venv or python3.
imuhost=../host/.venv/bin/imu-host
if [[ -x $imuhost ]] && command -v python3 >/dev/null; then
    liveinc=(-D_GNU_SOURCE -Itests/telnet -I../common/imu -I../common/drivers/lsm6dso)
    liveobjs=()
    for src in tests/mqtt/live.c tests/telnet/host_sock.c "${mqttsrcs[@]}"; do
        obj="$out/live_$(basename "$src" .c).o"
        extra=()
        case $src in
            ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
        esac
        case $src in
            */mqtt_min.c) extra+=(-Wno-sign-compare) ;;
        esac
        cc "${flags[@]}" "${strict[@]}" "${liveinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
        liveobjs+=("$obj")
    done
    cc "${flags[@]}" "${kernobjs[@]}" "${liveobjs[@]}" -o "$out/mqtt_live"
    if (( verbose )); then
        timeout 180 python3 tests/mqtt/live.py "$out/mqtt_live" "$imuhost" -v
    else
        timeout 180 python3 tests/mqtt/live.py "$out/mqtt_live" "$imuhost" >"$out/log" 2>&1 ||
            { cat "$out/log"; exit 1; }
    fi
    printf 'ok  mqtt live (against imu-host check on 127.0.0.1: all batches, stall, drop)\n'
else
    printf 'skip mqtt live (no host/.venv/bin/imu-host or python3)\n'
fi

# The time synchronization (freertos/tests/time_sync): the shared common/imu/time_sync.c and
# time_offset.c with the prelude, as drivers/imu/imu.cmake builds them (without
# drivers/imu/time_sync_prelude.h, which only takes away a name of the Pico SDK's headers), on
# the build's realtime clock (drivers/compat/zephyr_sys_clock.c), interface
# (drivers/compat/zephyr_net_if.c) and net_mgmt events (common/compat/zephyr_net_mgmt.c with the
# "net_mgmt" thread), the build's shell on a console that captures, fake zsock_*() calls with a
# fake NTP server, and the TIMER's registers on the test port's clock
# (tests/time_sync/fake/zephyr/sys/sys_io.h, before common/compat), on the same kernel and layer
# objects.  It waits for the real intervals (2 s): about 12 s.
tsinc=(-DZ_SHELL_CMD_ROOT_GLOBAL -DCONFIG_GETOPT_LONG=1 -U_FORTIFY_SOURCE
       -Itests/time_sync/fake -Itests/shell -Idrivers/shell -Idrivers/libc/include -I../common/shell
       -I../common/compat/lib/utils/getopt -I../common/imu -ffunction-sections -fdata-sections)
tsobjs=()
for src in tests/time_sync/main.c drivers/compat/zephyr_sys_clock.c drivers/compat/zephyr_net_if.c \
           drivers/libc/inet.c drivers/shell/getopt_state.c drivers/shell/nano_console.c \
           drivers/shell/nano_shell.c drivers/shell/shell_argv.c drivers/libc/stdio_wrap.c \
           ../common/shell/zshell.c ../common/shell/subcmd.c \
           ../common/compat/subsys/portability/posix/c_lib_ext/fnmatch.c \
           ../common/compat/lib/utils/getopt/getopt.c ../common/compat/lib/utils/getopt/getopt_long.c \
           ../common/imu/time_sync.c ../common/imu/time_offset.c ../common/compat/zephyr_net_mgmt.c; do
    obj="$out/time_sync_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/shell/*|*/fnmatch.c|*/getopt*.c) ;;
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    case $src in
        *zshell.c|*nano_console.c) extra+=(-Dcalloc=test_calloc -Dmalloc=test_malloc -Dfree=test_free) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${tsinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    tsobjs+=("$obj")
done
cc "${flags[@]}" -Wl,--gc-sections "${wraps[@]}" "${kernobjs[@]}" "${tsobjs[@]}" -o "$out/time_sync"
if (( verbose )); then
    timeout 60 "$out/time_sync"
else
    timeout 60 "$out/time_sync" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  time_sync (realtime clock, TIMER read, thread, NTP exchanges, ntp status, FreeRTOS POSIX port)\n'

# The sockets (freertos/tests/socket): the zsock_*() calls of the layer's <zephyr/net/socket.h>
# (drivers/nina/nina_socket.c: the descriptors, their locks, the Zephyr build's wait loops) on
# the shared core (common/drivers/nina/nina_sock.c, nina_dns.c, with the prelude, as
# drivers/nina/nina.cmake builds them) and the address conversions (drivers/libc/inet.c), on a
# fake module and a fake clock (the test's nina_request() and nina_os.h), on the same kernel and
# layer objects.  drivers/libc/include (newlib's missing network headers) comes before the host's.
sockinc=(-Idrivers/libc/include -I../common/drivers/nina)
sockobjs=()
for src in drivers/nina/nina_socket.c drivers/libc/inet.c tests/socket/main.c \
           ../common/drivers/nina/nina_sock.c ../common/drivers/nina/nina_dns.c \
           ../common/drivers/nina/nina_wait.c; do
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

# Wi-Fi (freertos/tests/wifi): the build's network parts (drivers/compat/zephyr_net_if.c, the
# "net_mgmt" thread of zephyr_net_mgmt_os.h, drivers/nina/nina_netif.c) under the shared ones as
# drivers/wifi/wifi.cmake builds them (the NINA driver common/drivers/nina/nina_net.c, the
# layer's net_mgmt and Wi-Fi requests, the stored credentials and their settings backend, the
# Wi-Fi profile, the "wifi" command and the copy of Zephyr's wifi_credentials_shell.c, with the
# "ifconfig" command (drivers/wifi/nano_ifconfig.c), with the
# prelude, CONFIG_NANO_WIFI_CREDENTIALS and the same per-file options), on the store and the
# copies of Zephyr's settings and NVS over the shared fake flash (common/tests/kv_flash_fake.c),
# the build's shell on a console that captures, and a fake module (the test's nina_request());
# on the same kernel and layer objects.  It waits for the driver's and the profile's real
# delays (the link monitor's 2 s, the reconnection's 4 s): about 45 s.  The scans with the
# firmware's capability bits (a fake nina_probe_capabilities()): the link kept, the asynchronous
# scan and its failures, a loss of the link during the scan.
wifiinc=(-D_GNU_SOURCE -DZ_SHELL_CMD_ROOT_GLOBAL -DCONFIG_GETOPT_LONG=1 -DCONFIG_NANO_WIFI_CREDENTIALS=1
         -U_FORTIFY_SOURCE -Itests/shell -Idrivers/shell -Idrivers/wifi -Idrivers/nina
         -I../common/shell -I../common/compat/lib/utils/getopt -I../common/drivers/nina
         -I../common/imu -I../common/tests -I../common/compat/subsys/settings/include
         -I../common/compat/subsys/settings/src -ffunction-sections -fdata-sections)
wifiobjs=()
for src in drivers/compat/zephyr_net_if.c drivers/nina/nina_netif.c drivers/wifi/nano_ifconfig.c \
           tests/wifi/main.c \
           drivers/shell/cli.c drivers/shell/getopt_state.c drivers/shell/nano_console.c \
           drivers/shell/nano_shell.c drivers/shell/shell_argv.c drivers/libc/stdio_wrap.c \
           ../common/shell/zshell.c ../common/shell/subcmd.c \
           ../common/compat/subsys/portability/posix/c_lib_ext/fnmatch.c \
           ../common/compat/lib/utils/getopt/getopt.c ../common/compat/lib/utils/getopt/getopt_long.c \
           ../common/drivers/nina/nina_net.c ../common/compat/zephyr_net_mgmt.c \
           ../common/compat/zephyr_wifi_mgmt.c ../common/compat/zephyr_wifi_utils.c \
           ../common/compat/wifi_credentials.c ../common/compat/wifi_credentials_backend_settings.c \
           ../common/imu/wifi_profile.c ../common/compat/zephyr_wifi_shell.c \
           ../common/compat/subsys/net/lib/wifi_credentials/wifi_credentials_shell.c \
           ../common/tests/kv_flash_fake.c ../common/imu/kv_store.c \
           ../common/compat/kv_store_wipe.c ../common/compat/zephyr_flash_kv.c \
           ../common/compat/lib/utils/hex.c ../common/compat/subsys/settings/src/settings.c \
           ../common/compat/subsys/settings/src/settings_store.c \
           ../common/compat/subsys/settings/src/settings_init.c \
           ../common/compat/subsys/settings/src/settings_nvs.c \
           ../common/compat/subsys/crc/crc8_sw.c ../common/compat/subsys/kvss/nvs/nvs.c; do
    obj="$out/wifi_$(basename "$src" .c).o"
    extra=()
    case $src in
        ../common/tests/*) ;;
        ../common/shell/*|*/fnmatch.c|*/getopt*.c) ;;
        ../common/*) extra=(-include drivers/compat/common_prelude.h) ;;
    esac
    case $src in
        */subsys/settings/*|*/subsys/crc/*|*/subsys/kvss/*) extra+=("${kvconfig[@]}") ;;
    esac
    case $src in
        */nvs.c) extra+=(-Wno-format -Wno-sign-compare -Wno-unused-parameter) ;;
        */wifi_credentials_shell.c) extra+=(-Wno-format -Wno-unused-parameter) ;;
        */wifi_credentials.c) extra+=(-Wno-type-limits) ;;
        */wifi_credentials_backend_settings.c) extra+=(-Wno-sign-compare) ;;
        *zshell.c|*nano_console.c) extra+=(-Dcalloc=test_calloc -Dmalloc=test_malloc -Dfree=test_free) ;;
        # A scan that never ends gives up after 1 s here (10 s on the board).
        */nina_net.c) extra+=(-DNINA_SCAN_WAIT_MS=1000) ;;
    esac
    cc "${flags[@]}" "${strict[@]}" "${wifiinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
    wifiobjs+=("$obj")
done
cc "${flags[@]}" -Wl,--gc-sections "${wraps[@]}" "${kernobjs[@]}" "${wifiobjs[@]}" -o "$out/wifi"
if (( verbose )); then
    timeout 120 "$out/wifi"
else
    timeout 120 "$out/wifi" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
fi
printf 'ok  wifi (interface, events, credentials, reconnection, wifi and ifconfig commands, FreeRTOS POSIX port)\n'

# The Telnet shell and "zperf" (freertos/tests/telnet): the build's Telnet backend
# (drivers/telnet/nano_telnet.c, its port a variable here) and shell, the port's iperf3 and the
# "zperf" command (common/compat/zperf_iperf3.c, zephyr_zperf_shell.c, with the prelude, as
# drivers/zperf/zperf.cmake builds them), on the zsock_*() calls over the host's sockets with
# the NINA's behaviour (tests/telnet/host_sock.c), on the same kernel and layer objects.  The
# program is the board; tests/telnet/client.py starts it and is the client: the Telnet
# protocol and line ends, a second client refused, 140 connections and disconnections, the
# USB session's output kept apart, a failed connection, a client that stops reading, and with
# the host's iperf3 (127.0.0.1 only; skipped without it) "zperf tcp upload" to "iperf3 -s" and
# "iperf3 -c" to "zperf tcp download".
if command -v python3 >/dev/null; then
    tninc=(-D_GNU_SOURCE -DZ_SHELL_CMD_ROOT_GLOBAL -DCONFIG_GETOPT_LONG=1 -U_FORTIFY_SOURCE
           -Itests/telnet -Itests/shell -Idrivers/shell -Idrivers/telnet -I../common/shell
           -I../common/compat/lib/utils/getopt -ffunction-sections -fdata-sections)
    tnobjs=()
    for src in drivers/telnet/nano_telnet.c tests/telnet/main.c tests/telnet/host_sock.c \
               drivers/shell/getopt_state.c drivers/shell/nano_console.c drivers/shell/nano_shell.c \
               drivers/shell/shell_argv.c drivers/libc/stdio_wrap.c ../common/shell/zshell.c \
               ../common/shell/subcmd.c ../common/compat/subsys/portability/posix/c_lib_ext/fnmatch.c \
               ../common/compat/lib/utils/getopt/getopt.c ../common/compat/lib/utils/getopt/getopt_long.c \
               ../common/compat/zperf_iperf3.c ../common/compat/zephyr_zperf_shell.c; do
        obj="$out/telnet_$(basename "$src" .c).o"
        extra=()
        case $src in
            */zperf_iperf3.c|*/zephyr_zperf_shell.c) extra=(-include drivers/compat/common_prelude.h) ;;
            */nano_telnet.c) extra=(-DCONFIG_SHELL_TELNET_PORT=test_telnet_port
                                    "-include" tests/telnet/port.h) ;;
            *zshell.c|*nano_console.c) extra=(-Dcalloc=test_calloc -Dmalloc=test_malloc -Dfree=test_free) ;;
        esac
        cc "${flags[@]}" "${strict[@]}" "${tninc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
        tnobjs+=("$obj")
    done
    cc "${flags[@]}" -Wl,--gc-sections "${wraps[@]}" "${kernobjs[@]}" "${tnobjs[@]}" -o "$out/telnet_test"
    if (( verbose )); then
        timeout 300 python3 tests/telnet/client.py "$out/telnet_test" -v
    else
        timeout 300 python3 tests/telnet/client.py "$out/telnet_test" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
        tail -n 1 "$out/log"
    fi
    printf 'ok  telnet (protocol, one client, reconnections, stalls, zperf, FreeRTOS POSIX port)\n'
else
    printf 'skip telnet (no python3)\n'
fi

# The "sensor" command (freertos/tests/sensor_shell): the shared common/drivers/sensor_shell/
# sensor_shell.c, the copy of Zephyr's sensor_shell_stream.c with its processing thread and the
# build's part (drivers/sensor_shell/sensor_shell_os.c), with "imu cal" (common/imu/imu_cal.c,
# imu_cal_codec.c, on a fake key-value store), on the shared LSM6DSO driver with its FIFO stream
# and ST's lsm6dso_reg.c (freertos/stmemsc), the layer's devices, gpio0, RTIO and its work queue
# and the default sensor iodev and decoder, with the prelude, CONFIG_NANO_LSM6DSO_STREAM and the
# per-file options of drivers/lsm6dso/lsm6dso.cmake and drivers/sensor_shell/sensor_shell.cmake;
# the layer's kernel part again with that option (gpio0); the build's shell with two sessions on
# consoles that capture; a fake LSM6DSOX behind a fake i2c0 and a fake INT1 on the layer's GPIO
# boundary.  On the same kernel objects.  The results on the session that asked, the wait of
# "sensor get", the calibration in the printed values, "sensor stream" from one session and "off"
# from the other, a session whose terminal is gone, the FIFO taken by another stream.  About 2 s.
if [[ -f stmemsc/lsm6dso_reg.c ]]; then
    ssinc=(-D_GNU_SOURCE -DZ_SHELL_CMD_ROOT_GLOBAL -DCONFIG_GETOPT_LONG=1 -DCONFIG_NANO_LSM6DSO_STREAM=1
           -U_FORTIFY_SOURCE -Itests/shell -Idrivers/shell -Idrivers/sensor_shell -Idrivers/lsm6dso
           -I../common/shell -I../common/compat/lib/utils/getopt -I../common/drivers/lsm6dso
           -I../common/drivers/stmemsc -Istmemsc -I../common/imu -ffunction-sections -fdata-sections)
    cc "${flags[@]}" -w -Istmemsc -c stmemsc/lsm6dso_reg.c -o "$out/sensor_lsm6dso_reg.o"
    ssobjs=("$out/sensor_lsm6dso_reg.o")
    for src in tests/sensor_shell/main.c drivers/compat/zephyr_kernel.c \
               drivers/sensor_shell/sensor_shell_os.c \
               drivers/shell/cli.c drivers/shell/getopt_state.c drivers/shell/nano_console.c \
               drivers/shell/nano_shell.c drivers/shell/shell_argv.c drivers/libc/stdio_wrap.c \
               ../common/shell/zshell.c ../common/shell/subcmd.c \
               ../common/compat/subsys/portability/posix/c_lib_ext/fnmatch.c \
               ../common/compat/lib/utils/getopt/getopt.c ../common/compat/lib/utils/getopt/getopt_long.c \
               ../common/drivers/sensor_shell/sensor_shell.c ../common/compat/sensor_shell_stream.c \
               ../common/imu/imu_cal.c ../common/imu/imu_cal_codec.c \
               ../common/drivers/lsm6dso/lsm6dso.c ../common/drivers/lsm6dso/lsm6dso_fifo_hw.c \
               ../common/drivers/lsm6dso/lsm6dso_fifo.c ../common/drivers/lsm6dso/lsm6dso_cal.c \
               ../common/drivers/lsm6dso/lsm6dso_zero.c ../common/drivers/lsm6dso/lsm6dso_stream.c \
               ../common/drivers/lsm6dso/lsm6dso_fifo_irq.c ../common/drivers/lsm6dso/lsm6dso_compat.c \
               ../common/drivers/lsm6dso/lsm6dso_fifo_decoder.c \
               ../common/drivers/stmemsc/stmemsc_i2c.c ../common/drivers/stmemsc/stmemsc_mdelay.c \
               ../common/compat/zephyr_device.c ../common/compat/zephyr_gpio.c \
               ../common/compat/sensor_default_rtio.c ../common/compat/zephyr_rtio.c \
               ../common/compat/zephyr_mem_blocks.c; do
        obj="$out/sensor_$(basename "$src" .c).o"
        extra=()
        case $src in
            ../common/shell/*|*/fnmatch.c|*/getopt*.c) ;;
            ../common/*|*/sensor_shell_os.c|tests/sensor_shell/*) extra=(-include drivers/compat/common_prelude.h) ;;
        esac
        case $src in
            */lsm6dso.c) extra+=(-Wno-unused-parameter -Wno-sign-compare -Wno-cast-function-type) ;;
            */lsm6dso_compat.c|*/sensor_default_rtio.c) extra+=(-Wno-unused-parameter -Wno-sign-compare) ;;
            */sensor_shell.c) extra+=(-Wno-sign-compare -Wno-unused-parameter) ;;
            *zshell.c|*nano_console.c) extra+=(-Dcalloc=test_calloc -Dmalloc=test_malloc -Dfree=test_free) ;;
        esac
        cc "${flags[@]}" "${strict[@]}" "${ssinc[@]}" "${inc[@]}" "${extra[@]}" -c "$src" -o "$obj"
        ssobjs+=("$obj")
    done
    sskern=()
    for o in "${kernobjs[@]}"; do
        [[ $(basename "$o") == zephyr_kernel.o ]] || sskern+=("$o")
    done
    cc "${flags[@]}" -Wl,--gc-sections "${wraps[@]}" "${sskern[@]}" "${ssobjs[@]}" -lm -o "$out/sensor_shell"
    if (( verbose )); then
        timeout 120 "$out/sensor_shell"
    else
        timeout 120 "$out/sensor_shell" >"$out/log" 2>&1 || { cat "$out/log"; exit 1; }
    fi
    printf 'ok  sensor_shell (sessions, get, calibration, stream, FreeRTOS POSIX port)\n'
else
    printf 'skip sensor_shell (no stmemsc/lsm6dso_reg.c: run script/setup.sh)\n'
fi
