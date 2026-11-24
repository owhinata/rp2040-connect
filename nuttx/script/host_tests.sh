#!/usr/bin/env bash
# Build and run the host tests for the OS-independent sources under nuttx/.
# The tests of the sources shared with the Zephyr build and of the compatibility layer's
# shared part run first (common/tests/run.sh).
# Most of the other sources and tests are unchanged copies of the Zephyr version's
# (rp2040_timeconv is the board's own); none needs the board.
set -euo pipefail
cd "$(dirname "$0")/.."
../common/tests/run.sh
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
imu=apps/imu
lsm=drivers/lsm6dso
cimu=../common/imu
clsm=../common/drivers/lsm6dso
# The driver's directories (its top for the tests that include the core) for the sources that
# include its Zephyr binding: port/nuttx before port/zephyr (lsm6dso_sensor.c reads this build's
# <lsm6dso_instance.h>).
lsmdirs=(-I"$clsm" -I"$clsm/port/nuttx" -I"$clsm/port/zephyr" -I"$clsm/include"
         -I"$clsm/internal" -I"$clsm/port")
# ST's glue to the Zephyr I2C API (stmemsc.h, stmemsc_i2c.c, stmemsc_mdelay.c), shared with the
# FreeRTOS build.
cstm=../common/drivers/stmemsc
# The Zephyr compatibility layer: its NuttX part (common/compat/port/nuttx, with the NuttX parts
# app_log_os.h and priorities_os.h of the shared headers), its shared part
# (common/compat, headers in include, port and internal) and the fake kernel of the host tests (common/tests/zephyr_fakes), which
# comes first (its zephyr_os*.h and <zephyr/sys/__assert.h> in place of the NuttX ones).
ccompat=../common/compat
cnuttx=../common/compat/port/nuttx
zfakes=../common/tests/zephyr_fakes
# The Zephyr shell's command handling and the subcommand dispatch (common/shell; the NuttX
# backends of the former are apps/subcmd/zshell_os_impl.h) and the logging macros
# (common/compat/include/app_log.h; their NuttX part app_log_os.h is in common/compat/port/nuttx).
cshell=../common/shell
# The RP2040's code that no OS owns (common/chip/rp2040: the TIMER's read).
cchip=../common/chip/rp2040/include
flags=(-std=c17 -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
link="-Wl,--gc-sections"
# The prelude that the NuttX build includes before each source of common/ (on the host: the
# Zephyr build's Kconfig values that those sources use).
prelude=$cnuttx/common_prelude.h
# The fake headers shared with the Zephyr build (common/tests/fakes) and the port's own
# (tests/fakes: nuttx/config.h).
inc=(-I../common/tests/fakes -Itests/fakes -I"$imu" -I"$cimu" -I"$lsm" -I"$clsm/include"
     -I"$clsm/internal" -I"$clsm/port" -I"$clsm/port/zephyr")

# run NAME CC-ARGS...: build tests/NAME.c with the arguments, run it, stop on failure.
run() {
    local name=$1
    shift
    cc "${flags[@]}" "${inc[@]}" "tests/$name.c" "$@" "$link" -o "$out/$name"
    "$out/$name" >/dev/null
    printf 'ok  %s\n' "$name"
}

# Application modules and the FIFO word parser (common/), for imu_raw_mqtt below.
lib=("$cimu/imu_raw_queue.c" "$cimu/imu_raw_packet.c" "$cimu/imu_raw_batch.c"
     "$cimu/imu_raw_status.c" "$cimu/imu_raw_time.c" "$cimu/imu_tick_clock.c"
     "$cimu/time_offset.c" "$clsm/lsm6dso_fifo.c" "$clsm/port/zephyr/lsm6dso_decoder.c")

# The LSM6DSO driver (common/drivers/lsm6dso: the core lsm6dso.c, which the test includes for its
# static functions, and the Zephyr binding port/zephyr/lsm6dso_sensor.c with
# port/nuttx/lsm6dso_instance.h) and its FIFO stream (the core's lsm6dso_stream.c; the binding's
# lsm6dso_rtio_stream.c, lsm6dso_os.c, lsm6dso_rtio.c, the decoder) on ST's real lsm6dso_reg.c, the
# Zephyr compatibility layer
# (I2C, devices, work queues, RTIO, gpio0, the default sensor iodev and decoder) with the fake
# kernel of common/tests/zephyr_fakes, and the board's devices, against a fake sensor behind a fake
# I2C master and a fake INT1.  Its own include path: the compatibility layer's headers and
# ST's, not the fakes of common/tests/fakes.  ST's file (fetched by setup.sh into nuttx/stmemsc) is
# built without warnings, as it is; the Zephyr driver's code casts its bus functions, leaves
# parameters unused and compares signed with unsigned, as in the Zephyr build.  The prelude
# follows the fake <nuttx/config.h>, as on NuttX (CONFIG_SENSOR_ASYNC_API from
# CONFIG_LSM6DSO_STREAM).
compat=("$ccompat/zephyr_work.c" "$ccompat/zephyr_rtio.c" "$ccompat/zephyr_mem_blocks.c"
        "$ccompat/zephyr_rtio_workq.c" "$zfakes/zephyr_kernel_host.c")
if [[ -f stmemsc/lsm6dso_reg.c ]]; then
    cc -std=c17 -w -Istmemsc -c stmemsc/lsm6dso_reg.c -o "$out/lsm6dso_reg.o"
    cc "${flags[@]}" -Wno-unused-parameter -Wno-cast-function-type -Wno-sign-compare \
        -D_DEFAULT_SOURCE -D'DEBUGASSERT(x)=assert(x)' -include tests/lsm6dso_fakes/nuttx/config.h \
        -include "$prelude" -Itests/lsm6dso_fakes \
        -I"$zfakes" \
        -I"$cnuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$ccompat/include" -Istmemsc -I"$lsm" "${lsmdirs[@]}" -I"$cstm" \
        -Iboards/arduino-nano-rp2040-connect/src \
        tests/lsm6dso_driver.c "$cstm/stmemsc_i2c.c" "$cstm/stmemsc_mdelay.c" \
        "$clsm/lsm6dso_cal.c" "$clsm/lsm6dso_fifo.c" "$clsm/lsm6dso_stream.c" \
        "$clsm/port/zephyr/lsm6dso_sensor.c" "$clsm/port/zephyr/lsm6dso_rtio.c" \
        "$clsm/port/zephyr/lsm6dso_rtio_stream.c" "$clsm/port/zephyr/lsm6dso_os.c" \
        "$clsm/port/zephyr/lsm6dso_decoder.c" \
        "$cnuttx/zephyr_i2c.c" "$ccompat/zephyr_device.c" \
        "$ccompat/zephyr_gpio.c" "$ccompat/sensor_default_rtio.c" "${compat[@]}" \
        boards/arduino-nano-rp2040-connect/src/rp2040_lsm6dso.c "$out/lsm6dso_reg.o" -lm "$link" \
        -o "$out/lsm6dso_driver"
    "$out/lsm6dso_driver" >/dev/null
    printf 'ok  lsm6dso_driver\n'
    # The calibration (common/imu/imu_cal.c, shared with the Zephyr build) on the same driver
    # and fake sensor (the test includes tests/lsm6dso_driver.c and runs it first) with a
    # fake key-value store: the lookup of the stored values, the driver's output with them,
    # range changes, reload, zero and the "imu cal" lines.
    cc "${flags[@]}" -Wno-unused-parameter -Wno-cast-function-type -Wno-sign-compare \
        -D_DEFAULT_SOURCE -D'DEBUGASSERT(x)=assert(x)' -include tests/lsm6dso_fakes/nuttx/config.h \
        -include "$prelude" -Itests/lsm6dso_fakes \
        -I"$zfakes" \
        -I"$cnuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$ccompat/include" -Istmemsc -I"$lsm" "${lsmdirs[@]}" -I"$cstm" -I"$imu" -I"$cimu" \
        -Iboards/arduino-nano-rp2040-connect/src \
        tests/imu_cal.c "$cimu/imu_cal.c" "$cimu/imu_cal_codec.c" \
        "$cstm/stmemsc_i2c.c" "$cstm/stmemsc_mdelay.c" \
        "$clsm/lsm6dso_cal.c" "$clsm/lsm6dso_fifo.c" "$clsm/lsm6dso_stream.c" \
        "$clsm/port/zephyr/lsm6dso_sensor.c" "$clsm/port/zephyr/lsm6dso_rtio.c" \
        "$clsm/port/zephyr/lsm6dso_rtio_stream.c" "$clsm/port/zephyr/lsm6dso_os.c" \
        "$clsm/port/zephyr/lsm6dso_decoder.c" \
        "$cnuttx/zephyr_i2c.c" "$ccompat/zephyr_device.c" \
        "$ccompat/zephyr_gpio.c" "$ccompat/sensor_default_rtio.c" "${compat[@]}" \
        boards/arduino-nano-rp2040-connect/src/rp2040_lsm6dso.c "$out/lsm6dso_reg.o" -lm "$link" \
        -o "$out/imu_cal"
    "$out/imu_cal" >/dev/null
    printf 'ok  imu_cal\n'
    # The "sensor" command (common/shell/sensor_shell.c, shared with the Zephyr
    # build, the copy of Zephyr's sensor_shell_stream.c in common/compat, and apps/sensor: the
    # NuttX part) on the Zephyr shell's command handling (common/shell/zshell.c) and
    # the driver with its fake sensor (lsm6dso_driver.c, included): the help, argument checks,
    # wildcards, device names, the output of get / attr_get / attr_set / stream / trig, printed
    # by the processing callback on the command's terminal.  Its main() is sensor_main().
    sinc=(-D_GNU_SOURCE -D'DEBUGASSERT(x)=assert(x)' -DSENSOR_SHELL_GET_WAIT_MS=1
          -include tests/lsm6dso_fakes/nuttx/config.h -include "$prelude"
          -Itests/lsm6dso_fakes -I"$zfakes" -I"$cnuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$ccompat/include" -Istmemsc
          -I"$lsm" "${lsmdirs[@]}" -I"$cstm" -Iapps/sensor -I"$cshell" -Iapps/subcmd -Iboards/arduino-nano-rp2040-connect/src)
    sflags=("${flags[@]}" -Wno-unused-parameter -Wno-cast-function-type -Wno-sign-compare)
    cc "${sflags[@]}" "${sinc[@]}" -Dmain=sensor_main -c ../common/shell/sensor_shell.c \
        -o "$out/sensor_shell.o"
    cc "${sflags[@]}" "${sinc[@]}" tests/sensor_shell.c "$out/sensor_shell.o" \
        "$ccompat/sensor_shell_stream.c" apps/sensor/sensor_shell_nuttx.c "$cshell/zshell.c" \
        "$cshell/subcmd.c" "$cstm/stmemsc_i2c.c" "$cstm/stmemsc_mdelay.c" \
        "$clsm/lsm6dso_cal.c" "$clsm/lsm6dso_fifo.c" "$clsm/lsm6dso_stream.c" \
        "$clsm/port/zephyr/lsm6dso_sensor.c" "$clsm/port/zephyr/lsm6dso_rtio.c" \
        "$clsm/port/zephyr/lsm6dso_rtio_stream.c" "$clsm/port/zephyr/lsm6dso_os.c" \
        "$clsm/port/zephyr/lsm6dso_decoder.c" \
        "$cnuttx/zephyr_i2c.c" "$ccompat/zephyr_device.c" \
        "$ccompat/zephyr_gpio.c" "$ccompat/sensor_default_rtio.c" "${compat[@]}" \
        boards/arduino-nano-rp2040-connect/src/rp2040_lsm6dso.c "$out/lsm6dso_reg.o" -lm "$link" \
        -o "$out/sensor_shell"
    "$out/sensor_shell" >/dev/null
    printf 'ok  sensor_shell\n'
else
    printf 'skip lsm6dso_driver (no stmemsc/lsm6dso_reg.c: run script/setup.sh)\n'
    printf 'skip imu_cal (no stmemsc/lsm6dso_reg.c: run script/setup.sh)\n'
    printf 'skip sensor_shell (no stmemsc/lsm6dso_reg.c: run script/setup.sh)\n'
fi
# The compatibility layer on the fake kernel, with its NuttX headers (<zephyr/net/socket.h>,
# <zephyr/shell/shell.h>, ...).  Its RTIO and work items are tested in common/tests/run.sh.
# <zephyr/net/socket.h> reads <nina_dns.h> of the NINA core (common/drivers/nina).
cnina=../common/drivers/nina
# The waits for NINA's READY on its edge interrupt (common/drivers/nina/port/nuttx/nina_os.c,
# nina_os_ready_*())
# under the shared core's wait_ready() (common/drivers/nina/internal/nina_proto.c, included by the test),
# through the layer's gpio0 (common/compat/zephyr_gpio.c, with the prelude and the fake kernel's
# spin lock), on fakes of NuttX's semaphores, mutexes, sleeps and clock (tests/nina_os_fakes) and
# of the board's GPIO bank 0: ready during the wait, the timeout, a change right after the read,
# stale wake-ups.  Also the socket events on the IRQ line (GPIO2, with CONFIG_NINA_USRSOCK as
# nina/Make.defs builds nina_os.c), with the socket loops' waits (nina_wait.c), and the Wi-Fi
# operations' lock and the Wi-Fi thread's wake-up (CONFIG_NINA_NET).
ninaos=(-DCONFIG_NINA_USRSOCK=1 -DCONFIG_NINA_NET=1 -Itests/nina_os_fakes -Itests/fakes -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal"
        -I"$cnina/include" -I"$cnina/internal" -I"$cnina/port" -I"$cnina/port/nuttx")
cc "${flags[@]}" "${ninaos[@]}" -I"$zfakes" -c "$cnina/port/nuttx/nina_os.c" -o "$out/nina_os_os.o"
cc "${flags[@]}" -include "$prelude" -I"$zfakes" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -c "$ccompat/zephyr_gpio.c" \
    -o "$out/nina_os_gpio.o"
cc "${flags[@]}" "${ninaos[@]}" tests/nina_os.c "$out/nina_os_os.o" "$out/nina_os_gpio.o" \
    "$cnina/internal/nina_wait.c" "$link" -o "$out/nina_os"
"$out/nina_os" >/dev/null
printf 'ok  nina_os\n'
# The NINA core's NuttX binding, wlan0 (common/drivers/nina/port/nuttx/nina_netdev.c, which the test
# includes), on NuttX's own <nuttx/wireless/wireless.h> (the fetched tree) and fakes of the other
# NuttX declarations it uses (tests/nina_netdev_fakes), of the core's Wi-Fi functions and of the
# applications' management: the start, the core's operations on the device, wapi's ioctls (scan,
# its results as wireless events, the settings and the connection with Zephyr's checks of the
# key, disconnect, status, modes), the threads.  The operations themselves are
# common/tests/nina_wifi.c's.
ndinc=(-D_DEFAULT_SOURCE -DFAR= -Itests/nina_netdev_fakes -Itests/fakes -I"$cnina/include"
       -I"$cnina/internal" -I"$cnina/port" -I"$cnina/port/nuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$cnuttx")
if [[ -f nuttx/include/nuttx/wireless/wireless.h ]]; then
    cc "${flags[@]}" "${ndinc[@]}" tests/nina_netdev.c "$link" -o "$out/nina_netdev"
    "$out/nina_netdev" >/dev/null
    printf 'ok  nina_netdev\n'
else
    printf 'skip nina_netdev (no nuttx/nuttx: run script/setup.sh)\n'
fi
# The applications' Zephyr Wi-Fi management on the core (common/drivers/nina/port/nuttx/
# nina_wifi_mgmt.c) under the layer's Wi-Fi requests (common/compat/zephyr_wifi_mgmt.c) and
# interface (common/compat/port/nuttx/zephyr_net_if.c), on fakes of the device's lock, net_mgmt's events, the
# core's connect: the start, NET_REQUEST_WIFI_CONNECT and the requests not served, the events,
# the addresses.
run nina_wifi_mgmt "${ndinc[@]}" -I"$zfakes" -I"$ccompat/include" -I"$cimu" \
    "$cnina/port/nuttx/nina_wifi_mgmt.c" "$ccompat/zephyr_wifi_mgmt.c" \
    "$cnuttx/zephyr_net_if.c"
zinc=(-D_DEFAULT_SOURCE -I"$zfakes" -I"$cnuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$ccompat/include" -I"$cnina/include")
# The layer's pieces for the IMU service: K_MUTEX_DEFINE, K_THREAD_DEFINE and its start,
# k_uptime_get's tick rounding, k_sleep, byteorder, snprintk, hwinfo (fake chip ID), the
# shell output macros, the address conversions.
cc "${flags[@]}" "${zinc[@]}" -Itests/lsm6dso_fakes tests/zephyr_compat_misc.c \
    "$cnuttx/zephyr_hwinfo.c" "${compat[@]}" "$link" -o "$out/zephyr_compat_misc"
"$out/zephyr_compat_misc" >/dev/null
printf 'ok  zephyr_compat_misc\n'
# The layer's k_mutex on NuttX (common/compat/port/nuttx/zephyr_mutex.c), built as for NuttX on a model
# of NuttX's recursive mutex (an all-zero one is held by thread 0): K_MUTEX_DEFINE, k_mutex_init,
# and a mutex left all zero (the MQTT client's before mqtt_client_init()), which is unlocked
# as in Zephyr.
cc "${flags[@]}" -D_DEFAULT_SOURCE -D__NuttX__ -Itests/zephyr_mutex_fakes -I"$cnuttx" \
    -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$cchip" -I"$ccompat/include" tests/zephyr_mutex.c "$cnuttx/zephyr_mutex.c" "$link" \
    -o "$out/zephyr_mutex"
"$out/zephyr_mutex" >/dev/null
printf 'ok  zephyr_mutex\n'
# The DMA part of the I2C0 reads (i2c_dw_dma_os_rp2040.c of the I2C core's NuttX binding,
# common/drivers/i2c_dw/port/nuttx, the boundary of common/drivers/i2c_dw/port/i2c_dw_dma_os.h)
# on a fake of NuttX's rp2040_dmac.c and of the DMA's registers (tests/i2c_dw_dma_os_fakes, and
# the test itself): the channels, their setup, start, interrupt, stop and release.
i2cdir=../common/drivers/i2c_dw
cc "${flags[@]}" -D_DEFAULT_SOURCE -Itests/i2c_dw_dma_os_fakes -Itests/fakes \
    -I"$i2cdir/port" tests/i2c_dw_dma_os.c "$link" -o "$out/i2c_dw_dma_os"
"$out/i2c_dw_dma_os" >/dev/null
printf 'ok  i2c_dw_dma_os\n'
# The channel allocation of NuttX's RP2040 DMA driver itself (arch/arm/src/rp2040/rp2040_dmac.c
# of the fetched tree, with the local patches 0011 and 0012) on a model of the DMA's registers
# and fakes of the kernel's lock and count (tests/rp2040_dmac_fakes): taking and freeing a
# channel leaves the other channels' pending interrupt flags, and the allocation that does not
# wait gives the count back when the allocator's lock is taken.
if [[ -f nuttx/arch/arm/src/rp2040/rp2040_dmac.c ]]; then
    cc "${flags[@]}" -D_DEFAULT_SOURCE -Wno-unused-function -Wno-unused-parameter \
        -Wno-sign-compare -Wno-pointer-to-int-cast -Itests/rp2040_dmac_fakes -Inuttx/arch/arm/src/rp2040 \
        tests/rp2040_dmac.c -o "$out/rp2040_dmac"
    "$out/rp2040_dmac" >/dev/null
    printf 'ok  rp2040_dmac\n'
else
    printf 'skip rp2040_dmac (no nuttx/nuttx: run script/setup.sh)\n'
fi
# The I2C core's NuttX binding (common/drivers/i2c_dw/port/nuttx: i2c_dw_os_impl.h,
# i2c_dw_instance.h) under the shared core (common/drivers/i2c_dw/i2c_dw.c),
# on fakes of the NuttX headers (tests/i2c_dw_instance_fakes), the board's include/board.h, the
# controllers' registers and the DMA boundary, with a thread that plays the controllers and
# their devices: three times, I2C0 only (the default configuration), with I2C1
# (CONFIG_I2C_DW_I2C1), whose run adds the tests of I2C1, and with a DMA boundary that fails to
# start (TEST_DMA_INIT_FAILS: I2C0 starts all the same, on the interrupts; tests/i2c_dw_instance.c).
# The host's cc reports -Wsign-compare for two MIN()s of i2c_dw.c, as in the FreeRTOS build's test.
for variant in i2c0 i2c1 nodma; do
    i2cdefs=(-DCONFIG_I2C_DW_I2C0=1 -DCONFIG_I2C_DW_RP2040_DMA=1 -DCONFIG_I2C_RESET=1)
    [[ $variant == i2c1 ]] && i2cdefs+=(-DCONFIG_I2C_DW_I2C1=1)
    [[ $variant == nodma ]] && i2cdefs+=(-DTEST_DMA_INIT_FAILS=1)
    cc "${flags[@]}" -Wno-sign-compare -D_DEFAULT_SOURCE -pthread "${i2cdefs[@]}" \
        -Itests/i2c_dw_instance_fakes -Itests/fakes -I"$i2cdir" -I"$i2cdir/include" \
        -I"$i2cdir/internal" -I"$i2cdir/port" -I"$i2cdir/port/nuttx" -I"$cnuttx" -I"$ccompat/include" \
        tests/i2c_dw_instance.c "$link" -o "$out/i2c_dw_instance_$variant"
    timeout 60 "$out/i2c_dw_instance_$variant" >/dev/null
done
printf 'ok  i2c_dw_instance (I2C0, I2C1 with CONFIG_I2C_DW_I2C1, I2C0 without the DMA)\n'
# The test of the minimal MQTT client (common/tests/mqtt_min.c; the client, common/imu/mqtt_min.c,
# is tested in common/tests/run.sh) on Zephyr's own library when the Zephyr tree is there
# (tests/mqtt_ref: its log, mutex and socket names on the host, with this build's
# <zephyr/net/socket.h>), so the expected bytes, events and errors are Zephyr's.
zmqtt=../zephyr/zephyr/subsys/net/lib/mqtt
if [[ -f $zmqtt/mqtt.c ]]; then
    zobjs=()
    for f in mqtt mqtt_rx mqtt_encoder mqtt_decoder mqtt_transport mqtt_transport_socket_tcp; do
        cc -std=c17 -w -DCONFIG_MQTT_KEEPALIVE=60 -DCONFIG_MQTT_LOG_LEVEL=0 -Itests/mqtt_ref \
            "${zinc[@]}" -I"$cimu" -include zephyr/net/socket.h -c "$zmqtt/$f.c" -o "$out/zmqtt_$f.o"
        zobjs+=("$out/zmqtt_$f.o")
    done
    cc "${flags[@]}" "${zinc[@]}" -I"$cimu" ../common/tests/mqtt_min.c "${zobjs[@]}" "${compat[@]}" "$link" \
        -o "$out/mqtt_zephyr"
    "$out/mqtt_zephyr" >/dev/null
    printf 'ok  mqtt_min (on Zephyr'\''s MQTT library)\n'
else
    printf 'skip mqtt_min on Zephyr'\''s MQTT library (no zephyr/zephyr: run west update)\n'
fi
# Board: TIMER count <-> OS tick conversions of the tickless system timer.
run rp2040_timeconv -Iboards/arduino-nano-rp2040-connect/src
# The registration of "imu", "ntp" and "kv" (the SHELL_*() text cut out of
# common/imu/imu_stream.c, time_sync.c and kv_store.c, built with Z_SHELL_CMD_NAMED_MAIN on
# common/shell/zshell.c): the same exit status, handler calls and output as the same sets on
# subcmd_main() (for "kv": the NuttX build's table before, with "wipe"), and where they differ.
sed -n '/^SHELL_STATIC_SUBCMD_SET_CREATE(imu_commands,/,/^SHELL_CMD_REGISTER(imu,/p' \
    "$cimu/imu_stream.c" >"$out/imu_shell.inc"
sed -n '/^SHELL_STATIC_SUBCMD_SET_CREATE(ntp_commands,/,/^SHELL_CMD_REGISTER(ntp,/p' \
    "$cimu/time_sync.c" >"$out/ntp_shell.inc"
sed -n '/^SHELL_STATIC_SUBCMD_SET_CREATE($/,/^SHELL_CMD_REGISTER(kv,/p' \
    "$cimu/kv_store.c" >"$out/kv_shell.inc"
grep -q '^SHELL_CMD_REGISTER(imu,' "$out/imu_shell.inc"
grep -q '^SHELL_CMD_REGISTER(ntp,' "$out/ntp_shell.inc"
grep -q '^SHELL_CMD_REGISTER(kv,' "$out/kv_shell.inc"
grep -q 'KV_STORE_OS_COMMANDS' "$out/kv_shell.inc"
run shell_register -D_DEFAULT_SOURCE -DZ_SHELL_CMD_NAMED_MAIN -DIMU_SHELL_INC="\"$out/imu_shell.inc\"" \
    -DNTP_SHELL_INC="\"$out/ntp_shell.inc\"" -DKV_SHELL_INC="\"$out/kv_shell.inc\"" \
    -I"$cnuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$cshell" -Iapps/subcmd "$cshell/zshell.c" "$cshell/subcmd.c"
# The "i2c_dma" command (common/shell/i2c_dma_shell.c, shared with the Zephyr and
# FreeRTOS builds, with the prelude as apps/i2c_dma/Makefile builds it and
# Z_SHELL_CMD_NAMED_MAIN) on common/shell/zshell.c, with a fake driver side (the I2C0 core's
# functions of <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>, common/compat): the same exit status,
# calls and output as the NuttX build's earlier command on subcmd_main(), the text itself, and
# where the two differ.
run i2c_dma_app -D_DEFAULT_SOURCE -DZ_SHELL_CMD_NAMED_MAIN -include "$prelude" \
    -I"$cnuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$cshell" -Iapps/subcmd \
    "$cshell/zshell.c" "$cshell/subcmd.c"
# The "i2c" command (common/shell/i2c_shell.c, the copy of the Zephyr build's I2C
# shell, with the prelude and the options as apps/i2c builds it; Zephyr's text compares signed
# with unsigned and leaves the handlers' argc unused) on common/shell/zshell.c and the
# compatibility layer's I2C bus device (common/compat/port/nuttx/zephyr_i2c.c,
# common/compat/zephyr_device.c) with a fake NuttX I2C master (I2C_RESET() with
# CONFIG_I2C_RESET, as in the imu configuration).  Its main() is i2c_main().
i2cinc=(-D_DEFAULT_SOURCE -DCONFIG_I2C_RESET=1 -DCONFIG_I2C_SHELL_BUFFER_SIZE=64
        -DCONFIG_LOG_DEFAULT_LEVEL=3 -include tests/fakes/nuttx/config.h -include "$prelude"
        -Itests/fakes -I"$cnuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$ccompat/include" -I"$cshell"
        -Iapps/subcmd -Itests/lsm6dso_fakes)
cc "${flags[@]}" -Wno-sign-compare -Wno-unused-parameter "${i2cinc[@]}" -Dmain=i2c_main \
    -c ../common/shell/i2c_shell.c -o "$out/i2c_shell.o"
cc "${flags[@]}" "${i2cinc[@]}" tests/i2c_shell.c "$out/i2c_shell.o" "$cshell/zshell.c" \
    "$cshell/subcmd.c" "$cnuttx/zephyr_i2c.c" "$ccompat/zephyr_device.c" "$link" \
    -o "$out/i2c_shell"
"$out/i2c_shell" >/dev/null
printf 'ok  i2c_shell\n'
# "nina passthrough" of apps/nina (CONFIG_NINA_PASSTHROUGH) on the subcommand dispatch, with
# fakes of the NINA requests and of the driver's nina_passthrough_request()
# (common/drivers/nina/include/nina_passthrough.h).  The mode itself is common/tests/nina_passthrough.c's.
run nina_passthrough -DCONFIG_NINA_PASSTHROUGH=1 -I../common/drivers/nina/include -Iapps/nina -I"$cshell" \
    "$cshell/subcmd.c"
# The time synchronization (common/imu/time_sync.c, shared with the Zephyr build) on the
# compatibility layer with a fake NINA socket (NuttX's socket calls on usrsock), clock (the
# TIMER's registers under common/chip's rp2040_timer_us.h, tests/time_sync_fakes), interface
# and realtime clock: the NTP request and the checks of the reply, where t1 and t4 are read, the 1 ms polling of the reply through recv(MSG_DONTWAIT),
# answered from the RECVFROM_AVAIL of the event thread (its delay ends up in t4), a receive
# that waits in NuttX's recv(), the 60 s age, the forward-only realtime clock,
# the Wi-Fi events and the "ntp status" lines, through z_shell_main_ntp() (the file is built
# with Z_SHELL_CMD_NAMED_MAIN, as in the NuttX build) on the Zephyr shell's command handling
# (common/shell/zshell.c).  The prelude follows the fake <nuttx/config.h>, as on NuttX.
cc "${flags[@]}" -D_DEFAULT_SOURCE -DZ_SHELL_CMD_NAMED_MAIN -include tests/time_sync_fakes/nuttx/config.h \
    -include "$prelude" -Itests/time_sync_fakes -I"$zfakes" -I"$cnuttx" -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" \
    -I"$ccompat/include" -I"$cnina/include" -I"$imu" -I"$cimu" -I"$cshell" -Iapps/subcmd tests/time_sync.c \
    "$cimu/time_offset.c" "$cshell/zshell.c" "$cshell/subcmd.c" "${compat[@]}" "$link" -o "$out/time_sync"
"$out/time_sync" >/dev/null
printf 'ok  time_sync\n'
# The MQTT transport (common/imu/imu_raw_mqtt.c, shared with the Zephyr build) with the
# port's MQTT client and its socket layer on the compatibility layer's zsock_*() calls, on a
# fake socket (NuttX's socket calls on usrsock, with the RECVFROM_AVAIL that the NINA driver
# keeps or drops after a receive), clock and interface: the start checks and topics, the
# thread, the socket calls in order (NuttX's poll() for the CONNACK wait and the input,
# receives with MSG_DONTWAIT, a CONNACK in two pieces), CONNECT, the CONNACK wait, the order
# of info, status and batch with header and body in one
# sendmsg(), the status and input intervals, dropped batches and the error paths.
cc "${flags[@]}" -Wno-sign-compare -D_DEFAULT_SOURCE -include "$prelude" -Itests/time_sync_fakes \
    -Itests/lsm6dso_fakes "${zinc[@]}" "${inc[@]}" tests/imu_raw_mqtt.c "$cimu/mqtt_min.c" "${lib[@]}" "${compat[@]}" "$link" \
    -o "$out/imu_raw_mqtt"
"$out/imu_raw_mqtt" >/dev/null
printf 'ok  imu_raw_mqtt\n'
# The key-value store of the build (the copies of Zephyr's settings and NVS in
# common/compat/subsys, built as they are, without warnings, with the Zephyr build's Kconfig
# values; the flash device of the area, common/compat/zephyr_flash_kv.c; a fake flash,
# common/tests/kv_flash_fake.c), for the tests of the "kv" command and the Wi-Fi credentials
# below.  The store itself is tested in common/tests/run.sh (kv_settings).
ctests=../common/tests
kvset=(-include "$ccompat/include/zephyr_kv_config.h" -I"$ccompat/subsys/settings/include"
       -I"$ccompat/subsys/settings/src" -I"$ctests")
kvobjs=()
for f in "$ccompat/subsys/kvss/nvs/nvs.c" "$ccompat/subsys/settings/src/settings.c" \
         "$ccompat/subsys/settings/src/settings_store.c" \
         "$ccompat/subsys/settings/src/settings_init.c" \
         "$ccompat/subsys/settings/src/settings_nvs.c" "$ccompat/subsys/crc/crc8_sw.c" \
         "$ccompat/lib/utils/hex.c"; do
    o="$out/kv_$(basename "$f" .c).o"
    cc -std=gnu17 -w -ffunction-sections -fdata-sections -D_DEFAULT_SOURCE -Itests/lsm6dso_fakes \
        "${zinc[@]}" "${kvset[@]}" -c "$f" -o "$o"
    kvobjs+=("$o")
done
kvobjs+=("$ctests/kv_flash_fake.c" "$ccompat/zephyr_flash_kv.c")
# The "kv" command (common/imu/kv_store.c with "kv wipe" of common/compat/kv_store_wipe.c, through
# z_shell_main_kv() as apps/kv runs it, on common/shell/zshell.c with the NuttX backends) on that
# store: the output of every subcommand (the NuttX build's before it was shared: Saved, Deleted,
# Reserved key, hex, the errors, the help) on stdout and stderr, the stored bytes, "kv wipe" on
# foreign data.
cc "${flags[@]}" -D_GNU_SOURCE -DZ_SHELL_CMD_NAMED_MAIN -Itests/lsm6dso_fakes "${zinc[@]}" \
    "${kvset[@]}" -I"$cimu" -I"$cshell" -Iapps/subcmd tests/kv_cmd.c "$cimu/kv_store.c" \
    "$ccompat/kv_store_wipe.c" "$cshell/zshell.c" "$cshell/subcmd.c" "${kvobjs[@]}" "${compat[@]}" \
    "$link" -o "$out/kv_cmd"
"$out/kv_cmd" >/dev/null
printf 'ok  kv_cmd\n'
# The stored Wi-Fi credentials (common/compat: the copies of Zephyr's wifi_credentials.c and its
# settings backend) and the "wifi cred" command (apps/wifi: the copy of wifi_credentials_shell.c
# on subcmd), on the store above: the stored entries and names, the cache, the lengths and
# options, the messages and the log.  The copied text leaves parameters unused and compares
# signed with unsigned, as in the Zephyr build; the command's main() is wifi_main().
winc=(-D_GNU_SOURCE -DCONFIG_WIFI_CREDENTIALS=1 -Itests/lsm6dso_fakes "${zinc[@]}"
      -I"$cshell" -Iapps/wifi -I"$ccompat/include" -I"$ccompat/port" -I"$ccompat/internal" -I"$ctests")
wflags=("${flags[@]}" -Wno-unused-parameter -Wno-sign-compare)
cc "${wflags[@]}" "${winc[@]}" -Dmain=wifi_main -c apps/wifi/wifi_credentials_shell.c \
    -o "$out/wifi_credentials_shell.o"
cc "${wflags[@]}" "${winc[@]}" tests/wifi_cred.c "$ccompat/wifi_credentials.c" \
    "$ccompat/wifi_credentials_backend_settings.c" "$ccompat/zephyr_wifi_utils.c" \
    "$cshell/subcmd.c" "$out/wifi_credentials_shell.o" "${kvobjs[@]}" "${compat[@]}" "$link" \
    -o "$out/wifi_cred"
"$out/wifi_cred" >/dev/null
printf 'ok  wifi_cred\n'
