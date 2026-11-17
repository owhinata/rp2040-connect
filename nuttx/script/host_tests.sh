#!/usr/bin/env bash
# Build and run the host tests for the OS-independent sources under nuttx/.
# The tests of the sources shared with the Zephyr build and of the compatibility layer's
# shared part run first (common/tests/run.sh).
# Most of the other sources and tests are unchanged copies of the Zephyr version's
# (nano_timeconv is the board's own); none needs the board.
set -euo pipefail
cd "$(dirname "$0")/.."
../common/tests/run.sh
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
imu=apps/imu
lsm=drivers/lsm6dso
cimu=../common/imu
clsm=../common/drivers/lsm6dso
# The Zephyr compatibility layer: its NuttX part (drivers/compat), its shared part
# (common/compat) and the fake kernel of the host tests (common/tests/zephyr_fakes), which
# comes first (its zephyr_os*.h and <zephyr/sys/__assert.h> in place of the NuttX ones).
ccompat=../common/compat
zfakes=../common/tests/zephyr_fakes
flags=(-std=c17 -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
link="-Wl,--gc-sections"
# The prelude that the NuttX build includes before each source of common/ (on the host: the
# Zephyr build's Kconfig values that those sources use).
prelude=drivers/compat/common_prelude.h
# The fake headers shared with the Zephyr build (common/tests/fakes) and the port's own
# (tests/fakes: nuttx/config.h).
inc=(-I../common/tests/fakes -Itests/fakes -I"$imu" -I"$cimu" -I"$lsm" -I"$clsm")

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
     "$cimu/time_offset.c" "$clsm/lsm6dso_fifo.c")

# The LSM6DSO driver (common/drivers/lsm6dso/lsm6dso.c with drivers/lsm6dso/lsm6dso_instance.h)
# and its FIFO stream (lsm6dso_stream.c, lsm6dso_fifo_irq.c,
# lsm6dso_compat.c, the decoder) on ST's real lsm6dso_reg.c, the Zephyr compatibility layer
# (I2C, devices, work queues, RTIO, gpio0, the default sensor iodev and decoder) with the fake
# kernel of common/tests/zephyr_fakes, and the board's devices, against a fake sensor behind a fake
# I2C master and a fake INT1.  Its own include path: the compatibility layer's headers and
# ST's, not the fakes of common/tests/fakes.  ST's file (fetched by setup.sh into nuttx/stmemsc) is
# built without warnings, as it is; the Zephyr driver's code casts its bus functions, leaves
# parameters unused and compares signed with unsigned, as in the Zephyr build.  The prelude
# follows the fake <nuttx/config.h>, as on NuttX (CONFIG_SENSOR_ASYNC_API from
# CONFIG_NANO_LSM6DSO_STREAM).
compat=("$ccompat/zephyr_work.c" "$ccompat/zephyr_rtio.c" "$ccompat/zephyr_mem_blocks.c"
        "$ccompat/zephyr_rtio_workq.c" "$zfakes/zephyr_kernel_host.c")
if [[ -f stmemsc/lsm6dso_reg.c ]]; then
    cc -std=c17 -w -Istmemsc -c stmemsc/lsm6dso_reg.c -o "$out/lsm6dso_reg.o"
    cc "${flags[@]}" -Wno-unused-parameter -Wno-cast-function-type -Wno-sign-compare \
        -D_DEFAULT_SOURCE -D'DEBUGASSERT(x)=assert(x)' -include tests/lsm6dso_fakes/nuttx/config.h \
        -include "$prelude" -Itests/lsm6dso_fakes \
        -I"$zfakes" \
        -Idrivers/compat -I"$ccompat" -Idrivers/include -Istmemsc -I"$lsm" -I"$clsm" \
        -Iboards/arduino-nano-rp2040-connect/src \
        tests/lsm6dso_driver.c "$clsm/lsm6dso.c" "$lsm/stmemsc_i2c.c" "$lsm/stmemsc_mdelay.c" \
        "$clsm/lsm6dso_fifo_hw.c" "$clsm/lsm6dso_fifo.c" "$clsm/lsm6dso_cal.c" \
        "$clsm/lsm6dso_zero.c" "$clsm/lsm6dso_stream.c" "$clsm/lsm6dso_fifo_irq.c" \
        "$clsm/lsm6dso_compat.c" \
        "$clsm/lsm6dso_fifo_decoder.c" drivers/compat/zephyr_i2c.c "$ccompat/zephyr_device.c" \
        "$ccompat/zephyr_gpio.c" "$ccompat/sensor_default_rtio.c" "${compat[@]}" \
        boards/arduino-nano-rp2040-connect/src/nano_lsm6dso.c "$out/lsm6dso_reg.o" -lm "$link" \
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
        -Idrivers/compat -I"$ccompat" -Idrivers/include -Istmemsc -I"$lsm" -I"$clsm" -I"$imu" -I"$cimu" \
        -Iboards/arduino-nano-rp2040-connect/src \
        tests/imu_cal.c "$cimu/imu_cal.c" "$cimu/imu_cal_codec.c" "$clsm/lsm6dso.c" \
        "$lsm/stmemsc_i2c.c" "$lsm/stmemsc_mdelay.c" \
        "$clsm/lsm6dso_fifo_hw.c" "$clsm/lsm6dso_fifo.c" "$clsm/lsm6dso_cal.c" \
        "$clsm/lsm6dso_zero.c" "$clsm/lsm6dso_stream.c" "$clsm/lsm6dso_fifo_irq.c" \
        "$clsm/lsm6dso_compat.c" \
        "$clsm/lsm6dso_fifo_decoder.c" drivers/compat/zephyr_i2c.c "$ccompat/zephyr_device.c" \
        "$ccompat/zephyr_gpio.c" "$ccompat/sensor_default_rtio.c" "${compat[@]}" \
        boards/arduino-nano-rp2040-connect/src/nano_lsm6dso.c "$out/lsm6dso_reg.o" -lm "$link" \
        -o "$out/imu_cal"
    "$out/imu_cal" >/dev/null
    printf 'ok  imu_cal\n'
    # The "sensor" command (common/drivers/sensor_shell/sensor_shell.c, shared with the Zephyr
    # build, the copy of Zephyr's sensor_shell_stream.c in common/compat, and apps/sensor: the
    # NuttX part) on the Zephyr shell's command handling (apps/subcmd/zshell.c) and
    # the driver with its fake sensor (lsm6dso_driver.c, included): the help, argument checks,
    # wildcards, device names, the output of get / attr_get / attr_set / stream / trig, printed
    # by the processing callback on the command's terminal.  Its main() is sensor_main().
    sinc=(-D_GNU_SOURCE -D'DEBUGASSERT(x)=assert(x)' -DSENSOR_SHELL_GET_WAIT_MS=1
          -include tests/lsm6dso_fakes/nuttx/config.h -include "$prelude"
          -Itests/lsm6dso_fakes -I"$zfakes" -Idrivers/compat -I"$ccompat" -Idrivers/include -Istmemsc
          -I"$lsm" -I"$clsm" -Iapps/sensor -Iapps/subcmd -Iboards/arduino-nano-rp2040-connect/src)
    sflags=("${flags[@]}" -Wno-unused-parameter -Wno-cast-function-type -Wno-sign-compare)
    cc "${sflags[@]}" "${sinc[@]}" -Dmain=sensor_main -c ../common/drivers/sensor_shell/sensor_shell.c \
        -o "$out/sensor_shell.o"
    cc "${sflags[@]}" "${sinc[@]}" tests/sensor_shell.c "$out/sensor_shell.o" \
        "$ccompat/sensor_shell_stream.c" apps/sensor/sensor_shell_nuttx.c apps/subcmd/zshell.c \
        apps/subcmd/subcmd.c "$clsm/lsm6dso.c" "$lsm/stmemsc_i2c.c" "$lsm/stmemsc_mdelay.c" \
        "$clsm/lsm6dso_fifo_hw.c" "$clsm/lsm6dso_fifo.c" "$clsm/lsm6dso_cal.c" \
        "$clsm/lsm6dso_zero.c" "$clsm/lsm6dso_stream.c" "$clsm/lsm6dso_fifo_irq.c" \
        "$clsm/lsm6dso_compat.c" \
        "$clsm/lsm6dso_fifo_decoder.c" drivers/compat/zephyr_i2c.c "$ccompat/zephyr_device.c" \
        "$ccompat/zephyr_gpio.c" "$ccompat/sensor_default_rtio.c" "${compat[@]}" \
        boards/arduino-nano-rp2040-connect/src/nano_lsm6dso.c "$out/lsm6dso_reg.o" -lm "$link" \
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
# <zephyr/net/socket.h> reads <nina_dns.h> and <nina_sockopt.h> of the NINA core (common/drivers/nina).
cnina=../common/drivers/nina
zinc=(-D_DEFAULT_SOURCE -I"$zfakes" -Idrivers/compat -I"$ccompat" -Idrivers/include -I"$cnina")
# The layer's pieces for the IMU service: K_MUTEX_DEFINE, K_THREAD_DEFINE and its start,
# k_uptime_get's tick rounding, k_sleep, byteorder, snprintk, hwinfo (fake chip ID), the
# shell output macros, the address conversions.
cc "${flags[@]}" "${zinc[@]}" -Itests/lsm6dso_fakes tests/zephyr_compat_misc.c \
    drivers/compat/zephyr_hwinfo.c "${compat[@]}" "$link" -o "$out/zephyr_compat_misc"
"$out/zephyr_compat_misc" >/dev/null
printf 'ok  zephyr_compat_misc\n'
# The layer's k_mutex on NuttX (drivers/compat/zephyr_mutex.c), built as for NuttX on a model
# of NuttX's recursive mutex (an all-zero one is held by thread 0): K_MUTEX_DEFINE, k_mutex_init,
# and a mutex left all zero (the MQTT client's before mqtt_client_init()), which is unlocked
# as in Zephyr.
cc "${flags[@]}" -D_DEFAULT_SOURCE -D__NuttX__ -Itests/zephyr_mutex_fakes -Idrivers/compat \
    -I"$ccompat" -Idrivers/include tests/zephyr_mutex.c drivers/compat/zephyr_mutex.c "$link" \
    -o "$out/zephyr_mutex"
"$out/zephyr_mutex" >/dev/null
printf 'ok  zephyr_mutex\n'
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
run nano_timeconv -Iboards/arduino-nano-rp2040-connect/src
# Board: the CPU load arithmetic of the idle loop (nano_idle.c).
run nano_cpuload -Iboards/arduino-nano-rp2040-connect/src
# RAM log: the retained ring, dmesg pieces, line prefix and rate limit.
run log_ram -Idrivers/log drivers/log/log_ring.c
# Board: the key-value area's range checks, page writes and contents check.
run nano_flash_kv -Iboards/arduino-nano-rp2040-connect/src
# Key-value store: key rules, and the hex conversions of the kv command.
# strnlen() is POSIX, not C17.
run kv_name -D_POSIX_C_SOURCE=200809L "$imu/kv_name.c"
run kv_hex -Iapps/kv apps/kv/kv_hex.c
# Subcommand dispatch with the Zephyr shell's output (kv, nina, ...).
run subcmd -Iapps/subcmd apps/subcmd/subcmd.c
# The registration of "imu" and "ntp" (the SHELL_*() text cut out of common/imu/imu_stream.c
# and time_sync.c, built with Z_SHELL_CMD_NAMED_MAIN on apps/subcmd/zshell.c): the same exit
# status, handler calls and output as the same sets on subcmd_main(), and where they differ.
sed -n '/^SHELL_STATIC_SUBCMD_SET_CREATE(imu_commands,/,/^SHELL_CMD_REGISTER(imu,/p' \
    "$cimu/imu_stream.c" >"$out/imu_shell.inc"
sed -n '/^SHELL_STATIC_SUBCMD_SET_CREATE(ntp_commands,/,/^SHELL_CMD_REGISTER(ntp,/p' \
    "$cimu/time_sync.c" >"$out/ntp_shell.inc"
grep -q '^SHELL_CMD_REGISTER(imu,' "$out/imu_shell.inc"
grep -q '^SHELL_CMD_REGISTER(ntp,' "$out/ntp_shell.inc"
run shell_register -D_DEFAULT_SOURCE -DZ_SHELL_CMD_NAMED_MAIN -DIMU_SHELL_INC="\"$out/imu_shell.inc\"" \
    -DNTP_SHELL_INC="\"$out/ntp_shell.inc\"" -Idrivers/compat -I"$ccompat" -Iapps/subcmd apps/subcmd/zshell.c \
    apps/subcmd/subcmd.c
# Board: the I2C0 driver's SCL counts, speeds and IC_DATA_CMD words against the Zephyr build's.
run nano_i2c_dw -Iboards/arduino-nano-rp2040-connect/src
# The i2c_dma command's output (fake driver side).
run i2c_dma_app -Idrivers/include -Iapps/i2c_dma -Iapps/subcmd apps/subcmd/subcmd.c
# The time synchronization (common/imu/time_sync.c, shared with the Zephyr build) on the
# compatibility layer with a fake NINA socket, clock (the TIMER's registers), interface and
# realtime clock: the NTP request and the checks of the reply, where t1 and t4 are read, the
# 1 ms polling of the reply through FIONREAD, the socket marked as polled by its callers only,
# a receive that waits (FIONREAD every 20 ms), the 60 s age, the forward-only realtime clock,
# the Wi-Fi events and the "ntp status" lines, through z_shell_main_ntp() (the file is built
# with Z_SHELL_CMD_NAMED_MAIN, as in the NuttX build) on the Zephyr shell's command handling
# (apps/subcmd/zshell.c).  The prelude follows the fake <nuttx/config.h>, as on NuttX.
cc "${flags[@]}" -D_DEFAULT_SOURCE -DZ_SHELL_CMD_NAMED_MAIN -include tests/time_sync_fakes/nuttx/config.h \
    -include "$prelude" -Itests/time_sync_fakes -I"$zfakes" -Idrivers/compat -I"$ccompat" \
    -Idrivers/include -I"$cnina" -I"$imu" -I"$cimu" -Iapps/subcmd tests/time_sync.c "$cimu/time_offset.c" \
    apps/subcmd/zshell.c apps/subcmd/subcmd.c "${compat[@]}" "$link" -o "$out/time_sync"
"$out/time_sync" >/dev/null
printf 'ok  time_sync\n'
# The MQTT transport (common/imu/imu_raw_mqtt.c, shared with the Zephyr build) with the
# port's MQTT client and its socket layer on the compatibility layer's zsock_*() calls, on a
# fake socket, clock and interface: the start checks and topics, the thread, the socket calls
# in order (the socket marked as polled by its callers only, FIONREAD before every read that
# does not wait, poll() as the Zephyr build's: one NINA_SO_POLL at once and every 20 ms),
# CONNECT, the CONNACK wait, the order of info, status and batch with header and body in one
# sendmsg(), the status and input intervals, dropped batches and the error paths.
cc "${flags[@]}" -Wno-sign-compare -D_DEFAULT_SOURCE -include "$prelude" -Itests/time_sync_fakes \
    "${zinc[@]}" "${inc[@]}" tests/imu_raw_mqtt.c "$cimu/mqtt_min.c" "${lib[@]}" "${compat[@]}" "$link" \
    -o "$out/imu_raw_mqtt"
"$out/imu_raw_mqtt" >/dev/null
printf 'ok  imu_raw_mqtt\n'
# The stored Wi-Fi credentials (common/compat: the copies of Zephyr's wifi_credentials.c and its
# settings backend) and the "wifi cred" command (apps/wifi: the copy of wifi_credentials_shell.c
# on subcmd), on a fake store of named items: the stored entries and names, the cache, the
# lengths and options, the messages and the log.  The copied text leaves parameters unused and
# compares signed with unsigned, as in the Zephyr build; the command's main() is wifi_main().
winc=(-D_GNU_SOURCE -DCONFIG_NANO_WIFI_CREDENTIALS=1 -Itests/lsm6dso_fakes "${zinc[@]}"
      -Iapps/subcmd -Iapps/wifi -I"$ccompat")
wflags=("${flags[@]}" -Wno-unused-parameter -Wno-sign-compare)
cc "${wflags[@]}" "${winc[@]}" -Dmain=wifi_main -c apps/wifi/wifi_credentials_shell.c \
    -o "$out/wifi_credentials_shell.o"
cc "${wflags[@]}" "${winc[@]}" tests/wifi_cred.c "$ccompat/wifi_credentials.c" \
    "$ccompat/wifi_credentials_backend_settings.c" "$ccompat/zephyr_wifi_utils.c" \
    apps/subcmd/subcmd.c "$out/wifi_credentials_shell.o" "${compat[@]}" "$link" -o "$out/wifi_cred"
"$out/wifi_cred" >/dev/null
printf 'ok  wifi_cred\n'
