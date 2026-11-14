#!/usr/bin/env bash
# Build and run the host tests for the OS-independent sources under nuttx/.
# Most sources and tests are unchanged copies of the Zephyr version's (nano_timeconv is the
# board's own); none needs the board.
set -euo pipefail
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
imu=apps/imu
lsm=drivers/lsm6dso
flags=(-std=c17 -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
link="-Wl,--gc-sections"
# imu_raw_packet.h includes "../drivers/lsm6dso/lsm6dso_fifo.h", which resolves from apps/.
inc=(-Itests/fakes -I"$imu" -I"$lsm" -Iapps)

# run NAME CC-ARGS...: build tests/NAME.c with the arguments, run it, stop on failure.
run() {
    local name=$1
    shift
    cc "${flags[@]}" "${inc[@]}" "tests/$name.c" "$@" "$link" -o "$out/$name"
    "$out/$name" >/dev/null
    printf 'ok  %s\n' "$name"
}

# Application modules and the FIFO word parser.
lib=("$imu/imu_raw_queue.c" "$imu/imu_raw_packet.c" "$imu/imu_raw_batch.c"
     "$imu/imu_raw_status.c" "$imu/imu_raw_time.c" "$imu/imu_tick_clock.c"
     "$imu/time_offset.c" "$lsm/lsm6dso_fifo.c")
for test in fifo_words imu_raw_packet imu_raw_batch imu_raw_queue imu_raw_status imu_raw_time \
            imu_tick_clock time_offset; do
    run "$test" "${lib[@]}"
done

# LSM6DSO FIFO driver pieces.
run fifo_hw "$lsm/lsm6dso_fifo_hw.c"
# The LSM6DSO driver (lsm6dso.c) and its FIFO stream (lsm6dso_stream.c, lsm6dso_fifo_irq.c,
# lsm6dso_compat.c, the decoder) on ST's real lsm6dso_reg.c, the Zephyr compatibility layer
# (I2C, devices, work queues, RTIO, gpio0, the default sensor iodev and decoder) with the fake
# kernel of tests/zephyr_fakes, and the board's devices, against a fake sensor behind a fake
# I2C master and a fake INT1.  Its own include path: the compatibility layer's headers and
# ST's, not the fakes of tests/fakes.  ST's file (fetched by setup.sh into nuttx/stmemsc) is
# built without warnings, as it is; the Zephyr driver's code casts its bus functions, leaves
# parameters unused and compares signed with unsigned, as in the Zephyr build.
compat=(drivers/compat/zephyr_work.c drivers/compat/zephyr_rtio.c drivers/compat/zephyr_mem_blocks.c
        drivers/compat/zephyr_rtio_workq.c tests/zephyr_fakes/zephyr_kernel_host.c)
if [[ -f stmemsc/lsm6dso_reg.c ]]; then
    cc -std=c17 -w -Istmemsc -c stmemsc/lsm6dso_reg.c -o "$out/lsm6dso_reg.o"
    cc "${flags[@]}" -Wno-unused-parameter -Wno-cast-function-type -Wno-sign-compare \
        -D_DEFAULT_SOURCE -D'DEBUGASSERT(x)=assert(x)' -Itests/lsm6dso_fakes -Itests/zephyr_fakes \
        -Idrivers/compat -Idrivers/include -Istmemsc -I"$lsm" -Iboards/arduino-nano-rp2040-connect/src \
        tests/lsm6dso_driver.c "$lsm/lsm6dso.c" "$lsm/stmemsc_i2c.c" "$lsm/stmemsc_mdelay.c" \
        "$lsm/lsm6dso_fifo_hw.c" "$lsm/lsm6dso_fifo.c" "$lsm/lsm6dso_cal.c" "$lsm/lsm6dso_zero.c" \
        "$lsm/lsm6dso_stream.c" "$lsm/lsm6dso_fifo_irq.c" "$lsm/lsm6dso_compat.c" \
        "$lsm/lsm6dso_fifo_decoder.c" drivers/compat/zephyr_i2c.c drivers/compat/zephyr_device.c \
        drivers/compat/zephyr_gpio.c drivers/compat/sensor_default_rtio.c "${compat[@]}" \
        boards/arduino-nano-rp2040-connect/src/nano_lsm6dso.c "$out/lsm6dso_reg.o" -lm "$link" \
        -o "$out/lsm6dso_driver"
    "$out/lsm6dso_driver" >/dev/null
    printf 'ok  lsm6dso_driver\n'
else
    printf 'skip lsm6dso_driver (no stmemsc/lsm6dso_reg.c: run script/setup.sh)\n'
fi
# The stream's state machine (lsm6dso_stream.c) with a fake FIFO and RTIO executor: an
# unchanged copy of the Zephyr build's test, which includes the unchanged copy of the source.
run stream_stop "$lsm/lsm6dso_fifo.c" "$lsm/lsm6dso_cal.c"
# The compatibility layer's RTIO (pool, completions, multishot, cancellation, work queue) and
# work items (submit, schedule, reschedule, cancel), on the fake kernel.
zinc=(-D_DEFAULT_SOURCE -Itests/zephyr_fakes -Idrivers/compat -Idrivers/include)
cc "${flags[@]}" "${zinc[@]}" tests/zephyr_rtio.c "${compat[@]}" "$link" -o "$out/zephyr_rtio"
"$out/zephyr_rtio" >/dev/null
printf 'ok  zephyr_rtio\n'
cc "${flags[@]}" "${zinc[@]}" tests/zephyr_work.c "${compat[@]}" "$link" -o "$out/zephyr_work"
"$out/zephyr_work" >/dev/null
printf 'ok  zephyr_work\n'
# The layer's pieces for the IMU service: K_MUTEX_DEFINE, K_THREAD_DEFINE and its start,
# k_uptime_get's tick rounding, k_sleep, byteorder, snprintk, hwinfo (fake chip ID), the
# shell output macros, the address conversions.
cc "${flags[@]}" "${zinc[@]}" -Itests/lsm6dso_fakes tests/zephyr_compat_misc.c \
    drivers/compat/zephyr_hwinfo.c "${compat[@]}" "$link" -o "$out/zephyr_compat_misc"
"$out/zephyr_compat_misc" >/dev/null
printf 'ok  zephyr_compat_misc\n'
# The minimal MQTT client (apps/imu/mqtt_min.c, the copied part of Zephyr's MQTT library; its
# text compares signed with unsigned, as in the Zephyr build) on a fake socket.  The same test
# also runs on Zephyr's own library when the Zephyr tree is there (tests/mqtt_ref: its log,
# mutex and socket names on the host), so the expected bytes, events and errors are Zephyr's.
cc "${flags[@]}" -Wno-sign-compare "${zinc[@]}" -Iapps/imu tests/mqtt_min.c apps/imu/mqtt_min.c \
    "${compat[@]}" "$link" -o "$out/mqtt_min"
"$out/mqtt_min" >/dev/null
printf 'ok  mqtt_min\n'
zmqtt=../zephyr/zephyr/subsys/net/lib/mqtt
if [[ -f $zmqtt/mqtt.c ]]; then
    zobjs=()
    for f in mqtt mqtt_rx mqtt_encoder mqtt_decoder mqtt_transport mqtt_transport_socket_tcp; do
        cc -std=c17 -w -DCONFIG_MQTT_KEEPALIVE=60 -DCONFIG_MQTT_LOG_LEVEL=0 -Itests/mqtt_ref \
            "${zinc[@]}" -Iapps/imu -include zephyr/net/socket.h -c "$zmqtt/$f.c" -o "$out/zmqtt_$f.o"
        zobjs+=("$out/zmqtt_$f.o")
    done
    cc "${flags[@]}" "${zinc[@]}" -Iapps/imu tests/mqtt_min.c "${zobjs[@]}" "${compat[@]}" "$link" \
        -o "$out/mqtt_zephyr"
    "$out/mqtt_zephyr" >/dev/null
    printf 'ok  mqtt_min (on Zephyr'\''s MQTT library)\n'
else
    printf 'skip mqtt_min on Zephyr'\''s MQTT library (no zephyr/zephyr: run west update)\n'
fi
run fifo_decoder "$lsm/lsm6dso_fifo_decoder.c" "$lsm/lsm6dso_fifo.c" "$lsm/lsm6dso_cal.c"
# Conversion to physical units and the stored calibration format.
run lsm6dso_cal "$lsm/lsm6dso_cal.c" "$imu/imu_cal_codec.c" -lm
# Gyro bias tracking at rest.
run lsm6dso_zero "$lsm/lsm6dso_zero.c"
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
# NINA SPI link: framing, answers, READY timeouts, reset sequence, CS order (fake board and clock).
run nina_spi -Idrivers/include -Idrivers/nina
# NINA sockets: the table, the socket commands and the data calls of nina_socket.c (fake
# nina_request() and clock).
run nina_sock -D_DEFAULT_SOURCE -Idrivers/include -Idrivers/nina drivers/nina/nina_sock.c
# NINA name resolution: hints, service, numeric addresses, the resolver commands (fake module).
run nina_dns -D_DEFAULT_SOURCE -Idrivers/include -Idrivers/nina drivers/nina/nina_dns.c
# Board: the I2C0 driver's SCL counts, speeds and IC_DATA_CMD words against the Zephyr build's.
run nano_i2c_dw -Iboards/arduino-nano-rp2040-connect/src
# Board: the command words of the I2C0 DMA reads (an unchanged copy of the Zephyr build's test).
run i2c_dma_commands -Iboards/arduino-nano-rp2040-connect/src \
    boards/arduino-nano-rp2040-connect/src/i2c_dw_rp2040_commands.c
# The i2c_dma command's output (fake driver side).
run i2c_dma_app -Idrivers/include -Iapps/i2c_dma -Iapps/subcmd apps/subcmd/subcmd.c
# The time synchronization (apps/imu/time_sync.c, the copy of the Zephyr build's) on the
# compatibility layer with a fake NINA socket, clock, interface and realtime clock: the NTP
# request and the checks of the reply, where t1 and t4 are read, the 1 ms polling of the
# reply through FIONREAD, the 60 s age, the forward-only realtime clock, the Wi-Fi events and
# the "ntp status" lines.
cc "${flags[@]}" -D_DEFAULT_SOURCE -Itests/time_sync_fakes -Itests/zephyr_fakes -Idrivers/compat \
    -Idrivers/include -I"$imu" -Iapps/subcmd tests/time_sync.c "$imu/time_offset.c" \
    apps/subcmd/subcmd.c "${compat[@]}" "$link" -o "$out/time_sync"
"$out/time_sync" >/dev/null
printf 'ok  time_sync\n'
# The MQTT transport (apps/imu/imu_raw_mqtt.c, the copy of the Zephyr build's) with the
# port's MQTT client and its socket layer on the compatibility layer's zsock_*() calls, on a
# fake socket, clock and interface: the start checks and topics, the thread, the socket calls
# in order (FIONREAD before every read that does not wait and before poll()), CONNECT, the
# CONNACK wait, the order of info, status and batch with header and body in one sendmsg(),
# the status and input intervals, dropped batches and the error paths.
cc "${flags[@]}" -Wno-sign-compare -D_DEFAULT_SOURCE -Itests/time_sync_fakes "${zinc[@]}" \
    "${inc[@]}" tests/imu_raw_mqtt.c apps/imu/mqtt_min.c "${lib[@]}" "${compat[@]}" "$link" \
    -o "$out/imu_raw_mqtt"
"$out/imu_raw_mqtt" >/dev/null
printf 'ok  imu_raw_mqtt\n'
# The stored Wi-Fi credentials (apps/wifi: the copies of Zephyr's wifi_credentials.c and its
# settings backend) and the "wifi cred" command (the copy of wifi_credentials_shell.c on
# subcmd), on a fake store of named items: the stored entries and names, the cache, the
# lengths and options, the messages and the log.  The copied text leaves parameters unused and
# compares signed with unsigned, as in the Zephyr build; the command's main() is wifi_main().
winc=(-D_GNU_SOURCE -DCONFIG_NANO_WIFI_CREDENTIALS=1 -Itests/lsm6dso_fakes "${zinc[@]}"
      -Iapps/subcmd -Iapps/wifi)
wflags=("${flags[@]}" -Wno-unused-parameter -Wno-sign-compare)
cc "${wflags[@]}" "${winc[@]}" -Dmain=wifi_main -c apps/wifi/wifi_credentials_shell.c \
    -o "$out/wifi_credentials_shell.o"
cc "${wflags[@]}" "${winc[@]}" tests/wifi_cred.c apps/wifi/wifi_credentials.c \
    apps/wifi/wifi_credentials_backend_settings.c drivers/compat/zephyr_wifi_utils.c \
    apps/subcmd/subcmd.c "$out/wifi_credentials_shell.o" "${compat[@]}" "$link" -o "$out/wifi_cred"
"$out/wifi_cred" >/dev/null
printf 'ok  wifi_cred\n'
