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
# The LSM6DSO driver (lsm6dso.c) on ST's lsm6dso_reg.c, the Zephyr compatibility layer and the
# board's devices, against a fake sensor behind a fake I2C master.  Its own include path: the
# compatibility layer's headers and ST's, not the fakes of tests/fakes.  ST's file (fetched by
# setup.sh into nuttx/stmemsc) is built without warnings, as it is; the Zephyr driver's code
# casts its bus functions and leaves parameters unused, as in the Zephyr build.
if [[ -f stmemsc/lsm6dso_reg.c ]]; then
    cc -std=c17 -w -Istmemsc -c stmemsc/lsm6dso_reg.c -o "$out/lsm6dso_reg.o"
    cc "${flags[@]}" -Wno-unused-parameter -Wno-cast-function-type -D_DEFAULT_SOURCE \
        -D'DEBUGASSERT(x)=assert(x)' -Itests/lsm6dso_fakes -Idrivers/compat -Istmemsc -I"$lsm" \
        -Iboards/arduino-nano-rp2040-connect/src tests/lsm6dso_driver.c "$lsm/lsm6dso.c" \
        "$lsm/stmemsc_i2c.c" "$lsm/stmemsc_mdelay.c" "$lsm/lsm6dso_fifo_hw.c" "$lsm/lsm6dso_fifo.c" \
        "$lsm/lsm6dso_cal.c" "$lsm/lsm6dso_zero.c" drivers/compat/zephyr_i2c.c \
        drivers/compat/zephyr_device.c boards/arduino-nano-rp2040-connect/src/nano_lsm6dso.c \
        "$out/lsm6dso_reg.o" -lm "$link" -o "$out/lsm6dso_driver"
    "$out/lsm6dso_driver" >/dev/null
    printf 'ok  lsm6dso_driver\n'
else
    printf 'skip lsm6dso_driver (no stmemsc/lsm6dso_reg.c: run script/setup.sh)\n'
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
# Board: the I2C0 driver's SCL counts, speeds and IC_DATA_CMD words against the Zephyr build's.
run nano_i2c_dw -Iboards/arduino-nano-rp2040-connect/src
# Board: the command words of the I2C0 DMA reads (an unchanged copy of the Zephyr build's test).
run i2c_dma_commands -Iboards/arduino-nano-rp2040-connect/src \
    boards/arduino-nano-rp2040-connect/src/i2c_dw_rp2040_commands.c
# The i2c_dma command's output (fake driver side).
run i2c_dma_app -Idrivers/include -Iapps/i2c_dma -Iapps/subcmd apps/subcmd/subcmd.c
