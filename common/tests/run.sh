#!/usr/bin/env bash
# Build and run the host tests of the sources shared by the Zephyr and NuttX builds (common/)
# and of the OS-independent part of the Zephyr compatibility layer (common/compat).  They use
# the fake headers of tests/fakes or the fake kernel of tests/zephyr_fakes and need no board
# and no Zephyr tree.  zephyr/script/host_tests.sh and nuttx/script/host_tests.sh run this
# before their own tests.
set -euo pipefail
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
imu=imu
lsm=drivers/lsm6dso
flags=(-std=c17 -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
link="-Wl,--gc-sections"
inc=(-Itests/fakes -I"$imu" -I"$lsm")

run() {
    local name=$1
    shift
    cc "$@" "$link" -o "$out/$name"
    "$out/$name" >/dev/null
    printf 'ok  %s\n' "$name"
}

# Application modules and the FIFO word parser.
lib=("$imu/imu_raw_queue.c" "$imu/imu_raw_packet.c" "$imu/imu_raw_batch.c"
     "$imu/imu_raw_status.c" "$imu/imu_raw_time.c" "$imu/imu_tick_clock.c"
     "$imu/time_offset.c" "$lsm/lsm6dso_fifo.c")
for test in fifo_words imu_raw_packet imu_raw_batch imu_raw_queue imu_raw_status imu_raw_time \
            imu_tick_clock time_offset; do
    run "$test" "${flags[@]}" "${inc[@]}" "tests/$test.c" "${lib[@]}"
done

# LSM6DSO FIFO driver pieces.
run fifo_hw "${flags[@]}" "${inc[@]}" tests/fifo_hw.c "$lsm/lsm6dso_fifo_hw.c"
run fifo_decoder "${flags[@]}" "${inc[@]}" tests/fifo_decoder.c \
    "$lsm/lsm6dso_fifo_decoder.c" "$lsm/lsm6dso_fifo.c" "$lsm/lsm6dso_cal.c"
# Conversion to physical units and the stored calibration format.
run lsm6dso_cal "${flags[@]}" "${inc[@]}" tests/lsm6dso_cal.c \
    "$lsm/lsm6dso_cal.c" "$imu/imu_cal_codec.c" -lm
# Gyro bias tracking at rest.
run lsm6dso_zero "${flags[@]}" "${inc[@]}" tests/lsm6dso_zero.c "$lsm/lsm6dso_zero.c"
# The stream's state machine (lsm6dso_stream.c, which the test includes) with a fake FIFO and
# RTIO executor.
run stream_stop "${flags[@]}" "${inc[@]}" tests/stream_stop.c \
    "$lsm/lsm6dso_fifo.c" "$lsm/lsm6dso_cal.c"

# RP2040 I2C DMA command parsing.
run i2c_dma_commands "${flags[@]}" -Idrivers/i2c_dw tests/i2c_dma_commands.c \
    drivers/i2c_dw/i2c_dw_rp2040_commands.c

# The compatibility layer on the fake kernel (tests/zephyr_fakes: the OS part, zephyr_os*.h,
# and its functions): RTIO (pool, completions, multishot, cancellation, work queue), work items
# (submit, schedule, reschedule, cancel), and the minimal MQTT client (imu/mqtt_min.c, the
# copied part of Zephyr's MQTT library; its text compares signed with unsigned, as in the
# Zephyr build) on a fake socket.  nuttx/script/host_tests.sh also runs the MQTT test on
# Zephyr's own library when the Zephyr tree is there.
compat=(compat/zephyr_work.c compat/zephyr_rtio.c compat/zephyr_mem_blocks.c
        compat/zephyr_rtio_workq.c tests/zephyr_fakes/zephyr_kernel_host.c)
zinc=(-D_DEFAULT_SOURCE -Itests/zephyr_fakes -Icompat)
run zephyr_rtio "${flags[@]}" "${zinc[@]}" tests/zephyr_rtio.c "${compat[@]}"
run zephyr_work "${flags[@]}" "${zinc[@]}" tests/zephyr_work.c "${compat[@]}"
run mqtt_min "${flags[@]}" -Wno-sign-compare "${zinc[@]}" -I"$imu" tests/mqtt_min.c \
    "$imu/mqtt_min.c" "${compat[@]}"
