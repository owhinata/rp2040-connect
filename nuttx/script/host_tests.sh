#!/usr/bin/env bash
# Build and run the host tests for the OS-independent sources under nuttx/.
# The sources and tests are unchanged copies of the Zephyr version's; they need no board.
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
run fifo_decoder "$lsm/lsm6dso_fifo_decoder.c" "$lsm/lsm6dso_fifo.c" "$lsm/lsm6dso_cal.c"
# Conversion to physical units and the stored calibration format.
run lsm6dso_cal "$lsm/lsm6dso_cal.c" "$imu/imu_cal_codec.c" -lm
# Gyro bias tracking at rest.
run lsm6dso_zero "$lsm/lsm6dso_zero.c"
