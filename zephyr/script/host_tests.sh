#!/usr/bin/env bash
# Build and run the host tests. They use fake Zephyr headers and need no board.
set -euo pipefail
cd "$(dirname "$0")/.."
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
app=app/usb-console
flags=(-std=c17 -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
link="-Wl,--gc-sections"

run() {
    local name=$1
    shift
    cc "$@" "$link" -o "$out/$name"
    "$out/$name" >/dev/null
    printf 'ok  %s\n' "$name"
}

# Application modules and the FIFO word parser.
lib=("$app/src/imu_raw_queue.c" "$app/src/imu_raw_packet.c" "$app/src/imu_raw_batch.c"
     "$app/src/imu_raw_status.c" "$app/src/imu_raw_time.c" "$app/src/imu_tick_clock.c"
     "$app/src/time_offset.c" "$app/drivers/lsm6dso/lsm6dso_fifo.c")
inc=(-I"$app/tests/fakes" -I"$app/src" -I"$app/drivers/lsm6dso")
for test in fifo_words imu_raw_packet imu_raw_batch imu_raw_queue imu_raw_status imu_raw_time \
            imu_tick_clock time_offset; do
    run "$test" "${flags[@]}" "${inc[@]}" "$app/tests/$test.c" "${lib[@]}"
done

# LSM6DSO FIFO driver pieces.
run fifo_hw "${flags[@]}" "${inc[@]}" "$app/tests/fifo_hw.c" "$app/drivers/lsm6dso/lsm6dso_fifo_hw.c"
run fifo_decoder "${flags[@]}" "${inc[@]}" "$app/tests/fifo_decoder.c" \
    "$app/drivers/lsm6dso/lsm6dso_fifo_decoder.c" "$app/drivers/lsm6dso/lsm6dso_fifo.c" \
    "$app/drivers/lsm6dso/lsm6dso_cal.c"
# Conversion to physical units and the stored calibration format.
run lsm6dso_cal "${flags[@]}" "${inc[@]}" "$app/tests/lsm6dso_cal.c" \
    "$app/drivers/lsm6dso/lsm6dso_cal.c" "$app/src/imu_cal_codec.c" -lm
# Gyro bias tracking at rest.
run lsm6dso_zero "${flags[@]}" "${inc[@]}" "$app/tests/lsm6dso_zero.c" \
    "$app/drivers/lsm6dso/lsm6dso_zero.c"
run stream_stop "${flags[@]}" "${inc[@]}" "$app/tests/stream_stop.c" "$app/drivers/lsm6dso/lsm6dso_fifo.c" \
    "$app/drivers/lsm6dso/lsm6dso_cal.c"

# RP2040 I2C DMA command parsing and NINA SPI framing.
run i2c_dma_commands "${flags[@]}" -I"$app/drivers/i2c_dw" "$app/tests/i2c_dma_commands.c" \
    "$app/drivers/i2c_dw/i2c_dw_rp2040_commands.c"
run nina_spi -std=gnu17 -Wall -I"$app/tests/nina_fakes" -I"$app/drivers/nina" "$app/tests/nina_spi.c"

for f in ../host/imu_host/*.py; do
    python3 -c 'import sys; compile(open(sys.argv[1]).read(), sys.argv[1], "exec")' "$f"
done
printf 'ok  host/imu_host (syntax)\n'
