#!/usr/bin/env bash
# Build and run the host tests of the sources shared by the Zephyr and NuttX builds (common/)
# and of the OS-independent parts that the builds other than Zephyr's share: the Zephyr
# compatibility layer (common/compat), the shell's subcommand dispatch (common/shell) and the
# RAM log's ring (common/log).  They use the fake headers of tests/fakes, the fake kernel of
# tests/zephyr_fakes or (the NINA core) fakes of its boundary defined by the tests, and need no
# board and no Zephyr tree (the checks that the copies of Zephyr's files are unchanged and that
# the I2C core matches the Zephyr build's patched driver run when the tree is there).
# zephyr/script/host_tests.sh and nuttx/script/host_tests.sh run this
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

# The I2C core's definitions (drivers/i2c_dw/i2c_dw.h): SCL counts, speeds and the
# IC_DATA_CMD words of whole transfers against the Zephyr build's, and the NuttX build's
# message flags.
run i2c_dw "${flags[@]}" -Idrivers/i2c_dw tests/i2c_dw.c

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

# The NINA-W102 core (drivers/nina) on fakes of its boundary (nina_bus.h: the SPI bus and the
# module's lines; nina_os.h: locks, requests, sleeps, clock), defined by the tests.
nina=drivers/nina
# The core has no OS of its own: each of its files (the top of drivers/nina; its OS bindings are
# in the subdirectories) compiles with the C library and POSIX socket types of the host and its
# own headers only.
for f in "$nina"/*.c "$nina"/*.h; do
    printf '#include "%s"\n' "$f" | cc "${flags[@]}" -D_DEFAULT_SOURCE -I. -I"$nina" -fsyntax-only \
        -x c -
done
printf 'ok  nina_core_independent (%d files)\n' "$(ls "$nina"/*.c "$nina"/*.h | wc -l)"
# SPI link (nina_spi.c, which the test includes): framing, answers, READY timeouts, reset
# sequence, CS order (fake board and clock); each case with READY sampled (the OS's
# nina_os_ready_start() fails) and with the OS's waits for READY (fake waits).
run nina_spi "${flags[@]}" -I"$nina" tests/nina_spi.c
# The waits of the socket loops (nina_wait.c): the poll intervals (before the start, or after
# nina_os_event_start() fails), or the IRQ line's edges with their timeout, deadline and
# back-off (fake clock and line).
run nina_wait "${flags[@]}" -I"$nina" tests/nina_wait.c "$nina/nina_wait.c"
# Sockets: the table, the socket commands and the data calls of the Zephyr build's
# nina_socket.c (fake nina_request() and clock).
run nina_sock "${flags[@]}" -D_DEFAULT_SOURCE -I"$nina" tests/nina_sock.c "$nina/nina_sock.c"
# The Wi-Fi operations (nina_wifi.c, which the test includes): connect, its thread and its ends,
# disconnect, the monitor, the scans of 3.0.2 and of the patched firmware, status (fake module,
# kernel services and binding).
run nina_wifi "${flags[@]}" -I"$nina" tests/nina_wifi.c
# Name resolution: hints, service, numeric addresses, the resolver commands (fake module).
run nina_dns "${flags[@]}" -D_DEFAULT_SOURCE -I"$nina" tests/nina_dns.c "$nina/nina_dns.c"
# The same with the results on the OS's heap and the registration (NINA_OS_DNS_OFFLOAD, as the
# Zephyr build compiles it).
run nina_dns_offload "${flags[@]}" -D_DEFAULT_SOURCE -DNINA_OS_DNS_OFFLOAD -I"$nina" tests/nina_dns.c \
    "$nina/nina_dns.c"

# The NINA passthrough mode (nina_passthrough.c, which the test includes) on fakes of its two
# boundaries (nina_passthrough_bus.h: the scratch registers, the module's lines, the UART, the LED;
# nina_passthrough_os.h: the clock, the host's writes, the log): the request against the fatal
# error record of the same registers (log/fault_record.c), DTR and RTS on the module's lines, the
# rate and the way out, the buffer from the UART's interrupt (also on two threads), the start, the
# data both ways with their counts, the tick and the way out's sequence.
run nina_passthrough "${flags[@]}" -I"$nina" -Ilog tests/nina_passthrough.c log/fault_record.c

# The shell's subcommand dispatch with the Zephyr shell's output (shell/subcmd.c: the kv, nina,
# ... tables of the NuttX build).
run subcmd "${flags[@]}" -Ishell tests/subcmd.c shell/subcmd.c
# The same with the copy of Zephyr's fnmatch() (compat/subsys/portability, which the FreeRTOS
# build links: newlib has none) instead of the host's.
run subcmd_fnmatch "${flags[@]}" -Ishell -Icompat tests/subcmd.c shell/subcmd.c \
    compat/subsys/portability/posix/c_lib_ext/fnmatch.c
# The copies of Zephyr's getopt (compat/lib/utils/getopt, for the FreeRTOS build): short and long
# options, permutation, errors, and two states parsing two command lines interleaved, as two
# shell sessions do.
run sys_getopt "${flags[@]}" -D_DEFAULT_SOURCE -DCONFIG_GETOPT_LONG=1 -Itests/kv_fakes \
    -Itests/zephyr_fakes -Icompat -Ilog -Icompat/lib/utils/getopt tests/sys_getopt.c \
    compat/lib/utils/getopt/getopt.c compat/lib/utils/getopt/getopt_long.c
# The RAM log's ring (log/log_ring.c): the retained ring, dmesg pieces, line prefix and rate
# limit.
run log_ring "${flags[@]}" -Ilog tests/log_ring.c log/log_ring.c
# The CPU load arithmetic of the idle loop (compat/nano_cpuload.h).
run nano_cpuload "${flags[@]}" -Icompat tests/nano_cpuload.c
# The CPU column of "ps" (FreeRTOS) and "kernel ps" (Zephyr): each task's share since the
# previous list (shell/ps_interval.h).
run ps_interval "${flags[@]}" -Ishell tests/ps_interval.c
# The record of a fatal error that the FreeRTOS build keeps across its restart
# (log/fault_record.c): the check, the watchdog scratch copy and the lines.
run fault_record "${flags[@]}" -Ilog tests/fault_record.c log/fault_record.c

# The hexadecimal conversions of the "kv" command (compat/lib/utils/hex.c, the copy of Zephyr's
# lib/utils/hex.c).
run hex "${flags[@]}" -Icompat tests/hex.c compat/lib/utils/hex.c
# The key-value area's helpers (compat/nano_flash_kv.h): range checks, page writes, the check of
# what the area holds (Zephyr's NVS, the NuttX build's earlier format, erased, other).
run nano_flash_kv "${flags[@]}" -Icompat tests/nano_flash_kv.c
# The key-value store (imu/kv_store.c, the Zephyr build's) on the copies of Zephyr's settings and
# NVS (compat/subsys, built as they are, without warnings, with the Zephyr build's Kconfig values
# of compat/zephyr_kv_config.h), the flash device and map of the area (compat/zephyr_flash_kv.c)
# and "kv wipe" (compat/kv_store_wipe.c), on a fake flash (tests/kv_flash_fake.c) and the fake
# kernel: the bytes on the flash, reads, writes, limits, garbage collection, foreign data, the
# device's refusals, a power loss at every flash operation of a sequence.  Its log, shell
# backend and app_log_os.h are tests/kv_fakes and the test's own.
kvinc=(-D_GNU_SOURCE -DZ_SHELL_CMD_NAMED_MAIN -Itests/kv_fakes -Itests/zephyr_fakes -Icompat -Ilog
       -Iimu -Icompat/subsys/settings/include -Icompat/subsys/settings/src
       -Icompat/subsys/kvss/nvs -Itests)
kvobjs=()
for f in compat/subsys/kvss/nvs/nvs.c compat/subsys/settings/src/settings.c \
         compat/subsys/settings/src/settings_store.c compat/subsys/settings/src/settings_init.c \
         compat/subsys/settings/src/settings_nvs.c compat/subsys/crc/crc8_sw.c \
         compat/lib/utils/hex.c; do
    o="$out/kv_$(basename "$f" .c).o"
    cc -std=gnu17 -w -ffunction-sections -fdata-sections -include compat/zephyr_kv_config.h \
        "${kvinc[@]}" -c "$f" -o "$o"
    kvobjs+=("$o")
done
run kv_settings "${flags[@]}" -include compat/zephyr_kv_config.h "${kvinc[@]}" tests/kv_settings.c \
    tests/kv_flash_fake.c compat/zephyr_flash_kv.c compat/kv_store_wipe.c imu/kv_store.c \
    tests/zephyr_fakes/zephyr_kernel_host.c "${kvobjs[@]}"

# The copies of Zephyr's files (v4.5.0) are unchanged: compared with the Zephyr tree when it is
# there (zephyr/zephyr, "west update").  wifi_credentials.c and
# wifi_credentials_backend_settings.c are copies with changes (see their headers).
ztree=../zephyr/zephyr
if [[ -f $ztree/VERSION ]]; then
    grep -q '^VERSION_MAJOR = 4$' "$ztree/VERSION"
    grep -q '^VERSION_MINOR = 5$' "$ztree/VERSION"
    grep -q '^PATCHLEVEL = 0$' "$ztree/VERSION"
    for pair in \
        compat/subsys/kvss/nvs/nvs.c:subsys/kvss/nvs/nvs.c \
        compat/subsys/kvss/nvs/nvs_priv.h:subsys/kvss/nvs/nvs_priv.h \
        compat/subsys/settings/src/settings.c:subsys/settings/src/settings.c \
        compat/subsys/settings/src/settings_store.c:subsys/settings/src/settings_store.c \
        compat/subsys/settings/src/settings_init.c:subsys/settings/src/settings_init.c \
        compat/subsys/settings/src/settings_nvs.c:subsys/settings/src/settings_nvs.c \
        compat/subsys/settings/src/settings_priv.h:subsys/settings/src/settings_priv.h \
        compat/subsys/settings/include/settings/settings_nvs.h:subsys/settings/include/settings/settings_nvs.h \
        compat/subsys/settings/include/settings/settings_file.h:subsys/settings/include/settings/settings_file.h \
        compat/subsys/crc/crc8_sw.c:subsys/crc/crc8_sw.c \
        compat/lib/utils/hex.c:lib/utils/hex.c \
        compat/zephyr/kvss/nvs.h:include/zephyr/kvss/nvs.h \
        compat/zephyr/settings/settings.h:include/zephyr/settings/settings.h \
        compat/zephyr/sys/slist.h:include/zephyr/sys/slist.h \
        compat/zephyr/sys/list_gen.h:include/zephyr/sys/list_gen.h \
        compat/subsys/portability/posix/c_lib_ext/fnmatch.c:subsys/portability/posix/c_lib_ext/fnmatch.c \
        compat/zephyr/posix/fnmatch.h:include/zephyr/posix/fnmatch.h \
        compat/lib/utils/getopt/getopt.c:lib/utils/getopt/getopt.c \
        compat/lib/utils/getopt/getopt_long.c:lib/utils/getopt/getopt_long.c \
        compat/lib/utils/getopt/getopt_common.h:lib/utils/getopt/getopt_common.h \
        compat/lib/utils/getopt/README:lib/utils/getopt/README \
        compat/zephyr/sys/sys_getopt.h:include/zephyr/sys/sys_getopt.h \
        compat/subsys/net/lib/wifi_credentials/wifi_credentials_shell.c:subsys/net/lib/wifi_credentials/wifi_credentials_shell.c; do
        cmp "${pair%%:*}" "$ztree/${pair#*:}"
    done
    printf 'ok  zephyr_copies (23 files, Zephyr v4.5.0)\n'

    # The I2C core (drivers/i2c_dw) against the Zephyr build's driver, Zephyr's with the local
    # patch 0003 (made from the tree's HEAD and the patch): the definitions and values both
    # take, and the pairs of functions, whose differences must be those of
    # tests/i2c_dw_sync.expected (tests/i2c_dw_sync.py; --update rewrites it after an intended
    # change).  The Kconfig values and devicetree properties are checked too when the Zephyr
    # build is there.
    sync=(tests/i2c_dw_sync.py "$ztree" ../zephyr/manifest/zephyr/patches/zephyr/0003-i2c-dw-rp2040-fifo-and-dma.patch)
    zbuild=../zephyr/build/usb-console
    if [[ -f $zbuild/zephyr/.config && -f $zbuild/zephyr/zephyr.dts ]]; then
        sync+=(--build "$zbuild")
    fi
    python3 "${sync[@]}" > "$out/i2c_dw_sync.log"
    printf 'ok  i2c_dw_sync (%d checks; %s)\n' "$(grep -c '^ok' "$out/i2c_dw_sync.log")" \
        "$(grep -q '^skip' "$out/i2c_dw_sync.log" && printf 'no Zephyr build' || printf 'with the Zephyr build')"
else
    printf 'skip zephyr_copies (no zephyr/zephyr: run west update)\n'
    printf 'skip i2c_dw_sync (no zephyr/zephyr: run west update)\n'
fi
