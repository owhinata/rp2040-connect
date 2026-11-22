#!/usr/bin/env bash
# Build and run the host tests of the sources shared by the Zephyr and NuttX builds (common/)
# and of the OS-independent parts that the builds other than Zephyr's share: the Zephyr
# compatibility layer (common/compat), the shell's subcommand dispatch (common/shell) and the
# RAM log (common/compat/log).  They use the fake headers of tests/fakes, the fake kernel of
# tests/zephyr_fakes or (the NINA core) fakes of its boundary defined by the tests, and need no
# board and no Zephyr tree (the checks that the copies of Zephyr's files are unchanged, that
# the I2C core matches the Zephyr build's patched driver and that the sensor shell is the file
# of the patched tree run when the tree is there).  tests/app_threads.py checks the
# application's table of threads against the sources and the build files.
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
# The LSM6DSO driver's headers: its public ones (include/), its core's (internal/, the boundary
# in port/) and its Zephyr binding's (port/zephyr), which the NuttX and FreeRTOS builds compile too.
inc=(-Itests/fakes -I"$imu" -I"$lsm/include" -I"$lsm/internal" -I"$lsm/port" -I"$lsm/port/zephyr")

run() {
    local name=$1
    shift
    cc "$@" "$link" -o "$out/$name"
    "$out/$name" >/dev/null
    printf 'ok  %s\n' "$name"
}

# Application modules, the FIFO word parser (lsm6dso_fifo.c) and the stream's buffers
# (port/zephyr/lsm6dso_decoder.c).
lib=("$imu/imu_raw_queue.c" "$imu/imu_raw_packet.c" "$imu/imu_raw_batch.c"
     "$imu/imu_raw_status.c" "$imu/imu_raw_time.c" "$imu/imu_tick_clock.c"
     "$imu/time_offset.c" "$lsm/lsm6dso_fifo.c" "$lsm/port/zephyr/lsm6dso_decoder.c")
for test in fifo_words imu_raw_packet imu_raw_batch imu_raw_queue imu_raw_status imu_raw_time \
            imu_tick_clock time_offset; do
    run "$test" "${flags[@]}" "${inc[@]}" "tests/$test.c" "${lib[@]}"
done
# The table of the threads that the builds other than Zephyr's start (imu/app_init.c) names each
# K_THREAD_DEFINE() of the sources once, under the option of its module that each build sets
# exactly when it compiles the source (tests/app_threads.py).
python3 tests/app_threads.py > "$out/app_threads.log" || { cat "$out/app_threads.log"; exit 1; }
printf 'ok  app_threads (%s)\n' "$(sed -n 's/^ok  FreeRTOS starts \([0-9]*\), NuttX \([0-9]*\),.*/FreeRTOS \1, NuttX \2/p' \
    "$out/app_threads.log")"

# The LSM6DSO driver's decoder of the stream's buffers (port/zephyr/lsm6dso_decoder.c).  The
# chip's registers (lsm6dso.c) are tested with the driver in nuttx/tests/lsm6dso_driver.c.
run fifo_decoder "${flags[@]}" "${inc[@]}" tests/fifo_decoder.c \
    "$lsm/port/zephyr/lsm6dso_decoder.c" "$lsm/lsm6dso_fifo.c" "$lsm/lsm6dso_cal.c"
# Conversion to physical units and the stored calibration format (lsm6dso_cal.c).
run lsm6dso_cal "${flags[@]}" "${inc[@]}" tests/lsm6dso_cal.c \
    "$lsm/lsm6dso_cal.c" "$imu/imu_cal_codec.c" -lm
# Gyro bias tracking at rest (the lsm6dso_zero_*() functions of lsm6dso_cal.c).
run lsm6dso_zero "${flags[@]}" "${inc[@]}" tests/lsm6dso_zero.c "$lsm/lsm6dso_cal.c"
# The FIFO stream's state machine (lsm6dso_stream.c) on the host's part of its boundaries
# (tests/lsm6dso_os_host: the OS's and the user's) with a fake core (the FIFO calls of
# lsm6dso_core.h, defined by the test) and the fake ST header of tests/fakes.
run lsm6dso_stream "${flags[@]}" -Itests/lsm6dso_os_host -I"$lsm/include" -I"$lsm/internal" \
    -I"$lsm/port" -Itests/fakes tests/lsm6dso_stream.c "$lsm/lsm6dso_stream.c" \
    "$lsm/lsm6dso_fifo.c" "$lsm/lsm6dso_cal.c"

# The LSM6DSO's core has no OS of its own: each of its files (the .c files at the top of
# drivers/lsm6dso, include/ but lsm6dso.h, the extended API on Zephyr's sensor API, internal/ and
# the boundaries of port/; the OS bindings are in port/'s subdirectories) compiles with the host's
# C library, its own headers, ST's register driver lsm6dso_reg.h (OS-independent; the copy that a
# build fetched, alone in a directory) and the host's parts of the boundaries
# (tests/lsm6dso_os_host: the OS's, lsm6dso_os_impl.h, and the stream's user's,
# lsm6dso_stream_req_impl.h).
st=
for d in ../zephyr/modules/hal/st/sensor/stmemsc/lsm6dso_STdC/driver ../nuttx/stmemsc \
         ../freertos/stmemsc; do
    if [[ -f $d/lsm6dso_reg.h ]]; then
        st=$d
        break
    fi
done
if [[ -n $st ]]; then
    mkdir -p "$out/st"
    cp "$st/lsm6dso_reg.h" "$out/st/"
    lsm_core=("$lsm"/*.c "$lsm"/include/lsm6dso_*.h "$lsm"/internal/*.h "$lsm"/port/*.h)
    for f in "${lsm_core[@]}"; do
        printf '#include "%s"\n' "$f" | cc "${flags[@]}" -I. -I"$lsm/include" -I"$lsm/internal" \
            -I"$lsm/port" -Itests/lsm6dso_os_host -I"$out/st" -fsyntax-only -x c -
    done
    printf 'ok  lsm6dso_core_independent (%d files)\n' "${#lsm_core[@]}"
else
    printf 'skip lsm6dso_core_independent (no lsm6dso_reg.h: run a setup.sh)\n'
fi
# The headers for the driver's users (include/) compile with that directory alone; lsm6dso.h
# (<zephyr/drivers/sensor/lsm6dso.h>) with the fakes of the sensor API and RTIO it reads
# (tests/fakes).
lsm_public=("$lsm"/include/*.h)
for f in "${lsm_public[@]}"; do
    fakes=()
    [[ ${f##*/} == lsm6dso.h ]] && fakes=(-Itests/fakes)
    printf '#include "%s"\n' "${f##*/}" | cc "${flags[@]}" -I"$lsm/include" "${fakes[@]}" \
        -fsyntax-only -x c -
done
printf 'ok  lsm6dso_public_headers (%d files)\n' "${#lsm_public[@]}"

# The I2C core's definitions (drivers/i2c_dw/internal/i2c_dw.h): SCL counts, speeds and the
# IC_DATA_CMD words of whole transfers against the Zephyr build's, and the NuttX build's
# message flags.
run i2c_dw "${flags[@]}" -Idrivers/i2c_dw/internal tests/i2c_dw.c
# The headers for the I2C driver's users (drivers/i2c_dw/include) compile with that directory
# alone: none of them reads internal/ or port/.
i2c_dw_public=(drivers/i2c_dw/include/*.h)
for f in "${i2c_dw_public[@]}"; do
    printf '#include "%s"\n' "${f##*/}" | cc "${flags[@]}" -Idrivers/i2c_dw/include \
        -fsyntax-only -x c -
done
printf 'ok  i2c_dw_public_headers (%d files)\n' "${#i2c_dw_public[@]}"

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
nina_inc=(-I"$nina/include" -I"$nina/internal" -I"$nina/port")
# The core has no OS of its own: each of its files (the top of drivers/nina, include/,
# internal/ and the boundary headers of port/; the OS bindings are in port/'s subdirectories)
# compiles with the C library and POSIX socket types of the host and its own headers only.
nina_core=("$nina"/*.c "$nina"/include/*.h "$nina"/internal/*.c "$nina"/internal/*.h
           "$nina"/port/*.h)
for f in "${nina_core[@]}"; do
    printf '#include "%s"\n' "$f" | cc "${flags[@]}" -D_DEFAULT_SOURCE -I. "${nina_inc[@]}" \
        -fsyntax-only -x c -
done
printf 'ok  nina_core_independent (%d files)\n' "${#nina_core[@]}"
# The headers for the driver's users (include/) compile with that directory alone: none of them
# reads internal/ or port/.
nina_public=("$nina"/include/*.h)
for f in "${nina_public[@]}"; do
    printf '#include "%s"\n' "${f##*/}" | cc "${flags[@]}" -D_DEFAULT_SOURCE -I"$nina/include" \
        -fsyntax-only -x c -
done
printf 'ok  nina_public_headers (%d files)\n' "${#nina_public[@]}"
# SPI link (nina_proto.c, which the test includes): framing, answers, READY timeouts, reset
# sequence, CS order (fake board and clock); each case with READY sampled (the OS's
# nina_os_ready_start() fails) and with the OS's waits for READY (fake waits).
run nina_proto "${flags[@]}" "${nina_inc[@]}" tests/nina_proto.c
# The waits of the socket loops (nina_wait.c): the poll intervals (before the start, or after
# nina_os_event_start() fails), or the IRQ line's edges with their timeout, deadline and
# back-off (fake clock and line).
run nina_wait "${flags[@]}" "${nina_inc[@]}" tests/nina_wait.c "$nina/internal/nina_wait.c"
# Sockets: the table, the socket commands and the data calls of the Zephyr build's
# nina_socket.c (fake nina_proto_request() and clock).
run nina_sock "${flags[@]}" -D_DEFAULT_SOURCE "${nina_inc[@]}" tests/nina_sock.c "$nina/nina_sock.c"
# The Wi-Fi operations (nina_wifi.c, which the test includes): connect, its thread and its ends,
# disconnect, the monitor, the scans of 3.0.2 and of the patched firmware, status (fake module,
# kernel services and binding).
run nina_wifi "${flags[@]}" "${nina_inc[@]}" tests/nina_wifi.c
# Name resolution: hints, service, numeric addresses, the resolver commands (fake module).
run nina_dns "${flags[@]}" -D_DEFAULT_SOURCE "${nina_inc[@]}" tests/nina_dns.c "$nina/nina_dns.c"
# The same with the results on the OS's heap and the registration (NINA_OS_DNS_OFFLOAD, as the
# Zephyr build compiles it).
run nina_dns_offload "${flags[@]}" -D_DEFAULT_SOURCE -DNINA_OS_DNS_OFFLOAD "${nina_inc[@]}" tests/nina_dns.c \
    "$nina/nina_dns.c"

# The NINA passthrough mode (nina_passthrough.c, which the test includes) on fakes of its two
# boundaries (nina_passthrough_bus.h: the scratch registers, the module's lines, the UART, the LED;
# nina_passthrough_os.h: the clock, the host's writes, the log): the request against the fatal
# error record of the same registers (compat/log/fault_record.c), DTR and RTS on the module's
# lines, the rate and the way out, the buffer from the UART's interrupt (also on two threads), the
# start, the data both ways with their counts, the tick and the way out's sequence.
run nina_passthrough "${flags[@]}" "${nina_inc[@]}" -Icompat/log tests/nina_passthrough.c \
    compat/log/fault_record.c

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
    -Itests/zephyr_fakes -Icompat -Icompat/log -Icompat/lib/utils/getopt tests/sys_getopt.c \
    compat/lib/utils/getopt/getopt.c compat/lib/utils/getopt/getopt_long.c
# The RAM log's ring (compat/log/log_ring.c): the retained ring, dmesg pieces, line prefix and
# rate limit.
run log_ring "${flags[@]}" -Icompat/log tests/log_ring.c compat/log/log_ring.c
# The RAM log on the ring (compat/log/log_ram.c, shared by the NuttX and FreeRTOS builds) on the
# host's part of its boundary (tests/log_ram_os_host): the lines of the LOG_* macros, the OS's raw
# bytes, the check at the start, the rate limit and the lock around the ring.
run log_ram "${flags[@]}" -Itests/log_ram_os_host -Icompat/log tests/log_ram.c \
    compat/log/log_ram.c compat/log/log_ring.c
# The CPU load arithmetic of the idle loop (compat/cpu_load.h).
run cpu_load "${flags[@]}" -Icompat tests/cpu_load.c
# The read of the RP2040's TIMER (chip/rp2040: rp2040_timer_us()) on scripted registers.
run rp2040_timer_us "${flags[@]}" -Ichip/rp2040/include tests/rp2040_timer_us.c
# The CPU column of "ps" (FreeRTOS) and "kernel ps" (Zephyr): each task's share since the
# previous list (shell/ps_interval.h).
run ps_interval "${flags[@]}" -Ishell tests/ps_interval.c
# The record of a fatal error that the FreeRTOS build keeps across its restart
# (compat/log/fault_record.c): the check, the watchdog scratch copy and the lines.
run fault_record "${flags[@]}" -Icompat/log tests/fault_record.c compat/log/fault_record.c

# The hexadecimal conversions of the "kv" command (compat/lib/utils/hex.c, the copy of Zephyr's
# lib/utils/hex.c).
run hex "${flags[@]}" -Icompat tests/hex.c compat/lib/utils/hex.c
# The key-value area's helpers (compat/flash_kv.h): range checks, page writes, the check of
# what the area holds (Zephyr's NVS, the NuttX build's earlier format, erased, other).
run flash_kv "${flags[@]}" -Icompat tests/flash_kv.c
# The RP2040's part of the area's flash that the NuttX and FreeRTOS builds share
# (chip/rp2040/rp2040_flash_kv.c) on a fake of the build's flash operations (rp2040_flash_kv_os.h)
# and 8 KiB mapped at the area's uncached XIP address: the start and the linker script's bounds,
# every program and erase offset around the area, the reads, the messages.
run rp2040_flash_kv "${flags[@]}" -D_GNU_SOURCE -Itests/kv_fakes -Icompat -Icompat/log \
    -Ichip/rp2040/include tests/rp2040_flash_kv.c chip/rp2040/rp2040_flash_kv.c
# The key-value store (imu/kv_store.c, the Zephyr build's) on the copies of Zephyr's settings and
# NVS (compat/subsys, built as they are, without warnings, with the Zephyr build's Kconfig values
# of compat/zephyr_kv_config.h), the flash device and map of the area (compat/zephyr_flash_kv.c)
# and "kv wipe" (compat/kv_store_wipe.c), on a fake flash (tests/kv_flash_fake.c) and the fake
# kernel: the bytes on the flash, reads, writes, limits, garbage collection, foreign data, the
# device's refusals, a power loss at every flash operation of a sequence.  Its log, shell
# backend and app_log_os.h are tests/kv_fakes and the test's own.
kvinc=(-D_GNU_SOURCE -DZ_SHELL_CMD_NAMED_MAIN -Itests/kv_fakes -Itests/zephyr_fakes -Icompat -Icompat/log
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

    # The sensor shell is the file of the Zephyr build's patched tree: Zephyr's
    # drivers/sensor/sensor_shell.c with the local patch 0006, applied here to the tree's HEAD in a
    # scratch directory, as script/setup.sh applies it to the tree, is byte for byte
    # drivers/sensor_shell/sensor_shell.c (its sensor_shell_os.h is per build).
    zpatches=../zephyr/manifest/zephyr/patches/zephyr
    ztmp=$out/zephyr_sensor_shell
    mkdir -p "$ztmp"
    git -C "$ztree" archive HEAD drivers/sensor/sensor_shell.c | tar -x -C "$ztmp"
    (cd "$ztmp" && git apply --include='drivers/sensor/*' \
        "$OLDPWD/$zpatches"/0006-sensor-shell-*.patch)
    cmp drivers/sensor_shell/sensor_shell.c "$ztmp/drivers/sensor/sensor_shell.c"
    printf 'ok  zephyr_sensor_shell_patched (patch 0006)\n'
else
    printf 'skip zephyr_copies (no zephyr/zephyr: run west update)\n'
    printf 'skip i2c_dw_sync (no zephyr/zephyr: run west update)\n'
    printf 'skip zephyr_sensor_shell_patched (no zephyr/zephyr: run west update)\n'
fi
