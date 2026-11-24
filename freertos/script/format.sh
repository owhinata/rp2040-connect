#!/usr/bin/env bash
# Format the C sources under freertos/app/, freertos/drivers/,
# freertos/tests/, common/drivers/*/port/freertos/ (the shared drivers'
# FreeRTOS bindings), common/compat/net_offload/ and common/compat/libc_net/
# (the layer's network part and the POSIX network headers, which only the
# FreeRTOS build uses), common/tests/zephyr_kernel/ (the host test of the layer's kernel),
# common/compat/shell_engine/ (the shell for an OS without one) and
# common/compat/kernel_generic/ (the kernel part for an OS that offers only
# basic primitives), which only the FreeRTOS build compiles, with k_heap
# (common/compat/zephyr_heap.c, zephyr_heap.h) and the host check's OS part
# of that kernel part (common/tests/kernel_generic_host/), and
# common/chip/ (the RP2040's code that no OS owns, which all three builds
# compile), with clang-format and freertos/.clang-format (the Zephyr project's style).  Run
# it from anywhere; it sources freertos/script/setup.sh itself, which
# installs clang-format.
#
#   freertos/script/format.sh            rewrite the files that differ
#   freertos/script/format.sh --check    only report them (exit status 1)
#
# FreeRTOSConfig.h (of the application and of the host test) is left out: it
# keeps the layout of a table of settings.  So are the reduced copies of
# Zephyr's files in the layer's network part (common/compat/net_offload: lib,
# subsys, include/zephyr/sys/fdtable.h, include/zephyr/net/socket_offload.h):
# they keep Zephyr's text.

set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

check=0
case "${1:-}" in
  "") ;;
  --check) check=1 ;;
  -h|--help) sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
esac

# setup.sh is written for interactive shells; relax the strict mode around it.
set +eu
# shellcheck source=/dev/null
source "${root}/script/setup.sh" >/dev/null
setup_status=$?
set -eu
(( setup_status == 0 )) || { printf 'format.sh: freertos/script/setup.sh failed\n' >&2; exit 1; }

net_offload="${root}/../common/compat/net_offload"
mapfile -t files < <(find "${root}/app" "${root}/drivers" "${root}/tests" \
  "${root}"/../common/drivers/*/port/freertos "${root}"/../common/drivers/*/port/pico_sdk \
  "${net_offload}" \
  "${root}/../common/compat/libc_net" "${root}/../common/tests/zephyr_kernel" \
  "${root}/../common/compat/shell_engine" "${root}/../common/compat/kernel_generic" \
  "${root}/../common/compat/zephyr_heap.c" "${root}/../common/compat/zephyr_heap.h" \
  "${root}/../common/tests/kernel_generic_host" "${root}/../common/chip" \
  "${root}/../common/pico_sdk" "${root}/../common/compat/os_generic" -type f \
  \( -name '*.c' -o -name '*.h' \) ! -name FreeRTOSConfig.h \
  ! -path "${net_offload}/lib/*" ! -path "${net_offload}/subsys/*" \
  ! -path "${net_offload}/include/zephyr/sys/fdtable.h" \
  ! -path "${net_offload}/include/zephyr/net/socket_offload.h" | sort)
(( ${#files[@]} )) || { printf 'format.sh: no sources under %s\n' "${root}" >&2; exit 1; }

if (( check )); then
  clang-format --style="file:${root}/.clang-format" --dry-run --Werror "${files[@]}"
else
  clang-format --style="file:${root}/.clang-format" -i "${files[@]}"
fi
