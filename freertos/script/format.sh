#!/usr/bin/env bash
# Format the C sources under freertos/app/, freertos/drivers/,
# freertos/tests/ and common/drivers/*/freertos/ (the shared drivers'
# FreeRTOS bindings) with clang-format and
# freertos/.clang-format (the Zephyr project's style).  Run it from anywhere;
# it sources freertos/script/setup.sh itself, which installs clang-format.
#
#   freertos/script/format.sh            rewrite the files that differ
#   freertos/script/format.sh --check    only report them (exit status 1)
#
# FreeRTOSConfig.h (of the application and of the host test) is left out: it
# keeps the layout of a table of settings.  So are the reduced copies of
# Zephyr's files in the compatibility layer (drivers/compat/lib,
# drivers/compat/subsys, zephyr/sys/fdtable.h, zephyr/net/socket_offload.h):
# they keep Zephyr's text.

set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

check=0
case "${1:-}" in
  "") ;;
  --check) check=1 ;;
  -h|--help) sed -n '2,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
esac

# setup.sh is written for interactive shells; relax the strict mode around it.
set +eu
# shellcheck source=/dev/null
source "${root}/script/setup.sh" >/dev/null
setup_status=$?
set -eu
(( setup_status == 0 )) || { printf 'format.sh: freertos/script/setup.sh failed\n' >&2; exit 1; }

mapfile -t files < <(find "${root}/app" "${root}/drivers" "${root}/tests" \
  "${root}"/../common/drivers/*/freertos -type f \
  \( -name '*.c' -o -name '*.h' \) ! -name FreeRTOSConfig.h \
  ! -path "${root}/drivers/compat/lib/*" ! -path "${root}/drivers/compat/subsys/*" \
  ! -path "${root}/drivers/compat/zephyr/sys/fdtable.h" \
  ! -path "${root}/drivers/compat/zephyr/net/socket_offload.h" | sort)
(( ${#files[@]} )) || { printf 'format.sh: no sources under %s\n' "${root}" >&2; exit 1; }

if (( check )); then
  clang-format --style="file:${root}/.clang-format" --dry-run --Werror "${files[@]}"
else
  clang-format --style="file:${root}/.clang-format" -i "${files[@]}"
fi
