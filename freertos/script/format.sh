#!/usr/bin/env bash
# Format the C sources under freertos/app/, freertos/drivers/ and
# freertos/tests/ with clang-format and
# freertos/.clang-format (the Zephyr project's style).  Run it from anywhere;
# it sources freertos/script/setup.sh itself, which installs clang-format.
#
#   freertos/script/format.sh            rewrite the files that differ
#   freertos/script/format.sh --check    only report them (exit status 1)
#
# FreeRTOSConfig.h (of the application and of the host test) is left out: it
# keeps the layout of a table of settings.

set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

check=0
case "${1:-}" in
  "") ;;
  --check) check=1 ;;
  -h|--help) sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
esac

# setup.sh is written for interactive shells; relax the strict mode around it.
set +eu
# shellcheck source=/dev/null
source "${root}/script/setup.sh" >/dev/null
setup_status=$?
set -eu
(( setup_status == 0 )) || { printf 'format.sh: freertos/script/setup.sh failed\n' >&2; exit 1; }

mapfile -t files < <(find "${root}/app" "${root}/drivers" "${root}/tests" -type f \
  \( -name '*.c' -o -name '*.h' \) ! -name FreeRTOSConfig.h | sort)
(( ${#files[@]} )) || { printf 'format.sh: no sources under %s\n' "${root}" >&2; exit 1; }

if (( check )); then
  clang-format --style="file:${root}/.clang-format" --dry-run --Werror "${files[@]}"
else
  clang-format --style="file:${root}/.clang-format" -i "${files[@]}"
fi
