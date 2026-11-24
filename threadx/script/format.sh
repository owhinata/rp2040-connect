#!/usr/bin/env bash
# Format the C sources under threadx/app/, threadx/drivers/, threadx/tests/,
# common/*/port/threadx/ and common/drivers/*/port/threadx/ (ThreadX's part
# of the layer, the shared drivers' ThreadX bindings), which only the
# ThreadX build compiles, with clang-format and threadx/.clang-format (the
# Zephyr project's style).  The shared sources the ThreadX build compiles
# with the FreeRTOS build (common/*/port/pico_sdk, common/pico_sdk,
# common/compat/kernel_generic, ...) are freertos/script/format.sh's.  Run
# it from anywhere; it sources threadx/script/setup.sh itself, which
# installs clang-format.
#
#   threadx/script/format.sh            rewrite the files that differ
#   threadx/script/format.sh --check    only report them (exit status 1)

set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

check=0
case "${1:-}" in
  "") ;;
  --check) check=1 ;;
  -h|--help) sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
  *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
esac

# setup.sh is written for interactive shells; relax the strict mode around it.
set +eu
# shellcheck source=/dev/null
source "${root}/script/setup.sh" >/dev/null
setup_status=$?
set -eu
(( setup_status == 0 )) || { printf 'format.sh: threadx/script/setup.sh failed\n' >&2; exit 1; }

# The port directories that exist (a part without one is left out).
shopt -s nullglob
mapfile -t files < <(find "${root}/app" "${root}/drivers" "${root}/tests" \
  "${root}"/../common/{*,drivers/*}/port/threadx -type f \
  \( -name '*.c' -o -name '*.h' \) | sort)
(( ${#files[@]} )) || { printf 'format.sh: no sources under %s\n' "${root}" >&2; exit 1; }

if (( check )); then
  clang-format --style="file:${root}/.clang-format" --dry-run --Werror "${files[@]}"
else
  clang-format --style="file:${root}/.clang-format" -i "${files[@]}"
fi
