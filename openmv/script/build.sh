#!/usr/bin/env bash
# Build the OpenMV firmware (MicroPython) for the Arduino Nano RP2040 Connect.
# Run it from anywhere; it sources openmv/script/setup.sh itself.
#
#   openmv/script/build.sh            build, copy the result to openmv/build/
#   openmv/script/build.sh --clean    remove OpenMV's build directory first
#
# See openmv/README.md ("Build").

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: openmv/script/build.sh [options]

  (no option)   Build mpy-cross and the firmware, copy firmware.uf2,
                firmware.bin and firmware.elf to openmv/build/.
  --clean       Remove openmv/build/<target> first.
  -j N          Parallel jobs for make (default: number of CPUs).
  -v            Verbose make output (V=1).
  -h, --help    Show this help.
EOF
}

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
omv="${root}/openmv"
target=ARDUINO_NANO_RP2040_CONNECT
out_dir="${root}/build"

clean=0
jobs=""
verbose=0
while (( $# )); do
  case "$1" in
    --clean) clean=1 ;;
    -j) shift; jobs="${1:?-j needs a number}" ;;
    -j*) jobs="${1#-j}" ;;
    -v) verbose=1 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done
[[ -n "$jobs" ]] || jobs="$(nproc 2>/dev/null || echo 4)"

die() { printf 'build.sh: %s\n' "$*" >&2; exit 1; }
step() { printf '\n== %s\n' "$*"; }

# Environment ------------------------------------------------------------------
# setup.sh is written for interactive shells; relax the strict mode around it.
set +eu
# shellcheck source=/dev/null
source "${root}/script/setup.sh"
setup_status=$?
set -eu
(( setup_status == 0 )) || die "openmv/script/setup.sh failed"
[[ -n "${OPENMV_SDK_DIR:-}" ]] || die "OPENMV_SDK_DIR is not set"

command -v arm-none-eabi-gcc >/dev/null || die "arm-none-eabi-gcc not found"
command -v cmake >/dev/null || die "cmake not found"

make_args=(-C "$omv" TARGET="$target" SDK_DIR="$OPENMV_SDK_DIR")
(( verbose )) && make_args+=(V=1)

# Build --------------------------------------------------------------------------
if (( clean )); then
  step "clean"
  make "${make_args[@]}" clean
fi

# mpy-cross runs on the host and is built with the host compiler.
step "mpy-cross"
make -C "${omv}/lib/micropython/mpy-cross" -j"$jobs"

step "make -j${jobs} TARGET=${target}"
make "${make_args[@]}" -j"$jobs"

# Output -------------------------------------------------------------------------
step "Output"
bin_dir="${omv}/build/${target}/bin"
[[ -f "${bin_dir}/firmware.uf2" ]] || die "firmware.uf2 is missing after the build"
mkdir -p "$out_dir"
cp -- "${bin_dir}/firmware.uf2" "${bin_dir}/firmware.bin" "${bin_dir}/firmware.elf" "$out_dir/"
arm-none-eabi-size "${out_dir}/firmware.elf"
printf '\nUF2: %s (%d bytes)\n' "${out_dir}/firmware.uf2" "$(stat -c %s "${out_dir}/firmware.uf2")"
