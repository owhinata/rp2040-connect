#!/usr/bin/env bash
# Build the Pico SDK + FreeRTOS firmware for the Arduino Nano RP2040 Connect.
# Run it from anywhere; it sources freertos/script/setup.sh itself.
#
#   freertos/script/build.sh            build freertos/app/usb-console
#   freertos/script/build.sh --clean    remove its build directory first
#
# See freertos/README.md ("Build").

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: freertos/script/build.sh [options] [APP]

  APP           Directory name under freertos/app/ (default: usb-console).
                The result is freertos/build/APP/APP.uf2.
  --clean       Remove freertos/build/APP first.
  -DNAME=VALUE  Pass a CMake option on (e.g. -DI2C_DW_I2C1=ON, the
                I2C1 controller).  CMake keeps it in the build directory
                until --clean.
  -j N          Parallel jobs (default: number of CPUs).
  -v            Verbose make output.
  -h, --help    Show this help.
EOF
}

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

app=""
clean=0
jobs=""
verbose=0
cmake_defs=()
while (( $# )); do
  case "$1" in
    --clean) clean=1 ;;
    -j) shift; jobs="${1:?-j needs a number}" ;;
    -j*) jobs="${1#-j}" ;;
    -v) verbose=1 ;;
    -D?*) cmake_defs+=("$1") ;;
    -h|--help) usage; exit 0 ;;
    -*) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    *)
      [[ -z "$app" ]] || { printf 'Only one APP can be given\n' >&2; exit 2; }
      app="$1"
      ;;
  esac
  shift
done
[[ -n "$app" ]] || app=usb-console
[[ -n "$jobs" ]] || jobs="$(nproc 2>/dev/null || echo 4)"

die() { printf 'build.sh: %s\n' "$*" >&2; exit 1; }
step() { printf '\n== %s\n' "$*"; }

src_dir="${root}/app/${app}"
build_dir="${root}/build/${app}"
[[ -f "${src_dir}/CMakeLists.txt" ]] || die "${src_dir}/CMakeLists.txt not found"

# Environment ------------------------------------------------------------------
# setup.sh is written for interactive shells; relax the strict mode around it.
set +eu
# shellcheck source=/dev/null
source "${root}/script/setup.sh"
setup_status=$?
set -eu
(( setup_status == 0 )) || die "freertos/script/setup.sh failed"
command -v arm-none-eabi-gcc >/dev/null || die "arm-none-eabi-gcc not found"

# Build --------------------------------------------------------------------------
if (( clean )); then
  step "clean"
  rm -rf -- "$build_dir"
fi

# The Pico SDK makes the UF2 with picotool.  Without one on the host it
# fetches picotool's source (the version of the SDK) and builds it for the
# host inside the build directory; the source is kept in freertos/picotool/,
# so only the first configuration needs the network.
step "cmake ${app}"
cmake -S "$src_dir" -B "$build_dir" -G "Unix Makefiles" \
  -DCMAKE_BUILD_TYPE=MinSizeRel \
  -DPICOTOOL_FETCH_FROM_GIT_PATH="${root}/picotool" \
  "${cmake_defs[@]}"

make_args=(-C "$build_dir" -j"$jobs")
(( verbose )) && make_args+=(VERBOSE=1)
step "make -j${jobs}"
make "${make_args[@]}"

# Output -------------------------------------------------------------------------
step "Output"
uf2="${build_dir}/${app}.uf2"
[[ -f "$uf2" ]] || die "${uf2} is missing after the build"
arm-none-eabi-size "${build_dir}/${app}.elf"
printf '\nUF2: %s (%d bytes)\n' "$uf2" "$(stat -c %s "$uf2")"
