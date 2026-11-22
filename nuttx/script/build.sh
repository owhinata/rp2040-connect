#!/usr/bin/env bash
# Build the NuttX firmware for the Arduino Nano RP2040 Connect and convert it
# to UF2.  Run it from anywhere; it sources nuttx/script/setup.sh itself.
#
#   nuttx/script/build.sh                 configure if needed, build, write UF2
#   nuttx/script/build.sh --clean         make distclean and configure first
#   nuttx/script/build.sh --menuconfig    configure if needed, run menuconfig
#   nuttx/script/build.sh --savedefconfig save .config to configs/imu/defconfig
#
# See nuttx/README.md ("Build") and design/nuttx-porting.md ("What `build.sh`
# does") for what is checked and where the output goes.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: nuttx/script/build.sh [options]

  (no option)       Configure when needed, build, check the image, write
                    nuttx/build/nuttx.uf2.
  --clean           Start from "make distclean" and configure again.
  --menuconfig      Configure when needed, then run "make menuconfig" (no build).
  --savedefconfig   Run "make savedefconfig" and write the result, with
                    CONFIG_APPS_DIR, to boards/arduino-nano-rp2040-connect/
                    configs/imu/defconfig (no build).
  -j N              Parallel jobs for make (default: number of CPUs).
  -v                Verbose make output (V=1).
  -h, --help        Show this help.
EOF
}

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
board_rel=boards/arduino-nano-rp2040-connect
config_rel="../${board_rel}/configs/imu"   # relative to nuttx/nuttx (TOPDIR)
defconfig="${root}/${board_rel}/configs/imu/defconfig"
nx="${root}/nuttx"
apps="${root}/nuttx-apps"
out_dir="${root}/build"
stamp="${out_dir}/.configure-stamp"

flash_base=0x10000000
image_limit=0x10ffe000   # first byte of the key-value area (see scripts/arduino-nano-rp2040-connect-flash.ld)

clean=0
action=build
jobs=""
verbose=0
while (( $# )); do
  case "$1" in
    --clean) clean=1 ;;
    --menuconfig) action=menuconfig ;;
    --savedefconfig) action=savedefconfig ;;
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
(( setup_status == 0 )) || die "nuttx/script/setup.sh failed"

[[ -n "${PICO_SDK_PATH:-}" ]] || die "PICO_SDK_PATH is not set (boot2 would be left out)"
[[ -f "${PICO_SDK_PATH}/src/rp2040/boot_stage2/boot2_at25sf128a.S" ]] || \
  die "PICO_SDK_PATH=${PICO_SDK_PATH} has no boot_stage2/boot2_at25sf128a.S"
command -v arm-none-eabi-objdump >/dev/null || die "arm-none-eabi-objdump not found"

make_args=(-C "$nx")
(( verbose )) && make_args+=(V=1)

# Configure ----------------------------------------------------------------------
# NuttX reads the board Kconfig (copied to boards/dummy/Kconfig) and generates
# the apps Kconfig files only when it is configured, so configure again (from
# distclean) whenever one of those inputs changes.  The stamp records them.
config_inputs() {
  local f
  printf 'nuttx %s\n' "$(git -C "$nx" rev-parse HEAD 2>/dev/null || echo unknown)"
  printf 'apps %s\n' "$(git -C "$apps" rev-parse HEAD 2>/dev/null || echo unknown)"
  # scripts/Make.defs: make does not rebuild boot2 when its flags (BOOT2CFLAGS) change.
  for f in "$defconfig" "${root}/${board_rel}/Kconfig" "${root}/${board_rel}/scripts/Make.defs"; do
    printf '%s %s\n' "${f#"${root}/"}" "$(sha256sum < "$f" | cut -d' ' -f1)"
  done
  # Kconfig files of the out-of-tree code, and which app directories exist
  # (the generated nuttx/apps/Kconfig lists those with a Makefile).
  while IFS= read -r f; do
    printf '%s %s\n' "${f#"${root}/"}" "$(sha256sum < "$f" | cut -d' ' -f1)"
  done < <(find "${root}/drivers" "${root}/apps" -name Kconfig -type f \
             ! -path "${root}/apps/Kconfig" | LC_ALL=C sort)
  while IFS= read -r f; do
    printf 'appdir %s\n' "${f#"${root}/"}"
  done < <(find "${root}/apps" -mindepth 2 -maxdepth 2 \( -name Makefile -o -name Make.defs \) \
             -type f | LC_ALL=C sort)
  if [[ -d "${root}/patches" ]]; then
    while IFS= read -r f; do
      printf '%s %s\n' "${f#"${root}/"}" "$(sha256sum < "$f" | cut -d' ' -f1)"
    done < <(find "${root}/patches" -name '*.patch' -type f | LC_ALL=C sort)
  fi
}

remove_generated_kconfig() {
  rm -f "${nx}/boards/dummy/Kconfig" "${root}/apps/Kconfig" "${root}/apps/.kconfig" \
        "${apps}/Kconfig"
}

configure() {
  step "Configure (${config_rel})"
  if [[ -f "${nx}/.config" || -e "${nx}/Make.defs" ]]; then
    make "${make_args[@]}" distclean
  fi
  remove_generated_kconfig
  (cd "$nx" && ./tools/configure.sh -l -a ../nuttx-apps "$config_rel")
  mkdir -p "$out_dir"
  config_inputs > "$stamp"
}

mkdir -p "$out_dir"
if (( clean )); then
  configure
elif [[ ! -f "${nx}/.config" ]]; then
  configure
elif [[ ! -f "$stamp" ]] || ! cmp -s "$stamp" <(config_inputs); then
  printf 'Configuration inputs changed since the last configure:\n'
  diff "$stamp" <(config_inputs) 2>/dev/null | sed -n 's/^[<>] /  /p' | sort -u || true
  configure
fi

config_value() {  # name -> value from .config ("" when unset)
  sed -n "s/^$1=//p" "${nx}/.config"
}

case "$action" in
  menuconfig)
    step "menuconfig"
    make "${make_args[@]}" menuconfig
    printf '\nRun "nuttx/script/build.sh --savedefconfig" to keep the changes in %s.\n' \
      "${defconfig#"${root}/"}"
    exit 0
    ;;
  savedefconfig)
    step "savedefconfig"
    make "${make_args[@]}" savedefconfig
    # savedefconfig drops CONFIG_APPS_DIR; put it back and keep the file sorted
    # the way savedefconfig sorts it (LC_ALL=C sort after the warning header).
    tmp="${defconfig}.tmp"
    {
      grep '^#' "${nx}/defconfig" | grep -v '^# CONFIG_'
      { grep -v '^#' "${nx}/defconfig"; grep '^# CONFIG_' "${nx}/defconfig" || true
        echo 'CONFIG_APPS_DIR="../nuttx-apps"'; } | LC_ALL=C sort -u
    } > "$tmp"
    if cmp -s "$tmp" "$defconfig"; then
      rm -f "$tmp"
      printf '%s is unchanged.\n' "${defconfig#"${root}/"}"
    else
      mv "$tmp" "$defconfig"
      printf 'Wrote %s.\n' "${defconfig#"${root}/"}"
    fi
    # .config already matches the saved defconfig: no need to configure again.
    config_inputs > "$stamp"
    exit 0
    ;;
esac

# Build --------------------------------------------------------------------------
[[ "$(config_value CONFIG_RP2040_FLASH_BOOT)" == y ]] || die "CONFIG_RP2040_FLASH_BOOT is not y"
[[ "$(config_value CONFIG_RAW_BINARY)" == y ]] || die "CONFIG_RAW_BINARY is not y (no nuttx.bin)"
[[ "$(config_value CONFIG_RP2040_FLASH_CHIP)" == '"at25sf128a"' ]] || \
  die "CONFIG_RP2040_FLASH_CHIP is $(config_value CONFIG_RP2040_FLASH_CHIP), not \"at25sf128a\""

step "make -j${jobs}"
make "${make_args[@]}" -j"$jobs"

# Check the image ----------------------------------------------------------------
step "Check the image"
elf="${nx}/nuttx"
bin="${nx}/nuttx.bin"
[[ -f "$elf" && -f "$bin" ]] || die "nuttx or nuttx.bin is missing after the build"

# "objdump -h" prints, per section, "Idx Name Size VMA LMA File-off Algn" and
# the flags on the next line.  Find .boot2 and the end of what is loaded into
# flash (the highest LMA + size of the LOAD sections).
boot2_size="" boot2_vma="" image_end=0
while read -r idx name size vma lma _; do
  [[ "$idx" =~ ^[0-9]+$ ]] || continue
  read -r flags
  if [[ "$name" == .boot2 ]]; then
    boot2_size=$((16#$size)) boot2_vma=$((16#$vma))
  fi
  if [[ "$flags" == *LOAD* ]] && (( 16#$lma + 16#$size > image_end )); then
    image_end=$((16#$lma + 16#$size))
  fi
done < <(arm-none-eabi-objdump -h "$elf")

[[ -n "$boot2_size" ]] || die ".boot2 is missing from the ELF (boot2 not built?)"
(( boot2_vma == flash_base )) || die "$(printf '.boot2 is at 0x%08x, not %s' "$boot2_vma" "$flash_base")"
(( boot2_size == 256 )) || die ".boot2 is ${boot2_size} bytes, not 256"
bin_size=$(stat -c %s "$bin")
(( flash_base + bin_size == image_end )) || \
  die "nuttx.bin is ${bin_size} bytes but the ELF loads up to $(printf 0x%08x "$image_end")"
(( image_end <= image_limit )) || \
  die "image ends at $(printf 0x%08x "$image_end"), beyond $(printf 0x%08x "$image_limit") (key-value area)"
# With the bootrom's memcpy() and memset() (CONFIG_RP2040_ROM_MEM_OPS, nuttx/patches/nuttx/0010),
# __start() must not call them before it has cleared .bss (rp2040_start.c keeps its loops).
if [[ "$(config_value CONFIG_RP2040_ROM_MEM_OPS)" == y ]]; then
  start_dis="$(arm-none-eabi-objdump -d --disassemble=__start "$elf")"
  grep -q '<__start>:' <<< "$start_dis" || die "__start is missing from the ELF"
  if grep -qE '<(memcpy|memset|__aeabi_mem[a-z0-9]*)>' <<< "$start_dis"; then
    die "__start calls memcpy() or memset(), which need .bss cleared (CONFIG_RP2040_ROM_MEM_OPS)"
  fi
fi
# With the SIO divider (CONFIG_RP2040_PICO_DIVIDER), every integer division
# goes to the Pico SDK's routines (ld --wrap, scripts/Make.defs): none of
# libgcc's may be linked (it would mean a call that the wrapping missed).
if [[ "$(config_value CONFIG_RP2040_PICO_DIVIDER)" == y ]]; then
  syms="$(arm-none-eabi-nm "$elf")"
  for s in __aeabi_idiv __aeabi_idivmod __aeabi_uidiv __aeabi_uidivmod \
           __aeabi_ldivmod __aeabi_uldivmod; do
    grep -qE " T __wrap_${s}\$" <<< "$syms" || die "__wrap_${s} is missing (CONFIG_RP2040_PICO_DIVIDER)"
  done
  for s in __aeabi_idiv __aeabi_idivmod __aeabi_uidiv __aeabi_uidivmod __aeabi_ldivmod \
           __aeabi_uldivmod __divsi3 __udivsi3 __modsi3 __umodsi3 __divdi3 __udivdi3 \
           __moddi3 __umoddi3 __udivmoddi4 __gnu_ldivmod_helper __gnu_uldivmod_helper; do
    if grep -qE " [TtWw] ${s}\$" <<< "$syms"; then
      die "libgcc's ${s} is linked (CONFIG_RP2040_PICO_DIVIDER: divisions go to the SIO divider)"
    fi
  done
fi
printf '.boot2 0x%08x, 256 bytes; image 0x%08x-0x%08x (%d bytes, %d bytes left before 0x%08x)\n' \
  "$flash_base" "$flash_base" $((image_end - 1)) "$bin_size" $((image_limit - image_end)) "$image_limit"

# UF2 ----------------------------------------------------------------------------
step "UF2"
uf2="${out_dir}/nuttx.uf2"
python3 "${root}/script/bin2uf2.py" --base "$flash_base" --end "$image_limit" "$bin" "$uf2"
printf '\nUF2: %s (%d bytes)\n' "$uf2" "$(stat -c %s "$uf2")"
