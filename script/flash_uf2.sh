#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Write a UF2 image to the Arduino Nano RP2040 Connect through the RP2040
# USB bootloader (BOOTSEL, the RPI-RP2 drive).  No sudo.
#
# The shared part of zephyr/script/flash.sh, nuttx/script/flash.sh and
# freertos/script/flash.sh, which run it with --target.  Whichever of the
# three builds is running, its shell's "bootsel" command restarts the board
# into BOOTSEL, so any of them can be written over any other.
#
#   script/flash_uf2.sh --target nuttx [--build] [UF2]
#   script/flash_uf2.sh other.uf2         any RP2040 UF2 (OpenMV, ...)
#
# See the "Flashing" sections of the READMEs.

set -euo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# Test hooks (for running against a fake system): the lsblk, udisksctl and
# fuser commands, and the directory that holds the by-id links.
lsblk_cmd="${FLASH_UF2_LSBLK:-lsblk}"
udisksctl_cmd="${FLASH_UF2_UDISKSCTL:-udisksctl}"
fuser_cmd="${FLASH_UF2_FUSER:-fuser}"
byid_dir="${FLASH_UF2_BYID_DIR:-/dev/serial/by-id}"

# The builds: USB console (/dev/serial/by-id name), its name in messages.
zephyr_glob='usb-Zephyr_Project_Zephyr_USB_Console_*'
nuttx_glob='usb-*NuttX_USB_Console*'
freertos_glob='usb-*FreeRTOS_USB_Console*'
declare -A build_name=([zephyr]=Zephyr [nuttx]=NuttX [freertos]=FreeRTOS)
declare -A build_glob=([zephyr]="$zephyr_glob" [nuttx]="$nuttx_glob" [freertos]="$freertos_glob")
declare -A build_uf2=(
  [zephyr]="${repo}/zephyr/build/usb-console/zephyr/zephyr.uf2"
  [nuttx]="${repo}/nuttx/build/nuttx.uf2"
  [freertos]="${repo}/freertos/build/usb-console/usb-console.uf2"
)
builds=(zephyr nuttx freertos)
# The shells' prompts (Zephyr, NuttX, FreeRTOS).
prompt_re='uart:~\$ |nsh\$ |freertos:~\$ '
drive_label=RPI-RP2
uf2_family=e48bff56          # RP2040
probe_s=2                    # for the shell's answer to Ctrl-C
bootsel_s=15                 # for the drive after "bootsel"

usage() {
  local prog=script/flash_uf2.sh def="(required)" cons="a USB console of the three builds"
  if [[ -n "$target" ]]; then
    prog="${target}/script/flash.sh"
    def="(default: ${build_uf2[$target]#"${repo}/"})"
    cons="the ${build_name[$target]} USB console"
  fi
  cat <<EOF
Usage: ${prog} [options] [UF2]

Write UF2 ${def} to the board:
  1. If an RPI-RP2 drive (RP2040 in BOOTSEL) is present, copy to it.
  2. Otherwise, if the USB console of one of the three builds (Zephyr, NuttX,
     FreeRTOS) is present and not open in another program, send it Ctrl-C;
     when its shell answers with a prompt, send "bootsel" and wait for the
     drive.
  3. Otherwise (no console, or no answer: an old or stuck image, the NINA
     passthrough), explain how to enter BOOTSEL by hand and wait for the
     drive.
The drive is mounted with udisksctl if it is not mounted already (no sudo).
After the copy, wait for the drive to go away and for ${cons}
to appear, and print its /dev/serial/by-id path.

Options:
  --build       Build first.
  --wait SEC    How long to wait for the drive, and then for the console
                (default: 60).
  --dry-run     Look for the drive and the consoles (lsblk and
                /dev/serial/by-id only) and print what would be done; do not
                build, open a console, mount, read the drive, copy or wait.
  -h, --help    Show this help.
EOF
}

target=""
uf2=""
build=0
dry=0
wait_s=60
help=0
while (( $# )); do
  case "$1" in
    --target) shift; target="${1:?--target needs zephyr, nuttx or freertos}" ;;
    --target=*) target="${1#--target=}" ;;
    --build) build=1 ;;
    --dry-run) dry=1 ;;
    --wait) shift; wait_s="${1:?--wait needs a number of seconds}" ;;
    --wait=*) wait_s="${1#--wait=}" ;;
    -h|--help) help=1 ;;
    -*) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    *)
      [[ -z "$uf2" ]] || { printf 'Only one UF2 can be given\n' >&2; exit 2; }
      uf2="$1"
      ;;
  esac
  shift
done
if [[ -n "$target" && -z "${build_name[$target]:-}" ]]; then
  printf 'Unknown target: %s (zephyr, nuttx or freertos)\n' "$target" >&2
  exit 2
fi
if (( help )); then
  usage
  exit 0
fi
[[ "$wait_s" =~ ^[0-9]+$ ]] || { printf -- '--wait needs a number of seconds\n' >&2; exit 2; }
if [[ -z "$target" ]]; then
  [[ -n "$uf2" ]] || { printf 'No UF2 given\n' >&2; usage >&2; exit 2; }
  (( !build )) || { printf -- '--build needs --target\n' >&2; exit 2; }
fi
[[ -n "$uf2" ]] || uf2="${build_uf2[$target]}"

die() { printf 'flash.sh: %s\n' "$*" >&2; exit 1; }
note() { printf 'flash.sh: %s\n' "$*"; }
plan() { printf 'flash.sh: [dry-run] would %s\n' "$*"; }

run_build() {
  case "$target" in
    zephyr)
      # setup.sh applies the local patches and sets ZEPHYR_SDK_INSTALL_DIR;
      # its west alias is not used in a script.
      (
        cd "${repo}/zephyr"
        set +eu   # setup.sh is written for an interactive shell
        # shellcheck source=/dev/null
        source script/setup.sh || exit 1
        set -eu
        .venv/bin/west build -b arduino_nano_connect app/usb-console -d build/usb-console
      )
      ;;
    nuttx | freertos)
      "${repo}/${target}/script/build.sh"
      ;;
  esac
}

build_cmd_text() {
  case "$target" in
    zephyr) printf 'source zephyr/script/setup.sh; west build -b arduino_nano_connect app/usb-console -d build/usb-console' ;;
    *) printf '%s' "${target}/script/build.sh" ;;
  esac
}

# UF2 ------------------------------------------------------------------------------
# Every 512-byte block starts with the magic numbers 0x0a324655 0x9e5d5157 and
# ends with 0x0ab16f30; flag 0x2000 says that word 7 is the family ID.
check_uf2() {
  local f="$1" size w end
  [[ -f "$f" ]] || return 1
  size=$(stat -c %s -- "$f")
  (( size > 0 && size % 512 == 0 )) || die "$f: size ${size} is not a multiple of 512 (not a UF2?)"
  read -r -a w < <(od -An -v -tx4 -w32 --endian=little -N32 -- "$f")
  end=$(od -An -v -tx4 --endian=little -j508 -N4 -- "$f" | tr -d ' ')
  [[ "${w[0]}" == 0a324655 && "${w[1]}" == 9e5d5157 && "$end" == 0ab16f30 ]] || \
    die "$f: not a UF2 file (bad magic numbers)"
  if (( (16#${w[2]} & 0x2000) == 0 )) || [[ "${w[7]}" != "$uf2_family" ]]; then
    die "$f: not a UF2 for the RP2040 (family 0x${w[7]}, flags 0x${w[2]}; expected family 0x${uf2_family})"
  fi
  note "UF2: $f ($((size / 512)) blocks, family 0x${w[7]})"
}

# The RPI-RP2 drive ------------------------------------------------------------------
# Print "<partition><TAB><mount point>" for every partition labelled RPI-RP2
# on a USB disk whose vendor/model is the RP2040 boot ROM's ("RPI", "RP2").
# The mount point is empty when the partition is not mounted.  lsblk -P
# quotes every value and writes special characters as \xHH.
lsblk_value() {  # KEY LINE -> unescaped value
  local re="(^| )$1=\"([^\"]*)\""
  if [[ "$2" =~ $re ]]; then
    printf '%b' "${BASH_REMATCH[2]}"
  fi
}

trim() {  # strip leading and trailing blanks (lsblk pads VENDOR with spaces)
  local v="$1"
  v="${v#"${v%%[![:space:]]*}"}"
  printf '%s' "${v%"${v##*[![:space:]]}"}"
}

find_drives() {
  local out line name type pk label mnt
  local -A tran=() vendor=() model=()
  out="$("$lsblk_cmd" -nP -p -o NAME,TYPE,PKNAME,LABEL,MOUNTPOINT,TRAN,VENDOR,MODEL 2>/dev/null)" || return 0
  while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    name=$(lsblk_value NAME "$line")
    tran[$name]=$(lsblk_value TRAN "$line")
    vendor[$name]=$(lsblk_value VENDOR "$line")
    model[$name]=$(lsblk_value MODEL "$line")
  done <<< "$out"
  while IFS= read -r line; do
    [[ -n "$line" ]] || continue
    name=$(lsblk_value NAME "$line")
    type=$(lsblk_value TYPE "$line")
    label=$(lsblk_value LABEL "$line")
    [[ "$label" == "$drive_label" && "$type" == part ]] || continue
    pk=$(lsblk_value PKNAME "$line")
    [[ "${tran[$pk]:-}" == usb ]] || continue
    [[ "$(trim "${vendor[$pk]:-}")" == RPI && "$(trim "${model[$pk]:-}")" == RP2 ]] || continue
    mnt=$(lsblk_value MOUNTPOINT "$line")
    printf '%s\t%s\n' "$name" "$mnt"
  done <<< "$out"
}

# INFO_UF2.TXT of the RP2040 boot ROM: "Model: Raspberry Pi RP2",
# "Board-ID: RPI-RP2".
is_rp2040_bootsel() {
  local info="$1/INFO_UF2.TXT"
  [[ -f "$info" ]] && grep -q 'Board-ID: RPI-RP2' "$info" && grep -q 'Model: Raspberry Pi RP2' "$info"
}

# Sets drive_dev and drive_mnt when exactly one drive is present.  Returns 1
# when there is none, and stops when there are several.
pick_drive() {
  local lines=() l
  while IFS= read -r l; do
    [[ -n "$l" ]] && lines+=("$l")
  done < <(find_drives)
  (( ${#lines[@]} )) || return 1
  if (( ${#lines[@]} > 1 )); then
    printf 'flash.sh: more than one %s drive:\n' "$drive_label" >&2
    printf '  %s\n' "${lines[@]//$'\t'/ on }" >&2
    die "connect only the board to be written"
  fi
  drive_dev="${lines[0]%%$'\t'*}"
  drive_mnt="${lines[0]#*$'\t'}"
  return 0
}

wait_for_drive() {  # timeout in s
  local t=$(( $1 * 2 ))
  while (( t-- > 0 )); do
    pick_drive && return 0
    sleep 0.5
  done
  pick_drive
}

mount_drive() {
  local tries
  if [[ -n "$drive_mnt" ]]; then
    return 0
  fi
  if (( dry )); then
    plan "mount ${drive_dev} with: ${udisksctl_cmd} mount -b ${drive_dev} --no-user-interaction"
    return 0
  fi
  # A desktop session mounts the drive by itself a few seconds after it
  # appears; give it that time before mounting it ourselves.
  for (( tries = 0; tries < 10; tries++ )); do
    sleep 0.5
    pick_drive || die "the ${drive_label} drive went away"
    [[ -n "$drive_mnt" ]] && return 0
  done
  note "${drive_dev} is not mounted; mounting it with udisksctl"
  if ! "$udisksctl_cmd" mount -b "$drive_dev" --no-user-interaction; then
    pick_drive && [[ -n "$drive_mnt" ]] && return 0   # mounted by someone else meanwhile
    printf 'flash.sh: could not mount %s without sudo.\n' "$drive_dev" >&2
    printf '  Mount it from the file manager, or run: udisksctl mount -b %s\n' "$drive_dev" >&2
    die "then run flash.sh again"
  fi
  # The mount point shows up in lsblk a moment later.
  for (( tries = 0; tries < 10; tries++ )); do
    pick_drive || break
    [[ -n "$drive_mnt" ]] && return 0
    sleep 0.5
  done
  die "${drive_dev} was mounted but lsblk shows no mount point; mount it by hand and run flash.sh again"
}

# Consoles -----------------------------------------------------------------------------
list_byid() {  # glob -> matching paths, one per line
  local f
  shopt -s nullglob
  for f in "$byid_dir"/$1; do
    printf '%s\n' "$f"
  done
  shopt -u nullglob
}

# Sets console and console_build when exactly one console of the three
# builds is present; returns 1 otherwise (and lists them when several).
pick_console() {
  local b f found=() kinds=()
  for b in "${builds[@]}"; do
    while IFS= read -r f; do
      [[ -n "$f" ]] || continue
      found+=("$f")
      kinds+=("$b")
    done < <(list_byid "${build_glob[$b]}")
  done
  (( ${#found[@]} )) || return 1
  if (( ${#found[@]} > 1 )); then
    note "more than one USB console; not choosing one:"
    printf '  %s\n' "${found[@]}"
    return 1
  fi
  console="${found[0]}"
  console_build="${kinds[0]}"
}

# Some other process (picocom, screen, ...) has the console open: nothing is
# sent then.
console_in_use() {
  command -v "$fuser_cmd" >/dev/null 2>&1 || return 1
  "$fuser_cmd" -s "$(readlink -f -- "$1")" 2>/dev/null
}

# Ctrl-C drops whatever is on the command line in all three shells (Zephyr's
# metakeys, NuttX's readline with patches/apps/0003, the FreeRTOS command
# line) and they answer with a new prompt.  Only then is "bootsel" sent; no
# prompt (an old or stuck image, the NINA passthrough, where the bytes go to
# the module's UART) sends nothing more.  The port is set to 115200 before it
# is opened and closed: a port closed at 300 bps leaves the NINA passthrough.
send_bootsel() {  # device
  local dev="$1" fd buf="" ch end now
  timeout 5 stty -F "$dev" 115200 raw -echo clocal 2>/dev/null || return 1
  exec {fd}<>"$dev" || return 1
  printf '\003' >&"$fd"
  end=$(( ${EPOCHREALTIME/./} + probe_s * 1000000 ))
  while :; do
    if IFS= read -r -t 0.1 -N 1 -u "$fd" ch; then
      buf+="$ch"
      # Drop the VT100 sequences (colors of the Zephyr prompt).
      buf="$(printf '%s' "$buf" | sed 's/\x1b\[[0-9;]*[A-Za-z]//g')"
      if [[ "$buf" =~ $prompt_re ]]; then
        printf 'bootsel\r' >&"$fd"
        sleep 0.2
        exec {fd}>&-
        return 0
      fi
    fi
    now=${EPOCHREALTIME/./}
    (( now < end )) || break
  done
  exec {fd}>&-
  return 1
}

manual_bootsel_help() {
  cat <<EOF
flash.sh: put the board into BOOTSEL by hand:
  - with a firmware of this repository running: type "bootsel" in its shell;
  - with the OpenMV (MicroPython) firmware: import machine; machine.bootloader()
    in the REPL;
  - otherwise (or in the NINA passthrough): press the RESET button twice
    quickly; if that does not work, connect REC to GND with a jumper wire,
    press RESET, and remove the wire when the ${drive_label} drive appears.
EOF
}

# The console to wait for after the copy: the target's, or any of the three.
wait_globs() {
  if [[ -n "$target" ]]; then
    printf '%s\n' "${build_glob[$target]}"
  else
    printf '%s\n' "${build_glob[@]}"
  fi
}

# Without --target (another firmware, perhaps), the wait ends 10 s after the
# drive went away.
wait_for_console() {  # timeout in s; prints the by-id path(s)
  local t=$(( $1 * 2 )) gone=0 found g
  while (( t-- > 0 )); do
    if ! pick_drive; then
      found=""
      while IFS= read -r g; do
        found+="$(list_byid "$g")"$'\n'
      done < <(wait_globs)
      found="$(printf '%s' "$found" | sed '/^$/d')"
      if [[ -n "$found" ]]; then
        printf '%s\n' "$found"
        return 0
      fi
      [[ -n "$target" ]] || (( ++gone < 20 )) || return 1
    fi
    sleep 0.5
  done
  return 1
}

# Main ---------------------------------------------------------------------------------
if (( build )); then
  if (( dry )); then
    plan "build: $(build_cmd_text)"
  else
    run_build
  fi
fi

if ! check_uf2 "$uf2"; then
  if (( dry && build )); then
    note "UF2: $uf2 (not built yet)"
  elif [[ -n "$target" ]]; then
    die "$uf2 not found (build it, or use --build)"
  else
    die "$uf2 not found"
  fi
fi

drive_dev="" drive_mnt=""
console="" console_build=""
if pick_drive; then
  note "found the ${drive_label} drive: ${drive_dev}${drive_mnt:+ on ${drive_mnt}}"
else
  note "no ${drive_label} drive"
  sent=0
  if pick_console; then
    name="${build_name[$console_build]}"
    if (( dry )); then
      plan "send Ctrl-C to the ${name} USB console ${console}, and \"bootsel\" when its shell answers"
      sent=1
    elif console_in_use "$console"; then
      note "the ${name} USB console ${console} is open in another program; not sending \"bootsel\""
    else
      note "sending Ctrl-C to the ${name} USB console ${console}"
      if send_bootsel "$console"; then
        note "its shell answered; sent \"bootsel\""
        sent=1
      else
        note "no shell prompt from ${console} within ${probe_s} s (an old or stuck image, or the NINA passthrough?)"
      fi
    fi
  fi
  if (( dry )); then
    (( sent )) || manual_bootsel_help
    plan "wait up to ${wait_s} s for the ${drive_label} drive"
  else
    found_drive=0
    if (( sent )); then
      t=$(( bootsel_s < wait_s ? bootsel_s : wait_s ))
      note "waiting up to ${t} s for the ${drive_label} drive"
      if wait_for_drive "$t"; then
        found_drive=1
      else
        note "no ${drive_label} drive within ${t} s after \"bootsel\""
      fi
    fi
    if (( !found_drive )); then
      manual_bootsel_help
      note "waiting up to ${wait_s} s for the ${drive_label} drive"
      wait_for_drive "$wait_s" || die "no ${drive_label} drive appeared within ${wait_s} s"
    fi
    note "found the ${drive_label} drive: ${drive_dev}${drive_mnt:+ on ${drive_mnt}}"
  fi
fi

if [[ -n "$target" ]]; then
  wait_text="the ${build_name[$target]} USB console (${byid_dir}/${build_glob[$target]})"
else
  wait_text="a USB console of the three builds"
fi

if (( dry )) && [[ -z "$drive_dev" ]]; then
  plan "mount the drive if needed (udisksctl), check INFO_UF2.TXT, copy $(basename -- "$uf2") to it and sync"
  plan "wait up to ${wait_s} s for the drive to go away and for ${wait_text}"
  exit 0
fi

mount_drive
if (( dry )); then
  # A dry run only lists devices (lsblk, the by-id directory); it does not
  # read the drive.
  plan "check that ${drive_mnt:-<mount point>}/INFO_UF2.TXT says \"Board-ID: RPI-RP2\""
  plan "copy ${uf2} to ${drive_mnt:-<mount point of ${drive_dev}>}/ and sync"
  plan "wait up to ${wait_s} s for the drive to go away and for ${wait_text}"
  exit 0
fi
is_rp2040_bootsel "$drive_mnt" || \
  die "${drive_mnt}/INFO_UF2.TXT does not say \"Board-ID: RPI-RP2\" / \"Model: Raspberry Pi RP2\"; not copying"
note "${drive_mnt}/INFO_UF2.TXT: RP2040 boot ROM"

note "copying $(basename -- "$uf2") to ${drive_mnt}/"
cp -- "$uf2" "${drive_mnt}/"
sync
note "copied; waiting up to ${wait_s} s for the board to restart"

if consoles="$(wait_for_console "$wait_s")"; then
  note "USB console:"
  printf '%s\n' "$consoles" | sed 's/^/  /'
  exit 0
fi
if pick_drive; then
  printf 'flash.sh: the %s drive is still there after %s s; the image was not taken.\n' "$drive_label" "$wait_s" >&2
  printf '  Check that no other program or person is writing to the same drive, then run flash.sh again.\n' >&2
  exit 1
fi
others="$(list_byid 'usb-*')"
if [[ -n "$others" ]]; then
  note "${wait_text} did not appear within ${wait_s} s; serial devices now:"
  printf '%s\n' "$others" | sed 's/^/  /'
fi
if [[ -n "$target" ]]; then
  die "no ${build_name[$target]} USB console appeared within ${wait_s} s (was the UF2 a ${build_name[$target]} build?)"
fi
note "the drive went away; no console of the three builds appeared (another firmware?)"
