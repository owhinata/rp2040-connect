#!/usr/bin/env bash
# Write a UF2 image to the Arduino Nano RP2040 Connect through the RP2040
# USB bootloader (BOOTSEL, the RPI-RP2 drive).  No sudo.
#
#   freertos/script/flash.sh              write freertos/build/usb-console/usb-console.uf2
#   freertos/script/flash.sh --build      run build.sh first
#   freertos/script/flash.sh other.uf2    write another UF2
#
# See freertos/README.md ("Flashing").

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: freertos/script/flash.sh [options] [UF2]

Write UF2 (default: freertos/build/usb-console/usb-console.uf2) to the board:
  1. If an RPI-RP2 drive (RP2040 in BOOTSEL) is present, copy to it.
  2. Otherwise, if the FreeRTOS USB console is present, send it the
     "bootsel" command and wait for the drive.
  3. Otherwise, explain how to enter BOOTSEL by hand and wait for the drive.
The drive is mounted with udisksctl if it is not mounted already (no sudo).
After the copy, wait for the FreeRTOS USB console to appear and print its
/dev/serial/by-id path.

Options:
  --build       Run freertos/script/build.sh first.
  --wait SEC    How long to wait for the drive, and then for the console
                (default: 60).
  -h, --help    Show this help.
EOF
}

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
byid_dir=/dev/serial/by-id
console_glob='usb-*FreeRTOS_USB_Console*'   # USBD_PRODUCT of app/usb-console
drive_label=RPI-RP2

uf2=""
build=0
wait_s=60
while (( $# )); do
  case "$1" in
    --build) build=1 ;;
    --wait) shift; wait_s="${1:?--wait needs a number of seconds}" ;;
    --wait=*) wait_s="${1#--wait=}" ;;
    -h|--help) usage; exit 0 ;;
    -*) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
    *)
      [[ -z "$uf2" ]] || { printf 'Only one UF2 can be given\n' >&2; exit 2; }
      uf2="$1"
      ;;
  esac
  shift
done
[[ "$wait_s" =~ ^[0-9]+$ ]] || { printf -- '--wait needs a number of seconds\n' >&2; exit 2; }
[[ -n "$uf2" ]] || uf2="${root}/build/usb-console/usb-console.uf2"

die() { printf 'flash.sh: %s\n' "$*" >&2; exit 1; }
note() { printf 'flash.sh: %s\n' "$*"; }

# Print "<partition> <mount point>" for every partition labelled RPI-RP2 on a
# USB disk; the mount point is missing when the partition is not mounted.
find_drives() {
  lsblk -nr -p -o NAME,TYPE,LABEL,TRAN,MOUNTPOINT 2>/dev/null | \
    awk -v label="$drive_label" '
      $2 == "disk" { usb = ($3 == "usb" || $4 == "usb") }
      $2 == "part" && $3 == label && usb { print $1, $4 }'
}

# Sets drive_dev and drive_mnt when exactly one drive is present.  Returns 1
# when there is none, and stops when there are several.
pick_drive() {
  local lines=()
  mapfile -t lines < <(find_drives)
  (( ${#lines[@]} )) || return 1
  if (( ${#lines[@]} > 1 )); then
    printf 'flash.sh: more than one %s drive:\n' "$drive_label" >&2
    printf '  %s\n' "${lines[@]}" >&2
    die "connect only the board to be written"
  fi
  drive_dev="${lines[0]%% *}"
  drive_mnt=""
  [[ "${lines[0]}" == *" "* ]] && drive_mnt="$(printf '%b' "${lines[0]#* }")"
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

find_consoles() {
  local f
  shopt -s nullglob
  for f in "$byid_dir"/$console_glob; do
    printf '%s\n' "$f"
  done
  shopt -u nullglob
}

# Main ---------------------------------------------------------------------------------
if (( build )); then
  "${root}/script/build.sh"
fi

# Every 512-byte block of a UF2 starts with the magic numbers 0x0a324655
# 0x9e5d5157; flag 0x2000 says that word 7 is the family ID (RP2040: e48bff56).
[[ -f "$uf2" ]] || die "$uf2 not found (build it with freertos/script/build.sh, or use --build)"
read -r -a w < <(od -An -v -tx4 -w32 --endian=little -N32 -- "$uf2")
[[ "${w[0]:-}" == 0a324655 && "${w[1]:-}" == 9e5d5157 ]] || die "$uf2: not a UF2 file"
(( (16#${w[2]} & 0x2000) != 0 )) && [[ "${w[7]}" == e48bff56 ]] || \
  die "$uf2: not a UF2 for the RP2040 (family 0x${w[7]})"
note "UF2: $uf2 ($(( $(stat -c %s -- "$uf2") / 512 )) blocks)"

drive_dev="" drive_mnt=""
if ! pick_drive; then
  mapfile -t consoles < <(find_consoles)
  if (( ${#consoles[@]} == 1 )); then
    note "sending \"bootsel\" to ${consoles[0]}"
    stty -F "${consoles[0]}" raw -echo || die "cannot open ${consoles[0]} (in use by a terminal?)"
    # Ctrl-C drops whatever is on the command line.
    printf '\003bootsel\r' > "${consoles[0]}"
  else
    (( ${#consoles[@]} == 0 )) || note "more than one FreeRTOS USB console; not choosing one"
    cat <<EOF
flash.sh: no ${drive_label} drive. Put the board into BOOTSEL:
  - with the FreeRTOS firmware running: the "bootsel" command; or
  - press the RESET button twice quickly; or, if that does not work,
  - connect REC to GND with a jumper wire, press RESET, and remove the wire
    when the ${drive_label} drive appears.
EOF
  fi
  note "waiting up to ${wait_s} s for the ${drive_label} drive"
  wait_for_drive "$wait_s" || die "no ${drive_label} drive appeared within ${wait_s} s"
fi
note "found the ${drive_label} drive: ${drive_dev}"

# A desktop session mounts the drive by itself shortly after it appears;
# give it a few seconds before mounting it ourselves.
for (( i = 0; i < 6 && ${#drive_mnt} == 0; i++ )); do
  sleep 0.5
  pick_drive || die "the ${drive_label} drive went away"
done
if [[ -z "$drive_mnt" ]]; then
  note "${drive_dev} is not mounted; mounting it with udisksctl"
  udisksctl mount -b "$drive_dev" --no-user-interaction || true
  for (( i = 0; i < 10 && ${#drive_mnt} == 0; i++ )); do
    sleep 0.5
    pick_drive || die "the ${drive_label} drive went away"
  done
  [[ -n "$drive_mnt" ]] || die "could not mount ${drive_dev}; mount it by hand and run flash.sh again"
fi

# INFO_UF2.TXT of the RP2040 boot ROM says "Board-ID: RPI-RP2".
grep -q 'Board-ID: RPI-RP2' "${drive_mnt}/INFO_UF2.TXT" 2>/dev/null || \
  die "${drive_mnt}/INFO_UF2.TXT does not say \"Board-ID: RPI-RP2\"; not copying"

note "copying $(basename -- "$uf2") to ${drive_mnt}/"
cp -- "$uf2" "${drive_mnt}/"
sync
note "copied; waiting up to ${wait_s} s for the board to restart"

for (( t = wait_s * 2; t > 0; t-- )); do
  if ! pick_drive; then
    found="$(find_consoles)"
    if [[ -n "$found" ]]; then
      note "FreeRTOS USB console:"
      printf '%s\n' "$found" | sed 's/^/  /'
      exit 0
    fi
  fi
  sleep 0.5
done
die "no FreeRTOS USB console appeared within ${wait_s} s (was the UF2 a FreeRTOS build?)"
