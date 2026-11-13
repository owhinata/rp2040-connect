#!/usr/bin/env bash
# Write a UF2 image to the Arduino Nano RP2040 Connect through the RP2040
# USB bootloader (BOOTSEL, the RPI-RP2 drive).  No sudo.
#
#   nuttx/script/flash.sh                 write nuttx/build/nuttx.uf2
#   nuttx/script/flash.sh --build         run build.sh first
#   nuttx/script/flash.sh other.uf2       write another UF2
#   nuttx/script/flash.sh --dry-run       only show what would be done
#
# See nuttx/README.md ("Flashing") for the steps.

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: nuttx/script/flash.sh [options] [UF2]

Write UF2 (default: nuttx/build/nuttx.uf2) to the board:
  1. If an RPI-RP2 drive (RP2040 in BOOTSEL) is present, copy to it.
  2. Otherwise, if the NuttX USB console
     (/dev/serial/by-id/usb-*NuttX_USB_Console*) is present, send "bootsel"
     to it and wait for the drive.
  3. Otherwise, explain how to enter BOOTSEL by hand and wait for the drive.
The drive is mounted with udisksctl if it is not mounted already (no sudo).
After the copy, wait for the drive to go away and for the NuttX USB console
to appear, and print its /dev/serial/by-id path.

Options:
  --build       Run nuttx/script/build.sh first.
  --wait SEC    How long to wait for the drive, and then for the console
                (default: 60).
  --dry-run     Look for the drive and the console (lsblk and
                /dev/serial/by-id only) and print what would be done; do not
                build, mount, read the drive, send, copy or wait.
  -h, --help    Show this help.
EOF
}

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# Test hooks (for running the discovery against a fake system): the lsblk and
# udisksctl commands, and the directory that holds the by-id links.
lsblk_cmd="${NANO_FLASH_LSBLK:-lsblk}"
udisksctl_cmd="${NANO_FLASH_UDISKSCTL:-udisksctl}"
byid_dir="${NANO_FLASH_BYID_DIR:-/dev/serial/by-id}"

nuttx_glob='usb-*NuttX_USB_Console*'
zephyr_glob='usb-Zephyr_Project_Zephyr_USB_Console_*'
drive_label=RPI-RP2
uf2_family=e48bff56          # RP2040

uf2=""
build=0
dry=0
wait_s=60
while (( $# )); do
  case "$1" in
    --build) build=1 ;;
    --dry-run) dry=1 ;;
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
[[ -n "$uf2" ]] || uf2="${root}/build/nuttx.uf2"

die() { printf 'flash.sh: %s\n' "$*" >&2; exit 1; }
note() { printf 'flash.sh: %s\n' "$*"; }
plan() { printf 'flash.sh: [dry-run] would %s\n' "$*"; }

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
  # A desktop session mounts the drive by itself shortly after it appears;
  # give it a few seconds before mounting it ourselves.
  for (( tries = 0; tries < 6; tries++ )); do
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

# Send "bootsel" to the NuttX console and show what comes back.  Ctrl-C
# first drops any half-typed line (and ends a command that reads the
# console, see nuttx/patches), as host/imu_host/board.py does, so that
# nothing else is executed.
send_bootsel() {
  python3 -I - "$1" <<'PY'
import os, re, select, sys, termios, time, tty

dev = sys.argv[1]
fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
out = b''
try:
    tty.setraw(fd)
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, b'\x03')
    time.sleep(0.1)
    os.write(fd, b'bootsel\r\n')
    end = time.monotonic() + 3.0
    while time.monotonic() < end and b'BOOTSEL' not in out:
        if select.select([fd], [], [], 0.2)[0]:
            try:
                data = os.read(fd, 4096)
            except OSError:
                break           # the device went away
            if not data:
                break
            out += data
finally:
    try:
        os.close(fd)
    except OSError:
        pass
text = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', out.decode('utf-8', 'replace')).replace('\r', '')
for line in text.splitlines():
    if line.strip():
        print('  | ' + line)
sys.exit(0 if 'BOOTSEL' in text else 3)
PY
}

wait_for_console() {  # timeout in s; prints the by-id path(s)
  local t=$(( $1 * 2 )) found
  while (( t-- > 0 )); do
    if ! pick_drive; then
      found="$(list_byid "$nuttx_glob")"
      if [[ -n "$found" ]]; then
        printf '%s\n' "$found"
        return 0
      fi
    fi
    sleep 0.5
  done
  return 1
}

manual_bootsel_help() {
  cat <<EOF
flash.sh: put the board into BOOTSEL by hand:
  - press the RESET button twice quickly; or, if that does not work,
  - connect REC to GND with a jumper wire, press RESET, and remove the wire
    when the ${drive_label} drive appears.
EOF
}

# Main ---------------------------------------------------------------------------------
if (( build )); then
  if (( dry )); then
    plan "run ${root}/script/build.sh"
  else
    "${root}/script/build.sh"
  fi
fi

if ! check_uf2 "$uf2"; then
  if (( dry && build )); then
    note "UF2: $uf2 (not built yet)"
  else
    die "$uf2 not found (build it with nuttx/script/build.sh, or use --build)"
  fi
fi

drive_dev="" drive_mnt=""
if pick_drive; then
  note "found the ${drive_label} drive: ${drive_dev}${drive_mnt:+ on ${drive_mnt}}"
else
  mapfile -t nuttx_consoles < <(list_byid "$nuttx_glob")
  if (( ${#nuttx_consoles[@]} > 1 )); then
    printf 'flash.sh: more than one NuttX USB console:\n' >&2
    printf '  %s\n' "${nuttx_consoles[@]}" >&2
    die "connect only the board to be written, or put it into BOOTSEL by hand"
  elif (( ${#nuttx_consoles[@]} == 1 )); then
    console="${nuttx_consoles[0]}"
    note "found the NuttX USB console: ${console}"
    if (( dry )); then
      plan "send \"bootsel\" to ${console} and wait up to ${wait_s} s for the ${drive_label} drive"
    else
      note "sending \"bootsel\""
      send_bootsel "$console" || note "no reply from bootsel seen; waiting for the drive anyway"
      wait_for_drive "$wait_s" || die "no ${drive_label} drive appeared within ${wait_s} s"
      note "found the ${drive_label} drive: ${drive_dev}${drive_mnt:+ on ${drive_mnt}}"
    fi
  else
    note "no ${drive_label} drive and no NuttX USB console (${byid_dir}/${nuttx_glob})"
    mapfile -t zephyr_consoles < <(list_byid "$zephyr_glob")
    if (( ${#zephyr_consoles[@]} )); then
      note "the Zephyr build is running (it has no bootsel command):"
      printf '  %s\n' "${zephyr_consoles[@]}"
    fi
    manual_bootsel_help
    if (( dry )); then
      plan "wait up to ${wait_s} s for the ${drive_label} drive"
    else
      note "waiting up to ${wait_s} s for the ${drive_label} drive"
      wait_for_drive "$wait_s" || die "no ${drive_label} drive appeared within ${wait_s} s"
      note "found the ${drive_label} drive: ${drive_dev}${drive_mnt:+ on ${drive_mnt}}"
    fi
  fi
fi

if (( dry )) && [[ -z "$drive_dev" ]]; then
  plan "mount the drive if needed (udisksctl), check INFO_UF2.TXT, copy $(basename -- "$uf2") to it and sync"
  plan "wait up to ${wait_s} s for the drive to go away and for ${byid_dir}/${nuttx_glob}"
  exit 0
fi

mount_drive
if (( dry )); then
  # A dry run only lists devices (lsblk, the by-id directory); it does not
  # read the drive.
  plan "check that ${drive_mnt:-<mount point>}/INFO_UF2.TXT says \"Board-ID: RPI-RP2\""
elif [[ -n "$drive_mnt" ]]; then
  is_rp2040_bootsel "$drive_mnt" || \
    die "${drive_mnt}/INFO_UF2.TXT does not say \"Board-ID: RPI-RP2\" / \"Model: Raspberry Pi RP2\"; not copying"
  note "${drive_mnt}/INFO_UF2.TXT: RP2040 boot ROM"
fi

if (( dry )); then
  plan "copy ${uf2} to ${drive_mnt:-<mount point of ${drive_dev}>}/ and sync"
  plan "wait up to ${wait_s} s for the drive to go away and for ${byid_dir}/${nuttx_glob}"
  exit 0
fi

note "copying $(basename -- "$uf2") to ${drive_mnt}/"
cp -- "$uf2" "${drive_mnt}/"
sync
note "copied; waiting up to ${wait_s} s for the board to restart"

if consoles="$(wait_for_console "$wait_s")"; then
  note "NuttX USB console:"
  printf '%s\n' "$consoles" | sed 's/^/  /'
else
  if pick_drive; then
    die "the ${drive_label} drive is still there after ${wait_s} s"
  fi
  others="$(list_byid 'usb-*')"
  if [[ -n "$others" ]]; then
    note "no NuttX USB console within ${wait_s} s; serial devices now:"
    printf '%s\n' "$others" | sed 's/^/  /'
  fi
  die "no NuttX USB console appeared within ${wait_s} s (was the UF2 a NuttX build?)"
fi
