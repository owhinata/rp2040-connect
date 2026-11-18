#!/usr/bin/env bash
# Write a UF2 image to the Arduino Nano RP2040 Connect through the RP2040
# USB bootloader (BOOTSEL, the RPI-RP2 drive).  No sudo.  Whichever of the
# three builds is running, its "bootsel" command is sent to enter BOOTSEL.
#
#   freertos/script/flash.sh              write freertos/build/usb-console/usb-console.uf2
#   freertos/script/flash.sh --build      run build.sh first
#   freertos/script/flash.sh other.uf2    write another UF2
#   freertos/script/flash.sh --dry-run    only show what would be done
#
# The work is done by script/flash_uf2.sh (shared by the three builds).

set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${root}/script/flash_uf2.sh" --target freertos "$@"
