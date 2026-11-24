#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Write a UF2 image to the Arduino Nano RP2040 Connect through the RP2040
# USB bootloader (BOOTSEL, the RPI-RP2 drive).  No sudo.  Whichever of the
# four builds is running, its "bootsel" command is sent to enter BOOTSEL.
#
#   zephyr/script/flash.sh              write zephyr/build/usb-console/zephyr/zephyr.uf2
#   zephyr/script/flash.sh --build      source zephyr/script/setup.sh and west build first
#   zephyr/script/flash.sh other.uf2    write another UF2
#   zephyr/script/flash.sh --dry-run    only show what would be done
#
# The work is done by script/flash_uf2.sh (shared by the four builds).

set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec "${root}/script/flash_uf2.sh" --target zephyr "$@"
