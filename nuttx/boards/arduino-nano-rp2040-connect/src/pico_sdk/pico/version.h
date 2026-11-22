/*
 * SPDX-License-Identifier: MIT
 *
 * Stands in for the pico/version.h that the Pico SDK's CMake build
 * generates from pico_base_headers/include/pico/version.h.in.  Only the
 * division routines (divider_hardware.S, CONFIG_RP2040_PICO_DIVIDER) are
 * built from the SDK, and they do not use the version macros; boot2's build
 * (arch/arm/src/rp2040/boot2/Make.defs) uses an empty file in its place.
 */

#ifndef _PICO_VERSION_H
#define _PICO_VERSION_H

#endif
