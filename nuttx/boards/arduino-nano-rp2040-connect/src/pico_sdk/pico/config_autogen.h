/*
 * SPDX-License-Identifier: MIT
 *
 * Stands in for the pico/config_autogen.h that the Pico SDK's CMake build
 * generates (it includes the board header and the PICO_CONFIG_HEADER
 * files).  Only the division routines (divider_hardware.S,
 * CONFIG_NANO_HW_DIVIDER) are built from the SDK.  Their options are
 * given here with the SDK's defaults, which divider_hardware.S tests with
 * #if without defining them (warnings under -Wundef):
 *
 * - PICO_DIVIDER_DISABLE_INTERRUPTS 0: a division that finds the divider
 *   in use (an interrupted division) saves and restores it, instead of
 *   each division disabling interrupts; the context switch moves the
 *   divider with the task (CONFIG_RP2040_HW_DIVIDER).
 * - PICO_DIVIDER_IN_RAM 0: the routines are in flash (.text).
 *
 * PICO_DIVIDER_CALL_IDIV0 and PICO_DIVIDER_CALL_LDIV0 keep their defaults
 * (1, set in divider_hardware.S): a division by zero calls __aeabi_idiv0
 * or __aeabi_ldiv0 (libgcc's).  boot2's build
 * (arch/arm/src/rp2040/boot2/Make.defs) uses an empty file in this one's
 * place.
 */

#ifndef _PICO_CONFIG_AUTOGEN_H
#define _PICO_CONFIG_AUTOGEN_H

#define PICO_DIVIDER_DISABLE_INTERRUPTS 0
#define PICO_DIVIDER_IN_RAM             0

#endif
