/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/rp2040_restart.c
 *
 * SPDX-License-Identifier: MIT
 *
 ****************************************************************************/

/* The board's restart into BOOTSEL of <arch/board/rp2040_restart.h>:
 * rp2040_reset_usb_boot(), the boot ROM's reset_usb_boot(), looked up in
 * the ROM's function table ('U','B': ROM_RESET_USB_BOOT of
 * arch/arm/src/rp2040/rp2040_rom.h, as rp2040_flash_mtd.c looks up the
 * flash functions).  Its arguments are those of the Pico SDK's
 * reset_usb_boot(usb_activity_gpio_pin_mask, disable_interface_mask)
 * (pico_bootrom/bootrom.c rom_reset_usb_boot(), which calls the same
 * function on the RP2040): 0, no GPIO as the mass storage's activity LED
 * (as a cold boot into BOOTSEL); 0, neither the mass storage nor the
 * PICOBOOT interface disabled.  The ROM leaves its request in the
 * watchdog's scratch registers 4..7 and restarts the chip through the
 * watchdog itself; it does not return.  Interrupts are masked first, so
 * that nothing runs between the lookup and the restart.
 *
 * Without that function in the ROM (not an RP2040) it restarts through the
 * watchdog: rp2040_watchdog_reboot() of common/chip/rp2040, the Pico SDK's
 * watchdog_reboot(0, 0, 0) on the raw registers, which the NINA
 * passthrough's restart (nina_passthrough_bus_rp2040.c) also uses.  NuttX's
 * board_reset() (SYSRESETREQ) is not used for that, so that the restart is
 * the same as the FreeRTOS build's.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>
#include <arch/board/rp2040_restart.h>

#include "arm_internal.h"
#include "rp2040_rom.h"
#include "rp2040_watchdog_reboot.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*rom_reset_usb_boot_fn)(uint32_t usb_activity_gpio_pin_mask,
                                      uint32_t disable_interface_mask);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void rp2040_reset_usb_boot(void)
{
  rom_reset_usb_boot_fn reset_usb_boot;

  up_irq_save();
  reset_usb_boot = (rom_reset_usb_boot_fn)ROM_LOOKUP(ROM_RESET_USB_BOOT);
  if (reset_usb_boot != NULL)
    {
      reset_usb_boot(0, 0);
    }

  /* No such function in the ROM (not an RP2040): a plain restart. */

  rp2040_watchdog_reboot();
}
