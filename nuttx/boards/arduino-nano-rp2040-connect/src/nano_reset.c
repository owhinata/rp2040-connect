/****************************************************************************
 * boards/arduino-nano-rp2040-connect/src/nano_reset.c
 *
 * SPDX-License-Identifier: MIT
 *
 ****************************************************************************/

/* The board's restarts of <arch/board/nano_reset.h>.
 *
 * - nano_reset_usb_boot(): the boot ROM's reset_usb_boot(), looked up in
 *   the ROM's function table ('U','B': ROM_RESET_USB_BOOT of
 *   arch/arm/src/rp2040/rp2040_rom.h, as rp2040_flash_mtd.c looks up the
 *   flash functions).  Its arguments are those of the Pico SDK's
 *   reset_usb_boot(usb_activity_gpio_pin_mask, disable_interface_mask)
 *   (pico_bootrom/bootrom.c rom_reset_usb_boot(), which calls the same
 *   function on the RP2040): 0, no GPIO as the mass storage's activity
 *   LED (as a cold boot into BOOTSEL); 0, neither the mass storage nor the
 *   PICOBOOT interface disabled.  The ROM leaves its request in the
 *   watchdog's scratch registers 4..7 and restarts the chip through the
 *   watchdog itself; it does not return.  Interrupts are masked first, so
 *   that nothing runs between the lookup and the restart.
 * - nano_watchdog_reboot(): the Pico SDK's watchdog_reboot(0, 0, 0)
 *   (hardware_watchdog/watchdog.c, _watchdog_enable()): the watchdog
 *   stopped, scratch 4 cleared (no boot vector for the ROM), every block
 *   of the power-on state machine but the oscillators selected for the
 *   watchdog's reset (PSM WDSEL), then the reset triggered at once.  The
 *   scratch registers 0..3 survive it (the NINA passthrough's request,
 *   nano_nina_passthrough.c).  NuttX's board_reset() (SYSRESETREQ) is not
 *   used for that, so that the restart is the same as the FreeRTOS
 *   build's.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>
#include <arch/board/nano_reset.h>

#include "arm_internal.h"
#include "hardware/rp2040_psm.h"
#include "hardware/rp2040_watchdog.h"
#include "rp2040_rom.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All the blocks of the power-on state machine (PSM_WDSEL_BITS of the Pico
 * SDK, ROSC in bit 0 up to PROC1 in bit 16).
 */

#define NANO_PSM_ALL  ((RP2040_PSM_PROC1 << 1) - 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*rom_reset_usb_boot_fn)(uint32_t usb_activity_gpio_pin_mask,
                                      uint32_t disable_interface_mask);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void nano_reset_usb_boot(void)
{
  rom_reset_usb_boot_fn reset_usb_boot;

  up_irq_save();
  reset_usb_boot = (rom_reset_usb_boot_fn)ROM_LOOKUP(ROM_RESET_USB_BOOT);
  if (reset_usb_boot != NULL)
    {
      reset_usb_boot(0, 0);
    }

  /* No such function in the ROM (not an RP2040): a plain restart. */

  nano_watchdog_reboot();
}

void nano_watchdog_reboot(void)
{
  up_irq_save();
  clrbits_reg32(RP2040_WATCHDOG_CTRL_ENABLE, RP2040_WATCHDOG_CTRL);
  putreg32(0, RP2040_WATCHDOG_SCRATCH4);
  setbits_reg32(NANO_PSM_ALL & ~(RP2040_PSM_ROSC | RP2040_PSM_XOSC),
                RP2040_PSM_WDSEL);
  setbits_reg32(RP2040_WATCHDOG_CTRL_TRIGGER, RP2040_WATCHDOG_CTRL);

  for (; ; )
    {
    }
}
