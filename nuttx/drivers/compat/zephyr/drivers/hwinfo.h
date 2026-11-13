/* SPDX-License-Identifier: Apache-2.0 */
/* hwinfo_get_device_id() of Zephyr's <zephyr/drivers/hwinfo.h>, on the
 * board's chip ID (nano_chip_id(), compat/zephyr_hwinfo.c).  Part of the
 * small Zephyr compatibility layer of the port (nuttx/drivers/compat, see
 * nuttx/drivers/README.md).
 */

#ifndef __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_HWINFO_H
#define __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_HWINFO_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

/* Copies up to length bytes of the device ID to buffer and returns how
 * many (at most 8).  The ID is what the Zephyr build's RP2040 driver
 * returns (drivers/hwinfo/hwinfo_rpi_pico.c without
 * CONFIG_HWINFO_RPI_PICO_CHIP_ID): the external flash's 64-bit unique ID
 * (command 0x4B), raw, in the order the flash sends it; the same bytes as
 * the USB serial number.  A negative errno if the board has not read the
 * ID yet (Zephyr's driver cannot fail).
 */

ssize_t hwinfo_get_device_id(uint8_t *buffer, size_t length);

#endif /* __NUTTX_DRIVERS_COMPAT_ZEPHYR_DRIVERS_HWINFO_H */
