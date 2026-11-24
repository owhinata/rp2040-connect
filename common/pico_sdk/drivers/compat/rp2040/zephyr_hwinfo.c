/* SPDX-License-Identifier: MIT */
/* hwinfo_get_device_id() of <zephyr/drivers/hwinfo.h> (common/compat) on the
 * Pico SDK's unique board ID (pico_unique_id): the Zephyr build's
 * drivers/hwinfo/hwinfo_rpi_pico.c z_impl_hwinfo_get_device_id() without
 * CONFIG_HWINFO_RPI_PICO_CHIP_ID returns the flash's 64-bit unique ID
 * (command 0x4B, flash_get_unique_id(), with interrupts disabled), raw, in
 * the order the flash sends it.  The SDK reads the same ID with the same
 * function once before main() (a constructor of pico_unique_id/unique_id.c;
 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES is FLASH_UNIQUE_ID_SIZE_BYTES, 8, on the
 * RP2040, so the bytes are copied unchanged) and pico_get_unique_board_id()
 * returns that copy, so nothing here stops the flash (XIP) while the
 * scheduler runs, as the NuttX build reads it once at boot
 * (nuttx/drivers/compat/zephyr_hwinfo.c).  The same bytes as the USB serial
 * number and the "id" command.  Part of the Zephyr compatibility layer for
 * the OSes on the Pico SDK (common/pico_sdk/drivers/compat, the RP2040's
 * part).
 */

#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include "pico/unique_id.h"

#include <zephyr/drivers/hwinfo.h>

ssize_t hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
	pico_unique_board_id_t id;

	pico_get_unique_board_id(&id);
	if (length > sizeof(id.id)) {
		length = sizeof(id.id);
	}
	memcpy(buffer, id.id, length);
	return (ssize_t)length;
}
