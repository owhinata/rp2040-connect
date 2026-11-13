/* SPDX-License-Identifier: Apache-2.0 */
/* hwinfo_get_device_id() (<zephyr/drivers/hwinfo.h>) on the board's chip
 * ID: the Zephyr build's drivers/hwinfo/hwinfo_rpi_pico.c
 * z_impl_hwinfo_get_device_id() (CONFIG_HWINFO_RPI_PICO_CHIP_ID off: the
 * flash's unique ID, read with flash_get_unique_id()), with the read
 * itself done once at boot by the board (src/nano_chipid.c, the same 0x4B
 * command with interrupts disabled).  Part of the small Zephyr
 * compatibility layer of the port (nuttx/drivers/compat).
 */

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <arch/board/board.h>

#include <zephyr/drivers/hwinfo.h>

ssize_t hwinfo_get_device_id(uint8_t *buffer, size_t length)
{
  uint8_t id[NANO_CHIP_ID_SIZE];
  int ret = nano_chip_id(id);

  if (ret < 0)
    {
      return ret;
    }

  if (length > sizeof(id))
    {
      length = sizeof(id);
    }

  memcpy(buffer, id, length);
  return (ssize_t)length;
}
