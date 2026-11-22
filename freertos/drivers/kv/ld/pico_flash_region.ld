/* SPDX-License-Identifier: MIT */
/* The Pico SDK's pico_flash_region.ld (pico_flash_region.template.ld, read
 * inside MEMORY by memory_flash.incl), replaced for the FreeRTOS build
 * (freertos/drivers/kv/kv.cmake, pico_add_linker_script_override_path()):
 * the image may use the 16 MiB flash except its last 8 KiB, the key-value
 * area (0x10ffe000-0x10ffffff, common/compat/flash_kv.h).  An image
 * that does not fit fails to link ("region `FLASH' overflowed").  The
 * target's PICO_FLASH_SIZE_BYTES stays 16 MiB (the flash's size for the
 * SDK and the binary info); section_extra_post_end.incl checks the two
 * against each other.
 */
FLASH(rx) : ORIGIN = 0x10000000, LENGTH = 0xffe000
