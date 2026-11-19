/* SPDX-License-Identifier: MIT */
/* The shared core's i2c_dw.c for the host test (main.c): the same translation
 * unit as in the build (i2c_dw.c with its i2c_dw_rp2040_dma_priv.h and the
 * OS's i2c_dw_instance.h), an entry to the static
 * i2c_dw_rp2040_read_commands() of i2c_dw_rp2040_dma_priv.h, which main.c
 * checks, and one that ends a quarantine (the board needs a restart), so
 * that the tests after the one that causes it can run.
 */

#include "i2c_dw.c"

int i2c_dw_test_read_commands(uint16_t *commands, size_t capacity, size_t length, bool restart);

int i2c_dw_test_read_commands(uint16_t *commands, size_t capacity, size_t length, bool restart)
{
	return i2c_dw_rp2040_read_commands(commands, capacity, length, restart);
}

void i2c_dw_test_clear_quarantine(void);

void i2c_dw_test_clear_quarantine(void)
{
	i2c_0_runtime.rp2040_dma_quarantined = false;
}
