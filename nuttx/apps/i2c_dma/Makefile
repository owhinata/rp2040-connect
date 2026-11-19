############################################################################
# nuttx/apps/i2c_dma/Makefile
#
# The "i2c_dma" builtin command (see Kconfig): the command of the Zephyr
# and FreeRTOS builds, common/shell/i2c_dma_shell.c (found through
# VPATH and DEPPATH, which are relative to this directory, as mkdeps needs
# them; its object is made here).  It is the main source: the compatibility
# layer's SHELL_CMD_REGISTER() makes main(), which runs the command on the
# Zephyr shell's command handling (z_shell_cmd_main() of
# common/shell/zshell.c, built by ../subcmd), and it is compiled with the
# prelude of the shared sources (common_prelude.h of the compatibility
# layer, -include).  The state it reads and sets is in the I2C0 core
# (common/drivers/i2c_dw/i2c_dw.c, which the flat build links in through
# the board); its header is i2c_dw_rp2040_dma.h there, which the command
# includes as <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h> (common/compat).
#
# Include paths: the Zephyr compatibility layer ($(TOPDIR)/external/compat
# and common/compat: <zephyr/shell/shell.h>,
# <zephyr/drivers/i2c/i2c_dw_rp2040_dma.h>).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = i2c_dma
PRIORITY  = $(CONFIG_I2C_DW_RP2040_DMA_SHELL_PRIORITY)
STACKSIZE = $(CONFIG_I2C_DW_RP2040_DMA_SHELL_STACKSIZE)
MODULE    = $(CONFIG_I2C_DW_RP2040_DMA_SHELL)

MAINSRC   = i2c_dma_shell.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat

VPATH    += :../../../common/shell
DEPPATH  += --dep-path ../../../common/shell

include $(APPDIR)/Application.mk

i2c_dma_shell.c$(SUFFIX)$(OBJEXT): \
  CFLAGS += -include $(TOPDIR)$(DELIM)external$(DELIM)compat$(DELIM)common_prelude.h
