############################################################################
# nuttx/apps/i2c_dma/Makefile
#
# The "i2c_dma" builtin command (see Kconfig): the command of the Zephyr
# and FreeRTOS builds, common/drivers/i2c_dw/i2c_dma_shell.c (found through
# VPATH and DEPPATH, which are relative to this directory, as mkdeps needs
# them; its object is made here).  It is the main source: the compatibility
# layer's SHELL_CMD_REGISTER() makes main(), which runs the command on the
# Zephyr shell's command handling (z_shell_cmd_main() of
# common/shell/zshell.c, built by ../subcmd), and it is compiled with the
# prelude of the shared sources (common_prelude.h of the compatibility
# layer, -include).  The state it reads and sets is in the I2C0 core
# (common/drivers/i2c_dw/nano_i2c.c, which the flat build links in through
# the board); its header nano_i2c_dma.h is in common/drivers/i2c_dw.
#
# Include paths: the Zephyr compatibility layer ($(TOPDIR)/external/compat
# and common/compat: <zephyr/shell/shell.h>) and common/drivers/i2c_dw.
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = i2c_dma
PRIORITY  = $(CONFIG_NANO_APP_I2C_DMA_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_I2C_DMA_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_I2C_DMA)

MAINSRC   = i2c_dma_shell.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)i2c_dw

VPATH    += :../../../common/drivers/i2c_dw
DEPPATH  += --dep-path ../../../common/drivers/i2c_dw

include $(APPDIR)/Application.mk

i2c_dma_shell.c$(SUFFIX)$(OBJEXT): \
  CFLAGS += -include $(TOPDIR)$(DELIM)external$(DELIM)compat$(DELIM)common_prelude.h
