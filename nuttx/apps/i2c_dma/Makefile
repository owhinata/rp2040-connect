############################################################################
# nuttx/apps/i2c_dma/Makefile
#
# The "i2c_dma" builtin command (see Kconfig).  The state it reads and sets
# is in the I2C0 core (common/drivers/i2c_dw/nano_i2c.c, which the flat
# build links in through the board); its header nano_i2c_dma.h is in
# common/drivers/i2c_dw.  subcmd.c is in ../subcmd.
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = i2c_dma
PRIORITY  = $(CONFIG_NANO_APP_I2C_DMA_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_I2C_DMA_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_I2C_DMA)

MAINSRC   = i2c_dma_main.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)i2c_dw
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd

include $(APPDIR)/Application.mk
