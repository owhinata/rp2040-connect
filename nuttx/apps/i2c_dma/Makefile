############################################################################
# nuttx/apps/i2c_dma/Makefile
#
# The "i2c_dma" builtin command (see Kconfig).  The state it reads and sets
# is in the board's I2C driver (the flat build links it in); its header
# nano_i2c_dma.h is in $(TOPDIR)/external/include.  subcmd.c is in
# ../subcmd.
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = i2c_dma
PRIORITY  = $(CONFIG_NANO_APP_I2C_DMA_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_I2C_DMA_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_I2C_DMA)

MAINSRC   = i2c_dma_main.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd

include $(APPDIR)/Application.mk
