############################################################################
# nuttx/apps/nina/Makefile
#
# The "nina" builtin command (see Kconfig).  nina_request() is in the
# kernel side (common/drivers/nina, built by nuttx/drivers/nina; the flat
# build links it in); its header nina_proto.h is in common/drivers/nina.
# subcmd.c is in common/shell (built by ../subcmd), and so is subcmd.h.
# "nina passthrough confirm" calls the board's
# nano_nina_passthrough_request() (<arch/board/nano_nina_passthrough.h>).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = nina
PRIORITY  = $(CONFIG_NANO_APP_NINA_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_NINA_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_NINA)

MAINSRC   = nina_main.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)shell

# "nina passthrough" (CONFIG_NANO_NINA_PASSTHROUGH) reads the mode's
# nina_passthrough_logic.h (common/drivers/nina, above), which includes
# <zephyr/shell/shell.h> of the Zephyr compatibility layer: its NuttX part
# ($(TOPDIR)/external/compat) and its shared part (common/compat).

ifeq ($(CONFIG_NANO_NINA_PASSTHROUGH),y)
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat
endif

include $(APPDIR)/Application.mk
