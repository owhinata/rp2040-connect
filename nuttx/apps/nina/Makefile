############################################################################
# nuttx/apps/nina/Makefile
#
# The "nina" builtin command (see Kconfig).  nina_request() is in the
# kernel side (common/drivers/nina, built by nuttx/drivers/nina; the flat
# build links it in); its header nina_proto.h is in common/drivers/nina.
# subcmd.c is in ../subcmd.
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = nina
PRIORITY  = $(CONFIG_NANO_APP_NINA_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_NINA_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_NINA)

MAINSRC   = nina_main.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd

include $(APPDIR)/Application.mk
