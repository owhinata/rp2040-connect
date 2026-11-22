############################################################################
# nuttx/apps/nina/Makefile
#
# The "nina" builtin command (see Kconfig).  nina_proto_request() is in the
# kernel side (common/drivers/nina, built by nuttx/drivers/nina; the flat
# build links it in); its header nina_proto.h is in
# common/drivers/nina/internal.
# subcmd.c is in common/shell (built by ../subcmd), and so is subcmd.h.
# "nina passthrough confirm" calls nina_passthrough_request() of the
# NINA driver (nina_passthrough.h, also in common/drivers/nina).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = nina
PRIORITY  = $(CONFIG_APP_NINA_PRIORITY)
STACKSIZE = $(CONFIG_APP_NINA_STACKSIZE)
MODULE    = $(CONFIG_APP_NINA)

MAINSRC   = nina_main.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)shell

include $(APPDIR)/Application.mk
