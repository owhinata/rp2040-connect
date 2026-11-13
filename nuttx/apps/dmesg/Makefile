############################################################################
# nuttx/apps/dmesg/Makefile
#
# The "dmesg" builtin command: prints the RAM log (see Kconfig).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = dmesg
PRIORITY  = $(CONFIG_NANO_APP_DMESG_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_DMESG_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_DMESG)

MAINSRC   = dmesg_main.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include

include $(APPDIR)/Application.mk
