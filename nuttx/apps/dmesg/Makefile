############################################################################
# nuttx/apps/dmesg/Makefile
#
# The "dmesg" builtin command: prints the RAM log (see Kconfig).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = dmesg
PRIORITY  = $(CONFIG_APP_DMESG_PRIORITY)
STACKSIZE = $(CONFIG_APP_DMESG_STACKSIZE)
MODULE    = $(CONFIG_APP_DMESG)

MAINSRC   = dmesg_main.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port$(DELIM)nuttx
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port

include $(APPDIR)/Application.mk
