############################################################################
# nuttx/apps/ntp/Makefile
#
# The "ntp" builtin command (see Kconfig).  Its subcommands and their
# handler are in ../imu/time_sync.c (ntp_root), subcmd.c in ../subcmd (both
# linked into libapps).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = ntp
PRIORITY  = $(CONFIG_NANO_APP_NTP_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_NTP_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_NTP)

MAINSRC   = ntp_main.c

CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd

include $(APPDIR)/Application.mk
