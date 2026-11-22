############################################################################
# nuttx/apps/ntp/Makefile
#
# The "ntp" builtin command (see Kconfig).  Its subcommands and their
# handler are in common/imu/time_sync.c (z_shell_main_ntp(), built by
# ../imu), the Zephyr shell's command handling in ../subcmd/zshell.c (both
# linked into libapps).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = ntp
PRIORITY  = $(CONFIG_APP_NTP_PRIORITY)
STACKSIZE = $(CONFIG_APP_NTP_STACKSIZE)
MODULE    = $(CONFIG_APP_NTP)

MAINSRC   = ntp_main.c

include $(APPDIR)/Application.mk
