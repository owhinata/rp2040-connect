############################################################################
# nuttx/apps/kv/Makefile
#
# The "kv" builtin command (see Kconfig).  Its subcommands and their
# handlers are in common/imu/kv_store.c (z_shell_main_kv(), built by
# ../imu), the Zephyr shell's command handling in common/shell/zshell.c
# (built by ../subcmd; both linked into libapps).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = kv
PRIORITY  = $(CONFIG_APP_KV_PRIORITY)
STACKSIZE = $(CONFIG_APP_KV_STACKSIZE)
MODULE    = $(CONFIG_APP_KV)

MAINSRC   = kv_main.c

include $(APPDIR)/Application.mk
