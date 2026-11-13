############################################################################
# nuttx/apps/kv/Makefile
#
# The "kv" builtin command (see Kconfig).  kv_store.c and kv_name.c are in
# ../imu, subcmd.c in ../subcmd (linked into libapps).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = kv
PRIORITY  = $(CONFIG_NANO_APP_KV_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_KV_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_KV)

MAINSRC   = kv_main.c
CSRCS     = kv_hex.c

CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)imu
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd

include $(APPDIR)/Application.mk
