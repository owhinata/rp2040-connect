############################################################################
# nuttx/apps/wifi/Makefile
#
# The stored Wi-Fi credentials (NANO_WIFI_CREDENTIALS: the copy of Zephyr's
# subsys/net/lib/wifi_credentials/ with its settings backend on /dev/config,
# settings_nvs.c), the Wi-Fi profile with its reconnection thread
# (NANO_WIFI_PROFILE: wifi_profile.c, the Zephyr build's src/wifi_profile.c;
# the init task starts the thread) and the "wifi" builtin command
# (NANO_APP_WIFI: wifi_credentials_shell.c is its main source), linked into
# libapps.
#
# Include paths: the Zephyr compatibility layer
# ($(TOPDIR)/external/compat), the port's shared headers
# ($(TOPDIR)/external/include: app_log.h, priorities.h, wifi_profile.h),
# the key-value store (kv_store.h) and subcmd.h.
############################################################################

include $(APPDIR)/Make.defs

CSRCS = wifi_credentials.c wifi_credentials_backend_settings.c settings_nvs.c

ifeq ($(CONFIG_NANO_WIFI_PROFILE),y)
CSRCS += wifi_profile.c
endif

ifneq ($(CONFIG_NANO_APP_WIFI),)
PROGNAME  = wifi
PRIORITY  = $(CONFIG_NANO_APP_WIFI_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_WIFI_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_WIFI)

MAINSRC   = wifi_credentials_shell.c
endif

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)imu
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd

include $(APPDIR)/Application.mk
