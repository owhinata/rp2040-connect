############################################################################
# nuttx/apps/wifi/Makefile
#
# The stored Wi-Fi credentials (WIFI_CREDENTIALS: the copy of Zephyr's
# subsys/net/lib/wifi_credentials/ in common/compat, compiled with the
# prelude of the shared sources, with its settings backend on the copy of
# Zephyr's settings built by ../imu), the Wi-Fi profile with its reconnection thread
# (WIFI_PROFILE: wifi_profile.c, shared with the Zephyr build in
# common/imu, found through VPATH and DEPPATH relative to this directory and
# compiled with the prelude of the shared sources, common_prelude.h of the
# compatibility layer; its NuttX part is wifi_profile_os.h here; the init
# task starts the thread) and the "wifi" builtin command (APP_WIFI:
# wifi_credentials_shell.c is its main source), linked into libapps.
#
# Include paths: the Zephyr compatibility layer (common/compat/port/nuttx
# and common/compat/include, port and internal, which has
# wifi_credentials_internal.h), the port's shared headers
# (common/compat/include: app_log.h, priorities.h; their NuttX parts in
# common/compat/port/nuttx), common/shell (subcmd.h), this
# directory (wifi_profile_os.h, for the file in common/imu), common/imu
# (wifi_profile.h, kv_store.h) and common/drivers/nina/include (nina_wifi.h:
# the profile gives the driver its reconnection).
############################################################################

include $(APPDIR)/Make.defs

CSRCS = wifi_credentials.c wifi_credentials_backend_settings.c

ifeq ($(CONFIG_WIFI_PROFILE),y)
CSRCS += wifi_profile.c
endif

ifneq ($(CONFIG_APP_WIFI),)
PROGNAME  = wifi
PRIORITY  = $(CONFIG_APP_WIFI_PRIORITY)
STACKSIZE = $(CONFIG_APP_WIFI_STACKSIZE)
MODULE    = $(CONFIG_APP_WIFI)

MAINSRC   = wifi_credentials_shell.c
endif

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port$(DELIM)nuttx
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)internal
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)shell
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)wifi
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)imu

VPATH    += :../../../common/imu
DEPPATH  += --dep-path ../../../common/imu
VPATH    += :../../../common/compat
DEPPATH  += --dep-path ../../../common/compat

include $(APPDIR)/Application.mk

wifi_credentials.c$(SUFFIX)$(OBJEXT) \
wifi_credentials_backend_settings.c$(SUFFIX)$(OBJEXT): \
  CFLAGS += -include $(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port$(DELIM)nuttx$(DELIM)common_prelude.h
ifeq ($(CONFIG_WIFI_PROFILE),y)
wifi_profile.c$(SUFFIX)$(OBJEXT): CFLAGS += -include $(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port$(DELIM)nuttx$(DELIM)common_prelude.h
endif
