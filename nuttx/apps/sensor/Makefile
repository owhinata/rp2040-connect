############################################################################
# nuttx/apps/sensor/Makefile
#
# The "sensor" builtin command: the Zephyr build's sensor shell
# (sensor_shell.c, shared with the Zephyr build in
# common/drivers/sensor_shell, found through VPATH and DEPPATH relative to
# this directory and compiled with the prelude of the shared sources,
# common_prelude.h of the compatibility layer; it is the main source, with
# SHELL_CMD_REGISTER() of the compatibility layer making main()), the copy
# of Zephyr's sensor_shell_stream.c (the processing thread, which the init
# task starts; common/compat, also with the prelude), and the port's
# additions (sensor_shell_os.h, the NuttX part that both include first,
# and sensor_shell_nuttx.c), linked into libapps.
#
# Include paths: the Zephyr compatibility layer ($(TOPDIR)/external/compat
# and common/compat, which has the copy of Zephyr's sensor_shell.h), the
# port's shared headers (common/log: app_log.h; common/compat: priorities.h;
# their NuttX parts in $(TOPDIR)/external/include), common/shell
# (subcmd.h), this directory (sensor_shell_os.h, for the files
# in common/) and common/drivers/lsm6dso (sensor_shell.c's
# "lsm6dso_attributes.h", shared with the Zephyr build).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = sensor
PRIORITY  = $(CONFIG_NANO_APP_SENSOR_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_SENSOR_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_SENSOR)

MAINSRC   = sensor_shell.c
CSRCS     = sensor_shell_stream.c sensor_shell_nuttx.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)log
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)shell
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)sensor
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)lsm6dso

VPATH    += :../../../common/drivers/sensor_shell
DEPPATH  += --dep-path ../../../common/drivers/sensor_shell
VPATH    += :../../../common/compat
DEPPATH  += --dep-path ../../../common/compat

include $(APPDIR)/Application.mk

sensor_shell.c$(SUFFIX)$(OBJEXT) sensor_shell_stream.c$(SUFFIX)$(OBJEXT): \
  CFLAGS += -include $(TOPDIR)$(DELIM)external$(DELIM)compat$(DELIM)common_prelude.h
