############################################################################
# nuttx/apps/sensor/Makefile
#
# The "sensor" builtin command: the Zephyr build's sensor shell
# (sensor_shell.c, shared with the Zephyr build in
# common/shell, found through VPATH and DEPPATH relative to
# this directory and compiled with the prelude of the shared sources,
# common_prelude.h of the compatibility layer; it is the main source, with
# SHELL_CMD_REGISTER() of the compatibility layer making main()), the copy
# of Zephyr's sensor_shell_stream.c (the processing thread, which the init
# task starts; common/compat, also with the prelude), and the port's
# additions (sensor_shell_os.h, the NuttX part that both include first,
# and sensor_shell_nuttx.c), linked into libapps.
#
# Include paths: the Zephyr compatibility layer (common/compat/port/nuttx
# and common/compat/include, which has the copy of Zephyr's sensor_shell.h and the
# LSM6DSO driver's <zephyr/drivers/sensor/lsm6dso.h>, which sensor_shell.c
# reads), the port's shared headers (common/compat/include: app_log.h,
# priorities.h; their NuttX parts in common/compat/port/nuttx),
# common/shell (subcmd.h) and this directory (sensor_shell_os.h, for the
# files in common/).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = sensor
PRIORITY  = $(CONFIG_APP_SENSOR_PRIORITY)
STACKSIZE = $(CONFIG_APP_SENSOR_STACKSIZE)
MODULE    = $(CONFIG_APP_SENSOR)

MAINSRC   = sensor_shell.c
CSRCS     = sensor_shell_stream.c sensor_shell_nuttx.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port$(DELIM)nuttx
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)shell
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)sensor

VPATH    += :../../../common/shell
DEPPATH  += --dep-path ../../../common/shell
VPATH    += :../../../common/compat
DEPPATH  += --dep-path ../../../common/compat

include $(APPDIR)/Application.mk

sensor_shell.c$(SUFFIX)$(OBJEXT) sensor_shell_stream.c$(SUFFIX)$(OBJEXT): \
  CFLAGS += -include $(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port$(DELIM)nuttx$(DELIM)common_prelude.h
