############################################################################
# nuttx/apps/sensor/Makefile
#
# The "sensor" builtin command: the copies of the Zephyr build's sensor
# shell (sensor_shell.c is its main source, with SHELL_CMD_REGISTER() of the
# compatibility layer making main()) and of Zephyr's sensor_shell_stream.c
# (the processing thread, which the init task starts), with the port's
# additions (sensor_shell_nuttx.c), linked into libapps.
#
# Include paths: the Zephyr compatibility layer ($(TOPDIR)/external/compat),
# the port's shared headers ($(TOPDIR)/external/include: app_log.h,
# priorities.h), subcmd.h, and common/drivers/lsm6dso (sensor_shell.c's
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
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)lsm6dso

include $(APPDIR)/Application.mk
