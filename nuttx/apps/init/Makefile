############################################################################
# nuttx/apps/init/Makefile
#
# The init task (CONFIG_INIT_ENTRYPOINT="nano_init_main").  Built as a plain
# library source (CSRCS, no MAINSRC/PROGNAME), so it is linked into libapps
# but not registered as a builtin: the shell cannot start a second init.
# It uses nshlib's internal headers (nsh.h, nsh_console.h) to run an nsh
# session without the login greeting (see nano_init.c).  app_log.h and
# priorities.h come from nuttx/drivers/include ($(TOPDIR)/external/include).
# It starts the IMU service's thread (zephyr_compat.h of
# $(TOPDIR)/external/compat, without the layer's Zephyr headers) and calls
# imu_cal_init() and time_sync_init() (../imu; imu_cal.h includes
# lsm6dso_cal.h of $(TOPDIR)/external/lsm6dso).  nano_telnet.c runs the
# Telnet shell (CONFIG_NANO_INIT_TELNET).
############################################################################

include $(APPDIR)/Make.defs

CSRCS = nano_init.c nano_telnet.c

CFLAGS += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)nshlib
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)imu
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)lsm6dso

include $(APPDIR)/Application.mk
