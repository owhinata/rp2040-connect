############################################################################
# nuttx/apps/init/Makefile
#
# The init task (CONFIG_INIT_ENTRYPOINT="nano_init_main").  Built as a plain
# library source (CSRCS, no MAINSRC/PROGNAME), so it is linked into libapps
# but not registered as a builtin: the shell cannot start a second init.
# It uses nshlib's internal headers (nsh.h, nsh_console.h) to run an nsh
# session without the login greeting (see nano_init.c).  app_log.h comes
# from common/log and priorities.h from common/compat, their NuttX parts
# (app_log_os.h, priorities_os.h) from nuttx/drivers/include
# ($(TOPDIR)/external/include).
# It starts the NINA's Wi-Fi threads (nina_net.h of common/drivers/nina/port/nuttx)
# and the IMU service's thread (zephyr_compat.h of common/compat, without
# the layer's Zephyr headers) and calls
# imu_cal_init() and time_sync_init() (../imu, and imu_cal.h of common/imu,
# which includes the LSM6DSO driver's <zephyr/drivers/sensor/lsm6dso.h> of
# common/compat).  nano_telnet.c
# runs the Telnet shell (CONFIG_NANO_INIT_TELNET).  When the board's NINA
# passthrough mode was asked for (CONFIG_NINA_PASSTHROUGH), it runs
# that mode instead of all this (nina_passthrough.h of
# common/drivers/nina).
############################################################################

include $(APPDIR)/Make.defs

CSRCS = nano_init.c nano_telnet.c

CFLAGS += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)nshlib
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)log
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)imu
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)imu
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina$(DELIM)port$(DELIM)nuttx
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina$(DELIM)include

include $(APPDIR)/Application.mk
