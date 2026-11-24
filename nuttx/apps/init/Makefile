############################################################################
# nuttx/apps/init/Makefile
#
# The init task (CONFIG_INIT_ENTRYPOINT="rp2040_init_main").  Built as a plain
# library source (CSRCS, no MAINSRC/PROGNAME), so it is linked into libapps
# but not registered as a builtin: the shell cannot start a second init.
# It uses nshlib's internal headers (nsh.h, nsh_console.h) to run an nsh
# session without the login greeting (see rp2040_init.c).  app_log.h comes
# and priorities.h from common/compat/include, their NuttX parts
# (app_log_os.h, priorities_os.h) from nuttx/drivers/include
# ($(TOPDIR)/external/include).
# It calls the application's start shared with the other builds
# (common/imu/app_main.h: app_main.c, app_init.c, found through VPATH and
# compiled with the prelude of the shared sources, common_prelude.h,
# -include): the Wi-Fi credentials (../wifi), the threads (the NINA's,
# nina_net.h of common/drivers/nina/port/nuttx; the ones of
# K_THREAD_DEFINE(), zephyr_compat.h of common/compat) and the modules
# (../imu, ../wifi; imu_cal.h of common/imu includes the LSM6DSO driver's
# <zephyr/drivers/sensor/lsm6dso.h> of common/compat).  rp2040_telnet.c
# runs the Telnet shell (CONFIG_INIT_TELNET).  When the board's NINA
# passthrough mode was asked for (CONFIG_NINA_PASSTHROUGH), it runs
# that mode instead of all this (nina_passthrough.h of
# common/drivers/nina).
############################################################################

include $(APPDIR)/Make.defs

CSRCS = rp2040_init.c rp2040_telnet.c app_main.c app_init.c

CFLAGS += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)nshlib
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)imu
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)include
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)imu
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina$(DELIM)port$(DELIM)nuttx
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina$(DELIM)include

VPATH   += :../../../common/imu
DEPPATH += --dep-path ../../../common/imu

include $(APPDIR)/Application.mk

app_main.c$(SUFFIX)$(OBJEXT) app_init.c$(SUFFIX)$(OBJEXT): \
  CFLAGS += -include $(TOPDIR)$(DELIM)external$(DELIM)compat$(DELIM)common_prelude.h
