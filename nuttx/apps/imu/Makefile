############################################################################
# nuttx/apps/imu/Makefile
#
# The application's modules, linked into libapps: the key-value store
# (NANO_KV_STORE) and the IMU acquisition service with its "imu" builtin
# command (NANO_APP_IMU).  imu_stream.c is the program's main source (its
# main() is the "imu" command; the thread it defines is started by the init
# task).  The copies of the Zephyr build's other sources in this directory
# that are not listed here are not built yet.
#
# Include paths of the service: the Zephyr compatibility layer
# ($(TOPDIR)/external/compat; NuttX's headers are included before it), the
# LSM6DSO driver's headers ($(TOPDIR)/external/lsm6dso), the port's shared
# headers ($(TOPDIR)/external/include: priorities.h, cpu_load.h), subcmd.h,
# and $(APPDIR)/external, through which "../drivers/lsm6dso/..." of
# imu_stream.c and imu_raw_packet.h reaches nuttx/drivers (external is a
# symlink to nuttx/apps).
############################################################################

include $(APPDIR)/Make.defs

CSRCS = kv_name.c kv_store.c

ifneq ($(CONFIG_NANO_APP_IMU),)
PROGNAME  = imu
PRIORITY  = $(CONFIG_NANO_APP_IMU_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_IMU_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_IMU)

MAINSRC   = imu_stream.c
CSRCS    += imu_raw_batch.c imu_raw_packet.c imu_raw_time.c imu_tick_clock.c
CSRCS    += imu_raw_queue.c

# Stubs, replaced by the ports of imu_raw_mqtt.c (T21) and imu_cal.c (T22).

CSRCS    += imu_raw_mqtt_null.c imu_cal_null.c

# The time synchronization (NANO_TIME_SYNC: time_sync.c, its estimate in
# time_offset.c, and the command set of the "ntp" command, nuttx/apps/ntp),
# or the stub without it.

ifeq ($(CONFIG_NANO_TIME_SYNC),y)
CSRCS    += time_sync.c time_offset.c
else
CSRCS    += time_sync_null.c
endif

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)lsm6dso
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd
CFLAGS   += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external
endif

include $(APPDIR)/Application.mk
