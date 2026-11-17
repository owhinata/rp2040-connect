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
# The sources shared with the Zephyr build (imu_stream.c, imu_raw_*,
# imu_tick_clock.c, time_sync.c, time_offset.c, imu_cal.c, imu_cal_codec.c,
# imu_raw_mqtt.c and their headers, kv_store.h) are in common/imu, found through VPATH
# and DEPPATH, which are relative to this directory (mkdeps needs relative
# ones); their objects are made here, each compiled with the prelude of the
# shared sources (common_prelude.h of the compatibility layer, -include).
#
# Include paths of the service: the Zephyr compatibility layer
# ($(TOPDIR)/external/compat; NuttX's headers are included before it: the
# prelude in the shared sources, an include of their own in the others), the
# LSM6DSO driver's headers ($(TOPDIR)/external/lsm6dso), the port's shared
# headers ($(TOPDIR)/external/include: priorities.h, cpu_load.h),
# and common/imu and common/drivers/lsm6dso (imu_stream.c's "lsm6dso_fifo.h"
# and "lsm6dso_fifo_stats.h", imu_cal.h's "lsm6dso_cal.h").
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

# The calibration: "imu cal" and the lookup of the stored values for the
# LSM6DSO driver (imu_cal.c, with the value format in imu_cal_codec.c).

CSRCS    += imu_cal.c imu_cal_codec.c

# The MQTT transport (NANO_IMU_RAW_MQTT: imu_raw_mqtt.c with its JSON
# messages in imu_raw_status.c, on the minimal MQTT client mqtt_min.c and
# its socket layer mqtt_min_sock.c), or the stub without it.

ifeq ($(CONFIG_NANO_IMU_RAW_MQTT),y)
CSRCS    += imu_raw_mqtt.c imu_raw_status.c mqtt_min.c mqtt_min_sock.c
else
CSRCS    += imu_raw_mqtt_null.c
endif

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
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)imu
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)lsm6dso

VPATH    += :../../../common/imu
DEPPATH  += --dep-path ../../../common/imu
endif

include $(APPDIR)/Application.mk

# The prelude of the shared sources (common_prelude.h of the compatibility
# layer) before each source of common/imu.

ifneq ($(CONFIG_NANO_APP_IMU),)
IMU_COMMON_CSRCS = $(notdir $(wildcard ../../../common/imu/*.c))
$(addsuffix $(SUFFIX)$(OBJEXT),$(filter $(IMU_COMMON_CSRCS),$(CSRCS) $(MAINSRC))): \
  CFLAGS += -include $(TOPDIR)$(DELIM)external$(DELIM)compat$(DELIM)common_prelude.h

# The "imu" command's SHELL_CMD_REGISTER() in imu_stream.c makes this
# program's main(); time_sync.c is not a main source, so its
# SHELL_CMD_REGISTER(ntp, ...) makes z_shell_main_ntp(), which ../ntp's
# main() calls (Z_SHELL_CMD_NAMED_MAIN, <zephyr/shell/shell.h> of the
# compatibility layer).

time_sync.c$(SUFFIX)$(OBJEXT): CFLAGS += -DZ_SHELL_CMD_NAMED_MAIN
endif
