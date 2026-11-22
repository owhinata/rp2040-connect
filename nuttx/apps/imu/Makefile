############################################################################
# nuttx/apps/imu/Makefile
#
# The application's modules, linked into libapps: the key-value store
# (KV_STORE) and the IMU acquisition service with its "imu" builtin
# command (APP_IMU).  imu_stream.c is the program's main source (its
# main() is the "imu" command; the thread it defines is started by the init
# task).
#
# The key-value store is the Zephyr build's: common/imu/kv_store.c (the
# store and the "kv" command's subcommands, z_shell_main_kv(), which
# ../kv's main() calls; with "kv wipe" of common/compat/kv_store_wipe.c)
# on the copies of Zephyr's settings and NVS (common/compat/subsys:
# settings.c, settings_store.c, settings_init.c, settings_nvs.c, nvs.c,
# crc8_sw.c; each read with the Zephyr build's Kconfig values,
# common/compat/zephyr_kv_config.h, and without -Wundef, since they test
# unset options with #if as Zephyr does), the flash device and flash map of
# the key-value area (common/compat/zephyr_flash_kv.c, on
# common/chip/rp2040/rp2040_flash_kv.c and the board's
# rp2040_flash_kv_os.c) and the hex conversions of the "kv" command (the
# copy of Zephyr's lib/utils/hex.c, common/compat/lib/utils).  The Wi-Fi
# credentials (../wifi) use the same settings.
#
# The sources shared with the Zephyr build (imu_stream.c, imu_raw_*,
# imu_tick_clock.c, time_sync.c, time_offset.c, imu_cal.c, imu_cal_codec.c,
# imu_raw_mqtt.c, kv_store.c and their headers), the null implementations
# of the transport and of the time synchronization (imu_raw_mqtt_null.c,
# time_sync_null.c, for the builds without them), and the port's MQTT
# client for the builds on the compatibility layer (mqtt_min.c,
# mqtt_min.h, and its socket layer on the layer's zsock_*() calls,
# mqtt_min_sock.c, shared with the FreeRTOS build), are in common/imu; the others named above in common/compat.  They
# are found through VPATH and DEPPATH, which are relative to this directory
# (mkdeps needs relative ones); their objects are made here, each compiled
# with the prelude of the shared sources (common_prelude.h of the
# compatibility layer, -include).
#
# Include paths: the Zephyr compatibility layer
# ($(TOPDIR)/external/compat, then common/compat; NuttX's headers are
# included before it: the prelude in the shared sources, an include of
# their own in the others), the copied settings' own headers
# (common/compat/subsys/settings/include: <settings/settings_nvs.h>;
# common/compat/subsys/settings/src: settings_priv.h for
# kv_store_wipe.c), the port's shared headers (common/compat:
# priorities.h, cpu_load.h, flash_kv.h, kv_store_os.h; common/compat/log:
# app_log.h; their NuttX parts in $(TOPDIR)/external/include), common/imu
# (the LSM6DSO driver's <zephyr/drivers/sensor/lsm6dso.h>, which the
# service reads, is common/compat's), and
# common/drivers/nina (<nina_dns.h> of the NINA core, which the layer's
# <zephyr/net/socket.h> reads).
############################################################################

include $(APPDIR)/Make.defs

CSRCS  = kv_store.c kv_store_wipe.c zephyr_flash_kv.c hex.c
KV_ZEPHYR_CSRCS = settings.c settings_store.c settings_init.c settings_nvs.c nvs.c crc8_sw.c
CSRCS += $(KV_ZEPHYR_CSRCS)

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)log
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)subsys$(DELIM)settings$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)subsys$(DELIM)settings$(DELIM)src
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)imu

VPATH    += :../../../common/imu
DEPPATH  += --dep-path ../../../common/imu
VPATH    += :../../../common/compat
DEPPATH  += --dep-path ../../../common/compat
VPATH    += :../../../common/compat/lib/utils
DEPPATH  += --dep-path ../../../common/compat/lib/utils
VPATH    += :../../../common/compat/subsys/settings/src
DEPPATH  += --dep-path ../../../common/compat/subsys/settings/src
VPATH    += :../../../common/compat/subsys/kvss/nvs
DEPPATH  += --dep-path ../../../common/compat/subsys/kvss/nvs
VPATH    += :../../../common/compat/subsys/crc
DEPPATH  += --dep-path ../../../common/compat/subsys/crc

ifneq ($(CONFIG_APP_IMU),)
PROGNAME  = imu
PRIORITY  = $(CONFIG_APP_IMU_PRIORITY)
STACKSIZE = $(CONFIG_APP_IMU_STACKSIZE)
MODULE    = $(CONFIG_APP_IMU)

MAINSRC   = imu_stream.c
CSRCS    += imu_raw_batch.c imu_raw_packet.c imu_raw_time.c imu_tick_clock.c
CSRCS    += imu_raw_queue.c

# The calibration: "imu cal" and the lookup of the stored values for the
# LSM6DSO driver (imu_cal.c, with the value format in imu_cal_codec.c).

CSRCS    += imu_cal.c imu_cal_codec.c

# The MQTT transport (IMU_RAW_MQTT: imu_raw_mqtt.c with its JSON
# messages in imu_raw_status.c, on the minimal MQTT client mqtt_min.c and
# its socket layer mqtt_min_sock.c), or without it its null implementation
# (common/imu/imu_raw_mqtt_null.c: every batch dropped, as while
# disconnected).

ifeq ($(CONFIG_IMU_RAW_MQTT),y)
CSRCS    += imu_raw_mqtt.c imu_raw_status.c mqtt_min.c mqtt_min_sock.c
else
CSRCS    += imu_raw_mqtt_null.c
endif

# The time synchronization (TIME_SYNC: time_sync.c, its estimate in
# time_offset.c, and the command set of the "ntp" command, nuttx/apps/ntp),
# or without it its null implementation (common/imu/time_sync_null.c: the
# monotonic clock only, which the service needs in every build).

ifeq ($(CONFIG_TIME_SYNC),y)
CSRCS    += time_sync.c time_offset.c
else
CSRCS    += time_sync_null.c
endif

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)drivers$(DELIM)nina$(DELIM)include
endif

include $(APPDIR)/Application.mk

# The prelude of the shared sources (common_prelude.h of the compatibility
# layer) before each source of common/imu.

IMU_COMMON_CSRCS  = $(notdir $(wildcard ../../../common/imu/*.c))
IMU_COMMON_CSRCS += kv_store_wipe.c zephyr_flash_kv.c hex.c $(KV_ZEPHYR_CSRCS)
$(addsuffix $(SUFFIX)$(OBJEXT),$(filter $(IMU_COMMON_CSRCS),$(CSRCS) $(MAINSRC))): \
  CFLAGS += -include $(TOPDIR)$(DELIM)external$(DELIM)compat$(DELIM)common_prelude.h
$(addsuffix $(SUFFIX)$(OBJEXT),$(KV_ZEPHYR_CSRCS)): \
  CFLAGS += -include $(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)zephyr_kv_config.h -Wno-undef

# nvs.c prints uint32_t values with %d, %u and %x: Zephyr's uint32_t is
# unsigned int, NuttX's unsigned long (the same 32 bits).

nvs.c$(SUFFIX)$(OBJEXT): CFLAGS += -Wno-format

# The "kv" command's SHELL_CMD_REGISTER() in kv_store.c makes
# z_shell_main_kv(), which ../kv's main() calls (kv_store.c is not a main
# source).

kv_store.c$(SUFFIX)$(OBJEXT): CFLAGS += -DZ_SHELL_CMD_NAMED_MAIN

ifneq ($(CONFIG_APP_IMU),)

# The "imu" command's SHELL_CMD_REGISTER() in imu_stream.c makes this
# program's main(); time_sync.c is not a main source, so its
# SHELL_CMD_REGISTER(ntp, ...) makes z_shell_main_ntp(), which ../ntp's
# main() calls (Z_SHELL_CMD_NAMED_MAIN, <zephyr/shell/shell.h> of the
# compatibility layer).

time_sync.c$(SUFFIX)$(OBJEXT): CFLAGS += -DZ_SHELL_CMD_NAMED_MAIN
endif
