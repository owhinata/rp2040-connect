############################################################################
# nuttx/apps/i2c/Makefile
#
# The "i2c" builtin command (see Kconfig): the Zephyr build's I2C shell,
# common/shell/i2c_shell.c (found through VPATH and DEPPATH,
# which are relative to this directory, as mkdeps needs them; its object is
# made here).  It is the main source: the compatibility layer's
# SHELL_CMD_REGISTER() makes main(), which runs the command on the Zephyr
# shell's command handling (z_shell_cmd_main() of common/shell/zshell.c,
# built by ../subcmd), and it is compiled with the prelude of the shared
# sources (common_prelude.h of the compatibility layer, -include).  The bus
# is the compatibility layer's I2C bus device i2c0
# (nuttx/drivers/compat/zephyr_i2c.c, bound by the board).
#
# Include paths: the Zephyr compatibility layer ($(TOPDIR)/external/compat
# and common/compat: <zephyr/drivers/i2c.h>, <zephyr/shell/shell.h>), and
# the logging macros (common/compat/log: app_log.h; its NuttX part in
# $(TOPDIR)/external/include).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = i2c
PRIORITY  = $(CONFIG_APP_I2C_PRIORITY)
STACKSIZE = $(CONFIG_APP_I2C_STACKSIZE)
MODULE    = $(CONFIG_APP_I2C)

MAINSRC   = i2c_shell.c

CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)log
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS   += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat

VPATH    += :../../../common/shell
DEPPATH  += --dep-path ../../../common/shell

include $(APPDIR)/Application.mk

i2c_shell.c$(SUFFIX)$(OBJEXT): \
  CFLAGS += -include $(TOPDIR)$(DELIM)external$(DELIM)compat$(DELIM)common_prelude.h
