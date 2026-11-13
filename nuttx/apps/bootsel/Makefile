############################################################################
# nuttx/apps/bootsel/Makefile
#
# The "bootsel" builtin command (development use; see Kconfig).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = bootsel
PRIORITY  = $(CONFIG_NANO_APP_BOOTSEL_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_BOOTSEL_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_BOOTSEL)

MAINSRC   = bootsel_main.c

include $(APPDIR)/Application.mk
