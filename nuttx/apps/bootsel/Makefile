############################################################################
# nuttx/apps/bootsel/Makefile
#
# The "bootsel" builtin command: restart into BOOTSEL (see Kconfig).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = bootsel
PRIORITY  = $(CONFIG_APP_BOOTSEL_PRIORITY)
STACKSIZE = $(CONFIG_APP_BOOTSEL_STACKSIZE)
MODULE    = $(CONFIG_APP_BOOTSEL)

MAINSRC   = bootsel_main.c

include $(APPDIR)/Application.mk
