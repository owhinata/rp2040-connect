############################################################################
# nuttx/apps/irqstack/Makefile
#
# The "irqstack" builtin command: interrupt stack use (see Kconfig).
############################################################################

include $(APPDIR)/Make.defs

PROGNAME  = irqstack
PRIORITY  = $(CONFIG_NANO_APP_IRQSTACK_PRIORITY)
STACKSIZE = $(CONFIG_NANO_APP_IRQSTACK_STACKSIZE)
MODULE    = $(CONFIG_NANO_APP_IRQSTACK)

MAINSRC   = irqstack_main.c

include $(APPDIR)/Application.mk
