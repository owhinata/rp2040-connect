############################################################################
# nuttx/apps/subcmd/Makefile
#
# subcmd_main() (see subcmd.h), linked into libapps for the commands that
# add ${INCDIR_PREFIX}$(APPDIR)/external/subcmd to their CFLAGS, and
# z_shell_cmd_main() with the shell backends (zshell.c) for the commands
# written with the Zephyr shell macros of the compatibility layer's
# <zephyr/shell/shell.h> ($(TOPDIR)/external/compat).
############################################################################

include $(APPDIR)/Make.defs

CSRCS = subcmd.c zshell.c

CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat

include $(APPDIR)/Application.mk
