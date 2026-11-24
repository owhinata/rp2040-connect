############################################################################
# nuttx/apps/subcmd/Makefile
#
# subcmd_main() (subcmd.c and subcmd.h, in common/shell), linked into
# libapps for the commands that add common/shell to their include path, and
# z_shell_cmd_main() (zshell.c, also in common/shell) for the commands
# written with the Zephyr shell macros of the compatibility layer's
# <zephyr/shell/shell.h> (common/compat/include; its NuttX part zephyr_os_shell.h is
# in $(TOPDIR)/external/compat), with the NuttX shell backends of this
# directory (zshell_os_impl.h, which common/shell/zshell_os.h includes).
# VPATH and DEPPATH of common/shell are relative to this directory (mkdeps
# needs relative ones); the objects are made here.
############################################################################

include $(APPDIR)/Make.defs

CSRCS = subcmd.c zshell.c

CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)compat
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)include
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)compat$(DELIM)port
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)..$(DELIM)..$(DELIM)common$(DELIM)shell
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)external$(DELIM)subcmd

VPATH   += :../../../common/shell
DEPPATH += --dep-path ../../../common/shell

include $(APPDIR)/Application.mk
