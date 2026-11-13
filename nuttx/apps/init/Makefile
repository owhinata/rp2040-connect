############################################################################
# nuttx/apps/init/Makefile
#
# The init task (CONFIG_INIT_ENTRYPOINT="nano_init_main").  Built as a plain
# library source (CSRCS, no MAINSRC/PROGNAME), so it is linked into libapps
# but not registered as a builtin: the shell cannot start a second init.
# It uses nshlib's internal headers (nsh.h, nsh_console.h) to run an nsh
# session without the login greeting (see nano_init.c).  app_log.h and
# priorities.h come from nuttx/drivers/include ($(TOPDIR)/external/include).
############################################################################

include $(APPDIR)/Make.defs

CSRCS = nano_init.c

CFLAGS += ${INCDIR_PREFIX}$(APPDIR)$(DELIM)nshlib
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include

include $(APPDIR)/Application.mk
