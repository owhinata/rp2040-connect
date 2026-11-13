############################################################################
# nuttx/drivers/Makefile
#
# Kernel-side code of the Nano RP2040 Connect port, built by NuttX as
# external/libexternal.a (nuttx/nuttx/external is a symlink to this
# directory; see tools/LibTargets.mk).  The rules follow
# nuttx/nuttx/wireless/Makefile (NuttX 13.1.0).
#
# Each subdirectory adds its sources from its own Make.defs, e.g.
#   include nina/Make.defs
# with "CSRCS += ...", "VPATH += :nina" and "DEPPATH += --dep-path nina".
# include/ holds the headers shared with the board code and the
# applications ($(TOPDIR)/external/include); compat/ the small Zephyr
# compatibility layer ($(TOPDIR)/external/compat).
############################################################################

include $(TOPDIR)/Make.defs

ASRCS :=
CSRCS :=

include log/Make.defs
include nina/Make.defs
include compat/Make.defs
include lsm6dso/Make.defs

CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)external$(DELIM)include

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libexternal$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE_REBUILD, $@, $(OBJS))

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds))
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
