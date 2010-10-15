include $(top_srcdir)/build-aux/init.mk

# Search for autoconf macros in build-aux.
ACLOCAL_AMFLAGS = -I build-aux

SUBDIRS = 		\
	doc		\
	include		\
	src		\
	tests


# Provide bootstrap with the generated package.
EXTRA_DIST += bootstrap deps.ac build-aux/bootstrap

# Nothing to do in this directory.
all:

# Libtool rules
LIBTOOL_DEPS = @LIBTOOL_DEPS@
libtool: $(LIBTOOL_DEPS)
	$(SHELL) ./config.status --recheck

include $(top_srcdir)/build-aux/pkg-config.mk

# Search for headers properly:
AM_CPPFLAGS += 					\
	$(HPP_UTIL_CFLAGS)

# Link against the library (here supposing your library is called
# `hpp-MYPACKAGE.la` and only depends on `hpp-util` which is usually wrong):
libhpp_MYPACKAGE_la_LIBADD = 		\
	$(HPP_UTIL_LIBS)

# Generate documentation.
.PHONY: doc html
doc html:
	cd doc && $(MAKE) $(AM_MAKEFLAGS) $@

