include $(top_srcdir)/build-aux/init.mk

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

# Search for headers properly:
AM_CPPFLAGS += 					\
	$(HPP_UTIL_CFLAGS)

lib_LTLIBRARIES = libkws-bezier-optimizer.la

libkws_bezier_optimizer_la_SOURCES = 			\
				optimizer.cc

libkws_bezier_optimizer_la_LIBADD = $(HPP_UTIL_LIBS)
libkws_bezier_optimizer_la_CPPFLAGS = $(AM_CPPFLAGS)
