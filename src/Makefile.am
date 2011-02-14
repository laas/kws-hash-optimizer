include $(top_srcdir)/build-aux/init.mk

# Add available warnings flags.
AM_CXXFLAGS += $(WARNING_CXXFLAGS)

# Add dependency to hppCore
AM_CXXFLAGS += $(HPPCORE_CFLAGS)

# Add dependencies CPPFLAGS.
AM_CPPFLAGS += 					\
	$(HPP_UTIL_CFLAGS)			\
	$(HPPCORE_CFLAGS)

lib_LTLIBRARIES = libkws-hash-optimizer.la

libkws_hash_optimizer_la_SOURCES = 			\
				optimizer.cc		\
				directpath.cc		\
				steeringmethod.cc	\
				distance.cc		\
				elliptic-directpath.cc	\
				elliptic-steeringmethod.cc

libkws_hash_optimizer_la_LIBADD = 		\
	$(HPP_UTIL_LIBS) 				\
	$(HPPCORE_LIBS)	

#libkws_hash_optimizer_la_CPPFLAGS = $( AM_CPPFLAGS)
