noinst_LTLIBRARIES = libde265_arm.la

libde265_arm_la_CXXFLAGS = -I.. $(CFLAG_VISIBILITY)
libde265_arm_la_SOURCES = arm.cc arm.h
libde265_arm_la_LIBADD =

if HAVE_VISIBILITY
	libde265_arm_la_CXXFLAGS += -DHAVE_VISIBILITY
endif


if ENABLE_NEON_OPT
# NEON specific functions

noinst_LTLIBRARIES += libde265_arm_neon.la
libde265_arm_la_LIBADD += libde265_arm_neon.la
libde265_arm_neon_la_CXXFLAGS = -I.. $(CFLAG_VISIBILITY)

#if ENABLE_ARM_THUMB
#	libde265_arm_neon_la_CCASFLAGS += -DCONFIG_THUMB
#endif

libde265_arm_neon_la_SOURCES = arm-motion.cc arm-motion.h arm-dct.cc arm-dct.h

if HAVE_VISIBILITY
	libde265_arm_neon_la_CXXFLAGS += -DHAVE_VISIBILITY
endif

endif
