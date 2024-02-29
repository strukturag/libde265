noinst_LTLIBRARIES = libde265_arm.la

libde265_arm_la_CXXFLAGS = -I.. $(CFLAG_VISIBILITY)
libde265_arm_la_SOURCES = neon_common.h neon_intrapred.cc neon_intrapred.h neon_dct.cc neon_dct.h arm.cc arm.h
libde265_arm_la_LIBADD =

if HAVE_VISIBILITY
	libde265_arm_la_CXXFLAGS += -DHAVE_VISIBILITY
endif


if ENABLE_NEON_OPT
# NEON specific functions

noinst_LTLIBRARIES += libde265_arm_neon.la
libde265_arm_la_LIBADD += libde265_arm_neon.la
libde265_arm_neon_la_CXXFLAGS = -mfpu=neon -I.. $(CFLAG_VISIBILITY)
libde265_arm_neon_la_CCASFLAGS = -mfpu=neon -I.. \
	-DHAVE_NEON \
	-DEXTERN_ASM= \
	-DHAVE_AS_FUNC \
	-DHAVE_SECTION_DATA_REL_RO

if ENABLE_ARM_THUMB
	libde265_arm_neon_la_CCASFLAGS += -DCONFIG_THUMB
endif

#libde265_arm_neon_la_SOURCES = \
#	asm.S \
#	cpudetect.S \
#	hevcdsp_qpel_neon.S \
#	neon.S

if HAVE_VISIBILITY
	libde265_arm_neon_la_CXXFLAGS += -DHAVE_VISIBILITY
endif

endif
