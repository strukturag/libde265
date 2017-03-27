LOCAL_PATH := $(call my-dir)

# Define vars for library that will be build statically.
include $(CLEAR_VARS)
LOCAL_MODULE := libde265
LOCAL_C_INCLUDES := ..
LOCAL_SRC_FILES :=  contextmodel.cc  dpb.cc  fallback-intra-dc.cc \
  frontend-syntax-decoder.cc  image-unit.cc  nal.cc         refpic.cc  slice.cc      util.cc \
  bitstream.cc    de265.cc         en265.cc         fallback-motion.cc    funcs.cc \
  intrapred.cc   nal-parser.cc  sao.cc     sps.cc        visualize.cc \
  cabac.cc        deblock.cc       fallback.cc      fallback-sao.cc       image.cc \
  md5.cc         pps.cc         scan.cc    threads.cc    vps.cc \
  decctx.cc        fallback-dct.cc  frame-dropper.cc      image-io.cc \
  motion.cc      quality.cc     sei.cc     transform.cc  vui.cc

# Optional compiler flags.
LOCAL_LDLIBS   =
LOCAL_CFLAGS   = -std=c++11 -g

ifeq ($(TARGET_ARCH_ABI),x86)
  LOCAL_SRC_FILES += x86/sse.cc  x86/sse-dct.cc  x86/sse-intra-dc.cc  x86/sse-motion.cc  x86/sse-motion-new.cc  x86/sse-sao.cc
endif

ifeq ($(TARGET_ARCH_AVI),x86_64)
  LOCAL_SRC_FILES += x86/sse.cc  x86/sse-dct.cc  x86/sse-intra-dc.cc  x86/sse-motion.cc  x86/sse-motion-new.cc  x86/sse-sao.cc
endif

ifeq ($(TARGET_ARCH_ABI),armeabi-v7a)
  LOCAL_SRC_FILES += arm/arm.cc  arm/arm-dct.cc  arm/arm-motion.cc
  LOCAL_CXXFLAGS += -DHAVE_NEON -DHAVE_AARCH64 -mfpu=neon -mfloat-abi=softfp
endif

ifeq ($(TARGET_ARCH_ABI),arm64-v8a)
  LOCAL_SRC_FILES += arm/arm.cc  arm/arm-dct.cc  arm/arm-motion.cc
  LOCAL_CXXFLAGS += -DHAVE_NEON -DHAVE_AARCH64
endif

include $(BUILD_STATIC_LIBRARY)
