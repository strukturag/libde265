APP_ABI := arm64-v8a # $(NDK_ARCH) # all
APP_PLATFORM := android-15 # android-21
APP_STL      := gnustl_static # c++_static # gnustl_static # stlport_static
APP_CPPFLAGS += -std=c++11
APP_CPPFLAGS += -frtti
APP_CPPFLAGS += -fexceptions
APP_OPTIM := release
NDK_TOOLCHAIN_VERSION := 4.9
