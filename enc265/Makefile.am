
bin_PROGRAMS = gen-enc-table yuv-distortion rd-curves

AM_CPPFLAGS = -I../libde265

gen_enc_table_DEPENDENCIES = ../libde265/libde265.la
gen_enc_table_CXXFLAGS =
gen_enc_table_LDFLAGS =
gen_enc_table_LDADD = ../libde265/libde265.la -lstdc++
gen_enc_table_SOURCES = tool_gen_entropy_table.cc

yuv_distortion_DEPENDENCIES = ../libde265/libde265.la
yuv_distortion_CXXFLAGS =
yuv_distortion_LDFLAGS =
yuv_distortion_LDADD = ../libde265/libde265.la -lstdc++
yuv_distortion_SOURCES = tool_yuv_distortion.cc

rd_curves_DEPENDENCIES = ../libde265/libde265.la
rd_curves_CXXFLAGS =
rd_curves_LDFLAGS =
rd_curves_LDADD = ../libde265/libde265.la -lstdc++
rd_curves_SOURCES = tool_rd_curves.cc

