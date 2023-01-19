
add_library(algo OBJECT
        algo.h algo.cc
        coding-options.h coding-options.cc
        ctb-qscale.h ctb-qscale.cc
        cb-split.h cb-split.cc
        cb-intrapartmode.h cb-intrapartmode.cc
        cb-interpartmode.h cb-interpartmode.cc
        cb-skip.h cb-skip.cc
        cb-intra-inter.h cb-intra-inter.cc
        cb-mergeindex.h cb-mergeindex.cc
        tb-split.h tb-split.cc
        tb-transform.h tb-transform.cc
        tb-intrapredmode.h tb-intrapredmode.cc
        tb-rateestim.h tb-rateestim.cc)

target_include_directories (algo PRIVATE ${PROJECT_SOURCE_DIR})
target_include_directories (algo PRIVATE ${PROJECT_BINARY_DIR})

set(ALGO_OBJECTS $<TARGET_OBJECTS:algo> PARENT_SCOPE)
