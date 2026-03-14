add_executable(stream_fuzzer stream_fuzzer.cc)
target_link_libraries(stream_fuzzer PRIVATE de265)
target_link_options(stream_fuzzer PRIVATE "-fsanitize=fuzzer")

add_executable(nal_fuzzer nal_fuzzer.cc)
target_link_libraries(nal_fuzzer PRIVATE de265)
target_link_options(nal_fuzzer PRIVATE "-fsanitize=fuzzer")
