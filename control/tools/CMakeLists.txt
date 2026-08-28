add_executable(ar_tuner_debug
    tuner_debug_cli.cpp
)
target_link_libraries(ar_tuner_debug PRIVATE ar_headers)
target_compile_options(ar_tuner_debug PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>)
