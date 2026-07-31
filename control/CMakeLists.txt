cmake_minimum_required(VERSION 3.14)
project(auto_rudder_control CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(AR_BUILD_TESTS "Build GoogleTest unit tests" ON)
option(AR_BUILD_SIM    "Build closed-loop simulation"  ON)

# 算法库为 header-only，作为 INTERFACE 库供 tests/sim 复用
add_library(ar_headers INTERFACE)
target_include_directories(ar_headers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(ar_headers INTERFACE
    $<$<AND:$<CONFIG:Debug>,$<CXX_COMPILER_ID:GNU,Clang,AppleClang>>:-Wall -Wextra -Wpedantic>)

if(AR_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(AR_BUILD_SIM)
    add_subdirectory(sim)
endif()
