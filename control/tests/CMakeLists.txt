# GoogleTest 经 FetchContent 拉取
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
# Windows 下强制共享库避免 MSVC 运行库问题；Linux 下用静态
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# 每个测试文件一个可执行目标，便于独立调试与故障定位
set(AR_TEST_SOURCES
    test_angle_utils
    test_pd_controller
    test_gain_schedule
    test_nomoto_identifier
    test_auto_tuner
    test_maneuver_sequencer
    test_param_store
    test_autopilot_core
)

foreach(t ${AR_TEST_SOURCES})
    add_executable(${t} ${t}.cpp)
    target_link_libraries(${t} PRIVATE ar_headers GTest::gtest_main)
    target_compile_options(${t} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>)
    gtest_discover_tests(${t} PROPERTIES ENVIRONMENT
        "ASAN_OPTIONS=detect_leaks=0:abort_on_error=0;LSAN_OPTIONS=detect_leaks=0")
endforeach()
