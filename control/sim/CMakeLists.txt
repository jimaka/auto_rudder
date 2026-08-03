add_executable(ar_sim
    main.cpp
)
target_link_libraries(ar_sim PRIVATE ar_headers)
# 不强制 -O2：优化等级由 CMAKE_BUILD_TYPE 决定，避免覆盖 Debug/ASan 构建
target_compile_options(ar_sim PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>)

# ar_sim 冒烟测试（依赖修复后的退出码语义：任一 FAIL → 非零退出）
if(AR_BUILD_TESTS)
    add_test(NAME sim_all           COMMAND ar_sim --all)
    add_test(NAME sim_validate      COMMAND ar_sim --validate)
    add_test(NAME sim_ident         COMMAND ar_sim --ident)
    add_test(NAME sim_esc           COMMAND ar_sim --esc)
    add_test(NAME sim_trial         COMMAND ar_sim --trial)
    add_test(NAME sim_maneuver_zigzag COMMAND ar_sim --maneuver zigzag)
    set_tests_properties(sim_all sim_validate sim_ident sim_esc sim_trial sim_maneuver_zigzag
        PROPERTIES TIMEOUT 120)
endif()
