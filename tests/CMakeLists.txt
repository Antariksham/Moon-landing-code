# Unit tests run on the host with GoogleTest. Test code is exempt from the
# no-exceptions rule (GoogleTest requires them), so it does NOT link
# lls_flight_options; the flight library itself is still built under the
# strict flags.

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
    GIT_SHALLOW    TRUE)
# Prevent GoogleTest from overriding compiler/linker options on Windows.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(selene_unit_tests
    gnc/test_pid_controller.cpp
    fsw/test_mission_state_machine.cpp)

target_link_libraries(selene_unit_tests PRIVATE
    selene_fsw
    GTest::gtest_main)

target_compile_options(selene_unit_tests PRIVATE
    -Wall -Wextra -Werror)

include(GoogleTest)
gtest_discover_tests(selene_unit_tests)
