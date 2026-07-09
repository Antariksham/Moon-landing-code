cmake_minimum_required(VERSION 3.20)

project(selene_fsw
    VERSION 0.1.0
    DESCRIPTION "Project SELENE - Autonomous Lunar Soft-Landing Flight Software"
    LANGUAGES CXX)

option(LLS_BUILD_TESTS "Build GoogleTest unit tests" ON)
option(LLS_BUILD_SIM   "Build the software-in-the-loop simulator" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Flight code compile rules (SELENE coding standard, rules #2 and #9):
# no exceptions, no RTTI, warnings are errors.
# ---------------------------------------------------------------------------
add_library(lls_flight_options INTERFACE)
target_compile_options(lls_flight_options INTERFACE
    -fno-exceptions
    -fno-rtti
    -Wall
    -Wextra
    -Wconversion
    -Wpedantic
    -Werror)

# ---------------------------------------------------------------------------
# Flight software library (everything under src/ + include/)
# ---------------------------------------------------------------------------
add_library(selene_fsw STATIC
    src/fdir/lls_assert.cpp
    src/fsw/mission_state_machine.cpp
    src/gnc/control/pid_controller.cpp)

target_include_directories(selene_fsw PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

# PRIVATE: the strict flags apply to flight code itself, not to consumers
# such as the GoogleTest executable (test code needs exceptions).
target_link_libraries(selene_fsw PRIVATE lls_flight_options)

# ---------------------------------------------------------------------------
# Unit tests (host only; GoogleTest fetched automatically)
# ---------------------------------------------------------------------------
if(LLS_BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# Software-in-the-loop simulator (host only)
# ---------------------------------------------------------------------------
if(LLS_BUILD_SIM AND NOT CMAKE_CROSSCOMPILING)
    add_subdirectory(sim)
endif()
