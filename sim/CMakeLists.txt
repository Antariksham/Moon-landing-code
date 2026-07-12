# SELENE software-in-the-loop simulator (host only).
#
# selene_sim_core links the real flight library — the flight code in the
# loop is compiled under the strict flight flags; only the sim harness
# itself is host-style code.

add_library(selene_sim_core STATIC
    src/lander_dynamics.cpp
    src/lander_dynamics_3dof.cpp
    src/sensor_models.cpp
    src/descent_sim.cpp
    src/approach_sim.cpp)

target_include_directories(selene_sim_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(selene_sim_core PUBLIC selene_fsw)

target_compile_options(selene_sim_core PRIVATE
    -Wall -Wextra -Wconversion -Wpedantic -Werror)

add_executable(selene_sim src/main.cpp)
target_link_libraries(selene_sim PRIVATE selene_sim_core)
target_compile_options(selene_sim PRIVATE
    -Wall -Wextra -Wconversion -Wpedantic -Werror)
