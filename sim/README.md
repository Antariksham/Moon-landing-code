# SELENE Software-in-the-Loop (SIL) Simulator

Closed-loop descent simulation for exercising the flight software on the
host: 3-DOF (later 6-DOF) lander dynamics, sensor models with realistic
noise, and a simulator-side HAL that replaces `src/hal/`.

**Status: scaffolding.** The first milestone is a 1-DOF vertical-descent
simulation closing the loop around the descent-rate PID controller
(`src/gnc/control/pid_controller.hpp`) — a well-scoped
`good-first-issue`-sized contribution. Build with `-DLLS_BUILD_SIM=ON` once
the first target lands here.

Simulation code runs on the host only and is exempt from the
no-heap/no-exceptions flight rules, but flight code linked into the sim is
still built under the strict flags — that is the point of the exercise.
