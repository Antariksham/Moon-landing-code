# SELENE Software-in-the-Loop (SIL) Simulator

Closed-loop descent simulation for exercising the flight software on the
host. The flight code in the loop is the **real** `selene_fsw` library,
compiled under the strict flight flags — only the sim harness itself is
host-style code.

## Milestone 1 (implemented): 1-DOF vertical descent

`selene_sim` flies the terminal descent from the handover gate
(default: 500 m altitude, −30 m/s) to touchdown:

- **Truth dynamics** (`src/lander_dynamics.hpp`) — point mass under lunar
  gravity with a deep-throttleable main engine (min-throttle clamp,
  Isp-based propellant depletion, engine-cutoff modeling).
- **Guidance profile** (`src/descent_sim.hpp`) — constant-deceleration
  braking envelope into a 1 m/s constant-rate final segment, engine cutoff
  at 0.5 m.
- **Flight control law** — mass-feedforward hover throttle plus the flight
  `lls::gnc::PidController` closing the descent-rate loop at 50 Hz.

```bash
cmake -B build -DLLS_BUILD_SIM=ON -DLLS_BUILD_TESTS=ON
cmake --build build -j
./build/sim/selene_sim --telemetry descent.csv
```

The process exit code is the verdict (0 = touchdown within the 2 m/s
limit), so the sim doubles as a CI gate. The closed-loop regression tests
in `tests/sim/` fly the full descent — including dispersed gate conditions —
on every test run.

## Next milestones

- YAML loading of `config/landing_params.yaml` (values are currently
  compiled-in defaults that mirror the file).
- 3-DOF planar dynamics with attitude and a pitch-over maneuver.
- Sensor models (noisy altimeter/IMU) feeding a navigation filter instead
  of truth state.
- Monte-Carlo dispersion runner with landing-footprint statistics.
