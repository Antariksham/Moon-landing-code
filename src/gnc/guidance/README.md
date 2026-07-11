# Guidance

Powered-descent guidance (PDG) and hazard-relative divert re-targeting.

## Implemented

- **`descent_guidance.hpp`** — altitude-keyed velocity references for the
  approach and terminal-descent phases (milestone 2). Vertical channel:
  constant-deceleration braking envelope into a constant-rate final
  segment. Horizontal channel: allowed ground speed shrinks linearly with
  altitude and reaches zero at the pitch-over altitude, forcing a vertical
  terminal descent. Also raises the pitch-over (`terminal_phase`) and
  main-engine-cutoff discretes the flight executive sequences on.

  Exercised closed-loop by `sim/` (`selene_sim --mode 3dof`) and unit
  tested in `tests/gnc/test_descent_guidance.cpp`.

## Not yet implemented

- Downrange landing-site targeting (position control, not just velocity).
- Fuel-optimal re-targeting and hazard-relative divert — see
  `docs/ARCHITECTURE.md` § Open design questions.
