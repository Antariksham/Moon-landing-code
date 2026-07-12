# Guidance

Powered-descent guidance (PDG) and hazard-relative divert re-targeting.

## Implemented

- **`descent_guidance.hpp`** — state-keyed velocity references for the
  approach and terminal-descent phases (milestones 2 + 5). Vertical
  channel: constant-deceleration braking envelope into a constant-rate
  final segment. Horizontal channel (site targeting): the signed
  ground-speed command toward the landing site is the most restrictive of
  the range-to-go braking envelope (stop at the site), a near-field linear
  law (soft arrival), and the altitude-keyed pitch-over ramp (vertical
  attitude guaranteed below 150 m). Also raises the pitch-over
  (`terminal_phase`) and main-engine-cutoff discretes the flight executive
  sequences on.

  Exercised closed-loop by `sim/` (`selene_sim --mode 3dof`): nominal
  landing miss ~1 m, Monte-Carlo worst case < 3 m across the dispersed
  gate envelope. Unit tested in `tests/gnc/test_descent_guidance.cpp`.

## Not yet implemented

- Hazard-relative divert (re-targeting to an HDA-selected safe site
  mid-descent) and fuel-optimal guidance — see `docs/ARCHITECTURE.md`
  § Open design questions.
