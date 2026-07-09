# SELENE Flight Software Architecture

> Living document. The README carries the executive summary; this file is
> where detailed design decisions accumulate. Each major subsystem will get
> its own design note under `docs/` as it matures.

## Design principles

1. **Static everything.** All memory, tasks, and component wiring are fixed
   at initialization. The runtime object graph is knowable — and reviewable —
   at compile time.
2. **One layer touches hardware.** Only `src/hal/` includes device headers.
   GNC and FSW code runs unmodified on the flight target and in the SIL
   simulator; the simulator provides an alternate HAL.
3. **Errors are data.** No exceptions. Every fallible call returns
   `lls::Status`, callers must check it (`[[nodiscard]]`), and every status
   value is a telemetry-visible number.
4. **The timeline is a reviewed artifact.** Mission phases and their legal
   transitions live in one `constexpr` table
   (`src/fsw/mission_state_machine.hpp`) that unit tests audit at compile
   time.

## Execution model

A fixed-rate cyclic executive (FreeRTOS task set, or bare-metal loop on
minimal targets) dispatches at 50 Hz:

```
sensor read (HAL) → navigation update → guidance update
  → control update → actuator write (HAL) → FDIR health scan → telemetry
```

Every component exposes `Init()` / `Update()` / `Reset()` and is budgeted a
worst-case execution time; the executive monitors overruns as FDIR inputs.

## Open design questions (good places to contribute)

- Flight computer selection (drives `cmake/arm-none-eabi.cmake`).
- EKF formulation for TRN fusion: error-state vs. total-state.
- Powered-descent guidance: polynomial (Apollo-heritage) vs. convex
  fuel-optimal (G-FOLD-style) — likely polynomial for Phase I.
- HDA sensor baseline: flash LIDAR vs. camera-only structure-from-motion.
