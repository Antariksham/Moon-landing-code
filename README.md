<div align="center">

# 🌑 Project SELENE
### Open-Source Flight Software for Autonomous Lunar Soft-Landing

**S**afe **E**ntry, **L**anding & **E**xploration **N**avigation **E**xecutive

[![Build](https://img.shields.io/badge/build-CMake%203.20%2B-informational)](#-building-the-software)
[![Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](#-technology-stack)
[![Coding Standard](https://img.shields.io/badge/coding%20standard-JPL%20%2F%20MISRA%20C%2B%2B-critical)](#-contribution-rules--coding-standard)
[![Coverage](https://img.shields.io/badge/unit%20test%20coverage-100%25%20required-success)](#-testing-policy)
[![License](https://img.shields.io/badge/license-Apache%202.0-green.svg)](LICENSE)

*"The Moon is the proving ground. Mars is the destination."*

</div>

---

## 🚀 Mission Vision: The Moon-to-Mars Roadmap

Project SELENE is a non-profit, open-source initiative to build **flight-grade
Guidance, Navigation & Control (GNC) software** for an autonomous lunar
soft-landing system. We are not building a toy simulator — we are building
software to the same engineering discipline used at NASA JPL and ISRO, in the
open, so that anyone on Earth can inspect, test, and improve it.

Our roadmap has three phases:

| Phase | Target | Objective |
|-------|--------|-----------|
| **Phase I — Descent** *(current)* | Lunar surface | Demonstrate autonomous powered-descent guidance, terrain-relative navigation, and hazard avoidance on a technology-demonstrator lander. |
| **Phase II — Traverse** | Lunar surface | Deploy and operate a small autonomous rover from the Phase I lander; validate surface autonomy, power management, and fault recovery over multiple lunar days. |
| **Phase III — Mars** | Martian surface | Reuse the flight-proven SELENE GNC core — hardened by two lunar missions — as the foundation of a Mars rover entry, descent, and landing (EDL) system. |

**Why the Moon first?** The Moon is three days away, has no atmosphere to
complicate descent dynamics, and offers round-trip communication latency of
~2.6 seconds — close enough for supervised autonomy, far enough that the
software must land the vehicle *by itself*. Every line of code we flight-prove
on the Moon de-risks the Mars mission, where a 4–24 minute light delay makes
full autonomy non-negotiable. A successful lunar demonstration is our strongest
argument to funding agencies that this team can be trusted with a Mars payload.

---

## 🧭 System Architecture

SELENE follows a classical, deterministic flight-software architecture: a
fixed-rate real-time executive dispatching statically-allocated components
through well-defined interfaces. **There is no dynamic behavior at runtime —
by design.**

```
┌─────────────────────────────────────────────────────────────────────┐
│                        FLIGHT EXECUTIVE (FSW)                        │
│         Fixed-rate scheduler · Mission State Machine · FDIR          │
├──────────────────┬───────────────────┬──────────────────────────────┤
│    GUIDANCE      │    NAVIGATION     │           CONTROL             │
│                  │                   │                               │
│ · Powered-descent│ · IMU propagation │ · Attitude control (RCS)      │
│   guidance (PDG) │ · Terrain-relative│ · Throttle control (PID/TVC)  │
│ · Trajectory     │   navigation (TRN)│ · Actuator command limiting   │
│   re-targeting   │ · Altimeter/      │ · Control allocation          │
│ · Hazard-relative│   velocimeter     │                               │
│   divert         │   fusion (EKF)    │                               │
├──────────────────┴───────────────────┴──────────────────────────────┤
│                    HAZARD DETECTION & AVOIDANCE (HDA)                │
│      LIDAR/camera terrain mapping · Slope & roughness scoring        │
│              Safe-site selection · Divert recommendation             │
├──────────────────────────────────────────────────────────────────────┤
│                   HARDWARE ABSTRACTION LAYER (HAL)                   │
│        Sensor drivers · Actuator drivers · Telemetry · Timekeeping   │
└──────────────────────────────────────────────────────────────────────┘
```

### Module Overview

- **Flight Executive (`src/fsw/`)** — The mission state machine that sequences
  the vehicle through `BOOT → STANDBY → DEORBIT → BRAKING → APPROACH →
  TERMINAL_DESCENT → TOUCHDOWN → SAFED`. All phase transitions are validated
  against a compile-time transition table; an illegal transition request is a
  reportable fault, never undefined behavior.
- **Guidance (`src/gnc/guidance/`)** — Computes the reference trajectory from
  current state to the targeted landing site (polynomial / fuel-optimal
  powered-descent guidance), and re-targets when HDA requests a divert.
- **Navigation (`src/gnc/navigation/`)** — Estimates vehicle state (position,
  velocity, attitude) by fusing IMU, star tracker, radar altimeter, and
  terrain-relative navigation fixes in an Extended Kalman Filter.
- **Control (`src/gnc/control/`)** — Closes the loop between the guidance
  reference and the navigation estimate. Includes the attitude and throttle
  controllers (see [`pid_controller.hpp`](src/gnc/control/pid_controller.hpp)
  for the reference implementation and coding gold standard).
- **Hazard Detection & Avoidance** — Builds a local terrain map during
  approach, scores candidate landing sites for slope, roughness, and fuel
  cost, and recommends divert targets to Guidance.
- **FDIR (`src/fdir/`)** — Fault Detection, Isolation & Recovery. Monitors
  component health words each cycle and commands safe-mode transitions.
- **HAL (`src/hal/`)** — The only layer that touches hardware. Everything
  above it runs unmodified on the flight target and in the simulator.

---

## 🛠 Technology Stack

| Layer | Choice | Rationale |
|-------|--------|-----------|
| Language | **C++17** (freestanding-friendly subset) | Strong typing, `constexpr`, `std::array` — without exceptions, RTTI, or heap. |
| Build | **CMake ≥ 3.20** | Cross-compilation via toolchain files; identical build for target and host. |
| RTOS | **FreeRTOS** (bare-metal cyclic executive also supported) | Deterministic, statically-allocated tasks; broad MCU support. |
| Unit testing | **GoogleTest / GoogleMock** | Runs on host; enforced 100% line coverage on flight code. |
| Static analysis | clang-tidy + cppcheck (MISRA profile) | Every PR is gated on a clean report. |
| Formatting | clang-format (config in repo) | Zero style debates in review. |
| Simulation | Software-in-the-loop (SIL) harness in `sim/` | 3-DOF/6-DOF descent dynamics for closed-loop testing. |

---

## 📐 Contribution Rules & Coding Standard

> **⚠️ Read this section before writing a single line of code.
> Pull requests that violate these rules will not be reviewed until fixed.**

This is spaceflight software. A bug does not produce a stack trace on a
server — it produces a crater. All flight code (`src/`, `include/`) **must**
comply with the
[JPL Institutional Coding Standard for the C Programming Language](https://web.archive.org/web/20190108051625/https://lars-lab.jpl.nasa.gov/JPL_Coding_Standard_C.pdf)
adapted for C++ (aligned with **MISRA C++:2008** and the
[Power of Ten](https://spinroot.com/gerard/pdf/P10.pdf) rules). The
non-negotiable core:

### The Ten Commandments of SELENE Flight Code

1. **No dynamic memory allocation after initialization.**
   No `malloc`, `free`, `new`, `delete`, and no STL containers that allocate
   (`std::vector`, `std::string`, `std::map`, …). Use `std::array` and
   statically-allocated objects. All memory is owned at compile time.
2. **No exceptions, no RTTI.** Compiled with `-fno-exceptions -fno-rtti`.
   Errors are reported through explicit status codes that the caller **must**
   check (enforced by `[[nodiscard]]`).
3. **No recursion. All loops must have a statically provable upper bound.**
   The scheduler must be able to guarantee worst-case execution time.
4. **Strict type safety.** Fixed-width integer types (`std::int32_t`,
   `std::uint8_t`, …) only — never bare `int`, `long`, or `unsigned`.
   No implicit narrowing conversions. Floating-point types are used
   deliberately and documented (`float` = F32 on target).
5. **Every function fits on two pages (≤ 60 lines) and does one thing.**
6. **Assertion density ≥ 2 per function** on flight-critical paths, via the
   side-effect-free `LLS_ASSERT` macro (compiles to fault-report, never abort).
7. **All variables initialized at declaration. All `switch` statements have a
   `default`. All enum classes are backed by a fixed-width type.**
8. **Const-correctness everywhere.** Data is immutable unless it has a
   documented reason not to be.
9. **Compile clean at the highest warning level.**
   `-Wall -Wextra -Wconversion -Wpedantic -Werror`. A warning is an error.
10. **Every public function carries a Doxygen contract:** what it does, every
    parameter's units and valid range, every possible return status, and its
    real-time characteristics.

### 🧪 Testing Policy

- **100% line coverage** of flight code by GoogleTest unit tests is
  **mandatory**. CI rejects PRs that lower coverage.
- Every bug fix ships with a regression test that fails before the fix.
- Every controller/estimator ships with a closed-loop SIL test in `sim/`.
- Tests must cover **off-nominal paths**: NaN inputs, saturations, invalid
  state transitions, sensor dropouts — not just the happy path.

### Pull Request Checklist

- [ ] `clang-format` applied (CI enforces).
- [ ] `clang-tidy` and `cppcheck` clean.
- [ ] Unit tests added/updated; coverage at 100%.
- [ ] No dynamic allocation, no exceptions, no recursion introduced.
- [ ] Doxygen contracts complete on all new public interfaces.
- [ ] Commit messages explain *why*, not just *what*.

See [CONTRIBUTING.md](CONTRIBUTING.md) for the full workflow.

---

## 🗂 Repository Layout

```
.
├── src/                    # Flight code (JPL/MISRA rules apply)
│   ├── fsw/                #   Flight executive & mission state machine
│   ├── gnc/                #   Guidance, Navigation & Control
│   │   ├── guidance/       #     Powered-descent guidance, site targeting
│   │   ├── navigation/     #     Vertical nav filter, TRN, sensor fusion
│   │   └── control/        #     PID / thrust allocation / attitude
│   ├── hda/                #   Hazard detection & avoidance (safe-site
│   │                       #   selection, divert recommendation)
│   ├── fdir/               #   Fault detection, isolation & recovery
│   └── hal/                #   Hardware abstraction layer (drivers)
├── include/lls/            # Public cross-module headers (types, assert)
├── tests/                  # GoogleTest unit tests (mirrors src/ layout)
├── sim/                    # Software-in-the-loop descent simulator
├── docs/                   # Architecture docs, ICDs, design reviews
├── config/                 # Mission & vehicle parameter files
├── cmake/                  # Toolchain files (host, arm-none-eabi)
├── tools/                  # Developer tooling (coverage, format checks)
└── .github/                # CI workflows, issue & PR templates
```

---

## 🔨 Building the Software

### Prerequisites

- CMake ≥ 3.20, a C++17 compiler (GCC ≥ 10 or Clang ≥ 12)
- For target builds: `arm-none-eabi-gcc` toolchain
- GoogleTest is fetched automatically by CMake for host test builds

### Host build + unit tests (start here)

```bash
git clone https://github.com/mayank7643/Moon-landing-code.git
cd Moon-landing-code

cmake -B build -DCMAKE_BUILD_TYPE=Debug -DLLS_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

### Flight target cross-build

```bash
cmake -B build-target \
      -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake \
      -DCMAKE_BUILD_TYPE=Release
cmake --build build-target -j
```

### Run the descent simulation

```bash
cmake -B build -DLLS_BUILD_SIM=ON
cmake --build build -j
./build/sim/selene_sim --telemetry descent.csv
```

The exit code is the verdict — `0` means touchdown within the 2 m/s limit —
so the simulator doubles as a CI gate. See [`sim/README.md`](sim/README.md)
for the scenario details and CLI flags.

---

## 🤝 Getting Involved

We need contributors across the stack: GNC engineers, embedded C++
developers, simulation/dynamics specialists, and technical writers. Start with
issues labeled **`good-first-issue`**, read the
[gold-standard reference implementation](src/gnc/control/pid_controller.hpp),
and introduce yourself in the Discussions tab.

**The bar is high on purpose.** Code you write here is written to land on
another world. If that excites you, welcome aboard. 🌍 → 🌑 → 🔴

---

## 📄 License

Licensed under the [Apache License 2.0](LICENSE).
