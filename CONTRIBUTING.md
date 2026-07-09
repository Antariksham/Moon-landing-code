# Contributing to Project SELENE

Thank you for helping build software that will land on another world. This
document is the contract every contribution is reviewed against. The short
version: **read the [coding standard section of the README](README.md#-contribution-rules--coding-standard),
study [`src/gnc/control/pid_controller.hpp`](src/gnc/control/pid_controller.hpp)
(the gold-standard reference), and match it exactly.**

## Workflow

1. **Pick an issue.** Start with `good-first-issue`. Comment on the issue so
   we can assign it to you — this avoids duplicate work.
2. **Branch** from `main`: `git checkout -b feat/<issue-number>-short-name`.
3. **Write the test first** where practical. Flight code without tests does
   not exist as far as CI is concerned.
4. **Develop** under the SELENE coding rules (below). Run locally before
   pushing:
   ```bash
   cmake -B build -DLLS_BUILD_TESTS=ON
   cmake --build build -j
   ctest --test-dir build --output-on-failure
   ```
5. **Open a PR** against `main` and complete every item of the PR checklist.
   One logical change per PR; large PRs will be asked to split.

## The rules, restated

Flight code (`src/`, `include/`) follows the JPL Institutional Coding
Standard adapted for C++17, aligned with MISRA C++:2008 and the Power of Ten:

| # | Rule | Enforced by |
|---|------|-------------|
| 1 | No heap after init: no `new`/`malloc`/allocating STL containers | Review + clang-tidy |
| 2 | No exceptions / RTTI; `[[nodiscard]] Status` returns | `-fno-exceptions -fno-rtti`, `-Werror` |
| 3 | No recursion; all loops statically bounded | Review + static analysis |
| 4 | Fixed-width types only (`lls::I32`, `lls::F32`, …) | Review + clang-tidy |
| 5 | Functions ≤ 60 lines, single responsibility | Review |
| 6 | ≥ 2 `LLS_ASSERT`s per flight-critical function | Review |
| 7 | Everything initialized; `default` in every `switch`; fixed-width enums | `-Werror` + review |
| 8 | Const-correctness throughout | Review |
| 9 | Zero warnings at `-Wall -Wextra -Wconversion -Wpedantic` | `-Werror` in CI |
| 10 | Full Doxygen contracts on public interfaces | Review |

Test code (`tests/`) may use GoogleTest idioms (which require exceptions)
but must still use fixed-width types and bounded loops.

## Commit messages

```
gnc/control: freeze integrator while output is saturated

During throttle-limited braking burns the integrator charged for the
full burn duration and caused a 12% overshoot on recovery in SIL run
#142. Freeze integration when the output is pinned and the error is
pushing further into the same limit.

Fixes #87.
```

Explain **why**. Reference the issue. Present tense, imperative mood.

## Code of conduct

Be rigorous about code and generous with people. Review the work, never the
person. We are all here to put a lander on the Moon.
