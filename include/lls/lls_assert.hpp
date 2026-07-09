/**
 * @file    lls_assert.hpp
 * @brief   Side-effect-free flight assertion macro (SELENE rule #6).
 *
 * @details `LLS_ASSERT` is the only sanctioned assertion mechanism in flight
 *          code. Unlike the C standard `assert`, it never calls `abort()`:
 *          crashing the flight computer during terminal descent is worse
 *          than almost any error it could report. Instead, a failed
 *          assertion invokes the FDIR fault hook, which logs the fault to
 *          telemetry and lets the caller degrade gracefully via its normal
 *          `Status` return path.
 *
 *          Rules for use:
 *            - The asserted expression must be free of side effects.
 *            - Target density is >= 2 assertions per flight-critical function.
 *            - `LLS_ASSERT` checks programmer errors (contract violations);
 *              expected runtime faults (bad sensor data, saturation) are
 *              handled with explicit `Status` codes instead.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_LLS_ASSERT_HPP
#define LLS_LLS_ASSERT_HPP

#include "lls/lls_types.hpp"

namespace lls {

/**
 * @brief   FDIR fault-reporting hook invoked on assertion failure.
 *
 * @details The default (weak) implementation increments a fault counter that
 *          ground can observe in telemetry. The FDIR module overrides it at
 *          link time to route faults into the fault-protection engine. It is
 *          guaranteed non-blocking and safe to call from any task context.
 *
 * @param   file  Null-terminated source file name (from `__FILE__`).
 * @param   line  Source line number (from `__LINE__`).
 */
void ReportAssertFailure(const char* file, I32 line) noexcept;

/**
 * @brief   Number of assertion failures since boot (telemetry point).
 * @return  Monotonic fault count; wraps at `U32` range by design.
 */
[[nodiscard]] U32 GetAssertFailureCount() noexcept;

}  // namespace lls

/**
 * @brief Flight assertion: report-and-continue, never abort.
 *
 * Evaluates @p expr exactly once. On failure, reports to FDIR and execution
 * continues — the enclosing function is responsible for returning a suitable
 * `Status`. Enabled in all build types, including flight Release builds.
 */
#define LLS_ASSERT(expr)                                                   \
    do {                                                                   \
        if (!(expr)) {                                                     \
            ::lls::ReportAssertFailure(__FILE__,                           \
                                       static_cast<::lls::I32>(__LINE__)); \
        }                                                                  \
    } while (false)

#endif  // LLS_LLS_ASSERT_HPP
