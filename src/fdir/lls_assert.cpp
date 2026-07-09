/**
 * @file    lls_assert.cpp
 * @brief   Default FDIR fault hook for `LLS_ASSERT` (see lls_assert.hpp).
 *
 * @details This is the bootstrap implementation used until the full FDIR
 *          fault-protection engine lands. It maintains a monotonic fault
 *          counter exported as a telemetry point. It performs no I/O and
 *          never blocks, so it is safe from any task or interrupt context.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "lls/lls_assert.hpp"

namespace lls {

namespace {
/** Fault counter since boot. Static storage — no allocation (rule #1). */
U32 g_assert_failure_count = 0U;
}  // namespace

void ReportAssertFailure(const char* file, const I32 line) noexcept {
    /* The file/line pair will be routed to the telemetry ring buffer once
     * the FDIR engine is integrated; until then the parameters are recorded
     * only through the counter. Marked as intentionally unused. */
    static_cast<void>(file);
    static_cast<void>(line);

    ++g_assert_failure_count;  /* U32 wrap-around is defined and acceptable. */
}

U32 GetAssertFailureCount() noexcept {
    return g_assert_failure_count;
}

}  // namespace lls
