/**
 * @file    lls_types.hpp
 * @brief   Project-wide fixed-width scalar types and status codes.
 *
 * @details Per SELENE coding rule #4, bare `int`/`long`/`unsigned` are
 *          forbidden in flight code. All modules use the aliases below so
 *          that the width and signedness of every scalar is explicit at the
 *          point of use and identical on the host and the flight target.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_LLS_TYPES_HPP
#define LLS_LLS_TYPES_HPP

#include <cstdint>

namespace lls {

/* ------------------------------------------------------------------------ */
/* Fixed-width scalar aliases (rule #4: no bare int/long/unsigned).          */
/* ------------------------------------------------------------------------ */

using I8  = std::int8_t;    /**< Signed  8-bit integer.  */
using I16 = std::int16_t;   /**< Signed 16-bit integer.  */
using I32 = std::int32_t;   /**< Signed 32-bit integer.  */
using I64 = std::int64_t;   /**< Signed 64-bit integer.  */

using U8  = std::uint8_t;   /**< Unsigned  8-bit integer. */
using U16 = std::uint16_t;  /**< Unsigned 16-bit integer. */
using U32 = std::uint32_t;  /**< Unsigned 32-bit integer. */
using U64 = std::uint64_t;  /**< Unsigned 64-bit integer. */

/**
 * @brief 32-bit IEEE-754 floating point.
 *
 * The flight target FPU is single-precision; `F32` is the default real type.
 * Use `F64` only where numerical analysis documents that 24 bits of mantissa
 * are insufficient (e.g. EKF covariance propagation), and record the
 * justification in the header of the module that uses it.
 */
using F32 = float;
using F64 = double;         /**< 64-bit IEEE-754 — requires documented need. */

static_assert(sizeof(F32) == 4U, "F32 must be 32 bits on this target");
static_assert(sizeof(F64) == 8U, "F64 must be 64 bits on this target");

/* ------------------------------------------------------------------------ */
/* Status reporting (rule #2: no exceptions; callers must check status).     */
/* ------------------------------------------------------------------------ */

/**
 * @brief Universal status code returned by fallible flight functions.
 *
 * Every fallible public function returns `Status` and is annotated
 * `[[nodiscard]]`, so an unchecked call fails compilation under `-Werror`.
 * Values are stable across builds: they appear verbatim in telemetry.
 */
enum class Status : U8 {
    kSuccess           = 0U,  /**< Operation completed nominally.            */
    kErrNotInitialized = 1U,  /**< Component used before successful init.    */
    kErrInvalidParam   = 2U,  /**< Argument outside its documented range.    */
    kErrNonFiniteInput = 3U,  /**< NaN or Inf received on a real-typed input.*/
    kErrIllegalTransition = 4U, /**< Rejected mission-phase transition.      */
    kErrSaturated      = 5U,  /**< Output limited; result is safe but clipped.*/
};

/**
 * @brief   Convenience predicate for nominal status.
 * @param   status  Status code under test.
 * @return  `true` if and only if @p status is `Status::kSuccess`.
 */
[[nodiscard]] constexpr bool IsSuccess(const Status status) noexcept {
    return (status == Status::kSuccess);
}

}  // namespace lls

#endif  // LLS_LLS_TYPES_HPP
