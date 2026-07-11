/**
 * @file    thrust_allocator.cpp
 * @brief   Implementation of the planar thrust-vector control allocation.
 *
 * @see     thrust_allocator.hpp for the geometry and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "gnc/control/thrust_allocator.hpp"

#include <cmath>

#include "lls/lls_assert.hpp"

namespace lls {
namespace gnc {

namespace {

/** @brief Upper bound (exclusive) for the pitch-authority limit, rad. */
constexpr F32 kMaxPitchLimitRad = 1.5707F; /* Just under pi/2. */

}  // namespace

Status ThrustAllocator::Init(const ThrustAllocatorConfig& config) noexcept {
    const bool valid = std::isfinite(config.max_thrust_n) &&
                       std::isfinite(config.max_pitch_rad) &&
                       (config.max_thrust_n > 0.0F) &&
                       (config.max_pitch_rad > 0.0F) &&
                       (config.max_pitch_rad < kMaxPitchLimitRad);
    if (!valid) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }
    config_ = config;
    is_initialized_ = true;
    return Status::kSuccess;
}

Status ThrustAllocator::Allocate(const F32 accel_cmd_x_mps2,
                                 const F32 accel_cmd_up_mps2, const F32 mass_kg,
                                 ThrustCommand* const cmd_out) const noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (cmd_out == nullptr) {
        LLS_ASSERT(cmd_out != nullptr); /* Caller wiring error.            */
        return Status::kErrInvalidParam;
    }
    if (!std::isfinite(accel_cmd_x_mps2) || !std::isfinite(accel_cmd_up_mps2)) {
        return Status::kErrNonFiniteInput;
    }
    if (!std::isfinite(mass_kg) || (mass_kg <= 0.0F)) {
        return Status::kErrInvalidParam;
    }

    /* An up-pointing engine cannot push the vehicle down: clamp the
     * vertical component to zero and let the pitch limit produce the
     * closest realizable command. */
    const F32 accel_up = (accel_cmd_up_mps2 > 0.0F) ? accel_cmd_up_mps2 : 0.0F;
    bool saturated = (accel_up != accel_cmd_up_mps2);

    const F32 magnitude_mps2 = std::sqrt((accel_cmd_x_mps2 * accel_cmd_x_mps2) +
                                         (accel_up * accel_up));
    LLS_ASSERT(std::isfinite(magnitude_mps2)); /* Finite in, finite out.   */

    ThrustCommand cmd{};
    if (magnitude_mps2 > 0.0F) {
        /* atan2 is well defined here: at least one argument is non-zero. */
        F32 pitch_rad = std::atan2(accel_cmd_x_mps2, accel_up);
        if (pitch_rad > config_.max_pitch_rad) {
            pitch_rad = config_.max_pitch_rad;
            saturated = true;
        } else if (pitch_rad < -config_.max_pitch_rad) {
            pitch_rad = -config_.max_pitch_rad;
            saturated = true;
        }
        cmd.pitch_cmd_rad = pitch_rad;

        F32 throttle = (mass_kg * magnitude_mps2) / config_.max_thrust_n;
        if (throttle > 1.0F) {
            throttle = 1.0F;
            saturated = true;
        }
        cmd.throttle_cmd_frac = throttle;
    }
    /* else: zero acceleration commanded — engine idle, attitude vertical. */

    LLS_ASSERT(cmd.throttle_cmd_frac >= 0.0F); /* Post-condition audit.    */
    LLS_ASSERT(std::fabs(cmd.pitch_cmd_rad) <= config_.max_pitch_rad);

    *cmd_out = cmd;
    return saturated ? Status::kErrSaturated : Status::kSuccess;
}

}  // namespace gnc
}  // namespace lls
