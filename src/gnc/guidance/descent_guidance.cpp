/**
 * @file    descent_guidance.cpp
 * @brief   Implementation of the altitude-keyed descent guidance.
 *
 * @see     descent_guidance.hpp for the profile equations and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "gnc/guidance/descent_guidance.hpp"

#include <cmath>

#include "lls/lls_assert.hpp"

namespace lls {
namespace gnc {

namespace {

/**
 * @brief   Whether a candidate profile satisfies the `Init()` contract.
 * @param   c  Candidate configuration.
 * @return  `true` if every field is finite and positive and the altitude
 *          gates are ordered cutoff < terminal <= pitch-over.
 */
[[nodiscard]] bool IsConfigValid(const DescentGuidanceConfig& c) noexcept {
    const bool all_finite = std::isfinite(c.brake_decel_mps2) &&
                            std::isfinite(c.max_descent_rate_mps) &&
                            std::isfinite(c.final_descent_rate_mps) &&
                            std::isfinite(c.terminal_altitude_m) &&
                            std::isfinite(c.engine_cutoff_altitude_m) &&
                            std::isfinite(c.pitchover_altitude_m) &&
                            std::isfinite(c.horizontal_rate_slope_hz) &&
                            std::isfinite(c.max_horizontal_rate_mps);
    if (!all_finite) {
        return false;
    }
    const bool all_positive =
        (c.brake_decel_mps2 > 0.0F) && (c.max_descent_rate_mps > 0.0F) &&
        (c.final_descent_rate_mps > 0.0F) && (c.terminal_altitude_m > 0.0F) &&
        (c.engine_cutoff_altitude_m > 0.0F) &&
        (c.pitchover_altitude_m > 0.0F) &&
        (c.horizontal_rate_slope_hz > 0.0F) &&
        (c.max_horizontal_rate_mps > 0.0F);
    if (!all_positive) {
        return false;
    }
    return (c.engine_cutoff_altitude_m < c.terminal_altitude_m) &&
           (c.terminal_altitude_m <= c.pitchover_altitude_m);
}

}  // namespace

Status DescentGuidance::Init(const DescentGuidanceConfig& config) noexcept {
    if (!IsConfigValid(config)) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }
    config_ = config;
    is_initialized_ = true;
    return Status::kSuccess;
}

Status DescentGuidance::Update(const F32 altitude_m,
                               GuidanceCommand* const cmd_out) const noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (cmd_out == nullptr) {
        LLS_ASSERT(cmd_out != nullptr); /* Caller wiring error.            */
        return Status::kErrInvalidParam;
    }
    if (!std::isfinite(altitude_m)) {
        return Status::kErrNonFiniteInput;
    }
    if (altitude_m < 0.0F) {
        return Status::kErrInvalidParam;
    }

    GuidanceCommand cmd{};

    /* Vertical channel: braking envelope into the constant-rate segment. */
    if (altitude_m > config_.terminal_altitude_m) {
        const F32 envelope_arg = 2.0F * config_.brake_decel_mps2 *
                                 (altitude_m - config_.terminal_altitude_m);
        LLS_ASSERT(envelope_arg >= 0.0F); /* Guarded by the branch above.  */
        F32 speed = config_.final_descent_rate_mps + std::sqrt(envelope_arg);
        speed = (speed > config_.max_descent_rate_mps)
                    ? config_.max_descent_rate_mps
                    : speed;
        cmd.vertical_rate_cmd_mps = -speed;
    } else {
        cmd.vertical_rate_cmd_mps = -config_.final_descent_rate_mps;
    }

    /* Horizontal channel: allowed ground speed ramps to zero at the
     * pitch-over gate, forcing the vehicle vertical before terminal
     * descent. */
    if (altitude_m > config_.pitchover_altitude_m) {
        F32 ground_speed = config_.horizontal_rate_slope_hz *
                           (altitude_m - config_.pitchover_altitude_m);
        ground_speed = (ground_speed > config_.max_horizontal_rate_mps)
                           ? config_.max_horizontal_rate_mps
                           : ground_speed;
        cmd.horizontal_rate_cmd_mps = ground_speed;
        cmd.terminal_phase = false;
    } else {
        cmd.horizontal_rate_cmd_mps = 0.0F;
        cmd.terminal_phase = true;
    }

    cmd.engine_cutoff = (altitude_m <= config_.engine_cutoff_altitude_m);

    /* A cutoff request implies the vehicle is already in the terminal
     * phase — the gate ordering enforced by Init() guarantees it. */
    LLS_ASSERT(!cmd.engine_cutoff || cmd.terminal_phase);

    *cmd_out = cmd;
    return Status::kSuccess;
}

}  // namespace gnc
}  // namespace lls
