/**
 * @file    descent_guidance.cpp
 * @brief   Implementation of the state-keyed descent guidance.
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
                            std::isfinite(c.max_horizontal_rate_mps) &&
                            std::isfinite(c.horizontal_brake_decel_mps2) &&
                            std::isfinite(c.near_field_gain_hz);
    if (!all_finite) {
        return false;
    }
    const bool all_positive =
        (c.brake_decel_mps2 > 0.0F) && (c.max_descent_rate_mps > 0.0F) &&
        (c.final_descent_rate_mps > 0.0F) && (c.terminal_altitude_m > 0.0F) &&
        (c.engine_cutoff_altitude_m > 0.0F) &&
        (c.pitchover_altitude_m > 0.0F) &&
        (c.horizontal_rate_slope_hz > 0.0F) &&
        (c.max_horizontal_rate_mps > 0.0F) &&
        (c.horizontal_brake_decel_mps2 > 0.0F) && (c.near_field_gain_hz > 0.0F);
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
                               const F32 downrange_to_go_m,
                               GuidanceCommand* const cmd_out) const noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (cmd_out == nullptr) {
        LLS_ASSERT(cmd_out != nullptr); /* Caller wiring error.            */
        return Status::kErrInvalidParam;
    }
    if (!std::isfinite(altitude_m) || !std::isfinite(downrange_to_go_m)) {
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

    /* Horizontal channel (site targeting): the signed ground-speed command
     * toward the site is the most restrictive of the range braking
     * envelope (stop at the site), the near-field linear law (soft
     * arrival), the altitude ramp (vertical by the pitch-over gate), and
     * the ground-speed cap. */
    if (altitude_m > config_.pitchover_altitude_m) {
        const F32 range_m = std::fabs(downrange_to_go_m);

        const F32 envelope_speed =
            std::sqrt(2.0F * config_.horizontal_brake_decel_mps2 * range_m);
        const F32 near_field_speed = config_.near_field_gain_hz * range_m;
        const F32 ramp_speed = config_.horizontal_rate_slope_hz *
                               (altitude_m - config_.pitchover_altitude_m);
        LLS_ASSERT(ramp_speed >= 0.0F); /* Guarded by the branch above.    */

        F32 speed = envelope_speed;
        speed = (near_field_speed < speed) ? near_field_speed : speed;
        speed = (ramp_speed < speed) ? ramp_speed : speed;
        speed = (speed > config_.max_horizontal_rate_mps)
                    ? config_.max_horizontal_rate_mps
                    : speed;
        cmd.horizontal_rate_cmd_mps =
            (downrange_to_go_m < 0.0F) ? -speed : speed;
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
