/**
 * @file    pid_controller.cpp
 * @brief   Implementation of the SELENE flight-grade PID controller.
 *
 * @see     pid_controller.hpp for the full interface contract and the
 *          rationale behind derivative-on-measurement and
 *          conditional-integration anti-windup.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "gnc/control/pid_controller.hpp"

#include <cmath>

#include "lls/lls_assert.hpp"

namespace lls {
namespace gnc {

namespace {

/**
 * @brief   Clamp @p value to the closed interval [@p low, @p high].
 * @param   value  Input value; must be finite (caller-validated).
 * @param   low    Interval lower bound; must be <= @p high.
 * @param   high   Interval upper bound.
 * @return  The nearest value inside [low, high].
 */
[[nodiscard]] constexpr F32 ClampF32(const F32 value,
                                     const F32 low,
                                     const F32 high) noexcept {
    return (value < low) ? low : ((value > high) ? high : value);
}

/**
 * @brief   Validate a candidate PID configuration (see Init() contract).
 * @param   config  Configuration under test.
 * @return  `true` if every field is finite and mutually consistent.
 */
[[nodiscard]] bool IsConfigValid(const PidConfig& config) noexcept {
    const bool gains_finite = std::isfinite(config.kp) &&
                              std::isfinite(config.ki) &&
                              std::isfinite(config.kd);
    const bool limits_finite = std::isfinite(config.output_min) &&
                               std::isfinite(config.output_max) &&
                               std::isfinite(config.integrator_min) &&
                               std::isfinite(config.integrator_max);
    if (!gains_finite || !limits_finite) {
        return false;
    }

    const bool gains_non_negative =
        (config.kp >= 0.0F) && (config.ki >= 0.0F) && (config.kd >= 0.0F);
    const bool output_range_valid = (config.output_min < config.output_max);

    /* The integrator clamp must straddle zero so that Reset() -> 0 is
     * always a legal integrator state. */
    const bool integrator_range_valid =
        (config.integrator_min < config.integrator_max) &&
        (config.integrator_min <= 0.0F) && (config.integrator_max >= 0.0F);

    return gains_non_negative && output_range_valid && integrator_range_valid;
}

}  // namespace

Status PidController::Init(const PidConfig& config) noexcept {
    if (!IsConfigValid(config)) {
        /* A rejected configuration disarms the controller outright: running
         * terminal descent on half-updated gains is not an option. */
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }

    config_ = config;
    is_initialized_ = true;
    Reset();
    return Status::kSuccess;
}

Status PidController::Update(const F32 setpoint,
                             const F32 measurement,
                             const F32 dt_s,
                             F32* const command_out) noexcept {
    /* --- Contract checks: reject, never propagate, bad inputs. ---------- */
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (command_out == nullptr) {
        LLS_ASSERT(command_out != nullptr);  /* Programmer error: report.   */
        return Status::kErrInvalidParam;
    }
    if (!std::isfinite(dt_s) || (dt_s <= 0.0F) || (dt_s > kMaxDtSeconds)) {
        return Status::kErrInvalidParam;
    }
    if (!std::isfinite(setpoint) || !std::isfinite(measurement)) {
        return Status::kErrNonFiniteInput;
    }

    const F32 error = setpoint - measurement;

    /* --- Proportional term. --------------------------------------------- */
    const F32 p_term = config_.kp * error;

    /* --- Derivative term (on measurement, first cycle inhibited). -------
     * Differentiating the measurement rather than the error means a step
     * in the guidance setpoint (e.g. divert re-target) produces no impulse
     * on the actuator. The sign is negative: a rising measurement opposes
     * the command. On the first cycle after Init()/Reset() there is no
     * history, so the D contribution is defined to be zero.               */
    F32 d_term = 0.0F;
    if (has_prev_measurement_) {
        const F32 measurement_rate = (measurement - prev_measurement_) / dt_s;
        d_term = -(config_.kd * measurement_rate);
    }
    prev_measurement_ = measurement;
    has_prev_measurement_ = true;

    /* --- Tentative integrator step (committed only if it won't wind up). */
    const F32 integrator_candidate = ClampF32(
        integrator_ + (config_.ki * error * dt_s),
        config_.integrator_min,
        config_.integrator_max);

    /* --- Combine and saturate. ------------------------------------------ */
    const F32 unsaturated = p_term + integrator_candidate + d_term;
    const F32 command = ClampF32(unsaturated,
                                 config_.output_min,
                                 config_.output_max);

    /* --- Conditional-integration anti-windup. ---------------------------
     * Commit the integrator step unless the output is saturated AND the
     * error is pushing further into that same limit. During a long
     * throttle-limited braking burn this freezes the integrator instead of
     * letting it charge for minutes and then overshoot on recovery.       */
    const bool pushing_past_max = (unsaturated > config_.output_max) &&
                                  (error > 0.0F);
    const bool pushing_past_min = (unsaturated < config_.output_min) &&
                                  (error < 0.0F);
    if (!pushing_past_max && !pushing_past_min) {
        integrator_ = integrator_candidate;
    }

    /* Postconditions: the command handed to the actuator layer is always
     * finite and always within the configured limits. */
    LLS_ASSERT(std::isfinite(command));
    LLS_ASSERT((command >= config_.output_min) &&
               (command <= config_.output_max));

    *command_out = command;

    const bool saturated = (command != unsaturated);
    return saturated ? Status::kErrSaturated : Status::kSuccess;
}

void PidController::Reset() noexcept {
    integrator_ = 0.0F;
    prev_measurement_ = 0.0F;
    has_prev_measurement_ = false;
}

F32 PidController::GetIntegratorState() const noexcept {
    return integrator_;
}

}  // namespace gnc
}  // namespace lls
