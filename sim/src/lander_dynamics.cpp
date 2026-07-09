/**
 * @file    lander_dynamics.cpp
 * @brief   Implementation of the 1-DOF lunar lander truth dynamics.
 *
 * @see     lander_dynamics.hpp for the model description and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "lander_dynamics.hpp"

#include <cmath>

namespace lls {
namespace sim {

namespace {

[[nodiscard]] bool AreParamsValid(const VehicleParams& p) noexcept {
    const bool finite =
        std::isfinite(p.dry_mass_kg) && std::isfinite(p.propellant_mass_kg) &&
        std::isfinite(p.max_thrust_n) && std::isfinite(p.min_throttle_frac) &&
        std::isfinite(p.specific_impulse_s);
    if (!finite) {
        return false;
    }
    return (p.dry_mass_kg > 0.0) && (p.propellant_mass_kg > 0.0) &&
           (p.max_thrust_n > 0.0) && (p.specific_impulse_s > 0.0) &&
           (p.min_throttle_frac > 0.0) && (p.min_throttle_frac <= 1.0);
}

}  // namespace

Status LanderDynamics::Init(const VehicleParams& params,
                            const F64 initial_altitude_m,
                            const F64 initial_velocity_mps) noexcept {
    if (!AreParamsValid(params) || !std::isfinite(initial_altitude_m) ||
        !std::isfinite(initial_velocity_mps) || (initial_altitude_m <= 0.0)) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }

    params_ = params;
    propellant_kg_ = params.propellant_mass_kg;
    state_.altitude_m = initial_altitude_m;
    state_.velocity_mps = initial_velocity_mps;
    state_.mass_kg = params.dry_mass_kg + params.propellant_mass_kg;
    touched_down_ = false;
    is_initialized_ = true;
    return Status::kSuccess;
}

Status LanderDynamics::Step(const F64 throttle_cmd_frac,
                            const F64 dt_s) noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (!std::isfinite(dt_s) || (dt_s <= 0.0) || (dt_s > 0.1) ||
        !std::isfinite(throttle_cmd_frac) || (throttle_cmd_frac < 0.0)) {
        return Status::kErrInvalidParam;
    }
    if (touched_down_) {
        return Status::kSuccess; /* Sitting on the surface: nothing moves. */
    }

    /* Engine model: 0 commands cutoff; anything else is clamped into the
     * physically realizable deep-throttle band. Dry tanks produce nothing. */
    F64 throttle = 0.0;
    if ((throttle_cmd_frac > 0.0) && (propellant_kg_ > 0.0)) {
        throttle = throttle_cmd_frac;
        throttle = (throttle < params_.min_throttle_frac)
                       ? params_.min_throttle_frac
                       : throttle;
        throttle = (throttle > 1.0) ? 1.0 : throttle;
    }
    const F64 thrust_n = throttle * params_.max_thrust_n;

    /* Propellant depletion: mdot = T / (Isp * g0), bounded by what's left. */
    const F64 mdot_kgps =
        thrust_n / (params_.specific_impulse_s * kStandardGravityMps2);
    F64 burned_kg = mdot_kgps * dt_s;
    burned_kg = (burned_kg > propellant_kg_) ? propellant_kg_ : burned_kg;
    propellant_kg_ -= burned_kg;
    state_.mass_kg = params_.dry_mass_kg + propellant_kg_;

    /* Semi-implicit Euler: update velocity, then integrate position with
     * the new velocity. */
    const F64 accel_mps2 = (thrust_n / state_.mass_kg) - kLunarGravityMps2;
    state_.velocity_mps += accel_mps2 * dt_s;
    state_.altitude_m += state_.velocity_mps * dt_s;

    if (state_.altitude_m <= 0.0) {
        state_.altitude_m = 0.0;
        touched_down_ = true;
    }
    return Status::kSuccess;
}

const LanderState& LanderDynamics::GetState() const noexcept {
    return state_;
}

F64 LanderDynamics::GetPropellantRemainingKg() const noexcept {
    return propellant_kg_;
}

bool LanderDynamics::HasTouchedDown() const noexcept {
    return touched_down_;
}

}  // namespace sim
}  // namespace lls
