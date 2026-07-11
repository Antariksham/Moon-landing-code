/**
 * @file    lander_dynamics_3dof.cpp
 * @brief   Implementation of the 3-DOF planar lander truth dynamics.
 *
 * @see     lander_dynamics_3dof.hpp for the model description and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "lander_dynamics_3dof.hpp"

#include <cmath>

namespace lls {
namespace sim {

namespace {

constexpr F64 kHalfPi = 1.5707963267948966;

[[nodiscard]] bool AreParamsValid(const VehicleParams3Dof& p) noexcept {
    const bool finite =
        std::isfinite(p.dry_mass_kg) && std::isfinite(p.propellant_mass_kg) &&
        std::isfinite(p.max_thrust_n) && std::isfinite(p.min_throttle_frac) &&
        std::isfinite(p.specific_impulse_s) &&
        std::isfinite(p.pitch_inertia_kgm2) &&
        std::isfinite(p.max_rcs_torque_nm);
    if (!finite) {
        return false;
    }
    return (p.dry_mass_kg > 0.0) && (p.propellant_mass_kg > 0.0) &&
           (p.max_thrust_n > 0.0) && (p.specific_impulse_s > 0.0) &&
           (p.min_throttle_frac > 0.0) && (p.min_throttle_frac <= 1.0) &&
           (p.pitch_inertia_kgm2 > 0.0) && (p.max_rcs_torque_nm > 0.0);
}

[[nodiscard]] bool IsGateValid(const Gate3Dof& g) noexcept {
    const bool finite =
        std::isfinite(g.altitude_m) && std::isfinite(g.velocity_x_mps) &&
        std::isfinite(g.velocity_z_mps) && std::isfinite(g.pitch_rad) &&
        std::isfinite(g.pitch_rate_radps);
    if (!finite) {
        return false;
    }
    return (g.altitude_m > 0.0) && (std::fabs(g.pitch_rad) < kHalfPi);
}

}  // namespace

Status LanderDynamics3Dof::Init(const VehicleParams3Dof& params,
                                const Gate3Dof& gate) noexcept {
    if (!AreParamsValid(params) || !IsGateValid(gate)) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }

    params_ = params;
    propellant_kg_ = params.propellant_mass_kg;
    state_ = LanderState3Dof{};
    state_.altitude_m = gate.altitude_m;
    state_.velocity_x_mps = gate.velocity_x_mps;
    state_.velocity_z_mps = gate.velocity_z_mps;
    state_.pitch_rad = gate.pitch_rad;
    state_.pitch_rate_radps = gate.pitch_rate_radps;
    state_.mass_kg = params.dry_mass_kg + params.propellant_mass_kg;
    touched_down_ = false;
    is_initialized_ = true;
    return Status::kSuccess;
}

Status LanderDynamics3Dof::Step(const F64 throttle_cmd_frac,
                                const F64 torque_cmd_frac,
                                const F64 dt_s) noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (!std::isfinite(dt_s) || (dt_s <= 0.0) || (dt_s > 0.1) ||
        !std::isfinite(throttle_cmd_frac) || (throttle_cmd_frac < 0.0) ||
        !std::isfinite(torque_cmd_frac)) {
        return Status::kErrInvalidParam;
    }
    if (touched_down_) {
        return Status::kSuccess; /* Sitting on the surface: nothing moves. */
    }

    /* Engine model: identical semantics to the 1-DOF truth model. */
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

    /* RCS pitch couple, clamped to the hardware's authority. */
    F64 torque_frac = torque_cmd_frac;
    torque_frac = (torque_frac < -1.0) ? -1.0 : torque_frac;
    torque_frac = (torque_frac > 1.0) ? 1.0 : torque_frac;
    const F64 torque_nm = torque_frac * params_.max_rcs_torque_nm;

    /* Semi-implicit Euler: rates first, then states with the new rates.   */
    state_.pitch_rate_radps += (torque_nm / params_.pitch_inertia_kgm2) * dt_s;
    state_.pitch_rad += state_.pitch_rate_radps * dt_s;

    const F64 accel_x_mps2 =
        (thrust_n * std::sin(state_.pitch_rad)) / state_.mass_kg;
    const F64 accel_z_mps2 =
        ((thrust_n * std::cos(state_.pitch_rad)) / state_.mass_kg) -
        kLunarGravityMps2;
    state_.velocity_x_mps += accel_x_mps2 * dt_s;
    state_.velocity_z_mps += accel_z_mps2 * dt_s;
    state_.downrange_m += state_.velocity_x_mps * dt_s;
    state_.altitude_m += state_.velocity_z_mps * dt_s;

    if (state_.altitude_m <= 0.0) {
        state_.altitude_m = 0.0;
        touched_down_ = true;
    }
    return Status::kSuccess;
}

const LanderState3Dof& LanderDynamics3Dof::GetState() const noexcept {
    return state_;
}

F64 LanderDynamics3Dof::GetPropellantRemainingKg() const noexcept {
    return propellant_kg_;
}

bool LanderDynamics3Dof::HasTouchedDown() const noexcept {
    return touched_down_;
}

}  // namespace sim
}  // namespace lls
