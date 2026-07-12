/**
 * @file    vertical_nav_filter.cpp
 * @brief   Implementation of the vertical-channel navigation filter.
 *
 * @see     vertical_nav_filter.hpp for the model and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "gnc/navigation/vertical_nav_filter.hpp"

#include <cmath>

#include "lls/lls_assert.hpp"

namespace lls {
namespace gnc {

namespace {

/**
 * @brief   Whether a candidate configuration satisfies the Init contract.
 * @param   c  Candidate configuration.
 * @return  `true` if every field is finite, the initial altitude is
 *          non-negative, and every deviation/gate is positive.
 */
[[nodiscard]] bool IsConfigValid(const VerticalNavFilterConfig& c) noexcept {
    const bool all_finite = std::isfinite(c.initial_altitude_m) &&
                            std::isfinite(c.initial_velocity_mps) &&
                            std::isfinite(c.initial_altitude_std_m) &&
                            std::isfinite(c.initial_velocity_std_mps) &&
                            std::isfinite(c.accel_noise_std_mps2) &&
                            std::isfinite(c.altimeter_noise_std_m) &&
                            std::isfinite(c.innovation_gate_sigma);
    if (!all_finite) {
        return false;
    }
    return (c.initial_altitude_m >= 0.0F) &&
           (c.initial_altitude_std_m > 0.0F) &&
           (c.initial_velocity_std_mps > 0.0F) &&
           (c.accel_noise_std_mps2 > 0.0F) &&
           (c.altimeter_noise_std_m > 0.0F) && (c.innovation_gate_sigma > 0.0F);
}

}  // namespace

Status VerticalNavFilter::Init(const VerticalNavFilterConfig& config) noexcept {
    if (!IsConfigValid(config)) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }

    config_ = config;
    altitude_m_ = static_cast<F64>(config.initial_altitude_m);
    velocity_mps_ = static_cast<F64>(config.initial_velocity_mps);

    const F64 sigma_h = static_cast<F64>(config.initial_altitude_std_m);
    const F64 sigma_v = static_cast<F64>(config.initial_velocity_std_mps);
    p00_ = sigma_h * sigma_h;
    p01_ = 0.0;
    p11_ = sigma_v * sigma_v;

    rejected_measurement_count_ = 0U;
    is_initialized_ = true;
    return Status::kSuccess;
}

Status VerticalNavFilter::Predict(const F32 vertical_accel_mps2,
                                  const F32 dt_s) noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (!std::isfinite(vertical_accel_mps2)) {
        return Status::kErrNonFiniteInput;
    }
    if (!std::isfinite(dt_s) || (dt_s <= 0.0F) || (dt_s > kMaxDtSeconds)) {
        return Status::kErrInvalidParam;
    }

    const F64 dt = static_cast<F64>(dt_s);
    const F64 accel = static_cast<F64>(vertical_accel_mps2);

    /* State: constant-acceleration kinematics over the interval.         */
    altitude_m_ += (velocity_mps_ * dt) + (0.5 * accel * dt * dt);
    velocity_mps_ += accel * dt;

    /* Covariance: P = F P F' + Q with F = [[1, dt], [0, 1]] and Q from
     * the accelerometer white noise sigma_a (discrete white-noise
     * acceleration model).                                               */
    const F64 sigma_a = static_cast<F64>(config_.accel_noise_std_mps2);
    const F64 q_var = sigma_a * sigma_a;
    const F64 dt2 = dt * dt;

    const F64 p00_new =
        p00_ + (2.0 * dt * p01_) + (dt2 * p11_) + (0.25 * q_var * dt2 * dt2);
    const F64 p01_new = p01_ + (dt * p11_) + (0.5 * q_var * dt * dt2);
    const F64 p11_new = p11_ + (q_var * dt2);
    p00_ = p00_new;
    p01_ = p01_new;
    p11_ = p11_new;

    LLS_ASSERT(p00_ > 0.0); /* Propagation must keep P positive-definite. */
    LLS_ASSERT(p11_ > 0.0);
    return Status::kSuccess;
}

Status VerticalNavFilter::UpdateAltitude(const F32 altitude_meas_m) noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (!std::isfinite(altitude_meas_m)) {
        return Status::kErrNonFiniteInput;
    }
    if (altitude_meas_m < 0.0F) {
        return Status::kErrInvalidParam;
    }

    const F64 sigma_r = static_cast<F64>(config_.altimeter_noise_std_m);
    const F64 r_var = sigma_r * sigma_r;

    /* Innovation and its variance (H = [1, 0]).                          */
    const F64 innovation = static_cast<F64>(altitude_meas_m) - altitude_m_;
    const F64 s_var = p00_ + r_var;
    LLS_ASSERT(s_var > 0.0); /* r_var > 0 by Init; p00 >= 0 by algebra.   */

    /* Gate: a return this far from the prediction is a glitch, not
     * information. State is left untouched; telemetry sees the count.    */
    const F64 gate = static_cast<F64>(config_.innovation_gate_sigma);
    if ((innovation * innovation) > (gate * gate * s_var)) {
        ++rejected_measurement_count_;
        return Status::kErrMeasurementRejected;
    }

    /* Kalman gain and state correction.                                  */
    const F64 k0 = p00_ / s_var;
    const F64 k1 = p01_ / s_var;
    altitude_m_ += k0 * innovation;
    velocity_mps_ += k1 * innovation;

    /* Covariance: P = (I - K H) P, symmetrized (2x2, H = [1, 0]).        */
    const F64 p00_new = (1.0 - k0) * p00_;
    const F64 p01_new = (1.0 - k0) * p01_;
    const F64 p11_new = p11_ - (k1 * p01_);
    p00_ = p00_new;
    p01_ = p01_new;
    p11_ = p11_new;

    LLS_ASSERT(p00_ > 0.0); /* Update must keep P positive-definite.      */
    LLS_ASSERT(p11_ > 0.0);
    return Status::kSuccess;
}

VerticalNavEstimate VerticalNavFilter::GetEstimate() const noexcept {
    VerticalNavEstimate estimate{};
    if (is_initialized_) {
        estimate.altitude_m = static_cast<F32>(altitude_m_);
        estimate.velocity_mps = static_cast<F32>(velocity_mps_);
        estimate.altitude_std_m = static_cast<F32>(std::sqrt(p00_));
        estimate.velocity_std_mps = static_cast<F32>(std::sqrt(p11_));
    }
    return estimate;
}

U32 VerticalNavFilter::GetRejectedMeasurementCount() const noexcept {
    return rejected_measurement_count_;
}

}  // namespace gnc
}  // namespace lls
