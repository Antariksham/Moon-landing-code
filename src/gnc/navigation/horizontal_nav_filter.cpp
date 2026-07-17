/**
 * @file    horizontal_nav_filter.cpp
 * @brief   Implementation of the horizontal-channel navigation filter.
 *
 * @see     horizontal_nav_filter.hpp for the model and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "gnc/navigation/horizontal_nav_filter.hpp"

#include <cmath>

#include "lls/lls_assert.hpp"

namespace lls {
namespace gnc {

namespace {

/**
 * @brief   Whether a candidate configuration satisfies the Init contract.
 * @param   c  Candidate configuration.
 * @return  `true` if every field is finite, every deviation/gate is
 *          positive, and the bias walk is non-negative.
 */
[[nodiscard]] bool IsConfigValid(const HorizontalNavFilterConfig& c) noexcept {
    const bool all_finite = std::isfinite(c.initial_downrange_m) &&
                            std::isfinite(c.initial_velocity_mps) &&
                            std::isfinite(c.initial_downrange_std_m) &&
                            std::isfinite(c.initial_velocity_std_mps) &&
                            std::isfinite(c.initial_bias_std_mps2) &&
                            std::isfinite(c.accel_noise_std_mps2) &&
                            std::isfinite(c.bias_walk_std_mps2) &&
                            std::isfinite(c.trn_noise_std_m) &&
                            std::isfinite(c.innovation_gate_sigma);
    if (!all_finite) {
        return false;
    }
    return (c.initial_downrange_std_m > 0.0F) &&
           (c.initial_velocity_std_mps > 0.0F) &&
           (c.initial_bias_std_mps2 > 0.0F) &&
           (c.accel_noise_std_mps2 > 0.0F) && (c.bias_walk_std_mps2 >= 0.0F) &&
           (c.trn_noise_std_m > 0.0F) && (c.innovation_gate_sigma > 0.0F);
}

}  // namespace

Status HorizontalNavFilter::Init(
    const HorizontalNavFilterConfig& config) noexcept {
    if (!IsConfigValid(config)) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }

    config_ = config;
    downrange_m_ = static_cast<F64>(config.initial_downrange_m);
    velocity_mps_ = static_cast<F64>(config.initial_velocity_mps);
    bias_mps2_ = 0.0; /* Best prior for a turn-on bias is zero.            */

    const F64 sigma_r = static_cast<F64>(config.initial_downrange_std_m);
    const F64 sigma_v = static_cast<F64>(config.initial_velocity_std_mps);
    const F64 sigma_b = static_cast<F64>(config.initial_bias_std_mps2);
    p00_ = sigma_r * sigma_r;
    p01_ = 0.0;
    p02_ = 0.0;
    p11_ = sigma_v * sigma_v;
    p12_ = 0.0;
    p22_ = sigma_b * sigma_b;

    rejected_measurement_count_ = 0U;
    is_initialized_ = true;
    return Status::kSuccess;
}

Status HorizontalNavFilter::Predict(const F32 accel_meas_mps2,
                                    const F32 dt_s) noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (!std::isfinite(accel_meas_mps2)) {
        return Status::kErrNonFiniteInput;
    }
    if (!std::isfinite(dt_s) || (dt_s <= 0.0F) || (dt_s > kMaxDtSeconds)) {
        return Status::kErrInvalidParam;
    }

    const F64 dt = static_cast<F64>(dt_s);
    /* Bias-corrected acceleration drives the kinematics.                  */
    const F64 accel = static_cast<F64>(accel_meas_mps2) - bias_mps2_;

    /* State: constant-acceleration kinematics over the interval; the bias
     * is a random walk (no deterministic dynamics of its own).            */
    downrange_m_ += (velocity_mps_ * dt) + (0.5 * accel * dt * dt);
    velocity_mps_ += accel * dt;

    /* Covariance: P = F P F' + Q with F = [[1, dt, -dt^2/2],
     * [0, 1, -dt], [0, 0, 1]] (the bias enters the kinematics negated),
     * Q from the accelerometer white noise sigma_a (discrete white-noise
     * acceleration model) plus the bias random walk sigma_b.              */
    const F64 sigma_a = static_cast<F64>(config_.accel_noise_std_mps2);
    const F64 sigma_b = static_cast<F64>(config_.bias_walk_std_mps2);
    const F64 q_var = sigma_a * sigma_a;
    const F64 dt2 = dt * dt;
    const F64 half_dt2 = 0.5 * dt2;

    const F64 p00_new = p00_ + (2.0 * dt * p01_) - (2.0 * half_dt2 * p02_) +
                        (dt2 * p11_) - (2.0 * dt * half_dt2 * p12_) +
                        (half_dt2 * half_dt2 * p22_) +
                        (0.25 * q_var * dt2 * dt2);
    const F64 p01_new = p01_ - (dt * p02_) + (dt * p11_) -
                        ((half_dt2 + dt2) * p12_) + (dt * half_dt2 * p22_) +
                        (0.5 * q_var * dt * dt2);
    const F64 p02_new = p02_ + (dt * p12_) - (half_dt2 * p22_);
    const F64 p11_new = p11_ - (2.0 * dt * p12_) + (dt2 * p22_) + (q_var * dt2);
    const F64 p12_new = p12_ - (dt * p22_);
    const F64 p22_new = p22_ + (sigma_b * sigma_b * dt);
    p00_ = p00_new;
    p01_ = p01_new;
    p02_ = p02_new;
    p11_ = p11_new;
    p12_ = p12_new;
    p22_ = p22_new;

    LLS_ASSERT(p00_ > 0.0); /* Propagation must keep P positive-definite. */
    LLS_ASSERT(p11_ > 0.0);
    LLS_ASSERT(p22_ > 0.0);
    return Status::kSuccess;
}

Status HorizontalNavFilter::UpdatePosition(
    const F32 downrange_meas_m) noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (!std::isfinite(downrange_meas_m)) {
        return Status::kErrNonFiniteInput;
    }

    const F64 sigma_z = static_cast<F64>(config_.trn_noise_std_m);
    const F64 r_var = sigma_z * sigma_z;

    /* Innovation and its variance (H = [1, 0, 0]).                        */
    const F64 innovation = static_cast<F64>(downrange_meas_m) - downrange_m_;
    const F64 s_var = p00_ + r_var;
    LLS_ASSERT(s_var > 0.0); /* r_var > 0 by Init; p00 >= 0 by algebra.    */

    /* Gate: a fix this far from the prediction is a mis-registered map
     * match, not information. State is left untouched; telemetry sees the
     * count.                                                              */
    const F64 gate = static_cast<F64>(config_.innovation_gate_sigma);
    if ((innovation * innovation) > (gate * gate * s_var)) {
        ++rejected_measurement_count_;
        return Status::kErrMeasurementRejected;
    }

    /* Kalman gain and state correction.                                   */
    const F64 k0 = p00_ / s_var;
    const F64 k1 = p01_ / s_var;
    const F64 k2 = p02_ / s_var;
    downrange_m_ += k0 * innovation;
    velocity_mps_ += k1 * innovation;
    bias_mps2_ += k2 * innovation;

    /* Covariance: P = (I - K H) P, symmetrized (3x3, H = [1, 0, 0]).      */
    const F64 p00_new = (1.0 - k0) * p00_;
    const F64 p01_new = (1.0 - k0) * p01_;
    const F64 p02_new = (1.0 - k0) * p02_;
    const F64 p11_new = p11_ - (k1 * p01_);
    const F64 p12_new = p12_ - (k1 * p02_);
    const F64 p22_new = p22_ - (k2 * p02_);
    p00_ = p00_new;
    p01_ = p01_new;
    p02_ = p02_new;
    p11_ = p11_new;
    p12_ = p12_new;
    p22_ = p22_new;

    LLS_ASSERT(p00_ > 0.0); /* Update must keep P positive-definite.      */
    LLS_ASSERT(p11_ > 0.0);
    LLS_ASSERT(p22_ > 0.0);
    return Status::kSuccess;
}

HorizontalNavEstimate HorizontalNavFilter::GetEstimate() const noexcept {
    HorizontalNavEstimate estimate{};
    if (is_initialized_) {
        estimate.downrange_m = static_cast<F32>(downrange_m_);
        estimate.velocity_mps = static_cast<F32>(velocity_mps_);
        estimate.accel_bias_mps2 = static_cast<F32>(bias_mps2_);
        estimate.downrange_std_m = static_cast<F32>(std::sqrt(p00_));
        estimate.velocity_std_mps = static_cast<F32>(std::sqrt(p11_));
        estimate.accel_bias_std_mps2 = static_cast<F32>(std::sqrt(p22_));
    }
    return estimate;
}

U32 HorizontalNavFilter::GetRejectedMeasurementCount() const noexcept {
    return rejected_measurement_count_;
}

}  // namespace gnc
}  // namespace lls
