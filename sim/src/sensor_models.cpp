/**
 * @file    sensor_models.cpp
 * @brief   Implementation of the deterministic sensor error models.
 *
 * @see     sensor_models.hpp for the model descriptions and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "sensor_models.hpp"

#include <cmath>

namespace lls {
namespace sim {

namespace {

constexpr F64 kTwoPi = 6.283185307179586;

}  // namespace

void GaussianNoiseGenerator::Seed(const U64 seed) noexcept {
    /* xorshift state must be non-zero; remap 0 to a fixed constant. */
    state_ = (seed != 0ULL) ? seed : 0x9E3779B97F4A7C15ULL;
    has_spare_ = false;
    spare_ = 0.0;
}

U64 GaussianNoiseGenerator::NextU64() noexcept {
    /* xorshift64* (Vigna): passes BigCrush except MatrixRank; more than
     * adequate for noise injection. */
    U64 x = state_;
    x ^= x >> 12U;
    x ^= x << 25U;
    x ^= x >> 27U;
    state_ = x;
    return x * 0x2545F4914F6CDD1DULL;
}

F64 GaussianNoiseGenerator::NextUniform() noexcept {
    /* Top 53 bits to (0, 1]: never returns 0, so log() below is safe. */
    const U64 bits = NextU64() >> 11U;
    return (static_cast<F64>(bits) + 1.0) * (1.0 / 9007199254740992.0);
}

F64 GaussianNoiseGenerator::NextGaussian() noexcept {
    if (has_spare_) {
        has_spare_ = false;
        return spare_;
    }
    /* Box-Muller: two uniforms -> two independent normals. */
    const F64 u1 = NextUniform();
    const F64 u2 = NextUniform();
    const F64 radius = std::sqrt(-2.0 * std::log(u1));
    const F64 angle = kTwoPi * u2;
    spare_ = radius * std::sin(angle);
    has_spare_ = true;
    return radius * std::cos(angle);
}

Status ImuModel::Init(const ImuModelParams& params) noexcept {
    const bool valid = std::isfinite(params.accel_noise_std_mps2) &&
                       std::isfinite(params.accel_bias_mps2) &&
                       (params.accel_noise_std_mps2 >= 0.0);
    if (!valid) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }
    params_ = params;
    noise_.Seed(params.noise_seed);
    is_initialized_ = true;
    return Status::kSuccess;
}

F64 ImuModel::MeasureAccel(const F64 true_accel_mps2) noexcept {
    if (!is_initialized_) {
        return true_accel_mps2;
    }
    return true_accel_mps2 + params_.accel_bias_mps2 +
           (params_.accel_noise_std_mps2 * noise_.NextGaussian());
}

Status AltimeterModel::Init(const AltimeterModelParams& params) noexcept {
    const bool valid = std::isfinite(params.noise_std_m) &&
                       (params.noise_std_m >= 0.0) &&
                       (params.update_divisor >= 1U);
    if (!valid) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }
    params_ = params;
    noise_.Seed(params.noise_seed);
    is_initialized_ = true;
    return Status::kSuccess;
}

F64 AltimeterModel::MeasureAltitude(const F64 true_altitude_m) noexcept {
    if (!is_initialized_) {
        return true_altitude_m;
    }
    const F64 measured =
        true_altitude_m + (params_.noise_std_m * noise_.NextGaussian());
    return (measured < 0.0) ? 0.0 : measured;
}

U32 AltimeterModel::GetUpdateDivisor() const noexcept {
    return params_.update_divisor;
}

}  // namespace sim
}  // namespace lls
