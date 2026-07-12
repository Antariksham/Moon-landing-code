/**
 * @file    sensor_models.hpp
 * @brief   Deterministic sensor error models for the SIL sim (milestone 3).
 *
 * @details Corrupts truth quantities the way the real sensors will, so the
 *          flight navigation filter earns its keep in the loop:
 *
 *            - `ImuModel` — accelerometer channel: constant bias plus
 *              white Gaussian noise per sample.
 *            - `AltimeterModel` — nadir radar altimeter: white Gaussian
 *              noise per return (bias-free; radar altimeter biases are
 *              calibrated out on the ground).
 *
 *          All randomness comes from `GaussianNoiseGenerator`, a
 *          fixed-seed xorshift64* PRNG with a Box-Muller transform: runs
 *          are bit-for-bit reproducible, which the regression tests rely
 *          on. No heap, no <random> (its distributions are not
 *          reproducible across standard-library implementations).
 *
 *          Simulation code is host-only and exempt from the flight rules,
 *          but follows the flight style anyway. Models use `F64`
 *          deliberately: they corrupt truth states, which are F64 so the
 *          simulator does not share the F32 flight code's rounding.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_SIM_SENSOR_MODELS_HPP
#define LLS_SIM_SENSOR_MODELS_HPP

#include "lls/lls_types.hpp"

namespace lls {
namespace sim {

/**
 * @brief Reproducible standard-normal deviate source.
 *
 * xorshift64* generator feeding a Box-Muller transform. Statistical
 * quality is far beyond what sensor-noise injection needs, and the
 * sequence is identical on every platform for a given seed.
 */
class GaussianNoiseGenerator {
 public:
    GaussianNoiseGenerator() noexcept = default;

    /**
     * @brief   Seed the generator.
     *
     * @param   seed  Any value; 0 is remapped internally (xorshift state
     *                must be non-zero).
     */
    void Seed(U64 seed) noexcept;

    /** @brief Next standard-normal deviate (mean 0, sigma 1). */
    [[nodiscard]] F64 NextGaussian() noexcept;

    /** @brief Next uniform deviate in (0, 1] (Bernoulli draws etc.). */
    [[nodiscard]] F64 NextUniform() noexcept;

 private:
    /** @brief Next raw xorshift64* draw. */
    [[nodiscard]] U64 NextU64() noexcept;

    U64 state_ = 0x9E3779B97F4A7C15ULL; /**< Non-zero default seed.        */
    F64 spare_ = 0.0;        /**< Second Box-Muller deviate, held.         */
    bool has_spare_ = false; /**< Whether `spare_` is loaded.              */
};

/** @brief Accelerometer channel error parameters. */
struct ImuModelParams {
    F64 accel_noise_std_mps2 = 0.05; /**< White noise per 50 Hz sample.    */
    F64 accel_bias_mps2 = 0.02;      /**< Constant turn-on bias.           */
    U64 noise_seed = 0x1A2B3C4D5E6F7081ULL; /**< PRNG seed for this run.   */
};

/**
 * @brief Accelerometer channel: truth in, biased + noisy measurement out.
 */
class ImuModel {
 public:
    ImuModel() noexcept = default;

    /**
     * @brief   Configure the error model.
     *
     * @param   params  Noise standard deviation must be finite and >= 0;
     *                  bias must be finite.
     *
     * @retval  Status::kSuccess          Model ready.
     * @retval  Status::kErrInvalidParam  A parameter is out of range.
     */
    [[nodiscard]] Status Init(const ImuModelParams& params) noexcept;

    /**
     * @brief   Corrupt one true acceleration sample.
     * @param   true_accel_mps2  Truth value from the dynamics.
     * @return  Measurement = truth + bias + white noise. Returns the truth
     *          unmodified if `Init()` has not succeeded.
     */
    [[nodiscard]] F64 MeasureAccel(F64 true_accel_mps2) noexcept;

 private:
    ImuModelParams params_{};
    GaussianNoiseGenerator noise_{};
    bool is_initialized_ = false;
};

/** @brief Radar-altimeter error parameters. */
struct AltimeterModelParams {
    F64 noise_std_m = 0.2;   /**< White noise per return.                    */
    U32 update_divisor = 5U; /**< Return every N control cycles (5 at
                                  50 Hz = 10 Hz altimeter), >= 1.          */
    U64 noise_seed = 0x0F1E2D3C4B5A6978ULL; /**< PRNG seed for this run.   */
};

/**
 * @brief Nadir radar altimeter: truth altitude in, noisy return out.
 */
class AltimeterModel {
 public:
    AltimeterModel() noexcept = default;

    /**
     * @brief   Configure the error model.
     *
     * @param   params  Noise standard deviation must be finite and >= 0;
     *                  the update divisor must be >= 1.
     *
     * @retval  Status::kSuccess          Model ready.
     * @retval  Status::kErrInvalidParam  A parameter is out of range.
     */
    [[nodiscard]] Status Init(const AltimeterModelParams& params) noexcept;

    /**
     * @brief   Corrupt one true altitude sample.
     * @param   true_altitude_m  Truth value from the dynamics.
     * @return  Measurement = max(0, truth + white noise) — a radar cannot
     *          return a negative range. Returns the truth unmodified if
     *          `Init()` has not succeeded.
     */
    [[nodiscard]] F64 MeasureAltitude(F64 true_altitude_m) noexcept;

    /** @brief Control cycles between returns (from the configuration). */
    [[nodiscard]] U32 GetUpdateDivisor() const noexcept;

 private:
    AltimeterModelParams params_{};
    GaussianNoiseGenerator noise_{};
    bool is_initialized_ = false;
};

}  // namespace sim
}  // namespace lls

#endif  // LLS_SIM_SENSOR_MODELS_HPP
