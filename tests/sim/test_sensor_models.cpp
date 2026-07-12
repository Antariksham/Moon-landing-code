/**
 * @file    test_sensor_models.cpp
 * @brief   Unit tests for the deterministic SIL sensor error models.
 *
 * @details Pins the noise generator to reproducibility and sane statistics,
 *          and the IMU/altimeter models to their documented error
 *          equations (bias, noise scaling, non-negative radar returns).
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sensor_models.hpp"

namespace lls {
namespace sim {
namespace {

/* ------------------------------------------------------------------ */
/* Gaussian noise generator                                            */
/* ------------------------------------------------------------------ */

TEST(GaussianNoiseGenerator, SameSeedSameSequence) {
    GaussianNoiseGenerator a;
    GaussianNoiseGenerator b;
    a.Seed(1234ULL);
    b.Seed(1234ULL);
    for (I32 i = 0; i < 100; ++i) { /* Bounded loop. */
        EXPECT_DOUBLE_EQ(a.NextGaussian(), b.NextGaussian());
    }
}

TEST(GaussianNoiseGenerator, DifferentSeedsDiverge) {
    GaussianNoiseGenerator a;
    GaussianNoiseGenerator b;
    a.Seed(1ULL);
    b.Seed(2ULL);
    bool any_different = false;
    for (I32 i = 0; i < 10; ++i) { /* Bounded loop. */
        if (a.NextGaussian() != b.NextGaussian()) {
            any_different = true;
        }
    }
    EXPECT_TRUE(any_different);
}

TEST(GaussianNoiseGenerator, ZeroSeedIsRemappedAndUsable) {
    GaussianNoiseGenerator gen;
    gen.Seed(0ULL);
    for (I32 i = 0; i < 100; ++i) { /* Bounded loop. */
        EXPECT_TRUE(std::isfinite(gen.NextGaussian()));
    }
}

TEST(GaussianNoiseGenerator, SampleStatisticsAreStandardNormal) {
    GaussianNoiseGenerator gen;
    gen.Seed(0xC0FFEEULL);

    const I32 n = 20000;
    F64 sum = 0.0;
    F64 sum_sq = 0.0;
    for (I32 i = 0; i < n; ++i) { /* Bounded loop. */
        const F64 x = gen.NextGaussian();
        sum += x;
        sum_sq += x * x;
    }
    const F64 mean = sum / n;
    const F64 variance = (sum_sq / n) - (mean * mean);
    EXPECT_NEAR(mean, 0.0, 0.03);
    EXPECT_NEAR(variance, 1.0, 0.05);
}

/* ------------------------------------------------------------------ */
/* IMU model                                                           */
/* ------------------------------------------------------------------ */

TEST(ImuModel, RejectsNonPhysicalParams) {
    ImuModel imu;
    ImuModelParams bad{};
    bad.accel_noise_std_mps2 = -0.1;
    EXPECT_EQ(imu.Init(bad), Status::kErrInvalidParam);

    bad = ImuModelParams{};
    bad.accel_bias_mps2 = std::numeric_limits<F64>::quiet_NaN();
    EXPECT_EQ(imu.Init(bad), Status::kErrInvalidParam);
}

TEST(ImuModel, PassesTruthThroughWhenUninitialized) {
    ImuModel imu;
    EXPECT_DOUBLE_EQ(imu.MeasureAccel(1.625), 1.625);
}

TEST(ImuModel, AppliesExactlyBiasWhenNoiseFree) {
    ImuModel imu;
    ImuModelParams params{};
    params.accel_noise_std_mps2 = 0.0;
    params.accel_bias_mps2 = 0.02;
    ASSERT_EQ(imu.Init(params), Status::kSuccess);

    EXPECT_DOUBLE_EQ(imu.MeasureAccel(1.0), 1.02);
    EXPECT_DOUBLE_EQ(imu.MeasureAccel(-3.0), -2.98);
}

TEST(ImuModel, NoiseIsReproducibleAcrossRuns) {
    ImuModel a;
    ImuModel b;
    const ImuModelParams params{};
    ASSERT_EQ(a.Init(params), Status::kSuccess);
    ASSERT_EQ(b.Init(params), Status::kSuccess);
    for (I32 i = 0; i < 50; ++i) { /* Bounded loop. */
        EXPECT_DOUBLE_EQ(a.MeasureAccel(1.0), b.MeasureAccel(1.0));
    }
}

/* ------------------------------------------------------------------ */
/* Altimeter model                                                     */
/* ------------------------------------------------------------------ */

TEST(AltimeterModel, RejectsNonPhysicalParams) {
    AltimeterModel altimeter;
    AltimeterModelParams bad{};
    bad.noise_std_m = -0.1;
    EXPECT_EQ(altimeter.Init(bad), Status::kErrInvalidParam);

    bad = AltimeterModelParams{};
    bad.update_divisor = 0U;
    EXPECT_EQ(altimeter.Init(bad), Status::kErrInvalidParam);
}

TEST(AltimeterModel, PassesTruthThroughWhenUninitialized) {
    AltimeterModel altimeter;
    EXPECT_DOUBLE_EQ(altimeter.MeasureAltitude(123.4), 123.4);
}

TEST(AltimeterModel, NoiseFreeModelIsExact) {
    AltimeterModel altimeter;
    AltimeterModelParams params{};
    params.noise_std_m = 0.0;
    ASSERT_EQ(altimeter.Init(params), Status::kSuccess);
    EXPECT_DOUBLE_EQ(altimeter.MeasureAltitude(500.0), 500.0);
}

TEST(AltimeterModel, NeverReturnsNegativeRange) {
    AltimeterModel altimeter;
    AltimeterModelParams params{};
    params.noise_std_m = 5.0; /* Huge noise against a 0.1 m truth.        */
    ASSERT_EQ(altimeter.Init(params), Status::kSuccess);

    for (I32 i = 0; i < 1000; ++i) { /* Bounded loop. */
        EXPECT_GE(altimeter.MeasureAltitude(0.1), 0.0);
    }
}

TEST(AltimeterModel, ReportsConfiguredUpdateDivisor) {
    AltimeterModel altimeter;
    AltimeterModelParams params{};
    params.update_divisor = 5U;
    ASSERT_EQ(altimeter.Init(params), Status::kSuccess);
    EXPECT_EQ(altimeter.GetUpdateDivisor(), 5U);
}

}  // namespace
}  // namespace sim
}  // namespace lls
