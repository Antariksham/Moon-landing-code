/**
 * @file    test_vertical_nav_filter.cpp
 * @brief   Unit tests for the vertical-channel navigation filter.
 *
 * @details Covers the `Init()` acceptance matrix, every `Predict()` /
 *          `UpdateAltitude()` failure path, the filter's estimation
 *          behavior against analytic truth (propagation kinematics,
 *          covariance growth and contraction, measurement fusion, velocity
 *          observability through altitude fixes), and the innovation gate.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "gnc/navigation/vertical_nav_filter.hpp"

namespace lls {
namespace gnc {
namespace {

constexpr F32 kNaN = std::numeric_limits<F32>::quiet_NaN();
constexpr F32 kInf = std::numeric_limits<F32>::infinity();

/** Filter armed at a 500 m / -30 m/s state with the default tuning. */
[[nodiscard]] VerticalNavFilterConfig MakeConfig() {
    VerticalNavFilterConfig cfg{};
    cfg.initial_altitude_m = 500.0F;
    cfg.initial_velocity_mps = -30.0F;
    return cfg;
}

/* ------------------------------------------------------------------ */
/* Init() acceptance matrix                                            */
/* ------------------------------------------------------------------ */

TEST(VerticalNavFilterInit, AcceptsDefaultAndCustomConfigs) {
    VerticalNavFilter filter;
    EXPECT_EQ(filter.Init(VerticalNavFilterConfig{}), Status::kSuccess);
    EXPECT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
}

TEST(VerticalNavFilterInit, RejectsNonFiniteFields) {
    VerticalNavFilter filter;
    VerticalNavFilterConfig cfg = MakeConfig();
    cfg.initial_velocity_mps = kNaN;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.altimeter_noise_std_m = kInf;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);
}

TEST(VerticalNavFilterInit, RejectsNegativeInitialAltitude) {
    VerticalNavFilter filter;
    VerticalNavFilterConfig cfg = MakeConfig();
    cfg.initial_altitude_m = -1.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);
}

TEST(VerticalNavFilterInit, RejectsNonPositiveDeviationsAndGate) {
    VerticalNavFilter filter;
    VerticalNavFilterConfig cfg = MakeConfig();
    cfg.initial_altitude_std_m = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.initial_velocity_std_mps = -1.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.accel_noise_std_mps2 = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.innovation_gate_sigma = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);
}

TEST(VerticalNavFilterInit, FailedReInitDisarmsTheFilter) {
    VerticalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);

    VerticalNavFilterConfig bad = MakeConfig();
    bad.altimeter_noise_std_m = -1.0F;
    ASSERT_EQ(filter.Init(bad), Status::kErrInvalidParam);

    EXPECT_EQ(filter.Predict(0.0F, 0.02F), Status::kErrNotInitialized);
    EXPECT_EQ(filter.UpdateAltitude(500.0F), Status::kErrNotInitialized);
}

TEST(VerticalNavFilterInit, ArmsAtTheConfiguredState) {
    VerticalNavFilter filter;
    const VerticalNavFilterConfig cfg = MakeConfig();
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    const VerticalNavEstimate est = filter.GetEstimate();
    EXPECT_FLOAT_EQ(est.altitude_m, cfg.initial_altitude_m);
    EXPECT_FLOAT_EQ(est.velocity_mps, cfg.initial_velocity_mps);
    EXPECT_FLOAT_EQ(est.altitude_std_m, cfg.initial_altitude_std_m);
    EXPECT_FLOAT_EQ(est.velocity_std_mps, cfg.initial_velocity_std_mps);
    EXPECT_EQ(filter.GetRejectedMeasurementCount(), 0U);
}

/* ------------------------------------------------------------------ */
/* Failure paths                                                       */
/* ------------------------------------------------------------------ */

TEST(VerticalNavFilterPredict, RefusesBeforeInit) {
    VerticalNavFilter filter;
    EXPECT_EQ(filter.Predict(0.0F, 0.02F), Status::kErrNotInitialized);
}

TEST(VerticalNavFilterPredict, RejectsNonFiniteAcceleration) {
    VerticalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(filter.Predict(kNaN, 0.02F), Status::kErrNonFiniteInput);
    EXPECT_EQ(filter.Predict(kInf, 0.02F), Status::kErrNonFiniteInput);
}

TEST(VerticalNavFilterPredict, RejectsInvalidTimeStep) {
    VerticalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(filter.Predict(0.0F, 0.0F), Status::kErrInvalidParam);
    EXPECT_EQ(filter.Predict(0.0F, -0.02F), Status::kErrInvalidParam);
    EXPECT_EQ(filter.Predict(0.0F, VerticalNavFilter::kMaxDtSeconds + 0.1F),
              Status::kErrInvalidParam);
    EXPECT_EQ(filter.Predict(0.0F, kNaN), Status::kErrInvalidParam);
}

TEST(VerticalNavFilterUpdate, RefusesBeforeInit) {
    VerticalNavFilter filter;
    EXPECT_EQ(filter.UpdateAltitude(100.0F), Status::kErrNotInitialized);
}

TEST(VerticalNavFilterUpdate, RejectsNonFiniteMeasurement) {
    VerticalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(filter.UpdateAltitude(kNaN), Status::kErrNonFiniteInput);
    EXPECT_EQ(filter.UpdateAltitude(kInf), Status::kErrNonFiniteInput);
}

TEST(VerticalNavFilterUpdate, RejectsNegativeMeasurement) {
    VerticalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(filter.UpdateAltitude(-0.1F), Status::kErrInvalidParam);
}

TEST(VerticalNavFilterEstimate, ZerosWhenUninitialized) {
    const VerticalNavFilter filter;
    const VerticalNavEstimate est = filter.GetEstimate();
    EXPECT_FLOAT_EQ(est.altitude_m, 0.0F);
    EXPECT_FLOAT_EQ(est.velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(est.altitude_std_m, 0.0F);
    EXPECT_FLOAT_EQ(est.velocity_std_mps, 0.0F);
}

/* ------------------------------------------------------------------ */
/* Estimation behavior                                                 */
/* ------------------------------------------------------------------ */

TEST(VerticalNavFilterPredict, PropagatesConstantAccelKinematics) {
    VerticalNavFilter filter;
    const VerticalNavFilterConfig cfg = MakeConfig();
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    /* 1 s of constant -1.625 m/s^2 at 50 Hz, no measurements.            */
    const F32 accel = -1.625F;
    const F32 dt = 0.02F;
    for (I32 i = 0; i < 50; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(accel, dt), Status::kSuccess);
    }
    const VerticalNavEstimate est = filter.GetEstimate();
    /* h = h0 + v0*t + a*t^2/2; v = v0 + a*t.                             */
    EXPECT_NEAR(est.altitude_m, 500.0F - 30.0F - 0.8125F, 0.05F);
    EXPECT_NEAR(est.velocity_mps, -31.625F, 1.0e-3F);
}

TEST(VerticalNavFilterPredict, CovarianceGrowsWithoutMeasurements) {
    VerticalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);

    const VerticalNavEstimate before = filter.GetEstimate();
    for (I32 i = 0; i < 100; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(0.0F, 0.02F), Status::kSuccess);
    }
    const VerticalNavEstimate after = filter.GetEstimate();
    EXPECT_GT(after.altitude_std_m, before.altitude_std_m)
        << "Dead-reckoning must not become more confident";
    EXPECT_GT(after.velocity_std_mps, before.velocity_std_mps);
}

TEST(VerticalNavFilterUpdate, FusionPullsEstimateAndShrinksCovariance) {
    VerticalNavFilter filter;
    const VerticalNavFilterConfig cfg = MakeConfig(); /* 10 m initial std. */
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    const VerticalNavEstimate before = filter.GetEstimate();
    /* Truth is 495 m; the 500 m prior moves most of the way there
     * because the prior variance (100) dwarfs the sensor's (0.04).       */
    ASSERT_EQ(filter.UpdateAltitude(495.0F), Status::kSuccess);
    const VerticalNavEstimate after = filter.GetEstimate();

    EXPECT_LT(std::fabs(after.altitude_m - 495.0F), 0.05F);
    EXPECT_LT(after.altitude_std_m, 0.5F * before.altitude_std_m);
}

TEST(VerticalNavFilterUpdate, AltitudeFixesMakeVelocityObservable) {
    VerticalNavFilter filter;
    VerticalNavFilterConfig cfg = MakeConfig();
    cfg.initial_velocity_mps = -20.0F; /* Truth will be -30: 10 m/s error. */
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    /* Fly 4 s of truth (constant -30 m/s, no acceleration) with perfect
     * altimeter fixes at 10 Hz. The velocity error must collapse even
     * though velocity is never measured directly.                        */
    F64 truth_altitude = 500.0;
    for (I32 i = 1; i <= 200; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(0.0F, 0.02F), Status::kSuccess);
        truth_altitude -= 30.0 * 0.02;
        if ((i % 5) == 0) {
            ASSERT_EQ(filter.UpdateAltitude(static_cast<F32>(truth_altitude)),
                      Status::kSuccess);
        }
    }
    const VerticalNavEstimate est = filter.GetEstimate();
    EXPECT_NEAR(est.velocity_mps, -30.0F, 0.5F)
        << "10 m/s initial velocity error did not converge";
    EXPECT_NEAR(est.altitude_m, static_cast<F32>(truth_altitude), 0.5F);
}

/* ------------------------------------------------------------------ */
/* Innovation gate                                                     */
/* ------------------------------------------------------------------ */

TEST(VerticalNavFilterGate, RejectsOutlierAndLeavesStateUntouched) {
    VerticalNavFilter filter;
    /* Hover scenario (zero velocity) so the constant-altitude measurement
     * stream is consistent with the prediction and convergence is clean. */
    VerticalNavFilterConfig cfg = MakeConfig();
    cfg.initial_velocity_mps = 0.0F;
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    /* Converge first so the innovation variance is small.                */
    for (I32 i = 0; i < 20; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(0.0F, 0.02F), Status::kSuccess);
        ASSERT_EQ(filter.UpdateAltitude(500.0F), Status::kSuccess);
    }
    const VerticalNavEstimate before = filter.GetEstimate();

    /* A 400 m jump is hundreds of sigmas: a glitched radar return.       */
    EXPECT_EQ(filter.UpdateAltitude(900.0F), Status::kErrMeasurementRejected);
    EXPECT_EQ(filter.GetRejectedMeasurementCount(), 1U);

    const VerticalNavEstimate after = filter.GetEstimate();
    EXPECT_FLOAT_EQ(after.altitude_m, before.altitude_m);
    EXPECT_FLOAT_EQ(after.velocity_mps, before.velocity_mps);
    EXPECT_FLOAT_EQ(after.altitude_std_m, before.altitude_std_m);

    /* A sane return afterwards is fused normally.                        */
    EXPECT_EQ(filter.UpdateAltitude(500.1F), Status::kSuccess);
    EXPECT_EQ(filter.GetRejectedMeasurementCount(), 1U);
}

}  // namespace
}  // namespace gnc
}  // namespace lls
