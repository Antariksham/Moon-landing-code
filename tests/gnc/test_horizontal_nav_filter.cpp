/**
 * @file    test_horizontal_nav_filter.cpp
 * @brief   Unit tests for the horizontal-channel navigation filter.
 *
 * @details Covers the `Init()` acceptance matrix, every `Predict()` /
 *          `UpdatePosition()` failure path, the filter's estimation
 *          behavior against analytic truth (propagation kinematics,
 *          covariance growth and contraction, measurement fusion, velocity
 *          AND bias observability through position fixes, dead-reckoning
 *          quality after a TRN blackout), and the innovation gate.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "gnc/navigation/horizontal_nav_filter.hpp"

namespace lls {
namespace gnc {
namespace {

constexpr F32 kNaN = std::numeric_limits<F32>::quiet_NaN();
constexpr F32 kInf = std::numeric_limits<F32>::infinity();

/** Filter armed at the approach gate (downrange 0, 60 m/s ground speed). */
[[nodiscard]] HorizontalNavFilterConfig MakeConfig() {
    HorizontalNavFilterConfig cfg{};
    cfg.initial_downrange_m = 0.0F;
    cfg.initial_velocity_mps = 60.0F;
    return cfg;
}

/* ------------------------------------------------------------------ */
/* Init() acceptance matrix                                            */
/* ------------------------------------------------------------------ */

TEST(HorizontalNavFilterInit, AcceptsDefaultAndCustomConfigs) {
    HorizontalNavFilter filter;
    EXPECT_EQ(filter.Init(HorizontalNavFilterConfig{}), Status::kSuccess);
    EXPECT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
}

TEST(HorizontalNavFilterInit, AcceptsNegativeInitialDownrange) {
    /* A map-relative coordinate is signed — unlike the vertical filter's
     * altitude, a negative initial position is a legal handover state.    */
    HorizontalNavFilter filter;
    HorizontalNavFilterConfig cfg = MakeConfig();
    cfg.initial_downrange_m = -250.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kSuccess);
    EXPECT_FLOAT_EQ(filter.GetEstimate().downrange_m, -250.0F);
}

TEST(HorizontalNavFilterInit, RejectsNonFiniteFields) {
    HorizontalNavFilter filter;
    HorizontalNavFilterConfig cfg = MakeConfig();
    cfg.initial_velocity_mps = kNaN;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.trn_noise_std_m = kInf;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.bias_walk_std_mps2 = kNaN;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);
}

TEST(HorizontalNavFilterInit, RejectsNonPositiveDeviationsAndGate) {
    HorizontalNavFilter filter;
    HorizontalNavFilterConfig cfg = MakeConfig();
    cfg.initial_downrange_std_m = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.initial_velocity_std_mps = -1.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.initial_bias_std_mps2 = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.accel_noise_std_mps2 = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.bias_walk_std_mps2 = -0.001F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.trn_noise_std_m = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.innovation_gate_sigma = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kErrInvalidParam);
}

TEST(HorizontalNavFilterInit, AcceptsZeroBiasWalk) {
    /* Constant-bias model: legal, just never re-opens the bias variance. */
    HorizontalNavFilter filter;
    HorizontalNavFilterConfig cfg = MakeConfig();
    cfg.bias_walk_std_mps2 = 0.0F;
    EXPECT_EQ(filter.Init(cfg), Status::kSuccess);
}

TEST(HorizontalNavFilterInit, FailedReInitDisarmsTheFilter) {
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);

    HorizontalNavFilterConfig bad = MakeConfig();
    bad.trn_noise_std_m = -1.0F;
    ASSERT_EQ(filter.Init(bad), Status::kErrInvalidParam);

    EXPECT_EQ(filter.Predict(0.0F, 0.02F), Status::kErrNotInitialized);
    EXPECT_EQ(filter.UpdatePosition(100.0F), Status::kErrNotInitialized);
}

TEST(HorizontalNavFilterInit, ArmsAtTheConfiguredStateWithZeroBias) {
    HorizontalNavFilter filter;
    const HorizontalNavFilterConfig cfg = MakeConfig();
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    const HorizontalNavEstimate est = filter.GetEstimate();
    EXPECT_FLOAT_EQ(est.downrange_m, cfg.initial_downrange_m);
    EXPECT_FLOAT_EQ(est.velocity_mps, cfg.initial_velocity_mps);
    EXPECT_FLOAT_EQ(est.accel_bias_mps2, 0.0F);
    EXPECT_FLOAT_EQ(est.downrange_std_m, cfg.initial_downrange_std_m);
    EXPECT_FLOAT_EQ(est.velocity_std_mps, cfg.initial_velocity_std_mps);
    EXPECT_FLOAT_EQ(est.accel_bias_std_mps2, cfg.initial_bias_std_mps2);
    EXPECT_EQ(filter.GetRejectedMeasurementCount(), 0U);
}

/* ------------------------------------------------------------------ */
/* Failure paths                                                       */
/* ------------------------------------------------------------------ */

TEST(HorizontalNavFilterPredict, RefusesBeforeInit) {
    HorizontalNavFilter filter;
    EXPECT_EQ(filter.Predict(0.0F, 0.02F), Status::kErrNotInitialized);
}

TEST(HorizontalNavFilterPredict, RejectsNonFiniteAcceleration) {
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(filter.Predict(kNaN, 0.02F), Status::kErrNonFiniteInput);
    EXPECT_EQ(filter.Predict(kInf, 0.02F), Status::kErrNonFiniteInput);
}

TEST(HorizontalNavFilterPredict, RejectsInvalidTimeStep) {
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(filter.Predict(0.0F, 0.0F), Status::kErrInvalidParam);
    EXPECT_EQ(filter.Predict(0.0F, -0.02F), Status::kErrInvalidParam);
    EXPECT_EQ(filter.Predict(0.0F, HorizontalNavFilter::kMaxDtSeconds + 0.1F),
              Status::kErrInvalidParam);
    EXPECT_EQ(filter.Predict(0.0F, kNaN), Status::kErrInvalidParam);
}

TEST(HorizontalNavFilterUpdate, RefusesBeforeInit) {
    HorizontalNavFilter filter;
    EXPECT_EQ(filter.UpdatePosition(100.0F), Status::kErrNotInitialized);
}

TEST(HorizontalNavFilterUpdate, RejectsNonFiniteMeasurement) {
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(filter.UpdatePosition(kNaN), Status::kErrNonFiniteInput);
    EXPECT_EQ(filter.UpdatePosition(kInf), Status::kErrNonFiniteInput);
}

TEST(HorizontalNavFilterUpdate, AcceptsNegativeMeasurement) {
    /* Contrast with the vertical filter: a radar range cannot be negative
     * but a map-relative fix can.                                         */
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(filter.UpdatePosition(-5.0F), Status::kSuccess);
}

TEST(HorizontalNavFilterEstimate, ZerosWhenUninitialized) {
    const HorizontalNavFilter filter;
    const HorizontalNavEstimate est = filter.GetEstimate();
    EXPECT_FLOAT_EQ(est.downrange_m, 0.0F);
    EXPECT_FLOAT_EQ(est.velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(est.accel_bias_mps2, 0.0F);
    EXPECT_FLOAT_EQ(est.downrange_std_m, 0.0F);
    EXPECT_FLOAT_EQ(est.velocity_std_mps, 0.0F);
    EXPECT_FLOAT_EQ(est.accel_bias_std_mps2, 0.0F);
}

/* ------------------------------------------------------------------ */
/* Estimation behavior                                                 */
/* ------------------------------------------------------------------ */

TEST(HorizontalNavFilterPredict, PropagatesConstantAccelKinematics) {
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);

    /* 1 s of constant -1.5 m/s^2 braking at 50 Hz, no fixes. The bias
     * estimate is zero, so the measured accel drives the state directly.  */
    const F32 accel = -1.5F;
    const F32 dt = 0.02F;
    for (I32 i = 0; i < 50; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(accel, dt), Status::kSuccess);
    }
    const HorizontalNavEstimate est = filter.GetEstimate();
    /* r = r0 + v0*t + a*t^2/2; v = v0 + a*t.                              */
    EXPECT_NEAR(est.downrange_m, 60.0F - 0.75F, 0.05F);
    EXPECT_NEAR(est.velocity_mps, 58.5F, 1.0e-3F);
    EXPECT_FLOAT_EQ(est.accel_bias_mps2, 0.0F); /* No fixes: no learning.  */
}

TEST(HorizontalNavFilterPredict, CovarianceGrowsWithoutMeasurements) {
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);

    const HorizontalNavEstimate before = filter.GetEstimate();
    for (I32 i = 0; i < 100; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(0.0F, 0.02F), Status::kSuccess);
    }
    const HorizontalNavEstimate after = filter.GetEstimate();
    EXPECT_GT(after.downrange_std_m, before.downrange_std_m)
        << "Dead-reckoning must not become more confident";
    EXPECT_GT(after.velocity_std_mps, before.velocity_std_mps);
    EXPECT_GT(after.accel_bias_std_mps2, before.accel_bias_std_mps2)
        << "The bias random walk must keep the bias variance open";
}

TEST(HorizontalNavFilterUpdate, FusionPullsEstimateAndShrinksCovariance) {
    HorizontalNavFilter filter;
    const HorizontalNavFilterConfig cfg = MakeConfig(); /* 10 m initial std. */
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    const HorizontalNavEstimate before = filter.GetEstimate();
    /* Truth is 8 m against a 0 m prior. Gain = P/(P+R) = 100/125 = 0.8,
     * so the estimate moves exactly 80% of the innovation: to 6.4 m.      */
    ASSERT_EQ(filter.UpdatePosition(8.0F), Status::kSuccess);
    const HorizontalNavEstimate after = filter.GetEstimate();

    EXPECT_NEAR(after.downrange_m, 6.4F, 0.01F);
    EXPECT_LT(after.downrange_std_m, 0.5F * before.downrange_std_m);
}

TEST(HorizontalNavFilterUpdate, PositionFixesMakeVelocityObservable) {
    HorizontalNavFilter filter;
    HorizontalNavFilterConfig cfg = MakeConfig();
    cfg.initial_velocity_mps = 50.0F; /* Truth will be 60: 10 m/s error.   */
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    /* Fly 8 s of truth (constant 60 m/s, no acceleration) with perfect
     * TRN fixes at 5 Hz. The velocity error must collapse even though
     * velocity is never measured directly.                                */
    F64 truth_downrange = 0.0;
    for (I32 i = 1; i <= 400; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(0.0F, 0.02F), Status::kSuccess);
        truth_downrange += 60.0 * 0.02;
        if ((i % 10) == 0) {
            ASSERT_EQ(filter.UpdatePosition(static_cast<F32>(truth_downrange)),
                      Status::kSuccess);
        }
    }
    const HorizontalNavEstimate est = filter.GetEstimate();
    EXPECT_NEAR(est.velocity_mps, 60.0F, 0.5F)
        << "10 m/s initial velocity error did not converge";
    EXPECT_NEAR(est.downrange_m, static_cast<F32>(truth_downrange), 2.0F);
}

TEST(HorizontalNavFilterUpdate, PositionFixesMakeAccelBiasObservable) {
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);

    /* Truth coasts at 60 m/s with zero true acceleration, but the IMU
     * reports a constant +0.03 m/s^2 bias. With perfect 5 Hz fixes over
     * 60 s the filter must pin the bias, even though the bias is never
     * measured directly.                                                  */
    const F32 bias_mps2 = 0.03F;
    F64 truth_downrange = 0.0;
    for (I32 i = 1; i <= 3000; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(bias_mps2, 0.02F), Status::kSuccess);
        truth_downrange += 60.0 * 0.02;
        if ((i % 10) == 0) {
            ASSERT_EQ(filter.UpdatePosition(static_cast<F32>(truth_downrange)),
                      Status::kSuccess);
        }
    }
    const HorizontalNavEstimate est = filter.GetEstimate();
    EXPECT_NEAR(est.accel_bias_mps2, bias_mps2, 0.01F)
        << "Bias state did not converge toward the injected bias";
    EXPECT_NEAR(est.velocity_mps, 60.0F, 0.2F);
}

TEST(HorizontalNavFilterUpdate, BiasCorrectionSurvivesTrnBlackout) {
    /* The mission case that justifies the third state: learn the bias
     * while TRN can see, then dead-reckon 30 s blind. Without the bias
     * state the velocity estimate would drift by 0.03 * 30 = 0.9 m/s —
     * most of the 1 m/s touchdown budget; with it the drift must stay
     * an order of magnitude smaller.                                      */
    HorizontalNavFilter filter;
    ASSERT_EQ(filter.Init(MakeConfig()), Status::kSuccess);

    const F32 bias_mps2 = 0.03F;
    F64 truth_downrange = 0.0;
    for (I32 i = 1; i <= 3000; ++i) { /* Bounded loop: 60 s with fixes.    */
        ASSERT_EQ(filter.Predict(bias_mps2, 0.02F), Status::kSuccess);
        truth_downrange += 60.0 * 0.02;
        if ((i % 10) == 0) {
            ASSERT_EQ(filter.UpdatePosition(static_cast<F32>(truth_downrange)),
                      Status::kSuccess);
        }
    }
    for (I32 i = 0; i < 1500; ++i) { /* Bounded loop: 30 s blind.          */
        ASSERT_EQ(filter.Predict(bias_mps2, 0.02F), Status::kSuccess);
        truth_downrange += 60.0 * 0.02;
    }
    const HorizontalNavEstimate est = filter.GetEstimate();
    EXPECT_NEAR(est.velocity_mps, 60.0F, 0.15F)
        << "Bias-corrected dead reckoning drifted in velocity";
    EXPECT_NEAR(est.downrange_m, static_cast<F32>(truth_downrange), 5.0F)
        << "Bias-corrected dead reckoning drifted in position";
}

/* ------------------------------------------------------------------ */
/* Innovation gate                                                     */
/* ------------------------------------------------------------------ */

TEST(HorizontalNavFilterGate, RejectsOutlierAndLeavesStateUntouched) {
    HorizontalNavFilter filter;
    /* Hover scenario (zero velocity) so the constant-position fix stream
     * is consistent with the prediction and convergence is clean.         */
    HorizontalNavFilterConfig cfg = MakeConfig();
    cfg.initial_velocity_mps = 0.0F;
    ASSERT_EQ(filter.Init(cfg), Status::kSuccess);

    /* Converge first so the innovation variance is small.                 */
    for (I32 i = 0; i < 50; ++i) { /* Bounded loop. */
        ASSERT_EQ(filter.Predict(0.0F, 0.02F), Status::kSuccess);
        ASSERT_EQ(filter.UpdatePosition(0.0F), Status::kSuccess);
    }
    const HorizontalNavEstimate before = filter.GetEstimate();

    /* A 500 m jump is a grossly mis-registered map match.                 */
    EXPECT_EQ(filter.UpdatePosition(500.0F), Status::kErrMeasurementRejected);
    EXPECT_EQ(filter.GetRejectedMeasurementCount(), 1U);

    const HorizontalNavEstimate after = filter.GetEstimate();
    EXPECT_FLOAT_EQ(after.downrange_m, before.downrange_m);
    EXPECT_FLOAT_EQ(after.velocity_mps, before.velocity_mps);
    EXPECT_FLOAT_EQ(after.accel_bias_mps2, before.accel_bias_mps2);
    EXPECT_FLOAT_EQ(after.downrange_std_m, before.downrange_std_m);

    /* A sane fix afterwards is fused normally.                            */
    EXPECT_EQ(filter.UpdatePosition(0.5F), Status::kSuccess);
    EXPECT_EQ(filter.GetRejectedMeasurementCount(), 1U);
}

}  // namespace
}  // namespace gnc
}  // namespace lls
