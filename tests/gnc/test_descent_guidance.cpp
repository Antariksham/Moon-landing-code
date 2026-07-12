/**
 * @file    test_descent_guidance.cpp
 * @brief   Unit tests for the state-keyed descent guidance.
 *
 * @details Covers the full `Init()` acceptance matrix, every `Update()`
 *          failure path (off-nominal inputs are mandatory per the testing
 *          policy), and the shape of both profile channels: the vertical
 *          braking envelope and the site-targeting horizontal law (range
 *          envelope, near field, altitude ramp, sign handling), plus the
 *          pitch-over/cutoff discretes.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "gnc/guidance/descent_guidance.hpp"

namespace lls {
namespace gnc {
namespace {

constexpr F32 kNaN = std::numeric_limits<F32>::quiet_NaN();
constexpr F32 kInf = std::numeric_limits<F32>::infinity();

/** A range so large that only the altitude ramp / cap can be binding. */
constexpr F32 kFarRangeM = 1.0e6F;

/* ------------------------------------------------------------------ */
/* Init() acceptance matrix                                            */
/* ------------------------------------------------------------------ */

TEST(DescentGuidanceInit, AcceptsDefaultConfig) {
    DescentGuidance guidance;
    EXPECT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);
}

TEST(DescentGuidanceInit, RejectsNonFiniteFields) {
    DescentGuidance guidance;
    DescentGuidanceConfig cfg{};
    cfg.brake_decel_mps2 = kNaN;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);

    cfg = DescentGuidanceConfig{};
    cfg.max_horizontal_rate_mps = kInf;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);

    cfg = DescentGuidanceConfig{};
    cfg.near_field_gain_hz = kNaN;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);
}

TEST(DescentGuidanceInit, RejectsNonPositiveFields) {
    DescentGuidance guidance;
    DescentGuidanceConfig cfg{};
    cfg.horizontal_rate_slope_hz = 0.0F;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);

    cfg = DescentGuidanceConfig{};
    cfg.final_descent_rate_mps = -1.0F;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);

    cfg = DescentGuidanceConfig{};
    cfg.horizontal_brake_decel_mps2 = 0.0F;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);

    cfg = DescentGuidanceConfig{};
    cfg.near_field_gain_hz = -0.1F;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);
}

TEST(DescentGuidanceInit, RejectsMisorderedAltitudeGates) {
    DescentGuidance guidance;

    /* Cutoff at/above the terminal gate. */
    DescentGuidanceConfig cfg{};
    cfg.engine_cutoff_altitude_m = cfg.terminal_altitude_m;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);

    /* Pitch-over below the terminal gate. */
    cfg = DescentGuidanceConfig{};
    cfg.pitchover_altitude_m = cfg.terminal_altitude_m - 1.0F;
    EXPECT_EQ(guidance.Init(cfg), Status::kErrInvalidParam);
}

TEST(DescentGuidanceInit, FailedReInitDisarmsTheModule) {
    DescentGuidance guidance;
    ASSERT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);

    DescentGuidanceConfig bad{};
    bad.brake_decel_mps2 = -1.0F;
    ASSERT_EQ(guidance.Init(bad), Status::kErrInvalidParam);

    GuidanceCommand cmd{};
    EXPECT_EQ(guidance.Update(100.0F, kFarRangeM, &cmd),
              Status::kErrNotInitialized);
}

/* ------------------------------------------------------------------ */
/* Update() failure paths                                              */
/* ------------------------------------------------------------------ */

TEST(DescentGuidanceUpdate, RefusesBeforeInit) {
    const DescentGuidance guidance;
    GuidanceCommand cmd{};
    EXPECT_EQ(guidance.Update(100.0F, kFarRangeM, &cmd),
              Status::kErrNotInitialized);
}

TEST(DescentGuidanceUpdate, RejectsNullOutputPointer) {
    DescentGuidance guidance;
    ASSERT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);
    EXPECT_EQ(guidance.Update(100.0F, kFarRangeM, nullptr),
              Status::kErrInvalidParam);
}

TEST(DescentGuidanceUpdate, RejectsNonFiniteInputs) {
    DescentGuidance guidance;
    ASSERT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);
    GuidanceCommand cmd{};
    EXPECT_EQ(guidance.Update(kNaN, kFarRangeM, &cmd),
              Status::kErrNonFiniteInput);
    EXPECT_EQ(guidance.Update(kInf, kFarRangeM, &cmd),
              Status::kErrNonFiniteInput);
    EXPECT_EQ(guidance.Update(100.0F, kNaN, &cmd), Status::kErrNonFiniteInput);
    EXPECT_EQ(guidance.Update(100.0F, kInf, &cmd), Status::kErrNonFiniteInput);
}

TEST(DescentGuidanceUpdate, RejectsNegativeAltitude) {
    DescentGuidance guidance;
    ASSERT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);
    GuidanceCommand cmd{};
    EXPECT_EQ(guidance.Update(-0.1F, kFarRangeM, &cmd),
              Status::kErrInvalidParam);
}

TEST(DescentGuidanceUpdate, FailureLeavesCommandUnmodified) {
    DescentGuidance guidance;
    ASSERT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);

    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(500.0F, kFarRangeM, &cmd), Status::kSuccess);
    const GuidanceCommand before = cmd;

    ASSERT_EQ(guidance.Update(kNaN, kFarRangeM, &cmd),
              Status::kErrNonFiniteInput);
    EXPECT_EQ(cmd.vertical_rate_cmd_mps, before.vertical_rate_cmd_mps);
    EXPECT_EQ(cmd.horizontal_rate_cmd_mps, before.horizontal_rate_cmd_mps);
    EXPECT_EQ(cmd.terminal_phase, before.terminal_phase);
    EXPECT_EQ(cmd.engine_cutoff, before.engine_cutoff);
}

/* ------------------------------------------------------------------ */
/* Vertical channel                                                    */
/* ------------------------------------------------------------------ */

TEST(DescentGuidanceVertical, CommandsFinalRateAtLowAltitude) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(cfg.terminal_altitude_m, 0.0F, &cmd),
              Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.vertical_rate_cmd_mps, -cfg.final_descent_rate_mps);

    ASSERT_EQ(guidance.Update(0.0F, 0.0F, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.vertical_rate_cmd_mps, -cfg.final_descent_rate_mps);
}

TEST(DescentGuidanceVertical, FollowsBrakingEnvelopeAboveTerminalGate) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    const F32 altitude = 60.0F;
    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(altitude, kFarRangeM, &cmd), Status::kSuccess);

    const F32 expected = -(cfg.final_descent_rate_mps +
                           std::sqrt(2.0F * cfg.brake_decel_mps2 *
                                     (altitude - cfg.terminal_altitude_m)));
    EXPECT_NEAR(cmd.vertical_rate_cmd_mps, expected, 1.0e-5F);
}

TEST(DescentGuidanceVertical, CapsCommandAtMaxDescentRate) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(10000.0F, kFarRangeM, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.vertical_rate_cmd_mps, -cfg.max_descent_rate_mps);
}

TEST(DescentGuidanceVertical, ProfileIsContinuousAtTerminalGate) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    /* The envelope's sqrt amplifies the altitude offset: an offset of
     * 1e-5 m maps to sqrt(2 * 1.2 * 1e-5) ~ 5e-3 m/s of command.        */
    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(cfg.terminal_altitude_m + 1.0e-5F, 0.0F, &cmd),
              Status::kSuccess);
    EXPECT_NEAR(cmd.vertical_rate_cmd_mps, -cfg.final_descent_rate_mps,
                1.0e-2F);
}

/* ------------------------------------------------------------------ */
/* Horizontal channel: altitude ramp (the pitch-over schedule)         */
/* ------------------------------------------------------------------ */

TEST(DescentGuidanceHorizontal, ZeroGroundSpeedAtAndBelowPitchover) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    /* Even with the site far away: below the pitch-over gate the descent
     * is vertical and the remaining range is the landing miss.           */
    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(cfg.pitchover_altitude_m, kFarRangeM, &cmd),
              Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.horizontal_rate_cmd_mps, 0.0F);
    EXPECT_TRUE(cmd.terminal_phase);

    ASSERT_EQ(
        guidance.Update(cfg.pitchover_altitude_m / 2.0F, kFarRangeM, &cmd),
        Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.horizontal_rate_cmd_mps, 0.0F);
    EXPECT_TRUE(cmd.terminal_phase);
}

TEST(DescentGuidanceHorizontal, RampBindsWhenSiteIsFarAway) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    const F32 altitude = cfg.pitchover_altitude_m + 500.0F;
    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(altitude, kFarRangeM, &cmd), Status::kSuccess);
    EXPECT_NEAR(cmd.horizontal_rate_cmd_mps,
                cfg.horizontal_rate_slope_hz * 500.0F, 1.0e-4F);
    EXPECT_FALSE(cmd.terminal_phase);
}

TEST(DescentGuidanceHorizontal, CapsGroundSpeedAtMax) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(50000.0F, kFarRangeM, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.horizontal_rate_cmd_mps, cfg.max_horizontal_rate_mps);
}

TEST(DescentGuidanceHorizontal, RampIsMonotonicInAltitude) {
    DescentGuidance guidance;
    ASSERT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);

    F32 previous = 0.0F;
    GuidanceCommand cmd{};
    for (I32 h = 0; h <= 3000; h += 10) { /* Bounded loop. */
        ASSERT_EQ(guidance.Update(static_cast<F32>(h), kFarRangeM, &cmd),
                  Status::kSuccess);
        EXPECT_GE(cmd.horizontal_rate_cmd_mps, previous - 1.0e-6F)
            << "Allowed ground speed must not grow as the vehicle descends "
               "(h = "
            << h << ")";
        previous = cmd.horizontal_rate_cmd_mps;
    }
}

/* ------------------------------------------------------------------ */
/* Horizontal channel: site targeting (milestone 5)                    */
/* ------------------------------------------------------------------ */

TEST(DescentGuidanceTargeting, RangeEnvelopeBindsApproachingTheSite) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    /* High altitude (ramp slack), medium range: sqrt envelope governs.   */
    const F32 range = 400.0F;
    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(3000.0F, range, &cmd), Status::kSuccess);
    const F32 envelope =
        std::sqrt(2.0F * cfg.horizontal_brake_decel_mps2 * range);
    EXPECT_NEAR(cmd.horizontal_rate_cmd_mps, envelope, 1.0e-4F);
}

TEST(DescentGuidanceTargeting, NearFieldIsLinearInRange) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    /* Small range: the linear law is below the sqrt envelope.            */
    const F32 range = 10.0F;
    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(3000.0F, range, &cmd), Status::kSuccess);
    EXPECT_NEAR(cmd.horizontal_rate_cmd_mps, cfg.near_field_gain_hz * range,
                1.0e-5F);
}

TEST(DescentGuidanceTargeting, ZeroRangeCommandsZeroGroundSpeed) {
    DescentGuidance guidance;
    ASSERT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);

    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(3000.0F, 0.0F, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.horizontal_rate_cmd_mps, 0.0F);
}

TEST(DescentGuidanceTargeting, OvershootFliesTheVehicleBack) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    GuidanceCommand forward{};
    GuidanceCommand back{};
    ASSERT_EQ(guidance.Update(3000.0F, 50.0F, &forward), Status::kSuccess);
    ASSERT_EQ(guidance.Update(3000.0F, -50.0F, &back), Status::kSuccess);
    EXPECT_GT(forward.horizontal_rate_cmd_mps, 0.0F);
    EXPECT_FLOAT_EQ(back.horizontal_rate_cmd_mps,
                    -forward.horizontal_rate_cmd_mps);
}

TEST(DescentGuidanceTargeting, CommandIsMonotonicInRange) {
    DescentGuidance guidance;
    ASSERT_EQ(guidance.Init(DescentGuidanceConfig{}), Status::kSuccess);

    F32 previous = 0.0F;
    GuidanceCommand cmd{};
    for (I32 r = 0; r <= 3000; r += 10) { /* Bounded loop. */
        ASSERT_EQ(guidance.Update(3000.0F, static_cast<F32>(r), &cmd),
                  Status::kSuccess);
        EXPECT_GE(cmd.horizontal_rate_cmd_mps, previous - 1.0e-6F)
            << "Commanded speed must not shrink as range-to-go grows (r = " << r
            << ")";
        previous = cmd.horizontal_rate_cmd_mps;
    }
}

/* ------------------------------------------------------------------ */
/* Discretes                                                           */
/* ------------------------------------------------------------------ */

TEST(DescentGuidanceDiscretes, EngineCutoffOnlyBelowCutoffAltitude) {
    DescentGuidance guidance;
    const DescentGuidanceConfig cfg{};
    ASSERT_EQ(guidance.Init(cfg), Status::kSuccess);

    GuidanceCommand cmd{};
    ASSERT_EQ(guidance.Update(cfg.engine_cutoff_altitude_m + 0.1F, 0.0F, &cmd),
              Status::kSuccess);
    EXPECT_FALSE(cmd.engine_cutoff);

    ASSERT_EQ(guidance.Update(cfg.engine_cutoff_altitude_m, 0.0F, &cmd),
              Status::kSuccess);
    EXPECT_TRUE(cmd.engine_cutoff);
    EXPECT_TRUE(cmd.terminal_phase) << "Cutoff implies terminal phase";
}

}  // namespace
}  // namespace gnc
}  // namespace lls
