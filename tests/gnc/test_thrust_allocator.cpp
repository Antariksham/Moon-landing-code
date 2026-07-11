/**
 * @file    test_thrust_allocator.cpp
 * @brief   Unit tests for the planar thrust-vector control allocation.
 *
 * @details Covers the `Init()` acceptance matrix, every `Allocate()`
 *          failure path, the allocation geometry (pitch from atan2,
 *          throttle from magnitude), and all saturation branches: pitch
 *          limit, throttle limit, and the non-positive vertical clamp.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "gnc/control/thrust_allocator.hpp"

namespace lls {
namespace gnc {
namespace {

constexpr F32 kNaN = std::numeric_limits<F32>::quiet_NaN();
constexpr F32 kInf = std::numeric_limits<F32>::infinity();

/** Test vehicle: 2500 N engine, 45 deg pitch authority (config values). */
[[nodiscard]] ThrustAllocatorConfig MakeConfig() {
    return ThrustAllocatorConfig{};
}

/* ------------------------------------------------------------------ */
/* Init() acceptance matrix                                            */
/* ------------------------------------------------------------------ */

TEST(ThrustAllocatorInit, AcceptsDefaultConfig) {
    ThrustAllocator allocator;
    EXPECT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);
}

TEST(ThrustAllocatorInit, RejectsNonFiniteFields) {
    ThrustAllocator allocator;
    ThrustAllocatorConfig cfg = MakeConfig();
    cfg.max_thrust_n = kNaN;
    EXPECT_EQ(allocator.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.max_pitch_rad = kInf;
    EXPECT_EQ(allocator.Init(cfg), Status::kErrInvalidParam);
}

TEST(ThrustAllocatorInit, RejectsNonPositiveThrust) {
    ThrustAllocator allocator;
    ThrustAllocatorConfig cfg = MakeConfig();
    cfg.max_thrust_n = 0.0F;
    EXPECT_EQ(allocator.Init(cfg), Status::kErrInvalidParam);
}

TEST(ThrustAllocatorInit, RejectsPitchLimitOutsideOpenInterval) {
    ThrustAllocator allocator;
    ThrustAllocatorConfig cfg = MakeConfig();
    cfg.max_pitch_rad = 0.0F;
    EXPECT_EQ(allocator.Init(cfg), Status::kErrInvalidParam);

    cfg = MakeConfig();
    cfg.max_pitch_rad = 1.6F; /* > pi/2: engine would point downward. */
    EXPECT_EQ(allocator.Init(cfg), Status::kErrInvalidParam);
}

TEST(ThrustAllocatorInit, FailedReInitDisarmsTheAllocator) {
    ThrustAllocator allocator;
    ASSERT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);

    ThrustAllocatorConfig bad = MakeConfig();
    bad.max_thrust_n = -1.0F;
    ASSERT_EQ(allocator.Init(bad), Status::kErrInvalidParam);

    ThrustCommand cmd{};
    EXPECT_EQ(allocator.Allocate(0.0F, 1.0F, 500.0F, &cmd),
              Status::kErrNotInitialized);
}

/* ------------------------------------------------------------------ */
/* Allocate() failure paths                                            */
/* ------------------------------------------------------------------ */

TEST(ThrustAllocatorAllocate, RefusesBeforeInit) {
    const ThrustAllocator allocator;
    ThrustCommand cmd{};
    EXPECT_EQ(allocator.Allocate(0.0F, 1.0F, 500.0F, &cmd),
              Status::kErrNotInitialized);
}

TEST(ThrustAllocatorAllocate, RejectsNullOutputPointer) {
    ThrustAllocator allocator;
    ASSERT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);
    EXPECT_EQ(allocator.Allocate(0.0F, 1.0F, 500.0F, nullptr),
              Status::kErrInvalidParam);
}

TEST(ThrustAllocatorAllocate, RejectsNonFiniteAccelerations) {
    ThrustAllocator allocator;
    ASSERT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);
    ThrustCommand cmd{};
    EXPECT_EQ(allocator.Allocate(kNaN, 1.0F, 500.0F, &cmd),
              Status::kErrNonFiniteInput);
    EXPECT_EQ(allocator.Allocate(0.0F, kInf, 500.0F, &cmd),
              Status::kErrNonFiniteInput);
}

TEST(ThrustAllocatorAllocate, RejectsNonPhysicalMass) {
    ThrustAllocator allocator;
    ASSERT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);
    ThrustCommand cmd{};
    EXPECT_EQ(allocator.Allocate(0.0F, 1.0F, 0.0F, &cmd),
              Status::kErrInvalidParam);
    EXPECT_EQ(allocator.Allocate(0.0F, 1.0F, -10.0F, &cmd),
              Status::kErrInvalidParam);
    EXPECT_EQ(allocator.Allocate(0.0F, 1.0F, kNaN, &cmd),
              Status::kErrInvalidParam);
}

TEST(ThrustAllocatorAllocate, FailureLeavesCommandUnmodified) {
    ThrustAllocator allocator;
    ASSERT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);

    ThrustCommand cmd{};
    cmd.pitch_cmd_rad = 0.123F;
    cmd.throttle_cmd_frac = 0.456F;
    ASSERT_EQ(allocator.Allocate(kNaN, 1.0F, 500.0F, &cmd),
              Status::kErrNonFiniteInput);
    EXPECT_FLOAT_EQ(cmd.pitch_cmd_rad, 0.123F);
    EXPECT_FLOAT_EQ(cmd.throttle_cmd_frac, 0.456F);
}

/* ------------------------------------------------------------------ */
/* Allocation geometry                                                 */
/* ------------------------------------------------------------------ */

TEST(ThrustAllocatorGeometry, PureVerticalCommandIsPitchZero) {
    ThrustAllocator allocator;
    const ThrustAllocatorConfig cfg = MakeConfig();
    ASSERT_EQ(allocator.Init(cfg), Status::kSuccess);

    const F32 accel = 2.0F;
    const F32 mass = 500.0F;
    ThrustCommand cmd{};
    ASSERT_EQ(allocator.Allocate(0.0F, accel, mass, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.pitch_cmd_rad, 0.0F);
    EXPECT_NEAR(cmd.throttle_cmd_frac, (mass * accel) / cfg.max_thrust_n,
                1.0e-6F);
}

TEST(ThrustAllocatorGeometry, AngledCommandMatchesAtan2AndMagnitude) {
    ThrustAllocator allocator;
    const ThrustAllocatorConfig cfg = MakeConfig();
    ASSERT_EQ(allocator.Init(cfg), Status::kSuccess);

    const F32 ax = 0.8F;
    const F32 az = 1.6F;
    const F32 mass = 550.0F;
    ThrustCommand cmd{};
    ASSERT_EQ(allocator.Allocate(ax, az, mass, &cmd), Status::kSuccess);

    EXPECT_NEAR(cmd.pitch_cmd_rad, std::atan2(ax, az), 1.0e-6F);
    const F32 magnitude = std::sqrt((ax * ax) + (az * az));
    EXPECT_NEAR(cmd.throttle_cmd_frac, (mass * magnitude) / cfg.max_thrust_n,
                1.0e-6F);
}

TEST(ThrustAllocatorGeometry, NegativeDownrangeCommandPitchesNegative) {
    ThrustAllocator allocator;
    ASSERT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);

    ThrustCommand cmd{};
    ASSERT_EQ(allocator.Allocate(-0.8F, 1.6F, 550.0F, &cmd), Status::kSuccess);
    EXPECT_LT(cmd.pitch_cmd_rad, 0.0F);
}

TEST(ThrustAllocatorGeometry, ZeroCommandIsEngineIdleVertical) {
    ThrustAllocator allocator;
    ASSERT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);

    ThrustCommand cmd{};
    cmd.pitch_cmd_rad = 0.5F;
    cmd.throttle_cmd_frac = 0.5F;
    ASSERT_EQ(allocator.Allocate(0.0F, 0.0F, 500.0F, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd.pitch_cmd_rad, 0.0F);
    EXPECT_FLOAT_EQ(cmd.throttle_cmd_frac, 0.0F);
}

/* ------------------------------------------------------------------ */
/* Saturation branches                                                 */
/* ------------------------------------------------------------------ */

TEST(ThrustAllocatorSaturation, ClampsPitchToAuthorityLimit) {
    ThrustAllocator allocator;
    const ThrustAllocatorConfig cfg = MakeConfig();
    ASSERT_EQ(allocator.Init(cfg), Status::kSuccess);

    ThrustCommand cmd{};
    /* Nearly horizontal demand: raw atan2 far beyond the limit. */
    ASSERT_EQ(allocator.Allocate(3.0F, 0.1F, 500.0F, &cmd),
              Status::kErrSaturated);
    EXPECT_FLOAT_EQ(cmd.pitch_cmd_rad, cfg.max_pitch_rad);

    ASSERT_EQ(allocator.Allocate(-3.0F, 0.1F, 500.0F, &cmd),
              Status::kErrSaturated);
    EXPECT_FLOAT_EQ(cmd.pitch_cmd_rad, -cfg.max_pitch_rad);
}

TEST(ThrustAllocatorSaturation, ClampsThrottleToUnity) {
    ThrustAllocator allocator;
    ASSERT_EQ(allocator.Init(MakeConfig()), Status::kSuccess);

    /* 500 kg * 10 m/s^2 = 5000 N demanded from a 2500 N engine. */
    ThrustCommand cmd{};
    ASSERT_EQ(allocator.Allocate(0.0F, 10.0F, 500.0F, &cmd),
              Status::kErrSaturated);
    EXPECT_FLOAT_EQ(cmd.throttle_cmd_frac, 1.0F);
    EXPECT_FLOAT_EQ(cmd.pitch_cmd_rad, 0.0F);
}

TEST(ThrustAllocatorSaturation, ClampsNonPositiveVerticalCommand) {
    ThrustAllocator allocator;
    const ThrustAllocatorConfig cfg = MakeConfig();
    ASSERT_EQ(allocator.Init(cfg), Status::kSuccess);

    /* The engine cannot push down: the vertical component is clamped to
     * zero and the remaining horizontal demand hits the pitch limit.    */
    ThrustCommand cmd{};
    ASSERT_EQ(allocator.Allocate(1.0F, -2.0F, 500.0F, &cmd),
              Status::kErrSaturated);
    EXPECT_FLOAT_EQ(cmd.pitch_cmd_rad, cfg.max_pitch_rad);
    EXPECT_NEAR(cmd.throttle_cmd_frac, (500.0F * 1.0F) / cfg.max_thrust_n,
                1.0e-6F);

    /* Pure downward demand: nothing realizable — engine idle, but the
     * clamp is still reported.                                          */
    ASSERT_EQ(allocator.Allocate(0.0F, -2.0F, 500.0F, &cmd),
              Status::kErrSaturated);
    EXPECT_FLOAT_EQ(cmd.pitch_cmd_rad, 0.0F);
    EXPECT_FLOAT_EQ(cmd.throttle_cmd_frac, 0.0F);
}

}  // namespace
}  // namespace gnc
}  // namespace lls
