/**
 * @file    test_descent_sim.cpp
 * @brief   Closed-loop SIL regression tests for the 1-DOF descent.
 *
 * @details These are the tests the coding standard demands for every
 *          controller: the flight PID flying the truth dynamics end to
 *          end, judged on mission criteria (touchdown speed, propellant),
 *          plus off-nominal scenarios (hot gate, dispersed start states).
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include "descent_sim.hpp"

namespace lls {
namespace sim {
namespace {

/** Mission touchdown limit (config: touchdown_velocity_limit_mps). */
constexpr F64 kTouchdownLimitMps = 2.0;

/* ------------------------------------------------------------------ */
/* Guidance profile                                                    */
/* ------------------------------------------------------------------ */

TEST(DescentGuidance, CommandsFinalRateAtLowAltitude) {
    const ScenarioParams params{};
    EXPECT_DOUBLE_EQ(ComputeDescentRateCommand(params, 5.0),
                     -params.final_descent_rate_mps);
    EXPECT_DOUBLE_EQ(ComputeDescentRateCommand(params, 0.0),
                     -params.final_descent_rate_mps);
}

TEST(DescentGuidance, CapsCommandAtMaxDescentRate) {
    const ScenarioParams params{};
    EXPECT_DOUBLE_EQ(ComputeDescentRateCommand(params, 10000.0),
                     -params.max_descent_rate_mps);
}

TEST(DescentGuidance, ProfileIsContinuousAtTerminalGate) {
    const ScenarioParams params{};
    const F64 just_above =
        ComputeDescentRateCommand(params, params.terminal_altitude_m + 1.0e-9);
    EXPECT_NEAR(just_above, -params.final_descent_rate_mps, 1.0e-3);
}

TEST(DescentGuidance, ProfileIsMonotonicInAltitude) {
    const ScenarioParams params{};
    F64 previous = ComputeDescentRateCommand(params, 0.0);
    for (I32 h = 1; h <= 600; ++h) { /* Bounded loop. */
        const F64 current =
            ComputeDescentRateCommand(params, static_cast<F64>(h));
        EXPECT_LE(current, previous + 1.0e-12)
            << "Commanded descent rate must not relax as altitude grows "
               "(h = "
            << h << ")";
        previous = current;
    }
}

/* ------------------------------------------------------------------ */
/* Truth dynamics                                                      */
/* ------------------------------------------------------------------ */

TEST(LanderDynamics, RefusesStepBeforeInit) {
    LanderDynamics dyn;
    EXPECT_EQ(dyn.Step(0.5, 0.02), Status::kErrNotInitialized);
}

TEST(LanderDynamics, RejectsNonPhysicalParams) {
    LanderDynamics dyn;
    VehicleParams bad{};
    bad.dry_mass_kg = -1.0;
    EXPECT_EQ(dyn.Init(bad, 500.0, -30.0), Status::kErrInvalidParam);
}

TEST(LanderDynamics, FreeFallMatchesKinematics) {
    LanderDynamics dyn;
    ASSERT_EQ(dyn.Init(VehicleParams{}, 100.0, 0.0), Status::kSuccess);

    /* 2 s of engine-off fall at 1 kHz; compare with v = -g*t.           */
    for (I32 i = 0; i < 2000; ++i) { /* Bounded loop. */
        ASSERT_EQ(dyn.Step(0.0, 0.001), Status::kSuccess);
    }
    EXPECT_NEAR(dyn.GetState().velocity_mps, -kLunarGravityMps2 * 2.0, 0.01);
    EXPECT_NEAR(dyn.GetState().altitude_m,
                100.0 - (0.5 * kLunarGravityMps2 * 4.0), 0.05);
}

TEST(LanderDynamics, ClampsThrottleToDeepThrottleFloor) {
    LanderDynamics dyn;
    const VehicleParams vp{};
    ASSERT_EQ(dyn.Init(vp, 100.0, 0.0), Status::kSuccess);

    /* Commanding 1% must produce the min-throttle acceleration, not 1%. */
    ASSERT_EQ(dyn.Step(0.01, 0.001), Status::kSuccess);
    const F64 expected_accel = (vp.min_throttle_frac * vp.max_thrust_n) /
                                   (vp.dry_mass_kg + vp.propellant_mass_kg) -
                               kLunarGravityMps2;
    EXPECT_NEAR(dyn.GetState().velocity_mps, expected_accel * 0.001, 1.0e-6);
}

TEST(LanderDynamics, NoThrustWhenPropellantExhausted) {
    LanderDynamics dyn;
    VehicleParams vp{};
    vp.propellant_mass_kg = 0.001; /* Nearly dry tanks. */
    ASSERT_EQ(dyn.Init(vp, 5000.0, 0.0), Status::kSuccess);

    /* Burn the tanks dry, then confirm full throttle produces free fall. */
    for (I32 i = 0; i < 100; ++i) { /* Bounded loop. */
        ASSERT_EQ(dyn.Step(1.0, 0.01), Status::kSuccess);
    }
    ASSERT_DOUBLE_EQ(dyn.GetPropellantRemainingKg(), 0.0);

    const F64 v_before = dyn.GetState().velocity_mps;
    ASSERT_EQ(dyn.Step(1.0, 0.01), Status::kSuccess);
    EXPECT_NEAR(dyn.GetState().velocity_mps - v_before,
                -kLunarGravityMps2 * 0.01, 1.0e-9);
}

/* ------------------------------------------------------------------ */
/* Closed-loop mission criteria                                        */
/* ------------------------------------------------------------------ */

TEST(DescentSimClosedLoop, NominalGateLandsSafely) {
    const ScenarioParams params{};
    SimResult result{};
    ASSERT_EQ(RunDescentSim(params, &result, nullptr), Status::kSuccess);

    EXPECT_TRUE(result.touched_down) << "Vehicle never reached the surface";
    EXPECT_LE(result.touchdown_speed_mps, kTouchdownLimitMps);
    EXPECT_EQ(result.controller_fault_count, 0U);
    EXPECT_LT(result.propellant_used_kg, params.vehicle.propellant_mass_kg)
        << "Tanks ran dry before touchdown";
}

TEST(DescentSimClosedLoop, SurvivesDispersedGateConditions) {
    /* Monte-Carlo-lite: corner cases of the handover envelope. Gates are
     * screened for physical feasibility first — the vehicle's maximum net
     * deceleration is bounded, and a gate that needs more than ~80% of it
     * (v^2 / 2h) is a guidance/mission-design violation upstream of the
     * controller, not a controller failure.                             */
    const ScenarioParams nominal{};
    const F64 total_mass_kg =
        nominal.vehicle.dry_mass_kg + nominal.vehicle.propellant_mass_kg;
    const F64 max_net_decel_mps2 =
        (nominal.vehicle.max_thrust_n / total_mass_kg) - kLunarGravityMps2;

    const F64 altitudes_m[] = {300.0, 500.0, 800.0};
    const F64 velocities_mps[] = {-15.0, -30.0, -45.0};

    U32 flown = 0U;
    for (const F64 h0 : altitudes_m) { /* Bounded loops. */
        for (const F64 v0 : velocities_mps) {
            const F64 required_decel_mps2 = (v0 * v0) / (2.0 * h0);
            if (required_decel_mps2 > (0.8 * max_net_decel_mps2)) {
                continue; /* Outside the vehicle's physical envelope.   */
            }
            ++flown;

            ScenarioParams params{};
            params.initial_altitude_m = h0;
            params.initial_velocity_mps = v0;

            SimResult result{};
            ASSERT_EQ(RunDescentSim(params, &result, nullptr),
                      Status::kSuccess);
            EXPECT_TRUE(result.touched_down)
                << "No touchdown from h=" << h0 << " v=" << v0;
            EXPECT_LE(result.touchdown_speed_mps, kTouchdownLimitMps)
                << "Hard landing from h=" << h0 << " v=" << v0;
        }
    }
    EXPECT_GE(flown, 6U) << "Feasibility screen rejected too many gates";
}

TEST(DescentSimClosedLoop, RejectsNullResultPointer) {
    const ScenarioParams params{};
    EXPECT_EQ(RunDescentSim(params, nullptr, nullptr),
              Status::kErrInvalidParam);
}

TEST(DescentSimClosedLoop, TelemetryLogCapturesFullDescent) {
    const ScenarioParams params{};
    SimResult result{};
    static TelemetryLog log; /* ~1.4 MB: static, not stack. */
    ASSERT_EQ(RunDescentSim(params, &result, &log), Status::kSuccess);

    ASSERT_GT(log.GetCount(), 0U);
    const TelemetrySample& first = log.GetSample(0U);
    const TelemetrySample& last = log.GetSample(log.GetCount() - 1U);
    EXPECT_DOUBLE_EQ(first.altitude_m, params.initial_altitude_m);
    EXPECT_LT(last.altitude_m, 1.0); /* Ends at (nearly) the surface.   */
}

}  // namespace
}  // namespace sim
}  // namespace lls
