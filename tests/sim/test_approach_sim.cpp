/**
 * @file    test_approach_sim.cpp
 * @brief   3-DOF truth-dynamics checks and closed-loop SIL regression tests
 *          for the approach + pitch-over descent (milestone 2).
 *
 * @details Physics checks pin the truth model to analytic solutions
 *          (free fall, rigid-body rotation, thrust direction); the
 *          closed-loop tests fly the full flight stack — guidance, three
 *          PID loops, thrust allocation, and the mission state machine —
 *          and judge it on mission criteria, including dispersed gates.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <cmath>

#include "approach_sim.hpp"

namespace lls {
namespace sim {
namespace {

/** Mission touchdown limits (config: terminal_descent section). */
constexpr F64 kVerticalLimitMps = 2.0;
constexpr F64 kHorizontalLimitMps = 1.0;
constexpr F64 kTiltLimitRad = 0.0873; /* 5 deg. */

/* ------------------------------------------------------------------ */
/* 3-DOF truth dynamics                                                */
/* ------------------------------------------------------------------ */

TEST(LanderDynamics3Dof, RefusesStepBeforeInit) {
    LanderDynamics3Dof dyn;
    EXPECT_EQ(dyn.Step(0.5, 0.0, 0.02), Status::kErrNotInitialized);
}

TEST(LanderDynamics3Dof, RejectsNonPhysicalParams) {
    LanderDynamics3Dof dyn;
    VehicleParams3Dof bad{};
    bad.pitch_inertia_kgm2 = -1.0;
    EXPECT_EQ(dyn.Init(bad, Gate3Dof{}), Status::kErrInvalidParam);
}

TEST(LanderDynamics3Dof, RejectsGateWithExcessivePitch) {
    LanderDynamics3Dof dyn;
    Gate3Dof gate{};
    gate.pitch_rad = 1.6; /* Beyond +/-90 deg: thrust axis below horizon. */
    EXPECT_EQ(dyn.Init(VehicleParams3Dof{}, gate), Status::kErrInvalidParam);
}

TEST(LanderDynamics3Dof, RejectsInvalidCommands) {
    LanderDynamics3Dof dyn;
    ASSERT_EQ(dyn.Init(VehicleParams3Dof{}, Gate3Dof{}), Status::kSuccess);
    EXPECT_EQ(dyn.Step(-0.1, 0.0, 0.02), Status::kErrInvalidParam);
    EXPECT_EQ(dyn.Step(0.5, 0.0, 0.0), Status::kErrInvalidParam);
    EXPECT_EQ(dyn.Step(0.5, 0.0, 0.2), Status::kErrInvalidParam);
}

TEST(LanderDynamics3Dof, FreeFallMatchesKinematicsInBothAxes) {
    LanderDynamics3Dof dyn;
    Gate3Dof gate{};
    gate.altitude_m = 1000.0;
    gate.velocity_x_mps = 20.0;
    gate.velocity_z_mps = 0.0;
    gate.pitch_rad = 0.0;
    ASSERT_EQ(dyn.Init(VehicleParams3Dof{}, gate), Status::kSuccess);

    /* 2 s of engine-off, torque-off fall at 1 kHz. Gravity acts only on
     * the vertical axis; the horizontal velocity must be untouched.      */
    for (I32 i = 0; i < 2000; ++i) { /* Bounded loop. */
        ASSERT_EQ(dyn.Step(0.0, 0.0, 0.001), Status::kSuccess);
    }
    const LanderState3Dof& s = dyn.GetState();
    EXPECT_NEAR(s.velocity_z_mps, -kLunarGravityMps2 * 2.0, 0.01);
    EXPECT_NEAR(s.velocity_x_mps, 20.0, 1.0e-9);
    EXPECT_NEAR(s.downrange_m, 40.0, 0.05);
    EXPECT_NEAR(s.altitude_m, 1000.0 - (0.5 * kLunarGravityMps2 * 4.0), 0.05);
    EXPECT_NEAR(s.pitch_rad, 0.0, 1.0e-12);
}

TEST(LanderDynamics3Dof, RcsTorqueIntegratesToRigidBodyRotation) {
    LanderDynamics3Dof dyn;
    const VehicleParams3Dof vp{};
    Gate3Dof gate{};
    gate.pitch_rad = 0.0;
    ASSERT_EQ(dyn.Init(vp, gate), Status::kSuccess);

    /* 1 s of full positive couple, engine off: omega = (tau/I)*t.        */
    for (I32 i = 0; i < 1000; ++i) { /* Bounded loop. */
        ASSERT_EQ(dyn.Step(0.0, 1.0, 0.001), Status::kSuccess);
    }
    const F64 expected_rate = vp.max_rcs_torque_nm / vp.pitch_inertia_kgm2;
    EXPECT_NEAR(dyn.GetState().pitch_rate_radps, expected_rate, 1.0e-9);
    /* theta = 0.5*(tau/I)*t^2, within integration tolerance.             */
    EXPECT_NEAR(dyn.GetState().pitch_rad, 0.5 * expected_rate, 1.0e-3);
}

TEST(LanderDynamics3Dof, ClampsTorqueCommandToAuthority) {
    LanderDynamics3Dof dyn_clamped;
    LanderDynamics3Dof dyn_unity;
    ASSERT_EQ(dyn_clamped.Init(VehicleParams3Dof{}, Gate3Dof{}),
              Status::kSuccess);
    ASSERT_EQ(dyn_unity.Init(VehicleParams3Dof{}, Gate3Dof{}),
              Status::kSuccess);

    ASSERT_EQ(dyn_clamped.Step(0.0, 5.0, 0.01), Status::kSuccess);
    ASSERT_EQ(dyn_unity.Step(0.0, 1.0, 0.01), Status::kSuccess);
    EXPECT_DOUBLE_EQ(dyn_clamped.GetState().pitch_rate_radps,
                     dyn_unity.GetState().pitch_rate_radps);
}

TEST(LanderDynamics3Dof, ThrustActsAlongTheBodyAxis) {
    LanderDynamics3Dof dyn;
    const VehicleParams3Dof vp{};
    Gate3Dof gate{};
    gate.velocity_x_mps = 0.0;
    gate.velocity_z_mps = 0.0;
    gate.pitch_rad = 0.3;
    ASSERT_EQ(dyn.Init(vp, gate), Status::kSuccess);

    /* One tiny step at full throttle, no torque: the velocity increment
     * must point along (sin(pitch), cos(pitch)) plus gravity.            */
    const F64 dt = 1.0e-4;
    ASSERT_EQ(dyn.Step(1.0, 0.0, dt), Status::kSuccess);
    const LanderState3Dof& s = dyn.GetState();
    const F64 accel = vp.max_thrust_n / s.mass_kg;
    EXPECT_NEAR(s.velocity_x_mps, accel * std::sin(0.3) * dt, 1.0e-9);
    EXPECT_NEAR(s.velocity_z_mps,
                ((accel * std::cos(0.3)) - kLunarGravityMps2) * dt, 1.0e-9);
}

TEST(LanderDynamics3Dof, ClampsThrottleToDeepThrottleFloor) {
    LanderDynamics3Dof dyn;
    const VehicleParams3Dof vp{};
    Gate3Dof gate{};
    gate.velocity_x_mps = 0.0;
    gate.velocity_z_mps = 0.0;
    gate.pitch_rad = 0.0;
    ASSERT_EQ(dyn.Init(vp, gate), Status::kSuccess);

    /* Commanding 1% must produce the min-throttle acceleration, not 1%. */
    ASSERT_EQ(dyn.Step(0.01, 0.0, 0.001), Status::kSuccess);
    const F64 expected_accel = (vp.min_throttle_frac * vp.max_thrust_n) /
                                   (vp.dry_mass_kg + vp.propellant_mass_kg) -
                               kLunarGravityMps2;
    EXPECT_NEAR(dyn.GetState().velocity_z_mps, expected_accel * 0.001, 1.0e-6);
}

TEST(LanderDynamics3Dof, TouchdownFreezesTheState) {
    LanderDynamics3Dof dyn;
    Gate3Dof gate{};
    gate.altitude_m = 0.05;
    gate.velocity_x_mps = 1.0;
    gate.velocity_z_mps = -2.0;
    ASSERT_EQ(dyn.Init(VehicleParams3Dof{}, gate), Status::kSuccess);

    ASSERT_EQ(dyn.Step(0.0, 0.0, 0.05), Status::kSuccess);
    ASSERT_TRUE(dyn.HasTouchedDown());
    const F64 vx_at_contact = dyn.GetState().velocity_x_mps;

    ASSERT_EQ(dyn.Step(1.0, 1.0, 0.05), Status::kSuccess);
    EXPECT_DOUBLE_EQ(dyn.GetState().velocity_x_mps, vx_at_contact);
    EXPECT_DOUBLE_EQ(dyn.GetState().altitude_m, 0.0);
}

/* ------------------------------------------------------------------ */
/* Closed-loop mission criteria                                        */
/* ------------------------------------------------------------------ */

TEST(ApproachSimClosedLoop, NominalGateLandsSafelyAndReachesSafed) {
    /* Default scenario = milestone 3: the vertical channel flies on the
     * flight nav filter fed by noisy sensors, not on truth.              */
    const ApproachScenarioParams params{};
    ApproachSimResult result{};
    ASSERT_EQ(RunApproachSim(params, &result, nullptr), Status::kSuccess);

    EXPECT_TRUE(result.touched_down) << "Vehicle never reached the surface";
    EXPECT_LE(result.touchdown_vertical_speed_mps, kVerticalLimitMps);
    EXPECT_LE(result.touchdown_horizontal_speed_mps, kHorizontalLimitMps);
    EXPECT_LE(result.touchdown_tilt_rad, kTiltLimitRad);
    EXPECT_EQ(result.controller_fault_count, 0U);
    EXPECT_EQ(result.final_phase, fsw::MissionPhase::kSafed)
        << "Executive did not complete the nominal timeline";
    EXPECT_EQ(result.rejected_transition_count, 0U);
    EXPECT_LT(result.propellant_used_kg, params.vehicle.propellant_mass_kg)
        << "Tanks ran dry before touchdown";
    EXPECT_LT(result.touchdown_miss_m, 5.0)
        << "Landed " << result.touchdown_miss_m << " m from the site";
}

TEST(ApproachSimClosedLoop, LandsOnRetargetedSites) {
    /* Site targeting (milestone 5): different mission-designed targets,
     * same gate — each must be hit to within meters. All three are inside
     * the gate's reachability envelope (stopping distance and the
     * altitude-keyed pitch-over ramp both respected).                    */
    const F64 targets_m[] = {800.0, 1200.0, 1600.0};
    for (const F64 target : targets_m) { /* Bounded loop. */
        ApproachScenarioParams params{};
        params.target_downrange_m = target;

        ApproachSimResult result{};
        ASSERT_EQ(RunApproachSim(params, &result, nullptr), Status::kSuccess);
        EXPECT_TRUE(result.touched_down) << "No touchdown, target " << target;
        EXPECT_LE(result.touchdown_vertical_speed_mps, kVerticalLimitMps);
        EXPECT_LT(result.touchdown_miss_m, 10.0)
            << "Missed target " << target << " by " << result.touchdown_miss_m
            << " m";
    }
}

TEST(ApproachSimClosedLoop, FliesThePitchOverManeuver) {
    const ApproachScenarioParams params{};
    ApproachSimResult result{};
    static ApproachTelemetryLog log; /* ~3 MB: static, not stack. */
    ASSERT_EQ(RunApproachSim(params, &result, &log), Status::kSuccess);
    ASSERT_GT(log.GetCount(), 0U);

    /* The vehicle must actually fly tilted during the approach...        */
    F64 max_tilt_rad = 0.0;
    bool saw_terminal_phase = false;
    for (U32 i = 0U; i < log.GetCount(); ++i) { /* Bounded loop. */
        const ApproachTelemetrySample& s = log.GetSample(i);
        const F64 tilt = std::fabs(s.pitch_rad);
        max_tilt_rad = (tilt > max_tilt_rad) ? tilt : max_tilt_rad;
        if (s.mission_phase ==
            static_cast<U8>(fsw::MissionPhase::kTerminalDescent)) {
            saw_terminal_phase = true;
            /* ...but must be nearly vertical throughout terminal descent
             * (10x the touchdown limit allows the straightening
             * transient right at the phase boundary).                   */
            EXPECT_LE(tilt, 10.0 * kTiltLimitRad)
                << "Excessive tilt during terminal descent at t=" << s.time_s;
        }
    }
    EXPECT_GT(max_tilt_rad, 0.15)
        << "Approach never pitched: not a pitch-over trajectory";
    EXPECT_TRUE(saw_terminal_phase)
        << "Executive never entered TERMINAL_DESCENT";

    /* Phase sequence: starts in APPROACH, ends in TERMINAL_DESCENT (the
     * TOUCHDOWN/SAFED transitions happen after the final sample).        */
    EXPECT_EQ(log.GetSample(0U).mission_phase,
              static_cast<U8>(fsw::MissionPhase::kApproach));
    EXPECT_EQ(log.GetSample(log.GetCount() - 1U).mission_phase,
              static_cast<U8>(fsw::MissionPhase::kTerminalDescent));
}

/* ------------------------------------------------------------------ */
/* Navigation in the loop (milestone 3)                                */
/* ------------------------------------------------------------------ */

TEST(ApproachSimNavigation, EstimatorConvergesAndTracksThroughTouchdown) {
    const ApproachScenarioParams params{};
    ApproachSimResult result{};
    static ApproachTelemetryLog log; /* ~3 MB: static, not stack. */
    ASSERT_EQ(RunApproachSim(params, &result, &log), Status::kSuccess);
    ASSERT_TRUE(result.touched_down);
    ASSERT_GT(log.GetCount(), 100U);

    /* The filter is seeded 5 m / 1 m/s off truth; after the first second
     * (50 cycles, >= 10 altimeter fixes) it must have converged, and it
     * must stay converged all the way down.                              */
    for (U32 i = 50U; i < log.GetCount(); ++i) { /* Bounded loop. */
        const ApproachTelemetrySample& s = log.GetSample(i);
        EXPECT_LT(std::fabs(s.nav_altitude_m - s.altitude_m), 1.0)
            << "Altitude estimate diverged at t=" << s.time_s;
        EXPECT_LT(std::fabs(s.nav_velocity_z_mps - s.velocity_z_mps), 0.5)
            << "Velocity estimate diverged at t=" << s.time_s;
    }

    /* Touchdown-time estimate quality (what the cutoff decision used).   */
    EXPECT_LT(result.touchdown_nav_altitude_error_m, 0.5);
    EXPECT_LT(result.touchdown_nav_velocity_error_mps, 0.3);

    /* Clean sensors this run: the innovation gate should stay quiet.     */
    EXPECT_EQ(result.nav_rejected_measurement_count, 0U);
}

TEST(ApproachSimNavigation, PerfectNavigationBaselineStillFlies) {
    /* Milestone 2 regression: bypassing sensors + filter must still land
     * safely and must report zero navigation error by definition.        */
    ApproachScenarioParams params{};
    params.nav.use_perfect_navigation = true;

    ApproachSimResult result{};
    ASSERT_EQ(RunApproachSim(params, &result, nullptr), Status::kSuccess);
    EXPECT_TRUE(result.touched_down);
    EXPECT_LE(result.touchdown_vertical_speed_mps, kVerticalLimitMps);
    EXPECT_LE(result.touchdown_horizontal_speed_mps, kHorizontalLimitMps);
    EXPECT_LE(result.touchdown_tilt_rad, kTiltLimitRad);
    EXPECT_EQ(result.controller_fault_count, 0U);
    EXPECT_DOUBLE_EQ(result.touchdown_nav_altitude_error_m, 0.0);
    EXPECT_DOUBLE_EQ(result.touchdown_nav_velocity_error_mps, 0.0);
    EXPECT_EQ(result.nav_rejected_measurement_count, 0U);
}

TEST(ApproachSimNavigation, RejectsBadSensorConfiguration) {
    ApproachScenarioParams params{};
    params.nav.altimeter.update_divisor = 0U; /* Model will refuse.       */
    ApproachSimResult result{};
    EXPECT_EQ(RunApproachSim(params, &result, nullptr),
              Status::kErrInvalidParam);
}

TEST(ApproachSimClosedLoop, SurvivesDispersedGateConditions) {
    /* Corner cases of the approach handover envelope. Every combination
     * must land safely (speeds, tilt); landing ACCURACY is only judged
     * inside the mission-designed gate envelope (the Monte-Carlo
     * acceptance test) — several of these deliberately out-of-envelope
     * gates cannot physically reach the default site and land long or
     * short instead, which is the correct degraded behavior.             */
    const F64 altitudes_m[] = {1500.0, 2000.0, 2500.0};
    const F64 horizontal_mps[] = {40.0, 60.0, 80.0};
    const F64 vertical_mps[] = {-20.0, -30.0, -40.0};

    for (const F64 h0 : altitudes_m) { /* Bounded loops. */
        for (const F64 vx0 : horizontal_mps) {
            for (const F64 vz0 : vertical_mps) {
                ApproachScenarioParams params{};
                params.gate.altitude_m = h0;
                params.gate.velocity_x_mps = vx0;
                params.gate.velocity_z_mps = vz0;

                ApproachSimResult result{};
                ASSERT_EQ(RunApproachSim(params, &result, nullptr),
                          Status::kSuccess);
                EXPECT_TRUE(result.touched_down)
                    << "No touchdown from h=" << h0 << " vx=" << vx0
                    << " vz=" << vz0;
                EXPECT_LE(result.touchdown_vertical_speed_mps,
                          kVerticalLimitMps)
                    << "Hard landing from h=" << h0 << " vx=" << vx0
                    << " vz=" << vz0;
                EXPECT_LE(result.touchdown_horizontal_speed_mps,
                          kHorizontalLimitMps)
                    << "Lateral drift at contact from h=" << h0 << " vx=" << vx0
                    << " vz=" << vz0;
                EXPECT_LE(result.touchdown_tilt_rad, kTiltLimitRad)
                    << "Tilted landing from h=" << h0 << " vx=" << vx0
                    << " vz=" << vz0;
            }
        }
    }
}

TEST(ApproachSimClosedLoop, RejectsNullResultPointer) {
    const ApproachScenarioParams params{};
    EXPECT_EQ(RunApproachSim(params, nullptr, nullptr),
              Status::kErrInvalidParam);
}

TEST(ApproachSimClosedLoop, RejectsBadControlRate) {
    ApproachScenarioParams params{};
    params.control_rate_hz = 0.0;
    ApproachSimResult result{};
    EXPECT_EQ(RunApproachSim(params, &result, nullptr),
              Status::kErrInvalidParam);
}

TEST(ApproachSimClosedLoop, RejectsAllocatorEngineMismatch) {
    /* The allocator's thrust constant must describe the actual engine;
     * a mismatch would scale every throttle command.                     */
    ApproachScenarioParams params{};
    params.allocator.max_thrust_n = 2000.0F;
    ApproachSimResult result{};
    EXPECT_EQ(RunApproachSim(params, &result, nullptr),
              Status::kErrInvalidParam);
}

TEST(ApproachSimClosedLoop, TelemetryLogCapturesFullApproach) {
    const ApproachScenarioParams params{};
    ApproachSimResult result{};
    static ApproachTelemetryLog log; /* ~3 MB: static, not stack. */
    ASSERT_EQ(RunApproachSim(params, &result, &log), Status::kSuccess);

    ASSERT_GT(log.GetCount(), 0U);
    const ApproachTelemetrySample& first = log.GetSample(0U);
    const ApproachTelemetrySample& last = log.GetSample(log.GetCount() - 1U);
    EXPECT_DOUBLE_EQ(first.altitude_m, params.gate.altitude_m);
    EXPECT_DOUBLE_EQ(first.velocity_x_mps, params.gate.velocity_x_mps);
    EXPECT_LT(last.altitude_m, 1.0); /* Ends at (nearly) the surface.   */
}

}  // namespace
}  // namespace sim
}  // namespace lls
