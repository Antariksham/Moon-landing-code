/**
 * @file    test_pid_controller.cpp
 * @brief   Unit tests for lls::gnc::PidController.
 *
 * @details Per the SELENE testing policy, off-nominal paths (NaN inputs,
 *          saturation, use-before-init, invalid dt) are first-class test
 *          cases, not afterthoughts. Every `Status` value the controller
 *          can return is exercised here.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "gnc/control/pid_controller.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace lls {
namespace gnc {
namespace {

constexpr F32 kDt = 0.02F;  /* Nominal 50 Hz control interval. */

/** @brief Returns a configuration accepted by Init() in every test. */
PidConfig MakeValidConfig() {
    PidConfig cfg{};
    cfg.kp = 1.0F;
    cfg.ki = 0.5F;
    cfg.kd = 0.1F;
    cfg.output_min = -1.0F;
    cfg.output_max = 1.0F;
    cfg.integrator_min = -0.5F;
    cfg.integrator_max = 0.5F;
    return cfg;
}

/* ------------------------------------------------------------------ */
/* Initialization contract                                             */
/* ------------------------------------------------------------------ */

TEST(PidControllerInit, AcceptsValidConfig) {
    PidController ctrl;
    EXPECT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);
}

TEST(PidControllerInit, RejectsNegativeGains) {
    PidController ctrl;
    PidConfig cfg = MakeValidConfig();
    cfg.kp = -0.1F;
    EXPECT_EQ(ctrl.Init(cfg), Status::kErrInvalidParam);
}

TEST(PidControllerInit, RejectsNonFiniteGain) {
    PidController ctrl;
    PidConfig cfg = MakeValidConfig();
    cfg.ki = std::numeric_limits<F32>::quiet_NaN();
    EXPECT_EQ(ctrl.Init(cfg), Status::kErrInvalidParam);
}

TEST(PidControllerInit, RejectsInvertedOutputLimits) {
    PidController ctrl;
    PidConfig cfg = MakeValidConfig();
    cfg.output_min = 1.0F;
    cfg.output_max = -1.0F;
    EXPECT_EQ(ctrl.Init(cfg), Status::kErrInvalidParam);
}

TEST(PidControllerInit, RejectsIntegratorRangeExcludingZero) {
    PidController ctrl;
    PidConfig cfg = MakeValidConfig();
    cfg.integrator_min = 0.1F;  /* Reset()->0 would be out of range. */
    cfg.integrator_max = 0.5F;
    EXPECT_EQ(ctrl.Init(cfg), Status::kErrInvalidParam);
}

TEST(PidControllerInit, FailedReInitDisarmsController) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);

    PidConfig bad = MakeValidConfig();
    bad.kd = -1.0F;
    ASSERT_EQ(ctrl.Init(bad), Status::kErrInvalidParam);

    F32 cmd = 0.0F;
    EXPECT_EQ(ctrl.Update(0.0F, 0.0F, kDt, &cmd),
              Status::kErrNotInitialized);
}

/* ------------------------------------------------------------------ */
/* Update contract: off-nominal inputs                                 */
/* ------------------------------------------------------------------ */

TEST(PidControllerUpdate, RefusesWhenUninitialized) {
    PidController ctrl;
    F32 cmd = 123.0F;
    EXPECT_EQ(ctrl.Update(1.0F, 0.0F, kDt, &cmd),
              Status::kErrNotInitialized);
    EXPECT_EQ(cmd, 123.0F);  /* Output untouched on failure. */
}

TEST(PidControllerUpdate, RejectsNullOutputPointer) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);
    EXPECT_EQ(ctrl.Update(1.0F, 0.0F, kDt, nullptr),
              Status::kErrInvalidParam);
}

TEST(PidControllerUpdate, RejectsNonPositiveDt) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);
    F32 cmd = 0.0F;
    EXPECT_EQ(ctrl.Update(1.0F, 0.0F, 0.0F, &cmd),
              Status::kErrInvalidParam);
    EXPECT_EQ(ctrl.Update(1.0F, 0.0F, -kDt, &cmd),
              Status::kErrInvalidParam);
}

TEST(PidControllerUpdate, RejectsDtBeyondOverrunBound) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);
    F32 cmd = 0.0F;
    EXPECT_EQ(ctrl.Update(1.0F, 0.0F,
                          PidController::kMaxDtSeconds * 2.0F, &cmd),
              Status::kErrInvalidParam);
}

TEST(PidControllerUpdate, RejectsNonFiniteSetpointAndMeasurement) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);
    F32 cmd = 55.0F;
    const F32 nan = std::numeric_limits<F32>::quiet_NaN();
    const F32 inf = std::numeric_limits<F32>::infinity();

    EXPECT_EQ(ctrl.Update(nan, 0.0F, kDt, &cmd), Status::kErrNonFiniteInput);
    EXPECT_EQ(ctrl.Update(0.0F, inf, kDt, &cmd), Status::kErrNonFiniteInput);
    EXPECT_EQ(cmd, 55.0F);  /* Previous safe command preserved. */
}

/* ------------------------------------------------------------------ */
/* Update contract: control law                                        */
/* ------------------------------------------------------------------ */

TEST(PidControllerUpdate, ZeroErrorYieldsZeroCommand) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);
    F32 cmd = 99.0F;
    ASSERT_EQ(ctrl.Update(2.0F, 2.0F, kDt, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd, 0.0F);
}

TEST(PidControllerUpdate, FirstCycleIsPurePlusIntegralNoDerivativeKick) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);

    /* error = 0.5; expect kp*e + ki*e*dt, D inhibited on first cycle.   */
    F32 cmd = 0.0F;
    ASSERT_EQ(ctrl.Update(0.5F, 0.0F, kDt, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd, (1.0F * 0.5F) + (0.5F * 0.5F * kDt));
}

TEST(PidControllerUpdate, DerivativeActsOnMeasurementNotSetpoint) {
    PidController ctrl;
    PidConfig cfg = MakeValidConfig();
    cfg.kp = 0.0F;
    cfg.ki = 0.0F;   /* Isolate the D term. */
    cfg.kd = 0.1F;
    ASSERT_EQ(ctrl.Init(cfg), Status::kSuccess);

    F32 cmd = 0.0F;
    ASSERT_EQ(ctrl.Update(0.0F, 0.0F, kDt, &cmd), Status::kSuccess);

    /* Setpoint step with constant measurement: no derivative kick.      */
    ASSERT_EQ(ctrl.Update(10.0F, 0.0F, kDt, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd, 0.0F);

    /* Measurement ramp: D opposes the measurement rate. The step is kept
     * small so the resulting command stays inside the output limits.    */
    ASSERT_EQ(ctrl.Update(10.0F, 0.01F, kDt, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd, -(0.1F * (0.01F / kDt)));
}

TEST(PidControllerUpdate, OutputSaturatesAndReportsIt) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);
    F32 cmd = 0.0F;
    /* Huge error: P term alone exceeds output_max = 1.                  */
    EXPECT_EQ(ctrl.Update(100.0F, 0.0F, kDt, &cmd), Status::kErrSaturated);
    EXPECT_FLOAT_EQ(cmd, 1.0F);
}

TEST(PidControllerUpdate, IntegratorFreezesWhileSaturated) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);
    F32 cmd = 0.0F;

    /* Drive hard into the upper limit twice; the integrator must not
     * accumulate while the output is pinned (anti-windup).              */
    ASSERT_EQ(ctrl.Update(100.0F, 0.0F, kDt, &cmd), Status::kErrSaturated);
    const F32 integrator_after_first = ctrl.GetIntegratorState();
    ASSERT_EQ(ctrl.Update(100.0F, 0.0F, kDt, &cmd), Status::kErrSaturated);
    EXPECT_FLOAT_EQ(ctrl.GetIntegratorState(), integrator_after_first);
    EXPECT_FLOAT_EQ(ctrl.GetIntegratorState(), 0.0F);
}

TEST(PidControllerUpdate, IntegratorClampHolds) {
    PidController ctrl;
    PidConfig cfg = MakeValidConfig();
    cfg.kp = 0.0F;   /* Keep output unsaturated so the integrator runs.  */
    cfg.kd = 0.0F;
    cfg.ki = 10.0F;
    ASSERT_EQ(ctrl.Init(cfg), Status::kSuccess);

    F32 cmd = 0.0F;
    /* Bounded loop (rule #3): enough cycles to hit the clamp.           */
    for (I32 i = 0; i < 200; ++i) {
        const Status status = ctrl.Update(1.0F, 0.0F, kDt, &cmd);
        ASSERT_TRUE((status == Status::kSuccess) ||
                    (status == Status::kErrSaturated));
    }
    EXPECT_LE(ctrl.GetIntegratorState(), cfg.integrator_max);
}

/* ------------------------------------------------------------------ */
/* Reset contract                                                      */
/* ------------------------------------------------------------------ */

TEST(PidControllerReset, ClearsIntegratorAndDerivativeHistory) {
    PidController ctrl;
    ASSERT_EQ(ctrl.Init(MakeValidConfig()), Status::kSuccess);

    F32 cmd = 0.0F;
    ASSERT_EQ(ctrl.Update(0.5F, 0.1F, kDt, &cmd), Status::kSuccess);
    ASSERT_NE(ctrl.GetIntegratorState(), 0.0F);

    ctrl.Reset();
    EXPECT_FLOAT_EQ(ctrl.GetIntegratorState(), 0.0F);

    /* After Reset() the next cycle is a "first" cycle: no D kick even
     * though the measurement jumped (0.1 -> 1.0). A fresh controller fed
     * the same inputs must produce the identical command.               */
    PidController ref;
    ASSERT_EQ(ref.Init(MakeValidConfig()), Status::kSuccess);
    F32 cmd_ref = 0.0F;
    ASSERT_EQ(ref.Update(0.5F, 1.0F, kDt, &cmd_ref), Status::kSuccess);
    ASSERT_EQ(ctrl.Update(0.5F, 1.0F, kDt, &cmd), Status::kSuccess);
    EXPECT_FLOAT_EQ(cmd, cmd_ref);
}

}  // namespace
}  // namespace gnc
}  // namespace lls
