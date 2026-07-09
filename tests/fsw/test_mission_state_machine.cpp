/**
 * @file    test_mission_state_machine.cpp
 * @brief   Unit tests for lls::fsw::MissionStateMachine.
 *
 * @details Exercises the nominal descent timeline end-to-end, every
 *          FDIR safe-mode edge, and the rejection paths (illegal edges,
 *          corrupted phase values). The phase-graph audits run at compile
 *          time via `static_assert`, so an unsound graph cannot even build.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "fsw/mission_state_machine.hpp"

#include <gtest/gtest.h>

namespace lls {
namespace fsw {
namespace {

/* ------------------------------------------------------------------ */
/* Compile-time phase-graph audits (rule: fail at build, not in orbit) */
/* ------------------------------------------------------------------ */

static_assert(MissionStateMachine::IsTransitionLegal(
                  MissionPhase::kBoot, MissionPhase::kStandby),
              "Nominal timeline must begin BOOT -> STANDBY");

static_assert(!MissionStateMachine::IsTransitionLegal(
                  MissionPhase::kStandby, MissionPhase::kTouchdown),
              "Timeline phases must not be skippable");

static_assert(!MissionStateMachine::IsTransitionLegal(
                  MissionPhase::kTouchdown, MissionPhase::kSafeMode),
              "Once on the surface, safe mode must be unreachable");

static_assert(!MissionStateMachine::IsTransitionLegal(
                  MissionPhase::kSafed, MissionPhase::kStandby),
              "SAFED is terminal: no edge may leave it");

/* ------------------------------------------------------------------ */
/* Runtime behavior                                                    */
/* ------------------------------------------------------------------ */

TEST(MissionStateMachine, StartsInBoot) {
    const MissionStateMachine sm;
    EXPECT_EQ(sm.GetPhase(), MissionPhase::kBoot);
    EXPECT_EQ(sm.GetRejectedTransitionCount(), 0U);
}

TEST(MissionStateMachine, WalksNominalDescentTimeline) {
    MissionStateMachine sm;
    const MissionPhase timeline[] = {
        MissionPhase::kStandby,   MissionPhase::kDeorbit,
        MissionPhase::kBraking,   MissionPhase::kApproach,
        MissionPhase::kTerminalDescent, MissionPhase::kTouchdown,
        MissionPhase::kSafed,
    };
    for (const MissionPhase next : timeline) {  /* Bounded loop (rule #3). */
        ASSERT_EQ(sm.RequestTransition(next), Status::kSuccess);
        ASSERT_EQ(sm.GetPhase(), next);
    }
    EXPECT_EQ(sm.GetRejectedTransitionCount(), 0U);
}

TEST(MissionStateMachine, RejectsPhaseSkipAndHoldsState) {
    MissionStateMachine sm;
    ASSERT_EQ(sm.RequestTransition(MissionPhase::kStandby), Status::kSuccess);

    EXPECT_EQ(sm.RequestTransition(MissionPhase::kTerminalDescent),
              Status::kErrIllegalTransition);
    EXPECT_EQ(sm.GetPhase(), MissionPhase::kStandby);
    EXPECT_EQ(sm.GetRejectedTransitionCount(), 1U);
}

TEST(MissionStateMachine, RejectsCorruptedPhaseValue) {
    MissionStateMachine sm;
    /* Simulate a corrupted command word aliasing an out-of-range value. */
    const auto corrupted = static_cast<MissionPhase>(0xFFU);
    EXPECT_EQ(sm.RequestTransition(corrupted), Status::kErrInvalidParam);
    EXPECT_EQ(sm.GetPhase(), MissionPhase::kBoot);
    EXPECT_EQ(sm.GetRejectedTransitionCount(), 1U);
}

TEST(MissionStateMachine, SafeModeReachableFromEveryInFlightPhase) {
    const MissionPhase in_flight[] = {
        MissionPhase::kStandby, MissionPhase::kDeorbit,
        MissionPhase::kBraking, MissionPhase::kApproach,
        MissionPhase::kTerminalDescent,
    };
    for (const MissionPhase phase : in_flight) {  /* Bounded loop.        */
        EXPECT_TRUE(MissionStateMachine::IsTransitionLegal(
            phase, MissionPhase::kSafeMode))
            << "Safe mode unreachable from phase "
            << static_cast<U32>(static_cast<U8>(phase));
    }
}

TEST(MissionStateMachine, SafeModeRecoversToStandbyOnly) {
    MissionStateMachine sm;
    ASSERT_EQ(sm.RequestTransition(MissionPhase::kStandby), Status::kSuccess);
    ASSERT_EQ(sm.RequestTransition(MissionPhase::kSafeMode), Status::kSuccess);

    EXPECT_EQ(sm.RequestTransition(MissionPhase::kBraking),
              Status::kErrIllegalTransition);
    EXPECT_EQ(sm.RequestTransition(MissionPhase::kStandby), Status::kSuccess);
    EXPECT_EQ(sm.GetPhase(), MissionPhase::kStandby);
}

TEST(MissionStateMachine, ApproachMayFallBackToBraking) {
    MissionStateMachine sm;
    ASSERT_EQ(sm.RequestTransition(MissionPhase::kStandby), Status::kSuccess);
    ASSERT_EQ(sm.RequestTransition(MissionPhase::kDeorbit), Status::kSuccess);
    ASSERT_EQ(sm.RequestTransition(MissionPhase::kBraking), Status::kSuccess);
    ASSERT_EQ(sm.RequestTransition(MissionPhase::kApproach), Status::kSuccess);

    EXPECT_EQ(sm.RequestTransition(MissionPhase::kBraking), Status::kSuccess);
    EXPECT_EQ(sm.GetPhase(), MissionPhase::kBraking);
}

}  // namespace
}  // namespace fsw
}  // namespace lls
