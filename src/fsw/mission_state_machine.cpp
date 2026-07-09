/**
 * @file    mission_state_machine.cpp
 * @brief   Implementation of the SELENE mission-phase state machine.
 *
 * @see     mission_state_machine.hpp for the phase graph and the full
 *          interface contract.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "fsw/mission_state_machine.hpp"

#include "lls/lls_assert.hpp"

namespace lls {
namespace fsw {

Status MissionStateMachine::RequestTransition(
    const MissionPhase target) noexcept {
    /* Reject raw values outside the enumeration before any comparison:
     * a corrupted command word must not be able to alias a legal phase. */
    if (static_cast<U8>(target) >= kMissionPhaseCount) {
        ++rejected_transition_count_;
        return Status::kErrInvalidParam;
    }

    if (!IsTransitionLegal(phase_, target)) {
        ++rejected_transition_count_;
        return Status::kErrIllegalTransition;
    }

    phase_ = target;

    /* Postcondition: the machine can only ever hold a valid phase. */
    LLS_ASSERT(static_cast<U8>(phase_) < kMissionPhaseCount);
    LLS_ASSERT(phase_ == target);

    return Status::kSuccess;
}

MissionPhase MissionStateMachine::GetPhase() const noexcept {
    return phase_;
}

U32 MissionStateMachine::GetRejectedTransitionCount() const noexcept {
    return rejected_transition_count_;
}

}  // namespace fsw
}  // namespace lls
