/**
 * @file    mission_state_machine.hpp
 * @brief   Mission-phase state machine for the SELENE descent sequence.
 *
 * @details Sequences the vehicle through the landing timeline:
 *
 *              BOOT → STANDBY → DEORBIT → BRAKING → APPROACH
 *                   → TERMINAL_DESCENT → TOUCHDOWN → SAFED
 *
 *          plus a SAFE_MODE phase reachable from every in-flight phase,
 *          commanded by FDIR on an unrecoverable fault.
 *
 *          Safety properties:
 *            - The set of legal transitions is a `constexpr` table checked
 *              at compile time — there is no code path that can move the
 *              mission to an unlisted phase (rules #1, #7).
 *            - `RequestTransition()` rejects illegal requests with
 *              `Status::kErrIllegalTransition` and leaves the current phase
 *              untouched; the rejection is counted for telemetry.
 *            - The class holds no pointers and allocates nothing; a single
 *              instance is statically allocated by the flight executive.
 *
 * @par Real-time characteristics
 *          `RequestTransition()` performs one bounded table scan
 *          (<= kTransitionTableSize comparisons); all other methods are
 *          O(1). Not thread-safe: phase changes are owned exclusively by
 *          the flight-executive task.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_FSW_MISSION_STATE_MACHINE_HPP
#define LLS_FSW_MISSION_STATE_MACHINE_HPP

#include <array>

#include "lls/lls_types.hpp"

namespace lls {
namespace fsw {

/**
 * @brief Mission phases of the descent timeline.
 *
 * Values are fixed-width and stable: they are downlinked verbatim in the
 * telemetry stream and must never be renumbered (rule #7).
 */
enum class MissionPhase : U8 {
    kBoot = 0U,            /**< Power-on self test, memory scrub.          */
    kStandby = 1U,         /**< Healthy and idle in lunar orbit.           */
    kDeorbit = 2U,         /**< Deorbit burn to descent-orbit insertion.   */
    kBraking = 3U,         /**< Main braking burn, killing orbital speed.  */
    kApproach = 4U,        /**< Pitch-over; HDA scans for a safe site.     */
    kTerminalDescent = 5U, /**< Vertical descent to the selected site.     */
    kTouchdown = 6U,       /**< Surface contact detected; engines cut.     */
    kSafed = 7U,           /**< Landed, vented, and passivated. Terminal.  */
    kSafeMode = 8U,        /**< FDIR refuge from any in-flight fault.      */
};

/** @brief Number of values in `MissionPhase` (bounds checks, telemetry). */
constexpr U8 kMissionPhaseCount = 9U;

/**
 * @brief Deterministic mission-phase state machine.
 *
 * Owned and stepped exclusively by the flight-executive task. Subsystems
 * (guidance, control, HDA) read the current phase via `GetPhase()` and
 * request transitions through the executive — never directly.
 */
class MissionStateMachine {
 public:
    /** @brief One legal edge of the phase graph. */
    struct Transition {
        MissionPhase from; /**< Phase the vehicle must currently be in.    */
        MissionPhase to;   /**< Phase the vehicle is allowed to enter.     */
    };

    /** @brief Starts in `MissionPhase::kBoot`; no `Init()` step required. */
    MissionStateMachine() noexcept = default;

    /* One vehicle, one mission timeline: duplication is a design error. */
    MissionStateMachine(const MissionStateMachine&) = delete;
    MissionStateMachine& operator=(const MissionStateMachine&) = delete;
    MissionStateMachine(MissionStateMachine&&) = delete;
    MissionStateMachine& operator=(MissionStateMachine&&) = delete;

    ~MissionStateMachine() = default;

    /**
     * @brief   Request a transition to @p target.
     *
     * @details The request succeeds if and only if the edge
     *          (current phase → @p target) appears in the compile-time
     *          transition table. On rejection the current phase is
     *          unchanged and the rejection counter increments — a spike in
     *          that counter in telemetry is a red flag that a subsystem's
     *          view of the timeline has diverged from the executive's.
     *
     * @param   target  Requested next phase. Must be a valid enumerator;
     *                  out-of-range raw values are rejected.
     *
     * @retval  Status::kSuccess               Phase is now @p target.
     * @retval  Status::kErrInvalidParam       @p target is not a valid
     *                                         `MissionPhase` value.
     * @retval  Status::kErrIllegalTransition  Edge not in the table; phase
     *                                         unchanged.
     */
    [[nodiscard]] Status RequestTransition(MissionPhase target) noexcept;

    /**
     * @brief   Current mission phase.
     * @return  The phase most recently entered; `kBoot` after construction.
     */
    [[nodiscard]] MissionPhase GetPhase() const noexcept;

    /**
     * @brief   Rejected-transition count since boot (telemetry point).
     * @return  Monotonic counter; wraps at `U32` range by design.
     */
    [[nodiscard]] U32 GetRejectedTransitionCount() const noexcept;

    /**
     * @brief   Whether an edge is legal, without taking it.
     *
     * @details `constexpr` so unit tests and ground tools can audit the
     *          phase graph at compile time.
     *
     * @param   from  Candidate current phase.
     * @param   to    Candidate next phase.
     * @return  `true` if (from → to) is in the transition table.
     */
    [[nodiscard]] static constexpr bool IsTransitionLegal(
        const MissionPhase from, const MissionPhase to) noexcept {
        for (const Transition& edge : kTransitionTable) { /* Bounded loop. */
            if ((edge.from == from) && (edge.to == to)) {
                return true;
            }
        }
        return false;
    }

    /** @brief Number of edges in the phase graph. */
    static constexpr U8 kTransitionTableSize = 15U;

    /**
     * @brief The complete set of legal phase transitions.
     *
     * The nominal timeline is the top block; the SAFE_MODE block encodes
     * the FDIR rule that safe mode is reachable from every in-flight phase
     * but never from `kTouchdown`/`kSafed` (once down, we stay down).
     */
    static constexpr std::array<Transition, kTransitionTableSize>
        kTransitionTable = {{
            /* Nominal descent timeline. */
            {MissionPhase::kBoot, MissionPhase::kStandby},
            {MissionPhase::kStandby, MissionPhase::kDeorbit},
            {MissionPhase::kDeorbit, MissionPhase::kBraking},
            {MissionPhase::kBraking, MissionPhase::kApproach},
            {MissionPhase::kApproach, MissionPhase::kTerminalDescent},
            {MissionPhase::kTerminalDescent, MissionPhase::kTouchdown},
            {MissionPhase::kTouchdown, MissionPhase::kSafed},

            /* Abort-to-orbit style hold: guidance may fall back one phase. */
            {MissionPhase::kApproach, MissionPhase::kBraking},

            /* FDIR: safe mode reachable from every in-flight phase.       */
            {MissionPhase::kStandby, MissionPhase::kSafeMode},
            {MissionPhase::kDeorbit, MissionPhase::kSafeMode},
            {MissionPhase::kBraking, MissionPhase::kSafeMode},
            {MissionPhase::kApproach, MissionPhase::kSafeMode},
            {MissionPhase::kTerminalDescent, MissionPhase::kSafeMode},

            /* Recovery: ground command returns safe mode to standby.      */
            {MissionPhase::kSafeMode, MissionPhase::kStandby},

            /* Boot faults park in safe mode pending ground contact.       */
            {MissionPhase::kBoot, MissionPhase::kSafeMode},
        }};

 private:
    MissionPhase phase_ = MissionPhase::kBoot; /**< Current phase.         */
    U32 rejected_transition_count_ = 0U;       /**< Telemetry counter.     */
};

}  // namespace fsw
}  // namespace lls

#endif  // LLS_FSW_MISSION_STATE_MACHINE_HPP
