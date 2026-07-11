/**
 * @file    approach_sim.cpp
 * @brief   Implementation of the closed-loop 3-DOF approach simulation.
 *
 * @see     approach_sim.hpp for the control architecture and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "approach_sim.hpp"

#include <cmath>

#include "gnc/control/pid_controller.hpp"
#include "lls/lls_assert.hpp"

namespace lls {
namespace sim {

void ApproachTelemetryLog::Record(
    const ApproachTelemetrySample& sample) noexcept {
    if (count_ < kCapacity) {
        samples_[count_] = sample;
        ++count_;
    }
}

U32 ApproachTelemetryLog::GetCount() const noexcept {
    return count_;
}

const ApproachTelemetrySample& ApproachTelemetryLog::GetSample(
    const U32 index) const noexcept {
    LLS_ASSERT(index < count_);
    const U32 safe_index = (index < count_) ? index : 0U;
    return samples_[safe_index];
}

namespace {

/** Gravity feedforward for the vertical velocity loop, m/s^2. */
constexpr F32 kGravityFeedforwardMps2 = static_cast<F32>(kLunarGravityMps2);

/** A status that is neither nominal nor mere saturation is a fault. */
[[nodiscard]] bool IsFault(const Status status) noexcept {
    return !IsSuccess(status) && (status != Status::kErrSaturated);
}

/**
 * @brief Horizontal velocity loop: ground-speed error to thrust
 *        acceleration along +x, m/s^2.
 *
 * Slow outer loop (time constant a few seconds) so it stays well separated
 * from the attitude loop that realizes its command. Gains match
 * config/landing_params.yaml.
 */
[[nodiscard]] gnc::PidConfig MakeHorizontalVelocityPidConfig() noexcept {
    gnc::PidConfig cfg{};
    cfg.kp = 0.40F;
    cfg.ki = 0.02F;
    cfg.kd = 0.0F;
    /* +/-2.0 m/s^2 keeps the commanded tilt inside the allocator's pitch
     * authority at all vertical-channel commands.                        */
    cfg.output_min = -2.0F;
    cfg.output_max = 2.0F;
    cfg.integrator_min = -0.30F;
    cfg.integrator_max = 0.30F;
    return cfg;
}

/**
 * @brief Vertical velocity loop: descent-rate error to thrust acceleration
 *        correction around the gravity feedforward, m/s^2.
 *
 * Same architecture as milestone 1, but in acceleration units so the
 * allocator can combine it with the horizontal command. Gains match
 * config/landing_params.yaml.
 */
[[nodiscard]] gnc::PidConfig MakeVerticalVelocityPidConfig() noexcept {
    gnc::PidConfig cfg{};
    cfg.kp = 1.50F;
    cfg.ki = 0.30F;
    cfg.kd = 0.0F;
    /* +/-2.5 m/s^2 correction spans the engine's net-acceleration band
     * around hover (max net accel is ~2.9 m/s^2 at full tanks).          */
    cfg.output_min = -2.5F;
    cfg.output_max = 2.5F;
    cfg.integrator_min = -0.50F;
    cfg.integrator_max = 0.50F;
    return cfg;
}

/**
 * @brief Pitch attitude loop: pitch error to RCS torque fraction in
 *        [-1, 1].
 *
 * PD design: kp sets stiffness, and the controller's
 * derivative-on-measurement D term provides pure rate damping (-kd *
 * pitch_rate). ki = 0 — a bias torque has nothing to trim in vacuum
 * flight. Gains match config/landing_params.yaml.
 */
[[nodiscard]] gnc::PidConfig MakeAttitudePidConfig() noexcept {
    gnc::PidConfig cfg{};
    cfg.kp = 5.0F;
    cfg.ki = 0.0F;
    cfg.kd = 4.5F;
    cfg.output_min = -1.0F;
    cfg.output_max = 1.0F;
    cfg.integrator_min = -0.10F;
    cfg.integrator_max = 0.10F;
    return cfg;
}

/**
 * @brief   Walk the executive along the nominal timeline to APPROACH.
 * @param   executive  State machine fresh from construction (in BOOT).
 * @return  `true` if every transition was accepted.
 */
[[nodiscard]] bool SequenceToApproach(
    fsw::MissionStateMachine& executive) noexcept {
    const std::array<fsw::MissionPhase, 4U> route = {
        fsw::MissionPhase::kStandby,
        fsw::MissionPhase::kDeorbit,
        fsw::MissionPhase::kBraking,
        fsw::MissionPhase::kApproach,
    };
    for (const fsw::MissionPhase phase : route) { /* Bounded loop. */
        if (!IsSuccess(executive.RequestTransition(phase))) {
            return false;
        }
    }
    return true;
}

/** @brief The flight components in the loop, initialized as one unit. */
struct FlightStack {
    gnc::DescentGuidance guidance;
    gnc::PidController horizontal_pid;
    gnc::PidController vertical_pid;
    gnc::PidController attitude_pid;
    gnc::ThrustAllocator allocator;
    fsw::MissionStateMachine executive;

    [[nodiscard]] bool Init(const ApproachScenarioParams& params) noexcept {
        return IsSuccess(guidance.Init(params.guidance)) &&
               IsSuccess(
                   horizontal_pid.Init(MakeHorizontalVelocityPidConfig())) &&
               IsSuccess(vertical_pid.Init(MakeVerticalVelocityPidConfig())) &&
               IsSuccess(attitude_pid.Init(MakeAttitudePidConfig())) &&
               IsSuccess(allocator.Init(params.allocator)) &&
               SequenceToApproach(executive);
    }
};

/** @brief Actuator commands produced by one control cycle. */
struct CycleCommands {
    F64 throttle_frac = 0.0;
    F64 torque_frac = 0.0;
    F32 pitch_cmd_rad = 0.0F;
};

/**
 * @brief   One 50 Hz pass through the flight control law.
 *
 * @details Guidance -> velocity loops -> allocation -> attitude loop. On a
 *          flight-code fault the previous cycle's commands are held, as the
 *          flight executive would.
 *
 * @param   stack        Flight components (updated in place).
 * @param   truth        Current truth state (perfect-navigation stand-in).
 * @param   dt_s         Control interval, s.
 * @param   gcmd         In/out: last valid guidance command.
 * @param   previous     Commands held from the previous cycle.
 * @param   fault_count  In/out: incremented once per faulted cycle.
 * @return  Commands to apply for this cycle.
 */
[[nodiscard]] CycleCommands RunControlCycle(FlightStack& stack,
                                            const LanderState3Dof& truth,
                                            const F64 dt_s,
                                            gnc::GuidanceCommand& gcmd,
                                            const CycleCommands& previous,
                                            U32& fault_count) noexcept {
    CycleCommands cmds{};
    const F32 dt_f32 = static_cast<F32>(dt_s);
    bool faulted = false;

    /* Guidance: on fault, gcmd is left holding the previous reference.   */
    if (IsFault(
            stack.guidance.Update(static_cast<F32>(truth.altitude_m), &gcmd))) {
        faulted = true;
    }

    /* Phase management: guidance's terminal discrete sequences the
     * executive, which in turn resets the controllers (stale integrator
     * charge must not cross a phase boundary).                           */
    if (gcmd.terminal_phase &&
        (stack.executive.GetPhase() == fsw::MissionPhase::kApproach)) {
        if (IsSuccess(stack.executive.RequestTransition(
                fsw::MissionPhase::kTerminalDescent))) {
            stack.horizontal_pid.Reset();
            stack.vertical_pid.Reset();
            stack.attitude_pid.Reset();
        }
    }

    /* Velocity loops + allocation (skipped after the cutoff discrete:
     * engine off, attitude held vertical for touchdown).                 */
    if (!gcmd.engine_cutoff) {
        F32 accel_x_cmd = 0.0F;
        F32 accel_z_corr = 0.0F;
        const Status sx = stack.horizontal_pid.Update(
            gcmd.horizontal_rate_cmd_mps,
            static_cast<F32>(truth.velocity_x_mps), dt_f32, &accel_x_cmd);
        const Status sz = stack.vertical_pid.Update(
            gcmd.vertical_rate_cmd_mps, static_cast<F32>(truth.velocity_z_mps),
            dt_f32, &accel_z_corr);

        gnc::ThrustCommand thrust_cmd{};
        Status sa = Status::kErrInvalidParam;
        if (!IsFault(sx) && !IsFault(sz)) {
            const F32 accel_up_cmd = accel_z_corr + kGravityFeedforwardMps2;
            sa = stack.allocator.Allocate(accel_x_cmd, accel_up_cmd,
                                          static_cast<F32>(truth.mass_kg),
                                          &thrust_cmd);
        }
        if (IsFault(sx) || IsFault(sz) || IsFault(sa)) {
            faulted = true;
            cmds.throttle_frac = previous.throttle_frac;
            cmds.pitch_cmd_rad = previous.pitch_cmd_rad;
        } else {
            cmds.throttle_frac = static_cast<F64>(thrust_cmd.throttle_cmd_frac);
            cmds.pitch_cmd_rad = thrust_cmd.pitch_cmd_rad;
        }
    }
    /* else: cutoff — throttle 0, pitch command 0 (vertical touchdown).   */

    /* Attitude loop runs to touchdown: RCS is independent of the engine. */
    F32 torque_frac = 0.0F;
    const Status st = stack.attitude_pid.Update(
        cmds.pitch_cmd_rad, static_cast<F32>(truth.pitch_rad), dt_f32,
        &torque_frac);
    if (IsFault(st)) {
        faulted = true;
        cmds.torque_frac = previous.torque_frac;
    } else {
        cmds.torque_frac = static_cast<F64>(torque_frac);
    }

    if (faulted) {
        ++fault_count;
    }
    return cmds;
}

}  // namespace

Status RunApproachSim(const ApproachScenarioParams& params,
                      ApproachSimResult* const result_out,
                      ApproachTelemetryLog* const log_out) noexcept {
    if (result_out == nullptr) {
        return Status::kErrInvalidParam;
    }
    if (!std::isfinite(params.control_rate_hz) ||
        (params.control_rate_hz <= 0.0) ||
        !std::isfinite(params.max_sim_duration_s) ||
        (params.max_sim_duration_s <= 0.0)) {
        return Status::kErrInvalidParam;
    }
    /* The allocator's thrust constant must describe the actual engine, or
     * every throttle command is scaled wrong.                            */
    if (std::fabs(static_cast<F64>(params.allocator.max_thrust_n) -
                  params.vehicle.max_thrust_n) > 0.5) {
        return Status::kErrInvalidParam;
    }

    LanderDynamics3Dof dynamics;
    if (!IsSuccess(dynamics.Init(params.vehicle, params.gate))) {
        return Status::kErrInvalidParam;
    }

    FlightStack stack;
    if (!stack.Init(params)) {
        return Status::kErrInvalidParam;
    }

    *result_out = ApproachSimResult{};
    const F64 dt_s = 1.0 / params.control_rate_hz;
    const F64 initial_propellant_kg = dynamics.GetPropellantRemainingKg();

    /* Bounded loop (rule #3): the step count is fixed by the scenario. */
    const U64 max_steps =
        static_cast<U64>(params.max_sim_duration_s / dt_s) + 1U;

    F64 time_s = 0.0;
    gnc::GuidanceCommand gcmd{};
    CycleCommands cmds{};

    for (U64 step = 0U; step < max_steps; ++step) {
        const LanderState3Dof& truth = dynamics.GetState();

        cmds = RunControlCycle(stack, truth, dt_s, gcmd, cmds,
                               result_out->controller_fault_count);

        if (log_out != nullptr) {
            ApproachTelemetrySample sample{};
            sample.time_s = time_s;
            sample.downrange_m = truth.downrange_m;
            sample.altitude_m = truth.altitude_m;
            sample.velocity_x_mps = truth.velocity_x_mps;
            sample.velocity_z_mps = truth.velocity_z_mps;
            sample.velocity_x_cmd_mps =
                static_cast<F64>(gcmd.horizontal_rate_cmd_mps);
            sample.velocity_z_cmd_mps =
                static_cast<F64>(gcmd.vertical_rate_cmd_mps);
            sample.pitch_rad = truth.pitch_rad;
            sample.pitch_cmd_rad = static_cast<F64>(cmds.pitch_cmd_rad);
            sample.throttle_frac = cmds.throttle_frac;
            sample.torque_frac = cmds.torque_frac;
            sample.mass_kg = truth.mass_kg;
            sample.mission_phase = static_cast<U8>(stack.executive.GetPhase());
            log_out->Record(sample);
        }

        /* --- Truth propagation. ---------------------------------------- */
        if (!IsSuccess(
                dynamics.Step(cmds.throttle_frac, cmds.torque_frac, dt_s))) {
            return Status::kErrInvalidParam;
        }
        time_s += dt_s;

        if (dynamics.HasTouchedDown()) {
            result_out->touched_down = true;
            /* Close out the timeline exactly as the flight executive
             * would: contact -> TOUCHDOWN, then passivation -> SAFED.   */
            if (IsSuccess(stack.executive.RequestTransition(
                    fsw::MissionPhase::kTouchdown))) {
                static_cast<void>(stack.executive.RequestTransition(
                    fsw::MissionPhase::kSafed));
            }
            break;
        }
    }

    const LanderState3Dof& final_state = dynamics.GetState();
    result_out->touchdown_vertical_speed_mps =
        std::fabs(final_state.velocity_z_mps);
    result_out->touchdown_horizontal_speed_mps =
        std::fabs(final_state.velocity_x_mps);
    result_out->touchdown_tilt_rad = std::fabs(final_state.pitch_rad);
    result_out->flight_time_s = time_s;
    result_out->propellant_used_kg =
        initial_propellant_kg - dynamics.GetPropellantRemainingKg();
    result_out->final_phase = stack.executive.GetPhase();
    result_out->rejected_transition_count =
        stack.executive.GetRejectedTransitionCount();
    return Status::kSuccess;
}

}  // namespace sim
}  // namespace lls
