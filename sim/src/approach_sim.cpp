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
    /* +/-2.5 m/s^2: enough braking authority to stop a +3 sigma hot gate
     * (75 m/s) inside the nominal 1200 m site range (stopping distance
     * 1125 m), realizable within the allocator's 60 deg pitch authority. */
    cfg.output_min = -2.5F;
    cfg.output_max = 2.5F;
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

/**
 * @brief   Compose the flight filter's configuration for this scenario.
 *
 * @details The filter is seeded from the gate state plus the configured
 *          navigation-handover errors — the upstream orbit solution is
 *          good, not perfect.
 */
[[nodiscard]] gnc::VerticalNavFilterConfig MakeNavFilterConfig(
    const ApproachScenarioParams& params) noexcept {
    gnc::VerticalNavFilterConfig cfg = params.nav.filter;
    cfg.initial_altitude_m = static_cast<F32>(
        params.gate.altitude_m + params.nav.initial_altitude_error_m);
    cfg.initial_velocity_mps = static_cast<F32>(
        params.gate.velocity_z_mps + params.nav.initial_velocity_error_mps);
    return cfg;
}

/** @brief The flight components in the loop, initialized as one unit. */
struct FlightStack {
    gnc::VerticalNavFilter nav_filter;
    gnc::DescentGuidance guidance;
    gnc::PidController horizontal_pid;
    gnc::PidController vertical_pid;
    gnc::PidController attitude_pid;
    gnc::ThrustAllocator allocator;
    hda::SafeSiteSelector site_selector;
    fsw::MissionStateMachine executive;

    [[nodiscard]] bool Init(const ApproachScenarioParams& params) noexcept {
        return IsSuccess(nav_filter.Init(MakeNavFilterConfig(params))) &&
               IsSuccess(guidance.Init(params.guidance)) &&
               IsSuccess(
                   horizontal_pid.Init(MakeHorizontalVelocityPidConfig())) &&
               IsSuccess(vertical_pid.Init(MakeVerticalVelocityPidConfig())) &&
               IsSuccess(attitude_pid.Init(MakeAttitudePidConfig())) &&
               IsSuccess(allocator.Init(params.allocator)) &&
               IsSuccess(site_selector.Init(params.hda.selector)) &&
               SequenceToApproach(executive);
    }
};

/**
 * @brief   Whether the HDA scenario parameters are self-consistent.
 * @param   hda  Candidate parameters (checked even when disabled: a bad
 *               configuration is a scenario bug either way).
 * @return  `true` when the gate/spacing values are usable and the survey
 *          fits the flight selector's fixed capacity.
 */
[[nodiscard]] bool AreHdaParamsValid(const HdaScenarioParams& hda) noexcept {
    if (!std::isfinite(hda.scan_altitude_m) || (hda.scan_altitude_m <= 0.0) ||
        !std::isfinite(hda.survey_halfwidth_m) ||
        (hda.survey_halfwidth_m <= 0.0) ||
        !std::isfinite(hda.sample_spacing_m) || (hda.sample_spacing_m <= 0.0)) {
        return false;
    }
    const F64 stations_per_side = hda.survey_halfwidth_m / hda.sample_spacing_m;
    const F64 station_count = (2.0 * stations_per_side) + 1.0;
    return station_count <= static_cast<F64>(hda::SiteSurvey::kMaxSiteSamples);
}

/**
 * @brief   Survey the terrain around the target (mapper stand-in).
 *
 * @details Perfect terrain sensing: stations read the truth model
 *          directly. LIDAR range/registration noise is a follow-up.
 *
 * @param   terrain      Truth terrain.
 * @param   hda          Survey geometry.
 * @param   center_m     Survey center (the current target).
 * @return  Survey ready for the flight selector.
 */
[[nodiscard]] hda::SiteSurvey SurveyTerrain(const TerrainModel& terrain,
                                            const HdaScenarioParams& hda,
                                            const F64 center_m) noexcept {
    hda::SiteSurvey survey{};
    for (F64 offset_m = -hda.survey_halfwidth_m;
         offset_m <= hda.survey_halfwidth_m;
         offset_m += hda.sample_spacing_m) { /* Bounded by validation. */
        if (survey.count >= hda::SiteSurvey::kMaxSiteSamples) {
            break;
        }
        const F64 station_m = center_m + offset_m;
        hda::SiteSample& sample = survey.samples[survey.count];
        sample.downrange_m = static_cast<F32>(station_m);
        sample.slope_deg = static_cast<F32>(terrain.GetSlopeDeg(station_m));
        sample.roughness_m = static_cast<F32>(terrain.GetRoughnessM(station_m));
        ++survey.count;
    }
    return survey;
}

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
 *          flight executive would. The vertical channel consumes the
 *          navigation solution; the horizontal and attitude channels read
 *          truth (perfect navigation) until they get sensors of their own.
 *
 * @param   stack              Flight components (updated in place).
 * @param   truth              Current truth state.
 * @param   nav_altitude_m     Navigation altitude estimate, m.
 * @param   nav_velocity_z_mps Navigation vertical-velocity estimate, m/s.
 * @param   downrange_to_go_m  Signed ground distance to the landing site
 *                             (truth-derived: the horizontal channel flies
 *                             perfect navigation until TRN exists), m.
 * @param   dt_s               Control interval, s.
 * @param   gcmd               In/out: last valid guidance command.
 * @param   previous           Commands held from the previous cycle.
 * @param   fault_count        In/out: incremented once per faulted cycle.
 * @return  Commands to apply for this cycle.
 */
[[nodiscard]] CycleCommands RunControlCycle(
    FlightStack& stack, const LanderState3Dof& truth, const F64 nav_altitude_m,
    const F64 nav_velocity_z_mps, const F64 downrange_to_go_m, const F64 dt_s,
    gnc::GuidanceCommand& gcmd, const CycleCommands& previous,
    U32& fault_count) noexcept {
    CycleCommands cmds{};
    const F32 dt_f32 = static_cast<F32>(dt_s);
    bool faulted = false;

    /* Guidance flies the navigation altitude. Noise can push the estimate
     * fractionally below zero near the surface; the executive sanitizes
     * to the guidance contract's domain instead of faulting.             */
    const F64 guidance_altitude_m =
        (nav_altitude_m > 0.0) ? nav_altitude_m : 0.0;

    /* Guidance: on fault, gcmd is left holding the previous reference.   */
    if (IsFault(stack.guidance.Update(static_cast<F32>(guidance_altitude_m),
                                      static_cast<F32>(downrange_to_go_m),
                                      &gcmd))) {
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
            gcmd.vertical_rate_cmd_mps, static_cast<F32>(nav_velocity_z_mps),
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
        (params.max_sim_duration_s <= 0.0) ||
        !std::isfinite(params.target_downrange_m)) {
        return Status::kErrInvalidParam;
    }
    /* The allocator's thrust constant must describe the actual engine to
     * within calibration tolerance (10%), or every throttle command is
     * scaled wrong beyond what the velocity loops can absorb. Inside the
     * tolerance the mismatch is realistic — the flight software carries
     * the nominal engine model while the real engine is dispersed — and
     * the loops trim it out.                                             */
    if (std::fabs(static_cast<F64>(params.allocator.max_thrust_n) -
                  params.vehicle.max_thrust_n) >
        (0.1 * params.vehicle.max_thrust_n)) {
        return Status::kErrInvalidParam;
    }

    if (!AreHdaParamsValid(params.hda) ||
        (params.hazard_zone_count > TerrainModel::kMaxHazardZones)) {
        return Status::kErrInvalidParam;
    }

    LanderDynamics3Dof dynamics;
    if (!IsSuccess(dynamics.Init(params.vehicle, params.gate))) {
        return Status::kErrInvalidParam;
    }

    TerrainModel terrain;
    if (!IsSuccess(terrain.Init(params.terrain))) {
        return Status::kErrInvalidParam;
    }
    for (U32 i = 0U; i < params.hazard_zone_count; ++i) { /* Bounded loop. */
        if (!IsSuccess(terrain.AddHazardZone(params.hazard_zones[i]))) {
            return Status::kErrInvalidParam;
        }
    }

    FlightStack stack;
    if (!stack.Init(params)) {
        return Status::kErrInvalidParam;
    }

    ImuModel imu;
    AltimeterModel altimeter;
    if (!params.nav.use_perfect_navigation) {
        if (!IsSuccess(imu.Init(params.nav.imu)) ||
            !IsSuccess(altimeter.Init(params.nav.altimeter))) {
            return Status::kErrInvalidParam;
        }
    }

    *result_out = ApproachSimResult{};
    const F64 dt_s = 1.0 / params.control_rate_hz;
    const F64 initial_propellant_kg = dynamics.GetPropellantRemainingKg();

    /* Bounded loop (rule #3): the step count is fixed by the scenario. */
    const U64 max_steps =
        static_cast<U64>(params.max_sim_duration_s / dt_s) + 1U;

    F64 time_s = 0.0;
    F64 prev_velocity_z_mps = params.gate.velocity_z_mps;
    F64 active_target_m = params.target_downrange_m;
    bool hda_scan_done = false;
    gnc::GuidanceCommand gcmd{};
    CycleCommands cmds{};

    for (U64 step = 0U; step < max_steps; ++step) {
        const LanderState3Dof& truth = dynamics.GetState();

        /* --- Navigation: sensors -> flight filter -> estimate. -------- */
        F64 nav_altitude_m = truth.altitude_m;
        F64 nav_velocity_z_mps = truth.velocity_z_mps;
        if (!params.nav.use_perfect_navigation) {
            if (step > 0U) {
                /* The IMU's delta-v over the previous interval, corrupted
                 * by the sensor model. An accelerometer measures specific
                 * force, hence the +g / -g bracket around the model.     */
                const F64 accel_true_mps2 =
                    (truth.velocity_z_mps - prev_velocity_z_mps) / dt_s;
                const F64 specific_force_meas_mps2 =
                    imu.MeasureAccel(accel_true_mps2 + kLunarGravityMps2);
                if (IsFault(stack.nav_filter.Predict(
                        static_cast<F32>(specific_force_meas_mps2 -
                                         kLunarGravityMps2),
                        static_cast<F32>(dt_s)))) {
                    ++result_out->controller_fault_count;
                }
            }
            if ((step % static_cast<U64>(altimeter.GetUpdateDivisor())) == 0U) {
                const F64 altitude_meas_m =
                    altimeter.MeasureAltitude(truth.altitude_m);
                const Status update_status = stack.nav_filter.UpdateAltitude(
                    static_cast<F32>(altitude_meas_m));
                /* A gate rejection is an expected sensor event, tracked
                 * by the filter itself — not a flight-code fault.        */
                if (IsFault(update_status) &&
                    (update_status != Status::kErrMeasurementRejected)) {
                    ++result_out->controller_fault_count;
                }
            }
            const gnc::VerticalNavEstimate estimate =
                stack.nav_filter.GetEstimate();
            nav_altitude_m = static_cast<F64>(estimate.altitude_m);
            nav_velocity_z_mps = static_cast<F64>(estimate.velocity_mps);
        }
        prev_velocity_z_mps = truth.velocity_z_mps;

        /* --- HDA decision gate (one-shot, during APPROACH). ------------ */
        if (params.hda.enabled && !hda_scan_done &&
            (stack.executive.GetPhase() == fsw::MissionPhase::kApproach) &&
            (nav_altitude_m <= params.hda.scan_altitude_m)) {
            hda_scan_done = true;
            const hda::SiteSurvey survey =
                SurveyTerrain(terrain, params.hda, active_target_m);
            hda::SiteSelection selection{};
            const Status hda_status = stack.site_selector.SelectSite(
                survey, static_cast<F32>(active_target_m), &selection);
            if (IsSuccess(hda_status)) {
                if (selection.diverted) {
                    result_out->hda_diverted = true;
                    result_out->hda_divert_distance_m = std::fabs(
                        static_cast<F64>(selection.target_downrange_m) -
                        active_target_m);
                    active_target_m =
                        static_cast<F64>(selection.target_downrange_m);
                }
            } else if (hda_status == Status::kErrNoSafeSite) {
                /* Mission event, not a code fault: hold the nominal site
                 * and let the landing be judged for what it is.          */
                result_out->hda_no_safe_site = true;
            } else {
                ++result_out->controller_fault_count;
            }
        }

        const F64 downrange_to_go_m = active_target_m - truth.downrange_m;
        cmds = RunControlCycle(stack, truth, nav_altitude_m, nav_velocity_z_mps,
                               downrange_to_go_m, dt_s, gcmd, cmds,
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
            sample.nav_altitude_m = nav_altitude_m;
            sample.nav_velocity_z_mps = nav_velocity_z_mps;
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
    result_out->touchdown_downrange_m = final_state.downrange_m;
    result_out->touchdown_miss_m =
        std::fabs(active_target_m - final_state.downrange_m);
    result_out->final_target_downrange_m = active_target_m;
    result_out->landed_on_hazard =
        (terrain.GetSlopeDeg(final_state.downrange_m) >
         static_cast<F64>(params.hda.selector.max_slope_deg)) ||
        (terrain.GetRoughnessM(final_state.downrange_m) >
         static_cast<F64>(params.hda.selector.max_roughness_m));
    result_out->flight_time_s = time_s;
    result_out->propellant_used_kg =
        initial_propellant_kg - dynamics.GetPropellantRemainingKg();
    result_out->final_phase = stack.executive.GetPhase();
    result_out->rejected_transition_count =
        stack.executive.GetRejectedTransitionCount();

    if (!params.nav.use_perfect_navigation) {
        const gnc::VerticalNavEstimate estimate =
            stack.nav_filter.GetEstimate();
        result_out->nav_rejected_measurement_count =
            stack.nav_filter.GetRejectedMeasurementCount();
        result_out->touchdown_nav_altitude_error_m = std::fabs(
            static_cast<F64>(estimate.altitude_m) - final_state.altitude_m);
        result_out->touchdown_nav_velocity_error_mps =
            std::fabs(static_cast<F64>(estimate.velocity_mps) -
                      final_state.velocity_z_mps);
    }
    return Status::kSuccess;
}

}  // namespace sim
}  // namespace lls
