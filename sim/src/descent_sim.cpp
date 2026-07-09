/**
 * @file    descent_sim.cpp
 * @brief   Implementation of the closed-loop 1-DOF descent simulation.
 *
 * @see     descent_sim.hpp for the guidance profile and loop description.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "descent_sim.hpp"

#include <cmath>

#include "gnc/control/pid_controller.hpp"
#include "lls/lls_assert.hpp"

namespace lls {
namespace sim {

void TelemetryLog::Record(const TelemetrySample& sample) noexcept {
    if (count_ < kCapacity) {
        samples_[count_] = sample;
        ++count_;
    }
}

U32 TelemetryLog::GetCount() const noexcept {
    return count_;
}

const TelemetrySample& TelemetryLog::GetSample(const U32 index) const noexcept {
    LLS_ASSERT(index < count_);
    const U32 safe_index = (index < count_) ? index : 0U;
    return samples_[safe_index];
}

F64 ComputeDescentRateCommand(const ScenarioParams& params,
                              const F64 altitude_m) noexcept {
    if (altitude_m <= params.terminal_altitude_m) {
        return -params.final_descent_rate_mps;
    }
    const F64 braking_speed =
        params.final_descent_rate_mps +
        std::sqrt(2.0 * params.brake_decel_mps2 *
                  (altitude_m - params.terminal_altitude_m));
    const F64 speed = (braking_speed > params.max_descent_rate_mps)
                          ? params.max_descent_rate_mps
                          : braking_speed;
    return -speed;
}

namespace {

/**
 * @brief Descent-rate PID configuration for the SIL run.
 *
 * The PID output is a throttle *correction* around the mass feedforward,
 * hence the symmetric output range; the engine model enforces the real
 * [min_throttle, 1.0] band. Gains match config/landing_params.yaml.
 */
[[nodiscard]] gnc::PidConfig MakeDescentRatePidConfig() noexcept {
    gnc::PidConfig cfg{};
    cfg.kp = 0.8F;
    cfg.ki = 0.15F;
    cfg.kd = 0.05F;
    /* +/-0.7 lets feedforward + correction span the full engine band
     * (hover throttle is ~0.36-0.39), so hot gates can command max
     * thrust; the engine model clamps to what the hardware can do.     */
    cfg.output_min = -0.70F;
    cfg.output_max = 0.70F;
    cfg.integrator_min = -0.20F;
    cfg.integrator_max = 0.20F;
    return cfg;
}

}  // namespace

Status RunDescentSim(const ScenarioParams& params,
                     SimResult* const result_out,
                     TelemetryLog* const log_out) noexcept {
    if (result_out == nullptr) {
        return Status::kErrInvalidParam;
    }
    if (!std::isfinite(params.control_rate_hz) ||
        (params.control_rate_hz <= 0.0) ||
        !std::isfinite(params.max_sim_duration_s) ||
        (params.max_sim_duration_s <= 0.0)) {
        return Status::kErrInvalidParam;
    }

    LanderDynamics dynamics;
    if (!IsSuccess(dynamics.Init(params.vehicle, params.initial_altitude_m,
                                 params.initial_velocity_mps))) {
        return Status::kErrInvalidParam;
    }

    gnc::PidController descent_rate_pid;
    if (!IsSuccess(descent_rate_pid.Init(MakeDescentRatePidConfig()))) {
        return Status::kErrInvalidParam;
    }

    *result_out = SimResult{};
    const F64 dt_s = 1.0 / params.control_rate_hz;
    const F64 initial_propellant_kg = dynamics.GetPropellantRemainingKg();

    /* Bounded loop (rule #3): the step count is fixed by the scenario. */
    const U64 max_steps =
        static_cast<U64>(params.max_sim_duration_s / dt_s) + 1U;

    F64 time_s = 0.0;
    F64 last_throttle_cmd = 0.0;

    for (U64 step = 0U; step < max_steps; ++step) {
        const LanderState& truth = dynamics.GetState();
        const F64 v_cmd_mps =
            ComputeDescentRateCommand(params, truth.altitude_m);

        /* --- Flight control law (this is the code under test). -------- */
        F64 throttle_cmd = 0.0;
        if (truth.altitude_m > params.engine_cutoff_altitude_m) {
            /* Mass feedforward carries the gravity load. */
            const F64 hover_throttle = (truth.mass_kg * kLunarGravityMps2) /
                                       params.vehicle.max_thrust_n;

            F32 correction = 0.0F;
            const Status pid_status = descent_rate_pid.Update(
                static_cast<F32>(v_cmd_mps),
                static_cast<F32>(truth.velocity_mps),
                static_cast<F32>(dt_s), &correction);
            if (!IsSuccess(pid_status) &&
                (pid_status != Status::kErrSaturated)) {
                ++result_out->controller_fault_count;
                /* Hold the previous safe command, as flight code would. */
                throttle_cmd = last_throttle_cmd;
            } else {
                throttle_cmd = hover_throttle + static_cast<F64>(correction);
                /* Command what the actuator can accept: [0, 1]. The
                 * deep-throttle floor is the engine model's business.   */
                throttle_cmd = (throttle_cmd < 0.0) ? 0.0 : throttle_cmd;
                throttle_cmd = (throttle_cmd > 1.0) ? 1.0 : throttle_cmd;
            }
        }
        /* else: below cutoff altitude — engine off, fall to the surface. */

        last_throttle_cmd = throttle_cmd;

        if (log_out != nullptr) {
            TelemetrySample sample{};
            sample.time_s = time_s;
            sample.altitude_m = truth.altitude_m;
            sample.velocity_mps = truth.velocity_mps;
            sample.velocity_cmd_mps = v_cmd_mps;
            sample.throttle_frac = throttle_cmd;
            sample.mass_kg = truth.mass_kg;
            log_out->Record(sample);
        }

        /* --- Truth propagation. ---------------------------------------- */
        if (!IsSuccess(dynamics.Step(throttle_cmd, dt_s))) {
            return Status::kErrInvalidParam;
        }
        time_s += dt_s;

        if (dynamics.HasTouchedDown()) {
            result_out->touched_down = true;
            break;
        }
    }

    const LanderState& final_state = dynamics.GetState();
    result_out->touchdown_speed_mps = std::fabs(final_state.velocity_mps);
    result_out->flight_time_s = time_s;
    result_out->propellant_used_kg =
        initial_propellant_kg - dynamics.GetPropellantRemainingKg();
    return Status::kSuccess;
}

}  // namespace sim
}  // namespace lls
