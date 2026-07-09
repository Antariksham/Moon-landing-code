/**
 * @file    descent_sim.hpp
 * @brief   Closed-loop 1-DOF terminal-descent simulation (SIL milestone 1).
 *
 * @details Closes the loop between the **flight** descent-rate PID
 *          controller (`src/gnc/control/pid_controller.hpp`, built under
 *          the strict flight flags) and the host-side truth dynamics
 *          (`lander_dynamics.hpp`). The guidance reference is a simple
 *          braking profile:
 *
 *              v_cmd(h) = -min(v_max, v_final + sqrt(2*a_brake*(h - h_t)))
 *                                                     for h >  h_t
 *              v_cmd(h) = -v_final                    for h <= h_t
 *
 *          i.e. decelerate along a constant-deceleration envelope, then
 *          ride down the final `h_t` meters at a constant `v_final`,
 *          cutting the engine at the cutoff altitude and falling the last
 *          half meter — the classic terminal-descent shape.
 *
 *          Throttle command = mass-feedforward hover throttle + PID
 *          correction. The feedforward carries the gravity load so the
 *          PID's clamped integrator only has to absorb modeling error,
 *          exactly how the controller will be used in flight.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_SIM_DESCENT_SIM_HPP
#define LLS_SIM_DESCENT_SIM_HPP

#include <array>

#include "lander_dynamics.hpp"
#include "lls/lls_types.hpp"

namespace lls {
namespace sim {

/** @brief Scenario configuration (defaults mirror config/landing_params.yaml). */
struct ScenarioParams {
    VehicleParams vehicle{};              /**< Truth-model constants.       */

    F64 initial_altitude_m   = 500.0;     /**< Terminal-descent gate.       */
    F64 initial_velocity_mps = -30.0;     /**< Descending at handover.      */

    F64 control_rate_hz = 50.0;           /**< Flight control loop rate.    */

    /* Guidance braking profile. */
    F64 brake_decel_mps2       = 1.2;     /**< Envelope deceleration; must
                                               be well under the vehicle's
                                               max net decel (~2.5).        */
    F64 max_descent_rate_mps   = 25.0;    /**< Speed cap on the envelope.   */
    F64 final_descent_rate_mps = 1.0;     /**< Constant-rate final segment. */
    F64 terminal_altitude_m    = 10.0;    /**< Start of the final segment.  */
    F64 engine_cutoff_altitude_m = 0.5;   /**< Free-fall below this.        */

    F64 max_sim_duration_s = 600.0;       /**< Hard bound on the sim loop.  */
};

/** @brief One telemetry sample of the closed-loop run. */
struct TelemetrySample {
    F64 time_s        = 0.0;
    F64 altitude_m    = 0.0;
    F64 velocity_mps  = 0.0;
    F64 velocity_cmd_mps = 0.0;
    F64 throttle_frac = 0.0;
    F64 mass_kg       = 0.0;
};

/** @brief Outcome of a completed (or aborted) run. */
struct SimResult {
    bool touched_down = false;      /**< Reached the surface within bounds. */
    F64 touchdown_speed_mps = 0.0;  /**< |velocity| at surface contact.     */
    F64 flight_time_s = 0.0;        /**< Elapsed sim time.                  */
    F64 propellant_used_kg = 0.0;   /**< Propellant consumed.               */
    U32 controller_fault_count = 0; /**< Non-nominal PID statuses observed. */
};

/** @brief Fixed-capacity telemetry recorder (rule #1: no heap, even here). */
class TelemetryLog {
 public:
    /** 600 s at 50 Hz. */
    static constexpr U32 kCapacity = 30000U;

    /** @brief Append a sample; silently drops once full (bounded storage). */
    void Record(const TelemetrySample& sample) noexcept;

    /** @brief Number of stored samples, <= kCapacity. */
    [[nodiscard]] U32 GetCount() const noexcept;

    /** @brief Sample at @p index; index must be < GetCount(). */
    [[nodiscard]] const TelemetrySample& GetSample(U32 index) const noexcept;

 private:
    std::array<TelemetrySample, kCapacity> samples_{};
    U32 count_ = 0U;
};

/**
 * @brief   Guidance reference: commanded descent rate for an altitude.
 *
 * @param   params      Scenario (profile constants).
 * @param   altitude_m  Current altitude, >= 0.
 * @return  Commanded vertical velocity (negative = descend), m/s.
 */
[[nodiscard]] F64 ComputeDescentRateCommand(const ScenarioParams& params,
                                            F64 altitude_m) noexcept;

/**
 * @brief   Run the closed-loop descent from gate to touchdown.
 *
 * @param   params       Scenario configuration.
 * @param   result_out   Non-null; receives the run outcome.
 * @param   log_out      Optional telemetry recorder; pass nullptr to skip.
 *
 * @retval  Status::kSuccess          Run completed (touchdown or timeout;
 *                                    inspect `result_out->touched_down`).
 * @retval  Status::kErrInvalidParam  Null result pointer or a scenario
 *                                    value the models reject.
 */
[[nodiscard]] Status RunDescentSim(const ScenarioParams& params,
                                   SimResult* result_out,
                                   TelemetryLog* log_out) noexcept;

}  // namespace sim
}  // namespace lls

#endif  // LLS_SIM_DESCENT_SIM_HPP
