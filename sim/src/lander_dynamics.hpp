/**
 * @file    lander_dynamics.hpp
 * @brief   1-DOF (vertical) lunar lander truth dynamics for the SIL sim.
 *
 * @details Point-mass model of the terminal-descent problem: altitude,
 *          vertical velocity, and vehicle mass under lunar gravity and a
 *          single throttleable main engine. Propellant depletion follows
 *          the ideal rocket relation `mdot = T / (Isp * g0)`.
 *
 *          Simulation code is host-only and exempt from the no-heap /
 *          no-exceptions flight rules, but this module follows the flight
 *          style anyway: static sizing, `Status` returns, fixed-width
 *          types. Truth states use `F64` deliberately — the simulator is
 *          the reference the F32 flight code is judged against, so it must
 *          not share the flight code's rounding.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_SIM_LANDER_DYNAMICS_HPP
#define LLS_SIM_LANDER_DYNAMICS_HPP

#include "lls/lls_types.hpp"

namespace lls {
namespace sim {

/** @brief Standard gravity used in the Isp relation, m/s^2. */
constexpr F64 kStandardGravityMps2 = 9.80665;

/** @brief Lunar surface gravity, m/s^2 (matches config/landing_params.yaml). */
constexpr F64 kLunarGravityMps2 = 1.625;

/** @brief Vehicle and engine constants for the truth model. */
struct VehicleParams {
    F64 dry_mass_kg        = 280.0;   /**< Mass with tanks empty.          */
    F64 propellant_mass_kg = 320.0;   /**< Usable propellant at sim start. */
    F64 max_thrust_n       = 2500.0;  /**< Thrust at throttle = 1.0.       */
    F64 min_throttle_frac  = 0.30;    /**< Deep-throttle floor; commands
                                           below this clamp up to it.      */
    F64 specific_impulse_s = 310.0;   /**< Isp for propellant depletion.   */
};

/** @brief Truth state of the 1-DOF lander. Up is positive. */
struct LanderState {
    F64 altitude_m   = 0.0;  /**< Height above the landing site.           */
    F64 velocity_mps = 0.0;  /**< Vertical velocity; descent is negative.  */
    F64 mass_kg      = 0.0;  /**< Total vehicle mass (dry + propellant).   */
};

/**
 * @brief 1-DOF truth dynamics stepped at a fixed rate by the sim executive.
 *
 * Lifecycle mirrors flight components: `Init()` once, then `Step()` at the
 * simulation rate until touchdown.
 */
class LanderDynamics {
 public:
    LanderDynamics() noexcept = default;

    /**
     * @brief   Configure the vehicle and set the initial truth state.
     *
     * @param   params              Vehicle constants; all fields must be
     *                              finite and positive, with
     *                              `min_throttle_frac` in (0, 1].
     * @param   initial_altitude_m  Start altitude, > 0.
     * @param   initial_velocity_mps Start vertical velocity (descent < 0).
     *
     * @retval  Status::kSuccess          Model ready; state initialized.
     * @retval  Status::kErrInvalidParam  A parameter is out of range.
     */
    [[nodiscard]] Status Init(const VehicleParams& params,
                              F64 initial_altitude_m,
                              F64 initial_velocity_mps) noexcept;

    /**
     * @brief   Advance the truth state by one time step.
     *
     * @details Semi-implicit Euler integration (velocity first, then
     *          position), adequate for the smooth 1-DOF problem at the
     *          50 Hz to 1 kHz rates the sim uses. Throttle commands below
     *          the deep-throttle floor are clamped up to it and commands
     *          above 1.0 are clamped down — matching the engine, not the
     *          controller, is the point of a truth model. A command of
     *          exactly 0.0 models engine cutoff (no thrust, no flow).
     *          When propellant is exhausted, thrust is zero regardless of
     *          the command.
     *
     * @param   throttle_cmd_frac  Commanded throttle in [0, 1]; 0 = cutoff.
     * @param   dt_s               Step size in seconds, in (0, 0.1].
     *
     * @retval  Status::kSuccess          State advanced.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam  @p dt_s or the command is invalid
     *                                    (non-finite or negative).
     */
    [[nodiscard]] Status Step(F64 throttle_cmd_frac, F64 dt_s) noexcept;

    /** @brief Current truth state. */
    [[nodiscard]] const LanderState& GetState() const noexcept;

    /** @brief Remaining propellant, kg (0 when tanks are dry). */
    [[nodiscard]] F64 GetPropellantRemainingKg() const noexcept;

    /** @brief True once altitude has reached zero (surface contact). */
    [[nodiscard]] bool HasTouchedDown() const noexcept;

 private:
    VehicleParams params_{};
    LanderState state_{};
    F64 propellant_kg_ = 0.0;
    bool touched_down_ = false;
    bool is_initialized_ = false;
};

}  // namespace sim
}  // namespace lls

#endif  // LLS_SIM_LANDER_DYNAMICS_HPP
