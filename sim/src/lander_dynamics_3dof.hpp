/**
 * @file    lander_dynamics_3dof.hpp
 * @brief   3-DOF planar lunar lander truth dynamics for the SIL sim
 *          (milestone 2).
 *
 * @details Planar rigid body in the vertical plane of the approach
 *          trajectory: two translational DOF (downrange x, altitude z) and
 *          one rotational DOF (pitch about the out-of-plane axis).
 *
 *          Actuators modeled:
 *            - Body-fixed main engine thrusting along the vehicle axis,
 *              with the same deep-throttle clamp, cutoff semantics, and
 *              Isp-based propellant depletion as the 1-DOF model. The
 *              thrust direction in inertial coordinates is
 *              (sin(pitch), cos(pitch)): pitch = 0 is thrust straight up,
 *              positive pitch tilts thrust toward +x.
 *            - RCS pitch torque, commanded as a fraction of the maximum
 *              couple. RCS propellant use is neglected (small against main
 *              engine flow; revisit with a real vehicle mass budget).
 *
 *          Integration is semi-implicit Euler (rates first, then states),
 *          matching the 1-DOF model.
 *
 *          Simulation code is host-only and exempt from the no-heap /
 *          no-exceptions flight rules, but follows the flight style anyway.
 *          Truth states use `F64` deliberately — the simulator is the
 *          reference the F32 flight code is judged against, so it must not
 *          share the flight code's rounding.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_SIM_LANDER_DYNAMICS_3DOF_HPP
#define LLS_SIM_LANDER_DYNAMICS_3DOF_HPP

#include "lander_dynamics.hpp" /* kStandardGravityMps2, kLunarGravityMps2. */
#include "lls/lls_types.hpp"

namespace lls {
namespace sim {

/** @brief Vehicle and actuator constants for the 3-DOF truth model. */
struct VehicleParams3Dof {
    F64 dry_mass_kg = 280.0;        /**< Mass with tanks empty.          */
    F64 propellant_mass_kg = 320.0; /**< Usable propellant at sim start. */
    F64 max_thrust_n = 2500.0;      /**< Thrust at throttle = 1.0.       */
    F64 min_throttle_frac = 0.30;   /**< Deep-throttle floor.            */
    F64 specific_impulse_s = 310.0; /**< Isp for propellant depletion.   */
    F64 pitch_inertia_kgm2 = 450.0; /**< Pitch-axis moment of inertia.
                                         Held constant: the propellant
                                         fraction of inertia is small for
                                         a center-mounted tank.          */
    F64 max_rcs_torque_nm = 400.0;  /**< RCS couple at command = +/-1.   */
};

/** @brief Handover (gate) state where the 3-DOF simulation begins. */
struct Gate3Dof {
    F64 altitude_m = 2000.0;    /**< Height above the site, > 0.        */
    F64 velocity_x_mps = 60.0;  /**< Ground speed toward the site.      */
    F64 velocity_z_mps = -30.0; /**< Vertical velocity (descent < 0).   */
    F64 pitch_rad = -0.30;      /**< Initial attitude; negative pitch
                                     tilts thrust against the direction
                                     of travel (braking attitude).      */
    F64 pitch_rate_radps = 0.0; /**< Initial body rate.                 */
};

/** @brief Truth state of the planar lander. Up and downrange positive. */
struct LanderState3Dof {
    F64 downrange_m = 0.0;      /**< Ground track distance flown.         */
    F64 altitude_m = 0.0;       /**< Height above the landing site.       */
    F64 velocity_x_mps = 0.0;   /**< Horizontal (downrange) velocity.     */
    F64 velocity_z_mps = 0.0;   /**< Vertical velocity (descent < 0).     */
    F64 pitch_rad = 0.0;        /**< 0 = vertical; +tilts thrust to +x.   */
    F64 pitch_rate_radps = 0.0; /**< Body pitch rate.                     */
    F64 mass_kg = 0.0;          /**< Total vehicle mass.                  */
};

/**
 * @brief 3-DOF truth dynamics stepped at a fixed rate by the sim executive.
 *
 * Lifecycle mirrors flight components: `Init()` once, then `Step()` at the
 * simulation rate until touchdown.
 */
class LanderDynamics3Dof {
 public:
    LanderDynamics3Dof() noexcept = default;

    /**
     * @brief   Configure the vehicle and set the gate state.
     *
     * @param   params  Vehicle constants; all fields must be finite and
     *                  positive, with `min_throttle_frac` in (0, 1].
     * @param   gate    Handover state; `altitude_m` must be > 0, all
     *                  fields finite, and `|pitch_rad|` < pi/2 (the
     *                  vehicle arrives thrust-up).
     *
     * @retval  Status::kSuccess          Model ready; state initialized.
     * @retval  Status::kErrInvalidParam  A parameter is out of range.
     */
    [[nodiscard]] Status Init(const VehicleParams3Dof& params,
                              const Gate3Dof& gate) noexcept;

    /**
     * @brief   Advance the truth state by one time step.
     *
     * @details Engine semantics match the 1-DOF model: a command of exactly
     *          0.0 is cutoff; positive commands are clamped into the
     *          deep-throttle band; dry tanks produce no thrust. The RCS
     *          torque command is clamped to [-1, 1] and remains effective
     *          regardless of main-engine state (separate propellant budget,
     *          neglected). After touchdown the state is frozen.
     *
     * @param   throttle_cmd_frac  Commanded throttle in [0, 1]; 0 = cutoff.
     * @param   torque_cmd_frac    Commanded RCS couple as a fraction of the
     *                             maximum, in [-1, 1] (clamped).
     * @param   dt_s               Step size in seconds, in (0, 0.1].
     *
     * @retval  Status::kSuccess          State advanced.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam  A command or @p dt_s is invalid
     *                                    (non-finite, or throttle negative,
     *                                    or dt out of range).
     */
    [[nodiscard]] Status Step(F64 throttle_cmd_frac, F64 torque_cmd_frac,
                              F64 dt_s) noexcept;

    /** @brief Current truth state. */
    [[nodiscard]] const LanderState3Dof& GetState() const noexcept;

    /** @brief Remaining propellant, kg (0 when tanks are dry). */
    [[nodiscard]] F64 GetPropellantRemainingKg() const noexcept;

    /** @brief True once altitude has reached zero (surface contact). */
    [[nodiscard]] bool HasTouchedDown() const noexcept;

 private:
    VehicleParams3Dof params_{};
    LanderState3Dof state_{};
    F64 propellant_kg_ = 0.0;
    bool touched_down_ = false;
    bool is_initialized_ = false;
};

}  // namespace sim
}  // namespace lls

#endif  // LLS_SIM_LANDER_DYNAMICS_3DOF_HPP
