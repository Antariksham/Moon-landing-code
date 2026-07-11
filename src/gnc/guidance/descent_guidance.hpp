/**
 * @file    descent_guidance.hpp
 * @brief   Altitude-keyed powered-descent guidance for approach and terminal
 *          descent (Phase I milestone 2).
 *
 * @details First flight guidance module of Project SELENE. Produces the
 *          velocity references the control loops track during the approach
 *          and terminal-descent mission phases, keyed to altitude in the
 *          Apollo tradition (references are a function of where the vehicle
 *          is, not of time, so the profile is robust to dispersions in when
 *          the vehicle gets there).
 *
 *          Vertical channel (identical shape to the milestone 1 profile):
 *
 *              vz_cmd(h) = -min(v_max, v_f + sqrt(2*a_brake*(h - h_t)))
 *                                                     for h >  h_t
 *              vz_cmd(h) = -v_f                       for h <= h_t
 *
 *          i.e. a constant-deceleration braking envelope into a constant
 *          final descent rate below the terminal gate `h_t`.
 *
 *          Horizontal channel (the pitch-over schedule):
 *
 *              vx_cmd(h) = min(vx_max, s * (h - h_po))   for h >  h_po
 *              vx_cmd(h) = 0                             for h <= h_po
 *
 *          i.e. the allowed ground speed shrinks linearly with altitude and
 *          reaches zero at the pitch-over altitude `h_po`. Tracking this
 *          ramp forces the vehicle to trade its horizontal approach speed
 *          for a vertical attitude as it descends — the pitch-over
 *          maneuver — and guarantees a purely vertical terminal descent.
 *
 *          The module also raises two discrete commands:
 *            - `terminal_phase` when h <= h_po: the executive should
 *              transition APPROACH -> TERMINAL_DESCENT and attitude must be
 *              held vertical.
 *            - `engine_cutoff` when h <= h_cutoff: guidance requests main
 *              engine shutdown for the final free-fall to the surface.
 *
 *          Compliance: no allocation, no exceptions, bounded execution
 *          (`Update()` is loop-free), F32 throughout (the profile needs no
 *          double precision: worst-case altitudes are < 2^13 m and the
 *          sqrt argument is well conditioned).
 *
 * @par Real-time characteristics
 *          `Update()` is allocation-free, loop-free, and intended to run in
 *          the 50 Hz guidance slot of the flight executive. Not thread-safe;
 *          single-task ownership is assumed.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_GNC_GUIDANCE_DESCENT_GUIDANCE_HPP
#define LLS_GNC_GUIDANCE_DESCENT_GUIDANCE_HPP

#include "lls/lls_types.hpp"

namespace lls {
namespace gnc {

/**
 * @brief Configuration of the descent-guidance profile.
 *
 * Defaults mirror `config/landing_params.yaml`. All values are validated by
 * `DescentGuidance::Init()`; see that contract for acceptance criteria.
 */
struct DescentGuidanceConfig {
    F32 brake_decel_mps2 = 1.2F;         /**< Vertical envelope deceleration.
                                              Range: (0, finite). Must be well
                                              under the vehicle's max net decel. */
    F32 max_descent_rate_mps = 25.0F;    /**< Speed cap on the envelope, > 0. */
    F32 final_descent_rate_mps = 1.0F;   /**< Constant-rate final segment, >0.*/
    F32 terminal_altitude_m = 10.0F;     /**< Start of the final segment.     */
    F32 engine_cutoff_altitude_m = 0.5F; /**< Free-fall below this.         */

    F32 pitchover_altitude_m = 150.0F;    /**< Ground speed must be zero here;
                                               must be >= terminal_altitude_m. */
    F32 horizontal_rate_slope_hz = 0.03F; /**< Allowed ground speed per meter
                                               of altitude above pitch-over,
                                               (m/s)/m = 1/s. Range > 0.    */
    F32 max_horizontal_rate_mps = 80.0F;  /**< Ground-speed cap, > 0.       */
};

/**
 * @brief Velocity references and discrete commands for one guidance cycle.
 *
 * Sign conventions: up is positive for the vertical rate (descent commands
 * are negative); the horizontal rate is a non-negative allowed ground speed
 * along the approach direction of travel.
 */
struct GuidanceCommand {
    F32 vertical_rate_cmd_mps = 0.0F;   /**< Commanded vertical velocity.   */
    F32 horizontal_rate_cmd_mps = 0.0F; /**< Allowed ground speed, >= 0.    */
    bool terminal_phase = false;        /**< True at/below pitch-over altitude:
                                             attitude must be vertical and the
                                             executive should enter
                                             TERMINAL_DESCENT.                     */
    bool engine_cutoff = false;         /**< True at/below cutoff altitude: shut
                                             down the main engine and fall.        */
};

/**
 * @brief Deterministic altitude-keyed descent guidance.
 *
 * Lifecycle: construct (trivially) -> `Init()` during system initialization
 * -> `Update()` at the guidance rate. The object holds only the validated
 * configuration; `Update()` is a pure function of altitude.
 */
class DescentGuidance {
 public:
    /** @brief Trivial constructor; the object is unusable until `Init()`. */
    DescentGuidance() noexcept = default;

    /* One guidance profile per vehicle: duplicating the object would allow
     * two divergent references into the control loops. */
    DescentGuidance(const DescentGuidance&) = delete;
    DescentGuidance& operator=(const DescentGuidance&) = delete;
    DescentGuidance(DescentGuidance&&) = delete;
    DescentGuidance& operator=(DescentGuidance&&) = delete;

    ~DescentGuidance() = default;

    /**
     * @brief   Validate the profile and arm the guidance module.
     *
     * @param   config  Candidate profile. Accepted if and only if every
     *                  field is finite and positive, and the altitude gates
     *                  are ordered:
     *                  `engine_cutoff_altitude_m < terminal_altitude_m
     *                   <= pitchover_altitude_m`.
     *
     * @retval  Status::kSuccess          Guidance armed.
     * @retval  Status::kErrInvalidParam  Profile rejected; the module
     *                                    remains (or becomes) uninitialized
     *                                    and `Update()` will refuse to run.
     */
    [[nodiscard]] Status Init(const DescentGuidanceConfig& config) noexcept;

    /**
     * @brief   Compute the velocity references for the current altitude.
     *
     * @details On any failure the value at @p cmd_out is left unmodified,
     *          so the caller's previous reference is preserved by
     *          construction (same convention as `PidController::Update()`).
     *
     * @param   altitude_m  Estimated height above the landing site, m.
     *                      Valid range: [0, finite).
     * @param   cmd_out     Non-null pointer receiving the references.
     *
     * @retval  Status::kSuccess           References written.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam   @p cmd_out is null or
     *                                     @p altitude_m is negative.
     * @retval  Status::kErrNonFiniteInput @p altitude_m is NaN or Inf.
     */
    [[nodiscard]] Status Update(F32 altitude_m,
                                GuidanceCommand* cmd_out) const noexcept;

 private:
    DescentGuidanceConfig config_{}; /**< Validated copy of the profile.    */
    bool is_initialized_ = false;    /**< Set only by a successful Init().  */
};

}  // namespace gnc
}  // namespace lls

#endif  // LLS_GNC_GUIDANCE_DESCENT_GUIDANCE_HPP
