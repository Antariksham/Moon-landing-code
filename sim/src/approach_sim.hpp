/**
 * @file    approach_sim.hpp
 * @brief   Closed-loop 3-DOF approach + pitch-over + terminal-descent
 *          simulation (SIL milestones 2 + 3).
 *
 * @details Flies the full approach phase: the vehicle arrives at the
 *          handover gate with significant horizontal velocity in a braking
 *          attitude, pitches over to vertical as guidance ramps its allowed
 *          ground speed to zero, and completes the same terminal descent
 *          the milestone 1 sim flies — this time with attitude in the loop
 *          and (milestone 3) the vertical channel flying on estimated
 *          state instead of truth.
 *
 *          Flight code under test (all built under the strict flight
 *          flags):
 *            - `lls::gnc::VerticalNavFilter` — altitude/velocity estimate
 *              fused from the noisy IMU + radar-altimeter models.
 *            - `lls::gnc::DescentGuidance` — altitude-keyed velocity
 *              references and the pitch-over/terminal/cutoff discretes,
 *              driven by the navigation altitude.
 *            - `lls::gnc::PidController` x3 — horizontal velocity,
 *              vertical velocity, and pitch attitude loops.
 *            - `lls::gnc::ThrustAllocator` — acceleration command to
 *              pitch + throttle.
 *            - `lls::fsw::MissionStateMachine` — sequenced through
 *              APPROACH -> TERMINAL_DESCENT -> TOUCHDOWN -> SAFED by the
 *              sim executive, exactly as the flight executive would.
 *
 *          Per 50 Hz cycle:
 *
 *              IMU delta-v (previous step)  -> nav.Predict
 *              altimeter return (10 Hz)     -> nav.UpdateAltitude
 *              guidance(h_nav) -> (vx_cmd, vz_cmd, discretes)
 *              PID_x(vx_cmd, vx) -> ax_cmd      [thrust accel, m/s^2]
 *              PID_z(vz_cmd, vz_nav) + g_ff -> az_cmd
 *              allocator(ax_cmd, az_cmd, m) -> (pitch_cmd, throttle)
 *              PID_att(pitch_cmd, pitch) -> RCS torque fraction
 *
 *          The vertical loop's gravity feedforward carries the static
 *          load, so integrators only absorb modeling error — the same
 *          feedforward-plus-correction doctrine as milestone 1. The
 *          horizontal and attitude channels still read truth state
 *          (perfect navigation) until TRN/star-tracker milestones give
 *          them sensors; the vehicle mass fed to the allocator is truth
 *          (propellant bookkeeping is deterministic on the real vehicle).
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_SIM_APPROACH_SIM_HPP
#define LLS_SIM_APPROACH_SIM_HPP

#include <array>

#include "fsw/mission_state_machine.hpp"
#include "gnc/control/thrust_allocator.hpp"
#include "gnc/guidance/descent_guidance.hpp"
#include "gnc/navigation/vertical_nav_filter.hpp"
#include "hda/safe_site_selector.hpp"
#include "lander_dynamics_3dof.hpp"
#include "lls/lls_types.hpp"
#include "sensor_models.hpp"
#include "terrain_model.hpp"

namespace lls {
namespace sim {

/**
 * @brief Navigation configuration for the closed-loop run (milestone 3).
 *
 * By default the vertical channel flies on the flight
 * `lls::gnc::VerticalNavFilter` fed by the noisy IMU/altimeter models:
 * guidance altitude, the vertical-rate loop's measurement, and the
 * engine-cutoff discrete all use estimated state. The horizontal and
 * attitude channels still read truth (perfect navigation) until the TRN
 * and star-tracker milestones give them sensors of their own.
 */
struct NavScenarioParams {
    bool use_perfect_navigation = false; /**< True: bypass sensors + filter
                                              and fly on truth (milestone 2
                                              behavior, for A/B runs).     */

    /** Flight filter tuning. The `initial_altitude_m` /
     *  `initial_velocity_mps` fields are overwritten by the sim: the
     *  filter is seeded from the gate state plus the offsets below.      */
    gnc::VerticalNavFilterConfig filter{};

    F64 initial_altitude_error_m = 5.0;   /**< Seed error vs. gate truth.   */
    F64 initial_velocity_error_mps = 1.0; /**< Seed error vs. gate truth.  */

    ImuModelParams imu{};             /**< Accelerometer error model.      */
    AltimeterModelParams altimeter{}; /**< Radar-altimeter error model.    */
};

/**
 * @brief Hazard detection & avoidance configuration (milestone 6).
 *
 * At the first control cycle in APPROACH with the navigation altitude at
 * or below `scan_altitude_m`, the sim's terrain-mapper stand-in surveys
 * the ground around the current target (perfect terrain sensing; LIDAR
 * noise is a follow-up) and the flight `SafeSiteSelector` decides whether
 * to keep the site or divert. Guidance re-targets on the spot — the
 * site-targeting law accepts any reachable target.
 */
struct HdaScenarioParams {
    bool enabled = true; /**< False: fly the nominal target blind.         */

    hda::SafeSiteSelectorConfig selector{}; /**< Flight mission limits.    */

    F64 scan_altitude_m = 1000.0;   /**< Decision gate: high enough that the
                                         full divert envelope is reachable
                                         before the pitch-over altitude.     */
    F64 survey_halfwidth_m = 300.0; /**< Mapped span around the target
                                         (matches the divert envelope).    */
    F64 sample_spacing_m = 4.0;     /**< Survey station spacing. Must not
                                         exceed the selector's footprint
                                         radius, or every candidate fails the
                                         coverage check.                       */
};

/** @brief Scenario configuration (defaults mirror config/landing_params.yaml).
 */
struct ApproachScenarioParams {
    VehicleParams3Dof vehicle{}; /**< Truth-model constants.               */
    Gate3Dof gate{};             /**< Approach handover state.             */

    TerrainParams terrain{}; /**< Base surface properties.                 */
    std::array<HazardZone, TerrainModel::kMaxHazardZones> hazard_zones{};
    U32 hazard_zone_count = 0U; /**< Valid entries in `hazard_zones`.      */
    HdaScenarioParams hda{};    /**< Terrain survey + divert decision.     */

    gnc::DescentGuidanceConfig guidance{};  /**< Flight guidance profile.   */
    gnc::ThrustAllocatorConfig allocator{}; /**< Control allocation limits.
                                                 `max_thrust_n` must match
                                                 the truth vehicle.        */
    NavScenarioParams nav{};                /**< Sensors + flight filter.   */

    F64 target_downrange_m = 1200.0; /**< Landing-site position, measured
                                          from the gate (downrange 0),
                                          along +x. Mission design must
                                          pick a site the gate energy and
                                          the guidance ramp can reach.     */

    F64 control_rate_hz = 50.0;     /**< Flight control loop rate.        */
    F64 max_sim_duration_s = 600.0; /**< Hard bound on the sim loop.      */
};

/** @brief One telemetry sample of the closed-loop 3-DOF run. */
struct ApproachTelemetrySample {
    F64 time_s = 0.0;
    F64 downrange_m = 0.0;
    F64 altitude_m = 0.0;
    F64 velocity_x_mps = 0.0;
    F64 velocity_z_mps = 0.0;
    F64 velocity_x_cmd_mps = 0.0;
    F64 velocity_z_cmd_mps = 0.0;
    F64 pitch_rad = 0.0;
    F64 pitch_cmd_rad = 0.0;
    F64 throttle_frac = 0.0;
    F64 torque_frac = 0.0;
    F64 mass_kg = 0.0;
    F64 nav_altitude_m = 0.0;     /**< Filter estimate in the loop.       */
    F64 nav_velocity_z_mps = 0.0; /**< Filter estimate in the loop.       */
    U8 mission_phase = 0U; /**< `MissionPhase` value, telemetry encoding. */
};

/** @brief Outcome of a completed (or aborted) 3-DOF run. */
struct ApproachSimResult {
    bool touched_down = false; /**< Reached the surface within bounds.     */
    F64 touchdown_vertical_speed_mps = 0.0;   /**< |vz| at contact.        */
    F64 touchdown_horizontal_speed_mps = 0.0; /**< |vx| at contact.        */
    F64 touchdown_tilt_rad = 0.0;             /**< |pitch| at contact.     */
    F64 touchdown_downrange_m = 0.0;          /**< Ground track flown to contact
                                                   (landing-footprint statistic).   */
    F64 touchdown_miss_m = 0.0; /**< |final target - contact point| along
                                     track (site-targeting accuracy,
                                     measured against the post-divert
                                     target).                              */
    F64 final_target_downrange_m = 0.0; /**< Target actually flown to
                                             (post-divert).                */
    bool hda_diverted = false;       /**< HDA moved the landing site.       */
    F64 hda_divert_distance_m = 0.0; /**< |new - nominal| target shift.    */
    bool hda_no_safe_site = false;   /**< Survey found nothing acceptable;
                                          vehicle held the nominal target
                                          under fault protection.           */
    bool landed_on_hazard = false;   /**< Truth terrain at the contact point
                                          violates the mission limits.      */
    F64 flight_time_s = 0.0;         /**< Elapsed sim time.                   */
    F64 propellant_used_kg = 0.0;    /**< Propellant consumed.                */
    U32 controller_fault_count = 0U; /**< Non-nominal flight-code statuses
                                          (saturation excluded).           */
    fsw::MissionPhase final_phase =
        fsw::MissionPhase::kBoot; /**< Executive phase at sim end.         */
    U32 rejected_transition_count = 0U; /**< State-machine rejections.     */

    U32 nav_rejected_measurement_count = 0U;    /**< Altimeter returns the
                                                     filter's innovation gate
                                                     refused.                 */
    F64 touchdown_nav_altitude_error_m = 0.0;   /**< |estimate - truth| at
                                                     sim end (0 when flying
                                                     perfect navigation).  */
    F64 touchdown_nav_velocity_error_mps = 0.0; /**< |estimate - truth| at
                                                     sim end (0 when flying
                                                     perfect navigation).  */
};

/** @brief Fixed-capacity telemetry recorder (rule #1: no heap, even here). */
class ApproachTelemetryLog {
 public:
    /** 600 s at 50 Hz. */
    static constexpr U32 kCapacity = 30000U;

    /** @brief Append a sample; silently drops once full (bounded storage). */
    void Record(const ApproachTelemetrySample& sample) noexcept;

    /** @brief Number of stored samples, <= kCapacity. */
    [[nodiscard]] U32 GetCount() const noexcept;

    /** @brief Sample at @p index; index must be < GetCount(). */
    [[nodiscard]] const ApproachTelemetrySample& GetSample(
        U32 index) const noexcept;

 private:
    std::array<ApproachTelemetrySample, kCapacity> samples_{};
    U32 count_ = 0U;
};

/**
 * @brief   Run the closed-loop 3-DOF descent from the approach gate to
 *          touchdown.
 *
 * @param   params      Scenario configuration.
 * @param   result_out  Non-null; receives the run outcome.
 * @param   log_out     Optional telemetry recorder; pass nullptr to skip.
 *
 * @retval  Status::kSuccess          Run completed (touchdown or timeout;
 *                                    inspect `result_out->touched_down`).
 * @retval  Status::kErrInvalidParam  Null result pointer or a scenario
 *                                    value the models reject.
 */
[[nodiscard]] Status RunApproachSim(const ApproachScenarioParams& params,
                                    ApproachSimResult* result_out,
                                    ApproachTelemetryLog* log_out) noexcept;

}  // namespace sim
}  // namespace lls

#endif  // LLS_SIM_APPROACH_SIM_HPP
