/**
 * @file    monte_carlo.hpp
 * @brief   Monte-Carlo dispersion runner for the 3-DOF approach descent
 *          (SIL milestone 4).
 *
 * @details Flies the closed-loop approach simulation many times with the
 *          uncertainties a real mission carries, and reports whether the
 *          flight software lands safely across the whole envelope — the
 *          statistical argument behind "this controller works", and a CI
 *          gate alongside the single nominal run.
 *
 *          Dispersed per run (truncated Gaussian, +/-3 sigma):
 *            - Gate state: altitude, ground speed, descent rate, pitch.
 *            - Vehicle: dry mass, propellant load, engine max thrust,
 *              specific impulse. The *flight* configuration (thrust
 *              allocator) keeps the nominal engine model — the control
 *              loops must absorb the calibration mismatch, as in flight.
 *            - Navigation: IMU accelerometer bias, filter handover errors
 *              (drawn centered on zero), and fresh sensor-noise seeds
 *              derived per run.
 *
 *          Every run is deterministic: one base seed fixes the entire
 *          campaign, so a Monte-Carlo failure is exactly reproducible by
 *          rerunning with the same seed — a regression test, not a shrug.
 *
 *          Simulation code is host-only and exempt from the flight rules,
 *          but follows the flight style anyway: fixed-capacity storage,
 *          bounded loops, `Status` returns.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_SIM_MONTE_CARLO_HPP
#define LLS_SIM_MONTE_CARLO_HPP

#include <array>

#include "approach_sim.hpp"
#include "lls/lls_types.hpp"

namespace lls {
namespace sim {

/** @brief Mission touchdown limits used to judge each run. */
struct LandingCriteria {
    F64 max_vertical_speed_mps = 2.0;   /**< config:
                                             touchdown_velocity_limit_mps. */
    F64 max_horizontal_speed_mps = 1.0; /**< config:
                                             touchdown_horizontal_limit_mps.*/
    F64 max_tilt_rad = 0.0873;          /**< config: touchdown_tilt_limit_deg
                                             (5 deg).                      */
    F64 max_miss_distance_m = 50.0;     /**< config: touchdown_miss_limit_m
                                             (site-targeting accuracy).    */
};

/**
 * @brief 1-sigma dispersion magnitudes (0 disables that dispersion).
 *
 * Draws are truncated at +/-3 sigma and the resulting scenario values are
 * clamped to physical guardrails, so every generated run is a valid — if
 * unlucky — mission, never a nonsensical one.
 */
struct DispersionSigmas {
    F64 gate_altitude_m = 100.0;   /**< Handover altitude knowledge.     */
    F64 gate_velocity_x_mps = 5.0; /**< Handover ground speed.           */
    F64 gate_velocity_z_mps = 3.0; /**< Handover descent rate.           */
    F64 gate_pitch_rad = 0.05;     /**< Handover attitude (~3 deg).      */

    F64 dry_mass_kg = 5.0;        /**< As-built mass uncertainty.       */
    F64 propellant_mass_kg = 5.0; /**< Loading uncertainty.             */
    F64 max_thrust_n = 50.0;      /**< Engine performance (2%). Must
                                       stay inside the 10% flight-config
                                       calibration tolerance.           */
    F64 specific_impulse_s = 3.0; /**< Engine performance.              */

    F64 imu_accel_bias_mps2 = 0.01;   /**< Turn-on bias, around nominal.    */
    F64 nav_altitude_error_m = 5.0;   /**< Filter handover seed error,
                                           drawn centered on zero.          */
    F64 nav_velocity_error_mps = 1.0; /**< Filter handover seed error,
                                           drawn centered on zero.         */
};

/** @brief Campaign configuration. */
struct MonteCarloParams {
    ApproachScenarioParams nominal{}; /**< Center of the dispersions.      */
    DispersionSigmas sigmas{};        /**< 1-sigma dispersion magnitudes.  */
    LandingCriteria criteria{};       /**< Pass/fail limits per run.       */
    U32 run_count = 200U;             /**< Runs to fly, in [1, kMaxRuns].  */
    U64 base_seed = 0x5E1E4E5EEDULL;  /**< Fixes the entire campaign.      */
};

/** @brief Outcome of one dispersed run (inputs echoed for traceability). */
struct MonteCarloRunRecord {
    U32 run_index = 0U;
    bool safe = false; /**< Touched down within every criterion.           */

    /* Dispersed inputs actually flown. */
    F64 gate_altitude_m = 0.0;
    F64 gate_velocity_x_mps = 0.0;
    F64 gate_velocity_z_mps = 0.0;
    F64 gate_pitch_rad = 0.0;

    /* Outcome. */
    bool touched_down = false;
    F64 touchdown_vertical_speed_mps = 0.0;
    F64 touchdown_horizontal_speed_mps = 0.0;
    F64 touchdown_tilt_rad = 0.0;
    F64 touchdown_downrange_m = 0.0;
    F64 touchdown_miss_m = 0.0;
    F64 flight_time_s = 0.0;
    F64 propellant_used_kg = 0.0;
    F64 nav_altitude_error_m = 0.0;
    F64 nav_velocity_error_mps = 0.0;
    U32 controller_fault_count = 0U;
    U32 nav_rejected_measurement_count = 0U;
};

/** @brief Mean/deviation/extremes of one scalar metric across the runs. */
struct MetricStats {
    F64 mean = 0.0;
    F64 std_dev = 0.0;
    F64 min = 0.0;
    F64 max = 0.0;
};

/** @brief Campaign-level aggregates. */
struct MonteCarloSummary {
    U32 run_count = 0U;  /**< Runs flown.                                  */
    U32 safe_count = 0U; /**< Runs inside every touchdown criterion.       */
    U32 first_failed_run = 0U; /**< Lowest failed run index (valid only
                                    when safe_count < run_count); rerun it
                                    with the same base seed to reproduce.  */

    MetricStats vertical_speed_mps{};     /**< Touchdown descent rate.       */
    MetricStats horizontal_speed_mps{};   /**< Touchdown lateral speed.      */
    MetricStats tilt_rad{};               /**< Touchdown tilt.               */
    MetricStats downrange_m{};            /**< Landing footprint (along
                                               track).                       */
    MetricStats miss_m{};                 /**< Distance from the targeted
                                               site at contact.              */
    MetricStats flight_time_s{};          /**< Gate to touchdown.            */
    MetricStats propellant_used_kg{};     /**< Fuel budget statistic.        */
    MetricStats nav_altitude_error_m{};   /**< Estimator error at contact.   */
    MetricStats nav_velocity_error_mps{}; /**< Estimator error at contact. */

    U32 total_controller_faults = 0U; /**< Sum across all runs.          */
    U32 total_nav_rejected_measurements = 0U; /**< Sum across all runs.    */
};

/** @brief Fixed-capacity per-run recorder (rule #1: no heap, even here). */
class MonteCarloLog {
 public:
    static constexpr U32 kCapacity = 2000U;

    /** @brief Append a record; silently drops once full (bounded storage).*/
    void Record(const MonteCarloRunRecord& record) noexcept;

    /** @brief Number of stored records, <= kCapacity. */
    [[nodiscard]] U32 GetCount() const noexcept;

    /** @brief Record at @p index; index must be < GetCount(). */
    [[nodiscard]] const MonteCarloRunRecord& GetRecord(
        U32 index) const noexcept;

 private:
    std::array<MonteCarloRunRecord, kCapacity> records_{};
    U32 count_ = 0U;
};

/** @brief Most runs a campaign may fly (bounded loop, log capacity). */
constexpr U32 kMaxMonteCarloRuns = MonteCarloLog::kCapacity;

/**
 * @brief   Fly a Monte-Carlo campaign of dispersed approach descents.
 *
 * @details A dispersed scenario the approach simulation rejects (possible
 *          only under extreme user-supplied sigmas) is scored as an unsafe
 *          run rather than aborting the campaign.
 *
 * @param   params       Campaign configuration. `run_count` must be in
 *                       [1, kMaxMonteCarloRuns]; sigmas must be finite and
 *                       non-negative; criteria must be positive.
 * @param   summary_out  Non-null; receives the campaign aggregates.
 * @param   log_out      Optional per-run recorder; pass nullptr to skip.
 *
 * @retval  Status::kSuccess          Campaign completed (inspect
 *                                    `summary_out->safe_count`).
 * @retval  Status::kErrInvalidParam  Null summary pointer or a campaign
 *                                    parameter out of range.
 */
[[nodiscard]] Status RunMonteCarlo(const MonteCarloParams& params,
                                   MonteCarloSummary* summary_out,
                                   MonteCarloLog* log_out) noexcept;

}  // namespace sim
}  // namespace lls

#endif  // LLS_SIM_MONTE_CARLO_HPP
