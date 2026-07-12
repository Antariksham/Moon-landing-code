/**
 * @file    monte_carlo.cpp
 * @brief   Implementation of the Monte-Carlo dispersion runner.
 *
 * @see     monte_carlo.hpp for the dispersion model and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "monte_carlo.hpp"

#include <cmath>

#include "lls/lls_assert.hpp"
#include "sensor_models.hpp"

namespace lls {
namespace sim {

void MonteCarloLog::Record(const MonteCarloRunRecord& record) noexcept {
    if (count_ < kCapacity) {
        records_[count_] = record;
        ++count_;
    }
}

U32 MonteCarloLog::GetCount() const noexcept {
    return count_;
}

const MonteCarloRunRecord& MonteCarloLog::GetRecord(
    const U32 index) const noexcept {
    LLS_ASSERT(index < count_);
    const U32 safe_index = (index < count_) ? index : 0U;
    return records_[safe_index];
}

namespace {

/** @brief Welford running statistics for one scalar metric. */
class RunningStats {
 public:
    void Add(const F64 value) noexcept {
        ++count_;
        const F64 delta = value - mean_;
        mean_ += delta / static_cast<F64>(count_);
        m2_ += delta * (value - mean_);
        min_ = (value < min_) ? value : min_;
        max_ = (value > max_) ? value : max_;
    }

    [[nodiscard]] MetricStats Finalize() const noexcept {
        MetricStats stats{};
        if (count_ > 0U) {
            stats.mean = mean_;
            stats.std_dev = std::sqrt(m2_ / static_cast<F64>(count_));
            stats.min = min_;
            stats.max = max_;
        }
        return stats;
    }

 private:
    U32 count_ = 0U;
    F64 mean_ = 0.0;
    F64 m2_ = 0.0;
    F64 min_ = 1.0e300;
    F64 max_ = -1.0e300;
};

/** @brief splitmix64 mixer: derives independent seeds from one base. */
[[nodiscard]] U64 MixSeed(const U64 base, const U64 salt) noexcept {
    U64 x = base + (salt * 0x9E3779B97F4A7C15ULL);
    x = (x ^ (x >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27U)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31U);
}

/** @brief One truncated-Gaussian dispersion draw: N(0, sigma) in +/-3 sigma.
 */
[[nodiscard]] F64 Draw(GaussianNoiseGenerator& gen, const F64 sigma) noexcept {
    F64 deviate = gen.NextGaussian();
    deviate = (deviate > 3.0) ? 3.0 : deviate;
    deviate = (deviate < -3.0) ? -3.0 : deviate;
    return deviate * sigma;
}

[[nodiscard]] F64 Clamp(const F64 value, const F64 lo, const F64 hi) noexcept {
    return (value < lo) ? lo : ((value > hi) ? hi : value);
}

[[nodiscard]] bool AreParamsValid(const MonteCarloParams& p) noexcept {
    const DispersionSigmas& s = p.sigmas;
    const F64 sigma_fields[] = {
        s.gate_altitude_m,
        s.gate_velocity_x_mps,
        s.gate_velocity_z_mps,
        s.gate_pitch_rad,
        s.dry_mass_kg,
        s.propellant_mass_kg,
        s.max_thrust_n,
        s.specific_impulse_s,
        s.imu_accel_bias_mps2,
        s.nav_altitude_error_m,
        s.nav_velocity_error_mps,
    };
    for (const F64 sigma : sigma_fields) { /* Bounded loop. */
        if (!std::isfinite(sigma) || (sigma < 0.0)) {
            return false;
        }
    }
    const bool criteria_valid =
        std::isfinite(p.criteria.max_vertical_speed_mps) &&
        (p.criteria.max_vertical_speed_mps > 0.0) &&
        std::isfinite(p.criteria.max_horizontal_speed_mps) &&
        (p.criteria.max_horizontal_speed_mps > 0.0) &&
        std::isfinite(p.criteria.max_tilt_rad) &&
        (p.criteria.max_tilt_rad > 0.0) &&
        std::isfinite(p.criteria.max_miss_distance_m) &&
        (p.criteria.max_miss_distance_m > 0.0);
    const bool hazards_valid = std::isfinite(p.hazards.hazard_probability) &&
                               (p.hazards.hazard_probability >= 0.0) &&
                               (p.hazards.hazard_probability <= 1.0) &&
                               std::isfinite(p.hazards.zone_length_m) &&
                               (p.hazards.zone_length_m > 0.0) &&
                               std::isfinite(p.hazards.center_offset_sigma_m) &&
                               (p.hazards.center_offset_sigma_m >= 0.0);
    return criteria_valid && hazards_valid && (p.run_count >= 1U) &&
           (p.run_count <= kMaxMonteCarloRuns);
}

/**
 * @brief   Generate the dispersed scenario for one run.
 *
 * @details Exactly eleven deviates are consumed per run in a fixed order,
 *          so the campaign is reproducible and every run is independent
 *          of how earlier runs terminated. Guardrail clamps keep extreme
 *          draws physical; the flight configuration (allocator) keeps the
 *          nominal engine model, so the truth engine's dispersion is a
 *          calibration mismatch the control loops must absorb.
 *
 * @param   params  Campaign configuration (nominal center + sigmas).
 * @param   gen     Campaign dispersion-draw generator (advanced in place).
 * @param   run     Run index, used to derive the sensor-noise seeds.
 * @return  A complete scenario for `RunApproachSim()`.
 */
[[nodiscard]] ApproachScenarioParams BuildDispersedScenario(
    const MonteCarloParams& params, GaussianNoiseGenerator& gen,
    const U32 run) noexcept {
    const DispersionSigmas& sig = params.sigmas;
    ApproachScenarioParams s = params.nominal;

    /* Gate state. */
    s.gate.altitude_m = Clamp(
        s.gate.altitude_m + Draw(gen, sig.gate_altitude_m), 500.0, 50000.0);
    s.gate.velocity_x_mps = Clamp(
        s.gate.velocity_x_mps + Draw(gen, sig.gate_velocity_x_mps), 0.0, 200.0);
    s.gate.velocity_z_mps =
        Clamp(s.gate.velocity_z_mps + Draw(gen, sig.gate_velocity_z_mps), -60.0,
              -5.0);
    s.gate.pitch_rad =
        Clamp(s.gate.pitch_rad + Draw(gen, sig.gate_pitch_rad), -0.6, 0.6);

    /* Vehicle build + engine performance. */
    s.vehicle.dry_mass_kg = Clamp(
        s.vehicle.dry_mass_kg + Draw(gen, sig.dry_mass_kg), 100.0, 1000.0);
    s.vehicle.propellant_mass_kg =
        Clamp(s.vehicle.propellant_mass_kg + Draw(gen, sig.propellant_mass_kg),
              50.0, 1000.0);
    /* Clamp inside the 10% flight-config calibration tolerance (9% margin)
     * so a legal campaign can never generate a scenario the approach sim
     * rejects for configuration mismatch.                                */
    const F64 nominal_thrust_n = params.nominal.vehicle.max_thrust_n;
    s.vehicle.max_thrust_n =
        Clamp(s.vehicle.max_thrust_n + Draw(gen, sig.max_thrust_n),
              0.91 * nominal_thrust_n, 1.09 * nominal_thrust_n);
    s.vehicle.specific_impulse_s =
        Clamp(s.vehicle.specific_impulse_s + Draw(gen, sig.specific_impulse_s),
              150.0, 500.0);

    /* Navigation: dispersed IMU bias, zero-centered handover errors, and
     * per-run sensor-noise seeds.                                        */
    s.nav.imu.accel_bias_mps2 += Draw(gen, sig.imu_accel_bias_mps2);
    s.nav.initial_altitude_error_m = Draw(gen, sig.nav_altitude_error_m);
    s.nav.initial_velocity_error_mps = Draw(gen, sig.nav_velocity_error_mps);
    const U64 run_salt = static_cast<U64>(run) + 1U;
    s.nav.imu.noise_seed = MixSeed(params.base_seed, run_salt * 2U);
    s.nav.altimeter.noise_seed =
        MixSeed(params.base_seed, (run_salt * 2U) + 1U);

    /* Terrain hazard: both deviates are always consumed so the draw
     * sequence stays aligned across hazard-probability settings.         */
    const F64 hazard_roll = gen.NextUniform();
    const F64 center_offset_m = Draw(gen, params.hazards.center_offset_sigma_m);
    if (hazard_roll < params.hazards.hazard_probability) {
        HazardZone zone{};
        const F64 center_m = s.target_downrange_m + center_offset_m;
        zone.start_m = center_m - (0.5 * params.hazards.zone_length_m);
        zone.end_m = center_m + (0.5 * params.hazards.zone_length_m);
        s.hazard_zones[0] = zone;
        s.hazard_zone_count = 1U;
    }

    return s;
}

/** @brief Whether one completed run satisfies every touchdown criterion. */
[[nodiscard]] bool IsRunSafe(const ApproachSimResult& result,
                             const LandingCriteria& criteria) noexcept {
    return result.touched_down &&
           (result.touchdown_vertical_speed_mps <=
            criteria.max_vertical_speed_mps) &&
           (result.touchdown_horizontal_speed_mps <=
            criteria.max_horizontal_speed_mps) &&
           (result.touchdown_tilt_rad <= criteria.max_tilt_rad) &&
           (result.touchdown_miss_m <= criteria.max_miss_distance_m) &&
           !result.landed_on_hazard;
}

/** @brief Accumulators for every summary metric, filled run by run. */
struct CampaignStats {
    RunningStats vertical_speed;
    RunningStats horizontal_speed;
    RunningStats tilt;
    RunningStats downrange;
    RunningStats miss;
    RunningStats flight_time;
    RunningStats propellant;
    RunningStats nav_altitude_error;
    RunningStats nav_velocity_error;
};

}  // namespace

Status RunMonteCarlo(const MonteCarloParams& params,
                     MonteCarloSummary* const summary_out,
                     MonteCarloLog* const log_out) noexcept {
    if ((summary_out == nullptr) || !AreParamsValid(params)) {
        return Status::kErrInvalidParam;
    }

    *summary_out = MonteCarloSummary{};
    summary_out->run_count = params.run_count;

    GaussianNoiseGenerator dispersion_gen;
    dispersion_gen.Seed(MixSeed(params.base_seed, 0xD15EA5EULL));

    CampaignStats stats{};
    bool any_failed = false;

    for (U32 run = 0U; run < params.run_count; ++run) { /* Bounded loop. */
        const ApproachScenarioParams scenario =
            BuildDispersedScenario(params, dispersion_gen, run);

        MonteCarloRunRecord record{};
        record.run_index = run;
        record.gate_altitude_m = scenario.gate.altitude_m;
        record.gate_velocity_x_mps = scenario.gate.velocity_x_mps;
        record.gate_velocity_z_mps = scenario.gate.velocity_z_mps;
        record.gate_pitch_rad = scenario.gate.pitch_rad;
        record.hazard_zone_present = (scenario.hazard_zone_count > 0U);
        if (record.hazard_zone_present) {
            ++summary_out->hazard_zone_count;
        }

        ApproachSimResult result{};
        const Status run_status = RunApproachSim(scenario, &result, nullptr);
        if (IsSuccess(run_status)) {
            record.touched_down = result.touched_down;
            record.touchdown_vertical_speed_mps =
                result.touchdown_vertical_speed_mps;
            record.touchdown_horizontal_speed_mps =
                result.touchdown_horizontal_speed_mps;
            record.touchdown_tilt_rad = result.touchdown_tilt_rad;
            record.touchdown_downrange_m = result.touchdown_downrange_m;
            record.touchdown_miss_m = result.touchdown_miss_m;
            record.flight_time_s = result.flight_time_s;
            record.propellant_used_kg = result.propellant_used_kg;
            record.nav_altitude_error_m = result.touchdown_nav_altitude_error_m;
            record.nav_velocity_error_mps =
                result.touchdown_nav_velocity_error_mps;
            record.controller_fault_count = result.controller_fault_count;
            record.nav_rejected_measurement_count =
                result.nav_rejected_measurement_count;
            record.hda_diverted = result.hda_diverted;
            record.hda_divert_distance_m = result.hda_divert_distance_m;
            record.landed_on_hazard = result.landed_on_hazard;
            record.hda_no_safe_site = result.hda_no_safe_site;
            if (result.hda_diverted) {
                ++summary_out->divert_count;
            }
            if (result.landed_on_hazard) {
                ++summary_out->hazard_landing_count;
            }
            if (result.hda_no_safe_site) {
                ++summary_out->no_safe_site_count;
            }
            record.safe = IsRunSafe(result, params.criteria);

            stats.vertical_speed.Add(result.touchdown_vertical_speed_mps);
            stats.horizontal_speed.Add(result.touchdown_horizontal_speed_mps);
            stats.tilt.Add(result.touchdown_tilt_rad);
            stats.downrange.Add(result.touchdown_downrange_m);
            stats.miss.Add(result.touchdown_miss_m);
            stats.flight_time.Add(result.flight_time_s);
            stats.propellant.Add(result.propellant_used_kg);
            stats.nav_altitude_error.Add(result.touchdown_nav_altitude_error_m);
            stats.nav_velocity_error.Add(
                result.touchdown_nav_velocity_error_mps);
            summary_out->total_controller_faults +=
                result.controller_fault_count;
            summary_out->total_nav_rejected_measurements +=
                result.nav_rejected_measurement_count;
        }
        /* else: scenario rejected (extreme user-supplied sigmas) — scored
         * as an unsafe run, excluded from the metric statistics.         */

        if (record.safe) {
            ++summary_out->safe_count;
        } else if (!any_failed) {
            any_failed = true;
            summary_out->first_failed_run = run;
        }

        if (log_out != nullptr) {
            log_out->Record(record);
        }
    }

    summary_out->vertical_speed_mps = stats.vertical_speed.Finalize();
    summary_out->horizontal_speed_mps = stats.horizontal_speed.Finalize();
    summary_out->tilt_rad = stats.tilt.Finalize();
    summary_out->downrange_m = stats.downrange.Finalize();
    summary_out->miss_m = stats.miss.Finalize();
    summary_out->flight_time_s = stats.flight_time.Finalize();
    summary_out->propellant_used_kg = stats.propellant.Finalize();
    summary_out->nav_altitude_error_m = stats.nav_altitude_error.Finalize();
    summary_out->nav_velocity_error_mps = stats.nav_velocity_error.Finalize();
    return Status::kSuccess;
}

}  // namespace sim
}  // namespace lls
