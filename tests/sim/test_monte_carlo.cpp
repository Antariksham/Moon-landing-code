/**
 * @file    test_monte_carlo.cpp
 * @brief   Tests for the Monte-Carlo dispersion runner.
 *
 * @details Pins the campaign to reproducibility (same seed, same numbers —
 *          a Monte-Carlo failure must be a regression test, not a shrug),
 *          checks the degenerate zero-dispersion case against a direct
 *          single run, exercises the parameter validation and failure
 *          accounting, and flies a moderately sized campaign as the
 *          statistical acceptance test.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "monte_carlo.hpp"

namespace lls {
namespace sim {
namespace {

/** All dispersions off: every run is the identical nominal mission. */
[[nodiscard]] DispersionSigmas MakeZeroSigmas() {
    DispersionSigmas sigmas{};
    sigmas.gate_altitude_m = 0.0;
    sigmas.gate_velocity_x_mps = 0.0;
    sigmas.gate_velocity_z_mps = 0.0;
    sigmas.gate_pitch_rad = 0.0;
    sigmas.dry_mass_kg = 0.0;
    sigmas.propellant_mass_kg = 0.0;
    sigmas.max_thrust_n = 0.0;
    sigmas.specific_impulse_s = 0.0;
    sigmas.imu_accel_bias_mps2 = 0.0;
    sigmas.nav_altitude_error_m = 0.0;
    sigmas.nav_velocity_error_mps = 0.0;
    return sigmas;
}

/* ------------------------------------------------------------------ */
/* Validation                                                          */
/* ------------------------------------------------------------------ */

TEST(MonteCarloValidation, RejectsNullSummaryPointer) {
    const MonteCarloParams params{};
    EXPECT_EQ(RunMonteCarlo(params, nullptr, nullptr),
              Status::kErrInvalidParam);
}

TEST(MonteCarloValidation, RejectsRunCountOutOfRange) {
    MonteCarloParams params{};
    MonteCarloSummary summary{};

    params.run_count = 0U;
    EXPECT_EQ(RunMonteCarlo(params, &summary, nullptr),
              Status::kErrInvalidParam);

    params.run_count = kMaxMonteCarloRuns + 1U;
    EXPECT_EQ(RunMonteCarlo(params, &summary, nullptr),
              Status::kErrInvalidParam);
}

TEST(MonteCarloValidation, RejectsNonPhysicalSigmasAndCriteria) {
    MonteCarloParams params{};
    MonteCarloSummary summary{};

    params.sigmas.gate_altitude_m = -1.0;
    EXPECT_EQ(RunMonteCarlo(params, &summary, nullptr),
              Status::kErrInvalidParam);

    params = MonteCarloParams{};
    params.sigmas.imu_accel_bias_mps2 = std::numeric_limits<F64>::quiet_NaN();
    EXPECT_EQ(RunMonteCarlo(params, &summary, nullptr),
              Status::kErrInvalidParam);

    params = MonteCarloParams{};
    params.criteria.max_vertical_speed_mps = 0.0;
    EXPECT_EQ(RunMonteCarlo(params, &summary, nullptr),
              Status::kErrInvalidParam);
}

/* ------------------------------------------------------------------ */
/* Reproducibility                                                     */
/* ------------------------------------------------------------------ */

TEST(MonteCarloReproducibility, SameSeedSameCampaign) {
    MonteCarloParams params{};
    params.run_count = 25U;

    MonteCarloSummary a{};
    MonteCarloSummary b{};
    ASSERT_EQ(RunMonteCarlo(params, &a, nullptr), Status::kSuccess);
    ASSERT_EQ(RunMonteCarlo(params, &b, nullptr), Status::kSuccess);

    EXPECT_EQ(a.safe_count, b.safe_count);
    EXPECT_DOUBLE_EQ(a.vertical_speed_mps.mean, b.vertical_speed_mps.mean);
    EXPECT_DOUBLE_EQ(a.downrange_m.max, b.downrange_m.max);
    EXPECT_DOUBLE_EQ(a.propellant_used_kg.std_dev,
                     b.propellant_used_kg.std_dev);
}

TEST(MonteCarloReproducibility, DifferentSeedsDisperseDifferently) {
    MonteCarloParams params{};
    params.run_count = 25U;

    MonteCarloSummary a{};
    ASSERT_EQ(RunMonteCarlo(params, &a, nullptr), Status::kSuccess);

    params.base_seed = params.base_seed + 1U;
    MonteCarloSummary b{};
    ASSERT_EQ(RunMonteCarlo(params, &b, nullptr), Status::kSuccess);

    EXPECT_NE(a.downrange_m.mean, b.downrange_m.mean);
}

/* ------------------------------------------------------------------ */
/* Degenerate campaign: zero dispersion                                */
/* ------------------------------------------------------------------ */

TEST(MonteCarloDegenerate, ZeroSigmasReproduceTheSingleNominalRun) {
    /* With every dispersion off and perfect navigation (so the per-run
     * sensor seeds are moot), all runs are the same mission: the spread
     * of every metric must be exactly zero and the mean must match a
     * direct single run.                                                 */
    MonteCarloParams params{};
    params.run_count = 10U;
    params.sigmas = MakeZeroSigmas();
    params.nominal.nav.use_perfect_navigation = true;

    MonteCarloSummary summary{};
    ASSERT_EQ(RunMonteCarlo(params, &summary, nullptr), Status::kSuccess);
    EXPECT_EQ(summary.safe_count, 10U);
    EXPECT_DOUBLE_EQ(summary.vertical_speed_mps.std_dev, 0.0);
    EXPECT_DOUBLE_EQ(summary.downrange_m.std_dev, 0.0);
    EXPECT_DOUBLE_EQ(summary.propellant_used_kg.std_dev, 0.0);

    ApproachScenarioParams nominal{};
    nominal.nav.use_perfect_navigation = true;
    ApproachSimResult single{};
    ASSERT_EQ(RunApproachSim(nominal, &single, nullptr), Status::kSuccess);
    EXPECT_DOUBLE_EQ(summary.vertical_speed_mps.mean,
                     single.touchdown_vertical_speed_mps);
    EXPECT_DOUBLE_EQ(summary.downrange_m.mean, single.touchdown_downrange_m);
    EXPECT_DOUBLE_EQ(summary.flight_time_s.mean, single.flight_time_s);
}

/* ------------------------------------------------------------------ */
/* Per-run records                                                     */
/* ------------------------------------------------------------------ */

TEST(MonteCarloLogging, RecordsEveryRunWithDispersedGates) {
    MonteCarloParams params{};
    params.run_count = 30U;

    MonteCarloSummary summary{};
    static MonteCarloLog log; /* ~250 KB: static, not stack. */
    ASSERT_EQ(RunMonteCarlo(params, &summary, &log), Status::kSuccess);
    ASSERT_EQ(log.GetCount(), 30U);

    const F64 nominal_altitude = params.nominal.gate.altitude_m;
    const F64 bound = 3.0 * params.sigmas.gate_altitude_m;
    bool any_dispersed = false;
    for (U32 i = 0U; i < log.GetCount(); ++i) { /* Bounded loop. */
        const MonteCarloRunRecord& r = log.GetRecord(i);
        EXPECT_EQ(r.run_index, i);
        EXPECT_LE(std::fabs(r.gate_altitude_m - nominal_altitude),
                  bound + 1.0e-9)
            << "Draw escaped the 3-sigma truncation on run " << i;
        if (std::fabs(r.gate_altitude_m - nominal_altitude) > 1.0) {
            any_dispersed = true;
        }
    }
    EXPECT_TRUE(any_dispersed) << "Campaign never actually dispersed";
}

/* ------------------------------------------------------------------ */
/* Failure accounting                                                  */
/* ------------------------------------------------------------------ */

TEST(MonteCarloFailures, ImpossibleCriteriaFailEveryRun) {
    MonteCarloParams params{};
    params.run_count = 5U;
    params.criteria.max_vertical_speed_mps = 0.001; /* Unachievable. */

    MonteCarloSummary summary{};
    ASSERT_EQ(RunMonteCarlo(params, &summary, nullptr), Status::kSuccess);
    EXPECT_EQ(summary.safe_count, 0U);
    EXPECT_EQ(summary.first_failed_run, 0U);
    /* The runs themselves still flew and produced statistics.            */
    EXPECT_GT(summary.vertical_speed_mps.mean, 0.0);
}

TEST(MonteCarloFailures, RejectedScenariosAreScoredUnsafe) {
    /* A nominal the approach sim refuses (bad control rate) must not
     * abort the campaign: every run is scored unsafe instead.            */
    MonteCarloParams params{};
    params.run_count = 3U;
    params.nominal.control_rate_hz = 0.0;

    MonteCarloSummary summary{};
    static MonteCarloLog log;
    ASSERT_EQ(RunMonteCarlo(params, &summary, &log), Status::kSuccess);
    EXPECT_EQ(summary.safe_count, 0U);
    EXPECT_EQ(summary.run_count, 3U);
    EXPECT_FALSE(log.GetRecord(0U).touched_down);
}

/* ------------------------------------------------------------------ */
/* Statistical acceptance                                              */
/* ------------------------------------------------------------------ */

TEST(MonteCarloAcceptance, DefaultDispersionsAllLandSafely) {
    MonteCarloParams params{};
    params.run_count = 150U;

    MonteCarloSummary summary{};
    ASSERT_EQ(RunMonteCarlo(params, &summary, nullptr), Status::kSuccess);

    EXPECT_EQ(summary.safe_count, summary.run_count)
        << "First failed run: " << summary.first_failed_run
        << " (rerun with the default seed to reproduce)";
    EXPECT_EQ(summary.total_controller_faults, 0U);

    /* Margin audit: the campaign worst case must clear each limit with
     * real margin, not by luck.                                          */
    EXPECT_LT(summary.vertical_speed_mps.max, 1.9);
    EXPECT_LT(summary.horizontal_speed_mps.max, 0.5);
    EXPECT_LT(summary.tilt_rad.max, 0.02);
    EXPECT_LT(summary.nav_altitude_error_m.max, 0.5);
}

}  // namespace
}  // namespace sim
}  // namespace lls
