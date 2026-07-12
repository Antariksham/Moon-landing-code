/**
 * @file    main.cpp
 * @brief   CLI entry point for the SELENE descent simulators.
 *
 * @details Runs a closed-loop descent with the parameters from
 *          `config/landing_params.yaml` (compiled-in defaults; a YAML
 *          loader is tracked as a follow-up), prints a landing report,
 *          and optionally writes per-cycle telemetry as CSV.
 *
 *          Usage:
 *              selene_sim [--mode 1dof|3dof|mc]
 *                         [--initial-altitude-m <v>]
 *                         [--initial-velocity-mps <v>]
 *                         [--initial-horizontal-mps <v>]   (3dof only)
 *                         [--perfect-nav]                  (3dof only)
 *                         [--runs <n>] [--seed <n>]        (mc only)
 *                         [--telemetry <out.csv>]
 *
 *          Modes:
 *            - `1dof` (default): milestone 1 vertical terminal descent
 *              from the 500 m gate.
 *            - `3dof`: milestones 2+3 planar approach from the 2 km gate —
 *              pitch-over maneuver, attitude control, terminal descent,
 *              navigation filter in the loop.
 *            - `mc`: milestone 4 Monte-Carlo campaign of dispersed 3-DOF
 *              descents with landing-footprint statistics; --telemetry
 *              writes the per-run results table.
 *
 *          Exit code 0 = safe landing (touchdown within the velocity and
 *          tilt limits; for `mc`: every run safe); 1 = crash, timeout, or
 *          bad arguments. This makes the simulator directly usable as a
 *          CI gate.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "approach_sim.hpp"
#include "descent_sim.hpp"
#include "lls/lls_types.hpp"
#include "monte_carlo.hpp"

namespace {

/** Touchdown vertical-speed limit, m/s (config:
 *  touchdown_velocity_limit_mps). */
constexpr lls::F64 kTouchdownVelocityLimitMps = 2.0;

/** Touchdown horizontal-speed limit, m/s (config:
 *  touchdown_horizontal_limit_mps). */
constexpr lls::F64 kTouchdownHorizontalLimitMps = 1.0;

/** Touchdown tilt limit, deg (config: touchdown_tilt_limit_deg). */
constexpr lls::F64 kTouchdownTiltLimitDeg = 5.0;

/** Landing-site miss limit, m (config: touchdown_miss_limit_m). */
constexpr lls::F64 kTouchdownMissLimitM = 50.0;

constexpr lls::F64 kRadToDeg = 57.29577951308232;

/** Telemetry buffers: file-scope static because they are megabytes each
 *  (rule #1: fixed allocation; too large for the stack). */
lls::sim::TelemetryLog g_telemetry_1dof;
lls::sim::ApproachTelemetryLog g_telemetry_3dof;
lls::sim::MonteCarloLog g_monte_carlo_log;

/** Simulation mode selected on the command line. */
enum class SimMode : lls::U8 {
    k1Dof = 0U,
    k3Dof = 1U,
    kMonteCarlo = 2U,
};

/** Parsed command line. */
struct CliOptions {
    SimMode mode = SimMode::k1Dof;
    bool perfect_nav = false;
    bool altitude_set = false;
    bool velocity_set = false;
    bool horizontal_set = false;
    bool target_set = false;
    lls::F64 initial_altitude_m = 0.0;
    lls::F64 initial_velocity_mps = 0.0;
    lls::F64 initial_horizontal_mps = 0.0;
    lls::F64 target_downrange_m = 0.0;
    lls::U32 mc_runs = 200U;
    lls::U64 mc_seed = 0x5E1E4E5EEDULL;
    const char* telemetry_path = nullptr;
};

void PrintUsage(const char* prog) {
    std::fprintf(stderr,
                 "Usage: %s [--mode 1dof|3dof|mc] [--initial-altitude-m <v>] "
                 "[--initial-velocity-mps <v>] [--initial-horizontal-mps <v>] "
                 "[--target-downrange-m <v>] [--perfect-nav] [--runs <n>] "
                 "[--seed <n>] [--telemetry <out.csv>]\n",
                 prog);
}

[[nodiscard]] bool ParseCli(const int argc, const char** const argv,
                            CliOptions* const opts) {
    for (int i = 1; i < argc; ++i) { /* Bounded by argc. */
        const bool has_value = ((i + 1) < argc);
        if ((std::strcmp(argv[i], "--mode") == 0) && has_value) {
            if (std::strcmp(argv[i + 1], "3dof") == 0) {
                opts->mode = SimMode::k3Dof;
            } else if (std::strcmp(argv[i + 1], "mc") == 0) {
                opts->mode = SimMode::kMonteCarlo;
            } else if (std::strcmp(argv[i + 1], "1dof") != 0) {
                return false;
            }
            ++i;
        } else if ((std::strcmp(argv[i], "--runs") == 0) && has_value) {
            const long runs = std::atol(argv[i + 1]);
            if ((runs < 1) ||
                (runs > static_cast<long>(lls::sim::kMaxMonteCarloRuns))) {
                return false;
            }
            opts->mc_runs = static_cast<lls::U32>(runs);
            ++i;
        } else if ((std::strcmp(argv[i], "--seed") == 0) && has_value) {
            opts->mc_seed = std::strtoull(argv[i + 1], nullptr, 10);
            ++i;
        } else if ((std::strcmp(argv[i], "--initial-altitude-m") == 0) &&
                   has_value) {
            opts->initial_altitude_m = std::atof(argv[i + 1]);
            opts->altitude_set = true;
            ++i;
        } else if ((std::strcmp(argv[i], "--initial-velocity-mps") == 0) &&
                   has_value) {
            opts->initial_velocity_mps = std::atof(argv[i + 1]);
            opts->velocity_set = true;
            ++i;
        } else if ((std::strcmp(argv[i], "--initial-horizontal-mps") == 0) &&
                   has_value) {
            opts->initial_horizontal_mps = std::atof(argv[i + 1]);
            opts->horizontal_set = true;
            ++i;
        } else if ((std::strcmp(argv[i], "--target-downrange-m") == 0) &&
                   has_value) {
            opts->target_downrange_m = std::atof(argv[i + 1]);
            opts->target_set = true;
            ++i;
        } else if (std::strcmp(argv[i], "--perfect-nav") == 0) {
            opts->perfect_nav = true;
        } else if ((std::strcmp(argv[i], "--telemetry") == 0) && has_value) {
            opts->telemetry_path = argv[i + 1];
            ++i;
        } else {
            return false;
        }
    }
    return true;
}

[[nodiscard]] bool WriteTelemetryCsv1Dof(const char* path,
                                         const lls::sim::TelemetryLog& log) {
    std::FILE* const file = std::fopen(path, "w");
    if (file == nullptr) {
        return false;
    }
    std::fprintf(file,
                 "time_s,altitude_m,velocity_mps,velocity_cmd_mps,"
                 "throttle_frac,mass_kg\n");
    for (lls::U32 i = 0U; i < log.GetCount(); ++i) { /* Bounded loop. */
        const lls::sim::TelemetrySample& s = log.GetSample(i);
        std::fprintf(file, "%.3f,%.3f,%.4f,%.4f,%.4f,%.3f\n", s.time_s,
                     s.altitude_m, s.velocity_mps, s.velocity_cmd_mps,
                     s.throttle_frac, s.mass_kg);
    }
    static_cast<void>(std::fclose(file));
    return true;
}

[[nodiscard]] bool WriteTelemetryCsv3Dof(
    const char* path, const lls::sim::ApproachTelemetryLog& log) {
    std::FILE* const file = std::fopen(path, "w");
    if (file == nullptr) {
        return false;
    }
    std::fprintf(file,
                 "time_s,downrange_m,altitude_m,velocity_x_mps,"
                 "velocity_z_mps,velocity_x_cmd_mps,velocity_z_cmd_mps,"
                 "pitch_rad,pitch_cmd_rad,throttle_frac,torque_frac,"
                 "mass_kg,nav_altitude_m,nav_velocity_z_mps,"
                 "mission_phase\n");
    for (lls::U32 i = 0U; i < log.GetCount(); ++i) { /* Bounded loop. */
        const lls::sim::ApproachTelemetrySample& s = log.GetSample(i);
        std::fprintf(file,
                     "%.3f,%.2f,%.3f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,"
                     "%.4f,%.3f,%.3f,%.4f,%u\n",
                     s.time_s, s.downrange_m, s.altitude_m, s.velocity_x_mps,
                     s.velocity_z_mps, s.velocity_x_cmd_mps,
                     s.velocity_z_cmd_mps, s.pitch_rad, s.pitch_cmd_rad,
                     s.throttle_frac, s.torque_frac, s.mass_kg,
                     s.nav_altitude_m, s.nav_velocity_z_mps,
                     static_cast<unsigned>(s.mission_phase));
    }
    static_cast<void>(std::fclose(file));
    return true;
}

[[nodiscard]] int Run1Dof(const CliOptions& opts) {
    lls::sim::ScenarioParams params{};
    if (opts.altitude_set) {
        params.initial_altitude_m = opts.initial_altitude_m;
    }
    if (opts.velocity_set) {
        params.initial_velocity_mps = opts.initial_velocity_mps;
    }

    lls::sim::SimResult result{};
    const lls::Status status =
        lls::sim::RunDescentSim(params, &result, &g_telemetry_1dof);
    if (!lls::IsSuccess(status)) {
        std::fprintf(stderr, "Simulation rejected the scenario (status %u)\n",
                     static_cast<unsigned>(status));
        return EXIT_FAILURE;
    }

    if ((opts.telemetry_path != nullptr) &&
        !WriteTelemetryCsv1Dof(opts.telemetry_path, g_telemetry_1dof)) {
        std::fprintf(stderr, "Could not write telemetry to %s\n",
                     opts.telemetry_path);
        return EXIT_FAILURE;
    }

    const bool safe = result.touched_down && (result.touchdown_speed_mps <=
                                              kTouchdownVelocityLimitMps);

    std::printf("=== SELENE 1-DOF descent report ===\n");
    std::printf("Gate:              %.1f m at %.1f m/s\n",
                params.initial_altitude_m, params.initial_velocity_mps);
    std::printf("Touchdown:         %s\n",
                result.touched_down ? "yes" : "NO (timeout)");
    std::printf("Touchdown speed:   %.2f m/s (limit %.1f)\n",
                result.touchdown_speed_mps, kTouchdownVelocityLimitMps);
    std::printf("Flight time:       %.1f s\n", result.flight_time_s);
    std::printf("Propellant used:   %.1f kg\n", result.propellant_used_kg);
    std::printf("Controller faults: %u\n",
                static_cast<unsigned>(result.controller_fault_count));
    std::printf("Verdict:           %s\n", safe ? "SAFE LANDING"
                                                : "LOSS OF "
                                                  "VEHICLE");

    return safe ? EXIT_SUCCESS : EXIT_FAILURE;
}

[[nodiscard]] int Run3Dof(const CliOptions& opts) {
    lls::sim::ApproachScenarioParams params{};
    if (opts.altitude_set) {
        params.gate.altitude_m = opts.initial_altitude_m;
    }
    if (opts.velocity_set) {
        params.gate.velocity_z_mps = opts.initial_velocity_mps;
    }
    if (opts.horizontal_set) {
        params.gate.velocity_x_mps = opts.initial_horizontal_mps;
    }
    if (opts.target_set) {
        params.target_downrange_m = opts.target_downrange_m;
    }
    params.nav.use_perfect_navigation = opts.perfect_nav;

    lls::sim::ApproachSimResult result{};
    const lls::Status status =
        lls::sim::RunApproachSim(params, &result, &g_telemetry_3dof);
    if (!lls::IsSuccess(status)) {
        std::fprintf(stderr, "Simulation rejected the scenario (status %u)\n",
                     static_cast<unsigned>(status));
        return EXIT_FAILURE;
    }

    if ((opts.telemetry_path != nullptr) &&
        !WriteTelemetryCsv3Dof(opts.telemetry_path, g_telemetry_3dof)) {
        std::fprintf(stderr, "Could not write telemetry to %s\n",
                     opts.telemetry_path);
        return EXIT_FAILURE;
    }

    const lls::F64 tilt_deg = result.touchdown_tilt_rad * kRadToDeg;
    const bool safe =
        result.touched_down &&
        (result.touchdown_vertical_speed_mps <= kTouchdownVelocityLimitMps) &&
        (result.touchdown_horizontal_speed_mps <=
         kTouchdownHorizontalLimitMps) &&
        (tilt_deg <= kTouchdownTiltLimitDeg) &&
        (result.touchdown_miss_m <= kTouchdownMissLimitM);

    std::printf("=== SELENE 3-DOF approach report ===\n");
    std::printf("Gate:              %.1f m, vx %.1f m/s, vz %.1f m/s\n",
                params.gate.altitude_m, params.gate.velocity_x_mps,
                params.gate.velocity_z_mps);
    std::printf("Touchdown:         %s\n",
                result.touched_down ? "yes" : "NO (timeout)");
    std::printf("Vertical speed:    %.2f m/s (limit %.1f)\n",
                result.touchdown_vertical_speed_mps,
                kTouchdownVelocityLimitMps);
    std::printf("Horizontal speed:  %.2f m/s (limit %.1f)\n",
                result.touchdown_horizontal_speed_mps,
                kTouchdownHorizontalLimitMps);
    std::printf("Tilt at contact:   %.2f deg (limit %.1f)\n", tilt_deg,
                kTouchdownTiltLimitDeg);
    std::printf("Landing miss:      %.2f m (target %.0f m, limit %.0f)\n",
                result.touchdown_miss_m, params.target_downrange_m,
                kTouchdownMissLimitM);
    std::printf("Flight time:       %.1f s\n", result.flight_time_s);
    std::printf("Propellant used:   %.1f kg\n", result.propellant_used_kg);
    std::printf("Controller faults: %u\n",
                static_cast<unsigned>(result.controller_fault_count));
    std::printf("Final phase:       %u (rejected transitions: %u)\n",
                static_cast<unsigned>(result.final_phase),
                static_cast<unsigned>(result.rejected_transition_count));
    if (opts.perfect_nav) {
        std::printf("Navigation:        perfect (sensors bypassed)\n");
    } else {
        std::printf(
            "Navigation:        filter in the loop; touchdown estimate "
            "error %.2f m / %.2f m/s, %u gated returns\n",
            result.touchdown_nav_altitude_error_m,
            result.touchdown_nav_velocity_error_mps,
            static_cast<unsigned>(result.nav_rejected_measurement_count));
    }
    std::printf("Verdict:           %s\n",
                safe ? "SAFE LANDING" : "LOSS OF VEHICLE");

    return safe ? EXIT_SUCCESS : EXIT_FAILURE;
}

[[nodiscard]] bool WriteMonteCarloCsv(const char* path,
                                      const lls::sim::MonteCarloLog& log) {
    std::FILE* const file = std::fopen(path, "w");
    if (file == nullptr) {
        return false;
    }
    std::fprintf(file,
                 "run,safe,gate_altitude_m,gate_velocity_x_mps,"
                 "gate_velocity_z_mps,gate_pitch_rad,touched_down,"
                 "vertical_speed_mps,horizontal_speed_mps,tilt_rad,"
                 "downrange_m,miss_m,flight_time_s,propellant_used_kg,"
                 "nav_altitude_error_m,nav_velocity_error_mps,"
                 "controller_faults,nav_rejected_measurements\n");
    for (lls::U32 i = 0U; i < log.GetCount(); ++i) { /* Bounded loop. */
        const lls::sim::MonteCarloRunRecord& r = log.GetRecord(i);
        std::fprintf(file,
                     "%u,%u,%.2f,%.3f,%.3f,%.4f,%u,%.4f,%.4f,%.5f,%.2f,"
                     "%.2f,%.2f,%.3f,%.4f,%.4f,%u,%u\n",
                     static_cast<unsigned>(r.run_index), r.safe ? 1U : 0U,
                     r.gate_altitude_m, r.gate_velocity_x_mps,
                     r.gate_velocity_z_mps, r.gate_pitch_rad,
                     r.touched_down ? 1U : 0U, r.touchdown_vertical_speed_mps,
                     r.touchdown_horizontal_speed_mps, r.touchdown_tilt_rad,
                     r.touchdown_downrange_m, r.touchdown_miss_m,
                     r.flight_time_s, r.propellant_used_kg,
                     r.nav_altitude_error_m, r.nav_velocity_error_mps,
                     static_cast<unsigned>(r.controller_fault_count),
                     static_cast<unsigned>(r.nav_rejected_measurement_count));
    }
    static_cast<void>(std::fclose(file));
    return true;
}

void PrintMetric(const char* label, const lls::sim::MetricStats& stats,
                 const char* unit) {
    std::printf("%-18s mean %8.3f  std %7.3f  min %8.3f  max %8.3f  %s\n",
                label, stats.mean, stats.std_dev, stats.min, stats.max, unit);
}

[[nodiscard]] int RunMonteCarlo(const CliOptions& opts) {
    lls::sim::MonteCarloParams params{};
    params.run_count = opts.mc_runs;
    params.base_seed = opts.mc_seed;
    if (opts.target_set) {
        params.nominal.target_downrange_m = opts.target_downrange_m;
    }

    lls::sim::MonteCarloSummary summary{};
    const lls::Status status =
        lls::sim::RunMonteCarlo(params, &summary, &g_monte_carlo_log);
    if (!lls::IsSuccess(status)) {
        std::fprintf(stderr, "Campaign rejected (status %u)\n",
                     static_cast<unsigned>(status));
        return EXIT_FAILURE;
    }

    if ((opts.telemetry_path != nullptr) &&
        !WriteMonteCarloCsv(opts.telemetry_path, g_monte_carlo_log)) {
        std::fprintf(stderr, "Could not write results to %s\n",
                     opts.telemetry_path);
        return EXIT_FAILURE;
    }

    const bool all_safe = (summary.safe_count == summary.run_count);
    const lls::F64 safe_pct =
        (100.0 * summary.safe_count) / static_cast<lls::F64>(summary.run_count);

    std::printf("=== SELENE Monte-Carlo dispersion report ===\n");
    std::printf("Runs:              %u (seed %llu)\n",
                static_cast<unsigned>(summary.run_count),
                static_cast<unsigned long long>(params.base_seed));
    std::printf("Safe landings:     %u/%u (%.1f%%)\n",
                static_cast<unsigned>(summary.safe_count),
                static_cast<unsigned>(summary.run_count), safe_pct);
    if (!all_safe) {
        std::printf(
            "First failed run:  %u (rerun with the same seed to "
            "reproduce)\n",
            static_cast<unsigned>(summary.first_failed_run));
    }
    PrintMetric("Vertical speed:", summary.vertical_speed_mps,
                "m/s (limit 2.0)");
    PrintMetric("Horizontal speed:", summary.horizontal_speed_mps,
                "m/s (limit 1.0)");
    lls::sim::MetricStats tilt_deg = summary.tilt_rad;
    tilt_deg.mean *= kRadToDeg;
    tilt_deg.std_dev *= kRadToDeg;
    tilt_deg.min *= kRadToDeg;
    tilt_deg.max *= kRadToDeg;
    PrintMetric("Tilt:", tilt_deg, "deg (limit 5.0)");
    PrintMetric("Downrange:", summary.downrange_m, "m");
    PrintMetric("Landing miss:", summary.miss_m, "m (limit 50)");
    PrintMetric("Flight time:", summary.flight_time_s, "s");
    PrintMetric("Propellant:", summary.propellant_used_kg, "kg");
    PrintMetric("Nav alt error:", summary.nav_altitude_error_m, "m");
    PrintMetric("Nav vel error:", summary.nav_velocity_error_mps, "m/s");
    std::printf("Controller faults: %u total; gated altimeter returns: %u\n",
                static_cast<unsigned>(summary.total_controller_faults),
                static_cast<unsigned>(summary.total_nav_rejected_measurements));
    std::printf("Verdict:           %s\n",
                all_safe ? "ALL RUNS SAFE" : "DISPERSION FAILURES");

    return all_safe ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace

int main(const int argc, const char** const argv) {
    CliOptions opts{};
    if (!ParseCli(argc, argv, &opts)) {
        PrintUsage(argv[0]);
        return EXIT_FAILURE;
    }
    switch (opts.mode) {
        case SimMode::k3Dof:
            return Run3Dof(opts);
        case SimMode::kMonteCarlo:
            return RunMonteCarlo(opts);
        case SimMode::k1Dof:
        default:
            return Run1Dof(opts);
    }
}
