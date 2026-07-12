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
 *              selene_sim [--mode 1dof|3dof]
 *                         [--initial-altitude-m <v>]
 *                         [--initial-velocity-mps <v>]
 *                         [--initial-horizontal-mps <v>]   (3dof only)
 *                         [--telemetry <out.csv>]
 *
 *          Modes:
 *            - `1dof` (default): milestone 1 vertical terminal descent
 *              from the 500 m gate.
 *            - `3dof`: milestone 2 planar approach from the 2 km gate —
 *              pitch-over maneuver, attitude control, terminal descent.
 *
 *          Exit code 0 = safe landing (touchdown within the velocity and
 *          tilt limits); 1 = crash, timeout, or bad arguments. This makes
 *          the simulator directly usable as a CI gate.
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

namespace {

/** Touchdown vertical-speed limit, m/s (config:
 *  touchdown_velocity_limit_mps). */
constexpr lls::F64 kTouchdownVelocityLimitMps = 2.0;

/** Touchdown horizontal-speed limit, m/s (config:
 *  touchdown_horizontal_limit_mps). */
constexpr lls::F64 kTouchdownHorizontalLimitMps = 1.0;

/** Touchdown tilt limit, deg (config: touchdown_tilt_limit_deg). */
constexpr lls::F64 kTouchdownTiltLimitDeg = 5.0;

constexpr lls::F64 kRadToDeg = 57.29577951308232;

/** Telemetry buffers: file-scope static because they are megabytes each
 *  (rule #1: fixed allocation; too large for the stack). */
lls::sim::TelemetryLog g_telemetry_1dof;
lls::sim::ApproachTelemetryLog g_telemetry_3dof;

/** Parsed command line. */
struct CliOptions {
    bool mode_3dof = false;
    bool perfect_nav = false;
    bool altitude_set = false;
    bool velocity_set = false;
    bool horizontal_set = false;
    lls::F64 initial_altitude_m = 0.0;
    lls::F64 initial_velocity_mps = 0.0;
    lls::F64 initial_horizontal_mps = 0.0;
    const char* telemetry_path = nullptr;
};

void PrintUsage(const char* prog) {
    std::fprintf(stderr,
                 "Usage: %s [--mode 1dof|3dof] [--initial-altitude-m <v>] "
                 "[--initial-velocity-mps <v>] [--initial-horizontal-mps <v>] "
                 "[--perfect-nav] [--telemetry <out.csv>]\n",
                 prog);
}

[[nodiscard]] bool ParseCli(const int argc, const char** const argv,
                            CliOptions* const opts) {
    for (int i = 1; i < argc; ++i) { /* Bounded by argc. */
        const bool has_value = ((i + 1) < argc);
        if ((std::strcmp(argv[i], "--mode") == 0) && has_value) {
            if (std::strcmp(argv[i + 1], "3dof") == 0) {
                opts->mode_3dof = true;
            } else if (std::strcmp(argv[i + 1], "1dof") != 0) {
                return false;
            }
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
        (tilt_deg <= kTouchdownTiltLimitDeg);

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

}  // namespace

int main(const int argc, const char** const argv) {
    CliOptions opts{};
    if (!ParseCli(argc, argv, &opts)) {
        PrintUsage(argv[0]);
        return EXIT_FAILURE;
    }
    return opts.mode_3dof ? Run3Dof(opts) : Run1Dof(opts);
}
