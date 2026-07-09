/**
 * @file    main.cpp
 * @brief   CLI entry point for the SELENE 1-DOF descent simulator.
 *
 * @details Runs the closed-loop terminal descent with the parameters from
 *          `config/landing_params.yaml` (compiled-in defaults; a YAML
 *          loader is tracked as a follow-up), prints a landing report,
 *          and optionally writes per-cycle telemetry as CSV.
 *
 *          Usage:
 *              selene_sim [--initial-altitude-m <v>]
 *                         [--initial-velocity-mps <v>]
 *                         [--telemetry <out.csv>]
 *
 *          Exit code 0 = safe landing (touchdown within the velocity
 *          limit); 1 = crash, timeout, or bad arguments. This makes the
 *          simulator directly usable as a CI gate.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "descent_sim.hpp"
#include "lls/lls_types.hpp"

namespace {

/** Touchdown velocity limit, m/s (config: touchdown_velocity_limit_mps). */
constexpr lls::F64 kTouchdownVelocityLimitMps = 2.0;

/** Telemetry buffer: file-scope static because it is ~1.4 MB (rule #1:
 *  fixed allocation; too large for the stack). */
lls::sim::TelemetryLog g_telemetry;

void PrintUsage(const char* prog) {
    std::fprintf(stderr,
                 "Usage: %s [--initial-altitude-m <v>] "
                 "[--initial-velocity-mps <v>] [--telemetry <out.csv>]\n",
                 prog);
}

[[nodiscard]] bool WriteTelemetryCsv(const char* path,
                                     const lls::sim::TelemetryLog& log) {
    std::FILE* const file = std::fopen(path, "w");
    if (file == nullptr) {
        return false;
    }
    std::fprintf(file,
                 "time_s,altitude_m,velocity_mps,velocity_cmd_mps,"
                 "throttle_frac,mass_kg\n");
    for (lls::U32 i = 0U; i < log.GetCount(); ++i) {  /* Bounded loop. */
        const lls::sim::TelemetrySample& s = log.GetSample(i);
        std::fprintf(file, "%.3f,%.3f,%.4f,%.4f,%.4f,%.3f\n", s.time_s,
                     s.altitude_m, s.velocity_mps, s.velocity_cmd_mps,
                     s.throttle_frac, s.mass_kg);
    }
    static_cast<void>(std::fclose(file));
    return true;
}

}  // namespace

int main(const int argc, const char** const argv) {
    lls::sim::ScenarioParams params{};
    const char* telemetry_path = nullptr;

    for (int i = 1; i < argc; ++i) {  /* Bounded by argc. */
        const bool has_value = ((i + 1) < argc);
        if ((std::strcmp(argv[i], "--initial-altitude-m") == 0) && has_value) {
            params.initial_altitude_m = std::atof(argv[i + 1]);
            ++i;
        } else if ((std::strcmp(argv[i], "--initial-velocity-mps") == 0) &&
                   has_value) {
            params.initial_velocity_mps = std::atof(argv[i + 1]);
            ++i;
        } else if ((std::strcmp(argv[i], "--telemetry") == 0) && has_value) {
            telemetry_path = argv[i + 1];
            ++i;
        } else {
            PrintUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    lls::sim::SimResult result{};
    const lls::Status status =
        lls::sim::RunDescentSim(params, &result, &g_telemetry);
    if (!lls::IsSuccess(status)) {
        std::fprintf(stderr, "Simulation rejected the scenario (status %u)\n",
                     static_cast<unsigned>(status));
        return EXIT_FAILURE;
    }

    if ((telemetry_path != nullptr) &&
        !WriteTelemetryCsv(telemetry_path, g_telemetry)) {
        std::fprintf(stderr, "Could not write telemetry to %s\n",
                     telemetry_path);
        return EXIT_FAILURE;
    }

    const bool safe = result.touched_down &&
                      (result.touchdown_speed_mps <=
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
    std::printf("Verdict:           %s\n", safe ? "SAFE LANDING" : "LOSS OF "
                                                                   "VEHICLE");

    return safe ? EXIT_SUCCESS : EXIT_FAILURE;
}
