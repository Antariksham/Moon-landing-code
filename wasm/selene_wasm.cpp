/**
 * @file    selene_wasm.cpp
 * @brief   Embind bridge exposing the SELENE flight software to the web.
 *
 * @details Compiled by Emscripten (see wasm/build.sh) into
 *          `selene_fsw.js` + `selene_fsw.wasm` and consumed by the
 *          CosmosDaily Next.js frontend. The bridge runs the exact
 *          closed-loop 3-DOF descent the SIL simulator flies —
 *          `lls::sim::RunApproachSim` with the real flight stack
 *          (`MissionStateMachine`, `DescentGuidance`, `PidController`,
 *          `ThrustAllocator`, `VerticalNavFilter`, `SafeSiteSelector`)
 *          in the loop — and records the 50 Hz telemetry log.
 *
 *          The frontend then pulls interpolated state vectors from the
 *          log at its own render rate (60 fps) via `getStateAtTime()`.
 *          All physics and GNC decisions are made by the C++ code; the
 *          JavaScript side only renders the coordinates it receives.
 *
 *          This translation unit is host/browser glue, not flight code:
 *          it is exempt from the strict flight flags (Embind requires
 *          RTTI), exactly like the sim executive in `sim/src/main.cpp`.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <emscripten/bind.h>

#include <algorithm>
#include <cmath>
#include <new>

#include "approach_sim.hpp"
#include "lls/lls_types.hpp"

namespace {

/** Scenario knobs the web UI may adjust (defaults mirror the CLI sim). */
struct WebScenarioConfig {
    double gateAltitudeM = 2000.0;
    double gateVelocityXMps = 60.0;
    double gateVelocityZMps = -30.0;
    double targetDownrangeM = 1200.0;
    bool perfectNav = false;
    bool hazardAtTarget = false;
};

/** One interpolated state vector handed to the renderer each frame. */
struct WebSimState {
    double timeS = 0.0;
    double altitudeM = 0.0;
    double downrangeM = 0.0;
    double velocityXMps = 0.0;
    double velocityZMps = 0.0;
    double velocityXCmdMps = 0.0;
    double velocityZCmdMps = 0.0;
    double pitchRad = 0.0;
    double pitchCmdRad = 0.0;
    double throttleFrac = 0.0;
    double torqueFrac = 0.0;
    double massKg = 0.0;
    double navAltitudeM = 0.0;
    double navVelocityZMps = 0.0;
    int missionPhase = 0;
    bool touchedDown = false;
};

/** End-of-run report for the verdict panel. */
struct WebSimResult {
    bool valid = false;
    bool touchedDown = false;
    bool safeLanding = false;
    double touchdownVerticalSpeedMps = 0.0;
    double touchdownHorizontalSpeedMps = 0.0;
    double touchdownTiltDeg = 0.0;
    double touchdownMissM = 0.0;
    double finalTargetDownrangeM = 0.0;
    bool hdaDiverted = false;
    double hdaDivertDistanceM = 0.0;
    bool hdaNoSafeSite = false;
    bool landedOnHazard = false;
    double flightTimeS = 0.0;
    double propellantUsedKg = 0.0;
    int finalPhase = 0;
    int controllerFaultCount = 0;
    double navAltitudeErrorM = 0.0;
    double navVelocityErrorMps = 0.0;
};

/* Mission limits mirrored from sim/src/main.cpp (the CLI verdict). */
constexpr double kTouchdownVelocityLimitMps = 2.0;
constexpr double kTouchdownHorizontalLimitMps = 1.0;
constexpr double kTouchdownTiltLimitDeg = 5.0;
constexpr double kTouchdownMissLimitM = 50.0;
constexpr double kRadToDeg = 57.29577951308232;

/* The telemetry log is megabytes; static storage, exactly like the CLI. */
lls::sim::ApproachTelemetryLog g_log;
lls::sim::ApproachSimResult g_result{};
bool g_has_run = false;
double g_dt_s = 0.02; /* 50 Hz control rate — set again on each run. */

/**
 * @brief Run one complete closed-loop descent with the flight stack.
 * @return true when the scenario was accepted and simulated.
 */
bool runScenario(const WebScenarioConfig& cfg) {
    lls::sim::ApproachScenarioParams params{};
    params.gate.altitude_m = cfg.gateAltitudeM;
    params.gate.velocity_x_mps = cfg.gateVelocityXMps;
    params.gate.velocity_z_mps = cfg.gateVelocityZMps;
    params.target_downrange_m = cfg.targetDownrangeM;
    params.nav.use_perfect_navigation = cfg.perfectNav;
    if (cfg.hazardAtTarget) {
        /* Demo scenario: a boulder field squarely on the nominal site. */
        lls::sim::HazardZone zone{};
        zone.start_m = cfg.targetDownrangeM - 40.0;
        zone.end_m = cfg.targetDownrangeM + 40.0;
        params.hazard_zones[0] = zone;
        params.hazard_zone_count = 1U;
    }

    /* Reset the multi-megabyte log in place: assigning a temporary would
     * materialize it on the (small) wasm stack. */
    g_log.~ApproachTelemetryLog();
    new (&g_log) lls::sim::ApproachTelemetryLog{};
    g_result = lls::sim::ApproachSimResult{};
    g_dt_s = 1.0 / params.control_rate_hz;

    const lls::Status status =
        lls::sim::RunApproachSim(params, &g_result, &g_log);
    g_has_run = lls::IsSuccess(status) && (g_log.GetCount() > 0U);
    return g_has_run;
}

/** @brief Run the default scenario (2 km gate, nominal target). */
bool runDefaultScenario() {
    return runScenario(WebScenarioConfig{});
}

/** @brief Total simulated flight time in seconds (0 before any run). */
double getFlightTimeS() {
    return g_has_run ? g_result.flight_time_s : 0.0;
}

/** @brief Number of 50 Hz telemetry samples recorded by the last run. */
int getSampleCount() {
    return g_has_run ? static_cast<int>(g_log.GetCount()) : 0;
}

/** @brief Copy one raw 50 Hz sample into the JS-facing state struct. */
WebSimState sampleToState(const lls::sim::ApproachTelemetrySample& s) {
    WebSimState out{};
    out.timeS = s.time_s;
    out.altitudeM = s.altitude_m;
    out.downrangeM = s.downrange_m;
    out.velocityXMps = s.velocity_x_mps;
    out.velocityZMps = s.velocity_z_mps;
    out.velocityXCmdMps = s.velocity_x_cmd_mps;
    out.velocityZCmdMps = s.velocity_z_cmd_mps;
    out.pitchRad = s.pitch_rad;
    out.pitchCmdRad = s.pitch_cmd_rad;
    out.throttleFrac = s.throttle_frac;
    out.torqueFrac = s.torque_frac;
    out.massKg = s.mass_kg;
    out.navAltitudeM = s.nav_altitude_m;
    out.navVelocityZMps = s.nav_velocity_z_mps;
    out.missionPhase = static_cast<int>(s.mission_phase);
    return out;
}

/**
 * @brief   State vector at mission time @p t_s, linearly interpolated
 *          between the two bracketing 50 Hz flight-software cycles.
 *
 * @details Called by the frontend once per rendered frame (60 fps).
 *          Times past touchdown clamp to the final sample with
 *          `touchedDown` set, so the lander settles on the surface.
 */
WebSimState getStateAtTime(const double t_s) {
    WebSimState out{};
    if (!g_has_run) {
        return out;
    }
    const lls::U32 count = g_log.GetCount();
    const double t = std::max(0.0, t_s);
    const double idx_f = t / g_dt_s;
    const lls::U32 i0 =
        std::min(static_cast<lls::U32>(idx_f), count - 1U);

    if (i0 >= (count - 1U)) {
        out = sampleToState(g_log.GetSample(count - 1U));
        out.touchedDown = g_result.touched_down;
        if (out.touchedDown) {
            /* Surface contact: pin the rendered state to the ground. */
            out.altitudeM = 0.0;
            out.velocityXMps = 0.0;
            out.velocityZMps = 0.0;
            out.throttleFrac = 0.0;
            out.missionPhase =
                static_cast<int>(g_result.final_phase);
        }
        return out;
    }

    const lls::sim::ApproachTelemetrySample& a = g_log.GetSample(i0);
    const lls::sim::ApproachTelemetrySample& b = g_log.GetSample(i0 + 1U);
    const double f =
        std::clamp(idx_f - static_cast<double>(i0), 0.0, 1.0);
    const auto lerp = [f](const double x, const double y) {
        return x + ((y - x) * f);
    };

    out = sampleToState(a);
    out.timeS = lerp(a.time_s, b.time_s);
    out.altitudeM = lerp(a.altitude_m, b.altitude_m);
    out.downrangeM = lerp(a.downrange_m, b.downrange_m);
    out.velocityXMps = lerp(a.velocity_x_mps, b.velocity_x_mps);
    out.velocityZMps = lerp(a.velocity_z_mps, b.velocity_z_mps);
    out.pitchRad = lerp(a.pitch_rad, b.pitch_rad);
    out.throttleFrac = lerp(a.throttle_frac, b.throttle_frac);
    out.massKg = lerp(a.mass_kg, b.mass_kg);
    out.navAltitudeM = lerp(a.nav_altitude_m, b.nav_altitude_m);
    out.navVelocityZMps = lerp(a.nav_velocity_z_mps, b.nav_velocity_z_mps);
    return out;
}

/** @brief End-of-run report with the same verdict logic as the CLI sim. */
WebSimResult getResult() {
    WebSimResult out{};
    if (!g_has_run) {
        return out;
    }
    const double tilt_deg = g_result.touchdown_tilt_rad * kRadToDeg;
    out.valid = true;
    out.touchedDown = g_result.touched_down;
    out.safeLanding =
        g_result.touched_down &&
        (g_result.touchdown_vertical_speed_mps <=
         kTouchdownVelocityLimitMps) &&
        (g_result.touchdown_horizontal_speed_mps <=
         kTouchdownHorizontalLimitMps) &&
        (tilt_deg <= kTouchdownTiltLimitDeg) &&
        (g_result.touchdown_miss_m <= kTouchdownMissLimitM) &&
        !g_result.landed_on_hazard;
    out.touchdownVerticalSpeedMps = g_result.touchdown_vertical_speed_mps;
    out.touchdownHorizontalSpeedMps =
        g_result.touchdown_horizontal_speed_mps;
    out.touchdownTiltDeg = tilt_deg;
    out.touchdownMissM = g_result.touchdown_miss_m;
    out.finalTargetDownrangeM = g_result.final_target_downrange_m;
    out.hdaDiverted = g_result.hda_diverted;
    out.hdaDivertDistanceM = g_result.hda_divert_distance_m;
    out.hdaNoSafeSite = g_result.hda_no_safe_site;
    out.landedOnHazard = g_result.landed_on_hazard;
    out.flightTimeS = g_result.flight_time_s;
    out.propellantUsedKg = g_result.propellant_used_kg;
    out.finalPhase = static_cast<int>(g_result.final_phase);
    out.controllerFaultCount =
        static_cast<int>(g_result.controller_fault_count);
    out.navAltitudeErrorM = g_result.touchdown_nav_altitude_error_m;
    out.navVelocityErrorMps = g_result.touchdown_nav_velocity_error_mps;
    return out;
}

}  // namespace

EMSCRIPTEN_BINDINGS(selene_fsw) {
    emscripten::value_object<WebScenarioConfig>("ScenarioConfig")
        .field("gateAltitudeM", &WebScenarioConfig::gateAltitudeM)
        .field("gateVelocityXMps", &WebScenarioConfig::gateVelocityXMps)
        .field("gateVelocityZMps", &WebScenarioConfig::gateVelocityZMps)
        .field("targetDownrangeM", &WebScenarioConfig::targetDownrangeM)
        .field("perfectNav", &WebScenarioConfig::perfectNav)
        .field("hazardAtTarget", &WebScenarioConfig::hazardAtTarget);

    emscripten::value_object<WebSimState>("SimState")
        .field("timeS", &WebSimState::timeS)
        .field("altitudeM", &WebSimState::altitudeM)
        .field("downrangeM", &WebSimState::downrangeM)
        .field("velocityXMps", &WebSimState::velocityXMps)
        .field("velocityZMps", &WebSimState::velocityZMps)
        .field("velocityXCmdMps", &WebSimState::velocityXCmdMps)
        .field("velocityZCmdMps", &WebSimState::velocityZCmdMps)
        .field("pitchRad", &WebSimState::pitchRad)
        .field("pitchCmdRad", &WebSimState::pitchCmdRad)
        .field("throttleFrac", &WebSimState::throttleFrac)
        .field("torqueFrac", &WebSimState::torqueFrac)
        .field("massKg", &WebSimState::massKg)
        .field("navAltitudeM", &WebSimState::navAltitudeM)
        .field("navVelocityZMps", &WebSimState::navVelocityZMps)
        .field("missionPhase", &WebSimState::missionPhase)
        .field("touchedDown", &WebSimState::touchedDown);

    emscripten::value_object<WebSimResult>("SimResult")
        .field("valid", &WebSimResult::valid)
        .field("touchedDown", &WebSimResult::touchedDown)
        .field("safeLanding", &WebSimResult::safeLanding)
        .field("touchdownVerticalSpeedMps",
               &WebSimResult::touchdownVerticalSpeedMps)
        .field("touchdownHorizontalSpeedMps",
               &WebSimResult::touchdownHorizontalSpeedMps)
        .field("touchdownTiltDeg", &WebSimResult::touchdownTiltDeg)
        .field("touchdownMissM", &WebSimResult::touchdownMissM)
        .field("finalTargetDownrangeM",
               &WebSimResult::finalTargetDownrangeM)
        .field("hdaDiverted", &WebSimResult::hdaDiverted)
        .field("hdaDivertDistanceM", &WebSimResult::hdaDivertDistanceM)
        .field("hdaNoSafeSite", &WebSimResult::hdaNoSafeSite)
        .field("landedOnHazard", &WebSimResult::landedOnHazard)
        .field("flightTimeS", &WebSimResult::flightTimeS)
        .field("propellantUsedKg", &WebSimResult::propellantUsedKg)
        .field("finalPhase", &WebSimResult::finalPhase)
        .field("controllerFaultCount",
               &WebSimResult::controllerFaultCount)
        .field("navAltitudeErrorM", &WebSimResult::navAltitudeErrorM)
        .field("navVelocityErrorMps", &WebSimResult::navVelocityErrorMps);

    emscripten::function("runScenario", &runScenario);
    emscripten::function("runDefaultScenario", &runDefaultScenario);
    emscripten::function("getStateAtTime", &getStateAtTime);
    emscripten::function("getResult", &getResult);
    emscripten::function("getFlightTimeS", &getFlightTimeS);
    emscripten::function("getSampleCount", &getSampleCount);
}
