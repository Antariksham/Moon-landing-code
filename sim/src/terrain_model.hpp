/**
 * @file    terrain_model.hpp
 * @brief   Along-track terrain hazard model for the SIL sim (milestone 6).
 *
 * @details Planar (1-D downrange) stand-in for the terrain mapper's world:
 *          a benign base surface (gentle slope, small roughness) with up to
 *          `kMaxHazardZones` rectangular hazard zones layered on top —
 *          crater walls, boulder fields — each raising slope and/or
 *          roughness over an interval. Where zones overlap, the worst
 *          value governs.
 *
 *          The model answers point queries (`GetSlopeDeg`, `GetRoughnessM`)
 *          that the sim uses both to build the HDA survey (perfect terrain
 *          sensing — LIDAR noise modeling is a follow-up) and to judge the
 *          actual touchdown point. Surface height is not modeled: hazard
 *          scoring is an overlay and the dynamics keep a flat surface at
 *          altitude zero.
 *
 *          Simulation code is host-only and exempt from the flight rules,
 *          but follows the flight style anyway.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_SIM_TERRAIN_MODEL_HPP
#define LLS_SIM_TERRAIN_MODEL_HPP

#include <array>

#include "lls/lls_types.hpp"

namespace lls {
namespace sim {

/** @brief One rectangular hazard interval layered on the base terrain. */
struct HazardZone {
    F64 start_m = 0.0;     /**< Zone start, downrange.                    */
    F64 end_m = 0.0;       /**< Zone end, > start.                        */
    F64 slope_deg = 20.0;  /**< Slope inside the zone, >= 0.              */
    F64 roughness_m = 0.6; /**< Roughness inside the zone, >= 0.          */
};

/** @brief Base (benign) surface properties. */
struct TerrainParams {
    F64 base_slope_deg = 2.0;    /**< Regolith background slope.          */
    F64 base_roughness_m = 0.05; /**< Regolith background roughness.      */
};

/**
 * @brief Deterministic along-track terrain: benign base + hazard zones.
 *
 * Lifecycle mirrors flight components: `Init()` once, `AddHazardZone()`
 * during scenario setup, then point queries during the run.
 */
class TerrainModel {
 public:
    /** Most hazard zones one scenario may define. */
    static constexpr U32 kMaxHazardZones = 8U;

    TerrainModel() noexcept = default;

    /**
     * @brief   Configure the base surface and clear all hazard zones.
     *
     * @param   params  Base properties; must be finite and non-negative.
     *
     * @retval  Status::kSuccess          Model ready (benign everywhere).
     * @retval  Status::kErrInvalidParam  A parameter is out of range.
     */
    [[nodiscard]] Status Init(const TerrainParams& params) noexcept;

    /**
     * @brief   Layer one hazard zone onto the surface.
     *
     * @param   zone  Interval and severity; must be finite, with
     *                `end_m > start_m` and non-negative severities.
     *
     * @retval  Status::kSuccess           Zone added.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam   Zone invalid or the fixed zone
     *                                     table is full.
     */
    [[nodiscard]] Status AddHazardZone(const HazardZone& zone) noexcept;

    /** @brief Slope at @p downrange_m: worst of base and covering zones. */
    [[nodiscard]] F64 GetSlopeDeg(F64 downrange_m) const noexcept;

    /** @brief Roughness at @p downrange_m: worst of base and covering
     *         zones. */
    [[nodiscard]] F64 GetRoughnessM(F64 downrange_m) const noexcept;

 private:
    TerrainParams params_{};
    std::array<HazardZone, kMaxHazardZones> zones_{};
    U32 zone_count_ = 0U;
    bool is_initialized_ = false;
};

}  // namespace sim
}  // namespace lls

#endif  // LLS_SIM_TERRAIN_MODEL_HPP
