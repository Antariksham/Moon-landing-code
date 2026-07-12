/**
 * @file    terrain_model.cpp
 * @brief   Implementation of the along-track terrain hazard model.
 *
 * @see     terrain_model.hpp for the model description and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "terrain_model.hpp"

#include <cmath>

namespace lls {
namespace sim {

Status TerrainModel::Init(const TerrainParams& params) noexcept {
    const bool valid = std::isfinite(params.base_slope_deg) &&
                       std::isfinite(params.base_roughness_m) &&
                       (params.base_slope_deg >= 0.0) &&
                       (params.base_roughness_m >= 0.0);
    if (!valid) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }
    params_ = params;
    zone_count_ = 0U;
    is_initialized_ = true;
    return Status::kSuccess;
}

Status TerrainModel::AddHazardZone(const HazardZone& zone) noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    const bool valid =
        std::isfinite(zone.start_m) && std::isfinite(zone.end_m) &&
        std::isfinite(zone.slope_deg) && std::isfinite(zone.roughness_m) &&
        (zone.end_m > zone.start_m) && (zone.slope_deg >= 0.0) &&
        (zone.roughness_m >= 0.0);
    if (!valid || (zone_count_ >= kMaxHazardZones)) {
        return Status::kErrInvalidParam;
    }
    zones_[zone_count_] = zone;
    ++zone_count_;
    return Status::kSuccess;
}

F64 TerrainModel::GetSlopeDeg(const F64 downrange_m) const noexcept {
    F64 slope_deg = is_initialized_ ? params_.base_slope_deg : 0.0;
    for (U32 i = 0U; i < zone_count_; ++i) { /* Bounded loop. */
        const HazardZone& zone = zones_[i];
        if ((downrange_m >= zone.start_m) && (downrange_m <= zone.end_m) &&
            (zone.slope_deg > slope_deg)) {
            slope_deg = zone.slope_deg;
        }
    }
    return slope_deg;
}

F64 TerrainModel::GetRoughnessM(const F64 downrange_m) const noexcept {
    F64 roughness_m = is_initialized_ ? params_.base_roughness_m : 0.0;
    for (U32 i = 0U; i < zone_count_; ++i) { /* Bounded loop. */
        const HazardZone& zone = zones_[i];
        if ((downrange_m >= zone.start_m) && (downrange_m <= zone.end_m) &&
            (zone.roughness_m > roughness_m)) {
            roughness_m = zone.roughness_m;
        }
    }
    return roughness_m;
}

}  // namespace sim
}  // namespace lls
