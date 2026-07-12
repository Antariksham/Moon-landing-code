/**
 * @file    test_terrain_model.cpp
 * @brief   Unit tests for the along-track terrain hazard model.
 *
 * @details Pins the base surface, zone layering (worst value governs,
 *          inclusive boundaries), the fixed zone capacity, and the
 *          validation paths.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <limits>

#include "terrain_model.hpp"

namespace lls {
namespace sim {
namespace {

TEST(TerrainModel, RejectsNonPhysicalBase) {
    TerrainModel terrain;
    TerrainParams bad{};
    bad.base_slope_deg = -1.0;
    EXPECT_EQ(terrain.Init(bad), Status::kErrInvalidParam);

    bad = TerrainParams{};
    bad.base_roughness_m = std::numeric_limits<F64>::quiet_NaN();
    EXPECT_EQ(terrain.Init(bad), Status::kErrInvalidParam);
}

TEST(TerrainModel, RefusesZonesBeforeInit) {
    TerrainModel terrain;
    EXPECT_EQ(terrain.AddHazardZone(HazardZone{0.0, 10.0, 20.0, 0.6}),
              Status::kErrNotInitialized);
}

TEST(TerrainModel, ReturnsBaseValuesEverywhereWhenBenign) {
    TerrainModel terrain;
    const TerrainParams params{};
    ASSERT_EQ(terrain.Init(params), Status::kSuccess);
    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(0.0), params.base_slope_deg);
    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(12345.0), params.base_slope_deg);
    EXPECT_DOUBLE_EQ(terrain.GetRoughnessM(500.0), params.base_roughness_m);
}

TEST(TerrainModel, ZoneGovernsInsideInclusiveBoundaries) {
    TerrainModel terrain;
    ASSERT_EQ(terrain.Init(TerrainParams{}), Status::kSuccess);
    ASSERT_EQ(terrain.AddHazardZone(HazardZone{100.0, 200.0, 20.0, 0.6}),
              Status::kSuccess);

    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(100.0), 20.0); /* Inclusive.     */
    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(150.0), 20.0);
    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(200.0), 20.0); /* Inclusive.     */
    EXPECT_DOUBLE_EQ(terrain.GetRoughnessM(150.0), 0.6);

    const TerrainParams params{};
    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(99.9), params.base_slope_deg);
    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(200.1), params.base_slope_deg);
}

TEST(TerrainModel, WorstValueGovernsWhereZonesOverlap) {
    TerrainModel terrain;
    ASSERT_EQ(terrain.Init(TerrainParams{}), Status::kSuccess);
    ASSERT_EQ(terrain.AddHazardZone(HazardZone{100.0, 200.0, 15.0, 0.4}),
              Status::kSuccess);
    ASSERT_EQ(terrain.AddHazardZone(HazardZone{150.0, 250.0, 25.0, 0.2}),
              Status::kSuccess);

    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(175.0), 25.0);
    EXPECT_DOUBLE_EQ(terrain.GetRoughnessM(175.0), 0.4);
}

TEST(TerrainModel, MildZoneCannotImproveOnTheBase) {
    TerrainModel terrain;
    const TerrainParams params{};
    ASSERT_EQ(terrain.Init(params), Status::kSuccess);
    /* A "zone" smoother than the base: queries keep the base values.     */
    ASSERT_EQ(terrain.AddHazardZone(HazardZone{100.0, 200.0, 0.5, 0.01}),
              Status::kSuccess);
    EXPECT_DOUBLE_EQ(terrain.GetSlopeDeg(150.0), params.base_slope_deg);
    EXPECT_DOUBLE_EQ(terrain.GetRoughnessM(150.0), params.base_roughness_m);
}

TEST(TerrainModel, RejectsInvalidZonesAndEnforcesCapacity) {
    TerrainModel terrain;
    ASSERT_EQ(terrain.Init(TerrainParams{}), Status::kSuccess);

    EXPECT_EQ(terrain.AddHazardZone(HazardZone{200.0, 100.0, 20.0, 0.6}),
              Status::kErrInvalidParam); /* end <= start */
    EXPECT_EQ(terrain.AddHazardZone(HazardZone{0.0, 10.0, -1.0, 0.6}),
              Status::kErrInvalidParam); /* negative slope */

    for (U32 i = 0U; i < TerrainModel::kMaxHazardZones;
         ++i) { /* Bounded loop. */
        const F64 start = static_cast<F64>(i) * 100.0;
        ASSERT_EQ(
            terrain.AddHazardZone(HazardZone{start, start + 50.0, 20.0, 0.6}),
            Status::kSuccess);
    }
    EXPECT_EQ(terrain.AddHazardZone(HazardZone{9000.0, 9100.0, 20.0, 0.6}),
              Status::kErrInvalidParam); /* Table full. */
}

}  // namespace
}  // namespace sim
}  // namespace lls
