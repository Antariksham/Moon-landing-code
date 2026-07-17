/**
 * @file    test_safe_site_selector.cpp
 * @brief   Unit tests for the HDA safe-site selection.
 *
 * @details Covers the `Init()` acceptance matrix, every `SelectSite()`
 *          failure path, and the decision rules: keep a safe nominal,
 *          divert to the nearest safe site, respect the divert envelope,
 *          enforce the footprint (radius + coverage) check, and stay
 *          deterministic on ties.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "hda/safe_site_selector.hpp"

namespace lls {
namespace hda {
namespace {

constexpr F32 kNaN = std::numeric_limits<F32>::quiet_NaN();

/** Uniform survey over [start, start + (count-1)*spacing], benign. */
[[nodiscard]] SiteSurvey MakeBenignSurvey(const F32 start_m,
                                          const F32 spacing_m,
                                          const U32 count) {
    SiteSurvey survey{};
    survey.count = count;
    for (U32 i = 0U; i < count; ++i) {
        survey.samples[i].downrange_m =
            start_m + (static_cast<F32>(i) * spacing_m);
        survey.samples[i].slope_deg = 2.0F;
        survey.samples[i].roughness_m = 0.05F;
    }
    return survey;
}

/** Mark every station in [start, end] hazardous (slope violation). */
void AddHazardInterval(SiteSurvey* const survey, const F32 start_m,
                       const F32 end_m) {
    for (U32 i = 0U; i < survey->count; ++i) {
        const F32 x = survey->samples[i].downrange_m;
        if ((x >= start_m) && (x <= end_m)) {
            survey->samples[i].slope_deg = 25.0F;
        }
    }
}

/** Standard test survey: 1000..1400 m at 4 m spacing (101 stations). */
[[nodiscard]] SiteSurvey MakeStandardSurvey() {
    return MakeBenignSurvey(1000.0F, 4.0F, 101U);
}

/* ------------------------------------------------------------------ */
/* Init() acceptance matrix                                            */
/* ------------------------------------------------------------------ */

TEST(SafeSiteSelectorInit, AcceptsDefaultConfig) {
    SafeSiteSelector selector;
    EXPECT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);
}

TEST(SafeSiteSelectorInit, RejectsNonFiniteAndNonPositiveLimits) {
    SafeSiteSelector selector;
    SafeSiteSelectorConfig cfg{};
    cfg.max_slope_deg = kNaN;
    EXPECT_EQ(selector.Init(cfg), Status::kErrInvalidParam);

    cfg = SafeSiteSelectorConfig{};
    cfg.max_roughness_m = 0.0F;
    EXPECT_EQ(selector.Init(cfg), Status::kErrInvalidParam);

    cfg = SafeSiteSelectorConfig{};
    cfg.min_safe_site_radius_m = -1.0F;
    EXPECT_EQ(selector.Init(cfg), Status::kErrInvalidParam);

    cfg = SafeSiteSelectorConfig{};
    cfg.max_divert_distance_m = 0.0F;
    EXPECT_EQ(selector.Init(cfg), Status::kErrInvalidParam);
}

TEST(SafeSiteSelectorInit, FailedReInitDisarmsTheSelector) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);

    SafeSiteSelectorConfig bad{};
    bad.max_slope_deg = -1.0F;
    ASSERT_EQ(selector.Init(bad), Status::kErrInvalidParam);

    const SiteSurvey survey = MakeStandardSurvey();
    SiteSelection selection{};
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrNotInitialized);
}

/* ------------------------------------------------------------------ */
/* SelectSite() failure paths                                          */
/* ------------------------------------------------------------------ */

TEST(SafeSiteSelectorSelect, RefusesBeforeInit) {
    const SafeSiteSelector selector;
    const SiteSurvey survey = MakeStandardSurvey();
    SiteSelection selection{};
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrNotInitialized);
}

TEST(SafeSiteSelectorSelect, RejectsNullOutputPointer) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);
    const SiteSurvey survey = MakeStandardSurvey();
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, nullptr),
              Status::kErrInvalidParam);
}

TEST(SafeSiteSelectorSelect, RejectsSurveyCountOutOfRange) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);

    SiteSurvey survey = MakeStandardSurvey();
    SiteSelection selection{};
    survey.count = 0U;
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrInvalidParam);
    survey.count = SiteSurvey::kMaxSiteSamples + 1U;
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrInvalidParam);
}

TEST(SafeSiteSelectorSelect, RejectsNonFiniteOrNegativeSamples) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);
    SiteSelection selection{};

    SiteSurvey survey = MakeStandardSurvey();
    survey.samples[3].slope_deg = kNaN;
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrNonFiniteInput);

    survey = MakeStandardSurvey();
    survey.samples[3].roughness_m = -0.1F;
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrNonFiniteInput);

    survey = MakeStandardSurvey();
    EXPECT_EQ(selector.SelectSite(survey, kNaN, &selection),
              Status::kErrNonFiniteInput);
}

TEST(SafeSiteSelectorSelect, FailureLeavesSelectionUntouched) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);

    SiteSelection selection{};
    selection.target_downrange_m = 777.0F;

    SiteSurvey survey = MakeStandardSurvey();
    survey.samples[0].slope_deg = kNaN;
    ASSERT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrNonFiniteInput);
    EXPECT_FLOAT_EQ(selection.target_downrange_m, 777.0F);
}

/* ------------------------------------------------------------------ */
/* Decision rules                                                      */
/* ------------------------------------------------------------------ */

TEST(SafeSiteSelectorDecision, KeepsASafeNominalExactly) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);

    const SiteSurvey survey = MakeStandardSurvey();
    SiteSelection selection{};
    /* Nominal off-station on purpose: the verdict comes from the nearest
     * station but the kept target is the nominal itself.                 */
    ASSERT_EQ(selector.SelectSite(survey, 1201.5F, &selection),
              Status::kSuccess);
    EXPECT_TRUE(selection.nominal_is_safe);
    EXPECT_FALSE(selection.diverted);
    EXPECT_FLOAT_EQ(selection.target_downrange_m, 1201.5F);
}

TEST(SafeSiteSelectorDecision, DivertsToNearestSafeSite) {
    SafeSiteSelector selector;
    const SafeSiteSelectorConfig cfg{};
    ASSERT_EQ(selector.Init(cfg), Status::kSuccess);

    SiteSurvey survey = MakeStandardSurvey();
    /* Hazard from 1160 to 1240: the nominal (1200) is inside. The nearest
     * safe SITE must also keep its whole footprint (radius 10 m) clear of
     * the hazard edge.                                                   */
    AddHazardInterval(&survey, 1160.0F, 1240.0F);
    SiteSelection selection{};
    ASSERT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kSuccess);
    EXPECT_FALSE(selection.nominal_is_safe);
    EXPECT_TRUE(selection.diverted);

    /* Candidates at 1148 (down) and 1252 (up) are the first whose whole
     * 10 m footprints clear the hazardous stations; both are 52 m from
     * the nominal — the tie breaks toward the lower downrange.           */
    EXPECT_FLOAT_EQ(selection.target_downrange_m, 1148.0F);
}

TEST(SafeSiteSelectorDecision, RespectsTheDivertEnvelope) {
    SafeSiteSelector selector;
    SafeSiteSelectorConfig cfg{};
    cfg.max_divert_distance_m = 30.0F; /* Tighter than the hazard span.   */
    ASSERT_EQ(selector.Init(cfg), Status::kSuccess);

    SiteSurvey survey = MakeStandardSurvey();
    AddHazardInterval(&survey, 1160.0F, 1240.0F);
    SiteSelection selection{};
    selection.target_downrange_m = 777.0F;
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrNoSafeSite);
    EXPECT_FLOAT_EQ(selection.target_downrange_m, 777.0F);
}

TEST(SafeSiteSelectorDecision, RejectsIsolatedSafeStation) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);

    /* Everything hazardous except one station: its footprint contains
     * hazardous neighbors, so it is not a safe SITE.                     */
    SiteSurvey survey = MakeStandardSurvey();
    AddHazardInterval(&survey, 0.0F, 5000.0F);
    survey.samples[50].slope_deg = 2.0F; /* Station at 1200 m.            */

    SiteSelection selection{};
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrNoSafeSite);
}

TEST(SafeSiteSelectorDecision, SingleStationSurveyCannotProveAFootprint) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);

    /* One benign station: no neighborhood coverage on either side, so
     * nothing can be certified.                                          */
    const SiteSurvey survey = MakeBenignSurvey(1200.0F, 4.0F, 1U);
    SiteSelection selection{};
    EXPECT_EQ(selector.SelectSite(survey, 1200.0F, &selection),
              Status::kErrNoSafeSite);
}

TEST(SafeSiteSelectorDecision, UnsurveyedNominalIsConservativelyUnsafe) {
    SafeSiteSelector selector;
    ASSERT_EQ(selector.Init(SafeSiteSelectorConfig{}), Status::kSuccess);

    /* Survey covers 1000-1400 m but the nominal is at 1500 m: the terrain
     * there was never mapped, so the selector must divert into the mapped
     * region even though nothing is known to be wrong at 1500.           */
    const SiteSurvey survey = MakeStandardSurvey();
    SiteSelection selection{};
    ASSERT_EQ(selector.SelectSite(survey, 1500.0F, &selection),
              Status::kSuccess);
    EXPECT_FALSE(selection.nominal_is_safe);
    EXPECT_TRUE(selection.diverted);
    /* Nearest certified station: the top of the surveyed span minus the
     * footprint-coverage margin (radius/2 = 5 m of surveyed reach needed
     * on the high side puts the highest certifiable station at 1392).    */
    EXPECT_FLOAT_EQ(selection.target_downrange_m, 1392.0F);
}

}  // namespace
}  // namespace hda
}  // namespace lls
