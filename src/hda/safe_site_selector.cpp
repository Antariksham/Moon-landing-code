/**
 * @file    safe_site_selector.cpp
 * @brief   Implementation of the HDA safe-site selection.
 *
 * @see     safe_site_selector.hpp for the decision rules and contracts.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#include "hda/safe_site_selector.hpp"

#include <cmath>

#include "lls/lls_assert.hpp"

namespace lls {
namespace hda {

namespace {

/**
 * @brief   Whether one station's terrain is inside the mission limits.
 * @param   sample  Surveyed station.
 * @param   config  Mission limits.
 * @return  `true` when slope and roughness are both acceptable.
 */
[[nodiscard]] bool IsSampleSafe(const SiteSample& sample,
                                const SafeSiteSelectorConfig& config) noexcept {
    return (sample.slope_deg <= config.max_slope_deg) &&
           (sample.roughness_m <= config.max_roughness_m);
}

/**
 * @brief   Whether a candidate station is a safe SITE (footprint check).
 *
 * @details Every surveyed station within the footprint radius must be
 *          individually safe, and the neighborhood must actually be
 *          surveyed on both sides — a footprint hanging off the survey
 *          edge is conservatively unsafe.
 *
 * @param   survey     Full survey (bounded scan).
 * @param   candidate  Index of the candidate station, < survey.count.
 * @param   config     Mission limits.
 * @return  `true` when the whole footprint is verified safe.
 */
[[nodiscard]] bool IsSiteSafe(const SiteSurvey& survey, const U32 candidate,
                              const SafeSiteSelectorConfig& config) noexcept {
    LLS_ASSERT(candidate < survey.count);
    const F32 center_m = survey.samples[candidate].downrange_m;
    const F32 radius_m = config.min_safe_site_radius_m;

    F32 min_offset_m = 0.0F; /* Most negative surveyed offset in radius.  */
    F32 max_offset_m = 0.0F; /* Most positive surveyed offset in radius.  */
    for (U32 i = 0U; i < survey.count; ++i) { /* Bounded loop. */
        const F32 offset_m = survey.samples[i].downrange_m - center_m;
        if (std::fabs(offset_m) <= radius_m) {
            if (!IsSampleSafe(survey.samples[i], config)) {
                return false;
            }
            min_offset_m = (offset_m < min_offset_m) ? offset_m : min_offset_m;
            max_offset_m = (offset_m > max_offset_m) ? offset_m : max_offset_m;
        }
    }
    /* Footprint coverage: the survey must extend to at least half the
     * footprint on each side (station-spacing tolerance of radius/2).    */
    const F32 required_reach_m = 0.5F * radius_m;
    return (-min_offset_m >= required_reach_m) &&
           (max_offset_m >= required_reach_m);
}

[[nodiscard]] bool IsConfigValid(const SafeSiteSelectorConfig& c) noexcept {
    const bool all_finite = std::isfinite(c.max_slope_deg) &&
                            std::isfinite(c.max_roughness_m) &&
                            std::isfinite(c.min_safe_site_radius_m) &&
                            std::isfinite(c.max_divert_distance_m);
    if (!all_finite) {
        return false;
    }
    return (c.max_slope_deg > 0.0F) && (c.max_roughness_m > 0.0F) &&
           (c.min_safe_site_radius_m > 0.0F) &&
           (c.max_divert_distance_m > 0.0F);
}

[[nodiscard]] bool IsSurveyValid(const SiteSurvey& survey) noexcept {
    if ((survey.count < 1U) || (survey.count > SiteSurvey::kMaxSiteSamples)) {
        return false;
    }
    return true;
}

[[nodiscard]] bool AreSamplesFinite(const SiteSurvey& survey) noexcept {
    for (U32 i = 0U; i < survey.count; ++i) { /* Bounded loop. */
        const SiteSample& s = survey.samples[i];
        if (!std::isfinite(s.downrange_m) || !std::isfinite(s.slope_deg) ||
            !std::isfinite(s.roughness_m) || (s.slope_deg < 0.0F) ||
            (s.roughness_m < 0.0F)) {
            return false;
        }
    }
    return true;
}

}  // namespace

Status SafeSiteSelector::Init(const SafeSiteSelectorConfig& config) noexcept {
    if (!IsConfigValid(config)) {
        is_initialized_ = false;
        return Status::kErrInvalidParam;
    }
    config_ = config;
    is_initialized_ = true;
    return Status::kSuccess;
}

Status SafeSiteSelector::SelectSite(
    const SiteSurvey& survey, const F32 nominal_target_m,
    SiteSelection* const selection_out) const noexcept {
    if (!is_initialized_) {
        return Status::kErrNotInitialized;
    }
    if (selection_out == nullptr) {
        LLS_ASSERT(selection_out != nullptr); /* Caller wiring error.      */
        return Status::kErrInvalidParam;
    }
    if (!IsSurveyValid(survey)) {
        return Status::kErrInvalidParam;
    }
    if (!AreSamplesFinite(survey) || !std::isfinite(nominal_target_m)) {
        return Status::kErrNonFiniteInput;
    }

    /* Nominal verdict: the station nearest the nominal target speaks for
     * it, and its whole footprint must check out. A nominal outside the
     * survey (nearest station farther than the footprint radius) is
     * conservatively unsafe — we cannot certify what we did not map.     */
    U32 nearest_index = 0U;
    F32 nearest_distance_m =
        std::fabs(survey.samples[0].downrange_m - nominal_target_m);
    for (U32 i = 1U; i < survey.count; ++i) { /* Bounded loop. */
        const F32 distance_m =
            std::fabs(survey.samples[i].downrange_m - nominal_target_m);
        if (distance_m < nearest_distance_m) {
            nearest_distance_m = distance_m;
            nearest_index = i;
        }
    }
    const bool nominal_covered =
        (nearest_distance_m <= config_.min_safe_site_radius_m);
    const bool nominal_safe =
        nominal_covered && IsSiteSafe(survey, nearest_index, config_);

    if (nominal_safe) {
        SiteSelection selection{};
        selection.target_downrange_m = nominal_target_m;
        selection.nominal_is_safe = true;
        selection.diverted = false;
        *selection_out = selection;
        return Status::kSuccess;
    }

    /* Divert search: the reachable safe site closest to the nominal;
     * ties break toward the lower downrange (deterministic).             */
    bool found = false;
    U32 best_index = 0U;
    F32 best_distance_m = 0.0F;
    for (U32 i = 0U; i < survey.count; ++i) { /* Bounded loop. */
        const F32 distance_m =
            std::fabs(survey.samples[i].downrange_m - nominal_target_m);
        if (distance_m > config_.max_divert_distance_m) {
            continue;
        }
        if (!IsSiteSafe(survey, i, config_)) {
            continue;
        }
        const bool better = !found || (distance_m < best_distance_m) ||
                            ((distance_m == best_distance_m) &&
                             (survey.samples[i].downrange_m <
                              survey.samples[best_index].downrange_m));
        if (better) {
            found = true;
            best_index = i;
            best_distance_m = distance_m;
        }
    }

    if (!found) {
        return Status::kErrNoSafeSite;
    }

    SiteSelection selection{};
    selection.target_downrange_m = survey.samples[best_index].downrange_m;
    selection.nominal_is_safe = false;
    selection.diverted = true;
    *selection_out = selection;
    return Status::kSuccess;
}

}  // namespace hda
}  // namespace lls
