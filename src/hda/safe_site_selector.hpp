/**
 * @file    safe_site_selector.hpp
 * @brief   Hazard detection & avoidance: safe-landing-site selection and
 *          divert recommendation (Phase I milestone 6).
 *
 * @details First flight HDA module of Project SELENE. During the approach
 *          phase the terrain mapper surveys the ground around the targeted
 *          landing site (slope and roughness per downrange station, planar
 *          for now — matching the 3-DOF sim). This module turns that
 *          survey into a decision:
 *
 *            1. A station is **individually safe** when its slope and
 *               roughness are inside the mission limits.
 *            2. A station is a **safe site** when every surveyed station
 *               within `min_safe_site_radius_m` of it is individually
 *               safe — the whole landing-gear footprint must fit, not
 *               just the center point. A site whose neighborhood is not
 *               fully surveyed is conservatively unsafe.
 *            3. If the nominal target is a safe site, keep it (a divert
 *               costs propellant and adds transient risk for no benefit).
 *            4. Otherwise recommend the safe site closest to the nominal
 *               target, provided it lies within `max_divert_distance_m`;
 *               ties break toward the lower downrange for determinism.
 *            5. If no reachable safe site exists, report
 *               `Status::kErrNoSafeSite` and leave the selection output
 *               untouched — the executive decides the fallback (land at
 *               nominal under fault protection).
 *
 *          Compliance: no allocation, no exceptions, fixed-capacity
 *          survey, bounded O(n^2) scan (n <= kMaxSiteSamples), F32
 *          throughout.
 *
 * @par Real-time characteristics
 *          `SelectSite()` is allocation-free with a worst case of
 *          kMaxSiteSamples^2 comparisons (~16k); it runs once per survey
 *          at the HDA decision gate, not in the 50 Hz loop. Not
 *          thread-safe; single-task ownership is assumed.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_HDA_SAFE_SITE_SELECTOR_HPP
#define LLS_HDA_SAFE_SITE_SELECTOR_HPP

#include <array>

#include "lls/lls_types.hpp"

namespace lls {
namespace hda {

/**
 * @brief Mission limits for site safety and divert reach.
 *
 * Defaults mirror `config/landing_params.yaml` (hazard_avoidance section);
 * validated by `Init()`.
 */
struct SafeSiteSelectorConfig {
    F32 max_slope_deg = 10.0F;          /**< Landing-gear tip-over limit.    */
    F32 max_roughness_m = 0.30F;        /**< Clearance under the deck.       */
    F32 min_safe_site_radius_m = 10.0F; /**< Footprint half-width that must
                                             be uniformly safe, > 0. Must
                                             budget for the landing
                                             dispersion: the vehicle lands
                                             where NAVIGATION thinks the
                                             site is, so the verified
                                             footprint has to absorb the
                                             touchdown position error
                                             (milestone 7 campaign: < 8 m
                                             worst case) — a 4 m footprint
                                             demonstrably put diverted
                                             landings onto adjacent
                                             hazards.                       */
    F32 max_divert_distance_m = 300.0F; /**< Guidance/propellant divert
                                             envelope, > 0.                */
};

/** @brief One surveyed terrain station. */
struct SiteSample {
    F32 downrange_m = 0.0F; /**< Station position along +x.               */
    F32 slope_deg = 0.0F;   /**< Local slope magnitude.                   */
    F32 roughness_m = 0.0F; /**< Local roughness (rock/crater) height.    */
};

/** @brief Fixed-capacity terrain survey handed over by the mapper. */
struct SiteSurvey {
    /** Largest survey the selector accepts (600 m at 4 m spacing, plus
     *  margin). Station spacing must not exceed
     *  `min_safe_site_radius_m`, or no candidate can ever prove its
     *  footprint surveyed.                                               */
    static constexpr U32 kMaxSiteSamples = 256U;

    std::array<SiteSample, kMaxSiteSamples> samples{}; /**< Stations.      */
    U32 count = 0U; /**< Valid entries, in [1, kMaxSiteSamples].           */
};

/** @brief Outcome of a site-selection decision. */
struct SiteSelection {
    F32 target_downrange_m = 0.0F; /**< Site to fly to (== nominal when no
                                        divert is needed).                 */
    bool nominal_is_safe = false;  /**< Survey verdict on the nominal.     */
    bool diverted = false;         /**< True when the target moved.        */
};

/**
 * @brief Deterministic safe-site selection from a terrain survey.
 *
 * Lifecycle: construct (trivially) -> `Init()` during system
 * initialization -> `SelectSite()` at the HDA decision gate during
 * approach.
 */
class SafeSiteSelector {
 public:
    /** @brief Trivial constructor; the object is unusable until `Init()`. */
    SafeSiteSelector() noexcept = default;

    /* One landing-site authority per vehicle: duplicating it would allow
     * two divergent targets into guidance. */
    SafeSiteSelector(const SafeSiteSelector&) = delete;
    SafeSiteSelector& operator=(const SafeSiteSelector&) = delete;
    SafeSiteSelector(SafeSiteSelector&&) = delete;
    SafeSiteSelector& operator=(SafeSiteSelector&&) = delete;

    ~SafeSiteSelector() = default;

    /**
     * @brief   Validate the mission limits and arm the selector.
     *
     * @param   config  Candidate limits. Accepted if and only if every
     *                  field is finite and positive.
     *
     * @retval  Status::kSuccess          Selector armed.
     * @retval  Status::kErrInvalidParam  Limits rejected; the selector
     *                                    remains (or becomes)
     *                                    uninitialized and `SelectSite()`
     *                                    will refuse to run.
     */
    [[nodiscard]] Status Init(const SafeSiteSelectorConfig& config) noexcept;

    /**
     * @brief   Choose the landing site from a terrain survey.
     *
     * @details On any failure — including `kErrNoSafeSite` — the value at
     *          @p selection_out is left unmodified, so the caller's
     *          current target is preserved by construction.
     *
     * @param   survey            Terrain stations. `count` must be in
     *                            [1, kMaxSiteSamples]; every used sample
     *                            must be finite with non-negative slope
     *                            and roughness. Stations need not be
     *                            sorted.
     * @param   nominal_target_m  The currently targeted site, m. Must be
     *                            finite.
     * @param   selection_out     Non-null pointer receiving the decision.
     *
     * @retval  Status::kSuccess           Selection written (kept nominal
     *                                     or diverted; see the output).
     * @retval  Status::kErrNoSafeSite     No reachable safe site in the
     *                                     survey; output untouched.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam   @p selection_out is null or the
     *                                     survey count is out of range.
     * @retval  Status::kErrNonFiniteInput A used sample field or
     *                                     @p nominal_target_m is NaN/Inf,
     *                                     or a slope/roughness is
     *                                     negative.
     */
    [[nodiscard]] Status SelectSite(
        const SiteSurvey& survey, F32 nominal_target_m,
        SiteSelection* selection_out) const noexcept;

 private:
    SafeSiteSelectorConfig config_{}; /**< Validated copy of the limits.   */
    bool is_initialized_ = false;     /**< Set only by successful Init().  */
};

}  // namespace hda
}  // namespace lls

#endif  // LLS_HDA_SAFE_SITE_SELECTOR_HPP
