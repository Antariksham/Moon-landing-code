/**
 * @file    horizontal_nav_filter.hpp
 * @brief   Horizontal-channel navigation filter: 3-state Kalman filter
 *          fusing IMU acceleration with terrain-relative-navigation
 *          position fixes (milestone 7).
 *
 * @details Estimates downrange position and horizontal velocity — the two
 *          states the site-targeting guidance and the horizontal control
 *          loop consume — plus the accelerometer bias that would otherwise
 *          poison them:
 *
 *            - **Predict** (IMU, control rate): the caller supplies the
 *              measured horizontal specific force (in the planar descent
 *              model gravity has no horizontal component, so this equals
 *              the inertial horizontal acceleration plus sensor errors),
 *              *uncorrected*. The filter subtracts its own bias estimate
 *              and propagates state and covariance forward.
 *            - **Update** (TRN, sensor rate): a map-relative downrange
 *              position fix corrects the state through the standard Kalman
 *              gain. An innovation gate rejects fixes more than
 *              `innovation_gate_sigma` standard deviations from the
 *              prediction (`Status::kErrMeasurementRejected`) so one
 *              mis-registered map match cannot yank the estimate;
 *              rejections are counted for telemetry/FDIR.
 *
 *          Model: x = [downrange, velocity, accel_bias], constant-
 *          acceleration kinematics driven by (measured accel - bias); the
 *          bias is a random walk. **The bias state is the difference from
 *          the vertical filter** (which absorbs bias through frequent
 *          altimeter returns): TRN goes blind below its minimum-altitude
 *          floor, and the horizontal channel must then dead-reckon on the
 *          IMU alone all the way to touchdown. An unestimated bias of
 *          0.02 m/s^2 integrates to ~1 m/s of phantom velocity over a
 *          60 s terminal descent — enough to breach the 1 m/s horizontal
 *          touchdown limit as the control loop faithfully chases the
 *          phantom. Estimating the bias while TRN can still see, and
 *          holding that correction through the blind final descent, keeps
 *          the dead-reckoned estimate honest.
 *
 *          Precision note (rule: F64 requires documented need, see
 *          lls_types.hpp): state and covariance are propagated in F64.
 *          Covariance propagation is the canonical case where F32 mantissa
 *          loss produces asymmetric/indefinite matrices; the public API
 *          remains F32 like every other flight interface.
 *
 * @par Real-time characteristics
 *          `Predict()` and `UpdatePosition()` are allocation-free and
 *          loop-free (fixed 3x3 algebra). Intended for the 50 Hz
 *          navigation slot. Not thread-safe; single-task ownership is
 *          assumed.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_GNC_NAVIGATION_HORIZONTAL_NAV_FILTER_HPP
#define LLS_GNC_NAVIGATION_HORIZONTAL_NAV_FILTER_HPP

#include "lls/lls_types.hpp"

namespace lls {
namespace gnc {

/**
 * @brief Configuration of the horizontal navigation filter.
 *
 * Defaults mirror `config/landing_params.yaml`; validated by `Init()`.
 */
struct HorizontalNavFilterConfig {
    F32 initial_downrange_m = 0.0F;  /**< State at arm time. Finite (a
                                          map-relative coordinate is
                                          signed).                         */
    F32 initial_velocity_mps = 0.0F; /**< State at arm time. Finite.       */
    F32 initial_downrange_std_m = 10.0F; /**< 1-sigma initial position
                                              uncertainty, > 0.            */
    F32 initial_velocity_std_mps = 2.0F; /**< 1-sigma initial velocity
                                              uncertainty, > 0.            */
    F32 initial_bias_std_mps2 = 0.05F;   /**< 1-sigma initial accel-bias
                                              uncertainty (the bias state is
                                              seeded at zero), > 0.          */
    F32 accel_noise_std_mps2 = 0.1F;     /**< IMU accel white noise (process
                                              noise driver), > 0.              */
    F32 bias_walk_std_mps2 = 0.001F;     /**< Bias random-walk growth over one
                                              second, >= 0 (0 = constant-bias
                                              model).                          */
    F32 trn_noise_std_m = 5.0F;          /**< TRN position-fix 1-sigma noise
                                              (measurement noise), > 0.        */
    F32 innovation_gate_sigma = 5.0F;    /**< Reject fixes beyond this many
                                              innovation sigmas, > 0.         */
};

/** @brief Navigation solution (state + 1-sigma uncertainties). */
struct HorizontalNavEstimate {
    F32 downrange_m = 0.0F;         /**< Estimated downrange position.        */
    F32 velocity_mps = 0.0F;        /**< Estimated horizontal velocity.       */
    F32 accel_bias_mps2 = 0.0F;     /**< Estimated accelerometer bias.        */
    F32 downrange_std_m = 0.0F;     /**< 1-sigma position uncertainty.        */
    F32 velocity_std_mps = 0.0F;    /**< 1-sigma velocity uncertainty.        */
    F32 accel_bias_std_mps2 = 0.0F; /**< 1-sigma bias uncertainty.         */
};

/**
 * @brief Deterministic 3-state horizontal-channel Kalman filter.
 *
 * Lifecycle: construct (trivially) -> `Init()` at the navigation handover
 * (state seeded from the upstream orbit solution, bias seeded at zero) ->
 * `Predict()` every control cycle and `UpdatePosition()` whenever a TRN
 * fix arrives. When TRN goes blind near the surface the updates simply
 * stop and the filter dead-reckons on the bias-corrected IMU.
 */
class HorizontalNavFilter {
 public:
    /** @brief Trivial constructor; the object is unusable until `Init()`. */
    HorizontalNavFilter() noexcept = default;

    /* One navigation solution per vehicle: duplicating the filter would
     * let two divergent state estimates into the control loops. */
    HorizontalNavFilter(const HorizontalNavFilter&) = delete;
    HorizontalNavFilter& operator=(const HorizontalNavFilter&) = delete;
    HorizontalNavFilter(HorizontalNavFilter&&) = delete;
    HorizontalNavFilter& operator=(HorizontalNavFilter&&) = delete;

    ~HorizontalNavFilter() = default;

    /**
     * @brief   Validate the configuration and arm the filter.
     *
     * @param   config  Candidate configuration. Accepted if and only if
     *                  every field is finite, every standard deviation and
     *                  the gate are > 0, and the bias walk is >= 0.
     *
     * @retval  Status::kSuccess          Filter armed at the initial state.
     * @retval  Status::kErrInvalidParam  Configuration rejected; the filter
     *                                    remains (or becomes) uninitialized
     *                                    and all other methods refuse to
     *                                    run.
     */
    [[nodiscard]] Status Init(const HorizontalNavFilterConfig& config) noexcept;

    /**
     * @brief   Propagate the state by one control interval.
     *
     * @param   accel_meas_mps2  Measured horizontal specific force over the
     *                           interval, m/s^2, *uncorrected* — the filter
     *                           subtracts its own bias estimate. Must be
     *                           finite.
     * @param   dt_s             Interval length, s. Valid range:
     *                           (0, kMaxDtSeconds].
     *
     * @retval  Status::kSuccess           State and covariance propagated.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam   @p dt_s outside its valid range.
     * @retval  Status::kErrNonFiniteInput @p accel_meas_mps2 is NaN/Inf.
     */
    [[nodiscard]] Status Predict(F32 accel_meas_mps2, F32 dt_s) noexcept;

    /**
     * @brief   Fuse one TRN downrange-position fix.
     *
     * @param   downrange_meas_m  Map-relative downrange position, m. Must
     *                            be finite; any sign is valid (unlike a
     *                            radar range, a map coordinate can be
     *                            negative).
     *
     * @retval  Status::kSuccess                Fix fused.
     * @retval  Status::kErrMeasurementRejected Innovation gate exceeded;
     *                                          state untouched, rejection
     *                                          counted. Expected under map
     *                                          mis-registration — not a
     *                                          caller error.
     * @retval  Status::kErrNotInitialized      `Init()` has not succeeded.
     * @retval  Status::kErrNonFiniteInput      Fix is NaN or Inf.
     */
    [[nodiscard]] Status UpdatePosition(F32 downrange_meas_m) noexcept;

    /** @brief Current navigation solution (zeros when uninitialized). */
    [[nodiscard]] HorizontalNavEstimate GetEstimate() const noexcept;

    /**
     * @brief   Fixes rejected by the innovation gate since `Init()`.
     * @return  Monotonic counter (telemetry/FDIR point).
     */
    [[nodiscard]] U32 GetRejectedMeasurementCount() const noexcept;

    /** @brief Longest propagation interval accepted by `Predict()`, s. */
    static constexpr F32 kMaxDtSeconds = 0.5F;

 private:
    HorizontalNavFilterConfig config_{}; /**< Validated copy of the config. */

    /* State and covariance in F64 — see the precision note above. */
    F64 downrange_m_ = 0.0;  /**< State: downrange position estimate.      */
    F64 velocity_mps_ = 0.0; /**< State: horizontal velocity estimate.     */
    F64 bias_mps2_ = 0.0;    /**< State: accelerometer bias estimate.      */
    F64 p00_ = 0.0;          /**< Covariance: position variance.           */
    F64 p01_ = 0.0;          /**< Covariance: position-velocity (= p10).   */
    F64 p02_ = 0.0;          /**< Covariance: position-bias (= p20).       */
    F64 p11_ = 0.0;          /**< Covariance: velocity variance.           */
    F64 p12_ = 0.0;          /**< Covariance: velocity-bias (= p21).       */
    F64 p22_ = 0.0;          /**< Covariance: bias variance.               */

    U32 rejected_measurement_count_ = 0U; /**< Gate rejections since init.  */
    bool is_initialized_ = false; /**< Set only by a successful Init().     */
};

}  // namespace gnc
}  // namespace lls

#endif  // LLS_GNC_NAVIGATION_HORIZONTAL_NAV_FILTER_HPP
