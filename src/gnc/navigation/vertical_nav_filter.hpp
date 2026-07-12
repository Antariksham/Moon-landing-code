/**
 * @file    vertical_nav_filter.hpp
 * @brief   Vertical-channel navigation filter: 2-state Kalman filter fusing
 *          IMU acceleration with radar-altimeter fixes (milestone 3).
 *
 * @details First flight navigation module of Project SELENE. Estimates
 *          altitude and vertical velocity — the two states the descent
 *          guidance and the vertical control loop consume — from the two
 *          sensors every lander carries:
 *
 *            - **Predict** (IMU, control rate): the caller supplies the
 *              vehicle's inertial vertical acceleration (vertical specific
 *              force from the accelerometers minus lunar gravity) and the
 *              filter propagates state and covariance forward.
 *            - **Update** (radar altimeter, sensor rate): a direct altitude
 *              measurement corrects the state through the standard Kalman
 *              gain. An innovation gate rejects measurements more than
 *              `innovation_gate_sigma` standard deviations from the
 *              prediction (`Status::kErrMeasurementRejected`) so a single
 *              corrupted return cannot yank the estimate; rejections are
 *              counted for telemetry/FDIR.
 *
 *          Model: x = [altitude, velocity], constant-acceleration kinematics
 *          driven by the measured acceleration; process noise from the
 *          accelerometer white-noise density, measurement noise from the
 *          altimeter specification. Unmodeled accelerometer bias is absorbed
 *          by the altimeter updates (bias estimation is a follow-up).
 *
 *          Precision note (rule: F64 requires documented need, see
 *          lls_types.hpp): state and covariance are propagated in F64.
 *          Covariance propagation is the canonical case where F32 mantissa
 *          loss produces asymmetric/indefinite matrices; the public API
 *          remains F32 like every other flight interface.
 *
 * @par Real-time characteristics
 *          `Predict()` and `UpdateAltitude()` are allocation-free and
 *          loop-free (fixed 2x2 algebra). Intended for the 50 Hz navigation
 *          slot. Not thread-safe; single-task ownership is assumed.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_GNC_NAVIGATION_VERTICAL_NAV_FILTER_HPP
#define LLS_GNC_NAVIGATION_VERTICAL_NAV_FILTER_HPP

#include "lls/lls_types.hpp"

namespace lls {
namespace gnc {

/**
 * @brief Configuration of the vertical navigation filter.
 *
 * Defaults mirror `config/landing_params.yaml`; validated by `Init()`.
 */
struct VerticalNavFilterConfig {
    F32 initial_altitude_m = 0.0F;      /**< State at arm time. Range:
                                             [0, finite).                     */
    F32 initial_velocity_mps = 0.0F;    /**< State at arm time. Finite.       */
    F32 initial_altitude_std_m = 10.0F; /**< 1-sigma initial altitude
                                             uncertainty, > 0.             */
    F32 initial_velocity_std_mps = 2.0F; /**< 1-sigma initial velocity
                                              uncertainty, > 0.             */
    F32 accel_noise_std_mps2 = 0.1F;     /**< IMU accel white noise (process
                                              noise driver), > 0.               */
    F32 altimeter_noise_std_m = 0.2F;    /**< Radar altimeter 1-sigma noise
                                              (measurement noise), > 0.        */
    F32 innovation_gate_sigma = 5.0F;    /**< Reject measurements beyond this
                                              many innovation sigmas, > 0.     */
};

/** @brief Navigation solution (state + 1-sigma uncertainties). */
struct VerticalNavEstimate {
    F32 altitude_m = 0.0F;       /**< Estimated height above the site.     */
    F32 velocity_mps = 0.0F;     /**< Estimated vertical velocity.         */
    F32 altitude_std_m = 0.0F;   /**< 1-sigma altitude uncertainty.        */
    F32 velocity_std_mps = 0.0F; /**< 1-sigma velocity uncertainty.        */
};

/**
 * @brief Deterministic 2-state vertical-channel Kalman filter.
 *
 * Lifecycle: construct (trivially) -> `Init()` at the navigation handover
 * (state seeded from the upstream orbit solution) -> `Predict()` every
 * control cycle and `UpdateAltitude()` whenever an altimeter return
 * arrives.
 */
class VerticalNavFilter {
 public:
    /** @brief Trivial constructor; the object is unusable until `Init()`. */
    VerticalNavFilter() noexcept = default;

    /* One navigation solution per vehicle: duplicating the filter would
     * let two divergent state estimates into the control loops. */
    VerticalNavFilter(const VerticalNavFilter&) = delete;
    VerticalNavFilter& operator=(const VerticalNavFilter&) = delete;
    VerticalNavFilter(VerticalNavFilter&&) = delete;
    VerticalNavFilter& operator=(VerticalNavFilter&&) = delete;

    ~VerticalNavFilter() = default;

    /**
     * @brief   Validate the configuration and arm the filter.
     *
     * @param   config  Candidate configuration. Accepted if and only if
     *                  every field is finite, `initial_altitude_m >= 0`,
     *                  and every standard deviation and the gate are > 0.
     *
     * @retval  Status::kSuccess          Filter armed at the initial state.
     * @retval  Status::kErrInvalidParam  Configuration rejected; the filter
     *                                    remains (or becomes) uninitialized
     *                                    and all other methods refuse to
     *                                    run.
     */
    [[nodiscard]] Status Init(const VerticalNavFilterConfig& config) noexcept;

    /**
     * @brief   Propagate the state by one control interval.
     *
     * @param   vertical_accel_mps2  Inertial vertical acceleration over the
     *                               interval (vertical specific force minus
     *                               gravity), m/s^2. Must be finite.
     * @param   dt_s                 Interval length, s. Valid range:
     *                               (0, kMaxDtSeconds].
     *
     * @retval  Status::kSuccess           State and covariance propagated.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam   @p dt_s outside its valid range.
     * @retval  Status::kErrNonFiniteInput @p vertical_accel_mps2 is NaN/Inf.
     */
    [[nodiscard]] Status Predict(F32 vertical_accel_mps2, F32 dt_s) noexcept;

    /**
     * @brief   Fuse one radar-altimeter altitude measurement.
     *
     * @param   altitude_meas_m  Measured height above the site, m. Valid
     *                           range: [0, finite) — a nadir radar cannot
     *                           return a negative range.
     *
     * @retval  Status::kSuccess                Measurement fused.
     * @retval  Status::kErrMeasurementRejected Innovation gate exceeded;
     *                                          state untouched, rejection
     *                                          counted. Expected under
     *                                          sensor glitches — not a
     *                                          caller error.
     * @retval  Status::kErrNotInitialized      `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam        Measurement is negative.
     * @retval  Status::kErrNonFiniteInput      Measurement is NaN or Inf.
     */
    [[nodiscard]] Status UpdateAltitude(F32 altitude_meas_m) noexcept;

    /** @brief Current navigation solution (zeros when uninitialized). */
    [[nodiscard]] VerticalNavEstimate GetEstimate() const noexcept;

    /**
     * @brief   Measurements rejected by the innovation gate since `Init()`.
     * @return  Monotonic counter (telemetry/FDIR point).
     */
    [[nodiscard]] U32 GetRejectedMeasurementCount() const noexcept;

    /** @brief Longest propagation interval accepted by `Predict()`, s. */
    static constexpr F32 kMaxDtSeconds = 0.5F;

 private:
    VerticalNavFilterConfig config_{}; /**< Validated copy of the config.   */

    /* State and covariance in F64 — see the precision note above. */
    F64 altitude_m_ = 0.0;   /**< State: altitude estimate.                 */
    F64 velocity_mps_ = 0.0; /**< State: vertical velocity estimate.        */
    F64 p00_ = 0.0;          /**< Covariance: altitude variance.            */
    F64 p01_ = 0.0;          /**< Covariance: altitude-velocity (= p10).    */
    F64 p11_ = 0.0;          /**< Covariance: velocity variance.            */

    U32 rejected_measurement_count_ = 0U; /**< Gate rejections since init.  */
    bool is_initialized_ = false; /**< Set only by a successful Init().     */
};

}  // namespace gnc
}  // namespace lls

#endif  // LLS_GNC_NAVIGATION_VERTICAL_NAV_FILTER_HPP
