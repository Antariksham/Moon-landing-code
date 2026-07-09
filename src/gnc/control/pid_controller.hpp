/**
 * @file    pid_controller.hpp
 * @brief   Flight-grade PID controller with anti-windup and output limiting.
 *
 * @details Reference implementation and **coding gold standard** for Project
 *          SELENE. This controller closes the loop between a guidance
 *          reference (e.g. commanded descent rate) and the navigation
 *          estimate (e.g. measured descent rate), producing a bounded
 *          actuator command (e.g. engine throttle fraction).
 *
 *          Safety and determinism properties, per the SELENE coding rules:
 *            - **No dynamic allocation** — all state lives inside the object,
 *              which contributors must allocate statically (rule #1).
 *            - **No exceptions** — every fallible operation returns a
 *              `Status` that the caller must check (rule #2).
 *            - **Bounded execution** — `Update()` contains no loops; its
 *              worst-case execution time is a small constant (rule #3).
 *            - **Input validation** — NaN/Inf inputs and out-of-range
 *              parameters are rejected, never propagated into the command
 *              path (rule #4).
 *            - **Derivative-on-measurement** — the D term acts on the
 *              measurement, not the error, so a step change in setpoint
 *              (e.g. a guidance re-target during divert) cannot produce a
 *              derivative kick into the actuators.
 *            - **Conditional-integration anti-windup** — the integrator is
 *              frozen while the output is saturated in the same direction,
 *              preventing windup during long throttle-limited braking burns.
 *
 * @par Real-time characteristics
 *          `Update()` is lock-free, allocation-free, loop-free, and intended
 *          to be called from a single control task at a fixed rate
 *          (nominally 50 Hz). The object is **not** thread-safe; concurrent
 *          access must be excluded by the task architecture.
 *
 * @par Usage
 * @code
 *          namespace {
 *          lls::gnc::PidController g_throttle_ctrl;  // static allocation
 *          }
 *
 *          lls::gnc::PidConfig cfg{};
 *          cfg.kp = 0.8F;   cfg.ki = 0.15F;  cfg.kd = 0.05F;
 *          cfg.output_min = 0.0F;            // engine cannot push
 *          cfg.output_max = 1.0F;            // full throttle
 *          cfg.integrator_min = -0.2F;
 *          cfg.integrator_max =  0.2F;
 *
 *          if (!lls::IsSuccess(g_throttle_ctrl.Init(cfg))) {
 *              // configuration rejected — raise FDIR fault, stay in STANDBY
 *          }
 *
 *          // In the 50 Hz control task:
 *          lls::F32 throttle_cmd = 0.0F;
 *          const lls::Status status = g_throttle_ctrl.Update(
 *              descent_rate_cmd_mps, descent_rate_est_mps, 0.02F,
 *              &throttle_cmd);
 *          if (lls::IsSuccess(status)) {
 *              // command throttle_cmd to the engine HAL
 *          } else {
 *              // hold previous safe command; report fault to FDIR
 *          }
 * @endcode
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_GNC_CONTROL_PID_CONTROLLER_HPP
#define LLS_GNC_CONTROL_PID_CONTROLLER_HPP

#include "lls/lls_types.hpp"

namespace lls {
namespace gnc {

/**
 * @brief Compile-time configuration for a `PidController` instance.
 *
 * All members are plain `F32` values validated by `PidController::Init()`;
 * see that function's contract for the exact acceptance criteria.
 */
struct PidConfig {
    F32 kp = 0.0F;  /**< Proportional gain. Range: [0, +finite). Unitless.   */
    F32 ki = 0.0F;  /**< Integral gain. Range: [0, +finite). Per second.     */
    F32 kd = 0.0F;  /**< Derivative gain. Range: [0, +finite). Seconds.      */

    F32 output_min = 0.0F;  /**< Lower actuator limit. Must be < output_max. */
    F32 output_max = 0.0F;  /**< Upper actuator limit. Must be > output_min. */

    F32 integrator_min = 0.0F;  /**< Integrator floor. Must be <= 0 and
                                     < integrator_max (windup clamp).        */
    F32 integrator_max = 0.0F;  /**< Integrator ceiling. Must be >= 0 and
                                     > integrator_min (windup clamp).        */
};

/**
 * @brief Discrete PID controller with clamped integrator and bounded output.
 *
 * Lifecycle: construct (trivially) → `Init()` once during system
 * initialization → `Update()` at a fixed rate → `Reset()` on mode/phase
 * change. `Init()` is the only place configuration enters the object;
 * gains cannot be changed mid-flight except through a new `Init()` issued
 * by the mission state machine while the control loop is inhibited.
 */
class PidController {
 public:
    /** @brief Trivial constructor; the object is unusable until `Init()`. */
    PidController() noexcept = default;

    /* A controller is bound to one actuator: copying or moving it would
     * duplicate integrator state and violate single-owner semantics. */
    PidController(const PidController&) = delete;
    PidController& operator=(const PidController&) = delete;
    PidController(PidController&&) = delete;
    PidController& operator=(PidController&&) = delete;

    ~PidController() = default;

    /**
     * @brief   Validate the configuration and arm the controller.
     *
     * @details Must be called during system initialization, before the
     *          real-time control loop starts (rule #1: all set-up happens
     *          at init time). May be called again later — e.g. on a
     *          guidance-commanded gain-schedule change — but only while the
     *          control loop is inhibited; re-initialization resets all
     *          dynamic state.
     *
     * @param   config  Candidate configuration. Accepted if and only if:
     *                  every field is finite; `kp`, `ki`, `kd` >= 0;
     *                  `output_min < output_max`; and
     *                  `integrator_min < integrator_max` with
     *                  `integrator_min <= 0 <= integrator_max`.
     *
     * @retval  Status::kSuccess          Controller is armed and reset.
     * @retval  Status::kErrInvalidParam  Configuration rejected; the
     *                                    controller remains (or becomes)
     *                                    uninitialized and `Update()` will
     *                                    refuse to run.
     */
    [[nodiscard]] Status Init(const PidConfig& config) noexcept;

    /**
     * @brief   Execute one fixed-rate control cycle.
     *
     * @details Computes the PID law with derivative-on-measurement and
     *          conditional-integration anti-windup, then clamps the result
     *          to `[output_min, output_max]`. On any failure the value at
     *          @p command_out is **left unmodified**, so the caller's
     *          previous safe command is preserved by construction.
     *
     * @param   setpoint     Commanded value from guidance. Units: caller's
     *                       (e.g. m/s). Must be finite.
     * @param   measurement  Estimated value from navigation. Same units as
     *                       @p setpoint. Must be finite.
     * @param   dt_s         Time since previous `Update()` in seconds.
     *                       Valid range: (0, kMaxDtSeconds]. The upper bound
     *                       rejects resumption after a task overrun, where a
     *                       huge integral step would be destabilizing.
     * @param   command_out  Non-null pointer receiving the actuator command,
     *                       guaranteed within `[output_min, output_max]`
     *                       when the returned status is `kSuccess` or
     *                       `kErrSaturated`.
     *
     * @retval  Status::kSuccess           Command written; output unsaturated.
     * @retval  Status::kErrSaturated      Command written and valid, but
     *                                     clipped at an output limit —
     *                                     informational for telemetry.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam   @p command_out is null or @p dt_s
     *                                     is outside its valid range.
     * @retval  Status::kErrNonFiniteInput @p setpoint or @p measurement is
     *                                     NaN or Inf.
     */
    [[nodiscard]] Status Update(F32 setpoint,
                                F32 measurement,
                                F32 dt_s,
                                F32* command_out) noexcept;

    /**
     * @brief   Clear all dynamic state (integrator, derivative history).
     *
     * @details Called by the mission state machine on every phase
     *          transition so that stale integrator charge from the previous
     *          phase cannot corrupt the first cycles of the next one.
     *          Configuration and initialization state are preserved.
     */
    void Reset() noexcept;

    /**
     * @brief   Current integrator state, for telemetry and unit tests.
     * @return  Integrator value in output units; 0 when uninitialized.
     */
    [[nodiscard]] F32 GetIntegratorState() const noexcept;

    /** @brief Longest control interval accepted by `Update()`, seconds. */
    static constexpr F32 kMaxDtSeconds = 0.5F;

 private:
    PidConfig config_{};                /**< Validated copy of the config.   */
    F32  integrator_ = 0.0F;            /**< Accumulated I-term, out. units. */
    F32  prev_measurement_ = 0.0F;      /**< Measurement at previous cycle.  */
    bool has_prev_measurement_ = false; /**< First-cycle guard for D-term.   */
    bool is_initialized_ = false;       /**< Set only by a successful Init().*/
};

}  // namespace gnc
}  // namespace lls

#endif  // LLS_GNC_CONTROL_PID_CONTROLLER_HPP
