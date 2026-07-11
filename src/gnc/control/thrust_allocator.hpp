/**
 * @file    thrust_allocator.hpp
 * @brief   Planar thrust-vector control allocation: acceleration command to
 *          pitch attitude + main-engine throttle.
 *
 * @details The velocity control loops produce a desired thrust acceleration
 *          vector (what the engine must add to the vehicle, gravity
 *          feedforward included). The lander realizes that vector with a
 *          single body-fixed main engine, so the command must be split into
 *          the two actuators that exist:
 *
 *            - a pitch attitude command for the attitude control loop
 *              (which points the thrust axis), and
 *            - a throttle command for the main engine (which sets the
 *              magnitude).
 *
 *          Geometry (planar, pitch about the out-of-plane axis):
 *
 *              thrust direction = (sin(pitch), cos(pitch))
 *              pitch = 0  -> thrust straight up
 *              pitch > 0  -> thrust tilted toward +x (accelerates downrange)
 *
 *          Allocation:
 *
 *              pitch_cmd    = atan2(ax_cmd, az_cmd)   clamped to
 *                             [-max_pitch, +max_pitch]
 *              throttle_cmd = m * |a_cmd| / T_max     clamped to [0, 1]
 *
 *          A clamped pitch or throttle is reported as `kErrSaturated`
 *          (informational, same convention as the PID): the command written
 *          is the closest realizable one, magnitude preserved.
 *
 *          A commanded acceleration with a non-positive vertical component
 *          cannot be realized by an up-pointing engine at a sane attitude;
 *          the vertical component is clamped to zero before allocation and
 *          the pitch limit then produces the closest safe command.
 *
 *          Compliance: no allocation, no exceptions, loop-free, F32
 *          throughout (commands are actuator-resolution quantities).
 *
 * @par Real-time characteristics
 *          `Allocate()` is allocation-free and loop-free; worst-case cost is
 *          one `atan2` and one `sqrt`. Runs in the 50 Hz control slot. Not
 *          thread-safe; single-task ownership is assumed.
 *
 * @copyright Copyright (c) 2026 Project SELENE Contributors.
 *            Licensed under the Apache License, Version 2.0.
 */

#ifndef LLS_GNC_CONTROL_THRUST_ALLOCATOR_HPP
#define LLS_GNC_CONTROL_THRUST_ALLOCATOR_HPP

#include "lls/lls_types.hpp"

namespace lls {
namespace gnc {

/**
 * @brief Configuration for a `ThrustAllocator` instance.
 *
 * Defaults mirror `config/landing_params.yaml`; validated by `Init()`.
 */
struct ThrustAllocatorConfig {
    F32 max_thrust_n = 2500.0F; /**< Engine thrust at throttle 1.0, > 0.    */
    F32 max_pitch_rad = 0.785F; /**< Attitude authority given to the
                                     translation loops; the vehicle never
                                     commands a tilt beyond this. Range:
                                     (0, pi/2).                             */
};

/** @brief Pitch + throttle realization of an acceleration command. */
struct ThrustCommand {
    F32 pitch_cmd_rad = 0.0F;     /**< Attitude reference for the pitch loop;
                                       0 = thrust straight up, positive tilts
                                       thrust toward +x.                        */
    F32 throttle_cmd_frac = 0.0F; /**< Main-engine throttle in [0, 1].      */
};

/**
 * @brief Deterministic planar control allocator for a single main engine.
 *
 * Lifecycle: construct (trivially) -> `Init()` during system initialization
 * -> `Allocate()` at the control rate. Stateless between calls; the object
 * holds only the validated configuration.
 */
class ThrustAllocator {
 public:
    /** @brief Trivial constructor; the object is unusable until `Init()`. */
    ThrustAllocator() noexcept = default;

    /* One allocator per engine: duplicating it would let two loops command
     * the same actuator. */
    ThrustAllocator(const ThrustAllocator&) = delete;
    ThrustAllocator& operator=(const ThrustAllocator&) = delete;
    ThrustAllocator(ThrustAllocator&&) = delete;
    ThrustAllocator& operator=(ThrustAllocator&&) = delete;

    ~ThrustAllocator() = default;

    /**
     * @brief   Validate the configuration and arm the allocator.
     *
     * @param   config  Candidate configuration. Accepted if and only if
     *                  every field is finite, `max_thrust_n > 0`, and
     *                  `max_pitch_rad` is in (0, pi/2).
     *
     * @retval  Status::kSuccess          Allocator armed.
     * @retval  Status::kErrInvalidParam  Configuration rejected; the
     *                                    allocator remains (or becomes)
     *                                    uninitialized and `Allocate()`
     *                                    will refuse to run.
     */
    [[nodiscard]] Status Init(const ThrustAllocatorConfig& config) noexcept;

    /**
     * @brief   Split an acceleration command into pitch + throttle.
     *
     * @details On any failure the value at @p cmd_out is left unmodified,
     *          so the caller's previous safe command is preserved by
     *          construction.
     *
     * @param   accel_cmd_x_mps2   Desired thrust acceleration along +x
     *                             (downrange), m/s^2. Must be finite.
     * @param   accel_cmd_up_mps2  Desired thrust acceleration along +z (up),
     *                             m/s^2, gravity feedforward included.
     *                             Must be finite; non-positive values are
     *                             clamped to zero before allocation.
     * @param   mass_kg            Current vehicle mass estimate, kg.
     *                             Valid range: (0, finite).
     * @param   cmd_out            Non-null pointer receiving the commands;
     *                             guaranteed within actuator limits when the
     *                             returned status is `kSuccess` or
     *                             `kErrSaturated`.
     *
     * @retval  Status::kSuccess           Command written; nothing clamped.
     * @retval  Status::kErrSaturated      Command written and valid, but the
     *                                     pitch and/or throttle limit was
     *                                     reached — informational for
     *                                     telemetry.
     * @retval  Status::kErrNotInitialized `Init()` has not succeeded.
     * @retval  Status::kErrInvalidParam   @p cmd_out is null or @p mass_kg
     *                                     is outside its valid range.
     * @retval  Status::kErrNonFiniteInput An acceleration input is NaN/Inf.
     */
    [[nodiscard]] Status Allocate(F32 accel_cmd_x_mps2, F32 accel_cmd_up_mps2,
                                  F32 mass_kg,
                                  ThrustCommand* cmd_out) const noexcept;

 private:
    ThrustAllocatorConfig config_{}; /**< Validated copy of the config.     */
    bool is_initialized_ = false;    /**< Set only by a successful Init().  */
};

}  // namespace gnc
}  // namespace lls

#endif  // LLS_GNC_CONTROL_THRUST_ALLOCATOR_HPP
