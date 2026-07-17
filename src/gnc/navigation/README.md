# Navigation

State estimation: IMU propagation, terrain-relative navigation, and EKF
sensor fusion.

## Implemented

- **`vertical_nav_filter.hpp`** — vertical-channel navigation filter
  (milestone 3). A 2-state (altitude, vertical velocity) Kalman filter:
  IMU-acceleration predict at the control rate, radar-altimeter update at
  the sensor rate, with an innovation gate that rejects glitched returns
  (`Status::kErrMeasurementRejected`, counted for telemetry/FDIR). State
  and covariance propagate in F64 per the documented-precision rule in
  `lls/lls_types.hpp`; the public API is F32.

  In the SIL loop (`selene_sim --mode 3dof`) the descent guidance, the
  vertical-rate control loop, and the engine-cutoff discrete all fly on
  this filter's estimate, fed by the noisy sensor models in
  `sim/src/sensor_models.hpp`. Unit tested in
  `tests/gnc/test_vertical_nav_filter.cpp`.

- **`horizontal_nav_filter.hpp`** — horizontal-channel navigation filter
  (milestone 7). A 3-state (downrange, ground speed, accelerometer bias)
  Kalman filter: IMU-acceleration predict at the control rate,
  terrain-relative-navigation position update at the sensor rate, same
  innovation-gate and F64-covariance doctrine as the vertical filter.

  The third state is the difference from the vertical channel: TRN goes
  blind below its minimum-altitude floor (100 m in the SIL model), so the
  filter must dead-reckon the whole terminal descent on the IMU alone.
  The accelerometer bias — learned while TRN can still see the ground —
  is what keeps that dead reckoning honest; unestimated, a 0.02 m/s²
  bias would integrate to ~1 m/s of phantom ground speed, most of the
  touchdown budget.

  In the SIL loop, guidance range-to-go (site targeting and divert
  execution) and the horizontal-velocity control loop fly on this
  filter's estimate. Unit tested in
  `tests/gnc/test_horizontal_nav_filter.cpp`.

## Not yet implemented

- Attitude estimation (star tracker / gyro fusion) — the attitude channel
  still reads truth in the SIL loop.
- Accelerometer bias state in the vertical filter (the horizontal filter
  demonstrates the formulation; the vertical channel's frequent altimeter
  updates make its bias less critical).
- The full EKF formulation question (error-state vs. total-state for the
  coupled channels) is tracked in `docs/ARCHITECTURE.md` § Open design
  questions.
