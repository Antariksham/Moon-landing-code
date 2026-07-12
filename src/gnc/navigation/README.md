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

## Not yet implemented

- Accelerometer bias estimation (third state).
- Horizontal-channel estimation (terrain-relative navigation) and
  attitude estimation (star tracker / gyro fusion) — the full EKF
  formulation question is tracked in `docs/ARCHITECTURE.md` § Open design
  questions.
