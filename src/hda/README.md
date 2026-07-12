# Hazard Detection & Avoidance (HDA)

Terrain hazard scoring, safe-site selection, and divert recommendation.

## Implemented

- **`safe_site_selector.hpp`** — safe-landing-site selection (milestone
  6). Consumes a terrain survey (slope/roughness per downrange station)
  and decides: keep the nominal site when its whole landing footprint is
  verified safe; otherwise divert to the closest safe site within the
  divert envelope; report `Status::kErrNoSafeSite` when nothing
  acceptable exists and let the executive own the fallback. Unsurveyed
  terrain is conservatively unsafe.

  Exercised closed-loop by `sim/` (`selene_sim --mode 3dof
  --hazard-at-target` for a demo divert; the Monte-Carlo campaign places
  hazard zones randomly). Unit tested in
  `tests/hda/test_safe_site_selector.cpp`.

## Not yet implemented

- Real terrain mapping: LIDAR/camera sensor models, map registration
  noise, and slope/roughness estimation from raw returns (the sim
  currently surveys the truth terrain directly).
- Two-dimensional site maps (the sim is planar) and fuel-cost-aware site
  scoring.
