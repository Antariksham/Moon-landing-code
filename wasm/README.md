# SELENE → WebAssembly bridge

This directory compiles the SELENE flight software into a WebAssembly module
consumed by the CosmosDaily website's lunar-landing simulator page
(`antariksham/Antariksham`, `/lunar-sim`).

## What it does

`selene_wasm.cpp` is an [Embind](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
glue layer around the **exact same closed-loop 3-DOF simulation the CLI and CI
fly** (`lls::sim::RunApproachSim`), with the real flight stack in the loop:

- `fsw::MissionStateMachine` — mission phase sequencing
- `gnc::DescentGuidance` — altitude-keyed velocity references + discretes
- `gnc::PidController` ×3 — horizontal / vertical / attitude loops
- `gnc::ThrustAllocator` — acceleration command → pitch + throttle
- `gnc::VerticalNavFilter` — altitude/velocity estimation from noisy sensors
- `hda::SafeSiteSelector` — hazard detection & divert decision

A scenario runs to touchdown inside the wasm module (milliseconds), recording
the 50 Hz telemetry log. The frontend then pulls **interpolated state vectors
at its render rate (60 fps)** via `getStateAtTime(t)` — all physics and GNC
decisions come from C++; JavaScript only renders the coordinates it receives.

## JS API (module factory: `createSeleneModule`)

| Function | Purpose |
|---|---|
| `runDefaultScenario(): boolean` | Fly the nominal 2 km gate scenario. |
| `runScenario(cfg): boolean` | Fly a custom scenario (`gateAltitudeM`, `gateVelocityXMps`, `gateVelocityZMps`, `targetDownrangeM`, `perfectNav`, `hazardAtTarget`). |
| `getStateAtTime(tS): SimState` | Interpolated state vector at mission time `tS` (altitude, velocities, pitch, throttle, mass, nav estimates, mission phase, touchdown flag). |
| `getResult(): SimResult` | End-of-run report with the same SAFE LANDING verdict logic as the CLI. |
| `getFlightTimeS(): number` | Total simulated flight time. |
| `getSampleCount(): number` | Recorded 50 Hz samples. |

## Building locally

```sh
git clone https://github.com/emscripten-core/emsdk.git
./emsdk/emsdk install 6.0.3 && ./emsdk/emsdk activate 6.0.3
source ./emsdk/emsdk_env.sh

wasm/build.sh --test    # builds wasm/dist/ and runs the node smoke test
```

Outputs `wasm/dist/selene_fsw.js` (loader) and `wasm/dist/selene_fsw.wasm`.
On the site these live at `public/wasm/`.

## Automatic site updates

`.github/workflows/wasm-publish.yml` rebuilds the bridge on every push to
`main` that touches `include/`, `src/`, `sim/`, or `wasm/`, smoke-tests it,
and commits the fresh artifacts to `antariksham/Antariksham:public/wasm/` —
the site host then redeploys automatically, so the simulator page always runs
the latest flight software.

Setup requirement (one-time): add a repository secret **`SITE_REPO_TOKEN`**
(fine-grained PAT with `contents: write` on `antariksham/Antariksham`) in
this repo's Settings → Secrets and variables → Actions.
