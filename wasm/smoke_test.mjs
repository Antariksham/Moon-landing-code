/**
 * Smoke test for the wasm bridge: run the default descent scenario and
 * verify the telemetry the web frontend will consume.
 *
 *   node wasm/smoke_test.mjs <path-to-node-build>/selene_fsw.js
 */
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const modulePath = process.argv[2];
if (!modulePath) {
  console.error('usage: node wasm/smoke_test.mjs <selene_fsw.js>');
  process.exit(1);
}

const createSeleneModule = require(modulePath);
const fsw = await createSeleneModule();

function assert(cond, msg) {
  if (!cond) {
    console.error(`FAIL: ${msg}`);
    process.exit(1);
  }
  console.log(`ok: ${msg}`);
}

assert(fsw.runDefaultScenario() === true, 'default scenario accepted and simulated');

const flightTime = fsw.getFlightTimeS();
assert(flightTime > 30 && flightTime < 600, `flight time plausible (${flightTime.toFixed(1)} s)`);
assert(fsw.getSampleCount() > 1000, `telemetry recorded (${fsw.getSampleCount()} samples at 50 Hz)`);

const t0 = fsw.getStateAtTime(0);
assert(Math.abs(t0.altitudeM - 2000) < 1, `gate altitude ~2000 m (${t0.altitudeM.toFixed(1)} m)`);

// Altitude must decrease monotonically-ish when sampled at 60 fps.
let prev = Infinity;
let decreasing = true;
for (let t = 0; t <= flightTime; t += 1 / 60) {
  const s = fsw.getStateAtTime(t);
  if (s.altitudeM > prev + 5.0) decreasing = false;
  prev = s.altitudeM;
}
assert(decreasing, '60 fps altitude trace descends toward the surface');

const mid = fsw.getStateAtTime(flightTime / 2);
console.log(
  `   sample @ t=${mid.timeS.toFixed(2)}s: altitude=${mid.altitudeM.toFixed(1)} m, ` +
  `vz=${mid.velocityZMps.toFixed(2)} m/s, phase=${mid.missionPhase}, ` +
  `throttle=${(mid.throttleFrac * 100).toFixed(0)}%`
);

const end = fsw.getStateAtTime(flightTime + 10);
assert(end.touchedDown === true, 'state after flight time reports touchdown');
assert(end.altitudeM === 0, 'lander pinned to the surface after touchdown');

const result = fsw.getResult();
assert(result.valid && result.touchedDown, 'result summary valid');
assert(result.safeLanding === true,
  `SAFE LANDING (vz=${result.touchdownVerticalSpeedMps.toFixed(2)} m/s, ` +
  `vx=${result.touchdownHorizontalSpeedMps.toFixed(2)} m/s, ` +
  `tilt=${result.touchdownTiltDeg.toFixed(2)} deg, miss=${result.touchdownMissM.toFixed(1)} m)`);

// Custom scenario + HDA divert demo must also be accepted.
assert(
  fsw.runScenario({
    gateAltitudeM: 2500,
    gateVelocityXMps: 55,
    gateVelocityZMps: -25,
    gatePitchRad: -0.3,
    dryMassKg: 280,
    targetDownrangeM: 1200,
    perfectNav: false,
    hazardAtTarget: true,
  }) === true,
  'hazard-at-target scenario accepted'
);
const hazardResult = fsw.getResult();
console.log(
  `   hazard run: diverted=${hazardResult.hdaDiverted} ` +
  `(${hazardResult.hdaDivertDistanceM.toFixed(0)} m), safe=${hazardResult.safeLanding}`
);

// Stochastic mission shape: low gate, small drift, payload variance, and
// terrain hazards staged from the (procedural) surface survey.
assert(fsw.addHazardZone(-1e3, -990, 25, 1.0) === true, 'hazard zone staged');
assert(fsw.addHazardZone(10, 5, 25, 1.0) === false, 'inverted zone rejected');
assert(fsw.addHazardZone(0, Number.NaN, 25, 1.0) === false, 'NaN zone rejected');
fsw.clearHazardZones();
assert(fsw.addHazardZone(180, 260, 22, 0.8) === true, 'survey zone A staged');
assert(fsw.addHazardZone(320, 344, 14, 0.5) === true, 'survey zone B staged');
assert(
  fsw.runScenario({
    gateAltitudeM: 1100,
    gateVelocityXMps: -4.0,
    gateVelocityZMps: -18,
    gatePitchRad: 0.0,
    dryMassKg: 305,
    targetDownrangeM: 240,
    perfectNav: false,
    hazardAtTarget: false,
  }) === true,
  'stochastic low-gate drift scenario accepted'
);
const stochasticResult = fsw.getResult();
assert(stochasticResult.valid && stochasticResult.touchedDown,
  'stochastic scenario reached the surface');
assert(stochasticResult.hdaDiverted === true,
  `HDA diverted off the staged survey hazard (${stochasticResult.hdaDivertDistanceM.toFixed(0)} m)`);
console.log(
  `   stochastic run: safe=${stochasticResult.safeLanding} ` +
  `(vz=${stochasticResult.touchdownVerticalSpeedMps.toFixed(2)} m/s, ` +
  `miss=${stochasticResult.touchdownMissM.toFixed(1)} m, ` +
  `target=${stochasticResult.finalTargetDownrangeM.toFixed(0)} m)`
);
fsw.clearHazardZones();

console.log('\nAll wasm bridge smoke tests passed.');
