#!/usr/bin/env bash
# Build the SELENE flight software into WebAssembly.
#
# Usage:
#   wasm/build.sh [output-dir]        # web artifact: selene_fsw.js + .wasm
#   wasm/build.sh --test              # also build a node variant and smoke-test it
#
# Requires an activated Emscripten SDK (emcc on PATH):
#   git clone https://github.com/emscripten-core/emsdk.git
#   ./emsdk/emsdk install latest && ./emsdk/emsdk activate latest
#   source ./emsdk/emsdk_env.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

RUN_TEST=0
OUT="$ROOT/wasm/dist"
for arg in "$@"; do
    case "$arg" in
        --test) RUN_TEST=1 ;;
        *) OUT="$arg" ;;
    esac
done
mkdir -p "$OUT"

# Flight library + sim truth models + the Embind bridge. The flight code
# itself carries no OS dependencies (no heap, no exceptions), so the same
# sources that fly on the target compile unmodified for wasm.
SOURCES=(
    "$ROOT/src/fdir/lls_assert.cpp"
    "$ROOT/src/fsw/mission_state_machine.cpp"
    "$ROOT/src/gnc/control/pid_controller.cpp"
    "$ROOT/src/gnc/control/thrust_allocator.cpp"
    "$ROOT/src/gnc/guidance/descent_guidance.cpp"
    "$ROOT/src/gnc/navigation/vertical_nav_filter.cpp"
    "$ROOT/src/hda/safe_site_selector.cpp"
    "$ROOT/sim/src/lander_dynamics.cpp"
    "$ROOT/sim/src/lander_dynamics_3dof.cpp"
    "$ROOT/sim/src/sensor_models.cpp"
    "$ROOT/sim/src/terrain_model.cpp"
    "$ROOT/sim/src/descent_sim.cpp"
    "$ROOT/sim/src/approach_sim.cpp"
    "$ROOT/wasm/selene_wasm.cpp"
)

COMMON_FLAGS=(
    -O3
    -std=c++17
    -I"$ROOT/include"
    -I"$ROOT/src"
    -I"$ROOT/sim/src"
    -lembind
    -sMODULARIZE=1
    -sEXPORT_NAME=createSeleneModule
    -sALLOW_MEMORY_GROWTH=1
    -sINITIAL_MEMORY=33554432
    -sSTACK_SIZE=1048576
)

echo "== Building web artifact -> $OUT/selene_fsw.js"
em++ "${SOURCES[@]}" "${COMMON_FLAGS[@]}" \
    -sENVIRONMENT=web,worker \
    -o "$OUT/selene_fsw.js"
ls -la "$OUT"

if [[ "$RUN_TEST" == "1" ]]; then
    NODE_OUT="$(mktemp -d)"
    echo "== Building node test artifact -> $NODE_OUT"
    em++ "${SOURCES[@]}" "${COMMON_FLAGS[@]}" \
        -sENVIRONMENT=node \
        -o "$NODE_OUT/selene_fsw.js"
    node "$ROOT/wasm/smoke_test.mjs" "$NODE_OUT/selene_fsw.js"
    rm -rf "$NODE_OUT"
fi
