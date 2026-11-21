# Optional AArch64 packet NEON contact solver

This backend uses two independent double-precision contacts per Advanced SIMD vector in `cpSpaceStep()`. It reuses the AVX2/Wasm packet preparation, dependency scheduling, allocation-failure fallback, mixed contact/joint synchronization, and scalar joint sweeps. It does not use `cpHastySpaceStep()` or create threads.

## Build and select

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_DEMOS=OFF -DBUILD_BENCHMARKS=ON -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

`MUNK2D_ENABLE_NEON_CONTACT_SOLVER` defaults **ON**, like optional AVX2 inclusion. CMake probes the actual compiler target: `__aarch64__`, `arm_neon.h` double-vector intrinsics, and `CP_USE_DOUBLES=1` are required. Unsupported targets or precision configurations omit NEON without failing configuration. Set the option OFF to omit it deliberately; old build directories retain their cached option value. There is no float or 32-bit ARM backend. The target is ordinary AArch64 with Advanced SIMD support, not optional-ISA ARM32 or a general-registers-only environment. No x86-style runtime feature probe is needed for this AArch64 target.

```c
if(cpContactSolverIsAvailable(CP_CONTACT_SOLVER_NEON)) {
    if(!cpSpaceSetContactSolver(space, CP_CONTACT_SOLVER_NEON)) {
        /* Locked space or allocation failure: selection was not changed. */
    }
}
```

Build-time enablement does not change the default solver. MunkBench accepts `--contact-solver neon`; the mixed benchmark (`munkbench_mixed`) uses `--solver neon`. Unavailable selection fails explicitly. HastySpace remains separate.

For Android use the NDK CMake toolchain with `ANDROID_ABI=arm64-v8a`. Native command-line/static-library consumers currently also need Android's `log` library for the pre-existing `cpMessage` implementation, e.g. `-DCMAKE_C_STANDARD_LIBRARIES=-llog` in a C-only NDK test build. Benchmarks run as ordinary ADB shell executables; neither root nor Wi-Fi is required.

## Layout and numerical policy

Logical packets remain four contacts wide. The backend solves each packet in two-contact halves, using the shared scalar tail for a final singleton. Loads transpose exactly six adjacent doubles per body into component lanes; stores touch only writable active bodies, never padding or a shared frozen side. Scalar packet and vector code share `cpContactSolverKernel.inc` and operation order. Compare/select min/max preserves scalar tie, signed-zero and unordered operand behavior rather than relying on hardware min/max.

NEON and scalar packet kernels disable floating-point contraction locally. **Including NEON no longer changes the baseline compiler policy**: `cpArbiter.c`, `cpSpaceStep.c` and `cpContactSolver.c` keep the ordinary compiler settings, including permitted FMA contraction. The default solver is still baseline; merely making NEON available must not change its results.

The ordinary AArch64 baseline may therefore round differently from the non-fused packet kernels. This is not an unsafe-instruction or memory failure and is not a promise of bit-identical baseline/NEON trajectories. Exact NEON checks use the same-schedule scalar packet oracle; baseline repeatability, physical-quality probes and exact baseline fallback checks remain separate. CI also compares ordinary baseline observations from independently built NEON-included and NEON-omitted libraries across all 26 reference scenes.

Selecting NEON can still change trajectories, including relative to the original 2.4.0 implementation: that implementation disabled contraction in baseline-related code even when baseline was selected. Validate replay/lockstep applications when upgrading.

The NEON backend translation unit is kept out of cross-file LTO. Public structs are unchanged and existing solver enum values retain their values; `CP_CONTACT_SOLVER_NEON` is appended as value 3. There is no Pymunk integration/policy change here.

## Validation and early performance evidence

The measurements below describe the **original 2.4.0 arithmetic policy**, including its build-wide non-fused baseline control. They are retained as historical evidence, not fresh qualification of the isolated compiler policy described above.

On one Galaxy S22 SM-S901B (Exynos 2200, Android 16), NDK r27d Release/default LTO, the packet kernel matched the scalar packet oracle byte-for-byte across all 26 smoke/reference benchmark checkpoint captures. The native solver tests cover partial packets, allocation boundaries, signed zero, frozen-side writes, poisoned/reused buffers, varying time steps/iterations, callbacks, sleep/wake, lifecycle and all ten built-in joint types. The comparison does not imply bit-identical trajectories to a differently compiled stock engine or all other backends.

Six interleaved rounds on two Cortex-A710 cores (affinity mask 0x30), CLOCK_MONOTONIC whole-step wall time, all 26 scenes, one untimed warmup and one timed fresh-world run per scene:

| Control | Smoke time difference | Reference time difference |
|---|---:|---:|
| NEON vs stock 2.3.0 baseline | -2.86% | -3.13% |
| NEON vs enabled-build non-fused baseline | -4.59% | -4.09% |
| NEON vs scalar packet control | -6.80% | -6.00% |
| Enabled-build baseline vs stock baseline | +1.75% | +1.11% |

Percentages are geometric means of per-scene median matched-round time ratios; negative means faster. Reference summed median scene times improve 2.65% vs stock. Gains are workload-dependent: reference FrictionalPyramid -15.75%, FallingSquares -8.35%, but AddPair +1.96% and ConstraintMix +1.42%. One round had an enabled-baseline timing outlier; medians retain all rounds rather than dropping it. OS thermal status was 0 at sampled round boundaries; frequency was not locked. This is one device/compiler/core-pair experiment, not a universal speedup guarantee or an unrestricted whole-phone scheduler result.

Use repeated paired whole-step measurements and validate behavior before selecting this backend in an application. Legacy HastySpace within-contact NEON and HastySpace threading are different algorithms/controls, not substitutes for these measurements.
