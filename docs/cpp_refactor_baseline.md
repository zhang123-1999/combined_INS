# C++ Refactor Baseline

Updated: `2026-03-13`

## Goal

This note locks the current low-risk C++ cleanup baseline so future refactors do
not silently change solver behavior while improving structure.

## Stable Boundaries

- Public runtime entrypoints stay unchanged: `eskf_fusion`, `uwb_generator`,
  `jacobian_audit`, `regression_checks`.
- Existing `config_*.yaml` field names remain compatible.
- Current solver flow remains:
  `Predict -> ZUPT -> Gravity diagnostics -> NHC -> ODO -> UWB -> GNSS -> record`.
- Current state layout remains `kStateDim = 31`.

## Current Module Split

- `src/app/dataset_loader.cpp`
  Data loading, format detection, and time-window cropping.
- `src/app/pipeline_fusion.cpp`
  Runtime orchestration and measurement scheduling.
- `src/app/initialization.cpp`
  Initial state and `P0` construction.
- `src/app/config.cpp`
  Runtime anchor construction.
- `src/core/*`
  Filter propagation, correction engine, and measurement models.

## Compatibility Locks

- Do not change official experiment configs during structural cleanup.
- Do not change output column layout in solver result files.
- Do not mix structural cleanup with algorithm-theory changes in the same patch.
- Any change touching `fusion.ablation.*`, `fusion.post_gnss_ablation.*`,
  `fusion.gnss_schedule.*`, or state block `21-30` must keep existing semantics.

## Regression Commands

- Configure/build:
  `cmake --build build --config Release --target eskf_fusion regression_checks`
- Fast contract checks:
  `build\\Release\\regression_checks.exe`
- Representative smoke run:
  `build\\Release\\eskf_fusion.exe --config config_data2_baseline_eskf.yaml`

## Immediate Refactor Priorities

1. Keep shrinking oversized app-layer files without changing solver math.
2. Isolate compatibility and legacy parsing paths from runtime code.
3. Move library-like code away from direct `cout` side effects where practical.
4. Add regression checks before any change that can affect official metrics.
