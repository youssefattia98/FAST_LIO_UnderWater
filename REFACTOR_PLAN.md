# Behavior-Preserving Refactor Plan

## Summary

Inspection is complete. This document records the proposed cleanup; no production
code was changed during the inspection or creation of this plan.

Baseline: `32abb7461b031efb490e10befd67876b55f37061`, on
`feature/uwfl2-ltaom-loop-closure`, including the pre-existing user changes in
`config/NTNU.yaml`, `config/default.yaml`, and `rviz/fastlio.rviz`. Preserve those
changes, `main`, `ROS2`, the reference repository, and the paper. Capture the
current worktree diff and configuration hashes before executing a checkpoint.

## Current Architecture

- `LaserMappingNode` combines configuration, synchronization, estimation,
  prediction, mapping, visualization, services, and loop commits in approximately
  3,250 lines, with substantial global state.
- IMU processing retains separate original FAST-LIO and timestamp-interleaved
  auxiliary paths.
- Auxiliary fusion applies timestamp-ordered DVL, pressure, and magnetic updates
  to the same estimator, using a 27-DOF IKFOM error state.
- Loop closure has separate graph, STD, shadow-map, and registration components
  with background workers. The front end commits corrections.
- Dense historical mapping output is separate from the compact active ikd-tree.

## Style Reference

The read-only reference is `/home/attia/ros2_ws/src/auv_core_ros2`.

- Prefer concrete node classes, short executable entrypoints, and separate
  declarations and implementations.
- Make ownership explicit through members, values, and smart pointers. Retain
  shared ownership where immutable asynchronous snapshots genuinely require it.
- Favor descriptive class/function names and responsibility-based member groups.
  The reference commonly uses PascalCase methods and lowerCamelCase members with
  trailing underscores. Apply naming improvements locally, not through a broad
  rename of inherited numerical code.
- Use ROS logging and explicit subscription, timer, publisher, and service
  members. Document units and transform directions where they matter.
- Keep configuration and package dependencies explicit. Do not copy the
  reference's FSM architecture, configuration library, backup files,
  raw-allocation patterns, or global build flags.
- Prefer deletion and simple concrete components over generic frameworks,
  additional ROS nodes, or speculative abstractions.

## Main Findings

- Unreachable preprocessing branches retain Ouster, Velodyne, Mid360, and
  feature-extraction machinery. Several exposed sonar parameters consequently
  have no effect.
- Unused plotting dependencies, legacy state definitions, mapping helpers,
  write-only status fields, and inactive diagnostic writers remain.
- Auxiliary summaries calculate unused statistics and duplicate DVL
  linearization.
- Benchmark tools still write obsolete parameter names, inspect removed topics,
  and can treat missing map hashes as equal.
- Shared headers contain non-inline definitions. Splitting implementation across
  translation units requires resolving these first.
- Tests emphasize loop closure. Initialization, auxiliary fusion, timing, and
  complete transaction behavior lack focused coverage.
- CMake repeats compiler settings, requires unused dependencies, and relies on
  incomplete dependency declarations.

## Checkpoints

Revalidate each finding against the current code
before execution. Dependencies identify required completed checkpoints, not
permission to implement multiple checkpoints together.

- [x] **CP-001 - Repair verification tools.** Repair benchmark configuration
  overrides and comparison checks. Reject unsupported options; distinguish
  missing evidence from passing results. Add Python fixture tests. No estimator
  changes. **Depends on:** none. **Validation:** fixture tests for current YAML
  overrides, unsupported options, missing maps/metrics, and explicit LC state.
  **Result (2026-10-07):** eight Python fixture tests passed with
  `source /opt/ros/jazzy/setup.bash; /usr/bin/python3 test/test_benchmark_tools.py`.
  Mapping/publication overrides now target current nested parameters; unsupported
  independent detection/marker settings fail explicitly. Empty/missing maps,
  absent metrics, non-finite/empty trajectories, and another sensor's disabled
  flag cannot pass comparisons. Explicit isolated workspace setup and full
  worktree-diff provenance added. No estimator changes. Commit: see
  `git log --oneline --grep='CP-001'`.

- [ ] **CP-002 - Establish characterization and baselines.** Add characterization
  tests for preprocessing, IMU startup, DVL/pressure/magnetic updates, timestamp
  ordering, fallback, and covariance. Establish current replay repeatability
  before structural edits. **Depends on:** CP-001. **Validation:** focused tests,
  clean build, and recorded short FL2/INS/UWFL2 baseline comparisons.

- [ ] **CP-003 - Remove unused plotting integration.** Remove the unused Python
  plotting include, `matplotlibcpp.h`, and their CMake requirements. Preserve
  Python analysis tools. **Depends on:** CP-002. **Validation:** clean build,
  complete active test suite, and executable startup.

- [ ] **CP-004 - Delete confirmed legacy mapping/state code.** Delete confirmed
  unused mapping helpers, globals, legacy `StatesGroup`, the unused duplicate
  math header, and the never-populated shutdown-save buffer/path. Preserve
  `/map_save` behavior. **Depends on:** CP-002. **Validation:** usage search,
  build/tests, and saved-map comparison on a short disabled-LC replay.

- [ ] **CP-005 - Simplify preprocessing and remove no-op parameters.** Delete
  unreachable preprocessing machinery. Remove obsolete `sonar.scan_line`,
  `sonar.scan_rate`, `sonar.timestamp_unit`, `sonar.point_filter_num`,
  `sonar.feature_extract_enable`, and `sonar.fov_degree` declarations and shipped
  YAML entries. Preserve the active decoder exactly; do not implement these
  previously ineffective options. Document the configuration migration.
  **Depends on:** CP-002. **Validation:** identical decoded points and metadata
  for supported input fixtures, all-config startup, and disabled-LC replay.

- [ ] **CP-006 - Remove unused diagnostic work.** Remove unreachable front-end
  diagnostic writers, write-only status, unused auxiliary summaries/getters,
  and redundant diagnostic-only DVL linearization. Retain live timeout and
  late-message warnings and test-used LC diagnostics. **Depends on:** CP-002.
  **Validation:** focused sensor tests, unchanged accepted updates and covariance,
  complete active suite, and disabled-LC replay comparison.

- [ ] **CP-007 - Consolidate build settings and dependencies.** Consolidate CMake
  settings and dependencies. Remove unused component/aggregate dependencies and
  declare direct ROS dependencies explicitly. Preserve optimization, OpenMP
  defaults, GTSAM discovery, and compatibility branches. **Depends on:** CP-003.
  **Validation:** clean desktop and Jetson builds/tests; inspect effective compiler
  flags and installed executable dependencies. Record unavailable platforms as
  blockers rather than claiming validation.

- [ ] **CP-008 - Make shared headers safe to split.** Make shared definitions safe
  across translation units and add missing include guards/direct includes.
  Preserve names, numeric constants, formulas, and state layout. **Depends on:**
  CP-004 and CP-007. **Validation:** multi-translation-unit link test, numerical
  characterization, complete active suite, and disabled-LC replay.

- [ ] **CP-009 - Separate auxiliary declarations and implementation.** Move
  auxiliary implementation into a `.cpp`, retaining the concrete class,
  subscriptions, locking, update order, calibration, and covariance operations.
  **Depends on:** CP-006 and CP-008. **Validation:** sensor characterization and
  disabled-mode replay comparisons, including INS without sonar.

- [ ] **CP-010 - Separate IMU declarations and implementation.** Move IMU
  implementation into a `.cpp`. Keep both propagation/deskew paths unchanged;
  do not merge them. **Depends on:** CP-008. **Validation:** startup, fallback,
  deskew, covariance, and FL2/INS/UWFL2 replay equivalence.

- [ ] **CP-011 - Clarify node setup and entrypoint.** Extract parameter setup into
  clearly named node methods and separate the executable entrypoint. Keep
  defaults, validation, initialization order, and launch arguments unchanged.
  **Depends on:** CP-005 and CP-007. **Validation:** every shipped YAML and both
  installed and symlink launch modes; compare declared parameters and defaults.

- [ ] **CP-012 - Consolidate Odometry/TF formatting.** Consolidate duplicated
  message construction using explicit state, covariance, and timestamp inputs.
  Preserve publication scheduling, coalescing, frames, and QoS.
  **Depends on:** CP-002. **Validation:** compare serialized message fields,
  covariance, timestamp monotonicity, and output cadence in both publication paths.

- [ ] **CP-013 - Make input-buffer ownership explicit.** Make input queues,
  timestamps, and mutexes explicitly node-owned. Preserve callback groups,
  lock ordering, regression clearing, and reorder deadlines. Do not alter
  IKFOM's callback interface. **Depends on:** CP-009, CP-010, and CP-011.
  **Validation:** concurrent delivery, shutdown, timestamp regression, sonar
  outage/recovery, and disabled-LC replay comparisons.

- [ ] **CP-014 - Separate historical-map/output responsibilities.** Separate
  historical-map/output responsibilities from estimation. Preserve dense saved
  history, existing display filtering, marker IDs/QoS, bounded visualization
  queue, and loop replacement behavior. **Depends on:** CP-012 and CP-013.
  **Validation:** save/display snapshots, late subscriber and queue-overflow
  behavior, LC-enabled replay, rollback behavior, and runtime/memory comparison.

## Validation Policy

### Inspection Result

Existing build artifacts passed six CTest targets containing 51 active GoogleTest
cases. Older result XML files also exist; do not count them as current tests.
No fresh build or bag replay was performed during this inspection. No checkpoint
is complete based on inspection alone.

### Build And Test Commands

Use isolated build artifacts so stale results do not inflate test counts:

```bash
cd /home/attia/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fast_lio --symlink-install \
  --build-base build_refactor --install-base install_refactor \
  --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build_refactor/fast_lio --output-on-failure
source install_refactor/setup.bash
```

Use the repaired benchmark runner from CP-001 for replay recording. Record its
exact command and resolved runtime configuration for each run.

- Characterize short `sim3` and real back-and-forth windows with FL2, INS, and
  UWFL2; explicitly disable LC in temporary configurations. Include missing,
  sparse, duplicate, and regressing sensor timestamps in focused fixtures.
- Compare pose, covariance, output timestamps/counts, saved-map contents, and
  runtime against repeated baseline runs. Deterministic fixtures should agree
  within `1e-12`; record replay repeatability rather than inventing tolerances.
  If repeatability cannot be established, stop before a risky refactor.
- Use full LC-enabled replay only for ownership/map changes. Record loop
  outcomes, rollback behavior, latency, memory, and dropped messages.
- Record bag/config/Git hashes, dirty diff, ROS domains, replay rate, exact
  commands, output directories, results, and failures. Preserve original bags
  and result directories. Never use a different sensor configuration to disguise
  a refactoring regression.
- Keep full SE(3), sensor/state/covariance handling, front-end ownership of the
  live estimator/tree, and optional loop closure unchanged.

## Decisions Requiring Human Review

These are not automatic refactors or permission to change behavior:

- **Bias-noise mismatch:** the outage manager currently returns gyro-bias process
  noise for both gyro and accelerometer bias channels. Treat correction as a
  separately approved behavior-changing bug fix.
- **Estimator mathematics:** review LC mean/covariance consistency and correlated
  graph information before changing covariance formulas.
- **Paper/code differences:** pressure Jacobian restrictions, magnetic gyro-bias
  protection, and descriptions of iterated auxiliary updates need reconciliation.
  Do not edit the paper as part of these cleanup checkpoints.
- **Safety and resource policy:** fixed correspondence buffers, unchecked input
  layouts, growing dense history, and expensive commit preparation need separate
  safety/performance decisions.
- **Architectural changes:** merging propagation paths, changing fusion order,
  altering frames/gates/QoS, replacing IKFOM, splitting into additional ROS nodes,
  or dropping platform compatibility require explicit approval.
- **Metadata:** confirm maintainer/authorship and supported platforms before
  replacing placeholder metadata. Retain upstream attribution and licensing.

The approved configuration migration removes only demonstrably obsolete keys.
No tuning, measurement changes, or user RViz replacement is included.

## Checkpoint Execution Rules

When requested to `Execute CP-XXX`:

1. Re-read that entry and the affected code; verify the proposed change still
   makes sense and its dependencies are complete.
2. Implement only that checkpoint, preserving behavior and following relevant
   reference conventions without unrelated formatting or abstraction changes.
3. Run appropriate tests/build/lint and record failures as well as successes.
4. Update this document with what changed, exact commands, results, timing where
   relevant, blockers, and the focused commit hash when a commit is made. Update
   `UWFL2_LC_PLAN.md` for applicable loop-closure/front-end work.
5. Add newly discovered cleanup as future checkpoints instead of expanding the
   active checkpoint. Never mark a checkpoint complete without validation.
6. Stop after the requested checkpoint.

## Execution Log

- Plan saved after read-only inspection; all implementation checkpoints remain
  pending. Production code, existing configuration edits, and RViz edits were
  left unchanged.
