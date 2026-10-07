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

- [x] **CP-002 - Establish characterization and baselines.** Add characterization
  tests for preprocessing, IMU startup, DVL/pressure/magnetic updates, timestamp
  ordering, fallback, and covariance. Establish current replay repeatability
  before structural edits. **Depends on:** CP-001. **Validation:** focused tests,
  clean build, and recorded short FL2/INS/UWFL2 baseline comparisons.
  **Result (2026-10-07):** fresh isolated build succeeded; nine CTest
  targets passed (60 C++ cases and nine Python fixtures). Fixtures cover padded
  PointCloud2 rows/range filtering, empty input, startup gravity/bias/covariance,
  original and interleaved empty-scan propagation, DVL native-frame Jacobians,
  covariance floors, scalar magnetic correction/protected states/Joseph PSD,
  pressure startup sample non-reuse, and callback queues including duplicates,
  regression clearing, deferred/late samples and one-time consumption.
  Initial failures retained in colcon logs: test-only `Time::to_msg()` is not
  supported in Jazzy; a covariance fixture incorrectly assumed message covariance
  was ignored. Both fixtures corrected without estimator changes.
  Replays: first 40 seconds of bag recording time at x5 (not necessarily 40
  simulated-clock seconds), two repetitions of FL2/INS/UWFL2 on both
  `DONE/sim3` and `DONE/backAndforth_CSSN3_processed`, domains 211--222,
  LC disabled, display publication disabled, sonar-run maps saved. Artifacts:
  `/home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp002_baseline`.
  Every run stores its exact command, runtime/resolved YAML, hashes, dirty diff,
  output bag, map and timing/resource results. No original config/bag edits.
  Strict sequence comparison already failed for identical FL2 simulation
  replays because prediction output coalescing changed counts (907 vs 932);
  their maps matched byte-for-byte and all 483 shared timestamp poses matched
  exactly. Keep this failed strict result; use separately reported shared-time
  comparisons plus exact saved maps, without spatial alignment, for cleanup.
  All 12 runs completed; all six repeated-baseline comparisons passed. Shared
  poses, complete published pose/twist covariance, and twist matched exactly;
  all four sonar-map pairs matched SHA-256. INS correctly has no map. Published
  covariance was finite/PSD in all runs. Comparison requires at least 100 common
  source timestamps spanning 90% of the overlapping interval and differences
  below `1e-9`; no frame/trajectory fitting or interpolation is performed.
  The new comparison reports count/coverage differences rather than asserting
  identical publication schedules. Commit: see `git log --oneline --grep='CP-002'`.

- [x] **CP-003 - Remove unused plotting integration.** Remove the unused Python
  plotting include, `matplotlibcpp.h`, and their CMake requirements. Preserve
  Python analysis tools. **Depends on:** CP-002. **Validation:** clean build,
  complete active test suite, and executable startup.
  **Result (2026-10-07):** deleted the 2,498-line unused wrapper, `Python.h`,
  plotting discovery and Python C++ linkage/includes. Python analysis and test
  tools remain. Fresh `build_refactor_clean`/`install_refactor_clean` build
  passed in 206 s; all nine CTest targets passed in 2.04 s. `readelf -d` confirms
  no `libpython` dependency. Disabled-LC sim3 FL2 startup/replay/map-save passed;
  all 822 shared-timestamp poses/covariances and map SHA matched the baseline
  exactly. Results: `REFACTOR_RESULTS/cp003_smoke/cleanup_comparison.json`.
  Commit: see `git log --oneline --grep='CP-003'`.

- [x] **CP-004 - Delete confirmed legacy mapping/state code.** Delete confirmed
  unused mapping helpers, globals, legacy `StatesGroup`, the unused duplicate
  math header, and the never-populated shutdown-save buffer/path. Preserve
  `/map_save` behavior. **Depends on:** CP-002. **Validation:** usage search,
  build/tests, and saved-map comparison on a short disabled-LC replay.
  **Result (2026-10-07):** repository-wide `rg` found no live consumers of
  `StatesGroup`, its `DIM_STATE`/`INIT_COV` macros, `Exp_mat.h`,
  `esti_normvector`, `pointBodyToWorld_ikfom`, `RGBpointBodyLidarToIMU`,
  `root_dir`, `featsFromMap`, `_featsArray`, or `downSizeFilterMap`. Deleted
  these, three unused local variables, and the never-filled `pcl_wait_save`
  allocation/shutdown write. `/map_save`, dense history, compact-tree insertion,
  live point transforms and sensor/state/covariance logic remain unchanged.
  Deliberately retained the matrix `pointBodyToWorld` overload (used by local
  map segmentation) and `acquire_removed_points` drain (has a storage side effect).
  Signal/shutdown handling and write-only diagnostics remain for later checkpoints.
  Build passed in 128 s; all nine CTest targets passed in 2.06 s (69 test cases).
  All six post-cleanup replay comparisons passed: shared pose, full published
  covariance and twist differences are exactly zero; all four sonar maps are
  byte-identical to baseline. Published covariance remains finite/PSD. INS
  correctly saves no map. Artifacts: `REFACTOR_RESULTS/cp004_candidate`, including
  `comparison.json`, individual manifests/logs/maps and exact commands. No full
  LC replay or Jetson build was needed/performed for these inactive-code deletions.
  Commit: see `git log --oneline --grep='CP-004'`. Stopped here; CP-005 onward
  and newly recorded CP-015 are not implemented.

- [x] **CP-005 - Simplify preprocessing and remove no-op parameters.** Delete
  unreachable preprocessing machinery. Remove obsolete `sonar.scan_line`,
  `sonar.scan_rate`, `sonar.timestamp_unit`, `sonar.point_filter_num`,
  `sonar.feature_extract_enable`, and `sonar.fov_degree` declarations and shipped
  YAML entries. Preserve the active decoder exactly; do not implement these
  previously ineffective options. Document the configuration migration.
  **Depends on:** CP-002. **Validation:** identical decoded points and metadata
  for supported input fixtures, all-config startup, and disabled-LC replay.
  **Result (2026-10-07):** removed unreachable vendor handlers, feature extraction,
  their buffers/settings, and unused FOV calculations. The active generic XYZI
  decoder is text-identical except for clearing two unused buffers. Expanded
  fixtures passed on both old and cleaned code: all eight field datatypes,
  reordered fields, row padding, range filtering, missing intensity, empty
  output and inherited metadata behavior. Build passed in 139 s; all nine CTest
  targets passed in 2.06 s (61 C++ cases plus nine Python fixtures). All eight
  node YAMLs and both LC profiles initialized and shut down successfully;
  startup evidence: `REFACTOR_RESULTS/cp005_startup/results.json`.
  Six x5 disabled-LC, first-40-recording-second FL2/INS/UWFL2 windows on sim3
  and real back-and-forth passed against CP-002: shared poses, published
  covariance and twist exactly unchanged; four saved maps byte-identical;
  covariance finite/PSD. Domains 211--216, evidence and per-run commands/hashes:
  `REFACTOR_RESULTS/cp005_candidate/comparison.json`.
  **Migration:** delete the six listed sonar keys from external YAMLs; they
  previously had no effect and have no replacement. Keep `sonar.min_range` and
  mapping voxel settings unchanged. Shipped YAMLs changed only by these removals;
  existing user tuning and RViz edits preserved. Standard install untouched;
  no Jetson or full LC replay claimed. Implementation commit: `f2ccf19`.
  Stopped here; CP-006 onward remain unimplemented.

  Commands (from workspace for build, repository for Python tools):
  ```bash
  source /opt/ros/jazzy/setup.bash
  MAKEFLAGS=-j2 colcon build --packages-select fast_lio --symlink-install \
    --build-base build_refactor_clean --install-base install_refactor_clean \
    --cmake-args -DBUILD_TESTING=ON
  ROS_DOMAIN_ID=229 ctest --test-dir /home/attia/ros2_ws/build_refactor_clean/fast_lio --output-on-failure
  source /home/attia/ros2_ws/install_refactor_clean/setup.bash
  /usr/bin/python3 test/check_config_startup.py \
    --executable /home/attia/ros2_ws/build_refactor_clean/fast_lio/fastlio_mapping \
    --output /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp005_startup
  /usr/bin/python3 tools/run_refactor_replays.py \
    --output /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp005_candidate \
    --workspace-setup /home/attia/ros2_ws/install_refactor_clean/setup.bash
  /usr/bin/python3 tools/compare_refactor_replays.py \
    --baseline /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp002_baseline \
    --candidate /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp005_candidate \
    --output /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp005_candidate/comparison.json
  ```

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

### Newly Discovered Follow-Up

- [ ] **CP-015 - Tighten benchmark serialization and timestamp evidence.**
  The intentionally retained strict-comparison failure contains JSON `Infinity`.
  Represent unavailable differences with standard JSON `null` plus explicit
  failure reasons; retain native integer ROS stamps and compare frame identifiers.
  Validate saved-file existence against stored manifest evidence. **Depends on:**
  CP-001 and CP-002. **Validation:** missing/unequal/non-finite fixtures and
  one-nanosecond/frame mismatch cases. Safe priority before structural refactors;
  recorded only, not included in CP-001--CP-004 execution.

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
- **Covariance policy:** current DVL fusion uses valid driver covariance with
  YAML diagonal floors; pressure similarly floors scalar driver variance. This
  is not "always ignore message covariance." Characterize it, do not silently
  change it during cleanup. Any policy change requires separate approval.
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

| Completed checkpoint | Implementation commit |
| --- | --- |
| CP-001 | `69c778f` |
| CP-002 | `cd333fe` |
| CP-003 | `efc20d4` |
| CP-004 | `2c82a96` |

Replay totals: CP-002 632.3 s across twelve windows; CP-004 315.8 s across
six windows. CP-003 adds one short startup/regression run. Artifacts retain
per-process CPU/RAM and timing measurements; no performance improvement is
claimed from these wall-time totals. No checkpoint after CP-004 was executed.

- Plan saved after read-only inspection; all implementation checkpoints remain
  pending. Production code, existing configuration edits, and RViz edits were
  left unchanged.

### CP-001 And CP-002 Commands

CP-001 commit: `69c778f`. CP-002: test/tool-only changes; production baseline
remains that commit's estimator, with captured worktree diff. Initial clean
build took 180 s and failed on the test-only Jazzy API; repaired build took
44 s, later expanded fixtures rebuilt in 43 s. Final CTest took 2.01 s.

```bash
cd /home/attia/ros2_ws
source /opt/ros/jazzy/setup.bash
MAKEFLAGS=-j2 colcon build --packages-select fast_lio --symlink-install \
  --build-base build_refactor --install-base install_refactor \
  --cmake-args -DBUILD_TESTING=ON
ROS_DOMAIN_ID=229 ctest --test-dir build_refactor/fast_lio --output-on-failure
cd /home/attia/ros2_ws/src/FAST_LIO_UnderWater
/usr/bin/python3 tools/run_refactor_replays.py \
  --output /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp002_baseline \
  --workspace-setup /home/attia/ros2_ws/install_refactor/setup.bash --repeats 2
/usr/bin/python3 tools/compare_refactor_replays.py \
  --baseline /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp002_baseline \
  --output /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp002_baseline/repeatability.json
```

`runs.json` retains each expanded replay command; `repeatability.json` retains
all six comparisons. Failed strict sample-by-sample comparison is preserved as
`sim3_FL2_1/comparison.json`, not replaced by a passing report.

### CP-003 Commands

CP-002 commit: `cd333fe`. Fresh build/test as above, substituting
`build_refactor_clean` and `install_refactor_clean`. Startup/regression command:

```bash
/usr/bin/python3 tools/run_lc_benchmark.py --label cp003_sim3_FL2 --domain-id 223 \
  --bag /home/attia/ros2_ws/bags/DONE/sim3 \
  --config /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp002_baseline/sim3_FL2.yaml \
  --output /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp003_smoke \
  --workspace-setup /home/attia/ros2_ws/install_refactor_clean/setup.bash \
  --duration 40 --rate 5 --loop-closure false --map-publication false \
  --map-save true --drain-seconds 3
readelf -d /home/attia/ros2_ws/build_refactor_clean/fast_lio/fastlio_mapping
```

Comparison uses `shared_pose_comparison`, `odometry_fields`, `disabled_loop`,
and `matching_maps` from the repaired tools; asserts zero shared-pose/covariance
differences and equal saved-map hashes. No production tuning or frame changes.

### CP-004 Commands

CP-003 commit: `efc20d4`. Usage proof before and after deletion:

```bash
rg -n 'StatesGroup|Exp_mat.h|esti_normvector|pointBodyToWorld_ikfom|RGBpointBodyLidarToIMU|pcl_wait_save|featsFromMap|_featsArray|downSizeFilterMap|root_dir|nearest_search_en|rematch_num|DIM_STATE|INIT_COV' src include
```

After deletion there are no matches. Build/CTest used the same
`build_refactor_clean`/`install_refactor_clean` commands as CP-003. Replays:

```bash
/usr/bin/python3 tools/run_refactor_replays.py \
  --output /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp004_candidate \
  --workspace-setup /home/attia/ros2_ws/install_refactor_clean/setup.bash
/usr/bin/python3 tools/compare_refactor_replays.py \
  --baseline /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp002_baseline \
  --candidate /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp004_candidate \
  --output /home/attia/ros2_ws/bags/REFACTOR_RESULTS/cp004_candidate/comparison.json
git diff --check
```

Six x5, first-40-recording-second windows on isolated domains 211--216; no RViz
or original config/bag edits. Current validated overlay is
`/home/attia/ros2_ws/install_refactor_clean/setup.bash`; the ordinary workspace
install was deliberately not overwritten. These are short-window preservation
tests, not a new accuracy claim or full-dataset/LC performance evaluation.
