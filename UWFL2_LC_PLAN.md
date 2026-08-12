# UWFL2 LTA-OM Loop-Closure Plan

## Status

- Branch: `feature/uwfl2-ltaom-loop-closure`
- Plan status: approved and active
- Implementation status: Checkpoints 0--3 complete; Checkpoint 4 in progress
- Estimator source-code changes through Checkpoint 0: none
- UWFL2 baseline commit: `27d71770951cc495efc36398fbde937efa0b060a`
- Pre-existing worktree change: `config/default.yaml` selects sonar plus IMU only. It is not part of loop-closure work and must not be silently committed.

No implementation checkpoint below is complete until its build, tests, replay results, timings, and focused commit are recorded in this file.

## Goal

Port the applicable LTA-OM loop-closure pipeline into UWFL2 while preserving the original real-time sonar/IMU/DVL/pressure/magnetometer IKFOM front end. The result must support full SE(3), keep loop closure optional, reconstruct corrected historical map geometry, and commit a corrected estimator state and shadow ikd-tree without exposing the front end to a mixed old/new map generation.

## References Studied

| Reference | Snapshot | Relevant finding |
| --- | --- | --- |
| UWFL2 source | `27d7177` | ROS 2, 27-DoF IKFOM state, asynchronous auxiliary measurements, one live `KD_TREE`, one processing callback owns EKF mutation. |
| UWFL2 paper | SHA-256 `e1dafcdd9737cfa684af195352d1ef61d24c508332b6e3d7fdcb7b7adc90304a` | Defines startup-local, vehicle, sonar, DVL, pressure, magnetic frames and the extra DVL/pressure bias states. |
| Original FAST-LIO2 source | `a4743b095409588842a5b30ddfa27e29d2f99164` (`ROS2`) | Establishes the scan-bounded IMU propagation, iterated point-to-map correction, and incremental ikd-tree behavior that disabled mode must retain. |
| FAST-LIO2 paper | SHA-256 `ba5072e06e75dec5222bcb528143f3aa3238d6f6692bac1e22a9fe1bc55a15dd` | Direct raw-point IEKF update and incremental map design. |
| LTA-OM source snapshot | tree SHA-256 `31223f8c0fa2cc40dddfb1ae0ea97dcea611c5c094baa2ae9bdbc3f03a3c8a98` | ROS 1 pipeline: accumulated submaps, STD candidates, pose graph, false-positive rejection, corrected historical points, background tree build, scan re-registration, then pose/tree replacement. |
| LTA-OM paper | SHA-256 `0f2d589805c370a44bc2afca5ac5ded904139035063af8133de691d405d00f9c` | Defines long-term association, graph consistency rejection, dynamic historical-map loading, and delayed-current-pose correction. |

### Important adaptation findings

1. LTA-OM's source is not a drop-in ROS 2 component. It uses ROS 1 nodes, a machine-specific TBB path, and custom `ISAM2::backup/recover` patches.
2. The local machine has TBB 2021.11 and OpenCV 4.6. Checkpoint 1 verified the standard, unpatched Ubuntu GTSAM 4.2 package; development packages were unpacked into an isolated `/tmp` prefix because system installation requires an interactive sudo password.
3. LTA-OM replaces only FAST-LIO2 pose and velocity. UWFL2 also has gravity, DVL bias, pressure bias, magnetic startup reference, auxiliary reference data, extrinsics, and full cross-covariance. A direct copy would be incomplete.
4. LTA-OM's active-tree lock can block the front end during scan re-registration. UWFL2-LC will instead build immutable shadow results in the background and let the front-end thread perform the validated commit.
5. STD was designed for LiDAR geometry. Its thresholds cannot be copied to sparse 3D sonar without first measuring keypoint and overlap statistics.

## Scope And Non-Goals

### In scope

- Full-SE(3) keyframes, odometry factors, manual loop factors, and automatic STD loop factors.
- Transactional false-loop rejection using standard graph-library APIs.
- Corrected historical-map reconstruction and spatially bounded shadow ikd-tree rebuilding.
- Latest-scan re-registration and a short, validated state/tree commit.
- Baseline, functional, accuracy, map-consistency, and Jetson resource evaluation.

### Not in the first delivery

- Multi-session mapping.
- Online sonar/IMU/DVL/pressure/magnetometer extrinsic calibration.
- Replacing the UWFL2 IKFOM front end with smoothing or a factor graph.
- Planar constraints or surface-vehicle assumptions.
- Silent fallback to another optimizer when GTSAM is unavailable.

## Proposed Architecture

### Frame and transform contract

- `<ell>` is the fixed startup-local/map frame used by UWFL2 and anchored by the first graph node.
- `<v>` is the vehicle/IMU frame and `<s>` is the sonar frame.
- Graph nodes store `T_ell_v`. Sonar poses are derived only as `T_ell_s = T_ell_v T_v_s` using the existing fixed sonar-IMU extrinsic.
- Keyframe clouds remain in `<s>` or `<v>`, never irreversibly stored only in `<ell>`. Corrected reconstruction applies the optimized full-SE(3) pose to the original local points.
- Loop measurements have one documented direction: `T_v_i_v_j = inverse(T_ell_v_i) T_ell_v_j`. Unit tests must fail if the source/target order is reversed.
- The graph uses `Pose3` without removing z, roll, or pitch.

### Runtime components

1. **Front-end adapter**
   - Runs in the existing processing callback.
   - Produces immutable keyframe jobs only after the sonar IEKF update and before/with map insertion metadata capture.
   - Remains the sole writer of `kf`, `state_point`, active map bounds, and the active ikd-tree.

2. **Bounded keyframe/STD worker**
   - Consumes downsampled immutable scans and pose snapshots.
   - Builds plane/binary/STD descriptors and searches historical candidates.
   - Uses bounded queues; records dropped/coalesced jobs rather than blocking sensor callbacks.

3. **Pose-graph worker**
   - Maintains committed full-SE(3) nodes and odometry factors.
   - Tests loop factors in a temporary graph/optimizer state. It never mutates the committed graph until all rejection checks pass.
   - Uses standard GTSAM APIs, not LTA-OM's patched backup/recover methods.

4. **Historical-map/shadow-tree worker**
   - Reconstructs only corrected keyframe points needed around the corrected current pose, with a configurable 3D load radius.
   - Builds a new `KD_TREE<PointType>` off the front-end thread.
   - Tags every result with graph, keyframe, and front-end map generations.

5. **Correction mailbox and front-end commit**
   - Carries one immutable pending correction at a time.
   - Coalesces superseded graph versions.
   - The front end accepts a correction only at a scan boundary and only if its anchor/generation is still valid.

### Keyframe record

Each keyframe will contain:

- monotonically increasing ID and exact sensor timestamp;
- raw `T_ell_v` and 6x6 pose covariance extracted from the full IKFOM covariance;
- fixed `T_v_s` snapshot and frame labels;
- voxel-filtered, undistorted sonar points in their local sensor/body frame;
- graph version and active-tree generation;
- optional STD planes, binary keypoints, descriptors, and quality statistics;
- the scan interval needed to relate a delayed graph correction to the latest state.

Selection uses full translation norm, full SO(3) angular distance, minimum elapsed time, and minimum usable point count. Thresholds remain parameters and will be selected from recorded sonar statistics, not LTA-OM LiDAR defaults.

### Pose graph and false-loop rejection

- Fix the graph gauge with a prior on the first `T_ell_v` keyframe.
- Add consecutive full-SE(3) odometry factors from UWFL2 relative poses. Derive factor covariance from the corresponding IKFOM pose covariance with documented numerical floors.
- A loop candidate passes, in order:
  1. temporal/keyframe separation;
  2. STD rough and fine matching;
  3. full-3D geometric verification and overlap;
  4. full-SE(3) innovation/cycle consistency;
  5. optimization on a temporary graph state;
  6. finite solution, graph residual, adjacent-factor residual, and map-overlap checks.
- Rejecting a loop leaves the committed graph, live state, and active tree unchanged.
- High-leverage first loops require either a second mutually consistent loop or an explicitly enabled manual-test override. No yaw-only or horizontal-distance rule is allowed.

### Corrected current state

For graph anchor keyframe `k`, first form the delayed full-SE(3) correction

```text
T_corr_graph = T_ell_v_k_optimized * inverse(T_ell_v_k_raw)
T_ell_v_now_candidate = T_corr_graph * T_ell_v_now_raw
```

The latest available local scan is then registered to the shadow tree to obtain a small full-SE(3) refinement `T_corr_reg`. The combined correction is validated before commit.

Because the first graph node fixes `<ell>`, this is a physical pose correction inside the existing startup-local frame, not a redefinition of `<ell>`. Therefore:

- update vehicle position and attitude with the accepted full-SE(3) correction;
- rotate startup-local velocity consistently with the correction rotation;
- keep body/sensor-frame IMU biases, DVL bias, pressure bias, and sonar extrinsics unchanged;
- keep startup-local gravity, the startup magnetic-heading reference, pressure startup reference, and World-to-`camera_init` transform unchanged;
- transport the complete 27x27 covariance with the analytic Jacobian of the chosen correction/retraction, including every cross block;
- add accepted graph-correction uncertainty conservatively instead of zeroing or shrinking unrelated covariance blocks;
- verify the covariance Jacobian with finite differences and require symmetric positive-semidefinite output.

If this frame interpretation conflicts with a measured UWFL2 residual during implementation, stop at that checkpoint and document the evidence rather than rotating reference states speculatively.

### Shadow-tree and atomic commit contract

The background worker may read immutable keyframe snapshots but may never access mutable `kf` or the active ikd-tree. It returns:

```text
PendingCorrection {
  graph_version,
  source_tree_generation,
  anchor_keyframe_id,
  correction_timestamp,
  corrected_keyframe_poses,
  corrected_state_proposal,
  corrected_covariance_proposal,
  shadow_tree,
  shadow_local_map_bounds,
  registration/residual statistics
}
```

At the next safe front-end boundary, the processing callback:

1. checks versions, timestamps, finiteness, covariance PSD, and registration improvement;
2. snapshots the old state/covariance/tree metadata for immediate rollback;
3. applies the state and covariance proposal;
4. swaps the active tree pointer and matching map bounds/generation together;
5. refreshes `state_point`, `pos_lid`, and other derived caches;
6. inserts the current scan only into the new generation;
7. rolls back the whole transaction if any postcondition fails.

No scan may query one tree and insert into another generation. Normal front-end queries require no backend mutex because only the processing callback mutates the active generation.

### Disabled-mode contract

With `loop_closure.enable: false`:

- no keyframes, graph, STD database, worker threads, or shadow trees are created;
- sensor ordering and IKFOM calls remain unchanged;
- odometry/cloud message counts and timestamps must match baseline;
- trajectory and map differences must remain within a documented floating-point tolerance;
- p95 scan latency must not regress by more than 2% on the same machine and replay.

## Common Commands

Run all final comparisons sequentially at x1. Parallel replays are prohibited for CPU/latency comparisons.

```bash
export UWFL2_WS=/home/attia/ros2_ws
export UWFL2_SRC=/home/attia/ros2_ws/src/FAST_LIO_UnderWater
export BAG_SIM=/home/attia/ros2_ws/bags/DONE/sim3
export BAG_REAL=/home/attia/ros2_ws/bags/DONE/backAndforth_CSSN3_processed
export LC_RESULTS=/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS
source /opt/ros/jazzy/setup.bash
cd "$UWFL2_WS"
colcon build --packages-select fast_lio --symlink-install --event-handlers console_direct+
source "$UWFL2_WS/install/setup.bash"
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --verbose
```

Dependency preflight:

```bash
cmake --find-package -DNAME=GTSAM -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST
pkg-config --modversion tbb
pkg-config --modversion opencv4
```

Dependency decision: use the standard, unpatched `libgtsam-dev` 4.2 package through `find_package(GTSAM 4.2 REQUIRED)`. The current Ubuntu Noble repository offers `4.2.0+dfsg-1build1`; installation and an aarch64/Jetson availability check are Checkpoint 1 prerequisites. Do not use LTA-OM's patched GTSAM or its `ISAM2::backup/recover` extension. If the Jetson image cannot provide compatible GTSAM, stop and document that result before choosing a pinned vendor build.

## Checkpoint 0: Freeze And Reproduce Original UWFL2 Baseline

### Work

- [x] Record Git hashes, source/reference hashes, host details, config hashes, bag metadata, and the pre-existing `default.yaml` diff.
- [x] Add a reusable sequential benchmark runner and analysis script without changing estimator behavior.
- [x] Build and test `27d7177` behavior on the feature branch with loop closure absent/disabled.
- [x] Record full simulation and real-bag baselines at x1.
- [x] Save odometry, TF, registered cloud, final map, stdout/stderr, `/usr/bin/time -v`, topic counts, and an end-to-end ROS-time output-lag proxy.
- [x] Resolve the GTSAM dependency decision.

### Exact baseline commands

```bash
cd "$UWFL2_SRC"
git branch --show-current
git rev-parse HEAD
git status --short --branch
git diff -- config/default.yaml
sha256sum config/sim.yaml config/backAndforth_CSSN3_processed.yaml
ros2 bag info "$BAG_SIM"
ros2 bag info "$BAG_REAL"
```

Canonical commands used:

```bash
python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label baseline_sim3 --domain-id 71 --rate 1.0 \
  --bag "$BAG_SIM" --config "$UWFL2_SRC/config/sim.yaml" \
  --loop-closure false --output "$LC_RESULTS/baseline_sim3" \
  --workspace /tmp/uwfl2_lc_cp0_ws --drain-seconds 10

python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label baseline_real_backforth --domain-id 72 --rate 1.0 \
  --bag "$BAG_REAL" --config "$UWFL2_SRC/config/backAndforth_CSSN3_processed.yaml" \
  --loop-closure false --output "$LC_RESULTS/baseline_real_backforth" \
  --workspace /tmp/uwfl2_lc_cp0_ws --drain-seconds 10
```

Manual fallback while creating the runner, in three terminals:

```bash
# Terminal 1
export ROS_DOMAIN_ID=71
mkdir -p "$LC_RESULTS/baseline_sim3/ros_logs"
cd "$LC_RESULTS/baseline_sim3"
/usr/bin/time -v -o process_time.txt env ROS_LOG_DIR="$PWD/ros_logs" \
  ros2 launch fast_lio mapping.launch.py \
  config_file:="$UWFL2_SRC/config/sim.yaml" rviz:=false use_sim_time:=true

# Terminal 2
export ROS_DOMAIN_ID=71
ros2 bag record -o "$LC_RESULTS/baseline_sim3/output_bag" \
  /Odometry /cloud_registered /tf /tf_static

# Terminal 3
export ROS_DOMAIN_ID=71
ros2 bag play "$BAG_SIM" --clock --rate 1.0
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

### Acceptance

- Build and tests pass.
- Replay starts, completes, drains buffers, and shuts down cleanly.
- Saved output is nonempty and can be replayed.
- Baseline metrics and all failures are retained under immutable result directories.

### Log

- Decisions: Keep baseline artifacts immutable under `bags/UWFL2_LC_RESULTS`; use each bag's recorded `/clock`; run comparisons sequentially at x1; use unpatched GTSAM 4.2; treat the monitor's `/clock` minus odometry-header value as an output-lag proxy, not internal scan execution time.
- Commands run: isolated `colcon build`, `colcon test`, `colcon test-result`; two 30 s runner smoke tests; the two canonical full replays above; `tools/analyze_lc_run.py` on each completed run. Exact commands, resolved parameters, host data, Git status, and bag metadata are retained in each run manifest.
- Build/test result: build passed in 87 s. `colcon test` and `test-result` passed with zero errors, but the repository currently contains zero registered tests. The first `test-result` invocation attempted to write under the sandboxed workspace log directory and failed; rerunning it with `--log-base /tmp/uwfl2_lc_cp0_ws/test_result_log` passed. This failure is retained here rather than hidden.
- Simulation result (`baseline_sim3`): 5,945 odometry and 5,944 registered-cloud messages; no invalid poses, nonmonotonic stamps, or invalid covariances. Translation ATE RMSE/max/final was 0.070/0.156/0.081 m, depth RMSE/max/final was 0.0073/0.0132/0.0045 m, and relative-attitude RMSE/max/final was 0.219/0.527/0.183 deg. The compact map has 5,739,064 points (183,650,301 bytes).
- Real result (`baseline_real_backforth`): 3,808 odometry and 3,799 registered-cloud messages; no invalid poses, nonmonotonic stamps, or invalid covariances. No trusted ground truth is present, so only continuity is reported: 93.494 m path length and 0.938 m start/end translation. The compact map has 1,850,885 points (59,228,573 bytes).
- Timing/resources: simulation wall time 1,309.75 s, CPU mean/p95/max 15.19/19.8/29.8%, RSS mean/p95/max 347.1/472.4/481.6 MB, output-lag proxy mean/p95/max 13.3/13.3/16.7 ms. Real wall time 663.94 s, CPU mean/p95/max 17.04/19.8/22.8%, RSS mean/p95/max 261.6/309.8/326.4 MB, output-lag proxy mean/p95/max 20.1/34.0/302.8 ms.
- Artifact paths: `/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/baseline_sim3` and `/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/baseline_real_backforth`. Both player, recorder, monitor, and launch wrappers exited zero after map save and controlled shutdown.
- Blockers: None for Checkpoint 0. Internal scan-stage timing is not available in the original estimator; later checkpoints must add optional low-overhead timing counters before claiming an internal latency comparison.
- Commit: `a7b9610` (`test: record UWFL2 loop-closure baseline`).

## Checkpoint 1: Optional Infrastructure, Keyframes, And Full-SE(3) Graph

### Work

- [x] Add `loop_closure.enable`, default `false`, and no-op lifecycle wiring.
- [x] Defer tree-pointer refactoring until the shadow-tree checkpoint so the disabled front-end remains byte-equivalent.
- [x] Add immutable keyframe records, bounded queues, counters, and asynchronous worker lifecycle.
- [x] Add full-SE(3) graph nodes, first-pose prior, and consecutive odometry factors.
- [x] Add unit tests for frame direction, Pose3 conversion, covariance extraction, keyframe selection, queue bounds, and clean shutdown.
- [x] Repeat disabled-mode baseline and compare it to Checkpoint 0.

### Tests and commands

```bash
cd "$UWFL2_WS"
colcon build --packages-select fast_lio --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --verbose
python3 "$UWFL2_SRC/tools/compare_lc_runs.py" \
  --baseline "$LC_RESULTS/baseline_sim3" \
  --candidate "$LC_RESULTS/cp1_disabled_sim3" \
  --require-no-loop --check-message-counts --check-timestamps --check-map
```

Smoke replay before the full regression:

```bash
python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label cp1_keyframes_smoke --domain-id 73 --rate 1.0 --duration 180 \
  --bag "$BAG_SIM" --config "$UWFL2_SRC/config/sim.yaml" \
  --loop-closure true --detection false \
  --output "$LC_RESULTS/cp1_keyframes_smoke"
```

### Acceptance

- Disabled mode meets the baseline contract.
- Keyframe poses and odometry factors pass full-SE(3) synthetic tests.
- Front-end queues remain bounded and no sensor messages are dropped due to backend work.

### Log

- Decisions: Use stock GTSAM 4.2 and its public iSAM2 API; keep all graph work on a bounded background queue; copy a scan only after keyframe selection; preserve the original `KD_TREE` representation until the shadow-tree checkpoint.
- Commands run: isolated `colcon build` and `colcon test` with `BUILD_TESTING=ON`; 25 s enabled x5 smoke replay; complete disabled x5 replay of `sim3`; `compare_lc_runs.py` against the immutable x1 Checkpoint-0 baseline.
- Results: All seven loop-closure unit tests pass. The enabled smoke processed 5/5 keyframes with zero failures/drops and a maximum graph insertion time of 2.54 ms. The complete disabled replay produced 5,945 odometry and 5,944 registered-cloud messages, identical header stamps and serialized poses, and the identical 5,739,064-point map SHA-256. Its ATE RMSE/max/final remained 0.070/0.156/0.081 m.
- Blockers: No implementation blocker. The build currently uses the official Ubuntu GTSAM packages from an isolated `/tmp` prefix; install `libgtsam-dev` normally on deployment hosts.
- Timing: Complete x5 disabled replay CPU mean/p95/max 46.7/66.7/82.3% and RSS mean/p95/max 349.3/481.1/482.9 MB. The x1-to-x5 ROS-time lag ratio is not a real-time regression metric; exact output equivalence is the disabled-mode criterion.
- Commit: `a7a8f9f` (`feat: add optional SE3 keyframes and pose graph`)

## Checkpoint 2: Manually Injected Full-SE(3) Loop

### Work

- [x] Add a test-only `/uwfl2_lc/inject_loop` service with explicit from/to IDs and a full relative pose/covariance.
- [x] Derive the simulation loop transform from ground truth at the selected keyframe timestamps; never assume identity or planar closure.
- [x] Optimize a temporary graph, validate it, then commit only the graph result. Do not correct the live state/tree yet.
- [x] Publish raw/optimized graph paths and loop diagnostics.
- [x] Test a valid loop, reversed transform, impossible transform, stale IDs, and non-finite data.

### Commands

```bash
python3 "$UWFL2_SRC/tools/make_manual_loop.py" \
  --bag "$BAG_SIM" \
  --keyframes "$LC_RESULTS/cp1_keyframes_smoke/keyframes.csv" \
  --select-return-pair --output /tmp/uwfl2_manual_loop.yaml

python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label cp2_manual_loop --domain-id 74 --rate 1.0 \
  --bag "$BAG_SIM" --config "$UWFL2_SRC/config/sim.yaml" \
  --loop-closure true --detection false \
  --inject-loop /tmp/uwfl2_manual_loop.yaml \
  --output "$LC_RESULTS/cp2_manual_loop"

python3 "$UWFL2_SRC/tools/check_pose_graph.py" \
  --run "$LC_RESULTS/cp2_manual_loop" --require-full-se3 \
  --require-one-accepted-loop --check-residuals
```

### Acceptance

- The valid loop reduces graph closure residual.
- Invalid loops are rejected without changing the committed graph.
- Live UWFL2 odometry and tree remain unchanged in this checkpoint.

### Log

- Decisions: Define every loop as `T_from_to = inverse(T_local_from) T_local_to`; use GTSAM tangent ordering `[rotation, translation]`; recover the simulator's IKF vehicle pose as `T_W_BROV T_BROV_imu` rather than assuming the ground-truth child is the IMU frame; validate with a temporary Levenberg--Marquardt graph and rebuild stock iSAM2 only after acceptance.
- Commands run: isolated build and 10-test GTest suite; `make_manual_loop.py` on `sim3`; 60 s x5 ROS replay with `/uwfl2_lc/inject_loop`; `check_pose_graph.py --require-full-se3 --require-one-accepted-loop --check-residuals`.
- Results: The ROS smoke processed 13/13 keyframes with zero drops/failures. One full-SE(3) loop was accepted; graph error fell from 0.4895 to 0.00183, translation residual from 0.0493 m to 0.000119 m, and rotation residual from 0.0838 deg to 0.0161 deg. Reversed, stale, and non-finite unit constraints are rejected without changing graph version, factors, or optimized poses. Live IKF state and active map remain unchanged in this checkpoint.
- Blockers: None. A diagnostics mutex self-deadlock found by the first service smoke was fixed and the replay was repeated successfully.
- Timing: Accepted 13-node loop transaction 0.94 ms; incremental graph insertion maximum 0.93 ms.
- Commit: `be06aa7` (`feat: validate manually injected SE3 loops`)

## Checkpoint 3: Corrected Historical Map And Shadow ikd-Tree

### Work

- [x] Reconstruct historical points from immutable local-frame keyframe clouds and optimized poses.
- [x] Build a 3D keyframe-position index and load only keyframes inside the configured corrected-map radius.
- [x] Match original `filter_size_map` downsampling semantics.
- [x] Build a shadow `KD_TREE<PointType>` asynchronously with graph/tree generation tags.
- [x] Do not expose or swap the shadow tree yet.
- [x] Test identity reconstruction, known full-SE(3) corrections, voxel determinism, stale generation rejection, and bounded memory.

### Commands

```bash
cd "$UWFL2_WS"
colcon build --packages-select fast_lio --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --verbose
python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label cp3_shadow_tree --domain-id 75 --rate 1.0 \
  --bag "$BAG_SIM" --config "$UWFL2_SRC/config/sim.yaml" \
  --loop-closure true --detection false \
  --inject-loop /tmp/uwfl2_manual_loop.yaml \
  --output "$LC_RESULTS/cp3_shadow_tree"
```

### Acceptance

- Identity correction reconstructs the baseline historical map within tolerance.
- Shadow-tree queries match the corrected point set.
- Front-end scan latency and active tree are unchanged.

### Log

- Decisions: Select keyframes with a true 3D position index around the latest optimized pose; reconstruct with `T_local_vehicle T_vehicle_sonar`; reproduce ikd-tree insertion downsampling by retaining the point nearest each voxel center; reject rather than truncate when the configured input-point budget is exceeded. Tree advancement does not invalidate a shadow build because the corrected tree is independent of the old active tree; graph-version advancement does.
- Commands run: isolated build and 15-test GTest suite; 60 s x5 ROS replay with the Checkpoint-2 manual loop; pose-graph checker; exact output comparison against Checkpoint 2.
- Results: The ROS replay rebuilt one ready shadow tree from 13 keyframes and 10,964 input points into 3,346 deterministic map points, with zero failed/stale builds. Active-tree odometry, timestamps, and saved-map hash remained exactly identical to Checkpoint 2. Unit tests cover identity/arbitrary full-SE(3) transforms, radius/index behavior, voxel determinism, ikd-tree queries, graph-generation rejection, and point-budget enforcement.
- Blockers: None.
- Timing: Reconstruction 0.77 ms, downsampling 0.87 ms, ikd-tree build 29.94 ms, total 31.57 ms; estimated ikd-tree node memory 0.59 MB for this smoke window.
- Commit: `5344d4b` (`feat: rebuild corrected map in a shadow ikd-tree`)

## Checkpoint 4: Latest-Scan Re-Registration

### Work

- [x] Snapshot the latest processed scan and raw pose without blocking sensor callbacks.
- [x] Propagate the delayed graph correction from its anchor to the scan timestamp.
- [x] Register that scan to the shadow tree in full SE(3), initialized by the graph correction.
- [x] Require improved point-to-plane residual, enough effective points, convergence, and a finite covariance.
- [x] Reject poor registration without changing state/tree.
- [x] Test x/y/z translation and roll/pitch/yaw perturbations independently and together.

### Commands

```bash
cd "$UWFL2_WS"
colcon build --packages-select fast_lio --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --verbose
python3 "$UWFL2_SRC/tools/check_reregistration.py" \
  --run "$LC_RESULTS/cp4_reregistration" \
  --require-residual-decrease --require-full-se3
```

### Acceptance

- Synthetic full-SE(3) cases converge within specified translation/rotation tolerances.
- Real replay registration reduces residual and does not pause the front end beyond the recorded budget.

### Log

- Decisions: Use UWFL2's additive-position/right-SO(3) perturbation and a six-dimensional point-to-plane solve; reject rank-deficient geometry; propagate delayed correction by left-composing the latest raw pose with the optimized/raw anchor transform. Mean robust residual is the acceptance cost; p95 remains diagnostic because correspondence changes can move it by sub-millimeter amounts.
- Commands run: isolated build and 16-test suite; 60 s x5 `sim3` replay with manual loop; registration diagnostic check.
- Results: Synthetic x/y/z and roll/pitch/yaw errors, separately and combined, converge in full SE(3) with finite PSD covariance. ROS smoke used 889/890 effective points and reduced mean residual from 0.02538 to 0.02524 m; no state or active-tree change occurs yet.
- Blockers: None.
- Timing: ROS registration 7.69 ms for five iterations.
- Commit: pending, message `feat: reregister latest scan against corrected map`

## Checkpoint 5: Atomic State, Covariance, And Tree Swap

### Work

- [ ] Implement the front-end-only transaction described above.
- [ ] Transport all 27x27 covariance blocks with an analytic, finite-difference-tested Jacobian.
- [ ] Preserve gravity, biases, auxiliary references, and extrinsics according to their frames.
- [ ] Refresh all pose/map caches and insert no point into a mismatched generation.
- [ ] Add rollback tests for stale versions, failed PSD checks, registration rejection, and exceptions.
- [ ] Add stress tests for concurrent job production and repeated accepted/rejected corrections.
- [ ] Repeat the complete disabled-mode baseline.

### Commands

```bash
cd "$UWFL2_WS"
colcon build --packages-select fast_lio --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --verbose
python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label cp5_manual_commit --domain-id 76 --rate 1.0 \
  --bag "$BAG_SIM" --config "$UWFL2_SRC/config/sim.yaml" \
  --loop-closure true --detection false \
  --inject-loop /tmp/uwfl2_manual_loop.yaml \
  --output "$LC_RESULTS/cp5_manual_commit"

python3 "$UWFL2_SRC/tools/compare_lc_runs.py" \
  --baseline "$LC_RESULTS/baseline_sim3" \
  --candidate "$LC_RESULTS/cp5_manual_commit" \
  --check-trajectory --check-map --check-state-invariants --check-psd
```

### Acceptance

- The valid manual loop changes state/tree exactly once and improves closure/map consistency.
- Protected state blocks remain physically consistent; covariance is finite, symmetric, and PSD.
- Failed transactions leave byte-for-byte-equivalent state/tree generation metadata.
- Disabled mode still meets the baseline contract.

### Log

- Decisions: TODO
- Commands run: TODO
- Results: TODO
- Blockers: TODO
- Timing: commit pause, rollback time, scan p50/p95/max.
- Commit: TODO, suggested message `feat: atomically commit loop-corrected state and map`

## Checkpoint 6: Automatic STD Detection And False-Loop Rejection

### Work

- [ ] Port the ROS-independent LTA-OM STD subset with attribution: voxel planes, binary descriptors, triangle descriptors, candidate retrieval, RANSAC/fine matching, and geometric verification.
- [ ] Exclude unused ROS 1 UI, bag, Ceres BA/ICP, and patched-GTSAM code unless a measured need is documented.
- [ ] Tune sonar STD parameters from recorded descriptor/keypoint/overlap distributions.
- [ ] Add the transactional graph checks and high-leverage-loop policy.
- [ ] Add loop statistics: proposed, descriptor-rejected, geometry-rejected, graph-rejected, registration-rejected, accepted, stale, and superseded.
- [ ] Test true revisits, non-revisit segments, reversed traversal, sparse scans, flat seabed, parallel walls, and synthetic false candidates.

### Commands

```bash
cd "$UWFL2_WS"
colcon build --packages-select fast_lio --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select fast_lio --event-handlers console_direct+
colcon test-result --verbose

python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label cp6_auto_sim3 --domain-id 77 --rate 1.0 \
  --bag "$BAG_SIM" --config "$UWFL2_SRC/config/sim.yaml" \
  --loop-closure true --detection true \
  --output "$LC_RESULTS/cp6_auto_sim3"

python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label cp6_auto_real_backforth --domain-id 78 --rate 1.0 \
  --bag "$BAG_REAL" --config "$UWFL2_SRC/config/backAndforth_CSSN3_processed.yaml" \
  --loop-closure true --detection true \
  --output "$LC_RESULTS/cp6_auto_real_backforth"
```

### Acceptance

- Simulation accepts at least one ground-truth-consistent loop and no ground-truth false loop.
- Real bag loop acceptance is supported by STD geometry, overlap, graph residual, and re-registration evidence.
- No accepted loop relies on a planar assumption.
- Backend queues stay bounded and the front end remains real-time.

### Log

- Decisions: TODO
- Commands run: TODO
- Results: TODO
- Blockers: TODO
- Timing: STD stages, graph, rebuild, registration, queue age, peak RAM.
- Commit: TODO, suggested message `feat: add asynchronous STD loop closure and rejection`

## Checkpoint 7: Baseline Versus UWFL2-LC Evaluation

### Work

- [ ] Re-run disabled UWFL2 and enabled UWFL2-LC sequentially on the exact same simulation and real bags at x1.
- [ ] Keep every non-loop parameter identical and archive resolved parameter dumps.
- [ ] Compare trajectory/map consistency, loop statistics, CPU, RAM, scan latency, graph time, tree-rebuild time, registration/commit time, queue drops, callback drops, and rosbag output counts.
- [ ] Run the same commands on the Jetson Orin Nano.
- [ ] Record every failed run alongside successful runs.

### Final commands

```bash
python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label final_uwfl2_disabled_sim3 --domain-id 81 --rate 1.0 \
  --bag "$BAG_SIM" --config "$UWFL2_SRC/config/sim.yaml" \
  --loop-closure false --output "$LC_RESULTS/final_uwfl2_disabled_sim3"

python3 "$UWFL2_SRC/tools/run_lc_benchmark.py" \
  --label final_uwfl2_lc_sim3 --domain-id 82 --rate 1.0 \
  --bag "$BAG_SIM" --config "$UWFL2_SRC/config/sim.yaml" \
  --loop-closure true --detection true \
  --output "$LC_RESULTS/final_uwfl2_lc_sim3"

python3 "$UWFL2_SRC/tools/compare_lc_runs.py" \
  --baseline "$LC_RESULTS/final_uwfl2_disabled_sim3" \
  --candidate "$LC_RESULTS/final_uwfl2_lc_sim3" \
  --ground-truth-topic /auv/pose_actual \
  --report "$LC_RESULTS/final_sim3_comparison"

python3 "$UWFL2_SRC/tools/compare_lc_runs.py" \
  --baseline "$LC_RESULTS/final_uwfl2_disabled_real" \
  --candidate "$LC_RESULTS/final_uwfl2_lc_real" \
  --no-ground-truth --map-overlap --closure-consistency \
  --report "$LC_RESULTS/final_real_comparison"
```

### Required final report

| Metric | UWFL2 disabled | UWFL2-LC | Difference |
| --- | ---: | ---: | ---: |
| ATE RMSE/max/final on simulation | TODO | TODO | TODO |
| RPE translation/rotation | TODO | TODO | TODO |
| Start/end full-SE(3) closure error | TODO | TODO | TODO |
| Overlap point-to-plane/map consistency | TODO | TODO | TODO |
| Loop proposed/accepted/rejected/false | TODO | TODO | TODO |
| CPU mean/p95/peak | TODO | TODO | TODO |
| RAM mean/peak | TODO | TODO | TODO |
| Scan latency p50/p95/max | TODO | TODO | TODO |
| Graph optimization p50/p95/max | N/A | TODO | TODO |
| Tree rebuild p50/p95/max | N/A | TODO | TODO |
| Re-registration and commit p50/p95/max | N/A | TODO | TODO |
| Sensor/backend queue drops | TODO | TODO | TODO |
| Published/recorded message counts | TODO | TODO | TODO |

### Acceptance

- Disabled mode reproduces the original UWFL2 baseline.
- Accepted loop closures improve or preserve trajectory and map consistency without corrupting any UWFL2 state block.
- No hidden sensor, frame, timestamp, or state assumption remains.
- Jetson Orin Nano resource and latency results demonstrate sustained real-time operation.

### Log

- Decisions: TODO
- Commands run: TODO
- Results: TODO
- Blockers: current development host is x86_64, not a Jetson Orin Nano. Final Jetson acceptance cannot be marked complete until the same x1 runs are executed on that target.
- Timing: TODO
- Commit: TODO, suggested message `test: compare UWFL2 baseline and loop closure`

## Stop Conditions

Stop at the active checkpoint and record the exact evidence when any of the following occurs:

- GTSAM cannot be provided reproducibly for ROS 2 Jazzy and Jetson aarch64.
- A transform direction or frame cannot be established from code, messages, CAD, or a controlled test.
- Disabled mode changes estimator output beyond tolerance.
- A correction requires an undocumented modification of gravity, bias, reference, or covariance state.
- The latest-scan registration does not reduce residual or is underconstrained in full SE(3).
- Atomic commit cannot keep state and tree generations consistent.
- STD cannot produce enough stable sonar keypoints without unacceptable false loops.
- p95 scan latency, memory, or dropped-message behavior violates the measured real-time budget.

Do not bypass a stop condition with gates, planar assumptions, hidden tuning, or an undocumented state reset.
