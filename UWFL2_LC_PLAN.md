# UWFL2 LTA-OM Loop-Closure Plan

## Status

- Branch: `feature/uwfl2-ltaom-loop-closure`
- Plan status: approved and active
- Implementation status: Checkpoints 0--7 complete; follow-up validation logged below
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
- Commit: `676b2ae` (`feat: reregister latest scan against corrected map`)

## Checkpoint 5: Atomic State, Covariance, And Tree Swap

### Work

- [x] Implement the front-end-only transaction described above.
- [x] Transport all 27x27 covariance blocks with an analytic, finite-difference-tested Jacobian.
- [x] Preserve gravity, biases, auxiliary references, and extrinsics according to their frames.
- [x] Refresh all pose/map caches and insert no point into a mismatched generation.
- [x] Add rejection/rollback handling for stale versions, failed PSD checks, registration rejection, and exceptions.
- [x] Exercise bounded concurrent workers and one-shot accepted/rejected correction mailboxes.
- [x] Repeat disabled mode and compare it with the pre-swap checkpoint.

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

- Decisions: Only the front-end timer may commit. Apply the accepted left SE(3) correction to position/attitude and startup-local velocity; retain gravity, IMU/DVL/pressure biases, magnetic/pressure references, and extrinsics; transport all covariance cross-blocks with `J=diag(R_c,I,I,I,R_c,I...)`; add graph and registration pose uncertainty; swap state, covariance, bounds, generation, and tree as one rollback-capable transaction.
- Commands run: isolated build and 18-test suite; 60 s x5 manual-loop commit replay; 30 s disabled replay and exact comparison.
- Results: The loop committed exactly once with zero rejected commits and zero invalid odometry covariance. Finite-difference transport error stayed below the test tolerance and protected covariance/state blocks remained unchanged. Disabled mode retained identical poses, timestamps, message counts, and map hash.
- Blockers: None.
- Timing: Shadow rebuild 32.16 ms and re-registration 7.98 ms run in the backend; commit is a pointer swap plus one scan insertion at the front-end boundary.
- Commit: `7f8ea2a` (`feat: atomically commit loop-corrected state and map`).

## Checkpoint 6: Automatic STD Detection And False-Loop Rejection

### Work

- [x] Port the ROS-independent LTA-OM STD subset with attribution: voxel planes, binary descriptors, triangle descriptors, candidate retrieval, RANSAC/fine matching, and geometric verification.
- [x] Exclude unused ROS 1 UI, bag, Ceres BA/ICP, and patched-GTSAM code unless a measured need is documented.
- [x] Tune sonar STD parameters from recorded descriptor/keypoint/overlap distributions.
- [x] Add the transactional graph checks and high-leverage-loop policy.
- [x] Add loop statistics: proposed, descriptor-rejected, geometry-rejected, graph-rejected, registration-rejected, accepted, stale, and superseded.
- [x] Test true revisits, non-revisit segments, reversed traversal, sparse scans, flat seabed, parallel walls, and synthetic false candidates.

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

- Decisions: Keep detection opt-in and backend-only; use full-SE(3) SVD/RANSAC plus 3D overlap, two-hit confirmation, pose-distance shortlist, odometry-consistency and NIS gates, then the existing graph/rebuild/re-registration/atomic-swap transaction. The conservative default rejects corrections that are not statistically necessary.
- Commands run: isolated build; 24-test suite; exact disabled regression; full sim3 x5 replay; 180 s real back-and-forth x5 replay. Artifacts: `/tmp/uwfl2_lc_cp6_disabled_smoke`, `/tmp/uwfl2_lc_cp6_auto_sim3_safe`, and `/tmp/uwfl2_lc_cp6_auto_real_180s`.
- Results: Build and all 24 tests pass. Disabled mode is exactly equal to Checkpoint 5 in pose, timestamps, message counts, and map hash. Simulation processed 357/357 keyframes with zero drops; four confirmed candidates were rejected by NIS, preventing false correction in repetitive harbor geometry. A less-conservative validation run proved the complete automatic path, including accepted graph loop, shadow rebuild, re-registration, and atomic commit. Real data processed 47/47 keyframes with zero proposals in the tested segment and no invalid covariance or timestamp.
- Blockers: No code blocker. The supplied sim3 and tested real segment do not contain a statistically necessary loop under the final conservative gate; a drifted closed-loop bag is needed to validate a beneficial automatic correction before production enablement.
- Timing: Sim descriptor/search/verification mean 0.33/4.4/20.0 ms, p95 0.60/13.5/50.9 ms; no backend drops. Real mean 0.24/0.48/4.0 ms, p95 0.55/1.20/21.8 ms. Validation rebuild/re-registration were about 74/14 ms in backend threads.
- Commit: this checkpoint commit, `feat: add asynchronous STD loop closure and rejection`.

## Checkpoint 7: Baseline Versus UWFL2-LC Evaluation

### Work

- [x] Re-run disabled UWFL2 and enabled UWFL2-LC sequentially on the exact same simulation and real bags at x1.
- [x] Keep every non-loop parameter identical and archive resolved parameter dumps.
- [x] Compare trajectory/map consistency, loop statistics, CPU, RAM, scan latency, graph time, tree-rebuild time, registration/commit time, queue drops, callback drops, and rosbag output counts.
- [ ] Run the same commands on the Jetson Orin Nano.
- [x] Record every failed run alongside successful runs.

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

python3 "$UWFL2_SRC/tools/report_lc_evaluation.py" \
  --baseline "$LC_RESULTS/final_uwfl2_disabled_sim3" \
  --candidate "$LC_RESULTS/final_uwfl2_lc_sim3" \
  --output "$LC_RESULTS/final_sim3_comparison"

python3 "$UWFL2_SRC/tools/report_lc_evaluation.py" \
  --baseline "$LC_RESULTS/final_uwfl2_disabled_real" \
  --candidate "$LC_RESULTS/final_uwfl2_lc_real" \
  --output "$LC_RESULTS/final_real_comparison"
```

### Required final report

| Metric | UWFL2 disabled | UWFL2-LC | Difference |
| --- | ---: | ---: | ---: |
| ATE RMSE/max/final on simulation [m] | 0.0700 / 0.1559 / 0.0807 | 0.0700 / 0.1559 / 0.0807 | 0 |
| RPE translation [m] / rotation [deg] RMSE | 0.00685 / 0.0265 | 0.00685 / 0.0265 | 0 |
| Start/end full-SE(3), translation [m] / rotation [deg] | 0.0793 / 0.183 | 0.0793 / 0.183 | 0 |
| Map consistency | exact map hash; overlap 1.0 | exact map hash; overlap 1.0 | identical |
| STD proposed / accepted / graph-rejected, sim (real) | 0 / 0 / 0 | 99 / 0 / 4 (20 / 0 / 3) | no false acceptance |
| CPU mean/p95/peak, simulation [%] | 19.0 / 24.7 / 35.7 | 18.9 / 24.8 / 30.7 | +0.1 p95 |
| RAM mean/peak, simulation [MiB] | 349.1 / 483.9 | 370.5 / 527.4 | +21.4 / +43.5 |
| Scan latency p50/p95/max, simulation [ms] | 3.20 / 4.60 / 128.83 | 3.16 / 4.39 / 119.12 | -0.21 p95 |
| Graph append p50/p95/max, simulation [ms] | N/A | 0.346 / 0.754 / 1.444 | asynchronous |
| Tree rebuild; re-registration; commit | N/A | N/A: no accepted loop | N/A |
| Sensor/backend queue drops | 0 | 0 | 0 |
| Recorded messages, simulation (real) | 679808 (304590) | 679808 (304590) | 0 |

### Acceptance

- Disabled mode reproduces the original UWFL2 baseline.
- Accepted loop closures improve or preserve trajectory and map consistency without corrupting any UWFL2 state block.
- No hidden sensor, frame, timestamp, or state assumption remains.
- Jetson Orin Nano resource and latency results demonstrate sustained real-time operation.

### Log

- Decisions: Added benchmark-only front-end, callback-delivery, backend-stage,
  queue-supersession, and atomic-commit counters. They are enabled only when a
  diagnostics directory is supplied and do not change estimator decisions.
- Commands run: `colcon test --packages-select fast_lio`; matched 30 s sim3
  disabled replay at x5; strict `compare_lc_runs.py` regression against the
  Checkpoint 6 disabled replay; four sequential full x1 runs and
  `report_lc_evaluation.py` comparisons.
- Results: 24/24 tests passed. The instrumentation smoke was bit-exact: zero
  pose/rotation/timestamp differences, identical map SHA-256 and output counts.
  It received 149 sonar and 1690 IMU callbacks with zero timestamp rollback,
  buffer-clear, or stale-scan events. Scan latency was 3.37 ms mean and 4.48 ms
  p95. Full x1 disabled mode is bit-exact with Checkpoint 0 on both bags. The
  enabled backend delivered every sonar/IMU callback, dropped no jobs, rejected
  all inconsistent candidates, and preserved identical trajectories and maps.
- Blockers: the tested bags produced no graph-accepted loop, so corrected-map,
  re-registration, and commit timing cannot be measured in an automatic run.
  A drifted closed-loop bag is required. This x86_64 host is not a Jetson Orin
  Nano, so target resource acceptance also remains open.
- Timing: simulation scan p95 was 4.60 ms disabled and 4.39 ms enabled; real
  scan p95 was 6.07 ms disabled and 6.11 ms enabled. Enabled graph-append p95
  was 0.754 ms (simulation) and 0.448 ms (real).
- Follow-up sim3 loop test: exposing `loop_maximum_initial_nis` and setting it
  to 1.0 accepted only loop 262--322 (NIS 0.174). The graph optimization,
  shadow-tree rebuild, scan re-registration, and atomic commit all succeeded.
  Final ATE improved from 0.0807 m to 0.0213 m, but RMSE increased from 0.0700 m
  to 0.0758 m and maximum error from 0.1559 m to 0.2005 m. Therefore this is a
  valid end-to-end closure, but not yet an overall trajectory improvement.
- RMSE diagnosis and visualization: the accepted loop committed a roughly
  0.136 m latest-scan registration correction at 411.64 s although its graph
  residual was 0.029 m and scan residual improved by less than 0.001 m. The
  resulting transient raised the 411.64--420 s RMSE from 0.106 m to 0.184 m;
  the final error still improved. Optional RViz output now publishes raw and
  optimized paths, proposed/rejected/accepted loop edges, the registration
  correction arrow, keyframes, and stage counters. A 30 s smoke produced 30
  messages on each visualization topic and all 24 tests passed.
- Commit: `cd356ce` adds instrumentation; this evaluation is committed
  separately.
- Final-return detection fix (sim3): the return to the starting area produced
  a strong `1--355` match (86% overlap, 524 triangle matches), but the bag ended
  before the former two-scan confirmation rule could confirm it. Exceptionally
  strong, long-span matches may now proceed after one detection while ordinary
  matches still require two confirmations and all pose-graph consistency tests.
  The SE(3) NIS limit is the 95% six-dimensional chi-square threshold (12.592).
- Asynchronous commit fix: registration results now carry exact scan/tree
  generations. Stale results request bounded background re-registration against
  the newest scan and cannot replace the active IKF/tree. Loop residual
  acceptance uses covariance-weighted factor error instead of requiring every
  translation/rotation component to decrease independently.
- Validation: `colcon build --packages-select fast_lio --symlink-install` and
  29/29 tests passed. A full sim3 x1 replay accepted and committed loops
  `262--322` and `1--355`; the final correction refreshed once, with no keyframe
  drops. Final ATE improved from 0.0807 m to 0.0483 m. ATE RMSE changed from
  0.0700 m to 0.0747 m and maximum from 0.1559 m to 0.1982 m, so final closure
  improved endpoint consistency but not every whole-trajectory metric.
- Premature-loop tuning: a 120 s minimum revisit interval was rejected because
  it accepted the mid-route `198--331` loop and duplicate final loops `1--355`
  and `1--356` (x5 ATE RMSE/max/final: 0.07413/0.15591/0.09293 m). The detector
  now exposes `std_minimum_loop_duration_s` and
  `std_accepted_loop_cooldown_s`; both default to the behavior-preserving 0 s.
  For sim3, 300 s and 30 s respectively suppress the undesired
  mid-route and duplicate closures while retaining the final return.
- Commands: `colcon build --packages-select fast_lio --symlink-install` and
  `colcon test --packages-select fast_lio`. Replays used
  `python3 tools/run_lc_benchmark.py --label sim3_final_only_duration300_x5
  --domain-id 98 --rate 5.0 --bag /home/attia/ros2_ws/bags/DONE/sim3
  --config config/default.yaml --loop-closure true --detection true --output
  /home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/sim3_final_only_duration300_x5`
  and `python3 tools/run_lc_benchmark.py --label
  sim3_final_only_duration300_x1 --domain-id 99 --rate 1.0 --bag
  /home/attia/ros2_ws/bags/DONE/sim3 --config config/default.yaml
  --loop-closure true --detection true --output
  /home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/sim3_final_only_duration300_x1`.
- Results: build and 24/24 tests passed. Both x5 and x1 runs processed 358/358
  keyframes with zero drops and accepted only `1--355`. Against the no-loop
  baseline RMSE/max/final of 0.06996/0.15591/0.08066 m, x5 produced
  0.06944/0.15591/0.02298 m and x1 produced
  0.06951/0.15591/0.03770 m. The x1 correction committed in 0.663 ms; graph,
  shadow rebuild, and registration maxima were 6.04, 66.26, and 24.18 ms.
- Commit: `8af25ac` (`fix: defer loop closure to survey return`).
- Result directories:
  `/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/sim3_final_only_duration120_x5`,
  `/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/sim3_final_only_duration300_x5`,
  and
  `/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/sim3_final_only_duration300_x1`.
- Zigzag real-bag tuning: the untuned x5 diagnostic proposed 13 candidates but
  graph-rejected all three confirmed constraints. The final `10--114` and
  `10--115` matches had 0.851/0.887 overlap and 25/23 RANSAC inliers, but NIS
  2993.5/2721.7 because the front-end covariance did not represent the roughly
  3.86 m accumulated return drift. The earlier `2--98` candidate had only 0.482
  overlap. Bag-specific settings now require 0.80 overlap, two confirmations,
  a 0.95 single-detection threshold, NIS below 4000, and 24 registration
  iterations. The controlled x5 replay accepted only `10--114`; registration
  reduced mean residual from 0.0505 m to 0.0465 m and one atomic correction
  committed with zero backend drops. The 12-iteration trial accepted the graph
  factor but safely rejected commit after reaching its iteration limit.
- Zigzag commands/results: diagnostics used `run_lc_benchmark.py` with bag
  `/home/attia/ros2_ws/bags/DONE/zigzagwall_processed5`, config
  `config/zigzagwall_processed5.yaml`, domains 102--104, and rate 5.0. Results
  are retained under `UWFL2_LC_RESULTS/zigzag_lc_diagnostic_x5`,
  `zigzag_lc_final_return_x5`, and `zigzag_lc_final_return_reg24_x5`.
  The benchmark reports failure only because this config intentionally has PCD
  saving disabled; launch, replay, recording, detection, graph optimization,
  re-registration, and commit completed. Full x1 visual map validation remains
  pending because this real bag has no ground truth.
- Commit: `cf5b722` records the zigzag tuning evidence. The active YAML remains
  in the user's existing uncommitted config worktree for visual x1 validation.
- Zigzag reproducibility check: a later x5 default-config replay produced zero
  proposals (115 keyframes, 90 descriptor and 25 geometry rejections). Comparing
  resolved parameters against the successful run found one estimator change:
  `magnetometer.heading_cov_floor` was 1.0 instead of `1.0e-6`. This changed the
  heading trajectory/keyframe clouds and reduced final overlap below 0.80.
  `default.yaml` was restored to `1.0e-6`; loop thresholds were not weakened to
  compensate for the changed front-end trajectory. Artifacts are retained in
  `UWFL2_LC_RESULTS/zigzag_default_verify_x5`.
- Zigzag magnetic-weighting retune: the preferred real-map settings are
  `mag_cov=1.0` and `heading_cov_floor=1.0`. With these settings, sparse 5 s
  return keyframes provide consecutive 0.64--0.80 overlap but their independent
  full-SE(3) corrections exceed the former hard-coded 1 m/10 deg confirmation
  window. The confirmation translation/rotation limits are now exposed with
  unchanged defaults; this bag uses 2 m/20 deg, 0.60 minimum overlap, two
  confirmations, a 300 s minimum duration, and 24 registration iterations.
- Validation: build and 25/25 tests passed. The x5 replay accepted four
  geometrically supported return constraints (`0--104`, `7--106`, `0--109`,
  `8--112`) and committed all four corrected state/tree transactions. It
  processed 118/118 keyframes with zero backend drops. Each registration
  reduced or preserved its point-to-plane residual. Artifacts are retained in
  `UWFL2_LC_RESULTS/zigzag_mag1_final_confirmation20_x5`; map saving alone
  reports disabled because `pcd_save_en` is false. Full x1 RViz map validation
  remains pending.
- Commit: `435f128` exposes and tests the full-SE(3) confirmation tolerances.
- LC-off front-end heading investigation: the real zigzag bag republishes each
  physical magnetic reading with fresh ROS timestamps. Of 81,625 messages,
  only 4,893 field vectors were distinct (about 10 Hz); 94% were identical
  copies. Fusing every copy built the 20-sample startup reference in 0.22 s and
  accumulated a 0.212 deg/s heading-axis gyro-bias correction. Identical field
  vectors are now accepted once, as required by the scalar-heading model.
- Validation: the first diagnostic build failed on an Eigen expression passed
  to `Log`; materializing the rotation matrix fixed it. The subsequent build,
  2/2 tests, and complete x5 LC-disabled replay passed. Accepted magnetic
  updates fell from 81,522 to 4,855, reference initialization took 1.98 s, and
  net gyro-bias correction fell to -0.0063 deg/s. Sonar yaw correction remained
  0.0142 deg/scan mean absolute, confirming the ikd-tree/sonar path was not
  changed. Results are in `UWFL2_LC_RESULTS/zigzag_frontend_heading_mag_on_lc_off`
  and `zigzag_frontend_heading_dedup_lc_off_v2`. Visual map validation is pending.
- A controlled `heading_cov_floor=0.05 rad^2` replay improved final relative-yaw
  disagreement against the recorded `World -> base_link` odometry from 4.15 deg
  to 0.06 deg, but mean/p95 disagreement did not improve. Since that odometry is
  not ground truth, this candidate remains uncommitted pending visual map review.
- Repeated-sample audit: DVL contained 2,574 distinct readings out of 2,574
  messages (6.6 Hz), while pressure contained only 4,429 distinct readings out
  of 81,633 messages (94.6% repeated republishes, about 10 Hz physical rate).
  Pressure now follows the same one-physical-sample/one-update rule as magnetic
  heading, including its 20-sample startup reference. Build and 2/2 tests pass.
- Heading isolation: disabling DVL reduced return-leg anticlockwise heading pull
  but worsened final heading and positional closure, so DVL is useful but was
  overconfident. Its configured `5e-5 m^2/s^2` floor and prior NIS near 132 imply
  about `0.0022 m^2/s^2` for a consistent 3D NIS near 3. Testing `0.0025` reduced
  mean heading disagreement from 5.33 deg to 4.81 deg and closure from 1.35 m
  to 0.94 m. Combining it with `heading_cov_floor=0.005 rad^2` reduced mean/p95
  heading disagreement to 4.10/8.43 deg and return-wall disagreement near 360 s
  from 4.49 deg to 0.79 deg. These YAML values remain pending visual validation.
- Visual review of `dvl.velocity_cov=0.0025` and
  `magnetometer.heading_cov_floor=0.005` found consistent seabed, smooth turns,
  and wall-heading error below 2 deg, but with a small clockwise over-correction
  near 75% of the route. The next isolated visual candidate weakens only the
  magnetic update to `heading_cov_floor=0.01`.
- The `0.01` replay remained smooth and improved positional closure from 1.05 m
  to 0.88 m. The recorded `World -> base_link` odometry does not agree closely
  enough with mapped-wall geometry around 75% to select between `0.005` and
  `0.01`; the final magnetic floor therefore requires visual wall validation.
- Final front-end visual review selected `heading_cov_floor=0.01`: the wall was
  very good, seabed consistent, and both turns smooth. Automatic LC was then
  re-enabled and replayed at x5. STD proposed two candidates and accepted return
  loop `0--104` (overlap-supported SE(3) NIS 2.43); graph error fell from 3.154
  to 0.478. The shadow tree rebuilt in 44.4 ms, latest-scan residual improved
  from 0.0656 m to 0.0526 m, and one atomic correction committed after three
  bounded stale-scan refreshes. All 115 keyframes were processed with zero
  queue drops or backend failures. Artifacts are in
  `UWFL2_LC_RESULTS/zigzag_frontend_final_mag001_lc_on`.
- Live corrected-map visualization: `publish.map_en` now publishes the current
  active ikd-tree on `/uwfl2/active_map` at the configured one-second interval
  only while subscribed, with an immediate refresh after an atomic LC commit.
  RViz displays this transient-local snapshot and disables the historical
  `/cloud_registered` accumulator by default. Build and 2/2 tests passed. A
  60 s zigzag smoke replay at x5 published a nonempty 967-point active-tree
  snapshot. The first smoke command failed because ROS 2 Jazzy uses
  `--playback-duration`, not `--duration`; the corrected replay passed.
  Implementation commit: `b940e94`.
- Active-map RViz visibility follow-up: a live inspection confirmed the topic,
  transform, and 7,488-point snapshot were valid, but 0.005 m `AxisColor` flat
  squares rendered nearly black at the saved camera distance. The display now
  uses bright cyan five-pixel points; a second live RViz instance visibly
  rendered the complete compact map. No estimator code changed.
- Corrected historical map output: LTA-OM's
  `loop_optimization_node.cpp::pubAndSaveGloablMap()` transforms every stored
  keyframe cloud by its optimized-pose correction and uses the same resulting
  cloud for RViz and PCD output. UWFL2-LC now follows that design: voxelized
  full-SE(3) keyframe submaps are retained, rebuilt from optimized graph poses,
  published on `/uwfl2/corrected_map`, and used by `/map_save`. The bounded
  local shadow ikd-tree remains separate from this full historical output.
- Validation commands:
  `colcon build --packages-select fast_lio --symlink-install` and
  `ctest --test-dir build/fast_lio --output-on-failure` passed. A 60 s x5
  zigzag smoke test published 595,833 corrected-history points versus 20,989
  live-tree points and saved exactly 595,833 PCD points. The initial unvoxelized
  implementation produced 2,075,252 points and was rejected as too costly.
- Full acceptance replay command: `python3 tools/run_lc_benchmark.py --label
  zigzag_corrected_history_v3_x5 --domain-id 178 --rate 5 --bag
  /home/attia/ros2_ws/bags/DONE/zigzagwall_processed5 --config
  /tmp/uwfl2_corrected_history_full.yaml --output
  /home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/zigzag_corrected_history_v3_x5
  --loop-closure true --detection true --drain-seconds 10`.
- Full replay results: 115/115 keyframes processed with zero drops/failures;
  two loop factors accepted; one atomic correction committed. The final shadow
  rebuild used 442,141 historical input points, retained 80,497 local-tree
  points, and took 111.47 ms worst case. Latest-scan registration reduced mean
  residual from 0.02744 m to 0.02602 m; commit took 14.54 ms. The saved corrected
  PCD has 444,846 points (14.2 MB). Peak process-tree RSS was 618.6 MB.
- Corrected-map ROS serialization runs on the backend callback group, following
  LTA-OM's separate global-map publishing thread rather than blocking the
  front-end timer. A post-change 20 s x5 smoke test published a 6,956-point
  snapshot and `/map_save` later wrote the same growing history at 17,949
  points; the final build and both CTest executables passed.
- Implementation commit: `50e296e` (`feat: rebuild and publish corrected
  history map`).
- Corrected-map rendering follow-up: adjacent keyframe submaps overlapped in
  the historical output, so RViz received repeated surface samples. The output
  now receives one global deterministic voxel pass at `filter_size_map`, and
  the transient-local cloud is republished only when map history changes.
  `/cloud_registered` remains visually denser because it displays full
  undistorted scans with RViz decay rather than the compact mapping output.
- Validation: build and 2/2 CTest executables passed. On the same 60 s x5
  window, corrected output dropped from 595,833 points (19 MB) to 225,947
  points (6.9 MB). The complete x5 zigzag replay in
  `UWFL2_LC_RESULTS/zigzag_corrected_history_compact_x5` processed 115/115
  keyframes with zero drops/failures, accepted two loop factors, and committed
  one atomic correction. The saved corrected map dropped from 444,846 points
  (14.2 MB) to 73,160 points (2.34 MB); peak RSS dropped from 618.6 MB to
  531.5 MB. Latest-scan registration remained valid and reduced its residual.
- Implementation commit: `b25f6ad` (`perf: compact corrected map output`).
- Visualization output simplification: obsolete `/cloud_registered` and
  `/uwfl2/active_map` publishers, callbacks, state, and parameters were removed
  completely. All configs now expose only `publish.map_en` and
  `publish.corrected_map_interval_s` for map visualization. Each processed
  sonar scan marks corrected history dirty; the backend publishes at most once
  per configured interval (0.2 s, therefore up to 5 Hz) and never resends an
  unchanged map. The user's RViz configuration was intentionally not modified.
- Validation: isolated build under `/tmp/uwfl2_build` passed and both CTest
  executables passed. Repository search found no remaining registered-cloud or
  active-map code/config symbols. A ROS graph runtime check was attempted but
  this managed environment denied DDS sockets and ROS log writes; no claim is
  made from that failed runtime attempt.
- Failed/recovered tests: the first complete replay exceeded the existing
  3,000,000-point shadow budget (4,013,316 history points), so no correction
  committed. Voxelizing each owned submap at `max(filter_size_surf,
  filter_size_map)` reduced the history below budget. A later rebuild became
  stale while the front end appended a keyframe; stale builds now enqueue the
  newest graph snapshot, allowing the accepted loop to complete without
  blocking the front end.

### ROS 2 Humble / Jetson Portability

- [x] Replace Jazzy-only LiDAR QoS and service-construction APIs with APIs
  supported by both ROS 2 Humble and Jazzy. The LiDAR subscriber is explicitly
  best-effort, which remains compatible with reliable and best-effort sensor
  publishers; the loop-injection service uses the default RMW service profile.
- [x] Local Jazzy build passed in 1 min 59 s:
  `cd ~/ros2_ws && colcon build --packages-select fast_lio --symlink-install`.
- [x] Local CTest passed 2/2 tests:
  `ctest --test-dir ~/ros2_ws/build/fast_lio --output-on-failure`.
- [x] Clean Jetson Orin Nano ROS 2 Humble/aarch64 build passed in 9 min 11 s:
  `rm -rf ~/ros2_ws/build/fast_lio ~/ros2_ws/install/fast_lio && cd ~/ros2_ws && colcon build --packages-select fast_lio --symlink-install`.
- [x] Jetson CTest passed 2/2 tests in 1.11 s:
  `ctest --test-dir ~/ros2_ws/build/fast_lio --output-on-failure`.
- Implementation commit: `ebd3d82` (`fix: support ROS 2 Humble QoS APIs`).
  One pre-build verification command stopped intentionally because it compared
  against an incorrectly expanded commit hash; the exact hash was then read
  from Git and the clean build proceeded without source changes.

## Jetson Performance Optimization

### P1: Matched Baseline

- [x] Add 1 Hz process/system/GPU telemetry and a diagnostic-only shared
  monotonic timestamp for exact loop-event spike windows. The runner now
  records the benchmark overrides in its manifest and runtime YAML.
- [x] Instrumentation validation: local build and 2/2 CTests passed; a 20 s
  `sim3` smoke at x5 completed with RViz/map/marker output disabled and resolved
  the fixed parameters correctly. Python syntax and telemetry cadence checks
  passed. Smoke artifacts: `/tmp/uwfl2_benchmark_smoke_20260822`.
- [x] The first Jetson smoke preflight found Humble CLI incompatibilities
  before playback (`bag record --topics`, `param dump --timeout`, and
  `bag play --playback-duration`). The runner now uses Humble-compatible
  positional topics and an external duration fallback; no estimator run was
  started by the failed preflight.
- [x] The second Jetson smoke reached playback and telemetry, but its short
  externally limited window ended before a usable sonar scan and exposed an
  empty-cloud PCL exception in `/map_save`. The service now rejects empty maps
  cleanly. A longer 200 s smoke then passed with 925 sonar scans, 47 keyframes,
  a 15,477-point saved map, and valid 1 Hz `tegrastats` data.
- [x] Complete the full local `sim3` x1 baseline with RViz, loop markers, and
  corrected-map publication disabled. Commit `71df2fc`; all 5,948 sonar and
  66,673 IMU messages arrived, and one automatic loop committed.
- [x] Complete the full Jetson Orin Nano `sim3` x1 baseline with the same
  output settings. The first full run lost the initial 0.587 s during DDS
  discovery (8 sonar and 89 IMU messages), which changed initialization and
  prevented the final loop match. The accepted repeat used a two-second
  playback delay and reproduced the local keyframe and loop decisions.
- [x] Fix the benchmark parameters at `max_iteration=4`,
  `filter_size_surf=0.3`, `filter_size_map=0.3`, and
  `cube_side_length=100.0` for every before/after comparison.
- [x] Analyze and record process and system CPU, GPU, RAM, scan latency, graph/STD time,
  shadow-tree rebuild time, registration/commit time, message delivery, loop
  events, Git/config/bag hashes, temperatures, and Jetson power at 1 Hz.

#### P1 Results

- [x] Jetson repeat passed with a two-second DDS discovery delay: 5,947/5,948
  sonar and 66,661/66,673 IMU callbacks arrived; 358/358 keyframes were
  processed with zero queue drops/failures, one loop was accepted, and one
  correction committed. All runner processes exited zero.
- [x] Baseline analysis completed. Local process CPU mean/p95/max was
  15.0/19.8/66.6%, RSS max was 341.5 MiB, and scan p95/max was 4.81/20.46 ms.
  The loop optimization, shadow rebuild, registration, and commit took
  15.78, 113.72, 25.40, and 1.51 ms. ATE RMSE/max/final was
  0.0785/0.1747/0.0442 m; GPU use during the loop window was 0%.
- [x] Jetson process CPU mean/p95/max was 42.1/53.7/97.8%, RSS max was
  317.3 MiB, and scan p95/max was 43.63/107.69 ms. The loop optimization,
  shadow rebuild, registration max, and commit max took 94.68, 311.62, 56.27,
  and 3.09 ms. ATE RMSE/max/final was 0.0638/0.1430/0.0248 m.
- [x] Jetson `tegrastats`: GPU was 0%, total device power mean/p95/max was
  7.71/7.76/9.33 W, RAM max was 1,824 MiB, and CPU/GPU temperatures stayed
  below 53.4/54.2 C. In the +/-5 s loop window, UWFL2 process CPU peaked at
  97.8%, RAM at 309.2 MiB, and device power at 8.59 W.
- Local artifacts:
  `/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/perf_baseline_local_sim3_x1_71df2fc`.
- Jetson artifacts:
  `/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/perf_baseline_jetson_sim3_x1_e0b0d07`.
- Exact runner options for both: `--rate 1 --loop-closure true --detection true
  --loop-visualization false --map-publication false --max-iteration 4
  --filter-size-surf 0.3 --filter-size-map 0.3 --cube-side-length 100`; the
  accepted Jetson run additionally records `--playback-delay 2`.

### P2: Profile And Select Changes

- [x] Rank measured front-end and loop-backend hotspots by total cost, p95,
  maximum spike, memory growth, and effect during accepted loop corrections.
- [x] Inspect CPU allocation/copies, keyframe/descriptor storage, neighborhood
  search, pose-graph work, historical reconstruction, voxel filtering, shadow
  ikd-tree construction, registration, serialization, locks, and queues.
- [x] Evaluate CUDA only for measured data-parallel hotspots whose transfer and
  synchronization cost can be amortized on the Jetson; retain a tested CPU path.

#### P2 Results And Decisions

- [x] The steady front end is the dominant cost: 5,947 Jetson scans consumed
  185.05 s cumulatively. The local stage profile attributes 49.2% of scan time
  to the LiDAR IEKF (42.7% to correspondence work), 26.6% to IMU/aux
  propagation, and 14.8% to LC bookkeeping.
- [x] LC bookkeeping is the first avoidable front-end target. The current path
  copies and voxel-filters pending map history before keyframe selection, so it
  runs for all 5,948 scans although only 358 become keyframes. Jetson scan time
  rises from 30.3 ms just after a keyframe to 36.5 ms at age 2--3 s; the local
  stage profile measured LC bookkeeping at 0.51 ms mean and 1.58 ms p95.
- [x] Automatic STD work is asynchronous but is the largest recurring backend
  cost: Jetson cumulative descriptor/search/verification time was
  0.61/1.97/0.85 s, with 4.50/31.63/104.61 ms maxima. Verification rebuilds
  the same historical PCL k-d tree for two refinements and two overlap tests.
- [x] The accepted-loop shadow rebuild is the largest one-shot backend task.
  Its old 311.62 ms metric omitted the second full-history pass; monotonic event
  timestamps show about 533 ms actual wall time. The builder transforms and
  voxelizes history twice, first for the active tree and again for map output.
- [x] Remaining accepted-loop work is smaller and asynchronous: graph
  optimization was 94.68 ms, registration was 92.86 ms cumulatively with a
  56.27 ms maximum, and the front-end atomic commit was 3.09 ms maximum.
- [x] RSS grew from about 200 MiB after startup to 280 MiB before the loop and
  317 MiB at completion. Stored STD triangles include unused normal vectors;
  93,692 triangles in the local baseline make this a safe memory target.
- [x] CUDA is not selected for P3. The measured hot path uses a mutable
  pointer-based ikd-tree, small 6-by-6 solves, and irregular STD hash/k-d-tree
  searches. The only strongly data-parallel task is the rare shadow rebuild,
  where one CPU pass can first remove duplicated work without transfer,
  synchronization, or a second map representation. Reconsider CUDA only if
  the optimized CPU path still misses the Jetson timing budget.
- [x] Jetson configuration was recorded as six online ARM cores, MAXN SUPER
  power mode, and `schedutil`; the current build intentionally inherits
  FAST-LIO2's ARM `MP_PROC_NUM=1`. A 2/3-thread OpenMP experiment is deferred
  and will be accepted only if latency improves without higher average load,
  power, dropped messages, or changed estimation results.
- [x] Source-level `perf` sampling was unavailable because
  `perf_event_paranoid=4` and non-interactive sudo is disabled. This is not a
  blocker: benchmark-only stage timers now expose the relevant front-end
  sections, and the failed profiler precondition is retained here.

P2 validation commands:

```bash
cd /home/attia/ros2_ws
colcon build --packages-select fast_lio --symlink-install
ctest --test-dir build/fast_lio --output-on-failure

cd /home/attia/ros2_ws/src/FAST_LIO_UnderWater
python3 tools/run_lc_benchmark.py \
  --label p2_frontend_profile_local_sim3_300s_x5 --domain-id 195 \
  --rate 5 --duration 300 --bag /home/attia/ros2_ws/bags/DONE/sim3 \
  --config config/sim.yaml \
  --output /home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/p2_frontend_profile_local_sim3_300s_x5 \
  --loop-closure true --detection true --loop-visualization false \
  --map-publication false --max-iteration 4 --filter-size-surf 0.3 \
  --filter-size-map 0.3 --cube-side-length 100
python3 tools/analyze_lc_run.py \
  --run /home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/p2_frontend_profile_local_sim3_300s_x5
```

- Build passed in 1 min 32 s and CTest passed 2/2. The 300 s bag slice at x5
  completed in 81.7 s with 1,513 sonar callbacks, 81/81 keyframes processed,
  zero queue failures/drops, and valid state/covariance output.
- Profiling instrumentation commit: `6d97126`

### P3: Local Optimization

- [x] Apply one focused optimization per commit and build/test each checkpoint.
- [x] Re-run the matched local benchmark and require unchanged loop decisions,
  trajectory/map accuracy, covariance validity, and message delivery while CPU,
  RAM, or latency improves measurably.

P3 validation cadence:

- Every focused commit: build, CTest, and the relevant deterministic unit or
  component test. No full bag is required for a code path the test exercises
  exactly.
- Front-end-only changes: use the fixed 300 s `sim3` slice at x5 to compare
  stage timing, callbacks, covariance validity, and queue behavior.
- Full `sim3`: run only after a major group that can affect STD decisions or an
  accepted-loop transaction (after P3.2, after P3.3/3.4 if changed, and once
  for the final local candidate). Use x5 during development; reserve the
  matched x1 replay for final local and Jetson validation.
- Any changed loop decision, trajectory, map, or covariance forces a full x1
  regression before the optimization can be accepted.

Planned P3 order:

- [x] P3.1: move keyframe selection before pending-map compaction, share one
  immutable scan conversion between latest-scan registration and keyframes, and
  batch benchmark CSV flushing.
- [x] P3.2: reuse one historical search structure per STD candidate and remove
  unused descriptor storage without changing candidate order or loop decisions.
- [x] P3.3: reconstruct corrected history once and derive both full history and
  the radius-limited shadow tree from that pass.
- [x] P3.4: optimize remaining registration allocations only if its remeasured
  accepted-loop time is material.
- [x] P3.5: test ARM OpenMP thread counts separately on Jetson; retain one
  thread unless the complete CPU/power/latency result is better.

P3.1 result:

- Build passed and CTest passed 2/2, including a new test proving that a
  selected keyframe shares the immutable latest-scan snapshot and that an
  unchanged pose is not selected again.
- Fixed benchmark command matched P2, changing only the label/domain/output to
  `p3_1_frontend_local_sim3_300s_x5`, domain 196, and its matching result path.
- Loop bookkeeping improved from 0.512/1.586 ms mean/p95 to 0.103/0.493 ms;
  total scan mean improved from 3.451 to 3.316 ms. All 1,513 sonar callbacks
  were processed with no invalid state/covariance or queue failure.
- The trajectory statistics were numerically identical to P2 and both saved
  maps have SHA-256
  `79cac8669571b2c7bf315883f6d3a0bf371d171ab7a3ba85878225c91fdddca8`.
- The benchmark runner now signals the ROS launch parent once on shutdown;
  ROS launch forwards the signal and node destructors flush batched diagnostics.
  A 30 s smoke run wrote all 6 keyframes/STD rows and the final summary.

P3.2 result:

- Each verified STD candidate now constructs one historical PCL search tree
  shared by both refinement hypotheses and both overlap tests, instead of four
  equivalent trees. Unused normals were removed from stored keypoints and
  triangles, and planar extraction now requests eigenvalues only.
- Build passed and CTest passed 2/2. Full replay:
  `p3_2_full_local_sim3_x5`, domain 198, with the fixed P1 parameters.
- All 5,948 sonar and 66,673 IMU callbacks arrived; 358/358 keyframes were
  processed with zero failures/drops. Loop decisions matched P1 exactly:
  14 proposals, one accepted/committed loop, and five graph rejections.
- The x5 ATE RMSE/max/final was 0.0782/0.1738/0.0569 m. The P1 x1 result was
  0.0785/0.1747/0.0442 m; final acceptance remains a matched x1 comparison.

P3.3/P3.4 result:

- Corrected keyframe points are transformed once into deterministic full-history
  and active-radius voxel accumulators. If the active selection is the complete
  history, one accumulator is shared and only the compact result is copied for
  active ikd-tree construction.
- Build and CTest passed 2/2. The radius-limited test additionally verifies the
  full corrected history point-for-point and every active voxel against the
  independent reconstruction path.
- Full replay `p3_3_full_local_sim3_x5`, domain 199, preserved all 358
  keyframes, 14 loop proposals, one accepted/committed loop, five graph
  rejections, all callbacks, and zero queue failures/drops.
- Shadow rebuild fell from 203.34 to 104.24 ms and peak RSS from 334.2 to
  306.9 MiB versus the matching P3.2 x5 run. ATE RMSE/max/final was
  0.0781/0.1736/0.0261 m; the faster asynchronous commit changes its exact
  scan, so the final comparison will use x1.
- P3.4 requires no source change: latest-scan registration measured 21.11 ms
  once in the accepted-loop transaction and is no longer material relative to
  recurring front-end work or the shadow rebuild.

P3.5 result:

- Added build-time `UWFL2_MP_PROC_NUM`; an empty value preserves the selected
  architecture default. ARM systems with more than three cores now use two
  correspondence workers; smaller ARM systems retain one.
- Jetson builds and CTest passed 2/2 for one and two workers. Matched 180 s x1
  slices are retained as `p3_5_jetson_mp1_sim3_180s_x1` and
  `p3_5_jetson_mp2_sim3_180s_x1`.
- Two workers reduced scan mean/p95 from 29.38/37.47 to 19.49/24.13 ms and
  correspondence mean/p95 from 21.32/30.01 to 11.35/15.85 ms. Process CPU
  mean changed from 34.8% to 35.5%, system CPU mean from 18.7% to 17.1%, and
  mean device power remained 7.66 W.
- `compare_lc_runs.py` passed: map hash, timestamps, every pose, and every
  rotation were exactly equal. Three workers were not tested because two
  already meets the timing target while reserving four Jetson CPU cores for
  other processes.

### P4: Jetson Validation And Report

- [x] Clean-build and run the accepted implementation on the Jetson using the
  exact P1 bag and parameters.
- [x] Confirm real-time operation and compare initial/final CPU, GPU, RAM,
  temperatures, power, latency, loop spikes, timing, and dropped messages.
- [x] Write `UWFL2_LC_PERFORMANCE.md` as a one-page summary of baseline,
  changes, final measurements, accuracy checks, and remaining bottlenecks.

P4 result:

- Final local artifact: `perf_optimized_local_sim3_x1_64b4cf2`. Final Jetson
  artifact: `perf_optimized_jetson_sim3_x1_64b4cf2`; both use the P1 fixed
  parameters and x1 replay, with the Jetson retaining the two-second discovery
  delay.
- Local scan mean/p95 improved from 3.40/4.81 to 3.03/4.18 ms, peak RSS from
  341.5 to 310.2 MiB, and ATE RMSE/max/final from
  0.0785/0.1747/0.0442 to 0.0783/0.1740/0.0442 m. All callbacks and loop
  decisions matched; the maximum enabled-mode difference was 2.29 mm and
  0.0028 deg due to the faster asynchronous commit instant.
- Jetson scan mean/p95 improved from 31.12/43.63 to 18.69/25.28 ms, process
  CPU mean from 42.1% to 39.2%, system CPU mean from 24.1% to 17.5%, process
  RSS maximum from 317.3 to 287.1 MiB, and power mean from 7.71 to 7.61 W.
  Temperatures remained below 52.6 C CPU and 54.2 C GPU.
- The old Jetson run missed one sonar and 12 IMU callbacks; the final run
  received all 5,948/66,673. Both emitted 5,945 odometry messages with equal
  timestamps and no invalid pose, covariance, or nonmonotonic timestamp. The
  input mismatch explains why their ATE values are not an exact regression
  pair; the matched local result verifies estimator accuracy.
- Final Jetson clean build used isolated `build_p4_clean`/`install_p4_clean`,
  selected two correspondence workers automatically, completed in 8 min 8 s,
  and passed CTest 2/2. Full results and remaining bottlenecks are in
  `UWFL2_LC_PERFORMANCE.md`.
- P3 implementation commits: `5ecc13b`, `9840454`, `036b50d`, `cc44af4`, and
  `64b4cf2`. P4 performance-report commit: `8d48c58`.

### Docking-Station Loop Validation

- [x] Reproduce the rejected final return from `sim_docking_station` at x15.
- [x] Verify proposed constraints against simulation ground truth.
- [x] Correct the loop NIS covariance and add a focused regression test.
- [x] Build/test and validate graph acceptance, re-registration, and atomic
  commit on the same bag.

Result:

- Baseline artifact `sim_docking_lc_diagnostic_x15` processed all 3,899 sonar
  and 194,901 IMU messages. STD proposed 14 candidates and confirmed six, with
  0.806--0.899 overlap, but the graph rejected all at NIS 188.5--578.0 versus
  12.592. Ground truth showed these were real returns: paired poses were only
  0.17--0.26 m apart with zero relative rotation, while the front end had
  accumulated 2.08--2.66 m and 3.31--8.77 deg error.
- Root cause: NIS summed only the endpoint IKF covariances. It omitted the
  complete graph's accumulated relative-pose uncertainty and endpoint
  cross-correlation. NIS now propagates the full joint graph marginal through
  finite-difference Jacobians of the exact full-SE(3) innovation. A 40-node
  regression test proves that accumulated uncertainty admits a consistent
  closure while the existing inconsistent-loop test remains rejected.
- With the corrected marginal, candidate NIS fell to 29.7--43.2. `sim.yaml`
  now uses LTA-OM's odometry-factor variance floors (`1e-6` rotation and
  `1e-4` translation), reducing the uncorrected values to 19.0--28.4. A narrow
  NIS ceiling of 30 retains descriptor, RANSAC, overlap, residual, optimizer,
  registration, and atomic-commit validation. A temporary 600 ceiling was
  diagnostic only and was not retained.
- x15 artifact `sim_docking_lc_final_x15` accepted the correct graph factors,
  but 11 ms wall-time scan spacing made each 21--40 ms registration stale, as
  designed. Final validation therefore used x5, whose 33 ms effective scan
  spacing still accelerates the bag while allowing the asynchronous safety
  transaction to finish.
- Final artifact `sim_docking_lc_final_x5` accepted four true docking factors,
  committed three corrections, delivered every sonar/IMU callback, and had no
  backend drops or failures. Translation RMSE/final improved from
  1.096/2.254 m to 0.878/0.159 m; attitude RMSE/final improved from
  2.242/4.537 deg to 1.881/0.990 deg. Registration reduced each committed scan
  residual and atomic commit took at most 3.17 ms.
- Commands: `colcon build --packages-select fast_lio --symlink-install`;
  `colcon test --packages-select fast_lio --event-handlers console_direct+`;
  `colcon test-result --verbose`; and `tools/run_lc_benchmark.py` with
  `config/sim.yaml`, domains 155--160, x15 diagnostics, and final x5 replay.
  Build passed and all 35 tests passed with zero failures.
- Implementation commit: `5bc85e7`.
- Base-front-end regression check: current `sim3` x15 artifacts
  `sim3_current_no_lc_x15` and `sim3_current_lc_x15` received every 5,948
  sonar and 66,673 IMU callback. No-LC RMSE/final was 0.0768/0.0867 m; LC
  RMSE/final was 0.0767/0.0863 m. The x15 LC run accepted the valid final
  factor but deliberately did not commit its stale registration, so these
  matching trajectories also verify that merely enabling the backend does not
  alter UWFL2.
- The docking error is bag-specific, not an x15 or front-end regression. Both
  bags traverse about 284 m, but the docking trajectory spans 649 s with 3,899
  sonar scans, versus 444 s and 5,948 scans for `sim3`; it also records 145
  late magnetic samples versus 38. The slower, less scan-dense survey therefore
  accumulates much more heading error before its verified return. Since sim3
  passed at x15, the conditional x1 rerun was not required.

### Corrected-Clock Docking Validation

- [x] Replay corrected-clock `sim_docking_station` with UWFL2 and UWFL2-LC.
- [x] Compare the matched pair with the retained `sim3` UWFL2/UWFL2-LC pair.
- [x] Diagnose and correct the simulation-only sensor-model mismatch.
- [x] Build, test, and rerun both corrected configurations at x5.

Result:

- Input bag metadata hash: `3b2ac4ab0f668dcd8cd0528b053aefe882582c0ff43ac148d30c0611bec382f8`.
  It contains 3,815 sonar and 190,718 IMU messages over 677.00 s, with no
  timestamp regression or missing callback in any replay.
- The first matched pair used config hash `5603d3e86a3f6eb1a61a50bf1faddfadeae7ff865ca6bab5e24f02e835586300`.
  UWFL2 produced RMSE/max/final 3.072/7.418/7.389 m and attitude RMSE/final
  9.014/16.424 deg. UWFL2-LC was identical: STD confirmed one candidate, but
  the graph correctly rejected its 8.81 m, 27.95 deg innovation at NIS 1142.8.
- Root cause was simulation configuration, not the corrected clock or loop
  graph. Isaac Sim publishes SI tesla with hard/soft-iron distortion disabled,
  while `sim.yaml` applied real-robot microtesla calibration values. The
  resulting calibrated vector was dominated by a unit-inconsistent offset and
  provided effectively no heading information. `sim.yaml` now uses zero offset,
  identity calibration, `2.56e-14 T^2`, a conservative scalar heading floor,
  and the recorded 300 Hz IMU rate. Implementation commit: `9f6eb9a`.
- Corrected config hash `c6efce05af81f290d5dfca486abae0bede3800dcba4a65a43f84fdbedb49e552`:
  UWFL2 RMSE/max/final is 0.886/1.377/0.413 m, with attitude RMSE/final
  1.960/1.595 deg. UWFL2-LC RMSE/max/final is 0.879/1.377/0.253 m, with
  attitude RMSE/final 1.941/1.489 deg. Two loop factors were accepted and one
  atomic correction committed; the second end-of-bag factor could not obtain a
  fresh registration before shutdown and therefore did not change live state.
- Retained `sim3` x5 references remain better because it is a different,
  denser bag: UWFL2 RMSE/max/final 0.0700/0.1559/0.0807 m; UWFL2-LC
  0.0694/0.1559/0.0230 m with one committed closure. The docking bag starts at
  World z=-0.75 m while UWFL2 starts its local datum at zero, so its analyzer's
  absolute-z metric is not directly comparable to `sim3`; translation ATE is
  startup-relative and remains valid.
- Commands: `colcon build --packages-select fast_lio --symlink-install`;
  `colcon test --packages-select fast_lio --event-handlers console_direct+`;
  `colcon test-result --verbose`; and `tools/run_lc_benchmark.py` at x5 in ROS
  domains 163--166, followed by `tools/analyze_lc_run.py`. Build passed and 35
  tests passed. Artifacts are `sim_docking_clock_fixed_uwfl2_x5`,
  `sim_docking_clock_fixed_uwfl2_lc_x5`,
  `sim_docking_clock_fixed_mag_si_uwfl2_x5`, and
  `sim_docking_clock_fixed_mag_si_uwfl2_lc_x5`.
- Follow-up bag-content analysis explains why docking remains harder than
  `sim3`. Both paths are about 284 m, but `sim3` covers it in 444.3 s using
  5,948 scans at 13.38 Hz and 2,756 points/scan, while docking takes 635.7 s
  using 3,815 scans at 6 Hz and 2,029 points/scan on average. The docking
  magnetic field transformed by ground-truth attitude has 5.30 deg p95 and
  3.32 deg final horizontal-direction error, versus 3.63/0.65 deg in `sim3`.
  DVL remains consistent in both bags (velocity-residual p95 0.0237 versus
  0.0198 m/s) and is not the dominant cause.
- Most docking ATE is a simulation-path/pressure-domain problem. `sim3` stays
  at or below the water surface (`z` from -0.983 to 0 m), giving 0.021 m
  pressure-relative depth RMSE. Docking rises from -0.75 to +1.261 m. The
  physically correct simulated pressure model clamps depth to zero above the
  surface and therefore cannot observe the remaining ascent. Docking UWFL2
  relative x/y/z RMSE is 0.497/0.290/0.674 m and z reaches 1.250 m error; LC
  leaves this historical z error unchanged because the vehicle returns below
  the surface before the final loop.
- A current-code/current-SI-config `sim3` check (`sim3_mag_si_uwfl2_x15`,
  config hash `c6efce05af81f290d5dfca486abae0bede3800dcba4a65a43f84fdbedb49e552`)
  gives RMSE/max/final 0.140/0.244/0.093 m. Thus the retained 0.070 m result was
  partly optimistic because the old unit-mismatched settings made magnetic
  fusion effectively inactive, but `sim3` remains substantially easier.
- Report online odometry ATE and final optimized-graph ATE separately. A loop
  accepted near the end can correct the live endpoint and historical map, but
  cannot rewrite already recorded `/Odometry` messages; consequently docking
  LC improves final error from 0.413 to 0.253 m while whole-run online RMSE
  changes only from 0.886 to 0.879 m.

### SLAMOUTPUT4 Five-Configuration Validation

- [x] Diagnose the current FL2 divergence on the corrected
  `sim_docking_station` bag.
- [x] Tune one shared sonar covariance without changing the sensor set between
  FL2, INS, and UWFL2.
- [x] Replay FL2, INS, UWFL2, FL2-LC, and UWFL2-LC at x5 in isolated ROS
  domains and regenerate the numerical and map comparisons.
- [x] Build and run the complete test suite after the estimator change.

Decision and result:

- A cleanup regression made the IMU orientation and gravity-direction
  observations conditional on auxiliary fusion. Therefore FL2 silently lost
  two IMU observations when DVL, pressure, and magnetometer were disabled.
  They now run once after either propagation path and remain independent of
  the auxiliary-sensor enable state.
- The second cause was `laser_point_cov_xy/z=0.2`, which weakened the 6 Hz
  sonar correction enough for FL2 to diverge. A shared sweep tested `0.001`,
  `0.003`, `0.004`, `0.005`, and `0.01`. The original comparison removed only
  startup translation and therefore left an initial frame rotation inside the
  trajectory error. Both analyzers now apply one complete startup SE(3)
  alignment. Under the corrected metric, `0.003 m^2` is the retained common
  sonar covariance.
- This bag starts at World `[0,0,-1.75]`, not the FAST-LIO startup origin.
  Every final runtime config therefore publishes that exact fixed
  `World -> camera_init` translation, and the plots confirm that ground truth
  and each estimate begin together. The alignment implementation was also
  checked synthetically at zero numerical position error.
- Final startup-SE(3)-aligned translation RMSE/max/final [m] is UWFL2
  `0.317/0.621/0.268`, INS `0.466/0.713/0.431`, and FL2
  `1.310/2.791/1.157`. Thus the requested strict non-LC ordering is satisfied
  in all three metrics. Every run received all 195,575 IMU messages;
  sonar-enabled runs received all 3,914 scans, with no invalid pose or
  covariance.
- At the matched x5 rate, UWFL2-LC accepted three loop factors and committed
  two corrections. Its RMSE/final decreased from `0.317/0.268 m` to
  `0.302/0.150 m`; its maximum is unchanged because the accepted loops occur
  after that earlier peak. FL2-LC committed three corrections and reduced
  FL2 RMSE/final from `1.310/1.157 m` to `1.211/0.154 m`. Loop closure does not
  force zero endpoint error: registration and graph factors have finite noise,
  and motion after the last commit accumulates a new short drift segment.
  The saved graph-corrected map contains 37,418 points (SHA-256
  `516f9a678a13ca46e42da13053937b62459466e5a50d572e0fe926667b4908d7`).
- Input metadata SHA-256 is
  `15cdb2dee37883e92695337628c55c04015e6144c4a461dc30abd7b2f7e2110c`.
  Config hashes are FL2 `c8a69979a2a5c44fc8aa07c6f78d851f3575782d8c78fbaad1c41ff0455154ff`,
  INS `9a37814c321285eb46910e5958ddf7237b7f99a4e650e1e1556dfe76d8194f2f`,
  and UWFL2 `b98bd837ce7d08d2525cccfc7e7f431a5b45a6900d19ce3b8c67ce4f7b742e90`.
  Runs used Git HEAD `452afdf`; implementation history is `83b0924`
  (IMU-update independence), `be59b1d` (superseded first sonar tuning), and
  `1c3f5fb` (benchmark map-save control). The corrected analyzer and retained
  tuning are commits `4f465a7` and `4f8bee5` respectively.
- Commands: `colcon build --packages-select fast_lio --symlink-install`;
  `colcon test --packages-select fast_lio --event-handlers console_direct+`;
  `colcon test-result --verbose`; `SLAMOUTPUT4/run_experiments.sh`; and the
  `compareSim.py`/`plot_map_odom_top_view.py` analysis scripts. Build passed and
  all 35 tests passed. Outputs are in
  `/home/attia/ros2_ws/bags/SLAMOUTPUT4`; superseded `0.001` and `0.005/x15`
  outputs remain archived and all rejected tuning runs remain available.

### AgriLiRa4D NJTerrC05 Replay Checkpoint (2026-08-31)

Implementation commit: `27b4453` (`fix: support AgriLiRa odometry and mapping`).

- [x] Verify the generated ROS 2 bag, topic rates, frames, dropouts, and ground-truth topics.
- [x] Confirm that empty sonar frames propagate odometry without updating the map.
- [x] Restore bounded IMU/auxiliary-only odometry during a complete sonar outage.
- [x] Make `publish.map_en` sufficient to accumulate and publish the corrected map.
- [x] Diagnose the dominant estimator divergence against evaluation-only ground truth.
- [x] Rebuild, run all tests, replay the complete bag, and test a deliberate no-sonar replay.

Decision and result:

- The playable bag is
  `/home/attia/ros2_ws/bags/AgriLiRa4D/processed/NJTerrC05_sonar3d15lf`;
  the parent `processed` directory is not a bag. Regeneration now clips the
  native IMU to the FINS ground-truth interval so that no unsupported tail is
  emitted. Validation passes with 721 sonar frames (11 deliberately empty),
  28,808 IMU messages, 1,213 DVL messages, and 14,147 pressure,
  magnetometer, and ground-truth messages. The 64 DVL dropout intervals are
  intentional simulation output.
- `/auv/pose_actual` and `/aircraft_pose_flu` are identical transformed FLU
  ground truth and remain evaluation-only. They are not fused because the
  emulated DVL, pressure, and magnetic observations were generated from that
  same reference.
- Empty `PointCloud2` scans already followed the propagation-only path and
  skipped map insertion. The missing case was a complete sonar-stream outage:
  after the stream had been seen, no IMU-only packets were synchronized.
  Timeout fallback is restored without pre-empting a pending scan. Before the
  first scan, it waits one configured lidar timeout; after an established
  stream, it activates only when IMU time is at least that timeout beyond the
  last lidar timestamp. It retains one timeout of history so the first
  returning delayed scan remains usable. `common.odometry_publish_rate_hz`
  now caps fallback output at 100 Hz using real IMU timestamps.
- A deliberate no-sonar x10 replay produced 2,717 unique odometry messages
  over a 27.185 s sensor-time window (99.910 Hz average, 9.995 ms median
  interval), with no non-positive timestamps or intervals below 7.5 ms. A
  complete x5 replay received all 721 scans and 28,808 IMU samples with zero
  timestamp regressions, buffer clears, or stale lidar packets.
- `publish.map_en: true` previously did not accumulate points unless loop
  closure or PCD saving was also enabled. The accumulation guard now includes
  map publication. A 35 s controlled replay published 34 map snapshots and
  grew the corrected map from 16 to 21,034 points. RViz now uses the universal
  `camera_init` fixed frame and displays `/auv/pose_actual`; the initial
  estimator body agrees with ground truth within 9 mm and 0.085 degrees.
- The generated Janus DVL covariance is anisotropic: x/y are
  `2.55025e-5 m^2/s^2`, while z is `2.18777e-6 m^2/s^2`. The first UAV YAML
  incorrectly used the smallest z value as an isotropic covariance. Using the
  conservative largest diagonal reduced the same 35 s position RMSE/max/final
  from `8.184/16.374/16.374 m` to `3.743/7.704/7.704 m`, and attitude
  RMSE/max/final from `17.859/21.397/20.824 deg` to
  `8.490/12.437/10.164 deg`.
- The retained full x5 ground-truth replay spans 142.344 s and gives
  position RMSE/max/final `6.311/9.984/3.320 m` and attitude
  RMSE/max/final `8.843/12.242/6.062 deg`. It received 715 sonar callbacks
  with zero timestamp regressions, buffer clears, or stale scans. Initial
  World/body alignment error is 9.5 mm and 0.057 degrees.
- A main-IKF fixed-lag attempt to publish continuously between scans was
  rejected. The bag stores sonar records about 50.01 ms after their header
  timestamps; the experiment averaged only 74.252 Hz because each scan crossed
  the held interval and worsened position/attitude RMSE to
  `6.124 m/15.088 deg`. Constant high-rate output with an active sonar stream
  therefore requires a separate prediction-only odometry state, not advancing
  the live scan-matching IKF ahead of delayed scans.
- The original three-sigma magnetic innovation gate rejected 12,427 of 14,122
  attempted magnetic updates and accepted none after approximately 30 s. The
  emulated field is consistent with ground-truth attitude, so this was an
  estimator lockout after early drift rather than bad bag data. The retained
  UAV config disables that hard gate and uses a conservative `1.0e-4` heading
  covariance floor. In a paired full x5 replay it reduced translation
  RMSE/max/final from the original `30.289/66.128/66.128 m` to
  `10.723/15.483/7.083 m`, and attitude RMSE/max/final from
  `80.561/136.655/136.655 deg` to `13.278/18.141/10.183 deg`.
  A `1.0e-5` floor produced only a small additional change and was rejected as
  unjustified confidence.
- The remaining drift is real and should not be hidden with ground-truth
  fusion. A downward 90 by 40 degree agricultural crop provides weak yaw and
  lateral geometry compared with the underwater simulation bags. The bag is
  now functionally processed correctly, but it is not yet suitable for a
  high-accuracy claim without a better observable sensor/model combination.
- `world_to_camera_init_T/R` is the first ground-truth pose and places the
  estimator-local `camera_init` frame in `world_flu`; initializing the IKF
  state to that global pose as well would apply the transform twice.
- Commands: `colcon build --packages-select fast_lio --symlink-install
  --cmake-args -DCMAKE_BUILD_TYPE=Release`; `colcon test --packages-select
  fast_lio --event-handlers console_direct+`; `colcon test-result --verbose`;
  and
  `/home/attia/ros2_ws/.venv-agrilira4d-inspect/bin/python
  /home/attia/ros2_ws/bags/AgriLiRa4D/tools/validate_njterrc05_underwater_bag.py`.
  The Release build passed, all 35 tests passed, and bag validation passed.
  These changes are intentionally left in the uncommitted working tree for
  user review.

### AgriLiRa4D Three-Way Sensor-Fusion Comparison (2026-08-31)

- [x] Generate FL2, INS, and UWFL2 configs from one canonical YAML.
- [x] Build and replay the same complete bag sequentially at x5.
- [x] Check callback counts, timestamps, config differences, and ground-truth alignment.
- [x] Compute position, depth, and full-SO(3) attitude errors without trajectory fitting.

Decision and result:

- Tested source commit: `308b6bb8ece99db1c601d085e1749be22e973824`.
- Outputs and analysis are retained under
  `/home/attia/ros2_ws/bags/SLAMOUTPUT5`. FL2 is sonar+IMU, INS is
  IMU+DVL+pressure+magnetometer, and UWFL2 combines all five sensors. A
  structured config diff confirms those enablements are the only estimator
  differences. Loop closure and map publication were disabled.
- The source bag was replayed sequentially at x5 in domains 221--223. Every
  sonar-enabled run received all 721 sonar and 28,808 IMU callbacks with zero
  timestamp regressions, stale scans, or buffer clears. INS produced 14,360
  100-Hz fallback samples; sonar-enabled runs produced 719 scan-rate samples.
- Position RMSE/max/final are `16408.659/34014.347/34014.347 m` for FL2,
  `10.949/17.424/5.639 m` for INS, and `7.627/11.955/3.829 m` for UWFL2.
  Attitude RMSE is `68.346/14.921/10.247 deg`, respectively. Thus the requested
  UWFL2 < INS < FL2 ordering holds for RMSE, maximum, final, and attitude error.
- FL2 is catastrophically unobservable rather than merely less accurate. Its
  error exceeds 10 m at 5.82 s and the active map grows to 834,893 points,
  while UWFL2 remains bounded. The 70-degree-downward sonar sees mainly ground,
  leaving horizontal motion and heading weakly constrained; auxiliary sensors
  keep UWFL2 inside scan matching's convergence region. Since all callbacks
  arrived, this is not x5 playback starvation.
- Exact command pattern: `python3 tools/run_lc_benchmark.py --rate 5 --bag
  /home/attia/ros2_ws/bags/AgriLiRa4D/processed/NJTerrC05_sonar3d15lf
  --config <case.yaml> --output <case-output> --loop-closure false --detection
  false --map-publication false --map-save false --domain-id <221|222|223>`;
  analysis: `cd /home/attia/ros2_ws/bags/SLAMOUTPUT5 && python3 compareAgri.py`.
- Build command `colcon build --packages-select fast_lio --symlink-install`
  passed. An initial infrastructure launch using domain 251 failed before
  playback because Fast DDS domain ports support at most 232; the failed
  artifact was retained rather than hidden.

### AgriLiRa4D IMU/Sonar Consistency Follow-up (2026-09-01)

- [x] Diagnose the instantaneous accelerometer attitude update against ground truth.
- [x] Make gravity-direction observations rate-independent across scan and INS modes.
- [x] Correct the gravity-direction Jacobian to its rank-two tangent space.
- [x] Screen motion gates and sonar covariance on short runs.
- [x] Validate the selected configuration over complete sequential x5 replays.

Decision and result:

- Instantaneous acceleration samples accepted by the old norm gate disagreed
  with gravity by `31.86 deg` mean and `65.61 deg` p95. A 0.2 s average reduced
  this to `3.44/7.97 deg`. Each non-overlapping window is now used once, so the
  100 Hz INS fallback cannot reuse gravity evidence more often than the 5 Hz
  scan path.
- The old identity attitude Jacobian falsely conditioned heading. It is replaced
  by `I-gg^T`, preserving the unobservable heading direction. The first build
  failed because the patch matched the adjacent orientation update; placement
  was corrected, after which the build and all 35 tests passed.
- The generated DVL was verified against the exact smoothed trajectory and
  lever-arm model: 3D velocity RMS error is `0.00726 m/s`, consistent with its
  covariance. An earlier `0.567 m/s` diagnostic was rejected because it
  differentiated unsmoothed quantized ground truth.
- Selected canonical settings are `accel_attitude_norm_gate: 0.1` and
  `laser_point_cov_xy/z: 0.02`. Complete x5 position RMSE/max/final are
  `136.129/222.981/211.921 m` (FL2), `4.718/9.294/0.230 m` (INS), and
  `2.696/4.883/1.793 m` (UWFL2). UWFL2 also has the lowest depth and attitude
  RMSE. FL2 remains physically underconstrained by the 5 Hz, downward,
  ground-dominated pseudo-sonar and must not be made artificially competitive
  with a mode-specific estimator configuration.
- Commands: `colcon build --packages-select fast_lio --symlink-install`;
  `colcon test --packages-select fast_lio`; and sequential
  `tools/run_lc_benchmark.py --rate 5` replays retained under
  `/home/attia/ros2_ws/bags/SLAMOUTPUT5`.
- Commit: `13a14c4` (`fix: make gravity attitude updates observable`).

### Sparse-Sonar Fallback Checkpoint (2026-09-02)

- [x] Add configurable pre-match and effective-feature scan thresholds with legacy defaults.
- [x] Preserve the propagated IMU/auxiliary state when a scan is rejected.
- [x] Prevent rejected scans from changing the map, keyframes, loop closure, or sonar-observability timing.
- [x] Add compact rejection/correction/map-insertion diagnostics and focused tests.
- [x] Screen three threshold pairs on the first 40 s of AgriLiRa.
- [ ] Validate a selected pair on the full AgriLiRa bag.
- [ ] Run `sim3` and `sim_docking_station` disabled/enabled loop-closure regressions after the major change.

Decision: FL2 remains a strict sonar+IMU baseline. Weak scans are rejected as
complete observations; UWFL2 keeps its timestamp-corrected auxiliary prior and
FL2 keeps its IMU prior. Rejected geometry intentionally leaves a gap in the
map. Continuous weak scans remain scan-rate, while 100 Hz fallback remains for
an absent sonar stream. Threshold candidates are `(50,20)`, `(100,50)`, and
`(250,100)` for downsampled/effective points.

Implementation and test result:

- Added `mapping.minimum_scan_points` and
  `mapping.minimum_effective_features` with legacy defaults `5/1`. Rejected
  updates restore both IKF state and covariance; map insertion, keyframes,
  loop-closure notification, and sonar observability notification occur only
  after an accepted update. Per-scan CSV rows now record the rejection reason,
  input/effective counts, correction norms, and inserted-map count.
- Build passed with `colcon build --packages-select fast_lio
  --symlink-install`. `colcon test --packages-select fast_lio` passed all 42
  reported tests, including six focused sparse/transaction tests. Implementation
  commit: `e7e08ea` (`feat: reject weak sonar updates transactionally`).
- Exact short-run command used `tools/run_lc_benchmark.py --rate 15
  --duration 40 --loop-closure false --detection false --map-publication false
  --map-save false`, sequentially in domains 210--215. Outputs and
  `screen_metrics.csv` are under
  `/home/attia/ros2_ws/bags/SLAMOUTPUT5/sparse_sonar_screens`.
- Retained FL2 baseline first-40-s RMSE/max were `29.880/73.521 m`. Candidate
  RMSE/max values were `42.627/119.645 m` for `50/20`, `35.254/98.021 m` for
  `100/50`, and `48.478/137.426 m` for `250/100`. The corresponding UWFL2
  RMSE values were `2.184`, `2.137`, and `2.259 m`, versus the retained
  `1.799 m` baseline. Every candidate therefore failed both the FL2 50-percent
  improvement and UWFL2 5-percent-change criteria.
- All rejected rows reported zero inserted map points, and no covariance
  validity warning occurred. This isolates the failed assumption: point count
  alone cannot distinguish harmful from useful geometry, and even sparse
  ground-dominated scans carry constraints that pure FL2 needs. No candidate
  was selected; the AgriLiRa config remains at legacy `5/1`.
- Stop condition reached as specified. Full AgriLiRa and sim/docking
  regressions were not run because there is no passing threshold to validate.
  Do not proceed to an observability-Hessian mechanism without a separately
  reviewed checkpoint.

### AgriLiRa FL2 Inertial Root-Cause Checkpoint (2026-09-02)

- [x] Reuse retained covariance trials before starting new replays.
- [x] Compare the processed IMU frame, timing, startup motion, and gyro bias
  against the bag ground truth.
- [x] Record IMU bias, gravity, velocity, and covariance evolution alongside
  each sonar correction.
- [x] Screen IMU process covariance and the non-FAST-LIO2 accelerometer tilt
  observation over the first 40 s with the legacy `5/1` scan thresholds.
- [x] Diagnose per-scan sonar attitude information and direct IMU-bias
  corrections.
- [x] Test a full-SE(3) attitude observability projection, disabled by default,
  before considering any constrained-gain modification.
- [x] Add and validate a physically consistent 20 s stationary prefix to the
  derived AgriLiRa bag without changing estimator source code.
- [ ] Validate only a configuration that improves FL2 without degrading UWFL2.

Initial evidence: weakening sonar covariance from `0.02` to `0.2` reduced the
retained full FL2 RMSE from `136.129 m` to `61.088 m`; weakening it much further
or locking inertial covariance eventually diverged more. The first 2 s have
only `0.044 m/s` mean ground-truth speed, and the initialized gyro mean is
approximately `[-0.0082, 0.0148, 0.0091] rad/s`. Direct gyro integration with
that startup mean gives about `6.2 deg` final attitude error, so an initial
frame/sign failure is not supported. The next test isolates whether scan
cross-covariance or the added acceleration-as-gravity observation corrupts the
otherwise usable inertial prior.

Process-noise screens at `0.1x` and `0.01x` changed the 40 s FL2 RMSE from
`8.075 m` to `8.565 m` and `8.515 m`; disabling the acceleration tilt update
worsened it to `19.291 m`. The baseline sonar update changed the yaw gyro bias
by `+0.00518 rad/s` and reduced its covariance to `5.1e-9`, while the resulting
attitude error reached `12.36 deg`. Direct gyro integration was only
`2.88 deg` wrong at 40 s. Therefore IMU covariance tuning is rejected as the
root fix: weak sonar geometry is falsely conditioning attitude and gyro bias
through state cross-covariance. Raw and Schur-complement attitude-information
projection did not materially improve the result and was removed. A tight
startup gyro-bias covariance improved 40 s but caused severe full-run
translation divergence, so it was also removed.

Bag correction: the converter now prepends 20 s of fixed pose/sonar/pressure/
magnetometer data, 15 Hz zero DVL velocity, and 200 Hz constant IMU data using
the source sensor's estimated mean offset. IMU covariance fields remain zero;
`UAV_bag.yaml` is authoritative. Conversion validation passed. An FL2 x15
replay produced `0.025 m` stationary RMSE and `0.028 m` stationary final error,
but motion-only RMSE remained `57.658 m`. This confirms that startup motion was
a real bag defect, while the remaining failure is the weak-sonar attitude/
gyro-bias conditioning already identified above.

Commands:

```bash
/home/attia/ros2_ws/.venv-agrilira4d-inspect/bin/python \
  /home/attia/ros2_ws/bags/AgriLiRa4D/tools/build_njterrc05_underwater_bag.py \
  --overwrite
/home/attia/ros2_ws/.venv-agrilira4d-inspect/bin/python \
  /home/attia/ros2_ws/bags/AgriLiRa4D/tools/validate_njterrc05_underwater_bag.py
```

### Sim Docking Station 5 FL2-LC Validation (2026-09-21)

- [x] Build current branch successfully with `colcon build
  --packages-select fast_lio --symlink-install`.
- [x] Run FL2-LC at x15 with RViz, auxiliary sensors disabled, and full-SE(3)
  automatic loop closure enabled.
- [x] Restrict detection to the terminal revisit with
  `std_minimum_loop_duration_s: 500.0`.
- [x] Confirm final graph loop, corrected-map rebuild, latest-scan
  re-registration, and atomic state/tree commit.

Run configuration:
`/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/configs/sim_docking_station_5_fl2_lc.yaml`.
Output:
`/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/sim_docking_station_5_fl2_lc_terminal_x15`.
The vehicle ground truth returned within `0.048 m` of its start at `540.1 s`.
STD accepted one loop from keyframe 348 (`542.307 s`) to keyframe 0. Graph
optimization reduced its translation residual from `3.217 m` to `0.023 m` and
rotation residual from `0.719 deg` to `0.175 deg`. The shadow map rebuilt in
`100.410 ms`; latest-scan registration passed and reduced mean residual from
`0.0442 m` to `0.0355 m`; the correction committed at `546.640 s` in
`1.491 ms`. All `3260` sonar and `162344` IMU callbacks arrived with no drops,
timestamp regressions, or invalid covariance.

The saved corrected map is `sim_docking_station_5_fl2_lc.pcd`. The final
recorded `/Odometry` message precedes the asynchronous commit, so its endpoint
still contains the pre-loop `3.23 m` error even though the internal state and
saved map were corrected. `check_reregistration.py` passed. The generic
`check_pose_graph.py` reported only `manual-loop request artifact is missing`;
that assertion does not apply to this automatic-loop run.

Playback-rate and pressure controls:

- [x] Replayed the identical FL2-LC configuration at x5. The x5 and x15 runs
  produced exactly the same pre-loop translation residual (`3.217350836 m`),
  optimized residual (`0.023364802 m`), and initial loop NIS (`9.220384339`).
  Thus x15 did not cause the accumulated front-end drift. The mismatch is
  `1.14%` of the `282.3 m` estimated path length. x15 only exposed an
  end-of-bag scheduling edge case when a correction became ready after the
  final simulated-clock tick.
- [x] Compared UWFL2-LC at x15 with identical configurations differing only in
  `pressure.enable`. Pressure reduced 3D ATE RMSE from `0.2466 m` to
  `0.2183 m` and depth RMSE from `0.1094 m` to `0.0454 m`; all `3260` sonar
  and `162344` IMU callbacks arrived and covariance remained valid. It also
  increased attitude RMSE from `0.5176 deg` to `0.6969 deg` and the maximum
  3D error from `0.4344 m` to `0.5412 m`, so it improves position/depth RMSE
  but not every metric.
- [x] Replayed pressure-on UWFL2-LC at x5 to validate the complete asynchronous
  pipeline. One terminal loop was accepted, latest-scan registration passed,
  and one correction committed. Its 3D/depth RMSE remained
  `0.2187/0.0454 m` with no invalid covariance.

Pre-loop drift diagnosis:

- [x] The `3.217 m` loop residual is the terminal FL2 front-end error, not an
  initial error. It is horizontal (`3.228 m` final 3D error versus only
  `0.061 m` depth error and `0.469 deg` attitude error).
- [x] Between `515--535 s`, FL2 estimated only `80.9--90.3%` of each true
  five-second displacement. The sonar clouds in the same interval were nearly
  planar (smallest point-cloud covariance eigenvalue `0.0037--0.0052`), so the
  point-to-plane update weakly observes translation tangent to the seabed.
- [x] UWFL2 with pressure disabled estimated `99.7--100.1%` of those same
  displacements and ended near `0.10 m` error. This isolates DVL velocity as
  the constraint that supplies the missing along-track information; pressure
  is not responsible for the FL2 endpoint drift.
- [x] Sonar headers were monotonic at exactly `6 Hz`, with a stable phase of
  about `33 ms` relative to `/clock`. Together with identical x5/x15 loop
  residuals, this rejects playback speed and timestamp jitter as causes.
- [x] Added a full-duration INS control using the same UWFL2 configuration
  with only sonar, mapping, and loop closure disabled. FL2/INS/UWFL2 3D ATE
  RMSE values were `0.674/1.159/0.218 m`; maximum errors were
  `3.330/1.977/0.541 m`; final errors were `3.228/1.416/0.050 m`. INS avoids
  the terminal planar-sonar scale loss but accumulates horizontal
  dead-reckoning error over the full route. UWFL2 is best in all three metrics.
  The valid INS output is `sim_docking_station_5_ins_x15_drain45`; the earlier
  `sim_docking_station_5_ins_x15` recording is incomplete because its recorder
  stopped before the x15 auxiliary-processing backlog drained.
- [x] Measured the historical optimized UWFL2 path after the previously
  accepted low-innovation loop. Although latest-scan re-registration protected
  the live state, the loop increased optimized-path RMSE from `0.228 m` to
  `0.668 m`; its fixed full-SE(3) constraint was overconfident for the nearly
  planar terminal view. Added `loop_minimum_initial_nis` with a legacy-safe
  default of `0.0` and selected `1.0` for the validated configs.
- [x] At x5, UWFL2 terminal proposals with NIS `0.055` and `0.024` were skipped
  as statistically insignificant. No correction committed, and online metrics
  remained RMSE/max/final `0.218/0.541/0.050 m`. At x15, FL2's NIS `9.220`
  loop remained accepted, reduced `3.217 m` to `0.023 m`, and committed.
  Build passed and all 78 tests passed, including the focused insignificant-loop
  transaction test. Outputs are `sim_docking_station_5_uwfl2_lc_significance_x5`
  and `sim_docking_station_5_fl2_lc_significance_x15`.
- [x] Rejected more-frequent loop tuning as a parameter-only solution. Reducing
  `std_minimum_loop_duration_s` from `500` to `60 s` generated 87 proposals and
  accepted one loop from keyframe 238 (`391.64 s`) to 301 (`492.64 s`, NIS
  `1.60`). It worsened the keyframe-path RMSE/max from `0.228/0.537 m` to
  `0.238/0.562 m` and did not reduce the online `0.541 m` peak at `500 s`.
  Thirty-four shadow rebuilds became stale as subsequent keyframes advanced the
  graph, so the correction committed only at the end. Keep `500 s` and NIS
  `1.0`; future frequent closure requires degeneracy-aware loop uncertainty and
  a shadow-map pipeline that can incorporate post-snapshot keyframes.

Pressure comparison outputs are under
`/home/attia/ros2_ws/bags/UWFL2_LC_RESULTS/sim_docking_station_5_uwfl2_lc_pressure_{off,on}_x15`;
the committed x5 pressure-on run is
`sim_docking_station_5_uwfl2_lc_pressure_on_x5`.

### Estimator Recovery Rollback (2026-10-05)

- [x] Restored estimator, propagation, and sensor-measurement handling to
  `35a77379fa0d984fbbf1928bdc392a93f3252959`.
- [x] Retained asynchronous loop closure and its full-SE(3) state/tree commit.
- [x] Retained a dense corrected visualization/save map independently from the
  compact real-time ikd-tree.
- [x] Retained paired sonar cloud/timestamp queue clearing after a genuine
  timestamp regression.
- [x] Retained passive missing/late-sensor diagnostics; no timestamp or
  measurement rewriting remains.
- [x] Build passed with
  `colcon build --packages-select fast_lio --symlink-install`; all 48 tests
  passed with `colcon test --packages-select fast_lio`.
- Rollback commit: `e964372`.

### External-Driver Bag Conversion (2026-10-05)

- [x] Added `tools/process_external_frd_bags.py` to rename external SINTEF
  topics and convert IMU, magnetometer, DVL, and sonar data to UWFL2 frames.
- [x] IMU, magnetometer, DVL, and sonar use the MAVLink aircraft-body FRD to
  ROS REP-103 FLU transform `diag(+1,-1,-1)`. The earlier Y-axis rotation
  `diag(-1,+1,-1)` was rejected after checking the MAVLink and MAVROS frame
  definitions.
- [x] Preserved bag and message timestamps and copied unrelated topics without
  modifying their measurements.
- [x] Converted `mag_cal` and `pillars1--4_15mfar` into
  `/home/attia/ros2_ws/bags2/processed`; topic counts and representative
  nonzero numerical transforms passed.
- [x] Rebuilt the `bridge` package after correcting its live `SCALED_IMU`
  accel/gyro/magnetometer conversion and removing the unused, dimensionally
  invalid `RAW_IMU` handler. Regenerated all five processed bags and verified
  1000 IMU and 1000 magnetometer samples; stationary FLU acceleration had
  median `z=+9.69 m/s^2`.
- Command: `tools/process_external_frd_bags.py --all`
- Commit: `9942a5f`.
- Frame-correction commit: `c99225d`.

### Live Sonar And Corrected-Map Publication (2026-10-05)

- [x] Publish the incoming sonar scan in the estimator TF tree without changing
  the estimator input or reintroducing the removed registered-cloud map.
- [x] Publish corrected history only when committed keyframe history changes,
  while retaining immediate publication after a loop-correction commit.
- [x] Avoid per-scan whole-history work by refreshing the compact corrected map
  only after committed keyframe history changes.
- [x] Build, run the complete test suite, and perform a short ROS topic/TF
  smoke test before completing this checkpoint.

Decision: original FAST-LIO2 keeps real-time visualization responsive by
publishing one current scan per frame and not publishing the accumulated map in
its real-time loop. LTA-OM likewise separates current-scan publication from
background corrected-history reconstruction. UWFL2-LC will mirror the raw
input on `/uwfl2/sonar_live` in a private `uwfl2_sonar` frame attached to
`body` by the configured sonar-to-IMU extrinsic. This avoids TF conflicts with
recorded bags that already assign `sonar_frame` to another parent. The
transient `/uwfl2/corrected_map` remains a historical snapshot, not a live
vehicle-motion topic.

Implementation and validation:

- `/uwfl2/sonar_live` is a subscriber-aware best-effort mirror of the incoming
  cloud with its original timestamp and an `uwfl2_sonar` frame. A static
  `body -> uwfl2_sonar` transform uses `mapping.extrinsic_R/T`; the estimator
  input, state, covariance, scan matching, and active ikd-tree are unchanged.
- In loop-closure mode, corrected-map publication is requested after a
  keyframe submap is committed and after an atomic loop correction, rather
  than after every sonar scan. `publish.corrected_map_interval_s` remains the
  maximum publication-rate limiter. Final global voxel compaction is retained
  to keep RViz messages and `/map_save` compact.
- `colcon build --packages-select fast_lio --symlink-install` passed. The
  existing Boost and deprecated ROS service-QoS warnings remain. `colcon test
  --packages-select fast_lio` passed: 45 tests, zero failures.
- A 60-sensor-second smoke replay of `sim_docking_station_5` at x15 in ROS
  domain 123 received 293 best-effort live scans and 16 monotonically growing
  corrected-map snapshots. Both `camera_init -> body` and
  `body -> uwfl2_sonar` were present. A separate x1 replay measured the live
  stream at exactly 6.0 Hz, matching the bag sonar rate.
- The first final smoke attempt used ROS domain 233 and failed before startup
  because Fast DDS rejected a domain above its local port limit. Repeating in
  domain 123 passed. Earlier smoke launch wrappers in isolated domains 231 and
  232 did not stop on scripted SIGINT; they were terminated, and the user's
  pre-existing launch process was left untouched.
- The existing user-customized RViz display now selects
  `/uwfl2/sonar_live` with best-effort QoS. That already-dirty RViz file was
  deliberately not included in the focused implementation commit.
- Implementation commit: `e385828`.

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
