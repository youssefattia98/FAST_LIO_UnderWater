# UWFL2-LC Jetson Performance

## Scope

- Branch: `feature/uwfl2-ltaom-loop-closure`
- Final tested commit: `64b4cf2`
- Bag: `sim3`, 1290.43 s, replayed sequentially at x1
- Fixed estimator parameters: `max_iteration=4`, surface/map voxels `0.3 m`,
  cube side `100 m`; RViz, map publication, and loop markers disabled.
- Loop behavior remained 358 keyframes, 14 proposals, one accepted and
  committed loop, five graph rejections, and zero queue drops/failures.

## Implemented Changes

1. Select keyframes before compacting pending map history and share one
   immutable scan snapshot between keyframes and latest-scan registration.
2. Reuse one historical PCL search tree per STD candidate and remove unused
   stored normals.
3. Transform corrected history once while deriving the full corrected map and
   radius-limited shadow ikd-tree.
4. Use two short-lived correspondence workers on ARM systems with at least four
   cores. `UWFL2_MP_PROC_NUM` remains available as a build-time override.
5. Batch diagnostics and use graceful benchmark shutdown.

## Results

| Metric | Local P1 | Local final | Jetson P1 | Jetson final |
|---|---:|---:|---:|---:|
| Scan mean / p95 [ms] | 3.40 / 4.81 | 3.03 / 4.18 | 31.12 / 43.63 | 18.69 / 25.28 |
| Process CPU mean / p95 [%] | 15.0 / 19.8 | 14.5 / 18.9 | 42.1 / 53.7 | 39.2 / 49.9 |
| System CPU mean / p95 [%] | 2.5 / 4.2 | 2.6 / 4.4 | 24.1 / 34.5 | 17.5 / 27.6 |
| Process RSS maximum [MiB] | 341.5 | 310.2 | 317.3 | 287.1 |
| Device power mean / p95 [W] | n/a | n/a | 7.71 / 7.76 | 7.61 / 7.64 |
| ATE RMSE / max / final [m] | .0785 / .1747 / .0442 | .0783 / .1740 / .0442 | .0638 / .1430 / .0248 | .0755 / .1697 / .0312 |

The local x1 runs received identical inputs and output timestamps. Their maximum
pose difference was 2.29 mm and 0.0028 deg, caused by the faster asynchronous
loop transaction completing at a slightly different scan; ATE did not regress.
The old Jetson baseline missed one sonar and 12 IMU callbacks, whereas the final
run received all 5,948 sonar and 66,673 IMU callbacks. Its ATE values therefore
do not form an exact input-sequence comparison; the matched local result is the
accuracy check. Both Jetson runs published the same 5,945 odometry samples with
identical timestamps and no invalid pose, covariance, or timestamp.

The old Jetson shadow metric reported 311.6 ms but omitted its second corrected-
history pass; event timing measured about 533 ms total. The final 291.6 ms value
includes the complete one-pass rebuild, approximately 45% lower. The two-worker
180 s isolation test produced exactly identical poses, rotations, timestamps,
and map hash to one worker while reducing scan p95 from 37.47 to 24.13 ms.

## Validation

```bash
colcon build --packages-select fast_lio --symlink-install
ctest --test-dir build/fast_lio --output-on-failure

# Jetson clean build in isolated directories
colcon --log-base log/p4_clean build --packages-select fast_lio \
  --symlink-install --build-base build_p4_clean \
  --install-base install_p4_clean
ctest --test-dir build_p4_clean/fast_lio --output-on-failure
```

The local and Jetson builds passed, as did both tests on each platform. Final
artifacts are `perf_optimized_local_sim3_x1_64b4cf2` and
`perf_optimized_jetson_sim3_x1_64b4cf2` under `bags/UWFL2_LC_RESULTS` on their
respective hosts.

## Remaining Work

- Latest-scan registration reached 106.5 ms once on the final Jetson run, but
  remains asynchronous and did not delay scan processing.
- Peak scan latency was 100.6 ms although p99 was 32.0 ms; retain telemetry on
  new bags and concurrent Jetson workloads.
- CUDA remains deferred. The recurring work is dominated by irregular ikd-tree
  access and small matrix operations; four CPU cores and the GPU remain free for
  other onboard processes.
