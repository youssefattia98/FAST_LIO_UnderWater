# UWFL2 Repository Rules

These rules apply to all work in this repository.

## Branch And Preservation

- Preserve the original UWFL2 branches and behavior. Treat `main` and `ROS2` as read-only references.
- Perform loop-closure work only on `feature/uwfl2-ltaom-loop-closure`.
- Keep loop closure optional and disabled by default. Disabled mode must reproduce the original UWFL2 outputs within documented numerical tolerances.
- Do not rewrite, squash, or silently absorb pre-existing user changes.

## Estimator And Frame Invariants

- Use full SE(3) throughout. Never assume planar motion, fixed roll or pitch, constant depth, or yaw-only loop corrections.
- Preserve the UWFL2 sonar, IMU, DVL, pressure, magnetometer, IKFOM, and real-time ikd-tree front end.
- Keep frame notation and transform direction explicit. Record the source frame, target frame, timestamp, and perturbation convention for every new pose or residual.
- Preserve UWFL2's complete estimator state and covariance handling, including DVL and pressure biases, IMU biases, gravity, sonar-IMU extrinsics, startup-relative magnetic heading, and World/startup-local transforms.
- Do not reset, rotate, zero, or otherwise alter a state block unless the mathematical reason and covariance transport are documented and tested.

## Loop-Closure Architecture

- Run keyframe construction, STD detection, graph optimization, historical-map correction, and shadow-tree rebuilding asynchronously.
- Keep the high-rate front end as the only owner that mutates the live IKFOM state and active ikd-tree.
- Use bounded queues and bounded commit work so the design remains suitable for the Jetson Orin Nano.
- Build corrected maps in a shadow ikd-tree. Re-register the latest scan before committing a loop correction.
- Replace the corrected state, covariance, map metadata, and active tree as one validated transaction at a front-end safe point.
- Reject stale, non-finite, inconsistent, or geometrically unsupported corrections without changing the live estimator.

## Checkpoint Discipline

- Work in the checkpoints defined by `UWFL2_LC_PLAN.md` and make focused commits.
- Maintain `UWFL2_LC_PLAN.md` as the authoritative engineering log. Every checkpoint must retain checkboxes and record decisions, exact commands, results, blockers, timing measurements, and the commit hash.
- Never mark a checkpoint complete without a successful build and its specified tests.
- Run and record disabled-mode regression tests after any change that can affect the front end, state, covariance, timing, frames, or ikd-tree ownership.
- Record failed builds, tests, loop candidates, and performance regressions. Do not hide or overwrite failed results.
- Do not make undocumented frame, timestamp, state, covariance, or sensor assumptions. Stop and record the blocker when evidence is insufficient.
- Do not tune against only one successful replay. Preserve the exact bag, config hash, Git hash, replay rate, ROS domain, and output directory for every comparison.

## Required References

Before changing the implementation, consult and compare:

- UWFL2 code in this repository and paper at `/home/attia/paper/article_underwater_SLAM`.
- Original FAST-LIO2 code and paper at `/home/attia/Downloads/FAST_LIO`.
- LTA-OM code and paper at `/home/attia/Downloads/LTAOM-main`.

Adapt algorithms to ROS 2 and UWFL2 deliberately. Do not copy ROS 1 threading, patched-library assumptions, or partial FAST-LIO2 state-reset logic without validating them against UWFL2.
