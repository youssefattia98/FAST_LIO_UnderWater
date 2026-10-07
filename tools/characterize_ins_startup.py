#!/usr/bin/env python3
"""Report native prediction-publication repeatability without aligning trajectories."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from analyze_lc_run import read_trajectories
from compare_refactor_replays import odometry_fields, shared_pose_comparison


def characterize(baseline, repeat):
    left = read_trajectories(baseline / "output_bag")
    right = read_trajectories(repeat / "output_bag")
    common, i, j = np.intersect1d(left["odom_stamps_ns"], right["odom_stamps_ns"],
                                 return_indices=True)
    first_fields = odometry_fields(baseline / "output_bag")
    second_fields = odometry_fields(repeat / "output_bag")
    position = np.linalg.norm(left["odom_poses"][i, :3, 3]
                              - right["odom_poses"][j, :3, 3], axis=1)
    rotation = np.max(np.abs(left["odom_poses"][i, :3, :3]
                            - right["odom_poses"][j, :3, :3]), axis=(1, 2))
    covariance = np.array([np.max(np.abs(first_fields[int(t)][0]
                                       - second_fields[int(t)][0])) for t in common])
    differing = (position > 1e-9) | (rotation > 1e-9) | (covariance > 1e-9)
    manifests = [json.loads((run / "manifest.json").read_text())
                 for run in (baseline, repeat)]
    def evidence(manifest, key):
        return manifest.get("commands", {}).get(key, {}).get("output")
    report = shared_pose_comparison(left, right)
    report.update({
        "baseline": str(baseline), "repeat": str(repeat),
        "same_git_head": evidence(manifests[0], "git_head") == evidence(manifests[1], "git_head"),
        "same_worktree_diff": evidence(manifests[0], "worktree_diff")
                              == evidence(manifests[1], "worktree_diff"),
        "same_bag_metadata": manifests[0].get("bag_metadata_sha256")
                             == manifests[1].get("bag_metadata_sha256"),
        "native_stamp_sha256": [hashlib.sha256(data["odom_stamps_ns"].tobytes()).hexdigest()
                               for data in (left, right)],
        "differing_shared_stamps_ns": common[differing].tolist(),
        "covariance_max_difference": float(covariance.max()) if len(common) else None,
        "invalid_covariance_counts": [data["invalid_odom_covariance"]
                                      for data in (left, right)],
        # Published odometry does not identify correction versus prediction.
        "runtime_correction_epochs_available": False,
        "callback_delivery_available": False,
        "no_timestamp_alignment_or_tolerance_change": True,
    })
    if len(common):
        steady = common > common[0] + 1000000000
        report["after_first_second"] = {
            "shared_samples": int(steady.sum()),
            "position_max_m": float(position[steady].max()) if steady.any() else None,
            "rotation_matrix_max": float(rotation[steady].max()) if steady.any() else None,
            "covariance_max_difference": float(covariance[steady].max()) if steady.any() else None,
        }
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--repeat", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(characterize(args.baseline, args.repeat),
                                      indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
