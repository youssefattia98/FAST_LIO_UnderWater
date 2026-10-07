#!/usr/bin/env python3
"""Fixture coverage for benchmark configuration and missing-evidence checks."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import compare_lc_runs as compare
import run_lc_benchmark as benchmark
import characterize_ins_startup as startup
from compare_refactor_replays import shared_pose_comparison


class BenchmarkToolsTest(unittest.TestCase):
    def test_startup_report_keeps_native_epochs_and_marks_missing_trace(self):
        baseline, repeat = self.root / "baseline", self.root / "repeat"
        for run in (baseline, repeat):
            run.mkdir()
            (run / "manifest.json").write_text(json.dumps({
                "bag_metadata_sha256": "bag",
                "commands": {"git_head": {"output": "head"},
                             "worktree_diff": {"output": "diff"}}}))
        stamps = np.array([1780000000000000001, 1780000000500000001,
                           1780000002000000001], dtype=np.int64)
        data = {"odom_stamps_ns": stamps, "odom_times": stamps / 1e9,
                "odom_frames": [("camera_init", "body")] * 3,
                "odom_poses": np.repeat(np.eye(4)[None], 3, axis=0),
                "invalid_odom_covariance": 0}
        other = dict(data, odom_poses=data["odom_poses"].copy())
        other["odom_poses"][0, 0, 3] = 0.01
        fields = {int(t): (np.zeros(72), np.zeros(6)) for t in stamps}
        with patch.object(startup, "read_trajectories", side_effect=[data, other]), \
             patch.object(startup, "odometry_fields", return_value=fields):
            report = startup.characterize(baseline, repeat)
        self.assertEqual(report["differing_shared_stamps_ns"], [int(stamps[0])])
        self.assertEqual(report["after_first_second"]["position_max_m"], 0.0)
        self.assertTrue(report["same_worktree_diff"])
        self.assertFalse(report["runtime_correction_epochs_available"])
        self.assertFalse(report["callback_delivery_available"])
        json.dumps(report, allow_nan=False)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.config = self.root / "input.yaml"
        self.document = {"/**": {"ros__parameters": {
            "mapping": {"max_iteration": 16},
            "loop_closure": {"enable": True}, "publish": {},
            "dvl": {"enable": False}}}}
        self.config.write_text(yaml.safe_dump(self.document))
        self.args = argparse.Namespace(
            config=self.config, loop_closure="false", detection=None,
            loop_visualization=None, max_iteration=4, filter_size_surf=0.3,
            filter_size_map=None, cube_side_length=None, map_publication="false",
            map_save="true")

    def test_current_nested_overrides_and_source_preserved(self):
        path = benchmark.write_runtime_config(self.args, self.root)
        parameters = yaml.safe_load(path.read_text())["/**"]["ros__parameters"]
        self.assertEqual(parameters["mapping"]["max_iteration"], 4)
        self.assertEqual(parameters["mapping"]["filter_size_surf"], 0.3)
        self.assertEqual(parameters["mapping"]["map_file_path"], str(self.root / "test.pcd"))
        self.assertFalse(parameters["publish"]["corrected_map_enable"])
        self.assertEqual(parameters["loop_closure"], {"enable": False})
        self.assertNotIn("max_iteration", parameters)
        self.assertEqual(yaml.safe_load(self.config.read_text()), self.document)

    def test_unsupported_detection_override(self):
        self.args.loop_closure = "true"
        self.args.detection = "false"
        with self.assertRaises(ValueError):
            benchmark.write_runtime_config(self.args, self.root)

    def test_unsupported_marker_override(self):
        self.args.loop_visualization = "true"
        with self.assertRaises(ValueError):
            benchmark.write_runtime_config(self.args, self.root)

    def test_invalid_parameter_group(self):
        self.document["/**"]["ros__parameters"]["mapping"] = False
        self.config.write_text(yaml.safe_dump(self.document))
        with self.assertRaises(ValueError):
            benchmark.write_runtime_config(self.args, self.root)

    def test_missing_or_empty_maps_do_not_match(self):
        self.assertFalse(compare.matching_maps({}, {}))
        missing = {"map": {"sha256": None}}
        self.assertFalse(compare.matching_maps(missing, missing))
        empty = {"map": {"exists": True, "sha256": "abc", "points": 0}}
        self.assertFalse(compare.matching_maps(empty, empty))
        path = self.root / "test.pcd"
        path.write_bytes(b"map fixture")
        present = {"map": {"exists": True, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                           "points": 5, "path": str(path)}}
        self.assertTrue(compare.matching_maps(present, present))
        path.unlink()
        self.assertFalse(compare.matching_maps(present, present))

    def test_other_disabled_sensor_is_not_disabled_lc(self):
        (self.root / "runtime_config.yaml").write_text(yaml.safe_dump(self.document))
        self.assertFalse(compare.disabled_loop(self.root))
        self.document["/**"]["ros__parameters"]["loop_closure"]["enable"] = False
        (self.root / "runtime_config.yaml").write_text(yaml.safe_dump(self.document))
        self.assertTrue(compare.disabled_loop(self.root))

    def test_empty_and_nonfinite_trajectories_fail(self):
        empty = {"odom_poses": np.empty((0, 4, 4))}
        self.assertEqual(compare.max_pose_difference(empty, empty), (None, None))
        bad = {"odom_poses": np.full((1, 4, 4), np.nan)}
        self.assertEqual(compare.max_pose_difference(bad, bad), (None, None))
        good = {"odom_poses": np.eye(4)[None, :, :]}
        self.assertEqual(compare.max_pose_difference(good, good), (0.0, 0.0))

    def test_missing_metrics_cannot_report_pass(self):
        with patch.object(compare, "parse_args", return_value=argparse.Namespace(
                baseline=self.root, candidate=self.root)):
            with self.assertRaises(FileNotFoundError):
                compare.main()

    def test_shared_timestamp_comparison_does_not_align_poses(self):
        left = {"odom_times": np.array([1.0, 2.0, 3.0]),
                "odom_stamps_ns": np.array([10**9, 2*10**9, 3*10**9]),
                "odom_frames": [("camera_init", "body")] * 3,
                "odom_poses": np.repeat(np.eye(4)[None], 3, axis=0)}
        right = {"odom_times": np.array([1.0, 3.0]),
                 "odom_stamps_ns": np.array([10**9, 3*10**9]),
                 "odom_frames": [("camera_init", "body")] * 2,
                 "odom_poses": np.repeat(np.eye(4)[None], 2, axis=0)}
        report = shared_pose_comparison(left, right)
        self.assertEqual(report["shared_samples"], 2)
        self.assertEqual(report["position_max_m"], 0.0)
        right["odom_poses"][1, 0, 3] = 0.2
        self.assertEqual(shared_pose_comparison(left, right)["position_max_m"], 0.2)
        right["odom_times"] += 10.0
        right["odom_stamps_ns"] += 10*10**9
        self.assertEqual(shared_pose_comparison(left, right)["shared_samples"], 0)

    def test_native_nanoseconds_and_frame_mismatch(self):
        stamp = 1780000000 * 10**9
        left = {"odom_times": np.array([1780000000.0]),
                "odom_stamps_ns": np.array([stamp]),
                "odom_frames": [("camera_init", "body")],
                "odom_poses": np.eye(4)[None]}
        right = dict(left, odom_stamps_ns=np.array([stamp + 1]))
        self.assertEqual(shared_pose_comparison(left, right)["shared_samples"], 0)
        right = dict(left, odom_frames=[("World", "body")])
        self.assertFalse(shared_pose_comparison(left, right)["frames_equal"])

    def test_manifest_map_evidence_and_standard_json(self):
        path = self.root / "test.pcd"
        path.write_bytes(b"map fixture")
        evidence = {"path": str(path), "exists": True, "points": 1,
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        metrics = {"map": evidence}
        (self.root / "manifest.json").write_text(json.dumps(metrics))
        self.assertTrue(compare.matching_maps(metrics, metrics, self.root, self.root))
        (self.root / "manifest.json").write_text("{}")
        self.assertFalse(compare.matching_maps(metrics, metrics, self.root, self.root))
        missing = compare.max_pose_difference({"odom_poses": []}, {"odom_poses": []})
        self.assertEqual(json.dumps(missing, allow_nan=False), "[null, null]")


if __name__ == "__main__":
    unittest.main()
