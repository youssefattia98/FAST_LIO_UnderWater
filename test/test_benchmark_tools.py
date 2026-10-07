#!/usr/bin/env python3
"""Fixture coverage for benchmark configuration and missing-evidence checks."""

import argparse
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
from compare_refactor_replays import shared_pose_comparison


class BenchmarkToolsTest(unittest.TestCase):
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
        present = {"map": {"exists": True, "sha256": "abc", "points": 5}}
        self.assertTrue(compare.matching_maps(present, present))

    def test_other_disabled_sensor_is_not_disabled_lc(self):
        (self.root / "runtime_config.yaml").write_text(yaml.safe_dump(self.document))
        self.assertFalse(compare.disabled_loop(self.root))
        self.document["/**"]["ros__parameters"]["loop_closure"]["enable"] = False
        (self.root / "runtime_config.yaml").write_text(yaml.safe_dump(self.document))
        self.assertTrue(compare.disabled_loop(self.root))

    def test_empty_and_nonfinite_trajectories_fail(self):
        empty = {"odom_poses": np.empty((0, 4, 4))}
        self.assertTrue(np.isinf(compare.max_pose_difference(empty, empty)).all())
        bad = {"odom_poses": np.full((1, 4, 4), np.nan)}
        self.assertTrue(np.isinf(compare.max_pose_difference(bad, bad)).all())
        good = {"odom_poses": np.eye(4)[None, :, :]}
        self.assertEqual(compare.max_pose_difference(good, good), (0.0, 0.0))

    def test_missing_metrics_cannot_report_pass(self):
        with patch.object(compare, "parse_args", return_value=argparse.Namespace(
                baseline=self.root, candidate=self.root)):
            with self.assertRaises(FileNotFoundError):
                compare.main()

    def test_shared_timestamp_comparison_does_not_align_poses(self):
        left = {"odom_times": np.array([1.0, 2.0, 3.0]),
                "odom_poses": np.repeat(np.eye(4)[None], 3, axis=0)}
        right = {"odom_times": np.array([1.0, 3.0]),
                 "odom_poses": np.repeat(np.eye(4)[None], 2, axis=0)}
        report = shared_pose_comparison(left, right)
        self.assertEqual(report["shared_samples"], 2)
        self.assertEqual(report["position_max_m"], 0.0)
        right["odom_poses"][1, 0, 3] = 0.2
        self.assertEqual(shared_pose_comparison(left, right)["position_max_m"], 0.2)
        right["odom_times"] += 10.0
        self.assertEqual(shared_pose_comparison(left, right)["shared_samples"], 0)


if __name__ == "__main__":
    unittest.main()
