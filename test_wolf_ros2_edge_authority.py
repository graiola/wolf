#!/usr/bin/env python3
import math
import os
import unittest
import yaml

class TestWolfRos2EdgeAuthority(unittest.TestCase):
    def setUp(self):
        self.manifest_path = os.path.join(os.path.dirname(__file__), "provider_manifest.yaml")
        self.assertTrue(os.path.exists(self.manifest_path), "provider_manifest.yaml must exist")
        with open(self.manifest_path, "r") as f:
            self.manifest = yaml.safe_load(f)

    def test_01_manifest_schema_and_hosting(self):
        self.assertEqual(self.manifest.get("schema_version"), "2.0")
        self.assertEqual(self.manifest.get("provider_id"), "wolf_provider_ros2")
        auth = self.manifest.get("authority_plane", {})
        self.assertEqual(auth.get("hosting"), "single_out_of_process")
        self.assertTrue(auth.get("direct_to_edge_teleop"))
        self.assertTrue(auth.get("epoch_fencing"))

    def test_02_map_context_aligned(self):
        auth = self.manifest.get("authority_plane", {})
        alignment = auth.get("map_context_alignment", {})
        aligned = alignment.get("aligned", {})
        self.assertTrue(aligned.get("enabled"))

    def test_03_map_context_degraded_speed_cap(self):
        auth = self.manifest.get("authority_plane", {})
        alignment = auth.get("map_context_alignment", {})
        degraded = alignment.get("degraded", {})
        cap = degraded.get("speed_resultant_cap_mps")
        self.assertIsNotNone(cap)
        self.assertAlmostEqual(cap, 0.5)

        # Validate velocity vector resultant scaling logic under DEGRADED alignment
        vx, vy = 0.6, 0.8
        resultant = math.hypot(vx, vy)
        self.assertAlmostEqual(resultant, 1.0)
        if resultant > cap:
            scale = cap / resultant
            vx_scaled = vx * scale
            vy_scaled = vy * scale
            scaled_resultant = math.hypot(vx_scaled, vy_scaled)
            self.assertAlmostEqual(scaled_resultant, 0.5)

    def test_04_map_context_unaligned_fallback(self):
        auth = self.manifest.get("authority_plane", {})
        alignment = auth.get("map_context_alignment", {})
        unaligned = alignment.get("unaligned", {})
        self.assertEqual(unaligned.get("fallback"), "local_frame")

    def test_05_defensive_goal_tolerance(self):
        limits = self.manifest.get("motion_limits", {})
        self.assertAlmostEqual(limits.get("goal_tolerance_xy_m"), 0.05)
        self.assertAlmostEqual(limits.get("goal_tolerance_yaw_rad"), 0.05)

if __name__ == "__main__":
    unittest.main()
