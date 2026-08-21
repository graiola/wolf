"""M21-S5: WoLF ROS 1 Track Provider integration unit and contract tests.

Verifies that the WoLF ROS 1 Track Provider integration adheres to M21
canonical track entity semantics, ingest policies, observation normalization,
source descriptor contracts, freshness/expiration requirements, reconnect
identity preservation, sequence reconciliation, and map-epoch invalidation
when interacting with TACTIX Edge ingest gateways.
"""

from __future__ import annotations

import math
import sys
import time
import unittest
from pathlib import Path
from typing import Any, Dict, List, Optional

# Ensure tactix_deployment is on sys.path for Edge ingest gateway tests
candidates = [
    Path("/home/graiola/workspace/ai-workspaces/vendor_refactoring/tactix_deployment"),
]
p = Path(__file__).resolve()
for parent in [p] + list(p.parents):
    candidates.append(parent / "tactix_deployment")
    candidates.append(parent.parent / "tactix_deployment")

for cand in candidates:
    if cand.is_dir() and str(cand) not in sys.path:
        sys.path.insert(0, str(cand))
        break

# Import production Track Provider implementation
from wolf_provider_ros1 import WolfRos1TrackProvider

try:
    from tactix_edge.track.ingest import TrackIngestGateway
    from tactix_edge.track.models import TrackSourceDescriptor, IngestPolicy
    HAVE_EDGE = True
except ImportError:
    HAVE_EDGE = False


class TestWolfRos1TrackProvider(unittest.TestCase):
    def setUp(self) -> None:
        self.provider = WolfRos1TrackProvider()

    def test_01_source_descriptor_contract(self) -> None:
        desc = self.provider.get_source_descriptor()
        self.assertEqual(desc["source_id"], "wolf_ros1")
        self.assertEqual(desc["provider_id"], "wolf_provider_ros1")
        self.assertEqual(desc["ingest_policy"], "FUSE_AS_OBSERVATION")
        self.assertEqual(desc["authority_level"], "STANDARD_MOT")
        self.assertIn("VEHICLE", desc["permitted_entity_types"])
        self.assertEqual(desc["coordinate_convention"], "ENU")
        self.assertEqual(desc["default_frame_id"], "map")

    def test_02_supported_track_actions(self) -> None:
        actions = self.provider.get_supported_track_actions()
        expected = [
            "TRACK_TARGET_VISUAL",
            "STOP_TRACK_TARGET_VISUAL",
            "FOLLOW_GLOBAL_TRACK",
            "STOP_FOLLOW_GLOBAL_TRACK",
            "CREATE_ROI_FROM_TRACK",
            "USE_TRACK_AS_FORMATION_ANCHOR",
        ]
        for act in expected:
            self.assertIn(act, actions)

    def test_03_normalize_observation_success(self) -> None:
        raw = {
            "track_id": "trk_wolf_01",
            "position": [10.5, -2.0, 0.0],
            "velocity": [1.0, 0.5, 0.0],
            "heading": 0.3,
            "confidence": 0.92,
            "entity_type": "VEHICLE",
            "classification": "SUSPECT",
            "affiliation": "HOSTILE",
            "frame_id": "map",
            "timestamp": 1000.0,
        }
        obs = self.provider.normalize_observation(raw, current_time=1001.0)
        self.assertEqual(obs["source_id"], "wolf_ros1")
        self.assertEqual(obs["source_instance_id"], "wolf_ros1_inst_01")
        self.assertEqual(obs["source_track_id"], "trk_wolf_01")
        self.assertEqual(obs["source_sequence"], 1)
        self.assertEqual(obs["input_mode"], "OBSERVATION")
        self.assertEqual(obs["entity_type"], "VEHICLE")
        self.assertEqual(obs["classification"], "SUSPECT")
        self.assertEqual(obs["affiliation"], "HOSTILE")
        self.assertAlmostEqual(obs["confidence"], 0.92)
        self.assertEqual(obs["pose_with_covariance"]["position"], [10.5, -2.0, 0.0])

    def test_04_normalize_observation_stale_rejection(self) -> None:
        raw = {
            "track_id": "trk_stale",
            "timestamp": 1000.0,
        }
        with self.assertRaises(ValueError) as ctx:
            self.provider.normalize_observation(raw, current_time=1010.0)
        self.assertIn("Observation stale", str(ctx.exception))

    def test_05_normalize_observation_future_rejection(self) -> None:
        raw = {
            "track_id": "trk_future",
            "timestamp": 1005.0,
        }
        with self.assertRaises(ValueError) as ctx:
            self.provider.normalize_observation(raw, current_time=1000.0)
        self.assertIn("Observation timestamp in future", str(ctx.exception))

    def test_06_cache_expiration(self) -> None:
        raw = {
            "track_id": "trk_expire",
            "timestamp": 1000.0,
        }
        self.provider.normalize_observation(raw, current_time=1001.0)
        expired = self.provider.expire_stale_cache(current_time=1007.0)
        key = ("wolf_ros1", "wolf_ros1_inst_01", "trk_expire")
        self.assertIn(key, expired)
        self.assertNotIn(key, self.provider._cache)

    def test_07_reconnect_instance_preservation(self) -> None:
        provider2 = WolfRos1TrackProvider(source_instance_id="wolf_ros1_inst_02")
        self.assertNotEqual(self.provider.source_instance_id, provider2.source_instance_id)
        self.assertEqual(self.provider.source_id, provider2.source_id)

    def test_07b_cache_is_bounded(self) -> None:
        provider = WolfRos1TrackProvider(max_cache_entries=2)
        for track_id in ("oldest", "middle", "newest"):
            provider.normalize_observation(
                {"track_id": track_id, "timestamp": 1000.0},
                current_time=1001.0,
            )

        self.assertEqual(len(provider._cache), 2)
        self.assertNotIn(
            ("wolf_ros1", "wolf_ros1_inst_01", "oldest"), provider._cache
        )

    def test_08_ros1_object_feed_processing(self) -> None:
        objects = [
            {"id": 101, "centroid": [5.0, 3.0], "type": "VEHICLE", "score": 0.95},
            {"id": 102, "centroid": [12.0, -4.0], "type": "ROBOT", "score": 0.88},
        ]
        obs_list = self.provider.process_ros1_object_feed(objects, stamp_sec=1000.0, current_time=1001.0)
        self.assertEqual(len(obs_list), 2)
        self.assertEqual(obs_list[0]["source_track_id"], "101")
        self.assertEqual(obs_list[1]["source_track_id"], "102")

    @unittest.skipUnless(HAVE_EDGE, "tactix_edge package required for Edge gateway integration contract test")
    def test_09_edge_gateway_integration_reconnect_and_sequence(self) -> None:
        gateway = TrackIngestGateway()
        desc1 = self.provider.to_edge_descriptor()
        gateway.register_source(desc1)

        raw1 = {
            "track_id": "trk_01",
            "timestamp": 1000.0,
            "position": [1.0, 2.0, 0.0],
            "velocity": [0.1, 0.0, 0.0],
        }
        obs1 = self.provider.normalize_observation(raw1, current_time=1001.0)
        res1 = gateway.validate_and_ingest_observation(obs1, current_time=1001.0)
        self.assertTrue(res1.valid, f"Ingestion failed: {res1.reason}")

        # Out-of-order / duplicate sequence rejection on same instance
        duplicate_obs = dict(obs1)
        res_dup = gateway.validate_and_ingest_observation(duplicate_obs, current_time=1001.0)
        self.assertFalse(res_dup.valid)
        self.assertEqual(res_dup.reason, "OUT_OF_ORDER_OR_DUPLICATE_SEQUENCE")

        # Reconnect with new instance ID
        reconnected_provider = WolfRos1TrackProvider(source_instance_id="wolf_ros1_inst_02")
        desc2 = reconnected_provider.to_edge_descriptor()
        gateway.register_source(desc2)

        raw2 = {
            "track_id": "trk_01",
            "timestamp": 1002.0,
            "position": [1.2, 2.0, 0.0],
            "velocity": [0.1, 0.0, 0.0],
        }
        obs2 = reconnected_provider.normalize_observation(raw2, current_time=1003.0)
        # Sequence reset to 1 on new instance ID
        self.assertEqual(obs2["source_sequence"], 1)
        res2 = gateway.validate_and_ingest_observation(obs2, current_time=1003.0)
        self.assertTrue(res2.valid, f"Reconnected ingestion failed: {res2.reason}")

        # Stale instance ID rejection
        stale_instance_obs = dict(obs1)
        stale_instance_obs["source_sequence"] = 10
        res_stale_inst = gateway.validate_and_ingest_observation(stale_instance_obs, current_time=1003.0)
        self.assertFalse(res_stale_inst.valid)
        self.assertEqual(res_stale_inst.reason, "UNAUTHORIZED_SOURCE_INSTANCE")

    @unittest.skipUnless(HAVE_EDGE, "tactix_edge package required for Edge gateway integration contract test")
    def test_10_edge_gateway_map_epoch_invalidation(self) -> None:
        gateway = TrackIngestGateway()
        desc = self.provider.to_edge_descriptor()
        gateway.register_source(desc)

        raw_valid = {
            "track_id": "trk_map",
            "timestamp": 1000.0,
            "map_epoch": 1,
        }
        obs_valid = self.provider.normalize_observation(raw_valid, current_time=1001.0)
        res_valid = gateway.validate_and_ingest_observation(obs_valid, current_time=1001.0)
        self.assertTrue(res_valid.valid)

        # Mismatched map epoch rejection by provider during normalization
        raw_mismatch = {
            "track_id": "trk_map2",
            "timestamp": 1002.0,
            "map_epoch": 2,
        }
        with self.assertRaises(ValueError):
            self.provider.normalize_observation(raw_mismatch, current_time=1003.0)

        # Expire cache when map epoch changes
        expired = self.provider.expire_stale_cache(current_time=1003.0, map_epoch=2)
        key = ("wolf_ros1", "wolf_ros1_inst_01", "trk_map")
        self.assertIn(key, expired)


if __name__ == "__main__":
    unittest.main()
