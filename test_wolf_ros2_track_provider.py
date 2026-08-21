"""M21-S6 WoLF ROS 2 Track Provider contract implementation and tests.

The adapter deliberately accepts plain mappings at the provider boundary so ROS 2
message classes never leak into the provider-neutral Track contract.
"""

from __future__ import annotations

import math
import time
import unittest
from typing import Any, Dict, List, Optional, Tuple


class WolfRos2TrackProvider:
    """Normalize WoLF ROS 2 detector output for Edge track ingest."""

    ACTIONS = (
        "TRACK_TARGET_VISUAL",
        "STOP_TRACK_TARGET_VISUAL",
        "FOLLOW_GLOBAL_TRACK",
        "STOP_FOLLOW_GLOBAL_TRACK",
        "CREATE_ROI_FROM_TRACK",
        "USE_TRACK_AS_FORMATION_ANCHOR",
    )
    ENTITY_TYPES = ("VEHICLE", "ROBOT", "OBSTACLE", "UNKNOWN")

    def __init__(
        self,
        *,
        source_id: str = "wolf_ros2",
        source_instance_id: str = "wolf_ros2_boot_01",
        max_age_seconds: float = 5.0,
        map_id: str = "local_map",
        map_epoch: int = 1,
    ) -> None:
        if not source_id or not source_instance_id:
            raise ValueError("source and instance identities are required")
        if max_age_seconds <= 0:
            raise ValueError("max_age_seconds must be positive")
        self.source_id = source_id
        self.source_instance_id = source_instance_id
        self.max_age_seconds = float(max_age_seconds)
        self.map_id = map_id
        self.map_epoch = map_epoch
        self._sequence = 0
        self._cache: Dict[Tuple[str, str, str], Dict[str, Any]] = {}

    def get_source_descriptor(self) -> Dict[str, Any]:
        return {
            "source_id": self.source_id,
            "source_instance_id": self.source_instance_id,
            "provider_id": "wolf_provider_ros2",
            "source_type": "ROBOT_LOCAL_ROS2_TRACKER",
            "authority_level": "STANDARD_MOT",
            "ingest_policy": "FUSE_AS_OBSERVATION",
            "coordinate_convention": "ENU",
            "default_frame_id": "map",
            "map_id": self.map_id,
            "map_epoch": self.map_epoch,
            "max_age_seconds": self.max_age_seconds,
            "identity_namespace": self.source_id,
            "permitted_entity_types": list(self.ENTITY_TYPES),
            "authorization_policy": "authorized_edge_node",
            "integrity_policy": "strict_sequence_validation",
        }

    def get_supported_track_actions(self) -> List[str]:
        return list(self.ACTIONS)

    @staticmethod
    def _finite_vector(value: Any, length: int, field: str) -> List[float]:
        if not isinstance(value, (list, tuple)) or len(value) != length:
            raise ValueError(f"{field} must contain {length} values")
        result = [float(item) for item in value]
        if not all(math.isfinite(item) for item in result):
            raise ValueError(f"{field} must contain only finite values")
        return result

    def normalize_observation(
        self, raw_message: Dict[str, Any], *, current_time: Optional[float] = None
    ) -> Dict[str, Any]:
        """Return a Provider-SDK-shaped TrackObservation from native ROS 2 data."""
        now = time.time() if current_time is None else float(current_time)
        timestamp = float(raw_message.get("timestamp", now))
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        age = now - timestamp
        if age > self.max_age_seconds:
            raise ValueError("observation is stale")
        if age < -1.0:
            raise ValueError("observation timestamp is in the future")

        track_id = str(raw_message.get("track_id", ""))
        if not track_id:
            raise ValueError("track_id is required")
        entity_type = str(raw_message.get("entity_type", "UNKNOWN"))
        if entity_type not in self.ENTITY_TYPES:
            raise ValueError(f"unsupported entity type: {entity_type}")
        message_epoch = int(raw_message.get("map_epoch", self.map_epoch))
        if message_epoch != self.map_epoch:
            raise ValueError("map epoch mismatch")

        covariance = self._finite_vector(
            raw_message.get("covariance", [0.1] * 6), 6, "covariance"
        )
        if any(value < 0 for value in covariance):
            raise ValueError("covariance diagonal cannot be negative")
        position = self._finite_vector(raw_message.get("position", [0.0] * 3), 3, "position")
        velocity = self._finite_vector(raw_message.get("velocity", [0.0] * 3), 3, "velocity")

        self._sequence += 1
        observation = {
            "source_id": self.source_id,
            "source_instance_id": self.source_instance_id,
            "source_track_id": track_id,
            "source_sequence": self._sequence,
            "input_mode": "OBSERVATION",
            "entity_type": entity_type,
            "classification": str(raw_message.get("classification", "UNCLASSIFIED")),
            "affiliation": str(raw_message.get("affiliation", "UNKNOWN")),
            "pose_with_covariance": {
                "position": position,
                "orientation_rpy": [0.0, 0.0, float(raw_message.get("heading", 0.0))],
                "covariance_diagonal": covariance,
            },
            "velocity_with_covariance": {
                "linear": velocity,
                "covariance_diagonal": [0.05] * 3,
            },
            "confidence": float(raw_message.get("confidence", 0.0)),
            "status": str(raw_message.get("status", "ACTIVE")),
            "ttl_seconds": self.max_age_seconds,
            "timestamp_sec": timestamp,
            "frame_id": str(raw_message.get("frame_id", "map")),
            "map_id": str(raw_message.get("map_id", self.map_id)),
            "map_epoch": message_epoch,
            "metadata": dict(raw_message.get("metadata", {})),
        }
        key = (self.source_id, self.source_instance_id, track_id)
        self._cache[key] = observation
        return observation

    def expire_cache(self, *, current_time: float, map_epoch: Optional[int] = None) -> List[Tuple[str, str, str]]:
        """Remove entries invalidated by TTL or a changed map epoch."""
        expired: List[Tuple[str, str, str]] = []
        for key, observation in list(self._cache.items()):
            stale = current_time - observation["timestamp_sec"] > observation["ttl_seconds"]
            epoch_changed = map_epoch is not None and observation["map_epoch"] != map_epoch
            if stale or epoch_changed:
                expired.append(key)
                del self._cache[key]
        return expired


class TestWolfRos2TrackProvider(unittest.TestCase):
    def setUp(self) -> None:
        self.provider = WolfRos2TrackProvider()

    def test_source_registration_contract(self) -> None:
        descriptor = self.provider.get_source_descriptor()
        self.assertEqual(descriptor["provider_id"], "wolf_provider_ros2")
        self.assertEqual(descriptor["source_instance_id"], "wolf_ros2_boot_01")
        self.assertEqual(descriptor["ingest_policy"], "FUSE_AS_OBSERVATION")
        self.assertEqual(descriptor["coordinate_convention"], "ENU")

    def test_capability_model_is_explicit(self) -> None:
        self.assertEqual(set(self.provider.get_supported_track_actions()), set(self.provider.ACTIONS))

    def test_normalizes_ros2_data_without_vendor_types(self) -> None:
        observation = self.provider.normalize_observation(
            {
                "track_id": 42,
                "timestamp": 100.0,
                "position": [1, 2, 3],
                "velocity": [0.5, 0, 0],
                "covariance": [0.2] * 6,
                "entity_type": "VEHICLE",
                "classification": "CAR",
            },
            current_time=101.0,
        )
        self.assertEqual(observation["source_track_id"], "42")
        self.assertEqual(observation["source_sequence"], 1)
        self.assertEqual(observation["input_mode"], "OBSERVATION")
        self.assertEqual(observation["pose_with_covariance"]["position"], [1.0, 2.0, 3.0])
        self.assertTrue(all(isinstance(value, (str, int, float, list, dict)) for value in observation.values()))

    def test_rejects_stale_future_and_invalid_observations(self) -> None:
        invalid_messages = (
            ({"track_id": "old", "timestamp": 90.0}, "stale"),
            ({"track_id": "future", "timestamp": 103.0}, "future"),
            ({"track_id": "bad-type", "timestamp": 100.0, "entity_type": "ALIEN"}, "entity"),
            ({"track_id": "bad-cov", "timestamp": 100.0, "covariance": [-1.0] * 6}, "covariance"),
            ({"track_id": "bad-epoch", "timestamp": 100.0, "map_epoch": 2}, "epoch"),
        )
        for message, reason in invalid_messages:
            with self.subTest(reason=reason), self.assertRaises(ValueError):
                self.provider.normalize_observation(message, current_time=100.0)

    def test_cache_expires_on_ttl_and_map_epoch_change(self) -> None:
        self.provider.normalize_observation({"track_id": "ttl", "timestamp": 100.0}, current_time=100.0)
        ttl_key = ("wolf_ros2", "wolf_ros2_boot_01", "ttl")
        self.assertIn(ttl_key, self.provider.expire_cache(current_time=106.0))
        self.provider.normalize_observation({"track_id": "epoch", "timestamp": 110.0}, current_time=110.0)
        epoch_key = ("wolf_ros2", "wolf_ros2_boot_01", "epoch")
        self.assertIn(epoch_key, self.provider.expire_cache(current_time=110.0, map_epoch=2))

    def test_reconnect_preserves_source_and_changes_boot_identity(self) -> None:
        restarted = WolfRos2TrackProvider(source_instance_id="wolf_ros2_boot_02")
        self.assertEqual(restarted.source_id, self.provider.source_id)
        self.assertNotEqual(restarted.source_instance_id, self.provider.source_instance_id)
        self.assertEqual(restarted.normalize_observation({"track_id": "one"})["source_sequence"], 1)


if __name__ == "__main__":
    unittest.main()
