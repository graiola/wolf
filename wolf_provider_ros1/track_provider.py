"""M21-S5: WoLF ROS 1 Track Provider integration adapter.

Normalizes provider-native WoLF ROS 1 perception/detection feeds into canonical
TrackObservation format for TACTIX Edge ingest, preserves source identity
across reconnects, enforces sequence validation, and manages bounded caching
with map-epoch and TTL invalidation.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional, Tuple


class WolfRos1TrackProvider:
    """Production WoLF ROS 1 Track Provider adapter."""

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
        provider_id: str = "wolf_provider_ros1",
        source_id: str = "wolf_ros1",
        source_instance_id: str = "wolf_ros1_inst_01",
        ingest_policy: str = "FUSE_AS_OBSERVATION",
        authority_level: str = "STANDARD_MOT",
        max_age_seconds: float = 5.0,
        max_cache_entries: int = 256,
        map_id: str = "sector_1",
        map_epoch: int = 1,
        coordinate_convention: str = "ENU",
        default_frame_id: str = "map",
    ) -> None:
        if not source_id or not source_instance_id:
            raise ValueError("source_id and source_instance_id are required")
        if max_age_seconds <= 0:
            raise ValueError("max_age_seconds must be positive")
        if max_cache_entries <= 0:
            raise ValueError("max_cache_entries must be positive")

        self.provider_id = provider_id
        self.source_id = source_id
        self.source_instance_id = source_instance_id
        self.ingest_policy = ingest_policy
        self.authority_level = authority_level
        self.max_age_seconds = float(max_age_seconds)
        self.max_cache_entries = int(max_cache_entries)
        self.map_id = map_id
        self.map_epoch = int(map_epoch)
        self.coordinate_convention = coordinate_convention
        self.default_frame_id = default_frame_id

        self._sequence = 0
        self._cache: Dict[Tuple[str, str, str], Dict[str, Any]] = {}

    def get_source_descriptor(self) -> Dict[str, Any]:
        """Return the TrackSourceDescriptor dict for WoLF ROS 1 provider."""
        return {
            "source_id": self.source_id,
            "source_instance_id": self.source_instance_id,
            "provider_id": self.provider_id,
            "source_type": "ROBOT_LOCAL_ROS1_TRACKER",
            "authority_level": self.authority_level,
            "ingest_policy": self.ingest_policy,
            "coordinate_convention": self.coordinate_convention,
            "default_frame_id": self.default_frame_id,
            "map_id": self.map_id,
            "map_epoch": self.map_epoch,
            "max_age_seconds": self.max_age_seconds,
            "permitted_entity_types": list(self.ENTITY_TYPES),
            "authorization_policy": "ALLOW",
            "integrity_policy": "ALLOW",
        }

    def to_edge_descriptor(self) -> Any:
        """Return a TrackSourceDescriptor dataclass object or compatible dict for Edge ingest."""
        try:
            from tactix_edge.track.models import TrackSourceDescriptor
            return TrackSourceDescriptor(
                source_id=self.source_id,
                source_instance_id=self.source_instance_id,
                authority_level=self.authority_level,
                ingest_policy=self.ingest_policy,
                max_age_seconds=self.max_age_seconds,
                permitted_entity_types=list(self.ENTITY_TYPES),
                authorization_policy="ALLOW",
                integrity_policy="ALLOW",
                coordinate_convention=self.coordinate_convention,
                default_frame_id=self.default_frame_id,
                map_id=self.map_id,
                map_epoch=self.map_epoch,
            )
        except ImportError:
            return {
                "source_id": self.source_id,
                "source_instance_id": self.source_instance_id,
                "authority_level": self.authority_level,
                "ingest_policy": self.ingest_policy,
                "max_age_seconds": self.max_age_seconds,
                "permitted_entity_types": list(self.ENTITY_TYPES),
                "authorization_policy": "ALLOW",
                "integrity_policy": "ALLOW",
                "coordinate_convention": self.coordinate_convention,
                "default_frame_id": self.default_frame_id,
                "map_id": self.map_id,
                "map_epoch": self.map_epoch,
            }

    def get_supported_track_actions(self) -> List[str]:
        """Return supported M21 track action capabilities."""
        return list(self.ACTIONS)

    @staticmethod
    def _finite_vector(value: Any, length: int, field_name: str) -> List[float]:
        if not isinstance(value, (list, tuple)) or len(value) != length:
            raise ValueError(f"{field_name} must contain {length} values")
        res = [float(v) for v in value]
        if not all(math.isfinite(v) for v in res):
            raise ValueError(f"{field_name} must contain only finite values")
        return res

    def normalize_observation(
        self,
        raw_msg: Dict[str, Any],
        current_time: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Normalize a provider-native WoLF ROS 1 observation message into canonical TrackObservation form."""
        now = time.time() if current_time is None else float(current_time)
        if not math.isfinite(now):
            raise ValueError("current_time must be finite")

        msg_time = float(raw_msg.get("timestamp", now))
        if not math.isfinite(msg_time) or msg_time <= 0.0:
            raise ValueError(f"Invalid timestamp: {msg_time}")

        age = now - msg_time
        if age > self.max_age_seconds:
            raise ValueError(f"Observation stale: age {age:.2f}s exceeds max_age {self.max_age_seconds}s")
        if age < -1.0:
            raise ValueError(f"Observation timestamp in future: {msg_time}")

        raw_track_id = raw_msg.get("track_id")
        if raw_track_id is None or str(raw_track_id) == "":
            raise ValueError("track_id is required")
        source_track_id = str(raw_track_id)

        entity_type = str(raw_msg.get("entity_type", "VEHICLE"))
        if entity_type not in self.ENTITY_TYPES:
            raise ValueError(f"Unsupported entity_type: {entity_type}")

        msg_epoch = int(raw_msg.get("map_epoch", self.map_epoch))
        if msg_epoch != self.map_epoch:
            raise ValueError(f"Map epoch mismatch: msg epoch {msg_epoch} vs provider epoch {self.map_epoch}")

        pos = self._finite_vector(raw_msg.get("position", [0.0, 0.0, 0.0]), 3, "position")
        vel = self._finite_vector(raw_msg.get("velocity", [0.0, 0.0, 0.0]), 3, "velocity")
        cov = self._finite_vector(raw_msg.get("covariance", [0.1] * 6), 6, "covariance")
        if any(c < 0.0 for c in cov):
            raise ValueError("Covariance diagonal elements cannot be negative")

        self._sequence += 1

        obs = {
            "source_id": self.source_id,
            "source_instance_id": self.source_instance_id,
            "source_track_id": source_track_id,
            "source_sequence": self._sequence,
            "input_mode": str(raw_msg.get("input_mode", "OBSERVATION")),
            "entity_type": entity_type,
            "classification": str(raw_msg.get("classification", "UNCLASSIFIED")),
            "affiliation": str(raw_msg.get("affiliation", "NEUTRAL")),
            "frame_id": str(raw_msg.get("frame_id", self.default_frame_id)),
            "map_id": str(raw_msg.get("map_id", self.map_id)),
            "map_epoch": msg_epoch,
            "confidence": float(raw_msg.get("confidence", 0.9)),
            "timestamp_sec": msg_time,
            "timestamp": msg_time,
            "pose": {
                "x": pos[0],
                "y": pos[1],
                "z": pos[2],
                "yaw": float(raw_msg.get("heading", 0.0)),
                "covariance": cov,
            },
            "velocity": {
                "vx": vel[0],
                "vy": vel[1],
                "vz": vel[2],
                "covariance": [0.05] * 3,
            },
            "pose_with_covariance": {
                "position": pos,
                "orientation_rpy": [0.0, 0.0, float(raw_msg.get("heading", 0.0))],
                "covariance_diagonal": cov,
            },
            "velocity_with_covariance": {
                "linear": vel,
                "covariance_diagonal": [0.05] * 3,
            },
        }

        key = (self.source_id, self.source_instance_id, source_track_id)
        if key not in self._cache and len(self._cache) >= self.max_cache_entries:
            oldest_key = next(iter(self._cache))
            del self._cache[oldest_key]
        self._cache[key] = obs
        return obs

    def expire_stale_cache(
        self, current_time: float, map_epoch: Optional[int] = None
    ) -> List[Tuple[str, str, str]]:
        """Expire cached observations exceeding max age/TTL or invalidated by map epoch change."""
        expired: List[Tuple[str, str, str]] = []
        for key, obs in list(self._cache.items()):
            stale = (current_time - obs["timestamp_sec"]) > self.max_age_seconds
            epoch_changed = map_epoch is not None and obs["map_epoch"] != map_epoch
            if stale or epoch_changed:
                expired.append(key)
                del self._cache[key]
        return expired

    def process_ros1_object_feed(
        self, objects: List[Dict[str, Any]], stamp_sec: float, current_time: Optional[float] = None
    ) -> List[Dict[str, Any]]:
        """Process a list of ROS 1 detection/object message dicts into normalized observations."""
        observations = []
        for obj in objects:
            centroid = obj.get("centroid", [0.0, 0.0])
            raw = {
                "track_id": obj.get("id", obj.get("track_id")),
                "timestamp": stamp_sec,
                "position": obj.get("position", [float(centroid[0]), float(centroid[1]), 0.0]),
                "entity_type": obj.get("type", "VEHICLE"),
                "confidence": float(obj.get("score", 0.9)),
                "classification": obj.get("classification", "UNCLASSIFIED"),
            }
            obs = self.normalize_observation(raw, current_time=current_time)
            observations.append(obs)
        return observations
