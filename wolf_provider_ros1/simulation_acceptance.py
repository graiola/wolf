"""WoLF ROS 1 provider adapter for the M23/M24 runtime acceptance harness.

The adapter keeps simulator data independent from canonical provider feedback:
Gazebo entity state is injected through ``state_reader`` and is the only source
used for RT-002 physical-effect assertions. The module has no ROS imports so
its contract tests remain deterministic on non-simulation CI runners.
"""

from __future__ import annotations

import math
import unittest
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence


TERMINAL_STATES = frozenset({"SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT"})


@dataclass(frozen=True)
class WolfRos1SpawnSpec:
    """Deterministic, provider-owned launch description consumed by M23/M24."""

    entity_id: str
    namespace: str
    world: str
    seed: int
    ros_domain_id: int

    def as_launch_arguments(self) -> List[str]:
        return [
            "roslaunch", "wolf_controller", "wolf_controller_bringup.launch",
            "robot_model:=spot", f"robot_name:={self.entity_id}",
            f"namespace:={self.namespace}", "use_sim_time:=true",
            f"world:={self.world}", f"seed:={self.seed}",
        ]


class WolfRos1SimulationAcceptanceAdapter:
    """Provider-owned RT-002 spawn, readiness, truth, and cleanup adapter."""

    REQUIRED_CAPABILITIES = frozenset({"GOTO", "HOLD"})

    def __init__(
        self,
        state_reader: Callable[[str], Mapping[str, Any]],
        *,
        entity_id: str = "ugv_1",
        namespace: str = "/ugv_1",
        world: str = "tactix_rt002.world",
        seed: int = 24001,
        ros_domain_id: int = 91,
    ) -> None:
        if not entity_id or namespace != f"/{entity_id}":
            raise ValueError("namespace must be uniquely scoped to entity_id")
        if not 0 <= ros_domain_id <= 232:
            raise ValueError("ROS domain id must be in [0, 232]")
        self.state_reader = state_reader
        self.spawn_spec = WolfRos1SpawnSpec(entity_id, namespace, world, seed, ros_domain_id)
        self._last_sim_time: Optional[float] = None
        self._cleanup: List[tuple[str, Callable[[], None]]] = []
        self._diagnostics: List[Dict[str, Any]] = []

    def readiness(self, observation: Mapping[str, Any]) -> Dict[str, bool]:
        """Report observed barriers; elapsed time is never accepted as readiness."""
        capabilities = set(observation.get("capabilities", ()))
        return {
            "WORLD_READY": bool(observation.get("world_ready")),
            "ENTITY_SPAWNED": observation.get("entity_id") == self.spawn_spec.entity_id,
            "BACKEND_CONNECTED": bool(observation.get("controller_active")),
            "PROVIDER_READY": bool(observation.get("provider_ready")),
            "EDGE_REGISTERED": bool(observation.get("edge_registered")),
            "ASSET_VISIBLE_IN_CORE": bool(observation.get("core_visible")),
            "CAPABILITY_AVAILABLE": self.REQUIRED_CAPABILITIES <= capabilities,
            "COMMAND_CHANNEL_READY": bool(observation.get("command_channel_ready")),
        }

    @staticmethod
    def _vector(value: Any, field: str) -> List[float]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
            raise ValueError(f"{field} must contain three values")
        result = [float(item) for item in value]
        if not all(math.isfinite(item) for item in result):
            raise ValueError(f"{field} must be finite")
        return result

    def get_state(self, entity_id: str) -> Dict[str, Any]:
        """Read independent Gazebo truth and normalize it to the M23 interface."""
        if entity_id != self.spawn_spec.entity_id:
            raise KeyError(f"adapter does not own entity {entity_id!r}")
        raw = self.state_reader(entity_id)
        if raw.get("entity_id") != entity_id:
            raise ValueError("Gazebo response identity mismatch")
        sim_time = float(raw["sim_time"])
        if not math.isfinite(sim_time):
            raise ValueError("simulator time must be finite")
        if self._last_sim_time is not None and sim_time < self._last_sim_time:
            raise ValueError("simulator clock moved backwards")
        self._last_sim_time = sim_time
        velocity = self._vector(raw["linear_velocity"], "linear_velocity")
        return {
            "entity_id": entity_id,
            "sim_time": sim_time,
            "position": self._vector(raw["position"], "position"),
            "linear_velocity": velocity,
            "speed_mps": math.sqrt(sum(component * component for component in velocity)),
            "source": "gazebo_entity_state",
        }

    def verify_effect(
        self,
        expected_state: Mapping[str, Any],
        actual_state: Mapping[str, Any],
        tolerance: Optional[Mapping[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Implement the M23 ground-truth comparison interface."""
        tolerance = tolerance or {}
        diffs: Dict[str, Any] = {}
        if "position" in expected_state:
            expected = self._vector(expected_state["position"], "expected.position")
            actual = self._vector(actual_state.get("position"), "actual.position")
            error = math.dist(expected, actual)
            limit = float(tolerance.get("position_m", 0.25))
            if error > limit:
                diffs["position"] = {"error_m": error, "tolerance_m": limit}
        if "max_speed_mps" in expected_state:
            speed = float(actual_state.get("speed_mps", math.inf))
            limit = float(expected_state["max_speed_mps"])
            if not math.isfinite(speed) or speed > limit:
                diffs["speed_mps"] = {"actual": speed, "maximum": limit}
        return {"matches": not diffs, "diffs": diffs,
                "expected": dict(expected_state), "actual": dict(actual_state)}

    @staticmethod
    def verify_command_lifecycle(events: Iterable[Mapping[str, Any]]) -> Dict[str, Any]:
        states = [str(event.get("state")) for event in events]
        terminal = [state for state in states if state in TERMINAL_STATES]
        valid = bool(states) and states[0] in {"ACCEPTED", "PENDING"} and len(terminal) == 1
        return {"matches": valid, "states": states, "terminal_states": terminal}

    def verify_goto(
        self,
        initial: Mapping[str, Any],
        final: Mapping[str, Any],
        target: Sequence[float],
        lifecycle: Iterable[Mapping[str, Any]],
        *,
        tolerance_m: float = 0.25,
        minimum_displacement_m: float = 0.10,
    ) -> Dict[str, Any]:
        position_result = self.verify_effect(
            {"position": target}, final, {"position_m": tolerance_m}
        )
        displacement = math.dist(
            self._vector(initial.get("position"), "initial.position"),
            self._vector(final.get("position"), "final.position"),
        )
        lifecycle_result = self.verify_command_lifecycle(lifecycle)
        matches = (position_result["matches"] and displacement >= minimum_displacement_m
                   and lifecycle_result["matches"]
                   and lifecycle_result["terminal_states"] == ["SUCCEEDED"])
        return {"matches": matches, "displacement_m": displacement,
                "truth": position_result, "lifecycle": lifecycle_result}

    def register_cleanup(self, resource_id: str, callback: Callable[[], None]) -> None:
        if not resource_id or any(existing == resource_id for existing, _ in self._cleanup):
            raise ValueError("cleanup resource ids must be non-empty and unique")
        self._cleanup.append((resource_id, callback))

    def cleanup(self) -> List[str]:
        """Release only resources explicitly registered by this adapter, in reverse order."""
        released: List[str] = []
        while self._cleanup:
            resource_id, callback = self._cleanup.pop()
            callback()
            released.append(resource_id)
        return released

    def collect_diagnostic(self, name: str, payload: Mapping[str, Any]) -> None:
        """Retain provider diagnostics as evidence, never as pass/fail input."""
        self._diagnostics.append({"name": name, "payload": dict(payload)})

    @property
    def diagnostics(self) -> List[Dict[str, Any]]:
        return list(self._diagnostics)
