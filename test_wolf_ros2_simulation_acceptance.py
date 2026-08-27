"""WoLF ROS 2 provider adapter for the M23 runtime acceptance harness.

The adapter keeps simulator data independent from canonical provider feedback:
Gazebo entity state is injected through ``state_reader`` and is the only source
used for RT-002 physical-effect assertions.  The module has no ROS imports so
its contract tests remain deterministic on non-simulation CI runners.
"""

from __future__ import annotations

import math
import unittest
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence


TERMINAL_STATES = frozenset({"SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT"})


@dataclass(frozen=True)
class WolfRos2SpawnSpec:
    """Deterministic, provider-owned launch description consumed by M23."""

    entity_id: str
    namespace: str
    world: str
    seed: int
    ros_domain_id: int

    def as_launch_arguments(self) -> List[str]:
        return [
            "ros2", "launch", "wolf_controller", "wolf_controller_bringup.launch.xml",
            "robot_model:=spot", f"robot_name:={self.entity_id}",
            f"namespace:={self.namespace}", "use_sim_time:=true",
            f"world:={self.world}", f"seed:={self.seed}",
        ]


class WolfRos2SimulationAcceptanceAdapter:
    """Provider-owned RT-002 spawn, readiness, truth, and cleanup adapter."""

    REQUIRED_CAPABILITIES = frozenset({"GOTO", "HOLD"})

    def __init__(
        self,
        state_reader: Callable[[str], Mapping[str, Any]],
        *,
        entity_id: str = "ugv_1",
        namespace: str = "/ugv_1",
        world: str = "tactix_rt002.sdf",
        seed: int = 24002,
        ros_domain_id: int = 91,
    ) -> None:
        if not entity_id or namespace != f"/{entity_id}":
            raise ValueError("namespace must be uniquely scoped to entity_id")
        if not 0 <= ros_domain_id <= 232:
            raise ValueError("ROS domain id must be in [0, 232]")
        self.state_reader = state_reader
        self.spawn_spec = WolfRos2SpawnSpec(entity_id, namespace, world, seed, ros_domain_id)
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
        valid = (bool(states) and states[0] in {"ACCEPTED", "PENDING"}
                 and len(terminal) == 1 and states[-1] == terminal[0])
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

    def verify_hold(
        self,
        final: Mapping[str, Any],
        lifecycle: Iterable[Mapping[str, Any]],
        *,
        max_speed_mps: float = 0.05,
    ) -> Dict[str, Any]:
        """Require both an independently observed stop and successful completion."""
        truth_result = self.verify_effect({"max_speed_mps": max_speed_mps}, final)
        lifecycle_result = self.verify_command_lifecycle(lifecycle)
        matches = (truth_result["matches"] and lifecycle_result["matches"]
                   and lifecycle_result["terminal_states"] == ["SUCCEEDED"])
        return {"matches": matches, "truth": truth_result,
                "lifecycle": lifecycle_result}

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


class TestWolfRos2SimulationAcceptanceAdapter(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = {"entity_id": "ugv_1", "sim_time": 1.0,
                    "position": [0.0, 0.0, 0.0], "linear_velocity": [0.0, 0.0, 0.0]}
        self.adapter = WolfRos2SimulationAcceptanceAdapter(lambda _entity: self.raw)

    def test_deterministic_spawn_and_all_observed_readiness_barriers(self) -> None:
        arguments = self.adapter.spawn_spec.as_launch_arguments()
        self.assertIn("seed:=24002", arguments)
        self.assertIn("use_sim_time:=true", arguments)
        observation = {
            "world_ready": True, "entity_id": "ugv_1", "controller_active": True,
            "provider_ready": True, "edge_registered": True, "core_visible": True,
            "capabilities": ["GOTO", "HOLD", "TELEOP"], "command_channel_ready": True,
        }
        ready = self.adapter.readiness(observation)
        self.assertTrue(all(ready.values()))
        barrier_inputs = {
            "WORLD_READY": ("world_ready", False),
            "ENTITY_SPAWNED": ("entity_id", "another_entity"),
            "BACKEND_CONNECTED": ("controller_active", False),
            "PROVIDER_READY": ("provider_ready", False),
            "EDGE_REGISTERED": ("edge_registered", False),
            "ASSET_VISIBLE_IN_CORE": ("core_visible", False),
            "CAPABILITY_AVAILABLE": ("capabilities", ["GOTO"]),
            "COMMAND_CHANNEL_READY": ("command_channel_ready", False),
        }
        for barrier, (field, value) in barrier_inputs.items():
            with self.subTest(barrier=barrier):
                not_ready = dict(observation)
                not_ready[field] = value
                result = self.adapter.readiness(not_ready)
                self.assertFalse(result[barrier])
                self.assertTrue(all(state for name, state in result.items()
                                    if name != barrier))

    def test_constructor_rejects_namespace_and_domain_boundaries(self) -> None:
        reader = lambda _entity: self.raw
        for entity, namespace, domain in [
            ("", "/", 91), ("ugv_1", "/shared", 91),
            ("ugv_1", "/ugv_1", -1), ("ugv_1", "/ugv_1", 233),
        ]:
            with self.subTest(entity=entity, namespace=namespace, domain=domain):
                with self.assertRaises(ValueError):
                    WolfRos2SimulationAcceptanceAdapter(
                        reader, entity_id=entity, namespace=namespace,
                        ros_domain_id=domain)
        for domain in (0, 232):
            self.assertEqual(WolfRos2SimulationAcceptanceAdapter(
                reader, ros_domain_id=domain).spawn_spec.ros_domain_id, domain)

    def test_gazebo_truth_is_normalized_and_clock_is_monotonic(self) -> None:
        state = self.adapter.get_state("ugv_1")
        self.assertEqual(state["source"], "gazebo_entity_state")
        self.assertEqual(state["speed_mps"], 0.0)
        self.raw["sim_time"] = 1.0
        self.assertEqual(self.adapter.get_state("ugv_1")["sim_time"], 1.0)
        self.raw["sim_time"] = 0.5
        with self.assertRaisesRegex(ValueError, "backwards"):
            self.adapter.get_state("ugv_1")

    def test_gazebo_truth_rejects_wrong_entity_and_malformed_values(self) -> None:
        with self.assertRaises(KeyError):
            self.adapter.get_state("ugv_2")
        invalid_cases = [
            ("entity_id", "ugv_2", "identity mismatch"),
            ("sim_time", math.inf, "time must be finite"),
            ("position", [0, 1], "three values"),
            ("position", "0,0,0", "three values"),
            ("linear_velocity", [0, math.nan, 0], "must be finite"),
        ]
        for field, value, message in invalid_cases:
            with self.subTest(field=field, value=value):
                self.raw = {"entity_id": "ugv_1", "sim_time": 1.0,
                            "position": [0, 0, 0], "linear_velocity": [0, 0, 0]}
                self.raw[field] = value
                self.adapter._last_sim_time = None
                with self.assertRaisesRegex(ValueError, message):
                    self.adapter.get_state("ugv_1")

    def test_effect_comparison_boundaries_and_diffs(self) -> None:
        at_position_limit = self.adapter.verify_effect(
            {"position": [0, 0, 0]}, {"position": [0.25, 0, 0]},
            {"position_m": 0.25})
        at_speed_limit = self.adapter.verify_effect(
            {"max_speed_mps": 0.05}, {"speed_mps": 0.05})
        self.assertTrue(at_position_limit["matches"])
        self.assertTrue(at_speed_limit["matches"])
        failed = self.adapter.verify_effect(
            {"position": [0, 0, 0], "max_speed_mps": 0.05},
            {"position": [0.26, 0, 0], "speed_mps": math.inf},
            {"position_m": 0.25})
        self.assertEqual(set(failed["diffs"]), {"position", "speed_mps"})

    def test_goto_requires_effect_and_exactly_one_success_terminal(self) -> None:
        initial = {"position": [0.0, 0.0, 0.0]}
        final = {"position": [1.0, 0.0, 0.0], "speed_mps": 0.0}
        events = [{"state": "ACCEPTED"}, {"state": "EXECUTING"}, {"state": "SUCCEEDED"}]
        self.assertTrue(self.adapter.verify_goto(initial, final, [1, 0, 0], events)["matches"])
        false_success = self.adapter.verify_goto(initial, initial, [1, 0, 0], events)
        self.assertFalse(false_success["matches"])
        duplicate = events + [{"state": "SUCCEEDED"}]
        self.assertFalse(self.adapter.verify_goto(initial, final, [1, 0, 0], duplicate)["matches"])

    def test_effect_without_success_and_hold_motion_fail(self) -> None:
        effect_without_status = self.adapter.verify_goto(
            {"position": [0, 0, 0]}, {"position": [1, 0, 0]}, [1, 0, 0],
            [{"state": "ACCEPTED"}, {"state": "EXECUTING"}],
        )
        self.assertFalse(effect_without_status["matches"])
        self.assertFalse(self.adapter.verify_effect(
            {"max_speed_mps": 0.05}, {"speed_mps": 0.06})["matches"])

    def test_command_lifecycle_boundaries(self) -> None:
        cases = [
            ([], False),
            ([{"state": "EXECUTING"}, {"state": "SUCCEEDED"}], False),
            ([{"state": "PENDING"}, {"state": "SUCCEEDED"}], True),
            ([{"state": "ACCEPTED"}, {"state": "FAILED"}], True),
            ([{"state": "ACCEPTED"}, {"state": "SUCCEEDED"},
              {"state": "EXECUTING"}], False),
            ([{"state": "ACCEPTED"}, {"state": "FAILED"},
              {"state": "SUCCEEDED"}], False),
        ]
        for events, expected in cases:
            with self.subTest(events=events):
                self.assertEqual(
                    self.adapter.verify_command_lifecycle(events)["matches"], expected)

    def test_goto_displacement_and_tolerance_boundaries(self) -> None:
        success = [{"state": "ACCEPTED"}, {"state": "SUCCEEDED"}]
        result = self.adapter.verify_goto(
            {"position": [0, 0, 0]}, {"position": [0.1, 0, 0]},
            [0.1, 0, 0], success, tolerance_m=0,
            minimum_displacement_m=0.1)
        self.assertTrue(result["matches"])
        failed_terminal = self.adapter.verify_goto(
            {"position": [0, 0, 0]}, {"position": [1, 0, 0]},
            [1, 0, 0], [{"state": "ACCEPTED"}, {"state": "FAILED"}])
        self.assertFalse(failed_terminal["matches"])

    def test_hold_requires_truth_and_exactly_one_success_terminal(self) -> None:
        success = [{"state": "PENDING"}, {"state": "EXECUTING"},
                   {"state": "SUCCEEDED"}]
        self.assertTrue(self.adapter.verify_hold(
            {"speed_mps": 0.05}, success, max_speed_mps=0.05)["matches"])
        for final, events in [
            ({"speed_mps": 0.051}, success),
            ({"speed_mps": 0.0}, [{"state": "PENDING"}, {"state": "FAILED"}]),
            ({"speed_mps": 0.0}, success + [{"state": "SUCCEEDED"}]),
            ({"speed_mps": 0.0}, [{"state": "PENDING"}]),
        ]:
            with self.subTest(final=final, events=events):
                self.assertFalse(self.adapter.verify_hold(final, events)["matches"])

    def test_diagnostics_are_copied_and_do_not_affect_results(self) -> None:
        payload = {"controller": "healthy"}
        self.adapter.collect_diagnostic("controller", payload)
        payload["controller"] = "mutated"
        diagnostics = self.adapter.diagnostics
        diagnostics.clear()
        self.assertEqual(self.adapter.diagnostics,
                         [{"name": "controller",
                           "payload": {"controller": "healthy"}}])
        self.assertTrue(self.adapter.verify_hold(
            {"speed_mps": 0.0},
            [{"state": "ACCEPTED"}, {"state": "SUCCEEDED"}])["matches"])

    def test_cleanup_registration_rejects_empty_and_duplicate_ids(self) -> None:
        with self.assertRaises(ValueError):
            self.adapter.register_cleanup("", lambda: None)
        self.adapter.register_cleanup("entity:ugv_1", lambda: None)
        with self.assertRaises(ValueError):
            self.adapter.register_cleanup("entity:ugv_1", lambda: None)

    def test_scoped_lifo_cleanup_is_idempotent(self) -> None:
        released: List[str] = []
        self.adapter.register_cleanup("entity:ugv_1", lambda: released.append("entity"))
        self.adapter.register_cleanup("process:provider", lambda: released.append("provider"))
        self.assertEqual(self.adapter.cleanup(), ["process:provider", "entity:ugv_1"])
        self.assertEqual(released, ["provider", "entity"])
        self.assertEqual(self.adapter.cleanup(), [])


if __name__ == "__main__":
    unittest.main()
