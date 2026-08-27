"""WoLF ROS 1 simulation acceptance unit and contract tests (M24-S1)."""

from __future__ import annotations

import unittest
from typing import Any, Dict, List

from wolf_provider_ros1 import (
    WolfRos1SimulationAcceptanceAdapter,
    WolfRos1SpawnSpec,
)


class TestWolfRos1SimulationAcceptanceAdapter(unittest.TestCase):
    def setUp(self) -> None:
        self.raw = {
            "entity_id": "ugv_1",
            "sim_time": 1.0,
            "position": [0.0, 0.0, 0.0],
            "linear_velocity": [0.0, 0.0, 0.0],
        }
        self.adapter = WolfRos1SimulationAcceptanceAdapter(lambda _entity: self.raw)

    def test_deterministic_spawn_and_all_observed_readiness_barriers(self) -> None:
        self.assertIn("seed:=24001", self.adapter.spawn_spec.as_launch_arguments())
        self.assertIn("roslaunch", self.adapter.spawn_spec.as_launch_arguments())
        ready = self.adapter.readiness({
            "world_ready": True,
            "entity_id": "ugv_1",
            "controller_active": True,
            "provider_ready": True,
            "edge_registered": True,
            "core_visible": True,
            "capabilities": ["GOTO", "HOLD", "TELEOP"],
            "command_channel_ready": True,
        })
        self.assertTrue(all(ready.values()))

    def test_gazebo_truth_is_normalized_and_clock_is_monotonic(self) -> None:
        state = self.adapter.get_state("ugv_1")
        self.assertEqual(state["source"], "gazebo_entity_state")
        self.assertEqual(state["entity_id"], "ugv_1")
        self.assertEqual(state["position"], [0.0, 0.0, 0.0])

        self.raw["sim_time"] = 0.5
        with self.assertRaisesRegex(ValueError, "backwards"):
            self.adapter.get_state("ugv_1")

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

    def test_scoped_lifo_cleanup_is_idempotent(self) -> None:
        released: List[str] = []
        self.adapter.register_cleanup("entity:ugv_1", lambda: released.append("entity"))
        self.adapter.register_cleanup("process:provider", lambda: released.append("provider"))
        self.assertEqual(self.adapter.cleanup(), ["process:provider", "entity:ugv_1"])
        self.assertEqual(released, ["provider", "entity"])
        self.assertEqual(self.adapter.cleanup(), [])

    def test_diagnostics_collection(self) -> None:
        self.adapter.collect_diagnostic("test_diag", {"metric": 42})
        diag = self.adapter.diagnostics
        self.assertEqual(len(diag), 1)
        self.assertEqual(diag[0]["name"], "test_diag")
        self.assertEqual(diag[0]["payload"]["metric"], 42)


if __name__ == "__main__":
    unittest.main()
