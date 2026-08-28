"""Deterministic M25 provider recovery and UGV bench-readiness tests."""

import unittest

from wolf_provider_ros1 import UgvBenchReadinessGate, WolfRos1RecoveryLedger


class TestWolfRos1RecoveryLedger(unittest.TestCase):
    def test_acceptance_is_checkpointed_before_physical_dispatch(self) -> None:
        order = []

        def write(checkpoint):
            order.append(("checkpoint", checkpoint["commands"]["cmd-1"]["state"]))

        ledger = WolfRos1RecoveryLedger(
            lambda _command: order.append(("dispatch", None)),
            lambda _reason: None,
            checkpoint_writer=write,
        )
        ledger.submit("cmd-1", {"action": "HOLD"})
        self.assertEqual(order[0], ("checkpoint", "ACCEPTED"))
        self.assertEqual(order[1], ("dispatch", None))
        self.assertEqual(order[2], ("checkpoint", "EXECUTING"))

    def test_restart_query_and_duplicate_delivery_do_not_repeat_effect(self) -> None:
        effects = []
        stops = []
        ledger = WolfRos1RecoveryLedger(effects.append, stops.append)
        first = ledger.submit("cmd-1", {"action": "GOTO", "x": 2.0})
        self.assertEqual(first["state"], "EXECUTING")

        restarted = WolfRos1RecoveryLedger(effects.append, stops.append, ledger.snapshot())
        duplicate = restarted.submit("cmd-1", {"action": "GOTO", "x": 2.0})
        self.assertTrue(duplicate["duplicate"])
        self.assertEqual(duplicate["effect_count"], 1)
        self.assertEqual(effects, [{"action": "GOTO", "x": 2.0}])
        self.assertEqual(restarted.query_status("cmd-1")["state"], "EXECUTING")

    def test_conflicting_duplicate_is_rejected(self) -> None:
        ledger = WolfRos1RecoveryLedger(lambda _command: None, lambda _reason: None)
        ledger.submit("cmd-1", {"action": "HOLD"})
        with self.assertRaisesRegex(ValueError, "different payload"):
            ledger.submit("cmd-1", {"action": "GOTO"})

    def test_provider_loss_safe_stops_once_and_requires_recovery(self) -> None:
        stops = []
        ledger = WolfRos1RecoveryLedger(lambda _command: None, stops.append)
        ledger.submit("cmd-1", {"action": "GOTO"})
        result = ledger.provider_unavailable("cmd-1")
        self.assertEqual(result["state"], "RECOVERY_REQUIRED")
        self.assertIn("EFFECT_UNKNOWN", result["detail"])
        ledger.provider_unavailable("cmd-1")
        self.assertEqual(stops, ["provider-unavailable:cmd-1"])

    def test_ambiguous_dispatch_is_not_retried_after_restart(self) -> None:
        effects = []
        stops = []

        def uncertain(command):
            effects.append(command)
            raise TimeoutError("ack lost")

        ledger = WolfRos1RecoveryLedger(uncertain, stops.append)
        result = ledger.submit("cmd-1", {"action": "GOTO"})
        self.assertEqual(result["state"], "RECOVERY_REQUIRED")
        restarted = WolfRos1RecoveryLedger(uncertain, stops.append, ledger.snapshot())
        restarted.submit("cmd-1", {"action": "GOTO"})
        self.assertEqual(len(effects), 1)
        self.assertEqual(stops, ["dispatch:cmd-1"])

    def test_corrupt_checkpoint_fails_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "corrupt"):
            WolfRos1RecoveryLedger(
                lambda _command: None,
                lambda _reason: None,
                {"schema_version": 1, "commands": []},
            )


class TestUgvBenchReadinessGate(unittest.TestCase):
    def test_absent_fixture_is_unavailable_not_passed(self) -> None:
        result = UgvBenchReadinessGate.assess({"fixture_available": False})
        self.assertEqual(result["status"], "UNAVAILABLE")
        self.assertFalse(result["ready"])

    def test_unrequested_bench_is_explicitly_skipped(self) -> None:
        result = UgvBenchReadinessGate.assess({}, requested=False)
        self.assertEqual(result["status"], "SKIPPED")
        self.assertFalse(result["ready"])

    def test_estop_and_safe_stop_are_mandatory(self) -> None:
        observation = {
            "fixture_available": True,
            "fixture_id": "ugv-bench-01",
            "reservation_owned": True,
            "fixture_isolated": True,
            "estop_ready": False,
            "safe_stop_verified": False,
            "controller_ready": True,
        }
        result = UgvBenchReadinessGate.assess(observation)
        self.assertEqual(result["status"], "NOT_READY")
        self.assertEqual(result["missing"], ["estop_ready", "safe_stop_verified"])

        observation.update(estop_ready=True, safe_stop_verified=True)
        self.assertEqual(UgvBenchReadinessGate.assess(observation)["status"], "READY")


if __name__ == "__main__":
    unittest.main()
