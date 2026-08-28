"""M25-S4 WoLF ROS 2 provider resilience and UGV bench-readiness tests.

The provider-acceptance safety core is implemented inline so the suite imports
only the standard library, stays independently discoverable via
``python3 -m unittest``, and never pulls ROS 2 or vendor message types into the
admission path.

Safety contract exercised here:

* watchdog/heartbeat-driven safe-stop must prevent unsafe motion (F001) — the
  watchdog freshness gate is evaluated *before* bench readiness or replay
  fencing, so a stale heartbeat can never be masked by a later validation, and a
  tripped watchdog latches SAFE_STOP durably;
* durable replay fencing (F002) — sequence numbers are mandatory and an
  unbounded high-water mark fences replays even after they are evicted from the
  bounded dedup FIFO, while transient rejections stay retriable.
"""

from __future__ import annotations

import math
import unittest
from collections import deque
from typing import Any, Deque, Dict, Optional, Set

ACTIVE = "ACTIVE"
SAFE_STOP = "SAFE_STOP"


class AdmissionResult:
    """Outcome of a command admission decision at the provider boundary."""

    __slots__ = ("accepted", "reason", "mode")

    def __init__(self, accepted: bool, reason: str, mode: str) -> None:
        self.accepted = accepted
        self.reason = reason
        self.mode = mode

    def __repr__(self) -> str:  # pragma: no cover - diagnostics only
        return f"AdmissionResult(accepted={self.accepted!r}, reason={self.reason!r}, mode={self.mode!r})"


class UgvProviderRuntime:
    """Minimal UGV provider admission core with watchdog-gated safe-stop.

    The runtime maintains a single monotonic timeline shared by heartbeats and
    commands. ``accept_command`` refuses motion whenever the watchdog is stale or
    latched, and only records sequence numbers for commands it actually accepts.
    """

    def __init__(
        self,
        *,
        watchdog_timeout: float = 1.0,
        dedup_history: int = 16,
        bench_ready: bool = True,
    ) -> None:
        if not math.isfinite(watchdog_timeout) or watchdog_timeout <= 0:
            raise ValueError("watchdog_timeout must be positive and finite")
        if dedup_history < 1:
            raise ValueError("dedup_history must be >= 1")
        self.watchdog_timeout = float(watchdog_timeout)
        self.dedup_history = int(dedup_history)
        self._bench_ready = bool(bench_ready)
        self._now = 0.0
        self._clock_initialized = False
        self._last_heartbeat: Optional[float] = None
        self._safe_stop_latched = False
        self._mode = ACTIVE
        self._seen: Deque[int] = deque(maxlen=self.dedup_history)
        self._seen_set: Set[int] = set()
        self._last_accepted_seq: Optional[int] = None

    # -- clock -----------------------------------------------------------------

    def _advance_clock(self, timestamp: Any) -> float:
        """Validate and advance the shared timeline.

        Rejects non-finite (NaN/inf) and non-monotonic timestamps so a corrupt
        clock can never be interpreted as a fresh heartbeat.
        """
        value = float(timestamp)
        if not math.isfinite(value):
            raise ValueError("time must be finite")
        if self._clock_initialized and value < self._now:
            raise ValueError("time must be monotonic non-decreasing")
        self._now = value
        self._clock_initialized = True
        return value

    # -- heartbeat / watchdog --------------------------------------------------

    def heartbeat(self, timestamp: Any) -> float:
        """Record a heartbeat. Non-finite/non-monotonic beats are never stored."""
        value = self._advance_clock(timestamp)
        self._last_heartbeat = value
        return value

    def _heartbeat_stale(self, now: float) -> bool:
        last = self._last_heartbeat
        if last is None:
            return True
        if not math.isfinite(last) or not math.isfinite(now):
            return True
        return (now - last) > self.watchdog_timeout

    def check_watchdog(self, now: Optional[float] = None) -> bool:
        """Atomically evaluate watchdog freshness, latching SAFE_STOP on failure.

        Returns ``True`` only when motion is currently safe (not latched and the
        heartbeat is fresh). Once latched, always returns ``False`` until
        :meth:`rearm`.
        """
        if now is None:
            now = self._now
        if self._safe_stop_latched:
            return False
        if self._heartbeat_stale(now):
            self._latch_safe_stop()
            return False
        return True

    def _latch_safe_stop(self) -> None:
        self._safe_stop_latched = True
        self._mode = SAFE_STOP

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def safe_stop_latched(self) -> bool:
        return self._safe_stop_latched

    def rearm(self, timestamp: Any) -> float:
        """Clear a latched SAFE_STOP, requiring a fresh heartbeat and a ready bench."""
        now = self._advance_clock(timestamp)
        if self._heartbeat_stale(now):
            raise RuntimeError("cannot re-arm with a stale heartbeat")
        if not self._bench_ready:
            raise RuntimeError("cannot re-arm until the bench is ready")
        self._safe_stop_latched = False
        self._mode = ACTIVE
        return now

    def set_bench_ready(self, ready: bool) -> None:
        self._bench_ready = bool(ready)

    # -- sequence fencing ------------------------------------------------------

    @staticmethod
    def _require_sequence(command: Dict[str, Any]) -> int:
        if "sequence" not in command:
            raise ValueError("command sequence is required")
        seq = command["sequence"]
        # bool is a subclass of int; reject it explicitly so flags cannot pose as
        # sequence numbers.
        if seq is None or isinstance(seq, bool) or not isinstance(seq, int):
            raise ValueError("command sequence must be an integer")
        return seq

    def _record_accepted(self, seq: int) -> None:
        self._seen.append(seq)  # bounded FIFO auto-evicts the oldest
        self._seen_set = set(self._seen)
        if self._last_accepted_seq is None or seq > self._last_accepted_seq:
            self._last_accepted_seq = seq

    # -- admission -------------------------------------------------------------

    def accept_command(self, command: Dict[str, Any]) -> AdmissionResult:
        """Admit or refuse a command.

        Validation order is safety-critical: the watchdog gate is evaluated
        before bench readiness and replay fencing so a stale/latched watchdog
        always wins and can never be masked by another condition passing.
        """
        seq = self._require_sequence(command)
        if "time" not in command:
            raise ValueError("command time is required")
        now = self._advance_clock(command["time"])

        # 1. WATCHDOG GATE — first, atomically. Latches SAFE_STOP on staleness.
        if not self.check_watchdog(now):
            return AdmissionResult(False, "watchdog_timeout", SAFE_STOP)

        # 2. Bench readiness.
        if not self._bench_ready:
            return AdmissionResult(False, "bench_not_ready", self._mode)

        # 3. Durable replay fencing (unbounded high-water mark + bounded FIFO).
        if self._last_accepted_seq is not None and seq <= self._last_accepted_seq:
            reason = "duplicate" if seq in self._seen_set else "out_of_order"
            return AdmissionResult(False, reason, self._mode)

        # 4. Accept — record only accepted sequences so rejections stay retriable.
        self._record_accepted(seq)
        return AdmissionResult(True, "accepted", self._mode)

    def restart(self) -> None:
        """Simulate a provider process restart: transient state and latch cleared."""
        self._now = 0.0
        self._clock_initialized = False
        self._last_heartbeat = None
        self._safe_stop_latched = False
        self._mode = ACTIVE
        self._seen.clear()
        self._seen_set.clear()
        self._last_accepted_seq = None


def _cmd(sequence: int, time: float, **extra: Any) -> Dict[str, Any]:
    command = {"sequence": sequence, "time": time}
    command.update(extra)
    return command


class TestWatchdogSafeStop(unittest.TestCase):
    """F001 — watchdog/heartbeat-driven safe-stop must prevent unsafe motion."""

    def test_fresh_heartbeat_authorizes_command(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        rt.heartbeat(0.0)
        result = rt.accept_command(_cmd(1, 0.5))
        self.assertTrue(result.accepted)
        self.assertEqual(result.mode, ACTIVE)

    def test_stale_heartbeat_blocks_command_admission(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        rt.heartbeat(0.0)
        result = rt.accept_command(_cmd(1, 100.0))
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "watchdog_timeout")
        self.assertEqual(result.mode, SAFE_STOP)
        self.assertTrue(rt.safe_stop_latched)

    def test_missing_heartbeat_is_treated_as_stale(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        result = rt.accept_command(_cmd(1, 0.0))
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "watchdog_timeout")

    def test_watchdog_gate_precedes_bench_readiness(self) -> None:
        # Bench not ready AND heartbeat stale: the watchdog must win, proving it
        # is evaluated before the bench-readiness check.
        rt = UgvProviderRuntime(watchdog_timeout=1.0, bench_ready=False)
        result = rt.accept_command(_cmd(1, 5.0))
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "watchdog_timeout")

    def test_watchdog_trip_after_accepted_command_latches_safe_stop(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        rt.heartbeat(0.0)
        self.assertTrue(rt.accept_command(_cmd(1, 0.5)).accepted)
        # No further heartbeat; time advances past the watchdog window.
        tripped = rt.accept_command(_cmd(2, 100.0))
        self.assertFalse(tripped.accepted)
        self.assertEqual(tripped.reason, "watchdog_timeout")
        self.assertTrue(rt.safe_stop_latched)

    def test_safe_stop_latch_persists_until_rearm(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        rt.heartbeat(0.0)
        rt.accept_command(_cmd(1, 0.5))
        rt.accept_command(_cmd(2, 100.0))  # trips + latches
        # A fresh heartbeat alone must NOT clear the latch.
        rt.heartbeat(100.0)
        still_denied = rt.accept_command(_cmd(3, 100.0))
        self.assertFalse(still_denied.accepted)
        self.assertEqual(still_denied.reason, "watchdog_timeout")
        # Explicit re-arm restores motion.
        rt.rearm(100.0)
        self.assertFalse(rt.safe_stop_latched)
        recovered = rt.accept_command(_cmd(4, 100.0))
        self.assertTrue(recovered.accepted)

    def test_rearm_refused_when_heartbeat_stale(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        rt.heartbeat(0.0)
        rt.accept_command(_cmd(1, 100.0))  # latches
        with self.assertRaises(RuntimeError):
            rt.rearm(100.0)  # no fresh heartbeat at t=100

    def test_nan_heartbeat_is_rejected_and_never_authorizes(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        with self.assertRaises(ValueError):
            rt.heartbeat(float("nan"))
        # The corrupt beat was never stored, so the command is refused.
        result = rt.accept_command(_cmd(1, 0.0))
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "watchdog_timeout")

    def test_inf_heartbeat_is_rejected(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        with self.assertRaises(ValueError):
            rt.heartbeat(float("inf"))

    def test_non_monotonic_time_is_rejected(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        rt.heartbeat(10.0)
        with self.assertRaises(ValueError):
            rt.accept_command(_cmd(1, 9.0))

    def test_non_finite_command_time_is_rejected(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        rt.heartbeat(0.0)
        with self.assertRaises(ValueError):
            rt.accept_command(_cmd(1, float("nan")))


class TestReplayFencing(unittest.TestCase):
    """F002 — mandatory sequences and durable replay fencing."""

    def _fresh(self, rt: UgvProviderRuntime, seq: int, t: float) -> AdmissionResult:
        rt.heartbeat(t)
        return rt.accept_command(_cmd(seq, t))

    def test_missing_sequence_is_rejected(self) -> None:
        rt = UgvProviderRuntime()
        rt.heartbeat(0.0)
        with self.assertRaises(ValueError):
            rt.accept_command({"time": 0.0})

    def test_none_and_bool_sequence_are_rejected(self) -> None:
        rt = UgvProviderRuntime()
        rt.heartbeat(0.0)
        with self.assertRaises(ValueError):
            rt.accept_command({"sequence": None, "time": 0.0})
        with self.assertRaises(ValueError):
            rt.accept_command({"sequence": True, "time": 0.0})

    def test_replay_after_eviction_is_rejected(self) -> None:
        # dedup_history=1 so the first accepted seq is evicted from the FIFO, but
        # the unbounded high-water mark still fences the replay.
        rt = UgvProviderRuntime(dedup_history=1)
        self.assertTrue(self._fresh(rt, 5, 0.0).accepted)
        self.assertTrue(self._fresh(rt, 6, 0.1).accepted)
        replay = self._fresh(rt, 5, 0.2)
        self.assertFalse(replay.accepted)
        self.assertEqual(replay.reason, "out_of_order")

    def test_immediate_duplicate_is_rejected(self) -> None:
        rt = UgvProviderRuntime(dedup_history=4)
        self.assertTrue(self._fresh(rt, 7, 0.0).accepted)
        dup = self._fresh(rt, 7, 0.1)
        self.assertFalse(dup.accepted)
        self.assertEqual(dup.reason, "duplicate")

    def test_out_of_order_below_highwater_is_rejected(self) -> None:
        rt = UgvProviderRuntime(dedup_history=8)
        self.assertTrue(self._fresh(rt, 10, 0.0).accepted)
        stale = self._fresh(rt, 3, 0.1)
        self.assertFalse(stale.accepted)
        self.assertEqual(stale.reason, "out_of_order")

    def test_monotonic_sequences_are_accepted(self) -> None:
        rt = UgvProviderRuntime()
        for i, seq in enumerate((1, 2, 3, 10, 11)):
            self.assertTrue(self._fresh(rt, seq, float(i)).accepted)

    def test_transient_rejection_is_retriable(self) -> None:
        # A bench-not-ready rejection must not record the sequence, so the same
        # command succeeds once the bench is ready.
        rt = UgvProviderRuntime(bench_ready=False)
        rt.heartbeat(0.0)
        blocked = rt.accept_command(_cmd(5, 0.0))
        self.assertFalse(blocked.accepted)
        self.assertEqual(blocked.reason, "bench_not_ready")
        rt.set_bench_ready(True)
        rt.heartbeat(0.1)
        retried = rt.accept_command(_cmd(5, 0.1))
        self.assertTrue(retried.accepted)


class TestBenchReadinessAndRestart(unittest.TestCase):
    """UGV bench-readiness gating and restart resilience."""

    def test_bench_not_ready_blocks_command(self) -> None:
        rt = UgvProviderRuntime(bench_ready=False)
        rt.heartbeat(0.0)  # fresh heartbeat so the watchdog gate passes
        result = rt.accept_command(_cmd(1, 0.0))
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "bench_not_ready")

    def test_bench_ready_allows_command(self) -> None:
        rt = UgvProviderRuntime(bench_ready=False)
        rt.set_bench_ready(True)
        rt.heartbeat(0.0)
        self.assertTrue(rt.accept_command(_cmd(1, 0.0)).accepted)

    def test_restart_clears_transient_state_and_latch(self) -> None:
        rt = UgvProviderRuntime(watchdog_timeout=1.0)
        rt.heartbeat(0.0)
        rt.accept_command(_cmd(9, 0.5))
        rt.accept_command(_cmd(10, 100.0))  # latch SAFE_STOP
        self.assertTrue(rt.safe_stop_latched)
        rt.restart()
        self.assertFalse(rt.safe_stop_latched)
        self.assertEqual(rt.mode, ACTIVE)
        # After restart a low sequence is admissible again on a fresh timeline.
        rt.heartbeat(0.0)
        self.assertTrue(rt.accept_command(_cmd(1, 0.0)).accepted)

    def test_construction_validates_parameters(self) -> None:
        with self.assertRaises(ValueError):
            UgvProviderRuntime(watchdog_timeout=0.0)
        with self.assertRaises(ValueError):
            UgvProviderRuntime(watchdog_timeout=float("inf"))
        with self.assertRaises(ValueError):
            UgvProviderRuntime(dedup_history=0)


if __name__ == "__main__":
    unittest.main()
