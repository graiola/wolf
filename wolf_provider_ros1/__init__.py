"""WoLF ROS 1 provider runtime contracts.

Besides the track and simulation adapters, this module owns the small,
dependency-free M25 recovery contract used at the ROS 1 process boundary.
Commands are checkpointed before dispatch so a restarted provider can answer
status queries and reject redelivery without repeating a physical effect.
Hardware runs are admitted only after the UGV bench safety preflight passes.
"""

from __future__ import annotations

import copy
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

from .track_provider import WolfRos1TrackProvider
from .simulation_acceptance import (
    WolfRos1SimulationAcceptanceAdapter,
    WolfRos1SpawnSpec,
)


class WolfRos1RecoveryLedger:
    """Keep command effects idempotent across provider process restarts.

    ``snapshot()`` is a serializable checkpoint for the surrounding runtime.
    Restoring it fences duplicate delivery. Ambiguous dispatch never retries:
    it requests a local safe-stop and reports ``RECOVERY_REQUIRED``.
    """

    SCHEMA_VERSION = 1
    TERMINAL_STATES = frozenset(
        {"SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT", "RECOVERY_REQUIRED"}
    )

    def __init__(
        self,
        dispatch: Callable[[Mapping[str, Any]], None],
        safe_stop: Callable[[str], None],
        checkpoint: Optional[Mapping[str, Any]] = None,
        checkpoint_writer: Optional[Callable[[Mapping[str, Any]], None]] = None,
    ) -> None:
        self._dispatch = dispatch
        self._safe_stop = safe_stop
        self._checkpoint_writer = checkpoint_writer
        self._commands: Dict[str, Dict[str, Any]] = {}
        self._stopped_incidents: set[str] = set()
        if checkpoint is not None:
            self.restore(checkpoint)

    @staticmethod
    def _command_id(command_id: Any) -> str:
        value = str(command_id) if command_id is not None else ""
        if not value:
            raise ValueError("command_id is required")
        return value

    def submit(self, command_id: str, command: Mapping[str, Any]) -> Dict[str, Any]:
        """Checkpoint acceptance, dispatch once, and identify redelivery."""
        command_id = self._command_id(command_id)
        existing = self._commands.get(command_id)
        if existing is not None:
            if existing["command"] != dict(command):
                raise ValueError("duplicate command_id has different payload")
            result = copy.deepcopy(existing)
            result["duplicate"] = True
            return result

        record = {
            "command_id": command_id,
            "command": dict(command),
            "state": "ACCEPTED",
            "effect_dispatched": False,
            "effect_count": 0,
            "detail": None,
        }
        self._commands[command_id] = record
        self._persist()
        try:
            self._dispatch(copy.deepcopy(record["command"]))
        except Exception as error:
            record["state"] = "RECOVERY_REQUIRED"
            record["detail"] = f"EFFECT_UNKNOWN: {type(error).__name__}"
            self.safe_stop(f"dispatch:{command_id}")
        else:
            record["effect_dispatched"] = True
            record["effect_count"] = 1
            record["state"] = "EXECUTING"
            self._persist()
        result = copy.deepcopy(record)
        result["duplicate"] = False
        return result

    def update_status(
        self, command_id: str, state: str, detail: Optional[str] = None
    ) -> Dict[str, Any]:
        command_id = self._command_id(command_id)
        if command_id not in self._commands:
            raise KeyError(command_id)
        state = str(state)
        if state not in self.TERMINAL_STATES | {"EXECUTING"}:
            raise ValueError(f"unsupported command state: {state}")
        record = self._commands[command_id]
        if record["state"] in self.TERMINAL_STATES and record["state"] != state:
            raise ValueError("terminal command status cannot be changed")
        record["state"] = state
        record["detail"] = detail
        self._persist()
        return copy.deepcopy(record)

    def query_status(self, command_id: str) -> Optional[Dict[str, Any]]:
        record = self._commands.get(self._command_id(command_id))
        return copy.deepcopy(record) if record is not None else None

    def provider_unavailable(self, command_id: str) -> Dict[str, Any]:
        """Fence an in-flight effect and require explicit reconciliation."""
        command_id = self._command_id(command_id)
        if command_id not in self._commands:
            raise KeyError(command_id)
        record = self._commands[command_id]
        if record["state"] not in self.TERMINAL_STATES:
            record["state"] = "RECOVERY_REQUIRED"
            record["detail"] = "EFFECT_UNKNOWN: provider unavailable"
            self.safe_stop(f"provider-unavailable:{command_id}")
            self._persist()
        return copy.deepcopy(record)

    def safe_stop(self, incident_id: str) -> bool:
        """Invoke the physical stop at most once for an incident identifier."""
        incident_id = str(incident_id)
        if not incident_id:
            raise ValueError("incident_id is required")
        if incident_id in self._stopped_incidents:
            return False
        self._safe_stop(incident_id)
        self._stopped_incidents.add(incident_id)
        self._persist()
        return True

    def _persist(self) -> None:
        if self._checkpoint_writer is not None:
            self._checkpoint_writer(self.snapshot())

    def snapshot(self) -> Dict[str, Any]:
        return {
            "schema_version": self.SCHEMA_VERSION,
            "commands": copy.deepcopy(self._commands),
            "stopped_incidents": sorted(self._stopped_incidents),
        }

    def restore(self, checkpoint: Mapping[str, Any]) -> None:
        if checkpoint.get("schema_version") != self.SCHEMA_VERSION:
            raise ValueError("unsupported or corrupt recovery checkpoint")
        commands = checkpoint.get("commands")
        incidents = checkpoint.get("stopped_incidents")
        if (
            not isinstance(commands, Mapping)
            or not isinstance(incidents, Sequence)
            or isinstance(incidents, (str, bytes))
        ):
            raise ValueError("corrupt recovery checkpoint")
        restored: Dict[str, Dict[str, Any]] = {}
        for key, value in commands.items():
            if not isinstance(value, Mapping) or value.get("command_id") != key:
                raise ValueError("corrupt recovery checkpoint command")
            restored[str(key)] = dict(value)
        self._commands = copy.deepcopy(restored)
        self._stopped_incidents = {str(value) for value in incidents}


class UgvBenchReadinessGate:
    """Return honest, evidence-bearing UGV hardware readiness results."""

    REQUIRED_PREFLIGHT = (
        "reservation_owned",
        "fixture_isolated",
        "estop_ready",
        "safe_stop_verified",
        "controller_ready",
    )

    @classmethod
    def assess(
        cls, observation: Mapping[str, Any], *, requested: bool = True
    ) -> Dict[str, Any]:
        evidence = {name: bool(observation.get(name)) for name in cls.REQUIRED_PREFLIGHT}
        fixture_id = observation.get("fixture_id")
        if not requested:
            return {
                "status": "SKIPPED", "ready": False, "fixture_id": fixture_id,
                "reason": "ugv-bench job was not requested", "preflight": evidence,
            }
        if not observation.get("fixture_available"):
            return {
                "status": "UNAVAILABLE", "ready": False, "fixture_id": fixture_id,
                "reason": "ugv-bench fixture is absent", "preflight": evidence,
            }
        missing = [name for name, passed in evidence.items() if not passed]
        if missing:
            return {
                "status": "NOT_READY", "ready": False, "fixture_id": fixture_id,
                "reason": "mandatory preflight failed", "missing": missing,
                "preflight": evidence,
            }
        return {
            "status": "READY", "ready": True, "fixture_id": fixture_id,
            "reason": "mandatory UGV bench safety preflight passed",
            "preflight": evidence,
        }

__all__ = [
    "WolfRos1TrackProvider",
    "WolfRos1SimulationAcceptanceAdapter",
    "WolfRos1SpawnSpec",
    "WolfRos1RecoveryLedger",
    "UgvBenchReadinessGate",
]
