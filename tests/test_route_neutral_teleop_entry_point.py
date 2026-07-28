"""M17: verify the WoLF ROS 1 controller exposes exactly one high-priority
teleop command entry point (the `priority_twist` device), so TACTIX's
central route and Direct-to-Edge route, which both dispatch through the
out-of-process gateway in `tactix_deployment/tactix_provider_wolf`, land on
the same route-blind vendor input rather than any caller-conditional path.

This is a structural/static check, not a runtime one: the vendor
`wolf_controller` package (a pinned git submodule of this repository) has
no dependency on TACTIX's route concept at all, and the message it consumes
(`geometry_msgs/Twist`) carries no route/session metadata. Route-neutrality
here reduces to there being exactly one HIGH-priority command device wired
into the vendor's `DevicesHandler` priority mux, matching the single
hardcoded endpoint (`priority_twist`) the TACTIX gateway
(`tactix_deployment/tactix_provider_wolf/gateway.py`) always targets. A
second HIGH-priority device, or the existing one moving off `priority_twist`,
would let some other path bypass the shared handling this test protects.

`wolf_controller` is fetched as a git submodule of this repository and is
not always materialized in every task workspace (see AGENTS.md on
runtime-only source synchronization); when it is absent this test skips
rather than fails, since that is a workspace sync gap, not a regression in
the vendor contract being verified.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_PLUGIN = REPO_ROOT / "wolf_controller" / "src" / "ros" / "controller_plugin.cpp"
EXPECTED_HANDLER_TYPE = "TwistHandler"
EXPECTED_TOPIC = "priority_twist"

HIGH_PRIORITY_DEVICE_LINE = re.compile(
    r"devices_\.addDevice\(\s*DevicesHandler::priority_t::HIGH\s*,"
    r"\s*std::make_shared<(\w+)>"
)


class RouteNeutralTeleopEntryPointTest(unittest.TestCase):
    def setUp(self) -> None:
        if not CONTROLLER_PLUGIN.is_file():
            self.skipTest(
                "wolf_controller submodule is not materialized in this workspace; "
                "run `aidev workspace sync` (or `git submodule update --init`) to "
                "check it out before this structural check can run."
            )

    def test_exactly_one_high_priority_command_device(self) -> None:
        matches = []
        for line in CONTROLLER_PLUGIN.read_text().splitlines():
            match = HIGH_PRIORITY_DEVICE_LINE.search(line)
            if match:
                matches.append((match.group(1), line.strip()))

        self.assertEqual(
            len(matches), 1,
            "Expected exactly one HIGH-priority command device wired into "
            "DevicesHandler; a second one would let a route bypass the single "
            "canonical teleop entry point TACTIX's gateway depends on. "
            f"Found: {matches!r}",
        )
        handler_type, line = matches[0]
        self.assertEqual(handler_type, EXPECTED_HANDLER_TYPE)
        self.assertIn(f'"{EXPECTED_TOPIC}"', line)


if __name__ == "__main__":
    unittest.main()
