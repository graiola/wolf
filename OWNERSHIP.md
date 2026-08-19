# ROS 2 WoLF Provider Ownership

## Responsibilities
- Own ROS 2 WoLF provider runtime artifacts, manifests, and edge authority plane integration.
- Isolate ROS 2 WoLF controller, navigation, and hardware interfaces behind Edge.
- Provide single out-of-process provider hosting behind Edge authority plane.

## Boundaries
- Write scope strictly preserved within `providers/wolf_ros2`.
- Direct-to-Edge teleoperation session support.
- Defensive validation of hard motion limits and goal tolerances (0.05m XY, 0.05rad yaw).
