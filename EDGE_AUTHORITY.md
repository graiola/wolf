# Edge Authority Plane Integration (ROS 2 WoLF Provider)

## Architecture
- Single out-of-process instance hosting behind Edge: Core -> Edge -> WoLF ROS 2.
- Direct-to-Edge teleoperation session: UI -> Edge -> WoLF ROS 2.
- Epoch fencing and MapContext alignment enforcement.

## MapContext Alignment Rules
- `ALIGNED`: Full speed vector and motion capability authorized.
- `DEGRADED`: Resultant physical speed vector capped at 0.5 m/s.
- `UNALIGNED`: Fallback to local-frame operation; shared-map coordinates rejected.

## Goal Tolerance & Motion Limits
- Defensive XY goal tolerance limit: 0.05 m.
- Defensive Yaw goal tolerance limit: 0.05 rad.
