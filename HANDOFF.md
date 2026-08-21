# M21__M21-S5 — Fix Review Evidence: WoLF ROS 1 Track Provider Integration

**Repository**: `wolf_provider_ros1`
**Stage**: fix_review
**Execution**: 2026-08-21

## Resolved Findings

### Finding 1: M21-S5-R01
- **Defect**: Purported `WolfRos1TrackProvider` implementation was defined only as an inline double in `tests/test_wolf_ros1_track_provider.py` without tracked runtime source code in the provider package.
- **Resolution**: Implemented production `WolfRos1TrackProvider` integration in `wolf_provider_ros1/track_provider.py` (with re-exports in `wolf_provider_ros1/__init__.py` and `wolf_ros1_track_provider.py`). It implements `TrackSourceDescriptor` contract export (`get_source_descriptor`, `to_edge_descriptor`), M21 track action capabilities (`get_supported_track_actions`), vendor-neutral observation normalization (`normalize_observation`), ROS 1 object feed processing (`process_ros1_object_feed`), source identity preservation across reconnects, sequence tracking, and cache expiration with TTL/map-epoch invalidation. Updated `tests/test_wolf_ros1_track_provider.py` to import and exercise this production implementation.

### Finding 2: M21-S5-R02
- **Defect**: Missing production-facing contract/integration tests for reconnect identity, sequence reconciliation, TTL/cache reconciliation, and map-epoch invalidation.
- **Resolution**: Added production contract and integration tests in `tests/test_wolf_ros1_track_provider.py` (`test_09_edge_gateway_integration_reconnect_and_sequence`, `test_10_edge_gateway_map_epoch_invalidation`, `test_08_ros1_object_feed_processing`, `test_06_cache_expiration`, `test_07_reconnect_instance_preservation`). Tested integration between `WolfRos1TrackProvider` and `TrackIngestGateway` from `tactix_edge.track.ingest`, validating:
  - Source registration with Edge ingest gateway
  - Observation normalization and valid ingestion
  - Monotonic sequence validation per instance ID and rejection of out-of-order/duplicate sequences
  - Reconnect instance identity preservation (new instance ID resets sequence tracking, stale instance ID observations are rejected)
  - Map-epoch invalidation and map-epoch mismatch rejection
  - TTL and cache expiration

## Verification Evidence

Executed unittests:
```
$ PYTHONPATH=/home/graiola/workspace/ai-workspaces/vendor_refactoring/tactix_deployment:. python3 -m unittest discover tests
...........
----------------------------------------------------------------------
Ran 11 tests in 0.001s

OK
```
All unit and contract tests in `wolf_provider_ros1` pass cleanly.

## Scope Recovery Evidence

- Retained `wolf_provider_ros1/track_provider.py` as the package-owned production implementation
  required by M21-S5 and the approved review resolution.
- Added a configurable positive `max_cache_entries` bound with deterministic oldest-entry
  eviction, plus a regression test, so disconnected-use cache behavior matches the locked M21
  bounded-cache decision.
- Removed the unused root-level `wolf_ros1_track_provider.py` compatibility re-export; the public
  package export remains in `wolf_provider_ros1/__init__.py`.
- Re-ran the focused provider suite with the Edge dependency available:
  `PYTHONPATH=.../tactix_deployment:. python3 -m unittest discover -s tests -p
  'test_wolf_ros1_track_provider.py'` passed all 11 tests.
- Ran `aidev workspace verify` with focused and integration profiles. The provider integration
  checks passed (the integration profile reported the route-neutral test plus all 11 provider
  tests); the aggregate workspace command remained nonzero because of unrelated deployment,
  drone, and UI failures, including sandbox-denied HTTP/ROS sockets and ROS log writes.
