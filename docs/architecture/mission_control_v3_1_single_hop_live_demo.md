# Mission Control V3.1 — Single-hop live semantic relay demo (design note)

**Scope:** Reuse **V3.0.1** `blocked_passage` unchanged. **Live** proof: robot A publishes one canonical JSON fact on ROS; robot B ingests into a **shared** belief store and the **existing** named-location policy returns the **frozen** blocked outcome. No transport redesign, no new fact types, no bridge contract change.

**Milestone status:** V3.1 is **checkpointed complete** — see [mission_control_v3_1_checkpoint.md](mission_control_v3_1_checkpoint.md) (artifacts, tests, recorded live success, out of scope, next steps).

---

## Related documents

- [mission_control_v3_1_checkpoint.md](mission_control_v3_1_checkpoint.md) — **closure:** what was proven, live record, handoff.
- [mission_control_v3_0_1_checkpoint.md](mission_control_v3_0_1_checkpoint.md) — semantic / policy / transport base.
