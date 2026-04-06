# Mission Control V3.1 — Milestone checkpoint (closed)

## Purpose

Close **V3.1** as **complete**: a **live-demo** milestone (not a new semantic contract) that reuses frozen **V3.0.1** `blocked_passage` semantics and proves **one bounded single-hop relay** plus **strict blocked navigation** in a real ROS graph.

Design note (scope only): [mission_control_v3_1_single_hop_live_demo.md](mission_control_v3_1_single_hop_live_demo.md).

---

## What V3.1 proved

- **Robot B dedicated runtime:** `RobotBBlockedPassageDemoRuntime` — one `BlockedPassageBeliefStore`, transport subscriber ingest, and `MissionTools` / `MissionAgentFacade` on the **same** store with explicit `now_utc` on `handle_command`.
- **Robot A one-shot sender:** `robot-a-blocked-passage-demo` / `build_robot_a_blocked_passage_wire` + canonical JSON on `std_msgs/String` (topic default `/semantic/blocked_passage_v301`).
- **Bounded live validator:** `scripts/validate_v3_1_single_hop_live_demo.py` — three-way classification (`success` / `environment_not_ready` / `demo_failed`), in-process B composition (facade + subscriber share store; subprocess B + raw bridge call would not hit the policy hook).
- **Live success path recorded:** with `mission_bridge_node` up, one publish → live ingest log → active belief for `location_ref` → `navigate_to_named_location` via facade returned the **frozen** blocked outcome; validator **exit 0**, `CLASSIFICATION: success`.

---

## Accepted artifacts and tests

**Code / entrypoints**

- `multi_robot_mission_stack/demo/robot_b_blocked_passage_runtime_v31.py` — `RobotBBlockedPassageDemoRuntime`, `robot-b-blocked-passage-demo`.
- `multi_robot_mission_stack/demo/robot_a_blocked_passage_sender_v31.py` — wire builders + `robot-a-blocked-passage-demo`.
- `scripts/validate_v3_1_single_hop_live_demo.py` — bounded live validator.
- `setup.py` console scripts for robot A/B demos.

**Deterministic tests**

- `tests/test_robot_b_blocked_passage_runtime_v31.py`
- `tests/test_robot_a_blocked_passage_sender_v31.py`

**V3.0.1 substrate** (unchanged contract): semantic, transport JSON, policy hook, shared runtime — as in [mission_control_v3_0_1_checkpoint.md](mission_control_v3_0_1_checkpoint.md).

---

## Live success record

Recorded run (representative):

- **Precondition:** `/navigate_to_named_location` available (`mission_bridge_node` running; workspace `install/local_setup.bash` sourced).
- **Sequence:** Validator composed B in-process → published one valid `blocked_passage` JSON string → receiver logged **ingested blocked_passage (belief stored)** → store showed **active** belief for `base` → `facade.handle_command` (named navigate, explicit `now_utc`) → **`outcome=navigation_target_blocked`**, **`reason_code=blocked_passage_peer_belief`**, strict checks passed → **`CLASSIFICATION: success`**, **exit 0**.
- **Teardown:** `cleanup_baseline_runtime.sh` **post** + **verify** clean.

---

## What remains explicitly out of scope (not required for V3.1 closure)

- Transport retries, acks, replay, multi-hop, new fact types, schema changes.
- LLM / NL on the wire; fallback expansion beyond frozen blocked outcome.
- Bridge / control-plane semantic redesign; map / TF / costmap mutation.
- Full **fully_integrated_swarm** as a gate for this demo (Nav2 not required on the **blocked** success path).
- Productizing default launch wiring for all fleet configurations.

---

## Why V3.1 stops here

Manager direction: **one successful live run** is recorded; the milestone is **live-demo complete** without broadening transport or semantics. Further work is a **new** milestone.

---

## Next milestone decision

Choose separately (examples only): deployment hardening, second fact type / schema revision, operational runbooks for varied stacks, policy extensions (e.g. alternate targets) — each with its own scope guard.

**Handoff:** Do not silently expand V3.1; treat follow-ons as **V3.2+** or unrelated tracks.

---

## Related documents

- [mission_control_v3_1_single_hop_live_demo.md](mission_control_v3_1_single_hop_live_demo.md) — V3.1 demo scope (short).
- [mission_control_v3_0_1_checkpoint.md](mission_control_v3_0_1_checkpoint.md) — frozen semantic + transport base.
