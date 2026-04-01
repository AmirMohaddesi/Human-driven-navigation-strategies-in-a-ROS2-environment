# Mission Control — operator / CLI JSON policy

## 1. Purpose

Operators and integrators were hitting **ambiguous shell exit behavior** for `mission-agent`: contract-correct responses (e.g. **wrong_robot**, **not_found**) still produce **nonzero** process exit codes today, while **JSON** carries the real semantics. That mismatch causes **CI and wrapper scripts** to mis-classify outcomes if they only check `$?`.

This note is the **source of truth** for:

- How to interpret **`mission-agent`** / **`python -m multi_robot_mission_stack.agent.cli`** output.
- How **exit codes** relate to **JSON** for **automation defaults**.
- What counts as a **contract-shaped** response vs **transport / internal** failure.

**No `cli.py` behavior change** should land until this policy is **frozen and approved**; then implementation should match this document.

**Governs:** the CLI entrypoint only (see Scope). It does **not** redefine bridge service semantics; bridge JSON remains as validated in [mission_system_runbook.md](mission_system_runbook.md) **§K** and V2.0.1 ownership notes.

---

## 2. Scope

| In scope | Out of scope (this note) |
|----------|---------------------------|
| `mission-agent` and `python3 -m multi_robot_mission_stack.agent.cli` | Bridge node logic, `MissionClient`, `.srv` schemas |
| **`--mock`** and **`--ros`** modes for the commands below | LangGraph, orchestration, launch/runtime |
| **Commands:** `navigate`, `navigate-pose`, `query-state`, `cancel` | Broad rewrites of all repo scripts |
| **Navigation control plane** slice aligned with §K: navigate, query-state, cancel | Terminal-goal lifecycle semantics as a product spec |

---

## 3. Primary vs secondary contract

1. **JSON is the primary machine contract.** For a successful CLI invocation that prints a result object, **parsers must use** `status`, `nav_status`, `message`, and other fields as defined by the facade/bridge path. Distinguishing **wrong_robot** vs **not_found** **requires** parsing JSON; **exit code alone is insufficient**.

2. **Shell exit code is a secondary signal** for automation: “should a default shell pipeline treat this invocation as the happy path (0) or not (nonzero)?” It must **not** redefine semantic meaning already expressed in JSON.

3. **stderr / non-JSON / crashes** are **out of band** relative to contract-shaped JSON. If the process prints **no** parseable result JSON (or prints usage/errors on stderr and exits), treat as **failure** for wrappers regardless of any partial stdout.

4. **Contract-shaped JSON** means: a **single JSON object** on stdout (typically one line) with the **usual result fields** produced by `handle_command` for that subcommand (`status`, `message`, `nav_status`, `goal_id` when applicable), not an argparse usage string.

---

## 4. Policy table

| Situation | JSON (primary) | Exit code (secondary) |
|-----------|----------------|------------------------|
| **Success-shaped JSON** — `status` in `accepted`, `success`, or `in_progress` (as produced today for successful navigate / accepted goals / in-progress paths) | Authoritative: operation succeeded or is in a **documented** non-failure state for that command | **0** — default automation may treat as CLI success |
| **Contract-correct failure JSON** — e.g. bridge-backed `status: failure` with `nav_status` **`not_found`**, **`wrong_robot`**, or other **documented** control-plane outcomes (see §K / V2.0.1 validators) | Authoritative: **which** failure (read `nav_status` + `message`) | **Nonzero** — default automation treats as “not primary success path”; **JSON still defines the exact reason** |
| **Runtime / transport / internal failure** — service timeout, `status: failed` from client layer, thrown exception in CLI/facade, “client closed”, etc. | May be `failed` or absent; message explains | **Nonzero** |
| **Malformed or missing JSON** when a JSON result was expected — empty stdout, non-JSON, truncated output | N/A | **Nonzero** |
| **Usage error / invalid CLI invocation** — argparse errors, missing required args, unknown flags | Typically none (stderr usage text) | **Nonzero** |

---

## 5. Recommended rule (this project, now)

**Contract-correct failure JSON uses nonzero exit**, with **JSON remaining fully authoritative** for semantics.

- **Rationale:** Keeps **shell defaults** honest (`set -e`, simple CI) while requiring **any wrapper that cares about failure kind** to **parse JSON**—already mandatory for **wrong_robot** vs **not_found**.
- **Corollary:** **Do not** interpret nonzero exit as “parse failed” or “unknown error” if stdout contains valid contract JSON; **read the object** first.

This matches manager direction: **nonzero for failure-shaped status is acceptable** when **clearly documented**; **JSON** is not demoted.

---

## 6. Operator / integrator guidance

- **Shell users** can trust **exit 0** as “CLI completed and reported a **success-shaped** JSON outcome” for the supported commands. For **nonzero**, they must **read JSON** (if present) before assuming timeout or crash.
- **Wrappers and CI** that branch on **wrong_robot** vs **not_found** vs **cancelling** **must parse JSON**; **never** rely on exit code alone for those distinctions.
- **wrong_robot** (V2.0.1): `status: failure`, `nav_status: wrong_robot`, stable `message` — means **goal id is owned by another robot** in the bridge registry sense.
- **not_found** (invalid / unknown goal for that robot): `status: failure`, `nav_status: not_found`, stable `message` — means **no registry ownership** for that `(robot_id, goal_id)` pair, not “wrong robot.”
- **Validators** that record `cli_exit_code` alongside parsed JSON are the **reference pattern** for documenting observed exit behavior during policy rollout.

---

## 7. Non-goals

- No bridge or ROS service behavior changes driven by this policy.
- No `MissionClient` or schema changes.
- No orchestration or multi-robot supervisor semantics.
- No terminal-goal or query-semantics redesign.
- No mass update of every script in the repository in the same step as policy publication (phase separately).

---

## 8. Follow-up implementation note

After policy approval:

1. **Smallest code change:** adjust **`_exit_code_from_result`** (or the single exit choke point) in **`multi_robot_mission_stack/agent/cli.py`** so behavior **matches §4–§5** explicitly (including consistent treatment of `failure` vs `failed` if the policy table requires it).
2. **Validators:** update **CLI validators only** if the policy changes **expected `cli_exit_code`** for a documented scenario; **MissionClient** and **facade** validators stay tied to bridge JSON, not CLI exit.
3. **Docs:** keep this file and [mission_system_runbook.md](mission_system_runbook.md) cross-linked; avoid large runbook rewrites.

---

## Revision

- **Frozen** under manager approval for operator/CLI policy alignment milestone; amend only via explicit review when behavior or commands change.
