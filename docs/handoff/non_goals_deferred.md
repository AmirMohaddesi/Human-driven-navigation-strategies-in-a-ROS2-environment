# Non-goals and deferred work (this phase)

**Purpose:** Explicit boundary so scope creep does not erase the Layer B checkpoint.

## Out of scope for the current checkpoint

- **Simulator / Gazebo debugging** — treat runtime issues there as a separate workstream.
- **Execution stack refactors** — Layer A bridge, client, and agent internals stay as-is unless fixing a verified defect.
- **New ROS services or messages** for coordinator contracts — Layer B stays Python-callable and manifest-driven.
- **LangGraph / planner integration** — no planning graph above the stable API in this phase.
- **Replacing** `MissionClient`, `MissionAgentFacade`, or core wait utilities — coordinator consumes them; it does not fork them.
- **Coordinator redesign** — additive thin helpers only.
- **Retries, recovery, cancellation orchestration** — not part of the current Layer B contract.
- **New execution semantics** (e.g. new mission modes, new batch policies) — deferred unless explicitly specified as a small Layer B addition.

## Sensible next branches (pick one)

1. **Planner integration above Layer B** — bind LangGraph (or similar) to `plan_team_named_mission_api_call` / checked entrypoints and manifests.
2. **Distributed execution ownership below Layer B** — stronger multi-robot runtime guarantees, ownership of goals, or bridge scaling without changing the public Layer B shapes.

## What *is* in scope for follow-on (examples)

- More **validation scripts** or **pytest** around new entrypoints (same style as today).
- **Documentation** and **runbooks** that reference the manifest and checked seams.
- **Optional** thin wrappers that only compose existing functions (no new semantics).
