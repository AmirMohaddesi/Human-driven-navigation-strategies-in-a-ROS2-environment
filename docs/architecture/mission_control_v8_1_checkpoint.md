# Mission Control V8.1 — degraded_passage contract freeze (checkpoint)

## Strategic question

Can one second semantic fact type be frozen with bounded contract discipline, clearly distinct
from `blocked_passage`, without changing runtime authority or reopening V4-V7 behavior lines?

## Scope

V8.1 is docs/test contract work only:

- define and freeze `degraded_passage` logical contract
- provide canonical accept/reject examples
- add lightweight doc-sync tests

No ROS launch/runtime integration is part of this milestone.

## Deliverables

- contract document:
  - `mission_control_v8_1_degraded_passage_contract.md`
- fixture examples:
  - `tests/fixtures/semantic_v8_1_degraded_passage_contract_examples.json`
- doc-sync test:
  - `tests/test_semantic_contract_v8_1_degraded_passage_docsync.py`

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_semantic_contract_v8_1_degraded_passage_docsync.py -q --tb=short
```

## Explicitly unchanged

- no bridge/service contract changes
- no coordinator alternate or hard-stop behavior changes
- no mission policy expansion
- no transport/launch topology changes
- no `blocked_passage` schema/semantics changes

## Related

- [mission_control_v7_3_checkpoint.md](mission_control_v7_3_checkpoint.md)
- [mission_control_v3_0_1_semantic_fact_exchange.md](mission_control_v3_0_1_semantic_fact_exchange.md)
