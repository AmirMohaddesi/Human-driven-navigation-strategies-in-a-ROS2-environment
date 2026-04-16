# Mission Control V11.1 — `shared_space_caution` contract freeze (checkpoint)

## Strategic question

Can we freeze a third `fact_type` literal whose meaning is neither refusal-capable blockage nor
passage-quality degradation, and document it with the same strict contract discipline—including
explicit reject examples—without any runtime integration?

## Scope

V11.1 is docs and fixture-aligned tests only:

- define and freeze `shared_space_caution` logical contract
- provide canonical accept/reject examples
- add a lightweight docsync test (no production `semantic/` validators)

No ROS launch, bridge, coordinator, MissionTools, transport, or mission behavior changes are part
of this milestone.

## Deliverables

- contract document:
  - `mission_control_v11_1_shared_space_caution_contract.md`
- fixture examples:
  - `tests/fixtures/semantic_v11_1_shared_space_caution_contract_examples.json`
- doc-sync test:
  - `tests/test_semantic_contract_v11_1_shared_space_caution_docsync.py`

## Test command

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_semantic_contract_v11_1_shared_space_caution_docsync.py -q --tb=short
```

## Explicitly unchanged

- bridge, services, coordinator, and mission execution paths
- `blocked_passage` and `degraded_passage` schemas, validators, and runtime wiring
- transport topics, belief stores, and policy hooks for `shared_space_caution`

## Related

- [mission_control_v11_7_local_proof_bundle_closure.md](mission_control_v11_7_local_proof_bundle_closure.md) — V11 local-only `shared_space_caution` proof bundle (closure)
- [mission_control_v11_2_checkpoint.md](mission_control_v11_2_checkpoint.md) (local validator)
- [mission_control_v8_1_degraded_passage_contract.md](mission_control_v8_1_degraded_passage_contract.md)
- [mission_control_v3_0_1_semantic_fact_exchange.md](mission_control_v3_0_1_semantic_fact_exchange.md)
- [mission_control_v11_1_shared_space_caution_contract.md](mission_control_v11_1_shared_space_caution_contract.md)
