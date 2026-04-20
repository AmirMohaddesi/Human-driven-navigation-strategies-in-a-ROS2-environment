from multi_robot_mission_stack.runtime.r3_local_hold_from_r1 import is_valid_r1_degraded_payload


def test_is_valid_r1_degraded_payload_accepts_valid_degraded_json() -> None:
    payload = '{"fact_type":"degraded_passage","belief_id":"deg-1","location_ref":"tf:/robot1_ns/tf<->/robot2_ns/tf"}'
    assert is_valid_r1_degraded_payload(payload) is True


def test_is_valid_r1_degraded_payload_rejects_wrong_fact_type() -> None:
    payload = '{"fact_type":"blocked_passage","belief_id":"blk-1","location_ref":"x"}'
    assert is_valid_r1_degraded_payload(payload) is False


def test_is_valid_r1_degraded_payload_rejects_malformed_or_missing_belief() -> None:
    assert is_valid_r1_degraded_payload("not-json") is False
    assert is_valid_r1_degraded_payload('{"fact_type":"degraded_passage","belief_id":""}') is False
