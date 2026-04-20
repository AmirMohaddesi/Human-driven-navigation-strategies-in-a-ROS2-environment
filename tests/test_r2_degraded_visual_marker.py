from multi_robot_mission_stack.runtime.r2_degraded_visual_marker import parse_r1_degraded_event


def test_parse_r1_degraded_event_accepts_valid_degraded_payload() -> None:
    payload = (
        '{"fact_type":"degraded_passage","belief_id":"deg-123","location_ref":"tf:/robot1_ns/tf<->/robot2_ns/tf",'
        '"ttl_sec":3.5}'
    )
    got = parse_r1_degraded_event(payload, default_ttl_sec=5.0)
    assert got is not None
    assert got.fact_type == "degraded_passage"
    assert got.belief_id == "deg-123"
    assert got.ttl_sec == 3.5


def test_parse_r1_degraded_event_rejects_non_degraded_fact() -> None:
    payload = '{"fact_type":"blocked_passage","belief_id":"blk-1","location_ref":"x","ttl_sec":3.0}'
    assert parse_r1_degraded_event(payload) is None


def test_parse_r1_degraded_event_clamps_bad_ttl_to_default_and_max() -> None:
    payload_default = '{"fact_type":"degraded_passage","belief_id":"deg-2","location_ref":"x","ttl_sec":"oops"}'
    got_default = parse_r1_degraded_event(payload_default, default_ttl_sec=4.0)
    assert got_default is not None
    assert got_default.ttl_sec == 4.0

    payload_max = '{"fact_type":"degraded_passage","belief_id":"deg-3","location_ref":"x","ttl_sec":100.0}'
    got_max = parse_r1_degraded_event(payload_max, default_ttl_sec=4.0)
    assert got_max is not None
    assert got_max.ttl_sec == 10.0
