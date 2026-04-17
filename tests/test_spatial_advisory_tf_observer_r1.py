from multi_robot_mission_stack.runtime.spatial_advisory_tf_observer_r1 import (
    euclidean_distance_m,
    seam_location_ref,
    should_emit_advisory,
)


def test_distance_helper_uses_xyz_norm() -> None:
    got = euclidean_distance_m(3.0, 4.0, 12.0)
    assert got == 13.0


def test_emit_condition_true_at_or_below_threshold() -> None:
    assert should_emit_advisory(1.2, 1.5) is True
    assert should_emit_advisory(1.5, 1.5) is True


def test_emit_condition_false_above_or_nonfinite() -> None:
    assert should_emit_advisory(1.51, 1.5) is False
    assert should_emit_advisory(float("inf"), 1.5) is False
    assert should_emit_advisory(1.2, float("nan")) is False


def test_location_ref_marker_is_stable_and_proof_friendly() -> None:
    got = seam_location_ref("/robot1_ns/tf", "/robot2_ns/tf", 2.5)
    assert got == "tf:/robot1_ns/tf<->/robot2_ns/tf:dist<=2.50"
