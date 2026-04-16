# Mission Control V8.9 — bounded ROS transport runtime proof for degraded_passage (checkpoint)

## Strategic question

Can `degraded_passage` participate in one bounded ROS transport seam — publish / receive / ingest /
local advisory observation — without bridge/coordinator, execution meaning, or mission-layer
behavior?

## Scope

V8.9 adds fact-specific ROS nodes only:

- `DegradedPassageTransportReceiverNode` — `std_msgs/String` JSON → `ingest_degraded_passage_transport_payload`
- `DegradedPassageTransportSenderNode` — optional one-shot CLI publisher (mirrors blocked sender)
- in-process ROS test: publisher + receiver + shared `DegradedPassageBeliefStore`

No launch file required for the proof; no bridge/coordinator/MissionTools wiring.

## Deliverables

- `src/multi_robot_mission_stack/multi_robot_mission_stack/transport/degraded_passage_receiver_node.py`
- `src/multi_robot_mission_stack/multi_robot_mission_stack/transport/degraded_passage_sender_node.py`
- `src/multi_robot_mission_stack/setup.py` — `degraded_passage_transport_receiver` / `_sender` entry points
- `tests/test_degraded_passage_transport_ros_v89.py`
- this checkpoint document

## Local / regression (no ROS process graph)

```bash
cd /path/to/HDNS
python3 -m pytest tests/test_degraded_passage_transport_v87.py -q --tb=short
```

## ROS runtime proof (in-process test)

Requires ROS 2 Python bindings (`rclpy`) on the machine:

```bash
cd /path/to/HDNS
source install/setup.bash   # if using colcon workspace
python3 -m pytest tests/test_degraded_passage_transport_ros_v89.py -q --tb=short
```

Optional manual two-process smoke (after install + source):

```bash
# Terminal A — receiver (default topic)
ros2 run multi_robot_mission_stack degraded_passage_transport_receiver --ros-args \
  -p allowed_source_robot_ids:="[robot1]"

# Terminal B — one-shot sender (set payload_json to compact JSON from V8.7 encode)
ros2 run multi_robot_mission_stack degraded_passage_transport_sender --ros-args \
  -p payload_json:='<json>' \
  -p publish_once_after_sec:=0.5
```

## Explicitly unchanged

- no bridge, coordinator, or mission policy integration
- no navigation execution semantics for `degraded_passage`

## Related

- [mission_control_v8_8_checkpoint.md](mission_control_v8_8_checkpoint.md)
- [mission_control_v8_7_checkpoint.md](mission_control_v8_7_checkpoint.md)
