# rmf_simulation_lift_controller

Simulated Lift Controller for Open-RMF. It receives `LiftSelectRequest` messages from the Lift Manager. It then picks a simulated lift and calls that lift to the boarding floor by publishing a standard `LiftRequest`, which the Gazebo lift plugin (`rmf_building_sim_gz_plugins/src/lift.cpp`) executes.

```text
Lift Manager --LiftSelectRequest--> simulated_lift_controller --LiftRequest--> Gazebo lift.cpp
                                              ^                                    |
                                              +------------- LiftState ------------+
```

This adds a second way to request a lift. Conventional `LiftRequest` messages, for example from the `lift_supervisor`, still reach `lift.cpp` as before.

## Interfaces

| Direction | Topic                   | Message                               | QoS              |
|-----------|-------------------------|---------------------------------------|------------------|
| Subscribe | `/lift_select_requests` | `rmf_lift_msgs/msg/LiftSelectRequest` | reliable, depth 100 |
| Subscribe | `/lift_states`          | `rmf_lift_msgs/msg/LiftState`         | reliable, depth 100 |
| Publish   | `/lift_requests`        | `rmf_lift_msgs/msg/LiftRequest`       | reliable, depth 100 |

These match the topic names and QoS used by `lift.cpp`.

### Parameters

| Name                   | Default | Description |
|------------------------|---------|-------------|
| `confirmation_timeout` | `5.0`   | How many seconds to wait for a `LiftState` that shows the assigned session. If none arrives in time, the internal reservation is released. `lift.cpp` drops requests it rejects (unknown floor, lift busy) without telling the sender, so without this timeout a rejected request would block that lift forever. |

## Behaviour

**Conversion.** An assigned request is published as a `LiftRequest` with these fields:

| LiftRequest field   | Value                                   |
|---------------------|-----------------------------------------|
| `lift_name`         | selected lift                           |
| `session_id`        | `LiftSelectRequest.session_id`          |
| `destination_floor` | `LiftSelectRequest.origin_floor`        |
| `request_type`      | `REQUEST_AGV_MODE` / `REQUEST_HUMAN_MODE` |
| `door_state`        | `DOOR_OPEN`                             |

The controller does not use `next_floor`. The rest of the journey is handled by the existing RMF lift workflow.

**Availability.** A lift is available only when all three of these hold:
- A `LiftState` has been received for it. A lift whose state is unknown is never treated as free.
- Its `LiftState.session_id` is empty.
- No assignment from this controller is still waiting for confirmation.

**Selection** uses availability-aware round-robin over `chosen_lifts`. Each distinct ordered candidate list keeps its own index. The search starts at the lift after the last one selected and skips busy or unknown lifts. A single-lift list is handled the same way.

**Pending queue.** If no candidate is available, the request is queued. The queue is processed only when a `LiftState` arrives, with no polling. Requests are served in FIFO order among those that can currently be served, so a request waiting on lift A does not hold up a later request for lift B. `priority` is ignored, because the Lift Manager handles priority.

**Assignment confirmation.** After publishing, the controller reserves the lift for that session. A `LiftState` with an empty `session_id` does not clear the reservation, because `lift.cpp` may not have processed the request yet. The reservation is cleared when a `LiftState` shows the session, or when `confirmation_timeout` passes.

**Duplicate handling.**
- If the session already holds a lift or has a reservation, the request is ignored.
- If the session is already queued, the request is ignored.
- If a queued session becomes active through another path, it is removed from the queue.

**Ignored requests** (logged as warnings):
- `REQUEST_END_SESSION`, because end-session handling is not implemented
- an unknown `request_type`
- an empty `chosen_lifts`, `session_id` or `origin_floor`

**Not implemented:** lift motion and door control, door-open timeouts, priority scheduling, rescue, end-session handling, and a second request to `next_floor`.

## Code layout

| File | Purpose |
|------|---------|
| `include/.../LiftAllocator.hpp`, `src/LiftAllocator.cpp` | Selection, queue, reservation and duplicate logic, with no ROS dependency; plus `make_lift_request()` |
| `include/.../SimulatedLiftController.hpp`, `src/SimulatedLiftController.cpp` | ROS 2 node: subscriptions, publisher, logging |
| `src/main.cpp` | Entry point. The single-threaded executor runs callbacks one at a time, so no locks are needed |
| `test/test_lift_allocator.cpp` | gtest unit tests |

## Build and test

`LiftSelectRequest` must be present in the built `rmf_lift_msgs`, so rebuild that package if it is older:

```bash
cd ~/workspace/open_rmf_multi_lift
source /opt/ros/jazzy/setup.bash
colcon build --mixin release lld --packages-select rmf_lift_msgs rmf_simulation_lift_controller
colcon test --packages-select rmf_simulation_lift_controller
colcon test-result --verbose --test-result-base build/rmf_simulation_lift_controller
```

## Running

Start it alongside a Gazebo simulation that uses the `lift` plugin:

```bash
source install/setup.bash
ros2 run rmf_simulation_lift_controller simulated_lift_controller
# with sim time / custom timeout:
ros2 run rmf_simulation_lift_controller simulated_lift_controller \
  --ros-args -p use_sim_time:=true -p confirmation_timeout:=5.0
```

## Manual test without Gazebo

Use four terminals, each with `source install/setup.bash`. You can set the same `ROS_DOMAIN_ID` in each to keep the test away from other ROS traffic.

```bash
# T1: controller
ros2 run rmf_simulation_lift_controller simulated_lift_controller

# T2: watch the output
ros2 topic echo /lift_requests

# T3: fake lift states. A is busy, B is free
ros2 topic pub -1 /lift_states rmf_lift_msgs/msg/LiftState "{lift_name: A, session_id: other}"
ros2 topic pub -1 /lift_states rmf_lift_msgs/msg/LiftState "{lift_name: B, session_id: ''}"

# T4: requests
# -> selects B (A skipped), publishes LiftRequest {B, robot1, L1, HUMAN, DOOR_OPEN}
ros2 topic pub -1 /lift_select_requests rmf_lift_msgs/msg/LiftSelectRequest \
  "{session_id: robot1, request_type: 2, origin_floor: L1, next_floor: L3, chosen_lifts: [A, B]}"
# -> queued (A busy, B reserved for robot1)
ros2 topic pub -1 /lift_select_requests rmf_lift_msgs/msg/LiftSelectRequest \
  "{session_id: robot2, request_type: 1, origin_floor: L2, chosen_lifts: [A, B]}"
# T3: free A -> robot2 assigned to A, publishes {A, robot2, L2, AGV, DOOR_OPEN}
ros2 topic pub -1 /lift_states rmf_lift_msgs/msg/LiftState "{lift_name: A, session_id: ''}"
# -> warning, ignored (empty chosen_lifts)
ros2 topic pub -1 /lift_select_requests rmf_lift_msgs/msg/LiftSelectRequest \
  "{session_id: robot3, request_type: 1, origin_floor: L1, chosen_lifts: []}"
# -> warning, ignored (REQUEST_END_SESSION)
ros2 topic pub -1 /lift_select_requests rmf_lift_msgs/msg/LiftSelectRequest \
  "{session_id: robot1, request_type: 0, origin_floor: L1, chosen_lifts: [A]}"
```

In Gazebo, the lift plugin publishes `/lift_states` by itself. Use real lift names and level names from the building map, then check that the chosen cabin moves to `origin_floor`, opens its doors, and reports the session in `LiftState.session_id`.

## Known limitations

- A lift that has never published a `LiftState` is never selected.
- If `lift.cpp` rejects a request (for example because `origin_floor` is not a floor that lift serves), the reservation expires after `confirmation_timeout` and a warning is logged. The request is not retried.
- Lift sessions are released only through the conventional `REQUEST_END_SESSION` sent on `/lift_requests` by the existing RMF workflow.
