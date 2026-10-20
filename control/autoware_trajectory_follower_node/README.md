# Trajectory Follower Nodes

## Purpose

Generate control commands to follow a given Trajectory.

## Design

This is a node of the functionalities implemented in the lateral controller class derived from the [autoware_trajectory_follower_base](../autoware_trajectory_follower_base/README.md#trajectory-follower) interface, and in the longitudinal controller class of the [autoware_pid_longitudinal_controller](../autoware_pid_longitudinal_controller/README.md) package. It has instances of those functionalities, gives them input data to perform calculations, and publishes control commands.

By default, the controller instance with the `Controller` class as follows is used.

```plantuml
@startuml
package autoware_trajectory_follower_base {
abstract class LateralController {
virtual run(InputData, stamp)
}

struct InputData {
trajectory
odometry
steering
accel
}
struct LateralSyncData {
is_steer_converged
}
}

package autoware_mpc_lateral_controller {
class MpcLateralControllerNode {
run(InputData, stamp) override
}
}
package pure_pursuit {
class PurePursuitLateralControllerNode {
run(InputData, stamp) override
}
}
package pid_longitudinal_controller {
class PidLongitudinalController {
run(InputData, stamp, is_steer_converged)
}
}

package autoware_trajectory_follower_node {
class TrajectoryFollower {
lateral_controller_
longitudinal_controller_
is_steer_converged_
run(InputData, stamp): TrajectoryFollowerResult
}
class Controller {
trajectory_follower_
onTimer()
createInputData(): InputData
}
}

MpcLateralControllerNode --|> LateralController
PurePursuitLateralControllerNode --|> LateralController

LateralSyncData --> PidLongitudinalController
InputData ..> LateralController
InputData ..> PidLongitudinalController

LateralController --o TrajectoryFollower
PidLongitudinalController --o TrajectoryFollower
TrajectoryFollower --o Controller
InputData ..> Controller
@enduml
```

`TrajectoryFollower` holds the control logic and both controllers. It reads no clock, no parameter and no topic, so a caller outside ROS 2 can drive it in a loop and evaluate a controller without starting an executor.

```cpp
const auto result = trajectory_follower.run(input_data, stamp);
```

The process flow of `Controller` class is as follows.

```cpp
// 1. create input data
const auto input_data = createInputData(*get_clock());
if (!input_data) {
  return;
}

// 2. run the control logic
const auto result = trajectory_follower_->run(*input_data, stamp);
if (!result) {
  return;  // the error side says why, and the node logs it
}

// 3. publish what the longitudinal controller reported
longitudinal_reporter_->report(result->longitudinal_result);

// 4. publish control command
control_cmd_pub_->publish(result->control_cmd);
```

Giving the longitudinal controller information about steer convergence allows it to control steer when stopped if following parameters are `true`

- lateral controller
  - `keep_steer_control_until_converged`
- longitudinal controller
  - `enable_keep_stopped_until_steer_convergence`

### Inputs / Outputs / API

#### Inputs

- `autoware_planning_msgs/Trajectory` : reference trajectory to follow.
- `nav_msgs/Odometry`: current odometry
- `autoware_vehicle_msgs/SteeringReport` current steering

#### Outputs

- `autoware_control_msgs/Control`: message containing both lateral and longitudinal commands.
- `autoware_control_msgs/ControlHorizon`: message containing both lateral and longitudinal horizon commands. this is NOT published by default. by using this, the performance of vehicle control may be improved, and by turning the default on, it can be used as an experimental topic.

#### Parameter

- `trajectory_reference_mode`: `spatial` or `temporal` (default: `spatial`)
  - `spatial`: the reference trajectory is tracked based on distance, calculating the time step of
    each prediction point from distance and velocity.
  - `temporal`: the reference trajectory's `time_from_start` field is used directly to track the
    trajectory based on time steps.
  - This parameter is declared here and shared with the lateral (MPC) and longitudinal (PID)
    controllers.
- `ctrl_period`: control commands publishing period
- `timeout_thr_sec`: duration in second after which input messages are discarded.
  - Each time the node receives lateral and longitudinal commands from each controller, it publishes an `Control` if the following two conditions are met.
    1. Both commands have been received.
    2. The last received commands are not older than defined by `timeout_thr_sec`.
- `cyclic_message_timeout_thr_sec`: duration in second for monitoring incoming trajectory messages via diagnostic updater (default: 0.9)
  - The diagnostic updater will report ERROR status if:
    1. No trajectory message has been received yet.
    2. The elapsed time since the last trajectory message exceeds this threshold.
  - This parameter helps monitor the health of the trajectory input stream.
- `lateral_controller_mode`: `mpc` or `pure_pursuit`
  - (currently there is only `PID` for longitudinal controller)
- `enable_control_cmd_horizon_pub`: publish `ControlHorizon` or not (default: false)

## Debugging

Debug information are published by the lateral and longitudinal controller using `autoware_internal_debug_msgs/Float32MultiArrayStamped` messages.

A configuration file for [PlotJuggler](https://github.com/facontidavide/PlotJuggler) is provided in the `config` folder which, when loaded, allow to automatically subscribe and visualize information useful for debugging.

In addition, the predicted MPC trajectory is published on topic `output/lateral/predicted_trajectory` and can be visualized in Rviz.
