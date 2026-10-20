# Trajectory Follower

This is the design document for the `trajectory_follower` package.

## Purpose / Use cases

<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package provides the lateral controller interface used by the node of the `autoware_trajectory_follower_node` package, and the data types shared between that node and the controllers.
We can implement a detailed lateral controller by deriving the lateral interface.

## Design

Each lateral algorithm inherits from `LateralController` and implements one function.

```cpp
tl::expected<LateralOutput, std::string> run(const InputData & input_data, const rclcpp::Time & stamp);
```

One call takes everything the controller reads for a cycle and returns everything it produced, so there is no order a caller has to know.
A controller that cannot produce a command yet returns the reason on the error side, and the node logs it.

The longitudinal controller has no interface. The node calls [autoware_pid_longitudinal_controller](../autoware_pid_longitudinal_controller/README.md) directly, because PID is its only implementation. It hands the controller the steer convergence reported by the lateral controller in the previous cycle.

See [the Design of Trajectory Follower Nodes](../autoware_trajectory_follower_node/README.md#Design) for how these functions work in the node.

## Separated lateral (steering) and longitudinal (velocity) controls

This longitudinal controller assumes that the roles of lateral and longitudinal control are separated as follows.

- Lateral control computes a target steering to keep the vehicle on the trajectory, assuming perfect velocity tracking.
- Longitudinal control computes a target velocity/acceleration to keep the vehicle velocity on the trajectory speed, assuming perfect trajectory tracking.

Ideally, dealing with the lateral and longitudinal control as a single mixed problem can achieve high performance. In contrast, there are two reasons to provide velocity controller as a stand-alone function, described below.

### Complex requirements for longitudinal motion

The longitudinal vehicle behavior that humans expect is difficult to express in a single logic. For example, the expected behavior just before stopping differs depending on whether the ego-position is ahead/behind of the stop line, or whether the current speed is higher/lower than the target speed to achieve a human-like movement.

In addition, some vehicles have difficulty measuring the ego-speed at extremely low speeds. In such cases, a configuration that can improve the functionality of the longitudinal control without affecting the lateral control is important.

There are many characteristics and needs that are unique to longitudinal control. Designing them separately from the lateral control keeps the modules less coupled and improves maintainability.

### Nonlinear coupling of lateral and longitudinal motion

The lat-lon mixed control problem is very complex and uses nonlinear optimization to achieve high performance. Since it is difficult to guarantee the convergence of the nonlinear optimization, a simple control logic is also necessary for development.

Also, the benefits of simultaneous longitudinal and lateral control are small if the vehicle doesn't move at high speed.

## Related issues

<!-- Required -->
