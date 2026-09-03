# xArm ROS2 Backend

This backend should adapt `TeleopCommand` into the command formats used by `xarm_ros2`.

The reference implementation in `third_party/xarm_ros2/xarm_moveit_servo` uses:

```text
TeleopCommand-like Cartesian velocity
  -> geometry_msgs/msg/TwistStamped
  -> /servo_server/delta_twist_cmds
  -> moveit_servo::ServoNode
  -> /xarm6_traj_controller/joint_trajectory
  -> ros2_control
  -> xArm SDK set_servo_angle_j
```

Initial implementation target:

```text
TeleopCommand.linear_vel_mps + angular_vel_radps
  -> TwistStamped.header.frame_id = command.frame
  -> TwistStamped.twist.linear / angular
```

The official xArm Gripper should use the backend-neutral normalized closedness
field:

```text
TeleopCommand.gripper = 0.0  fully open
TeleopCommand.gripper = 1.0  fully closed
```

Map that value into the official xArm ROS2 interfaces inside this backend:

```text
/xarm_gripper/gripper_action position = 0.86 * gripper
/xarm/set_gripper_position pos = 850 * (1 - gripper)
```

The service path is reversed because xArm API pulses use `0` for closed and
`850` for fully open. During teleop, publish gripper targets at a lower rate
than the Cartesian servo loop, for example `10-20 Hz`, and skip service calls
when the closedness target has not changed by a meaningful deadband.

The backend should add real-robot safety gates before publishing to the servo topic:

- deadman required
- command timeout publishes zero
- conservative speed limits
- optional frame switch between `link_base` and tool/TCP
- recorder hooks for commanded joint target and observed state
- gripper enable/mode/speed initialization before accepting gripper commands
