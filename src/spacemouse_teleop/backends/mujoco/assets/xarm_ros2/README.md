# xArm ROS2 MuJoCo Asset Subset

This directory contains the minimal UFACTORY `xarm_ros2` files needed by the
MuJoCo xArm6 tabletop scene generator.

It is intentionally copied into the MuJoCo backend package so simulator support
does not depend on the ignored `third_party/xarm_ros2` reference clone at
runtime. The local `third_party` checkout remains useful for auditing and
refreshing these assets, but generated MuJoCo models should resolve their mesh
and xacro inputs from this package-local directory.

Included subset:

- `xarm_description` xArm6 and xArm gripper xacro files
- xArm6 kinematics and inertial yaml files
- xArm6, end-tool, and xArm gripper meshes
- the small `xarm_controller/config` files referenced by the upstream xacro

The copied files come from UFACTORY's `xArm-Developer/xarm_ros2` repository.
See `LICENSE` in this directory for the upstream license.
