# Third-Party Sources

This directory is reserved for upstream projects used as local references or
optional model sources.

## xarm_ros2

`third_party/xarm_ros2` is expected to be a read-only clone of UFACTORY's
official ROS2 repository:

```bash
git clone https://github.com/xArm-Developer/xarm_ros2.git third_party/xarm_ros2
```

The MuJoCo backend uses this clone when present to render the official xArm6
xacro/URDF and gripper meshes. The top-level `.gitignore` excludes the local
clone so this project does not accidentally vendor the upstream repository. If
we decide to pin it in git later, add it deliberately as a submodule.
