# Third-Party Sources

This directory is reserved for upstream projects used as local references.

## xarm_ros2

`third_party/xarm_ros2` is expected to be a read-only clone of UFACTORY's
official ROS2 repository:

```bash
git clone https://github.com/xArm-Developer/xarm_ros2.git third_party/xarm_ros2
```

The MuJoCo backend does not depend on this ignored clone at runtime. The
simulation asset subset needed for xArm6 tabletop teleop is copied into
`src/spacemouse_teleop/backends/mujoco/assets/xarm_ros2`, with the upstream
license kept there. Use this local clone only to audit, compare, or refresh that
package-local asset subset. If we decide to pin the complete upstream repo in
git later, add it deliberately as a submodule.
