# Menagerie Assets

Source: https://github.com/google-deepmind/mujoco_menagerie

Pinned commit: `4d038b3feae26ec82b46a4d586379114012a8ac7`.

The `franka_fr3` and `robotiq_2f85` directories are copied unmodified, including
their upstream licenses. Runtime assembly uses MjSpec attachment, prefixes the
tool names, adds the tabletop scene, and assigns pad/shell contact materials.
The extra attachment transform is identity. This is an idealized simulation
mount, not a measured physical adapter. No physical payload calibration is implied.

Robotiq shell contact uses solref=(0.006, 1); pads use (0.003, 1).
The separate xArm dynamics shell uses solref=(0.002, 1) and
solimp=(0.95, 0.99, 0.001, 0.5, 2) for bounded top-press
penetration; its historical kinematic scene retains original parameters.

The controller slews normalized gripper targets at 2/s to avoid an instantaneous
open-to-close setpoint jump. Hold captures the measured position immediately.
This target rate is not a calibrated physical jaw speed.

Runtime contact material refinement: each original pad prism is split across its
thickness into an inward rubber layer (half-thickness 1.5 mm, center y=-5.1 mm)
and a low-friction backing (half-thickness 2.5 mm, center y=-1.1 mm). Their union
has the same boundary as the upstream 4 mm half-thickness prism centered at
y=-2.6 mm. Both are massless collision proxies; inertial, tendon and equality
definitions remain unchanged. This is a simulation material assignment, not a
claim about measured Robotiq rubber-layer thickness.
