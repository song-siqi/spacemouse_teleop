import unittest
import xml.etree.ElementTree as ET

from spacemouse_teleop.backends.mujoco import (
    CAMERA_NAMES,
    GRIPPER_ACTUATOR_NAMES,
    GRIPPER_BODY_NAMES,
    GRIPPER_JOINT_NAMES,
    GRIPPER_FINGER_MESH_COLLISION_GEOM_NAMES,
    GRIPPER_PAD_GEOM_NAMES,
    JOINT_NAMES,
    XArm6TableCubeEnv,
    ensure_official_xarm6_table_cube_mjcf,
)
from spacemouse_teleop.spacemouse.command import TeleopCommand


class MujocoAssetTest(unittest.TestCase):
    def test_xarm6_table_cube_model_has_expected_robot_scene_contract(self):
        tree = ET.parse(ensure_official_xarm6_table_cube_mjcf())
        root = tree.getroot()

        joint_names = {joint.attrib["name"] for joint in root.findall(".//joint")}
        actuator_names = {
            actuator.attrib["name"] for actuator in root.findall(".//actuator/position")
        }
        body_names = {body.attrib["name"] for body in root.findall(".//body")}
        camera_names = {
            camera.attrib["name"] for camera in root.findall(".//camera")
        }
        gravcomp_body_names = {
            body.attrib["name"]
            for body in root.findall(".//body")
            if body.attrib.get("gravcomp") == "1"
        }
        option = root.find("option")
        site_names = {site.attrib["name"] for site in root.findall(".//site")}
        geom_names = {
            geom.attrib["name"]
            for geom in root.findall(".//geom")
            if "name" in geom.attrib
        }

        self.assertIsNotNone(option)
        self.assertEqual(option.attrib.get("timestep"), "0.001")
        self.assertEqual(option.attrib.get("solver"), "Newton")
        self.assertEqual(option.attrib.get("noslip_iterations"), "12")
        self.assertTrue(set(JOINT_NAMES).issubset(joint_names))
        self.assertTrue(set(GRIPPER_JOINT_NAMES).issubset(joint_names))
        self.assertTrue({f"{name}_pos" for name in JOINT_NAMES}.issubset(actuator_names))
        self.assertTrue(set(GRIPPER_ACTUATOR_NAMES).issubset(actuator_names))
        self.assertTrue(set(CAMERA_NAMES).issubset(camera_names))
        self.assertTrue({f"link{i}" for i in range(1, 7)}.issubset(gravcomp_body_names))
        self.assertIn("cube", body_names)
        self.assertIn("xarm_gripper_base_link", body_names)
        eef_site = root.find(".//body[@name='xarm_gripper_base_link']/site[@name='eef']")
        self.assertIsNotNone(eef_site)
        self.assertEqual(eef_site.attrib.get("pos"), "0 0 0.112")
        self.assertIn("eef", site_names)
        self.assertIn("table", geom_names)
        self.assertTrue(set(GRIPPER_PAD_GEOM_NAMES).issubset(geom_names))
        self.assertTrue(
            set(GRIPPER_FINGER_MESH_COLLISION_GEOM_NAMES).issubset(geom_names)
        )
        table_geom = root.find(".//geom[@name='table']")
        self.assertIsNotNone(table_geom)
        self.assertEqual(table_geom.attrib.get("condim"), "3")
        self.assertEqual(table_geom.attrib.get("priority"), "2")
        self.assertEqual(table_geom.attrib.get("solref"), "0.0015 1")
        cube_geom = root.find(".//geom[@name='cube_geom']")
        self.assertIsNotNone(cube_geom)
        self.assertEqual(cube_geom.attrib.get("contype"), "3")
        self.assertEqual(cube_geom.attrib.get("conaffinity"), "3")
        self.assertEqual(cube_geom.attrib.get("condim"), "3")
        self.assertGreaterEqual(
            len(root.findall(".//equality/joint")),
            len(GRIPPER_JOINT_NAMES) - 1,
        )
        for geom_name in GRIPPER_PAD_GEOM_NAMES:
            pad_geom = root.find(f".//geom[@name='{geom_name}']")
            self.assertIsNotNone(pad_geom)
            self.assertEqual(pad_geom.attrib.get("contype"), "2")
            self.assertEqual(pad_geom.attrib.get("conaffinity"), "2")
            self.assertEqual(pad_geom.attrib.get("condim"), "3")
        active_finger_meshes = set(GRIPPER_FINGER_MESH_COLLISION_GEOM_NAMES)
        for body_name in GRIPPER_BODY_NAMES:
            body = root.find(f".//body[@name='{body_name}']")
            self.assertIsNotNone(body)
            for geom in body.findall("geom"):
                if geom.attrib.get("type") == "mesh":
                    if geom.attrib.get("name") in active_finger_meshes:
                        self.assertEqual(geom.attrib.get("contype"), "2")
                        self.assertEqual(geom.attrib.get("conaffinity"), "2")
                        self.assertEqual(geom.attrib.get("condim"), "3")
                    else:
                        self.assertEqual(geom.attrib.get("contype"), "0")
                        self.assertEqual(geom.attrib.get("conaffinity"), "0")

    def test_generated_model_loads_with_mujoco_actuators(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        model_path = ensure_official_xarm6_table_cube_mjcf()
        model = mujoco.MjModel.from_xml_path(str(model_path))
        actuator_names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
            for index in range(model.nu)
        }
        camera_names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, index)
            for index in range(model.ncam)
        }

        self.assertEqual(model.nu, len(JOINT_NAMES) + len(GRIPPER_ACTUATOR_NAMES))
        self.assertTrue({f"{name}_pos" for name in JOINT_NAMES}.issubset(actuator_names))
        self.assertTrue(set(GRIPPER_ACTUATOR_NAMES).issubset(actuator_names))
        self.assertTrue(set(CAMERA_NAMES).issubset(camera_names))

    def test_mujoco_env_actuates_closedness_to_mimic_gripper_joints(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        env = XArm6TableCubeEnv(control_hz=60)
        env.reset()
        command = TeleopCommand(
            linear_vel_mps=(0.0, 0.0, 0.0),
            angular_vel_radps=(0.0, 0.0, 0.0),
            delta_pos_m=(0.0, 0.0, 0.0),
            delta_rot_rad=(0.0, 0.0, 0.0),
            gripper=0.5,
            enabled=True,
            frame="link_base",
            dt=1.0 / 60.0,
            timestamp=0.0,
        )
        observation = env.observe()
        for _ in range(180):
            observation = env.step_command(command)

        self.assertAlmostEqual(observation.gripper_closedness, 0.5, delta=0.005)
        self.assertTrue(
            all(abs(value - 0.425) < 0.005 for value in observation.gripper_joint_pos)
        )

    def test_kinematic_gripper_lift_can_raise_cube(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        hz = 60.0
        env = XArm6TableCubeEnv(
            control_hz=hz,
            target_mode="velocity",
            arm_control_mode="kinematic",
            ik_position_gain=1.0,
            ik_orientation_gain=0.35,
        )
        observation = env.reset()
        start_pos = observation.ee_pos
        target_pos = (0.450, 0.0, 0.755)
        approach_steps = int(round(2.0 * hz))
        for _ in range(approach_steps):
            delta = tuple(
                (target_pos[i] - start_pos[i]) / approach_steps for i in range(3)
            )
            observation = env.step_command(_mujoco_command(delta, 0.0, hz))

        close_steps = int(round(3.0 * hz))
        for index in range(1, close_steps + 1):
            gripper = min(1.0, index / close_steps)
            observation = env.step_command(
                _mujoco_command((0.0, 0.0, 0.0), gripper, hz)
            )
        closed_cube_z = observation.cube_pos[2]

        lift_steps = int(round(1.5 * hz))
        for _ in range(lift_steps):
            observation = env.step_command(
                _mujoco_command((0.0, 0.0, 0.12 / lift_steps), 1.0, hz)
            )

        self.assertGreater(observation.cube_pos[2] - closed_cube_z, 0.08)

    def test_kinematic_gripper_press_keeps_cube_above_table(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        hz = 60.0
        env = XArm6TableCubeEnv(
            control_hz=hz,
            target_mode="velocity",
            arm_control_mode="kinematic",
            ik_position_gain=1.0,
            ik_orientation_gain=0.35,
        )
        observation = env.reset()
        start_pos = observation.ee_pos
        target_pos = (0.450, 0.0, 0.775)
        min_cube_bottom_gap = _cube_bottom_gap(observation)
        min_table_contact_dist = _cube_table_contact_dist(env)

        approach_steps = int(round(2.0 * hz))
        for _ in range(approach_steps):
            delta = tuple(
                (target_pos[i] - start_pos[i]) / approach_steps for i in range(3)
            )
            observation = env.step_command(_mujoco_command(delta, 0.0, hz))
            min_cube_bottom_gap = min(
                min_cube_bottom_gap, _cube_bottom_gap(observation)
            )
            min_table_contact_dist = _min_optional(
                min_table_contact_dist, _cube_table_contact_dist(env)
            )

        close_steps = int(round(1.5 * hz))
        for index in range(1, close_steps + 1):
            gripper = min(1.0, index / close_steps)
            observation = env.step_command(
                _mujoco_command((0.0, 0.0, 0.0), gripper, hz)
            )
            min_cube_bottom_gap = min(
                min_cube_bottom_gap, _cube_bottom_gap(observation)
            )
            min_table_contact_dist = _min_optional(
                min_table_contact_dist, _cube_table_contact_dist(env)
            )

        press_steps = int(round(1.0 * hz))
        for _ in range(press_steps):
            observation = env.step_command(
                _mujoco_command((0.0, 0.0, -0.04 / press_steps), 1.0, hz)
            )
            min_cube_bottom_gap = min(
                min_cube_bottom_gap, _cube_bottom_gap(observation)
            )
            min_table_contact_dist = _min_optional(
                min_table_contact_dist, _cube_table_contact_dist(env)
            )

        contact_penetration = max(0.0, -(min_table_contact_dist or 0.0))
        bottom_penetration = max(0.0, -min_cube_bottom_gap)
        self.assertLessEqual(max(contact_penetration, bottom_penetration), 0.002)


def _mujoco_command(delta_pos, gripper, hz):
    dt = 1.0 / hz
    return TeleopCommand(
        linear_vel_mps=tuple(value / dt for value in delta_pos),
        angular_vel_radps=(0.0, 0.0, 0.0),
        delta_pos_m=delta_pos,
        delta_rot_rad=(0.0, 0.0, 0.0),
        gripper=gripper,
        enabled=True,
        frame="link_base",
        dt=dt,
        timestamp=0.0,
    )


def _cube_bottom_gap(observation):
    return float(observation.cube_pos[2]) - 0.025 - 0.72


def _cube_table_contact_dist(env):
    distances = []
    for index in range(env.data.ncon):
        contact = env.data.contact[index]
        geom1 = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
        )
        geom2 = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
        )
        if {str(geom1), str(geom2)} == {"table", "cube_geom"}:
            distances.append(float(contact.dist))
    return min(distances) if distances else None


def _min_optional(first, second):
    if first is None:
        return second
    if second is None:
        return first
    return min(first, second)


if __name__ == "__main__":
    unittest.main()
