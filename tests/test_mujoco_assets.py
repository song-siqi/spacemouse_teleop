import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from spacemouse_teleop.backends.mujoco import (
    CAMERA_NAMES,
    CUBE_FRICTION,
    FINGER_PAD_FRICTION,
    FINGER_PAD_SOLIMP,
    FINGER_PAD_SOLREF,
    GRIPPER_ACTUATOR_FORCE_RANGE,
    GRIPPER_ACTUATOR_KP,
    GRIPPER_ACTUATOR_KV,
    GRIPPER_ACTUATOR_NAMES,
    GRIPPER_BODY_NAMES,
    GRIPPER_FINGER_MESH_COLLISION_GEOM_NAMES,
    GRIPPER_GUARD_COLLISION_BIT,
    GRIPPER_GUARD_FRICTION,
    GRIPPER_GUARD_GEOM_NAMES,
    GRIPPER_GUARD_SOLIMP,
    GRIPPER_GUARD_SOLREF,
    GRIPPER_JOINT_LIMIT_RAD,
    GRIPPER_JOINT_NAMES,
    GRIPPER_PAD_COLLISION_BIT,
    GRIPPER_PAD_GEOM_NAMES,
    JOINT_NAMES,
    LOCAL_XARM_ROS2_PATH,
    MANIPULATION_OBJECT_COLLISION_MASK,
    SCENE_COLLISION_BIT,
    TABLE_FRICTION,
    XArm6TableCubeEnv,
    ensure_official_xarm6_table_cube_mjcf,
)
from spacemouse_teleop.backends.mujoco.contact_diagnostics import (
    capture_contact_snapshot,
    summarize_contact_snapshots,
)
from spacemouse_teleop.spacemouse.command import TeleopCommand


class MujocoAssetTest(unittest.TestCase):
    def test_xarm6_table_cube_model_has_expected_robot_scene_contract(self):
        tree = ET.parse(ensure_official_xarm6_table_cube_mjcf())
        root = tree.getroot()
        model_version = root.find(
            ".//custom/numeric[@name='spacemouse_teleop_model_version']"
        )

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
        self.assertIsNotNone(model_version)
        self.assertEqual(model_version.attrib.get("data"), "3")
        self.assertEqual(option.attrib.get("timestep"), "0.001")
        self.assertEqual(option.attrib.get("solver"), "Newton")
        self.assertEqual(option.attrib.get("noslip_iterations"), "12")
        self.assertTrue(set(JOINT_NAMES).issubset(joint_names))
        self.assertTrue(set(GRIPPER_JOINT_NAMES).issubset(joint_names))
        self.assertTrue(
            {f"{name}_pos" for name in JOINT_NAMES}.issubset(actuator_names)
        )
        self.assertTrue(set(GRIPPER_ACTUATOR_NAMES).issubset(actuator_names))
        self.assertTrue(set(CAMERA_NAMES).issubset(camera_names))
        self.assertTrue({f"link{i}" for i in range(1, 7)}.issubset(gravcomp_body_names))
        self.assertIn("cube", body_names)
        self.assertIn("xarm_gripper_base_link", body_names)
        eef_site = root.find(
            ".//body[@name='xarm_gripper_base_link']/site[@name='eef']"
        )
        self.assertIsNotNone(eef_site)
        self.assertEqual(eef_site.attrib.get("pos"), "0 0 0.112")
        self.assertIn("eef", site_names)
        self.assertIn("table", geom_names)
        gripper_actuator = root.find(".//actuator/position[@name='gripper_pos']")
        self.assertIsNotNone(gripper_actuator)
        self.assertEqual(gripper_actuator.attrib.get("kp"), GRIPPER_ACTUATOR_KP)
        self.assertEqual(gripper_actuator.attrib.get("kv"), GRIPPER_ACTUATOR_KV)
        self.assertEqual(
            gripper_actuator.attrib.get("forcerange"), GRIPPER_ACTUATOR_FORCE_RANGE
        )
        self.assertTrue(set(GRIPPER_PAD_GEOM_NAMES).issubset(geom_names))
        self.assertTrue(
            set(GRIPPER_FINGER_MESH_COLLISION_GEOM_NAMES).issubset(geom_names)
        )
        table_geom = root.find(".//geom[@name='table']")
        self.assertIsNotNone(table_geom)
        self.assertEqual(table_geom.attrib.get("condim"), "3")
        self.assertEqual(table_geom.attrib.get("priority"), "2")
        self.assertEqual(table_geom.attrib.get("friction"), TABLE_FRICTION)
        self.assertEqual(table_geom.attrib.get("solref"), "0.0015 1")
        self.assertEqual(table_geom.attrib.get("contype"), str(SCENE_COLLISION_BIT))
        self.assertEqual(
            table_geom.attrib.get("conaffinity"), str(SCENE_COLLISION_BIT)
        )
        floor_geom = root.find(".//geom[@name='floor']")
        self.assertIsNotNone(floor_geom)
        self.assertEqual(floor_geom.attrib.get("contype"), str(SCENE_COLLISION_BIT))
        self.assertEqual(
            floor_geom.attrib.get("conaffinity"), str(SCENE_COLLISION_BIT)
        )
        cube_geom = root.find(".//geom[@name='cube_geom']")
        self.assertIsNotNone(cube_geom)
        self.assertEqual(cube_geom.attrib.get("contype"), str(SCENE_COLLISION_BIT))
        self.assertEqual(
            cube_geom.attrib.get("conaffinity"),
            str(MANIPULATION_OBJECT_COLLISION_MASK),
        )
        self.assertEqual(cube_geom.attrib.get("condim"), "3")
        self.assertEqual(cube_geom.attrib.get("priority"), "1")
        self.assertEqual(cube_geom.attrib.get("friction"), CUBE_FRICTION)
        self.assertFalse(
            any(
                "cube_geom" in (pair.attrib.get("geom1"), pair.attrib.get("geom2"))
                and bool(
                    set(GRIPPER_PAD_GEOM_NAMES).intersection(
                        (pair.attrib.get("geom1"), pair.attrib.get("geom2"))
                    )
                )
                for pair in root.findall(".//contact/pair")
            )
        )
        self.assertGreaterEqual(
            len(root.findall(".//equality/joint")),
            len(GRIPPER_JOINT_NAMES) - 1,
        )
        for geom_name in GRIPPER_PAD_GEOM_NAMES:
            pad_geom = root.find(f".//geom[@name='{geom_name}']")
            self.assertIsNotNone(pad_geom)
            self.assertEqual(
                pad_geom.attrib.get("contype"), str(GRIPPER_PAD_COLLISION_BIT)
            )
            self.assertEqual(pad_geom.attrib.get("conaffinity"), "0")
            self.assertEqual(pad_geom.attrib.get("condim"), "4")
            self.assertEqual(pad_geom.attrib.get("priority"), "3")
            self.assertEqual(pad_geom.attrib.get("friction"), FINGER_PAD_FRICTION)
            self.assertEqual(pad_geom.attrib.get("solimp"), FINGER_PAD_SOLIMP)
            self.assertEqual(pad_geom.attrib.get("solref"), FINGER_PAD_SOLREF)
            self.assertEqual(pad_geom.attrib.get("margin"), "0.001")
        for geom_name in GRIPPER_GUARD_GEOM_NAMES:
            guard_geom = root.find(f".//geom[@name='{geom_name}']")
            self.assertIsNotNone(guard_geom)
            self.assertEqual(
                guard_geom.attrib.get("contype"), str(GRIPPER_GUARD_COLLISION_BIT)
            )
            self.assertEqual(guard_geom.attrib.get("conaffinity"), "0")
            self.assertEqual(guard_geom.attrib.get("condim"), "3")
            self.assertEqual(guard_geom.attrib.get("priority"), "2")
            self.assertEqual(guard_geom.attrib.get("friction"), GRIPPER_GUARD_FRICTION)
            self.assertEqual(guard_geom.attrib.get("solimp"), GRIPPER_GUARD_SOLIMP)
            self.assertEqual(guard_geom.attrib.get("solref"), GRIPPER_GUARD_SOLREF)
            self.assertEqual(guard_geom.attrib.get("margin"), "0")
        for body_name in GRIPPER_BODY_NAMES:
            body = root.find(f".//body[@name='{body_name}']")
            self.assertIsNotNone(body)
            for geom in body.findall("geom"):
                if geom.attrib.get("type") == "mesh":
                    self.assertEqual(geom.attrib.get("contype"), "0")
                    self.assertEqual(geom.attrib.get("conaffinity"), "0")

    def test_mujoco_assets_are_repo_local_not_third_party_runtime_links(self):
        self.assertTrue(LOCAL_XARM_ROS2_PATH.is_dir())
        self.assertNotIn("third_party", LOCAL_XARM_ROS2_PATH.as_posix())

        expected_files = (
            "xarm_description/urdf/xarm_device.urdf.xacro",
            "xarm_description/urdf/xarm6/xarm6_robot_macro.xacro",
            "xarm_description/urdf/gripper/xarm_gripper_macro.xacro",
            "xarm_description/meshes/xarm6/visual/link_base.stl",
            "xarm_description/meshes/gripper/xarm/left_finger.stl",
            "xarm_controller/config/xarm6_controllers.yaml",
        )
        for relative_path in expected_files:
            self.assertTrue((LOCAL_XARM_ROS2_PATH / relative_path).is_file())

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
        self.assertTrue(
            {f"{name}_pos" for name in JOINT_NAMES}.issubset(actuator_names)
        )
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

    def test_mujoco_gripper_delta_reverses_from_observed_closedness(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        hz = 10.0
        env = XArm6TableCubeEnv(control_hz=hz)
        env.reset()
        lower, upper = GRIPPER_JOINT_LIMIT_RAD
        observed_closedness = 0.55
        env.target_gripper_closedness = 1.0
        env.data.qpos[env.gripper_qpos_ids] = lower + (
            upper - lower
        ) * observed_closedness
        env.mujoco.mj_forward(env.model, env.data)
        observed = env.observe().gripper_closedness

        observation = env.step_command(
            TeleopCommand(
                linear_vel_mps=(0.0, 0.0, 0.0),
                angular_vel_radps=(0.0, 0.0, 0.0),
                delta_pos_m=(0.0, 0.0, 0.0),
                delta_rot_rad=(0.0, 0.0, 0.0),
                enabled=True,
                frame="link_base",
                dt=1.0 / hz,
                timestamp=0.0,
                delta_gripper=-0.1,
                gripper_velocity=-1.0,
            )
        )

        self.assertAlmostEqual(
            observation.target_gripper_closedness, observed - 0.1, delta=0.01
        )
        self.assertLess(observation.target_gripper_closedness, observed)

    def test_mujoco_gripper_intent_targets_endpoints_immediately(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        env = XArm6TableCubeEnv(control_hz=60.0)
        env.reset()
        env.target_gripper_closedness = 1.0

        opened = env.step_command(
            TeleopCommand(
                linear_vel_mps=(0.0, 0.0, 0.0),
                angular_vel_radps=(0.0, 0.0, 0.0),
                delta_pos_m=(0.0, 0.0, 0.0),
                delta_rot_rad=(0.0, 0.0, 0.0),
                enabled=True,
                frame="link_base",
                dt=1.0 / 60.0,
                timestamp=0.0,
                gripper_intent="open",
            )
        )
        self.assertEqual(opened.target_gripper_closedness, 0.0)

        closed = env.step_command(
            TeleopCommand(
                linear_vel_mps=(0.0, 0.0, 0.0),
                angular_vel_radps=(0.0, 0.0, 0.0),
                delta_pos_m=(0.0, 0.0, 0.0),
                delta_rot_rad=(0.0, 0.0, 0.0),
                enabled=True,
                frame="link_base",
                dt=1.0 / 60.0,
                timestamp=0.0,
                gripper_intent="close",
            )
        )
        self.assertEqual(closed.target_gripper_closedness, 1.0)

    def test_kinematic_gripper_lift_can_raise_cube(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        for hz in (30.0, 60.0, 120.0):
            with self.subTest(hz=hz):
                env = _make_env(hz)
                observation = _approach_and_close(env, hz, close_duration=3.0)
                closed_cube_z = observation.cube_pos[2]

                lift_steps = int(round(1.5 * hz))
                snapshots = []
                for _ in range(lift_steps):
                    observation = env.step_command(
                        _mujoco_command(
                            (0.0, 0.0, 0.12 / lift_steps), 1.0, hz
                        )
                    )
                    snapshots.append(capture_contact_snapshot(env, "lift"))

                summary = summarize_contact_snapshots(snapshots)[0]
                self.assertGreater(observation.cube_pos[2] - closed_cube_z, 0.08)
                self.assertGreaterEqual(summary.left_pad.contact_fraction, 0.8)
                self.assertGreaterEqual(summary.right_pad.contact_fraction, 0.8)

                hold_start_pos = observation.cube_pos
                hold_start_quat = observation.cube_quat
                for _ in range(int(round(2.0 * hz))):
                    observation = env.step_command(
                        _mujoco_command((0.0, 0.0, 0.0), 1.0, hz)
                    )
                self.assertLess(math.dist(hold_start_pos, observation.cube_pos), 0.005)
                self.assertLess(
                    _quat_distance(hold_start_quat, observation.cube_quat),
                    math.radians(5.0),
                )

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

    def test_kinematic_top_press_stops_at_gripper_guard(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        for hz in (30.0, 60.0, 120.0):
            with self.subTest(hz=hz):
                env = _make_env(hz)
                blocked, guard_penetration, table_penetration, retreat = (
                    _exercise_top_press(env, hz, "cube_geom")
                )
                self.assertTrue(blocked)
                self.assertLessEqual(guard_penetration, 0.0005)
                self.assertLessEqual(table_penetration, 0.002)
                self.assertGreater(retreat, 0.02)

    def test_kinematic_lateral_guard_contact_pushes_cube(self):
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
        initial = env.reset()
        start_pos = initial.ee_pos
        approach_pos = (0.450, 0.140, 0.755)
        approach_steps = int(round(2.0 * hz))
        for _ in range(approach_steps):
            delta = tuple(
                (approach_pos[index] - start_pos[index]) / approach_steps
                for index in range(3)
            )
            env.step_command(_mujoco_command(delta, 0.0, hz))

        before_push = env.observe()
        push_steps = int(round(2.0 * hz))
        min_guard_dist = None
        after_push = before_push
        for _ in range(push_steps):
            after_push = env.step_command(
                _mujoco_command((0.0, -0.160 / push_steps, 0.0), 0.0, hz)
            )
            min_guard_dist = _min_optional(
                min_guard_dist, _guard_cube_contact_dist(env)
            )

        cube_displacement = before_push.cube_pos[1] - after_push.cube_pos[1]
        self.assertGreater(cube_displacement, 0.04)
        self.assertLessEqual(max(0.0, -(min_guard_dist or 0.0)), 0.0025)

        retreat_start = after_push.cube_pos
        retreat_steps = int(round(0.75 * hz))
        for _ in range(retreat_steps):
            after_push = env.step_command(
                _mujoco_command(
                    (0.0, 0.05 / retreat_steps, 0.0), 0.0, hz
                )
            )
        self.assertIsNone(_guard_object_contact_dist(env, "cube_geom"))
        self.assertLess(math.dist(retreat_start, after_push.cube_pos), 0.005)

    def test_kinematic_gripper_rotation_tracks_cube_without_flipping(self):
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
        initial = env.reset()
        target_pos = (0.450, 0.0, 0.755)
        approach_steps = int(round(2.0 * hz))
        for _ in range(approach_steps):
            delta = tuple(
                (target_pos[index] - initial.ee_pos[index]) / approach_steps
                for index in range(3)
            )
            env.step_command(_mujoco_command(delta, 0.0, hz))

        close_steps = int(round(2.0 * hz))
        for index in range(1, close_steps + 1):
            env.step_command(
                _mujoco_command(
                    (0.0, 0.0, 0.0), min(1.0, index / close_steps), hz
                )
            )

        lift_steps = int(round(1.5 * hz))
        for _ in range(lift_steps):
            env.step_command(
                _mujoco_command((0.0, 0.0, 0.08 / lift_steps), 1.0, hz)
            )

        rotate_steps = int(round(1.5 * hz))
        snapshots = []
        for _ in range(rotate_steps):
            env.step_command(
                _mujoco_command(
                    (0.0, 0.0, 0.0),
                    1.0,
                    hz,
                    delta_rot=(0.0, 0.0, math.radians(60.0) / rotate_steps),
                )
            )
            snapshots.append(capture_contact_snapshot(env, "rotate"))

        summary = summarize_contact_snapshots(snapshots)[0]
        self.assertGreater(summary.net_cube_rotation_rad, math.radians(4.0))
        self.assertLess(summary.relative_rotation_drift_rad, math.radians(10.0))
        self.assertGreaterEqual(summary.left_pad.contact_fraction, 0.8)
        self.assertGreaterEqual(summary.right_pad.contact_fraction, 0.8)
        self.assertLess(summary.max_normal_force_n, 80.0)
        self.assertLess(summary.max_penetration_m, 0.002)

    def test_gripper_collision_debug_toggles_pad_and_guard_visibility(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        env = _make_env(60.0)
        env.reset()
        env.set_gripper_collision_debug(True)
        for geom_name in (*GRIPPER_PAD_GEOM_NAMES, *GRIPPER_GUARD_GEOM_NAMES):
            geom_id = env.mujoco.mj_name2id(
                env.model, env.mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            self.assertAlmostEqual(float(env.model.geom_rgba[geom_id][3]), 0.35)

        env.set_gripper_collision_debug(False)
        for geom_name in (*GRIPPER_PAD_GEOM_NAMES, *GRIPPER_GUARD_GEOM_NAMES):
            geom_id = env.mujoco.mj_name2id(
                env.model, env.mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            self.assertEqual(float(env.model.geom_rgba[geom_id][3]), 0.0)

    def test_guard_constraints_support_renamed_box_and_cylinder_objects(self):
        try:
            import mujoco  # noqa: F401
        except ImportError:
            self.skipTest("MuJoCo is not installed")

        base_model_path = ensure_official_xarm6_table_cube_mjcf()
        with tempfile.TemporaryDirectory() as temp_dir:
            for shape in ("box", "cylinder"):
                with self.subTest(shape=shape):
                    body_name = f"test_{shape}_object"
                    geom_name = f"test_{shape}_geom"
                    model_path = Path(temp_dir) / f"{shape}.xml"
                    _write_object_variant(
                        base_model_path,
                        model_path,
                        body_name=body_name,
                        geom_name=geom_name,
                        shape=shape,
                    )

                    top_env = _make_env(
                        60.0,
                        model_path=model_path,
                        cube_body_name=body_name,
                    )
                    blocked, guard_penetration, table_penetration, retreat = (
                        _exercise_top_press(top_env, 60.0, geom_name)
                    )
                    self.assertTrue(blocked)
                    self.assertLessEqual(guard_penetration, 0.0005)
                    self.assertLessEqual(table_penetration, 0.002)
                    self.assertGreater(retreat, 0.02)

                    push_env = _make_env(
                        60.0,
                        model_path=model_path,
                        cube_body_name=body_name,
                    )
                    displacement, guard_penetration = _exercise_lateral_push(
                        push_env, 60.0, geom_name
                    )
                    self.assertGreater(displacement, 0.04)
                    self.assertLessEqual(guard_penetration, 0.0025)
                    snapshot = capture_contact_snapshot(
                        push_env,
                        "generic_object",
                        object_body_name=body_name,
                        object_geom_names=(geom_name,),
                    )
                    self.assertEqual(snapshot.object_body_name, body_name)
                    self.assertEqual(snapshot.object_geom_names, (geom_name,))

    def test_actuator_mode_contact_smoke_is_finite_and_bounded(self):
        try:
            import mujoco  # noqa: F401
            import numpy as np
        except ImportError:
            self.skipTest("MuJoCo and NumPy are required")

        hz = 60.0
        env = _make_env(hz)
        _approach_and_close(env, hz, close_duration=2.0)
        env.arm_control_mode = "actuator"
        snapshots = []
        for gripper in (0.0, 1.0):
            for _ in range(int(round(0.5 * hz))):
                observation = env.step_command(
                    _mujoco_command((0.0, 0.0, -0.002 / hz), gripper, hz)
                )
                snapshots.append(capture_contact_snapshot(env, "actuator"))

        summary = summarize_contact_snapshots(snapshots)[0]
        state = np.concatenate(
            (
                np.asarray(observation.joint_pos),
                np.asarray(observation.gripper_joint_pos),
                np.asarray(observation.cube_pos),
                np.asarray(env.data.qvel),
            )
        )
        self.assertTrue(np.all(np.isfinite(state)))
        self.assertLess(float(np.max(np.abs(env.data.qvel))), 100.0)
        self.assertLessEqual(summary.max_penetration_m, 0.003)


def _make_env(
    hz,
    *,
    model_path=None,
    cube_body_name="cube",
    arm_control_mode="kinematic",
):
    return XArm6TableCubeEnv(
        model_path=model_path,
        cube_body_name=cube_body_name,
        control_hz=hz,
        target_mode="velocity",
        arm_control_mode=arm_control_mode,
        ik_position_gain=1.0,
        ik_orientation_gain=0.35,
    )


def _approach_and_close(env, hz, close_duration):
    observation = env.reset()
    start_pos = observation.ee_pos
    target_pos = (0.450, 0.0, 0.755)
    approach_steps = int(round(2.0 * hz))
    for _ in range(approach_steps):
        delta = tuple(
            (target_pos[index] - start_pos[index]) / approach_steps
            for index in range(3)
        )
        observation = env.step_command(_mujoco_command(delta, 0.0, hz))

    close_steps = int(round(close_duration * hz))
    for index in range(1, close_steps + 1):
        observation = env.step_command(
            _mujoco_command(
                (0.0, 0.0, 0.0), min(1.0, index / close_steps), hz
            )
        )
    return observation


def _write_object_variant(
    source_path,
    output_path,
    *,
    body_name,
    geom_name,
    shape,
):
    tree = ET.parse(source_path)
    root = tree.getroot()
    body = root.find(".//body[@name='cube']")
    if body is None:
        raise AssertionError("generated model has no cube body")
    body.set("name", body_name)
    joint = body.find("joint")
    geom = body.find("geom")
    if joint is None or geom is None:
        raise AssertionError("generated cube body is incomplete")
    joint.set("name", f"{body_name}_freejoint")
    geom.set("name", geom_name)
    geom.set("type", shape)
    geom.set("size", "0.025 0.025" if shape == "cylinder" else "0.025 0.025 0.025")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def _exercise_top_press(env, hz, object_geom_name):
    initial = env.reset()
    start_pos = initial.ee_pos
    approach_pos = (0.450, 0.0, 0.820)
    approach_steps = int(round(2.0 * hz))
    for _ in range(approach_steps):
        delta = tuple(
            (approach_pos[index] - start_pos[index]) / approach_steps
            for index in range(3)
        )
        env.step_command(_mujoco_command(delta, 0.0, hz))

    blocked = False
    min_guard_dist = None
    min_table_dist = None
    observation = env.observe()
    press_steps = int(round(1.0 * hz))
    for _ in range(press_steps):
        observation = env.step_command(
            _mujoco_command(
                (0.0, 0.0, (0.755 - 0.820) / press_steps), 0.0, hz
            )
        )
        blocked = blocked or env.last_kinematic_guard_blocked
        min_guard_dist = _min_optional(
            min_guard_dist,
            _guard_object_contact_dist(env, object_geom_name),
        )
        min_table_dist = _min_optional(
            min_table_dist,
            _object_table_contact_dist(env, object_geom_name),
        )
    blocked_z = observation.ee_pos[2]
    retreat_steps = int(round(0.5 * hz))
    for _ in range(retreat_steps):
        observation = env.step_command(
            _mujoco_command((0.0, 0.0, 0.03 / retreat_steps), 0.0, hz)
        )
    return (
        blocked,
        max(0.0, -(min_guard_dist or 0.0)),
        max(0.0, -(min_table_dist or 0.0)),
        observation.ee_pos[2] - blocked_z,
    )


def _exercise_lateral_push(env, hz, object_geom_name):
    initial = env.reset()
    start_pos = initial.ee_pos
    approach_pos = (0.450, 0.140, 0.755)
    approach_steps = int(round(2.0 * hz))
    for _ in range(approach_steps):
        delta = tuple(
            (approach_pos[index] - start_pos[index]) / approach_steps
            for index in range(3)
        )
        env.step_command(_mujoco_command(delta, 0.0, hz))

    before_push = env.observe()
    min_guard_dist = None
    push_steps = int(round(2.0 * hz))
    after_push = before_push
    for _ in range(push_steps):
        after_push = env.step_command(
            _mujoco_command((0.0, -0.160 / push_steps, 0.0), 0.0, hz)
        )
        min_guard_dist = _min_optional(
            min_guard_dist,
            _guard_object_contact_dist(env, object_geom_name),
        )
    return (
        before_push.cube_pos[1] - after_push.cube_pos[1],
        max(0.0, -(min_guard_dist or 0.0)),
    )


def _mujoco_command(delta_pos, gripper, hz, delta_rot=(0.0, 0.0, 0.0)):
    dt = 1.0 / hz
    return TeleopCommand(
        linear_vel_mps=tuple(value / dt for value in delta_pos),
        angular_vel_radps=tuple(value / dt for value in delta_rot),
        delta_pos_m=delta_pos,
        delta_rot_rad=delta_rot,
        gripper=gripper,
        enabled=True,
        frame="link_base",
        dt=dt,
        timestamp=0.0,
    )


def _cube_bottom_gap(observation):
    return float(observation.cube_pos[2]) - 0.025 - 0.72


def _cube_table_contact_dist(env):
    return _object_table_contact_dist(env, "cube_geom")


def _object_table_contact_dist(env, object_geom_name):
    distances = []
    for index in range(env.data.ncon):
        contact = env.data.contact[index]
        geom1 = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
        )
        geom2 = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
        )
        if {str(geom1), str(geom2)} == {"table", object_geom_name}:
            distances.append(float(contact.dist))
    return min(distances) if distances else None


def _guard_cube_contact_dist(env):
    return _guard_object_contact_dist(env, "cube_geom")


def _guard_object_contact_dist(env, object_geom_name):
    distances = []
    guard_names = set(GRIPPER_GUARD_GEOM_NAMES)
    for index in range(env.data.ncon):
        contact = env.data.contact[index]
        geom1 = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
        )
        geom2 = env.mujoco.mj_id2name(
            env.model, env.mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
        )
        names = {str(geom1), str(geom2)}
        if object_geom_name in names and names.intersection(guard_names):
            distances.append(float(contact.dist))
    return min(distances) if distances else None


def _quat_distance(first, second):
    import numpy as np

    first_value = np.asarray(first, dtype=float)
    second_value = np.asarray(second, dtype=float)
    first_value /= np.linalg.norm(first_value)
    second_value /= np.linalg.norm(second_value)
    dot = abs(float(np.dot(first_value, second_value)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _min_optional(first, second):
    if first is None:
        return second
    if second is None:
        return first
    return min(first, second)


if __name__ == "__main__":
    unittest.main()
