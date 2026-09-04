from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from pathlib import Path
from typing import Mapping, Optional, Sequence, Tuple

from spacemouse_teleop.backends.mujoco.constants import (
    END_EFFECTOR_NAMES,
    END_EFFECTOR_NONE,
    END_EFFECTOR_XARM_GRIPPER,
    GRIPPER_ACTUATOR_NAMES,
    GRIPPER_BODY_NAMES,
    GRIPPER_DRIVE_JOINT_NAME,
    GRIPPER_FINGER_MESH_COLLISION_GEOM_NAMES,
    GRIPPER_GUARD_GEOM_NAMES,
    GRIPPER_JOINT_LIMIT_RAD,
    GRIPPER_JOINT_NAMES,
    GRIPPER_PAD_GEOM_NAMES,
    JOINT_LIMITS,
    JOINT_NAMES,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_GENERATED_DIR = REPO_ROOT / ".generated" / "mujoco"
DEFAULT_ROBOT_DESCRIPTIONS_CACHE = REPO_ROOT / ".robot_descriptions_cache"
DEFAULT_END_EFFECTOR = END_EFFECTOR_XARM_GRIPPER
GENERATED_MODEL_VERSION = "3"
MUJOCO_ASSET_ROOT = Path(__file__).resolve().parent / "assets"
LOCAL_XARM_ROS2_PATH = MUJOCO_ASSET_ROOT / "xarm_ros2"
LOCAL_XARM_DESCRIPTION_PATH = LOCAL_XARM_ROS2_PATH / "xarm_description"
ARM_BODY_NAMES = ("link1", "link2", "link3", "link4", "link5", "link6")
CAMERA_NAMES = ("rear_side", "overview", "front", "side", "top")
DEFAULT_CAMERA = "rear_side"
Vector3 = Tuple[float, float, float]

CAMERA_SPECS: Mapping[str, Tuple[Vector3, Vector3, Vector3, float]] = {
    "rear_side": ((-0.65, -1.35, 1.22), (0.42, 0.00, 0.88), (0.0, 0.0, 1.0), 48.0),
    "overview": ((1.20, -1.05, 1.35), (0.40, 0.00, 0.92), (0.0, 0.0, 1.0), 42.0),
    "front": ((1.20, 0.00, 1.05), (0.40, 0.00, 0.92), (0.0, 0.0, 1.0), 38.0),
    "side": ((0.45, -1.25, 0.98), (0.45, 0.00, 0.90), (0.0, 0.0, 1.0), 38.0),
    "top": ((0.45, 0.00, 1.80), (0.45, 0.00, 0.72), (1.0, 0.0, 0.0), 34.0),
}

# The table/object friction scale follows the tuned MuJoCo xArm task scene in
# MingqianW/embodied-ai-xarm (commit 42bf191). Gripper contacts are tuned by the
# local manipulation regressions, with stiffer pads and a low-friction guard.
SCENE_COLLISION_BIT = 1
GRIPPER_PAD_COLLISION_BIT = 2
GRIPPER_GUARD_COLLISION_BIT = 4
MANIPULATION_OBJECT_COLLISION_MASK = (
    SCENE_COLLISION_BIT | GRIPPER_PAD_COLLISION_BIT | GRIPPER_GUARD_COLLISION_BIT
)
TABLE_FRICTION = "1.0 0.01 0.001"
CUBE_FRICTION = "1.2 0.01 0.001"
FINGER_PAD_FRICTION = "2.0 0.005 0.0005"
FINGER_PAD_SOLIMP = "0.95 0.99 0.001"
FINGER_PAD_SOLREF = "0.003 1"
GRIPPER_GUARD_FRICTION = "0.2 0.001 0.0001"
GRIPPER_GUARD_SOLIMP = "0.9 0.95 0.001"
GRIPPER_GUARD_SOLREF = "0.01 1"
GRIPPER_ACTUATOR_KP = "20"
GRIPPER_ACTUATOR_KV = "3"
GRIPPER_ACTUATOR_FORCE_RANGE = "-4 4"

XARM6_XACRO_ARGS = {
    "add_gripper": "false",
    "add_vacuum_gripper": "false",
    "add_bio_gripper": "false",
    "add_realsense_d435i": "false",
    "add_other_geometry": "false",
    "limited": "true",
    "mesh_suffix": "stl",
}


class LocalXarm6Description:
    REPOSITORY_PATH = str(LOCAL_XARM_ROS2_PATH)
    PACKAGE_PATH = str(LOCAL_XARM_DESCRIPTION_PATH)
    XACRO_PATH = str(LOCAL_XARM_DESCRIPTION_PATH / "urdf" / "xarm_device.urdf.xacro")
    XACRO_ARGS = {
        "dof": "6",
        "robot_type": "xarm",
    }


def default_model_path(end_effector: str = DEFAULT_END_EFFECTOR) -> Path:
    return ensure_official_xarm6_table_cube_mjcf(end_effector=end_effector)


def official_xarm6_urdf_path(end_effector: str = DEFAULT_END_EFFECTOR) -> Path:
    """Render the official xArm6 xacro/URDF source into a concrete URDF."""

    _configure_robot_descriptions_cache()
    try:
        from robot_descriptions._xacro import get_urdf_path
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "robot_descriptions and xacrodoc are required. "
            "Run: uv pip install -e '.[sim]'"
        ) from exc

    description = _official_xarm6_description()
    return Path(
        get_urdf_path(description, _xacro_args_for_end_effector(end_effector))
    ).resolve()


def ensure_official_xarm6_table_cube_mjcf(
    output_path: Optional[Path] = None,
    end_effector: str = DEFAULT_END_EFFECTOR,
    force: bool = False,
) -> Path:
    """Generate a controllable MuJoCo scene from the official xArm6 URDF source."""

    end_effector = _normalize_end_effector(end_effector)
    output_path = (
        Path(output_path)
        if output_path
        else DEFAULT_GENERATED_DIR / _default_model_name(end_effector)
    )
    if output_path.exists() and not force:
        try:
            _validate_mjcf(output_path, end_effector)
            return output_path
        except (RuntimeError, ValueError):
            pass

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with _generation_lock(output_path):
        if output_path.exists() and not force:
            try:
                _validate_mjcf(output_path, end_effector)
                return output_path
            except (RuntimeError, ValueError):
                pass

        urdf_path = official_xarm6_urdf_path(end_effector=end_effector)
        normalized_urdf_path = output_path.with_suffix(".normalized.urdf")
        raw_mjcf_path = output_path.with_suffix(".raw.xml")

        _write_normalized_official_urdf(urdf_path, normalized_urdf_path)
        _save_mjcf_from_urdf(normalized_urdf_path, raw_mjcf_path)
        _write_table_cube_scene(raw_mjcf_path, output_path, end_effector)
        _validate_mjcf(output_path, end_effector)
    return output_path


@contextmanager
def _generation_lock(output_path: Path):
    try:
        import fcntl
    except ImportError:
        yield
        return

    lock_path = output_path.with_suffix(f"{output_path.suffix}.lock")
    with lock_path.open("a+", encoding="utf-8") as lock_file:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        try:
            yield
        finally:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _default_model_name(end_effector: str) -> str:
    suffix = "bare" if end_effector == END_EFFECTOR_NONE else end_effector
    return f"xarm6_table_cube.{suffix}.official_derived.xml"


def _normalize_end_effector(end_effector: str) -> str:
    value = str(end_effector or END_EFFECTOR_NONE)
    if value not in END_EFFECTOR_NAMES:
        names = ", ".join(END_EFFECTOR_NAMES)
        raise RuntimeError(
            f"Unsupported MuJoCo end effector: {value}. Use one of: {names}"
        )
    return value


def _xacro_args_for_end_effector(end_effector: str) -> dict:
    end_effector = _normalize_end_effector(end_effector)
    args = dict(XARM6_XACRO_ARGS)
    if end_effector == END_EFFECTOR_XARM_GRIPPER:
        args["add_gripper"] = "true"
    return args


def _configure_robot_descriptions_cache() -> None:
    os.environ.setdefault(
        "ROBOT_DESCRIPTIONS_CACHE",
        str(DEFAULT_ROBOT_DESCRIPTIONS_CACHE),
    )
    Path(os.environ["ROBOT_DESCRIPTIONS_CACHE"]).mkdir(parents=True, exist_ok=True)


def _official_xarm6_description():
    if Path(LocalXarm6Description.XACRO_PATH).exists():
        return LocalXarm6Description

    try:
        import robot_descriptions.xarm6_description as xarm6_description
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "robot_descriptions is required to fetch the official xArm6 model. "
            "Run: uv pip install -e '.[sim]'"
        ) from exc
    return xarm6_description


def _write_normalized_official_urdf(source_path: Path, output_path: Path) -> None:
    tree = ET.parse(source_path)
    root = tree.getroot()

    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if filename and filename.startswith("file://"):
            mesh.set("filename", filename[len("file://") :])

    mujoco = root.find("mujoco")
    if mujoco is None:
        mujoco = ET.Element("mujoco")
        root.insert(0, mujoco)
    compiler = mujoco.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco, "compiler")
    compiler.set("discardvisual", "false")
    compiler.set("fusestatic", "false")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    _indent(tree)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def _save_mjcf_from_urdf(urdf_path: Path, output_path: Path) -> None:
    mujoco = _require_mujoco()
    model = mujoco.MjModel.from_xml_path(str(urdf_path))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveLastXML(str(output_path), model)


def _write_table_cube_scene(
    source_path: Path, output_path: Path, end_effector: str
) -> None:
    tree = ET.parse(source_path)
    root = tree.getroot()
    root.set("model", f"xarm6_table_cube_{end_effector}_official_derived")
    _set_generated_model_version(root)
    _configure_physics_options(root)

    worldbody = _required(root.find("worldbody"), "worldbody")
    link_base = _required(root.find(".//body[@name='link_base']"), "body link_base")
    link_base.set("pos", "0 0 0.72")

    # xArm's real servo stack is not an unpowered arm, so the tabletop MuJoCo
    # adapter uses gravity compensation before applying position targets.
    for body_name in ("link_base", *ARM_BODY_NAMES):
        body = _required(
            root.find(f".//body[@name='{body_name}']"), f"body {body_name}"
        )
        body.set("gravcomp", "1")

    if end_effector == END_EFFECTOR_XARM_GRIPPER:
        for body_name in GRIPPER_BODY_NAMES:
            body = root.find(f".//body[@name='{body_name}']")
            if body is not None:
                body.set("gravcomp", "1")
        _tune_gripper_joints(root)
        _configure_gripper_mesh_collisions(root)
        _add_gripper_guard_collisions(root)
        _add_gripper_pad_collisions(root)
        _add_gripper_mimic_equalities(root)

    site_parent_name = (
        "xarm_gripper_base_link"
        if end_effector == END_EFFECTOR_XARM_GRIPPER
        else "link_eef"
    )
    site_pos = "0 0 0.112" if end_effector == END_EFFECTOR_XARM_GRIPPER else "0 0 0"
    site_size = "0.002" if end_effector == END_EFFECTOR_XARM_GRIPPER else "0.015"
    site_parent = root.find(f".//body[@name='{site_parent_name}']")
    if site_parent is None:
        site_parent = _required(
            root.find(".//body[@name='link_eef']"), "body link_eef"
        )
    for body in root.findall(".//body"):
        _remove_children_by_name(body, "site", "eef")
    ET.SubElement(
        site_parent,
        "site",
        {
            "name": "eef",
            "pos": site_pos,
            "size": site_size,
            "rgba": "0.1 0.8 0.2 0",
        },
    )

    _remove_children_by_name(worldbody, "geom", "floor")
    _remove_children_by_name(worldbody, "geom", "table")
    _remove_children_by_name(worldbody, "body", "cube")
    for camera_name in CAMERA_NAMES:
        _remove_children_by_name(worldbody, "camera", camera_name)

    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "floor",
            "type": "plane",
            "pos": "0 0 0",
            "size": "2.0 2.0 0.05",
            "rgba": "0.78 0.80 0.83 1",
            "contype": str(SCENE_COLLISION_BIT),
            "conaffinity": str(SCENE_COLLISION_BIT),
        },
    )
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "table",
            "type": "box",
            "pos": "0.45 0 0.36",
            "size": "0.55 0.45 0.36",
            "rgba": "0.52 0.48 0.42 1",
            "friction": TABLE_FRICTION,
            "condim": "3",
            "priority": "2",
            "solimp": "0.995 0.999 0.0001",
            "solref": "0.0015 1",
            "contype": str(SCENE_COLLISION_BIT),
            "conaffinity": str(SCENE_COLLISION_BIT),
        },
    )
    cube = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "cube",
            "pos": "0.45 0 0.765",
        },
    )
    ET.SubElement(cube, "joint", {"name": "cube_freejoint", "type": "free"})
    ET.SubElement(
        cube,
        "geom",
        {
            "name": "cube_geom",
            "type": "box",
            "size": "0.025 0.025 0.025",
            "mass": "0.05",
            "rgba": "0.90 0.24 0.14 1",
            "friction": CUBE_FRICTION,
            "condim": "3",
            "priority": "1",
            "solimp": "0.995 0.999 0.0001",
            "solref": "0.0015 1",
            "contype": str(SCENE_COLLISION_BIT),
            "conaffinity": str(MANIPULATION_OBJECT_COLLISION_MASK),
        },
    )

    for camera_name in CAMERA_NAMES:
        pos, target, up, fovy = CAMERA_SPECS[camera_name]
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": camera_name,
                "pos": _format_vec(pos),
                "xyaxes": _format_vec(_lookat_xyaxes(pos, target, up)),
                "fovy": f"{fovy:.1f}",
            },
        )

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    for joint_name in JOINT_NAMES:
        _remove_children_by_name(actuator, "position", f"{joint_name}_pos")
    for joint_name, (lower, upper) in zip(
        JOINT_NAMES, JOINT_LIMITS, strict=True
    ):
        ET.SubElement(
            actuator,
            "position",
            {
                "name": f"{joint_name}_pos",
                "joint": joint_name,
                "kp": "80",
                "kv": "8",
                "ctrlrange": f"{lower:.6f} {upper:.6f}",
                "forcerange": "-80 80",
            },
        )
    for actuator_name in GRIPPER_ACTUATOR_NAMES:
        _remove_children_by_name(actuator, "position", actuator_name)
    if end_effector == END_EFFECTOR_XARM_GRIPPER:
        lower, upper = GRIPPER_JOINT_LIMIT_RAD
        ET.SubElement(
            actuator,
            "position",
            {
                "name": GRIPPER_ACTUATOR_NAMES[0],
                "joint": GRIPPER_DRIVE_JOINT_NAME,
                "kp": GRIPPER_ACTUATOR_KP,
                "kv": GRIPPER_ACTUATOR_KV,
                "ctrlrange": f"{lower:.6f} {upper:.6f}",
                "forcerange": GRIPPER_ACTUATOR_FORCE_RANGE,
            },
        )

    _indent(tree)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def _validate_mjcf(path: Path, end_effector: str) -> None:
    xml_root = ET.parse(path).getroot()
    version = xml_root.find(
        ".//custom/numeric[@name='spacemouse_teleop_model_version']"
    )
    if version is None or version.attrib.get("data") != GENERATED_MODEL_VERSION:
        raise RuntimeError("Generated MuJoCo model has an outdated build version")

    mujoco = _require_mujoco()
    model = mujoco.MjModel.from_xml_path(str(path))
    if model.opt.timestep > 0.0011 or model.opt.noslip_iterations < 12:
        raise RuntimeError("Generated MuJoCo model has outdated physics options")

    expected_actuators = {f"{name}_pos" for name in JOINT_NAMES}
    if end_effector == END_EFFECTOR_XARM_GRIPPER:
        expected_actuators.add("gripper_pos")
    actual_actuators = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for i in range(model.nu)
    }
    missing = expected_actuators - actual_actuators
    if missing:
        raise RuntimeError(
            f"Generated MuJoCo model is missing actuators: {sorted(missing)}"
        )

    for geom_name in ("table", "cube_geom"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        if geom_id < 0:
            raise RuntimeError(f"Generated MuJoCo model is missing geom: {geom_name}")
        if int(model.geom_condim[geom_id]) != 3:
            raise RuntimeError(
                f"Generated MuJoCo model has outdated contact dim on {geom_name}"
            )
    cube_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom"
    )
    if int(model.geom_contype[cube_geom_id]) != SCENE_COLLISION_BIT or int(
        model.geom_conaffinity[cube_geom_id]
    ) != MANIPULATION_OBJECT_COLLISION_MASK:
        raise RuntimeError("Generated MuJoCo model has outdated cube contact mask")

    if end_effector == END_EFFECTOR_XARM_GRIPPER:
        missing_gripper_actuators = set(GRIPPER_ACTUATOR_NAMES) - actual_actuators
        if missing_gripper_actuators:
            raise RuntimeError(
                "Generated MuJoCo model is missing xArm gripper actuators: "
                f"{sorted(missing_gripper_actuators)}"
            )
        gripper_actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, GRIPPER_ACTUATOR_NAMES[0]
        )
        expected_force_range = tuple(
            float(value) for value in GRIPPER_ACTUATOR_FORCE_RANGE.split()
        )
        actual_force_range = tuple(
            float(value)
            for value in model.actuator_forcerange[gripper_actuator_id]
        )
        if any(
            abs(actual - expected) > 1e-9
            for actual, expected in zip(
                actual_force_range, expected_force_range, strict=True
            )
        ):
            raise RuntimeError(
                "Generated MuJoCo model has outdated gripper actuator force range"
            )
        missing_gripper_joints = []
        for joint_name in GRIPPER_JOINT_NAMES:
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name) < 0:
                missing_gripper_joints.append(joint_name)
        if missing_gripper_joints:
            raise RuntimeError(
                "Generated MuJoCo model is missing xArm gripper joints: "
                f"{missing_gripper_joints}"
            )
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_tcp") < 0:
            raise RuntimeError("Generated MuJoCo gripper model is missing link_tcp")
        if model.neq < len(GRIPPER_JOINT_NAMES) - 1:
            raise RuntimeError(
                "Generated MuJoCo gripper mimic equalities are incomplete"
            )
        missing_pad_geoms = []
        for geom_name in GRIPPER_PAD_GEOM_NAMES:
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name) < 0:
                missing_pad_geoms.append(geom_name)
        if missing_pad_geoms:
            raise RuntimeError(
                "Generated MuJoCo model is missing gripper pad collision geoms: "
                f"{missing_pad_geoms}"
            )
        missing_guard_geoms = []
        for geom_name in GRIPPER_GUARD_GEOM_NAMES:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            if geom_id < 0:
                missing_guard_geoms.append(geom_name)
                continue
            if int(model.geom_contype[geom_id]) != GRIPPER_GUARD_COLLISION_BIT or int(
                model.geom_conaffinity[geom_id]
            ) != 0:
                raise RuntimeError(
                    "Generated MuJoCo model has outdated gripper guard mask on "
                    f"{geom_name}"
                )
        if missing_guard_geoms:
            raise RuntimeError(
                "Generated MuJoCo model is missing gripper guard collision geoms: "
                f"{missing_guard_geoms}"
            )
        palm_geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, "gripper_palm_collision"
        )
        expected_palm_pos = (0.0, 0.0, 0.055)
        expected_palm_size = (0.040, 0.050)
        actual_palm_pos = tuple(float(value) for value in model.geom_pos[palm_geom_id])
        actual_palm_size = tuple(
            float(value) for value in model.geom_size[palm_geom_id][:2]
        )
        if any(
            abs(actual - expected) > 1e-9
            for actual, expected in zip(
                actual_palm_pos, expected_palm_pos, strict=True
            )
        ) or any(
            abs(actual - expected) > 1e-9
            for actual, expected in zip(
                actual_palm_size, expected_palm_size, strict=True
            )
        ):
            raise RuntimeError("Generated MuJoCo model has outdated palm guard shape")
        missing_finger_mesh_geoms = []
        for geom_name in GRIPPER_FINGER_MESH_COLLISION_GEOM_NAMES:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            if geom_id < 0:
                missing_finger_mesh_geoms.append(geom_name)
            elif int(model.geom_contype[geom_id]) != 0 or int(
                model.geom_conaffinity[geom_id]
            ) != 0:
                raise RuntimeError(
                    "Generated MuJoCo model has active finger mesh collision on "
                    f"{geom_name}"
                )
        if missing_finger_mesh_geoms:
            raise RuntimeError(
                "Generated MuJoCo model is missing named gripper finger mesh "
                f"geoms: {missing_finger_mesh_geoms}"
            )
        for geom_name in GRIPPER_PAD_GEOM_NAMES:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            if geom_id >= 0 and int(model.geom_condim[geom_id]) != 4:
                raise RuntimeError(
                    "Generated MuJoCo model has outdated contact dim on "
                    f"{geom_name}"
                )

    missing_gravcomp = []
    for body_name in ARM_BODY_NAMES:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0 or model.body_gravcomp[body_id] < 0.99:
            missing_gravcomp.append(body_name)
    if missing_gravcomp:
        raise RuntimeError(
            "Generated MuJoCo model is missing arm gravity compensation: "
            f"{missing_gravcomp}"
        )

    if end_effector == END_EFFECTOR_XARM_GRIPPER:
        missing_gripper_gravcomp = []
        for body_name in GRIPPER_BODY_NAMES:
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            if body_id < 0 or model.body_gravcomp[body_id] < 0.99:
                missing_gripper_gravcomp.append(body_name)
        if missing_gripper_gravcomp:
            raise RuntimeError(
                "Generated MuJoCo model is missing gripper gravity compensation: "
                f"{missing_gripper_gravcomp}"
            )

    actual_cameras = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
        for i in range(model.ncam)
    }
    missing_cameras = set(CAMERA_NAMES) - actual_cameras
    if missing_cameras:
        raise RuntimeError(
            f"Generated MuJoCo model is missing cameras: {sorted(missing_cameras)}"
        )


def _required(value, label: str):
    if value is None:
        raise RuntimeError(f"Generated MuJoCo model is missing {label}")
    return value


def _remove_children_by_name(parent: ET.Element, tag: str, name: str) -> None:
    for child in list(parent):
        if child.tag == tag and child.attrib.get("name") == name:
            parent.remove(child)


def _configure_physics_options(root: ET.Element) -> None:
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        compiler = root.find("compiler")
        insert_index = 0
        if compiler is not None:
            insert_index = list(root).index(compiler) + 1
        root.insert(insert_index, option)
    option.set("timestep", "0.001")
    option.set("solver", "Newton")
    option.set("iterations", "80")
    option.set("noslip_iterations", "12")
    option.set("integrator", "implicitfast")
    option.set("cone", "elliptic")


def _configure_gripper_mesh_collisions(root: ET.Element) -> None:
    for body_name in GRIPPER_BODY_NAMES:
        body = root.find(f".//body[@name='{body_name}']")
        if body is None:
            continue
        for geom in body.findall("geom"):
            if geom.attrib.get("type") != "mesh":
                continue
            geom.set("contype", "0")
            geom.set("conaffinity", "0")

    for body_name, geom_name in (
        ("left_finger", "left_finger_mesh_collision"),
        ("right_finger", "right_finger_mesh_collision"),
    ):
        body = _required(
            root.find(f".//body[@name='{body_name}']"), f"body {body_name}"
        )
        configured = False
        for geom in body.findall("geom"):
            if geom.attrib.get("type") != "mesh" or geom.attrib.get("group") == "1":
                continue
            geom.set("name", geom_name)
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            configured = True
            break
        if not configured:
            raise RuntimeError(
                f"Generated MuJoCo model is missing collision mesh for {body_name}"
            )


def _tune_gripper_joints(root: ET.Element) -> None:
    for joint_name in GRIPPER_JOINT_NAMES:
        joint = _required(
            root.find(f".//joint[@name='{joint_name}']"), f"joint {joint_name}"
        )
        joint.set("damping", "0.08")
        joint.set("armature", "0.0008")


def _add_gripper_guard_collisions(root: ET.Element) -> None:
    specs = (
        (
            "xarm_gripper_base_link",
            "gripper_palm_collision",
            "cylinder",
            {"pos": "0 0 0.055", "size": "0.040 0.050"},
        ),
        (
            "left_outer_knuckle",
            "left_outer_knuckle_guard_collision",
            "capsule",
            {"fromto": "0 0.007 -0.014 0 0.033 0.055", "size": "0.007"},
        ),
        (
            "right_outer_knuckle",
            "right_outer_knuckle_guard_collision",
            "capsule",
            {"fromto": "0 -0.007 -0.014 0 -0.033 0.055", "size": "0.007"},
        ),
        (
            "left_finger",
            "left_finger_guard_collision",
            "box",
            {"pos": "0 -0.006 0.027", "size": "0.012 0.006 0.034"},
        ),
        (
            "right_finger",
            "right_finger_guard_collision",
            "box",
            {"pos": "0 0.006 0.027", "size": "0.012 0.006 0.034"},
        ),
    )
    for body_name, geom_name, geom_type, shape in specs:
        body = _required(
            root.find(f".//body[@name='{body_name}']"), f"body {body_name}"
        )
        _remove_children_by_name(body, "geom", geom_name)
        attributes = {
            "name": geom_name,
            "type": geom_type,
            "density": "0",
            "rgba": "0.15 0.55 0.85 0",
            "contype": str(GRIPPER_GUARD_COLLISION_BIT),
            "conaffinity": "0",
            "friction": GRIPPER_GUARD_FRICTION,
            "condim": "3",
            "priority": "2",
            "solimp": GRIPPER_GUARD_SOLIMP,
            "solref": GRIPPER_GUARD_SOLREF,
            "margin": "0",
        }
        attributes.update(shape)
        ET.SubElement(body, "geom", attributes)


def _set_generated_model_version(root: ET.Element) -> None:
    custom = root.find("custom")
    if custom is None:
        custom = ET.Element("custom")
        asset = root.find("asset")
        insert_at = list(root).index(asset) if asset is not None else len(root)
        root.insert(insert_at, custom)
    _remove_children_by_name(custom, "numeric", "spacemouse_teleop_model_version")
    ET.SubElement(
        custom,
        "numeric",
        {
            "name": "spacemouse_teleop_model_version",
            "data": GENERATED_MODEL_VERSION,
        },
    )


def _add_gripper_pad_collisions(root: ET.Element) -> None:
    specs = (
        ("left_finger", "left_finger_pad_collision", "0 -0.020 0"),
        ("right_finger", "right_finger_pad_collision", "0 0.020 0"),
    )
    for body_name, geom_name, pos in specs:
        body = _required(
            root.find(f".//body[@name='{body_name}']"), f"body {body_name}"
        )
        _remove_children_by_name(body, "geom", geom_name)
        ET.SubElement(
            body,
            "geom",
            {
                "name": geom_name,
                "type": "box",
                "pos": pos,
                "size": "0.017 0.006 0.022",
                "density": "0",
                "rgba": "0.05 0.65 0.20 0",
                "contype": str(GRIPPER_PAD_COLLISION_BIT),
                "conaffinity": "0",
                "friction": FINGER_PAD_FRICTION,
                "condim": "4",
                "priority": "3",
                "solimp": FINGER_PAD_SOLIMP,
                "solref": FINGER_PAD_SOLREF,
                "margin": "0.001",
            },
        )


def _add_gripper_mimic_equalities(root: ET.Element) -> None:
    equality = root.find("equality")
    if equality is None:
        equality = ET.Element("equality")
        actuator = root.find("actuator")
        if actuator is None:
            root.append(equality)
        else:
            root.insert(list(root).index(actuator), equality)

    for joint_name in GRIPPER_JOINT_NAMES[1:]:
        equality_name = f"{joint_name}_mimic"
        _remove_children_by_name(equality, "joint", equality_name)
        ET.SubElement(
            equality,
            "joint",
            {
                "name": equality_name,
                "joint1": joint_name,
                "joint2": GRIPPER_DRIVE_JOINT_NAME,
                "polycoef": "0 1 0 0 0",
                "solref": "0.004 1",
                "solimp": "0.95 0.99 0.001",
            },
        )


def _lookat_xyaxes(pos: Vector3, target: Vector3, up: Vector3) -> Tuple[float, ...]:
    z_axis = _normalize(_subtract(pos, target))
    x_axis_raw = _cross(up, z_axis)
    if _norm(x_axis_raw) < 1e-6:
        x_axis_raw = _cross((0.0, 1.0, 0.0), z_axis)
    x_axis = _normalize(x_axis_raw)
    y_axis = _normalize(_cross(z_axis, x_axis))
    return (*x_axis, *y_axis)


def _subtract(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(vector: Sequence[float]) -> Vector3:
    norm = _norm(vector)
    if norm <= 1e-12:
        return (1.0, 0.0, 0.0)
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def _norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _format_vec(values: Sequence[float]) -> str:
    return " ".join(f"{value:.6g}" for value in values)


def _indent(tree: ET.ElementTree) -> None:
    try:
        ET.indent(tree, space="  ")
    except AttributeError:
        pass


def _require_mujoco():
    try:
        import mujoco
    except ImportError as exc:
        raise RuntimeError(
            "MuJoCo is not installed. Run: uv pip install -e '.[sim]'"
        ) from exc
    return mujoco
