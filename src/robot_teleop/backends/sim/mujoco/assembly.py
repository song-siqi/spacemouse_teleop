"""Assemble upstream mechanisms using MuJoCo's structured attachment API."""
from pathlib import Path
from .bindings import BINDINGS
from .model_source import default_model_path


def build_model(robot="xarm6", tool="xarm_gripper", *, object_name=None, object_shape="box"):
    import mujoco
    binding = BINDINGS[(robot, tool)]
    if object_shape not in ("box", "cylinder"):
        raise ValueError("Unsupported object shape")
    if robot == "xarm6":
        spec = mujoco.MjSpec.from_file(str(default_model_path()))
        if object_name:
            spec.body("cube").name = object_name
            spec.geom("cube_geom").name = object_name + "_geom"
        if object_shape == "cylinder":
            spec.geom((object_name + "_geom") if object_name else "cube_geom").type = mujoco.mjtGeom.mjGEOM_CYLINDER
        model = spec.compile()
        for name in ("table", "floor"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if gid >= 0:
                model.geom_conaffinity[gid] = 7
        for gid in range(model.ngeom):
            if model.geom_contype[gid] == 4:
                model.geom_solref[gid] = (.002, 1.)
                model.geom_solimp[gid] = (.95, .99, .001, .5, 2.)
        return model, binding
    assets = Path(__file__).parent / "assets" / "menagerie"
    spec = mujoco.MjSpec.from_file(str(assets / "franka_fr3" / "fr3.xml"))
    gripper = mujoco.MjSpec.from_file(str(assets / "robotiq_2f85" / "2f85.xml"))
    for key in list(spec.keys):
        spec.delete(key)
    # Preserve each upstream pad prism's outer boundary, but do not assign the
    # inward rubber material to its entire backing volume. Both parts are massless
    # collision proxies; the upstream body inertia is unchanged.
    for side in ("left", "right"):
        for number in (1, 2):
            pad = gripper.geom(f"{side}_pad{number}")
            pad.pos[1], pad.size[1] = -.0051, .0015
            gripper.body(f"{side}_pad").add_geom(
                name=f"{side}_pad_back{number}", type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=(0., -.0011, float(pad.pos[2])), size=(.011, .0025, .009375),
                mass=0., contype=4, conaffinity=0, group=3, rgba=(0.,0.,0.,0.))
    for index, geom in enumerate(gripper.geoms):
        if geom.contype == 0 and geom.conaffinity == 0:
            continue
        pad = geom.name in ("left_pad1", "left_pad2", "right_pad1", "right_pad2")
        if not geom.name:
            geom.name = f"shell_{index}"
        geom.contype, geom.conaffinity = (2 if pad else 4), 0
        geom.condim, geom.priority = (4, 3) if pad else (3, 2)
        geom.friction = (2., .005, .0005) if pad else (.2, .001, .0001)
        geom.solref = (.003, 1.) if pad else (.006, 1.)
        geom.solimp = (.95, .99, .001, .5, 2.) if pad else (.9, .95, .001, .5, 2.)
        geom.margin = .001 if pad else 0.
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10
    spec.attach(gripper, prefix="tool_", site=spec.site("attachment_site"))
    spec.body("base").pos = (0, 0, .72)
    for body in spec.bodies:
        body.gravcomp = 1.
    spec.add_exclude(bodyname1="fr3_link7", bodyname2="tool_base_mount")
    spec.add_exclude(bodyname1="fr3_link7", bodyname2="tool_base")
    spec.option.timestep = .001
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.iterations = 80
    spec.option.noslip_iterations = 12
    world = spec.worldbody
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=(3, 3, .1),
                   contype=1, conaffinity=7, rgba=(.25, .28, .3, 1))
    world.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, pos=(.4, 0, .69),
                   size=(.65, .6, .03), contype=1, conaffinity=7, friction=(1., .01, .001), rgba=(.65, .65, .65, 1))
    obj = world.add_body(name=object_name or "object", pos=(.5, 0, .765))
    obj.add_freejoint(name="object_free")
    obj.add_geom(name=(object_name or "object")+"_geom", type=mujoco.mjtGeom.mjGEOM_BOX if object_shape == "box" else mujoco.mjtGeom.mjGEOM_CYLINDER, size=(.025, .025, .025),
                 mass=.05, contype=1, conaffinity=7, friction=(1.2, .01, .001), rgba=(.8, .15, .1, 1))
    world.add_light(pos=(0, -1, 3), dir=(0, 0, -1))
    world.add_camera(name="overview", pos=(1.5, -1.5, 1.8), xyaxes=(.707, .707, 0, -.3, .3, .9))
    world.add_camera(name="side", pos=(.5, -1.8, 1.2), xyaxes=(1, 0, 0, 0, .25, .97))
    return spec.compile(), binding
