import importlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from robot_teleop.core import CartesianTwist, GripperCommand, Stamp, JointPositionTarget


class ContractsTests(unittest.TestCase):
    def test_clocks_and_validation(self):
        with self.assertRaises(ValueError):
            Stamp(1, "simulation").remaining(Stamp(0, "monotonic"))
        with self.assertRaises(ValueError):
            Stamp(1, "simulation", 1).remaining(Stamp(0, "simulation", 0))
        with self.assertRaises(ValueError):
            CartesianTwist((float("nan"), 0, 0))
        with self.assertRaises(ValueError):
            GripperCommand("position", 2)

    def test_core_import_is_engine_free(self):
        subprocess.run([sys.executable, "-c", "import robot_teleop.core, sys; assert 'mujoco' not in sys.modules; assert 'xarm' not in sys.modules"], check=True)


class DynamicsTests(unittest.TestCase):
    def backend(self, robot, hz=60):
        from robot_teleop.backends.sim.mujoco.backend import MujocoBackend
        b = MujocoBackend(robot, "xarm_gripper" if robot == "xarm6" else "robotiq_2f85", hz)
        self.addCleanup(b.close)
        return b

    def test_bindings_clocks_reset_and_pause(self):
        for robot in ("xarm6", "fr3"):
            for hz in (30, 60, 120):
                with self.subTest(robot=robot, hz=hz):
                    b = self.backend(robot, hz)
                    self.assertEqual(len(b.observe().joint_names), 6 if robot == "xarm6" else 7)
                    for _ in range(hz):
                        b.advance()
                    self.assertAlmostEqual(b.data.time, 1., places=8)
                    b.paused = True
                    b.advance()
                    self.assertAlmostEqual(b.data.time, 1.)
                    b.reset()
                    with self.assertRaises(ValueError):
                        b.submit(CartesianTwist(), Stamp(3, "simulation", 0))

    def test_dynamics_has_no_pose_writes_outside_reset(self):
        import ast
        import inspect
        from robot_teleop.backends.sim.mujoco.backend import MujocoBackend
        tree = ast.parse(inspect.getsource(MujocoBackend))
        for function in tree.body[0].body:
            if not isinstance(function,ast.FunctionDef) or function.name == "reset":
                continue
            for node in ast.walk(function):
                if isinstance(node,ast.Subscript) and isinstance(node.ctx,ast.Store):
                    self.assertNotIn(ast.unparse(node.value), ("self.data.qpos","self.data.qvel"),function.name)

    def test_joint_limit_rejection_and_observed_hold(self):
        for robot in ("xarm6", "fr3"):
            b = self.backend(robot)
            deadline = Stamp(10, "simulation", b.epoch)
            with self.assertRaises(ValueError):
                b.submit(JointPositionTarget(b.binding.joints, (100.,)*len(b.binding.joints)), deadline)
            b.submit(GripperCommand("close"), deadline)
            for _ in range(20):
                b.advance()
            observed = b.observe().gripper.normalized_position
            b.submit(GripperCommand("hold"), deadline)
            lo, hi = b.binding.gripper.control_range
            self.assertAlmostEqual(float(b.data.ctrl[b.tool_actuators][0]), lo+observed*(hi-lo))

    def test_independent_camera_schedule(self):
        import numpy as np
        from robot_teleop.runtime.simulation import run
        from robot_teleop.inputs.scripted import ScriptedInput
        b = self.backend("xarm6",30)
        b.render = lambda *args:np.zeros((24,32,3),np.uint8)
        class Sink:
            def __init__(self): self.samples = {}
            def publish(self,source,sequence,stamp,values,**kwargs):
                self.samples.setdefault(source,[]).append((sequence,stamp))
        sink = Sink()
        cameras = [{"id":str(rate),"fps":rate,"width":32,"height":24} for rate in (15,20,60)]
        run(b,ScriptedInput([]),1.,recorder=sink,cameras=cameras)
        self.assertEqual({k:len(v) for k,v in sink.samples.items()}, {"robot":30,"commands":30,"15":15,"20":20,"60":60})
        self.assertAlmostEqual(b.data.time,1.)

    def test_both_dynamics_grasps(self):
        from robot_teleop.backends.sim.mujoco.diagnostics import grasp_probe
        for robot in ("xarm6", "fr3"):
            for hz in (30, 60, 120):
                with self.subTest(robot=robot, hz=hz):
                    summary, _ = grasp_probe(self.backend(robot, hz))
                    self.assertTrue(summary["passed_grasp"], summary)

    def test_shell_collision_matrix(self):
        from robot_teleop.backends.sim.mujoco.backend import MujocoBackend
        from robot_teleop.backends.sim.mujoco.assembly import build_model
        from robot_teleop.backends.sim.mujoco.diagnostics import collision_probe
        for robot in ("xarm6", "fr3"):
            for hz, shape, tilt in ((30,"box",0.),(60,"box",0.),(120,"box",0.),(60,"cylinder",0.),(60,"box",.1)):
                with self.subTest(robot=robot,hz=hz,shape=shape,tilt=tilt):
                    tool = "xarm_gripper" if robot == "xarm6" else "robotiq_2f85"
                    model,binding = build_model(robot,tool,object_name="workpiece",object_shape=shape)
                    b = MujocoBackend(control_hz=hz,model=model,binding=binding)
                    self.addCleanup(b.close)
                    metrics = collision_probe(b, tilt)
                    top,side = metrics["top"],metrics["side"]
                    self.assertGreater(top["shell_contact_count"],0,metrics)
                    self.assertLessEqual(top["shell_penetration_m"],.0005,metrics)
                    self.assertLessEqual(top["table_penetration_m"],.002,metrics)
                    self.assertGreater(top["retreat_m"],.02,metrics)
                    self.assertEqual(top["retreat_contacts"],0,metrics)
                    self.assertGreaterEqual(side["displacement_m"],.04,metrics)
                    self.assertLessEqual(side["shell_penetration_m"],.0025,metrics)
                    self.assertLessEqual(side["retreat_drag_m"],.005,metrics)
                    self.assertEqual(side["retreat_contacts"],0,metrics)
