#!/usr/bin/env python3
"""Lightweight ROS2 topic simulator for the Onero H1 adapter.

This is a protocol/integration simulator, not a physics simulator. It publishes
all standard ROS topics consumed by :class:`OneroH1Robot`, accepts the adapter's
command topics, and can optionally publish a synthetic homogeneous leader.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CompressedImage, JointState
from std_msgs.msg import Bool, Float64MultiArray, Int32, String, UInt8

LEFT_JOINTS = tuple(f"joint{i}-l" for i in range(1, 8))
RIGHT_JOINTS = tuple(f"joint{i}-r" for i in range(1, 8))
HEAD_JOINTS = ("head_pitch_joint", "head_yaw_joint")
LIFT_JOINT = "lift_joint"

# ROS API names differ from the canonical BeaVR X1/H1 URDF names.
MJ_JOINT_NAMES = {
    **{name: f"l{i}" for i, name in enumerate(LEFT_JOINTS, 1)},
    **{name: f"r{i}" for i, name in enumerate(RIGHT_JOINTS, 1)},
    HEAD_JOINTS[0]: "t02",
    HEAD_JOINTS[1]: "t01",
    LIFT_JOINT: "sj",
}
MJ_GRIPPER_JOINT_NAMES = {
    "left": ("left_fake_gripper_left_joint", "left_fake_gripper_right_joint"),
    "right": ("right_fake_gripper_left_joint", "right_fake_gripper_right_joint"),
}
FAKE_GRIPPER_MAX_OPENING_M = 0.022
# The SDK reports lift height, while the X1 URDF models a downward slide from
# its fully raised pose (sj=0). This offset maps 1.30 m -> 0 and 0.45 m -> -0.85.
URDF_LIFT_HEIGHT_OFFSET_M = 1.30


@dataclass
class SimState:
    left: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    right: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    left_target: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    right_target: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    left_velocity: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    right_velocity: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    # Start fully raised: the protocol reports lift height, and 1.30 m maps
    # to the URDF slide joint's top position (sj=0).
    lift: float = 1.30
    lift_target: float = 1.30
    head_pitch: float = 0.0
    head_yaw: float = 0.0
    gripper_left: float = 0.5
    gripper_right: float = 0.5
    leader_left: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    leader_right: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=np.float64))
    leader_gripper_left: float = 0.5
    leader_gripper_right: float = 0.5
    base_x: float = 0.0
    base_y: float = 0.0
    base_yaw: float = 0.0
    base_vx: float = 0.0
    base_vy: float = 0.0
    base_wz: float = 0.0


class H1TopicSimulator(Node):
    def __init__(
        self,
        fps: float,
        camera_fps: float,
        cameras: bool,
        leader: bool,
        mujoco_model: str | None = None,
        viewer: bool = False,
    ) -> None:
        super().__init__("onero_h1_topic_sim")
        self.fps = max(float(fps), 1.0)
        self.camera_fps = max(float(camera_fps), 0.1)
        self.cameras_enabled = cameras
        self.leader_enabled = leader
        self.state = SimState()
        self._lock = threading.RLock()
        self._last_camera_time = -1.0
        self._leader_phase = 0.0
        self._mj = None
        self._mj_model = None
        self._mj_data = None
        self._mj_viewer = None
        self._mj_joint_qpos: dict[str, int] = {}
        self._mj_joint_qvel: dict[str, int] = {}
        self._mj_actuators: dict[str, int] = {}
        if mujoco_model:
            self._init_mujoco(mujoco_model, viewer)

        self.pub_joint = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_left = self.create_publisher(JointState, "/left_joint_states", 10)
        self.pub_right = self.create_publisher(JointState, "/right_joint_states", 10)
        self.pub_head = self.create_publisher(JointState, "/head/joint_states", 10)
        self.pub_lift = self.create_publisher(JointState, "/lift/joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "/agv/odom", 10)
        self.pub_battery = self.create_publisher(BatteryState, "/battery/state", 10)
        self.pub_bumper = self.create_publisher(Bool, "/front_bumper", 10)
        self.pub_left_gripper = self.create_publisher(UInt8, "/left_gripper_state", 10)
        self.pub_right_gripper = self.create_publisher(UInt8, "/right_gripper_state", 10)
        self.pub_left_pose = self.create_publisher(PoseWithCovarianceStamped, "/left_pose", 10)
        self.pub_right_pose = self.create_publisher(PoseWithCovarianceStamped, "/right_pose", 10)
        self.pub_left_diag = self.create_publisher(DiagnosticStatus, "/left_arm/diagnostics", 10)
        self.pub_right_diag = self.create_publisher(DiagnosticStatus, "/right_arm/diagnostics", 10)

        self.pub_leader_left = self.create_publisher(JointState, "/left/joint_states", 10)
        self.pub_leader_right = self.create_publisher(JointState, "/right/joint_states", 10)
        self.pub_leader_gripper = self.create_publisher(Int32, "/joystick_info", 10)

        self.camera_publishers: dict[str, object] = {}
        if cameras:
            for name, topic in {
                "head": "/head/camera/rgb",
                "left": "/left/camera/rgb",
                "right": "/right/camera/rgb",
            }.items():
                self.camera_publishers[name] = self.create_publisher(
                    CompressedImage, topic, qos_profile_sensor_data
                )

        self.create_subscription(Float64MultiArray, "/record_data", self._on_record_data, 10)
        self.create_subscription(String, "/left_arm/movej", self._on_left_movej, 10)
        self.create_subscription(String, "/right_arm/movej", self._on_right_movej, 10)
        self.create_subscription(JointState, "/lift/joint_states/update", self._on_lift, 10)
        self.create_subscription(JointState, "/head/joint_states/update", self._on_head, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Int32, "/joystick_info", self._on_gripper, 10)

        self.create_timer(1.0 / self.fps, self._tick)
        self.get_logger().info(
            f"H1 topic simulator ready: {self.fps:.1f} Hz, "
            f"cameras={cameras}, synthetic_leader={leader}, "
            f"backend={'mujoco' if self._mj_model is not None else 'kinematic'}"
        )

    def _init_mujoco(self, model_path: str, viewer: bool) -> None:
        import mujoco

        path = Path(model_path).expanduser().resolve()
        self._mj = mujoco
        self._mj_model = mujoco.MjModel.from_xml_path(str(path))
        self._mj_data = mujoco.MjData(self._mj_model)
        for ros_name, model_name in MJ_JOINT_NAMES.items():
            joint_id = mujoco.mj_name2id(
                self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, model_name
            )
            if joint_id < 0:
                raise ValueError(
                    f"MuJoCo model is missing joint {model_name!r} "
                    f"(required for ROS joint {ros_name!r})"
                )
            self._mj_joint_qpos[ros_name] = int(self._mj_model.jnt_qposadr[joint_id])
            self._mj_joint_qvel[ros_name] = int(self._mj_model.jnt_dofadr[joint_id])
        for joint_names in MJ_GRIPPER_JOINT_NAMES.values():
            for model_name in joint_names:
                joint_id = mujoco.mj_name2id(
                    self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, model_name
                )
                if joint_id < 0:
                    raise ValueError(f"MuJoCo model is missing fake gripper joint {model_name!r}")
                self._mj_joint_qpos[model_name] = int(self._mj_model.jnt_qposadr[joint_id])

        self._write_urdf_pose()
        mujoco.mj_forward(self._mj_model, self._mj_data)
        if viewer:
            import mujoco.viewer

            self._mj_viewer = mujoco.viewer.launch_passive(self._mj_model, self._mj_data)
            with self._mj_viewer.lock():
                self._mj_viewer.cam.lookat[:] = np.array([0.0, 0.0, 0.85])
                self._mj_viewer.cam.distance = 2.5
                self._mj_viewer.cam.azimuth = 180.0
                self._mj_viewer.cam.elevation = -5.0

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def _on_record_data(self, msg: Float64MultiArray) -> None:
        values = np.asarray(msg.data, dtype=np.float64)
        if values.size != 28:
            self.get_logger().warning(
                f"Ignoring /record_data with {values.size} values; expected 28"
            )
            return
        with self._lock:
            self.state.left_target = values[0:7].copy()
            self.state.right_target = values[14:21].copy()

    def _movej_target(self, msg: String) -> np.ndarray | None:
        try:
            values = np.asarray(json.loads(msg.data)["joints"], dtype=np.float64)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning("Ignoring malformed MoveJ payload")
            return None
        return values if values.size == 7 else None

    def _on_left_movej(self, msg: String) -> None:
        target = self._movej_target(msg)
        if target is not None:
            with self._lock:
                self.state.left_target = target

    def _on_right_movej(self, msg: String) -> None:
        target = self._movej_target(msg)
        if target is not None:
            with self._lock:
                self.state.right_target = target

    def _on_lift(self, msg: JointState) -> None:
        if msg.position:
            with self._lock:
                self.state.lift_target = float(np.clip(msg.position[0], 0.4, 1.3))

    def _on_head(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position, strict=False))
        with self._lock:
            self.state.head_pitch = float(by_name.get(HEAD_JOINTS[0], self.state.head_pitch))
            self.state.head_yaw = float(by_name.get(HEAD_JOINTS[1], self.state.head_yaw))

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self.state.base_vx = float(msg.linear.x)
            self.state.base_vy = float(msg.linear.y)
            self.state.base_wz = float(msg.angular.z)

    def _on_gripper(self, msg: Int32) -> None:
        with self._lock:
            if 100 <= msg.data < 200:
                self.state.gripper_left = (msg.data - 100) / 100.0
            elif 200 <= msg.data < 300:
                self.state.gripper_right = (msg.data - 200) / 100.0

    def _advance(self, dt: float) -> None:
        s = self.state
        if self.leader_enabled:
            self._leader_phase += dt
            wave = 0.18 * math.sin(0.6 * self._leader_phase)
            s.leader_left = np.array([wave, -wave, wave * 0.7, 0.0, 0.0, 0.0, 0.0])
            s.leader_right = np.array([-wave, wave, -wave * 0.7, 0.0, 0.0, 0.0, 0.0])
            s.leader_gripper_left = 0.5 + 0.4 * math.sin(0.35 * self._leader_phase)
            s.leader_gripper_right = 0.5 + 0.4 * math.cos(0.35 * self._leader_phase)

        if self._mj_model is not None:
            self._advance_mujoco(dt)
            return

        old_left = s.left.copy()
        old_right = s.right.copy()
        max_step = 1.5 * dt
        s.left += np.clip(s.left_target - s.left, -max_step, max_step)
        s.right += np.clip(s.right_target - s.right, -max_step, max_step)
        s.left_velocity = (s.left - old_left) / max(dt, 1e-6)
        s.right_velocity = (s.right - old_right) / max(dt, 1e-6)
        s.lift += float(np.clip(s.lift_target - s.lift, -0.2 * dt, 0.2 * dt))
        s.base_x += s.base_vx * dt
        s.base_y += s.base_vy * dt
        s.base_yaw += s.base_wz * dt

    def _write_urdf_pose(self) -> None:
        """Write protocol state into the actuator-free canonical X1 URDF."""
        assert self._mj_model is not None and self._mj_data is not None
        s = self.state
        for name, value in zip(LEFT_JOINTS, s.left, strict=True):
            self._mj_data.qpos[self._mj_joint_qpos[name]] = float(value)
        for name, value in zip(RIGHT_JOINTS, s.right, strict=True):
            self._mj_data.qpos[self._mj_joint_qpos[name]] = float(value)
        self._mj_data.qpos[self._mj_joint_qpos[HEAD_JOINTS[0]]] = s.head_pitch
        self._mj_data.qpos[self._mj_joint_qpos[HEAD_JOINTS[1]]] = s.head_yaw
        lift_q = np.clip(s.lift - URDF_LIFT_HEIGHT_OFFSET_M, -0.85, 0.0)
        self._mj_data.qpos[self._mj_joint_qpos[LIFT_JOINT]] = float(lift_q)
        for side, amount in (
            ("left", s.gripper_left),
            ("right", s.gripper_right),
        ):
            opening = FAKE_GRIPPER_MAX_OPENING_M * float(np.clip(amount, 0.0, 1.0))
            for joint_name in MJ_GRIPPER_JOINT_NAMES[side]:
                self._mj_data.qpos[self._mj_joint_qpos[joint_name]] = opening

    def _advance_mujoco(self, dt: float) -> None:
        assert self._mj is not None and self._mj_model is not None and self._mj_data is not None
        s = self.state
        old_left = s.left.copy()
        old_right = s.right.copy()
        max_step = 1.5 * dt
        s.left += np.clip(s.left_target - s.left, -max_step, max_step)
        s.right += np.clip(s.right_target - s.right, -max_step, max_step)
        s.left_velocity = (s.left - old_left) / max(dt, 1e-6)
        s.right_velocity = (s.right - old_right) / max(dt, 1e-6)
        s.lift += float(np.clip(s.lift_target - s.lift, -0.2 * dt, 0.2 * dt))
        s.base_x += s.base_vx * dt
        s.base_y += s.base_vy * dt
        s.base_yaw += s.base_wz * dt

        # X1_mjcf.urdf is the canonical visual/kinematic model used by BeaVR.
        # It intentionally has no actuators, so drive qpos and run forward
        # kinematics instead of inventing arm dynamics or actuator gains.
        self._write_urdf_pose()
        self._mj.mj_forward(self._mj_model, self._mj_data)

        if self._mj_viewer is not None:
            if self._mj_viewer.is_running():
                self._mj_viewer.sync()
            else:
                self.get_logger().warning("MuJoCo viewer was closed; simulation continues headless")
                self._mj_viewer = None

    def _joint_msg(self, names, positions, velocities=None, efforts=None) -> JointState:
        msg = JointState()
        msg.header.stamp = self._stamp()
        msg.name = list(names)
        msg.position = [float(v) for v in positions]
        msg.velocity = [float(v) for v in (velocities if velocities is not None else np.zeros(len(names)))]
        msg.effort = [float(v) for v in (efforts if efforts is not None else np.zeros(len(names)))]
        return msg

    def _publish_state(self) -> None:
        s = self.state
        left_effort = 0.05 * np.sin(s.left)
        right_effort = 0.05 * np.sin(s.right)
        left_msg = self._joint_msg(LEFT_JOINTS, s.left, s.left_velocity, left_effort)
        right_msg = self._joint_msg(RIGHT_JOINTS, s.right, s.right_velocity, right_effort)
        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)
        self.pub_joint.publish(
            self._joint_msg(
                LEFT_JOINTS + RIGHT_JOINTS + HEAD_JOINTS + (LIFT_JOINT,),
                np.concatenate((s.left, s.right, [s.head_pitch, s.head_yaw, s.lift])),
                np.concatenate((s.left_velocity, s.right_velocity, [0.0, 0.0, 0.0])),
                np.concatenate((left_effort, right_effort, [0.0, 0.0, 0.0])),
            )
        )
        self.pub_head.publish(self._joint_msg(HEAD_JOINTS, [s.head_pitch, s.head_yaw]))
        self.pub_lift.publish(self._joint_msg((LIFT_JOINT,), [s.lift]))

        left_gripper = UInt8()
        left_gripper.data = int(np.clip(round(s.gripper_left * 255), 0, 255))
        right_gripper = UInt8()
        right_gripper.data = int(np.clip(round(s.gripper_right * 255), 0, 255))
        self.pub_left_gripper.publish(left_gripper)
        self.pub_right_gripper.publish(right_gripper)

        self.pub_left_pose.publish(self._pose_msg(0.45, 0.25, 0.8, s.left))
        self.pub_right_pose.publish(self._pose_msg(0.45, -0.25, 0.8, s.right))

        odom = Odometry()
        odom.header.stamp = self._stamp()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = s.base_x
        odom.pose.pose.position.y = s.base_y
        odom.pose.pose.orientation.z = math.sin(s.base_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(s.base_yaw / 2.0)
        odom.twist.twist.linear.x = s.base_vx
        odom.twist.twist.linear.y = s.base_vy
        odom.twist.twist.angular.z = s.base_wz
        self.pub_odom.publish(odom)

        battery = BatteryState()
        battery.header.stamp = self._stamp()
        battery.voltage = 48.0
        battery.current = -1.5
        battery.percentage = 0.82
        self.pub_battery.publish(battery)
        bumper = Bool()
        bumper.data = False
        self.pub_bumper.publish(bumper)

        diag = DiagnosticStatus()
        diag.level = DiagnosticStatus.OK
        diag.message = "simulated"
        self.pub_left_diag.publish(diag)
        self.pub_right_diag.publish(diag)

        if self.leader_enabled:
            self.pub_leader_left.publish(self._joint_msg(LEFT_JOINTS, s.leader_left))
            self.pub_leader_right.publish(self._joint_msg(RIGHT_JOINTS, s.leader_right))
            command = Int32()
            command.data = 100 + int(np.clip(s.leader_gripper_left * 99, 0, 99))
            self.pub_leader_gripper.publish(command)
            command = Int32()
            command.data = 200 + int(np.clip(s.leader_gripper_right * 99, 0, 99))
            self.pub_leader_gripper.publish(command)

    def _pose_msg(self, x: float, y: float, z: float, joints: np.ndarray) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = "base_link"
        msg.pose.pose.position.x = x + 0.03 * math.sin(float(joints[0]))
        msg.pose.pose.position.y = y + 0.03 * math.sin(float(joints[1]))
        msg.pose.pose.position.z = z + 0.03 * math.sin(float(joints[2]))
        msg.pose.pose.orientation.w = 1.0
        return msg

    def _publish_cameras(self, elapsed: float) -> None:
        if not self.cameras_enabled or elapsed - self._last_camera_time < 1.0 / self.camera_fps:
            return
        self._last_camera_time = elapsed
        try:
            import cv2
        except ImportError:
            self.get_logger().warning("cv2 unavailable; disabling simulated cameras")
            self.cameras_enabled = False
            return
        colors = {"head": (55, 85, 180), "left": (55, 160, 85), "right": (160, 85, 55)}
        for name, publisher in self.camera_publishers.items():
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            image[:] = colors[name]
            cv2.putText(image, f"Onero H1 SIM - {name}", (35, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
            cv2.putText(image, f"t={elapsed:8.2f}s", (35, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self._stamp()
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            publisher.publish(msg)

    def _tick(self) -> None:
        with self._lock:
            dt = 1.0 / self.fps
            self._advance(dt)
            self._publish_state()
            elapsed = self.get_clock().now().nanoseconds / 1e9
            self._publish_cameras(elapsed)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a lightweight Onero H1 ROS2 topic simulator")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--camera-fps", type=float, default=10.0)
    parser.add_argument("--no-cameras", action="store_true")
    parser.add_argument(
        "--leader",
        action="store_true",
        help="Publish a synthetic homogeneous leader trajectory; use lerobot-teleoperate to drive the follower",
    )
    parser.add_argument("--mujoco-model", default=None, help="Use the given MuJoCo XML model as the dynamics backend")
    parser.add_argument("--viewer", action="store_true", help="Open the interactive MuJoCo viewer")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = H1TopicSimulator(
        args.fps,
        args.camera_fps,
        not args.no_cameras,
        args.leader,
        mujoco_model=args.mujoco_model,
        viewer=args.viewer,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
