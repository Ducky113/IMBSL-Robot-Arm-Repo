"""Table-frame motion for SO-101.

Coordinate convention (table frame):
  - (0, 0, 0) at the end-effector pose when all arm joints are at calibration
    midpoint (0 degrees in use_degrees mode; gripper at default_gripper)
  - x / y / z are offsets in robot-base axes from that home pose
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.rotation import Rotation


def _main_root() -> Path:
    return Path(__file__).resolve().parents[1]


def load_table_config(config_path: str | Path | None = None) -> dict[str, Any]:
    path = Path(config_path) if config_path else _main_root() / "config/robot/table_frame.json"
    with path.open() as f:
        cfg = json.load(f)
    cfg["_config_path"] = str(path.resolve())
    urdf = cfg["kinematics"]["urdf_path"]
    urdf_path = Path(urdf)
    if not urdf_path.is_absolute():
        urdf_path = (_main_root() / urdf).resolve()
    cfg["kinematics"]["urdf_path"] = str(urdf_path)
    return cfg


class TableMotionController:
    """Move the arm to absolute (x, y, z) poses in the table frame."""

    def __init__(self, config: dict[str, Any] | None = None):
        self.cfg = config or load_table_config()
        robot_cfg = self.cfg["robot"]
        self.follower = SO101Follower(
            SO101FollowerConfig(
                port=robot_cfg["port"],
                id=robot_cfg["id"],
                use_degrees=robot_cfg.get("use_degrees", True),
                max_relative_target=robot_cfg.get("max_relative_target", 10.0),
            )
        )
        self.motor_names = list(self.follower.bus.motors.keys())
        self.kinematics = RobotKinematics(
            urdf_path=self.cfg["kinematics"]["urdf_path"],
            target_frame_name=self.cfg["kinematics"]["target_frame_name"],
            joint_names=self.motor_names,
        )
        ws = self.cfg["workspace_m"]
        self.workspace_min = np.array([ws["x"][0], ws["y"][0], ws["z"][0]], dtype=float)
        self.workspace_max = np.array([ws["x"][1], ws["y"][1], ws["z"][1]], dtype=float)
        self._ref_ee_pos = np.zeros(3, dtype=float)
        self._ref_rotvec = np.zeros(3, dtype=float)
        self._init_reference_pose()

    def _neutral_joint_positions(self) -> np.ndarray:
        """Mid-range joint targets: 0 deg for arm joints after calibration, default gripper."""
        if "neutral_joint_positions" in self.cfg:
            return np.array(self.cfg["neutral_joint_positions"], dtype=float)
        gripper = float(self.cfg.get("default_gripper", 50.0))
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, gripper], dtype=float)

    def _init_reference_pose(self) -> None:
        q_neutral = self._neutral_joint_positions()
        t_ref = self.kinematics.forward_kinematics(q_neutral)
        self._ref_ee_pos = t_ref[:3, 3].copy()
        self._ref_rotvec = Rotation.from_matrix(t_ref[:3, :3]).as_rotvec()

    def connect(self, calibrate: bool = True, home: bool | None = None) -> None:
        self.follower.connect(calibrate=calibrate)
        if home is None:
            home = bool(self.cfg.get("table_frame", {}).get("home_on_connect", True))
        if home:
            self.go_home()

    def disconnect(self) -> None:
        self.follower.disconnect()

    def table_to_robot_xyz(self, x_m: float, y_m: float, z_m: float) -> np.ndarray:
        """Table (x,y,z) offset from home -> robot-base frame position for IK."""
        return self._ref_ee_pos + np.array([x_m, y_m, z_m], dtype=float)

    def robot_to_table_xyz(self, pos_robot: np.ndarray) -> np.ndarray:
        """Robot-base frame position -> table offset from home."""
        return np.array(pos_robot, dtype=float) - self._ref_ee_pos

    def _clip_table_xyz(self, x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
        table = np.clip(np.array([x_m, y_m, z_m]), self.workspace_min, self.workspace_max)
        return float(table[0]), float(table[1]), float(table[2])

    def _joints_from_obs(self, obs: dict[str, float]) -> np.ndarray:
        return np.array([float(obs[f"{name}.pos"]) for name in self.motor_names], dtype=float)

    def _interpolate_joints(self, q_start: np.ndarray, q_goal: np.ndarray, duration_s: float | None) -> None:
        motion = self.cfg.get("motion", {})
        duration = float(duration_s if duration_s is not None else motion.get("duration_s", 2.0))
        fps = int(motion.get("fps", 30))
        n_steps = max(int(duration * fps), 1)
        for i in range(1, n_steps + 1):
            alpha = i / n_steps
            q_cmd = q_start + alpha * (q_goal - q_start)
            action = {f"{name}.pos": float(q_cmd[j]) for j, name in enumerate(self.motor_names)}
            self.follower.send_action(action)
            precise_sleep(1.0 / fps)

    def go_home(self, duration_s: float | None = None) -> None:
        """Move all joints to calibration mid-range (starting position)."""
        q_goal = self._neutral_joint_positions()
        obs = self.follower.get_observation()
        q_start = self._joints_from_obs(obs)
        self._interpolate_joints(q_start, q_goal, duration_s)

    def get_table_pose(self) -> dict[str, float]:
        """Current end-effector pose expressed in the table frame."""
        obs = self.follower.get_observation()
        q = self._joints_from_obs(obs)
        t = self.kinematics.forward_kinematics(q)
        pos = self.robot_to_table_xyz(t[:3, 3])
        rotvec = Rotation.from_matrix(t[:3, :3]).as_rotvec()
        return {
            "x_m": float(pos[0]),
            "y_m": float(pos[1]),
            "z_m": float(pos[2]),
            "wx": float(rotvec[0]),
            "wy": float(rotvec[1]),
            "wz": float(rotvec[2]),
            "gripper": float(obs["gripper.pos"]),
        }

    def _solve_ik(
        self,
        x_m: float,
        y_m: float,
        z_m: float,
        rotvec: np.ndarray,
        q_guess: np.ndarray,
        gripper: float,
    ) -> np.ndarray:
        pos_robot = self.table_to_robot_xyz(x_m, y_m, z_m)
        t_des = np.eye(4, dtype=float)
        t_des[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
        t_des[:3, 3] = pos_robot
        q_target = self.kinematics.inverse_kinematics(q_guess, t_des)
        q_target = q_target.copy()
        q_target[self.motor_names.index("gripper")] = gripper
        return q_target

    def move_to_table(
        self,
        x_m: float,
        y_m: float,
        z_m: float | None = None,
        *,
        wz_rad: float | None = None,
        gripper: float | None = None,
        keep_orientation: bool = True,
        duration_s: float | None = None,
    ) -> dict[str, float]:
        """Move gripper to table-frame (x, y, z). Returns the commanded table pose."""
        if z_m is None:
            z_m = float(self.cfg.get("default_hover_z_m", 0.0))
        if gripper is None:
            gripper = float(self.cfg.get("default_gripper", 50.0))

        x_m, y_m, z_m = self._clip_table_xyz(x_m, y_m, z_m)

        obs = self.follower.get_observation()
        q_start = self._joints_from_obs(obs)
        t_curr = self.kinematics.forward_kinematics(q_start)
        rotvec = Rotation.from_matrix(t_curr[:3, :3]).as_rotvec()
        if wz_rad is not None:
            rotvec = np.array([0.0, 0.0, float(wz_rad)], dtype=float)
        elif not keep_orientation:
            rotvec = self._ref_rotvec.copy()

        q_goal = self._solve_ik(x_m, y_m, z_m, rotvec, q_start, gripper)
        self._interpolate_joints(q_start, q_goal, duration_s)

        return {"x_m": x_m, "y_m": y_m, "z_m": z_m, "gripper": gripper}

    def set_gripper(self, value: float) -> None:
        obs = self.follower.get_observation()
        q = self._joints_from_obs(obs)
        q[self.motor_names.index("gripper")] = value
        action = {f"{name}.pos": float(q[j]) for j, name in enumerate(self.motor_names)}
        self.follower.send_action(action)
