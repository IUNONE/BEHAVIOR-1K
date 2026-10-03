from __future__ import annotations

import numpy as np
import torch as th

import omnigibson.controllers.ik_controller as ik_controller
import omnigibson.utils.transform_utils_np as transform_np
from omnigibson.utils.usd_utils import ControllableObjectViewAPI

EEF20_DIM = 20
_ARMS = (
    ("left", slice(0, 3), slice(3, 9), 9),
    ("right", slice(10, 13), slice(13, 19), 19),
)


def _numpy(value) -> np.ndarray:
    """将仿真张量或数组转为 numpy."""
    if isinstance(value, th.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def quat_xyzw_to_matrix(quat_xyzw) -> np.ndarray:
    """将 xyzw 单位四元数转换为旋转矩阵.

    参数
    ----
    quat_xyzw : array-like
        形状为 (4,) 的四元数, 顺序为 x, y, z, w.

    返回
    ----
    numpy.ndarray
        形状为 (3, 3) 的旋转矩阵.
    """
    quat = np.asarray(quat_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        raise ValueError("EEF quaternion norm is zero.")
    x, y, z, w = quat / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_rot6d(matrix) -> np.ndarray:
    """取旋转矩阵的前两列并按列拼接为 rot6d.

    参数
    ----
    matrix : array-like
        形状为 (3, 3) 的旋转矩阵.

    返回
    ----
    numpy.ndarray
        形状为 (6,) 的 rot6d.
    """
    rotation = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    return np.concatenate([rotation[:, 0], rotation[:, 1]]).astype(np.float32)


def rot6d_to_matrix(rot6d) -> np.ndarray:
    """用 Gram-Schmidt 将 rot6d 还原为旋转矩阵.

    参数
    ----
    rot6d : array-like
        形状为 (6,) 的 rot6d, 前三维和后三维分别是旋转矩阵的前两列.

    返回
    ----
    numpy.ndarray
        形状为 (3, 3) 的旋转矩阵.
    """
    raw = np.asarray(rot6d, dtype=np.float64).reshape(6)
    first = raw[:3]
    first_norm = float(np.linalg.norm(first))
    if first_norm <= 1e-8:
        raise ValueError("rot6d first column is degenerate.")
    first = first / first_norm
    second = raw[3:] - np.dot(first, raw[3:]) * first
    second_norm = float(np.linalg.norm(second))
    if second_norm <= 1e-8:
        raise ValueError("rot6d second column is degenerate.")
    second = second / second_norm
    return np.column_stack([first, second, np.cross(first, second)])


def gripper_opening(robot, arm: str) -> float:
    """按关节限位把夹爪开合归一化到 0 闭合, 1 张开.

    参数
    ----
    robot : omnigibson.robots.robot.Robot
        当前 R1Pro.
    arm : str
        left 或 right.

    返回
    ----
    float
        两指归一化开合的平均值.
    """
    index = robot.gripper_control_idx[arm]
    position = _numpy(robot.get_joint_positions()[index]).astype(np.float64)
    lower = _numpy(robot.joint_lower_limits[index]).astype(np.float64)
    upper = _numpy(robot.joint_upper_limits[index]).astype(np.float64)
    opening = (position - lower) / np.maximum(upper - lower, 1e-6)
    return float(np.clip(opening.mean(), 0.0, 1.0))


def model_gripper_to_native(opening: float) -> float:
    """将模型夹爪开合映射为 MultiFingerGripperController 的原生命令.

    参数
    ----
    opening : float
        0 为闭合, 1 为张开.

    返回
    ----
    float
        -1 为闭合, +1 为张开.
    """
    return 2.0 * float(np.clip(opening, 0.0, 1.0)) - 1.0


class R1ProActionAdapter:
    """把绝对 EEF20 写成有界的单步关节命令.

    计算原理
    --------
    每只手臂读取 eef_link 相对 base_footprint 的位姿. 模型给出的绝对目标用同一坐标系的 rot6d.
    关节更新使用 InverseKinematicsController 的一次 J-PARSE, 再把相对当前关节的增量限制在
    max_joint_delta_rad 内, 并裁剪到关节限位. 求解只使用该手臂的 7 个关节列.
    这不是收敛到目标位姿的迭代逆解, 也没有对候选关节做正向运动学验收.

    注意事项
    --------
    EEF20 没有躯干通道. reset 记录回合开始时的躯干关节, 之后每步原样写回.
    雅可比行与 InverseKinematicsController 一致, 根连杆不在 Isaac 雅可比中, 所以行号使用 body index 减 1.
    ik_failures 只统计退化旋转, 不可用雅可比和非有限解. 这些情况下该手臂保持当前关节.
    跟踪误差由 observe_execution 在 env.step 之后计算, 比较刚下发的目标与执行后的实测末端, 与 ik_failures 分开累计.
    """

    def __init__(self, robot, max_joint_delta_rad: float = 0.2):
        """保存机器人, 并检查双臂末端名称可用.

        参数
        ----
        robot : omnigibson.robots.robot.Robot
            环境中的 R1Pro.
        max_joint_delta_rad : float
            单步每个手臂关节允许的最大绝对增量, 单位弧度.
        """
        if robot.model != "r1pro":
            raise ValueError(f"adapter expects r1pro, got {robot.model}.")
        if float(max_joint_delta_rad) <= 0.0:
            raise ValueError(f"max_joint_delta_rad must be positive, got {max_joint_delta_rad}.")
        for arm, _, _, _ in _ARMS:
            if arm not in robot.eef_link_names or arm not in robot.arm_control_idx:
                raise ValueError(f"R1Pro is missing the {arm} arm description.")
        self.robot = robot
        self.max_joint_delta_rad = float(max_joint_delta_rad)
        self._trunk_q = None
        self._step = 0
        self.ik_failures = 0
        self._targets = {}
        self._max_tracking_position_error_m = None
        self._max_tracking_orientation_error_rad = None
        # Importing the controller module registers compute_ik_qpos_batch.
        self._ik = ik_controller._compute_ik_qpos_batch_numpy

    def reset(self) -> None:
        """记录当前躯干关节并清空本回合求解与跟踪统计."""
        trunk = self.robot.get_joint_positions()[self.robot.trunk_control_idx]
        self._trunk_q = trunk.detach().to(dtype=th.float32, device="cpu").clone()
        self._step = 0
        self.ik_failures = 0
        self._targets = {}
        self._max_tracking_position_error_m = None
        self._max_tracking_orientation_error_rad = None

    def execution_stats(self) -> dict:
        """返回本回合的求解失败次数和已观测跟踪误差.

        返回
        ----
        dict
            ik_failures 为求解失败次数. 两个跟踪字段在尚无已执行命令被观测时为 None.
        """
        return {
            "ik_failures": int(self.ik_failures),
            "max_tracking_position_error_m": self._max_tracking_position_error_m,
            "max_tracking_orientation_error_rad": self._max_tracking_orientation_error_rad,
        }

    def current_eef20(self) -> np.ndarray:
        """读取双臂当前 EEF20.

        返回
        ----
        numpy.ndarray
            形状为 (20,) 的 float32. 左臂 xyz, rot6d, gripper, 然后是右臂.
        """
        state = np.zeros(EEF20_DIM, dtype=np.float32)
        for arm, position_slice, rotation_slice, gripper_index in _ARMS:
            position, quaternion = self.robot.get_relative_eef_pose(arm)
            state[position_slice] = _numpy(position).astype(np.float32)
            state[rotation_slice] = matrix_to_rot6d(quat_xyzw_to_matrix(_numpy(quaternion)))
            state[gripper_index] = np.float32(gripper_opening(self.robot, arm))
        return state

    def native_action(self, eef20) -> th.Tensor:
        """将一条 EEF20 写成环境动作.

        参数
        ----
        eef20 : array-like
            形状为 (20,) 的绝对末端目标.

        返回
        ----
        torch.Tensor
            形状为 (action_dim,) 的原生动作. 底盘速度为 0, 躯干为目标初始关节位置.
        """
        if self._trunk_q is None:
            raise RuntimeError("call reset() after the environment reset before requesting an action.")
        command = np.asarray(eef20, dtype=np.float64).reshape(-1)
        if command.shape != (EEF20_DIM,) or not np.all(np.isfinite(command)):
            raise ValueError(f"EEF20 action must contain {EEF20_DIM} finite values, got shape {command.shape}.")

        action = th.zeros(self.robot.action_dim, dtype=th.float32)
        action[self.robot.base_action_idx] = 0.0
        action[self.robot.trunk_action_idx] = self._trunk_q
        for arm, position_slice, rotation_slice, gripper_index in _ARMS:
            measured_position, measured_matrix = self._measured_pose(arm)
            arm_q, solved, goal_position, goal_matrix = self._solve_arm(
                arm,
                command[position_slice],
                command[rotation_slice],
                measured_position,
                measured_matrix,
            )
            self._targets[arm] = (goal_position, goal_matrix) if solved else (measured_position, measured_matrix)
            action[self.robot.arm_action_idx[arm]] = th.tensor(arm_q, dtype=th.float32)
            action[self.robot.gripper_action_idx[arm]] = model_gripper_to_native(command[gripper_index])
        self._step += 1
        return action

    def observe_execution(self) -> None:
        """根据刚执行完的命令更新双臂跟踪误差."""
        if not self._targets:
            return
        for arm, _, _, _ in _ARMS:
            measured_position, measured_matrix = self._measured_pose(arm)
            self._update_tracking(arm, measured_position, measured_matrix)

    def _measured_pose(self, arm: str):
        """读取该手臂当前末端位置和旋转矩阵.

        参数
        ----
        arm : str
            left 或 right.

        返回
        ----
        tuple
            位置 (3,) 和旋转矩阵 (3, 3).
        """
        position, quaternion = self.robot.get_relative_eef_pose(arm)
        matrix = np.ascontiguousarray(quat_xyzw_to_matrix(_numpy(quaternion)), dtype=np.float64)
        return _numpy(position).astype(np.float64).reshape(3), matrix

    def _update_tracking(self, arm: str, measured_position, measured_matrix) -> None:
        """用上一条已下发目标更新跟踪误差.

        参数
        ----
        arm : str
            left 或 right.
        measured_position : numpy.ndarray
            当前末端位置.
        measured_matrix : numpy.ndarray
            当前末端旋转矩阵.
        """
        previous = self._targets.get(arm)
        if previous is None:
            return
        previous_position, previous_matrix = previous
        position_error = float(np.linalg.norm(previous_position - measured_position))
        orientation_error = transform_np.orientation_error(
            np.ascontiguousarray(previous_matrix, dtype=np.float64),
            np.ascontiguousarray(measured_matrix, dtype=np.float64),
        ).reshape(3)
        orientation_error = float(np.linalg.norm(orientation_error))
        if self._max_tracking_position_error_m is None or position_error > self._max_tracking_position_error_m:
            self._max_tracking_position_error_m = position_error
        if (
            self._max_tracking_orientation_error_rad is None
            or orientation_error > self._max_tracking_orientation_error_rad
        ):
            self._max_tracking_orientation_error_rad = orientation_error

    def _solve_arm(self, arm: str, position, rot6d, measured_position, measured_matrix):
        """计算单臂的一步有界关节命令.

        参数
        ----
        arm : str
            left 或 right.
        position : array-like
            基座坐标系下的目标位置, 形状为 (3,).
        rot6d : array-like
            目标 rot6d, 形状为 (6,).
        measured_position : numpy.ndarray
            当前末端位置.
        measured_matrix : numpy.ndarray
            当前末端旋转矩阵.

        返回
        ----
        tuple
            关节命令, 是否得到有限解, 以及对应的目标位置和旋转矩阵.
        """
        current = self._current_arm_position(arm)
        try:
            goal_position = np.asarray(position, dtype=np.float64).reshape(3)
            goal_matrix = np.ascontiguousarray(rot6d_to_matrix(rot6d), dtype=np.float64)
        except ValueError as error:
            self._record_failure(arm, str(error))
            return current, False, None, None
        if not np.all(np.isfinite(goal_position)) or not np.all(np.isfinite(goal_matrix)):
            self._record_failure(arm, "target pose is not finite")
            return current, False, None, None

        jacobian, lower, upper = self._arm_jacobian_and_limits(arm)
        if jacobian.shape != (6, current.shape[0]) or not np.all(np.isfinite(jacobian)):
            self._record_failure(arm, f"jacobian shape {jacobian.shape} is not usable")
            return current, False, None, None

        solved = self._ik(
            current.reshape(1, -1).astype(np.float32),
            jacobian.reshape(1, 6, -1).astype(np.float32),
            measured_position.reshape(1, 3).astype(np.float32),
            measured_matrix.reshape(1, 3, 3).astype(np.float32),
            goal_position.reshape(1, 3).astype(np.float32),
            goal_matrix.reshape(1, 3, 3).astype(np.float32),
            lower.reshape(1, -1).astype(np.float32),
            upper.reshape(1, -1).astype(np.float32),
        )[0]
        solved = np.asarray(solved, dtype=np.float64).reshape(-1)
        if solved.shape != current.shape or not np.all(np.isfinite(solved)):
            self._record_failure(arm, "IK solution is not finite")
            return current, False, None, None
        solved = np.clip(solved, current - self.max_joint_delta_rad, current + self.max_joint_delta_rad)
        solved = np.clip(solved, lower, upper)
        return solved, True, goal_position, goal_matrix

    def _current_arm_position(self, arm: str) -> np.ndarray:
        """返回该手臂当前关节位置.

        参数
        ----
        arm : str
            left 或 right.

        返回
        ----
        numpy.ndarray
            手臂关节位置.
        """
        index = self.robot.arm_control_idx[arm]
        return _numpy(self.robot.get_joint_positions()[index]).astype(np.float64).reshape(-1)

    def _arm_jacobian_and_limits(self, arm: str):
        """提取单臂雅可比及对应关节限位.

        参数
        ----
        arm : str
            left 或 right.

        返回
        ----
        tuple
            雅可比 (6, dof), 下限和上限.
        """
        path = self.robot.articulation_root_path
        link_name = self.robot.eef_link_names[arm]
        jacobian = _numpy(ControllableObjectViewAPI.get_relative_jacobian(path))
        joints = _numpy(ControllableObjectViewAPI.get_joint_positions(path)).reshape(-1)
        # Isaac 的连杆雅可比不包含根 body, 而行索引来自包含根 body 的 link 列表.
        row = int(ControllableObjectViewAPI.get_link_index(path, link_name)) - 1
        if row < 0 or row >= jacobian.shape[0]:
            raise RuntimeError(f"{arm} EEF jacobian row {row} is outside shape {jacobian.shape}.")
        column_offset = int(jacobian.shape[-1] - joints.shape[0])
        columns = _numpy(self.robot.arm_control_idx[arm]).astype(np.int64) + column_offset
        arm_jacobian = np.asarray(jacobian[row][:, columns], dtype=np.float64)
        index = self.robot.arm_control_idx[arm]
        lower = _numpy(self.robot.joint_lower_limits[index]).astype(np.float64).reshape(-1)
        upper = _numpy(self.robot.joint_upper_limits[index]).astype(np.float64).reshape(-1)
        return arm_jacobian, lower, upper

    def _record_failure(self, arm: str, reason: str) -> None:
        """记录一次手臂逆解失败.

        参数
        ----
        arm : str
            失败的手臂.
        reason : str
            失败原因.
        """
        self.ik_failures += 1
        if self.ik_failures <= 5 or self.ik_failures % 50 == 0:
            print(f"ADEPT IK failed: arm={arm} step={self._step} count={self.ik_failures} {reason}")
