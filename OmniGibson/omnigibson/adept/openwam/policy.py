from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from omnigibson.adept.openwam.r1pro_action_adapter import EEF20_DIM, R1ProActionAdapter
from omnigibson.adept.openwam.ws_client import (
    ACTION,
    EVAL_READY,
    PONG,
    RESET_ACK,
    WSPolicyClient,
    build_payload,
    encode_numpy_b64,
)

_PROMPT_PREFIX = "A video recorded from a robot's point of view executing the following instruction: "
_DEFAULT_CAMERAS = {
    "head": "zed_link",
    "left_wrist": "left_realsense_link",
    "right_wrist": "right_realsense_link",
}
_SUPPORTED_CONTRACT = {
    "representation": "eef",
    "eef_frame": "robot_base",
    "pose_type": "absolute",
    "rotation": "rot6d",
    "gripper_model": "zero_closed_one_open",
    "gripper_native": "minus_one_closed_plus_one_open",
    "torso_mode": "hold_initial",
    "endpoint": "eef_link",
}


def load_policy_config(config_path: str | None, action_frequency: float) -> dict:
    """读取并校验当前适配器支持的推理约定.

    参数
    ----
    config_path : str or None
        policy_config.yml 路径. None 时使用适配器目录中的默认文件.
    action_frequency : float
        环境每个控制步对应的动作频率, 单位 Hz.

    返回
    ----
    dict
        通过校验的配置.
    """
    path = Path(__file__).with_name("policy_config.yml") if config_path is None else Path(config_path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"policy config does not exist: {path}")
    with path.open() as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"policy config must be a mapping: {path}")
    required = (*_SUPPORTED_CONTRACT, "control_hz")
    missing = [key for key in required if key not in loaded]
    if missing:
        raise ValueError("policy config is missing required keys: " + ", ".join(missing))
    for key, expected in _SUPPORTED_CONTRACT.items():
        actual = str(loaded[key]).strip()
        if actual != expected:
            raise ValueError(f"policy config {key}={actual!r} is unsupported. This adapter accepts {expected!r}.")
    if float(loaded["control_hz"]) != float(action_frequency):
        raise ValueError(
            f"policy config control_hz={loaded['control_hz']} does not match "
            f"environment action_frequency={action_frequency}."
        )
    config = {
        "action_source": "model",
        "host": "127.0.0.1",
        "port": 8848,
        "request_timeout": 300,
        "prompt_prefix": _PROMPT_PREFIX,
        "cameras": dict(_DEFAULT_CAMERAS),
        "max_joint_delta_rad": 0.2,
    }
    config.update(loaded)
    cameras = dict(_DEFAULT_CAMERAS)
    cameras.update(config.get("cameras") or {})
    config["cameras"] = cameras
    if float(config["max_joint_delta_rad"]) <= 0.0:
        raise ValueError(f"max_joint_delta_rad must be positive, got {config['max_joint_delta_rad']}.")
    return config


def check_server_contract(pong: dict, config: dict) -> None:
    """核对服务端声明的动作表达, 并把未声明的几何约定绑定到策略配置.

    参数
    ----
    pong : dict
        推理服务的 ping 响应.
    config : dict
        已通过校验的策略配置.
    """
    if pong.get("type") != PONG:
        raise RuntimeError(f"OpenWAM server ping returned {pong}.")
    if "representation" not in pong:
        raise RuntimeError("OpenWAM server did not advertise representation.")
    if str(pong["representation"]) != config["representation"]:
        raise RuntimeError(
            f"representation mismatch: server={pong['representation']!r}, config={config['representation']!r}."
        )
    if "gripper_convention" in pong and str(pong["gripper_convention"]) != config["gripper_model"]:
        raise RuntimeError(
            "gripper_convention mismatch: "
            f"server={pong['gripper_convention']!r}, config={config['gripper_model']!r}."
        )
    for key in ("eef_frame", "pose_type", "rotation", "endpoint"):
        if key in pong and str(pong[key]) != config[key]:
            raise RuntimeError(f"{key} mismatch: server={pong[key]!r}, config={config[key]!r}.")
    binary_dims = pong.get("binary_action_dims") or []
    if binary_dims:
        raise RuntimeError(f"checkpoint advertises binary_action_dims={binary_dims}. This adapter expects continuous EEF20.")
    undeclared = [key for key in ("gripper_convention", "eef_frame", "pose_type", "rotation", "endpoint") if key not in pong]
    gripper = pong["gripper_convention"] if "gripper_convention" in pong else config["gripper_model"]
    print(
        "OpenWAMBehaviorPolicy "
        f"representation={pong['representation']} gripper_convention={gripper} "
        f"eef_frame={config['eef_frame']} pose_type={config['pose_type']} "
        f"rotation={config['rotation']} endpoint={config['endpoint']} "
        f"server_undeclared={undeclared}"
    )


def _rgb_uint8(image) -> np.ndarray:
    """将相机观测转换为连续的 HxWx3 uint8 RGB.

    参数
    ----
    image : array-like
        相机 RGB 或 RGBA 图像.

    返回
    ----
    numpy.ndarray
        uint8 RGB 图像.
    """
    if hasattr(image, "detach"):
        image = image.detach().cpu().numpy()
    array = np.asarray(image)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3 or array.shape[-1] < 3:
        raise ValueError(f"camera image must be HxWx3 or HxWx4, got {array.shape}.")
    array = array[..., :3]
    if array.dtype != np.uint8:
        array = np.asarray(array, dtype=np.float32)
        if float(np.nanmax(array)) <= 1.0:
            array = array * 255.0
        array = np.clip(array, 0.0, 255.0).astype(np.uint8)
    return np.ascontiguousarray(array)


class OpenWAMBehaviorPolicy:
    """在 BEHAVIOR 进程内连接 OpenWAM 服务并输出 R1Pro 原生动作.

    计算原理
    --------
    每个仿真步读取头部, 左腕和右腕 RGB, 以及当前 EEF20.
    指令使用训练读取器的前缀加上任务文本. 请求使用 predict_once, 动作缓存由服务端执行器维护.
    返回的 EEF20 交给 R1ProActionAdapter 转换为关节与夹爪命令.

    注意事项
    --------
    策略配置里的动作表达, 坐标系, 末端, 夹爪语义, 躯干处理和 control_hz 必须与本适配器支持的取值一致.
    control_hz 还要等于环境的 action_frequency. 服务端若声明了其中的字段, 声明值也必须一致.
    action_source 为 identity 时, 把当前末端位姿作为目标, 不连接模型服务.
    reset 同时清空服务端动作缓存和适配器保存的躯干, 求解失败计数与跟踪误差.
    """

    def __init__(self, env, config_path: str | None = None, host: str | None = None, port: int | None = None):
        """建立动作适配器, 并在模型模式下连接推理服务.

        参数
        ----
        env : omnigibson.envs.Environment
            已创建的 ADEPT 环境.
        config_path : str or None
            推理约定 YAML.
        host : str or None
            显式服务地址. None 时使用配置文件中的 host.
        port : int or None
            显式服务端口. None 时使用配置文件中的 port.
        """
        self._env = env
        self._config = load_policy_config(config_path, env.env_config["action_frequency"])
        self._action_source = str(self._config["action_source"]).strip().lower()
        if self._action_source not in {"model", "identity"}:
            raise ValueError(f"action_source must be model or identity, got {self._action_source!r}.")
        robot = env.robots[0]
        self._adapter = R1ProActionAdapter(
            robot,
            max_joint_delta_rad=float(self._config["max_joint_delta_rad"]),
        )
        self._cameras = self._config["cameras"]
        self._prompt_prefix = str(self._config["prompt_prefix"])
        self._client = None
        self._model_io = None
        self._sim_step_idx = 0
        if self._action_source == "model":
            server_host = self._config["host"] if host is None else host
            server_port = int(self._config["port"] if port is None else port)
            self._client = WSPolicyClient(
                f"ws://{server_host}:{server_port}",
                timeout=float(self._config["request_timeout"]),
            )
            check_server_contract(self._client.ping(), self._config)
            print(
                f"OpenWAMBehaviorPolicy server=ws://{server_host}:{server_port} "
                f"eef_links={dict(robot.eef_link_names)} max_joint_delta_rad={self._config['max_joint_delta_rad']}"
            )
        else:
            print(
                "OpenWAMBehaviorPolicy action_source=identity "
                f"eef_links={dict(robot.eef_link_names)} endpoint={self._config['endpoint']} "
                f"eef_frame={self._config['eef_frame']} torso_mode={self._config['torso_mode']}"
            )

    @property
    def ik_failures(self) -> int:
        """返回本回合已记录的求解失败次数."""
        return int(self._adapter.ik_failures)

    def execution_stats(self) -> dict:
        """返回本回合的求解失败次数和跟踪误差."""
        return self._adapter.execution_stats()

    def observe_execution(self) -> None:
        """在 env.step 之后记录末端跟踪误差并推进控制步编号."""
        self._adapter.observe_execution()
        self._sim_step_idx += 1

    def reset(self) -> None:
        """清空服务端动作缓存和本回合逆解状态."""
        self._adapter.reset()
        self._sim_step_idx = 0
        if self._client is None:
            return
        instruction = str(self._env.task.parameters["instruction"])
        print(f"OpenWAMBehaviorPolicy prompt={self._prompt_prefix + instruction!r}")
        ack = self._client.reset()
        if ack.get("type") != RESET_ACK:
            raise RuntimeError(f"OpenWAM server reset returned {ack}.")

    def close(self) -> None:
        """关闭与推理服务的连接."""
        self.end_recording()
        if self._client is not None:
            self._client.close()

    def begin_recording(self, rollout_dir, identity):
        """预热模型并创建本回合的输入输出记录."""
        from omnigibson.adept.openwam.recording import ModelIOWriter

        if self._client is None:
            return
        response = self._client.prepare_evaluation(self._payload())
        if response.get("type") != EVAL_READY:
            raise RuntimeError(f"OpenWAM evaluation preparation returned {response}.")
        self._model_io = ModelIOWriter(
            Path(rollout_dir) / "model_io.hdf5", response["metadata"], self._config,
            self._env.task.parameters, identity,
        )

    def record_outcome(self, reward, terminated, truncated, info):
        """保存已完成控制步的任务结果."""
        if self._model_io is not None:
            self._model_io.record_outcome(reward, terminated, truncated, info)

    def end_recording(self, reason=None):
        """保存回合结束原因并关闭模型记录."""
        if self._model_io is not None:
            try:
                if reason is not None:
                    self._model_io.finish(reason)
            finally:
                self._model_io.close()
                self._model_io = None

    def _payload(self):
        """读取同一仿真状态下的三路 RGB 与实测 EEF20."""
        return build_payload(
            head=encode_numpy_b64(self._camera_rgb("head")),
            left_wrist=encode_numpy_b64(self._camera_rgb("left_wrist")),
            right_wrist=encode_numpy_b64(self._camera_rgb("right_wrist")),
            prompt=self._prompt_prefix + str(self._env.task.parameters["instruction"]),
            state=self._adapter.current_eef20().tolist(),
        )

    def forward(self, obs=None):
        """返回一步 R1Pro 原生动作.

        参数
        ----
        obs : dict or None
            评估循环传入的观测. 适配器直接读取机器人, 不使用平铺本体向量.

        返回
        ----
        torch.Tensor
            原生动作.
        """
        if self._action_source == "identity":
            return self._adapter.native_action(self._adapter.current_eef20())
        payload = self._payload()
        if self._model_io is not None:
            payload.update(record_model_io=True, sim_step_idx=self._sim_step_idx)
        response = self._client.predict_once(payload)
        if response.get("type") != ACTION:
            raise RuntimeError(f"OpenWAM server returned {response}.")
        action = np.asarray(response["action"], dtype=np.float32).reshape(-1)
        if action.shape != (EEF20_DIM,):
            raise ValueError(
                f"OpenWAM server returned action shape {action.shape}. "
                f"The checkpoint must denormalize and gather its output to EEF20."
            )
        if self._model_io is not None and response["model_io"] is not None:
            self._model_io.record_inference(response["model_io"])
        return self._adapter.native_action(action)

    def _camera_rgb(self, role: str) -> np.ndarray:
        """读取指定角色的当前 RGB 帧.

        参数
        ----
        role : str
            head, left_wrist 或 right_wrist.

        返回
        ----
        numpy.ndarray
            uint8 RGB 图像.
        """
        robot = self._env.robots[0]
        link = self._cameras[role]
        sensor = robot.sensors[f"{robot.name}:{link}:Camera:0"]
        return _rgb_uint8(sensor.get_obs()[0]["rgb"])
