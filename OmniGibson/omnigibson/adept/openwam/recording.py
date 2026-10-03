import json
from pathlib import Path

import h5py
import numpy as np

from omnigibson.adept.openwam.ws_client import decode_numpy_b64
from omnigibson.utils.config_utils import TorchEncoder


class ModelIOWriter:
    """增量保存模型输入输出与逐步任务结果."""

    def __init__(self, path, metadata, policy_config, task_parameters, identity):
        """创建回合元数据与可扩展数据集."""
        self.file = h5py.File(Path(path), "x")
        attrs = {
            "task": identity["task"],
            "run_name": identity["run_name"],
            "instance_id": identity["instance_id"],
            "rollout_id": identity["rollout_id"],
            "task_description": task_parameters["instruction"],
            "control_hz": policy_config["control_hz"],
            "inference_mode": metadata["inference_mode"],
            "action_horizon": metadata["action_horizon"],
            "exec_action_horizon": metadata["exec_action_horizon"],
            "future_frame_count": metadata["future_frame_count"],
            "future_frame_stride": metadata["future_frame_stride"],
            "eef_frame": policy_config["eef_frame"],
            "pose_type": policy_config["pose_type"],
            "position_unit": "m",
            "rotation": policy_config["rotation"],
            "gripper_convention": policy_config["gripper_model"],
        }
        self.file.attrs.update(attrs)
        meta = self.file.create_group("meta")
        text_dtype = h5py.string_dtype("utf-8")
        for name, value in (
            ("model_config", metadata["model_config"]),
            ("policy_config", policy_config),
            ("normalization_stats", metadata["normalization_stats"]),
            ("task_parameters", task_parameters),
        ):
            meta.create_dataset(name, data=json.dumps(value, cls=TorchEncoder, ensure_ascii=False), dtype=text_dtype)
        for name in ("inference", "outcome"):
            self.file.create_group(name)
        image_shape = (metadata["image_height"], metadata["image_width"], 3)
        action_horizon = metadata["action_horizon"]
        future_frame_count = metadata["future_frame_count"]
        for path, shape, dtype in (
            ("inference/sim_step_idx", (), "int64"),
            ("inference/input_imgs", image_shape, "uint8"),
            ("inference/input_prop", (20,), "float32"),
            ("inference/input_instruction", (), text_dtype),
            ("inference/output_action", (action_horizon, 20), "float32"),
            ("inference/output_future_imgs", (future_frame_count, *image_shape), "uint8"),
            ("inference/gpu_time", (), "float64"),
            ("outcome/success", (), "bool"),
            ("outcome/terminated", (), "bool"),
            ("outcome/truncated", (), "bool"),
            ("outcome/reward", (), "float32"),
            ("outcome/goal_status", (), text_dtype),
        ):
            options = {"chunks": True}
            if dtype == "uint8":
                options = {
                    "chunks": (1,) * (len(shape) - 2) + image_shape,
                    "compression": "gzip", "compression_opts": 1,
                }
            self.file.create_dataset(path, shape=(0, *shape), maxshape=(None, *shape), dtype=dtype, **options)
        self.file["outcome"].attrs.update(
            final_success=False, first_success_step_idx=-1, termination_reason="incomplete",
        )

    def _append(self, path, value):
        """在指定数据集的首维追加一条记录."""
        dataset = self.file[path]
        index = len(dataset)
        dataset.resize(index + 1, axis=0)
        dataset[index] = value

    def record_inference(self, record):
        """保存一次真实推理的完整预测块和无损 RGB 画布."""
        for name in ("sim_step_idx", "input_prop", "input_instruction", "output_action", "gpu_time"):
            self._append(f"inference/{name}", record[name])
        self._append("inference/input_imgs", decode_numpy_b64(record["input_imgs"]))
        self._append("inference/output_future_imgs", np.stack([
            decode_numpy_b64(frame) for frame in record["output_future_imgs"]
        ]))
        self.file.flush()

    def record_outcome(self, reward, terminated, truncated, info):
        """保存当前控制步的任务结果与首次成功时刻."""
        step = len(self.file["outcome/success"])
        success = bool(info["done"]["success"])
        for name, value in (
            ("success", success), ("terminated", bool(terminated)),
            ("truncated", bool(truncated)), ("reward", float(reward)),
            ("goal_status", json.dumps(info["done"]["goal_status"], cls=TorchEncoder, ensure_ascii=False)),
        ):
            self._append(f"outcome/{name}", value)
        attrs = self.file["outcome"].attrs
        attrs["final_success"] = success
        if success and attrs["first_success_step_idx"] < 0:
            attrs["first_success_step_idx"] = step + 1

    def finish(self, reason):
        """保存正常结束原因并提交文件缓冲."""
        self.file["outcome"].attrs["termination_reason"] = reason
        self.file.flush()

    def close(self):
        """关闭模型记录文件."""
        self.file.close()
