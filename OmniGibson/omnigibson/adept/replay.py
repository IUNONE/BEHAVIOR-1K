import json
import os
from pathlib import Path

import cv2
import h5py
import numpy as np

import omnigibson as og
from omnigibson.adept.environment import configure_robot_physics
from omnigibson.adept.openwam.r1pro_action_adapter import gripper_opening, quat_xyzw_to_matrix
from omnigibson.adept.scene import ADEPTScene
from omnigibson.adept.visuals import VideoWriter, cameras, mosaic
from omnigibson.envs.data_wrapper import DataPlaybackWrapper
from omnigibson.macros import gm


def as_numpy(value):
    """将仿真张量转换为 CPU 数组."""
    return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)


def pose_matrix(position, quaternion):
    """将位置和 xyzw 四元数组合为局部到父坐标系的矩阵."""
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, :3] = quat_xyzw_to_matrix(as_numpy(quaternion))
    matrix[:3, 3] = as_numpy(position)
    return matrix


class ADEPTPlaybackWrapper(DataPlaybackWrapper):
    """在源状态恢复后导出同步图像, 相机和机器人状态."""

    def create_dataset(self, output_path, env, overwrite=True):
        """创建逐回合 HDF5, 保留原始仿真记录."""
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        self.output_hdf5 = h5py.File(output_path, "w" if overwrite else "x")
        self.preview_writer = None
        self.exported_frames = 0

    def prepare_export(self, group, task_name, input_path):
        """分配状态及动作数据集, 保存任务文本和回放来源."""
        output = self.output_hdf5
        self.has_terminal = "terminal_state" in group
        self.frame_count = len(group["state"]) + int(self.has_terminal)
        transitions = self.frame_count - 1
        parameters = json.loads(group.attrs["task_parameters"])
        source_name = f"episode_{Path(input_path).stem}_d{int(group.name.rsplit('_', 1)[1]):06d}"
        source_id = group.attrs.get("source_episode_id", f"robot/{task_name}/{source_name}")
        robot = self.robots[0]
        output.attrs.update({
            "source": "BEHAVIOR-1K", "embodiment": robot.model,
            "source_episode_id": source_id,
            "source_file": os.path.relpath(Path(input_path).resolve(), Path(output.filename).resolve().parent),
            "source_group": group.name,
            "task": task_name, "task_description": parameters["instruction"],
            "instance_id": int(group.attrs["instance_id"]), "fps": self.fps,
            "record_type": group.attrs.get("record_type", "unknown"),
            "has_terminal_state": self.has_terminal,
            "recorded_final_success": bool(group["success"][-1]),
            "world_frame": "omnigibson_world",
            "position_unit": "m",
            "gripper_convention": "zero_closed_one_open",
        })
        meta = output.create_group("meta")
        meta.create_dataset("timestamps", data=np.arange(self.frame_count, dtype=np.float64) / self.fps)
        parameters.pop("instruction")
        parameters.pop("thresholds", None)
        if "sampling" in parameters:
            parameters["sampling"] = {key: value for key, value in parameters["sampling"].items()
                                      if key in ("seed", "instance_seed", "settled_poses")}
        meta.create_dataset("task_parameters", data=json.dumps(parameters), dtype=h5py.string_dtype())
        control = output.create_group("control")
        control.create_dataset("native_action", data=group["action"][:transitions])
        control.attrs["channel_indices"] = json.dumps({
            "base": as_numpy(robot.base_action_idx).tolist(), "trunk": as_numpy(robot.trunk_action_idx).tolist(),
            **{f"arm_{arm}": as_numpy(robot.arm_action_idx[arm]).tolist() for arm in ("left", "right")},
            **{f"gripper_{arm}": as_numpy(robot.gripper_action_idx[arm]).tolist() for arm in ("left", "right")},
        })
        outcome = output.create_group("outcome")
        for key in ("success", "terminated", "truncated", "reward"):
            outcome.create_dataset(key, data=group[key][:transitions])
        state = output.create_group("robot")
        state.attrs["joint_names"] = json.dumps(list(robot.dof_names_ordered))
        state.attrs["joint_units"] = "rad for revolute, m for prismatic"
        state.create_dataset("base_pose", (self.frame_count, 4, 4), dtype="f4")
        state.create_dataset("joint_positions", (self.frame_count, len(robot.get_joint_positions())), dtype="f4")
        for arm in ("left", "right"):
            part = state.create_group(arm)
            part.attrs["eef_link"] = robot.eef_link_names[arm]
            part.create_dataset("ee_pose", (self.frame_count, 4, 4), dtype="f4")
            part.create_dataset("gripper_opening", (self.frame_count,), dtype="f4")
        self.export_cameras = cameras(self.env)
        for view in self.export_cameras:
            output.create_group(f"vision/{view}").create_dataset(
                "rgb", (self.frame_count,), dtype=h5py.vlen_dtype(np.dtype("uint8")))
            camera = output.create_group(f"cameras/{view}")
            camera.attrs["axes"] = "opencv_x_right_y_down_z_forward"
            camera.create_dataset("camera2world", (self.frame_count, 4, 4), dtype="f4")
        self.preview_writer = VideoWriter(Path(output.filename).with_suffix(".mp4"), self.fps)

    def capture_state(self, state_index):
        """在物理传播前渲染源状态并同步保存图像及世界系位姿."""
        for _ in range(4):
            og.sim.render()
        output, robot = self.output_hdf5, self.robots[0]
        output["robot/base_pose"][state_index] = pose_matrix(*robot.get_position_orientation())
        output["robot/joint_positions"][state_index] = as_numpy(robot.get_joint_positions())
        for arm in ("left", "right"):
            output[f"robot/{arm}/ee_pose"][state_index] = pose_matrix(
                *robot.eef_links[arm].get_position_orientation(frame="world")
            )
            output[f"robot/{arm}/gripper_opening"][state_index] = gripper_opening(robot, arm)
        images = {}
        for view, sensor in self.export_cameras.items():
            rgb = as_numpy(sensor.get_obs()[0]["rgb"])[..., :3].astype(np.uint8)
            images[view] = rgb
            ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
            if not ok:
                raise RuntimeError(f"JPEG encoding failed: {view}, state {state_index}")
            output[f"vision/{view}/rgb"][state_index] = encoded.reshape(-1)
            camera = output[f"cameras/{view}"]
            if state_index == 0:
                camera.create_dataset("intrinsics", data=as_numpy(sensor.intrinsic_matrix))
                camera.create_dataset("resolution", data=np.array(rgb.shape[:2], dtype=np.int64))
            camera["camera2world"][state_index] = pose_matrix(*sensor.get_position_orientation()) @ np.diag([1, -1, -1, 1])
        self.preview_writer.write(mosaic(images))
        self.exported_frames += 1

    def close_dataset(self):
        """关闭导出 HDF5, 输入记录与预览视频."""
        if self.preview_writer is not None:
            self.preview_writer.close()
            self.preview_writer = None
        self.output_hdf5.close()
        self.input_hdf5.close()


def replay(input_path, task_name, episode_id=None, run_qa=False, output_dir=None, overwrite=False,
           output_episode_index=0):
    """按指定输出编号导出原始回合的 ADEPT HDF5 及同步预览."""
    if output_dir is None:
        raise ValueError("ADEPT replay requires --output-dir")
    gm.ENABLE_TRANSITION_RULES = False
    gm.USE_GPU_DYNAMICS = False
    path = Path(input_path).expanduser()
    with h5py.File(path, "r") as source:
        config = json.loads(source["data"].attrs["config"])
        if config["task"]["activity_name"] != task_name:
            raise ValueError("The requested task does not match the recording.")
        episodes = {int(key.removeprefix("demo_")): group.attrs["num_samples"]
                    for key, group in source["data"].items() if key.startswith("demo_") and group.attrs["num_samples"] > 0}
        episode_id = max(episodes, key=episodes.get) if episode_id is None else episode_id
    episode_name = f"episode_{output_episode_index:06d}"
    directory = Path(output_dir).expanduser()
    destination = directory / f"{episode_name}.hdf5"
    # JoyLo's shoulder cameras are GUI-only; export the fixed dataset view.
    sensors = [sensor for sensor in config["env"]["external_sensors"] if sensor["name"] == "table_side"]
    for sensor in sensors:
        sensor["include_in_obs"] = True
        sensor["sensor_kwargs"].pop("viewport_name", None)
        # Collection only renders GUI viewports; playback needs sensor RGB data.
        sensor["modalities"] = ["rgb"]
    env = None
    try:
        env = ADEPTPlaybackWrapper.create_from_hdf5(
            input_path=str(path), output_path=str(destination), overwrite=overwrite,
            include_task=False, include_contacts=True, include_robot_control=False,
            robot_obs_modalities=["rgb"], external_sensors_config=sensors,
            n_render_iterations=1, only_successes=False,
        )
        configure_robot_physics(env)
        group = env.input_hdf5[f"data/demo_{episode_id}"]
        env.prepare_export(group, task_name, path)
        env.playback_episode(episode_id, record_data=False, state_observation_callback=env.capture_state)
        if env.has_terminal:
            import torch as th

            og.sim.load_state(th.as_tensor(group["terminal_state"][:], dtype=th.float32), serialized=True)
            env.capture_state(env.frame_count - 1)
        else:
            print("Legacy recording has no terminal state; the final command remains only in the raw recording.")
        if env.exported_frames != env.frame_count:
            raise RuntimeError("Playback did not export every recorded state.")
        if run_qa:
            successes = group["success"][:].astype(bool)
            print(f"Recorded execution: task={task_name}, instance={int(group.attrs['instance_id'])}, "
                  f"episode={episode_id}, steps={len(successes)}, final_success={bool(successes[-1])}, "
                  f"success_steps={int(successes.sum())}, has_terminal_state={env.has_terminal}", flush=True)
    finally:
        if env is not None:
            env.close_dataset()
        if og.app is not None:
            og.shutdown()
    print(f"Exported ADEPT episode {episode_id}: {destination}")
    return episode_id
