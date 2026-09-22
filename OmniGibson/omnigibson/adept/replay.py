import json
from pathlib import Path

import h5py
import numpy as np

import omnigibson as og
from omnigibson.adept.environment import configure_robot_physics, write_json
from omnigibson.adept.scene import ADEPTScene
from omnigibson.adept.visuals import CAMERA_LINKS, mosaic
from omnigibson.envs.data_wrapper import DataPlaybackWrapper
from omnigibson.eval.utils.obs_utils import create_video_writer, write_video
from omnigibson.macros import gm


class ADEPTPlaybackWrapper(DataPlaybackWrapper):
    """使用原生状态回放导出四路视频及动作帧对应关系."""

    def create_dataset(self, output_path, env, overwrite=True):
        """创建视频目录并保留输入 HDF5 文件."""
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)

    def close_dataset(self):
        """关闭输入文件和仍打开的视频编码流."""
        self._flush_video_writers()
        self.input_hdf5.close()

    def _create_video_writers(self, video_keys):
        """创建四路独立视频及拼接预览的视频流."""
        self.frame_map = []
        self.camera_keys = {role: f"robot::robot:{link}:Camera:0::rgb" for role, link in CAMERA_LINKS.items()}
        self.camera_keys["table_side"] = "external::table_side::rgb"
        for role, key in self.camera_keys.items():
            shape = self.env.observation_space[key].shape
            container, stream = create_video_writer(str(Path(self.video_output_dir) / f"{role}.mp4"), shape[:2], rate=self.fps)
            self.video_writers.append((container, stream, role))
        container, stream = create_video_writer(str(Path(self.video_output_dir) / "preview.mp4"), (960, 1280), rate=self.fps)
        self.video_writers.append((container, stream, "preview"))

    def _write_video_frames(self):
        """写入同步相机帧并登记原生回放的状态及动作索引."""
        images = {role: self.current_obs[key][..., :3].detach().cpu().numpy() for role, key in self.camera_keys.items()}
        images["preview"] = mosaic(images)
        for container, stream, role in self.video_writers:
            write_video(images[role][np.newaxis], (container, stream), mode="rgb")
        frame_index = len(self.frame_map)
        source_index = max(0, frame_index - 1)
        self.frame_map.append({"frame_index": frame_index, "source_state_index": source_index,
                               "action_index": None if frame_index == 0 else source_index,
                               "source_time_seconds": source_index / self.fps,
                               "phase": "initial_state" if frame_index == 0 else "state_restore_then_action_render"})


def replay(input_path, task_name, episode_id=None, run_qa=False):
    """回放指定演示并导出视频及执行时记录的任务检查结果."""
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
        group = source[f"data/demo_{episode_id}"]
        successes = group["success"][:].astype(bool).tolist()
        instance_id = int(group.attrs["instance_id"])
    directory = path.parent / f"{path.stem}_episode_{episode_id}"
    sensors = config["env"]["external_sensors"]
    for sensor in sensors:
        sensor["include_in_obs"] = True
    env = None
    try:
        env = ADEPTPlaybackWrapper.create_from_hdf5(
            input_path=str(path), output_path=str(directory / "observations.hdf5"),
            include_task=False, include_contacts=True, include_robot_control=False,
            robot_obs_modalities=["rgb"], external_sensors_config=sensors,
            n_render_iterations=1, only_successes=False,
        )
        configure_robot_physics(env)
        env.playback_episode(episode_id, record_data=False, video_keys={"adept": "preview"})
        write_json(directory / "frame_indices.json", env.frame_map)
        if run_qa:
            write_json(directory / "qa.json", {"task_name": task_name, "instance_id": instance_id,
                                               "episode_id": episode_id, "source": "recorded_execution",
                                               "success": successes[-1], "success_by_step": successes})
    finally:
        if env is not None:
            env.close_dataset()
        if og.app is not None:
            og.shutdown()
    print(f"Rendered ADEPT episode {episode_id}: {directory}")
    return episode_id
