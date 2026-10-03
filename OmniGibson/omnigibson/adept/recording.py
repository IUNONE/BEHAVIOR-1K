from copy import deepcopy
from pathlib import Path

import h5py

import omnigibson as og
from omnigibson.envs.hdf5_data_wrapper import HDF5CollectionWrapper


class ADEPTCollectionWrapper(HDF5CollectionWrapper):
    """保存原生动作状态, BDDL 成功结果和实例元数据."""

    def __init__(self, *args, record_type="demonstration", source_episode_id=None, **kwargs):
        """登记专家演示或策略执行身份后初始化原生记录器."""
        self.record_type = record_type
        self.source_episode_id = source_episode_id
        super().__init__(*args, **kwargs)

    def create_dataset(self, output_path, env, overwrite=True):
        """创建包含场景和任务配置的记录文件."""
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        self.hdf5_file = h5py.File(output_path, "w" if overwrite else "a")
        if self.source_episode_id is not None:
            self.hdf5_file.attrs["source_episode_id"] = self.source_episode_id
        self.recording_id = Path(output_path).stem
        group = self.hdf5_file.require_group("data")
        env.task.update_bddl_scope_metadata(env)
        config = deepcopy(env.config)
        config["task"]["activity_instance_id"] = env.task.activity_instance_id
        config["task"]["parameters"] = deepcopy(env.task.parameters)
        self.add_metadata(group, "config", config)
        self.add_metadata(group, "scene_file", env.scene.save())
        self.add_metadata(group, "adept", env.scene.get_task_metadata("adept"))

    def _parse_step_data(self, action, obs, reward, terminated, truncated, info):
        """将执行后的 BDDL 成功结果与当前动作共同保存."""
        data = super()._parse_step_data(action, obs, reward, terminated, truncated, info)
        data["success"] = bool(info["done"]["success"])
        return data

    def _process_traj_to_hdf5(self, traj_data, traj_grp_name, nested_keys=("obs",), data_grp=None):
        """写入原生轨迹及实例编号和参数."""
        terminal = traj_data[-1]["state"][:int(traj_data[-1]["state_size"])].clone()
        group = super()._process_traj_to_hdf5(traj_data, traj_grp_name, nested_keys, data_grp)
        group.create_dataset("terminal_state", data=terminal.cpu().numpy())
        task = self.env.task.activity_name
        name = f"episode_{self.recording_id}_d{int(traj_grp_name[5:]):06d}" if traj_grp_name.startswith("demo_") else f"{self.recording_id}/{traj_grp_name}"
        self.add_metadata(group, "source_episode_id", self.source_episode_id or f"robot/{task}/{name}")
        self.add_metadata(group, "record_type", self.record_type)
        self.add_metadata(group, "instance_id", self.env.task.activity_instance_id)
        self.add_metadata(group, "task_parameters", self.env.task.parameters)
        return group

    def close_dataset(self):
        """关闭回合文件并解除录制回调, 供后续实例复用环境."""
        try:
            super().close_dataset()
        finally:
            og.sim.remove_callback_on_system_init("data_collection")
            og.sim.remove_callback_on_system_clear("data_collection")
            og.sim.remove_callback_on_add_obj("data_collection")
            og.sim.remove_callback_on_remove_obj("data_collection")

    @property
    def should_save_current_episode(self):
        """仅保存包含实际动作的录制回合."""
        return len(self.current_traj_history) > 1 and super().should_save_current_episode
