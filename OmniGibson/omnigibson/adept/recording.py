from copy import deepcopy
from pathlib import Path

import h5py

from omnigibson.envs.hdf5_data_wrapper import HDF5CollectionWrapper


class ADEPTCollectionWrapper(HDF5CollectionWrapper):
    """保存原生动作状态, BDDL 成功结果和实例元数据."""

    def create_dataset(self, output_path, env, overwrite=True):
        """创建包含场景和任务配置的记录文件."""
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        self.hdf5_file = h5py.File(output_path, "w" if overwrite else "a")
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
        group = super()._process_traj_to_hdf5(traj_data, traj_grp_name, nested_keys, data_grp)
        self.add_metadata(group, "instance_id", self.env.task.activity_instance_id)
        self.add_metadata(group, "task_parameters", self.env.task.parameters)
        return group
