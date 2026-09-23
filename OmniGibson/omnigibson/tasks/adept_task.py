import omnigibson as og
import omnigibson.lazy as lazy

from omnigibson.tasks.behavior_task import BehaviorTask


class ADEPTTask(BehaviorTask):
    """保留 ADEPT 实例元数据并使用原生 BDDL 成功判定."""

    def __init__(self, activity_name, parameters, activity_instance_id=1, termination_config=None, include_obs=False):
        """保存实例元数据并初始化原生任务接口."""
        self.parameters = parameters
        super().__init__(activity_name=activity_name, activity_instance_id=activity_instance_id,
                         termination_config=termination_config, include_obs=include_obs,
                         online_object_sampling=False, use_presampled_robot_pose=True)

    @classmethod
    def verify_scene_and_task_config(cls, scene_cfg, task_cfg):
        """要求使用显式保存的 ADEPT 场景文件."""
        if not scene_cfg.get("scene_file"):
            raise ValueError("ADEPTTask requires scene_file. Run omnigibson.adept.build_env first.")

    def _load(self, env):
        """加载 BDDL 绑定并设置自定义场景标识."""
        super()._load(env)
        self.scene_name = "adept_room"

    def reset(self, env):
        """恢复实例初态与画面, 并在渲染更新后重新绑定物理句柄, 不推进物理步."""
        super().reset(env)
        og.sim.sync_physx_to_fabric()
        env.external_sensors["table_side"].reset_render_product()
        lazy.omni.usd.get_context().reset_renderer_accumulation()
        for _ in range(32):
            og.sim.render()
        og.sim.update_handles()
