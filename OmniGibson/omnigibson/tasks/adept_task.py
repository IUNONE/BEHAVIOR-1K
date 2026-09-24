import math

import torch as th
import omnigibson.utils.transform_utils as T
from omnigibson.termination_conditions.predicate_goal import PredicateGoal

import omnigibson as og
import omnigibson.lazy as lazy

from omnigibson.tasks.behavior_task import BehaviorTask


class ADEPTTask(BehaviorTask):
    """结合 BDDL 目标与配置的书本姿态约束判定 ADEPT 任务成功."""

    def __init__(self, activity_name, parameters, activity_instance_id=1, termination_config=None, include_obs=False, book_pose_goal=None):
        """保存实例元数据并初始化原生任务接口."""
        self.parameters = parameters
        self.book_pose_goal = book_pose_goal
        super().__init__(activity_name=activity_name, activity_instance_id=activity_instance_id,
                         termination_config=termination_config, include_obs=include_obs,
                         online_object_sampling=False, use_presampled_robot_pose=True)

    def check_goal(self):
        """联合检查 BDDL 目标和书本立放及书脊朝外条件."""
        success, status = self.compiled_task.check_goal(self._evaluate_predicate)
        if self.book_pose_goal is None:
            return success, status
        config = self.book_pose_goal
        book = self.object_scope["book.n.02_1"]
        bookcase = self.object_scope["bookcase.n.01_1"]
        rotation = T.quat2mat(book.get_position_orientation()[1])
        cabinet_rotation = T.quat2mat(bookcase.get_position_orientation()[1])
        long_axis = rotation @ rotation.new_tensor(config["long_axis"])
        spine_axis = rotation @ rotation.new_tensor(config["spine_outward_axis"])
        outward = cabinet_rotation @ cabinet_rotation.new_tensor(config["cabinet_outward_axis"])
        upright = abs(float(long_axis[2])) >= math.cos(math.radians(config["upright_tolerance_degrees"]))
        facing = float(th.dot(spine_axis, outward)) >= math.cos(math.radians(config["facing_tolerance_degrees"]))
        status = {key: list(value) for key, value in status.items()}
        first_index = len(status["satisfied"]) + len(status["unsatisfied"])
        for index, satisfied in enumerate((upright, facing), start=first_index):
            status["satisfied" if satisfied else "unsatisfied"].append(index)
        return bool(success and upright and facing), status

    def _create_termination_conditions(self):
        """让原生成功终止条件使用联合目标判定."""
        conditions = super()._create_termination_conditions()
        conditions["predicate"] = PredicateGoal(check_goal_fn=self.check_goal)
        return conditions

    def get_potential(self, env):
        """根据联合目标的满足比例计算势函数."""
        _, status = self.check_goal()
        return -len(status["satisfied"]) / (len(status["satisfied"]) + len(status["unsatisfied"]))

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
