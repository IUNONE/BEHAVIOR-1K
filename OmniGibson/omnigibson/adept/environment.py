import json
from copy import deepcopy
from pathlib import Path

import torch as th
import yaml

import omnigibson as og
import omnigibson.utils.transform_utils as T
from omnigibson.adept import TASK_NAMES
from omnigibson.adept.scene import ADEPTScene
from omnigibson.macros import gm
from omnigibson.utils.config_utils import TorchEncoder
from omnigibson.utils.python_utils import recursively_convert_to_torch


def read_json(path):
    """读取场景或实例 JSON 文件."""
    with Path(path).open() as stream:
        return json.load(stream)


def write_json(path, data):
    """保存包含张量的场景或实例数据."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as stream:
        json.dump(data, stream, cls=TorchEncoder, indent=2, ensure_ascii=False)


def load_task_config(task_name):
    """读取共用环境配置与指定任务配置."""
    if task_name not in TASK_NAMES:
        raise ValueError(f"Unknown ADEPT task: {task_name}")
    directory = Path(__file__).parent / "configs"
    with (directory / "common.yaml").open() as stream:
        common = yaml.safe_load(stream)
    with (directory / f"{task_name}.yaml").open() as stream:
        task = yaml.safe_load(stream)
    return {**common, **task, "task_name": task_name}


def task_directory(task_name):
    """返回指定任务的生成文件目录."""
    return Path(gm.DATA_PATH) / "adept-task-instances" / task_name


def camera_config(config):
    """根据位置和观察目标构造固定第三视角相机."""
    camera = deepcopy(config["camera"])
    position = th.tensor(camera.pop("position"), dtype=th.float32)
    target = th.tensor(camera.pop("target"), dtype=th.float32)
    backward = th.nn.functional.normalize(position - target, dim=0)
    right = th.nn.functional.normalize(th.linalg.cross(th.tensor([0., 0., 1.]), backward), dim=0)
    upward = th.linalg.cross(backward, right)
    orientation = T.mat2quat(th.stack([right, upward, backward], dim=1))
    return {
        "sensor_type": "VisionSensor",
        "name": "table_side",
        "relative_prim_path": "/table_side",
        "modalities": ["rgb"],
        "sensor_kwargs": camera,
        "position": position.tolist(),
        "orientation": orientation.tolist(),
        "pose_frame": "scene",
    }


def robot_config(config, purpose):
    """构造与 JoyLo 一致的 R1Pro 控制器及相机配置."""
    path = Path(__file__).parents[1] / "eval" / "r1pro.yaml"
    with path.open() as stream:
        robot = yaml.safe_load(stream)
    robot.pop("eval")
    robot["name"] = "robot"
    robot.update(deepcopy(config["robot"]))
    robot["obs_modalities"] = [] if purpose == "collection" else ["proprio", "rgb"]
    robot["sensor_config"] = {f"{link}:Camera:0": {
        "modalities": ["rgb"],
        "sensor_kwargs": {"image_height": 480, "image_width": 480},
    } for link in ("zed_link", "left_realsense_link", "right_realsense_link")}
    robot["sensor_config"]["zed_link:Camera:0"]["sensor_kwargs"]["horizontal_aperture"] = 40.0
    return robot


def build_environment_config(task_name, purpose="check_env", instance_id=1, max_steps=None, task_dir=None):
    """创建使用已保存模板和实例参数的单环境配置."""
    from omnigibson.tasks.adept_task import ADEPTTask

    config = load_task_config(task_name)
    directory = task_directory(task_name) if task_dir is None else Path(task_dir)
    scene_path = directory / "scene_template.json"
    parameters = read_json(directory / "instances" / str(instance_id) / "task_parameters.json")
    sensor = camera_config(config)
    sensor["include_in_obs"] = purpose != "collection"
    return {
        "env": {**config["frequencies"], "external_sensors": [sensor]},
        "scene": {"type": ADEPTScene.__name__, "scene_file": str(scene_path), "include_robots": False, "use_floor_plane": False},
        "robots": [robot_config(config, purpose)],
        "task": {
            "type": ADEPTTask.__name__,
            "activity_name": task_name,
            "activity_instance_id": instance_id,
            "parameters": parameters,
            "termination_config": {"max_steps": config["max_steps"] if max_steps is None else max_steps},
            "include_obs": False,
        },
    }


def constrain_action(action, robot):
    """将传入环境的底盘速度分量置零."""
    action = action.clone()
    action[..., robot.base_action_idx] = 0
    return action


def make_hold_action(robot):
    """生成保持当前手臂和躯干姿态且夹爪打开的动作."""
    action = th.zeros(robot.action_dim, dtype=th.float32)
    joints = robot.get_joint_positions()
    for arm in robot.arm_names:
        action[robot.arm_action_idx[arm]] = joints[robot.arm_control_idx[arm]]
        action[robot.gripper_action_idx[arm]] = 1.0
    action[robot.trunk_action_idx] = joints[robot.trunk_control_idx]
    return constrain_action(action, robot)


def configure_robot_physics(env):
    """统一底盘质量, 连续碰撞设置和 JoyLo 头部相机位姿."""
    og.sim.stop()
    env.robots[0].base_footprint_link.mass = 250.0
    for obj in env.scene.objects:
        if not obj.kinematic_only:
            for link in obj.links.values():
                link.ccd_enabled = True
    og.sim.play()
    og.sim.update_handles()
    robot = env.robots[0]
    head = robot.sensors[f"{robot.name}:zed_link:Camera:0"]
    head.set_position_orientation(th.tensor([0.06, 0.0, 0.01]), th.tensor([-1., 0., 0., 0.]), frame="parent")


def load_instance(env, task_name, instance_id, task_dir=None):
    """恢复指定实例并更新后续 reset 使用的场景基线."""
    directory = (task_directory(task_name) if task_dir is None else Path(task_dir)) / "instances" / str(instance_id)
    parameters = read_json(directory / "task_parameters.json")
    states = recursively_convert_to_torch(read_json(directory / "tro_state.json"))
    robot = env.robots[0]
    env.scene.reset()
    robot.reset()
    for name, state in states.items():
        if name == "robot_poses":
            pose = state["robot"][0]
            robot.set_position_orientation(pose["position"], pose["orientation"])
            env.scene.write_task_metadata("robot_poses", state)
        else:
            env.task.object_scope[name].load_state(state, serialized=False)
    env.task.parameters = parameters
    env.task.activity_instance_id = int(instance_id)
    env.scene.write_task_metadata("adept", {"task_name": task_name, "instance_id": instance_id, "parameters": parameters})
    og.sim.update_handles()
    env.scene.update_initial_file()
    return env.reset()


def initial_report(env):
    """检查当前 BDDL 初始关系并记录对象与关节状态."""
    from bddl.condition_evaluation import evaluate_state

    task = env.task
    valid, conditions = evaluate_state(task.activity_initial_conditions, task._evaluate_predicate)
    goal, _ = task.compiled_task.check_goal(task._evaluate_predicate)
    objects = {}
    for name, obj in task.object_scope.items():
        objects[name] = {
            "name": obj.name,
            "category": obj.category,
            "model": getattr(obj, "model", None),
            "position": obj.get_position_orientation()[0],
            "orientation": obj.get_position_orientation()[1],
            "aabb": obj.aabb,
            "joints": {key: {"position": joint.get_state()[0], "lower_limit": joint.lower_limit,
                              "upper_limit": joint.upper_limit} for key, joint in obj.joints.items() if joint.articulated},
        }
    return {"initial_conditions_valid": bool(valid), "initial_conditions": conditions,
            "initial_goal_satisfied": bool(goal), "objects": objects}
