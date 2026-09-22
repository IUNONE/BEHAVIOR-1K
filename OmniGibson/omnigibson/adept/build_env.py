import argparse
import math
import random
import shutil
import tempfile
from collections import Counter
from copy import deepcopy
from itertools import combinations
from pathlib import Path

import torch as th

import omnigibson as og
import omnigibson.utils.transform_utils as T
from bddl.condition_evaluation import evaluate_state
from omnigibson.adept import TASK_NAMES
from omnigibson.adept.environment import (
    build_environment_config, camera_config, configure_robot_physics, initial_report,
    load_instance, load_task_config, make_hold_action, robot_config, task_directory, write_json,
)
from omnigibson.adept.scene import ADEPTScene
from omnigibson.macros import gm
from omnigibson.object_states import Open
from omnigibson.utils.bddl_utils import evaluate_bddl_predicate, get_knowledge_base
from omnigibson.utils.usd_utils import RigidContactAPI


def place_object(env, name, placement):
    """根据支撑面高度放置对象并设置明确的关节初态."""
    obj = env.scene.object_registry("name", name)
    position = th.tensor([*placement["xy"], 1.5], dtype=th.float32)
    obj.set_position_orientation(position, th.tensor(placement["orientation"], dtype=th.float32))
    if placement.get("closed", False):
        _, joints, directions = obj.states[Open].relevant_joints_info
        for joint, direction in zip(joints, directions):
            closed = joint.lower_limit if direction == 1 else joint.upper_limit
            joint.set_pos(th.tensor([closed]), drive=False)
    support = placement.get("support")
    target_z = float(env.scene.object_registry("name", support).aabb[1][2]) + placement.get("gap", 0.003) if support else placement["bottom_z"]
    position[2] += target_z - float(obj.aabb[0][2])
    obj.set_position_orientation(position)
    obj.keep_still()


def resolve_parameters(config, instance):
    """保存实例说明, 采样检查阈值和任务指令."""
    parameters = deepcopy(instance["parameters"])
    parameters["thresholds"] = deepcopy(config["thresholds"])
    parameters["instruction"] = config["instruction"]
    return parameters


def motion_stable(obj, thresholds):
    """检查初态中对象各刚体连杆的线速度和角速度."""
    return all(
        float(th.linalg.vector_norm(link.get_linear_velocity())) <= thresholds["linear_speed"]
        and float(th.linalg.vector_norm(link.get_angular_velocity())) <= thresholds["angular_speed"]
        for link in obj.links.values()
    )


def sample_placements(config, instance_id, seed, max_attempts):
    """在创建仿真环境前生成指定实例的候选位置与竖直轴旋转."""
    rng = random.Random(seed + instance_id)
    candidates = []
    for _ in range(max_attempts):
        placements = deepcopy(config["initial_state"]["placements"])
        for name, bounds in config["sampling"].items():
            low, high = bounds["xy_bounds"]
            yaw = rng.uniform(*bounds["yaw_degrees"])
            rotation = T.euler2quat(th.tensor([0., 0., math.radians(yaw)], dtype=th.float32))
            orientation = T.quat_multiply(rotation, th.tensor(placements[name]["orientation"], dtype=th.float32))
            placements[name].update({
                "xy": [rng.uniform(low[axis], high[axis]) for axis in range(2)],
                "orientation": orientation.tolist(),
                "yaw_offset_degrees": yaw,
            })
        candidates.append(placements)
    return candidates


def rotation_distance(first, second):
    """计算两个四元数之间的最小旋转角, 单位为度."""
    relative = T.quat_distance(first, second)
    return math.degrees(2 * math.atan2(float(th.linalg.vector_norm(relative[:3])), abs(float(relative[3]))))


def check_layout(objects, table, thresholds, obstacles=None):
    """检查桌面边界以及任务物体和固定桌面设施的间距."""
    margin = thresholds["sampling_table_margin"]
    low, high = table.aabb
    entities = {**(obstacles or {}), **objects}
    boxes = {name: obj.aabb for name, obj in entities.items()}
    for name, (obj_low, obj_high) in boxes.items():
        if not bool(((obj_low[:2] >= low[:2] + margin) & (obj_high[:2] <= high[:2] - margin)).all()):
            return f"{name}: outside table footprint"
    for first, second in combinations(boxes, 2):
        low_a, high_a = boxes[first]
        low_b, high_b = boxes[second]
        gap = th.maximum(low_a[:2] - high_b[:2], low_b[:2] - high_a[:2]).clamp_min(0)
        if float(th.linalg.vector_norm(gap)) < thresholds["sampling_object_clearance"]:
            return f"{first}/{second}: insufficient clearance"
    return None


def validate_candidate(env, objects, table, config, compiled, predicate, robot_pose, robot_joints, action, obstacles):
    """在仿真中稳定候选初态并检查谓词, 接触和持续稳定性."""
    thresholds = config["thresholds"]
    error = check_layout(objects, table, thresholds, obstacles)
    if error:
        return error
    proposed = {name: obj.get_position_orientation()[1].clone() for name, obj in objects.items()}
    for _ in range(config["settle_steps"]):
        env.step(action)
    baseline = {name: tuple(value.clone() for value in obj.get_position_orientation()) for name, obj in objects.items()}
    for name, (_, orientation) in baseline.items():
        if rotation_distance(orientation, proposed[name]) > thresholds["sampling_rotation_degrees"]:
            return f"{name}: changed support orientation while settling"
    for step in range(config["sampling_check_steps"] + 1):
        if step:
            env.step(action)
        valid, _ = evaluate_state(compiled.initial_conditions, predicate)
        if not valid:
            return "BDDL initial conditions failed"
        goal, _ = compiled.check_goal(predicate)
        if goal:
            return "goal already satisfied"
        error = check_layout(objects, table, thresholds, obstacles)
        if error:
            return error
        for name, obj in objects.items():
            if RigidContactAPI.is_in_contact(env.scene.idx, [obj], None, [obj, table], current_only=True):
                return f"{name}: contact outside supporting table"
            if not motion_stable(obj, thresholds):
                return f"{name}: not stable"
            position, orientation = obj.get_position_orientation()
            if float(th.linalg.vector_norm(position - baseline[name][0])) > thresholds["initial_object_drift"]:
                return f"{name}: position drift"
            if rotation_distance(orientation, baseline[name][1]) > thresholds["sampling_rotation_degrees"]:
                return f"{name}: rotation drift"
        robot = env.robots[0]
        position, orientation = robot.get_position_orientation()
        if float(th.linalg.vector_norm(position - robot_pose[0])) > thresholds["base_drift"]:
            return "robot base drift"
        if rotation_distance(orientation, robot_pose[1]) > thresholds["sampling_rotation_degrees"]:
            return "robot base rotation"
        if float((robot.get_joint_positions() - robot_joints).abs().max()) > 0.03:
            return "robot joint drift"
    return None


def build(task_name, instance_ids, overwrite=False, seed=0, max_attempts=50):
    """自动采样, 构建和验证全部实例后保存共用模板与原生状态."""
    config = load_task_config(task_name)
    directory = task_directory(task_name)
    if directory.exists() and not overwrite:
        raise FileExistsError(f"{directory} exists. Use --overwrite to rebuild the task.")
    if (directory / "instances").exists():
        instance_ids = list(set(instance_ids) | {int(path.name) for path in (directory / "instances").iterdir() if path.is_dir()})
    instance_ids = sorted(set(instance_ids))
    if not instance_ids or instance_ids[0] < 1 or max_attempts < 1:
        raise ValueError("Instance IDs and max_attempts must be positive.")
    initial = config["initial_state"]
    if set(config["sampling"]) != set(initial["parameters"]["manipulated_objects"]):
        raise ValueError("sampling must specify exactly the task's manipulated_objects.")
    for name, bounds in config["sampling"].items():
        low, high = bounds["xy_bounds"]
        if any(a >= b for a, b in zip(low, high)) or bounds["yaw_degrees"][0] >= bounds["yaw_degrees"][1]:
            raise ValueError(f"Invalid sampling bounds for {name}.")
    candidates = {index: sample_placements(config, index, seed, max_attempts) for index in instance_ids}
    cfg = {
        "env": {**config["frequencies"], "external_sensors": [camera_config(config)]},
        "scene": {"type": ADEPTScene.__name__, "use_floor_plane": False},
        "robots": [robot_config(config, "collection")],
        "objects": deepcopy(config["room_objects"] + [config["table"]] + config["objects"]),
        "task": {"type": "DummyTask", "include_obs": False},
    }
    env = og.Environment(configs=cfg)
    configure_robot_physics(env)
    place_object(env, "work_table", {"xy": config["table"]["position"][:2], "bottom_z": 0.,
                                     "orientation": config["table"]["orientation"]})
    for name, placement in config["placements"].items():
        place_object(env, name, placement)
    env.scene.write_task_metadata("inst_to_name", config["bindings"])
    robot = env.robots[0]
    robot.reset()
    robot.set_position_orientation(th.tensor(config["robot"]["position"]), th.tensor(config["robot"]["orientation"]))
    robot.keep_still()
    robot_pose = tuple(value.clone() for value in robot.get_position_orientation())
    robot_joints = robot.get_joint_positions().clone()
    robot_poses = {"robot": [{"position": robot_pose[0], "orientation": robot_pose[1]}]}
    env.scene.write_task_metadata("robot_poses", robot_poses)
    env.scene.update_initial_file()
    baseline = deepcopy(env.scene.dump_state(serialized=False))
    hold = make_hold_action(robot)
    scope = {key: env.scene.object_registry("name", name) for key, name in config["bindings"].items()}
    objects = {name: env.scene.object_registry("name", name) for name in config["sampling"]}
    table = env.scene.object_registry("name", "work_table")
    obstacles = {name: env.scene.object_registry("name", name) for name, placement in config["placements"].items()
                 if placement.get("support") == "work_table"}
    error = check_layout(obstacles, table, config["thresholds"])
    if error:
        raise ValueError(f"Invalid fixed tabletop layout: {error}")
    compiled = get_knowledge_base().get_task(f"{task_name}-0").compile(scene_layout={})

    def predicate(predicate_cls, *names):
        """将 BDDL 对象名称绑定到当前仿真实体进行求值."""
        return evaluate_bddl_predicate(predicate_cls, *[scope[name] for name in names])

    fixed_states = {name: obj.dump_state(serialized=False) for name, obj in scope.items()
                    if obj.name not in objects and obj is not robot}
    directory.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="adept-build-", dir=directory.parent) as temporary:
        staging = Path(temporary)
        for index, instance_id in enumerate(instance_ids):
            th.manual_seed(seed + instance_id)
            failures = Counter()
            for attempt, placements in enumerate(candidates[instance_id], start=1):
                env.reset(get_obs=False)
                env.scene.load_state(deepcopy(baseline), serialized=False)
                for name, placement in placements.items():
                    place_object(env, name, placement)
                error = validate_candidate(env, objects, table, config, compiled, predicate, robot_pose, robot_joints, hold, obstacles)
                if error:
                    failures[error] += 1
                    continue
                parameters = resolve_parameters(config, initial)
                parameters["sampling"] = {
                    "seed": seed, "instance_seed": seed + instance_id, "accepted_attempt": attempt,
                    "ranges": deepcopy(config["sampling"]), "proposed_placements": placements,
                    "settled_poses": {name: {"position": obj.get_position_orientation()[0],
                                             "orientation": obj.get_position_orientation()[1]}
                                      for name, obj in objects.items()},
                }
                states = deepcopy(fixed_states)
                states.update({name: obj.dump_state(serialized=False) for name, obj in scope.items() if obj.name in objects})
                states["robot_poses"] = robot_poses
                env.scene.write_task_metadata("adept", {"task_name": task_name, "instance_id": instance_id, "parameters": parameters})
                if index == 0:
                    env.scene.save(json_path=str(staging / "scene_template.json"))
                output = staging / "instances" / str(instance_id)
                write_json(output / "tro_state.json", states)
                write_json(output / "task_parameters.json", parameters)
                print(f"Validated {task_name} instance {instance_id}, attempt {attempt}")
                break
            else:
                raise RuntimeError(f"Instance {instance_id} failed after {max_attempts} attempts: {dict(failures)}")
        og.clear()
        env = og.Environment(configs=build_environment_config(task_name, "collection", instance_ids[0], task_dir=staging))
        configure_robot_physics(env)
        for instance_id in instance_ids:
            load_instance(env, task_name, instance_id, task_dir=staging)
            report = initial_report(env)
            if not report["initial_conditions_valid"] or report["initial_goal_satisfied"]:
                raise RuntimeError(f"Instance {instance_id} failed saved-state validation: {report}")
        shutil.copytree(staging, directory, dirs_exist_ok=True)
    print(f"Built {task_name}: {len(instance_ids)} instances in {directory}")


def main():
    """解析远程场景生成参数并释放仿真资源."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--task-name", choices=TASK_NAMES, required=True)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--instance-indices", nargs="+", type=int)
    selection.add_argument("--num-instances", type=int, help="Sample instance IDs 1 through N; default: 1.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-attempts", type=int, default=50)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    args = parser.parse_args()
    if args.num_instances is not None and args.num_instances < 1:
        parser.error("--num-instances must be positive")
    instance_ids = args.instance_indices or list(range(1, (args.num_instances or 1) + 1))
    gm.HEADLESS = args.headless
    gm.ENABLE_TRANSITION_RULES = False
    gm.USE_GPU_DYNAMICS = False
    try:
        build(args.task_name, instance_ids, args.overwrite, args.seed, args.max_attempts)
    finally:
        if og.app is not None:
            og.shutdown()


if __name__ == "__main__":
    main()
