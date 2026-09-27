import importlib
from pathlib import Path

import torch as th

import omnigibson as og
import omnigibson.utils.transform_utils as T
from omnigibson.adept.environment import (
    build_environment_config, configure_robot_physics, constrain_action, load_instance,
    load_task_config, make_hold_action, write_json,
)
from omnigibson.adept.recording import ADEPTCollectionWrapper
from omnigibson.adept.visuals import CAMERA_LINKS
from omnigibson.macros import gm


def policy_observation(env, obs):
    """构造原生关节动作策略使用的平铺观测与任务指令."""
    from omnigibson.utils.gym_utils import recursively_generate_flat_dict

    robot = env.robots[0]
    flat = recursively_generate_flat_dict(obs)
    flat["instruction"] = env.task.parameters["instruction"]
    flat["task_name"] = env.task.activity_name
    flat["instance_id"] = env.task.activity_instance_id
    base_pose = robot.get_position_orientation()
    poses = []
    for role in ("left_wrist", "right_wrist", "head"):
        sensor = robot.sensors[f"{robot.name}:{CAMERA_LINKS[role]}:Camera:0"]
        poses.append(th.cat(T.relative_pose_transform(*sensor.get_position_orientation(), *base_pose)))
    flat[f"{robot.name}::cam_rel_poses"] = th.cat(poses)
    return flat


def build_policy(args, env):
    """创建进程内策略适配器或原有 websocket 策略.

    参数
    ----
    args : argparse.Namespace
        评估命令参数.
    env : omnigibson.envs.Environment
        已加载的 ADEPT 环境.

    返回
    ----
    object or None
        带有 reset 与 forward 的策略. 未指定策略时返回 None.
    """
    adapter_spec = getattr(args, "policy_adapter", None)
    if adapter_spec:
        module_name, separator, class_name = str(adapter_spec).partition(":")
        if not separator or not module_name or not class_name:
            raise ValueError("policy adapter must use module:Class.")
        policy_cls = getattr(importlib.import_module(module_name), class_name)
        return policy_cls(
            env,
            config_path=getattr(args, "policy_config", None),
            host=args.host,
            port=args.port,
        )
    if args.policy == "websocket":
        from omnigibson.eval.policies import WebsocketPolicy

        return WebsocketPolicy(
            host="127.0.0.1" if args.host is None else args.host,
            port=8000 if args.port is None else args.port,
        )
    return None


def run(args):
    """执行 ADEPT 单环境评估并保存结果及可选离线回放记录."""
    gm.HEADLESS = args.headless
    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_TRANSITION_RULES = False
    config = load_task_config(args.task_name)
    max_steps = config["max_steps"] if args.max_steps is None else args.max_steps
    if max_steps < 1 or args.num_rollouts < 1:
        raise ValueError("max_steps and num_rollouts must be positive.")
    results = []
    policy = None
    try:
        env = og.Environment(configs=build_environment_config(args.task_name, "evaluation", args.instance_indices[0], max_steps))
        configure_robot_physics(env)
        policy = build_policy(args, env)
        for instance_id in args.instance_indices:
            load_instance(env, args.task_name, instance_id)
            for rollout in range(args.num_rollouts):
                obs, _ = env.reset()
                initial_base = env.robots[0].get_position_orientation()[0].clone()
                active_env = env
                if args.write_video:
                    recording = Path(args.output_dir).expanduser() / "recordings" / f"{args.task_name}_{instance_id}_{rollout}.hdf5"
                    active_env = ADEPTCollectionWrapper(env, str(recording), only_successes=False,
                                                       viewport_camera_path=None, enable_dump_filters=False)
                    obs, _ = active_env.reset()
                if policy is not None:
                    policy.reset()
                try:
                    for step in range(max_steps):
                        if policy is None:
                            action = make_hold_action(env.robots[0])
                        elif getattr(args, "policy_adapter", None):
                            action = policy.forward()
                        else:
                            action = policy.forward(policy_observation(env, obs))
                        action = constrain_action(action, env.robots[0])
                        obs, _, terminated, truncated, info = active_env.step(action)
                        if policy is not None and hasattr(policy, "observe_execution"):
                            policy.observe_execution()
                        if terminated or truncated:
                            break
                    result = {"task": args.task_name, "instance_id": instance_id, "rollout_id": rollout,
                              "steps": step + 1, "success": bool(env.task.success),
                              "goal_status": info["done"]["goal_status"],
                              "base_displacement": float(th.linalg.vector_norm(env.robots[0].get_position_orientation()[0] - initial_base)),
                              "timed_out": bool(truncated and not env.task.success)}
                    if policy is not None and hasattr(policy, "execution_stats"):
                        result.update(policy.execution_stats())
                    results.append(result)
                    write_json(Path(args.output_dir).expanduser() / "json" / f"{args.task_name}_{instance_id}_{rollout}.json", result)
                finally:
                    if args.write_video:
                        active_env.save_data()
                if args.write_video:
                    print(f"Replay after evaluation: python -B joylo/scripts/replay_data.py {recording} --task {args.task_name} --episode-id 0")
    finally:
        try:
            if policy is not None and hasattr(policy, "close"):
                policy.close()
        finally:
            if og.app is not None:
                og.shutdown()
    ik_text = ""
    if any("ik_failures" in result for result in results):
        ik_failures = sum(result.get("ik_failures", 0) for result in results)
        tracked = [
            result["max_tracking_position_error_m"]
            for result in results
            if result.get("max_tracking_position_error_m") is not None
        ]
        tracking = f", max tracking position error {max(tracked):.4f} m" if tracked else ""
        ik_text = f", IK failures {ik_failures}{tracking}"
    print(f"ADEPT: {sum(result['success'] for result in results)}/{len(results)} successful rollouts{ik_text}")
