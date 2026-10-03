import importlib
from contextlib import ExitStack
from pathlib import Path

import torch as th

import omnigibson as og
import omnigibson.utils.transform_utils as T
from omnigibson.adept.environment import (
    build_environment_config, configure_robot_physics, constrain_action, load_instance,
    load_task_config, make_hold_action, task_directory, write_json,
)
from omnigibson.adept.preview_video_utils import compose_instance_previews, list_instance_ids
from omnigibson.adept.recording import ADEPTCollectionWrapper
from omnigibson.adept.visuals import CAMERA_LINKS, VideoWriter, frames, robot_camera_row
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
    """执行 ADEPT 评估, 保存逐回合模型记录, 仿真状态与汇总视频."""
    gm.HEADLESS = args.headless
    gm.USE_GPU_DYNAMICS = False
    gm.ENABLE_TRANSITION_RULES = False
    config = load_task_config(args.task_name)
    max_steps = config["max_steps"] if args.max_steps is None else args.max_steps
    if max_steps < 1 or args.num_rollouts < 1:
        raise ValueError("max_steps and num_rollouts must be positive.")
    args.instance_indices = (
        list_instance_ids(task_directory(args.task_name))
        if args.all_instances else sorted(set(args.instance_indices))
    )
    output_dir = Path(args.output_dir).expanduser()
    run_name = output_dir.name
    for instance_id in args.instance_indices:
        for rollout in range(args.num_rollouts):
            destination = output_dir / f"instance_{instance_id:03d}" / f"rollout_{rollout:03d}"
            if destination.exists():
                raise FileExistsError(f"Rollout output already exists: {destination}. Use a new run_name.")
    results = []
    videos, success_steps = {}, {}
    policy = None
    try:
        env = og.Environment(configs=build_environment_config(args.task_name, "evaluation", args.instance_indices[0], max_steps))
        configure_robot_physics(env)
        control_hz = env.env_config["action_frequency"]
        if args.write_video and args.video_fps != control_hz:
            raise ValueError("ADEPT video_fps must equal action_frequency to align video and simulation states.")
        policy = build_policy(args, env)
        for instance_id in args.instance_indices:
            load_instance(env, args.task_name, instance_id)
            for rollout in range(args.num_rollouts):
                rollout_dir = output_dir / f"instance_{instance_id:03d}" / f"rollout_{rollout:03d}"
                rollout_dir.mkdir(parents=True)
                identity = {
                    "task": args.task_name, "run_name": run_name,
                    "instance_id": instance_id, "rollout_id": rollout,
                    "source_episode_id": f"policy/{args.task_name}/{run_name}/instance_{instance_id:03d}/rollout_{rollout:03d}",
                }
                first_success_step = -1
                with ExitStack() as cleanup:
                    active_env = ADEPTCollectionWrapper(
                        env, str(rollout_dir / "sim_state.hdf5"), source_episode_id=identity["source_episode_id"],
                        record_type="policy_execution", only_successes=False, overwrite=False,
                        viewport_camera_path=None, enable_dump_filters=False,
                    )
                    cleanup.callback(active_env.save_data)
                    obs, _ = active_env.reset()
                    initial_base = env.robots[0].get_position_orientation()[0].clone()
                    if policy is not None:
                        policy.reset()
                    if policy is not None and hasattr(policy, "begin_recording"):
                        cleanup.callback(policy.end_recording)
                        policy.begin_recording(rollout_dir, identity)
                    if args.write_video:
                        robot_writer = VideoWriter(rollout_dir / "videos" / "robot_cams.mp4", control_hz)
                        cleanup.callback(robot_writer.close)
                        side_writer = VideoWriter(rollout_dir / "videos" / "table_side_cam.mp4", control_hz)
                        cleanup.callback(side_writer.close)
                        images = frames(env)
                        robot_writer.write(robot_camera_row(images))
                        side_writer.write(images["table_side"])
                    for step in range(max_steps):
                        if policy is None:
                            action = make_hold_action(env.robots[0])
                        elif getattr(args, "policy_adapter", None):
                            action = policy.forward()
                        else:
                            action = policy.forward(policy_observation(env, obs))
                        action = constrain_action(action, env.robots[0])
                        obs, reward, terminated, truncated, info = active_env.step(action)
                        if policy is not None and hasattr(policy, "observe_execution"):
                            policy.observe_execution()
                        if policy is not None and hasattr(policy, "record_outcome"):
                            policy.record_outcome(reward, terminated, truncated, info)
                        success = bool(info["done"]["success"])
                        if success and first_success_step < 0:
                            first_success_step = step + 1
                        if args.write_video:
                            images = frames(env)
                            robot_writer.write(robot_camera_row(images))
                            side_writer.write(images["table_side"])
                        if success or terminated or truncated:
                            break
                    reason = "success" if success else "terminated" if terminated else "truncated" if truncated else "max_steps"
                    result = {"task": args.task_name, "instance_id": instance_id, "rollout_id": rollout,
                              "steps": step + 1, "success": success,
                              "source_episode_id": identity["source_episode_id"],
                              "first_success_step_idx": first_success_step, "termination_reason": reason,
                              "goal_status": info["done"]["goal_status"],
                              "base_displacement": float(th.linalg.vector_norm(env.robots[0].get_position_orientation()[0] - initial_base)),
                              "timed_out": bool((truncated or reason == "max_steps") and not success)}
                    if policy is not None and hasattr(policy, "execution_stats"):
                        result.update(policy.execution_stats())
                    results.append(result)
                    write_json(rollout_dir / "result.json", result)
                    if policy is not None and hasattr(policy, "end_recording"):
                        policy.end_recording(reason)
                if args.write_video:
                    key = (instance_id, rollout)
                    videos[key] = rollout_dir / "videos" / "table_side_cam.mp4"
                    success_steps[key] = first_success_step
        write_json(output_dir / "summary.json", {
            "task": args.task_name, "run_name": run_name,
            "num_rollouts": len(results), "num_success": sum(result["success"] for result in results),
            "success_rate": sum(result["success"] for result in results) / len(results),
            "rollouts": results,
        })
        if videos:
            preview = compose_instance_previews(
                videos, output_dir / "all_preview_videos.mp4", layout=args.layout or "3x4",
                success_steps=success_steps,
            )
            print(f"Summary preview: {preview}")
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
    finally:
        try:
            if policy is not None and hasattr(policy, "close"):
                policy.close()
        finally:
            if og.app is not None:
                og.shutdown()
