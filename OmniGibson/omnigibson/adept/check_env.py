from pathlib import Path
import sys
import traceback

import torch as th

import omnigibson as og
from omnigibson.adept.environment import (
    build_environment_config, configure_robot_physics, initial_report, load_instance,
    load_task_config, make_hold_action,
)
from omnigibson.adept.visuals import VideoWriter, frames, mosaic
from omnigibson.macros import gm


def run(args):
    """恢复指定实例, 检查稳定性并保存四路拼接预览视频."""
    gm.HEADLESS = args.headless
    gm.ENABLE_TRANSITION_RULES = False
    gm.USE_GPU_DYNAMICS = False
    config = load_task_config(args.task_name)
    check_steps = config["check_steps"] if args.max_steps is None else args.max_steps
    if check_steps < 1:
        raise ValueError("check_env steps must be positive.")
    failed = False
    env_config = build_environment_config(
        args.task_name, instance_id=args.instance_indices[0], max_steps=check_steps,
    )
    tile_size = (640, 480)
    if args.camera_resolution is not None:
        resolution = args.camera_resolution
        sensors = list(env_config["robots"][0]["sensor_config"].values())
        sensors.extend(env_config["env"]["external_sensors"])
        for sensor in sensors:
            sensor["sensor_kwargs"].update(image_width=resolution, image_height=resolution)
        tile_size = (resolution, resolution)
    stage = "creating environment"
    try:
        print(f"check_env: {stage}", flush=True)
        env = og.Environment(configs=env_config)
        stage = "configuring robot physics"
        print(f"check_env: {stage}", flush=True)
        configure_robot_physics(env)
        for instance_id in args.instance_indices:
            stage = f"loading instance {instance_id}"
            print(f"check_env: {stage}", flush=True)
            load_instance(env, args.task_name, instance_id)
            stage = f"checking and recording instance {instance_id}"
            video_path = Path(args.output_dir).expanduser() / "check_env" / args.task_name / f"{instance_id}_preview.mp4"
            report = initial_report(env)
            base = env.robots[0].get_position_orientation()[0].clone()
            initial_positions = {name: env.scene.object_registry("name", name).get_position_orientation()[0].clone()
                                 for name in env.task.parameters["manipulated_objects"]}
            max_drift = {name: 0.0 for name in initial_positions}
            max_base_drift = 0.0
            writer = VideoWriter(video_path, env.env_config["action_frequency"])
            action = make_hold_action(env.robots[0])
            try:
                for _ in range(check_steps):
                    env.step(action)
                    writer.write(mosaic(frames(env), tile_size=tile_size))
                    max_base_drift = max(max_base_drift, float(th.linalg.vector_norm(env.robots[0].get_position_orientation()[0] - base)))
                    for name, position in initial_positions.items():
                        current = env.scene.object_registry("name", name).get_position_orientation()[0]
                        max_drift[name] = max(max_drift[name], float(th.linalg.vector_norm(current - position)))
            finally:
                writer.close()
            final = initial_report(env)
            report["passed"] = (report["initial_conditions_valid"] and final["initial_conditions_valid"]
                                and not report["initial_goal_satisfied"] and not final["initial_goal_satisfied"]
                                and max_base_drift <= config["thresholds"]["base_drift"]
                                and all(value <= config["thresholds"]["initial_object_drift"] for value in max_drift.values()))
            print(f"{args.task_name} instance {instance_id}: passed={report['passed']}, video={video_path}")
            failed |= not report["passed"]
    except Exception:
        print(f"check_env failed while {stage}:", file=sys.stderr, flush=True)
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        if og.app is not None:
            og.shutdown()
    if failed:
        raise SystemExit(1)
