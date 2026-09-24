"""Calibrate one or both JoyLo arms using the reference poses in ../imgs."""

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np
import tyro
import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "configs"
IMAGE_DIR = CONFIG_DIR.parent / "imgs"


@dataclass
class Args:
    gello_name: str = "default"
    """Configuration name; single-arm defaults become default_left/default_right."""
    port: str = "/dev/ttyUSB0"
    baudrate: int = 2000000
    robot: Literal["R1", "R1Pro"] = "R1Pro"
    arm: Literal["both", "left", "right"] = "both"
    overwrite: bool = False
    """Replace an existing calibration file."""


def reference_positions(robot_name: str, arm: str = "both"):
    if robot_name == "R1":
        zero = np.array([0, 0, 45, 45, -45, 0, 0, 0] * 2)
        calibration = np.array([90, 90, 180, 180, -180, 90, 90, -90,
                                -90, -90, 180, 180, -180, -90, -90, 90])
    elif robot_name == "R1Pro":
        zero = np.zeros(18)
        calibration = np.array([-90, -90, 90, 90, -90, -90, 90, 60, 90,
                                -90, -90, -90, -90, 90, -90, -90, 60, -90])
    else:
        raise ValueError("Robot must be R1 or R1Pro")
    half = len(zero) // 2
    selection = {"both": slice(None), "left": slice(0, half), "right": slice(half, None)}[arm]
    ids = list(range(len(zero)))[selection]
    return ids, zero[selection], calibration[selection]


def compute_joint_offsets_and_signs(joints_1, joints_2, robot_name, arm="both"):
    ids, expected_1, expected_2 = reference_positions(robot_name, arm)
    joints_1, joints_2 = np.asarray(joints_1), np.asarray(joints_2)
    if joints_1.shape != expected_1.shape or joints_2.shape != expected_2.shape:
        raise ValueError(f"Expected {len(ids)} motor readings for {arm}")
    if not np.all(np.isfinite([joints_1, joints_2])):
        raise ValueError("Non-finite motor readings")
    delta = joints_2 - joints_1
    expected_delta = expected_2 - expected_1
    unmoved = np.abs(delta) < 1.0
    if np.any(unmoved):
        raise ValueError(f"Motors barely moved between poses: {np.array(ids)[unmoved].tolist()}")
    signs = np.sign(delta / expected_delta)
    # Preserve the upstream assumption: assembly offsets are multiples of 90 degrees.
    offsets_1 = 90 * np.rint((joints_1 - signs * expected_1) / 90)
    offsets_2 = 90 * np.rint((joints_2 - signs * expected_2) / 90)
    mismatched = offsets_1 != offsets_2
    if np.any(mismatched):
        raise ValueError(
            f"Reference poses disagree for motor IDs {np.array(ids)[mismatched].tolist()}. "
            "Check both poses and repeat; do not change motor offsets to force a pass."
        )
    return signs, offsets_1


def main(args: Args):
    name = args.gello_name
    if not name or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in name):
        raise ValueError("Use letters, digits, '_' or '-' for --gello-name")
    if args.arm != "both" and name == "default":
        name = f"default_{args.arm}"
    output = CONFIG_DIR / f"joint_config_{name}.yaml"
    if output.exists() and not args.overwrite:
        raise FileExistsError(f"{output} already exists. Choose another name or pass --overwrite.")
    ids, expected_1, expected_2 = reference_positions(args.robot, args.arm)
    print(f"Robot: {args.robot}; arm: {args.arm}; motor IDs: {ids}")
    print(f"Port: {args.port}; baudrate: {args.baudrate}; output: {output}")
    input("Close DYNAMIXEL Wizard and support the arm(s). Torque will be disabled. Press Enter to connect:")

    from gello.utils.dynamixel_utils import DynamixelDriver

    driver = DynamixelDriver(ids=ids, port=args.port, baudrate=args.baudrate)
    try:
        # Existing driver only disables torque on initialization; no position commands are sent here.
        first, second = np.zeros(len(ids)), np.zeros(len(ids))
        sides = ("left", "right") if args.arm == "both" else (args.arm,)
        per_arm = 9 if args.robot == "R1Pro" else 8
        image_prefix = "R1pro" if args.robot == "R1Pro" else "R1"
        for i, side in enumerate(sides):
            section = slice(i * per_arm, (i + 1) * per_arm)
            for pose, values in (("zero", first), ("calibration", second)):
                photo = IMAGE_DIR / f"{image_prefix}_{pose}_{side[0].upper()}.jpg"
                print(f"\n{side.upper()} {pose} reference: {photo}")
                input("Place the arm in this pose, hold still, then press Enter to record:")
                values[section] = np.rad2deg(driver.get_joints())[section]
                print(f"IDs {ids[section]}: {np.round(values[section], 2).tolist()} degrees")
    finally:
        driver.close()

    signs, offsets = compute_joint_offsets_and_signs(first, second, args.robot, args.arm)
    data = {
        "robot": args.robot,
        "arm": args.arm,
        "angle_unit": "degrees",
        "joints": {"ids": ids, "offsets": offsets.tolist(), "signs": signs.astype(int).tolist()},
        "calibration": {
            "method": "two_reference_poses_90_degree_offsets",
            "measured_zero_deg": first.tolist(),
            "measured_calibration_deg": second.tolist(),
            "expected_zero_deg": expected_1.tolist(),
            "expected_calibration_deg": expected_2.tolist(),
        },
    }
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with output.open("w" if args.overwrite else "x") as file:
        yaml.safe_dump(data, file, sort_keys=False)
    print(f"\nSaved: {output}")
    print(f"Signs: {data['joints']['signs']}\nOffsets (degrees): {offsets.tolist()}")
    if args.arm != "both":
        print("Single-arm configuration: the original run_joylo.py/test_joints.py still require both arms.")
    print("Next: verify measured joint angles against physical motion before teleoperation.")


if __name__ == "__main__":
    main(tyro.cli(Args))
