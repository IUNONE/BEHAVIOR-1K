import glob
import yaml
from dataclasses import dataclass
from typing import Optional, Tuple, Literal

import numpy as np
import tyro

from gello.agents.bimanual_agent import (
    ROBOT_TELEOP_CONFIGS,
    BimanualAgent,
    MotorFeedbackConfig,
)
from gello.agents.dynamixel_arm_agent import DynamixelArmAgent, DynamixelRobotConfig
from gello.agents.joycon_agent import JoyconAgent
from gello.agents.passive_teleop_agent import PassiveTeleopAgent
from gello.env import RobotEnv
from gello.robots.base_robot import PrintRobot
from gello.utils.zmq_utils import ZMQRobotClient
from gello import REPO_DIR


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    robot_port: int = 6001
    hostname: str = "127.0.0.1"
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None
    gello_model: str = "r1pro"
    gello_name: str = "default"
    gello_port: Optional[str] = None
    mock: bool = False
    damping_motor_kp: float = 0.3
    motor_feedback_type: str = "NONE"
    use_joycons: bool = True
    only_joycon: bool = False
    """Use Joy-Cons with the simulator without opening a motor serial port."""
    no_enable_joylo_torque: bool = False
    """Disable motor torque and only read angles; no reset, locking, or force feedback."""
    motor_confirm: bool = False
    """Use left Joy-Con right/left arrows to confirm/cancel motor moves instead of lights."""


def make_agent(args, bimanual_config, joycon_agent):
    if args.only_joycon:
        return PassiveTeleopAgent(bimanual_config, joycon_agent)
    # Find gello port
    gello_port = args.gello_port
    if gello_port is None:
        import platform

        if platform.system().lower() == "linux":
            usb_ports = glob.glob("/dev/serial/by-id/*")
        elif platform.system().lower() == "darwin":
            usb_ports = glob.glob("/dev/cu.usbserial-*")
        else:
            raise ValueError(f"Unsupported platform {platform.system()}")
        print(f"Found {len(usb_ports)} ports")
        if len(usb_ports) > 0:
            gello_port = usb_ports[0]
            print(f"using port {gello_port}")
        else:
            raise ValueError("No gello port found, please specify one or plug in gello")

    # Read joint config from yaml
    with open(f"{REPO_DIR}/configs/joint_config_{args.gello_name}.yaml", "r") as file:
        joint_config = yaml.load(file, Loader=yaml.SafeLoader)

    num_motors = bimanual_config.motors_per_arm * 2
    dynamixel_config = DynamixelRobotConfig(
        joint_ids=tuple(np.arange(num_motors).tolist()),
        joint_offsets=[np.deg2rad(x) for x in joint_config["joints"]["offsets"]],
        joint_signs=joint_config["joints"]["signs"],
        gripper_config=None,
    )

    if args.no_enable_joylo_torque:
        from gello.utils.dynamixel_utils import DynamixelDriver

        joints = joint_config["joints"]
        if joints.get("ids") != list(range(num_motors)) or joint_config.get("angle_unit") != "degrees":
            raise ValueError("Torque-off mode requires a combined calibration in degrees with ordered motor IDs")
        offsets = np.asarray(dynamixel_config.joint_offsets)
        signs = np.asarray(dynamixel_config.joint_signs)
        if not np.isfinite(offsets).all() or not np.isin(signs, [-1, 1]).all():
            raise ValueError("Invalid calibration offsets/signs")
        # The driver constructor checks Torque OFF writes. Do not construct the
        # active arm agent: its start/reset/locking paths can enable torque.
        driver = DynamixelDriver(ids=dynamixel_config.joint_ids, port=gello_port)
        return PassiveTeleopAgent(
            bimanual_config, joycon_agent,
            read_joints=lambda: (driver.get_joints() - offsets) * signs,
            close_reader=driver.close,
        )

    # Default start joints
    start_joints = args.start_joints
    if start_joints is None:
        start_joints = bimanual_config.start_joints.copy()

    arm_agent = DynamixelArmAgent(
        port=gello_port,
        dynamixel_config=dynamixel_config,
        start_joints=start_joints,
        damping_motor_kp=args.damping_motor_kp,
    )
    return BimanualAgent(
        config=bimanual_config,
        arm_agent=arm_agent,
        joycon_agent=joycon_agent,
        motor_feedback_type=MotorFeedbackConfig[args.motor_feedback_type],
        motor_confirm=args.motor_confirm,
    )


def main(args):
    if args.only_joycon and args.no_enable_joylo_torque:
        raise ValueError("--only_joycon and --no_enable_joylo_torque are mutually exclusive")
    passive = args.only_joycon or args.no_enable_joylo_torque
    if args.motor_confirm and (passive or not args.use_joycons):
        raise ValueError("--motor_confirm requires active JoyLo mode and connected Joy-Cons")
    if args.only_joycon and not args.use_joycons:
        raise ValueError("--only_joycon requires Joy-Cons")
    if passive and (args.mock or args.start_joints is not None or args.motor_feedback_type != "NONE"):
        raise ValueError("Passive modes require a simulator and do not support mock, start-joints, or motor feedback")
    assert args.gello_model in ROBOT_TELEOP_CONFIGS, f"Unsupported gello model: {args.gello_model}"
    bimanual_config = ROBOT_TELEOP_CONFIGS[args.gello_model]
    robot_client = (PrintRobot(bimanual_config.joints_per_arm * 2, dont_print=True) if args.mock
                    else ZMQRobotClient(port=args.robot_port, host=args.hostname))
    env = RobotEnv(robot_client, control_rate_hz=args.hz)

    # Create JoyCon agent
    joycon_agent = None
    if args.use_joycons:
        joycon_agent = JoyconAgent(
            calibration_dir=f"{REPO_DIR}/configs",
            deadzone_threshold=0.2,
            max_translation=0.35,
            max_rotation=0.3,
            max_trunk_translate=0.1,
            max_trunk_tilt=0.05,
            enable_rumble=False,
        )

    if args.no_enable_joylo_torque:
        print("Support both arms: connecting will disable torque. No motor motion commands will be sent.")
    agent = make_agent(args, bimanual_config, joycon_agent)

    agent.start()

    if passive:
        print("Joy-Con only; motors disconnected." if args.only_joycon else "Torque OFF; reading JoyLo angles only.")
        print("No motor resets, locks or force feedback. Press X to resume simulation.")
    elif args.motor_confirm:
        print("Motor confirmation: left Joy-Con RIGHT confirms, LEFT cancels/releases. Lights are disabled.")
    else:
        print("Automatic motor motion: return to start position, then align to simulation.")

    print_color("*" * 40, color="magenta", attrs=("bold",))
    print_color(
        f"\nWelcome to JoyLo ({args.gello_model.upper()})!\n",
        color="magenta",
        attrs=("bold",),
    )
    print_color(
        f"{args.gello_model.upper()} Teleoperation Commands:\n",
        color="magenta",
        attrs=("bold",),
    )
    print_color("\t ZL / ZR: Toggle grasping", color="magenta", attrs=("bold",))
    print_color(
        "\t Left Joystick (not pressed): Translate the robot base",
        color="magenta",
        attrs=("bold",),
    )
    print_color(
        "\t Right Joystick: Rotate the robot base + tilt the trunk torso",
        color="magenta",
        attrs=("bold",),
    )
    print_color(
        "\t Up / Down Button: Raise / Lower the trunk torso",
        color="magenta",
        attrs=("bold",),
    )
    print_color(
        "\t Left / Right Button: Cancel / confirm motor motion" if args.motor_confirm
        else "\t Left / Right Button: Toggle gripper light",
        color="magenta",
        attrs=("bold",),
    )
    if not passive and args.gello_model == "r1":
        print_color(
            "\t L / R: Lock the lower two wrist joints while leaving the upper joints free",
            color="magenta",
            attrs=("bold",),
        )
    elif not passive and args.gello_model == "r1pro":
        print_color(
            "\t L / R: Lock the lower three wrist joints while leaving the upper joints free",
            color="magenta",
            attrs=("bold",),
        )
    if not passive:
        print_color(
            "\t - / +: Toggle left / right whole-arm position hold"
            if args.gello_model == "r1pro" else
            "\t - / +: Toggle arm lock request",
            color="magenta",
            attrs=("bold",),
        )
        if args.motor_confirm:
            print_color("\t L / R and - / + request locks; RIGHT arrow confirms them.", color="magenta")
    print_color(
        "\t Y: Move the robot towards its reset pose", color="magenta", attrs=("bold",)
    )
    print_color("\t B: Toggle camera", color="magenta", attrs=("bold",))
    print_color("\t A: Toggle X-ray", color="magenta", attrs=("bold",))
    print_color("\t Home: Reset the environment\n", color="magenta", attrs=("bold",))
    print_color("*" * 40, color="magenta", attrs=("bold",))

    try:
        obs = env.get_obs()
        print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))
        while True:
            action = agent.act(obs)
            obs = env.step(action)
    finally:
        agent.close()


if __name__ == "__main__":
    main(tyro.cli(Args))
