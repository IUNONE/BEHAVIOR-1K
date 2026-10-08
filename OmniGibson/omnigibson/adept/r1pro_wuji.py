"""把 Wuji Hand 2 安装到 R1Pro 2026 法兰上, 生成 r1pro_wuji 资产.

源文件写在 ``adept/custom_assets/r1pro_wuji``. ``compose()`` 不启动 Isaac.
``import_usd()`` 在带 Isaac Sim 的机器上调用现有导入器, 再把导入结果复制回来.
"""

import json
import math
import os
import posixpath
import shutil
import subprocess
import tempfile
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import yaml


MODEL = "r1pro_wuji"
ASSET_DIR = Path(__file__).resolve().parent / "custom_assets" / MODEL
R1_URDF = Path(__file__).resolve().parents[3] / "joylo" / "assets" / "robot" / "r1_pro_a2_2026" / "r1_pro_a2_2026.urdf"
R1_MESH_LINK = Path("../../../../../../joylo/assets/robot/r1_pro_a2_2026/meshes")

WUJI_REPO = "wuji-technology/wuji-description"
WUJI_URDF = {
    "right": "hand2/hand2_beta2/body/urdf/right_with_mount.urdf",
    "left": "hand2/hand2_beta2/body/urdf/left_with_mount.urdf",
}
WUJI_MJCF = {
    "right": "hand2/hand2_beta2/body/mjcf/right_with_mount.xml",
    "left": "hand2/hand2_beta2/body/mjcf/left_with_mount.xml",
}
SIDE_PREFIX = {"left": "l", "right": "r"}
HOLONOMIC_JOINTS = [
    "base_footprint_x_joint",
    "base_footprint_y_joint",
    "base_footprint_z_joint",
    "base_footprint_rx_joint",
    "base_footprint_ry_joint",
    "base_footprint_rz_joint",
]
DRIVEN_BASE_JOINTS = [
    "base_footprint_x_joint",
    "base_footprint_y_joint",
    "base_footprint_rz_joint",
]
TORSO_JOINTS = [f"torso_joint{index}" for index in range(1, 5)]
TORSO_RESET = (1.025, -1.45, -0.47, 0.0)
ARM_RESET_DEGREES = {
    "left": (0.0, 60.0, 0.0, -90.0, 60.0, 45.0, -45.0),
    "right": (0.0, -60.0, 0.0, -90.0, -60.0, 45.0, 45.0),
}
# 夹爪固定关节相对 arm_link7 的原点. 两侧 URDF 相同, rpy 为 0 时子连杆平移可以直接相加.
GRIPPER_XYZ = (-0.0295, 0.0, -0.1637)
D405_XYZ = (0.074676, 0.009, 0.022183)
D405_RPY = (2.4435, 0.0, -1.5708)
GMSL_XYZ = (0.06129, 0.0, 0.0082885)
GMSL_RPY = (-3.1416, 0.0, -1.5708)
# 导入器创建的 eef 连杆不写进 URDF. 局部 +z 指向指尖, 即 arm_link7 的 -z.
EEF_POSITION = (0.0, 0.0, -0.22)
EEF_ORIENTATION_XYZW = (1.0, 0.0, 0.0, 0.0)
# 拇指 MCP 没有 _flex 后缀. _abd 已在 _is_curl 里排除, 所以 _mcp 不会把外展关节算成弯曲.
CURL_TOKENS = ("_flex", "_pip", "_dip", "_ip", "_mcp")
ACTUATED_TYPES = {"revolute", "continuous", "prismatic"}
LIMIT_TOLERANCE = 1e-4


def _request(url):
    """下载 URL, 带浏览器标识, 避免 GitHub 拒绝默认 Python 客户端."""
    request = urllib.request.Request(url, headers={"User-Agent": "behavior-1k-r1pro-wuji"})
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            return response.read()
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"download failed {error.code}: {url}") from error


def _wuji_commit():
    """记录本次组装使用的 wuji-description main 提交."""
    url = f"https://api.github.com/repos/{WUJI_REPO}/commits/main"
    try:
        return json.loads(_request(url))["sha"]
    except (RuntimeError, urllib.error.URLError, KeyError, json.JSONDecodeError):
        completed = subprocess.run(
            ["gh", "api", f"repos/{WUJI_REPO}/commits/main", "--jq", ".sha"],
            check=True,
            capture_output=True,
            text=True,
        )
        return completed.stdout.strip()


def _download(url, destination):
    """下载到目标文件. 已有非空文件则跳过."""
    if destination.is_file() and destination.stat().st_size > 0:
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = _request(url)
    if payload.startswith(b"version https://git-lfs.github.com"):
        raise RuntimeError(f"{url} returned a Git LFS pointer instead of file contents.")
    destination.write_bytes(payload)
    print(f"downloaded {destination.name} ({len(payload)} bytes)", flush=True)


def _raw_url(commit, repo_path):
    return f"https://raw.githubusercontent.com/{WUJI_REPO}/{commit}/{repo_path}"


def _floats(text):
    return tuple(float(item) for item in text.split())


def _format_floats(values):
    return " ".join(f"{value:.10g}" for value in values)


def _close(actual, expected, label):
    if len(actual) != len(expected) or any(abs(a - b) > 1e-6 for a, b in zip(actual, expected)):
        raise ValueError(f"{label} does not match the expected origin: {actual} != {expected}")


def _origin(joint):
    origin = joint.find("origin")
    return _floats(origin.get("xyz")), _floats(origin.get("rpy", "0 0 0"))


def _set_origin(joint, xyz, rpy):
    origin = joint.find("origin")
    origin.set("xyz", _format_floats(xyz))
    origin.set("rpy", _format_floats(rpy))


def _find_named(root, tag, name):
    for element in root.findall(tag):
        if element.get("name") == name:
            return element
    raise KeyError(f"URDF has no {tag} named {name}")


def _remove_named(root, tag, names):
    for element in list(root.findall(tag)):
        if element.get("name") in names:
            root.remove(element)


def _insert_after(root, anchor_name, elements):
    children = list(root)
    for index, child in enumerate(children):
        if child.tag == "joint" and child.get("name") == anchor_name:
            for offset, element in enumerate(elements):
                root.insert(index + 1 + offset, element)
            return
    raise KeyError(f"cannot find anchor joint {anchor_name}")


def _rewrite_meshes(element, rename):
    for mesh in element.iter("mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh.set("filename", rename(filename))


def _mesh_names(element):
    return [mesh.get("filename") for mesh in element.iter("mesh") if mesh.get("filename")]


def _limits(joint):
    limit = joint.find("limit")
    if limit is None or limit.get("lower") is None or limit.get("upper") is None:
        raise ValueError(f"joint {joint.get('name')} has no position limits")
    return float(limit.get("lower")), float(limit.get("upper"))


def _within(value, lower, upper):
    return lower - LIMIT_TOLERANCE <= value <= upper + LIMIT_TOLERANCE


def _open_qpos(lower, upper):
    """限位内取 0, 否则取离 0 最近的限位."""
    if lower <= 0.0 <= upper:
        return 0.0
    return min((lower, upper), key=abs)


def _is_curl(name):
    return "_abd" not in name and any(token in name for token in CURL_TOKENS)


def _closed_qpos(name, lower, upper, open_value):
    """弯曲关节收到绝对值更大的限位, 外展保持张开."""
    if not _is_curl(name):
        return open_value
    return max((lower, upper), key=abs)


def _massless_link(name):
    """给相机坐标系一个极小质量. PhysX 不接受字面量为 0 的刚体."""
    link = ET.Element("link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "1e-6"})
    ET.SubElement(
        inertial,
        "inertia",
        {"ixx": "1e-9", "ixy": "0", "ixz": "0", "iyy": "1e-9", "iyz": "0", "izz": "1e-9"},
    )
    return link


def _fixed_joint(name, parent, child, xyz, rpy=(0.0, 0.0, 0.0)):
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": _format_floats(xyz), "rpy": _format_floats(rpy)})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def _is_wheel_joint(name):
    return name.startswith("steer_motor_joint") or name.startswith("wheel_motor_joint")


def _download_hand(commit, side):
    """下载一只手的 URDF、网格和 MuJoCo 排除表."""
    urdf_repo_path = WUJI_URDF[side]
    urdf = ET.fromstring(_request(_raw_url(commit, urdf_repo_path)))
    meshes = {}
    for filename in _mesh_names(urdf):
        repo_mesh = posixpath.normpath(posixpath.join(posixpath.dirname(urdf_repo_path), filename))
        local_name = Path(filename).name
        previous = meshes.get(local_name)
        if previous is not None and previous != repo_mesh:
            raise ValueError(f"{side} hand has conflicting mesh filenames: {local_name}")
        meshes[local_name] = repo_mesh
    for local_name, repo_mesh in meshes.items():
        _download(_raw_url(commit, repo_mesh), ASSET_DIR / "meshes" / "wuji" / side / local_name)
    _rewrite_meshes(urdf, lambda name, side=side: f"../meshes/wuji/{side}/{Path(name).name}")
    mjcf = b""
    try:
        mjcf = _request(_raw_url(commit, WUJI_MJCF[side]))
    except RuntimeError as error:
        print(f"{side} MuJoCo exclude list unavailable, using link names: {error}", flush=True)
    return urdf, mjcf


def _retarget_cameras(root, side):
    """删掉夹爪和两根手指, 把 d405 与 gmsl 改接到 arm_link7."""
    parent_link = f"{side}_arm_link7"
    gripper = _find_named(root, "joint", f"{side}_gripper_joint")
    parent = gripper.find("parent").get("link")
    if parent != parent_link:
        raise ValueError(f"{side} gripper parent is {parent}, expected {parent_link}")
    gripper_xyz, gripper_rpy = _origin(gripper)
    _close(gripper_xyz, GRIPPER_XYZ, f"{side} gripper xyz")
    if any(abs(value) > 1e-4 for value in gripper_rpy):
        raise ValueError(f"{side} gripper rpy is not zero, so translations cannot be added: {gripper_rpy}")

    for joint_name, child_xyz, child_rpy in (
        (f"{side}_d405_joint", D405_XYZ, D405_RPY),
        (f"{side}_gmsl_joint", GMSL_XYZ, GMSL_RPY),
    ):
        joint = _find_named(root, "joint", joint_name)
        if joint.find("parent").get("link") != f"{side}_gripper_link":
            raise ValueError(f"{joint_name} parent is not the gripper link")
        xyz, rpy = _origin(joint)
        _close(xyz, child_xyz, f"{joint_name} xyz")
        _close(rpy, child_rpy, f"{joint_name} rpy")
        _set_origin(joint, tuple(a + b for a, b in zip(gripper_xyz, xyz)), rpy)
        joint.find("parent").set("link", parent_link)

    _remove_named(
        root,
        "joint",
        {f"{side}_gripper_joint", f"{side}_gripper_finger_joint1", f"{side}_gripper_finger_joint2"},
    )
    _remove_named(
        root,
        "link",
        {f"{side}_gripper_link", f"{side}_gripper_finger_link1", f"{side}_gripper_finger_link2"},
    )
    d405_xyz, d405_rpy = _origin(_find_named(root, "joint", f"{side}_d405_joint"))
    realsense = f"{side}_realsense_link"
    _insert_after(
        root,
        f"{side}_d405_joint",
        [
            _massless_link(realsense),
            _fixed_joint(f"{side}_realsense_joint", parent_link, realsense, d405_xyz, d405_rpy),
        ],
    )


def _hand_elements(hand_root):
    """取出手的连杆和关节. 先从源树上摘下, 避免一个元素同时挂在两棵树上."""
    elements = []
    for child in list(hand_root):
        if child.tag in {"link", "joint"}:
            hand_root.remove(child)
            elements.append(child)
    return elements


def _exclusion_pairs(side, prefix, hand_links, mjcf):
    """官方装配重叠排除, 再加法兰与手腕/安装座."""
    wrist = f"{prefix}_wrist"
    known = set(hand_links)
    matched = []
    if mjcf:
        for exclude in ET.fromstring(mjcf).iter("exclude"):
            pair = [exclude.get("body1"), exclude.get("body2")]
            if pair[0] in known and pair[1] in known:
                matched.append(pair)
    if len(matched) < 10:
        matched = [[wrist, name] for name in hand_links if "proximal" in name]
        source = "proximal-link-name"
    else:
        source = "mujoco"
    pairs = [[f"{side}_arm_link7", f"{prefix}_mount"], [f"{side}_arm_link7", wrist], *matched]
    unique = []
    for pair in pairs:
        ordered = tuple(pair)
        if ordered not in unique and (ordered[1], ordered[0]) not in unique:
            unique.append(ordered)
    return [list(pair) for pair in unique], source


def _actuated_joints(root):
    """导入后仍占自由度的 URDF 关节. 轮子和转向会被导入器改成固定关节."""
    names = []
    for joint in root.findall("joint"):
        name = joint.get("name")
        if joint.get("type") in ACTUATED_TYPES and not _is_wheel_joint(name):
            names.append(name)
    return names


def _link_r1_meshes():
    link = ASSET_DIR / "meshes" / "r1pro"
    if link.is_symlink():
        link.unlink()
    elif link.exists():
        raise FileExistsError(f"{link} exists and is not a symlink")
    link.parent.mkdir(parents=True, exist_ok=True)
    link.symlink_to(R1_MESH_LINK)
    if not link.resolve().is_dir():
        raise FileNotFoundError(f"R1Pro mesh symlink is invalid: {link} -> {link.resolve()}")


def _validate_tree(root, hand_joints, finger_links):
    names = {element.get("name") for element in root.findall("link")}
    joint_names = [element.get("name") for element in root.findall("joint")]
    if len(joint_names) != len(set(joint_names)):
        raise ValueError("duplicate joint names")
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        if parent not in names or child not in names:
            raise ValueError(f"joint {joint.get('name')} parent or child link is missing")
    for dropped in (
        "left_gripper_link",
        "right_gripper_link",
        "left_gripper_finger_link1",
        "left_gripper_finger_link2",
        "right_gripper_finger_link1",
        "right_gripper_finger_link2",
    ):
        if dropped in names:
            raise ValueError(f"gripper link is still in the URDF: {dropped}")
    for side, prefix in SIDE_PREFIX.items():
        mount = _find_named(root, "joint", f"{prefix}_mount_joint")
        if mount.find("parent").get("link") != f"{side}_arm_link7":
            raise ValueError(f"{prefix}_mount_joint is not attached to {side}_arm_link7")
        if mount.find("child").get("link") != f"{prefix}_mount":
            raise ValueError(f"{prefix}_mount_joint child is not {prefix}_mount")
        if len(hand_joints[side]) != 20:
            raise ValueError(f"{side} hand has {len(hand_joints[side])} actuated joints, expected 20")
    for link_name in ("zed_link", "left_realsense_link", "right_realsense_link"):
        if link_name not in names:
            raise ValueError(f"missing camera link {link_name}")
    for side in ("left", "right"):
        for name in finger_links[side]:
            if name not in names:
                raise ValueError(f"finger link does not exist: {name}")
    urdf_dir = ASSET_DIR / "urdf"
    for filename in _mesh_names(root):
        if filename.startswith("package://") or os.path.isabs(filename):
            raise ValueError(f"mesh path must be relative: {filename}")
        mesh_path = (urdf_dir / filename).resolve()
        if not mesh_path.is_file():
            raise FileNotFoundError(f"mesh does not exist: {mesh_path}")
    actuated = _actuated_joints(root)
    expected = (
        TORSO_JOINTS
        + [f"left_arm_joint{index}" for index in range(1, 8)]
        + hand_joints["left"]
        + [f"right_arm_joint{index}" for index in range(1, 8)]
        + hand_joints["right"]
    )
    if actuated != expected:
        raise ValueError(f"actuated joint order does not match the expected order:\n{actuated}")
    if len(HOLONOMIC_JOINTS) + len(actuated) != 64:
        raise ValueError(f"expected 64 DOFs after import, got {len(HOLONOMIC_JOINTS) + len(actuated)}")


def _reset_vector(root, hand_joints, open_qpos):
    """按导入后的树顺序生成 64 维默认姿态."""
    by_name = {joint.get("name"): joint for joint in root.findall("joint")}
    values = {}
    for index, name in enumerate(TORSO_JOINTS):
        lower, upper = _limits(by_name[name])
        value = TORSO_RESET[index]
        if not _within(value, lower, upper):
            raise ValueError(f"{name} reset value {value} is outside limits [{lower}, {upper}]")
        values[name] = value
    for side, degrees in ARM_RESET_DEGREES.items():
        for index, degree in enumerate(degrees, start=1):
            name = f"{side}_arm_joint{index}"
            lower, upper = _limits(by_name[name])
            value = math.radians(degree)
            if not _within(value, lower, upper):
                raise ValueError(f"{name} reset value {value} is outside limits [{lower}, {upper}]")
            values[name] = value
    for side, names in hand_joints.items():
        for name, value in zip(names, open_qpos[side]):
            values[name] = value
    ordered = _actuated_joints(root)
    return [0.0] * len(HOLONOMIC_JOINTS) + [values[name] for name in ordered]


def _write_yaml(path, header, payload):
    path.write_text(header + yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")


def _definition_payload(hand_joints, finger_links, exclusions, reset):
    def side_map(values):
        return {side: values[side] for side in ("left", "right")}

    arms = {
        side: [f"{side}_arm_link{index}" for index in range(1, 8)] for side in ("left", "right")
    }
    arm_joints = {
        side: [f"{side}_arm_joint{index}" for index in range(1, 8)] for side in ("left", "right")
    }
    return {
        "raw_controller_order": [
            "base",
            "trunk",
            "arm_left",
            "gripper_left",
            "arm_right",
            "gripper_right",
        ],
        "default_controllers": {
            "base": "HolonomicBaseJointController",
            "trunk": "JointController",
            "arm_left": "JointController",
            "arm_right": "JointController",
            "gripper_left": "MultiFingerGripperController",
            "gripper_right": "MultiFingerGripperController",
        },
        "default_joint_pos": reset,
        "self_collisions": True,
        "base_footprint_link_name": "base_link",
        "usd_path": f"imported/usd/{MODEL}.usda",
        "urdf_path": f"urdf/{MODEL}.urdf",
        "linear_velocity_gain_for_primitives": 0.3,
        "angular_velocity_gain_for_primitives": 0.2,
        "disabled_collision_pairs": exclusions,
        "holonomic_base": {"force_sphere_wheel_approximation": True},
        "locomotion": {
            "base_joint_names": DRIVEN_BASE_JOINTS,
            "floor_touching_base_link_names": [f"wheel_motor_link{index}" for index in range(1, 4)],
        },
        "articulated_trunk": {
            "trunk_joint_names": TORSO_JOINTS,
            "trunk_link_names": [f"torso_link{index}" for index in range(1, 5)],
        },
        "mobile_manipulation": {
            "untucked_default_joint_pos": reset,
            "tucked_default_joint_pos": reset,
        },
        "manipulation": {
            "n_arms": 2,
            "arm_names": ["left", "right"],
            "arm_link_names": arms,
            "arm_joint_names": arm_joints,
            "eef_link_names": {"left": "left_eef_link", "right": "right_eef_link"},
            "finger_link_names": side_map(finger_links),
            "finger_joint_names": side_map(hand_joints),
            "gripper_link_names": {
                "left": ["l_wrist"],
                "right": ["r_wrist"],
            },
            "arm_workspace_range": {"left": [-180, 180], "right": [-180, 180]},
            # 平行夹爪推断要求恰好两根手指, 灵巧手会留下空的默认点并在辅助抓取时 KeyError.
            # 这里放一对腕部占位射线, 真正的指尖点以后再测.
            "assisted_grasp_start_points": {
                "left": [["l_wrist", [0.0, 0.0, 0.0]]],
                "right": [["r_wrist", [0.0, 0.0, 0.0]]],
            },
            "assisted_grasp_end_points": {
                "left": [["l_wrist", [0.0, 0.0, -0.02]]],
                "right": [["r_wrist", [0.0, 0.0, -0.02]]],
            },
        },
    }


def _import_config(convex_links, wheel_links, wheel_joints):
    def camera(link):
        return {
            "link": link,
            "parent_link": None,
            "offset": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]},
        }

    def eef(side):
        return {
            "link": f"{side}_eef_link",
            "parent_link": f"{side}_arm_link7",
            "offset": {
                "position": list(EEF_POSITION),
                "orientation": list(EEF_ORIENTATION_XYZW),
            },
        }

    return {
        "urdf_path": f"urdf/{MODEL}.urdf",
        "name": MODEL,
        "headless": True,
        "overwrite": True,
        "merge_fixed_joints": False,
        "dataset_name": "omnigibson-robot-assets",
        "base_motion": {
            "wheel_links": wheel_links,
            "wheel_joints": wheel_joints,
            "use_sphere_wheels": True,
            "use_holonomic_joints": True,
        },
        "collision": {
            "decompose_method": "convex",
            "hull_count": 1,
            "coacd_links": [],
            "convex_links": convex_links,
            "no_decompose_links": [],
            "no_collision_links": [],
        },
        "eef_vis_links": [eef("left"), eef("right")],
        "camera_links": [camera("zed_link"), camera("left_realsense_link"), camera("right_realsense_link")],
        "lidar_links": [],
    }


def _validate_definition(path):
    """用 RobotDefinition 结构校验, 不导入 omnigibson 包."""
    import importlib.util

    from omegaconf import OmegaConf

    schema_path = Path(__file__).resolve().parents[1] / "robots" / "definition_schema.py"
    spec = importlib.util.spec_from_file_location("r1pro_wuji_definition_schema", schema_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    merged = OmegaConf.merge(OmegaConf.structured(module.RobotDefinition), OmegaConf.load(path))
    OmegaConf.to_object(merged)


def compose():
    """下载 Hand 2, 组装 URDF, 并写出定义、姿态和导入配置."""
    if not R1_URDF.is_file():
        raise FileNotFoundError(f"R1Pro URDF not found: {R1_URDF}")
    commit = _wuji_commit()
    print(f"wuji-description {commit}", flush=True)
    _link_r1_meshes()
    root = ET.parse(R1_URDF).getroot()
    root.set("name", MODEL)
    for side in ("left", "right"):
        _retarget_cameras(root, side)
    _rewrite_meshes(root, lambda filename: f"../meshes/r1pro/{Path(filename).name}")

    hand_joints = {}
    finger_links = {}
    exclusions = []
    exclusion_sources = {}
    stored = {"open": {}, "closed": {}, "limits": {}}
    for side in ("left", "right"):
        prefix = SIDE_PREFIX[side]
        hand_root, mjcf = _download_hand(commit, side)
        revolute = []
        for joint in hand_root.findall("joint"):
            if joint.get("type") == "revolute":
                lower, upper = _limits(joint)
                revolute.append((joint.get("name"), lower, upper))
        link_names = [link.get("name") for link in hand_root.findall("link")]
        finger_links[side] = [
            link.get("name")
            for link in hand_root.findall("link")
            if link.find("collision") is not None and link.get("name") not in {f"{prefix}_mount", f"{prefix}_wrist"}
        ]
        elements = _hand_elements(hand_root)
        _insert_after(root, f"{side}_arm_joint7", [
            _fixed_joint(f"{prefix}_mount_joint", f"{side}_arm_link7", f"{prefix}_mount", GRIPPER_XYZ),
            *elements,
        ])
        hand_joints[side] = [name for name, _, _ in revolute]
        open_values = [_open_qpos(lower, upper) for _, lower, upper in revolute]
        closed_values = [
            _closed_qpos(name, lower, upper, open_value)
            for (name, lower, upper), open_value in zip(revolute, open_values, strict=True)
        ]
        for name, lower, upper in revolute:
            open_value = open_values[hand_joints[side].index(name)]
            closed_value = closed_values[hand_joints[side].index(name)]
            if not _within(open_value, lower, upper) or not _within(closed_value, lower, upper):
                raise ValueError(f"{name} open or closed pose is outside limits [{lower}, {upper}]")
        side_pairs, exclusion_sources[side] = _exclusion_pairs(side, prefix, link_names, mjcf)
        exclusions.extend(side_pairs)
        stored["open"][side] = open_values
        stored["closed"][side] = closed_values
        stored["limits"][side] = {name: [lower, upper] for name, lower, upper in revolute}
        idle = [name for name, _, _ in revolute if not _is_curl(name) and "_abd" not in name]
        if idle:
            print(
                f"{side} joints are neither curl nor abduction; closed pose keeps them open: {idle}",
                flush=True,
            )

    link_names = {link.get("name") for link in root.findall("link")}
    for pair in exclusions:
        missing = [name for name in pair if name not in link_names]
        if missing:
            raise ValueError(f"collision exclusion references a missing link: {pair}")

    head_left = _origin(_find_named(root, "joint", "camera_head_left_joint"))[0]
    head_right = _origin(_find_named(root, "joint", "camera_head_right_joint"))[0]
    zed_xyz = tuple(round((left + right) / 2.0, 9) for left, right in zip(head_left, head_right))
    _insert_after(root, "head_joint", [
        _massless_link("zed_link"),
        _fixed_joint("zed_joint", "head_link", "zed_link", zed_xyz),
    ])

    urdf_path = ASSET_DIR / "urdf" / f"{MODEL}.urdf"
    urdf_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(urdf_path, encoding="utf-8", xml_declaration=True)
    _validate_tree(root, hand_joints, finger_links)
    reset = _reset_vector(root, hand_joints, stored["open"])
    if len(reset) != 64:
        raise ValueError(f"default_joint_pos length is {len(reset)}, expected 64")

    convex_links = sorted(
        link.get("name")
        for link in root.findall("link")
        if any(mesh.get("filename") for mesh in link.findall("collision/geometry/mesh"))
    )
    wheel_links = [f"wheel_motor_link{index}" for index in range(1, 4)]
    wheel_joints = [
        joint.get("name") for joint in root.findall("joint") if _is_wheel_joint(joint.get("name"))
    ]
    definition_path = ASSET_DIR / f"{MODEL}.yaml"
    _write_yaml(
        definition_path,
        "# ADEPT R1Pro with Wuji Hand 2. This definition lives here, not in omnigibson-robot-assets.\n"
        "# default_joint_pos has 64 values: 6 holonomic base joints (x, y, z, rx, ry, rz),\n"
        "# then 4 torso, 7 left arm, 20 left hand, 7 right arm, 20 right hand.\n"
        "# Steer and wheel joints are fixed during import and are not DOFs.\n"
        "# Palm yaw about the flange is not measured yet; the mount rpy is 0.\n"
        "# assisted_grasp points are wrist placeholders so assisted mode does not read an empty default.\n",
        _definition_payload(hand_joints, finger_links, exclusions, reset),
    )
    _validate_definition(definition_path)
    dof_order = HOLONOMIC_JOINTS + _actuated_joints(root)
    _write_yaml(
        ASSET_DIR / "poses.yaml",
        "# Open and closed poses are not RobotDefinition fields; OmegaConf rejects unknown keys.\n",
        {
            "source": {
                "repo": WUJI_REPO,
                "commit": commit,
                "urdf": WUJI_URDF,
                "exclusion_pairs": exclusion_sources,
            },
            "mount_xyz": list(GRIPPER_XYZ),
            "mount_rpy": [0.0, 0.0, 0.0],
            "mount_note": "Palm yaw is not measured. rpy 0 keeps the fingers along the old gripper -z.",
            "eef_position_in_arm_link7": list(EEF_POSITION),
            "eef_orientation_xyzw": list(EEF_ORIENTATION_XYZW),
            "zed_link_xyz_in_head_link": list(zed_xyz),
            "joints": hand_joints,
            "open_qpos": stored["open"],
            "closed_qpos": stored["closed"],
            "limits": stored["limits"],
            "dof_order": dof_order,
        },
    )
    _write_yaml(
        ASSET_DIR / "import_config.yaml",
        "# Do not pass this file directly to import_custom_robot.py.\n"
        "# Collision decomposition rewrites the URDF. Run import_usd(), which copies first.\n"
        "# EEF and camera poses are initial values to tune after import.\n",
        _import_config(convex_links, wheel_links, wheel_joints),
    )
    load_poses.cache_clear()
    print(
        f"composed {urdf_path} joints={len(hand_joints['left'])}+{len(hand_joints['right'])} "
        f"dof={len(reset)} meshes={len(_mesh_names(root))}",
        flush=True,
    )


def import_usd():
    """在 Isaac Sim 中导入, 并把整个模型目录复制到 custom_assets/r1pro_wuji/imported."""
    from omnigibson.examples.robots.import_custom_robot import main as import_main
    from omnigibson.utils.asset_utils import get_dataset_path

    source_urdf = ASSET_DIR / "urdf" / f"{MODEL}.urdf"
    if not source_urdf.is_file():
        raise FileNotFoundError(f"run compose() first; missing {source_urdf}")
    model_dir = Path(get_dataset_path("omnigibson-robot-assets")) / "objects" / "robot" / MODEL
    if model_dir.exists():
        shutil.rmtree(model_dir)
    shutil.copytree(ASSET_DIR / "urdf", model_dir / "urdf")
    shutil.copytree(ASSET_DIR / "meshes", model_dir / "meshes", symlinks=False)
    config = yaml.safe_load((ASSET_DIR / "import_config.yaml").read_text(encoding="utf-8"))
    config["urdf_path"] = str(model_dir / "urdf" / f"{MODEL}.urdf")
    config["overwrite"] = True
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False, encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False, allow_unicode=True)
        temporary = Path(handle.name)
    try:
        import_main(["--config", str(temporary)], standalone_mode=False)
    finally:
        temporary.unlink(missing_ok=True)
    destination = ASSET_DIR / "imported"
    if destination.exists():
        shutil.rmtree(destination)
    shutil.copytree(model_dir, destination)
    usda = destination / "usd" / f"{MODEL}.usda"
    if not usda.is_file():
        raise FileNotFoundError(f"importer did not write {usda}")
    print(f"imported USD: {usda}", flush=True)


@lru_cache(maxsize=1)
def load_poses():
    """读取张开、闭合和关节顺序. 这些字段不能放进 RobotDefinition."""
    with (ASSET_DIR / "poses.yaml").open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def open_positions(side):
    """返回该侧手指张开姿态, 键为关节名."""
    poses = load_poses()
    return {name: float(value) for name, value in zip(poses["joints"][side], poses["open_qpos"][side], strict=True)}


def use_wuji_hand(robot):
    """把 ADEPT 机器人配置切到 r1pro_wuji.

    common.yaml 的 reset_joint_pos 是 28 维. Robot 加载时会按它调用 set_joint_positions,
    长度必须等于自由度, 所以这里丢掉该向量, 改由定义文件里的 64 维默认姿态加载.
    夹爪保持一维 binary, 1 为张开, 这样 hold 动作和 EEF20 不用改维度.
    """
    robot["model"] = MODEL
    robot.pop("reset_joint_pos", None)
    poses = load_poses()
    for side in ("left", "right"):
        gripper = robot["controller_config"][f"gripper_{side}"]
        gripper["name"] = "MultiFingerGripperController"
        gripper["motor_type"] = "position"
        gripper["mode"] = "binary"
        gripper["open_qpos"] = [float(value) for value in poses["open_qpos"][side]]
        gripper["closed_qpos"] = [float(value) for value in poses["closed_qpos"][side]]
    return robot


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Compose r1pro_wuji, or import its USD in Isaac Sim")
    parser.add_argument("--import-usd", action="store_true", help="Run import_custom_robot and copy the USD")
    args = parser.parse_args()
    if args.import_usd:
        import_usd()
    else:
        compose()
