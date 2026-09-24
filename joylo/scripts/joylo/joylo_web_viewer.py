"""GPU browser viewer: upload meshes once, then change joint rotations only."""

from __future__ import annotations

import threading
from collections.abc import Sequence

import numpy as np
import viser
from scipy.spatial.transform import Rotation
from yourdfpy import URDF
from joylo_motor_ids import motor_labels

from joylo_comparison import (R1_URDF, arm_sides, native_config, unpack_native,
                              mirror_shared, shared_limits, r1_config, calibration_shared)


def quaternion(matrix: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to Viser's scalar-first quaternion."""
    return Rotation.from_matrix(matrix).as_quat()[[3, 0, 1, 2]]


class GpuArmScene:
    """Keep URDF parent/child frames and immutable geometry on the client GPU."""

    def __init__(
        self, server: viser.ViserServer, model: URDF, *, root: str = "/robot",
        position: Sequence[float] = (0, 0, 0), scale: float = 1.0,
    ) -> None:
        self.server = server
        self.scale = scale
        self.frames: dict[str, viser.FrameHandle] = {}
        self.paths: dict[str, str] = {}
        self.motor_handles: list[viser.MeshHandle] = []
        self.printed_handles: list[viser.MeshHandle] = []
        self.joycon_handles: list[viser.MeshHandle] = []
        self.visual_paths: dict[str, str] = {}
        self.axis_handles: list[viser.FrameHandle | viser.LabelHandle] = []
        incoming = {joint.child: joint for joint in model.robot.joints}

        def add_link(name: str) -> str:
            if name in self.paths:
                return self.paths[name]
            joint = incoming.get(name)
            if joint is None:
                path, origin = f"{root}/{name}", np.eye(4)
            else:
                path = f"{add_link(joint.parent)}/{name}"
                origin = joint.origin if joint.origin is not None else np.eye(4)
            self.frames[name] = server.scene.add_frame(
                path, show_axes=False, wxyz=quaternion(origin[:3, :3]),
                position=origin[:3, 3] * scale,
            )
            self.paths[name] = path
            return path

        self.root_handle = server.scene.add_frame(root, show_axes=False, position=position)
        geometry_nodes: dict[str, list[str]] = {}
        for node in model.scene.graph.nodes_geometry:
            parent = model.scene.graph.transforms.parents[node]
            geometry_nodes.setdefault(parent, []).append(node)
        for link in model.robot.links:
            path = add_link(link.name)
            nodes = geometry_nodes.get(link.name, [])
            if len(nodes) != len(link.visuals):
                raise ValueError(f"Missing or unsupported visuals for {link.name}")
            for index, (visual, node) in enumerate(zip(link.visuals, nodes)):
                origin, geometry_name = model.scene.graph.get(node, frame_from=link.name)
                mesh = model.scene.geometry[geometry_name]
                rgba = (visual.material.color.rgba
                        if visual.material is not None and visual.material.color is not None
                        else np.array([0.7, 0.72, 0.75, 1.0]))
                visual_name = visual.name or f"visual_{index}"
                handle = server.scene.add_mesh_simple(
                    f"{path}/{visual_name}",
                    vertices=np.asarray(mesh.vertices * scale, dtype=np.float32),
                    faces=np.asarray(mesh.faces, dtype=np.uint32),
                    color=tuple(int(round(value * 255)) for value in rgba[:3]),
                    opacity=float(rgba[3]),
                    wxyz=quaternion(origin[:3, :3]), position=origin[:3, 3] * scale,
                    side="double",
                )
                if visual.name:
                    self.visual_paths[visual.name] = f"{path}/{visual_name}"
                if visual.name and visual.name.endswith('_printed'):
                    self.printed_handles.append(handle)
                if 'joycon' in link.name:
                    self.joycon_handles.append(handle)
                if visual.name and any(tag in visual.name for tag in ("motor", "horn_", "shoulder_pair", "flange", "idler")):
                    self.motor_handles.append(handle)

        self.joints = model.actuated_joints
        self.joint_frames = [self.frames[joint.child] for joint in self.joints]
        self.origins = Rotation.from_matrix(np.array([
            joint.origin[:3, :3] if joint.origin is not None else np.eye(3)
            for joint in self.joints
        ]))
        self.axes = np.array([joint.axis for joint in self.joints], dtype=float)
        self.axes /= np.linalg.norm(self.axes, axis=1, keepdims=True)
        self.translations = np.array([joint.origin[:3, 3] for joint in self.joints])
        self.prismatic = np.array([joint.type == "prismatic" for joint in self.joints])
        self.previous = np.full(len(self.joints), np.nan)
        for index, joint in enumerate(self.joints):
            path = self.paths[joint.child]
            # Local frame and label inherit all upstream joint transformations.
            self.axis_handles.append(server.scene.add_frame(
                f"{path}/axes", axes_length=0.024 * scale, axes_radius=0.0006 * scale,
                origin_radius=0.001 * scale, visible=False,
            ))
            joint_number = joint.name.rsplit("joint", 1)[-1].lstrip("_")
            side_label = "L" if joint.name.startswith("left_") else ("R" if joint.name.startswith("right_") else "J")
            self.axis_handles.append(server.scene.add_label(
                f"{path}/label", f"{side_label}{joint_number}",
                position=self.axes[index] * 0.026 * scale, visible=False,
            ))

    def update_angles(self, radians: np.ndarray) -> None:
        """Send only changed joint quaternions; never transform or resend vertices."""
        angles = np.asarray(radians, dtype=float)
        if angles.shape != self.previous.shape or not np.isfinite(angles).all():
            raise ValueError(f"Expected {len(self.joints)} finite joint positions")
        changed = np.flatnonzero(angles != self.previous)
        if not len(changed):
            return
        rotational = changed[~self.prismatic[changed]]
        quaternions = np.empty((0, 4))
        if len(rotational):
            rotations = self.origins[rotational] * Rotation.from_rotvec(
                self.axes[rotational] * angles[rotational, None]
            )
            quaternions = rotations.as_quat()[:, [3, 0, 1, 2]]
        with self.server.atomic():
            for index, value in zip(rotational, quaternions):
                self.joint_frames[index].wxyz = value
            for index in changed[self.prismatic[changed]]:
                offset = self.origins[index].apply(self.axes[index] * angles[index])
                self.joint_frames[index].position = (
                    self.translations[index] + offset
                ) * self.scale
        self.previous[:] = angles


def run_viewer(model: URDF, initial_degrees: Sequence[float], port: int = 8080,
               solo: bool = False, live=None) -> None:
    """Serve the interactive model on localhost until interrupted with Ctrl+C."""
    r1 = None if solo else URDF.load(str(R1_URDF), load_collision_meshes=False)
    server = create_server(port)
    try:
        if live is not None:
            if 'right' not in arm_sides(model):
                raise ValueError('Live mode requires a right or dual-arm model')
            input('Close DYNAMIXEL Wizard and support the right arm. Enter to disable torque and connect: ')
            live.start()
        _serve(server, model, initial_degrees, r1, live)
    finally:
        if live is not None and live.thread.ident is not None:
            live.close()
        server.stop()


def create_server(port: int, label: str = "JoyLo 7-DoF") -> viser.ViserServer:
    """Handle the conda Viser 0.2.23 startup race with disappearing processes."""
    if viser.__version__ != "0.2.23":
        return viser.ViserServer(host="127.0.0.1", port=port, label=label)
    import psutil
    from viser import _client_autobuild

    original_check = _client_autobuild._check_viser_yarn_running

    def check_yarn() -> bool:
        try:
            return original_check()
        except psutil.NoSuchProcess:
            # Development-server detection is optional. The packaged client
            # still goes through Viser's normal build existence/timestamp check.
            return False

    _client_autobuild._check_viser_yarn_running = check_yarn
    try:
        return viser.ViserServer(host="127.0.0.1", port=port, label=label)
    finally:
        _client_autobuild._check_viser_yarn_running = original_check


def _serve(server: viser.ViserServer, model: URDF, initial_degrees: Sequence[float],
           r1: URDF | None = None, live=None) -> None:
    """Display left/right JoyLo and independently map each arm to R1 Pro."""
    sides = arm_sides(model)
    dual = len(sides) == 2
    server.scene.set_up_direction("+z")
    server.gui.configure_theme(control_layout="floating", control_width="medium",
                               show_logo=False, brand_color=(41, 115, 168))
    scale = 2.5 if r1 is not None else 1.0
    offset = np.array([0., 0.75, 1.45]) if r1 is not None else np.zeros(3)
    robot_offset = np.array([0., -0.65, 0.])
    scene = GpuArmScene(server, model, root="/joylo", position=offset, scale=scale)
    id_labels = []
    for motor in motor_labels(model):
        label_offset = (0, 0, .025 * scale)
        if 'shoulder_pair_a' in motor.visual:
            label_offset = (0, -.025 * scale, .015 * scale)
        elif 'shoulder_pair_b' in motor.visual:
            label_offset = (0, .025 * scale, .015 * scale)
        # Parent to the actual motor visual: follows its housing, not the rotor.
        id_labels.append(server.scene.add_label(
            scene.visual_paths[motor.visual] + '/motor_id',
            f"{'L' if motor.side == 'left' else 'R'} ID {motor.motor_id} · J{motor.joint}",
            position=label_offset, visible=True,
        ))
    robot_scene = (GpuArmScene(server, r1, root="/r1pro", position=robot_offset)
                   if r1 is not None else None)
    ranges = [shared_limits(model, r1, side) for side in sides]
    low = np.array([v[0] for v in ranges]); high = np.array([v[1] for v in ranges])
    initial = np.clip(unpack_native(model, np.asarray(initial_degrees)), low, high)

    def apply(values: np.ndarray) -> None:
        scene.update_angles(native_config(model, values))
        if robot_scene is not None:
            robot_scene.update_angles(r1_config(r1, values, sides=sides))

    def get_bounds(values: np.ndarray, focus: str = "both") -> np.ndarray:
        model.update_cfg(native_config(model, values))
        if focus.endswith('_j6'):
            side = focus.split('_')[0]
            link = side + '_l7' if dual else 'l7'
            center = model.get_transform(link)[:3, 3] * scale + offset
            return np.array([center - .050 * scale, center + .050 * scale])
        jb = model.scene.bounds * scale + offset
        if r1 is None or focus == "joylo":
            return jb
        r1.update_cfg(r1_config(r1, values, sides=sides))
        rb = r1.scene.bounds + robot_offset
        if focus == "r1":
            return rb
        return np.array([np.minimum(jb[0], rb[0]), np.maximum(jb[1], rb[1])])

    apply(initial)
    fit_bounds = get_bounds(initial)
    dirty = threading.Event()
    state_lock = threading.Lock()
    desired = initial.copy()
    camera_requests: list[tuple[viser.ClientHandle, str]] = []
    server.gui.add_markdown(
        "## 双臂 JoyLo" + (" + R1 Pro" if r1 is not None else "") + "\n"
        "原模型为左臂，右臂为几何镜像。\n\n"
        "左右各 7 轴，分别联动对应侧。滑块采用 R1 Pro 参考角度。"
        "初始为截图参考姿态，不是电机零位。"
    )
    mirror = server.gui.add_checkbox("左右镜像联动", initial_value=False, visible=dual)
    show_ids = server.gui.add_checkbox("显示每台电机 ID", initial_value=True)
    server.gui.add_markdown("**装配编号方案**：左臂 ID 0–8，右臂 ID 9–17。请按电机标签设置 ID。")
    with server.gui.add_folder("电机分组与零位依据", expand_by_default=False):
        server.gui.add_markdown(
            "|关节|左臂 ID|右臂 ID|\n|---|---|---|\n"
            "|J1|0 / 1|9 / 10|\n|J2|2 / 3|11 / 12|\n|J3|4|13|\n"
            "|J4|5|14|\n|J5|6|15|\n|J6|7|16|\n|J7|8|17|\n\n"
            "编号按当前模型的电机位置分配。旧版六轴文档采用另一套编号。"
            "当前未运行硬件校准。编码器零位需要参考姿态的实测读数。"
        )

    @show_ids.on_update
    def on_ids(_: object) -> None:
        for handle in id_labels:
            handle.visible = show_ids.value
    tabs = server.gui.add_tab_group()
    sliders: list[list] = []
    for row, side in enumerate(sides):
        with tabs.add_tab("左臂 Left" if side == "left" else "右臂 Right"):
            sliders.append([server.gui.add_slider(
                f"{'L' if side == 'left' else 'R'} J{i + 1} / 度",
                min=float(low[row, i]), max=float(high[row, i]), step=0.1,
                initial_value=float(initial[row, i]),
                marks=((float(low[row, i]), f"{low[row, i]:.0f}°"),
                       (float(high[row, i]), f"{high[row, i]:.0f}°")),
            ) for i in range(7)])

    def on_slider(row: int, event: viser.GuiEvent) -> None:
        # Server-side updates (reset / mirror) must not recursively copy back.
        if event.client is None or live is not None:
            return
        with state_lock:
            desired[row] = np.clip([h.value for h in sliders[row]], low[row], high[row])
            if dual and mirror.value:
                other = 1 - row
                desired[other] = np.clip(mirror_shared(desired[row], sides[row]),
                                         low[other], high[other])
                with server.atomic():
                    for handle, value in zip(sliders[other], desired[other]):
                        handle.value = float(value)
            dirty.set()

    for row, handles in enumerate(sliders):
        for handle in handles:
            handle.on_update(lambda event, row=row: on_slider(row, event))

    @mirror.on_update
    def on_mirror(event: viser.GuiEvent) -> None:
        if dual and mirror.value:
            on_slider(0, event)

    reset = server.gui.add_button("恢复零位")
    calibrate = server.gui.add_button("恢复标定位")
    fit = server.gui.add_button("同屏适应画面" if r1 is not None else "机械臂适应画面")
    focus_joylo = server.gui.add_button("聚焦 JoyLo")
    focus_robot = server.gui.add_button("聚焦 R1 Pro", visible=r1 is not None)
    motors = server.gui.add_checkbox("显示 JoyLo 电机占位件", initial_value=True)
    hide_prints = server.gui.add_checkbox("隐藏打印件，看电机与法兰", initial_value=False)
    show_joycons = server.gui.add_checkbox("显示 Joy-Con 手柄", initial_value=True)
    axes = server.gui.add_checkbox("显示关节坐标轴与编号", initial_value=False)
    if dual:
        for side in sides:
            position = model.get_transform(side + '_base_link')[:3, 3] + [0., 0., .06]
            server.scene.add_label(f"/joylo/{side}_label",
                                   "左臂 / Left" if side == 'left' else "右臂 / Right",
                                   position=np.array(position) * scale)
    if r1 is not None:
        server.scene.add_label("/joylo_title", "双臂 JoyLo ×2.5" if dual else "JoyLo ×2.5",
                               position=offset + [0, 0, .18])
        server.scene.add_label("/r1pro_title", "R1 Pro · A2 7+7 DoF",
                               position=robot_offset + [0, 0, 1.80])

    live_panel = None
    if live is not None:
        mirror.disabled = True
        reset.disabled = True
        calibrate.disabled = True
        for handles in sliders:
            for handle in handles:
                handle.disabled = True
        live_panel = server.gui.add_markdown("Connecting to right arm...")

    @reset.on_click
    def on_reset(_: object) -> None:
        if live is not None:
            return
        with state_lock:
            desired[:] = np.zeros_like(initial)
            with server.atomic():
                for row, handles in enumerate(sliders):
                    for handle, value in zip(handles, desired[row]):
                        handle.value = float(value)
            dirty.set()

    @calibrate.on_click
    def on_calibrate(_: object) -> None:
        if live is not None:
            return
        with state_lock:
            desired[:] = np.clip(calibration_shared(model), low, high)
            with server.atomic():
                for row, handles in enumerate(sliders):
                    for handle, value in zip(handles, desired[row]):
                        handle.value = float(value)
            dirty.set()

    @motors.on_update
    def on_motors(_: object) -> None:
        with server.atomic():
            for handle in scene.motor_handles:
                handle.visible = motors.value

    @axes.on_update
    def on_axes(_: object) -> None:
        with server.atomic():
            for handle in scene.axis_handles:
                handle.visible = axes.value
            if robot_scene is not None:
                for index, joint in enumerate(robot_scene.joints):
                    visible = axes.value and any(joint.name.startswith(s + "_arm_joint") for s in sides)
                    for handle in robot_scene.axis_handles[index * 2:index * 2 + 2]:
                        handle.visible = visible

    @hide_prints.on_update
    def on_prints(_: object) -> None:
        for handle in scene.printed_handles:
            handle.visible = not hide_prints.value

    @show_joycons.on_update
    def on_joycons(_: object) -> None:
        for handle in scene.joycon_handles:
            handle.visible = show_joycons.value

    def set_camera(client: viser.ClientHandle, bounds: np.ndarray) -> None:
        midpoint = bounds.mean(axis=0)
        direction = np.array([2.8, 0.9, 0.55])
        direction /= np.linalg.norm(direction)
        right = np.cross([0., 0., 1.], direction)
        right /= np.linalg.norm(right)
        up = np.cross(direction, right)
        corners = np.array([[x, y, z] for x in bounds[:, 0]
                            for y in bounds[:, 1] for z in bounds[:, 2]])
        width = np.ptp(corners @ right)
        height = np.ptp(corners @ up)
        depth = np.ptp(corners @ direction)
        tangent = np.tan(np.deg2rad(42) / 2)
        # Fit projected extents, reserving horizontal room for the control panel.
        distance = 1.12 * max(height / (2 * tangent),
                             width / (2 * tangent * client.camera.aspect * 0.76)) + depth / 2
        # Center the robots in the canvas area left of the floating control panel.
        midpoint = midpoint + right * (distance * tangent * client.camera.aspect * 0.22)
        with client.atomic():
            client.camera.up_direction = (0.0, 0.0, 1.0)
            client.camera.fov = np.deg2rad(42)
            client.camera.position = midpoint + direction * max(distance, 0.12)
            client.camera.look_at = midpoint

    @server.on_client_connect
    def on_connect(client: viser.ClientHandle) -> None:
        set_camera(client, fit_bounds)

    def request_fit(event: viser.GuiEvent, focus: str) -> None:
        if event.client is not None:
            with state_lock:
                camera_requests.append((event.client, focus))

    fit.on_click(lambda event: request_fit(event, "both"))
    focus_joylo.on_click(lambda event: request_fit(event, "joylo"))
    focus_robot.on_click(lambda event: request_fit(event, "r1"))
    print(f"GPU viewer: http://127.0.0.1:{server.get_port()}", flush=True)
    print(f"JoyLo: {', '.join(sides)}; independent arms + optional mirrored motion.", flush=True)
    timer = threading.Event()
    last_sample = None
    try:
        while not timer.wait(1 / 60):
            if live is not None:
                import time
                status, sample = live.snapshot()
                if sample is not None and status.startswith('Connected') and time.monotonic() - sample[0] > 1:
                    status = 'ERROR: stale readings; display frozen'
                if sample is not None and status.startswith('Connected') and sample[0] != last_sample:
                    last_sample, raw, calibrated = sample
                    q = calibrated[[0, 2, 4, 5, 6, 7, 8]]
                    row = sides.index('right')
                    with state_lock:
                        desired[row] = q
                        dirty.set()
                    with server.atomic():
                        for handle, value, lo, hi in zip(sliders[row], q, low[row], high[row]):
                            handle.value = float(np.clip(value, lo, hi))
                    details = '| ID | Raw ° | Calibrated ° |\n|---|---:|---:|\n'
                    details += '\n'.join(f'| {mid} | {a:.2f} | {b:.2f} |' for mid, a, b in zip(live.ids, raw, calibrated))
                    details += '\n\nJ1–J7: ' + ', '.join(f'{v:.2f}°' for v in q)
                    details += f'\n\nPair difference 9−10: {calibrated[0]-calibrated[1]:.2f}°; 11−12: {calibrated[2]-calibrated[3]:.2f}°'
                    outside = np.flatnonzero((q < low[row]) | (q > high[row])) + 1
                    if len(outside):
                        details += f'\n\n**Outside display limits: J{outside.tolist()}. JoyLo shows measured angles; R1 is clipped to its limits.**'
                    live_panel.content = f'**{status}**\n\n' + details
                elif not status.startswith('Connected'):
                    live_panel.content = f'**{status}**'

            with state_lock:
                changed = dirty.is_set()
                dirty.clear()
                values = desired.copy()
                requests = camera_requests.copy()
                camera_requests.clear()
            if changed:
                apply(values)
            for client, focus in requests:
                bounds = get_bounds(values, focus)
                if focus == "both":
                    fit_bounds = bounds
                set_camera(client, bounds)
    except KeyboardInterrupt:
        pass
