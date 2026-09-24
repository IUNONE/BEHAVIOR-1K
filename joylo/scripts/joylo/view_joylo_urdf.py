"""Inspect the reconstructed JoyLo URDF without connecting to robot hardware.

Launch without options for the GPU browser viewer. Use --save preview.png for
headless rendering, or --backend matplotlib for the old desktop viewer.
Angles are degrees.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from yourdfpy import URDF
from joylo_comparison import JOYLO_HOME_DEG, default_native

DEFAULT_URDF = (
    Path(__file__).resolve().parents[2]
    / "hardware/joylo_v2_7dof_arm/urdf/joylo_dual.urdf"
)


def load_model(path: Path) -> URDF:
    """Load and check the URDF and all referenced visual meshes."""
    model = URDF.load(str(path.resolve()))
    if not model.validate():
        raise ValueError(f"Invalid URDF: {path}")
    if model.num_dofs not in (7, 14):
        raise ValueError(f"Expected 7 or 14 movable joints, found {model.num_dofs}")
    if not model.scene.geometry:
        raise ValueError("No visual geometry loaded")
    return model


def check_motion(model: URDF) -> None:
    """Verify connectivity and downstream motion, including the shoulder rod."""
    joints = model.robot.joints
    children = {joint.child: joint.parent for joint in joints}
    link_names = {link.name for link in model.robot.links}
    if len(children) != len(joints) or len(joints) != len(link_names) - 1:
        raise ValueError("Joint graph is not a tree")
    roots = link_names - children.keys()
    if roots != {"base_link"}:
        raise ValueError(f"Unexpected roots: {roots}")
    home = np.deg2rad(default_native(model))
    model.update_cfg(home)
    baseline = {name: model.get_transform(name).copy() for name in link_names}
    for index, joint in enumerate(model.actuated_joints):
        cfg = home.copy()
        cfg[index] += 0.05
        model.update_cfg(cfg)
        for name in link_names:
            current = name
            visited: set[str] = set()
            while current in children and current != joint.child:
                if current in visited:
                    raise ValueError("Cycle in joint graph")
                visited.add(current)
                current = children[current]
            downstream = current == joint.child
            changed = not np.allclose(model.get_transform(name), baseline[name])
            if changed != downstream:
                raise ValueError(f"Unexpected motion: {joint.name} -> {name}")
        # Revolving a joint must leave its own pivot position unchanged.
        np.testing.assert_allclose(
            model.get_transform(joint.child)[:3, 3],
            baseline[joint.child][:3, 3],
            atol=1e-10,
        )
    model.update_cfg(home)


def main() -> None:
    """Run validation, render a preview, or open the interactive viewer."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--save", type=Path, help="Save a two-view PNG without a GUI")
    parser.add_argument("--check", action="store_true", help="Validate and exit")
    parser.add_argument("--joints", nargs='+', type=float,
                        help="7 native angles (both arms) or 14 angles (left then right)")
    parser.add_argument("--backend", choices=["viser", "matplotlib"], default="viser")
    parser.add_argument("--port", default="8080", help="Viewer port, or serial device in live mode")
    parser.add_argument("--viewer-port", type=int, default=8080)
    parser.add_argument("--live-arm", choices=["right"])
    parser.add_argument("--baudrate", type=int, default=2000000)
    parser.add_argument("--joint-config", type=Path)
    parser.add_argument("--solo", action="store_true", help="Show only JoyLo")
    parser.add_argument("--robot", choices=["r1_pro", "mobile_nero"], default="r1_pro")
    parser.add_argument("--without_joylo", action="store_true", help="Show only the selected robot")
    args = parser.parse_args()
    live = None
    if args.live_arm:
        if args.backend != "viser" or args.save or args.check or args.without_joylo or args.robot != "r1_pro":
            parser.error("Live mode requires the JoyLo Viser viewer")
        if args.joint_config is None or args.port == "8080":
            parser.error("Live mode requires --joint-config and --port SERIAL_DEVICE")
        from joylo_live import RightArmStream
        live = RightArmStream(args.port, args.baudrate, args.joint_config)
        args.port = args.viewer_port
    else:
        try:
            args.port = int(args.port)
        except ValueError:
            parser.error("Offline --port must be an integer")
    if args.robot == "mobile_nero" and not args.without_joylo:
        parser.error("Nero JoyLo design has been removed; use --robot mobile_nero --without_joylo")
    if args.without_joylo:
        from mobile_robot_viewer import load_robot, run_robot_viewer, save_preview
        from joylo_comparison import R1_URDF
        path = (Path(__file__).resolve().parents[2] / "assets/robot/mobile_nero/mobile_nero.urdf"
                if args.robot == "mobile_nero" else R1_URDF)
        if args.joints is not None or args.urdf != DEFAULT_URDF or args.solo:
            parser.error("Robot-only mode does not use --joints, --urdf or --solo")
        robot = load_robot(path)
        print(f"Validated: {len(robot.robot.links)} links, {robot.num_dofs} movable joints")
        if args.check:
            return
        if args.save:
            save_preview(robot, args.save)
        elif args.backend == "matplotlib":
            parser.error("Robot-only interactive mode uses --backend viser")
        else:
            run_robot_viewer(robot, args.port)
        return
    model = load_model(args.urdf)
    check_motion(model)
    print(f"Validated: {len(model.robot.links)} links, {model.num_dofs} DOFs, all visual meshes loaded")
    if args.check:
        return
    if args.joints is None:
        args.joints = default_native(model).tolist()
    if model.num_dofs == 14 and len(args.joints) == 7:
        args.joints = args.joints * 2
    if len(args.joints) != model.num_dofs:
        parser.error(f"Expected {model.num_dofs} native joint angles")
    low = np.rad2deg([joint.limit.lower for joint in model.actuated_joints])
    high = np.rad2deg([joint.limit.upper for joint in model.actuated_joints])
    if (not np.isfinite(args.joints).all() or np.any(np.asarray(args.joints) < low)
            or np.any(np.asarray(args.joints) > high)):
        parser.error(f"Native JoyLo angles must lie within {list(zip(low, high))}")

    if not args.save and args.backend == "viser":
        from joylo_web_viewer import run_viewer

        run_viewer(model, args.joints, port=args.port, solo=args.solo, live=live)
        return

    import matplotlib

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    from matplotlib.widgets import Button, CheckButtons, Slider
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    model.update_cfg(np.deg2rad(args.joints))
    # Read URDF colors directly: the OBJ files refer to absent MTL files.
    materials = {
        visual.name: visual.material.color.rgba
        for link in model.robot.links
        for visual in link.visuals
    }
    # Trimesh disambiguates a visual node when its name equals a link name.
    # Resolve colors through each link's visual order instead of assuming names.
    geometry_nodes: dict[str, list[str]] = {}
    for node in model.scene.graph.nodes_geometry:
        geometry_nodes.setdefault(model.scene.graph.transforms.parents[node], []).append(node)
    for link in model.robot.links:
        for node, visual in zip(geometry_nodes.get(link.name, []), link.visuals):
            materials[node] = visual.material.color.rgba
    entries = []
    for node in model.scene.graph.nodes_geometry:
        _, geometry_name = model.scene.graph[node]
        mesh = model.scene.geometry[geometry_name]
        entries.append((node, mesh, materials[node]))

    fig = plt.figure(figsize=(13, 9), facecolor="#f5f7fa")
    fig.suptitle("JoyLo dual arms · 7 + 7 DoF" if model.num_dofs == 14 else "JoyLo 7-DoF arm",
                 fontsize=22, x=0.06, ha="left", y=0.96)
    fig.text(0.06, 0.912, "Reconstructed assembly | visual zero, not motor zero", color="#526174")
    if args.save:
        axes = [fig.add_axes([0.04, 0.12, 0.45, 0.76], projection="3d"),
                fig.add_axes([0.51, 0.12, 0.45, 0.76], projection="3d")]
        views = [(15, 32), (15, 125)]
    else:
        axes = [fig.add_axes([0.01, 0.07, 0.65, 0.81], projection="3d")]
        views = [(15, 32)]
    bounds = model.scene.bounds
    center = bounds.mean(axis=0)
    span = np.maximum(bounds[1] - bounds[0], 0.10) * 1.20
    patches = []
    joint_artists: list = []
    show_motors = True
    show_axes = True
    light = np.array([0.3, -0.4, 0.866])
    for ax, (elevation, azimuth) in zip(axes, views):
        ax.set_facecolor("#f5f7fa")
        ax.set_proj_type("ortho")
        for dimension, setter in enumerate([ax.set_xlim, ax.set_ylim, ax.set_zlim]):
            setter(center[dimension] - span[dimension] / 2,
                   center[dimension] + span[dimension] / 2)
        ax.set_box_aspect(span.copy(), zoom=1.0 if args.save else 1.2)
        ax.view_init(elevation, azimuth)
        ax.set_axis_off()
        for node, mesh, color in entries:
            collection = Poly3DCollection([], linewidth=0, rasterized=True)
            ax.add_collection3d(collection)
            patches.append((collection, node, mesh, color))

    def redraw() -> None:
        """Update mesh poses from URDF forward kinematics."""
        for artist in joint_artists:
            artist.remove()
        joint_artists.clear()
        for collection, node, mesh, color in patches:
            transform, _ = model.scene.graph[node]
            vertices = mesh.vertices @ transform[:3, :3].T + transform[:3, 3]
            collection.set_verts(vertices[mesh.faces])
            normals = mesh.face_normals @ transform[:3, :3].T
            shade = 0.52 + 0.48 * np.abs(normals @ light)
            face_colors = np.tile(color, (len(mesh.faces), 1))
            face_colors[:, :3] *= shade[:, None]
            collection.set_facecolor(face_colors)
            collection.set_visible(show_motors or not any(
                tag in node for tag in ("motor", "horn_", "shoulder_pair", "flange", "idler")))
        if show_axes:
            for ax in axes:
                for index, joint in enumerate(model.actuated_joints):
                    transform = model.get_transform(joint.child)
                    pivot = transform[:3, 3]
                    direction = transform[:3, :3] @ joint.axis * 0.021
                    joint_artists.append(ax.quiver(
                        *pivot, *direction, color="#26354a", arrow_length_ratio=0.22,
                        linewidth=1.2,
                    ))
                    label = (f"{'L' if joint.name.startswith('left_') else 'R'}{joint.name.rsplit('_',1)[-1]}"
                             if model.num_dofs == 14 else f"J{index + 1}")
                    joint_artists.append(ax.text(
                        *(pivot + direction * 1.15), label,
                        fontsize=9, color="#26354a", zorder=100,
                    ))
        fig.canvas.draw_idle()

    prefix = 'left_' if model.num_dofs == 14 else ''
    legend = [Patch(color=materials[f"{prefix}{name}_printed"][:3], label=name.upper())
              for name in ("l2", "l3", "l4", "l5", "l6", "l7", "ljc")]
    fig.legend(handles=legend, loc="lower center", ncol=7, frameon=False,
               bbox_to_anchor=(0.5, 0.035))
    redraw()
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=160, facecolor=fig.get_facecolor())
        print(f"Saved {args.save}")
        plt.close(fig)
        return

    sliders = []
    for index in range(model.num_dofs):
        x = 0.72 if model.num_dofs == 7 else 0.70 + (index // 7) * 0.17
        width = 0.21 if model.num_dofs == 7 else 0.10
        slider_ax = fig.add_axes([x, 0.78 - (index % 7) * 0.07, width, 0.022])
        label = f"J{index + 1}" if model.num_dofs == 7 else f"{'L' if index < 7 else 'R'}{index % 7 + 1}"
        slider = Slider(slider_ax, label, low[index], high[index],
                        valinit=args.joints[index], valstep=1, valfmt="%d deg")
        sliders.append(slider)

    def on_angle_change(_: float) -> None:
        model.update_cfg(np.deg2rad([slider.val for slider in sliders]))
        redraw()

    for slider in sliders:
        slider.on_changed(on_angle_change)
    reset = Button(fig.add_axes([0.72, 0.20, 0.21, 0.04]), "Reset to initial pose")

    def on_reset(_: object) -> None:
        for slider in sliders:
            slider.eventson = False
            slider.reset()
            slider.eventson = True
        on_angle_change(0)

    reset.on_clicked(on_reset)
    fit = Button(fig.add_axes([0.72, 0.145, 0.21, 0.04]), "Fit arm to view")

    def on_fit(_: object) -> None:
        """Frame the current pose without changing the user's viewing direction."""
        current_bounds = model.scene.bounds
        midpoint = current_bounds.mean(axis=0)
        extent = np.maximum(current_bounds[1] - current_bounds[0], 0.10) * 1.20
        for ax in axes:
            for dimension, setter in enumerate([ax.set_xlim, ax.set_ylim, ax.set_zlim]):
                setter(midpoint[dimension] - extent[dimension] / 2,
                       midpoint[dimension] + extent[dimension] / 2)
            ax.set_box_aspect(extent.copy(), zoom=1.2)
        fig.canvas.draw_idle()

    fit.on_clicked(on_fit)

    def on_scroll(event: object) -> None:
        """Zoom about the view center when scrolling over the model."""
        if event.inaxes not in axes:
            return
        scale = 1.15 ** (-event.step)
        ax = event.inaxes
        for getter, setter in [(ax.get_xlim, ax.set_xlim), (ax.get_ylim, ax.set_ylim),
                               (ax.get_zlim, ax.set_zlim)]:
            low, high = getter()
            midpoint = (low + high) / 2
            radius = np.clip((high - low) * scale / 2, 0.001, 5.0)
            setter(midpoint - radius, midpoint + radius)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("scroll_event", on_scroll)
    checks = CheckButtons(fig.add_axes([0.72, 0.255, 0.21, 0.08]),
                          ["Motor placeholders", "Joint axes"], [True, True])

    def on_visibility(label: str) -> None:
        nonlocal show_motors, show_axes
        if label == "Motor placeholders":
            show_motors = not show_motors
        else:
            show_axes = not show_axes
        redraw()

    checks.on_clicked(on_visibility)
    fig.text(0.70, 0.085, "Drag to rotate; scroll to zoom.\nUse Fit after changing pose.\nDisplay limits are not physical stops.",
             fontsize=9, color="#526174")
    plt.show()


if __name__ == "__main__":
    main()
