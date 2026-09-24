"""Robot-only viewer with independent arm and lift controls, in native URDF units."""
from pathlib import Path
import threading
import numpy as np
from yourdfpy import URDF
from joylo_web_viewer import GpuArmScene, create_server


def load_robot(path: Path) -> URDF:
    model = URDF.load(str(path.resolve()), load_collision_meshes=False)
    if not model.validate() or not model.scene.geometry:
        raise ValueError(f'Invalid robot or missing visual geometry: {path}')
    links = {link.name for link in model.robot.links}
    children = [j.child for j in model.robot.joints]
    if len(children) != len(set(children)) or len(children) != len(links)-1:
        raise ValueError('Robot must have a single connected tree')
    if len(links-set(children)) != 1:
        raise ValueError('Robot must have exactly one root')
    return model


def initial_configuration(model: URDF) -> np.ndarray:
    cfg = np.zeros(model.num_dofs)
    if model.robot.name == 'mobile_nero':
        for side in ('left', 'right'):
            cfg[model.actuated_joint_names.index(side + '_arm_joint2')] = -np.pi / 2
    return cfg


def save_preview(model: URDF, path: Path):
    model.update_cfg(initial_configuration(model))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(12, 6))
    bounds = model.scene.bounds
    center = bounds.mean(axis=0)
    span = np.maximum(bounds[1]-bounds[0], .1)*1.08
    for index, azimuth in enumerate([35, 135]):
        ax = fig.add_subplot(1, 2, index+1, projection='3d')
        for node in model.scene.graph.nodes_geometry:
            pose, name = model.scene.graph[node]
            mesh = model.scene.geometry[name]
            verts = mesh.vertices@pose[:3,:3].T+pose[:3,3]
            shade = np.clip(.55+.4*(mesh.face_normals@pose[:3,:3].T)@np.array([.3,-.4,.866]),.15,1)
            colors = shade[:,None]*np.array([.65,.72,.8])
            ax.add_collection3d(Poly3DCollection(verts[mesh.faces], facecolors=colors, linewidth=0, rasterized=True))
        for i, setter in enumerate([ax.set_xlim, ax.set_ylim, ax.set_zlim]):
            setter(center[i]-span[i]/2, center[i]+span[i]/2)
        ax.set_box_aspect(span, zoom=1.15); ax.view_init(elev=15, azim=azimuth);ax.set_axis_off()
    fig.suptitle(model.robot.name)
    fig.savefig(path, dpi=150, bbox_inches='tight');plt.close(fig)


def run_robot_viewer(model: URDF, port: int):
    server = create_server(port, label=model.robot.name)
    try:
        server.scene.set_up_direction('+z')
        server.gui.configure_theme(control_layout='floating', control_width='medium', show_logo=False)
        scene = GpuArmScene(server, model)
        cfg = initial_configuration(model)
        scene.update_angles(cfg)
        dirty = threading.Event(); lock = threading.Lock()
        server.gui.add_markdown(f'## {model.robot.name}\n双臂独立控制；升降单位为毫米，转动单位为度。')
        controls = []
        joints = list(model.actuated_joints)
        for label, match in [('升降平台', lambda j: j.type=='prismatic'),
                             ('左臂 Nero', lambda j: j.name.startswith('left_')),
                             ('右臂 Nero', lambda j: j.name.startswith('right_'))]:
            with server.gui.add_folder(label, expand_by_default=label!='底盘轮子'):
                for index, joint in enumerate(joints):
                    if not match(joint):continue
                    factor = 1000. if joint.type=='prismatic' else 180./np.pi
                    if joint.type=='continuous':lo,hi=-180.,180.
                    else:lo,hi=joint.limit.lower*factor,joint.limit.upper*factor
                    handle = server.gui.add_slider(joint.name+(' / mm' if joint.type=='prismatic' else ' / °'),
                                                  min=lo,max=hi,step=1. if joint.type=='prismatic' else .1,
                                                  marks=((float(lo), f"{lo:.0f}"), (float(hi), f"{hi:.0f}")),
                                                  initial_value=float(np.clip(cfg[index]*factor,lo,hi)))
                    cfg[index]=handle.value/factor
                    controls.append((handle,index,factor))
                    def changed(event,index=index,factor=factor,handle=handle):
                        if event.client is None:return
                        with lock:cfg[index]=handle.value/factor;dirty.set()
                    handle.on_update(changed)
        reset = server.gui.add_button('恢复零位')
        fit = server.gui.add_button('适应画面')
        axes = server.gui.add_checkbox('显示关节坐标轴', initial_value=False)
        @axes.on_update
        def toggle(_):
            for handle in scene.axis_handles:handle.visible=axes.value
        @reset.on_click
        def zero(_):
            with lock:
                for handle,index,factor in controls:
                    handle.value=float(np.clip(0,handle.min,handle.max));cfg[index]=handle.value/factor
                dirty.set()
        def fit_client(client):
            with lock:positions=cfg.copy()
            model.update_cfg(positions)
            bounds=model.scene.bounds.copy()
            center=bounds.mean(axis=0)
            direction=np.array([2.8,2.,1.3]);direction/=np.linalg.norm(direction)
            right=np.cross([0.,0.,1.],direction);right/=np.linalg.norm(right)
            up=np.cross(direction,right)
            corners=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
            tangent=np.tan(np.deg2rad(42)/2)
            distance=1.15*max(np.ptp(corners@up)/(2*tangent),np.ptp(corners@right)/(2*tangent*client.camera.aspect*.73))+np.ptp(corners@direction)/2
            center+=right*distance*tangent*client.camera.aspect*.23
            client.camera.up_direction=(0,0,1);client.camera.fov=np.deg2rad(42)
            client.camera.position=center+direction*distance;client.camera.look_at=center
        @server.on_client_connect
        def connected(client):fit_client(client)
        @fit.on_click
        def on_fit(event):
            if event.client is not None:fit_client(event.client)
        scene.update_angles(cfg)
        print(f'Robot viewer: http://127.0.0.1:{server.get_port()}',flush=True)
        timer=threading.Event()
        while not timer.wait(1/60):
            with lock:
                if not dirty.is_set():continue
                positions=cfg.copy();dirty.clear()
            scene.update_angles(positions)
    finally:
        server.stop()
