"""Assemble supplied chassis and two identical Nero arms (no reflected meshes)."""
from pathlib import Path
from copy import deepcopy
import argparse
import json
import shutil
import xml.etree.ElementTree as ET
import trimesh
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / 'assets/robot/mobile_nero'

def pose(element):
    origin = element.find('origin')
    matrix = np.eye(4)
    if origin is not None:
        matrix[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        matrix[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    return matrix


def set_pose(element, matrix):
    origin = element.find('origin')
    if origin is None:
        origin = ET.SubElement(element, 'origin')
    origin.set('xyz', ' '.join(f'{x:.12g}' for x in matrix[:3, 3]))
    origin.set('rpy', ' '.join(f'{x:.12g}' for x in Rotation.from_matrix(matrix[:3, :3]).as_euler('xyz')))


def simplify_chassis(robot):
    """Freeze wheel assemblies and lump their geometry and inertia into base_link."""
    links = {link.get('name'): link for link in robot.findall('link')}
    incoming = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    frames = {'base_link': np.eye(4)}
    def frame(name):
        if name not in frames:
            joint = incoming[name]
            frames[name] = frame(joint.find('parent').get('link')) @ pose(joint)
        return frames[name]
    wheels = ['link_1', 'link_2', 'link_3']
    floor = float('inf')
    for name in wheels:
        for visual in links[name].findall('visual'):
            mesh = visual.find('geometry/mesh')
            geometry = trimesh.load(OUTPUT/mesh.get('filename'), force='mesh')
            vertices = geometry.vertices * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            transform = frame(name) @ pose(visual)
            floor = min(floor, float((vertices@transform[:3,:3].T+transform[:3,3])[:,2].min()))
    shift = np.eye(4); shift[2,3] = -floor
    names = [name for name in links if name not in ('tatai_link_1', 'tatai_link_2', 'tatai_link_3')]
    merged = ET.Element('link', name='base_link')
    inertias = []
    for name in names:
        transform = shift @ frame(name)
        for kind in ('visual', 'collision'):
            for original in links[name].findall(kind):
                item = deepcopy(original)
                set_pose(item, transform @ pose(original))
                merged.append(item)
        inertia = links[name].find('inertial')
        if inertia is not None:
            mass = float(inertia.find('mass').get('value'))
            ipose = transform @ pose(inertia)
            values = inertia.find('inertia').attrib
            tensor = np.array([[float(values['i'+a+b if a<=b else 'i'+b+a]) for b in 'xyz'] for a in 'xyz'])
            inertias.append((mass, ipose[:3,3], ipose[:3,:3]@tensor@ipose[:3,:3].T))
    mass = sum(item[0] for item in inertias)
    center = sum(m*c for m,c,_ in inertias)/mass
    tensor = sum(i+m*(np.dot(c-center,c-center)*np.eye(3)-np.outer(c-center,c-center)) for m,c,i in inertias)
    inertial = ET.SubElement(merged, 'inertial')
    cp = np.eye(4);cp[:3,3] = center;set_pose(inertial, cp)
    ET.SubElement(inertial, 'mass', value=str(mass))
    ET.SubElement(inertial, 'inertia', **{'i'+a+b: str(tensor[i,j]) for i,a in enumerate('xyz') for j,b in enumerate('xyz') if i<=j})
    lift = incoming['tatai_link_1']
    set_pose(lift, shift @ frame('tatai_base_link') @ pose(lift))
    lift.find('parent').set('link', 'base_link')
    lift.set('name', 'torso_joint1')
    for element in list(robot):
        if element.tag == 'link' and element.get('name') in names:
            robot.remove(element)
        elif element.tag == 'joint' and element.find('child').get('link') in names:
            robot.remove(element)
    robot.insert(0, merged)
    for element in robot.iter():
        if element.tag == 'link' and element.get('name') == 'tatai_link_1':
            element.set('name','torso_link1')
        if element.tag in ('parent','child') and element.get('link') == 'tatai_link_1':
            element.set('link','torso_link1')
    return floor


def build(base: Path, arm: Path):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    source = OUTPUT / 'sources'
    source.mkdir(exist_ok=True)
    shutil.copy2(base / 'twonero.urdf', source / 'chassis.urdf')
    shutil.copy2(arm / 'urdf/nero_description.urdf', source / 'nero.urdf')
    robot = ET.parse(source / 'chassis.urdf').getroot()
    robot.set('name', 'mobile_nero')
    nero = ET.parse(source / 'nero.urdf').getroot()
    converted = {}
    def meshes(tree, directory, category):
        for mesh in tree.iter('mesh'):
            name = mesh.get('filename')
            relative = 'meshes/' + name.split('/meshes/', 1)[1] if name.startswith('package://') else name
            path = directory / relative
            dest = OUTPUT / category / relative
            if path.suffix.lower() == '.dae':
                dest = dest.with_suffix('.stl')
            if str(path) not in converted:
                dest.parent.mkdir(parents=True, exist_ok=True)
                if path.suffix.lower() == '.dae':
                    scene = trimesh.load(path, force='scene')
                    scene.to_geometry().export(dest)
                else:
                    shutil.copy2(path, dest)
                converted[str(path)] = dest
            mesh.set('filename', str(dest.relative_to(OUTPUT)))
    meshes(robot, base, 'chassis')
    meshes(nero, arm, 'nero')
    floor = simplify_chassis(robot)
    # User-confirmed direct mounting: omit the two original spacer brackets.
    removed = {'tatai_link_2', 'tatai_link_3'}
    for element in list(robot):
        if element.tag == 'link' and element.get('name') in removed:
            robot.remove(element)
        elif element.tag == 'joint' and element.find('child').get('link') in removed:
            robot.remove(element)
    mounts = {}
    for side, sign in [('left', 1), ('right', -1)]:
        tree = deepcopy(nero)
        for element in list(tree):
            if element.get('name') in ('world', 'world_to_base_link'):
                tree.remove(element)
        for element in tree.iter():
            if 'name' in element.attrib:
                element.set('name', side + '_arm_' + element.get('name'))
            if element.tag in ('parent', 'child'):
                element.set('link', side + '_arm_' + element.get('link'))
            if element.tag == 'mimic':
                element.set('joint', side + '_arm_' + element.get('joint'))
        robot.extend(tree)
        # Direct platform faces Y=+/-100 mm; retain mounting center
        # X=61.95408 mm, Z=827.5 mm in tower mesh coordinates.
        xyz = [.07400408, sign * .100, -.0525]
        # Looking at each base from outside: left CCW, right CW.
        # Equivalent to Rx(-sign*pi/2) @ Rz(sign*pi/2), without Euler singularity warnings.
        rpy = [-sign*np.pi/2, np.pi/2, 0.]
        joint = ET.SubElement(robot, 'joint', name=side+'_arm_base_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='torso_link1')
        ET.SubElement(joint, 'child', link=side+'_arm_base_link')
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
        mounts[side] = {'parent': 'torso_link1', 'xyz_m': xyz, 'rpy_rad': rpy}
    # Explicit names and colors simplify conversion from multi-part CAD scenes.
    for link in robot.findall('link'):
        for i, visual in enumerate(link.findall('visual')):
            visual.set('name', f"{link.get('name')}_visual_{i}")
            if visual.find('material') is None:
                mat = ET.SubElement(visual, 'material', name=link.get('name')+'_color')
                ET.SubElement(mat, 'color', rgba='0.72 0.75 0.79 1')
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(OUTPUT/'mobile_nero.urdf', encoding='utf-8', xml_declaration=True)
    (OUTPUT/'mounts.json').write_text(json.dumps({'source_ground_z_m': floor, 'mounts': mounts, 'status': 'Mesh-plane placement; mounting-hole alignment and flange clocking require physical confirmation.'}, indent=2)+'\n')
    print(OUTPUT/'mobile_nero.urdf')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--arm', type=Path, required=True)
    args = parser.parse_args()
    build(args.base, args.arm)
