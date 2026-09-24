"""Build a true mirrored right JoyLo and a portable 14-DOF dual-arm URDF.

The supplied assembly is treated as LEFT per the owner's identification.
Reflection is about XZ: p'=S p, R'=S R S, revolute axis'=-S axis.
The mirrored mesh winding is reversed; every URDF rotation stays proper.
"""
from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import argparse
import xml.etree.ElementTree as ET

import numpy as np
from build_extrusion import build_profile
from build_joycon import make_shell, attach_joycon

ROOT = Path(__file__).resolve().parents[2]
DIRECTORY = ROOT / 'hardware/joylo_v2_7dof_arm/urdf'
REFLECTION = np.diag([1., -1., 1.])


def vector(text: str) -> np.ndarray:
    return np.fromstring(text, sep=' ')


def numbers(values: np.ndarray) -> str:
    return ' '.join(f'{v:.12g}' if abs(v) > 1e-11 else '0' for v in values)


def mirror_obj(source: Path, target: Path) -> None:
    """Reflect vertices and normals, reverse polygon winding, keep UV indexing."""
    lines = [f'# Mirrored across local XZ from {source.name}; derived visualization mesh.\n']
    for line in source.read_text().splitlines(keepends=True):
        parts = line.split()
        if not parts:
            lines.append(line)
        elif parts[0] in ['v', 'vn']:
            xyz = REFLECTION @ np.array([float(x) for x in parts[1:4]])
            tail = (' ' + ' '.join(parts[4:])) if len(parts) > 4 else ''
            lines.append(parts[0] + ' ' + numbers(xyz) + tail + '\n')
        elif parts[0] == 'f':
            lines.append('f ' + ' '.join(reversed(parts[1:])) + '\n')
        elif parts[0] not in ['mtllib', 'usemtl']:
            lines.append(line)
    target.write_text(''.join(lines))


def reflect_tree(tree: ET.Element) -> None:
    """Reflect every local transform and the axial joint vectors in place."""
    for origin in tree.iter('origin'):
        p = vector(origin.get('xyz', '0 0 0'))
        rpy = vector(origin.get('rpy', '0 0 0'))
        origin.set('xyz', numbers(REFLECTION @ p))
        # For XZ reflection, conjugating Rz(yaw) Ry(pitch) Rx(roll) gives
        # Rz(-yaw) Ry(pitch) Rx(-roll), avoiding Euler singularity roundoff.
        origin.set('rpy', numbers(rpy * np.array([-1., 1., -1.])))
    for joint in tree.iter('joint'):
        axis = joint.find('axis')
        if axis is not None:
            parity = 1 if joint.get('type') == 'prismatic' else -1
            axis.set('xyz', numbers(parity * REFLECTION @ vector(axis.get('xyz'))))


def prefix_names(tree: ET.Element, prefix: str) -> None:
    """Give every arm-local link, joint and visual a unique name."""
    for element in tree.iter():
        if element.tag in ['link', 'joint', 'visual', 'material'] and 'name' in element.attrib:
            element.set('name', prefix + element.get('name'))
        if element.tag in ['parent', 'child']:
            element.set('link', prefix + element.get('link'))
        if element.tag == 'mimic':
            element.set('joint', prefix + element.get('joint'))


def save(tree: ET.Element, path: Path) -> None:
    ET.indent(tree, space='  ')
    ET.ElementTree(tree).write(path, encoding='utf-8', xml_declaration=True)


def build(length: float = .20) -> Path:
    """Build arms on a 2550 extrusion; length is meters, plates are 5 mm thick."""
    if length <= 0:
        raise ValueError('Extrusion length must be positive')
    separation = length + .010
    source = ET.parse(DIRECTORY / 'joylo_v2_7dof.urdf',
                      parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))).getroot()
    source.set('name', 'joylo_left_7dof')
    # Original mesh is now explicitly LEFT. Use official left-arm preview ranges.
    r1 = ET.parse(ROOT / 'assets/robot/r1_pro_a2_2026/r1_pro_a2_2026.urdf')
    signs = np.array([1., 1., -1., 1., -1., -1., 1.])
    home = np.deg2rad([0., 0., 0., 0., 0., 90., 0.])
    for i in range(7):
        ref = r1.find(f"joint[@name='left_arm_joint{i+1}']/limit")
        low, high = float(ref.get('lower')), float(ref.get('upper'))
        if i == 1:
            low = -np.pi / 2
        if i in (5, 6):
            low, high = -np.pi / 2, np.pi / 2
        pair = home[i] + signs[i] * np.array([low, high])
        limit = source.find(f"joint[@name='joint_{i+1}']/limit")
        limit.set('lower', f'{min(pair):.12g}'); limit.set('upper', f'{max(pair):.12g}')
    make_shell()
    attach_joycon(source, 'left')
    save(source, DIRECTORY / 'joylo_v2_7dof.urdf')
    save(source, DIRECTORY / 'joylo_left.urdf')
    right = deepcopy(source)
    right.set('name', 'joylo_right_7dof_mirrored')
    # Right Joy-Con has ABXY above its stick; it is not a mirrored left layout.
    attach_joycon(right, 'right')
    reflect_tree(right)
    mirror_dir = DIRECTORY / 'mirrored'
    mirror_dir.mkdir(exist_ok=True)
    for mesh in right.iter('mesh'):
        original = (DIRECTORY / mesh.get('filename')).resolve()
        target = mirror_dir / original.name
        mirror_obj(original, target)
        mesh.set('filename', f'mirrored/{original.name}')
    save(right, DIRECTORY / 'joylo_right.urdf')
    dual = ET.Element('robot', name='joylo_dual_7dof')
    dual.append(ET.Comment('Original assembly is LEFT. RIGHT is a true XZ reflection. '
                           '2550 extrusion has user-specified dimensions and schematic slots. '
                           'Mount plates meet its ends; fasteners and hole fit are not verified. '
                           'Inferred limits; not calibrated motor or collision limits.'))
    base = ET.SubElement(dual, 'link', name='base_link')
    extrusion_name = f'extrusion_2550_{length * 1000:g}mm.obj'
    build_profile(DIRECTORY / 'support' / extrusion_name, length * 1000)
    visual = ET.SubElement(base, 'visual', name='extrusion_2550')
    ET.SubElement(visual, 'origin', xyz='-0.034827 0 0', rpy='0 0 0')
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'mesh', filename='support/' + extrusion_name,
                  scale='0.001 0.001 0.001')
    material = ET.SubElement(visual, 'material', name='extrusion_aluminum')
    ET.SubElement(material, 'color', rgba='0.62 0.66 0.70 1')
    for side, arm, sign in [('left', source, 1), ('right', right, -1)]:
        arm = deepcopy(arm)
        prefix_names(arm, side + '_')
        for element in arm:
            dual.append(element)
        mount = ET.SubElement(dual, 'joint', name=side + '_shoulder_mount', type='fixed')
        ET.SubElement(mount, 'parent', link='base_link')
        ET.SubElement(mount, 'child', link=side + '_base_link')
        ET.SubElement(mount, 'origin', xyz=f'0 {sign * separation / 2:.9g} 0', rpy='0 0 0')
    output = DIRECTORY / 'joylo_dual.urdf'
    save(dual, output)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--length', type=float, default=.20,
                        help='2550 extrusion length in meters')
    print(build(parser.parse_args().length))
