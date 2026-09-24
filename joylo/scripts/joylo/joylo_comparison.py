"""Model-coordinate alignment for original LEFT JoyLo and mirrored RIGHT JoyLo."""
from __future__ import annotations

from pathlib import Path
import numpy as np
from yourdfpy import URDF

REPO_ROOT = Path(__file__).resolve().parents[2]
R1_URDF = REPO_ROOT / 'assets/robot/r1_pro_a2_2026/r1_pro_a2_2026.urdf'
JOYLO_HOME_DEG = np.array([0., 0., 0., 0., 0., 90., 0.])
SIGNS = {'left': np.array([1., 1., -1., 1., -1., -1., 1.]),
         'right': np.array([1., -1., 1., 1., 1., -1., -1.])}
JOYLO_SIGNS = SIGNS['left']
R1_ARM_NAMES = tuple(f'left_arm_joint{i}' for i in range(1, 8))


def arm_sides(model: URDF) -> tuple[str, ...]:
    """Infer whether this is a left, mirrored right, or dual JoyLo model."""
    if 'left_joint_1' in model.actuated_joint_names:
        return ('left', 'right')
    return ('right',) if 'right' in model.robot.name else ('left',)


def joylo_joint_names(model: URDF, side: str) -> tuple[str, ...]:
    prefix = side + '_' if model.num_dofs == 14 else ''
    return tuple(f'{prefix}joint_{i}' for i in range(1, 8))


def shared_to_joylo(degrees: np.ndarray, side: str = 'left') -> np.ndarray:
    """R1-reference angles to native JoyLo angles; offsets are not encoder calibration."""
    return JOYLO_HOME_DEG + SIGNS[side] * np.asarray(degrees)


def joylo_to_shared(degrees: np.ndarray, side: str = 'left') -> np.ndarray:
    return (np.asarray(degrees) - JOYLO_HOME_DEG) / SIGNS[side]


def mirror_shared(degrees: np.ndarray, source_side: str) -> np.ndarray:
    """Equal native angles yield exact spatial reflection under mirrored joint axes."""
    target = 'right' if source_side == 'left' else 'left'
    return joylo_to_shared(shared_to_joylo(degrees, source_side), target)


def native_config(model: URDF, values: np.ndarray) -> np.ndarray:
    """Pack per-arm shared angles into the URDF's actuated-joint order."""
    result = np.zeros(model.num_dofs)
    indices = {name: i for i, name in enumerate(model.actuated_joint_names)}
    for side, arm_values in zip(arm_sides(model), np.atleast_2d(values)):
        for name, angle in zip(joylo_joint_names(model, side), shared_to_joylo(arm_values, side)):
            result[indices[name]] = np.deg2rad(angle)
    return result


def unpack_native(model: URDF, native_degrees: np.ndarray) -> np.ndarray:
    """Convert a native command vector to one shared-angle row per arm."""
    by_name = dict(zip(model.actuated_joint_names, native_degrees))
    return np.array([joylo_to_shared(np.array([by_name[n] for n in joylo_joint_names(model, side)]), side)
                     for side in arm_sides(model)])


def default_native(model: URDF) -> np.ndarray:
    """Photo-reference pose, separate from the R1/encoder alignment zero.

    J3 uses the owner's selected viewer-zero pose as native zero and startup pose.
    J6 and J7 also start at viewer zero as requested by the owner.
    The right arm is its spatial mirror, not a second independently guessed pose.
    """
    left = np.zeros(7)
    reference = {'left': left, 'right': mirror_shared(left, 'left')}
    return np.rad2deg(native_config(model, np.array([reference[s] for s in arm_sides(model)])))


def reference_limits(model: URDF, side: str = 'left') -> tuple[np.ndarray, np.ndarray]:
    names = tuple(f'{side}_arm_joint{i}' for i in range(1, 8))
    by_name = {joint.name: joint for joint in model.robot.joints}
    missing = set(names) - by_name.keys()
    if missing:
        raise ValueError('R1 Pro A2 requires 7 arm joints; missing: ' + ', '.join(sorted(missing)))
    return (np.rad2deg([by_name[n].limit.lower for n in names]),
            np.rad2deg([by_name[n].limit.upper for n in names]))


def shared_limits(joylo: URDF, r1: URDF | None, side: str = 'left') -> tuple[np.ndarray, np.ndarray]:
    by_name = {j.name: j for j in joylo.actuated_joints}
    joints = [by_name[n] for n in joylo_joint_names(joylo, side)]
    a = joylo_to_shared(np.rad2deg([j.limit.lower for j in joints]), side)
    b = joylo_to_shared(np.rad2deg([j.limit.upper for j in joints]), side)
    low, high = np.minimum(a, b), np.maximum(a, b)
    if r1 is not None:
        rlow, rhigh = reference_limits(r1, side)
        # User-expanded JoyLo joints retain their ranges independently of R1.
        for i in (0, 2, 3, 4):
            low[i], high[i] = max(low[i], rlow[i]), min(high[i], rhigh[i])
    return low, high


def r1_config(model: URDF, shared_degrees: np.ndarray,
              side: str = 'left', sides: tuple[str, ...] | None = None) -> np.ndarray:
    """Set selected R1 arms; base, torso and any unselected arm remain at zero."""
    values = np.atleast_2d(shared_degrees)
    selected = sides or (('left', 'right') if len(values) == 2 else (side,))
    if len(values) != len(selected):
        raise ValueError('One shared-angle row is required for each selected arm')
    cfg = np.zeros(model.num_dofs)
    indices = {name: i for i, name in enumerate(model.actuated_joint_names)}
    for arm, angles in zip(selected, values):
        lo, hi = reference_limits(model, arm)
        for i, value in enumerate(np.deg2rad(np.clip(angles, lo, hi))):
            cfg[indices[f'{arm}_arm_joint{i+1}']] = value
    return cfg


def calibration_shared(model: URDF) -> np.ndarray:
    """User-selected left calibration angles, with an exactly mirrored right arm."""
    left = np.array([-90., 90., -90., -90., 90., 60., 90.])
    poses = {"left": left, "right": mirror_shared(left, "left")}
    return np.array([poses[side] for side in arm_sides(model)])
