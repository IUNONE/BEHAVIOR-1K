"""Proposed 18-motor IDs for the reconstructed seven-DoF JoyLo.

Existing project code documents only the legacy 16-motor version. These labels
are proposals, not hardware discoveries. No raw encoder zero is assumed.
"""
from dataclasses import dataclass
from yourdfpy import URDF
from joylo_comparison import arm_sides


@dataclass(frozen=True)
class MotorLabel:
    side: str
    motor_id: int
    joint: int
    visual: str
    role: str
    placement_confirmed: bool = False
    raw_zero_tick: int | None = None


# Order follows the legacy proximal-to-distal grouping, adding upper-arm roll.
SLOTS = (
    ('rear_shoulder_motor', 1, '主电机候选 / rear'),
    ('shoulder_motor', 1, '从电机候选 / front'),
    ('shoulder_pair_a', 2, 'A侧，主从待核对'),
    ('shoulder_pair_b', 2, 'B侧，主从待核对'),
    ('upper_arm_motor', 3, '上臂旋转'),
    ('elbow_motor', 4, '肘关节'),
    ('forearm_motor', 5, '前臂旋转'),
    ('wrist_bend_motor', 6, '腕关节 / 带背面惰轮'),
    ('wrist_rotation_motor', 7, '末端旋转'),
)


def motor_labels(model: URDF) -> list[MotorLabel]:
    """Return one provisional ID per motor body, never label a passive flange."""
    names={v.name for link in model.robot.links for v in link.visuals}
    result=[]
    for side in arm_sides(model):
        prefix=side+'_' if model.num_dofs==14 else ''
        for local,(visual,joint,role) in enumerate(SLOTS):
            name=prefix+visual
            if name not in names:
                raise ValueError(f'Motor body missing from URDF: {name}')
            result.append(MotorLabel(side,local+(0 if side=='left' else 9),joint,name,role))
    return result
