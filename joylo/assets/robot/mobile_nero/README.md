# Mobile Nero：移动底盘 + 升降平台 + 双臂

```bash
conda activate joylo-urdf
python scripts/joylo/view_joylo_urdf.py --robot mobile_nero --without_joylo
```

默认地址 http://127.0.0.1:8080。可独立控制左右各 7 个关节和升降平台。关节角使用源 Nero URDF 的零位和限位；升降单位为 mm。车轮、滚子、底盘与固定立柱统一合入 `base_link`，不再保留车轮 link 或自由度；车轮外观保持。升降零位对应源模型最高位置，范围为 -620～0 mm。初始姿态为左右臂 J2 均 -90°，其余关节 0°；“恢复零位”仍恢复全部关节 0°。

## 文件与来源

- `mobile_nero.urdf`：总装入口，18 links、15 可动关节（双臂 14 + 升降 1）。
- `chassis/meshes/`：来自用户提供的 `Downloads/base/twonero`。
- `nero/meshes/`：来自用户提供的 `Downloads/agx_arm_urdf/nero`。
- `sources/*.urdf`：原始两份 URDF，未经修改，留作比对；其 mesh URI 不是本目录的运行入口。
- `mounts.json`：安装参数记录；生成参数在 `scripts/joylo/build_mobile_nero.py` 中。
- `preview.png`：总装初始姿态预览。

DAE 视觉网格转换为 STL，保留场景内变换，不需要 ROS package 路径或 Downloads 目录才能运行。材质使用统一简化颜色，不保留 DAE 纹理；碰撞网格与惯性参数保留。两条手臂是同一 Nero 实物模型的两次安装，不是镜像网格。源文件仅包含裸臂，未额外添加夹爪。

## 安装关系

源底盘 `link_1` 是车轮；真正的升降平台叫 **`tatai_link_1`**（总装中改名为 `torso_link1`）。两臂固定关节都挂在该平台下，所以升降带动两臂一起移动。

坐标采用底盘 X 前、Y 左、Z 上。按用户确认，去掉侧支架后直接安装到平台两侧，平台坐标下：

| 安装关节 | xyz (m) | rpy (rad) |
|---|---|---|
| `left_arm_base_joint` | `[0.07400408, 0.100, -0.0525]` | `[-π/2, π/2, 0]` |
| `right_arm_base_joint` | `[0.07400408, -0.100, -0.0525]` | `[π/2, π/2, 0]` |

平台侧面为 Y=±100 mm，保留原安装中心的 X/Z 坐标。双臂底座中心间距 200 mm，Nero 底座 +Z 朝身体外侧。

这是依据网格平面得到的初版安装位置。安装孔匹配、全工作空间碰撞尚未由实物确认；移动底盘控制器、遥操作映射和新 JoyLo 硬件不在本步骤中。

## 重建与检查

首次创建环境可使用仓库现有 `hardware/joylo_v2_7dof_arm/urdf/environment.yml`。重建需要 `pycollada`；当前环境已安装，运行导出的 STL 总装不依赖它。

```bash
python scripts/joylo/build_mobile_nero.py \
  --base /path/to/base/twonero \
  --arm /path/to/agx_arm_urdf/nero
python scripts/joylo/view_joylo_urdf.py --robot mobile_nero --without_joylo --check
python scripts/joylo/view_joylo_urdf.py --robot mobile_nero --without_joylo --save assets/robot/mobile_nero/preview.png
```

## 命名和底座整合

按 R1 Pro 命名：`torso_joint1` / `torso_link1` 是升降轴；`left_arm_base_joint` / `left_arm_base_link`、`left_arm_joint1..7` / `left_arm_link1..7` 是左臂，右臂采用 `right_arm_` 前缀。底盘、固定塔柱、车轮与滚子在原关节零位下合入 `base_link`，惯性张量通过平行轴定理合并，保留原视觉和碰撞几何。

`base_link` 的 XY 原点沿用源底盘，Z 原点在轮胎网格最低点；源模型本来几乎就在地面（差约 1e-10 m）。底座安装从外侧看左臂逆时针、右臂顺时针各转 90°，Nero 底座电源接口侧（局部 -X）朝 +Z。

平台左右的 `part_008_part.stl` / `part_009_part.stl` 是源模型带的侧支架，不是重复机械臂。已按用户确认从总装中删除，不参与视觉、碰撞或惯性；原始文件保留用于追溯。

## 关节限位核对（2026-09-23）

已逐项核对 [AgileX 官方 Nero URDF](https://github.com/agilexrobotics/agx_arm_urdf/blob/main/nero/urdf/nero_description.urdf)，左右臂全部一致。面板端点标签四舍五入为整数，实际 min/max 仍使用原始弧度换算值，不因显示取整扩大限位。拖动精度仍为 0.1°。

| 关节 | 原始范围（rad） | 面板近似范围（°） |
|---|---|---|
| J1 | -2.70526 ~ 2.70526 | -155 ~ 155 |
| J2 | -1.74 ~ 1.74 | -100 ~ 100 |
| J3 | -2.75 ~ 2.75 | -158 ~ 158 |
| J4 | -1.01 ~ 2.14 | -58 ~ 123 |
| J5 | -2.75 ~ 2.75 | -158 ~ 158 |
| J6 | -0.73 ~ 0.95 | -42 ~ 54 |
| J7 | -1.5707963 ~ 1.5707963 | -90 ~ 90 |

这里确认的是官方模型限位；尚未读取实物控制器中的限位配置。
