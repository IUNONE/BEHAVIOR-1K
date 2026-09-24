# 电机 ID 与零位核对

## 结论及版本

当前模型每臂 9 台 XL330：J1 两台、J2 两台、J3–J7 各一台；背面惰轮不计为电机。
**左 0–8、右 9–17 是用户选定的七轴装配编号方案；后续按模型标签设置实物 ID。目前尚未开始组装或写入 ID。**

本仓库 `scripts/joylo/{real_joylo,sim_joylo,joylo_move,r1_to_joylo}.py` 和
[官方 Setup and Calibration](https://behavior-robot-suite.github.io/docs/sections/joylo/setup_and_calibration.html)
使用的是旧六轴配置：左 0–7、右 8–15。
旧代码主电机为左 `[0,2]`、右 `[8,10]`，从电机为左 `[1,3]`、右 `[9,11]`。
不能把旧版右臂从 8 开始的表直接用于 18 台电机。

## 查看器中的装配编号表

每个电机盒显示设置时应采用的 ID；标签随电机壳体移动。
“显示每台电机 ID”开关可隐藏标签。J2 的 A/B 侧对应关系尤其需要确认。

| 关节 | 左侧 ID | 右侧 ID | 组织方式 |
|---|---|---|---|
| J1 | 0 后置 / 1 前置 | 9 后置 / 10 前置 | 两台共轴；主/从候选 |
| J2 | 2 A侧 / 3 B侧 | 11 A侧 / 12 B侧 | 两台共轴；A/B 与主从待确认 |
| J3 | 4 | 13 | 新增上臂旋转 |
| J4 | 5 | 14 | 肘部 |
| J5 | 6 | 15 | 前臂旋转 |
| J6 | 7 | 16 | 腕部，前驱动法兰 + 后惰轮 |
| J7 | 8 | 17 | 末端旋转 |

这是沿旧版近端→远端顺序扩展、在上臂旋转处增加一台电机的方案。
每台电机的 `raw_zero_tick` 都是 null，见 `motor_id_map.json`。

## `scripts/calibration` 实际校准什么

该目录只有 `calibrate_joycons.py`：采集 Nintendo Joy-Con 拇指摇杆中心和行程，
不包含 Dynamixel 电机 ID 配置或电机零位表。

电机关节参考在 `brs_ctrl/joylo/joylo_arms/joylo_arms.py` 中建立：

```text
q_robot = (p - p_init) * sign * π / 2048 + q_reset
```

- `p_init` 是构造控制器时实际读取的位置，而不是仓库里预先保存的固定数值。
- `q_reset` 是所采用参考姿态的机器人关节角。旧脚本显式给出的左侧为
  `[1.56,2.94,-2.54,0,0,0]` rad，右侧为 `[-1.56,2.94,-2.54,0,0,0]` rad。
- 旧脚本 signs 左侧 `[-1,-1,1,1,1,1]`，右侧 `[-1,-1,1,1,-1,1]`；官方要求按安装方向核对。
- `dxl/position.py` 先为从电机设置反向，清除 homing offset，再读主从位置差并设置从电机偏移。
  因此这里的 p 坐标还受驱动模式及 homing offset 影响。

若要计算映射中机器人 q=0 对应的编码器位置，需要已有实际 `p_init`：

```text
p_at_robot_zero = p_init - q_reset * 2048 / (sign * π)
```

没有实测 p_init 和新版七轴 signs/q_reset，就不能给出每台电机真实零位。
也不能把当前显示角度直接换成 ticks 或统一填写 2048。

## 旧教程的装配参考角

[旧版分步装配教程](https://behavior-robot-suite.github.io/docs/sections/joylo/step_by_step_assembly_guidance.html)
在指定图示姿态要求：左肩第一组 90°、右肩第一组 270°；第二组 90°；后续四关节 180°。
这是六轴机构的**装配参考轴角**，不是当前七轴机构已确认的零位表，新增上臂旋转也没有可直接套用的条目。

## 接下来如何更新零位

先核对实物 ID、每个双电机组的主从及方向，再在同一个明确参考姿态下记录各台
Present Position、Drive Mode 和 Homing Offset。用它们建立七轴 p_init/sign/q_reset 映射，
然后再定义 URDF 零位和显示偏移。

本次只添加 ID 核对标签，**没有更改现有初始姿态、URDF joint origin、显示偏移或硬件零位**。
原本“共同角归零”仅为 R1 参考角归零，不是 Dynamixel 编码器归零。
