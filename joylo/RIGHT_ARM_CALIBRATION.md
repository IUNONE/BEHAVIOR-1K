# 右臂关节标定

从 `~/BEHAVIOR-1K/joylo` 执行。需要已安装 JoyLo 的 Python 环境
（`numpy`、`tyro`、`PyYAML`、`dynamixel-sdk` 和本项目 `gello`）。
按仓库约定使用 `behavior`；仅有查看器的 `joylo-urdf` 环境不代表标定依赖已安装。

```bash
cd ~/BEHAVIOR-1K/joylo
conda activate behavior
python scripts/calibrate_joints.py --help
python scripts/calibrate_joints.py --arm right --robot R1Pro \
  --gello-name right_arm --baudrate 2000000 --port /dev/cu.usbserial-FTBIHTHX
```

串口填写连接电机的电脑上的实际串口；上述是本机此次检测到的 macOS 串口。
Linux 常见为 `/dev/ttyUSB0`，以实际设备为准。

1. 关闭 DYNAMIXEL Wizard 对串口的连接，接好电源，托住手臂。
2. 程序显示 ID 9–17，按 Enter 连接。驱动会关闭这些电机的扭矩。
3. 对照 [右臂零位](imgs/R1pro_zero_R.jpg) 摆好，保持不动，按 Enter 采集。
4. 对照 [右臂标定位](imgs/R1pro_calibration_R.jpg) 摆好，保持不动，按 Enter 采集。
5. 成功后生成 `configs/joint_config_right_arm.yaml`。

可用 Ctrl+C 取消。程序不发送目标位置，不修改 Homing Offset 或 Drive Mode。
不要在两次采集之间断电、重启或更改电机配置。

## 文件内容

- `robot` / `arm`：机器人与已标定的侧别。
- `angle_unit`：degrees。
- `joints.ids`：本次真实读取的电机 ID，右臂为 9–17。
- `joints.offsets` / `joints.signs`：九台电机的角度偏移与方向。
- `calibration`：两次电机角度读数（度）与对应参考角度；不是原始 ticks。

换算公式：`关节角度 = (电机角度 - offset) * sign`。
保留官方算法，将偏移取整到 90° 的倍数；这不是精密连续零位标定。
两次参考姿态的偏移必须一致，且每台电机必须有可辨识的移动。
通过此检查后仍需逐关节验证方向、角度和实物姿态。

默认 `--arm both` 保留双臂流程。`--arm left` 仅标定左臂。
不指定设备名时，单臂输出 `joint_config_default_right.yaml` 或
`joint_config_default_left.yaml`，双臂仍为 `joint_config_default.yaml`。
已有文件不会自动覆盖；重新标定可换设备名，或明确添加 `--overwrite`。

原版 `run_joylo.py` 和 `test_joints.py` 尚未适配单臂，不要直接给它们使用九电机配置。
本阶段仅采集标定，后续再进行单臂读取验证与仿真联动。
