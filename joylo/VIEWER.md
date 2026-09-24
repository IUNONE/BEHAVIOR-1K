# 装配查看器与迁移说明

此目录是 BEHAVIOR-1K 的 JoyLo 子项目。后续开发在 BEHAVIOR-1K 的 `adept-sim` 分支进行。

## 启动查看器

使用已有的 `joylo-urdf` 环境，从本目录运行：

```bash
cd ~/BEHAVIOR-1K/joylo
conda activate joylo-urdf
python scripts/joylo/view_joylo_urdf.py
```

打开终端输出的网址，默认 http://127.0.0.1:8080 。
查看器支持双臂 JoyLo / R1 Pro、各关节滑块、电机 ID、恢复零位和恢复标定位。
它只显示模型，不连接实物；显示的零位和方向不能直接当作电机标定值。

Mobile Nero：

```bash
python scripts/joylo/view_joylo_urdf.py --robot mobile_nero --without_joylo
```

环境依赖清单保存在 `hardware/joylo_v2_7dof_arm/urdf/environment.yml`。
查看器使用独立环境；BEHAVIOR 仿真使用官方 `behavior` 环境。

## 文件位置

| 目录 | 内容 |
|---|---|
| `scripts/joylo/` | 自定义查看器、模型生成脚本、角度映射和电机标签 |
| `hardware/joylo_v2_7dof_arm/` | JoyLo 原始 OBJ、可编辑 F3D、左右臂及双臂 URDF、参考图和装配文档 |
| `assets/robot/r1_pro_a2_2026/` | 查看器使用的 R1 Pro A2 模型及来源说明 |
| `assets/robot/mobile_nero/` | Mobile Nero 模型、原始模型、网格和安装参数 |
| `hardware/mobile_nero/` | 从原工作区保留的未提交资产快照；当前查看器入口不使用该目录 |
| `scripts/`、`gello/` | BEHAVIOR 官方标定、遥操作和仿真程序 |

保留原相对目录结构，脚本以自身位置定位资产。装配文档中的命令均从
`~/BEHAVIOR-1K/joylo` 执行，而不是 BEHAVIOR-1K 根目录。

更多说明：

- [JoyLo 模型与装配](hardware/joylo_v2_7dof_arm/urdf/README.md)
- [电机 ID 与零位的历史说明](hardware/joylo_v2_7dof_arm/urdf/MOTOR_IDS_AND_ZERO.md)
- [Mobile Nero 模型](assets/robot/mobile_nero/README.md)
- [R1 Pro 模型来源](assets/robot/r1_pro_a2_2026/README.md)

## 迁移范围与来源

迁移日期：2026-09-24。
来源：本机 `~/brs-ctrl`，HEAD `f6c2b63`，相对 `e4e150a` 新增文件
，以及未提交的 `hardware/mobile_nero/`。
原仓库保留。旧六轴硬件控制脚本未迁入；官方的标定和遥操作实现继续作为后续基础。
资产来源说明和已有文档保留在各目录中。`hardware/mobile_nero/` 是历史快照，
其中复制的 JoyLo 文档也属于历史资料。

## 当前硬件进度与下一步

用户已经组装右臂，并设置波特率 2 Mbps、ID 9–17。实体零位与方向尚待标定。
旧装配文档中“尚未组装 / 尚未写入 ID”的记载，以及 `motor_id_map.json` 中的旧状态，
反映当时的建模进度；以这里的硬件进度为准，不能将标签状态当作设备检测结果。

下一阶段：为官方流程补充仅右臂的标定与读取支持，核对实物与参考姿态，
然后接入 OmniGibson。官方默认流程仍按双臂读取，迁移本身不提供右臂运行支持。

## 已有检查入口

以下命令供后续在已有查看器环境中检查模型使用，不连接电机：

```bash
python scripts/joylo/view_joylo_urdf.py --check
```

## 右臂实物跟随

```bash
cd ~/BEHAVIOR-1K/joylo
conda activate joylo-minimal
python -B scripts/joylo/view_joylo_urdf.py --live-arm right --port /dev/cu.usbserial-FTBIHTHX --baudrate 2000000 --joint-config configs/joint_config_right_arm.yaml
```

关闭 Wizard 并托住手臂，按终端提示回车连接。只读取 ID 9–17，初始化关闭扭矩，之后不发送运动目标。
页面显示原始读数、标定角度、J1–J7 和双电机差值；左臂固定，滑块与恢复姿态按钮禁用。
JoyLo 按实测角度显示，R1 与禁用的滑块限制在各自展示范围内，超限会显示提示；以读数面板为准。
通信失败冻结画面，修复连接后重新启动；Ctrl+C 退出释放串口。网页端口另用 --viewer-port 指定。
腕部翻转安装后，J6 原生角度改为 90°减去标定后的关节角度；J7 仍使用原映射符号，180°零位已在 URDF 中体现。
本功能尚未进行实体跟随验证，请先逐关节小幅移动核对方向。
