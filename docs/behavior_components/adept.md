# ADEPT 双臂环境

三个任务共用简单房间, 桌子, R1Pro 和固定桌侧相机. 代码针对 adept-sim 的单环境接口实现. 仿真生成, 检查, 采集和评估在 Linux GPU 服务器运行.

任务名:

- pick_book_to_bookcase
- put_object_in_hinged_jar
- load_and_place_tray

## 环境与资产

使用已安装的 behavior 环境, OmniGibson 和本仓库的 bddl3 应以 editable 方式安装. 数据根目录由 OMNIGIBSON_DATA_PATH 指定, 默认是仓库 datasets.

需要 R1Pro 机器人资产及以下模型:

| 对象 | category/model |
| --- | --- |
| 桌子 | desk/iotfzl |
| 书本 | hardback/aceozs |
| 桌面书柜 | bookcase/iwziew |
| 铰链罐 | hinged_jar/vzwhbg |
| 装入物 | bratwurst/pqfrrn |
| 托盘 | tray/gsxbym |
| 装载物 | plate/xfjmld |
| 托盘目标垫 | 固定薄型 PrimitiveObject, 类别 place_mat |

配置位于 OmniGibson/omnigibson/adept/configs. common.yaml 定义房间, 桌子, 机器人开始位姿, 侧视相机和阈值. 三个任务 YAML 定义对象, BDDL 绑定和实例.

build_env 根据 YAML 的 initial_state 和 sampling 自动生成全部实例. 桌面书柜采用原生 iwziew, 按底部高度对齐桌面并固定; 绿色目标垫固定在桌面另一侧. 物体采样避开这两类设施的占地. 布局与开口方向需要根据 check_env 画面确认. 当前代码未在本机执行仿真验证.

任务成功使用原生 BehaviorTask 的 BDDL goal 判定:

| 任务 | 成功条件 |
| --- | --- |
| 书本 | inside(book, bookcase) |
| 铰链罐 | inside(payload, jar) 且 not open(jar) |
| 托盘 | ontop(payload, tray) 且 ontop(tray, target_pad) |

支撑手选择, 重抓方式和操作顺序由操作者自由执行. 超时终止沿用原生设置. 生成初态的稳定性检查用于筛选可加载的场景.

## 生成和检查

在 BEHAVIOR-1K 仓库根目录执行一条 build_env 命令, 自动采样并生成实例 1–300:

```bash
TASK_NAME="pick_book_to_bookcase"

python -B -m omnigibson.adept.build_env \
  --task-name "${TASK_NAME}" \
  --num-instances 300 --seed 0 --headless
```

首次调试可将 --num-instances 设为 5. 也可以使用 --instance-indices 1 3 5 指定编号; 两个实例选择参数互斥. 省略实例选择参数时生成实例 1. 每个实例默认最多尝试 50 次, 可通过 --max-attempts 调整.

内部流程为: 读取 YAML → 生成候选初态 → 创建共用仿真场景 → 应用候选并进行物理检查 → 保存临时结果并验证重新加载 → 写入任务目录. 所有编号均按 sampling 范围自动采样, 首次运行直接生成实例 1.

任务 YAML 的 initial_state 定义物体支撑面, 基础姿态及实例说明, sampling 定义随机范围. 例如:

```yaml
sampling:
  book:
    xy_bounds: [[0.61, -0.35], [0.73, 0.35]]
    yaw_degrees: [-180.0, 180.0]
```

xy_bounds 是对象根坐标在场景世界坐标系中的 [xmin, ymin] 与 [xmax, ymax], 单位为米. yaw_degrees 是相对于 initial_state 基础姿态, 绕世界 Z 轴的旋转范围, 单位为度. 位置和朝向候选独立均匀采样, 高度根据当前姿态的包围盒及桌面支撑高度计算.

机器人开始位姿和默认关节姿态固定, 底盘动作置零. 房间, 桌子, 桌面书柜, 相机及目标垫保持一致. 三个任务分别改变 book, jar 与 payload, tray 与 payload 的位置及 yaw.

物理检查包括桌面边界, 物体与固定设施间距, 非支撑接触, BDDL 初始条件, 目标尚未满足, 物体速度及持续稳定性, 机器人位姿漂移. common.yaml 的 settle_steps 和 sampling_check_steps 定义生成阶段的落稳与检查窗口. 当前筛选未执行双臂 IK 或完整任务规划; 采样区域和铰链开盖空间需要通过实际操作确认.

每个实例的候选随机种子为 seed + instance_id. 在相同配置下, 分批生成同一编号使用相同候选序列; 物理结果受仿真版本与运行平台影响. 达到尝试上限仍无有效初态时报告拒绝原因, 正式任务目录保留原状态. 全部采样和重新加载检查通过后才发布生成文件.

输出结构:

```text
OMNIGIBSON_DATA_PATH/adept-task-instances/TASK_NAME/
├── scene_template.json
└── instances/
    ├── 1/
    │   ├── tro_state.json
    │   └── task_parameters.json
    ├── 2/
    │   ├── tro_state.json
    │   └── task_parameters.json
    └── .../
```

scene_template.json 保存对象创建信息和 BDDL 绑定. tro_state.json 保存任务对象状态及固定 robot_poses. task_parameters.json 保存实例说明, 指令, 生成检查阈值, 种子, 范围, 接受尝试编号, 候选位姿和实际落稳位姿.

任务目录已存在时使用 --overwrite. 此时按当前 YAML 和种子重新采样本次指定编号及目录内全部已有编号, 并重建共用模板, 保持实例与模板一致. 现有实例编号保留, 内容更新; 仅调整采样范围时也使用这一条 build_env 命令.

生成后直接检查实例:

```bash
python -B -m omnigibson.eval.eval \
  --task-name "${TASK_NAME}" \
  --mode check_env --instance-indices 1 2 3 4 5 --headless \
  --output-dir outputs/adept
```

每个实例只保存 outputs/adept/check_env/TASK_NAME/{instance_ID}_preview.mp4. 视频包含四路拼接画面, 默认 60 步, 30 FPS, 约 2 秒. 初始关系和稳定性检查结果在终端显示.

check_env 可通过 --max-steps 300 指定每个实例检查 300 步, 默认 30 Hz 下对应 10 秒. 省略该参数时读取 common.yaml 的 check_steps. 视频 FPS 使用实际 action_frequency; 默认动作和渲染频率为 30 Hz, 物理频率为 120 Hz. 修改 check_steps 不影响 build_env 的 sampling_check_steps.

## JoyLo 采集

```bash
python -B joylo/scripts/launch_og.py \
  --task_name pick_book_to_bookcase --instance_id 1 \
  --recording_path /path/to/recordings/books.hdf5

python -B joylo/scripts/run_joylo.py
```

采集使用原生控制器和 assisted grasp, 动作进入记录 wrapper 前将 base_action_idx 置零. reset 保持当前实例, 重复采集相同初态. 更换实例通过重启服务端并指定 instance_id 完成. 机器人默认关节姿态来自 eval/r1pro.yaml, 与 JoyLo 初始姿态一致.

HDF5 保留原生 action/state 和 transitions, 并在各 demo 属性中记录 instance_id 与 task_parameters. success 数据集逐步保存 BDDL 成功结果, 与 action 一一对应.

检查点和回退使用 JoyLo 原生物理状态恢复流程. 中止和回退数据沿用 JoyLo 的保存流程. 头部, 双腕和固定桌侧相机均为 480×480 RGB, 采集期间保存动作和状态, 相机视频在回放时生成.

## 离线回放

```bash
python -B joylo/scripts/replay_data.py \
  /path/to/recordings/books.hdf5 \
  --task pick_book_to_bookcase --episode-id 0 --qa --headless
```

输出到 books_episode_0/: 四路独立 MP4, preview.mp4, frame_indices.json; --qa 额外输出 qa.json, 包含执行时的逐步 BDDL 成功结果及最终结果.

回放使用原生 DataPlaybackWrapper, 逐帧恢复状态, 施加动作并渲染. 第一帧为初态, 后续帧分别对应源状态及其动作; frame_indices.json 明确标注 source_state_index, action_index 和源采样时间. 回放采用原生微小物理步长, 源时间来自采集频率. 四路视频采用相同帧序.

侧视相机固定在场景坐标系. 回放读取 HDF5 中保存的相机配置, 保持已采集数据的视角可重复性. 修改 YAML 中的相机设置立即影响新的 check_env 和采集记录.

## 策略评估

```bash
python -B -m omnigibson.eval.eval \
  --task-name pick_book_to_bookcase --mode train \
  --instance-indices 1 --max-steps 1800 \
  --host 127.0.0.1 --port 8000 --headless \
  --output-dir outputs/adept
```

ADEPT 的 train 与 check_env 均使用直接实例 ID. public_test/hidden_test 的官方索引映射保留用于 challenge 任务. ADEPT 复用 WebsocketPolicy 通信, 接收当前 R1Pro 控制器的原生 23 维动作. OpenWAM EEF20 输出需在策略适配端转换为该动作协议.

观测使用原生平铺键, 机器人名称为 robot, 另含 instruction, task_name, instance_id 及相机相对位姿. --policy local 使用保持当前姿态动作. --write-video 在评估时保存可回放 HDF5, 完成后按终端给出的命令离线生成视频. 该路径使用 30 Hz 采集频率.

结果 JSON 包含 BDDL 成功状态, 满足和未满足的目标谓词, 超时状态和底盘位移. 当前场景与 BDDL 绑定已有更新, 服务器上已有场景应先通过 build_env --overwrite 重建, 再进行采集和回放.
