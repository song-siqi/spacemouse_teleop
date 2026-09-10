# xArm6 三路 RGB 与 20 Hz 原始数据采集

2026-09-10：默认视觉存储已接入 raw v2 三路 H.264 视频；代码与离线验证见 [视频接入验收](collection_video_validation.md)，随后单条现场视频录制已通过，并完成一次 N 返回遥操，见 [现场记录](collection_video_field_validation.md)。此前 PNG 采集软件、静止只读采集、首条实际遥操录制和离线导出已经验证。静止 60 秒录制保存了 1201 组 observation、3603 张 RGB PNG；首条实际遥操保存 **1288 组 observation、3864 张 RGB PNG、5473 条原生状态，20 Hz 网格跨度 64.35 秒**，完整性检查通过，状态为 `complete / kept`。最新代码回归 215 项通过。连续多条实际录制和机械臂/夹爪同时运动仍待验收。

实际机械臂运动的六个窗口共 44.317 秒，关节/TCP 各 1322 对相邻应答全部有超过检查阈值的位姿变化，变化约 30 Hz，支持本轮 20 Hz 位姿来源时效；控制发送保持约 60 Hz。夹爪实际 271…841 pulse，应答约 20 Hz，有效 moving 窗中的位置变化约 18.8 Hz，不能说每次查询都得到新位移。设备内部测量时刻仍未知。详见 [实际运动验证](collection_motion_validation.md)。

本入口使用原 xArm SDK 后端控制机械臂、夹爪和复位；`backends/xarm_sdk/` 内容保持不变。现有 demo 和仿真入口的默认行为不变；新的 coordinator 只在采集入口显式接入。频率证据见 [验证记录](collection_frequency_validation.md)，原始字段见 [数据格式](collection_format.md)。

## 环境与配置

在仓库根目录使用现有 Python 3.12.14 环境：

```bash
cd /home/siqis3/workspace/spacemouse_teleop
uv pip install --python .venv/bin/python -e '.[hardware,xarm,collection]' -c requirements/collection-validated.txt
```

本机已验证的采集依赖包括 `numpy==2.5.3`、`opencv-python==5.0.0.93`、`pyrealsense2==2.58.4.10922`；SDK 保持 `1.18.4`，PySpaceMouse 保持 `2.1.0`。完整组合见 [版本约束](../requirements/collection-validated.txt)。复用本机现有 HID 库、设备访问规则及网络设置，普通账号运行即可；采集安装不需要 MuJoCo、ROS2 或训练环境。

默认 [采集配置](../configs/spacemouse_xarm6_collection.json) 独立声明相机、采样、质量及输出参数，通过 `robot_config` 引用完整的 `spacemouse_xarm6_demo.json`。它不合并仿真配置，不读取仿真启动器的配置环境变量，也不把相机字段加进旧 SDK 配置。

| 参数 | 当前配置 |
| --- | --- |
| 保存频率 `sample_hz` | 20 Hz，单条 episode 内固定 |
| 关节/TCP 只读查询 `robot_poll_hz` | 30 Hz |
| 夹爪 | live 复用现有 worker；observe-only 独立只读查询目标 20 Hz |
| 相机 | 每路 RGB 640×480、30 fps；只保存选中的 20 Hz 帧 |
| 来源最大接收 age | 相机 100 ms、关节/TCP 100 ms、夹爪 120 ms |
| 相机间主机接收时差上限 | 50 ms |
| 视觉存储 | `video_after_episode`：临时 PNG → 正常结束后后台编码/校验 → 三路 MP4；旧配置缺项为 PNG |
| 有界写入队列 | 128；三路 PNG 使用固定三个后台编码线程 |
| 默认输出 | 仓库下 `data/collections/` |
| 磁盘保留空间 | 10 GiB（10,737,418,240 bytes） |
| 会话时限 | 默认 0，即无限；退出由 Q 或 Ctrl-C 控制 |

配置的 `quality_validation` 已记录 60 秒静止与 64.35 秒单条遥操通过，阈值数值没有放宽；连续多条与臂爪同时实际运动仍待验收，不等于覆盖所有工况或完成现场标定。配置中的 `robot_config` 和 `output_root` 相对采集配置文件解析；`--output` 可以指定其他采集磁盘。

三个角色均为必需，已按用户确认绑定，两个固定视角地位相同：

| 角色 | RealSense 序列号 |
| --- | --- |
| `wrist` | `215122252880` |
| `fixed_1` | `311322303615` |
| `fixed_2` | `311322303806` |

不依赖 USB 枚举顺序；GelSight Mini 不初始化也不采集。重新接线或换相机后，用独立诊断核对预览内容和角色：

```bash
# 只枚举，不打开 RGB 流，也不连接机器人。
.venv/bin/spacemouse-realsense-diagnose

# 打开配置中的三路 RGB；Q 结束。这个诊断不连接机器人。
.venv/bin/spacemouse-realsense-diagnose --config configs/spacemouse_xarm6_collection.json --preview
```

## 开始一条真机轨迹

```bash
./scripts/run_xarm_collection.sh --task pick_place
```

`--task` 必填，写入任务元数据。可附加 `--config`、`--ip` 或 `--output`；脚本从项目环境启动。一次运行一个采集/遥操进程。

1. 首先检查 FFmpeg、ffprobe 和 libx264；缺少时在机器人连接/复位前退出。通过后连接、打开三路相机，预览显示 `RESETTING`。
2. **先打开夹爪并确认，再复位机械臂**。上条轨迹持有的物体会在这一步释放；启动和每次 N 都使用同一既有复位流程。
3. 复位完成后显示 **WAITING FOR SPACEMOUSE**。轻推鼠标旋钮然后完全松开，同时释放两个按钮，让程序取得有效的回中报告。
4. 等待显示 **TELEOP ACTIVE**，此时鼠标已可以控制机械臂，录制尚未开始。
5. 点击预览窗口使它获得键盘焦点，再按 Space 开始录制。确认显示 `recording` 且样本数增长。
6. 再按 Space 结束，等待后台压缩及全量校验完成（可继续遥操），再按 K 保留或 X 标记弃用。按 N 准备下一条，或者在当前姿态重新按 Space 开始一个新的连续片段。

当前 SpaceMouse 在完全静止时可能尚无 HID 报告。**出现预览不等于遥操已经激活，Space 也不是机械臂使能键。** 若复位后一直没有有效回中报告，现有 30 秒启动等待会退出。首次 live 尝试曾在这个阶段超时：复位完成，但没有 `teleop_started` 或录制 episode。界面增加明确状态与提示后，用户第二轮已完成上述实际遥操录制；回中等待问题不再作为当前阻塞。

| 预览按键 | 行为 |
| --- | --- |
| Space | 遥操激活后开始/结束录制；复位及等待回中阶段不能录制 |
| K | 保留已结束轨迹；需要先等压缩/校验完成 |
| X | 标记已结束轨迹为弃用，原始文件仍保留 |
| N | 当前轨迹结束且已 K/X 后，开夹爪→复位→回中，准备下一条 |
| Q / 关闭预览 | 退出，先停止机械臂，再处理相机、写入和既有归档任务 |
| Ctrl-C | 退出；正在录制的片段标记不完整 |

SpaceMouse button 0 长按打开夹爪、button 1 长按关闭，松开或双键同按保持当前位置。录制键不占用这两个按钮。N 会关闭旧运动后端并创建新实例，相机、输入和会话记录保留；旧后端关闭失败时结束运动会话。

## 只读与离线检查

真实相机与机器人只读状态采集，不读取 HID，也**不会使能、打开夹爪、复位或发送运动**：

```bash
./scripts/run_xarm_collection.sh --task readonly_check --observe-only

# 无窗口自动录制一条 60 秒检查片段；正常完成后自动保留。
./scripts/run_xarm_collection.sh --task readonly_check --observe-only --headless --auto-record --duration 60 --output logs/collection_checks
```

`--observe-only` 的 N 不会执行复位。需要完全不接硬件时使用合成输入、相机与状态：

```bash
./scripts/run_xarm_collection.sh --task mock_check --dry-run --headless --auto-record --duration 3 --output logs/collection_mock
```

`--headless` 和 `--auto-record` 仅允许这两种检查模式；headless 还要求正数 `--duration`。live 入口需要预览和显式录制操作。它们使用不同 `capture_mode`，不会把只读或模拟记录伪装成实际示教。

频率诊断与离线重算见 [频率验证记录](collection_frequency_validation.md)。机器人查询应答到达 30/20 Hz 已有静止和实际遥操证据；运动段也核对了实际位姿变化。主机收到新应答仍不单独证明设备内部时钟或采样频率。

## 原始数据、故障与存储

目录结构：

```text
<output>/<task>/session_<UTC>_<id>/
  session.json                 配置、代码/依赖版本、设备及时间基准
  control.jsonl                整个会话输入/命令/反馈和复位事件
  episodes/episode_000001/
    meta.json                  完整性、保留状态、结束原因、统计
    observations.jsonl         20 Hz 选中图像与绝对状态及来源引用
    raw_states.jsonl           原生关节/TCP/夹爪/附加反馈流
    videos/{wrist,fixed_1,fixed_2}.mp4
    _staging/                  录制/归档暂存；成功提交后自动清理
```

使用主机 monotonic 50 ms 网格，选取接收时间不晚于网格时刻的最近来源。图像设备时间和时间域另外保留，关节、TCP、夹爪各有自己的查询区间；这是可追溯的软件配对，不是硬件同时曝光/测量。附加 report 仍约 5 Hz，不重标成新的 20 Hz 测量。绝对关节、完整 TCP、夹爪 raw position 保留，实时采集不生成训练 actions。

采集来源过期、缺少视角、队列满或写盘错误会停止当前录制并标记不完整，**遥操仍继续**，便于处理手上的物体。机器人/输入故障则走原有停止退出路径。主动 Q/关窗使用独立退出信号，即使按键队列已满也能请求退出；预览进程意外崩溃只标记录制故障，不冒充用户退出指令。故障不会自动续录；恢复后需先处理当前片段，再显式开始新片段。K 不会把 `incomplete` 改成 `complete`。完整性与保留状态是两个独立字段，也不代表任务成功标签。

空间检测在运动开始前及后台写入线程执行，录制开始只读取内存缓存；不足 10 GiB 时拒绝新录制或结束当前片段，不等待磁盘操作阻塞控制。根目录 `data/`（含采集和导出）与 `logs/`（仅保留 `.gitkeep`）是被 Git 忽略的本机产物区；总结性验证文档保存在 `docs/` 随仓库维护。这些目录不是自动备份，外部 `--output` 也不受本仓库忽略规则管理。X 只做标记，不释放空间。

首条实际遥操 session 共约 **1.529 GB**，按 64.35 秒网格折合约 **1.426 GB/分钟（85.54 GB/小时）**，包含实际 60 Hz 控制日志；静止检查为 1.449 GB/分钟。实际场景会改变占用。大批量采集可将 `--output` 指向有足够容量的数据磁盘。上述为旧 PNG 容量。默认新配置录制时仍需约 1.4 GB/分钟的临时空间，结束后才压缩为三路视频并清理 PNG；最终容量与耗时见 [视频接入验收](collection_video_validation.md)。归档期间 Space、K/X、N 不推进流程，最多归档一条，不同时录制下一条。

归档失败保留 `_staging/`，本条显示错误，遥操继续。先 K/X 处理当前片段后可显式开新片段；采样完整但归档失败的 v2 数据可离线重试：

```bash
.venv/bin/spacemouse-collection-finalize /absolute/path/to/episode_000001
```

只接受已验证 `capture_complete: true` 的新版片段，或重验完整片段后清理遗留暂存；不修复采样缺口/输入故障/录制中断，不迁移旧 PNG。成功后才删除本条暂存；清理失败记录警告且完整视频保留。Q 先停机器人：若仍在录制则不完整，若已正常结束则归档可继续。归档总期限为 `max(120 秒, 2×录制时长)`，退出等待剩余期限；超时终止编码并保留 PNG。

本机沿用已有 FFmpeg 4.2.7/ffprobe/libx264，不增加训练依赖。若使用旧 PNG 模式，配置 `visual_storage.mode=png`；缺项也为 PNG，不进行视频工具前置检查。

## 检查和离线导出

从终端输出找到一条 `episode_...` 目录，先检查：

```bash
COLLECTION_EPISODE=/absolute/path/to/episodes/episode_000001
.venv/bin/spacemouse-collection-check "$COLLECTION_EPISODE"
```

导出必须同时满足 `complete` 和 `kept`，图像与原生来源引用需要完整。导出目录为空或不存在，且不能放在 raw episode 内：

```bash
# 旧 raw 桥接格式：明确选择哪一路固定相机作为 image。
.venv/bin/spacemouse-collection-export "$COLLECTION_EPISODE" data/exports/pick_place_20 --format legacy --fps 20 --fixed-view fixed_1

# 已有轻量训练适配器格式：显式降采样到 10 Hz，重新计算下一状态标签。
.venv/bin/spacemouse-collection-export "$COLLECTION_EPISODE" data/exports/pick_place_10 --format light --fps 10 --fixed-view fixed_2
```

`wrist` 总是对应 `wrist_image`，所选固定相机对应 `image`，另一固定视角仍留在 raw 中。兼容训练状态为六关节 rad 加 `gripper_raw`，action 为下一步绝对状态；末帧没有伪造下一状态，因此 N 个有效 observation 最多产生 N−1 对。旧 CSV 中误称 mm/m 的历史字段只在桥接时明确映射，新 raw 单位保持正确。

`--format derived` 提供关节 delta、TCP 位移/相对旋转以及夹爪派生，必须显式传入 `--gripper-open` 和 `--gripper-close`，映射/阈值随导出版本记录。当前 demo 端点是临时控制值，不能当作毫米开口或夹持力标定。20/10 Hz 是不同数据版本，派生不跨轨迹、复位或故障断点。

真实静止 episode 已得到：20 Hz 的 1200 对、10 Hz 的 600 对。已有 OpenPI-facing 适配器已读取 RGB batch `[4,480,640,3]` 及 state/actions `[4,7]`。这验证了字段和读取接口；随后已在独立环境验证 H.264 视频的原生 LeRobot v2.1/v3.0 加载，见 [兼容记录](lerobot_video_compatibility.md)。采集仍输出本项目 raw 数据，正式 LeRobot 导出和训练没有加入此次实现。该静止验证片段不含示教动作。

## 下一轮现场验收

最新单条视频轨迹通过全量检查，结束后完成压缩/校验与 K 保留，并在一次 N 后重新进入遥操。日志只有一条 episode，因此下一轮仍需实际保存第二条及更多轨迹，覆盖 K/X、重复 N 和片段边界。近似归档窗口的发送仍约 60 Hz；更多动作及精确归档阶段统计继续验收。臂爪同时实际运动也仍需单独检查，不能把同一会话分别操作过两者当作同时运动验收。

运动示例之后再扩展不同姿态/接触工况，持续核对 20 Hz 来源年龄、重复、图像和磁盘队列；在相同操作条件下比较采集前后的控制节奏。
R1/R2、正式 TCP/负载标定和物理故障停止仍按 [进度计划](../PROGRESS_PLAN.md) 保留待办，不因静止采集通过而标记完成。当前任务继续暂停 MuJoCo 和 GelSight 检查。
