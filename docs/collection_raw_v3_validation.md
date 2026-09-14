# raw v3 实施与验收记录

更新：2026-09-11。高频 raw 录制已完成并通过当前现场验收；本页分别保留自动测试、只读基线和实际遥操证据。同步标定、扩展操作和收尾调度优化单独跟进。

证据保存约定：本页、测试和一个真实报告 fixture 随仓库保存；下列 `data/`、`logs/` 和 `/tmp/` 路径是本机证据，未纳入版本控制，不保证长期存在。需要长期保留时应复制完整 session。

## 实施范围

新入口使用 schema v2 配置，保存 raw v3。三路 RGB 独立实时编码，逐帧索引保留原始时间与 metadata；机器人通过只接收的 TCP 30000 逐包记录，原始 bytes 与解码字段关联；夹爪/附加反馈独立保存。新增持续时钟对照、前后缓冲、边界和完整性清单。

移除 observation scheduler、在线配对质量阈值、训练导出和 action 派生；保留 v1/v2 读取/检查及 v2 正常采集后的归档恢复。raw v3 最初实施时，SDK 后端和当时的时间报告经 SHA-256 比较逐字节未变。后续文档维护为时间报告添加了历史阶段说明及版本链接，原实验统计保留。

## 2026-09-11 高频 raw 现场验收

**结论：高频原始录制里程碑完成。** 用户完成两个实际遥操会话、共 3 条轨迹；第一会话连续录制并保留两条，第二会话覆盖未录制、录制与回中后收尾。三条均 `complete / kept`，独立全量视频解码、逐帧索引、原始机器人包与解码值、文件和会话 checksum 均通过，共 **32,403 帧**。保存范围（含缓冲）无缺帧、无重复/倒退帧号、无无效状态。

两次 Space 的 `start_time/end_time` 定义有效示教区间，原始文件额外保留前后约 0.5 秒。收尾校验位于该区间之外。按 2026-09-11 用户确认，收尾调度不是本阶段录制的阻塞项；此结论不放宽存储完整性检查，也不把故障片段自动当作有效轨迹。

本机证据：

- 会话 A：`data/collections/native60_motion_demo/session_20260911T192946.912958Z_30a31564/`，两条轨迹；复查 `logs/native60_motion_audit_20260911/{REPORT.md,summary.json}`。
- 会话 B：`data/collections/native60_motion_demo/session_20260911T200403.065775Z_6f6eaf03/`，一条轨迹；复查 `logs/native60_live_monitor_audit_20260911T200403/{REPORT.md,summary.json,integrity_check.json}`。
- 两次会话记录的源码 SHA-256 和 collection 配置一致。此次文档维护没有重新连接设备或重跑自动测试，自动测试结果仍为已通过的 211 项。

### 轨迹、帧率与容量

帧数顺序为 wrist / fixed_1 / fixed_2。频率使用来源接收时间计算，视频播放时间仅定位；MB 为十进制。三路均为 640×480 H.264、CRF 18、veryfast、每路两线程、GOP 30，保留逐帧 metadata 和原生状态。

| 轨迹 | Space 间隔 / 保存跨度 | 三路帧数 | robot / gripper / auxiliary 数量 | 三路视频 / episode 大小 |
| --- | ---: | --- | --- | ---: |
| A / 1 | 70.25 / 71.24 s | 4250 / 4250 / 4257 | 17578 / 1423 / 356 | 24.12 / 98.61 MB |
| A / 2 | 56.65 / 57.65 s | 3438 / 3439 / 3445 | 14215 / 1151 / 288 | 20.02 / 79.25 MB |
| B / 1 | 51.08 / 52.08 s | 3106 / 3106 / 3112 | 12845 / 1040 / 261 | 17.37 / 71.56 MB |

- 三路相机实际约 **59.65 / 59.65 / 59.76 fps**；三条机器人报告频率分别 **246.7175 / 246.5643 / 246.6271 Hz**；夹爪约 **19.97 Hz**、附加反馈约 **5 Hz**。这不证明内部测量刷新率或经过标定的采样时刻。
- 每条视频队列峰值不超过 **31/64**，状态队列不超过 **138/4096**，无溢出；峰值包含一次提交的约 0.5 秒预缓冲，不能直接视为持续积压或延迟。
- 完整会话 A / B 分别 **206.54 / 82.35 MB**，含录制外等待与准备日志。只将录制窗口控制日志计入轨迹时约 **89–90 MB/分钟**。场景、运动和等待时长影响容量，继续保留 10 GiB 门槛。
- 会话 A 两条视频合计 44.13 MB，原生机器人 JSONL 约 76.65 MB、二进制报告约 24.93 MB、帧索引约 30.38 MB；后续节省空间可评估文本无损压缩，本轮未改变存储编码。

### 控制发送与收尾范围

| 窗口 | 发送 Hz | 发送间隔 P99 / 最大 | ACK P99 / 最大 |
| --- | ---: | ---: | ---: |
| A / 1 保存区间（含缓冲） | 60.0000 | 17.525 / 22.440 ms | 1.251 / 4.800 ms |
| A / 2 保存区间（含缓冲） | 60.0000 | 17.487 / 21.110 ms | 1.147 / 3.612 ms |
| B 录制前 | 59.9998 | 17.374 / 20.689 ms | 1.052 / 2.653 ms |
| B 两次 Space 之间 | 60.0004 | 17.353 / 18.699 ms | 1.070 / 2.385 ms |
| B 收尾校验 | 59.9240 | 18.334 / 27.292 ms | 5.085 / 14.991 ms |

- 所有窗口无失败返回。B 的 Space 区间最大发送间隔 18.70 ms，说明本次 60 fps 捕获/编码、原生状态与遥操并发满足正常录制吞吐；未测极限带宽或所有长时工况。ACK 含本机调度与通信时间，不是纯网络延迟。
- A 两次收尾耗时 **18.14 / 14.79 秒**，最大发送间隔 **146.21 / 172.42 ms**；相机共缺失 55 个帧号，首次分别在结束 Space 后约 **18.05 / 15.00 秒**，均位于 episode 保存范围之外。A / 2 的停顿前后仍有非零速度请求，运行影响仍保留记录。
- B 收尾耗时 **13.34 秒**，始终零速度；无缺帧事件、最大发送间隔 **27.29 ms**。源码和配置未变，不能据此宣称 A 的偶发尖峰已修复。
- 只读软件复现不连接任何设备：同进程校验时 60 Hz 定时线程最大间隔 **99.81 ms**，独立进程校验时 **16.74 ms**。这支持后续排查校验执行隔离，不等于已完成产品修复；结果在 `logs/native60_motion_audit_20260911/software_scheduling_probe.json`。
- 现阶段约定结束录制后回中、等待校验，再处理 K/X/N。校验后的帧与调度不是示教内容；它们不影响已验证的 raw，但若未来要求校验期间仍持续平滑遥操，需要另行优化。
- A 两条内臂非零命令与夹爪 moving 回读重叠约 **1.15 / 0.30 秒**，B 约 **0.28 秒**，仅覆盖少量联动。A 两条间没有 N 复位记录，三条均 K 保留；当前配置下的 N/X 扩展验收仍未完成，不将其计入本阶段完成证据。

## 60 fps 默认配置与真机只读验证

本节是 2026-09-10 默认配置升级时的只读基线；下面的初版 mock/只读实验仍保留其当时的 30 fps 条件。此次只把三路配置的 `fps` 从 30 改为 60；序列号、640×480 分辨率、H.264 CRF 18 / veryfast / 两线程 / GOP 30、前后缓冲及队列、机器人来源、夹爪和运动配置均未改变。

测试时间：2026-09-10（America/Chicago）；会话 UTC 起点为 2026-09-11T02:30:48。命令：

```bash
./scripts/run_xarm_collection.sh --observe-only --headless --auto-record --duration 60 \
  --task rgb60_readonly --output /tmp/spacemouse_fps60_acceptance/collections
```

该模式不读取 HID、不使能、不复位、不发送机械臂或夹爪动作；同时运行真实三路捕获、实时视频编码、被动 TCP 30000、只读夹爪查询与低频附加反馈。产物：

`/tmp/spacemouse_fps60_acceptance/collections/rgb60_readonly/session_20260911T023048.637157Z_51e08550/episodes/episode_000001`

| 来源 | 保存数量 | 主机到达频率 | 最大接收间隔 |
| --- | ---: | ---: | ---: |
| wrist | 3609 | 59.6494 fps | 18.264 ms |
| fixed_1 | 3609 | 59.6479 fps | 18.266 ms |
| fixed_2 | 3615 | 59.7561 fps | 18.690 ms |
| robot | 14918 | 246.5584 Hz | 6.825 ms |
| gripper | 1210 | 19.9998 Hz | 53.746 ms |
| auxiliary | 300 | 4.9978 Hz | 202.393 ms |

- `complete / kept`，原生保存跨度 **60.501 秒**（含实际可用前后缓冲）。三路共 **10,833 帧**，accepted 与 written 相同；帧号无缺口/重复，未发生写入丢帧，状态无无效记录。
- 按相机 SDK 时间戳独立计算为 **59.6504 / 59.6505 / 59.7573 fps**，与主机到达频率吻合。逐帧 `actual_fps` 中位数为 **59.648 / 59.649 / 59.756**。60 是请求模式，不是严格等间隔；不能只凭编码器的 60 fps 标记判断实际捕获。
- 每个视频索引均为实际写入帧，PTS 为 `index / 60`；三路全部保留设备 `frame_timestamp`、`sensor_timestamp` 和 `actual_fps` 元数据，另存 SDK 时间域及主机接收时间。PTS 仍不是采样时间。
- 三路视频队列峰值各 **5/64**，状态队列峰值 **17/4096**，会话保留 **78 个时钟锚点**。无连续性事件或故障；检查没有用补帧使三路计数相等。
- 关闭后独立检查通过：视频全量解码、每帧 PTS/索引、原始机器人字节对照、文件和会话 checksum；无 PNG staging、observation 网格或训练 action。
- 本次 episode **74,641,640 bytes**，含共享日志的 session **74,675,613 bytes**，按保存跨度折算约 **74.06 MB/分钟**（十进制）。这是静止场景实测，运动与纹理会影响压缩大小；不能作为批量动态采集容量保证。
- 修改配置后再次运行 **211 项非仿真回归，全部通过**（28.403 秒）；日志 `/tmp/spacemouse_fps60_acceptance/regression.log`。验收汇总、完整性结果及逐项复核分别为同目录 `summary.json`、`integrity_check.json`、`verification.log`。

这次验证不包含 GUI、机械臂运动及其控制发送节奏；约 246.6 Hz 仍是报告到达频率，不是已证明的底层测量刷新率。该只读试验当时未覆盖连续真实操作；后续现场证据见上文。跨设备同步标定仍未完成。本次文档更新仅引用上述已完成结果，没有重跑硬件或自动测试。

## 初版自动验证

**211 项非仿真测试全部通过**。结果日志：`/tmp/spacemouse_raw_v3_regression.log`。包含：

- 真实报告 fixture、784/804-byte 布局、拆包/粘包、同批次相同接收时间、时间回退、无效浮点和不可用监测字段；未知包前已经收到的完整包仍保存。
- 三路不同帧率、独立计数、逐帧一一对应、原始包偏移；不存在 PNG staging、20 Hz 网格或补帧。
- 夹爪失败查询区间、缺失 metadata、设备时间域切换、主机时间跳变及原始值保留。
- 前后缓冲、连续两条 mock 轨迹、复位边界、来源故障、单个无效状态、来源缺口诊断。
- 慢编码器/有界队列、写盘 ENOSPC、编码进程退出、后台慢校验与控制日志隔离；异常视频保存可解码前缀检查及不完整文件清单。
- 新/旧 raw 检查，v2 归档恢复、乱序读取与原有 SDK、HID、启动、复位、夹爪、停止和录制编排回归。

重现本轮集合：

```bash
PYTHONPATH=src:tests .venv/bin/python - <<'PY'
from pathlib import Path
import unittest
names = [p.stem for p in sorted(Path("tests").glob("test_*.py"))
         if p.stem != "test_mujoco_assets"]
suite = unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromName(n) for n in names)
result = unittest.TextTestRunner(verbosity=1).run(suite)
raise SystemExit(not result.wasSuccessful())
PY
```

最初的全量 discovery 共运行 219 项，其中一个 MuJoCo 资源测试因缺少 `robot_descriptions/xacrodoc` 报错，12 项跳过。未安装或扩展仿真依赖；最终 211 项集合明确排除此仿真资源模块。不同阶段测试数量不能直接相减作为本轮覆盖变化结论。

纯标准库环境（`python3 -S` + `PYTHONPATH=src`）可导入 raw reader、collection-check、collection-finalize 与采集 CLI。可编辑安装已刷新，旧 collection-export 命令入口已移除，依赖未升级。`git diff --check` 通过。

## 原诊断报告回归

将现有 `/tmp/spacemouse_timestamp_audit/report30000.bin` 的 **14,792 个完整 784-byte 报告**重新拆包、解析，与独立保存的诊断 JSONL 逐项比较：设备原始时间、关节、TCP 旋转向量、关节速度、TCP 速度均一致，无余尾。

`tests/fixtures/xarm30000_784.bin` 保存该实验的首个实际包，配套 JSON 保存独立期望值、固件 2.5.5 和 SHA-256。804-byte 兼容使用合成包及已安装 SDK 1.18.4 的结构大小、字段偏移与解码结果检查；**本次现场实际收到的仍全部是 784-byte 包，不把 804 的测试描述为实机验证**。

## 初版 30 fps：60 秒并发 mock

入口：`--dry-run --headless --auto-record --duration 60`，当时默认三路 640×480、30 fps，合成报告约 250 Hz，独立夹爪约 20 Hz。

产物：
`/tmp/spacemouse_raw_v3_acceptance/raw_v3_concurrent/session_20260910T222901.223277Z_d52a79f7/episodes/episode_000001`

| 来源 | 保存数量 | 实际到达频率 |
| --- | ---: | ---: |
| wrist | 1815 | 29.9997 fps |
| fixed_1 | 1815 | 29.9997 fps |
| fixed_2 | 1815 | 29.9996 fps |
| robot | 15125 | 250.0000 Hz |
| gripper | 1210 | 20.0003 Hz |

`complete / kept`，无缺口事件、无无效状态。操作窗口 59.950 秒，实际原生保存跨度 60.497 秒（受启动可用缓冲与尾部影响）；视频/index/原包/JSONL 计数一致。视频队列最高各 2，状态队列最高 14；69 个会话时钟锚点。原始轨迹文件总量 35,336,452 bytes，不含共享会话日志。

## 初版 30 fps：60 秒真机只读录制

入口：`--observe-only --headless --auto-record --duration 60`。使用真实三路相机、被动 TCP 30000、独立只读夹爪查询和既有低频附加反馈；没有 HID、使能、复位或运动调用。

产物：
`/tmp/spacemouse_raw_v3_acceptance/raw_v3_readonly/session_20260910T223202.883838Z_165cd424/episodes/episode_000001`

| 来源 | 保存数量 | 实际到达频率 | 最大接收间隔 |
| --- | ---: | ---: | ---: |
| wrist | 1814 | 29.9867 fps | 34.776 ms |
| fixed_1 | 1814 | 29.9867 fps | 36.299 ms |
| fixed_2 | 1813 | 29.9639 fps | 35.060 ms |
| robot | 14934 | 246.8253 Hz | 7.001 ms |
| gripper | 1210 | 20.0000 Hz | 55.079 ms |
| auxiliary | 300 | 4.9978 Hz | 202.087 ms |

`complete / kept`，无缺口事件、无无效状态。操作窗口 59.960 秒，实际原生保存跨度 60.500 秒。独立关闭后检查通过：三路视频全量解码、PTS/索引、机器人原包对照、文件与会话 checksum 均一致。没有 observation 网格或 action 文件。三路数量不同，没有通过补帧强制相等。

视频队列最高 wrist/fixed_1/fixed_2 为 2/1/2，状态队列最高 14；72 个时钟锚点覆盖会话开始、周期和结束。轨迹原始文件总量 70,787,483 bytes，不含共享日志。统计汇总：
`/tmp/spacemouse_raw_v3_acceptance/summary.json`。

相机均为 D455 / USB 3.2；wrist 与 fixed_1 固件 5.16.0.1，fixed_2 固件 5.17.0.10。SDK 时间域为 global_time，实际 global-time 选项读回已保存。逐帧保留 frame_timestamp、sensor_timestamp、actual_fps、time_of_arrival 和 backend_timestamp；RGB actual_exposure 不可用时保持 null。设备曝光/读出元数据与 SDK 映射时间同帧保存。arrival/backend 数值保留原样，未以未经验证的单位转换参与对齐。

246.8253 Hz 仍是本次接收频率；静止数值相同不能证明或否定每次测量刷新。该次没有测试运动指令与编码/报告的并发延迟。

## 初版 30 fps：实现短时复验

在补齐编码器版本、已接受计数和异常视频诊断后，再运行一次 10 秒真机只读录制：

`/tmp/spacemouse_raw_v3_acceptance/raw_v3_final_readonly/session_20260910T224235.786493Z_2ff95c03/episodes/episode_000001`

三路 315/315/314 帧、2592 个原生报告、210 条夹爪及 50 条附加反馈，15 个会话时钟锚点；complete/kept，无缺口事件。关闭后再次检查通过，accepted 与 written 计数一致，FFmpeg/ffprobe 4.2.7 版本信息随元数据保存。该次仍只读，不包含机械臂运动验收。

## 强制中断

仅对本任务新建的 mock 进程组发送 SIGKILL（包含其编码器），未连接硬件。结果：
`/tmp/spacemouse_raw_v3_interruption/result.json`。

三个视频、三个 frame JSONL、机器人 binary/JSONL、夹爪/附加 JSONL 和初始 meta 均保留。meta 仍为 recording，检查明确返回“没有成功完成标记”。没有将可能只写入部分数据的文件升级为 complete。此快速中断实验不保证每路 MP4 都已有完整可播放 fragment；正常捕获故障路径另外保存实际解码前缀检查。

## 独立后续事项

- 高频 raw 录制及当前录制期并发验收已完成。N 再次复位、X 弃用、更长会话和更多臂爪联动工况作为扩展操作验收保留。
- 收尾校验的调度优化后置，不阻塞当前里程碑；历史尖峰未标记修复，继续保留边界和故障记录。
- 设备 timestamp 的物理事件、内部测量刷新、跨设备同步和固定偏差依然未知。后续在本仓库实现独立离线处理，负责时间映射、20 Hz observation、质量检查与派生产物；具体算法和输出尚未实现，不覆盖 raw。C5 完成状态保持不变，后处理按 [C6 计划](../PROGRESS_PLAN.md#c6--本仓库离线数据处理计划中尚未实现) 跟进。

旧事实见 [v2 现场记录](collection_video_field_validation.md)；当前契约见 [raw v3 格式](collection_format.md)。时间报告保留历史数值，并标注其对应阶段。
