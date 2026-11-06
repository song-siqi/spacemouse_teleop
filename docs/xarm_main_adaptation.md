# xArm 真机适配 main 新架构

日期：2026-11-01。基础提交为 `d31d0b0`，实施位于独立工作树
`/home/siqis3/workspace/spacemouse_teleop_xarm_merge`，分支为
`dev/real/xarm6-main-adaptation`。原真机工作区及其未提交修改保持原样。

## 已实现的行为

- 使用 `robot_teleop` 架构；当前位置启动 demo，H 先停止并关闭旧实例，再打开夹爪、回 home、等待六轴回中和双键释放；默认不限时。
- 平移遥操保持零角速度和向下姿态检查；pose 工具读取同一套工具标定。main 的 R1 输入就绪和 R2 反馈有效期限制保留。
- 采集入口保持自动打开夹爪、复位、等待回中，随后 Space 开始/结束，K 保留、X 弃用、N 准备、Q 停止退出。H 只属于 demo。
- SDK 1.18.4，TCP `[0,0,172,0,0,0]` mm/rad，负载/质心 `[0.991,14.5,-3.23,61.24]` kg/mm。只检查，不修改控制器参数。
- wrist `246322303938` 为 30 fps；fixed_1 `151422251501` 和 fixed_2 `231622300915` 为 60 fps，均为 640×480。
- `configs/xarm_real.json` 和原采集脚本引用同一份三相机生产配置；生产磁盘保留门槛为 10 GiB。

新环境：

```bash
uv sync --frozen --extra spacemouse --extra xarm --extra realsense --extra recording
```

启动和专项诊断：

```bash
./scripts/run_xarm_demo.sh
./scripts/run_xarm_translation.sh
./scripts/run_xarm_collection.sh --task pick_place
.venv/bin/python -m robot_teleop.cli.xarm_frequency --help
.venv/bin/python -m robot_teleop.cli.realsense_diagnose --help
```

实际运动需用户在场并核实场景；本轮验证不连接机器人、HID 或相机。

## raw v4 的真机约定

每条 episode 包含 `meta.json` 和来源声明，视频/索引为 `<role>.mp4`、
`<role>.jsonl`，状态为 `robot.jsonl`、`gripper.jsonl`、`report.jsonl`，
原始机器人包为 `robot.bin`；控制证据另有 `commands.jsonl`。
会话继续保留 `session.json`、`control.jsonl`、`events.jsonl` 和 `clock_anchors.jsonl`。

顶层索引记录来源时间（秒）、主机接收时间、session 时间和双方时钟段；
`values` 保留原始字段，包括机器人整数微秒、接收批次、相机帧号/metadata、
夹爪查询起止和原始 pulse。关节为 rad，TCP 为 mm/旋转向量 rad；
通用公开状态的米/四元数或归一化夹爪不替换这些原始字段。

`start_time/end_time` 精确记录两次 Space；requested/actual capture 范围保留
前后各 0.5 秒上下文，准备/复位/故障/时钟边界切断旧缓冲。来源参数和历史标定
随配置保存。没有延迟扣除、时钟拟合、插值、对齐 observation 或训练 action。

每条写入排空后先保存 `finalizing`；独立进程执行视频全量解码、尺寸/PTS/索引、
计数、checksum、xArm 原包与解码值及共享时钟证据核对，再原子提交 `complete`。
原始字节不能只靠 checksum 通过而忽略字段差异。公开 check 只读，不提升未完成片段。
校验期限为 `max(120 秒, 2 × 操作时长)`；超时终止所属校验/FFmpeg 进程组并保留数据。
来源事件消费和控制日志不等待校验；writer 无法关闭时停止会话，不创建下一控制实例。

孤立无效响应保存诊断但不刷新健康时间；必需来源有效反馈过期后停止录制。
用户处理当前片段、来源恢复后才能显式开始下一条；采集故障不向运动后端发送新调用。
机器人/夹爪控制通信或输入故障仍执行停止退出路径。R2 的 `waiting_for_feedback`
单独计数，不能把 60 Hz 控制循环标成 60 次实际发送。

## 验证记录

2026-11-01 软件验收结果：

- 315 项非仿真自动测试通过（33.016 秒），覆盖公共层、真机控制、raw v4、实际 xArm 采集适配器、路由与故障恢复。仿真引擎测试未运行；`test_multi_robot` 仅运行 `ContractsTests`。
- 额外 headless/mock 采集通过：526 帧三路视频、875 条机器人状态、70 条夹爪状态，最终 `complete / kept`，无采集错误。不连接硬件，实际控制发送数为零。
- 5 条历史真机数据离线回放，共 42,373 帧、61,112 条机器人状态、4,946 条夹爪状态。4 条完整源轨迹通过 raw v4 完整检查；夹爪故障源轨迹保持 incomplete，未因回放成功而提升完整性。
- 全部原始包字节、状态值、逐帧来源 metadata 和操作边界逐项一致；历史输入文件及原真机工作区的 Git 状态、受跟踪/未忽略文件哈希均保持不变。

磁盘清理后的再次检查：315 项非仿真测试再次通过（32.671 秒）；直接使用生产默认配置
进行无硬件采集，三路共 525 帧、875 条机器人状态和 70 条夹爪状态，
`complete / kept`，关闭会话后独立完整检查通过。日志为
`/tmp/xarm_adaptation_reaudit_tests.log` 和 `/tmp/xarm_adaptation_reaudit_mock.log`。
5 条历史轨迹也再次完成全量离线回放，结果仍为 4 条完整检查通过、1 条故障片段
按预期拒绝；42,373 帧及 61,112 条机器人状态的原始证据一致，源文件未修改。
第二轮回放明细为 `/tmp/xarm_adaptation_reaudit_replay/summary.json`。

| 历史输入 | 三路帧数合计 | 机器人状态 | 回放结果 |
| --- | ---: | ---: | --- |
| 09-11 19:29 会话，第 1 条 | 12,757 | 17,578 | complete，检查通过 |
| 09-11 19:29 会话，第 2 条 | 10,322 | 14,215 | complete，检查通过 |
| 09-11 20:04 会话，第 1 条 | 9,324 | 12,845 | complete，检查通过 |
| 10-22 22:00 会话，第 1 条 | 6,732 | 11,123 | complete，检查通过 |
| 10-22 21:59 会话，第 1 条 | 3,238 | 5,351 | incomplete，检查按预期拒绝 |

新增测试覆盖慢校验时来源与日志持续写入、校验超时及进程回收、编码/日志失败、
字段篡改、共享证据损坏、无效反馈不刷新健康、边界切换及连续录制。
最终日志在本机 `/tmp/xarm_adaptation_final_tests.log`、
`/tmp/xarm_adaptation_final_mock.log`；真实回放明细为
`/tmp/xarm_adaptation_real_replay/summary.json`。这些临时证据不进入 Git。

可重复运行无硬件 smoke（输出文件系统须满足 10 GiB 空间门槛）：

```bash
.venv/bin/python -m robot_teleop.cli.xarm_collect \
  --task adaptation_mock --dry-run --headless --auto-record --duration 3 \
  --output /tmp/xarm_adaptation_smoke
PYTHONPATH=src .venv/bin/python scripts/validate_xarm_adaptation.py \
  /absolute/path/to/raw_v3/episode_000001 --output /tmp/new_replay_directory
```

真实数据回归入口是 `scripts/validate_xarm_adaptation.py`：以显式原 episode
和一个全新的输出目录运行。它不连接硬件，用虚拟历史时钟回放，生成标记为
`migration_replay` 的测试产物，保持原数据和保留状态不变。输出不是新采集示教，
也不是正式训练导出。原时间证据逐项相等；视频回放会重新编码，只核对帧序、
数量、尺寸和源 metadata，不宣称新旧有损码流或解码像素逐字节一致。

本地输入：9 月 11 日三条 complete/kept；10 月 22 日一条 complete/pending，
以及一次夹爪 SDK code 1 的 incomplete/pending。中间的 TCP/负载/相机变化按各自
session 配置解释，不能统一替换为当前标定。

## 现场验收与剩余边界

2026-11-06 用户现场试运行：自动开夹爪、复位、回中、机械臂遥操和按住闭合/松开保持
均已运行。首次启动曾因静止 SpaceMouse 在 0.25 秒内没有初始报告退出；该启动等待
问题仍待改进。随后一条约 31.35 秒的录制在按下打开时被夹爪短有效期检查中断，
保留为 incomplete（`first_real_demo/session_20261106T213630_e9f25382`）。
控制日志显示最新按钮报告到实际发送约 15.6 ms；反馈约束后的剩余有效期约 58 ms，
低于夹爪通信预算 100 ms，并非按钮输入已经过期。现修复为有效期不足时保持并等待
下一次有效指令，不提前启动目标；真正过期和通信失败仍停止退出。
此修复须重新现场验证，尚未完成一条完整轨迹或连续多条验收。
修复后 320 项非仿真测试通过（32.633 秒），新增用例复现反馈缩短有效期的发送链路，
并覆盖续期后发送、松开取消、反向先保持，以及真正过期仍报错。
日志为 `/tmp/teleop_gripper_fix_regression.log`。

- [ ] 非 home 位置启动，连续两次 H，确认旧夹爪线程退出、回中后恢复。
- [ ] 标定 TCP 下的平移、pose 与夹爪行为。
- [ ] 两条连续三路录制，覆盖 K/X/N；逐条完整检查和实际来源频率。
- [ ] 比较实际发送、等待反馈、ACK 和收尾节奏；原 raw v3 的硬件测量不作为新实现验收。

磁盘清理后再次核查，`/home` 可用约 74.8 GiB，满足生产 10 GiB 门槛；没有降低阈值。
历史回放产物位于 `/tmp`；本轮生产默认配置的无硬件 smoke 保存于
`data/collections/adaptation_reaudit/`，已确认被 Git 忽略。重要证据应另行备份。

提交前核查时，适配改动尚未推送。通过 `git ls-remote` 只读确认远程
不存在 `dev/real/xarm6-main-adaptation`，远程 main 仍为 `d31d0b0`。

离线对齐、训练格式及 action 定义仍为后续 C6；MuJoCo/FR3 行为本轮不修改，
不追加引擎或物理验收。
