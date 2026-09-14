# 仓库交付与本机数据边界

更新：2026-09-11。当前工作区已完成 raw v3 高频原生采集并通过当前现场录制验收，默认三路 640×480 RGB、60 fps；机器人报告和夹爪保持原生反馈节奏。当前分支为 `dev/real/xarm6`，HEAD 为 `de026f8`，raw v3 与默认 60 fps 等更新尚在工作区；提交前以实际 `git status` 和 diff 为准。本次文档维护不自动 commit 或 push。

## 适合提交并推送的内容

| 范围 | 内容与原因 |
| --- | --- |
| `src/spacemouse_teleop/backends/xarm6_camera/` | 组合后端、被动 TCP 30000 解码、夹爪反馈采集、配置、预览和录制编排 |
| `src/spacemouse_teleop/sensors/` | RealSense 按角色/序列号捕获、逐帧时间元数据和合成输入 |
| `recording/{collection,native_writer,clocks,raw,visual,archive}.py` | 原生录制、独立实时编码/队列、时钟证据、只读检查和旧 v2 归档恢复；路径相对 `src/spacemouse_teleop/` |
| 采集/频率/相机/检查/finalize CLI | 原生录制和诊断入口；finalize 仅恢复有正常采样证据的 v2 |
| 删除旧 `recording/export.py`、`cli/collection_export.py` 及对应测试/入口 | raw v3 切换时已移除的旧导出接口，相关删除一起审查；后续在本仓库实现适配 raw v3 的离线处理，不把旧接口当作已经可用的新处理入口 |
| `configs/spacemouse_xarm6_collection.json` | schema v2 默认配置：三路 60 fps、角色绑定、被动机器人来源、缓冲/队列及 native_video |
| `scripts/run_xarm_collection.sh`、`pyproject.toml`、`requirements/collection-validated.txt` | 启动器、入口注册和已验证依赖；按实际差异一起审查 |
| 相关采集/相机/原生报告/时钟测试及 `tests/fixtures/` | 回归和小型、可复现协议样本；真实 784-byte 报告 fixture 可以纳入 Git |
| `README.md`、`PROGRESS_PLAN.md`、`docs/` 与需要的忽略规则 | 当前行为、版本边界、指南、验证结果及待办 |

将完整的 raw v3 实现、旧导出移除、60 fps 默认配置、测试和文档一起审查，例如以 `Record native RGB at 60 fps with timestamped robot streams` 描述最终功能。不要为拆提交重建中间状态，也不要只推配置而遗漏原生写入器、协议解析、时钟记录或入口变化。

后续离线处理（C6）的源码、处理配置、测试与文档也在本仓库维护和提交；当前尚未实现，不在此次文档更新中恢复旧导出代码。原始数据、处理后的数据集、运行日志和实验环境继续留在被忽略的数据/日志目录。离线处理与采集采用独立入口及依赖配置，按需要使用独立环境。

`backends/xarm_sdk/` 没有本轮待推送修改。已有仿真源码和模型资产继续受版本管理；不要用全局 `*.png`、`*.json`、`*.mp4`、`*.bin` 或 `*.stl` 忽略规则隐藏合法配置、测试夹具和资产。

## 留在本机，进入 gitignore

| 路径 | 内容 |
| --- | --- |
| `/data/` | 录制轨迹、三路视频、逐帧索引、机器人原始包/状态与会话日志，以及已有数据和离线处理生成的派生产物 |
| `/logs/*` | 运行日志、诊断输出、验证数据、容量统计及临时验证脚本；已跟踪文件不受新增忽略规则影响 |
| `.venv*/` 和现有缓存目录 | 本机 Python/SDK/训练环境与下载缓存 |
| `/build/`、`/dist/`、`*.egg-info/` | 构建产物 |
| `/configs/*.local.json` | 可选的本机完整配置副本，需显式 `--config` 选择 |
| `/third_party/xarm_ros2/` | 已有下载 checkout；`src/` 内已经版本管理的资产仍保留 |

规则按目录约束，不删除文件，也不等于备份。`/tmp/` 的本轮验收数据位于仓库之外，Git 不会包含它们；需要长期保留时单独备份完整 session。其他 `--output` 位置也需要按实际路径管理。小型测试夹具放在 `tests/`，不放在被忽略的 `data/`。

默认配置的三路序列号是已确认现场绑定，应随配置保留。换机器可复制为 `configs/<name>.local.json` 并显式加载；它是完整配置，不引入自动合并或隐藏覆盖。不要将相机帧率变化混同于机器人频率或最终训练采样率。

## 验证材料的维护方式

当前结论：

- [raw v3 验收](collection_raw_v3_validation.md)：初版 30 fps 基线、60 fps 的 211 项非仿真回归和只读测试，以及 2026-09-11 两个真实遥操会话共 3 条轨迹的完整检查、控制节奏和动态容量。
- [raw v3 格式](collection_format.md)与[采集指南](xarm_collection_setup.md)：原生数据、逐帧索引、时钟证据与使用方式。
- [进度计划](../PROGRESS_PLAN.md)：已完成的高频 raw 录制里程碑，以及本仓库待实现的 C6 离线处理、非阻塞收尾优化和扩展操作验收。

历史证据保留原条件：

- [v2 现场视频记录](collection_video_field_validation.md)：1258 组 observation、后台归档、一次 N 返回遥操及当时的控制发送间隔。
- [v2 视频实现与验收](collection_video_validation.md)：当时 215 项测试、压缩副本、索引和容量。
- [时间戳报告](collection_timing_report.md)：旧 v2 相机延迟和独立机器人报告探测，不代表当前实现仍未接入原生状态。
- [LeRobot 兼容性实验](lerobot_video_compatibility.md)：独立版本实验；当前尚无正式 LeRobot 转换器，未来是否提供及输出版本在本仓库后处理方案中另行确定。

不要将历史 30 fps 数据改写成 60 fps，也不要将只读成功写成运动验收；本次运动结论来自独立的 2026-09-11 现场数据。文档中的本机证据路径不是能从 origin 下载的文件；必要结论随文档保存，完整日志和采集文件另行备份。

## 提交前复核

检查 `git status --short`、`git diff --check` 和待提交列表；源码、配置、测试、文档及有意删除的旧接口都应纳入审查。`git diff --stat` 不显示未跟踪新文件，不能遗漏它们。确认没有把完整 session、实验环境或大批视频加入 Git；推送前重新核对远端分支状态。

本次只同步文档，未改变忽略规则、源码或配置，也未重新运行硬件或 MuJoCo。211 项测试沿用已有结果；新增文档结论引用已完成的 3 条真实轨迹检查。`logs/native60_motion_audit_20260911/` 和 `logs/native60_live_monitor_audit_20260911T200403/` 的统计、复查与 live monitor 脚本继续留在本机，关键验收结论保存到版本化文档。
