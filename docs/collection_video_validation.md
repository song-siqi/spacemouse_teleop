# 三路视频 raw v2 接入：代码与离线验收

证据保存约定：本页结论随仓库提交；`logs/`、`data/` 和外部绝对路径仅为历史/本机证据定位，不随 clone 提供，也不保证历史原始文件仍在本机。

日期：2026-09-10。默认采集配置已切换为 `video_after_episode`。这是本项目 raw 数据格式，采集不生成 LeRobot dataset 或训练 state/actions。旧 PNG 数据不迁移、不删除。机器人 SDK、复位和夹爪后端七个源码文件与本轮实施前 SHA-256 一致；这里的代码实施/离线验证没有连接机器人、读取实时相机或运行 MuJoCo 检查。其后用户完成首次现场视频录制，最新结果见 [现场验证](collection_video_field_validation.md)。

## 已交付行为

- 录制继续用三路临时 PNG，每组图像写完才追加 observation。正常结束后排空写入，由独立归档任务核验采样、持久化 `capture_complete`、编码和校验。
- 每路 MP4/libx264、CRF 18、medium、yuv420p、GOP/keyint_min 40、场景切换额外关键帧关闭、两个编码线程；保持原始分辨率，视频 fps 来自每条 sample_hz。完整记录 FFmpeg/ffprobe 版本和参数。
- 全量检查源视频 PTS/尺寸、RGB 解码帧数/连续时间、observation 索引及原生状态引用，记录 JSON/视频 SHA-256。先提交三个最终视频和原子 complete 元数据，再清理 `_staging/`。
- 归档线程独立于控制日志写入；最多一个 episode 归档任务、三个编码子进程。归档期间可遥操；预览显示阶段和进度，Space/K/X/N 不推进流程。之后 K/X，再 N。
- 编码/校验失败保留 PNG，片段不完整；清理失败保留 complete 与警告，不能据此声称空间已释放。Q 先停机器人，再等待已正常结束的归档；总期限 `max(120 秒, 2×录制时长)`，等待剩余期限而非固定 30 秒，超时终止并回收子进程。
- 新命令 `spacemouse-collection-finalize EPISODE` 可重试正常采样完成的 v2 或清理已验证完整片段，不修复采样缺口/中断、不迁移旧 PNG。旧格式与缺少正常完成标记的拒绝路径不创建锁文件或改写源数据。
- 统一视觉读取层支持 PNG/视频索引；原检查与 legacy/light/derived 导出兼容 20/10 Hz。配置缺少 `visual_storage` 时继续 PNG；视频模式在机器人连接/复位前检查工具和 libx264，失败不回退。

格式及字段见 [raw 约定](collection_format.md)，操作见 [采集指南](xarm_collection_setup.md)。

## 自动验证

最终相关套件 **215 项通过，0 失败、0 错误、0 跳过**。原有 190 项上新增两个配置检查和 23 项视频/生命周期测试。记录见 回归日志（本机证据：`logs/video_storage_implementation/regression.log`） 和 [视频测试](../tests/test_collection_video.py)。

覆盖：

- 三路帧数、全部末帧/状态保留、20/10 Hz 导出、首尾和 GOP 边界的乱序读取；源 PTS 偏移被拒绝。
- 缺 PNG、索引间断、原始状态引用错误、编码失败、损坏视频、提交写盘失败、归档超时、提交前后中断、清理失败及重试；未完成不能导出。
- 同一片段并发归档拒绝、暂存符号链接保护；故障片段/旧版本不能升级为有效轨迹。
- 人为暂停归档时，70 次模拟运动委托与 70 条控制日志继续，零日志丢弃；K/开始下一条被拒绝。编码失败后可弃用该条并正常归档新条。
- 退出顺序明确先关闭运动后端，再清理来源与等待归档；录制中退出不启动归档；超时子进程已回收，归档线程结束。

默认脚本的无硬件 headless smoke 保存 **60 组 observation / 三路 180 帧 / 3 个 MP4**，`complete / kept`，队列高水位 5/128、丢弃 0、暂存已清理。合成图像较简单，不能作为真实容量或运动性能证据。见 入口日志（本机证据：`logs/video_storage_implementation/mock.log`）。

## 真实数据独立副本：一致性

源轨迹来自 2026-09-09 已完成的真实遥操；本次只复制后离线处理，没有再次操作硬件。

- 原始 session：`data/collections/first_motion_demo/session_20260909T214050.947957Z_dbc73846`。
- v2 验证副本：`logs/video_storage_implementation/real_roundtrip/session_20260909T214050.947957Z_dbc73846`。
- 原始 session 全部文件实施前后 SHA-256 一致，原文件未变化。测试夹具只在副本上把 PNG 放入 `_staging/` 并添加 v2 图像引用；这不是对用户旧数据的自动迁移功能。
- **1288 组 observation、三路 3864 帧、5473 条原生状态**。每路视频 1288 帧、640×480、20 fps；末帧索引 1287/PTS 64.35 秒保留。
- `raw_states.jsonl` 与源文件逐字节相同。全部 observation 除 `images[role].file` 和新增 `video_frame_index`/`video_pts_s` 外逐字段相同；关节、TCP、夹爪、来源、时间和质量信息不变。
- 全量源 PTS 与 RGB 解码验证所有 3864 帧。每路独立乱序读取 **195 帧，共 585 次**，覆盖首尾、每个 GOP 边界及全轨迹不同位置，与 FFmpeg 顺序解码的 RGB MD5 一致。
- 上述一致性是**索引读取与同一视频顺序解码一致**；H.264/yuv420p 本身仍有损，不代表与原始 PNG 像素完全一致。压缩误差对比见 [此前实验](collection_storage_validation.md)。
- 完整新视频副本导出 light 20 Hz：1287 对；10 Hz：643 对（选中 644 observation）。每一行 state/actions 与相邻所选原始绝对状态逐项一致；末帧没有伪造 action。legacy 和 derived 路径另由实际视频小夹具覆盖。新视频检查分别报告 `image_count=3864`、`video_file_count=3`。

机器可读结果：real_validation.json（本机证据：`logs/video_storage_implementation/real_validation.json`）。已有独立 LeRobot v2.1/v3.0 兼容实验见 [报告](lerobot_video_compatibility.md)；本次没有新增正式 LeRobot 转换器，也没有再次运行训练端。

## 容量与归档耗时

以下为文件逻辑大小，MB/GB 使用十进制。一次真实动作场景，不能代表所有光照、背景或运动。

| 内容 | 本条轨迹 | 折合每分钟 |
| --- | ---: | ---: |
| 旧 PNG 完整 session | 1.529 GB | 1.425 GB |
| 新三路视频本身 | 24.90 MB | 23.20 MB |
| 新完整 session，含实际控制日志/状态/索引/元数据 | **39.51 MB** | **36.81 MB** |
| 暂存 PNG 本身 | 1.515 GB | 1.411 GB |
| PNG + MP4 + 会话 JSON/日志峰值容量估算 | 1.554 GB | 1.448 GB |

- N/20 视频时长 **64.40 秒**；原采集网格首尾跨度 **64.35 秒**。每分钟容量以 N/20 归一化，未改变采集时间轴。
- 新完整 session 为旧版本的 2.58%，约缩小 **38.7 倍**。按该场景粗估约 2.21 GB/小时；不同 session 的等待时间也会增加控制日志。
- 最终代码的归档调用总耗时 **28.21 秒**，包括临时 PNG/来源校验、三路并行编码、源 PTS/全量 RGB 解码、提交和清理。没有与机器人控制同时运行；这不是现场并发耗时保证。初版一次测量为 25.15 秒，加强源 PTS 检查后的本次为上述最终值。
- 临时峰值是同时保有 PNG/视频/JSON 的容量计算，不是块分配高水位实测。视频提交前仍需要约 **1.4 GB/分钟临时 PNG 空间**；磁盘保留门槛仍为 10 GiB。归档校验成功后，此副本 `_staging/` 已清理。
- 系统编码工具为本机现有 FFmpeg/ffprobe 4.2.7-0ubuntu0.1；版本字符串和确切参数在副本 meta 中。项目 Python 依赖未增加 LeRobot、PyTorch 或训练库。

## 现场进展与待办

其后用户已完成一条实际视频录制、归档和保留，并在一次 N 后返回遥操；全量文件和近似归档窗口控制节奏已复查，见 [首次现场验证](collection_video_field_validation.md)。尚未录制第二条，连续多条、臂爪同时运动和更多工况仍待验收。R1/R2、工具标定、GelSight 和 MuJoCo 按进度文档后置。
