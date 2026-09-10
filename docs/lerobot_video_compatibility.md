# 现有 H.264 视频到 LeRobot：离线兼容性验证

证据保存约定：本页结论随仓库提交；`logs/`、`data/` 和外部绝对路径仅为历史/本机证据定位，不随 clone 提供，也不保证历史原始文件仍在本机。

结论：现有三路 640×480、20 fps、H.264 CRF 18 / yuv420p / GOP 40 的 MP4 可以转换成并由原生 LeRobotDataset 读取。本次同时验证旧 OpenPI 锁定版本的 v2.1 数据集和 LeRobot 0.6.1 的 v3.0 数据集。

原实验仅使用已有压缩 MP4 与原始 observation 状态；没有读取源 PNG 列表，没有连接硬件，没有修改采集源码、配置、README 或 PROGRESS_PLAN。验证环境位于两个独立 /tmp 虚拟环境，未改变项目 .venv。未上传任何数据。

## 实际转换路径

1. 读取已有三路 MP4，解码成官方旧版写入器使用的临时 PNG，计算图像统计。
2. 使用旧训练端锁定提交的官方 LeRobotDataset.create/add_frame/save_episode 创建 v2.1 数据集。官方写入器检测到已放置的 MP4 后跳过编码，最终清除其临时 PNG。三路输出 MP4 的整个文件 SHA-256 与输入完全一致。
3. 对 v2.1 实验副本使用 LeRobot 0.6.1 官方 convert_dataset_v21_to_v30 迁移，明确 push_to_hub=False。原始 observation 时间来源作为附加元数据复制到两个版本。
4. v3.0 迁移会重封装 MP4：增加 SPS/PPS 头并改变容器字段，文件哈希不再相同；三路全部 H.264 图像切片字节哈希相同，且解码 RGB 全量逐像素一致。因此没有增加第二次有损编码。

此验证脚本是离线实验产物，尚未作为正式导出器合并。临时 PNG 是本次复用官方写入器统计路径的实现方式，不是未来训练时必须永久展开视频。

## 读取验证结果

| 目标 | 数据集格式 | 20 Hz 有效行 | TorchCodec 全量乱序图像核对 | PyAV 抽样图像核对 | 数值与标签 |
| --- | --- | ---: | ---: | ---: | --- |
| 旧 OpenPI 锁定提交 0cf864870cf29f4738d3ade893e6fd13fbd7cdb5 | v2.1 | 1287 | 3861/3861 通过 | 117 通过 | 全部 1287 行通过 |
| LeRobot 0.6.1 官方发布包 | v3.0 | 1287 | 3861/3861 通过 | 117 通过 | 全部 1287 行通过 |

- 两种读取器均使用未经修改的官方 LeRobotDataset。随机读取结果与转换前 MP4 通过 PyAV 顺序解码的 RGB 哈希逐帧核对；不是仅检查张量形状。
- 原生数据集时间戳使用默认 0.0001 秒容差，没有放宽容差掩盖错位。
- 原生历史图像窗口 [-0.1, -0.05, 0] 秒及未来 actions 窗口 [0, 0.05, 0.1] 秒通过；覆盖帧 0、1、39、40、485、1285、1286，核对边界 padding。
- DataLoader 的 0 worker 与 2 个 spawn worker 模式均返回正确批次；每相机图像 [4,3,480,640]，state/actions [4,7]，TCP [4,6]。有界 8 行/两个批次完整读取后，两个 worker 均退出码 0。
- 初次读取首个 batch 后立即销毁迭代器时，两环境日志出现原生库 `terminate called without an active exception`，主进程仍退出码 0。已单独补测完整消费有界批次并确认正常退出；提前中断解码任务的完整生命周期仍需在未来训练集成时检查，不能据此声称任意中断路径都已稳定。

## 字段与标签约定

| 来源 | 本次 LeRobot 字段 |
| --- | --- |
| fixed_1 | image（video） |
| wrist | wrist_image（video） |
| fixed_2 | fixed_2_image（video） |
| 六关节 rad + 夹爪实测 pulse | state，7 维 float32 |
| 下一时刻的上述实测状态 | actions，7 维 float32 |
| TCP xyz mm + RPY rad | tcp_mm_rad，6 维 float32 |

- 保留现有旧训练配置的 image/wrist_image/state/actions 命名。LeRobot 的其他策略若要求 observation.images.* / observation.state / action，需要显式字段映射；合法数据集格式不等于任意模型都无需配置。
- 原始 1,288 组 observation 生成 1,287 组相邻标签。最后一个原始状态参与最后一行的 action，不为末帧伪造下一状态。每个视频仍完整保留 1,288 帧，最后一帧不被暴露为训练行。
- actions 是下一步实测绝对状态标签，不是 SpaceMouse 命令，也不是已证明可直接下发机器人的控制接口。
- 训练数值按既定 float32 约定逐项精确核对；完整原始数值、设备帧号及来源时间保存在 meta/source_observations.jsonl，SHA-256 与原文件相同。

## 版本和产物

- v2.1 实验数据集（本机证据：`logs/lerobot_video_compat_20260910/dataset_v21`）；v3.0 实验数据集（本机证据：`logs/lerobot_video_compat_20260910/dataset_v30`）。
- 旧版读取证据（本机证据：`logs/lerobot_video_compat_20260910/v21_openpi_read_result.json`）；新版读取证据（本机证据：`logs/lerobot_video_compat_20260910/v30_061_read_result.json`）。
- 原视频/迁移码流核对（本机证据：`logs/lerobot_video_compat_20260910/video_payload_checks.json`）。
- 旧版依赖清单（本机证据：`logs/lerobot_video_compat_20260910/requirements_openpi_pinned.txt`）；新版依赖清单（本机证据：`logs/lerobot_video_compat_20260910/requirements_lerobot_061.txt`）。
- 两环境 uv pip check 均通过。使用 Python 3.12.14、CPU PyTorch 2.7.1、TorchVision 0.22.1、TorchCodec 0.3.0、PyAV 15.1.0；其余确切依赖见清单。
- 旧版根据本机 /home/siqis3/data/backup-xingyu/pi_0.5/openpi/pyproject.toml 与 uv.lock 的精确 git SHA 安装，而非仅按 PyPI 的 0.1.0 版本号推断。

## 历史实验范围与后续集成

本报告描述集成前的独立实验。随后已实现结束 episode 后独立后台编码、逐帧索引校验和完整性提交，见 [视频接入验证](collection_video_validation.md)。后续实时编码应放在独立有界队列/进程中，保留采集故障与机器人控制的隔离。
原始层继续保存三路视频、来源时间和原始状态；LeRobot 导出保持离线。在旧 OpenPI 训练流程下优先导出 v2.1，新增训练环境可用已验证的 v3.0。避免把 LeRobot 的训练依赖加入遥操环境。

## 本次范围

已验证：一条实际运动轨迹、三路 RGB、20 Hz、两个明确版本、官方转换/读取、数值与图像对应、时间窗口、正常批次收尾以及无二次有损编码。
未验证：完整训练任务、策略执行、10 Hz 导出、多 episode 视频拼接后的跨边界读取、长时训练吞吐、随时中断解码、编码与真机控制并发。

## 官方参考

- [LeRobot 视频参数：H.264、yuv420p 与编码设置](https://huggingface.co/docs/lerobot/video_encoding_parameters)。
- [LeRobotDataset v3.0：MP4、Parquet 与元数据](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)。
- [官方 v2.1 → v3.0 迁移说明](https://huggingface.co/docs/lerobot/porting_datasets_v3)。
