# 仓库交付与本机数据边界

更新：2026-09-10。本次交付覆盖此前尚未提交的三路采集和视频存储整体，当前本地分支是 `dev/real/xarm6`，实现基线提交为 `09ec8d8`。文档与忽略规则已整理；此次维护不自动 commit 或 push。

## 适合提交并推送的内容

| 范围 | 内容与原因 |
| --- | --- |
| `src/spacemouse_teleop/backends/xarm6_camera/` | 组合后端、状态代理、相机配置、预览和录制编排 |
| `src/spacemouse_teleop/sensors/` | RealSense 按角色/序列号捕获及合成输入 |
| `src/spacemouse_teleop/recording/{collection,archive,visual,export}.py` | 原始记录、视频归档/恢复、统一读取及离线导出 |
| 新增采集/频率/相机/检查/导出/finalize CLI | 独立入口，属于可复用实现 |
| `cli/xarm_teleop.py` 的小幅改动 | 默认关闭的 coordinator 接口和复位退出检查；采集入口依赖它，不能只推新增文件而漏掉这处修改 |
| `configs/spacemouse_xarm6_collection.json` | 项目默认采集配置，明确 20 Hz、三路角色、质量阈值及视频存储 |
| `scripts/run_xarm_collection.sh`、`pyproject.toml`、`requirements/collection-validated.txt` | 启动器、入口注册和已验证依赖，部署新功能所必需 |
| `tests/test_collection*.py`、`tests/test_realsense_capture.py` | 时间/来源、故障隔离、归档、读取、配置及生命周期回归 |
| `README.md`、`PROGRESS_PLAN.md`、`docs/` 新增/更新文档、`.gitignore` | 当前行为、数据格式、使用指南、验证结论、待办及产物边界 |

建议把上述代码、测试、配置及文档作为一个完整功能提交，例如：
`Add three-camera 20 Hz raw collection with video archival`。
当前 PNG 采集与视频归档已经相互集成，单独推视频文件会缺少依赖；无需为了拆提交重建尚未提交过的中间状态。

`backends/xarm_sdk/` 没有本轮待推送修改。已有仿真源码和模型资产继续受版本管理；不要用全局 `*.png`、`*.json`、`*.mp4` 或 `*.stl` 忽略规则隐藏合法配置、测试夹具和资产。

## 留在本机，进入 gitignore

| 路径 | 内容 |
| --- | --- |
| `/data/` | 录制轨迹、三路视频、临时 PNG、状态与会话日志、离线导出和训练数据集 |
| `/logs/*`，保留 `/logs/.gitkeep` | 运行日志、诊断输出、压缩/LeRobot 实验副本、容量统计及临时验证脚本 |
| `.venv*/` 和现有缓存目录 | 本机 Python/SDK/训练环境与下载缓存 |
| `/build/`、`/dist/`、`*.egg-info/` | 构建产物 |
| `/configs/*.local.json` | 可选的本机完整配置副本，需显式 `--config` 选择 |
| `/third_party/xarm_ros2/` | 已有下载 checkout；`src/` 内已经纳入版本管理的资产仍保留 |

规则按目录约束；不会删除任何本机文件，也不等于数据备份。若 `--output` 指向其他位置，其备份和忽略策略需要按实际位置管理。小型、可复现测试夹具应放在 `tests/` 下，不放在被忽略的 `data/`。

默认采集配置中的三个相机序列号是已确认的现场绑定，应随项目配置保留。本次不修改序列号、机器人 IP 或自动复位默认行为。换机器时可以复制为 `configs/<name>.local.json` 并显式加载；该文件是完整配置，不引入自动合并/隐藏覆盖。

## 验证材料的维护方式

结论和可复核口径保存在版本管理的 `docs/`：

- [首次现场视频记录](collection_video_field_validation.md)：1258 组 observation、归档、一次 N 返回遥操和真实发送间隔。
- [视频实现与离线验收](collection_video_validation.md)：215 项自动测试、压缩副本、全帧索引和容量。
- [LeRobot 兼容性摘要](lerobot_video_compatibility.md)：独立 v2.1/v3.0 实验结论和具体版本；尚未交付正式转换器。
- 原始频率、动作和容量证据分别见其他 `collection_*_validation.md`。

原始机器日志、图片/视频和实验环境留在 `logs/` 或 `data/`。文档中的本机证据定位不是可从 origin 下载的文件，不保证历史产物仍保留；不要为了让链接可点击把整个日志目录强制加入 Git。必要的结论已整理成独立文档，因此 clone 仓库后仍可了解实现和验收边界。

## 提交前复核

检查 `git status --short`、`git diff --check` 和待提交文件列表；正常候选应仅包含上述源码、配置、测试和文档。`git diff --stat` 单独不会显示未跟踪的新文件，审查时不能遗漏它们。建议先推送功能分支 `dev/real/xarm6` 再审阅合入 main；推送前重新核对远端分支状态。

本次仅修改文档与忽略规则，复查了最新已保存 episode 的全部视频/状态引用，没有重新运行控制软件或 MuJoCo。已有 215 项测试结果对应上一轮实现，不把文档整理描述成重新完成所有硬件或自动验收。
