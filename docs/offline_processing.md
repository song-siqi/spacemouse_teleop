# Raw 数据处理、LeRobot v3 与 Replay

本模块处理 raw v4 和历史 xArm raw v3。原始文件只读，默认只处理完整且保留的轨迹。
采集环境与离线处理环境独立；普通遥操安装不需要 LeRobot、SciPy 或 PyTorch。

## 安装与入口

当前验证组合锁定在 `requirements/processing.lock.txt`，目标为本机 Linux / Python 3.12。
该文件包含完整依赖（包括 PyTorch 的平台包），不是跨平台通用锁文件。

```bash
uv venv .venv-processing --python 3.12
uv pip install --python .venv-processing/bin/python -r requirements/processing.lock.txt
```

`requirements/processing.in` 是直接依赖声明；锁定 LeRobot 0.6.1，输出 v3.0。
脚本自动设置本仓库的 Python 搜索路径：

```bash
./scripts/run_processing.sh process \
  /home/siqis3/teleop-collection/first_real_demo/session_20261106T214518_79c90d78 \
  --config configs/processing/xarm6_lerobot.json \
  --fps 20 --action-mode delta_joint_position \
  --output /home/siqis3/teleop-collection/processed/first_demo_20hz

./scripts/run_processing.sh dataset-check \
  /home/siqis3/teleop-collection/processed/first_demo_20hz/episode_000001

./scripts/run_processing.sh replay \
  /home/siqis3/teleop-collection/processed/first_demo_20hz/episode_000001 \
  --mode offline --plot /tmp/first_demo_curves.png

./scripts/run_processing.sh replay \
  /home/siqis3/teleop-collection/processed/first_demo_20hz/episode_000001 --mode preview
```

输出必须是新目录。Session 输入生成每条保留 episode 的独立数据集，并写批次报告；弃用轨迹跳过，故障轨迹拒绝。单 episode 输入直接生成一个输出。
每个导出数据集只有一个 episode，replay 的 `--episode` 当前只能为 0；多条数据通过其输出目录选择。
预览支持 Space 暂停、A/D 单步、滑条定位、Q 退出。GUI 不在无显示环境自动验收；静态曲线可以 headless 生成。

## 数据与时间约定

处理分为来源读取、机器人字段适配、时钟映射、公共重采样、action 表达和格式导出。
公共数学和重采样不固定关节数量；目前注册的真实来源适配器是 xArm6。
未来机器人需注册数据适配器、机器人定义和可选执行器，不复制时间处理或 LeRobot writer。
夹爪与相机配置独立于机械臂关节向量。

- 真机统一到主机 monotonic。相机利用 SDK global time 和 realtime/monotonic anchors 映射；同帧曝光/读出 metadata 可用时应用其事件差值，不可用时明确退回 SDK 帧时间。
- 机器人使用设备时间和主机接收时间的稳健仿射估计；它不能辨识固定传输延迟，误差残差不是绝对同步精度。
- 夹爪时间估计采用查询中点，完整查询区间保留；只选择网格时刻之前已返回的反馈。
- 如有经过验证的时间偏移，使用配置 `source_offsets_s`；默认不猜测补偿。
- 20 Hz 为默认，支持正整数 fps，不超过最慢必需相机的配置捕获 fps。现有数据可生成 10/20/30 Hz。
- 图像取最近真实帧，允许离线未来邻帧；关节/位置插值，姿态 SLERP。没有图像插值、平滑或滤波。
- 默认图像误差 25 ms、相机间跨度 40 ms、机器人插值间隔 20 ms、夹爪 age 100 ms。超限拒绝，不自动放宽。
- 两次 Space 之外的缓冲仅用于边界选样，不额外输出为训练样本。
- 当前 xArm 有界关节不做 unwrap；连续关节来源需要其适配器提供连续的角度轨迹。

内部状态保留具名关节 rad、TCP xyz m / quaternion wxyz、实测 pulse 与归一化夹爪。
xArm 原生 TCP 的旋转向量与低频反馈 RPY 分别解释，不共用错误的角度转换。
xArm 关节范围来自仓库 vendored xarm6 URDF 默认值，作为软件范围检查，不替代现场标定。

## Action 定义

| mode | 机械臂标签 | observation.state |
| --- | --- | --- |
| joint_position | 下一时刻绝对关节 rad | joints + gripper |
| delta_joint_position | 相邻对齐关节差 rad | joints + gripper |
| ee_pose | 下一时刻 xyz m + rotvec rad | xyz + quaternion wxyz + gripper |
| delta_ee_pose | 基座平移差 + 相对 rotvec rad | xyz + quaternion wxyz + gripper |

所有模式末维为下一时刻实测夹爪对应的绝对位置，open=0 / closed=1。
映射外数据拒绝，不静默裁剪。实测夹爪与控制器目标分别保留，受阻目标不是测量值。

末端增量：`dp = p_next - p`，`dR = R_next @ R.T`。这是基座表达的逐步增量，
不是整个 action chunk 相对于首帧的偏移。不得在训练端无配置地再做一次 relative 转换。

先插值状态再差分；不同 fps 独立生成 delta。N 个对齐状态生成 N-1 行，
最后状态作为最终目标保留，不伪造末帧 action。这里的 action 是实测轨迹派生标签，
不是当时发送的速度命令。训练归一化交给训练端，不写入 raw 或替代物理单位。

## 产物与校验

每个处理目录包含：

- `processing_manifest.json`：状态、输入和输出哈希、配置、依赖版本、机器人定义、原始标定。
- `alignment_report.json`：映射参数、未知时间偏差、选帧误差、重复使用和运动统计。
- `source_refs.jsonl`：输出帧到原始图像/状态的索引、插值权重、夹爪查询区间。
- `reference_trajectory.json`：全部对齐状态，包括最后一个目标状态。
- `actions.npy`：float32 action 参考副本，不是第二份训练格式。
- `lerobot/`：官方 v3 数据集，三路 observation.images.*、state、action 和额外原始状态。
- `validation_report.json`：全量视频解码、全部数值读回、抽样来源图像比较和轨迹重建结果。

视频按输出网格重新编码，H.264 CRF18、veryfast、GOP40。允许有损编码误差；
图像来源比较使用原始视频解码图像，不宣称像素完全一致。数据集检查在 raw 仍可用时比较来源图像；
raw 不在本机时仍可检查数据集，但明确记录无法做来源像素比较。

先写 `.partial`，通过官方读取器、全量视频解码与来源检查后才提交 complete。
失败保留 incomplete 暂存目录，不覆盖或清理 raw。处理输出内部哈希检查覆盖生成文件；
manifest 是本地处理记录，不是防篡改签名。

## Replay 的实现与现场边界

离线 replay 从实际导出的 action 还原绝对参考轨迹，与参考状态比较。
真机适配器实现 Mode4 关节速度 / Mode5 TCP 速度跟随，包含速度前馈与 2 s^-1 比例反馈、
加速度限制、有限指令有效期、反馈检查和独立夹爪位置线程。
当前项目配置为 60 Hz、1.0 倍速（按录制时间轴）；终点额外保持 0.3 秒后停止。
`speed` 只缩放回放时间轴，不修改数据集 action；准备到首帧仍使用独立的低速限制。

真机必须显式 `--mode robot --execute --config configs/replay/xarm6.json`。
默认只检查首帧，不自动 home 或开夹爪；起始关节/TCP/夹爪不匹配即拒绝运动。
显式添加 `--prepare-start` 后，会使用已有 JointHomer 限速/限加速度移动到该 episode
首帧关节位置（不是固定 home），在容差内稳定后核对 TCP/夹爪，再开始 replay 计时。
准备过程中和回放过程中均不提供避障规划，现场应确认准备运动路径。
默认保持当前夹爪；不匹配时在机械臂移动前拒绝，要求手动准备或显式添加
`--prepare-gripper`。该选项在机械臂到位后将夹爪移动到首帧实测位置，并等待稳定，
不会先自动打开夹爪。夹爪故障/受阻导致位置无法到达时超时并保持、停止退出。
机械臂准备默认使用配置的 homing 超时（当前120秒），可用 `--prepare-timeout SECONDS` 覆盖；
夹爪准备使用配置的 startup_open 超时（当前10秒）。准备阶段日志独立于轨迹时间轴。
准备失败或 Ctrl-C 时不执行后续 action 序列。
还要求当前工具参数与采集时一致。不使用 `--prepare-start` 时须人工准备首帧位置；
使用准备选项时也需在现场核对准备路径。
起始容差为关节 .02 rad、TCP 5 mm/.05 rad、夹爪15 pulse。
运行跟随保护为关节 .15 rad、TCP30 mm/.15 rad，沿用配置速度/加速度上限。
正常或异常退出均停止机械臂，夹爪保持测量位置而不自动释放。

本轮未连接真机。执行器经过 fake SDK 验证，不能据此宣称真实抓取或接触轨迹已复现。
现场验收应从空载、低速、单轴开始，再比较轨迹误差、夹爪、异常停止和任务结果。

## 自动测试

```bash
PYTHONPATH=src:tests .venv-processing/bin/python -m unittest \
  test_processing test_xarm_replay test_lerobot_processing -v
```

覆盖时间漂移/异常值、图像选帧、插值和差分、六/七关节适配、非交换旋转、
闭环响应模型、起始拒绝、慢ACK、反馈超时、夹爪去重/保持、线程回收、写盘失败、
官方四模式读回、padding和0/2个DataLoader worker。全部使用离线数据或fake SDK。
原环境另跑公共/采集/真机适配回归，不追加 MuJoCo 物理测试。


## 2026-11-06 验收记录

- 原遥操环境：320 项非仿真测试通过；新处理依赖未装入原环境。
- 独立处理环境：24 项新增测试通过，包括四模式官方读回、padding、0/2 worker、磁盘失败、fake SDK 与闭环响应模型。
- 两条真实保留轨迹完成 12 组导出：两条 ×（10/20/30 Hz 关节增量，以及20 Hz关节绝对/TCP绝对/TCP增量），全部检查通过。
- 最终20 Hz样本分别为930和1064行；终端状态单独保留，没有伪造末帧action。原始文件哈希复查未变化。
- 所有视频全量解码；全部训练数值官方读回，另做首尾/中间等抽样图像来源比较。没有将抽样像素比较表述为每帧像素相同。
- 最终20 Hz样本位于本机 `/tmp/teleop_processing_final20/`；组合验收明细为 `/tmp/processing_real_matrix_results.json`。
- 测试日志：`/tmp/processing_all_new_tests.log`、`/tmp/processing_base_regression.log`。曲线示例为 `/tmp/teleop_processing_curves.png`。

这些 `/tmp` 路径是本机临时验收产物，不随 Git 提供，也不作为长期数据备份。
以上为离线开发验收记录；后续用户现场关节replay结果见文末，GUI交互预览和TCP回放未据此验收。


### 首帧准备选项

以下是供后续现场验收使用的命令，本轮未执行真机运动：

```bash
./scripts/run_processing.sh replay /path/to/processed/episode_000001 \
  --mode robot --execute --config configs/replay/xarm6.json \
  --prepare-start --prepare-timeout 120
# 如需同时恢复首帧夹爪位置，显式追加 --prepare-gripper
```

新增后处理/回放测试合计34项通过，覆盖真实关节准备的fake响应、
Mode4准备后切换Mode5、稳定等待、最终TCP不匹配、准备超时、取消、
反馈过期、夹爪受阻保持，以及准备时间不计入replay。
日志为 `/tmp/prepare_start_final_tests.log`。后续单条关节轨迹的首帧准备已现场通过，见文末。

### 1.0 倍速与关节跟随上限修复

首次现场回放已经完成首帧准备，但正式回放约5.17秒后因第5关节误差超过0.15 rad停止。
原因是正式关节跟随错误复用了homing的0.1 rad/s速度向量上限；原参考轨迹峰值约0.235 rad/s。
SDK SyntaxWarning不是这次退出原因。

当前默认在配置、CLI回退值和公共runner中统一为1.0倍速。准备参数与正式跟随参数分开：

| 阶段 | 速度限制 | 加速度限制 |
| --- | --- | --- |
| 准备到首帧 | 沿用homing配置，当前每关节0.1 rad/s | 沿用homing配置，当前每关节0.2 rad/s² |
| 正式关节回放 | `tracking_limits.joint_speed_radps = 0.5`（向量范数） | `tracking_limits.joint_accel_radps2 = 1.5`（向量范数） |

正式回放上限独立于准备参数；TCP回放仍使用现场配置中的笛卡尔速度/加速度限制。
关节参考轨迹在运动连接前按目标倍速检查采样速度及加速度，超过上限则拒绝，不自动降速。
这是必要条件检查，不保证分段插值、反馈滞后及真实动力学下总能精确跟随。
0.15 rad关节跟随误差保护保留。日志新增目标/误差、反馈时间、请求速度和速度/加速度限幅标志；
故障行给出具体关节与误差。

当前两条轨迹在1倍速下峰值速度范数约0.235/0.304 rad/s，采样加速度约0.779/0.947 rad/s²，
均通过上述上限检查。39项处理/回放测试、47项现有复位/输入/夹爪回归通过。
另用5Hz采样反馈、有限速度有效期和理想速度响应模型离线跟随两条完整参考轨迹，
最大关节误差约0.021/0.024 rad，模拟回放时长约46.82/53.52秒（含末端保持），没有速度限幅。
该模型不包含真实电机与接触动力学，不代替现场复测。
证据：`/tmp/replay_1x_final_tests.log`、`/tmp/replay_1x_existing_tests.log`、
`/tmp/replay_real_reference_1x_simulation.json`。此次修复未启动机器人。


### 首条1倍速真机回放通过

用户现场运行 `logs/replay_first_live_03.jsonl`，结尾为 `replay_end / complete: true`。
回放对象为第一条保留轨迹的20Hz `delta_joint_position`，action为六关节增量rad加绝对归一化夹爪位置。
已通过低速首帧关节准备、最终起点检查和1.0倍速回放。
完整回放约46.82秒（含0.3秒终点保持）；循环约59.55Hz，实际发送约59.08Hz，
最大关节跟随误差约0.0247 rad（1.41°）。这是单条轨迹、当前现场配置的结果，
不代表任意机器人、TCP action、起始路径或接触任务均已验收。

原数据20Hz与底层60Hz控制循环分开：增量先还原为绝对目标，控制周期在目标间插值，
不重复累加同一条增量。采集、回放和离线派生数据均不随Git提交。
