# Gazebo Harmonic / ROS 2 Jazzy 适配说明

此文描述附加教学包采用的新运行接口。原有 Classic 11 / Noetic 课程、编号和个人练习保持独立；现代课程的编写、实际验收和交付逐课记录。框架能启动某个版本，不代表所有课程已经在该版本验证，也不代表 Classic 插件能够原样运行。

## 版本边界

| 环境 | 运行配置 | 说明 |
|---|---|---|
| Ubuntu 20.04 + Classic 11 + Noetic | `classic11-noetic` | 保留现有课程及其原验收基线 |
| Ubuntu 24.04 + Gazebo Harmonic（Sim 8）+ ROS 2 Jazzy | `harmonic-jazzy` | 新增运行器和独立 ROS 2 课包；逐课记录实测结果 |
| Harmonic Sim 8、没有 ROS | `harmonic-jazzy` 中 `requires_ros=false` 的课 | 可运行独立 SDF 实验，不伪造 ROS 依赖 |
| Ubuntu 22.04 + Humble / Fortress、其它现代 Gazebo 主版本 | 只读识别与教材浏览 | 不自动认定与 Harmonic/Jazzy 兼容；需独立接口移植和验收 |

Harmonic/Jazzy 是官方推荐配对之一；[官方版本配对说明](https://gazebosim.org/docs/harmonic/ros_installation/)列出了其它组合及二进制支持条件。检测从 `gz sim --versions` 的实际数字结果确认 Sim 8，不以 `gz` 或 `ign` 的文件名认定版本。安装与激活分开记录；即使当前 shell 激活的是 Noetic，现代课只在自己的子环境加载 Jazzy。

## 环境检测的算法流程

1. 现有探测器继续收集 Classic 与已安装 ROS 发行版；新探测器 `detect_modern` 检查现代 Sim 和 Jazzy 包索引。
2. 在子环境中剔除已有 ROS 发行版、overlay 前缀，以及外层 shell 的 Gazebo Sim/IGN 插件与配置路径，再加载 `/opt/ros/jazzy/setup.bash`。例如 Ionic 的 `GZ_SIM_SYSTEM_PLUGIN_PATH`、`GZ_SIM_SERVER_CONFIG_PATH`、`GZ_CONFIG_PATH` 不得带进 Harmonic。清理发生在 source 之前，Jazzy setup 设置的正确路径会保留；图形会话变量保留。不修改主进程、shell 配置或全局系统环境。
3. 读取 Sim 版本和 `ros2 pkg list`，并在选定子环境的 PATH 解析每个工具，存入 `modern.tools`。课程不借用外层 Ionic/Noetic shell 的程序路径补齐依赖。探测失败被记为“检测未完成”，包缺失被记为“缺少依赖”，确认的主版本不符被记为“版本不匹配”。普通 `gz` 命令存在不等于 Sim 8 存在。
4. `catalog.compatibility` 和环境结果窗口使用同一套现代依赖判断。独立 SDF 课不需要 ROS；ROS 2 课另外检查 Jazzy、包索引及各自声明的包。
5. NativeBackend 按 `lesson.profile` 构造 Classic 或 ModernLessonSession。CLI 与桌面入口共同使用这个工厂，避免界面与 smoke 选择不同的运行器。

现代 Sim 版本或 Jazzy 包索引探测失败时，同时汇入全局 `incomplete_checks` 与 `detection_errors`，去重保留具体错误；恢复的检测快照也使用相同汇总逻辑，避免课程显示“检测未完成”而总报告却显示完成。现代检查名称单独登记，不会因此关闭依赖已满足的 Classic 课程。

检测不会安装依赖、启动 daemon、拉镜像或更改 ROS 全局激活状态。满足声明依赖只是运行前提，具体行为通过对应实验验证。

## 会话启动、控制与清理

现代运行器继承已有练习存储、日志、真实 PTY 终端、验收记录和进程所有权管理，替换模拟器相关接口：

**个人副本 → 子环境 → domain 租约 → prepare → `gz sim -s -r` → control 服务与实际统计就绪 → 辅助节点 → GUI/终端 → 实验操作。**

- 每次实验使用不可预测 token，`GZ_PARTITION` 与 `IGN_PARTITION` 都包含该 token。
- ROS 2 使用 DDS，不启动 `roscore`。应用在当前 Linux 用户的文件锁租约中分配 `ROS_DOMAIN_ID` 20～99，避免本应用不同会话复用 domain；发现范围限定本机。此租约不能替外部程序预留 domain，外部项目仍应使用自己的 domain。
- 服务端与 GUI 分别启动：`gz sim --force-version 8 -s -r` 与 `gz sim --force-version 8 -g`。GUI 退出可单独重开，服务端与当前物理状态继续存在。
- 就绪检查同时要求 `/world/<name>/control` 服务与真正的 `WorldStatistics`。仅有进程或端口不算世界就绪。
- 暂停与继续发送一次 `WorldControl` 请求，再订阅新统计确认实际 `paused`。服务 Boolean 只表示请求接收，不能代替行为结果。
- 单步先确认暂停，读取基准迭代数 I，发送一次 `multi_step=N`，只在实际统计显示 `iterations=I+N` 且仍暂停时报告成功。超时或额外客户端导致迭代超标时报告不确定/失败，不自动重发控制。
- 查询和控制 CLI 都是本会话拥有的进程，使用有界超时；退出通过 token、UID、PID 启动时间与进程组身份只清理本课。释放 DDS domain 租约发生在进程清理之后。

### 必须保留的语义差异

**Harmonic Sim 8 不支持保持时钟的仅模型复位。** 上游 `OnWorldControlState` 对 `model_only` 打印不支持，却仍可能返回 Boolean true。[上游实现](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/SimulationRunner.cc#L1270)

因此本运行器明确拒绝该操作，提示使用“重启实验”恢复整个场景。没有悄悄替换成 `reset.all=true`，因为后者会使仿真时间归零并影响 TF 缓存、控制积分与录包。关于完整 reset 的实际接口见[官方 Reset 说明](https://gazebosim.org/api/sim/8/reset_simulation.html)。

Classic 的 Model/World/Sensor/Visual/GUI/System 插件分类不直接映射为现代课程代码。现代 Gazebo 使用实体组件管理与系统插件；ROS 2 通过 `ros_gz_bridge` 显式桥接支持的消息类型。`rostopic`、`roslaunch`、全局参数服务器和 `gazebo_ros_api_plugin` 均不会混入现代课。

## 运行入口

在准备好的 Ubuntu 24.04 图形环境安装系统 PyQt5 后，从源目录执行：

```bash
./launch.sh --diagnose
./launch.sh --list
./launch.sh --profile harmonic-jazzy
python3 -m gazeboteach --smoke ID --profile harmonic-jazzy --gui
```

最后一条的 ID 仅适用于已经单独交付且满足依赖的现代课程。`--profile` 在两个课包使用相同课号时避免歧义。不同 profile 的练习目录和学习记录分开存储。

基础依赖可由用户按官方源准备：

```bash
sudo apt update
sudo apt install python3-pyqt5 ros-jazzy-ros-gz
```

具体 ROS 2 课程会额外声明 `rclpy`、`tf2_ros`、`rosbag2_py`、`robot_state_publisher`、动作消息和 colcon 等依赖。`gz_ros2_control` 不是原生关节 PID 的别名，需自己的硬件接口、控制器配置及真实反馈验收；不能由安装 `ros_gz` 推断它已经可用。

Docker 可作为隔离的 Ubuntu 24.04 用户环境，从容器内部启动同一桌面应用；平台原有 Docker 连接配置接口与本机执行是两件事。本说明没有将“保存 Docker 配置”描述为已实现容器调度。GUI 需要实际 X11/Wayland 显示转发，GPU 激光还需可用的渲染上下文；不应为了演示开放全局 X 访问或替换宿主驱动。

## G03 框架验证入口与证据解释

```bash
python3 -m unittest tests.test_modern -v
python3 tests/modern_runtime_acceptance.py --output .deploy/g03-modern-framework.json
python3 tests/modern_runtime_acceptance.py --gui --terminals \
  --output .deploy/g03-modern-framework-gui.json
```

自动框架验收使用 `tests/fixtures/modern_diagnostic`：真正落块到地面、暂停、精确单步/十步、继续、两个同时存在的真实世界的 partition/domain 分配、结束第一个世界而第二个继续推进，以及本测试所有 token 无残留进程。该测试确认域号不重复，没有将域号租约等同于已验证 DDS 消息隔离。

可选 GUI 验收除独立重开外，必须通过 X11 `_NET_WM_PID` 找到属于该 GUI 进程组且带本课 token 的真实映射窗口；使用 Qt 抓取初始与重开后的窗口，记录 PNG、尺寸和 PID，供视觉检查。截图还必须通过采样颜色数、亮度跨度与非黑像素比例检查；GL drawable 失败造成的纯黑/空白窗口在有界等待后使验收失败，保留 `rejected-blank.png` 和像素指标。只看到 GUI PID 存活、窗口尺寸正常或PNG保存成功不会计为图形通过。

像素变化只是拒绝黑屏的必要条件，不能证明场景、材质或机器人正确；报告始终保留 `manual_scene_review_required=true`，实际三维场景需查看截图独立确认。可选终端验收检查真实交互 PTY。

该 fixture 不计为课程，也不代表 PBR、激光、ROS 2 动作或控制器已验收。单元测试检查版本边界、ROS 环境隔离、消息解析及不确定步进不得误报成功；真实 Gazebo 验收报告与课程自己的行为验证分别记录。最终验证状态由实际输出报告决定，本设计文档不提前宣称全部平台已通过。

## 官方先修入口

G03已在隔离Ubuntu24.04.5 / Jazzy / Sim8.15.0中验证基础运行器、桌面、真实Gazebo窗口和PTY，详见[逐阶段验证记录](EXPANSION_VALIDATION.md)。本次软件Mesa直接连接宿主NVIDIA X服务器曾出现全黑窗口，失败记录保留；最终通过现有驱动对应库的只读绑定和设备透传完成图形核对。宿主驱动与Docker配置均未改动。这不是任意容器/显卡都可用的承诺。

常规NVIDIA容器环境按[NVIDIA Container Toolkit官方指南](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)配置运行时与设备，图形能力及X11授权还需分别验证；不要把仅有CUDA计算可用视为GUI渲染已可用。中文桌面还需`fonts-noto-cjk`等中文字体。此处只链接环境配置，不会由平台启动时自动改写宿主配置。

- [ROS 2 与 Gazebo 桥接](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [ROS 2 QoS 的请求与提供兼容规则](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [ROS 2 Python 动作](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [ROS 2 URDF 教程](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html)
- [gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
