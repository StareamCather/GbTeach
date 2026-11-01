# GazeboTeach V2 安装与运行指南

V2 提供中文原生桌面工作台、L01–L37 主课和 L091 / L092 倒立摆选做，共 **39 个独立项目**。课程教材、个人练习模板、参考材料、离线场景、原生插件源码和验收程序随应用交付。课程范围见 [V2 课程地图](V2_CURRICULUM.md)，变化见 [V2 发布说明](RELEASE_NOTES_V2.md)。

本包是源码应用发行包。文件名中的 `ubuntu20.04-x86_64` 表示实验基准环境，**不表示捆绑 Ubuntu、Gazebo、ROS、系统 Python / PyQt5、编译器、Qt 开发包或显卡驱动**。安装器不运行 apt / sudo；课程需要编译时使用已准备好的本机工具链。

## 1. 运行环境与开放条件

原生基准为 **Ubuntu 20.04 x86_64、系统 Python 3.8、PyQt5 / Qt 5.12、Gazebo Classic 11.15.1、ROS 1 Noetic、gazebo_plugins 2.9.3**。逐课实测、框架压力测试与安装验收属于不同证据，请查发行页随附的 `V2_VERIFICATION.md` / `.json` 和仓库[验证记录](VALIDATION_LOG.md)；版本清单和打包成功本身不证明实验目标达成。

| 现有环境 | 课程行为与准备方向 |
| --- | --- |
| Classic 11.15.1 + 对应 SDK / C++ / Noetic 依赖 | 按每课声明检查工具、库、插件、Qt 和图形会话，满足后开放实验 |
| Classic 11，缺少 SDK 或 Noetic | 教材均可阅读；只运行依赖满足的基础课，SDK / ROS 课程分别给出原因 |
| 其他 Classic 11 补丁版本或 SDK / 运行库不一致 | 精确 SDK 课程保持禁用；不要用改版本字符串绕过守卫 |
| Ubuntu 22.04 / ROS 2 / 现代 Gazebo | 可阅读教材、查看具体禁用原因；本课没有自动移植到 ROS 2，手动兼容环境见 [Docker 指南](V2_DOCKER.md) |
| 无图形会话 | 可做只读诊断及适用的无 GUI 实验；GUI 编辑器、相机渲染、VisualPlugin、GUIPlugin 和 RViz 要检查实际显示能力 |
| ARM64、Windows、macOS 或其他组合 | 本版本没有相应原生实验验收结论 |

启动会显示实际 Gazebo / SDK、已安装与当前激活的 ROS、命令与依赖，并列出逐课状态。未知、超时或缺少依赖不当作就绪。检测算法与安全的前置资料阅读方式见 [环境检测说明](ENVIRONMENT_V2.md)。满足静态条件之后，还要确认传感器数据、真实物理、插件行为及 GUI / OpenGL 是否正常。

框架的暂停、继续、精确单步和模型复位在精确 Classic 11.15.1 SDK / 运行库及 C++ 工具链可用时使用每会话一份持久助手。SDK 可选能力不满足时记录 `legacy-cli` 及原因；这不会让全部基础课都变成 SDK 必修课。已支持环境中的编译或握手错误会明确报告；修改请求发送后超时不通过换后端重发。流程与边界见 [原生世界控制](NATIVE_CONTROL.md)。C++ 课程自身仍必须满足其声明的精确 SDK 条件。

## 2. 原生 Ubuntu 20.04 的依赖准备

已准备好的实验设备先运行诊断，无需为了安装源码应用更新系统。需要补依赖时，由设备维护者在兼容的 Focal / Noetic 环境中按官方来源准备；不要把 Focal 软件源加入 Ubuntu 22.04 / 24.04 宿主。ROS 1 集成见 [Classic 官方 Noetic 安装说明](https://classic.gazebosim.org/tutorials?tut=ros_installing)。Classic 与 Noetic 已结束上游常规维护，旧源与镜像的可获得性应先核实。[Classic 官方维护提示](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)、[ROS 官方 Noetic 状态](https://www.ros.org/blog/noetic-eol/)

在**已经配置且核对了对应官方软件源**的兼容环境中，以下包名可作为依赖清单，由维护者先查看候选版本再决定安装：

```bash
apt-cache policy gazebo11 libgazebo11-dev ros-noetic-gazebo-plugins
apt-cache policy ros-noetic-gazebo-ros-control qtbase5-dev

# 仅在确认候选版本及其依赖符合目标平台后，由维护者执行：
sudo apt-get install \
  python3-pyqt5 python3-numpy python3-dev \
  build-essential cmake pkg-config libgazebo11-dev qtbase5-dev qtbase5-dev-tools \
  ros-noetic-desktop-full ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control ros-noetic-ros-control ros-noetic-ros-controllers \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  xterm x11-utils gedit fonts-noto-cjk
```

`libgazebo11-dev` 这个包名不保证实际版本就是 11.15.1，`gazebo-ros-pkgs` 也不保证 gazebo_plugins 就是 2.9.3。特定课依赖的运行库、SDK 头文件、pkg-config 元数据及插件版本必须一致；源里找不到目标版本时，使用经过核对的兼容环境或在独立容器中固定构建，见 [V2 Docker 指南](V2_DOCKER.md)。安装器不会自动降级、改源或修改主机 SDK。

在实际图形桌面终端检查：

```bash
/usr/bin/python3 -c 'from PyQt5 import QtCore, QtWidgets; import numpy; print(QtCore.QT_VERSION_STR)'
gazebo --version
gzserver --version
pkg-config --modversion gazebo
pkg-config --modversion Qt5Core Qt5Widgets
command -v g++
GT_QT_HOST_BINS="$(pkg-config --variable=host_bins Qt5Core)"
"$GT_QT_HOST_BINS/moc" -v
test -f /opt/ros/noetic/setup.bash
bash -c 'source /opt/ros/noetic/setup.bash; rosversion -d; rosversion gazebo_plugins'
printf 'DISPLAY=%s\n' "$DISPLAY"
```

Qt 开发库与 MOC 用于 Qt 原生插件；只有 Python 的 PyQt5 不足以编译 GUIPlugin。上面从实际 Qt5Core 开发包的 host_bins 选择 MOC 并检查版本，避免把存在但未配置 qtchooser 的 `/usr/bin/moc` 当作可用工具；29/30课按已验证的 Qt 5.12.8 开发环境声明要求。终端和编辑器需要真实桌面会话。平台继承当前 DISPLAY，不猜测设备上的显示编号；ROS 课程在其私有环境加载 Noetic，无需全局覆盖宿主的 ROS 2 配置。ros_control 的控制器接口、transmission 和 controller_manager 背景可查 [Classic 官方教程](https://classic.gazebosim.org/tutorials?cat=connect_ros&tut=ros_control)。

## 3. 下载、校验和安装

从仓库的 [GitHub Releases](https://github.com/StareamCather/GazeboTeach/releases) 中选择 V2，至少下载：

- `GazeboTeach-2.0.0-ubuntu20.04-x86_64.sh`：包含完整源码载荷的自解包安装器。
- `SHA256SUMS` 和 `release-manifest.json`：资产摘要、来源提交 / tree 与逐文件摘要。
- `V2_INSTALL.md`、`RELEASE_NOTES_V2.md` 和 `V2_VERIFICATION.md` / `.json`：使用说明与该次发行的实际验证范围。

`GazeboTeach-2.0.0-ubuntu20.04-x86_64.tar.gz` 是同一载荷的归档形式；使用 `.sh` 时不必另外下载。发布文档也包含在载荷的 `docs/` 中。仓库为私有时，下载需要你已有的仓库访问权限；安装器不索要或保存 GitHub 凭证。

```bash
cd /你的下载目录
sha256sum --ignore-missing -c SHA256SUMS
bash GazeboTeach-2.0.0-ubuntu20.04-x86_64.sh --check
bash GazeboTeach-2.0.0-ubuntu20.04-x86_64.sh
```

务必看到目标 `.sh` 在校验输出中显示 `OK`；`--ignore-missing` 允许只下载所需资产，不代表缺失的文件已校验。`--check` 再验证内嵌载荷长度 / SHA256、路径、清单与每个文件，不创建安装目录。文件转入另一台设备后，在接收端重新校验；无需复制任何 GitHub 或 SSH 私钥。

默认结构：

```text
~/.local/opt/GazeboTeach/
├── bin/gazeboteach
├── current -> versions/2.0.0
└── versions/2.0.0/
    ├── courses/                    # 39课模板与源码
    ├── resources/native_control.cc
    ├── docs/
    └── release-manifest.json
```

自定义前缀与可选桌面文件：

```bash
bash GazeboTeach-2.0.0-ubuntu20.04-x86_64.sh \
  --prefix "$HOME/Applications/GazeboTeach" --desktop
```

`--desktop` 只写 `PREFIX/share/applications/gazeboteach.desktop`。需要桌面菜单时，检查该文件的 Exec 后自行复制到 `~/.local/share/applications/`；不要复制旧版本中指向错误前缀的入口。安装前缀应为自己的独立应用目录。

## 4. 启动与第一轮实验

```bash
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --diagnose
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --list
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
```

自定义安装时替换为实际 `PREFIX/bin/gazeboteach`。先读环境结果，再从 L01 开始：阅读目标和预期现象 → 开始实验 → 等初始化与本课窗口就绪 → 按教材预测、观察和单变量修改 → 运行本课验收并记录实际结果 → 结束实验。平台提供的前置资料入口只阅读内部教材；查资料时不用停止或重建当前世界。

可选的桌面检查和单课真实检查：

```bash
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --gui-check
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --smoke 01 --gui --terminals
```

`--smoke` 默认为这一次检查新建临时个人数据目录，输出真实验收和日志路径，结束时清理本会话进程。无 GUI 的 smoke 不覆盖 VisualPlugin、GUIPlugin 或人工图形目标，窗口打开也不等于课程完成。不要把 `--smoke 091` / `092` 默认模板直接当作参考答案回归：这两课保留关键代码填空，默认未评分是预期状态。学习时填写自己的副本；发行回归在**全新临时副本**中使用独立参考答案，学生工作不被覆盖。两课范围是近直立稳定控制，不包含完整起摆算法。

暂停 / 继续操作要等待实际回读；精确单步按物理迭代数确认。框架“模型复位”保留仿真时钟，不等于重开进程或清空传感器 / 控制器全部缓存。L36 / L37 有自己的任务复位和恢复步骤，应依各课说明操作。L37 的 HOLD 阶段仍以 PD 力矩保持末端目标并读取真实腕部力矩反馈，看到任务完成后存在非零力矩并不等于仍在推送。

## 5. 个人练习、升级和卸载

应用模板和个人数据分开。默认个人目录为 `~/.local/share/GazeboTeach`，包含练习、学习记录、会话日志和课程构建输出。可指定版本目录外的独立路径：

```bash
GAZEBOTEACH_DATA_DIR="$HOME/GazeboTeachLearning" \
  "$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
```

桌面入口也支持 `--data-dir`。共享的原生控制 / 倒立摆构建缓存位于 `${XDG_CACHE_HOME:-~/.cache}/gazeboteach`，缓存可重新生成；备份优先保留个人数据。新版安装保留旧练习，不用旧副本假冒新版课程的验证。需要新版模板时先备份自己的修改，再使用明确的恢复模板操作。

升级前结束当前实验和应用，校验新版安装器，在同一 `--prefix` 安装。不同版本并存于 `versions/`，`current` 原子切到新版本。重复安装同版本会校验复用；修改过的版本目录和无关文件会被拒绝覆盖。

使用**同一版本、同一载荷**的安装器卸载：

```bash
bash GazeboTeach-2.0.0-ubuntu20.04-x86_64.sh --uninstall
```

自定义安装要加相同 `--prefix`。卸载仅移除该份未修改的版本、适用的当前入口及前缀内受管桌面文件，保留其他版本和个人数据；不会自动切回旧版本。你手动复制到用户菜单的 `.desktop` 文件由你管理。安装器拒绝已修改目录时，先检查和备份，不对个人数据做递归删除。

## 6. 日志与排错

| 现象 | 首要证据 |
| --- | --- |
| 检测到 ROS 2 / `gz sim`，39课仍禁用 | 具体 Classic / SDK / Noetic 缺失项；现代 Gazebo 命令同名不意味着 API 相同 |
| 基础课可开，C++课不可开 | SDK 11.15.1、g++、pkg-config、实际运行库；Qt课还要 Qt5 开发包和 MOC |
| SDK 版本正确仍编译失败 | 本次构建日志、头文件 / DSO 来源和指纹；不要改系统头文件“补兼容” |
| Qt / Gazebo / RViz 黑屏或无显示 | 当前 DISPLAY、Xauthority、图形会话和实际 OpenGL renderer |
| 原生相机、深度、GPU 激光无有效消息 | 服务端 / GUI 日志、渲染能力、时间戳和真实传感器输出 |
| L091 / L092 未评分或学生验收失败 | 个人控制器关键行是否完成；参考答案检查与默认模板分开 |
| 暂停 / 单步报告未知结果 | session.json、native-control.stderr.log；不要向同一世界重发修改命令 |
| 更新后仍运行旧模板 | 安装入口、source_commit、个人副本版本和是否按教材重载 |

反馈时附：应用版本、`release-manifest.json` 的 source_commit、`--diagnose` 输出、课程号、实际操作和会话日志。平台只管理自己创建的 Gazebo、ROS、终端与助手；不要使用全局 `killall` / `pkill` 清理其他项目。

## 7. 发行包构建与安装流程

V2 打包器从已提交的单一 Git commit 读取 blob，不混入工作区未提交修改。流程为：拒绝脏的已跟踪源文件 → 解析 revision 为完整 commit / tree → 读取所有允许的已跟踪常规文件 → 校验精确 39 个课程 ID、Classic profile、原生控制源码和 V2 文档 → 为每个文件计算大小 / 权限 / SHA256 → 用提交时间构建确定性归档 → 将归档及摘要嵌入安装器 → 从同一提交导出独立文档资产 → 写 SHA256SUMS。私钥、Git 历史、缓存、实验日志、checkpoint、生成构建和主机依赖不进入载荷。

安装流程沿用受管版本目录：完整校验归档与清单 → 检查专用前缀和数据路径 → 加安装锁 → 写临时版本目录 → 再次核对全部文件 → 原子安装版本 → 写受管入口 → 原子切换 current。只有完整安装成功才切换入口。构建测试覆盖确定性、精确 V2 课数、缺课拒绝、显式 V1 构建、脏源拒绝、资产摘要和损坏载荷拒绝；Linux 安装 / 启动 / 重装 / 卸载结果由最终验证文件记录，不能用 Windows 上跳过的测试替代。
