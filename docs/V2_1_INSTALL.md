# GazeboTeach v2.1.0 安装与运行

GazeboTeach 是通过 Bash 启动的中文桌面教学应用。v2.1.0 包含 62 个实验，其中 55 个属于 Gazebo Classic 11 / ROS Noetic，7 个属于 Gazebo Harmonic / ROS 2 Jazzy。课号在两个教学包中可以重复，命令行运行时用 `--profile` 指定教学包。

## 1. 选择环境

| 教学包 | 课程范围 | 项目验证环境 |
|---|---|---|
| `classic11-noetic` | 01–53 及 091、092 | ikaros 原生 Ubuntu 20.04 x86_64，Classic 11.15.1、ROS Noetic、Python 3.8、Qt 5.12 |
| `harmonic-jazzy` | 01、11、22–26 | ikaros 上的 Ubuntu 24.04 x86_64 隔离容器，Harmonic Sim 8.15.0、ROS 2 Jazzy、Python 3.12、Qt 5.15 |

环境窗口会列出当前识别到的版本、课程缺失的依赖和运行受限的原因。满足各课声明依赖的无 ROS 环境可运行独立场景，例如 Sim 8 上的现代 01 课。Ubuntu 22.04 / Humble / Fortress 等其他组合可运行环境检测、浏览教材，具体实验以兼容状态为准。已验证运行环境、交互证据和实验记录见[扩展验证记录](EXPANSION_VALIDATION.md)。

- **Classic 原生环境**可沿用 Focal、系统 PyQt5、Classic 11 和 Noetic 的准备步骤。额外编译工具、传感器插件和 ROS 包按课程环境检查提示补齐。[Classic 依赖准备](V2_INSTALL.md#2-原生-ubuntu-2004-的依赖准备)
- **Harmonic 原生环境**在 Ubuntu 24.04 配置 Jazzy 的官方软件源后，安装 `python3-pyqt5` 和 `ros-jazzy-ros-gz`。根据所选课程再准备 `rosbag2_py`、`robot_state_publisher`、colcon 和 ROS 接口生成工具等。该安装方式在自己的原生桌面上用下文命令单独验证。[Jazzy 官方安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) · [Gazebo 官方配对说明](https://gazebosim.org/docs/harmonic/ros_installation/) · [现代教学包说明](MODERN_GAZEBO.md)

## 2. 下载、校验和安装

在 [GitHub Releases](https://github.com/StareamCather/GazeboTeach/releases) 选择 v2.1.0，下载 `GazeboTeach-2.1.0-linux-x86_64.sh` 和 `SHA256SUMS` 到同一目录。`release-manifest.json` 记录来源提交、课程及逐文件摘要。扩展课程的验收记录另有 `EXPANSION_VALIDATION.md` 和 `EXPANSION_VERIFICATION.json` 供查阅。本文的相对文档链接可在解包后的 `docs/` 中打开。需要手动解包时可下载同名 `.tar.gz`，在展开后的目录运行 `./launch.sh`。

```bash
sha256sum --ignore-missing -c SHA256SUMS
bash GazeboTeach-2.1.0-linux-x86_64.sh --check
bash GazeboTeach-2.1.0-linux-x86_64.sh
```

先确认目标安装器在摘要检查中显示 `OK`。`--ignore-missing` 会跳过未下载的资产。`--check` 进一步核对内嵌归档、清单和逐文件摘要，适合将资产中转到另一台机器后再次执行。

默认安装到 `~/.local/opt/GazeboTeach`，入口是 `bin/gazeboteach`，当前版本由 `current` 指向 `versions/2.1.0`。可以在安装命令加 `--prefix /自己的专用目录`，也可加 `--desktop` 在安装前缀的 `share/applications` 生成桌面文件。安装器只部署应用文件，宿主的软件源、系统包、ROS、Gazebo、GPU 驱动及容器由用户准备。

## 3. 首次启动与原生验证

在实际 Linux 图形会话的终端运行。修改过安装前缀时，替换以下入口路径。

```bash
gt="$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
"$gt" --diagnose
"$gt" --list
"$gt" --gui-check
```

再选择当前环境对应的一组。

```bash
# Classic 11 / Noetic
"$gt" --profile classic11-noetic --smoke 01 --gui --terminals
"$gt" --profile classic11-noetic

# Harmonic / Jazzy
"$gt" --profile harmonic-jazzy --smoke 01 --gui --terminals
"$gt" --profile harmonic-jazzy --smoke 11 --gui --terminals
"$gt" --profile harmonic-jazzy
```

`--gui-check` 检查 Qt 桌面窗口。01 验证该教学包的物理实验，现代 11 进一步验证 ROS 2 桥接、差速车和 GPU 激光。无图形终端可省略 smoke 命令的 `--gui --terminals` 进行有界自动验收。涉及渲染或人工操作的课仍需在真实图形环境检查。

在桌面选择课程后点“开始实验”。平台准备个人副本，启动本课所需模拟器、节点和终端，再开放操作。运行中可暂停、继续或在暂停下单步，观察课内日志及仿真时钟。Harmonic 场景用“重启实验”恢复初始状态。离开时点“结束实验”，由应用清理本次会话创建的进程。

## 4. 手动配置 Docker 环境

在 Ubuntu 22.04 或其他组合上可以保留现有宿主环境，单独配置 Focal / Classic 或 Noble / Harmonic 容器。平台可保存 Docker 连接参数供接入使用，当前的容器创建、依赖安装及应用启动均从容器外部或内部手动执行。

1. Classic 容器复用 Focal 示例、版本核对、有限 X11 授权及持久数据卷配置。到容器安装步骤时，使用本页的 v2.1.0 安装器及命令替换旧资产。[Classic Docker 手工指南](V2_DOCKER.md)
2. Harmonic 容器用 Ubuntu 24.04，按上文的 Jazzy 和 `ros_gz` 配对安装。沿用有限显示授权思路，使用当前会话的 X11 授权数据和套接字。GPU 实验需在容器内能实际建立 Ogre2 渲染上下文，按宿主驱动和容器运行时配置图形设备。将课程数据和依赖缓存放到各自可写的持久卷。[现代教学包的容器说明](MODERN_GAZEBO.md#运行入口)
3. 在**容器内**重新运行 `--diagnose`、`--list`、`--gui-check` 以及该教学包的 01 smoke。Harmonic 继续运行 11 smoke 检查 GPU 激光。在容器内的 ROS 环境核对 Jazzy 包索引与 `gz sim --versions`，Classic 环境核对 `gazebo --version`、`rosversion -d`。宿主通过的结果不能用于判断容器内的显示、渲染和依赖。

## 5. 首次运行四门 LIO 课

50 FAST-LIO、51 FAST-LIO2、52 Point-LIO、53 LIO-SAM 属于 `classic11-noetic`。应用包包含教材、仿真场景、构建脚本和适配代码。上游估计器在选课初始化时按固定提交和归档 SHA256 下载、验证并构建到各课独立的用户缓存。四门课的构建不会调用系统包管理器或写入系统 ROS 工作区。需要的系统开发包按课程环境检测提示预先准备，53 还会在课内依赖缓存构建固定版本的 GTSAM。

| 课号 | 练习内构建脚本 | `${XDG_CACHE_HOME:-$HOME/.cache}/gazeboteach` 下的目录 |
|---|---|---|
| 50 | `build_fast_lio1.py` | `a50-fast-lio1` |
| 51 | `build_fast_lio2.py` | `a51-fast-lio2` |
| 52 | `build_point_lio.py` | `a52-point-lio` |
| 53 | `build_lio_sam.py` | `a53-lio-sam` |

设备离线时，按各课脚本里的固定文件名和摘要在联网设备准备原始归档，再放入目标缓存的 `archives/`。也可以在对应个人练习目录运行构建脚本并传 `--archives /实际归档目录`，由脚本先校验再构建。首次编译完成后可复用缓存。各课教材给出结果路径与日志，运行时采集本课仿真的雷达和 IMU，再回放到对应估计器。

## 6. 个人数据、升级和排错

默认个人数据目录为 `~/.local/share/GazeboTeach`，可通过 `GAZEBOTEACH_DATA_DIR` 指定专用目录。个人练习、学习记录、会话日志独立于安装树。不同教学包的同号课程使用独立副本。沿用原安装前缀即可通过同一个 `bin/gazeboteach` 入口运行新版本，升级无需再次加 `--desktop`。保留数据目录即可继续练习，模板更新提示出现时可比较差异，确认后再恢复模板。

若需卸载该版本，使用同一安装器执行 `bash GazeboTeach-2.1.0-linux-x86_64.sh --uninstall`。若安装时指定了 `--prefix`，卸载时沿用同一路径。个人数据和依赖缓存位于安装目录外。

遇到问题时记录环境窗口给出的缺项，保存 `--diagnose`、教学包和课号、会话日志及清单里的 `source_commit`。Qt 启动、Gazebo 图形显示、ROS 通信和课程行为是不同检查项，可分别用第 3 节命令定位。
