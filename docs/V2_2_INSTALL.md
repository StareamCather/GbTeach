# GazeboTeach v2.2.0 安装与部署

GazeboTeach 通过 Bash 启动中文桌面应用。v2.2.0 包含 62 个独立项目，按三个运行配置列出 69 个课程入口。Classic／Noetic 有 55 个项目，七门 ROS 2 专项分别提供 Humble／Fortress 和 Jazzy／Harmonic 变体。同号课程的个人练习和学习记录按课包保存。

## 1. 选择运行环境

| 课包 | 课程入口 | 实测环境 |
|---|---|---|
| `classic11-noetic` | 01–53、091、092 | ikaros 原生 Ubuntu 20.04 x86_64，Gazebo Classic 11.15.1、ROS Noetic、Python 3.8、Qt 5.12 |
| `fortress-humble` | 01、11、22–26 | ikaros 的 Ubuntu 22.04.5 x86_64 GPU 容器，Gazebo Fortress 6.18.0、ROS 2 Humble、Python 3.10、Qt 5.15 |
| `harmonic-jazzy` | 01、11、22–26 | ikaros 的 Ubuntu 24.04 x86_64 GPU 容器，Gazebo Harmonic 8.15.0、ROS 2 Jazzy、Python 3.12、Qt 5.15 |

启动时，应用分别探测 Gazebo 主版本、ROS 安装及各课依赖，优先选择可运行的课包。进入环境检测窗口可查看缺项，再从左侧切换教学包。其他组合的运行能力按逐课检测结果显示。独立 SDF 课 01 可在满足对应 Gazebo 和图形依赖时运行。

本版复验两套 ROS 2 课包各七门的真实行为、图形控制和桌面交互，Classic 采用既有全课验收基线并回归代表课程。具体运行记录和来源提交写入发行资源 `V2_2_VERIFICATION.json`。ROS 2 的接口差异、观测值和课内流程见 [现代 Gazebo 适配记录](MODERN_GAZEBO.md)。

## 2. 下载并核对发行资源

在 GitHub 选择 v2.2.0，下载下面六项资源。仓库为私有时，先登录有访问权限的账号。[v2.2.0 发行页](https://github.com/StareamCather/GazeboTeach/releases/tag/v2.2.0)

| 文件 | 用途 |
|---|---|
| `GazeboTeach-2.2.0-linux-x86_64.sh` | 含应用资源的安装器 |
| `GazeboTeach-2.2.0-linux-x86_64.tar.gz` | 同一提交的源码应用归档 |
| `release-manifest.json` | 来源提交、课程身份与逐文件 SHA-256 |
| `V2_2_INSTALL.md` | 本安装部署指南 |
| `V2_2_VERIFICATION.json` | 发行验收记录 |
| `SHA256SUMS` | 上述资源的摘要 |

将资源放到同一目录，在 Linux 终端运行。

```bash
sha256sum --ignore-missing -c SHA256SUMS
bash GazeboTeach-2.2.0-linux-x86_64.sh --check
bash GazeboTeach-2.2.0-linux-x86_64.sh --desktop
```

先确认目标安装器显示 `OK`。`--ignore-missing` 会跳过未下载的资源，`--check` 会继续校验安装器内嵌归档、清单和逐文件摘要。通过本机中转资源后，在目标设备再执行一次上述校验。

默认前缀为 `~/.local/opt/GazeboTeach`，启动入口为 `bin/gazeboteach`，应用文件位于 `versions/2.2.0`，`current` 指向当前版本。可用 `--prefix /自己的专用目录` 指定前缀。`--desktop` 在该前缀的 `share/applications` 中生成桌面文件，可从文件管理器打开，或自行复制到个人桌面启动项目录。

安装器部署应用、教材、场景和构建脚本。ROS、Gazebo、系统 Python 包和图形运行时按下面的环境步骤准备。采用手动解包时，在归档展开后的目录执行 `./launch.sh`。

## 3. 原生环境依赖

### Ubuntu 22.04、Humble、Fortress

先按 ROS 官方说明配置 Humble 软件源和 ROS 2 基础环境，再补齐七门专项所需的依赖。[Humble 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

```bash
sudo apt update
sudo apt install python3-pyqt5 python3-numpy build-essential cmake pkg-config \
  python3-colcon-common-extensions xterm fonts-noto-cjk \
  ros-humble-ros-gz ros-humble-rosbag2 ros-humble-rosbag2-py \
  ros-humble-rosbag2-storage-default-plugins ros-humble-robot-state-publisher \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  ros-humble-ros2lifecycle
source /opt/ros/humble/setup.bash
ign gazebo --versions
```

`ign gazebo --versions` 应列出主版本 6。Fortress 的命令、消息和插件使用 Ignition 名称，课程会据课包选择对应配置。Humble 的官方配对是 Fortress。[Gazebo 官方配对说明](https://gazebosim.org/docs/fortress/ros_installation/)

### Ubuntu 24.04、Jazzy、Harmonic

配置 Jazzy 官方软件源后安装对应依赖，读取实际 Sim 版本。[Jazzy 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

```bash
sudo apt update
sudo apt install python3-pyqt5 python3-numpy build-essential cmake pkg-config \
  python3-colcon-common-extensions xterm fonts-noto-cjk \
  ros-jazzy-ros-gz ros-jazzy-rosbag2 ros-jazzy-rosbag2-py \
  ros-jazzy-rosbag2-storage-default-plugins ros-jazzy-robot-state-publisher \
  ros-jazzy-rosidl-default-generators ros-jazzy-rosidl-default-runtime \
  ros-jazzy-ros2lifecycle
source /opt/ros/jazzy/setup.bash
gz sim --versions
```

### Ubuntu 20.04、Noetic、Classic

沿用 Classic 的系统包、SDK、ROS 插件和 Qt 开发工具准备流程。精确 SDK 课程会核对 Classic 11.15.1 的头文件、运行程序和所需库，额外依赖在环境窗口逐课列出。[Classic 依赖准备](V2_INSTALL.md#2-原生-ubuntu-2004-的依赖准备)

## 4. 启动和验证

在实际 Linux 图形桌面的终端运行。指定了安装前缀时，相应替换入口。

```bash
gt="$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
"$gt" --diagnose
"$gt" --list
"$gt" --gui-check
"$gt"
```

选择当前机器对应的课包运行实验。

```bash
# Ubuntu 22.04 / Humble / Fortress
"$gt" --profile fortress-humble --smoke 01 --gui --terminals
"$gt" --profile fortress-humble --smoke 11 --gui --terminals
"$gt" --profile fortress-humble

# Ubuntu 24.04 / Jazzy / Harmonic
"$gt" --profile harmonic-jazzy --smoke 11 --gui --terminals
"$gt" --profile harmonic-jazzy

# Ubuntu 20.04 / Noetic / Classic
"$gt" --profile classic11-noetic --smoke 01 --gui --terminals
"$gt" --profile classic11-noetic
```

`--gui-check` 打开 Qt 工作台和环境结果窗口。01 检查真实物理场景，现代 11 进一步检查差速车、ROS 2 桥和 GPU 激光。渲染课程需要可用的图形上下文，GPU 激光需在目标环境实际运行。

在桌面选择课程并点“开始实验”。应用准备个人副本，执行本课构建和初始化，启动服务端、节点、Gazebo 窗口与终端，再开放操作。暂停后点“单步”，日志显示确认过的仿真时间和迭代增量。现代 Gazebo 用“重启实验”恢复整个场景。点击“结束实验”关闭本课拥有的进程。

## 5. 手动部署 Jammy Docker 环境

本节给出本地 Linux x86_64、NVIDIA GPU 与 X11 桌面的示例。Docker Engine、当前用户的 Docker 使用权限及 NVIDIA 容器图形运行时应已可用。`docker info` 检查 daemon，GPU 容器需同时能使用现有驱动和图形设备。[Docker 安装说明](https://docs.docker.com/engine/install/ubuntu/) · [NVIDIA 容器运行检查](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/sample-workload.html)

实际项目验证使用 ikaros 的独立 Jammy GPU 容器和当前桌面授权。下面的可执行示例固定同一个官方基础镜像摘要，系统包在构建时从镜像的软件源安装，用户需在自己的机器上重新完成第 4 节检查。软件源中包的小版本可能继续更新。

```bash
mkdir -p "$HOME/GazeboTeachDocker22"/{release,home,data,cache}
cd "$HOME/GazeboTeachDocker22"
```

将已校验的 v2.2.0 六项资源放入 `release/`，创建 `Dockerfile`。

```dockerfile
FROM ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config python3-pyqt5 python3-numpy \
    python3-colcon-common-extensions ignition-fortress ros-humble-ros-gz \
    ros-humble-rosbag2 ros-humble-rosbag2-py ros-humble-rosbag2-storage-default-plugins \
    ros-humble-robot-state-publisher ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime ros-humble-ros2lifecycle \
    libgl1 libglx0 libegl1 mesa-utils xterm xauth x11-utils fonts-noto-cjk \
    && rm -rf /var/lib/apt/lists/*
ARG LOCAL_UID=1000
ARG LOCAL_GID=1000
RUN groupadd --gid "$LOCAL_GID" student \
    && useradd --uid "$LOCAL_UID" --gid "$LOCAL_GID" \
       --create-home --shell /bin/bash student
ENV HOME=/home/student
USER student
WORKDIR /home/student
```

```bash
docker build --build-arg LOCAL_UID="$(id -u)" --build-arg LOCAL_GID="$(id -g)" \
  -t gazeboteach:humble-2.2 .
```

创建 `run-jammy.sh`。脚本读取当前桌面的 Xauthority cookie，并只读挂载显示授权和发行资源。`home` 保存安装目录，`data` 保存练习和学习记录，`cache` 保存课内构建缓存。路径对应 Docker daemon 所在的本机文件系统。[Docker bind mount 说明](https://docs.docker.com/engine/storage/bind-mounts/)

```bash
#!/bin/bash
set -euo pipefail
GT_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
: "${DISPLAY:?请在本地 Linux 图形桌面终端运行}"
GT_XAUTH="$(mktemp "${XDG_RUNTIME_DIR:-/tmp}/gazeboteach-22-xauth.XXXXXX")"
trap 'rm -f -- "$GT_XAUTH"' EXIT
chmod 600 "$GT_XAUTH"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$GT_XAUTH" nmerge -
test -s "$GT_XAUTH"
docker run --rm -it --init --name gazeboteach-humble-22 \
  --gpus all --env NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
  --env DISPLAY="$DISPLAY" --env XAUTHORITY=/tmp/gazeboteach.xauth \
  --env QT_X11_NO_MITSHM=1 --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
  --env HOME=/home/student --env GAZEBOTEACH_DATA_DIR=/home/student/learning \
  --env XDG_CACHE_HOME=/home/student/.cache \
  --mount 'type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly' \
  --mount "type=bind,src=$GT_XAUTH,dst=/tmp/gazeboteach.xauth,readonly" \
  --mount "type=bind,src=$GT_ROOT/release,dst=/release,readonly" \
  --mount "type=bind,src=$GT_ROOT/home,dst=/home/student" \
  --mount "type=bind,src=$GT_ROOT/data,dst=/home/student/learning" \
  --mount "type=bind,src=$GT_ROOT/cache,dst=/home/student/.cache" \
  gazeboteach:humble-2.2 bash
```

```bash
chmod +x run-jammy.sh
./run-jammy.sh
```

首次进入容器后安装应用，并从容器内核对版本、显示和课程。

```bash
source /opt/ros/humble/setup.bash
cd /release
sha256sum --ignore-missing -c SHA256SUMS
bash GazeboTeach-2.2.0-linux-x86_64.sh --check
bash GazeboTeach-2.2.0-linux-x86_64.sh
gt="$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
ign gazebo --versions
glxinfo -B
"$gt" --diagnose
"$gt" --gui-check
"$gt" --profile fortress-humble --smoke 11 --gui --terminals
"$gt" --profile fortress-humble
```

后续从同一 `run-jammy.sh` 进入，使用已保存在 `home/` 的入口。Wayland 桌面可在可用的 Xwayland 会话中检查 `DISPLAY`、X11 socket 与授权数据。Docker Rootless、用户命名空间映射或 UID／GID 已占用时，按实际映射调整容器用户与持久目录权限。

Jazzy／Harmonic 容器采用 Ubuntu 24.04 和对应的 `ros-jazzy-*` 依赖，再运行 `--profile harmonic-jazzy` 验收。Classic 容器可沿用 Focal 示例，并在容器内核对精确 SDK、运行库及插件版本。[Classic 手工 Docker 指南](V2_DOCKER.md) · [现代 Gazebo 环境说明](MODERN_GAZEBO.md)

平台的 Docker 连接配置界面保存接入参数，镜像构建、容器启动和应用执行使用本节手动流程。

## 6. LIO 课程和离线资源

Classic 课 50 FAST-LIO、51 FAST-LIO2、52 Point-LIO、53 LIO-SAM 包含教材、仿真场景和适配构建脚本。上游源码按固定提交与 SHA-256 下载到每课独立缓存。系统开发包按环境检测结果预先准备，53 的脚本会在课内缓存构建固定版 GTSAM。

| 课号 | 练习内脚本 | `${XDG_CACHE_HOME:-$HOME/.cache}/gazeboteach` 下的缓存 |
|---|---|---|
| 50 | `build_fast_lio1.py` | `a50-fast-lio1` |
| 51 | `build_fast_lio2.py` | `a51-fast-lio2` |
| 52 | `build_point_lio.py` | `a52-point-lio` |
| 53 | `build_lio_sam.py` | `a53-lio-sam` |

离线设备可在联网机器按各脚本声明的文件名和摘要准备归档，再中转到目标缓存的 `archives/`。也可在个人练习目录运行脚本并传入 `--archives /归档目录`。脚本校验归档后构建，后续复用已完成的缓存。记录和观测均来自对应仿真的雷达与 IMU，结果位置见各课教材。

## 7. 升级、个人数据与卸载

沿用已有安装前缀安装 v2.2.0，入口通过 `current` 切到 `versions/2.2.0`。安装前若编辑过应用安装树，安装器会报告差异，先保存这些改动，再在新专用前缀安装。日常练习应通过应用的“编辑练习文件”进入个人副本。

个人数据默认位于 `~/.local/share/GazeboTeach`，可用 `GAZEBOTEACH_DATA_DIR` 指定。练习、学习记录和会话日志独立于应用版本目录。模板更新时先查看差异，恢复起点会按应用流程保存备份。Docker 用户保留宿主的 `home/`、`data/` 和 `cache/`，下次用同样挂载继续学习。

卸载指定版本时，使用同一安装器和安装前缀。

```bash
bash GazeboTeach-2.2.0-linux-x86_64.sh --uninstall
# 自定义前缀示例
bash GazeboTeach-2.2.0-linux-x86_64.sh --prefix /自己的专用目录 --uninstall
```

排错时保存 `--diagnose`、课包、课号、会话日志和 `release-manifest.json` 的 `source_commit`。Qt 窗口、Gazebo 渲染、ROS 通信与课程行为可按第 4 节分步检查。容器内的版本、图形和依赖以容器内检查结果为准。
