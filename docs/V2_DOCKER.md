# V2 手动 Docker 兼容环境

本指南面向已有 Ubuntu 22.04 / 24.04、ROS 2 或现代 Gazebo 的 Linux 桌面用户，在独立容器内准备 Focal / Noetic / Classic 环境。V2 提供 Docker 后端接口与配置校验，**没有自动连接 daemon、拉取镜像或在容器内执行课程**。以下操作由用户手动完成；本指南的镜像构建、显示、GPU 和39课组合尚未完成端到端实测。原生发行基准与实际回归结果另见 [安装指南](V2_INSTALL.md) 和发布验证文件。

## 1. 边界与宿主条件

ROS 1 Noetic 的 `rospy`、ROS Master、catkin 和 Classic 插件，不会因为宿主安装了 ROS 2 或 `gz sim` 就转换为 ROS 2 / 现代 Gazebo。现代 Gazebo 使用自己的库、系统和 `ros_gz` 集成；共通的物理与坐标概念可以学习，节点、插件、消息和启动文件需要专门移植。[现代 Gazebo 官方 ROS 2 概览](https://gazebosim.org/docs/latest/ros2_overview/)

示例需要本地 Linux x86_64、可用的 Docker Engine、当前用户已获准使用的本地 Docker context、X11 或可用的 Xwayland。先按 [Docker 官方 Ubuntu 安装指南](https://docs.docker.com/engine/install/ubuntu/) 自行准备，或在满足 UID 映射等前提时采用 [Docker Rootless 模式](https://docs.docker.com/engine/security/rootless/)。本文不更改宿主服务、驱动、用户组、SSH、路由或访问控制。仅安装 Docker CLI 而没有可用 daemon 不能执行下列流程。

```bash
docker context show
docker version
docker info
command -v xauth
test -n "$DISPLAY"
test -d /tmp/.X11-unix
```

bind mount 使用 daemon 所在主机的路径，本例不覆盖远程 daemon、Windows Docker Desktop、WSL 或 SSH X11 转发；不要把这些路径直接照搬到另一台 daemon 主机。[Docker 官方挂载说明](https://docs.docker.com/engine/storage/bind-mounts/)

## 2. 核对镜像与精确版本

2026-11-01 只读核对时，OSRF 的 `osrf/ros:noetic-desktop-full` 标签元数据仍存在，架构为 amd64，索引摘要为 `sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797`，元数据最后更新时间为 2025-06-10。OSRF Dockerfile 的基础仍是 Focal / Noetic，维护清单已将 Noetic 标为 EOL。[OSRF Dockerfile](https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/desktop-full/Dockerfile)、[官方标签元数据](https://hub.docker.com/v2/repositories/osrf/ros/tags/noetic-desktop-full/)、[OSRF 维护清单](https://github.com/osrf/docker_images/blob/master/ros/manifest.yaml)

这个镜像是**环境候选**，不是已经制作好的 GazeboTeach 全课镜像。上述摘要只标识镜像内容，不证明它包含 Classic 11.15.1 SDK、匹配的运行 DSO、gazebo_plugins 2.9.3 或全部 Qt / ROS 依赖。镜像标签可变化；使用时重新核对，保存实际 digest，并在需要复现时固定 digest。[Docker 镜像引用与摘要](https://docs.docker.com/engine/containers/run/)

```bash
docker pull --platform linux/amd64 osrf/ros:noetic-desktop-full
docker image inspect osrf/ros:noetic-desktop-full --format '{{json .RepoDigests}}'
docker run --rm osrf/ros:noetic-desktop-full bash -lc \
  'source /opt/ros/noetic/setup.bash; gazebo --version; gzserver --version; rosversion -d; rosversion gazebo_plugins; pkg-config --modversion gazebo'
```

缺少 pkg-config / SDK 时最后一项失败会暴露缺项，不能把它解释成11.15.1。下一节安装依赖后再次检查。

## 3. 构建用户自己的教学环境

在宿主桌面终端建立专用目录，把已经校验的 V2 安装器和 SHA256SUMS 放入 `release/`：

```bash
mkdir -p "$HOME/GazeboTeachDockerV2"/{release,home,data,cache}
cd "$HOME/GazeboTeachDockerV2"
```

创建 `Dockerfile`。`GT_BASE` 可替换为上一节核对过的完整 `osrf/ros@sha256:...`；不要使用没有实际拉取验证的自编摘要。

```dockerfile
ARG GT_BASE=osrf/ros:noetic-desktop-full
FROM ${GT_BASE}
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pyqt5 python3-numpy python3-dev \
    build-essential cmake pkg-config libgazebo11-dev qtbase5-dev qtbase5-dev-tools \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control ros-noetic-ros-controllers \
    ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
    xterm xauth x11-utils mesa-utils gedit fonts-noto-cjk \
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
  -t gazeboteach:noetic-v2 .
docker run --rm gazeboteach:noetic-v2 bash -lc \
  'source /opt/ros/noetic/setup.bash; gazebo --version; gzserver --version; pkg-config --modversion gazebo Qt5Core Qt5Widgets; rosversion gazebo_plugins; GT_QT_HOST_BINS=$(pkg-config --variable=host_bins Qt5Core); "$GT_QT_HOST_BINS/moc" -v; /usr/bin/python3 -c "from PyQt5 import QtWidgets; import numpy"'
```

构建使用镜像内的软件源，不修改宿主的 Noetic 或 ROS 2。UID / GID 已占用时构建应失败：检查基础镜像用户后调整 Dockerfile，不覆盖已有账号。普通 rootful daemon 下，运行用户的 UID / GID 应与专用 bind 目录所有者一致。Rootless 或 userns-remap 有额外映射，按第6节处理。

**版本门槛在此生效。** `apt install libgazebo11-dev` 不保证11.15.1，`ros-noetic-gazebo-ros-pkgs` 不保证 gazebo_plugins 2.9.3。版本不满足时，可以先阅读、运行明确符合条件的基础课；精确 SDK / 插件课不会因此解禁。要构建完整兼容环境，由维护者在这个独立镜像内固定可用的同源 deb 版本，或从官方 [Classic 11.15.1 标签](https://github.com/gazebosim/gazebo-classic/tree/gazebo11_11.15.1) 构建到独立前缀，并让头文件、pkg-config、gzserver 与加载的 DSO 全部来自该前缀；需源码固定 ROS 插件时参考 [gazebo_ros_pkgs 2.9.3](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3)。随后重新构建受影响插件，记录源码 commit、依赖版本和路径，再运行应用诊断与实际验收。

官方[源码构建原理说明](https://classic.gazebosim.org/tutorials?tut=install_from_source&cat=install)可用于理解依赖 / 编译 / 安装流程，其中旧发行版的删包、apt-key 和全局 shell 配置示例不应照抄到现代宿主。本文没有提供已验证的全依赖锁文件，也没有证明两套 Gazebo 混装的 ABI 兼容性。SDK 字符串匹配后仍必须核对实际库身份，不能仅将 pkg-config 的版本字段改成11.15.1。

镜像或旧软件源下载失败时保留错误并检查来源；可以在已有合法联网环境中 `docker save` / `docker load` 转移经核对的镜像。不要跳过 TLS 或 apt 签名，也不要为了这份教学包改宿主软件源。

## 4. X11 有限授权与持久数据

下面脚本使用当前 DISPLAY 对应的 Xauthority cookie，退出后删除临时副本。它没有使用 `xhost +`、`--privileged`、宿主网络、端口发布或 Docker socket 挂载。容器内部的 ROS / Gazebo 通信留在独立网络；X11 socket 和 cookie 为只读。Xauthority 是显示访问凭证，应只交给你信任的教学镜像。

在专用目录创建 `run-container.sh`：

```bash
#!/bin/bash
set -euo pipefail
GT_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
: "${DISPLAY:?请在本地图形桌面终端中运行}"
GT_XAUTH="$(mktemp "${XDG_RUNTIME_DIR:-/tmp}/gazeboteach-v2-xauth.XXXXXX")"
trap 'rm -f -- "$GT_XAUTH"' EXIT
chmod 600 "$GT_XAUTH"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$GT_XAUTH" nmerge -
test -s "$GT_XAUTH" || { echo '没有取得当前显示的 Xauthority cookie'; exit 2; }
GT_USER_ARGS=()
if [ -n "${GT_CONTAINER_USER:-}" ]; then
  GT_USER_ARGS=(--user "$GT_CONTAINER_USER")
fi
docker run --rm -it --name gazeboteach-v2 \
  "${GT_USER_ARGS[@]}" \
  --env DISPLAY="$DISPLAY" \
  --env XAUTHORITY=/tmp/gazeboteach.xauth \
  --env QT_X11_NO_MITSHM=1 --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env HOME=/home/student \
  --env GAZEBOTEACH_DATA_DIR=/home/student/learning \
  --env XDG_CACHE_HOME=/home/student/.cache \
  --mount 'type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly' \
  --mount "type=bind,src=$GT_XAUTH,dst=/tmp/gazeboteach.xauth,readonly" \
  --mount "type=bind,src=$GT_ROOT/release,dst=/release,readonly" \
  --mount "type=bind,src=$GT_ROOT/home,dst=/home/student" \
  --mount "type=bind,src=$GT_ROOT/data,dst=/home/student/learning" \
  --mount "type=bind,src=$GT_ROOT/cache,dst=/home/student/.cache" \
  gazeboteach:noetic-v2 bash
```

```bash
chmod +x run-container.sh
./run-container.sh
```

cookie 为空时，检查宿主 `XAUTHORITY` 指向当前桌面的实际授权文件，并检查 `xauth nlist "$DISPLAY"`。Wayland 没有可用 Xwayland / DISPLAY 时此示例不适用；先按桌面发行版文档准备显示。不要通过全局开放 X server 排错。目录路径中若含逗号，Docker `--mount` 还需要专门转义，本例建议使用上面不含逗号的专用路径。

## 5. 容器内安装和逐项验证

```bash
cd /release
sha256sum --ignore-missing -c SHA256SUMS
bash GazeboTeach-2.0.0-ubuntu20.04-x86_64.sh --check
bash GazeboTeach-2.0.0-ubuntu20.04-x86_64.sh
source /opt/ros/noetic/setup.bash
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --diagnose
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --list
xdpyinfo >/dev/null
glxinfo -B
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --gui-check
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --smoke 01 --gui --terminals
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
```

逐项检查 PyQt、Gazebo、本课终端、物理读回，再分别测试相机、深度、GPU 激光、RViz 与29/30插件。软件渲染适合先检查显示路径，性能与渲染能力由实际结果决定，不能保证 GPU 激光可用。L091 / L092 默认填空保留；参考答案验收应在独立临时副本中进行。

退出前结束课程和应用，再从容器 shell `exit`。`--rm` 只删除容器，专用 home/data/cache 目录保留。再次运行宿主脚本即可继续；不要同时启动共享同一个个人数据目录的两个课程实例。不要用 `docker system prune` 清理其他项目资源。

## 6. Rootless 与硬件加速的额外条件

Rootless 的容器 UID 0 通常映射为启动 daemon 的普通宿主用户，其他容器 UID 映射到 subordinate UID；与 rootful 的“同号 UID”策略不同。先确认 `docker info` 的 Security Options 包含 rootless，再只读检查映射与目录权限：

```bash
docker run --rm --user 0:0 gazeboteach:noetic-v2 bash -c 'id; cat /proc/self/uid_map; cat /proc/self/gid_map'
```

**仅对已经确认的 Rootless context**，可用 `GT_CONTAINER_USER=0:0 ./run-container.sh`，使容器内进程通过该 user namespace 映射为普通宿主用户，读取600权限cookie并写专用宿主目录。这个覆盖值不得套用到 rootful daemon；有额外 userns-remap 时先按实际映射准备 Docker 管理的持久卷与最小目录权限。不要用 `chmod 777`、放宽cookie权限或关闭访问控制补偿 UID 错配。映射机制见 [Docker 用户命名空间说明](https://docs.docker.com/engine/security/userns-remap/)。

需要硬件渲染时，先验证软件显示路径，再按显卡实际情况自行配置：

- Intel / AMD Mesa：宿主确有 `/dev/dri` 时，仅传入需要的 DRM render 设备，并给容器用户实际所需 render / video 组 GID，例如基于 `stat` 检查的数值使用 `--device` 和 `--group-add`；移除 `LIBGL_ALWAYS_SOFTWARE=1` 后核对 `glxinfo -B` 的 renderer。Rootless 设备访问还受宿主用户权限与 namespace 限制，映射成功不保证可用。
- NVIDIA：宿主已具备兼容驱动，并由维护者按 [NVIDIA Container Toolkit 官方指南](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) 配置适合当前 Docker 模式的运行时。确认 GPU 测试通过，再根据实际需求加入 `--gpus all`、`NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute` 并移除软件渲染变量。安装器不会替换驱动或配置 daemon。

不要把设备权限扩大、驱动安装或 daemon 配置当成自动安装教学应用的一部分。最终记录镜像digest、容器内实际SDK / 插件版本、显示、renderer、设备映射和逐课结果；成功启动一个容器不构成全课程验证。
