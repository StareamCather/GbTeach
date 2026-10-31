# GazeboTeach V1 安装与运行指南



V1 提供中文原生桌面工作台，以及 **L01–L24 和 L09 两个倒立摆选做实验（091、092），共 26 个项目**。L25–L37（含两个综合项目）的后续工作尚未交付。请以本版本课程地图和发布说明为准。

发行页：[GazeboTeach v1.0.0](https://github.com/StareamCather/GazeboTeach/releases/tag/v1.0.0)。

## 1. 先确认运行环境

本次真实验证平台为 **Ubuntu 20.04、x86_64、系统 Python 3.8、PyQt5 / Qt 5.12、Gazebo Classic 11.15.1、ROS 1 Noetic**，设备为 ikaros。其他 Classic 11 补丁版本需在目标设备重新验证。

应用、中文教材、练习模板、参考材料和场景资源均包含在安装包中。安装包**不包含 Ubuntu、Gazebo、ROS、显卡驱动、系统 Python / PyQt5 或完整系统依赖**，也不执行 apt 或 sudo。部分实验首次启动需使用本机 Gazebo SDK 编译随包提供的程序；模型和教材无需在线下载。

| 设备情况 | V1 的行为与建议 |
| --- | --- |
| Classic 11，且具备课程声明的依赖 | 可以进入对应实验；仍需检查真实图形、插件和设备行为 |
| Classic 11，没有 Noetic | 可以阅读全部教材；独立 Gazebo 课程按各自依赖开放，ROS 课程无法运行 |
| Ubuntu 22.04 / ROS 2 / 现代 Gazebo | 不能把 ROS 2 或现代 Gazebo 当作 Classic 11 / Noetic；阅读教材后，按本文 Docker 方案自行准备兼容环境 |
| 无图形会话 | 可做命令行诊断和适用的无 GUI 验收，桌面、Gazebo GUI、RViz 和视觉检查需要图形会话 |
| ARM64 或其他操作系统 | 尚未进行本版本实验验证；文件名中的 x86_64 表示已验证平台，不表示提供了通用二进制兼容层 |

启动时，平台检测实际软件版本、已安装和已激活的 ROS、命令、插件及课程依赖，并弹出逐课结果。检测结果区分可满足的已声明条件、版本不匹配、缺依赖和检测未完成；**静态检测通过不替代相机、GPU 激光或 RViz 的实际图形验收**。遇到不兼容实验时仍可阅读教材。

Gazebo Classic 已结束上游维护，官方建议新项目迁移现代 Gazebo。本 V1 保留 Classic / Noetic 是为了交付已验证的教学包，后续课程包按环境扩展。[Gazebo Classic 官方说明](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

## 2. 原生 Ubuntu 20.04：准备依赖

以下命令仅适用于 **已经配置好 Ubuntu 20.04 和 ROS Noetic 官方软件源的兼容设备**。不要将 Noetic 的 Focal 软件源加入 Ubuntu 22.04 / 24.04 宿主机。ROS 初次安装可参考 [Noetic 官方安装资料](https://wiki.ros.org/noetic/Installation/Ubuntu)；若旧软件源失效，先检查官方维护状态，不要绕过签名校验。

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-pyqt5 python3-numpy python3-dev \
  build-essential cmake pkg-config libgazebo11-dev \
  ros-noetic-desktop-full ros-noetic-gazebo-ros-pkgs \
  ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz \
  xterm x11-utils gedit fonts-noto-cjk
```

在实际 Ubuntu 图形桌面打开终端检查：

```bash
/usr/bin/python3 -c 'from PyQt5 import QtCore, QtWidgets; import numpy; print(QtCore.QT_VERSION_STR)'
gazebo --version
test -f /opt/ros/noetic/setup.bash
source /opt/ros/noetic/setup.bash
rosversion -d
pkg-config --modversion gazebo
printf 'DISPLAY=%s\n' "$DISPLAY"
```

普通桌面启动继承当前图形会话。不要照搬开发设备的 DISPLAY 编号。平台会为课程加载 Noetic 环境；不需要在宿主机全局修改默认 ROS 版本。

## 3. 下载、校验、安装

从发行页下载以下文件，放在同一目录：

- `GazeboTeach-1.0.0-ubuntu20.04-x86_64.sh`：推荐的自解包安装器。
- `SHA256SUMS`：发布文件校验值。
- `release-manifest.json`：来源提交、课程列表及逐文件摘要。
- `V1_INSTALL.md`：本指南的独立副本。

若还下载了 `GazeboTeach-1.0.0-ubuntu20.04-x86_64.tar.gz`，它是同一应用载荷的归档形式。自解包安装器已包含载荷，无需同时下载归档。

```bash
cd /你的下载目录
sha256sum --ignore-missing -c SHA256SUMS
bash GazeboTeach-1.0.0-ubuntu20.04-x86_64.sh --check
bash GazeboTeach-1.0.0-ubuntu20.04-x86_64.sh
```

确认需要的 `.sh` 文件在校验输出中明确显示 `OK`。`--ignore-missing` 允许只下载所需资产，不能把缺少目标文件视为已校验。安装器随后再次校验内嵌载荷、文件路径、清单和每个文件的 SHA256，校验成功后才安装。

目标设备下载困难时，可先在 Windows 下载上述资产，再通过已配置并核对主机身份的 SCP / SFTP 连接传入 Linux。在 Linux 接收端重新执行同样的 SHA256 检查即可；不要把 Windows 的 GitHub 私钥一并复制到教学设备。

默认安装结构：

```text
~/.local/opt/GazeboTeach/
├── bin/gazeboteach                 # 日常入口
├── current -> versions/1.0.0       # 当前应用版本
└── versions/1.0.0/                 # 应用与课程模板
```

也可指定独立目录：

```bash
bash GazeboTeach-1.0.0-ubuntu20.04-x86_64.sh \
  --prefix "$HOME/Applications/GazeboTeach" --desktop
```

`--desktop` 仅生成 `PREFIX/share/applications/gazeboteach.desktop`。若需要桌面菜单入口，确认其 Exec 路径正确后自行注册：

```bash
mkdir -p "$HOME/.local/share/applications"
cp "$HOME/Applications/GazeboTeach/share/applications/gazeboteach.desktop" \
  "$HOME/.local/share/applications/gazeboteach.desktop"
```

同版本、同内容重复安装会复用已验证文件；安装器拒绝覆盖改过的版本目录或无关目录。练习请在平台准备的个人副本中修改，不要修改安装目录内的模板。

## 4. 启动和第一轮检查

默认目录下运行：

```bash
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --diagnose
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --list
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
```

自定义安装时，把上述路径替换为实际 `PREFIX/bin/gazeboteach`。也可进入 `PREFIX/current` 后运行 `./launch.sh`。

第一次进入先阅读环境结果窗口。检查不通过的课程与原因；不要将“未检测完整”理解成已安装或可运行。关闭结果窗口后，从 L01 开始：

1. 阅读目标、背景、文件结构与预期现象。
2. 点击“开始实验”，观察初始化步骤，等待 Gazebo 和本课终端出现。
3. 按教材进行预测、观察和单变量修改，使用本课验收记录真实结果。
4. 结束实验后再切换课程；平台只管理它为本课创建的进程。

可选的独立安装自检：

```bash
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --gui-check
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --smoke 01 --gui --terminals
```

`--smoke` 默认建立新的临时练习副本，执行本课真实验收并清理进程，输出日志位置。它不会使用你已有练习替代新版模板。部分 GUI 编辑和视觉目标仍需按教材人工观察。

L091 / L092 是近直立倒立摆控制填空作业。默认 TODO 供学员填写，参考演示与学生控制器验收分开；没有填写关键代码时，不能把学生控制器验收失败当作安装损坏。

## 5. 个人数据、更新与卸载

个人练习、学习记录、构建输出及实验日志位于 `~/.local/share/GazeboTeach`，与应用版本目录分开。可通过 `GAZEBOTEACH_DATA_DIR` 或 `--data-dir` 指定独立目录；路径应位于安装版本目录外。

```bash
GAZEBOTEACH_DATA_DIR="$HOME/GazeboTeachLearning" \
  "$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
```

倒立摆共用编译缓存另位于 `${XDG_CACHE_HOME:-~/.cache}/gazeboteach`。备份时优先保存个人数据目录；缓存可重建。模板升级会保留个人练习，恢复模板操作会影响对应个人副本，请先自行备份需要保留的修改。

安装新版本前先结束实验和应用，校验新安装包，再以同一个 `--prefix` 安装。不同版本保存在 `versions/`，入口指向新版本。清单中的 `source_commit` 可用于和 GitHub 标签核对来源。

使用 **与已安装版本完全相同** 的安装器卸载：

```bash
bash GazeboTeach-1.0.0-ubuntu20.04-x86_64.sh --uninstall
```

若使用自定义目录，追加相同的 `--prefix`。卸载只移除这一份未经修改的版本及由它管理的入口，保留个人数据和其他版本；手动复制到桌面菜单的 `.desktop` 文件需自行移除。安装器因模板被修改而拒绝卸载时，先备份并检查修改，不要对个人数据目录执行递归删除。

## 6. Ubuntu 22.04 / ROS 2 / 现代 Gazebo：手动 Docker 环境

本节提供一个由用户手动构建、手动启动的兼容环境方案。容器内使用 Focal / Noetic / Classic 11，宿主机可保留原有 ROS 2 和现代 Gazebo。OSRF 的 Noetic desktop-full 镜像定义属于 Focal 系列，包含 Noetic 桌面完整安装。[OSRF 官方 Dockerfile](https://github.com/osrf/docker_images/blob/master/ros/noetic/ubuntu/focal/desktop-full/Dockerfile)

**本次 V1 已在 ikaros 原生环境验证；本节 Docker 构建、GPU 和显示转发方案尚未进行端到端实测。** 镜像标签、旧软件源可用性、宿主显卡驱动和 X11 / Xwayland 配置会影响结果。用户应完成下列检查后再学习，不能据此认定所有容器实验已通过验证。

V1 平台预留了执行后端和 Docker 配置接口，能描述镜像、工作目录、显示、上下文及挂载配置；**V1 不自动连接 Docker daemon、拉取镜像或启动容器课程**。这里的 `docker run` 是独立手动流程。平台在容器内运行后，检测到的是容器内的实际环境。

### 6.1 宿主前提

以下示例面向 **本地 Linux x86_64 图形桌面、Docker Engine、本地 X11 或带 Xwayland 的 Wayland 会话**，不覆盖 Windows Docker Desktop、WSL、远程 Docker daemon 或 SSH X11 转发。Docker 的 bind mount 访问 daemon 所在主机的路径，远程 daemon 不能直接使用本机这些目录。[Docker 挂载说明](https://docs.docker.com/engine/storage/bind-mounts/)

按 [Docker 官方 Ubuntu 安装指南](https://docs.docker.com/engine/install/ubuntu/) 配置 Docker，随后确认当前用户具备运行权限：

```bash
docker version
docker info
command -v xauth
test -n "$DISPLAY"
test -d /tmp/.X11-unix
```

宿主缺少 xauth 时，可安装宿主发行版提供的 `xauth`。Wayland 下没有可用的 DISPLAY / Xwayland 时，此示例不能显示 GUI；先按桌面发行版文档配置兼容显示会话。

### 6.2 建立持久目录和教学镜像

使用普通桌面用户，在宿主机运行：

```bash
mkdir -p "$HOME/GazeboTeachDocker"/{release,home,data,cache}
cd "$HOME/GazeboTeachDocker"
```

把校验成功的 V1 `.sh` 安装器放入 `release/`。`home/` 保存容器内应用与用户配置，`data/` 保存练习与记录，`cache/` 保存可重建缓存；这些宿主目录在 `--rm` 删除容器后仍然保留。

创建 `Dockerfile`：

```dockerfile
FROM osrf/ros:noetic-desktop-full

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pyqt5 python3-numpy python3-dev \
    build-essential cmake pkg-config libgazebo11-dev \
    ros-noetic-gazebo-ros-pkgs ros-noetic-robot-state-publisher \
    ros-noetic-xacro ros-noetic-rviz \
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

构建并检查，不使用宿主 Noetic 环境：

```bash
docker pull osrf/ros:noetic-desktop-full
docker image inspect osrf/ros:noetic-desktop-full --format '{{json .RepoDigests}}'
docker build --build-arg LOCAL_UID="$(id -u)" --build-arg LOCAL_GID="$(id -g)" \
  -t gazeboteach:noetic-v1 .
docker run --rm gazeboteach:noetic-v1 bash -lc \
  'source /opt/ros/noetic/setup.bash; gazebo --version; rosversion -d; pkg-config --modversion gazebo; /usr/bin/python3 -c "from PyQt5 import QtWidgets; import numpy"'
```

把实际 `RepoDigests` 记录到自己的部署笔记；需要可重复构建时，将 Dockerfile 的 FROM 固定为实际拉取的 `osrf/ros@sha256:...`。不要照抄不存在的摘要。若 UID / GID 已被基础镜像占用，构建会明确失败；检查镜像已有用户后调整用户创建步骤，使容器用户和上述宿主持久目录具有相同的实际 UID / GID，不要改用 root 运行日常实验。

下载或 apt 失败时先检查网络、官方镜像与软件源状态。可以在联网机器拉取并 `docker save`，再传输到目标机 `docker load`；这仍需核对来源和校验值。不要通过禁用证书或 apt 签名来跳过错误。

### 6.3 有限授权 X11，并保存一个启动脚本

在同一宿主目录创建 `run-container.sh`：

```bash
#!/bin/bash
set -euo pipefail
GT_ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
: "${DISPLAY:?请在本地图形桌面设置的 DISPLAY 中运行}"
GT_XAUTH="$(mktemp "${XDG_RUNTIME_DIR:-/tmp}/gazeboteach-xauth.XXXXXX")"
trap 'rm -f -- "$GT_XAUTH"' EXIT
chmod 600 "$GT_XAUTH"
xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$GT_XAUTH" nmerge -
test -s "$GT_XAUTH" || { echo '未取得当前显示的 Xauthority cookie'; exit 2; }

docker run --rm -it --name gazeboteach-v1 \
  --env DISPLAY="$DISPLAY" \
  --env XAUTHORITY=/tmp/gazeboteach.xauth \
  --env QT_X11_NO_MITSHM=1 \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env GAZEBOTEACH_DATA_DIR=/home/student/learning \
  --env XDG_CACHE_HOME=/home/student/.cache \
  --mount "type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly" \
  --mount "type=bind,src=$GT_XAUTH,dst=/tmp/gazeboteach.xauth,readonly" \
  --mount "type=bind,src=$GT_ROOT/release,dst=/release,readonly" \
  --mount "type=bind,src=$GT_ROOT/home,dst=/home/student" \
  --mount "type=bind,src=$GT_ROOT/data,dst=/home/student/learning" \
  --mount "type=bind,src=$GT_ROOT/cache,dst=/home/student/.cache" \
  gazeboteach:noetic-v1 bash
```

运行：

```bash
chmod +x run-container.sh
./run-container.sh
```

该脚本只导出当前 DISPLAY 对应的 cookie 到临时文件，以只读方式交给容器，并在退出后删除文件；不使用 `xhost +`，不开放全部 X server 客户端。容器未使用 `--privileged`、宿主网络、端口发布或 Docker socket 挂载。它的 ROS / Gazebo 通信留在容器网络内。参数含义见 [Docker run 官方说明](https://docs.docker.com/engine/containers/run/)。

若显示授权失败，检查宿主 `XAUTHORITY` 是否指向当前桌面使用的实际授权文件，以及 `xauth nlist "$DISPLAY"` 是否有记录。不要改成关闭访问控制。cookie 文件为临时凭证，不应提交仓库或分享。

### 6.4 在容器内安装和验证

进入容器终端后执行：

```bash
bash /release/GazeboTeach-1.0.0-ubuntu20.04-x86_64.sh --check
bash /release/GazeboTeach-1.0.0-ubuntu20.04-x86_64.sh
source /opt/ros/noetic/setup.bash
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --diagnose
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --list
xdpyinfo >/dev/null
glxinfo -B
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --gui-check
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach" --smoke 01 --gui --terminals
"$HOME/.local/opt/GazeboTeach/bin/gazeboteach"
```

逐项确认 PyQt 窗口、Gazebo 窗口和本课两个终端均正常；继续相机、深度、GPU 激光及 RViz 前分别观察真实结果。软件渲染适合先排除显示连接问题，但性能和渲染能力受宿主限制，不能据此保证 GPU 相关课程可用。

退出时先结束课程和应用，再在容器终端 `exit`。次日重新运行宿主的 `./run-container.sh`，容器内直接运行已安装入口即可，应用和个人数据仍在持久目录。不要同时启动两个使用同一数据目录的教学容器。

### 6.5 可选硬件加速

先保证上述软件渲染路径能打开窗口，再独立配置硬件加速：

- Intel / AMD Mesa：宿主确有 `/dev/dri` 时，可按本机权限向 `docker run` 添加所需 DRM 设备和对应 render / video 组 GID，并移除 `LIBGL_ALWAYS_SOFTWARE=1`。应以容器内 `glxinfo -B` 的实际 renderer 为准。
- NVIDIA：按 [NVIDIA Container Toolkit 官方指南](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) 自行配置兼容的容器运行时；在成功的 GPU 测试基础上，使用 `--gpus all` 和适用的 `NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute`，移除软件渲染变量，再验证 Gazebo。V1 安装器不更换宿主驱动或配置该运行时。

硬件型号和驱动决定可用组合；这里没有宣称通过某个选项即可在所有设备运行 GPU 激光。若仍不兼容，保留诊断、renderer 和课程日志，以实际设备证据判断。

## 7. 常见问题与日志

| 现象 | 先检查什么 |
| --- | --- |
| 仅有 ROS 2 / `gz sim`，课程按钮不可用 | V1 教材目标是 Classic 11 / Noetic；准备兼容环境或手动容器，不用命令名称代替版本 |
| Noetic 已装但弹窗显示 ROS 未激活 | “已安装”和“当前已激活”分开显示；课程运行器会加载其需要的环境，继续检查逐课依赖 |
| PyQt 无法导入 | 使用系统 `/usr/bin/python3`；不要用不包含系统 PyQt5 的虚拟环境替代启动器 |
| Gazebo / RViz 黑屏或 Qt 无法连接显示 | 实际 DISPLAY、Xauthority、图形会话、OpenGL renderer；在容器还要检查挂载和 cookie |
| 首次课程编译失败 | Gazebo 开发头文件、`pkg-config gazebo`、g++、Python 开发包；查看初始化输出中的实际命令 |
| 相机或 GPU 激光无数据 | 课程原生插件和真实渲染条件；先看服务端 / GUI 日志，不能只凭界面打开认定通过 |
| 检测超时或未知版本 | 重做检测并查看诊断输出；未知结果不会自动当作就绪 |
| 开不了终端 / 编辑器 | 系统 xterm 或 gnome-terminal、编辑器以及图形会话是否可用 |
| 修改模板后没有变化 | 修改平台打开的个人练习，按教材重新加载；已启动进程不会自动读取所有磁盘修改 |

平台在初始化和实验状态中显示实际命令、错误与日志路径。反馈问题时保留：版本清单的 source_commit、`--diagnose` 输出、课程编号、实际操作和对应实验日志。实验端口与进程是按课隔离的；不要为排错使用全局 `killall gazebo` 或杀死其他项目的 ROS Master。

更多课程验证与边界见 [验证记录](VALIDATION_LOG.md)，本版本范围见 [V1 发布说明](RELEASE_NOTES_V1.md)。
