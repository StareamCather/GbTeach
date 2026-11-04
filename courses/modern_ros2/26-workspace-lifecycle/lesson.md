# M26 · ROS 2工作区与受管理节点

一个节点可以已经出现在ROS图里，但还没准备好处理传感器数据。本课从可安装的Python包启动生命周期节点，再用状态服务和实际消息检查它何时处理数据、何时停止以及何时释放资源。所有课程构建产物保存在个人练习的工作区。

Fortress/Humble 在当前 ROS 基础环境构建并加载本课 overlay。已安装的 launch 根据教学包选择时钟和 IMU 的 Gazebo 消息类型。验收通过标准 `GetParameters` 服务核对节点加载的位置，之后检查生命周期转换和实际 IMU 数据。[Humble 生命周期服务](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/lifecycle/node.py)

## 从源码到可发现的包

`ros_ws/src/gazeboteach_lifecycle`是一个`ament_python`包。ament定义包的构建与安装约定，colcon查找工作区的包并按依赖组织构建。

| 文件或目录 | 在本课中的作用 |
|---|---|
| `package.xml` | 声明包名、依赖和构建类型 |
| `setup.py` | 安装模块、launch、资源索引标记与命令入口 |
| `setup.cfg` | 将脚本放进ROS 2按包寻找的安装位置 |
| `resource/gazeboteach_lifecycle` | 供ament资源索引发现包的标记，文件可以为空 |
| `gazeboteach_lifecycle/sensor.py` | 生命周期节点的具体实现 |
| `launch/lab.launch.py` | 组织节点、桥接和时钟参数 |

构建后`build`保存中间文件，`install`保存可运行的包，`log`保存构建日志。ROS 2基础环境是underlay，当前课程安装是overlay。加载overlay后，工具会在环境中的安装前缀里查包名、入口和已安装的launch。系统和个人工作区若有同名包，先用`ros2 pkg prefix`核对实际解析位置，再检查执行文件。本课按顺序构建包并安装文件。修改个人包源码、入口、launch或依赖后，重新构建并重启节点，确保当前进程加载新的安装结果。[Jazzy colcon工作区教程](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.rst) · [ament资源索引](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md)

平台按`build.sh → start.sh → ros2 launch`运行。构建脚本在个人工作区安装本课包，启动脚本在当前 ROS 2 基础环境上加载本课`local_setup.bash`，然后用包名找到已安装的launch。操作终端可执行下面的检查。

```bash
. ros_ws/install/local_setup.bash
ros2 pkg prefix gazeboteach_lifecycle
ros2 pkg executables gazeboteach_lifecycle
ros2 lifecycle get /sensor_lifecycle
python3 lab.py
```

`pkg prefix`应指向本课个人工作区。验收通过节点参数核对其实际加载的模块路径。报告`installed_package`保存安装前缀、模块路径和SHA256，模块应在安装空间中并与个人源码内容一致。若要看构建问题，先打开`ros_ws/log`最近一次build的日志，再按错误检查依赖、入口或安装路径。启动后的节点与桥接输出可从课程运行日志读取。

## 生命周期转换与资源

| 主状态 | 含义 |
|---|---|
| unconfigured | 节点已存在，业务资源等待配置 |
| inactive | 资源已经配置，数据输出仍受门控 |
| active | 按已配置的策略处理新数据并输出 |
| finalized | shutdown后进入终态，等待进程或对象被销毁 |

`configure`从unconfigured进入inactive，`activate`再进入active。`deactivate`从active回到inactive并停止受管理输出，保留可用于再次激活的配置。`cleanup`从inactive回到unconfigured，销毁本次配置的业务资源。各转换调用对应回调，回调返回成功或失败。进程还在、转换服务可用与业务输出有效，是三层不同的可观察事实。[ROS 2受管理节点设计](https://design.ros2.org/articles/node_lifecycle.html)

rclpy的LifecyclePublisher只在启用时发布。普通ROS订阅、定时器和自定义资源是否运行，还要看应用怎样在回调中管理它们。业务回调可明确检查当前活动状态，必要时在停用时移除或重置相应资源。验收时分别看状态、输出与实体资源，避免从状态名猜程序行为。[Jazzy LifecyclePublisher源码](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/lifecycle/publisher.py)

## 用数据观察每次转换

Gazebo中的转子由原生关节速度控制维持约0.55rad/s。转子上的IMU配置为25Hz，通过GZ_TO_ROS桥接将真实惯性样本发到`/gt/lifecycle/raw_imu`。生命周期节点在active时把同一个`sensor_msgs/Imu`样本转发到`/gt/lifecycle/sample`，保留原始时间戳和数据。输入源在生命周期切换时继续运行，可以用原始IMU推进证明业务输出的启停来自门控。

`lab.py`通过GetState和ChangeState做实际请求。先确认unconfigured时业务订阅和输出发布端尚未创建。`configure`建立输入订阅及尚未启用的LifecyclePublisher。`activate`后用新时间戳的实际样本核对转发。`deactivate`后资源还在，原始IMU继续推进，转发输出保持安静。`cleanup`销毁两个业务端点。脚本验证未配置时直接activate，以及active时直接cleanup都会被拒绝。随后执行两次完整配置到清理，检查状态能重复使用。最后留下unconfigured，便于继续练习。

每段先留0.16s仿真时间过渡，再收至少8条原始IMU，时间跨度至少0.36s。验收观察陀螺Z轴中位数在0.40至0.70rad/s，并用姿态偏航变化确认转子持续运动。active窗口至少有6条转发与同时间戳原始样本的CDR字节相同。其它窗口多等0.12s仿真时间，确认无新转发。报告中的`cycles`按配置、激活、停用、清理四阶段记录输入输出数和时间，`transitions`保留转换编号、是否接受与读回状态。也可用`gz topic -e -t /gt/lifecycle/raw_imu -n 1`直接查看Gazebo端原生IMU。

仿真时间由Gazebo推进，节点的ROS时钟跟随桥接`/clock`。输入的IMU采样时间独立保留，等待时限使用单调墙钟。暂停时先区分是仿真停止产生样本，还是生命周期门控停止输出。可用`ros2 lifecycle list /sensor_lifecycle`查看当前允许转换，再用图信息和实际接收作验证。

## 排查与独立挑战

- 包找不到时检查`ros2 pkg prefix`，然后核对underlay、overlay与ament索引标记。
- 包能找到但入口或launch缺失时，检查`setup.py`的数据安装、入口和`setup.cfg`，重建后从安装路径确认结果。
- 节点已发现但无输出时，分别检查生命周期状态、实际输入、桥接方向和QoS。已配置且inactive时，输出门控符合预期。
- cleanup后仍见业务端点时，先确认图中端点属于当前进程，再检查是否销毁订阅、发布者或定时器。

独立挑战是在configure阶段读取一份本地校准文件。先定义文件有效时建立资源的路径，再定义缺文件或格式错时的失败回调。用同一观察脚本检查失败后状态、业务端点以及成功重新配置后的输出。

本课先通过 GetAvailableTransitions 读取当前允许的转换。合法转换用 ID；验证不可用的 activate 和 cleanup 时用标签请求，核对 success=false 且原状态未变。此次 Jazzy 的 rclpy 对未注册数字 ID 会抛出异常并可能终止节点，所以实验不发送这种请求。[Jazzy 生命周期服务实现](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/lifecycle/node.py)
