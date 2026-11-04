# M25 · 从URDF关节到实际机器人TF

机械臂在Gazebo里转过一个角度，ROS 2中的TF怎样知道它转了多少？本课把同一份机器人描述交给Harmonic和robot_state_publisher，再用真实关节反馈核对TF。建议先完成M24的变换方向与时间查询。

Fortress/Humble 从同一个个人 `robot.urdf` 读取运动学描述，并通过 `ign sdf -p` 生成 SDF。生成文件的系统插件名转换为 Fortress 的名称，连杆与关节保持原有校验。Fortress 的 PosePublisher 把位姿发布到 `/model/urdf_arm/pose`，可用 `ign topic -e -t /model/urdf_arm/pose -n 1` 观察。检查程序通过标准 `GetParameters` 服务核对 RSP 的原始 URDF，再以实测关节时刻查询 TF。[Fortress PosePublisher 源码](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/pose_publisher/PosePublisher.cc)

## 一个URDF，两条消费路径

URDF描述机器人树。link保存连杆几何、惯性与碰撞。joint指定父子连杆、安装位姿、运动轴、类型和边界。转动关节的`origin`先定位关节坐标系，`axis`在该坐标系中给出转轴。改变几何尺寸后也应核对质量、质心和惯性。在相同质量、细杆近似下，长度增加会按平方增大垂直于杆轴的转动惯量。

Gazebo接收URDF后，由libsdformat转换成SDF，创建物理实体与系统。SDF还可以描述世界、传感器和Gazebo插件。URDF的`<gazebo>`扩展为转换提供插件配置。转换有时会将固定连杆合并到父连杆，这会改变物理实体名称和数量。机器人在ROS中的运动学树仍由输入的URDF描述。排查名称或插件时，可以用`gz sdf -p robot.urdf`查看实际转换结果。[Harmonic生成URDF实体](https://gazebosim.org/docs/harmonic/spawn_urdf/) · [SDFormat转换与固定关节合并](https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/)

`robot_state_publisher`从`robot_description`读取同一份URDF，然后将`/joint_states`中的实测关节值代入运动学。固定关节发布到`/tf_static`，可运动关节按对应关节反馈更新`/tf`。位置控制则由Gazebo原生JointPositionController执行。理解两条路径有助于定位模型能动、TF不动或方向异常的故障。[Robot State Publisher官方说明](https://github.com/ros/robot_state_publisher/blob/ros2/README.md) · [Harmonic关节位置控制器](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html)

## 从指令到关节和TF

1. `spawn.py`向当前世界的create服务发送EntityFactory请求。脚本先用`gz sdf -p robot.urdf`转换并核对结果，保存为`build/robot.converted.sdf`，核对`anchor`与`tool_mount`仍为fixed，`shoulder`仍为revolute，再以明确的`urdf_arm`模型名请求创建。
2. `robot.launch.py`组织描述参数、`m25_state_publisher`和数据链所需的桥接端。launch将URDF作为字符串参数传入，`use_sim_time`默认true。桥接配置分别指明关节状态、位置命令和时钟的方向。
3. ROS 2在`/gt/shoulder`发Float64位置目标，桥接将其转为Gazebo Double。关节位置系统根据当前角度控制关节。
4. JointStatePublisher读取实际关节，Gazebo侧`/gt/joints`的Model消息经桥接成为ROS 2 `/joint_states`。按`name`找到`shoulder`，再读取同索引的`position`和`velocity`。转动关节用rad与rad/s。
5. robot_state_publisher使用实测关节角和URDF的固定安装关系生成TF。按反馈所对应的仿真时刻查询，再核对Gazebo原生PosePublisher在`/gt/arm_link_poses`提供的模型相对连杆位姿。[Jazzy JointState消息定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/JointState.msg)

区分世界中的模型名、URDF的link名、joint名和ROS话题。创建时传入的模型名用于在Gazebo中识别实体，TF frame来自连杆命名和发布配置。若加入命名空间或frame前缀，还需检查发布和查询双方都使用对应名字。

## 用正运动学检查frame

机器人链为`world → base → arm → tool`。world到base是固定锚定，肩关节从base到arm，安装高度0.70m，转轴为Z。arm到tool固定偏移为(0.82,0.10,0.12)m，偏航0.25rad。本课使用`preserveFixedJoint`保留工具固定关节的物理实体，因此ROS TF之外还能读取实际工具连杆位姿。

设肩关节的实测角为q，则base下的工具位置和偏航为

```text
x = 0.82 cos(q) - 0.10 sin(q)
y = 0.82 sin(q) + 0.10 cos(q)
z = 0.70 + 0.12
yaw = q + 0.25
```

例如q=0.65rad时，工具位置约为(0.592,0.576,0.82)m，偏航0.90rad。q=−0.45rad时，位置约为(0.782,−0.267,0.82)m，偏航−0.20rad。计算用实际角度替换示例值。转轴在关节坐标系中定义，关节原点则从父连杆定位到关节。检查每段变换的方向，并用同一时刻的姿态组合。

查询base、arm返回从arm到base的变换。关节角来自观测。瞬态过程中，命令和测量可能明显不同。用上述正运动学先作预测。脚本比较Gazebo原生关节角、ROS关节角、对应时刻TF和独立的原生连杆位姿。原生位姿以模型坐标给出，验收先用base的原生位姿转换为base相对量，再与TF比较。固定锚定使world到base在本课是单位变换。

## 实验操作

启动课程，在操作终端查看launch参数、真实JointState与实际验证结果。

```bash
ros2 launch ./robot.launch.py --show-args
ros2 topic echo /joint_states --once --no-daemon
gz topic -e -t /gt/arm_link_poses -n 1
python3 lab.py
```

`--show-args`只展示参数。课程启动流程已经运行了节点。对照+0.65、−0.45、0rad三阶段的实际角度、反馈时间和TF变化。先观察是否到达，再看关节和TF是否在同一个时刻解释了同一个姿态。报告`robot_description_sha256`来自与个人URDF逐字核对的描述。`stages`保留原始关节时间戳、原生和ROS的角度与速度、原生和TF的arm及tool位姿。默认要求三个递增时间样本同时满足角误差≤0.035rad、速度绝对值≤0.06rad/s，再核对TF与原生连杆位置差≤0.012m、偏航差≤0.015rad。回中后继续保持0.25s仿真时间，验证速度绝对值≤0.03rad/s。

若创建失败，先检查URDF、文件路径和模型名是否已占用。若Gazebo能动但ROS未收到状态，按顺序检查原生关节发布、桥方向、消息类型和ROS QoS。若收到关节角却找不到TF，核对`robot_description`中的关节名、父子连杆名、时间戳和缓存范围。变换方向异常时先检查关节`origin`和`axis`。修改URDF后用桌面“重启实验”，让物理模型和发布器同时重读描述。[Jazzy Python launch教程](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Intermediate/Launch/Launch-Main.rst)

## 单变量练习与挑战

先保持肩关节控制和基座配置，单独改变工具安装偏移，再用两组实测肩角算出新工具点位置。检查固定工具偏移是否在`/tf_static`正确出现，以及随动态肩关节转动后的组合位置。记录个人URDF、转换后的SDF和测量结果。

进一步可添加一个肘关节，并给两个关节独立控制入口。先预测两组角度下的末端，再用实际关节角串联两段变换。对照TF和Gazebo中的末端位姿，解释未到位时的差异。
