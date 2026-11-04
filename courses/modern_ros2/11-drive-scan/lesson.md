# M11 · ROS 2差速车、原生轮速与GPU激光

一条ROS 2速度指令发出后，怎样确认它真的让轮子转动、小车前进，且墙距与位移相符？本课在当前 ROS 2 教学包中连接一辆差速车与三个ROS 2话题。先直行接近墙面，再原地左转，用原生状态和传感器数据检验整条链路。

Fortress/Humble 加载 `world.fortress.sdf` 和 `bridge.fortress.yaml`。桥保持同一组话题、方向及 ROS 类型，Gazebo 消息类型写成 `ignition.msgs.*`。Harmonic 的激光桥显式设置 `SENSOR_DATA`，Humble 桥使用该版本的默认发布 QoS。验收以 BEST_EFFORT 订阅观测激光，能匹配两种可靠性。Fortress 的原生话题观察使用 `ign topic`。[Humble 桥配置](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge#example-5-configuring-the-bridge-via-yaml)

## 消息怎样到达车轮

Gazebo Sim的ECS是实体、组件、系统三者协作的结构。模型、连杆与关节是实体，关节状态与速度命令是组件，DiffDrive和Physics是系统。DiffDrive收到Gazebo Transport速度指令，查找左右轮关节，按轮距和轮半径换算关节速度命令。Physics计算轮地接触及车体运动。轮速积分形成里程计估计，SceneBroadcaster发布仿真实体位姿。两路数据可独立核对。

`bridge.yaml`指定跨中间件的类型与方向。课程启动时会创建桥接进程。

| 话题 | ROS 2类型 | Gazebo类型 | 方向 |
|---|---|---|---|
| `/gt/cmd_vel` | `geometry_msgs/msg/Twist` | `gz.msgs.Twist` | ROS_TO_GZ |
| `/gt/odom` | `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` | GZ_TO_ROS |
| `/gt/scan` | `sensor_msgs/msg/LaserScan` | `gz.msgs.LaserScan` | GZ_TO_ROS |

两个中间件即使使用相同的字符串名称，消息类型与发现范围也独立。桥负责转换与转发。车辆关节、控制参数和物理响应由SDF与系统决定。观测实际车体位姿及关节状态的测试可直接读Gazebo Transport。[Harmonic DiffDrive接口](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html) · [ros_gz桥接指南](https://gazebosim.org/docs/harmonic/ros2_integration/)

## 用轴向和几何预测轮速

小车模型X向前、Y向左、Z向上。两轮轴明确沿模型+Y表达。SDF里车轮圆柱视觉和碰撞要旋转到侧向，关节轴仍应按实际旋转轴定义。若把同一轴向分量误写在已旋转的关节局部坐标中，轮子可能绕错误方向转动。轮距b=0.52m，物理轮半径r=0.15m，DiffDrive参数取相同值。

设v为车体X方向线速度，ω为绕Z轴偏航角速度，左右轮的角速度单位为rad/s。以下是理想纯滚动的关系。

```text
omega_left  = (v - omega * b/2) / r
omega_right = (v + omega * b/2) / r
v           = r * (omega_right + omega_left) / 2
omega       = r * (omega_right - omega_left) / b
```

直行用v=0.25m/s、ω=0，两轮期望约+1.67rad/s，2.0s仿真时间对应理想位移0.50m。原地左转用v=0、ω=0.55rad/s，左轮约−0.95rad/s，右轮约+0.95rad/s。理想1.35s转过约0.74rad。实际结果还受启动与停车过渡、摩擦、侧滑和仿真步离散影响。

`Odometry`的pose用消息`header.frame_id`给出的里程计坐标表达，twist用`child_frame_id`的车体坐标表达。Gazebo原生车体位姿用仿真世界坐标。原生直行量取世界X差，里程计直行量取两点的平面距离，避免累计里程计朝向与世界坐标不同造成比较偏差。轮子打滑或标定偏差会让它们分离。[ROS 2 Odometry定义](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/Odometry.msg)

## 181束激光怎样验证接近墙面

`Sensors`系统用Ogre2计算实际GPU lidar回波。水平扫描约从−1.57到+1.57rad，共181束，配置频率10Hz。量程0.08至12m。角度零沿传感器坐标X轴，索引i的角度为`angle_min + i * angle_increment`。先看中间附近的前向束，再用有限、在量程内的样本估计前方距离。本课取中央五束的中位数来减小单束抖动的影响。

在车头近似垂直朝向墙时，前向墙距下降量应接近车体的前进量。初始墙面位于墙体中心x=4m减去半厚0.10m的位置。若车体在x≈0、雷达在车体前方约0.20m，理想初始距离约3.70m。视角偏转后，前向束会斜射墙面，因此在直行段结束、转向开始前对比距离。实际雷达值来自渲染射线，位移来自原生车体状态，两种证据分开采集。[LaserScan消息定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg) · [Harmonic传感器教程](https://gazebosim.org/docs/harmonic/sensors/)

## 观察QoS和真实控制结果

在本课终端执行以下命令。课程会话提供正确的DDS域和Gazebo Transport分区。

```bash
ros2 topic list
ros2 topic info /gt/scan -v
ros2 topic info /gt/cmd_vel -v
gz topic -l
python3 lab.py
```

QoS是服务质量配置。读取端点详情时先看发布端提供的可靠性与订阅端请求的可靠性。Harmonic 的激光桥显式采用 `SENSOR_DATA`，偏向新鲜样本并使用 BEST_EFFORT。Fortress 的桥使用默认发布 QoS，可用话题详情查看实际配置。订阅端请求RELIABLE时无法与只提供BEST_EFFORT的发布端匹配。使用BEST_EFFORT订阅可接受BEST_EFFORT或RELIABLE发布端。速度指令端点应按实际详细信息核对，不能把传感器QoS直接套到所有消息。主题能列出来只说明发现了名称，确认收到数据还要读取端点与消息。[Jazzy QoS策略与兼容表](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

脚本先等待实际Odometry、LaserScan和指令订阅端，发送零速度并将车体放回物理基准位姿。里程计仍保留累计坐标，因此读取起止变化量。脚本在每个动作阶段反复发有限速度，按实际Odometry时间戳计量仿真时长，并用墙钟超时退出失去反馈的阶段。直行后先发零速度，再读取位姿、里程计和前向墙距，然后执行原地转向。原生JointStatePublisher在`/gt/wheels`提供左右轮速度，分别核对直行、转向与停止的读数。退出路径会尝试再次发送零速度。修改SDF或桥配置后，通过桌面的“重启实验”重新加载。

报告将多条证据放在一起。默认要求原生前进0.28～0.75m，里程计与该值之差小于0.13m，墙距下降量与前进量之差小于0.10m。直行前后分别比较Gazebo和ROS中央五束的中位数，允许差小于0.03m。转向原生偏航变化要求0.35～1.05rad，原生与里程计的偏航变化差小于0.25rad，平移小于0.20m，左轮为负且右轮为正。零速度后检查物理漂移小于0.03m和0.04rad，轮速绝对值小于0.25rad/s。结合`scan_stamp_before_s`和`scan_stamp_after_s`可确认直行前后使用了新扫描。

## 练习与故障定位

1. 保持持续时间不变，将直行速度改成0.15m/s。先预测理想位移0.30m、轮速1.00rad/s和墙距变化，再读真实结果。若改动影响了课程验收范围，保存比较结果后恢复默认参数。
2. 仅把DiffDrive配置轮半径改成0.20m，保留实际几何轮半径0.15m。预测同一速度指令得到更小轮速，理想实际直行速度约为指令的0.75倍。并排比较原生位移和里程计估计。
3. 若ROS发出指令却不动，从`/gt/cmd_vel`的ROS端点、桥方向、Gazebo话题和原生轮速依次排查。轮速变化但原生位姿几乎不变时，再核对关节轴、支撑高度和接触摩擦。能动但ROS端无激光时，先看Gazebo侧是否生成扫描，再检查Sensors/Ogre2、桥日志及QoS。扫描存在但回波无效时，核对扫描高度、视场、量程和墙面位置。

独立挑战是编写一个带停止距离的ROS 2控制节点。保存最近一次有效扫描的接收时间，按前向扇区内有限且在量程内的距离决定速度。达到阈值、数据无效或超时都发送零速度。用实际墙距、停车后的物理位置与时序记录检查它的行为。
