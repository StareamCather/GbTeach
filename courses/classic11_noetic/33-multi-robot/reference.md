# 参考：多机器人名称、原生差速插件与真实隔离证据

本页解释Gazebo Classic 11.15.1、ROS 1 Noetic下本课的配置语义和验收路径。它不提供ROS 2、现代Gazebo或多机器人导航栈的替代配置。预测值与实际报告分开保存；发布两套接口和加载两个模型只是必要条件。

## 名称解析与模型作用域

Gazebo模型名alpha、beta在世界中唯一。它们内部可以重复使用相同link/joint局部名；本课重复使用chassis、left_wheel、right_wheel等link名，而实际轮关节还额外加上机器人名，完整实体如alpha::alpha_left_joint、beta::beta_left_joint。GazeboRosDiffDrive加载时持有所属ModelPtr，GazeboRos::getJoint调用该model的GetJoint，不会自动跨到另一模型。

ROS namespace是另一套名称系统。GazeboRos为每个模型插件按robotNamespace构造NodeHandle，相对cmd_vel、odom、joint_states通过该handle解析。节点完整名、话题完整名、Gazebo模型名和frame内容没有天然的一一改名机制。[GazeboRos辅助类](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/include/gazebo_plugins/gazebo_ros_utils.h)

| 配置 | alpha | beta |
| --- | --- | --- |
| Gazebo model name | alpha | beta |
| robotNamespace | /alpha | /beta |
| commandTopic | cmd_vel | cmd_vel |
| odometryTopic | odom | odom |
| ROS完整速度话题 | /alpha/cmd_vel | /beta/cmd_vel |
| ROS完整里程计话题 | /alpha/odom | /beta/odom |
| ROS完整轮关节状态话题 | /alpha/joint_states | /beta/joint_states |
| odometryFrame | alpha/odom | beta/odom |
| robotBaseFrame | alpha/base_footprint | beta/base_footprint |
| leftJoint / rightJoint | alpha_left_joint / alpha_right_joint | beta_left_joint / beta_right_joint |

若把commandTopic写为全局/cmd_vel，前面的/会使它跳出各自namespace；两个插件可能订阅同一个命令流。相对名、全局名、私有名~name与remap的解析入口可查[Noetic roscpp names.cpp](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/names.cpp)。话题改名不会改变Twist类型或消息中的任何字段。

## launch group、remap与本课配置的关系

在ROS 1 launch中，`<group ns="alpha">`为组内相对节点、参数与接口建立上下文；`<remap from="cmd_vel" to="drive_cmd"/>`改变受影响节点解析到的ROS接口名。以/开头的全局名需要按其全局语义理解。group不改Gazebo模型名，remap也不替换Odometry.child_frame_id里的字符串。[Noetic launch解析器](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/xmlloader.py)

本课没有用launch group生成两车，而是由world.sdf明确放入两模型与两个插件，再以robotNamespace决定各自NodeHandle。因此不能在一个额外launch中只包一层group，就断言已经在gzserver内加载的插件全部换了namespace。若以后采用URDF/Xacro加spawn，应分别传入Gazebo model名、插件namespace、各节点参数和frame策略，并从实际图与消息回读确认。

URDF中的link/joint树也不自动是所有物理实体的完整TF。固定关节的转换、robot_state_publisher是否存在、关节状态名称是否匹配，均决定最终结果。本课没有RSP，不发布车轮TF；当前joint_states是原生插件读取实际轮角形成的状态流，不是让用户拖动轮子的一组虚拟输入。[ROS 1 URDF教程](https://github.com/ros/urdf_tutorial/tree/ros1)

## 显式frame与tf_prefix

GazeboRos::readCommonParameter使用tf::getPrefixParam在插件NodeHandle所在参数层级向上查找tf_prefix；resolveTF再调用tf::resolve(prefix, frame)。这与话题的NodeHandle名称解析是两条路径。[GazeboRos实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_utils.cpp)、[Noetic getPrefixParam](https://github.com/ros/geometry/blob/noetic-devel/tf/include/tf/transform_listener.h)

本课选择显式alpha/odom、beta/odom等frame；prepare在插件加载前写入/alpha/tf_prefix=""、/beta/tf_prefix=""，阻断向上搜索继承全局前缀，ready也回读为空。实际消息内容仍须匹配当前策略；仅从SDF里看到alpha/odom尚不足以推断最终输出。

frame ID采用不带开头/的名称。它是消息内容，不是ROS Master登记的一个topic。ROS remap一般处理接口名称，不自动进入任意消息字段重写frame。若采用前缀方案，必须由实际发布器支持并在各个相关发布器间保持一致。

共享/tf是Noetic广播器的正常使用方式；广播器向全局/tf发布TFMessage，每包可带多条TransformStamped。唯一child及一致的父子关系让buffer能组织树，按机器人拆成/alpha/tf、/beta/tf不是本课所选方案。[Noetic动态广播器](https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/transform_broadcaster.cpp)

bridge.py作为/lesson33_world_bridge节点使用全局/tf_static，一次发送world→alpha/odom、world→beta/odom两个单位变换，并持续存活以维护latched连接。它还在/lesson33/configuration发布latched源/参数/插件身份。静态广播器latch使后来的订阅者可读固定关系；动态边由各自Gazebo插件发到/tf。[Noetic静态广播器](https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/static_transform_broadcaster.py)

## WORLD与ENCODER不是同一个odom来源

本课显式odometrySource=world。原生插件在publishOdometry中读取所属模型WorldPose及世界速度，把pose写入Odometry，同时把同一位姿发布为odom→base_footprint。线速度通过当前yaw转到车体方向，angular.z取世界偏航角速度。两个frame名独立，不意味着两份坐标数值必须各自从零开始。

因此本课的坐标关系为：

```text
T_world_alpha/odom = I
T_alpha/odom_alpha/base_footprint = T_world_alpha/model

T_world_beta/odom = I
T_beta/odom_beta/base_footprint = T_world_beta/model
```

初始alpha/beta的y偏移已经包含在动态位姿。把出生平移写入静态桥会重复叠加。base_footprint在本课是插件给模型位姿标注的frame身份，不是插件自动求出的接地投影；它是否与某个URDF link重合，需要描述者自己约定。

ENCODER分支则读取实际左右关节角速度，以r、b和模拟时间增量积分pose_encoder。它使用中点航向近似计算短程增量，不从WorldPose回填。编码初态、重置行为、轮地滑移与几何标定会影响结果；若改用此来源，需要重新定义world→odom关系和验收，不沿用当前绝对世界位姿判据。

REP-105中的odom通常用于连续的局部运动估计，可随时间漂移；map承担另一类长期定位参考。本课利用仿真world来源把坐标身份问题单独讲清，并未实现轮编码估计、融合定位或map→odom校正。[REP-105官方源文](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)

## 原生ROS差速插件的控制过程

本课使用gazebo_ros_pkgs 2.9.3中的GazeboRosDiffDrive，库名libgazebo_ros_diff_drive.so。它读取Twist.linear.x和Twist.angular.z，保存为x_、rot_；在物理更新中先算两轮线速度v∓Ωb/2，再除半径转成关节速度目标，通过关节SetParam("vel")与fmax电机上限驱动。两车使用各自所属模型的关节。[2.9.3插件源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)

这条路径与第11课Classic自带DiffDrivePlugin不同：ROS版用标准Twist和显式wheelDiameter/wheelSeparation，不能照搬旧Pose字段编码、转向符号或从碰撞包围盒推半径的规则。几何轮半径与插件配置半径都应为0.1 m，wheelDiameter为0.2 m，wheelSeparation为0.36 m。

本课publishTf=true使publishOdometry被执行；publishOdomTF=true使其发送odom→base边。该版本外层publishTf门控整个publishOdometry，因此设false会同时影响odom消息，不能按标签名字猜成“只关TF”。publishWheelTF=false避免额外车轮frame；publishWheelJointState=true发布真实关节名和角度，velocity、effort数组为空，不能把它们理解成测得零轮速/零力矩。updateRate=50 Hz按模拟时间调度，物理步长1 ms；wheelTorque=5 N·m、wheelAcceleration=0沿用原生无加速度限制分支，不另加课程PID。

回调队列由独立线程按WallDuration轮询，暂停时可以接收新速度目标；真正关节更新、世界位姿变化和定期odom发布依赖模拟物理步。原生插件没有速度命令超时：发送端退出不会自动把x_、rot_归零。课程有限控制的结束和异常收尾需要主动向两路发零Twist。

## 差速预测与比较参考点

两车采用X前、Y左、Z上坐标，左右轮位于y=±0.18 m，两轮半径r=0.1 m。v为前向线速度，Ω为绕+Z偏航角速度，正转向右轮更快：

```text
ωL=(v−Ωb/2)/r       ωR=(v+Ωb/2)/r
v=r(ωL+ωR)/2        Ω=r(ωR−ωL)/b
```

在恒速、理想无滑的T秒窗口中，直行Δx=vT，自转Δyaw=ΩT。当前默认T=2、v=.20、|Ω|=.60，对应直行.40 m、自转角幅值1.20 rad。正自转轮速为(−1.08,+1.08) rad/s，负自转相反。这些数是预测，报告中的实际运动量由真实状态独立计算。[作者教材中的差速模型](https://www.roboticsbook.org/S52_diffdrive_actions.html)

位移使用指定窗口实际起点，不能把reset造成的坐标跳变算成行驶。若起点航向不为0，世界位移应转到起始车体系：

```text
[Δx_local, Δy_local] = Rz(−yaw_start) [x_end−x_start, y_end−y_start]
```

航向差用最短角差或连续展开处理±π边界，四元数q和−q表示相同姿态。只比较世界vx不能覆盖转弯后的车体速度语义。Odometry.pose属于header.frame_id，twist属于child_frame_id；本课世界真值与odom做位置/姿态比较时有相同参考，速度则须先统一坐标。[Odometry消息定义](https://github.com/ros/common_msgs/blob/noetic-devel/nav_msgs/msg/Odometry.msg)

## 共享物理世界的意义

两车命令与状态通道独立，仍能通过碰撞、地面和物理求解相互作用。未受命令的一车若被另一车撞动，世界状态应该如实变化；不能用它的目标速度为零推导其实际速度必为零。当前出生间隔为1.6 m，有限直行/自转对照在分离区域开展，便于观察通信隔离。

轮子、车身和支撑具有真实碰撞与惯性。接触会留下小量滑移、起停误差和自转平移，因此理想运动学只提供方向、量级和预计关系。验收预设有限阈值，不能在看到数据以后随意放大容差，也不能以发送目标值替代实际轮速或车身运动。

零目标车的真实轮角接近不变时，ODE接触与约束求解仍可能带来有限被动漂移。通道隔离不等于所有实际状态精确为零；判定需同时看该车全阶段只收到零Twist、实际轮角与世界位姿/速度处于有限阈值内。另一车碰撞推动、轮地接触漂移和错误命令驱动应分别用证据定位。

## 时钟、原始stamp与接收时间

/use_sim_time在ROS节点初始化前设置为true，Gazebo提供唯一全局/clock。rospy初始化时按该参数订阅/clock，收到clock前可读到零；暂停后模拟时间停止，墙钟仍继续。[Noetic模拟时间初始化](https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/impl/simtime.py)

插件的物理更新调度使用世界模拟时间，Odometry与动态TF的header.stamp使用ROS Time::now。共享stamp可用于核对消息标签，但不能把它解释成独立取得的精确物理采样时刻。TFMessage内每条变换各有自己的stamp，应保留原字段。

若观察Gazebo ModelStates，它以并行name/pose/twist数组表达world状态，没有header，必须按模型名找相同索引，不能固定取第一个或第二个模型。接收monotonic时刻是客户端观察时间，不是原消息不存在的采样stamp。本课工具实际采用按模型名的GetModelState查询，而非订阅ModelStates。[ModelStates定义](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_msgs/msg/ModelStates.msg)

models查询在两次GetWorldProperties之间依次读取alpha和beta的GetModelState(relative_entity_name="world")，报告保留sim_time_before、sim_time_after。该服务的header在ROS API中填写ROS当前时间，不是保证的原生采样stamp。课程保存位置、四元数、yaw及实际linear/angular速度，快照还附带最新所选ROS字段。当前工具没有对/clock逐条归档，而是用实际world服务时间控制阶段。

世界状态服务与odom/TF回调不保证在同一物理步发生。应以实际记录的时间邻近关系和有限误差核对，不声称每条ROS消息都与一次稍后的服务查询严格同步。Time(0)的TF查询表示最新共同可用关系，也不是请求仿真初始零时刻；课程当前直接核对消息边，并未以tf2 lookup插值结果替代原消息。

就绪和消息等待设有限单调墙钟边界，避免用冻结的模拟时间等待“超时”。暂停图检查可以确认实体、服务、topic连接与静态桥；新动态流必须在明确推进后检查。暂停中能读取上次缓存，不等于取得了新鲜运动证据。

## 实验与验收的证据层次

| 层次 | 需要证明的事实 | 不能替代的下一层 |
| --- | --- | --- |
| 源配置 | 两model、两namespace、唯一frame写法合法 | 实际实例已加载此输入 |
| 运行图 | 正确Master中的两路连接与共享TF存在 | 有新消息或物理响应 |
| 原生消息 | odom与TF字段、来源、stamp属于当前两车 | 对应模型在真实世界中运动 |
| 世界状态 | 目标车的实际位移/转向、另一车有限漂移 | 图形窗口已被人工观察 |
| 并发对照 | 两车同时执行不同目标且身份保持 | 多机器人导航或避障能力 |

三个正式动作分别为alpha正向直行/beta零目标、beta正向自转/alpha零目标，以及alpha正向直行/beta负向自转。对每阶段保存实际初末基线、命令记录、模拟经过时间、两路原生odom及动态TF的选定字段、静态桥与Gazebo实际状态。未受命令车也纳入评价，防止只验证“目标车有动”。

odom与动态TF来自同一个Gazebo插件，因此相互一致不是两种独立定位方法的交叉验证。世界服务查询补充物理对象的独立读数；WORLD来源仍与物理模型同源，结论是命名、发布与物理响应一致，不是编码定位精度。

## 实际命令、报告与有限时间

| 入口 | 实际作用与边界 |
| --- | --- |
| python3 lab.py ready | 不推进物理；读回实际模型/关节、1 ms步长、两路接口、空前缀、静态桥和配置身份 |
| python3 lab.py bootstrap | ready后两路零目标，推进至少0.5模拟秒，取有真实动态数据的快照，结束暂停 |
| python3 lab.py status | 只读世界/物理状态与本进程已有最新消息，不保证新动态消息 |
| python3 lab.py alpha / beta / both --seconds 2 | ready后运行相应阶段，支持0.5～3模拟秒 |
| python3 lab.py pause | 反复发两路零Twist，再确认实际世界暂停；返回zero_requested、paused_confirmed，不推进停车段 |
| python3 lab.py reset | 零目标/暂停后调用一次reset_simulation；读回时间0、暂停及出生位置，作废旧动态缓存 |
| python3 lab.py exercise | bootstrap后依次alpha、beta、both，保留真实阶段证据 |
| python3 verify.py | 同一exercise算法，保存build/verify_report.json并打印概要 |

每次零目标通过两路各发送五次Twist、每轮间隔25 ms墙钟完成。advance先等待连接，发目标、继续世界，按GetWorldProperties监测达到请求的模拟时长；墙钟上限为max(20 s,请求时长×12)。无论正常结束或抛出异常，都尝试两路零目标并暂停。暂停/继续只提交必要的一次服务调用，再只读轮询实际pause与时间三次稳定结果，不盲目重发变更。

每个phase先advance(.4)预热、暂停取before，再advance(T,commands)，随后advance(.6)发零尾段，暂停取after。phase事件从before后开始，包括运动与零尾段。位移和轮角从before到after计算，包含起停过程；timing中的实际结束也包含收尾开销，不能把requested_sim_seconds当作无限精度的恒速积分窗。三个阶段接续当前实际位置，阶段间不复位。

reset使用真正世界复位，会使模拟时间回到0，不是只把两车移回出生点。它读回三次时间0且暂停，检查两车x/y距出生值各小于.005 m，保留历史事件但移除旧动态latest，再等待下一次bootstrap取得新时间段反馈。报告不把复位前的stamp当作复位后新鲜样本。

lab用build/lab.lock排斥同一练习目录中的多个课程控制入口；它不替外部用户节点仲裁cmd_vel。当前课程动作结束保持暂停，不恢复进入时位姿、速度、pause或旧时钟。运动窗没有SetModelState，也没有按理论轨迹写回实际位置。

## 课程内数值判据

check_feedback在每个动态快照要求两车odom、JointState和动态TF都存在，stamp距当前world时间在[−.001,.12] s内；frame必须为该车的odom→base_footprint，JointState名集合必须是该车两个实际轮关节。odom和TF位置各自距实际模型三维位置小于.045 m，yaw差小于.08 rad。

每阶段两车各至少十条odom；活动车必须观察到相应目标Twist，非活动车整个阶段只可出现零Twist。alpha直行要求起始车体系前向投影>.5vT、航向差幅值<.12 rad，两实际轮角增量均>.3 rad。beta自转要求所命令方向的航向增量>.45|Ω|T、平面净位移<.08 m，左右实际轮角呈预期相反符号且幅值>.3 rad。非活动车净位移<.03 m、航向差幅值<.06 rad。

每阶段0.6 s零目标尾段后的after快照还提供实际Gazebo模型速度。对两车分别计算terminal_xy_speed_mps=√(vx²+vy²)、terminal_yaw_rate_rps=ωz，要求平面速度<.04 m/s、|偏航角速度|<.08 rad/s。这是实际终点读数的有限速度判据，不从目标Twist推算；单独pause仍只保证目标归零及世界暂停，没有推进一个物理停车段。

这些是当前固定模型与有限实验的下限、漂移和终点速度检验，不是严格的恒速轨迹误差测量；不能从passed推导全窗口速度均等于目标或静止车精确零漂移。更严格的实际停车速度窗口由独立开发验收覆盖。

单次lab操作写入build/{operation}_report.json，verify写build/verify_report.json。events按接收顺序保留id、source、receive_monotonic_ns及选定data：odom保留位置/四元数/原stamp/父子frame/linear.x/angular.z，JointState保留name/position/velocity，TF按每条边保留原stamp/变换，Twist保留本课使用的两个字段。它不保留所有原消息字段、连接header或原始序列化字节，不能称为完整ROS包归档。

## 独立开发验收的范围

tests/lesson33_acceptance.py另外创建全新Storage、私有ROS/Gazebo Master和独立rospy观察进程，直接发送真实Twist，不调用课程phase控制逻辑，也不改模型位姿或速度。它保留完整ROS序列化字节、原消息字段、连接header、stamp及接收时刻，核对真实Gazebo进程加载的库、实际TCPROS连接、唯一TF父子与发布者、世界来源odom、真实关节服务读数和终端速度窗口。

独立对照采用基线(.20,.60)、只改线速(.30,.60)、只改角速(.20,.90)，每个案例的三阶段都实际运行。自转变式预期更大转角；它与教材把角速降为.40的手动挑战属于相反方向的同一单变量关系。测试是否通过由本次生成报告决定；教材没有预填运行数据。

该测试中的停止段持续1模拟秒，末端0.3 s实际查询要求平面速度<.025 m/s、|偏航角速度|<.06 rad/s，末段位移<.012 m，并检查原生odom报告停止。动态图形检查不在自动测试内，报告明确保留graphical_checks_performed=false；Gazebo窗口另由人机观察确认。

验收会改变实际机器人状态并推进模拟时间。是否通过与本次真实报告绑定；静态解析、脚本编译或进程启动不单独形成运动验收。

## 参数单变量与诊断顺序

parameters.json支持linear_speed_mps=.15/.20/.30、angular_speed_rps=.40/.60/.90，默认.20与.60。修改保存并重载后再做当前配置的实验。线速度.20→.30令直行轮速与理想路程乘1.5，自转目标不变；角速度.60→.40令自转轮速和理想转角乘2/3，直行目标不变。

诊断按“目标文件→实际配置→完整ROS接口→原始frame字符串→所属Gazebo模型与关节→物理时间及接触”逐层推进。topic重名、frame重名、旧命令残留、暂停无新消息与碰撞推动会有相似画面，但原始证据不同。先定位断在哪一层，再修改对应一个变量。

本课算法流程为：**准备时校验世界/参数和原生插件并设置时钟/空前缀→真实世界加载两份物理模型→分别解析ROS接口并建立唯一frame→静态桥把两套world来源odom接入共同world→按墙钟确认基础连接→零目标预热/暂停取起点→按模拟时间执行有限双通道命令→零目标尾段/暂停取终点并检查两车实际末速→由实际阶段增量、轮角与命名归属判断隔离→记录实际暂停及复位边界。** 课程工具组织实验，原生差速插件和物理引擎产生运动与原始状态。
