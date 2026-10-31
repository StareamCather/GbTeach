# 多机器人：让命令、状态与坐标各归其车

同一个Gazebo世界里放入两辆小车，只完成了物理模型的复制。若它们仍订阅同一个速度话题，一条命令就可能让两车一起走；若它们都发布名为base_footprint的TF，观察者又无法可靠区分两辆车的坐标。本课把这两类问题分开，用真实运动检验隔离是否成立。

环境固定为Gazebo Classic 11.15.1、ROS 1 Noetic和Ubuntu 20.04。场景包含蓝色alpha与橙色beta两辆真实差速车，初始世界位置分别为(0,−0.8,0)、(0,+0.8,0)，初始车头都朝世界+X。它们各有自己的左右轮、后部球形支撑、碰撞和差速插件，处在同一个重力、地面和时钟下。

## 从已有知识接过来

第11课解释两轮差速运动，第22课解释ROS图与名称解析，第24课解释TF和模拟时间。本课把三条线合在一起，学习目标是自己说明：一条命令为什么只到指定机器人，返回的状态属于谁，两个frame为什么能放进同一幅世界图。

| 前置概念 | 本课需要知道的最小内容 | 官方入口 |
| --- | --- | --- |
| link与joint | link表示刚体，joint连接刚体并约束相对运动 | [ROS 1 URDF教程](https://github.com/ros/urdf_tutorial/tree/ros1) |
| ROS名称 | 相对名在namespace下展开，/开头的是全局名 | [Noetic名称解析实现](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/names.cpp) |
| frame与TF | frame是具名坐标系，TF给出父子系在某时刻的关系 | [ROS geometry2](https://github.com/ros/geometry2/tree/noetic-devel) |
| odom与base | odom是运动参考系，base是随车移动的车体系 | [REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst) |

本课直接用SDF描述两车，没有另外启动URDF、robot_state_publisher或机器人launch。URDF与SDF都能描述机器人，但ROS名称空间和TF命名问题并不依赖某一种描述文件。此处提供的TF只覆盖world、两套odom与base_footprint，不把每个车轮link都列入TF树。

## 三种名字，分别回答三个问题

| 层次 | alpha例子 | beta例子 | 它决定什么 |
| --- | --- | --- | --- |
| Gazebo模型与内部实体 | alpha、alpha::alpha_left_joint | beta、beta::beta_left_joint | 哪个物理模型、哪个关节 |
| ROS接口名称空间 | /alpha/cmd_vel、/alpha/odom | /beta/cmd_vel、/beta/odom | 命令和消息接到哪个接口 |
| TF frame ID | alpha/odom、alpha/base_footprint | beta/odom、beta/base_footprint | 消息中的坐标系身份 |

Gazebo的双冒号表示模型内部的实体作用域；ROS接口的斜杠参与名称解析；TF里的斜杠是frame字符串的一部分。三个层次使用相似的alpha/beta字样，是本课主动建立的对应关系，不能只改一层就期待其它两层自动同步。

两模型内部的link可以重名，本课两车都有chassis、left_wheel、right_wheel；插件从各自所属model查找关节。本课还把实际关节名明确写为alpha_left_joint、alpha_right_joint及beta对应名称，便于直接辨认JointState。反过来，即使两ROS话题名字不同，两个控制程序若最终都操作alpha的关节，物理对象仍没有隔离。必须沿链路一直查到实际模型。

ROS节点名也不等于机器人名。本课两份gazebo_ros_diff_drive都在Gazebo服务端进程内运行，ROS图中它们的topic发布/订阅可显示同一个Gazebo节点身份。看到共同的发布节点，不代表两车必须共享cmd_vel；应继续看完整topic名、连接和真实运动。

## 一条命令怎样变成真实运动

```text
alpha有限命令 → /alpha/cmd_vel → alpha模型内的ROS差速插件
                                → alpha左右轮关节电机 → 轮地接触 → alpha车身

beta有限命令  → /beta/cmd_vel  → beta模型内的ROS差速插件
                                → beta左右轮关节电机 → 轮地接触 → beta车身

各模型真实world位姿 → 各自/odom消息与odom→base_footprint动态TF
Gazebo状态服务     → 独立读取alpha/beta实际世界状态
静态桥            → world→alpha/odom、world→beta/odom
Gazebo /clock     → 所有使用模拟时间的本课ROS节点
```

速度消息是geometry_msgs/Twist，本课只使用linear.x作为前向速度v、angular.z作为偏航角速度Ω。单位分别为m/s、rad/s，车体X向前、Y向左、Z向上；从上往下看逆时针为正转向。其余分量为零。消息没有header，不能把发送方的接收时刻称作命令自带的时间戳。[Twist定义](https://github.com/ros/common_msgs/blob/noetic-devel/geometry_msgs/msg/Twist.msg)

两车使用ROS版libgazebo_ros_diff_drive.so。它与第11课Classic Transport版DiffDrivePlugin不是同一个接口：这里直接发送标准Twist，正Ω不需要第11课旧Pose接口的符号转换。插件先算左右轮缘速度，再除半径，给关节电机速度目标；车身由关节、碰撞和接触推动。实验不通过连续写模型位姿来制造运动。[当前ROS插件实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)

## 先算出应该发生什么

两车轮半径r=0.1 m、轮距b=0.36 m，左右轮正轴都为+Y。理想无滑时：

```text
ωL = (v − Ωb/2)/r
ωR = (v + Ωb/2)/r
```

默认前向速度v=0.20 m/s，转向角速度幅值为0.60 rad/s。下表只给恒定命令持续2 s的理想预测；实际起步、停车、轮地滑移和调度边界应从测量读取。

| 对照 | alpha目标(v,Ω) | beta目标(v,Ω) | 理想2 s结果 |
| --- | --- | --- | --- |
| alpha前进、beta静止 | (0.20,0) | (0,0) | alpha两轮各2 rad/s、前进0.40 m；beta保持 |
| beta正转、alpha静止 | (0,0) | (0,+0.60) | beta左轮−1.08、右轮+1.08 rad/s，转角+1.20 rad；alpha保持 |
| 并发不同命令 | (0.20,0) | (0,−0.60) | alpha前进；beta左轮+1.08、右轮−1.08 rad/s，转角−1.20 rad |

单独实验说明一个通道可工作，并发实验进一步检查两个通道是否能同时承载不同指令。隔离判定还要观察未受命令的那辆车，不能只看目标车已经动了。

## /tf共享，frame身份独立

本课保留全局/tf、/tf_static，树为：

```text
world
  ├─ alpha/odom ─ alpha/base_footprint
  └─ beta/odom  ─ beta/base_footprint
     静态桥        各自差速插件发布动态边
```

同一条/tf话题可承载多条变换，TF按消息中的header.frame_id与child_frame_id组织树。alpha/base_footprint和beta/base_footprint互不重名，观察者才能同时保存两套关系。若两发布者都用base_footprint，即使它们发布频率相同、数值偶尔相同，也没有两个独立的子frame。

namespace不会自动改frame。`<robotNamespace>/alpha</robotNamespace>`让相对话题cmd_vel解析到/alpha/cmd_vel；它不会单凭这项就把字符串base_footprint改成alpha/base_footprint。本课显式填写odometryFrame=alpha/odom、robotBaseFrame=alpha/base_footprint，beta对应改名。

Noetic插件还支持从ROS参数服务器查找tf_prefix，再用它解析frame。prepare在插件加载前把/alpha/tf_prefix、/beta/tf_prefix显式设为空，防止继承全局前缀；实际frame已经完整区分两车。若再叠加alpha前缀，可能得到alpha/alpha/odom等不匹配名字。tf_prefix是frame命名机制，不是ROS namespace的别名，也不重命名Gazebo关节。[GazeboRos名称与frame处理](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/include/gazebo_plugins/gazebo_ros_utils.h)

## 为什么两车的odom起点不是零

本课两插件明确使用odometrySource=world。插件读取所属模型WorldPose，把实际世界位置写入各自odom消息。因此alpha初始y应接近−0.8 m，beta初始y应接近+0.8 m；两套odom的数值坐标轴与Gazebo world对齐，原点也与world重合。

所以world→alpha/odom和world→beta/odom都是单位变换：平移(0,0,0)、四元数(0,0,0,1)。出生偏移已经在odom→base中，静态桥不能再平移−0.8或+0.8，否则在世界图里会重复计算一次出生位置。

若以后把来源改为encoder，插件会积分轮速，通常从自己的编码初态开始；这需要重新定义odom原点与world的关系，也要改变比较方式。不能只改一个单词，却继续使用本课world来源的单位桥和世界绝对坐标判据。本课学习的是多机器人隔离，当前odom用于观察真实世界状态，不把它称作独立的轮编码定位。

Odometry中的pose由header.frame_id指定参考系，twist由child_frame_id指定参考系。本课pose使用各自与world对齐的odom系，线速度经插件转成车体系。world中的vx和转弯后车体linear.x不能直接逐项比较。[Odometry定义](https://github.com/ros/common_msgs/blob/noetic-devel/nav_msgs/msg/Odometry.msg)

## 时间与就绪分两步看

私有ROS Master中先设置/use_sim_time=true，再初始化Gazebo ROS与本课节点。全场只有一个Gazebo/clock，两车共享模拟时间；不为alpha和beta分别制造时钟。节点准备、ROS连接以及失败超时使用有限墙钟等待，运动持续时间按实际模拟时间计量。

初始暂停时，模型、插件和topic连接可以已经存在，但没有继续推进物理就未必有新odom和动态TF。ready只检查暂停下可确认的实际配置与图连接，不推进世界；bootstrap会发两路零目标，实际推进至少0.5 s模拟时间并取得真实消息，结束保持暂停。一个新订阅者在暂停中等不到动态数据，与名字配置错误是两种情况。

共享/tf_static有latch，新订阅者可取得已发布的固定桥；/tf和odom则要看新消息及其stamp。暂停后画面仍显示上次位置，并不能证明/clock还在前进。`rostopic hz --wall-time`测接收墙钟节奏，也不等于按原header计算采样频率。

## 个人文件与配置入口

| 文件 | 负责什么 |
| --- | --- |
| world.sdf | 两模型、出生位置、真实刚体/关节与各自ROS差速插件 |
| parameters.json | linear_speed_mps与angular_speed_rps两项实验速度 |
| config.py、prepare.py | 检查输入、插件版本与指纹，设置模拟时间及空frame前缀 |
| bridge.py | 持续维护两条静态桥与latched配置身份，不驱动车辆 |
| lab.py | 名称检查、有限控制、零目标、复位与状态观察 |
| verify.py | 用真实命令、odom、TF与Gazebo状态检验隔离 |
| build/multi_robot.json | 当前准备输入、源文件与系统插件指纹 |
| build/*_report.json | 操作结果、消息所选字段与真实世界快照 |

参数支持前向速度0.15、0.20、0.30 m/s，角速度幅值0.40、0.60、0.90 rad/s。用应用的编辑速度入口或`gedit --new-window parameters.json`修改个人文件，保存并重载，再检查当前运行配置。参数文件决定课程命令的目标值，不会改变轮半径或车轮惯性；保存文件也不意味着正在运行的实例已经采用新输入。源文件、world或参数改变而未重载时，指纹检查会拒绝混用新文件和旧实例。

启动顺序是：应用先建立私有Master，prepare设置时钟/前缀并核对当前输入；随后以暂停方式加载世界和系统ROS API，再启动bridge，最后执行ready。本课终端继承该会话的ROS_MASTER_URI与Gazebo连接，不需要再启动roscore或另一份世界。场景使用SDF内的基础几何、材质和灯光，不依赖在线模型下载或外部mesh路径。

## 实验一：先把三层身份对上

在本课操作终端运行：

```sh
python3 lab.py ready
python3 lab.py bootstrap
```

bootstrap报告保存在build/bootstrap_report.json，里面同时有真实模型位置和本次接收到的odom/TF/JointState字段。接着查看两车接口与静态桥：

```sh
rostopic info /alpha/cmd_vel
rostopic info /beta/cmd_vel
rostopic echo -n 1 /tf_static
```

若要亲自执行`rostopic echo -n 1 /alpha/odom`、beta/odom或/tf，先在另一个本课终端订阅，再从应用运行bootstrap或有限动作，使订阅者处在真正的物理推进期间。bootstrap完成后已经暂停，新建的echo不会因之前产生过数据就必然得到一条动态消息。

`python3 lab.py status`只读当前世界快照，不推进物理；新进程的动态缓存也可能为空。逐项记录完整话题名、实际frame字符串和各车世界位置。不要用topic名字推断frame，也不要只看到两个模型的颜色就认定接口已经隔离。

## 实验二：只命令alpha，测量beta是否保持

先预测左右轮速和位移，再执行：

```sh
python3 lab.py reset
python3 lab.py alpha --seconds 2
```

观察蓝车前进、橙车保持，同时读两车返回状态。每个动作先运行0.4 s零目标预热并暂停取起点，再对alpha发送前进Twist、对beta明确发零Twist；按实际模拟时间等待至少请求的2 s，接着运行0.6 s零目标尾段，暂停取终点。报告中的位移是起点到尾段结束的真实增量，包含起停过渡，不能要求严格等于恒速2 s预测的0.40 m。

`--seconds`支持0.5～3模拟秒。世界步进、服务往返与停止阶段会使实际总经过时间多于这个目标，报告保留timing和zero_tail的实际边界。模型复位只用于建立初态；真正测量运动的窗口靠速度命令与动力学推进。

没有给beta新命令不等于beta一定停止。此版本原生ROS差速插件会保留最后一次cmd_vel，没有本课另设的原生超时刹车；若它先前收过非零命令，必须显式归零。课程命令统一负责有限结束和零命令。

`python3 lab.py pause`会重复向两路发零目标，再确认世界暂停，报告标记zero_requested与paused_confirmed；它不另推进停车时间，也不把“命令已归零”当作实际速度精确为零。reset先这样处理，再调用一次真实reset_simulation，把世界时间和模型一同复位，读回时间0、暂停和出生位置；旧动态缓存作废，之后用bootstrap取得新时间段数据。它没有恢复复位前的轨迹或时钟。

## 实验三：交换角色，再发不同并发命令

```sh
python3 lab.py reset
python3 lab.py beta --seconds 2
python3 lab.py reset
python3 lab.py both --seconds 2
```

beta阶段应橙车正向自转、蓝车保持；并发阶段应蓝车前进、橙车反向自转。轮轴中点附近的自转会允许接触与支撑产生小量平移，判定使用本课有限容差和真实状态，不要求每个浮点坐标都完全不变。

如果两车碰撞，未受命令的一车也可能被接触力推动。这不必然是ROS串线。先核对两路cmd_vel是否真的独立，再查两车间距离与实际碰撞。本课初始间隔和有限动作留出了分离空间；隔离的是命令与状态身份，两车仍共享物理世界，能够相互作用。

即使没有两车相撞，ODE的轮地接触和约束求解也可能让零目标小车产生少量被动漂移，轮角则接近不变。因此本课用有限位移、航向与末速阈值判断“保持”，不要求浮点位置绝对不变；应结合零命令记录、真实轮角和世界状态，区分被动漂移与另一通道错误驱动。

## 实验四：运行真实数据验收

```sh
python3 verify.py
```

验收使用实际Gazebo、两路原生odom、共享TF与独立Gazebo状态查询，依次检查alpha前进/beta保持、beta正转/alpha保持和两车不同指令并发。它先bootstrap，再从每阶段当前真实状态建立基线，不在阶段之间复位位姿。检查包含前向/转向方向、真实轮角增量、未受命令车的漂移、至少十条各车odom，以及消息frame归属；公式预测只作对照，不会回填为反馈。

完整结果写入build/verify_report.json，单动作结果为build/alpha_report.json、beta_report.json、both_report.json。报告保存原生消息的选定字段及接收顺序、原stamp、单调墙钟接收时刻；它不是完整ROS消息的逐字节归档。应分别读命令目标、odom父子frame、/tf中的边、独立世界位置以及模拟时间。odom与TF来自同一原生插件，二者一致说明这层发布相符；与Gazebo状态查询相符才补上物理模型身份检查。这也不把world来源的odom冒称为独立编码测量。

每阶段0.6 s零目标尾段后，课程还从实际Gazebo终点状态计算两车的terminal_xy_speed_mps和terminal_yaw_rate_rps，分别要求平面速度<0.04 m/s、|偏航角速度|<0.08 rad/s。验收结束保持暂停，不恢复进入时的位姿、速度或pause，也不回退已经经过的模拟时间。独立开发验收另检查更严格的完整末速窗口；这些有限阈值不表示任意时刻都精确零速。

自动验收针对数据和有限动作。真实Gazebo窗口是否显示两辆车、颜色与运动是否对应，需要另外作图形观察；本课没有为RViz窗口另作显示验收。报告记录何种检查通过，就据此报告，不以世界加载成功代替运动与隔离结论。

## 单变量实验：只改变前向速度

将linear_speed_mps从0.20改为0.30，angular_speed_rps保持0.60。保存、重载、重新ready后，重跑alpha与both。预测alpha轮速从2变为3 rad/s，2 s理想直行从0.40变为0.60 m；beta转向的目标轮速和理想转角不变。若beta也变快，先查是否同时改了转速、发错topic或把两车记录混在一起。

第二次只把angular_speed_rps从0.60改为0.40，前向速度保持原值。预测beta两轮幅值从1.08变为0.72 rad/s，2 s转角幅值从1.20变为0.80 rad，alpha直行预测保持。每次只变一个字段，完整记录采用的参数及实际时间窗。

## 沿信息流诊断

| 现象 | 先核对哪一层 | 能区分什么 |
| --- | --- | --- |
| 两车一起前进 | 两路cmd_vel实际名称、订阅连接、命令记录 | 共享topic还是物理接触 |
| 两topic存在，但一车不动 | 所属model、left/rightJoint、轮轴/接触与暂停状态 | 名称登记成功还是实际驱动成功 |
| odom名字分开，frame仍重名 | 原消息header.frame_id、child_frame_id及tf_prefix | ROS接口隔离还是TF身份隔离 |
| 世界位置被平移了两次 | odometrySource与world→odom桥 | world绝对位姿是否又加了出生偏移 |
| 暂停中echo无输出 | 实际世界pause、新订阅者与动态消息 | 尚无物理更新还是接口错误 |
| 停止发命令后还在走 | 最后一条非零/零Twist、真实轮速 | 插件保留旧目标还是惯性/接触响应 |
| odom与GT速度分量不同 | pose/twist参考系与车头朝向 | world速度还是车体速度 |

## 挑战：从证据说明隔离

为一次并发实验整理一张两行表，每行对应一车：Gazebo模型名、cmd_vel完整名、odom完整名、TF父子frame、发送的(v,Ω)、实际模拟时间、实测位移/航向与是否符合预测。再写出若两车都叫base_footprint，会在哪一列丢失身份。

最后用一句有证据的话解释“另一辆车为什么保持”：应能指出它收到零目标，真实轮角没有受另一通道驱动，独立世界状态在有限漂移与末速阈值内，再区分两车碰撞和轮地接触的影响。只写“因为设置了namespace”，还没有覆盖物理对象和TF两层。

本课运行流程为：**检查个人输入与实际插件→加载两独立模型并建立两套接口→发布共享世界的固定桥→用有限墙钟确认图与配置→零目标推进取得真实消息→预热后记录起点→对两车分别发送有限Twist→零目标尾段后暂停取终点并检查两车实际末速→按真实位移、轮角、航向与frame身份判断隔离。** 各阶段时间与恢复边界见参考页，不把复位动作算作速度控制产生的运动。
