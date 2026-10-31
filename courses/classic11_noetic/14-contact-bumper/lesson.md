# 碰撞保险杠：接触发生了什么

红色方块落在蓝色台面上，画面几乎不动，接触消息却不断到来。这表示一次次新的碰撞，还是同一次持续支撑？方块靠近台面时能否提前检测？本课用一个可重复释放的落块，观察空中、碰撞和落稳三个阶段。

蓝色模型叫bumper，用来模拟接触式保险杠的检测面；它是固定台面，没有弹簧、形变或真实开关电路。红色puck是质量默认1 kg、尺寸0.35×0.35×0.2 m的刚体。你将看到真正的碰撞几何、接触约束和ROS消息，但不能由这个理想模型推断实体保险杠的安全性能。

## 先使用ROS观测，不要求先学完整ROS

平台为本课启动独立ROS Master和Gazebo，加载系统的gazebo_ros_bumper插件，把接触结果发布到`/lesson14/contacts`。**主题（topic）**是按名称发布和订阅消息的通道；**消息类型**规定里面有哪些字段。节点、Master和启动文件从第22课集中讲解，这里先把它们当作准备好的数据观察通道。

请使用平台打开的本课终端。另一个实验可能有同名主题，却连接另一个Master。

```sh
rostopic type /lesson14/contacts
rostopic echo -n 5 /lesson14/contacts
python3 lab.py observe --seconds 3
python3 lab.py ready
```

第一条应显示gazebo_msgs/ContactsState；第二条打印五条真实消息后退出；observe给出有界观察摘要。世界暂停时没有新的物理传感器更新，等待不到消息不能直接诊断为网络故障。不了解发布/订阅，可先看[ROS1主题教程](https://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)，无需现在完成整套ROS课程。

## 数据如何从两个物体走到终端

```text
刚体位姿与速度
  → collision几何的碰撞检测
  → ODE接触约束求解
  → Contact Sensor筛选指定collision
  → gazebo_ros_bumper转换消息
  → /lesson14/contacts
  → 终端或本课验收程序
```

**碰撞几何（collision）**决定物理引擎用什么形状检测接触，visual只是画出来的外观。**接触法线（normal）**描述接触面的方向，**穿透深度（depth）**是离散接触计算中的几何量；微小非零值不能直接解释成模型失效。**接触约束**通过求解力阻止继续穿透，参与支撑和摩擦，不要求刚体具有可见弹性形变。

Contact Sensor只关心它绑定的碰撞体。本课绑定bumper模型plate连杆的bumper_collision，所以方块掉到远处地面时，保险杠不应报告那一对接触。[Classic接触传感器教程](https://classic.gazebosim.org/tutorials?tut=contact_sensor&cat=sensors)介绍了这层筛选关系。

## 一条ContactsState包含什么

外层有header和states。每项ContactState是一条接触对记录，里面又有多个接触点。**本版本还会批量保留传感器两次更新之间的多个物理时刻**：同一对碰撞名称可能在一条ROS消息里重复出现。因此states长度既不是物体数量，也不等于同一瞬间的不同接触对数量。不能把所有states的力直接相加当成一个时刻的支持力。

| 字段 | 含义与单位 | 阅读时注意 |
| --- | --- | --- |
| header.stamp | 本次传感器更新时间，仿真s | 不为每项state单独提供精确物理时刻 |
| header.frame_id | 插件写入的参考系标签 | 单改标签不等于坐标变换 |
| states | 本次批量接触记录 | 可含同一碰撞对在不同时刻的多条记录；空列表也是有效数据 |
| collision1_name / collision2_name | 完整碰撞体名称 | 不保证保险杠排第一 |
| contact_positions | 各接触点位置，m | 本课在世界坐标中检查台面附近的位置 |
| contact_normals | 各点法线 | 方向与碰撞顺序有关 |
| depths | 各点穿透深度，m | 不与台面厚度混淆 |
| wrenches | 各点的力和力矩，N与N·m | 插件取collision1一侧的反馈 |
| total_wrench | 本条接触记录中各点反馈的分量和 | 不跨states求和，不是包括重力的全部外力 |

**wrench**是力与力矩组成的六维量，先分别看force与torque三个分量即可。字段定义见[ContactState](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/msg/ContactState.msg)和[ContactsState](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/msg/ContactsState.msg)。批量语义见本设备对应的[Gazebo 11.15.1 ContactSensor实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/ContactSensor.cc)；不能仅凭ROS消息字段名猜测它已把多个物理时刻合并成一个快照。

### 参考系不能只看header

本课设置frameName=world，但不能据此宣布所有字段都经过TF转换。Noetic的[bumper插件实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_bumper.cpp)中，查找参考连杆和变换的代码被注释，实际使用恒等变换。与此同时，Gazebo 11的[ODE实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODEPhysics.cc)已把力、力矩反馈转到各自连杆轴下。位置、法线与wrench不能无条件当成同一参考系的数据。

本课台面不旋转，落块从单位朝向释放，落稳时连杆轴与世界轴近似一致，才适合比较竖直反馈的量级；程序还检查实际姿态。若任意旋转落块，或把frameName改成base_link，不能只换字符串就继续使用世界Z分量。作用反作用力也应先表达在同一坐标轴下比较。坐标基础可回看第3课，TF在后续课程展开。

## 先预测，再重新释放

开始实验时方块可能已经落稳，不能用此时的一条消息证明自由落体阶段。先做三个预测：

1. 方块底面还高于台面时，没有保险杠接触；能收到有效的空states不表示传感器损坏。
2. 刚撞上台面时，速度在短时间内改变，接触力可能超过重力；50 Hz数据不保证采到最高冲击峰值。
3. 落稳后，方块中心约在z=0.3 m，支持力量级接近mg，不会因释放更高而永久变大。

```sh
python3 verify.py
```

验收先在X=1.5 m释放方块，让它真实落到保险杠外的地面，取得有效的空接触窗口；再移回台面上方释放，检查新的空中、接触和连续落稳数据。它不是读取一条已有非零消息就宣布通过。结束时保留本轮落稳后的方块，并恢复验收前的暂停或运行状态，不恢复旧运动和仿真时间。测量时不要另行拖动物体、重置模型或同时执行drop。

台面落稳窗口至少含20条不同时间戳消息、跨度至少0.4仿真秒；每条消息先平均本包各条原生接触记录的力模，再统计窗口均值和中位数，与mg比较。它们应落在mg的±15%内，每条消息的平均量级在±35%内。程序同时检查实际高度、水平偏移、速度和姿态，不能靠取绝对值把错误运动变成“支撑正确”。原始批量记录与分项结果写入当前会话目录的contact-verify-唯一编号.json，终端打印trace_path；可用`python3 verify.py --report build/my-contact-check.json`明确指定个人报告位置。

要自己观察，先在一个本课终端启动echo或observe，再在另一个终端执行：

```sh
python3 drop.py --height 1.8
```

height是方块中心的世界Z，不是底面到台面的距离。工具只在释放边界设置位姿和零速度；后续下落、冲击和支撑由物理引擎计算。它保留原暂停状态，原来暂停时需点击“继续”。

## 支持力、冲击与采样率

台面顶面z=0.2 m，落块半高0.1 m，水平落稳的预测是`z_center=0.2+0.1=0.3 m`。竖直静止平衡时，方块受到的向上接触力约为`N=mg`：1 kg对应9.81 N，2 kg对应19.62 N。消息若记录台面一侧，力方向相反，不能要求force.z永远为正。

冲击阶段不是静止平衡。增大释放高度通常增加接触前速度，但力峰还受步长、约束参数和采样影响。本课没有将刚体接触峰值标定为实体材料冲击载荷。每条state的total_wrench已经加总该条记录的接触点；稳定窗口应比较各条记录的反馈量级，而不是把一包约20个物理时刻累加成约20mg。

物理步长0.001 s对应目标1000步/仿真秒，传感器目标50次/仿真秒；终端接收频率还受实时因子和调度影响。50 Hz发布不保证每个物理步都送达客户端，批量消息也不自动提供完整1 kHz无遗漏轨迹。暂停时仿真时间不推进；用墙钟超时控制程序等待，与用仿真时间判断落稳是两件事。

## 单变量实验：只改一件事

world.sdf是个人真实输入，准备工具只检查，不重写文件。保存后须重载，旧世界不会自动采用新质量。

| 实验 | 操作 | 先预测什么 |
| --- | --- | --- |
| 质量1→2 kg | `python3 lab.py config --mass 2`，重载再验收 | 落稳高度不变，支持力量级约加倍 |
| 初始中心高度1.4→1.8 m | `python3 lab.py config --height 1.8`，重载再验收 | 下落时间和冲击可变，最终静止支持力不因落高改变 |
| 落到保险杠外 | `python3 drop.py --height 1.4 --x 1.5` | 地面接住方块，保险杠无对应接触 |

质量工具联动更新均匀盒体的三个对角惯性，只改这一组物理参数，不改形状或重力；高度工具只改初始中心Z。允许质量0.25到4 kg、高度0.6到3 m，固定机构和传感器名称保留以便比较。这是本课范围，不是Gazebo建模上限。

第三个实验请分开记录“方块与地面碰撞”和“保险杠报告接触”。回到台面可运行普通drop或完整验收，无需恢复模板抹掉其它修改。

## 文件和程序分别承担什么

| 文件 | 职责 |
| --- | --- |
| world.sdf | 台面、地面、方块、惯性、Contact Sensor与ROS插件 |
| prepare.py | 启动前检查依赖和保存场景，不覆盖个人输入 |
| lab.py | 输入检查、局部参数编辑、有界观察及测量公共逻辑 |
| drop.py | 明确重新摆放本课方块并释放，保留原暂停状态 |
| verify.py | 真实下落、接触与落稳的完整验收入口 |
| reference.md | 只读推导与示例，不覆盖练习 |

碰撞体、传感器和插件的名字不是同一回事。sensor/contact/collision必须指向所在link中的真实collision名称，填visual名称不会自动生成传感器。ROS插件还需要Gazebo侧ROS初始化；平台准备这层环境，不要求你手工再启动一套roscore。

## 故障诊断与独立挑战

| 现象 | 按顺序检查 |
| --- | --- |
| 完全没有消息 | 本课终端/Master、世界是否推进、真实发布者与插件日志 |
| 能收到空states | 方块是否在空中或保险杠范围之外；这是有意义的数据 |
| 已经碰撞却无保险杠接触 | collision几何、传感器绑定名、是否其实碰在地面 |
| 力符号与预期相反 | collision1是哪一侧、该侧连杆姿态，不能先取绝对值掩盖问题 |
| 支持力不接近mg | 是否连续落稳、质量是否加载、是否有其它接触和运动 |
| 改质量后仍是旧结果 | 是否保存个人文件并重载；检查文件不等于运行世界更新 |
| states很多，合力竟约20mg | 是否把批量保存的不同时刻误作同时接触而跨states相加 |

故障练习先备份world.sdf，把传感器绑定名改为不存在的名字，运行`python3 lab.py ready`并阅读明确诊断，再恢复原名。不要为了制造“坏传感器”继续运行已知错误课包。进一步可在课外副本移除传感器、保留collision，理解物理支撑与观测能分开存在，再回到本课固定结构。

独立挑战：做一个“进入接触→持续接触→离开接触”的事件计数器。同一碰撞对的多条批量记录、多帧持续消息不应算多次撞击。先把完整名称排序形成无序键，再考虑偶发空数据、批量时间和暂停；写清去抖规则及证据边界。本课自动通过不代表完成事件计数、读懂参考系或掌握手工读消息。
