# ros_control：让位置目标经过真实控制器到达关节

本课控制一根固定在台座上的水平转杆。你向 ROS 发送一个角度目标，真实的 `effort_controllers/JointPositionController` 读取关节反馈、计算力矩，再由 `gazebo_ros_control` 把力矩交给 Gazebo。杆怎样加速、超调和停下来，继续由物理引擎计算。

第10课已经用小转台认识 PID，第23课给出了 URDF 机器人，第24课核对了状态与时钟。本课把它们接到 ROS 1 Noetic 的控制框架中，重点是看清控制器、硬件接口、资源和反馈各自负责什么。

## 先认识这几个对象

| 对象 | 本课职责 |
| --- | --- |
| URDF joint | 定义实际转轴、连接关系、运动范围和执行能力 |
| transmission | 描述受控关节及其控制接口；本课声明 EffortJointInterface |
| hardware_interface | 为控制器提供读状态、写命令的句柄；它本身不计算 PID |
| RobotHWSim | 把 Gazebo 关节与这些句柄连接起来；本课用 DefaultRobotHWSim |
| controller_manager | 加载控制器插件，并在更新周期中管理运行、停止和切换 |
| JointPositionController | 用位置误差计算 effort 输出；本课 effort 单位为 N·m |
| joint_state_controller | 发布接口中的实际关节状态；它不占用本课的力矩命令资源 |

控制器类型名中的 `JointPositionController` 说明它调节位置，`effort_controllers` 说明它通过 effort 接口输出。两者组合表示“位置闭环输出力矩”，不能看到 position 就认为程序会直接设置 Gazebo 关节角度。

句柄（handle）是访问指定关节状态或命令的接口对象。控制器取得带有关节名的句柄后，便可通过统一接口读取或写入，不必直接调用Gazebo物理API；硬件仿真层负责把它与本课具体关节连接。

本课的控制器是加载进 Gazebo 进程的 C++ 插件。`rosnode list` 不要求为每个控制器显示一个独立操作系统进程；启动控制器的 spawner 节点退出后，已运行的控制器仍可留在 controller_manager 中。

不了解共享库与插件，可先读 [ROS pluginlib 官方入门](https://wiki.ros.org/pluginlib)。不了解 URDF 的 joint，可回看第23课，再查 [URDF joint 官方说明](https://wiki.ros.org/urdf/XML/joint)。本课使用 ROS 1 接口，不套用 ROS 2 的 controller_manager 命令。

## 真实闭环怎样流动

```text
ROS Float64 位置目标
  → position_servo 的命令缓冲
  → controller_manager 更新控制器
  → JointPositionController 读取 q、dq，计算误差与 PID 力矩
  → EffortJointInterface 命令
  → DefaultRobotHWSim 的关节限幅和 SetForce
  → Gazebo 推进刚体运动
  → DefaultRobotHWSim 读取实际 q、dq、effort
  → 下次控制更新，并由 joint_state_controller 发布反馈
```

`/lesson25/position_servo/command` 发的是 rad，不是 N·m。控制器把目标保存在缓冲中，后续更新继续使用它；不是收到一次消息只施力一个瞬间。高频重发相同目标也不等于提高了物理更新频率。

本课实体名为 `servo_arm`，实际关节名为 `lesson25_hinge`，ROS 名称空间为 `/lesson25`。URDF joint、transmission 中的 joint 和控制器的 `joint` 参数必须指向同一关节。ROS 名称空间负责组织话题、服务和参数，不会自动改写 Gazebo 实体名。

## 模型与文件

转杆长0.6 m、质量1 kg，绕竖直 Z 轴转动，台座固定于世界。URDF给出的质心轴惯量为0.0303 kg·m²，质心离转轴0.3 m，因此名义转轴惯量为 `0.0303 + 1×0.3² = 0.1203 kg·m²`。世界使用零重力，让本课先研究一个简单转动闭环。它没有仿真电机电流、减速器柔性或真实驱动器内部环路；本课学习的是理想力矩执行接口上的位置控制。

URDF 中关节位置范围为 `[-0.8, 0.8] rad`，effort 上限为 `2 N·m`，速度规格为 `4 rad/s`。它们分别约束位置目标、执行命令和运动边界，含义不能混用。

| 文件或目录 | 用途 |
| --- | --- |
| parameters.json | 个人实验的 p、d 和目标角度；保存后重新加载实验 |
| world.sdf | 离线世界与物理设置 |
| prepare.py | 检查输入、准备真实控制器库、生成本课描述与配置证据 |
| ros_ws/src/gazeboteach_l25/ | 本课机器人描述、控制器参数与 launch 文件 |
| lab.py | 读取运行图、执行有限实验、记录真实状态并恢复入口状态 |
| verify.py | 在当前真实实验中检查控制链与行为 |
| build/ | 当前个人副本的生成文件、准备记录与实验输出 |

应用附带官方 `effort_controllers` 0.22.0 的源码与许可。当目标环境缺少这份控制器时，本课在个人工作区准备和编译它，再通过本课包路径加载；不要求为此改动系统 ROS 安装。首次初始化可能比以后慢，应查看平台列出的实际编译命令与错误。

个人构建使用独立库名 `libgazeboteach_l25_effort.so` 和对应的生成导出声明，准备记录保留实际库摘要，以免与系统里的同名控制器库混淆；附带的上游C++源码和原始声明作为来源材料保留。

这份源码位于应用 `resources/third_party/effort_controllers`，不是让学员重新实现一个同名控制器。后面的插件专题再集中练习插件编写；本课先使用真实上游控制器理解框架。

个人参数的基准内容是：

```json
{"p": 4.0, "d": 0.35, "target_rad": 0.45}
```

本课接受的个人配置为P在[1,6]、D在[0.15,1.5]、目标在[-0.5,0.5]，且必须是有限数值。这个目标配置范围用于常规阶跃实验；专门的 `limits` 子实验另外发送超出物理位置范围的请求，以观察真实限位链。

再到 `ros_ws/src/gazeboteach_l25/urdf/servo.urdf` 找到传动声明，核对关节名与接口：

```xml
<transmission name="hinge_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="lesson25_hinge">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="virtual_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

同一URDF中的插件选择 `gazebo_ros_control/DefaultRobotHWSim`，控制周期声明为0.002 s。世界物理步长、控制更新周期和ROS消息发布周期是三个配置，后面用真实记录分别确认。这里的周期按模拟时间解释，机器负载较高时，墙钟运行可能变慢，并不保证每2 ms墙钟执行一次控制更新。

## 启动顺序与四个不同的“成功”

开始实验后，平台先建立本课私有 ROS Master，设置模拟时钟并准备个人输入，再启动 Gazebo。准备阶段生成URDF和控制器YAML；launch将描述载入 `/lesson25/robot_description`，在已有Gazebo中spawn一次模型，由模型加载实际硬件仿真接口和控制器。`position_servo` 为主控制器，`soft_servo` 是同一关节的低 P 对照；名为 `joint_state` 的 `joint_state_controller/JointStateController` 将状态发布率配置为50 Hz。

就绪检查还会核对描述、输入指纹、实际参数、控制器类型和资源。不要把下面四层合并成一个结论：

1. 包与共享库可被发现：只能证明加载来源存在。
2. 控制器已经加载：它可能仍处于 `initialized` 或 `stopped`。
3. 控制器实际 `running`：它才参与更新，仍需检查命令是否送到正确实例。
4. 关节按目标产生真实响应：需要状态序列和物理查询证明。

保存了磁盘参数不代表已运行控制器自动读到新值。平台会比较准备记录、输入与运行状态；出现不一致时，先按提示重新加载实验，不用旧进程的效果来判断新参数。

## 先只读，确认正在控制谁

在本课操作终端中运行：

```sh
python3 lab.py status
rosservice call /lesson25/controller_manager/list_controllers "{}"
rostopic info /lesson25/joint_states
rostopic echo -n 1 /lesson25/joint_states
rostopic echo -n 1 /lesson25/position_servo/state
```

若 Gazebo 已暂停，新订阅者可能等不到一条新消息。先查看实际暂停与时钟状态，再通过平台继续或单步；不要把等待中的 echo 直接判断为插件没有加载。

`list_controllers` 的 `claimed_resources` 列出控制器声明使用的接口与关节。未运行的已加载控制器也可能列有这些声明；必须结合 `state` 判断当前运行组合。只读状态控制器与力矩控制器也不是同一种资源使用方式。

## 怎样读控制器反馈

| 字段 | 在本课中的含义 |
| --- | --- |
| set_point | 控制器使用的关节目标，已按本课位置上下限处理 |
| process_value | 本次控制更新读到的关节角度 q |
| process_value_dot | 读到的角速度 dq |
| error | 本实现计算的位置误差，局部可按目标减当前解释 |
| time_step | 这次控制更新的实际周期，单位 s |
| command | PID计算得到的原始力矩命令，尚不能当作最终施加力矩 |
| p、i、d | 本次运行控制器实际使用的增益 |

本课通过 Float64 发送纯位置目标。控制器由误差变化和更新周期计算微分项；目标突然跳变时，微分项也可能出现尖峰。随后 DefaultRobotHWSim 还会执行关节限幅。因此，即使 `state.command` 大于2 N·m，也不能仅据此判定执行器突破了2 N·m限制。

`joint_states.effort` 来自硬件仿真接口对 Gazebo 关节 effort 的读取，它不是本课另装的一只独立力矩传感器。控制器输出、接口读回和真实运动是不同证据，分析时保留它们的来源。

本上游控制器每10次 update 尝试发布一次 state；发布锁和实际更新节奏都会影响收到的序列。JointState又由状态控制器独立调度。消息时间标签、接收时刻与 Gazebo 服务查询时刻分别记录，不能把近邻样本称作严格同一物理时刻。

一条state里的 `time_step` 是控制更新周期，不是前后两条state的时间差。state只公布了部分控制周期，不能拿两条低频state的误差差值、除以一条state的time_step，就宣称重建了内部微分项。

## 第一个实验：位置阶跃

先预测：从接近零位发出正目标后，最初的角加速度应朝哪个方向？关节是否应立刻出现在目标角度？如果 `command` 已改变，`process_value` 仍暂时接近零，是否矛盾？

```sh
python3 lab.py demo --seconds 3
```

默认个人设置为 `p=4.0`、`d=0.35`、`target_rad=0.45`，积分增益固定为零。观察真实记录中的最初运动方向、峰值、末段误差与末段速度，并查看独立 Gazebo 查询是否支持这些位置变化。窗口中的转杆运动用于理解方向，数值结论来自记录。

分析记录时先约定指标。超调要沿本次目标变化的方向计算；目标为负时，不能仍把最大的正角度当峰值。末段误差宜同时查看绝对值与符号，正负误差平均后接近零不代表已经稳定。若自行比较调节时间，应明确误差带与持续时间；有限三秒记录只能说明这个观察窗口内的结果。消息采样也可能漏掉真实峰值，因此报告要说明采样来源和时间标签。

这个命令是有限实验：保存入口物理状态与控制器状态，准备起点，真正推进物理并取流，结束后恢复入口状态。演示结束后杆回到原处不代表刚才没有运动。已经推进的模拟时间和输出记录保留，不把历史伪装成没发生。

命令行 `demo` 默认测试主控制器 `position_servo`，可以显式选择对照控制器：

```sh
python3 lab.py demo --controller soft_servo --seconds 3
```

该有限实验同样在结束时恢复入口组合。未指定 `--controller` 时，先切到soft再执行demo也仍会选择默认主控制器；完整验收分别保存 `main_response` 与 `soft_response`，便于比较两条真实记录。

手动控制时可以直接发目标：

```sh
rostopic pub -1 /lesson25/position_servo/command std_msgs/Float64 "data: 0.45"
```

这条原生命令改变正在运行的主控制器目标，不替你完成有限实验的保存与恢复。若已切到 `soft_servo`，向不运行的 `position_servo` 发送消息也不能据此期待关节响应。

## PID：本课实际调整哪两项

局部可写成：

```text
e = q_target - q
tau_raw = P·e + I·积分(e dt) + D·de/dt
```

转动关节中，P 的单位可写 N·m/rad，D 可写 N·m·s/rad。本课 I=0，因此实验主要比较 PD 位置控制；使用的上游类仍是完整 PID 控制器。需要积分消除持续负载误差的背景，可回看第10课。

在尚未饱和、机械阻尼较简单的局部模型里，`J·ddq` 与控制力矩有关。提高P通常使相同位置误差对应更大的力矩；D根据误差变化影响瞬态。但执行上限、目标阶跃、采样周期和实际惯量都会改变响应，不能只凭一个P值断言系统一定更快或更稳定。

做两个独立实验，每次从基准副本开始，只改一项：

| 实验 | 唯一修改 | 需要比较的证据 |
| --- | --- | --- |
| 增益实验 | parameters.json 中 p：4.0→2.8 | 相同目标的实际增益、初段运动、峰值、末段误差与速度 |
| 目标实验 | target_rad：0.45→0.30 | 相同增益下目标、位移、方向和实际位置误差 |

保存后使用平台“重载项目”，再运行同样的 demo。先确认运行记录中的实际增益与输入一致，再比较响应；两次同时改P和目标会失去单变量解释。

## 控制器切换：一次交接同一资源

`position_servo` 与 `soft_servo` 都通过 effort 接口控制 `lesson25_hinge`。后者使用P=1.0、D=0.35，用于对照；它们可以同时被加载，却不能同时作为这一路命令资源的运行者。

```sh
python3 lab.py switch --name soft_servo
python3 lab.py status
python3 lab.py switch --name position_servo
python3 lab.py conflict
```

合法切换在同一次 controller_manager 请求中停止原控制器、启动新控制器。`conflict` 保存入口后先停用两个servo，再故意同时请求启动它们；真实请求应被拒绝，并保持两个servo均未运行。随后工具启动主控制器，再请求一个不存在的控制器，检查STRICT拒绝与主控制器仍运行。最后恢复进入子实验时的组合。

STRICT还用于检查非法控制器名字等错误；它不是“无论如何强制运行”。BEST_EFFORT允许处理请求中可处理的部分，也不授权两个互斥的力矩写入者同时控制同一关节。服务 `ok`、服务调用是否成功返回以及列表中的运行状态应分别检查。

切换由控制更新循环完成。暂停世界时，服务可能等待物理更新；本课工具会在受控推进后读回结果，并恢复相应暂停状态。不要在未知状态下无界等待，或连续重放同一切换请求来猜测有没有生效。

## 限位：目标裁剪、命令限幅和物理运动

```sh
python3 lab.py limits
```

这个子实验发送超过位置范围的请求，检查原请求、控制器实际采用的目标和真实运动边界。先预测请求大于0.8 rad时 `set_point` 应是多少，再用返回记录核对。

这三层各有职责：控制器把位置目标限制在URDF范围；硬件 effort 限幅约束写给关节的命令，并根据当前位置/速度避免继续向越界方向驱动；物理引擎仍按惯性、阻尼和约束推进。一个速度上限字段不是每个采样点都精确等于该上限的数学钳制器，关节边界附近也应按实际数值与求解容差判断。

`SimpleTransmission`在本课连接关节与控制接口，默认硬件仿真直接操作该关节。把 `mechanicalReduction` 改成某个数字，不能据此声称已加入一套真实齿轮、效率或齿隙模型；那些效果需要对应的硬件仿真或机械模型。

## 暂停与离散步进

```sh
python3 lab.py step --steps 100
```

先区分模拟时间、控制更新和ROS发布。真正的100个物理步会推进世界，控制器按实际周期更新，状态发布可能低于控制频率；不能要求100步一定收到100条state。比较世界时间差、原消息标签与关节变化，而不是根据终端输出行数猜步数。

## 验收与恢复的边界

平台“验收实验”或 `python3 verify.py` 要同时核对运行图、实际参数、真实关节响应、控制器切换/冲突与限位。结果中的 `main_response`、`soft_response`、`lifecycle`、`limits` 分别保留对应子实验。单纯出现一个 `running` 或 `/joint_states` 话题，不足以完成本课。

演示和验收会先保留入口快照，再进行受控的初始化、启停和真实运动。新接入的暂停客户端可能没有收到非latch的控制器state，工具先保存物理状态，再明确推进一小段真实模拟时间读取当前保持目标，随后恢复保存的物理状态；这段观察时间也会记入结果。

结束时停止相关servo，在暂停边界恢复关节角度与link位姿/速度，恢复入口控制器和观测到的保持目标，允许命令进入更新循环后再次恢复物理快照并读回记录，最后返回原暂停状态。旋转link的线速度设置沿用第23课的质心参考点补偿。控制器内部积分/微分历史、已经产生的消息和模拟时间不倒回；恢复物理状态不等于重建整个软件历史。

快照也保存全部三个控制器的生命周期状态。`initialized`与`stopped`虽都未运行，恢复时仍逐项对照；若实验使原本initialized的非活动servo变为stopped，工具会实际卸载并重新加载该实例，核对它消失、重新出现为initialized，再恢复原运行组合、目标和物理状态。重载会创建新的控制器/PID及订阅对象，不表示原软件实例或内部历史被倒回。

## 常见问题

| 现象 | 优先核对 |
| --- | --- |
| 控制器类型不存在 | 官方源码准备/编译、个人包路径和 pluginlib 导出，查看初始化日志 |
| 有模型但没有 controller_manager 服务 | 模型中的 gazebo_ros_control 插件、robot_description、transmission及实际加载日志 |
| 已加载但不运动 | 是否 running、命令是否发给当前控制器、世界是否暂停、目标是否接近当前角度 |
| 切换服务等待 | 世界是否有控制更新，工具的有限等待与实际服务结果 |
| 两个控制器都列出同一 joint | 声明资源可以相同，结合state核对当前运行组合和冲突返回 |
| 保存参数后仍像旧实验 | 是否重载，磁盘输入/准备记录/实际state增益是否一致 |
| 力矩command超过2 | 先分清PID原始输出与硬件限幅后的执行，不把一个字段当作独立测力 |
| 降低P仍有超调 | 比较D、惯量、阻尼、命令阶跃和实际记录，不能只按静态比例推断瞬态 |

## 独立挑战

1. 用一次状态查询说明：为何“控制器名含Position”并不能判断硬件接收的是角度还是力矩？
2. 用完整验收中的 `main_response` 与 `soft_response` 比较相同阶跃，给出峰值与末段误差，解释差异；结论附实际增益和原始记录。
3. 设计一次错误名称空间的发布，证明消息没有送到当前控制器，再修正话题并观察真实响应。
4. 用限位子实验解释“请求目标”“实际set_point”“实际关节位置”三个值为什么可能不同。

进一步阅读：[Gazebo Classic的ros_control教学](https://classic.gazebosim.org/tutorials?tut=ros_control)、[ROS 1 controller_manager](https://wiki.ros.org/controller_manager)、[上游0.22.0位置控制器](https://github.com/ros-controls/ros_controllers/blob/0.22.0/effort_controllers/src/joint_position_controller.cpp)、[DefaultRobotHWSim](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp)。
