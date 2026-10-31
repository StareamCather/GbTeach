# 参考：ROS 1位置闭环、硬件资源和真实反馈

本说明解释本课的运行边界与记录读法，不替换个人配置。项目使用 Gazebo Classic 11、ROS Noetic 的 `gazebo_ros_control/DefaultRobotHWSim`，以及附带官方源码的 `effort_controllers` 0.22.0；不是ROS 2的控制器实现。

## 模型、接口与控制律

本课 `servo_arm` 的 `lesson25_hinge` 是固定台座上的竖直转轴，连接长0.6 m、质量1 kg的水平杆。世界重力为零，URDF质心轴惯量0.0303加平行轴项1×0.3²，名义轴惯量为0.1203 kg·m²；URDF位置范围±0.8 rad、effort2 N·m、速度规格4 rad/s。机器人实体与ROS名称空间 `/lesson25` 是两套标识。

transmission声明 `hardware_interface/EffortJointInterface`。DefaultRobotHWSim查找对应Gazebo joint，注册状态句柄和effort命令句柄。控制器以这些句柄读取状态、写输出，控制器类型不能只凭目标话题的数据类型判断。

本课默认 `position_servo` 为 `effort_controllers/JointPositionController`，p=4.0、i=0、d=0.35。`soft_servo`是同接口、同关节、p=1.0、i=0、d=0.35的对照。两者调节位置而输出力矩；实例名 `joint_state` 的只读状态控制器独立发布JointState。Gazebo插件声明控制周期0.002 s，状态控制器声明发布率50 Hz；最终按实际时序记录解释。

离线准备保留上游C++及原始XML来源，个人构建生成独立的 `libgazeboteach_l25_effort.so` 和指向它的导出声明，并记录实际库SHA。控制器的pluginlib类型名保持上游类名，私有库文件名用于区分实际加载来源。

单关节、水平轴向转动的局部近似可写为 `J·ddq + b·dq = tau`。在未触发约束和限幅、目标保持不变时，PD反馈形成与误差和角速度相关的力矩。这个近似适合解释方向和瞬态，但不会替代Gazebo实际惯量、阻尼、采样和约束求解。

## 每次控制更新的实际顺序

1. Gazebo已经具有上一步的物理状态。
2. 硬件仿真读关节位置、速度及effort，更新状态接口。
3. controller_manager调用正在运行的控制器；位置控制器读取最新目标，将有限关节的目标裁剪到URDF范围。
4. 对本课有限转动关节计算允许范围内的位置误差，再按实际period执行PID。Float64命令只提供位置，因此微分从误差序列计算。
5. 控制器把原始effort写入命令句柄，并按自己的发布节奏尝试输出state。
6. 硬件层执行effort关节限制，调用Gazebo `SetForce`；后续物理步产生真实运动。

这里用到的是上游控制器和真实硬件仿真路径。实验程序不会逐帧设置角度来制造阶跃轨迹。初始化实验起点与恢复入口状态则属于明确标注的边界操作，应与控制响应区间分开。

DefaultRobotHWSim的默认effort路径直接作用受控关节。URDF中的SimpleTransmission让接口得以匹配，不等于模型中已经有真实电机转子、齿轮效率或柔性传动；本课不以mechanicalReduction数字宣称这些物理效果。

## 读state时先核对实现

上游0.22.0位置控制器在发送state之前已处理位置目标，所以 `set_point` 是实际采用目标。有限转动关节的 `error` 使用其角度差实现；在本课小角度范围内可按 `set_point - process_value` 理解。不要只凭通用消息中一句误差注释猜本控制器的符号。

`state.command`是PID计算值，发布发生在硬件限幅之外。它不保证落在±2 N·m内，也不应被改写为一个看上去满足限幅的数。DefaultRobotHWSim之后把执行命令交给EffortJointSaturationHandle等限制接口，最后调用SetForce。

effort关节饱和接口会限制effort幅度，并在检测到位置/速度越界时限制继续向越界方向驱动的effort；它不是把物理速度瞬间设置为某值。惯性和物理约束仍影响下一步运动，应从实际位置、速度与边界记录判断。

JointState的effort来源于硬件仿真读取的Gazebo `GetForce(0)`。它是这一接口的读回值，不能自动解释为一只独立F/T传感器测得的总约束反力。本课原始PID输出、接口effort和物理响应分别记录，保留各自含义。

控制器state在每10次update的发布机会中尝试获取锁；JointState由另一状态控制器调度。实际消息可能并非逐控制周期一包。原消息stamp、接收单调时间与独立物理查询的前后时间要分别保留，近邻交叉检查不能声称严格同步。

`state.time_step`对应单次内部控制更新，前后两条state通常跨过多次更新。用低频state误差直接除以time_step重建D项，会混用两个时间尺度；物理响应还受目标裁剪与执行限幅影响。核对数据时保留原字段，不用计算后的“应该力矩”覆盖原始command。

## 加载、启动与资源冲突

控制器加载通过pluginlib创建实例、读取参数并取得接口句柄。加载成功之后，可能处于initialized；停止过的实例可能为stopped。只有running实例参与正常控制更新。合法状态以本机 `list_controllers` 的实际字符串为准。

该服务会把已加载实例的 `claimed_resources` 都列出，包括未运行的实例。它表达控制器需要哪些资源，不能独立证明该实例正在驱动关节。本课两个位置控制器都声明同一effort关节，运行时只能选择允许的一组；状态发布器没有同样的effort写入冲突。

合法切换应在一次SwitchController请求中列出旧控制器的停止与新控制器的启动。STRICT会拒绝非法名称等不满足请求条件的情况；资源冲突也应检查服务的ok与前后运行组合。BEST_EFFORT的成功含义不同，不是“允许忽略硬件冲突”。

本课 `conflict` 的第一项先停止两个servo，再同时请求启动它们，拒绝后应无servo运行；第二项先运行主控制器，再以非法名字请求替换，拒绝后主控制器应保持运行。这两项有不同的前置状态，不能把“拒绝后都应保留主控制器”作为统一断言。

控制器切换完成需要更新循环。暂停世界后仅发服务，可能等不到更新；实验工具通过有限、受控的物理推进和读回确认完成，并处理原暂停状态。服务返回成功、控制器实际running以及关节响应仍是三个不同观察层次。

位置控制器在starting时以当前关节角度初始化目标并重置PID状态。切换后旧实例收到过的命令不应被无条件当作新实例的现行目标；应看实际set_point和本课工具的显式目标操作。

## 单变量实验怎样解释

基准配置为 p4.0/d0.35/target0.45。只改p为2.8时，先核对实际运行state.p，随后比较相同目标的完整响应；只改target为0.30时则保持增益不变。观测至少包含运动方向、峰值、末段位置误差和末段速度，不能只拿一张最终截图判断调参效果。

CLI的demo默认使用position_servo，也可用 `--controller soft_servo` 显式选择对照；内部有限实验会在结束时恢复进入时的运行组合。完整verify依次调用主控制器和soft_servo的真实响应实验，结果分别位于main_response与soft_response，随后还有生命周期与限位记录。用这两条记录比较控制器时，应一并保留入口恢复记录与实际增益。

I固定为零，因此本课主要比较PD闭环。P改变位置误差对应的力矩，D影响误差变化带来的输出；阶跃目标还可能产生微分尖峰。触发执行限幅后，增大P不保证按线性比例加快运动。不要把未触发饱和的理想线性结论套到整个大幅运动。

修改磁盘参数后要重新加载。准备阶段保存输入指纹和生成配置，运行阶段还核对实际参数/增益；保存但未重载的实验应先报告不一致，而不是继续用旧控制器生成“新参数结果”。

## 边界操作与恢复

有限演示和验收保存入口物理快照、当前运行控制器、需要恢复的目标及暂停状态，再明确停止控制、准备起点、启动控制、真正推进并采集。控制响应区间不通过角度设置服务强制轨迹。

暂停新客户端缺少当前保持目标时，snapshot先冻结并保存q、link姿态和twist，随后真实推进一小段模拟时间，读取新controller state中的裁剪后目标，再暂停恢复之前的物理快照。这属于已记录的目标观察步骤，不称作完全只读、零时间影响的快照。

结束时重新停止相关控制器，在暂停边界恢复关节物理状态及原运行组合/目标。目标送入更新循环后，再次恢复物理快照、保存读回，最后按入口暂停状态返回。恢复角度与速度时遵守Gazebo API的参考点和固定约束含义：GetLinkState原点速度写回SetLinkState之前，按实际COM与姿态使用 `v_COM = v_origin + omega × (R·COM)` 换算；原始快照与读回不改写。不把模型整体平移当作一个关节的重置。

快照中的`controller_states`同时保存三个控制器的实际枚举。initialized与stopped都不运行，但不是相同生命周期状态；对实验后从initialized变成stopped的原非活动servo，恢复先停用servo，再通过真实UnloadController/LoadController验证该实例消失、重新以initialized出现，随后恢复入口运行组合、保持目标和最终暂停边界的物理快照。结果用`reinitialized_controllers`记录重建者，`controllers_after`记录读回，并逐项核对三个原状态。重建改变控制器、PID与订阅对象的身份，不宣称原对象或内部历史恢复。

模拟时间不会倒回，输出消息仍保留。PID实例的内部微分/积分历史、通信队列和过去的控制更新也不宣称完整恢复。物理快照恢复成功与内部软件历史重建是不同命题。

## 按实际证据回答挑战

- 控制器类型回答“调节什么量”，硬件接口回答“写哪种命令”。本课调节位置、写effort，位置不应在收到命令后瞬间跳到目标。
- `/lesson25`下的资源、类型和运行状态需与`servo_arm::lesson25_hinge`的实际查询共同核对。错误名称空间中的消息即使发布成功，也不证明被当前控制器消费。
- 位置超范围请求、裁剪后的set_point、实际关节角度可以不同。分别读取三层记录，再解释差别，不能把请求值直接写进“真实位置”。
- 限幅证据不能要求所有原始PID输出先天满足执行上限；应把控制器原值、硬件限制实现与物理响应分开。

## 官方实现与先修链接

- [Gazebo Classic ros_control教程](https://classic.gazebosim.org/tutorials?tut=ros_control)：插件、传动和硬件仿真的整体关系。
- [effort_controllers 0.22.0 JointPositionController](https://github.com/ros-controls/ros_controllers/blob/0.22.0/effort_controllers/src/joint_position_controller.cpp)：目标限位、误差、PID、state发布和starting。
- [DefaultRobotHWSim](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp)：真实状态读取、接口注册、限幅与SetForce。
- [EffortJointSaturationHandle](https://github.com/ros-controls/ros_control/blob/noetic-devel/joint_limits_interface/include/joint_limits_interface/joint_limits_interface.h)：effort/位置/速度限制的具体行为。
- [ROS 1 controller_manager](https://github.com/ros-controls/ros_control/blob/noetic-devel/controller_manager/src/controller_manager.cpp)及[SwitchController.srv](https://github.com/ros-controls/ros_control/blob/noetic-devel/controller_manager_msgs/srv/SwitchController.srv)：运行状态、资源与切换。
- [硬件接口基础](https://wiki.ros.org/hardware_interface)、[pluginlib](https://wiki.ros.org/pluginlib)：不了解句柄、共享库与插件导出时，可从这里补充。
