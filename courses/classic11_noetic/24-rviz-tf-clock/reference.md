# 参考说明：Noetic TF、RViz与模拟时钟

本课使用ROS Noetic、Gazebo Classic 11、RViz1.14.26、robot_state_publisher1.15.3及tf2_ros0.7.10。数据链从Gazebo实际关节状态开始，经JointState和RSP得到TF；机器人几何、时间标签、接收时刻与图形显示分别保留证据。

## 实际JointState入口

[GazeboRosJointStatePublisher](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_joint_state_publisher.cpp)是Gazebo中的ModelPlugin。它按jointName找到实际joint，世界更新回调中读取Position(0)/GetVelocity(0)，由robotNamespace确定ROS NodeHandle，发布的相对话题名固定joint_states。

本课输出为/lesson24/joint_states，包含两个moving joint的name、position和velocity；effort未填、header.frame_id为空。tf_prefix不改变这些joint名字。空effort表示此接口未报告力矩，不能补成一次零力矩测量。

本机课包只开放joint_state_rate_hz=20/40、tf_prefix=""/"demo"两组设置。机器人保持tf_arm实体名、lesson24_shoulder与lesson24_elbow关节名。前缀只作用TF；Gazebo实体、JointState.name、ROS namespace均不由它自动重命名。Xacro展开和gz sdf -p结果保存在build/arm.urdf、build/arm.sdf，build/arm_lab.json记录输入、生成文件与转换结果的SHA。

插件用WorldSimTime调度更新，header.stamp却取ros::Time::now缓存。二者来源不同；原消息stamp、独立世界时间与接收单调时钟都应保留。发布阶段读取真实状态，不通过GUI关节滑条生成本课反馈。

[JointState.msg](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/JointState.msg)规定按name索引关联数组；转动关节使用rad、rad/s、N·m。消息各数组允许按其接口约定为空，但Noetic RSP需要name与position大小一致，才能据位置计算动态变换。

## RSP怎样发布变换

[JointStateListener](https://github.com/ros/robot_state_publisher/blob/noetic-devel/src/joint_state_listener.cpp)订阅joint_states，解析URDF/KDL结构并按输入位置触发动态TF。默认publish_frequency=50、listener_queue_size=1、ignore_timestamp=false、use_tf_static=true；本课具体配置由个人生成文件保存。

动态TF使用JointState.header.stamp，publish_frequency对应输入stamp门控，不是生产新关节数据的定时器。没有新输入时不会补帧；收到过旧输入、暂停或倒时，也应按实际时基与门控诊断。

本课显式采用publish_frequency=50.0、ignore_timestamp=false、use_tf_static=true。原生JointState的20/40Hz源低于门控上限，验收仍分别读取真实消息的stamp、authority与变换边，不用参数值替代实际频率测量。

固定边由ROS定时器触发；use_tf_static=true时该触发为一次性，再由静态广播器latch供后来的订阅者接收。时钟初值0且一直暂停可能让定时器尚未触发，初始化应先允许模拟时钟推进。

[RobotStatePublisher实现](https://github.com/ros/robot_state_publisher/blob/noetic-devel/src/robot_state_publisher.cpp)将fixed与moving segments分开。动态边stamp来自输入JointState；固定边消息stamp取发布时ROS now。static消息仍有时间字段，而StaticCache的查询语义使其区别于动态时间历史。

Noetic参数为tf_prefix，通过向上searchParam查找。ROS namespace影响节点与话题名字，frame前缀则显式应用到TransformStamped的parent/child字符串；不使用其他版本的frame_prefix参数代替本课接口。

## 变换的方向和表示

对header.frame_id=P、child_frame_id=C的变换，translation为C原点在P中的位置，rotation为C相对P的旋转：

```text
p_P = R_PC p_C + t_PC
T_W_tool = T_W_base T_base_arm1 T_arm1_arm2 T_arm2_tool
```

lookup_transform(target, source, time)取得把source坐标表达转换到target的关系。反向查询用逆变换；组合顺序不能交换。工具frame不要求独立物理body，visual/inertial原点也不会因URDF含有它们就自动成为RSP额外frame。

[TransformStamped](https://github.com/ros/common_msgs/blob/noetic-devel/geometry_msgs/msg/TransformStamped.msg)包含header、child_frame_id和Transform；[TFMessage](https://github.com/ros/geometry2/blob/noetic-devel/tf2_msgs/msg/TFMessage.msg)仅是这些变换的数组，不存在整个包共用的单一header。

四元数按x/y/z/w存储，比较姿态应承认q与−q等价，并检查归一化。[REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)给出右手系、SI单位与旋转表示背景；[Modern Robotics变换讲解](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/)补充矩阵组合和参考系基础。

[tf2 Quaternion](https://github.com/ros/geometry2/blob/noetic-devel/tf2/include/tf2/LinearMath/Quaternion.h)给出轴角、固定轴RPY、归一化与最短旋转角比较接口。绕单位轴u的旋转θ表示为(u·sin(θ/2),cos(θ/2))，每项仍按本课消息的x/y/z/w顺序解释。

## TF缓存和查询时间

tf2保存动态历史，指定时间的查询需要整条链在该时刻可用。Time(0)表示最新共同可用时间；不能把结果无条件称作“当前物理步的真值”。两个已知frame不连通与一个frame根本未知，也是不同错误。

动态历史可在已有记录间插值；超出已有时间范围会发生extrapolation错误，而非自动预测未来轨迹。缓存保留时长属于具体Buffer配置，不把不同客户端默认值一概视作相同；RViz有自己的FrameManager/Buffer。

[StaticCache](https://github.com/ros/geometry2/blob/noetic-devel/tf2/src/static_cache.cpp)保存静态关系，getData把返回stamp设为查询时间，clearList为空操作。消息原stamp、缓存查询结果stamp和实际动态观测时间因此应分别理解，不能拿静态返回的时间推断它刚被重新测量。

## 模拟时间与真正回退

/use_sim_time在私有Master上先设置，再初始化Gazebo/API、发布器、RSP及listener。/clock尚未有效时ROS时间可能为0；暂停后ROS时间不按墙钟增长。节点就绪和故障等待采用有限实际耗时，物理任务与采样调度则按实际模拟时间。

本课/clock、JointState与动态TF不靠latch向新订阅者补齐暂停前历史。暂停后新建客户端可能读到ROS初始时间0、没有新动态消息，已有客户端却保留最后记录；这两种观测不可混用。ready在此条件下只确认authority/参数/实际模型并标明延后，verify先保存入口状态再显式resume获取完整数据，不把延后检查写成动态链已通过。

RSP在下一JointState回调中比较ROS now与上一回调时间，发现倒退后清last_publish_time_。这与直接比较新旧JointState标签并非完全同一条件；收到/clock本身也不会立即替所有消费者清历史。

[Python TransformListener](https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/transform_listener.py)在TF或静态TF回调调用check_for_reset，发现ROS now回退后对同一个buffer调用clear；动态历史清空，StaticCache遵循其保留语义。观察自动恢复需要保留同一listener/Buffer的身份、记录清理与新时间段变换。

新建listener查询成功只证明新缓存建立，不能替代原listener跨重置的行为。重置后的最新查询也应核对其时间段，避免把旧缓存中的可查姿态当作已收到新TF。

reset_simulation会改变世界时间及模型初态。实验可以恢复机器人状态/pause，不能回填模拟时钟、TF动态缓存或已发布消息历史。记录应保留重置前后时间顺序与接收单调时钟，明确这是一次真实时间回退。

本课的lab.py clock-reset在暂停后核对已有旧时间段观测；新客户端若缺clock/JointState/TF，先真实推进150tick预热并记录pre_reset_observation。随后清本课两关节API任务，调用reset_simulation，再以150个真实物理tick刷新原listener，最后回到进入的pause。旧观测已具备时跳过预热。观察回退用完整阶段事件，不能只看命令首末净时间差；世界刚启动时预热和重置后的最终时间甚至可能相同。

Noetic API的世界重置与有限关节力任务队列不是同一件事；清队列避免旧任务在新时间轴再次命中其start_time。只有本课唯一关节名通过匹配检查后才清理，保留其他项目隔离。

单步命令使用本世界的gz world -m，真实世界步长1ms；lab.py step --steps 1/100要求暂停，以GetWorldProperties.sim_time确认完成。/clock回调传递与服务回读有时序差，验收容许/clock观测差在一个物理tick内；它不把命令成功退出当成世界已经推进的全部证据。

## 验收与恢复的算法上下文

verify.py建立一个Graph和一个持续的stock tf2 listener。Graph订阅/clock、/lesson24/joint_states、/tf及/tf_static；额外的AuditedBuffer.clear只记录调用及读钟，再调用原实现，不替换tf2的自动倒时判断。clear_events因此表示实际回调触发的清缓存操作，listener_identity和buffer_identity核对重置前后没有偷偷重建对象。

trace中每行有event_id、source、receive_monotonic_ns、ros_time_at_receive_ns、callerid与latching。clock和joint_state的顶层stamp_ns分别是clock值和原header；tf/tf_static顶层stamp_ns为null，每条data.transforms成员保存自己的时间与parent/child、translation、rotation。事件编号表达观测接收顺序，不是物理采样序号。Graph保留最近30000条观测行，单次报告从本次客户端生命周期取出记录。

连续窗至少取得rate+1条JointState、跨1s原ROS时间标签，检查严格递增及名义频率±5%。动态TF核对authority为/lesson24/robot_state_publisher、边集合正确，并逐条关联所接收JointState的原stamp；这是数据来源与标签关联，不是额外证明精确原生采样时刻。实际关节服务查询另保存世界时间与host包夹。

频率公式为(N−1)/(最后标签−首个标签)，与接收CLOCK_MONOTONIC间隔统计分开。[Noetic rostopic hz实现](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rostopic/src/rostopic/__init__.py)默认用接收回调中的rospy.get_rostime，--wall-time改用time.time；并不读取消息header作为其时间轴。因此本课不把默认hz输出统一称作墙钟频率，也不把它当作原header标签频率的同一测量。

几何阶段在零速度、暂停的稳定边界初始化三个姿态并真实推进150步，取得新JointState/TF后再次暂停。对照实际GetLinkState，而不是只对照同一个URDF公式的两次计算：四个物理link的world变换及base_link→tool_link均检查位置和旋转角误差小于10⁻⁵。TF前缀在ROS查询中应用，物理API仍使用实际tf_arm::link名称。

暂停阶段保持0.15s墙钟，比较世界读回、ROS当前读钟及最后/clock；步骤阶段比较1/100tick真实时间差。late_static_phase在暂停后另起订阅，检查真实连接latching="1"、RSP authority及两条固定边。reset_phase检查原clock序列存在下降、同一Buffer.clear实际增加、对象身份保持以及新时间段的动态查询；没有要求静态广播重新发一包来证明静态边可用。

恢复先保存入口的真实关节、全部物理link姿态/速度及pause。结束时先清任务，把原姿态临时置为零速度，推进150tick刷新真实发布链；再在暂停边界逐link写回原姿态/速度，并检查实际回读，最后恢复原pause。阶段间没有将原始消息stamp改成目标时间，也没有把reset前的缓存拷贝进新时间段。

速度恢复沿用前课已查明的Classic ODE接口差异：GetLinkState返回link原点的世界线速度，而SetLinkState内部SetLinearVel使用CoM线速度。用实际GetLinkProperties质心c及目标四元数得到r=R·c，设置v_COM=v_origin+ω_world×r；不改原始快照。相关实现见[ODELink](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODELink.cc)与[Noetic Gazebo ROS API](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp)。恢复后的状态与pause可核对，已经推进/回退的世界时间、旧力任务与TF历史不重建。

默认报告路径为会话目录/tf-clock-verify-UUID.json，--report指定其他路径；包含输入/生成SHA、stream、pose_phases、pause_phase、step_phases、late_static_phase、wrong_frame_probe、reset_phase、restore与trace。失败原因保留在报告中，不能以部分阶段成功代替本次passed；GUI窗口渲染另行检查。

## RViz显示与自动验证的关系

RViz RobotModel从描述参数取得URDF，并通过TF更新各link显示；TF Display负责坐标轴/名称/树，Grid为显示参照。[RobotModel源码](https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/robot_model_display.cpp)及[FrameManager](https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/frame_manager.cpp)给出实际数据路径。

Fixed Frame定义显示坐标参考；Robot Description参数与TF Prefix须与本课图一致。图形进程存在、真实TF可查询与RobotModel已经渲染，是不同检查。自动验收负责实际关节、原TF、lookup与Gazebo几何/时间对照；真实RViz窗口中的模型、坐标轴、状态与交互另作图形检查。

本课随图形实验准备RViz窗口，并支持独立重开；无图形运行保持TF/物理数据验证能力。重开窗口不等于重启Gazebo或制造一份新的状态源。

rviz_manager.py start在after_gui执行，reopen只结束并重开本会话RViz；status/stop用于检查与结束。管理器验证PID、进程启动身份与会话token，RViz继承本课环境，日志/生成配置写入会话目录lesson24-rviz.log与lesson24-rendered.rviz。模板Robot Description为/lesson24/robot_description，按tf_prefix生成Fixed Frame和RobotModel TF Prefix。进程仍存活只能作为窗口生命周期证据，RobotModel实际可见、TF轴/名称与Display状态由真实图形检查确认。

库存RViz1.14.26含[官方Noetic EOL模态提示](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/noetic_eol_dialog.cpp)。管理器仅在RViz子进程环境副本中设置官方DISABLE_ROS1_EOL_WARNINGS=1，以便课程窗口直接进入显示；宿主环境配置与渲染/数据路径保持原状，实际RobotModel仍单独作图形检查。

reopen不为补齐消息而偷偷解除暂停或单步。新RViz的动态缓存从新订阅开始建立；暂停中可先收到/tf_static，但要等之后实际JointState/TF发布才能取得动态链。这与持续打开的旧窗口仍持有末次动态变换不同。
