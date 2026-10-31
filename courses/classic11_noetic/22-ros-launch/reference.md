# 参考说明：Noetic启动组织与有限力任务

本课使用ROS 1 Noetic、Gazebo Classic 11及既有Gazebo ROS API。它介绍计算图、名称解析、包环境和launch生命周期；具体机器人描述、TF及控制器在后续课程展开。课程是否通过由真实节点/消息/服务与物理行为共同判断。

## Master与实际数据路径

[master_api.py](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/src/rosmaster/master_api.py)的registerPublisher、registerSubscriber登记名称、类型和节点XML-RPC端点；订阅方得到当前发布者列表，也可收到publisherUpdate通知。lookupService返回服务端的ROSRPC URI。

[rospy tcpros_pubsub.py](https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/impl/tcpros_pubsub.py)建立发布/订阅之间的TCP/IP连接。Master帮助发现，不把所有主题负载转发给订阅者。参数服务器处理配置键值，不能因此把高频状态主题等同于参数服务器里的一个变量。

```text
名称发现：节点 ↔ Master登记/查询
主题负载：发布节点 → 订阅节点
服务负载：客户端 → 服务端 → 客户端
参数配置：节点 ↔ Parameter Server
```

因此Master可达、节点注册、主题有连接、收到新消息、物理响应应分别检查。本课的平台已经启动私有Master和Gazebo，launch只加入本课节点；ROS_MASTER_URI必须继承当前实验，不另起core/world替换平台生命周期。

## package查找与环境

[rospkg环境文档](https://github.com/ros-infrastructure/rospkg/blob/master/doc/environment.rst)说明ROS_PACKAGE_PATH是包搜索路径，同名包按路径先后选择。本课启动入口将个人源包所在目录放入本次进程环境，节点应实际来自个人练习副本，不能悄悄运行系统另一份同名包。

[catkin环境文档](https://github.com/ros/catkin/blob/noetic-devel/doc/user_guide/environment.rst)说明setup.bash/setup.sh及环境hook的作用。source改变当前shell与后续子进程；改变工作区后，旧节点、旧终端不会自动重新加载新环境。需要构建的包还必须构建生成相应产物，ROS_PACKAGE_PATH本身不负责编译。

本课使用标准消息与纯Python源包，没有自定义消息生成或C++目标，所以不以catkin_make作为运行前提。节点仍需要正确shebang与可执行位；roslaunch工作目录默认属于ROS_HOME语义，不能假设它就是个人课程目录。节点读取资源应按明确路径/包路径定位，不能依赖偶然的cwd。

本课声明ros_package_paths包含ros_ws/src，由应用Session为startup、命令和真实终端统一prepend个人路径。ros_lab.sh/ros_entry.py另显式建立相同overlay并隔离本次ROS_HOME/log；子进程没有向父shell回写环境的机制。rospack find应返回当前个人gazeboteach_l22路径。

| 文件 | 本课作用 |
| --- | --- |
| launch_settings.json | 保存force_n=2/4、robot_ns=cart_a/cart_b |
| ros_ws/src/gazeboteach_l22/package.xml | 最小源包与标准依赖描述 |
| launch/lab.launch | 两个arg默认值与include转交 |
| launch/nodes.launch | group、两节点、私有param、remap和required |
| scripts/pulse_driver.py | Trigger服务接纳有限力请求，调用实际Gazebo API |
| scripts/state_observer.py | 按模型名选取真实ModelStates，输出JSON反馈 |
| scripts/common.py | 有限服务等待、实际节点身份与字段转换 |
| prepare.py、config.py | 输入/源指纹、包解析与支持参数 |
| ros_lab.sh、ros_entry.py | 预览包/节点/参数及正式exec启动入口 |
| lab.py | 查询真实图、状态与本课pulse/cancel/reset入口 |
| verify.py | 空闲、加速、匀速、反馈一致性与恢复验证 |

表内launch与scripts路径相对gazeboteach_l22包；world、settings与准备记录属于个人课程目录。准备会解析真实rospack路径与roslaunch --nodes，但静态解析成功不等于节点已经进入运行图。

## launch输入、参数与名称

Noetic的[xmlloader.py](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/xmlloader.py)分别处理arg、param、node、include与remap；[core.py](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/core.py)保存节点namespace、remap、required等配置。

| 对象 | 解析/运行位置 | 常见误解 |
| --- | --- | --- |
| arg | launch解析上下文 | 只声明arg就能由rosparam get读取 |
| node内param | 节点私有命名空间的参数键 | 参数放在节点外层却仍以~名读取 |
| group namespace | 展开相对节点/接口名 | 以/开头的全局名也自动套前缀 |
| remap | 节点ROS名称解析 | 自动转换消息类型、改参数值或改Gazebo实体名 |
| include | 引入另一launch描述并传入所需arg | 任意arg会自动变成被包含文件的参数服务器值 |

参数需要显式从arg写入param；数值类型由param的type等规则决定，节点自己的读取/校验仍重要。rosparam set影响服务器值，节点是否立即采用新值取决于程序何时读取。

实际nodes.launch在driver内部用param name=force_n、type=double、value=$(arg force_n)写入私有键。入口按保存设置显式追加force_n:=2/4和robot_ns:=cart_a/cart_b，覆盖lab.launch的default，再由include传入nodes.launch。driver启动时缓存这些私有参数，未实现自动热更新。

ros_lab.sh nodes调用--nodes，params调用--dump-params，两者只是静态解析；可带force_n:=4或robot_ns:=cart_b预览，不改变保存输入与现存图。start只接受已保存设置，需要当前平台session和已存在Master，并以--wait启动真实roslaunch；正式运行由平台管理，不重复启动同名图。

名称规则可查[rosgraph.names.resolve_name](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosgraph/src/rosgraph/names.py)：全局名以/开头；私有~name相对节点完整名；普通相对名相对节点namespace。本课逻辑服务pulse经remap成为start_pulse，Gazebo服务和ModelStates使用明确全局目标。

namespace变化只改变ROS图中的名称。Gazebo实体使用实际model::link标识；若两个控制节点仍指向同一body，即使ROS名字不同也没有实现物理隔离。本课每次由平台管理一个正式启动图，避免同时用另一套同名launch接管它。

## 启动条件与进程生命周期

roslaunch可以设置参数并启动进程，但XML节点顺序不提供“服务已可调用”“订阅已经连通”或“反馈已更新”的保证。就绪检查应针对实际Master、期望节点、接口类型/提供者和新鲜消息，并使用有限实际时间超时。

[roslaunch入口](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/__init__.py)的--wait等待已有Master；它不是启动第二个roscore的命令。等待本身也需要由本课维护入口给出有限失败边界，错误ROS_MASTER_URI不应让界面一直无状态等待。

required=true监控的是进程存活：节点退出，包括正常退出0，都会触发本次roslaunch关闭其余受管节点。它不表示依赖就绪，也不直接关闭由平台独立管理的Gazebo/Master。required与respawn不能同时true，详见[Node数据结构校验](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/core.py)。

实际初始化先建立平台私有Master，再由prepare设置该Master的/use_sim_time=true、核对包路径/执行权限/解析结果并保存build/launch_lab.json；之后才初始化gzserver和ROS API。lab.launch也声明同一布尔参数。after_start只做world-ready，startup启动真实roslaunch，after_startup再检查launch图和新反馈。ROS API及两个节点初始化前的时钟选择，不能靠节点启动后只看一遍参数值来替代。

ready核对真实Master与节点XML-RPC身份、PID、session token、个人workspace、输入与节点源SHA、私有参数及缓存值、服务提供者、订阅/发布关系和remap结果。运行状态还需要至少三条在本次client创建后收到的新反馈。pulse/cancel/reset以及验收可在暂停状态先做图检查，此时会明确标记反馈尚待继续；普通ready/observe需要运行中的新反馈。

服务调用有响应不等于物理动作已完成；通信超时也不自动撤销一个可能已进入Gazebo调度队列的任务。本课需要分开记录接纳、任务时间与实际反馈，避免无依据自动重发物理命令。

## 模拟时钟与无header的状态

/use_sim_time使ROS时间采用/clock；clock未建立前可能读到0，暂停后不持续推进。物理任务duration使用模拟时间，节点就绪及失败等待使用有限墙钟时间。这与第21课区分的时基一致，不能将回调接收时间伪称原生物理采样时间。

初始化依据见[rospy simtime.init_simtime](https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/impl/simtime.py)与[roscpp start](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/init.cpp)：初始化阶段读取/use_sim_time，启用时订阅/clock并先置零。参数值是启动条件；改变服务器键值并不等于让所有旧节点自动重新初始化时钟。

[ModelStates.msg](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/msg/ModelStates.msg)包含并行的name、pose、twist数组，按同一索引对应模型，定义中没有header。observer必须按实际模型名定位，不固定取第几个数组元素；它附加的接收ROS时间/序号只是观察字段。

最终物理效果还可通过独立Gazebo状态服务查询交叉检查。主题消息与服务查询都来自实际世界，但回调阶段不同，不宣称它们严格同一采样时刻。TF与URDF不由改主题名称自动建立。

GetModelState服务虽然带header，其Noetic API实现填写的是ros::Time::now，不应直接改称精确native采样stamp。本课将真实GetModelState查询夹在前后GetWorldProperties世界时间之间，同时保留原service_header；用包夹时段约束观测，而不伪造同刻真值。

observer_receipt_monotonic_ns和observer_receipt_ros_time_ns是接收时刻；source_stamp_ns保持null、source_has_header=false。source_message_sequence是本节点原始接收回调计数，输出受50墙钟Hz上限节流，可能跳号；它不意味着消息丢失，也不是原生帧序号。feedback/status的latch使新订阅者可先收到旧值，freshness需要时间界和后续更新判断。

反馈state中的position单位为m，linear为m/s，angular为rad/s；orientation按x、y、z、w保存四元数。它们从实际ModelStates所选模型复制，采用该API的world状态语义。client_receipt_monotonic_ns是进一步接收JSON的客户端墙钟读数，与observer收到原ModelStates的时刻也要区分。

## ApplyBodyWrench有限任务

[ApplyBodyWrench.srv](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/srv/ApplyBodyWrench.srv)定义body_name、reference_frame、reference_point、wrench、start_time与duration，并返回success/status_message。body_name指实际Gazebo scoped link，不是ROS node或namespace。

本课固定world方向、reference_point零和torque零，使用有限模拟持续时间。预测只涉及平移：a=F/m、Δv≈FT/m。有限力到期不意味着速度变零；无阻尼/无其他水平力时应继续匀速。这是实际动力学对照，不根据命令积分合成反馈。

[gazebo_ros_api_plugin.cpp](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp)的applyBodyWrench将任务加入调度队列，wrenchBodySchedulerSlot在模拟时间达到开始值后，每物理步设置world力/力矩，结束后删除有限任务。当前判据包含开始与结束两端，所以实际有效时长受步长网格约束，可能比理想T多一tick；负duration才表示持续任务，本课不采用它。

该调度器对同一body的重叠任务使用SetForce/SetTorque路径，不能假设多个任务自动作矢量求和；本课driver应拒绝重叠脉冲。服务成功表示任务接口接纳，真实速度变化与到期后的匀速段用于检查实际行为。

ClearBodyWrenches使用[BodyRequest.srv](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/srv/BodyRequest.srv)，请求是body_name、响应为空；实现按确切scoped body名字清除队列任务。空响应不报告删除数，也不清除模型速度。维护时仅针对本课body，后续实际状态与到期行为仍需验证。

## 操作入口与保存边界

| 命令 | 实际行为 |
| --- | --- |
| python3 lab.py ready | 检查当前实际启动图、参数、身份与新反馈 |
| python3 lab.py observe --seconds 2 | 就绪后采集两墙钟秒反馈，支持0.5～5秒及--report PATH |
| python3 lab.py pulse | 按当前namespace调用Trigger，提交一次0.1秒lead、0.3秒duration的有限任务 |
| python3 lab.py cancel | 请求清除launch_slider::body的任务队列，保留速度 |
| python3 lab.py reset | 暂停、清本课队列、设置world(0,0,1)/单位姿态/零速度，再恢复原pause |
| python3 lab.py config --force 4 | 保存force_n，只支持2或4；重载后生效 |
| python3 lab.py config --namespace cart_b | 保存robot_ns，只支持cart_a或cart_b；重载后生效 |
| python3 verify.py --report PATH | 运行真实完整验收并保存原始观测与评价 |

配置编辑只改对应launch_settings.json标量。准备记录包含world、settings与节点/launch等源SHA；已保存但未重新准备的输入会被拒绝，避免把新文件与旧运行节点混用。nodes/params静态预览仍可展示保存配置或显式预览arg，不能据此宣称live图已经切换。

pulse的Trigger空请求不承载force；success/message表示driver处理结果。任务记录含pulse_id、command_sequence、force_n、requested_start_time_ns、requested_end_time_ns等字段。request_monotonic_ns是在实际API调用返回后填写的读钟，不能解释成精确网络发送瞬间。driver在物理RPC前预留busy窗口，拒绝同body重叠任务；通信结果不明时也不盲目重发可能已入队的物理请求。

driver_status的scheduled是最后任务记录，任务到期不会自动将此状态改写成实时力测量。reset明确改变模型状态；cancel只清队列。两者都以当前私有世界的确切body名工作。

## 验收算法与真实证据

验收先检查输入、实际启动图和刚体质量1 kg/禁重力/物理步长0.001秒。保存进入时的pause及真实pose/twist后，暂停并清本课力任务，将试块复位静止；随后按实际模拟时间完成以下阶段：

1. **空闲基线。** 至少0.25模拟秒，取得至少十次独立状态查询和三条新反馈，确认launch启动本身保持试块静止。
2. **有限请求。** 暂停时通过真实driver提交一次任务；紧接着第二次请求必须因已有任务被拒绝。随后继续模拟，保存请求、状态与原始反馈。
3. **加速窗口。** 从请求start/end各剔除30 ms边界，至少十次实际状态、至少0.15模拟秒，用WorldProperties包夹中点为时间坐标，对真实vx做线性拟合。斜率需在F/m的5%以内。
4. **匀速窗口。** 从end后50 ms起选择状态，至少十次实际状态、至少0.3模拟秒。拟合加速度绝对值不超过0.05 m/s²，速度相对FT/m误差不超过0.02 m/s；实际位移与有限力模型差不超过0.01 m。该容差包含当前调度端点的一个tick效应，而不是把连续公式当作无限精度模拟实现。
5. **独立一致性。** 只在匀速查询覆盖的主机时段内，将observer接收墙钟与独立服务查询的主机包夹中点作近邻比较，至少五对、间隔不超过50 ms；位置误差不超过0.05 m，速度误差不超过0.02 m/s。这是接收邻近的一致性检查，不是相同native采样stamp的同步证据。
6. **恢复。** 再暂停、清本课任务并恢复进入时pose/twist，然后恢复进入时pause；每步尝试并记录清理错误。四元数q与−q按同一姿态比较。

验收请求与测量分开保存。状态查询保留前后世界/主机时间、原service_header和真实状态；ROS反馈保留header缺失事实、observer及client接收时刻。baseline/active/coast统计从这些实际值计算，期望公式只用作比较，不作为反馈输入。

报告默认位于session的ros-launch-verify-UUID.json，--report可覆盖位置；trace保留每次状态查询与反馈，phases保存选择窗口、拟合结果与配对误差，restore保存前后状态及清理结果。平台为整体验收设置120秒上限；正常就绪和单次服务也各有有限超时。

恢复不重建验收前的排队力任务，不回退driver任务历史或模拟时间。若进入时刚体已经运动，恢复的是当时快照的pose/twist；继续后由物理世界重新演化。运行结论以实际报告和图形检查记录为准，不能以脚本存在或静态包解析成功代替运行验收。
