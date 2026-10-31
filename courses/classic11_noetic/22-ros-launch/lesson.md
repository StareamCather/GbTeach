# ROS启动：把节点、名称、参数和仿真连接起来

前面的传感器课程已经替你准备了ROS观察通道。本课开始集中理解ROS Noetic怎样组织一个实验：哪些程序在运行、数据发往哪个名字、参数从哪里来、服务是否就绪，以及启动成功之后怎样确认真实物体确实响应。

项目只用一个短力脉冲和实际模型状态，让通信图与启动流程容易看清。课程提供节点程序；学习重点是读懂并修改launch组织、参数和名称，不要求先写控制器、URDF或完整机器人系统。

## 六个概念先放到各自位置

| 概念 | 本课含义 | 检查入口 |
| --- | --- | --- |
| node，节点 | ROS图中的计算单元；本课每个Python进程对应一个节点 | rosnode list/info |
| topic，主题 | 有类型的命名消息流，发布者与订阅者通过它连接 | rostopic list/info/type/echo |
| service，服务 | 有请求与响应的命名接口，由客户端调用服务端 | rosservice list/info/type/call |
| parameter，参数 | 参数服务器上的命名配置值 | rosparam list/get/set |
| namespace，命名空间 | 给名称分组、决定相对名字如何展开 | 看最终完整名称 |
| launch argument，启动参数 | launch解析时使用的输入，可用于展开节点名、参数或包含文件 | roslaunch的name:=value |

节点是ROS通信身份，进程是操作系统实体；本课采用一节点一Python进程的简单结构。主题适合持续状态，服务适合明确的请求/响应，参数适合配置；把它们都叫作“一个变量”会掩盖生命周期和通信方式的差别。

同名接口还必须类型相容。改一个主题名字不会自动转换消息结构、单位或坐标系；服务名存在也不证明你调用的就是预期服务端。

## Master负责发现，不是所有消息的中转站

本课应用已经创建私有ROS Master、Gazebo服务端与ROS API。节点向Master注册自己的名称、发布/订阅或服务信息；Master帮助找到通信对端，通常的TCPROS主题数据随后在发布者与订阅者之间点对点传输。

```text
driver / observer / Gazebo ROS API
  └─ 向Master登记、查询名称与端点

实际状态：Gazebo发布者 ──主题数据──→ observer
实际请求：driver ──服务请求──→ Gazebo ROS API ──响应──→ driver
配置读取：节点 ──参数查询──→ Parameter Server
```

所以能运行rosnode list，不等于所有数据连接都已成功。节点可能注册了错误名字、类型不匹配、对端尚未就绪，或者公布了无法连接的地址。先检查图，再看真实消息与服务响应，最后看物理结果。

本课的launch只启动课程节点，复用平台已存在的私有Master和Gazebo；不要再开一套roscore或把启动新Gazebo世界的通用launch包含进来。操作终端继承本课的ROS_MASTER_URI，不把11311当成每个实验都固定使用的端口。

这一发现与点对点数据路径可查[Noetic Master API](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/src/rosmaster/master_api.py)与[rospy TCPROS实现](https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/impl/tcpros_pubsub.py)。它们是接口依据，学员不必先读完实现源码才能完成实验。

## 本课实际连接的图

默认namespace为/lesson22/cart_a。driver提供start_pulse与cancel_pulse服务，调用已有Gazebo API；observer订阅真实ModelStates，选出launch_slider，再发布本课JSON反馈。

图中还会出现/gazebo、/rosout及临时检查客户端；“两个课程节点”指本课launch管理的driver和observer。start_pulse使用std_srvs/Trigger，空请求中没有force字段；力值来自driver启动时读取的私有参数。反馈使用std_msgs/String承载本课JSON，并保留真实模型状态的来源说明。

```text
实验工具 → /lesson22/cart_a/start_pulse [Trigger] → driver
driver → /gazebo/apply_body_wrench [ApplyBodyWrench] → Gazebo实际施力
driver → /gazebo/clear_body_wrenches [BodyRequest] → 清本课body任务
Gazebo → /gazebo/model_states [ModelStates] → observer
observer → /lesson22/cart_a/state_feedback [String中的JSON] → 实验工具
```

Gazebo模型名固定为launch_slider，受力link为launch_slider::body；它与ROS节点driver不是同一种名字。蓝块是质量1 kg、边长0.2 m的自由刚体，初始位于world(0,0,1)，禁重力与速度衰减；黄色尺是视觉参考，不承担滑轨约束。作用力沿world+X。

```sh
rosnode info /lesson22/cart_a/driver
rosnode info /lesson22/cart_a/observer
rosservice type /lesson22/cart_a/start_pulse
rostopic info /lesson22/cart_a/state_feedback
rosparam get /lesson22/cart_a/driver/force_n
```

这是默认cart_a的运行名字；换成cart_b并重载后，应查询对应的新路径。不要在平台已启动这两个节点时，再手工启动一份相同完整节点名的launch。

## package、工作区和source各做什么

**package（功能包）**组织节点、launch、依赖描述和资源。**catkin工作区**用于组织并构建ROS 1包，构建后可生成devel/setup.bash等环境入口。执行source会改变当前shell及以后从它启动的子进程环境，不会追溯修改已经启动的节点或另一个终端。

ROS_PACKAGE_PATH告诉包查找工具到哪里找包；它不是Python的导入路径，也不会替你编译C++、生成消息或给脚本添加可执行权限。同名包的搜索顺序也重要：rospack find显示的是这一终端实际找到的那份包。

本课包名是gazeboteach_l22，位于个人练习的ros_ws/src。平台通过课程声明将该路径统一放入本次session环境，所以启动进程、命令和真实操作终端使用同一份包；不是让一个子shell修改环境后再“传回”父进程。ros_lab.sh也显式准备这份环境，供预览工具使用。

本课采用现有标准消息与纯Python源包，由平台准备个人包路径与可执行节点。是否需要catkin构建，要看包是否有编译或生成产物；不能因为看见ROS包就一律运行catkin_make，也不能对需要构建的包只改ROS_PACKAGE_PATH便认为已经完成构建。

```sh
rospack find gazeboteach_l22
bash ros_lab.sh nodes
bash ros_lab.sh params
```

nodes调用真实roslaunch --nodes，params调用--dump-params，分别预览解析出的完整节点名和参数。它们不启动节点，也不修改当前参数服务器；与rosnode list、rosparam get看到的运行状态要分开。

```sh
echo "$ROS_MASTER_URI"
rosnode list
rostopic list
rosservice list
rosparam get /use_sim_time
```

不了解环境入口，可以看[catkin环境文件说明](https://github.com/ros/catkin/blob/noetic-devel/doc/user_guide/environment.rst)与[rospkg环境变量说明](https://github.com/ros-infrastructure/rospkg/blob/master/doc/environment.rst)。本课由应用完成初始化，先用本课终端验证实际环境，再排查配置。

## namespace、私有参数与remap

ROS名称有三种常见写法：以/开头是全局名；不加前缀的是相对名，按节点所在命名空间展开；以~开头的是私有名，按该节点的完整名字展开。

例如节点完整名为/lesson22/cart_a/driver时，相对服务pulse通常落到/lesson22/cart_a/pulse，私有参数~force_n落到/lesson22/cart_a/driver/force_n。全局/gazebo/model_states继续指向这个全局名，不会因为节点换了namespace就自动多加一层前缀。

**remap（名称重映射）**改变节点解析后的接口名称。程序可以使用逻辑名字pulse，而launch将它映射成start_pulse；程序逻辑不用硬编码整个最终路径。它不改变参数值或消息类型，也不把Gazebo的model::link实体名改成另一个机器人。ROS命名空间分组与不同Master的实验隔离是两件事。

修改namespace后，预期变化的是节点、相对topic/service与私有param的完整路径。物理模型是否换了一个，要另看Gazebo实体和实际状态；仅改ROS名字不会创建第二台机器人。

## launch arg不是自动存在的ROS参数

force_n:=4首先是launch解析上下文里的一个输入。只有launch将它明确写入param，节点才能通过参数服务器读取对应值；如果只声明arg，rosparam get并不会凭空找到同名键。

本课真实输入链是：launch_settings.json → ros_entry.py显式传入robot_ns/force_n启动arg → lab.launch通过include把arg交给nodes.launch → group展开namespace → driver内的param把force_n以double写入私有键 → driver启动时读取~force_n。显式命令行arg会覆盖lab.launch中的default，单改default不一定改变平台实际运行值。

例如`bash ros_lab.sh params force_n:=4`只预览这个解析变式，不修改保存设置、当前节点或Parameter Server。要让正式实验采用4 N，需要保存设置并重载，再查实际私有参数与节点状态。

node内部的param通常是该节点的私有参数，节点用~force_n读取。类型也要明确：字符串“4”、整数4与浮点4.0不是在所有程序中都可互换；力的单位为N，持续时间为模拟秒，不能用单位猜类型。

rosparam set只修改参数服务器的当前值。一个节点若仅在初始化时读取参数，修改服务器并不会让它自动重读；另一些程序会主动重读或提供动态配置接口。需要看当前节点的实现，而不是假设ROS替所有程序完成热更新。本课保存参数与重载的实际入口由课程工具提供。

本课driver的force、duration、lead和body在启动时读取一次。运行中仅改参数服务器，不会自动改掉已经缓存的力值；不要把rosparam get的新值与当前driver实际使用的值混为一谈。

本课工具还会比较参数服务器、保存配置与driver缓存。若临时用rosparam set造成三者不一致，就绪检查会拒绝继续；通过应用重载，可让运行实例重新采用已保存配置。

打开个人包的launch/nodes.launch，可以把名称与配置对应到实际片段：

```xml
<node pkg="gazeboteach_l22" type="pulse_driver.py"
      name="driver" output="screen" required="true">
  <param name="force_n" value="$(arg force_n)" type="double"/>
  <remap from="pulse" to="start_pulse"/>
  <remap from="apply_wrench" to="/gazebo/apply_body_wrench"/>
  <!-- 其余字段见完整文件 -->
</node>
```

外层group决定namespace；param将解析输入写入该节点的私有键；remap将程序内部名字连接到最终接口。先用静态预览确认展开结果，再用实际图、实际参数和新反馈检查运行结果。

## launch描述组织，不保证对端已就绪

roslaunch读取XML、展开arg/include/namespace/remap、准备参数、启动并监控节点。XML中先写一个node，再写另一个node，不等于前者的主题或服务在后者运行前已经就绪。

本课按真实条件开放操作：Master与Gazebo API可用，课程节点已进入图，期望服务绑定到正确节点，实际状态消息已经更新，之后才允许发脉冲。等待需要有限墙钟超时；重复sleep某个固定秒数不能代替依赖条件。

平台顺序为：启动私有Master → 准备个人包、输入指纹并设置本Master的/use_sim_time=true → 启动Gazebo/API → 检查世界就绪 → roslaunch启动两个节点 → after_startup检查实际ROS图与新反馈。最后一步在节点启动之后，不能把依赖这两个节点的检查放到更早的世界准备步骤里。

required=true表示该节点退出时，roslaunch终止这次launch中的其余受管节点；即使节点正常返回0也属于退出。它不是“等该节点初始化好”的指令，也不直接关闭平台独立创建的Master或Gazebo。required与respawn不能同时启用。

launch节点进程与Gazebo内部力任务也是不同生命周期。节点退出不代表已经接纳的力任务立刻撤销；本课任务有有限duration，必要时通过本课body的cancel接口请求清除，并继续以实际状态判断行为。

## 时间与真实反馈

/use_sim_time使ROS节点采用模拟时钟，Gazebo提供/clock。刚启动、尚未收到有效clock时，ROS时间可能为0；暂停世界后，它也不继续按实际秒增长。物理任务的持续时间应按模拟时间理解，节点就绪/服务等待的失败超时则需要有限实际耗时。

本课在Gazebo和课程节点初始化前，为本实验私有Master设置/use_sim_time=true，launch也声明同一值，就绪检查再核对它。该设置有明确作用域和初始化时机；运行中随意切换它，会使原先的时间假设失效。

本课用Gazebo ROS API的实际模型状态作反馈。ModelStates本身没有header；observer在接收时记录的ROS时间或序号属于观察者，不应伪装成原生物理采样stamp。服务返回成功、状态消息收到、物体实际发生预期变化是三层证据，不能互相代替。

observer最多按50个墙钟Hz输出选取到的真实反馈；这是观察程序的限流上限，不是Gazebo物理采样率。source_message_sequence统计本observer收到ModelStates的回调，也不是生产者原生帧号。反馈和状态主题会保存最后一条消息供新订阅者读取，所以echo到一条不代表它刚刚更新；实际就绪与验收还检查新序号、接收时刻及独立状态。

TF变换和URDF结构将在后续课程集中展开；本课只使用已提供模型和明确的world坐标，先把启动组织与通信图读清楚。

## 短力脉冲的物理含义

本项目通过Gazebo ROS API的ApplyBodyWrench安排有限持续时间的world方向力任务。它是服务请求，参数包括实际Gazebo body名、参考系、力/力矩、开始时间与duration；不是把节点名称映射成一个力，也不是不断改写模型位置。

对初始静止、无重力与无速度衰减的试块，理想预测为a=F/m，力持续T后速度增量约FT/m。任务到期后不再施力，试块仍会匀速前进；清除力任务不等于清除速度。这个实验只需要牛顿第二定律，不需要先学PID。

默认F=2 N、m=1 kg、T=0.3模拟秒，速度增量约0.6 m/s；只改为4 N，增量约1.2 m/s。正式请求还留0.1模拟秒提前量。若之前已有速度v0，应比较脉冲前后的Δv，不无条件要求最终速度为0.6；重复脉冲会继续改变运动状态。

实际调度落在物理步长网格，当前API按开始/结束两端包含的条件施力，有限任务可能多一个物理tick。预测用于数量级和持续行为验证，不要求脱离离散实现无限精度相等。开始时间、服务响应与实际运动窗口仍分别保留。

driver_status中的scheduled说明最后一条请求已被记录，不是实时力传感器，也不保证任务现在仍未到期。请求的start/end和pulse_id用于追踪本次调用；真正的加速与匀速仍由状态反馈和独立查询确认。

不了解力、质量与加速度关系，可看[OpenStax牛顿第二定律](https://openstax.org/books/university-physics-volume-1/pages/5-3-newtons-second-law)。本课的API字段依据见[ApplyBodyWrench定义](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/srv/ApplyBodyWrench.srv)。

## 完成一次真实闭环

启动后先确认两个课程节点及其实际反馈已就绪，蓝块应保持静止。保持世界运行，使用课程按钮或操作终端：

```sh
python3 lab.py reset
python3 lab.py pulse
python3 lab.py observe --seconds 2
```

reset清除本课body的力任务，将蓝块摆回world(0,0,1)、单位姿态和零速度，并恢复进入命令时的暂停状态。pulse提交一次有限任务；若世界暂停，任务已排入模拟时间队列，需要点击“继续”才能运动。观察蓝块先加速、后匀速，同时读取实际position与linear.x。再发脉冲时应先预测已有速度基础上的变化；已有任务未结束时，driver拒绝重叠请求。

observe先检查图与新反馈，再按--seconds给出的墙钟时长采集，范围0.5～5秒；它需要世界继续运行以取得新反馈。可用`python3 lab.py observe --seconds 2 --report /tmp/lesson22-observe.json`保存结果。`python3 lab.py cancel`请求清除本课body队列，保留已有速度；要回到静止初态，用reset。只凭状态主题中的一条旧消息判断当前运动，会漏掉latch与暂停问题。

对照按钮运行`python3 verify.py`。验收会先保存进入时的位姿、速度与暂停状态，清除本课body任务并复位静止，再执行“空闲基线 → 一次脉冲与重叠拒绝 → 持续加速 → 到期匀速 → 恢复”的完整流程。它比较真实状态查询、ROS反馈和配置，不用命令推算值替代测量。

验收结束会恢复进入时的位姿、速度与暂停状态；原先排队的力任务、driver最后状态、已经流逝的模拟时间和运动历史不会随之倒回。因此先完成当前手动脉冲，再运行验收。报告默认保存在本次session的ros-launch-verify-UUID.json，也可用`python3 verify.py --report /tmp/lesson22-verify.json`指定路径。

## 两个单变量实验

**实验一，只改力值。** 默认2 N时先预测约0.6 m/s的速度增量，然后保存4 N并通过应用“重启实验”重载：

```sh
python3 lab.py config --force 4
```

重载后查询实际driver私有参数，确认为4.0，再复位、发一次脉冲并观察。预期增量约1.2 m/s，节点、主题与服务的完整路径保持原namespace。保存后尚未重载时，运行实例仍使用旧配置；准备指纹会拒绝把新文件与旧节点混作同一实验。最后用`python3 lab.py config --force 2`保存并重载，恢复基线。

**实验二，只改namespace。** 保持力为2 N，先预测哪些完整名字会改变，再保存并重载：

```sh
python3 lab.py config --namespace cart_b
```

比较静态nodes预览与运行图，检查/lesson22/cart_b/driver、/lesson22/cart_b/observer及新服务路径，再执行同样脉冲。Gazebo实体仍是launch_slider::body，物理响应应保持同一量级；只改ROS命名没有创建第二个物块。最后保存`python3 lab.py config --namespace cart_a`并重载。

每个实验都按“预测 → 保存一个变量 → 重载 → 查图/参数 → 实测 → 解释 → 恢复”记录。不要同时改force和namespace，否则难以判断哪项变化造成差异。

## 常见错误与独立挑战

按“包环境 → 完整名称与类型 → 节点身份与参数 → 新消息 → 实际物理反馈”的顺序排查，一次处理一个明确问题。

| 现象 | 先看什么 |
| --- | --- |
| package找不到 | 当前ROS_PACKAGE_PATH与rospack实际找到的路径 |
| 找到包但node无法启动 | node type、脚本shebang/执行权限、依赖与日志 |
| launch在运行但服务调用失败 | 最终完整名称、类型、服务提供者与就绪超时 |
| arg改变，param没变 | 是否显式将arg写入正确节点的私有param，是否已经重载 |
| rosparam set后行为没变 | 节点读取参数的时机，而不只看服务器当前键值 |
| 名称看起来相似但不连通 | namespace、绝对/相对名、remap和当前Master |
| 状态topic存在但没有新数据 | 世界是否暂停、节点是否仍活着、实际订阅连接与消息更新 |
| ROS时间为0或暂停后不增长 | /use_sim_time初始化、/clock是否建立与世界是否继续运行 |
| 工具报告输入或参数不一致 | 保存配置是否重载、运行节点是否仍来自旧副本、服务器值是否被临时修改 |
| 脉冲结束块仍在走 | 有限力已结束后可能正常匀速；看加速度而不把零力等同刹车 |
| 清除服务返回空响应 | 它不提供删除任务数，需结合真实状态/到期行为判断 |

独立挑战：给本课画一张含Master、两个课程节点、Gazebo API、状态topic、脉冲service和私有param的图。用实查到的完整名称标边，解释每个箭头是什么通信；再设计一个只改remap就能让图出现错误连接的反例，先预测诊断结果，恢复后用真实反馈证明修复。不要用启动进程成功代替图与行为验收。
