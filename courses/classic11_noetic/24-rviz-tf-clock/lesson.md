# RViz、TF与时钟：看清机器人状态从哪里来

第23课把URDF变成了真实Gazebo机器人。本课把它的实际关节状态送进ROS，再观察robot_state_publisher怎样生成TF、RViz怎样按TF摆放机器人，以及模拟时间停止、单步和回退时会发生什么。

RViz是ROS的三维可视化工具，显示模型和消息；Gazebo负责本课动力学。学习目标是把两边的数据链核对清楚，让画面、坐标和时间各有明确来源。

## 三种树，回答三个问题

| 对象 | 回答的问题 | 本课怎样核对 |
| --- | --- | --- |
| URDF树 | 哪些link由哪些joint连接、零位变换是什么 | robot_description与生成URDF |
| TF树 | 某时刻各frame之间有什么变换 | /tf、/tf_static与tf2查询 |
| Gazebo物理link集合 | 哪些独立刚体真正进入了动力学 | 实际模型/物性与GetLinkState |

frame是一个带名字的坐标系。它可以表示刚体原点，也可以表示工具或传感器安装位置，不要求每个frame都对应独立物理刚体。

例如URDF保留了工具的fixed连接，即使Classic把工具合并进arm2，robot_state_publisher仍可发布arm2→tool_link固定变换。TF里的工具frame与Gazebo物理link数量不同，属于这两种表示各自的职责。坐标变换描述位姿，不自动给出质心、碰撞或质量。

## 本课真实数据链

```text
Gazebo中的tf_arm实际关节
  → 系统Gazebo JointState发布插件
  → /lesson24/joint_states
  → /lesson24/robot_state_publisher + 同一份URDF
  → /tf中的动态边、/tf_static中的固定边
  → tf2查询与RViz的RobotModel/TF显示
```

本课采用libgazebo_ros_joint_state_publisher.so。它在Gazebo世界更新回调中读取实际Position与GetVelocity；ROS图里发布者属于Gazebo进程，话题按robotNamespace放在/lesson24下。独立的robot_state_publisher读取这些状态并计算正向运动学；它不负责对机器人施力。

因此本课状态入口来自真实Gazebo关节。先核对这个入口，再看TF和画面，而不把一个能拖动模型的界面当作物理反馈。

本课模型名为tf_arm，两个实际关节是lesson24_shoulder与lesson24_elbow。桌面2R机器人沿用前课的水平转轴结构：URDF中有world、base_link、arm1、arm2、tool_link，默认保留工具的物理link。world是描述中的固定锚点参考，不能凭URDF的link名单就假定它也作为一个普通动力学刚体存在。

## 文件与初始化顺序

| 文件 | 在本课中的用途 |
| --- | --- |
| arm_settings.json | JointState发布频率和TF前缀；修改后重启实验 |
| ros_ws/src/gazeboteach_l24/urdf/arm.urdf.xacro | 机器人描述与真实JointState插件配置 |
| ros_ws/src/gazeboteach_l24/launch/arm.launch | 描述参数、一次性spawn及RSP节点 |
| ros_ws/src/gazeboteach_l24/config/arm.rviz | RobotModel、TF、Grid的RViz模板 |
| prepare.py、build/arm.urdf、build/arm.sdf | 展开Xacro、实际转换URDF、保留生成证据 |
| lab.py、graph.py | 物理操作、四路消息记录及持续tf2 listener |
| rviz_manager.py | 启动、重开与结束本会话的RViz |
| verify.py | 状态、变换、时钟及恢复的真实数据验收 |

点击“开始实验”后，应用先为私有Master设置/use_sim_time，生成URDF和转换SDF，启动已有实验的Gazebo，再由本课launch载入描述、spawn一次机器人并启动RSP。它继承本课Master与Gazebo，不另开一套相同Master。图形就绪后才打开RViz；无图形验收不依赖RViz进程。

保存的设置、生成描述、实际robot_description与正在运行的节点要一致。修改arm_settings.json后使用应用的“重启实验”重新准备；只编辑生成URDF不会自动重新spawn，也不会更新已加载的RSP描述。

## JointState怎样表达状态

JointState通过name数组标识关节，position、velocity按相同索引对应。本课两个转动关节的位置单位rad、速度rad/s。effort若存在，转动关节使用N·m；本插件没有填写该数组，因此空effort表示本消息未提供力矩测量，不等于测得零力矩。

header.frame_id在本插件中为空。关节状态由URDF中的joint名称关联，不能靠随意填入world来改变它的坐标含义。

```sh
rostopic info /lesson24/joint_states
rostopic echo -n 1 /lesson24/joint_states
rosnode info /lesson24/robot_state_publisher
```

先读name，再读相同索引的数值。tf_prefix改变TF frame的名称，不把JointState关节名或Gazebo实体名一起改掉。

消息定义可查[JointState](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/JointState.msg)，实际发布行为见[Noetic Gazebo插件源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_joint_state_publisher.cpp)。

## 一条TF变换是什么意思

TransformStamped的header.frame_id是父frame，child_frame_id是子frame；translation描述子原点在父系的位置，rotation描述子系相对父系的朝向。若点在子系表示为p_child，则：

```text
p_parent = R_parent_child · p_child + t_parent_child
```

这里的R与t既有方向，也有参考系。换到反方向需要取逆变换，不能只把translation三个数随意取负。

本课树沿world→base_link→arm1→arm2→tool_link复合。查询“工具在world中哪里”时，目标系是world，源系是tool_link；得到的变换把工具坐标中的点转换到world坐标。tf2会沿树组合所需边，不要求每一对frame都由一个节点直接发布。

rotation使用四元数x、y、z、w，它不是四个欧拉角。单位四元数表示旋转，q与−q表示同一姿态；比较姿态应用旋转角误差，而不是只逐分量相减。

一个入门关系是：绕单位轴u旋转θ时，四元数的前三项为u·sin(θ/2)，最后一项为cos(θ/2)；单位姿态为(0,0,0,1)。具体轴角、RPY和归一化操作可查[Noetic tf2四元数接口](https://github.com/ros/geometry2/blob/noetic-devel/tf2/include/tf2/LinearMath/Quaternion.h)。

不了解齐次变换，可看[Modern Robotics坐标变换](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/)；不了解旋转表示，可看[旋转矩阵与姿态基础](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-2-1-rotation-matrices-part-1-of-2/)与[REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。

## /tf与/tf_static

本课world→base_link、arm2→tool_link来自fixed joint；base_link→arm1、arm1→arm2随关节状态变化。Noetic robot_state_publisher默认把固定边放到/tf_static，动态边放到/tf。

/tf_static使用静态广播与latch，新订阅者能收到已发布的固定关系。它不需要随每条JointState重复发送。静态消息本身仍可带发布时的stamp；在tf2中，静态边的可用性不按动态历史窗口解释。

动态TF携带用于计算它的JointState.header.stamp。RSP的publish_frequency是按输入stamp作发布门控，不是在没有新状态时补出新姿态；提高这个参数也不会提高物理引擎的采样率。

一个TFMessage包可含多条TransformStamped，每条都有自己的stamp。接收包的墙钟时刻是另一层观测，不应覆盖这些原始字段。

## 名称空间与frame前缀

ROS namespace组织节点、topic、service和参数；TF frame名字是消息内容中的标识。把节点放进/lesson24，不会自动把base_link改名为lesson24/base_link。

本课Noetic RSP使用tf_prefix参数。空前缀时查询world、arm1、tool_link；设为demo时，对应demo/world、demo/arm1、demo/tool_link。ROS话题仍在同一/lesson24命名空间，JointState仍使用实际关节名。

修改前缀后，RViz的Fixed Frame及RobotModel的TF Prefix也要与实际TF一致。Robot Description则继续指向本课的描述参数。前缀一致是名称匹配问题，不改变机械臂尺寸或运动学。

## 时间：采样、标注、接收与查询

本课在Gazebo和ROS节点初始化前，为私有Master设置/use_sim_time=true，Gazebo提供/clock。该初始化顺序沿用前课建立的真实流程。

系统JointState插件用世界模拟时间调度发布，在更新回调中读取关节，但header.stamp取ROS Time::now的时钟缓存。因此应分别保留世界时间、原JointState stamp与接收CLOCK_MONOTONIC；这个stamp不是额外得到的精确原生采样时刻。

RSP把JointState stamp复制到动态TF。相同stamp说明这两层记录可以按标签关联，不证明Gazebo状态与其他传感器实现了硬件级同步。

tf2查询还需要选择时间。指定某个stamp是在查询该时刻的变换；Time(0)通常表示最新共同可用时间，而不是要求返回仿真刚启动的零秒状态。链路中某条动态边太旧、未来时刻还没有数据，都会影响查询。插值使用已有历史构造所查时刻的变换，也不等于预测未来物理运动。

## 暂停、单步和真实时间回退

暂停Gazebo后，墙钟继续走，模拟时间停止；原生插件不再因物理步推进产生新状态。TF缓存中已有的姿态仍可能可查，RViz也能继续显示，所以“窗口还在刷新”不能单独证明模拟时间继续前进。

还要区分何时接入订阅：暂停后才新建的客户端，可能尚未收到/clock、JointState或动态TF；一直连接的客户端则可能持有上一次样本。/tf_static的latch又是另一种情况。本课ready会在暂停时核对实际图、参数与模型，并注明消息观察延后，保持原pause；动态验证先保存进入快照，再明确受控继续。

单步应以实际世界时间/迭代变化确认，再观察状态与TF何时产生新记录。发布频率可能低于物理更新频率，一个物理tick并不保证触发一条JointState或TF消息。

reset_simulation会真正回退世界时间。RSP在后续JointState回调发现ROS now下降时重置发布门控；tf2 listener在后续TF或静态TF回调发现倒时后清理同一buffer中的动态历史。它们的处理依赖各自回调，不是在收到/clock的那一瞬间由所有节点同时完成。

本课用同一个持续listener跨越回退观察清理与重新获得新时间段的动态TF，静态边则保持其静态缓存语义。重新创建listener后能查询，只能证明重建成功，不能替代对原listener自动恢复的观察。

机器人状态与pause可以按实验快照恢复；已经改变的模拟时间、TF缓存历史和消息历史不回填成原样。分析重置前后数据时，保留各时间段与接收顺序。

## 在RViz中读画面

进入本课图形实验时会准备RViz配置，显示RobotModel、TF和Grid；RViz窗口可以单独重新打开。无图形的自动实验仍可检查真实状态和TF，RViz画面另作图形检查。

先看Global Options中的Fixed Frame，再看RobotModel读取的Robot Description与TF Prefix。Fixed Frame选择显示数据统一转换到的参考系，并不把Gazebo刚体焊死在那里。

RobotModel按URDF的visual几何与TF显示连杆；TF显示坐标轴、frame名称及父子关系。几何中心可能相对link原点有偏移，因此轴不必都在长条中点。Grid只是观察参照，不是Gazebo中的碰撞地板。

模板的Robot Description为/lesson24/robot_description，默认Fixed Frame为world、TF Prefix为空；demo实验会由管理器生成对应的demo/world及demo配置。下面命令在本课操作终端运行：

```sh
python3 rviz_manager.py status
python3 rviz_manager.py reopen
```

reopen会停止当前会话所管理的RViz再重新打开，不重启Gazebo。直接关闭RViz窗口后也可用它恢复显示。若需要暂时只看Gazebo，可执行python3 rviz_manager.py stop；管理器按会话与进程身份限制范围。

暂停中重新打开的RViz能立即取得latched固定边，动态TF要等继续或足够多步后才会有新样本；reopen本身不推进世界。因此新窗口暂时缺动态连杆时，先核对暂停与消息来源，再判断模型或前缀是否有错。

实际操作时展开TF的Frames/Tree，选一个frame对照模型原点，旋转视角检查两段连杆与工具。记录Global Status和各Display的具体错误文字，再按数据链定位来源。

RViz概念入口见[官方ROS RViz项目](https://github.com/ros-visualization/rviz/tree/noetic-devel)，RobotModel实际如何读取URDF与TF可查[显示插件](https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/robot_model_display.cpp)。

## 实验一：用真实姿态核对两幅画面

先预测肩转正、肘转负后工具会落在哪一侧，再执行：

```sh
python3 lab.py pose --name positive_mixed
python3 lab.py pose --name negative_mixed
python3 lab.py reset
```

三个预置分别包含zero=[0,0]、positive_mixed=[π/6,−π/4]、negative_mixed=[−0.4,0.6]，单位rad。pose是实际关节初始化：先暂停、清本课有限力矩任务、设置真实关节位置及零速度，再推进150个物理步让原生发布器和TF更新，最后恢复进入时的pause。物理步长为1ms，因此刷新会真实推进0.15s模拟时间；它不是一次只改RViz画面的显示操作，也不是电机控制器。

每次等待命令完成后，在Gazebo看连杆，在RViz看RobotModel和TF轴；比较工具相对base_link与world的方向。reset复位机械臂零姿态并刷新TF，不把模拟时钟清零。对照消息时，应把实际name与position对应起来，不根据数组先后猜肩肘身份。

可选动态观察：先reset，再执行一次python3 lab.py pulse --sign 1，若处于暂停则点击“继续”。它向真实elbow提交未来50ms开始、持续200ms的有限0.03N·m力矩；sign −1反向。多体耦合会让肩肘共同响应，这里只用于确认RViz随真实状态变化。每次独立尝试先reset，不叠加多次任务。

## 实验二：暂停、物理单步与时间回退

先在应用中暂停，再观察/clock与JointState。保留一个已连接的观察终端，比较等待一段墙钟后最后的模拟stamp是否变化；另开一个新订阅者，理解它可能没有新样本可读。

```sh
python3 lab.py step --steps 1
python3 lab.py step --steps 100
```

命令要求已经暂停。第一个使真实世界前进1ms，第二个前进100ms，然后保持暂停。先以实际世界时间确认推进量，再看消息：20Hz状态每约50ms一次，一个1ms物理步可能尚未跨过发布门限；不能把未出现新TF误诊为单步失败。

接着执行：

```sh
python3 lab.py clock-reset
```

它先暂停；若新客户端尚无旧时间段的clock/JointState/TF，会先真实推进150步预热，取得可比较的重置前观察。随后清除本课两关节的力矩任务，调用真实reset_simulation，再推进150步，使同一listener接收倒时后的新数据，最后恢复原pause。有旧时间段消息时不额外预热。

因此命令结束时世界已经从重置点继续推进，不能要求最终读数仍严格为0。应读pre_reset_observation与重置阶段事件：例如极早期预热所得的重置前时间与重置后刷新所得时间可能同为0.15s，仍能从中间/clock序列看到回退。API任务队列与世界重置是不同机制；清任务防止旧任务等到重置后的时间再次执行。

世界重置也会使用模型的初态。本命令恢复进入的pause，机械臂姿态由这次重置决定；完整verify则另外保存入口快照，在结束时恢复机器人状态。

观察异常出现在哪个回调，以及固定边和动态边恢复有什么不同。已有RViz也有自己的TF缓存，真实窗口的状态变化应单独观察；命令中被审计的listener不是RViz内部的同一个对象。

## 实验三：运行真实数据验收

```sh
python3 verify.py
```

也可用python3 verify.py --report /tmp/lesson24-check.json指定报告。默认写入本会话目录的tf-clock-verify-UUID.json，应用入口给出150s有限超时。验收会操作机械臂、推进和重置模拟时间，结束时恢复进入的机器人状态与pause。

验收沿数据链执行：

1. 先保存实际状态；受控继续，取得至少1s、至少rate+1条真实JointState，检查实际20/40Hz的stamp窗口频率在名义值±5%内，且动态TF的每条stamp能对应观测到的JointState。
2. 在三个静止预置姿态，对照实际关节、四个link的world查询与Gazebo GetLinkState，再单独对照base_link→tool_link；位置误差小于10⁻⁵m、旋转角误差小于10⁻⁵rad。
3. 暂停保持0.15s墙钟，检查实际世界时间与ROS读钟不前进；再分别真正推进1步与100步，以世界时间核对，而不要求每步必有JointState。
4. 暂停中建立新的静态订阅，检查真实latch；故意查询lesson24/base_link，记录把namespace误当frame的失败。
5. 同一listener与Buffer跨越真实时间回退，记录/clock下降、原Buffer.clear调用增加和新时间段lookup成功，检查固定边仍能参与整条链的查询。
6. 清本课任务，恢复原关节与link姿态，在零速度下推进150步刷新状态链，再恢复原link速度及进入的pause；按实际回读检查恢复。

报告的trace按接收顺序记录clock、joint_state、tf、tf_static。clock行保留/clock值，JointState保留原header，TF包的每条变换各自保留stamp；receive_monotonic_ns单列实际接收时刻，callerid与latching帮助核对发布来源。pose_phases、stream、step_phases、reset_phase与restore分别保存各阶段证据，不把服务查询结果伪装成一条原生TF消息。

图形检查请另外记录：RViz是否真正显示完整RobotModel、TF轴/名称、Grid及无关节名/前缀错误的状态；改变姿态后显示是否跟随。自动数据检查成功与完成这一步人机观察分别标记。

## 先预测，再改一个变量

只改JointState发布频率，预期状态与动态TF的观测节奏改变，URDF、frame名称和机械臂尺寸保持。分别计算原header标签频率f_stamp=(N−1)/(最后stamp−首个stamp)，以及单调墙钟接收频率，时间差均换算成秒。

Noetic rostopic hz默认在接收回调中读取ROS当前时间；使用--wall-time才按墙钟计时。两者都不直接读取消息header.stamp，不能与验收报告中的原header标签频率混为一谈。可另开终端运行rostopic hz --wall-time /lesson24/joint_states作接收侧观察，暂停/继续后分别记录所用时基。

```sh
python3 lab.py config --joint-state-rate 40
```

保存后重启实验，重新运行验收。恢复基线用--joint-state-rate 20。这里RSP显式publish_frequency=50，因此两种源频率都低于其门控上限；提升源频率没有要求物理步长也一起修改。

只把tf_prefix从空改为demo，预期frame字符串及RViz对应配置改变，同一个机械状态的变换数值保持。先列出应改的名称，再从真实消息与查询中核对；不要通过同时修改物理尺寸掩盖名称问题。

```sh
python3 lab.py config --tf-prefix demo
```

同样重启实验后核对demo/world与demo/tool_link；恢复空前缀用python3 lab.py config --tf-prefix ""。两组单变量实验分别从基线开始，每次只改一个设置，再观察实际运行的参数与消息。

| 现象 | 沿哪一层检查 |
| --- | --- |
| 有URDF但RobotModel不完整 | 描述参数路径、实际TF frame与Fixed Frame |
| JointState有值但TF没有新记录 | 关节名与URDF是否匹配、stamp、RSP门控及模拟时间 |
| tool没有独立物理link却有TF | 查看URDF工具fixed边与Classic合并策略 |
| namespace正确但frame找不到 | 实际frame字符串、tf_prefix与RViz前缀设置 |
| LookupException | 目标/源frame是否真的存在 |
| ConnectivityException | 两个frame是否属于同一连通树 |
| ExtrapolationException | 查询时刻是否位于动态历史范围 |
| 刚启动时固定边尚未出现 | 时钟是否已推进到RSP固定边定时器触发 |
| 暂停后新开echo一直等待 | 非latched话题没有新样本；比较已连接订阅的末样本，或明确继续实验 |
| 保存设置后ready提示不一致 | 重启实验，让生成描述与已加载节点使用同一配置 |
| RViz关闭而Gazebo仍运行 | 使用reopen恢复本课显示窗口 |
| 重置后暂时查询不到动态变换 | 原listener是否收到倒时后的TF回调与新时间段数据 |

独立挑战：画出一条真实关节状态从Gazebo到RViz的路径，分别标明消息名、frame名和时间字段。再选择工具frame，解释它为何可以存在于TF却不对应独立物理link；最后预测一次暂停、一个物理单步和一次时间回退分别会改变哪些记录。自动数据验证与自己在RViz中的观察分开记录。
