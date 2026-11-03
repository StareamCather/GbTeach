# ROS 1 通信专项：节点图、类型、队列与锁存

这是独立附加专项，保留原01—37课。从第22课知道如何启动节点后，本课回答：**数据究竟从哪里来、由谁发布、谁在接收？** 使用实际运动滑块作为观测源，先画通信图，再通过ROS Master、TCPROS连接和原生关节反馈核实。

## 背景：发现节点与传输数据

节点（node）是运行的ROS程序；话题（topic）是具有确定消息类型的数据流。ROS Master负责注册与发现，连接建立后，话题数据通常由发布者和订阅者直接通过TCPROS传递，不经过Master中转。

本课只有一个仪器节点，通常名为 `/workshop42/instrument`，实际名称以 `rosnode list` 为准：

```text
Gazebo ODE 真实关节
  → /gazebo/get_joint_properties 服务读回
  → 本课 instrument.py
  → /workshop42/state [geometry_msgs/PoseStamped]
  → 学员 rostopic / subscriber / 自动验收节点

迟加入的订阅器 ← /workshop42/ready [std_msgs/String, latch=true]
操作终端 → /workshop42/pulse [std_srvs/Trigger]
  → 一次有限 ApplyJointEffort → ODE运动 → 新关节观测与状态消息
```

PoseStamped带时间戳与坐标系名称。本课把一维关节位移放在 `pose.position.x`，frame为 `workshop_slide_zero`，表示滑块关节零位坐标；其余位置为0、方向为单位四元数，仅是这个一维装置的消息表示，**不代表另有六自由度定位传感器**。本课没有TF广播器；坐标变换将在另一个专项讨论。

GetJointProperties没有原生测量时间戳。仪器在服务返回后读取当前 `/clock` 放入header.stamp；同步服务查询、时钟回调和消息发送不是原子快照。报告的两次clock读回及墙钟查询区间只说明观测流程，不能把它们叫底层传感器采样时刻。

## 文件与初始化的实际流程

| 文件 | 作用 |
|---|---|
| `world.sdf` | 1kg滑块、X向关节、±1.2m轨道、粘性阻尼 |
| `instrument.py` | **仅本课**的发布、锁存、有限力服务及参考验收 |
| `ros_ws/src/gazeboteach_workshop42/` | 本课ROS包、launch、个人配置和节点入口 |
| `prepare.py` | 检查XML、设置本课脚本可执行及私有Master中的use_sim_time |
| `lab.py` | 从个人副本运行 ready/state/pulse/verify 命令 |
| `verify.py` | 从受信任的本课源码加载验收，读取实际个人副本和运行世界 |
| `student.py` / `reference.py` | 独立速度估计填空与答案，互不覆盖 |
| `build/verification.json` | 最近一次参考实验的结果、轨迹或失败原因 |

点击开始实验：平台准备个人练习副本、私有ROS Master和Gazebo；prepare只检查文件和初始化本课ROS时钟配置，**不会从共享资源复制support.py，也不会覆盖个人instrument.py或student.py**。roslaunch启动本课instrument，首次真实关节读回成功后只发布一次ready字符串；后续持续读实际关节并发送状态。

长操作在平台后台运行。结束实验只清理本应用创建的进程，保留个人文件。若修改了个人仪器代码，需要重启本课让节点加载新文件；自动验收会比较运行时源指纹与当前个人instrument.py，防止把未重载的新代码算成已测试。

## 操作、预测与可观察结果

1. 在本课操作终端运行 `rosnode list`、`rostopic list`，找到仪器节点和两个话题。运行 `rostopic info /workshop42/state`，记下实际发布者、订阅者和类型。
2. 用 `rosmsg show geometry_msgs/PoseStamped` 逐字段解释。再运行 `rostopic echo -n 6 /workshop42/state`，观察seq、stamp、frame与x。静止时多个x相同是合理结果，不能因此判断没有数据。
3. 在节点已经运行后，新开本课终端运行 `rostopic echo -n 1 /workshop42/ready`。预测为什么一次性ready早已发布，你仍能立即收到；随后检查ready连接确实声明latching，而不是仪器每隔一会儿重发。
4. 运行 `rostopic hz /workshop42/state` 和 `rostopic bw /workshop42/state`，记录实际接收率/吞吐。仪器每轮墙钟休眠.04s，此外还包含服务耗时；这不是精确25Hz实时调度保证。
5. 运行 `python3 lab.py state`，读独立原生关节反馈；再运行 `python3 lab.py pulse`，观察真实滑块运动以及新的x消息。

默认脉冲幅度2N、持续.2仿真秒，方向根据当前关节位置朝原点选择；在原点从正方向开始。服务先清理本课关节的待执行力，再调度一次有限力。**清除力不等于清除速度**，脉冲结束后滑块仍会因惯性继续运动，再由阻尼减速。手工改幅度须保持在.2—5N内；不要在模型到限位或高速运动时盲目连续点脉冲。

## queue_size 与 latch 的单变量实验

`queue_size=8` 是发布队列容量，不是可靠性、延迟或“永不丢消息”的承诺。消息seq的缺口也不能单独定位问题发生在发送队列、网络还是订阅回调。

在个人 `instrument.py` 中只把状态发布队列8改成1，其他配置不动；重启本课。编写慢订阅回调并连续记录接收seq、header时间与墙钟接收时间，比较现象。恢复队列后，只把取样休眠.04改成.08，再测实际频率。不要为“修复丢包”重写header.seq或事后排序时间戳。

latch让新订阅器拿到最近一次已发布数据，适合一次性配置/状态；它不保留完整消息历史，也通常不适合不断变化的运动指令。可在独立副本中把ready的latch改为false后预测迟订阅结果；参考验收应明确失败，恢复后再重启。

## 自动验收究竟检查什么

运行 `python3 verify.py`，或点击平台验收。**为保证多次验收可重复，验收会先清除待执行力，并将本课私有世界的模型/速度恢复起点；不回退仿真时钟、不改个人文件。** 若你要保留某次运动状态，先记录数据再验收。

验收会：

- 作为迟加入的真实订阅者，读取ready连接头的callerid/latching，核实ready发布早于订阅器创建。
- 检查状态消息连接的真实类型与发布者；向Master核对同一节点的发布注册、服务注册及**验收节点自身**的订阅注册，报告完整节点名。
- 接收至少6条状态并确认仿真时间推进；只存在话题名称不能通过。
- 调用一次 `/gazebo/reset_world`，用新收到的零位状态消息和独立GetJointProperties确认位置/速度复位，并用GetWorldProperties确认时钟未回退。
- 调度一次有限脉冲，要求随后新收到的消息沿命令方向改变；再独立读取原生关节，确认实际位移超过2.5cm。
- 用接收年龄小于0.25秒的最近状态，与独立关节读回比较位置，允许0.05m异步差异；两个读数都在移动，也不能据此放过单位/比例严重错误的消息。
- 保存个人世界指纹、仪器运行指纹、原生关节读回、实际消息轨迹和结果；失败也保存原因。

这验证参考仪器的通信和真实物理闭环，`assignment_pass=false`，不会代填或自动判定student.py。GUI用于观察滑块，通信目标以ROS图、TCPROS连接和原生关节读回为证；不以窗口出现作为通过。

## 超时、暂停与错误诊断

所有等待证据使用单调墙钟，因此暂停 `/clock` 不会让检查无限等候。rospy内部可能把网络socket设为阻塞；本课用daemon工作线程执行一次服务请求，并限制调用者的墙钟等待。**超时不等于远端请求已取消**：结果不确定时明确失败，不自动重放ApplyJointEffort。仪器的原生读回超时后退出，不继续无限创建新请求；实际力始终有有限仿真时长。遇到不确定调度，结束并重启本课实验后再试。

有名字没数据：检查发布者连接、仪器日志和Gazebo是否暂停。类型不匹配：核对 `rostopic type` 与消息定义。改了代码但提示指纹变化：重启本课，不要删除校验来强行通过。真实位移不够：看是否暂停、关节是否到限位、幅度是否合规以及调用是否成功；不要改报告数值。

远端节点无法互通时，先理解ROS_MASTER_URI负责发现地址、ROS_IP/ROS_HOSTNAME负责节点可达地址。本课在私有Master中运行，不要把试验修改写入 `~/.bashrc`。

## 填空、独立挑战与先修链接

使用student.py完成两次观测的速度估计：位移除以严格为正的时间差，单位m/s。先用简单正向、反向、零时间差例子检查，再实现subscriber保存至少10条真实观测并计算相邻差分。解释服务/发送时延、噪声与“此x实际对应哪个时刻”的限制；有限差分不能恢复原生采样时刻。

答案单独放在reference.py和参考页。扩展挑战：让订阅器启动得更晚，比较锁存与非锁存；增加一个类型错误的订阅器并从连接日志解释拒绝原因；画出Master发现和节点直传数据的两条不同路径。

不了解TCPROS？看[ROS官方通信概览](https://wiki.ros.org/ROS/Technical%20Overview)与[rospy发布订阅教程](https://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)。[GetJointProperties接口](https://docs.ros.org/en/noetic/api/gazebo_msgs/html/srv/GetJointProperties.html)可用于核对其确实没有Header；[ApplyJointEffort接口](https://docs.ros.org/en/noetic/api/gazebo_msgs/html/srv/ApplyJointEffort.html)说明有限力的起始时间和持续时间语义。
