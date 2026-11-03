# 附加专项 43：服务与 Action，怎样确认一个运动任务结束

本课在同一条真实滑轨上比较两种请求：**施加一次短脉冲**，以及**持续控制滑块到达指定位置**。第42专项解决消息由谁发送、谁接收；这一课进一步问：任务什么时候开始，执行到哪了，怎样取消，失败时又能相信哪些结果？

先修第09、10、22课。你需要理解位置、速度、力、比例与阻尼反馈；Action相关术语会在这里说明。

## 两种接口，两个完成条件

`/workshop43/pulse` 使用 `std_srvs/Trigger` 服务。请求体为空，返回 `success` 和 `message`。它完成一次有限施力调度，响应成功不表示滑块已经移动到某个位置。施力后滑块怎样运动，还要从真实关节状态读取。

`/workshop43/move` 使用本课自定义的 `MoveSlide.action`。Action允许客户端发送目标，持续接收反馈，等待最终结果，并请求取消。它适合有执行过程的任务。

本课默认脉冲为2 N、持续0.2仿真秒；位置Action的参考控制器根据真实关节位置和速度反复更新力。服务响应、Action状态和物理到位分别表示不同事实，实验时要一起核对。

## 先读懂自定义 Action

文件位于 `ros_ws/src/gazeboteach_workshop43/action/MoveSlide.action`，用两行 `---` 分成三段：

```text
float64 target_m
float64 timeout_wall_s
---
bool reached
float64 final_position_m
string message
---
float64 x_m
float64 v_m_s
float64 error_m
float64 elapsed_wall_s
```

- **Goal目标**：希望到达的位置，以及服务端本次控制循环允许的墙钟时间。位置单位为米，允许范围为±1 m；`timeout_wall_s`必须是0.5～12 s内的有限数值。NaN、无穷和越界值应被拒绝。
- **Result结果**：是否达到条件，结束时测得的位置，以及原因说明。`final_position_m`不是对未来位置的保证。
- **Feedback反馈**：执行期间实际位置、速度、位置误差和墙钟经过时间。它帮助你分清“仍在运动”“快到位了”和“长时间没有进展”。

该接口只描述本课单滑轨的位置到达任务，没有声明多关节时间参数化轨迹跟踪能力。自定义消息让任务含义明确，也使你亲自经历`.action`文件到Python消息类的构建过程。

## Action为什么出现五条话题

用 `rostopic list` 查看 `/workshop43/move` 下的接口：

| 后缀 | 主要方向 | 作用 |
|---|---|---|
| `/goal` | 客户端→服务端 | 发送带身份编号的目标 |
| `/cancel` | 客户端→服务端 | 请求取消指定目标 |
| `/status` | 服务端→客户端 | 发布目标状态列表 |
| `/feedback` | 服务端→客户端 | 发布执行中的连续反馈 |
| `/result` | 服务端→客户端 | 发布终止结果 |

目标ID把一次请求与相应反馈、状态和结果关联起来。向cancel发出消息只是请求；客户端观察到终止状态，才能知道服务端已经处理它。

本课常见状态包括 `ACTIVE=1`、`PREEMPTED=2`、`SUCCEEDED=3`、`ABORTED=4`。`PREEMPTING`表示取消仍在处理中；它与已经终止的`PREEMPTED`不同。本教学服务端对非法目标报告失败及原因，具体状态以实际输出为准。[状态码官方定义](https://github.com/ros/common_msgs/blob/noetic-devel/actionlib_msgs/msg/GoalStatus.msg)。

## 从文件到真实运动的完整流程

| 文件或目录 | 作用 |
|---|---|
| `world.sdf` | 1 kg滑块、阻尼和受限的真实prismatic关节 |
| `instrument.py` | ROS服务、Action服务端、实际关节读取与闭环执行 |
| `lab.py` | ready、state、pulse、action、verify、student-check操作入口 |
| `student.py` | 学员控制函数，保留误差和阻尼两处关键填空 |
| `reference.py` | 独立完整参考控制函数 |
| `ros_ws/src/gazeboteach_workshop43` | 自定义Action、package/CMake、launch与本课配置 |
| `ros_ws/src/gazeboteach_workshop43/config/controller.yaml` | 选择`reference_demo`或`student`模式，以及脉冲参数 |
| `prepare.py`、`ros_ws/build`、`ros_ws/devel` | 个人工作空间构建与消息生成产物 |
| `build/verification.json` | 实际实验验证结果 |

平台准备个人副本与Noetic子环境，并把声明的`ros_python_paths`预置到**本课子进程**Python搜索路径→启动私有ROS Master→构建本课catkin消息→启动Gazebo和仪器→等待真实接口→开放终端与操作。构建前生成目录可以尚不存在；消息生成成功后，仪器与验收进程才从该目录导入。生成消息目录属于当前练习，不需要修改`~/.bashrc`、全局`PYTHONPATH`或其他项目环境。

`prepare.py`还将catkin真正生成的7个`.msg`定义镜像到个人 `ros_ws/src/gazeboteach_workshop43/msg`。原因是Python模块搜索路径解决`import`，而原生`rosmsg`根据ROS包路径查找消息定义，两者不是同一套发现机制。镜像内容直接来自 `devel/share`，并通过 `rosmsg show -r` 和MD5与生成的Python接口核对，结果记入 `build/action-build.json`。这是可重建的私人生成物，不应手工编辑来冒充接口生成。

roslaunch默认可能让节点在ROS_HOME（通常是`~/.ros`）运行；若直接用当前目录找`reference.py`，会找错地方。本课节点入口`node.py`依据自身位于个人包中的位置定位练习根目录，并显式切换到那里，再导入仪器。控制器、构建记录和实验输出因此都属于当前个人副本，保留学员修改。


Action执行过程是：接收目标→检查有限数值、范围和墙钟预算→每轮先检查取消与墙钟期限→读取真实关节`x,v`→发布本次观测的反馈→再次检查取消及到位条件→计算PD控制力→清除本课旧的短期施力→调度新有限施力→进入下一轮。任务结束前，清除待执行力并作最后一次真实关节读回，再确认终止状态；清除失败不能报告已完成取消。真实位移由Gazebo物理引擎计算，Python不靠自己积分一条“假位置曲线”来充当反馈。

参考控制律为：

```text
error = target - position
force = clip(18*error - 8*velocity, -10, +10) N
```

每次新施力持续0.06**仿真秒**。位置误差小于0.025 m且速度小于0.07 m/s时才算到位。仅误差很小但仍高速经过目标，不算稳定到达。

状态消息与关节服务读数经过通信，`/clock`取样也有延迟；不要将它们解释为同一原子采样时刻。Feedback里的`elapsed_wall_s`是墙钟经过时间，不应与仿真时间直接相减。

## 第一次实验：短脉冲与持续控制

开始实验后，在本课终端执行：

```bash
python3 lab.py ready
python3 lab.py state
rosservice type /workshop43/pulse
rossrv show std_srvs/Trigger
python3 lab.py pulse
python3 lab.py state
```

先预测运动方向，再看真实位置与速度。脉冲服务返回时，施力可能刚开始；关节还可能在后续继续滑动。要做可比较的实验，请重启实验获得相同初始状态，再执行下一种操作。

```bash
python3 lab.py action --target .35 --timeout 8
```

另开本课终端观察：

```bash
rostopic echo /workshop43/move/feedback
rostopic echo /workshop43/move/result
rostopic echo -n 1 /workshop43/state
```

检查结果状态与`reached`，再用 `python3 lab.py state` 独立读取真实关节，确认接近0.35 m且速度已减小。反馈到达并不等于到位，状态成功也不应免除这次独立物理检查。

运行 `python3 verify.py` 可执行本课参考闭环验收。它应验证真实到位、反馈、取消和非法目标，并将观测结果保存到个人副本中。正式通过范围以报告字段为准。

## 取消：停止任务不等于速度瞬间归零

先重启到干净初态，再运行：

```bash
python3 lab.py action --target -.7 --timeout 8 --cancel-after .5
```

该客户端只取消自己刚发出的目标。观察先出现反馈和实际运动，随后收到`PREEMPTED`，结果`reached=false`。服务端清除已经调度的关节力，但不会通过强设速度为0来伪装停止。

关节仍有惯性，并受场景阻尼作用。取消后立即读到非零速度并不证明取消失败；要结合取消终止状态、清除力的处理和后续减速判断。如果清除力的远端调用结果不确定，就不能承诺已完成取消清理。

人工实验时避免在位置Action执行中反复注入pulse，否则会把额外外力混入控制器响应。要研究扰动，可以专门设计一次有限脉冲，并记录发生时刻。

## 清除队列的名字：返回成功还需要行为验证

在本课Noetic插件中，读取关节和安排施力可以使用作用域名称 `workshop::slide`，但 `clearJointForces` 遍历已安排的任务时，比较的是关节的 `GetName()`，得到的是短名 `slide`。即使没有匹配到任务，该实现也会返回成功；因此把 `workshop::slide` 原样交给清除接口，可能得到成功返回而留下任务。[Noetic官方实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L1599)

本课`clear()`因此只向 `/gazebo/clear_joint_forces` 传 `slide`，读取和施力仍使用 `workshop::slide`。每个实验的独立世界只有这一个同名关节，这个选择有明确的作用范围；多模型都叫`slide`时，不能把它当作按模型精确清除的方法。

如果旧短期施力没被清除，就可能与新施力任务重叠。每个任务分别“有限时长”并不能保证整段控制只有一份有效施力。另一方面，正确清除的是待执行力任务，已有速度仍由惯性和阻尼演化。

### 用暂停安排一个可反证的实验

1. 重启实验，确认真实关节位置与速度都接近初始零值。
2. 在Gazebo窗口暂停，读取 `python3 lab.py state`，记下位置、速度和停止前进的仿真时间。
3. 执行 `python3 lab.py pulse`，安排0.2仿真秒脉冲。暂停期间，这个时长不会随着墙钟等待而消耗。
4. 在恢复前明确清除：

```bash
rosservice call /gazebo/clear_joint_forces "joint_name: 'slide'"
```

5. 恢复仿真，确认仿真时间已经继续前进，再读取真实位置和速度。预测它们应仍接近零；若恢复后开始运动，就要怀疑队列没有清净，而不能用刚才的成功返回替代证据。
6. 可另起一个干净实验作为对照：同样暂停并安排脉冲，省略清除后恢复，预测应出现真实运动。

仅在暂停时看到滑块不动，不能验证清除，因为暂停本来就阻止物理前进。新增验证会记录暂停、安排、清除和恢复后的实际状态；是否满足预期以真实报告为准。

## 暂停与超时：三种“时间”各管什么

| 时间或预算 | 含义 | 仿真暂停时 |
|---|---|---|
| Gazebo仿真时间 | 决定0.06 s施力持续多久、物理前进多少 | 停止前进 |
| 目标`timeout_wall_s` | 服务端允许本任务执行多久 | 继续计时 |
| 客户端RPC/等待预算 | 调用者最多等远端或结果多久 | 继续计时 |

在Gazebo窗口暂停后，发出一个与当前位置不同的目标：

```bash
python3 lab.py action --target .8 --timeout 2
```

预测：滑块不能在暂停中移动，墙钟预算仍会耗尽，任务应以失败结束并说明原因。目标预算限制控制循环；最终清除施力与实际状态读回另有有界预算，因此结果确认可能略晚于2秒。恢复仿真前，查看任务结果与清除处理；需要重新开始实验时使用平台的重启入口。

客户端等待超时，只能说明它没有在预算内得到确定结果。底层ROS请求可能已经送达，调用者超时**不会自动撤销远端请求**；程序不会在这种不确定状态下自动重发运动请求。Action取消使用自己的goal身份与取消协议，是另外一种明确的操作。

## 两行填空：把PD控制律写出来

`student.controller(target, position, velocity)`的两处关键填空是位置误差和速度阻尼。先判断符号：

1. 目标在当前位置右侧时，误差应为正，比例力指向+x。
2. 速度为正时，阻尼项应为负；把速度项写成正反馈会助推而非制动。

填好后执行：

```bash
python3 lab.py student-check
```

然后在 `ros_ws/src/gazeboteach_workshop43/config/controller.yaml` 选择 `controller_mode: student`，重启实验，执行同样的真实Action。单独函数检查只能发现部分算术与符号问题，仍需观察闭环是否实际到位。

默认`reference_demo`运行完整参考控制器，便于先建立可运行基线；其验收报告标注`assignment_pass=false`。参考演示通过不表示你的填空作业已通过，也不会覆盖`student.py`。完整答案在独立`reference.py`，建议先预测再查看。

## 单变量实验：比例与阻尼

先保存默认结果，再一次只改一个增益；改动后重启，使仪器使用个人函数。

- 保持Kd=8，把Kp从18改为9。预测位置收敛更慢，记录反馈中的到位时间与最终误差。
- 恢复Kp=18，把Kd从8改为2。预测制动减弱，更容易越过目标；观察误差是否换符号。
- 保持增益不动，增加目标距离，检查±10 N限幅是否参与。不能把饱和后的控制响应当作未限幅的线性系统。

默认模式改`reference.py`；学员模式改个人`student.py`。本题的`student-check`固定核对18/8及±10 N限幅，student模式完整验收也先做这项算式检查。因此学员函数改增益属于探索实验，可能因不再符合原题而无法通过作业验收；需要恢复18/8再检查。检查失败不等于该新增益一定无法控制滑轨。不要同时改目标、增益、阻尼和超时后，仍将结果归因于某一个参数。

## 故障诊断

| 现象 | 检查方向 |
|---|---|
| 导入`gazeboteach_workshop43.msg`失败 | Action是否成功生成、本课devel Python目录是否进入子进程路径；先查prepare实际输出 |
| 只有ACTIVE，没有结果 | 检查真实关节是否前进、目标墙钟预算、服务调用日志和cancel通道 |
| 取消后仍在滑动 | 区分清除队列与清零速度；确认清除使用短名slide，再结合PREEMPTED和恢复后的真实速度判断 |
| NaN、Inf或2 m目标被接受 | 目标校验缺失；不要让非法值进入控制律或施力API |
| `student-check`报NotImplementedError | 两行关键代码尚未完成；参考模式不替学员作答 |
| 反馈很多，位置却不动 | 仿真暂停、受限关节、力调度失败；用独立关节读取定位 |
| 客户端超时后又看到运动 | 远端请求可能已经执行，超时不是远端取消；不要盲目重发 |

## 独立挑战与验收范围

把“瞬时满足到位条件”改成“连续0.2仿真秒处于误差与速度范围”，并解释暂停对该保持时间有什么影响。保留独立的墙钟超时与取消处理，分别设计正常到位、运动中取消、暂停超时、非法目标四组实验。

自动验收检查本课定义的真实任务协议与关节行为。它不认证真实机械设备安全，也不表示已实现完整机器人轨迹控制。应用关闭时仅清理本实验创建的进程，个人源文件和实验记录保留。

不了解Action？查看[actionlib官方源码与说明](https://github.com/ros/actionlib)。需要查有限施力的时间约定？看[ApplyJointEffort官方定义](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/srv/ApplyJointEffort.srv)。不熟悉自定义消息构建？[catkin官方消息/服务/Action构建指南](https://github.com/ros/catkin/blob/noetic-devel/doc/howto/format2/building_msgs.rst)。
