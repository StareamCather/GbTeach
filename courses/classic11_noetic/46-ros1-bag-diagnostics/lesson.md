# 附加专项 46：rosbag与诊断，怎样留下可以复查的真实证据

第32课记录Gazebo世界状态，这一课记录ROS消息。项目包含两件事：把真实滑块运动录进bag，再用真正的`rosbag play`隔离回放；主动停止状态发布，让另一个诊断节点发现消息变旧，并观察恢复。

你要区分：**原来发生了什么、文件保存了什么、现在回放了什么，以及诊断究竟检查了什么。** 先修21、22、32、35课。

## 数据来源与两个独立节点

默认ROS接口如下：

| 接口或节点 | 内容与职责 |
|---|---|
| `/workshop46/observer` | 查询Gazebo真实关节位置/速度，发布state |
| `/workshop46/state` | `sensor_msgs/JointState`，真实q与v；effort字段为空 |
| `/workshop46/diagnostic_monitor` | 独立订阅实时state，按接收年龄判断数据新鲜度 |
| `/workshop46/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| `/workshop46/observer_ready`、`monitor_ready` | 各自的锁存就绪信息 |
| `/workshop46/observer/emit_state` | observer私有布尔参数，控制是否继续发布state |
| `/clock` | 当前Gazebo仿真时钟，另作为历史数据录入bag |

`JointState.effort`为空，因为本课没有把调度的力指令伪装成受力传感器测量。运动来自真实Gazebo关节；发出一次有限pulse后，记录中的位置和速度应实际变化。

monitor与observer是两个节点。observer停止发state时，monitor仍可运行并给出过期诊断；这比让发布者自己说“我没有数据”更能展示独立监视链路。

## 三种时间，外加消息中的clock值

| 时间 | 含义与用途 |
|---|---|
| bag record time | 录制器收到消息时记录的ROS时间索引，播放器据此安排节奏 |
| `message.header.stamp` | 消息自己的时间语义；本课state是观察节点的clock回读，不是原生采集stamp |
| 主机monotonic接收时间 | 用于接收年龄、墙钟播放跨度和调用预算 |
| `Clock.clock` | 被录下的历史时钟消息内容；Clock本身没有header.stamp |

Noetic录制器在入队时读取ROS `Time::now()`。启用`use_sim_time`时，bag record time可来自仿真时钟，不能一概称作墙钟时间。[官方Recorder实现](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/recorder.cpp#L304)

header stamp与bag时间接近，不等于二者必然相等；主机monotonic时间也不能直接减ROS仿真stamp。本课保存这些原始值，不通过排序、插值或覆盖stamp来假装重建了传感器采样过程。

## 文件与运行流程

| 文件或目录 | 作用 |
|---|---|
| `world.sdf` | 真实滑轨与可观察运动 |
| `instrument.py` | 独立observer/monitor、真实录制回放、文件审计与验证 |
| `ros_ws/src/gazeboteach_workshop46` | source-only ROS包、launch、节点入口与配置 |
| `lab.py` | ready、state、pulse、verify、student-check入口 |
| `student.py`、`reference.py` | 话题选择和健康等级的局部填空与独立参考 |
| `build/bags/<uuid>/motion.bag` | 本次实际录制的独有文件，不覆盖旧bag |
| `build/verification.json` | 录制、文件关闭、回放内容、速率和诊断证据 |

初始化准备个人副本、Noetic环境和私有Master→启动Gazebo与observer/monitor→确认真实输入与独立就绪信息→开放终端。节点定位个人练习根目录，保留个人代码、配置和已有bag。

验收流程是：等待真实录制器订阅就绪→施加一次有限pulse并录制实际运动、diagnostics、clock→约2.6墙钟秒的录制窗口后向本课录制进程发送SIGINT→等待完整关闭→重新打开文件审计→先后启动真正的rate1和rate2播放器→在隔离话题收包并逐条比较→停止实时state流验证诊断→恢复新消息验证回到OK。

Python负责调用工具、观察和审计；回放由真正的`rosbag play`执行，不能用新建Publisher逐条重发来冒充播放器的速率行为。

## 第一组实验：完整录一次有限运动

开始实验，先运行：

```bash
python3 lab.py ready
python3 lab.py state
python3 verify.py
```

验收创建唯一的bag目录，并在报告中给出真实路径。查看那一份文件：

```bash
BAG='/把报告中的实际bag绝对路径填在这里'
rosbag info "$BAG"
```

检查三类topic、类型、条数和起止record time。只有文件名存在不算录制完成；应确认录制器已正常退出、文件可以完整打开、有预期消息，并且录到真实位置变化。

### 手动理解record与SIGINT

本课终端中可以另外录一份独立练习：

```bash
mkdir -p build/manual_records
RUN_DIR="$(mktemp -d build/manual_records/run-XXXXXX)"
rosbag record -O "$RUN_DIR/messages.bag" /workshop46/state /workshop46/diagnostics /clock
```

等待record实际订阅，再在另一个本课终端执行 `python3 lab.py pulse`。观察一段运动后，回录制终端按Ctrl+C，让工具完成关闭，最后再查看 `rosbag info "$RUN_DIR/messages.bag"`。

录制中可能看到`.bag.active`。它表示还在写入，不应当作已经完整关闭的正式文件。不能完整收尾时，应保留文件与日志排查，不能把文件名存在当作通过。平台只管理本实验创建的录制/播放进程，并为等待与收尾设置墙钟边界。

## 回放前先隔离全部topic，尤其是clock

自动回放把三类原话题逐一改名到本次独有的 `/workshop46/replay_<runid>/...`：

| 原话题 | 回放目标后缀 |
|---|---|
| `/workshop46/state` | `state` |
| `/workshop46/diagnostics` | `diagnostics` |
| `/clock` | `recorded_clock` |

播放器使用明确的三topic白名单，不把历史运动指令送进当前执行器。`recorded_clock`只是历史数据，当前Gazebo仍拥有实时`/clock`。

**不加`--clock`还不够。** 它只是不让播放器额外生成播放时钟；bag里原来录下的`/clock`仍会作为普通话题被播放，所以必须显式重映射。否则即使没写`--clock`，也可能把历史clock混入正在运行的仿真。

用你的实际bag进行手动隔离回放：

```bash
rosbag play "$BAG" -r 1 --pause \
  /workshop46/state:=/workshop46/manual_replay/state \
  /workshop46/diagnostics:=/workshop46/manual_replay/diagnostics \
  /clock:=/workshop46/manual_replay/recorded_clock \
  __ns:=/workshop46/manual_replay/player_internal \
  --topics /workshop46/state /workshop46/diagnostics /clock
```

这里给播放器一个独有的`player_internal`命名空间。Noetic播放器即使不启用`--clock`，也会先注册内部管理用的相对话题`clock`；它现在解析到`/workshop46/manual_replay/player_internal/clock`。bag中记录的绝对话题`/clock`仍按上面的显式规则播放到`/workshop46/manual_replay/recorded_clock`。两个名称分开，可避免历史Clock数据复用内部发布器的小队列；当前Gazebo的全局`/clock`保持原来的来源。`__ns`须放在`--topics`之前，避免被后者当成白名单话题。

这个例子以暂停开始，按空格继续/暂停；在另一个本课终端订阅`manual_replay`里的消息观察。不要为了让旧header stamp“看起来新”，把它覆盖成当前时间。更换bag时，要先检查其topic清单并重新明确白名单与重映射，不能默认任意文件都与本课三类topic一致。

## 第二组实验：倍速改变节奏，不改原消息

自动验收分别运行`-r 1`和`-r 2`，两次都实际播放state、diagnostics和记录的clock。比较三项证据：

1. 每个topic的接收条数、序列化内容和原stamp是否与bag一致。
2. 每个topic内部的消息顺序是否保持。不同topic的回调接收全局先后可能受线程调度影响，不必逐次完全相同。
3. 同一组已匹配state消息，从首条到末条的monotonic接收跨度是否约缩短一半。

播放器按bag record time安排节奏。理想情况下 `播放墙钟跨度 ≈ bag时间跨度 / rate`；判断时需排除工具启动、连接等待和手动暂停。不能拿整个子进程用时直接要求精确2倍，也不能以旧header stamp差值不变就说倍速没有生效。

消息原来的采样误差、丢包和传输影响也被保留在记录里。加速回放不会创造新的真实采样；改rate只改变这次播放的墙钟节奏。

## 第三组实验：故意停流，让独立monitor发现它

monitor使用最近一次实时state接收的monotonic时刻计算age，规则是：

| 已收到state后的接收年龄 | 本课等级 |
|---|---|
| age < 0.25 s | OK=0 |
| 0.25 s ≤ age < 0.7 s | WARN=1 |
| age ≥ 0.7 s | ERROR=2 |

在另一个本课终端持续观察：

```bash
rostopic echo /workshop46/diagnostics
```

然后只关闭state发布：

```bash
rosparam set /workshop46/observer/emit_state false
```

预测：随着没有新消息到达，monitor先进入WARN，再进入ERROR；observer与Gazebo并不因此被关闭。诊断不是凭参数false直接判错，而是根据实际接收年龄。

恢复：

```bash
rosparam set /workshop46/observer/emit_state true
```

要等到恢复后**新的实时state**实际抵达，再确认新的OK诊断。一个旧的OK消息、参数服务器已经为true，或者在隔离话题中播放历史state，都不能替代这个恢复证据。

本课OK只表示这条输入状态流最近有消息；它不证明所有ROS节点、渲染、物理时钟、传感器或网络都正常。DiagnosticStatus规范还定义了STALE=3，但本实验使用上面的0/1/2年龄规则，术语“数据变旧”不代表这里已经使用STALE状态码。

## DiagnosticArray应该怎样阅读

DiagnosticArray可包含多个DiagnosticStatus，每条状态有level、name、message、hardware_id和KeyValue列表。先看它声称检查的对象与实际判据，再看等级；不要只凭一个绿色OK图标判断整个系统。

本课关键判据是实时state的接收年龄与0.25/0.7 s门限。真实位置、速度与原始消息可以帮助解释现场，但它们不把这套新鲜度检查扩展成“所有动力学正确”的认证。需要诊断时钟是否前进、值是否物理合理，应另外定义相应检查。

如果希望复盘一次停流故障，需要把故障前、故障中与恢复后的diagnostics都录进窗口；一个只录到正常运动的bag不会凭空包含未录制的故障过程。

## 局部填空与独立挑战

`student.is_selected_topic(topic, allowed)`填写是否属于明确允许的话题集合；不要把字符串包含关系当作精确白名单。

`student.health_level(age_s,warn_s,error_s)`填写按年龄判断0/1/2的关键逻辑。注意恰等于warn或error门限的边界，不能把`age==error`仍留在WARN。

```bash
python3 lab.py student-check
```

完整参考在独立`reference.py`。填空与可运行教学仪器分开，默认`assignment_pass=false`，不会覆盖你的答案或旧bag。

挑战：为一次人为停流录制完整事件窗口，写一个只读审计器，列出诊断变化、各topic数量、record time与原stamp差异，并用隔离回放复查。保留原始数据，审计输出另存，不用排序或插值覆盖历史证据。

## 故障诊断与调用边界

| 现象 | 检查方向 |
|---|---|
| 有bag文件名却打不开 | 录制器是否已SIGINT完整退出，是否仍是active文件，索引与磁盘状态如何 |
| bag里没运动 | 实际录制订阅是否建立，pulse是否真正产生位移，录制窗口是否覆盖运动 |
| rate2没明显缩短用时 | 比较匹配state首末接收跨度，排除连接/启动/手动暂停时间 |
| 回放让实时ROS时间回退 | 录下的/clock是否也被隔离；不加--clock并不能阻止普通/clock话题回放 |
| emit_state=false后仍短暂OK | 根据最后一条实际接收消息计算age，队列中旧消息可能尚未送完 |
| 恢复参数后仍ERROR | 新state是否实际到达；历史回放话题不计入实时monitor |

参数/RPC与owned子进程都使用调用者墙钟边界。不确定的运动请求不自动重发，超时也不等于远端已取消。读取与施力用`workshop::slide`，Noetic清队列用短名`slide`；清任务队列不清零已有速度。

实际录制、关闭、两种倍速、全部topic隔离及诊断恢复的通过状态，以本次运行报告为准，不以命令存在或窗口出现替代。

需要核对工具选项？[Noetic rosbag官方命令实现](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/rosbag/rosbag_main.py)。要查诊断字段？[DiagnosticStatus官方定义](https://github.com/ros/common_msgs/blob/noetic-devel/diagnostic_msgs/msg/DiagnosticStatus.msg)与[DiagnosticArray定义](https://github.com/ros/common_msgs/blob/noetic-devel/diagnostic_msgs/msg/DiagnosticArray.msg)。
