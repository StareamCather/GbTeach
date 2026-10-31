# 参考说明：时间来源、严格更新与窗口统计

本说明依据Gazebo Classic 11.15.1与ROS Noetic接口，补充主教材的时间术语和算法。源码实现用于理解数据来源，课程是否完成仍由真实运行、原始消息与状态检查决定。

## 时间字段来自哪里

| 来源 | 原生字段 | 语义边界 |
| --- | --- | --- |
| ImageStamped | time | Classic图像消息的仿真时间标签，不是现实曝光起止时间 |
| IMU | stamp | 该原生接口从incoming LinkData的time取得，不要误读成字段time |
| WorldStatistics | sim_time、real_time、pause_time、paused、iterations | 世界时钟与运行统计，不是每个传感器自己的采样stamp |
| PerformanceMetrics | real_time_factor、各sensor的real_update_rate/sim_update_rate及可选fps | 本身无time字段，不能伪造原生采样时间 |
| 观察者 | 回调接收时间/单调时钟 | 属于观察链路，不能替代原生stamp |

主实验的观察链路是本课SDK观察程序与Python ROS适配管道，并非stock GazeboRosCamera或GazeboRosImuSensor。相机与IMU采用上表的原生字段；程序另记观察/转发/订阅接收时刻。这些接收时刻用于分析该路径，不能倒写进原生消息改变其时间标签。

native_observer.cc在各路原生回调进入后取得observer_receive_monotonic_ns，随后序列化原生消息；到互斥输出处才分配observer_sequence。因不同消息的序列化耗时不同，跨source的输出序号与最初回调接收时间不必给出完全相同的排序。该序号用于识别观察事件，不是原生硬件帧号或全局采样顺序。

原生ImageStamped中的image.data在SDK输出的protobuf JSON中以base64保留。Python解码真实原字节后转存image-内容SHA256.rgb，将data从报告/ROS事件中移出，改记binary_storage.path、sha256、byte_count及removed_field。当前图像为128×96的R8G8B8，原生pixel_format是Classic RGB_INT8的数值3，step=384、共36864字节；按当前Classic接口解析，不能套用其他版本的枚举含义。报告引用与.rgb共同保留原始像素；ROS事件传的是字段与文件引用，不是标准ROS Image像素流。IMU原生stamp来自传入LinkData，而非替换成C++或Python接收时刻。

metadata_plugin.cc是只读WorldPlugin，读取实际rendering_lockstep_enabled、sensor.StrictRate/UpdateRate、物理步长、real_time_update_rate和world迭代数，并监听Camera成功Update信号、维护本地累积计数。它通过原生Any发布一致仪表快照，内部world_sim_time_ns属于该快照；Any本身没有传感器原生stamp。不能将这一路元数据当作相机/IMU又多采了一帧。

| 本课记录字段 | 来源与用途 |
| --- | --- |
| source_stamp_ns | 相机time、IMU stamp或WorldStats sim_time；Perf/metadata为null |
| observer_receive_monotonic_ns | C++原生回调进入时CLOCK_MONOTONIC |
| observer_sequence | C++互斥输出序号，用于识别事件 |
| python_receive_monotonic_ns | Python从SDK stdout读到一行的时刻 |
| ros_publish_monotonic_ns | 本课适配器调用ROS发布的时刻 |
| ros_receive_monotonic_ns | 本课真实ROS订阅回调收到的时刻 |
| stream_id | 区分本次ROS适配实例，避免混入其他观察器事件 |
| binary_storage | 相机原字节路径、SHA256和布局引用 |

SDK输出的纳秒/序号为十进制字符串，Python按契约转换为整数；null保持为null，不用0伪装。整型纳秒是表示单位，实际采样分辨率仍受步长和调度约束，不表示测量具有纳秒级精度。相同主机Linux CLOCK_MONOTONIC用于各进程耗时差，不能把这些主机时间与某个world的模拟stamp直接相减。

ros_send_to_receive_ms统计发送到真实订阅回调的该路径耗时；observer_to_ros_receive_ms还包括SDK序列化、stdout、Python读取与转发等步骤。两者都不是纯网络时延，更不是由原生仿真stamp减出来的现实曝光延迟。由于像素已外存，不能将这些数值当作完整ROS Image运输基准。

可直接查官方[image_stamped.proto](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/image_stamped.proto)、[imu.proto](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/imu.proto)、[world_stats.proto](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/world_stats.proto)、[performance_metrics.proto](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/performance_metrics.proto)。不同类型的字段名与语义不完全相同，泛型桥不能对所有消息只找time，再把缺失处随意填成收到消息的时刻。

### WorldStats.real_time不是操作总耗时

[World.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/World.cc)的RealTime使用世界启动后的实际时间减去realTimeOffset；暂停时取pauseStartTime，恢复时将这段暂停耗时累加进offset。因此real_time不包含累计暂停等待，暂停时也固定。PublishWorldStats分别填写SimTime、RealTime、PauseTime、iterations与paused。

独立观察者使用monotonic计算整体实际耗时，仍包含人停在暂停状态的等待。若要比较两个RTF，选择连续运行窗口或明确暂停处理方式；不要把不同时间源强行当作相同分母。

### 相机与IMU不是同一回调阶段

[CameraSensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/CameraSensor.cc)在strict-rate路径由PrerenderEnded安排下一渲染时刻，Render真正渲染，UpdateImpl在PostRender后发布原生ImageStamped；原生图像时间取scene SimTime。某些ROS相机包装使用Sensor的LastMeasurementTime，不能在未查实现时将不同包装的stamp来源混为一谈。

本课额外监听Sensor::ConnectUpdated。根据[Sensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/Sensor.cc)，只有UpdateImpl返回true才触发updated信号；Camera.UpdateImpl在rendered为false时返回false，成功路径经过PostRender后返回true。该钩子统计成功原生sensor Update事件，不将其命名为硬件曝光、完整capture或已经成功交付Image消息。Camera LastMeasurementTime在渲染调度阶段设置，而ImageStamped.time在发布路径读取scene时间，二者须保留来源区别。

[ImuSensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/ImuSensor.cc)读取传入LinkData.time，设置lastMeasurementTime和原生IMU.stamp，并据LinkData速度计算加速度。读取的状态、世界当前姿态和观察者回调仍有更新阶段差别；stamp相同不能扩展成“所有物理量来自严格同一个硬件瞬间”。

ROS使用仿真时钟的基本消息见[Clock.msg](https://github.com/ros/ros_comm_msgs/blob/noetic-devel/rosgraph_msgs/msg/Clock.msg)，时间抽象可查[Noetic rospy.rostime实现](https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/rostime.py)。平台自动准备该环境，课程观察程序仍需单独记录实际耗时，避免在暂停的ROS仿真时间上等待一个墙钟超时。

## 相机成功Update的独立计数

| 层次 | 证据 | 所证明的范围 |
| --- | --- | --- |
| 目标配置 | 实际Sensor UpdateRate与StrictRate | 服务器当前的目标参数 |
| 相机成功原生Update | 服务器内ConnectUpdated计数与LastMeasurementTime一致快照 | 挂钩后窗口内成功更新事件的节奏 |
| 原生消息被观察 | native ImageStamped.time、raw事件与unique标签集合 | 这条观察链路实际收到的事件/标签 |
| ROS回调接收 | 实际ros_publish/receive单调时刻 | 本课ROS事件交付路径 |

成功Update钩子是轻量只读观察，不改变sensor rate、lockstep或世界状态；卸载时断开连接，计数和测量时间成对快照。仪表可能晚挂接，所以event_count是局部累计值，不能当成world从零开始的全量渲染/曝光计数。元数据接收也可能迟到，窗口按生产者counter建立独立统计索引，raw metadata观测顺序仍保留。

实际metadata.camera.producer_update字段如下，整数与纳秒以十进制字符串保存：

| 字段 | 含义 |
| --- | --- |
| hook_attached | 是否已连接真实成功Update信号 |
| hook_requested_world_sim_time_ns | 请求挂接时的world模拟时间上下文 |
| hook_requested_monotonic_ns | 请求挂接时主机单调时间上下文 |
| event_count | 挂钩后成功Update事件累计数 |
| first_measurement_time_ns | 首次被计数事件的Camera LastMeasurementTime |
| last_measurement_time_ns | 最近被计数事件的Camera LastMeasurementTime |
| producer_last_callback_monotonic_ns | 最近成功Update回调的主机单调时刻，不是Image接收时刻 |
| same_stamp_count | 成功Update事件测量标签与前次相同的累计次数 |
| regressed_stamp_count | 成功Update事件测量标签相对前次回退的累计次数 |

window.producer_updates.camera保留first与last两份证据，各含原metadata事件observer_sequence与producer_update快照。只使用hook_attached=true且event_count>0的快照，按counter建立独立索引并取最小/最大counter；同一counter的重复快照必须内容一致。off/on两个子实验都等待至少1模拟秒的相机成功Update计数窗口，它与consumer标签覆盖跨度分开。本课只给相机增加该counter，IMU没有新增producer计数或门限。

```text
event_count_delta = C_last − C_first
measurement_span_s = (last.last_measurement_time_ns − first.last_measurement_time_ns)/1e9
successful_update_rate_hz = event_count_delta / measurement_span_s
```

计数差本来就是两快照间的新成功事件数，不再减1。报告另含same_stamp_delta、regressed_stamp_delta、snapshot_count与unique_counter_count，后两者分别是符合条件的快照数量和不同counter数量；它们帮助检查仪表窗口，不能与consumer重复/回退混成同一层。hook_requested时间用于说明挂接上下文，不作为rate分母。内置PerformanceMetrics瞬时sim_update_rate也不替代这条窗口计数证据。

## 完整lockstep启动链路

```text
gzserver启动时解析 --lockstep
  → rendering lockstep flag
  → 初始化世界时选择直接更新渲染场景姿态的回调
  → Sensor.Load缓存strict-rate状态
  → Camera安排NextRequiredTimestamp
  → SensorManager取下一图像要求时刻
  → World.Step推进前按要求等待传感器/预渲染
```

[Server.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/Server.cc)不仅设置rendering标志，还在初始化physics worlds和Run循环中使用该模式。只在运行时调用rendering::set_lockstep_enabled不会重做全部启动链路，不能将它冒充与启动参数等价的切换。

[SensorManager.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/SensorManager.cc)的NextRequiredTimestamp遍历活跃IMAGE传感器并选最早有效值；WaitForSensors根据当前时刻、步长和下一要求时刻等待预渲染。World.Step调用这个等待函数后再继续推进。[Sensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/Sensor.cc)对IMAGE与非IMAGE有不同节流路径，不能断言所有传感器都共享相同时间补偿算法。

相关用户教程为[Classic物理与传感器lockstep](https://classic.gazebosim.org/tutorials?tut=lockstep_physics_sensors)。本课讨论Classic物理与图像传感器调度；比较必须完整创建两次服务器生命周期，分别保存模式与原始证据。

## 观测标签、接收率与RTF的计算

主窗口先保留原始消息，按本路观测顺序选择每个标签首次出现的代表事件，要求其不同标签严格递增。对同一组N个代表事件，标签s_i与观察者时刻w_i一一对应，可计算f_sim=(N−1)/(s_last−s_first)、f_wall=(N−1)/(w_last−w_first)及同事件端点的ratio_span=标签跨度/接收跨度。这个样本跨度比不直接冒称世界RTF；它们都描述这份观察记录，不额外证明生产者每个内部更新都已到达。

child则保留两种不同视图，不能把标签排序结果与原receipt按数组下标重新配对：

```text
原始观测事件：保持trace顺序和每条原始标签、receipt、observer_sequence
  → 统计接收事件、重复标签、相邻原始事件的标签回退
  → 从同一raw记录另建ascending unique label索引
       → 标签覆盖密度、标签间隔网格、离线nearest查询
```

| channels字段 | 精确定义 |
| --- | --- |
| received_count | 本路窗口内原始接收事件数M，包含重复标签 |
| unique_stamp_count、count | 不同原生标签数U |
| duplicate_stamp_count | M−U；只称重复时间标签事件，不推定重复包/重复曝光 |
| stamps_ns | main为首次观测不同标签的严格递增原顺序；child为升序unique标签视图 |
| stamps_order | strict_unique_arrival或ascending_unique_native_label，声明上行的视图 |
| stamp_span_s | 标签最大值与最小值之差 |
| sim_rate_hz | (U−1)/stamp_span_s，保留字段名，解释为观察标签覆盖密度 |
| sim_rate_interpretation | observed_unique_time_label_coverage_density |
| arrival_stamp_regressions | 相邻raw本路事件b<a的次数、max_backstep_ns与前5个原序号/标签例子 |
| observer_receive_span_s | 全部raw事件的max(receipt)−min(receipt) |
| observer_wall_rate_hz | (M−1)/observer_receive_span_s，原始实际接收率 |
| unique_stamp_observer_receive_span_s | 每个标签首次观测代表事件的receipt范围 |
| unique_stamp_observer_wall_rate_hz | (U−1)/该代表事件receipt范围，另列去重分子与分母 |

ROS主链路的ros_wall_rate_hz同样按原始实际ROS接收事件和其receipt范围计算。原始标签和接收时刻不改；gap/grid在相应声明的标签视图中计算，不能将child升序gap宣传成原始到达间隔已连续或同步已修复。

同一source的标签相对观察序回退，只证明这条观察链路中标签与该事件顺序不一致。仅凭此记录不能定位源调度、交付、线程或网络中的具体因果层。原始重复/回退诊断必须与密度一起呈现，不能在去重排序后隐藏。

若窗口没有推进、N<2或出现时钟重置，分母无意义；应明确标记不可计算，不能除零后把结果改成目标Hz。一次world重启/重置后应建立新窗口，不把重置前后的stamp串成负时长。

f_wall=f_sim×ratio_span只对前述主窗口中相同代表事件端点和计数成立。child的raw接收率与升序unique标签密度分子、事件边界不同，不能直接套用该等式，更不能拿标签覆盖跨度除receipt范围就声称得到世界RTF。WorldStats的world_statistics_rtf单独按其sim_time/real_time窗口计算；内置指标也有自身窗口。配置Hz是输入参数，不直接填成测量结果。

本课接收字段使用主机单调时钟，不能直接减原生模拟stamp得到传输耗时；它们的起点与推进尺度不同。实际处理耗时使用同一主机的observer/send/receive单调时钟差。若另外记录了同world的接收仿真时间，它与原生stamp的差也只反映该仿真路径的时间年龄，不能自动拆成纯网络时延；不同world的模拟时间更不能直接相减得到延迟。

## PerformanceMetrics里的Hz和fps

原生指标主题为/gazebo/performance_metrics，字段按sensor name区分。real_update_rate、sim_update_rate与可选camera fps是生产者内置统计，renderer的平均fps不等于某个ROS订阅者的接收Hz。该消息没有原生time字段，只能另记接收/观察时间。

源码中只有在存在订阅且仿真时间有推进等条件下才发布有效新统计；短窗口、暂停、启动过渡或零分母不能被当作稳态测量。不要只摘一条内置即时值与另一个长窗口平均值比较。官方例子及其取样差异见[Performance metrics教程](https://classic.gazebosim.org/tutorials?tut=performance_metrics)。

## 步长网格与样本配对

ODE固定步长h时，模拟状态只能在相应离散时刻推进。周期T=1/f为h的整数倍，使该周期可被网格精确表示；非整数时不可能每段间隔都严格等于T。实际可能有调度抖动、补偿或平均率偏差，必须按当前传感器/运行模式测量，不预设400 Hz一定表现为某种间隔序列。

exact matching保留原stamp，只接收两路相等者；nearest matching保留两路stamp、差值与最大容差。不能通过覆盖stamp来“同步”。配对率、未匹配样本、边界样本和时间差分布应一起呈现，否则只报告成功匹配的平均误差会掩盖未配上的数据。

相同stamp只表明标签一致，不能推导现实硬件触发、曝光区间、IMU积分时间或完整数值可重复性。lockstep改进物理/传感器调度一致性，也不自动提供这些额外模型。

## 本课的比较边界

主world固定lockstep，默认camera128×96@20 Hz、IMU100 Hz，h=0.001 s、real_time_update_rate=1000；保存参数支持camera10/20 Hz和h=0.001/0.002 s。步长变化联动real_time_update_rate=1000/500，保持dt×rate=1这个名义上限，不保证实际RTF必须达到1。

连续主窗口采集2仿真秒后，实际暂停与步进分别检查时间停止和N步量化，不从暂停期间的零分母计算“传感器实际Hz”。WorldStats在暂停时仍可能有新的状态消息但相同sim_time；这种统计通知不等于产生了新的相机/IMU采样。

两个对照child使用相同临时场景副本，real_time_update_rate共同为0，串行完整启动/结束gzserver；唯一模式差别是启动参数`--lockstep`。它们各有私有Gazebo Master与Ignition partition，原生观察不另开ROS。主world在比较期间暂停，比较完成恢复原暂停状态；本课不接管其他项目进程。

不要预先要求off一定漏帧、on比off更快、RTF一定大于或小于某个绝对值。可要求来源、模式、配置、窗口与数据契约正确，再用真实结果讨论差异。初期PerformanceMetrics可能出现不适于解释的数值，保留原始诊断但不拿它充当已通过的成功门槛；稳态结论以明确窗口的原生事件和world统计为依据。

## 文件、构建与输入契约

| 文件/目录 | 内容 |
| --- | --- |
| world.sdf | 实际RGB相机、IMU、静止试架与场景；主服务器以--lockstep启动 |
| metadata_plugin.cc | 只读WorldPlugin，报告实际配置与相机成功Update一致计数快照 |
| native_observer.cc | 原生Gazebo Transport观察器，保留各消息及独立接收时间 |
| prepare.py | 用已安装SDK构建私有观察器/元数据库，记录来源指纹 |
| config.py | 固定场景预检；只开放camera-hz与step-ms保存输入 |
| lab.py | 原生probe、真实ROS事件observe、独立child比较及参数保存 |
| verify.py | 连续窗口、暂停、真正步进、child比较与恢复 |
| build/sync_lab.json | 当前准备配置、源码和构建产物SHA256 |

输入支持`python3 lab.py config --camera-hz 10`或`--camera-hz 20`，以及`--step-ms 1`或`--step-ms 2`。步长改动联动real_time_update_rate维持主RTF名义上限；保存后必须重载。预检会拒绝未加载的新world输入或变化的观察器/元数据产物，避免报告对应到另一套运行程序。这是来源契约检查，真实采样与步进仍由运行结果验证。

## 维护命令的实际行为

| 命令 | 数据路径和状态作用 |
| --- | --- |
| python3 lab.py ready | 纯原生就绪窗口，至少1仿真秒 |
| python3 lab.py probe --seconds 2 | 纯原生，最低2仿真秒，不主动恢复运行 |
| python3 lab.py observe --seconds 2 | 原生→Python ROS发布→真实订阅；临时运行后恢复原pause |
| python3 lab.py compare | 主world暂停，串行创建off/on隔离child后清理并恢复原pause |
| python3 verify.py | 完整连续/暂停/步进/独立比较，结束恢复原pause |

probe/observe的seconds是最低仿真跨度，支持0.5至5秒，不是固定墙钟秒；compare采用固定最低窗口，没有seconds参数。采样先等待camera、imu、world_stats、performance_metrics、metadata五种来源就绪；这里只前两路是本课要比较的传感器采样流。每路至少8个相机事件或20个IMU事件并达到所需跨度，child还要求至少2实际秒与1仿真秒，均有有限墙钟等待。PerformanceMetrics存在性用于确认来源连接，数值仅诊断。

默认lab报告在会话目录timing-唯一编号/report.json，verify在synchronization-唯一编号/verify.json；可用`--report PATH`指定JSON。图像原始.rgb和观察日志仍在本次创建的会话目录，外移JSON时需要保留它引用的文件。SDK/主观察器/child服务器退出结果也记录在报告；它只停止明确由本课创建的PID。

## 自动验收采用的判据

| 项目 | 验证方式 |
| --- | --- |
| 主相机观测标签密度 | 不同标签原顺序严格递增，相对配置值误差≤2% |
| 主IMU观测标签密度 | 不同标签原顺序严格递增，相对100 Hz误差≤5% |
| 声明的标签视图间隔网格 | 各间隔除以实际h接近整数，误差小于10⁻⁵个tick；child不冒称raw顺序已连续 |
| 主窗口nearest差 | 不超过0.01+2h秒，h为实际物理步长 |
| 真正暂停 | 至少0.35实际秒内，原生sim_time/iterations及ROS服务查询世界时间不推进 |
| 真正单步 | iterations差=1，Δsim=h，结束仍暂停 |
| 真正多步 | 目标0.1仿真秒；1 ms时100步，2 ms时50步，Δsim=N×h |
| 模式与配置 | 实际metadata的flag、StrictRate、sensor rate、h和限速值符合本次模式 |
| lockstep_on child | 相机成功Update率Δcount/ΔmeasurementTime相对配置误差≤2%，测量跨度≥1 s，producer same/regressed增量均为0；consumer密度/率/回退仍只诊断 |
| child对比完整性 | 同一临时benchmark.sdf指纹、完整独立服务器生命周期、私有Master/partition、清理结果 |
| 真实图像证据 | 128×96、step384、36864原字节、SHA256及实际中心像素 |

连续主窗口至少2仿真秒，首先保留原始事件，再将重复native stamp标成duplicate_native_stamp；它们不增加unique计数。main首次观测的不同标签必须按原顺序严格增加，仍使用原20/100 Hz误差门限；child才允许用独立升序标签视图统计，并显式报告原观测序回退，不以排序“修好”原始消息。

nearest是离线标签查询：用排序索引仅查落在已观察IMU覆盖区间内的相机标签，不跨窗口边缘外推。保存camera_stamp_ns、imu_stamp_ns、signed_skew_ns及camera_observer_sequence/imu_observer_sequence，序号指向各标签首次观测的代表事件。它可能用到较晚才收到的事件，不能当作在线同步器已在当时拿到数据的证明。完整相机标签另存，边界未配数量可由总数减配对数复核。exact策略是独立比较练习，当前自动结果不冒称实现了exact matcher或融合。

暂停/步进的ros_service_world_sim_time_ns来自通过ROS GetWorldProperties服务读取的Gazebo世界时间，ros_service_paused记录服务返回的暂停状态；它们并非另外订阅到的`/clock`事件或所有ROS节点的Time.now。WorldStats新鲜回执、实际服务结果与迭代差共同验证步进；`gz world`命令返回成功本身不足以证明已前进指定步数。

完整verify恢复进入时的pause，已前进的时间和迭代数保留；compare也不会将主world的模拟时间与child重新起点拼接。错误/取消路径尝试清理本课观察器与child，并记录退出结果，不能用无界等待代替可诊断的失败。
