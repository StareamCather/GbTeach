# 附加专项 45：TF与同步，把“在哪里”和“什么时候”分开验证

本课在滑块上安装一个有平移、有旋转的真实`probe`刚体，建立TF链，并把一次真实关节查询拆成位置、速度两路消息。你要回答两个问题：**probe里的一个点怎样转换到world？两条晚到或时间戳不同的消息，为什么有时能配对、有时不能？**

先修03、21、24课。这里的probe是安装参考框架与真实SDF link，不是自动生成的相机或IMU。

## 一棵可以与真实实体核对的TF树

默认关系是：

```text
world
  └─ rail       静态：z = .7 m
       └─ cart  动态：x = 实际关节位移q
            └─ probe  静态安装：(.1,.15,.05)m，绕z偏航+90°
```

`rail`在这里是选定的滑轨参考frame，原点放在滑块中心高度；它不必等于黄色导轨visual的几何中心。TF的frame字符串与Gazebo实体名称也不会自动对应，映射由仪器明确建立。

`world→rail`与`cart→probe`是静态关系；`rail→cart`随真实滑块运动变化。静态关系通过`/tf_static`发布，动态关系进入`/tf`和监听器的时间缓存。静态安装来自个人`world.sdf`中真实`workshop::probe` link的`pose relative_to="body"`，不是另一份写死的教材常数。

默认probe原点在world里应为：

```text
(q + .1, .15, .75) m
```

球心或刚体原点本身不能检验旋转是否正确，所以验收还使用probe中的非零局部点 `p=(.2,0,0)`。偏航+90°把该方向转到+y，得到：

```text
p_world = (q + .1, .35, .75) m
```

这里的`.75`是`.7+.05`；`.35`是`.15+.2`。修改安装高度或偏航后，应重新推导，不要保留默认结果冒充预测。

## 变换方向：lookup的两个frame不能颠倒

用记号`T_world_probe`表示“将probe坐标表达转换为world坐标表达”的变换。对局部点：

```text
p_world = R_world_probe * p_probe + t_world_probe
```

对应查询为：

```text
lookup_transform(target_frame="world", source_frame="probe", time=Time(0))
```

target是输出坐标系，source是输入坐标系。反过来查询会得到逆变换；逆变换的平移不是简单取负，还要经过逆旋转。多段组合顺序也重要：`T_world_probe = T_world_rail * T_rail_cart * T_cart_probe`。

`Time(0)`在TF查询中表示最新共同可用时刻，不是请求仿真开始时刻，也不是把各条边互不相干的最新数据随意拼接。动态缓存必须在同一条路径上支持所需时刻。查询`now()`时，如果最新TF尚未到达，就可能出现向未来外推错误。

[tf2官方Buffer接口](https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/buffer.py)定义了target/source和time参数，排查方向问题时应对照它。

## 旋转与四元数

ROS四元数按`x,y,z,w`字段保存旋转。绕z偏航+90°时，可写为约`(0,0,.7071,.7071)`；它不是四个RPY角。有效单位四元数的范数应接近1，`q`和`-q`表达相同旋转，因此不能仅凭四项符号不同判断两个姿态不一致。

本课对默认z轴旋转有直观预测，但实际变换仍要使用旋转矩阵或四元数算法。非零局部点能揭示“只加平移、忘掉旋转”的错误。

## 文件与信息流

| 文件或接口 | 作用 |
|---|---|
| `world.sdf` | 真正的probe link、相对body安装pose与fixed joint |
| `instrument.py` | 从个人SDF读取安装关系，查询真实关节并广播TF，发布同步实验消息 |
| `ros_ws/src/gazeboteach_workshop45/config/controller.yaml` | 软件发布延迟、注入时间戳偏差与近似配对容差 |
| `lab.py` | ready、state、pulse、verify、student-check入口 |
| `student.py` | 局部点变换、时间差两处独立函数练习 |
| `reference.py` | 独立完整参考，不覆盖学生文件 |
| `/workshop45/publication_trace` | String中的JSON源迹，关联独立观察事务ID与三路实际发布头字段 |
| `build/verification.json` | TF、真实link、原始包与配对、源迹关联和两种时间域的延迟证据 |

初始化准备个人副本、Noetic子环境和私有Master→检查SDF与个人参数→启动Gazebo及本课仪器→读取真实安装关系→发布静态TF与实际关节驱动的动态TF→打开真实终端和图形。节点入口定位个人练习根目录；初始化保留个人源文件。

几何验证走两条路径：一条用TF监听器从真实广播和缓存查询`world←probe`；另一条直接调用Gazebo的`GetLinkState`读取真实`workshop::probe`位姿。比较原点、旋转与同一个非零局部点，避免把仪器自己算出的TF当成唯一真值。两路查询的时刻也要记录，不能宣称它们是原子同刻。

## 三路消息，只有一次真实关节观察事务

默认namespace为`/workshop45`：

| 话题 | 类型与内容 | 时间处理 |
|---|---|---|
| `/workshop45/pose` | `PoseStamped`，rail坐标中的q，单位m | 使用本次观察事务stamp |
| `/workshop45/companion` | `TwistStamped`，真实关节速度，单位m/s | 受控延迟发布，保留同一原始stamp |
| `/workshop45/companion_offset` | 同一速度的影子通道 | 在原始stamp上人为加offset |

三路frame_id均为`rail`。Pose中的单位旋转是这个一维位置表示的一部分；probe的真实安装旋转由TF链描述。Twist的线速度不是加速度，不能把它称作IMU输出。

位置与速度来自**同一次真实GetJointProperties观察事务**。companion是用来演示消息配对的第二路输出，不是第二个独立物理传感器。stamp来自教学节点对`/clock`的回读，并保留查询前后时间信息；它不是底层传感器的原生采集时间，也不证明关节读数与clock是原子快照。

### Header.seq不是跨话题的共同事务号

Noetic rospy序列化带Header的消息时，会用每个Publisher自己的计数覆盖`Header.seq`；没有订阅连接时还会跳过该次序列化计数。不同通道的订阅建立时刻不同，raw seq就可以合法不同。不能把它们改成相同数字后，再宣称数据来自同一次观察。[rospy官方序列化实现](https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/msg.py#L119)

本课另行维护`source_observation_seq`，并通过`/workshop45/publication_trace`发布如下关联：

- 共同来源：`source_observation_seq`、`query_stamp_ns`。
- 实际发布头：`pose_publisher_seq/pose_stamp_ns`、`companion_publisher_seq/companion_stamp_ns`、`shadow_publisher_seq/shadow_stamp_ns`。
- 三路调用`Publisher.publish`时回读的仿真时间：`pose_publish_call_sim_ns`、`companion_publish_call_sim_ns`、`shadow_publish_call_sim_ns`。

验收以`(话题种类, 原始stamp, 该Publisher的raw seq)`查源迹，映射回共同观察ID，再检查配对是否同源。接收窗口边界可能只有消息而没有完整源迹；这类未映射对单列计数，不猜测来源。原始seq与stamp始终保留，源迹和接收证据写入`build/verification.json`。

## Exact与Approximate分别验证什么

`TimeSynchronizer`按`header.stamp`精确相等配对。消息到达的先后顺序可以不同，只要队列中仍保留相同stamp，Exact就可以配成一组。

`ApproximateTimeSynchronizer`按`slop`窗口允许时间接近的stamp。本课Noetic实现要求同组最大与最小stamp的跨度**严格小于slop**，恰好等于边界也不配。它按时间搜索候选，不会自动校正时钟、插值状态，也不会根据`header.seq`知道两条数据是不是同一次采样。因此本课通过独立观察事务ID与发布源迹核对同源关系，保留每一路原始Header.seq，不要求它们跨话题相等。

默认设置：

```yaml
delay_sim_s: 0.03
offset_sim_s: 0.012
slop_s: 0.02
```

| 配对对象 | 默认预期 |
|---|---|
| pose与延迟companion，Exact | 能配；原始stamp相同 |
| pose与offset影子通道，Exact | 不能配；人为加了12 ms |
| pose与offset影子通道，Approx，slop20 ms | 能配，但时间戳仍差12 ms |
| offset12 ms、slop改成5 ms | 影子Approx应零配对 |

这张表是预测，不代替真实运行报告。同步器的队列长度也重要：等不到配套消息前，旧消息已被挤出，即使stamp相同仍可能丢配。

本课受控观察事务至少相隔0.06仿真秒，offset限制为0～0.03 s、slop为0.001～0.02 s，避免拿下一事务的消息借配来伪装同步。真实系统里仍需分析各传感器频率、抖动和序号，不能只把slop越调越大。

## 操作：先看几何，再看时间

开始实验后，在本课终端运行：

```bash
python3 lab.py ready
python3 lab.py state
rosrun tf tf_echo world probe
```

这里使用已安装的ROS 1 `tf_echo`命令查看变换，教材中的监听缓存与程序查询仍采用TF2接口。

另开本课终端查看消息：

```bash
rostopic echo -n 1 /workshop45/pose
rostopic echo -n 1 /workshop45/companion
rostopic echo -n 1 /workshop45/companion_offset
```

查看各通道原始seq、stamp、frame_id和单位，并用 `rostopic echo -n 1 /workshop45/publication_trace` 查看独立来源关联。分次执行echo截到的可能不是同一观察事务，真正配对要结合连续捕获的源迹或验收报告。再运行一次有限脉冲：

```bash
python3 lab.py pulse
python3 verify.py
```

预测cart和probe的世界x随滑块运动，而cart→probe的静态安装关系不变。验收会记录来自真实消息的配对、TF查询和原生link读回；具体通过范围以报告为准。

默认控制namespace变体仍使用ROS重映射，例如变体已经完整启动时可执行 `python3 lab.py state __ns:=/demo45`。ROS namespace只改变接口名，不会自动给`frame_id`加上`demo45/`；TF frame字符串要按系统设计另行明确。两个节点若在同一个TF图中发布相同child frame，可能产生竞争，不能把命名空间当作TF自动隔离。

## 单变量实验一：改真实安装，不改验收答案

在个人`world.sdf`中找到真实probe link的相对pose。先只把z从0.05改为0.10，重启，预测probe世界高度从0.75变为0.80 m。仪器从这份SDF读取mount，新的TF应与新的真实link一致，而不是要求你到验证程序里改一个硬编码高度。

恢复高度后，只把偏航从`pi/2`对应弧度值改为0.3 rad。非零点的旋转部分应变为 `(.2*cos(.3), .2*sin(.3), 0)`，再加安装平移与滑块位移。修改SDF时填写数值弧度，不把字符串`pi/2`当作可执行表达式。

## 单变量实验二：延迟、偏差与slop

先保持offset和slop不动，仅改变一组合法的`delay_sim_s`并重启。companion的软件发布更晚，但原始stamp不改，Exact仍应配对。验收至少关联8组完整源迹与接收样本，分开检查两种证据：

1. 在源节点的仿真时间域中，`companion_publish_call_sim_ns - query_stamp_ns`不小于配置delay。这里记录的是调用publish的时刻，不是线上真正发送或订阅者接收时刻。
2. 对映射到同一`source_observation_seq`的pose与companion，计算主机monotonic接收时刻之差；正delay条件下，中位数应大于0，确认整体上companion较晚送达。

单条接收差可能受传输/回调调度影响，不能把每条墙钟差都要求等于仿真delay，更不能用主机时钟减ROS仿真stamp。这是受控的软件发布延迟，不是已经模拟了真实网络的全部特性。

恢复delay，只把`slop_s`从0.02改成0.005，保持offset=0.012。预测影子Approx零配，但正常companion的Exact仍能配。把“拒绝不合容差的消息”设计为明确的负例，比为了得到配对而改写stamp更能说明同步器在做什么。

本课的`offset_sim_s`是故意注入的时间戳偏置，用来研究配对规则。它不会改变真实关节状态，也不表示已经估计或校准了两个硬件时钟。

## TF与时间的常见错误

| 现象 | 检查方向 |
|---|---|
| 点的平移正确，朝向不对 | 是否漏掉R、乘法顺序错误，或把四元数当RPY |
| Extrapolation到未来 | 请求时间是否晚于最新动态TF；尝试理解Time(0)，不要随手覆盖消息stamp |
| Extrapolation到过去 | 查询时间是否已超出动态缓存，或混用了旧时钟数据 |
| frame找不到或树不连通 | 核对frame字符串、parent/child关系及广播者 |
| namespace改了，frame没改 | 这是两个命名体系；按设计显式定义frame前缀 |
| Exact没有配对 | stamp是否逐项相同，消息是否因延迟和队列限制被丢弃 |
| Approx配上了但语义不对 | 经publication_trace映射后检查共同观察ID；raw seq不必相同，时间接近也不代表同一物理事件 |
| 暂停后消息/延迟队列不前进 | delay由仿真时间驱动，等待操作仍应受墙钟预算限制 |

所有仪器与验收等待都有调用者墙钟预算。原生RPC超时不等于远端请求被撤销，不确定的运动请求不自动重放。读取/施力用`workshop::slide`，Noetic清队列用短名`slide`；清队列也不清零惯性速度。

## 两处关键练习与独立挑战

`student.transform_point(R,t,p)`接受3×3嵌套Python列表和两个三维向量，填写每行点积加平移；这里不需要额外引入NumPy。`student.stamp_delta(t0,t1)`接受以秒表示的数值，计算两stamp的绝对差。完成后运行：

```bash
python3 lab.py student-check
```

独立完整参考在`reference.py`；函数检查和教学参考物理验收分开，默认`assignment_pass=false`，不会覆盖你的答案。

挑战：设计一个带非零roll、pitch的安装，手工预测一个局部点，再与真实link与TF对照；同时保留配对前的原始seq、stamp、接收时间和独立观察ID关联，绘制时间差分布。进一步把真实相机或激光接进来时，应使用各自原生采集stamp，说明曝光/扫描周期与传输延迟，而不是沿用本课同一次关节事务拆流的简化假设。

不了解ROS单位和坐标轴？[REP103官方规范](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)也解释了相机光学frame与普通机体frame的区别。要核对同步器具体规则？[Noetic message_filters官方实现](https://github.com/ros/ros_comm/blob/noetic-devel/utilities/message_filters/src/message_filters/__init__.py)。真实传感器的数据链路可回看15、17、21课。
