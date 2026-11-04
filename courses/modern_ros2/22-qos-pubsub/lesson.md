# M22 · 从ROS 2发现到QoS实际收包

两个订阅者使用相同话题名和类型，一个有数据，另一个却一直没有回调。再换一个场景，两个订阅者都能收到新消息，但只有一个能补收加入前的样本。本课在当前 ROS 2 教学包会话中实际创建发布端和订阅端，观察可靠性、持久性与历史深度。

Fortress/Humble 使用 `ign` 查询原生世界时间，时钟桥读取 `ignition.msgs.Clock`。Humble 的订阅事件回调来自 `rclpy.qos_event`，Jazzy 从 `rclpy.event_handler` 导入同一接口。两套实验均等待实际 DDS 事件，再核对接收序号和原生时钟。[Humble QoS 事件接口](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos_event.py)

## 节点、端点和ROS图

节点是功能单元，一个节点可以创建多个发布者和订阅者。话题把同名同类型的端点关联起来。课程终端继承本实验的`ROS_DOMAIN_ID`，DDS在相同域及可通信的网络范围中发现节点。发现阶段可以查到端点及其QoS，建立数据连接还要满足匹配条件。回调由执行器调度，程序需持续spin来处理已到达的数据。[Jazzy发现机制](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Basic/About-Discovery.rst) · [rclpy执行器源码](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/executors.py)

```bash
ros2 node list
ros2 topic list -t
ros2 topic info /clock -v
python3 lab.py
```

脚本创建`source`、`reader_a`和`reader_b`，打印本次唯一的`/gt/qos/run_<id>`命名空间。可在另一个继承同一会话环境的操作终端用`ros2 topic info --no-daemon <本次命名空间>/reliability -v`观察短暂运行的实验节点，执行时替换占位部分。脚本结束后会销毁本次节点，图里只剩仍在运行的端点。话题详情能帮助区分发布端与订阅端，并显示类型、节点名和请求或提供的QoS。

## 可靠性匹配

发布端提供一种交付能力，订阅端请求自己能接受的最低能力。BEST_EFFORT尽力交付并允许样本丢失，RELIABLE可借助确认与重传满足更高要求。以下表只讨论可靠性，其他会影响匹配的策略仍需核对。[Jazzy QoS兼容表](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst#qos-compatibilities)

| 发布端提供 | 订阅端请求 | 可靠性匹配 |
|---|---|---|
| BEST_EFFORT | BEST_EFFORT | 可以 |
| BEST_EFFORT | RELIABLE | 不可以 |
| RELIABLE | BEST_EFFORT | 可以 |
| RELIABLE | RELIABLE | 可以 |

实验在同一话题创建BEST_EFFORT提供端、BEST_EFFORT订阅端和RELIABLE订阅端。运行前预测各端点能否收到序号。报告`reliability`把图中发现的一写两读端点、实际DDS RELIABILITY不兼容事件以及两端的收包分开记录。发现两个订阅端后，只有一个与发布者匹配。脚本随后发送Int32序号0至11，要求兼容端收到至少8个有序且不重复的序号，不兼容端收到0条。负例应与收到实测序号的正例一起读。这样可以检查发布者实际发出过样本、执行器确实工作，并定位是某个端点要求过高。仅延长等待或提高发送频率不会改变可靠性匹配。

## 持久性与晚加入

持久性决定较晚加入的订阅者能否得到发布者保留的历史样本。VOLATILE匹配后接收新数据。TRANSIENT_LOCAL允许仍活跃的发布端保留历史，并补给同样请求TRANSIENT_LOCAL的晚加入订阅端。观察同一个TRANSIENT_LOCAL发布者时，可以同时设置两个晚加入订阅者。

| 晚加入订阅者请求 | 加入前的样本 | 匹配后的新样本 |
|---|---|---|
| TRANSIENT_LOCAL | 可收到仍在缓存范围内的历史 | 可以 |
| VOLATILE | 从匹配后的新样本开始 | 可以 |

证明历史补发需要清楚的先后顺序。先发有唯一标识的旧样本，保持发布者存活，再创建晚加入订阅者，等待匹配后另发新样本。旧样本与新样本的标识要区分。只有旧样本的结果会告诉你是否得到历史。两端收到新样本则提供发现和回调正常的对照。本课先发历史序号10、11、12，随后创建两个晚加入订阅者。匹配后再发13。预期TRANSIENT_LOCAL收到历史10、11、12，VOLATILE没有历史样本，两端均收到新序号13。若发布端仅提供VOLATILE，请求TRANSIENT_LOCAL的订阅端会因持久性不匹配而无法连接。[Jazzy持久性说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst#qos-compatibilities)

## History和depth

KEEP_LAST为每个实例保留至多N个样本，N由depth指定。KEEP_ALL受中间件资源限制。深度描述保留多少样本，与发布频率分开。缓存可以出现在发布端与订阅端，先确定实验改的是哪一端。

本课用两个已匹配的RELIABLE订阅者测试接收深度。脚本让执行器暂时不执行接收回调，发布端连续发0至11，等待DDS确认后再恢复spin。深度3的订阅端预期读取9、10、11，深度16的订阅端预期读取全部0至11。DDS确认属于中间件层，应用回调仍需等待执行器读取消息。这个安排把接收缓存的影响与发现延迟分开。对照晚加入实验时，注意前者改变已匹配订阅者的缓存容量，后者观察发布者保留的历史是否交给后来者。

## Gazebo时钟和实验时间

平台用单向`ros_gz_bridge`把Gazebo Transport的时钟转成ROS 2的`/clock`。仿真时钟的数值反映世界推进，实验脚本的等待上限取自单调墙钟。暂停后仿真时间停止前进，墙钟等待仍会结束。脚本先记ROS时钟，再通过原生`gz topic`读一个Gazebo仿真时刻，最后等待更新的ROS时钟。报告检查`0 < ros_before_ns <= native_sim_ns < ros_after_ns`。可在本课终端运行`ros2 topic echo /clock --once --no-daemon`查看当前消息。若已暂停且没有新时钟消息，先继续仿真再观察。[Gazebo与ROS 2桥接](https://gazebosim.org/docs/harmonic/ros2_integration/)

## 练习与排查

先根据两张表写下预测，再运行`python3 lab.py`。用实际接收序号区分历史与实时数据，并查端点QoS。终端JSON按`reliability`、`durability`、`history`与`clock`分组，并记录本次`rmw`实现。单变量练习可在独立对照函数中把原来的RELIABLE请求改为BEST_EFFORT。此时应等待两个匹配端点，并预期两个订阅者收到实时数据。原课的等待条件依赖一次不兼容事件，适合保留作负例。

普通终端看不到节点时，先核对它加载的ROS版本和`ROS_DOMAIN_ID`。能发现端点却收不到数据时，依次查话题类型、可靠性、持久性和执行器。只有晚加入的旧样本缺失时，查看发布端是否仍在、旧样本是否还在发布缓存、订阅请求是否为TRANSIENT_LOCAL。若只有`/clock`缺失，检查Gazebo是否正在推进、桥接方向和进程日志。

独立挑战是为一条低频地图摘要设计晚加入策略。先预测depth=1和depth=3时能得到哪些已发布样本，再用序号和两个晚加入订阅者验证。同时记录发布时刻、匹配时刻和首次回调时刻。
