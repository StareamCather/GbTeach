# 参考：真实录制、隔离回放与接收新鲜度

## 正确的信息流

真实Gazebo joint→observer的JointState→真正rosbag record订阅；独立monitor接收实时state，按monotonic年龄发布DiagnosticArray。录制state、diagnostics、/clock后，向本课record进程发送SIGINT并等待完整关闭，再读文件审计。

每次采用build/bags/<uuid>/motion.bag，保留旧bag和个人代码。只有进程启动或active文件出现不能代替完整录制；文件应关闭、可读并包含真实运动。

回放由真正rosbag play执行。三原话题都显式白名单、逐一重映射到唯一replay_<runid>下，历史/clock目标为recorded_clock，不使用--clock。Python重发不是本课播放器验收。

每个播放器还设置独有的`__ns:=<replay前缀>/player_internal`，并放在`--topics`之前。手动例子使用`__ns:=/workshop46/manual_replay/player_internal`。这样先隔离播放器内部注册的相对`clock`，再让历史绝对`/clock`按显式remap进入`recorded_clock`；全局实时`/clock`不接收这两路回放输出。

实现原因可核对Noetic的[TimePublisher构造与发布开关](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/player.cpp#L714)：构造时已注册队列长度为1的相对`clock`，是否生成播放时钟由另一个开关控制。[Player注册bag话题](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/player.cpp#L423)时才使用播放队列大小；若两者解析成同一话题，[roscpp会复用已有Publication](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/topic_manager.cpp#L314)，不会靠后一次注册自动扩大队列。命名空间将两者分开，仍须逐条核对三路消息内容、stamp与顺序，不能用延长进程寿命或放宽缺包条件代替证据。

## 三种时间与rate

record time由录制器接收时读取ROS时间得到；header.stamp由消息自身定义，两者不保证相等。Clock没有header，其clock字段是历史时钟内容。monotonic接收时间只用于这次墙钟等待/播放节奏。

rate2时原消息内容、原stamp、每topic顺序应保持；已匹配state的首末接收跨度约为rate1的一半。不要把启动连接耗时算进播放跨度，也不要把header差不变当成倍速无效。两次回放保留逐条核验与原始接收时间。

即使不--clock，bag里原有的/clock仍会播放，必须remap。隔离后的历史state也不能让只订阅实时state的monitor恢复OK。

## 监视规则与边界

observer私有/workshop46/observer/emit_state关闭只停发state。独立diagnostic_monitor从最后一次实际接收计算墙钟age：小于.25为0；从.25到小于.7为1；大于等于.7为2。恢复新消息后age重置并回OK。

因此health_level可以先判断age>=error，再判断age>=warn，否则OK；参数本身应满足有效门限条件。is_selected_topic用精确集合成员关系，不能用topic字符串片段匹配代替白名单。

标准诊断码还有STALE=3，本课未使用它作为年龄等级。这里ERROR表示输入过期，不证明整个仿真故障；OK表示最近收到state，也不证明物理时钟前进。完整字段与条件应和原始消息一起保存。

## 验证与学习范围

有限pulse提供可录制的真实运动。故意停流要验证独立monitor仍在、WARN/ERROR确实出现，再以恢复后的新实时state和新OK证明恢复。参数已改或旧OK记录都不足以证明恢复。

正式结果见build/verification.json；实际bag独立保存。学生两个局部函数与参考仪器分开，默认assignment_pass=false。等待与收尾有墙钟边界；不重放结果不确定的施力请求。读/施力使用作用域joint名，Noetic清队列用短名slide，已有速度保留。
