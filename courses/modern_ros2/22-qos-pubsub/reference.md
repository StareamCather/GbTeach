# 参考 · QoS匹配与历史交付

BEST_EFFORT提供端可匹配BEST_EFFORT订阅。RELIABLE订阅要求RELIABLE提供端。提高发布频率不会让不兼容端点匹配。调试时先查图中端点，再查实际回调收包。[Jazzy QoS兼容表](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst#qos-compatibilities)

TRANSIENT_LOCAL发布者保持存活时，可向晚加入且请求TRANSIENT_LOCAL的订阅者提供缓存中的旧样本。VOLATILE晚加入订阅者接收匹配后的新样本。先发独特的历史序号，再创建订阅者，最后发新序号，这样可分开验证历史与实时传输。

KEEP_LAST的depth限定保留样本数。发布序号1到5、depth=2，理想历史缓存是4和5。记录端点发现、发布与回调顺序，才能判断缺样本与历史深度是否有关。

`python3 lab.py`运行真实rclpy节点。ROS图表示发现与端点配置，序号表示实际接收。桥接的`/clock`读数来自Gazebo世界时间，等待上限使用墙钟。可分别核对实验期间的图信息与时钟消息。


本课可靠性负例同时要求DDS报告RELIABILITY不兼容、图中可查到端点、正例收到真实序号。持久性阶段的历史样本为10、11、12，匹配后再发13。深度阶段在两个订阅者已匹配且执行器暂不取样时发0至11，DDS确认后恢复回调，深度3预期只取9、10、11，深度16预期取全部。


报告的`reliability`列出一写两读、唯一实际匹配的订阅端与收包。`durability`与`history`分别保留历史交付和接收缓存的序号。`clock`保存一对ROS时钟及中间的原生Gazebo时刻，按纳秒数核对先后关系。
