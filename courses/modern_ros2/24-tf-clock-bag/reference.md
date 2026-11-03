# 参考 · 变换方向与录包时间

`lookup_transform(target, source, time)`把source中的点变换到target。链上静态边描述固定安装，动态边由其采样时刻确定。时间零取最新可用共同时间，传感器计算优先使用测量时刻。若动态缓存只覆盖旧时间，请求未来时刻会外推失败。[TF2 Buffer](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros_py/tf2_ros/buffer.py)

ROS节点设`use_sim_time=true`后读取实时`/clock`。bag记录时间规定保存和回放的时间轴，Clock载荷则保存原始仿真时刻。主机单调墙钟用于等待和测量到达节奏。回放时将历史Clock显式重映射，使实时与历史时钟保持各自来源。[Jazzy回放选项](https://github.com/ros2/rosbag2/blob/jazzy/ros2bag/ros2bag/verb/play.py)

`python3 lab.py`运行实际场景。按报告核对指定时刻TF、消息读回、隔离回放与实时仿真时间。若bag打不开，先检查写入是否完整关闭、目录权限与存储插件。


本课的链是`world → m24_rig → m24_sensor`。动态边给世界下的车架位姿，静态边给传感器安装。在车架偏航+90°时，安装偏移(0.45,0.20,0.15)m旋转到世界轴成为(−0.20,0.45,0.15)m，再加车架世界位置。查指定采样时刻得到当时的组合，查时间零得到缓存中最新共同时间的组合。


默认两次传感器世界位置为(−0.20,−0.35,0.80)m与(0.45,0.80,0.80)m。暂停阶段读取WorldStatistics与ROS时钟，20个1ms步应推进20000000ns。逐条比对原CDR、bag读回和重映射后实收消息。报告里的`historical_tf`用于核对回放缓存按两条旧时间戳给出的组合，`live_clock_publishers_unchanged`用于核对实时发布端标识。
