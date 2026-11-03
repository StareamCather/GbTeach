# 参考 · 包解析与生命周期证据

包的解析依据是已加载安装环境中的ament资源索引。`package.xml`提供元数据，`setup.py`安装索引、launch和入口，`setup.cfg`决定脚本安装位置。用`. ros_ws/install/local_setup.bash`加载课程overlay，再用`ros2 pkg prefix gazeboteach_lifecycle`查看解析结果。构建阶段出错查看`ros_ws/log`。[colcon工作区教程](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.rst)

正常转换为unconfigured→configure→inactive→activate→active→deactivate→inactive→cleanup→unconfigured。配置建立业务资源，激活打开处理与输出，停用关闭输出，清理销毁本次资源。每步验证GetState，并用实际消息或图端点核对行为。[ROS 2生命周期](https://design.ros2.org/articles/node_lifecycle.html)

加载本课安装环境后，`python3 lab.py`执行真实状态与数据对照。若停用后的短时间仍收到旧数据，先区分过渡窗口和其后的新观察窗口。若cleanup后资源仍存在，按节点名定位实际端点。


本课数据源是Gazebo原生转子IMU，原始话题`/gt/lifecycle/raw_imu`在状态变化时继续产生样本。激活后输出`/gt/lifecycle/sample`保留其IMU数据与原时间戳。停用后以原始时间继续前进和新输出为零证明门控，cleanup后再检查两个业务端点消失。


验收核对实际节点加载的模块路径与内容哈希。两轮状态对照中，用递增的原始IMU时间、陀螺和偏航说明输入持续运动。active按原始时间戳核对至少6条完全一致的CDR消息，停用和清理阶段用之后的新窗口检查零转发。
