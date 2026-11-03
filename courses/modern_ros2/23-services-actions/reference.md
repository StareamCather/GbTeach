# 参考 · 服务与动作的完成证据

异步服务返回Future。执行器处理响应后读取结果，超时只描述调用者的等待状态。对可能改变世界的请求，先用独立观测核对状态再继续。

动作客户端依次检查Goal接受、Feedback、取消响应和最终Result。正常到达用SUCCEEDED，中途完成取消用CANCELED。取消实验还要读滑块实际速度和位置，核对停住后没有持续向原目标运动。[ROS 2动作状态机](https://design.ros2.org/articles/actions.html)

比例控制按误差给有界速度。目标在右侧时误差为正，速度指令向正轴。目标在左侧时符号相反。反馈位置来自Gazebo关节状态。参数设置后用读回值确认生效，验证拒绝时保留原值。[Jazzy参数](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Basic/About-Parameters.rst)

在课程终端执行`. ros_ws/install/local_setup.bash`，再运行`python3 lab.py`做服务、参数与实际动作对照。对照输出里的目标、状态码、反馈位置和最终关节状态，判断每段实际完成到哪一步。


`/gt/rail/state`是ReadRail，`/gt/rail/move`是MoveRail。成功与取消都检查停止后的实际速度。控制公式用`remaining_m = target_m - position_m`和`command_m_s = clip(gain * remaining_m, -0.42, 0.42)`。比如误差0.20m时，gain=0.8和1.8分别给0.16和0.36m/s。若误差较大导致饱和，两组命令可能相同，因此增益切换应比较未饱和反馈。
