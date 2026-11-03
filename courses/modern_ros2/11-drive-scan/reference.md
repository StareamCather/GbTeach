# 参考 · 差速轮速、前向测距与桥方向

给定轮距0.52m和轮半径0.15m，直行0.25m/s时左右轮都约+1.67rad/s。偏航速度+0.55rad/s且线速度为零时，左轮约−0.95rad/s，右轮约+0.95rad/s。各自的期望值来自(v∓ωb/2)/r。理想直行2.0s前进0.50m，原地转向1.35s转过约0.74rad。实际状态与里程计用各自的起止观测计算增量。[Harmonic DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)

墙中心x=4m，半厚0.10m，初始雷达前移约0.20m。车头正对墙时预期距离约3.70m，前进0.50m后约3.20m。中央五束的中位数用于比较墙距下降量。若激光在转向后斜着看墙，先恢复直行初始朝向或分别解释几何关系。

`/gt/cmd_vel`从ROS转Gazebo，`/gt/odom`和`/gt/scan`从Gazebo转ROS。传感器QoS的可靠性按提供与请求匹配。BEST_EFFORT发布端无法满足RELIABLE请求，端点能发现时也要确认实际收到了数据。[官方桥接教程](https://gazebosim.org/docs/harmonic/ros2_integration/) · [Jazzy QoS](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

运行`python3 lab.py`复查实际车体位移、轮速、里程计、墙距和停车结果。将配置轮半径改成0.20m会让同一指令对应的轮速降低到原来的0.75倍。物理轮半径仍为0.15m，因此比较原生运动和按配置计算的里程计有助于识别标定差异。


实际轮速取`/gt/wheels`的JointStatePublisher反馈。报告的`actual_dx`是世界X增量，`odom_dx`是里程计两点的平面距离。`laser_front_delta_m`应与实际直行量接近，扫描前后时间戳应递增。两阶段持续时间取实际Odometry仿真时间差，停车以原生漂移和轮速复查。
