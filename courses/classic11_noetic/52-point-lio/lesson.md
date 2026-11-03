# 附加专项52 · Point-LIO的点时间与输出节奏

一圈激光里有几千个回波。如果一列16束在同一物理时刻采样，这16个点会产生16个不同的估计时刻吗？本课通过同源回放观察Point-LIO的时间分组、IMU观测模型和里程计发布。建议先完成第50课的逐点时间与第51课的点到面更新。

本课固定上游原始master提交`1510c3bbf1743f254d83d0b22fbabb5b2d729f6b`。Point-LIO把更新安排到激光点的采样时间与IMU观测时间。不同时间的激光点分别在各自时刻建立残差，同一时刻的点可以共同约束同一状态。论文中的高频结果对应其设备、扫描方式和实验设置。[Point-LIO论文](https://onlinelibrary.wiley.com/doi/full/10.1002/aisy.202200459) · [固定原始实现](https://github.com/hku-mars/Point-LIO/tree/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b)

## 分时回波与等时组

Gazebo每0.5ms执行一次射线查询，一列有16个垂直束，200列组成约0.1s的一圈。每列保留下来的回波共用该物理时刻。点云的`ring`来自垂直束编号，`time`用秒表示相对首列的偏移，消息头存首列时刻。同一帧中点的采样位置和姿态随平台运动而变化。

算法接收ROS消息后将偏移转换成毫秒，按时间排序。固定实现的`time_compressing`遍历有序点，以相同时间的连续点数表示每组。例子中，偏移为[0,0,0.5,0.5,0.5,1.0]ms，分组长度为[2,3,1]。这是6个点和3个时间事件。关闭空间体素降采样并设置`point_filter_num=1`可保留较完整的原始列结构，盲区与输入有效性筛选仍会影响实际点数。名义列间隔0.5ms对应2000个时间槽每仿真秒，实际成功匹配的组和收到的里程计要分别数。[等时分组代码](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/include/common_lib.h#L80) · [排序与分组调用](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/laserMapping.cpp#L921)

滤波推进到某组点的采样时刻，再用当前位姿和固定LiDAR→IMU外参把该组点投到世界系。局部地图近邻拟合平面，点到面误差与雅可比进入迭代更新。同一时刻的多点提供多条几何约束，但时间间隔不会因为增加点数而变短。地图仍利用局部空间索引管理近邻，原程序也仍借扫描消息接收和批量组织工作。收到整圈后在处理循环内逐组计算，状态时间粒度可细于数据包的交付间隔。[观测模型](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/Estimator.cpp#L284) · [观测模式的分组循环](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/laserMapping.cpp#L999)

## IMU作为输入与作为观测

通常的惯性传播把去零偏的角速度和比力作为运动方程的输入。若测量因硬件限幅被削平，积分会按错误幅值推进。例如实际角速度为2rad/s而量程只有1rad/s，观测到的1rad/s无法单独给出超过量程的幅值。

本课的对照使用`use_imu_as_input=false`。Point-LIO的观测模式将角速度ω和比力a也作为状态，以随机过程描述其演化。IMU消息到来时，观测模型把读数与相应状态加零偏的预测比较。忽略噪声和尺度修正时，可写成`gyro ≈ omega + b_g`、`specific_force ≈ a + b_a`。状态方程再用ω、a推进姿态和速度。激光残差经状态关联帮助校正运动。两种模式在IMU信息进入滤波的位置上不同。[状态定义](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/Estimator.h#L49) · [IMU观测模型](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/Estimator.cpp#L372)

默认200Hz IMU对应0.005s采样间隔。`prop_at_freq_of_imu=true`在IMU时刻推进相应协方差，激光组继续按点时间更新。静止时加速度计比力模长约9.81m/s²，观测模式配置`acc_norm=9.81`。加速度计或陀螺的量程是每轴读数的物理边界，先确认单位，再设置`satu_acc`与`satu_gyro`。

启用`check_satu`时，固定实现逐轴检测读数绝对值是否达到配置量程的99%，标记相应IMU分量并屏蔽其残差贡献。其余IMU分量、过程模型与激光约束仍可使用。默认量程参数为30m/s²和35rad/s，基准回放使用原生IMU数据。要验证饱和处理，应在独立输入副本中记录每个原始和截断值、受影响的轴与时间，并核对估计器确实使用相符的量程。调小检测阈值只改变过滤判断。构造受控饱和对照时，还要让输入样本实际达到设定量程。极端运动时几何约束和初始化仍会影响估计效果。[饱和分量判断](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/Estimator.cpp#L372)

## 同一记录的两种发布节奏

课程准备阶段用`prepare.py`构建原生扫描插件，并以`build_point_lio.py --download`建立独立的Noetic工作区。固定归档与构建记录见`build/point-lio.json`。启动本课并保持Gazebo运行，在课程终端执行以下命令。

```bash
python3 adapter.py run
```

适配器采集一份实测激光与IMU记录，按0.5倍墙钟节奏分别回放给从初态启动的Point-LIO。两次读取同一份`sensors.bag`。其话题为`/gt_lio/replay_points`与`/gt_lio/replay_imu`，真值另供离线对比。默认对照仅切换`odometry/publish_odometry_without_downsample`。关闭时按处理后的扫描发布低频里程计，打开时在完成相应点时间组更新后可发布高频里程计。两个模式的内部激光更新仍沿点时间顺序执行。看发布分支时，同时核对消息头时间是扫描结束还是当前点组时间。[输出时间戳与发布函数](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/laserMapping.cpp#L681)

读结果时分别比较这四类数量与时间。

| 指标 | 表示什么 |
|---|---|
| 原始回波点数 | 一圈中收到多少有效点，一列可含多个 |
| 不同采样时刻与原生更新数 | 真实时间槽、处理的组、实际完成的更新分别计数 |
| 里程计header时间间隔 | 估计状态所对应的传感器时间 |
| 订阅端单调墙钟间隔 | 消息在回放、处理与传输后到达的节奏 |

一次包交付后可能紧接着发布多个里程计，墙钟上形成突发。即使header每0.5ms递增，也需看实际到达间隔、丢失或合并的消息与统计覆盖。传感器时钟和单调墙钟各有自己的起点，两者的绝对读数不能直接相减得到处理延迟。比较同一时域的间隔、事件计数与对应窗口，再解释算法的实际输出节奏。

结果写入`results/run-<uuid>/`，最新报告见`build/verification.json`。模式目录`scan/`和`point/`的`odometry.json`保留header采样时间、接收端单调时间和消息序号。`native-scan-diagnostics.json`保存每帧的时间组数、成功激光更新数、发布调用数和触发饱和屏蔽的轴计数。报告另列未被订阅端观察到的发布调用与消息序号间隙，便于进一步定位消息链路。程序只统计这些现象，具体原因需要结合队列、日志和运行负载分析。

`paired-filter-counts.json`按两组共有的原始扫描首时刻核对内部组数。`scan-end-comparison.json`检查扫描发布时刻附近是否有对应高频输出，匹配容差为1µs。缺少对应输出时保留缺口。两组都按各自里程计时间与IMU连杆真值配对，首个匹配位姿确定一次SE(3)对齐。`trajectory-comparison.json`记录估计、对齐真值和逐时刻误差。

每组的`initial-map.pcd`保存该版发布的初始化地图快照。`registered.pcd`保存最近一次配准扫描，可与地图及估计轨迹结合观察。读取这两个文件时，先确认各自的采样时间和坐标系。

## 两处数值练习

在个人`learner.py`实现`group_sizes(sorted_stamps_ns)`和`event_rate_hz(distinct_stamps_ns)`。第一处统计连续相同时间的组长。第二处对N个互异有序时间取(N−1)除以首末时间差，时间差换算成秒。N小于2时返回0。先做整数纳秒减法，再转成秒，可以保留大绝对时刻中的小间隔。

```bash
python3 student_check.py
```

先手算三列各16点、相邻列间隔0.5ms时的分组与频率。48个点属于三个组，跨越两个0.5ms间隔，事件率为2000Hz。局部检查验证分组与频率公式，真实输出的点时刻、更新与里程计来自完整回放。

## 观察与挑战

先预测切换发布节奏会改变哪些记录。用相同bag和配置核对原生时间组、成功更新与有效里程计输出，再对照两个结果目录。把每条高频输出按自己的采样时间与离线真值匹配。评估时看输出覆盖、实际运动和轨迹误差，并结合点云或地图观察。

若高频模式的消息数比预计少，依次核对点时间字段、同刻分组、近邻有效性与订阅传输。若发布时刻集中在少量墙钟区间，先确认回放按扫描包交付的节奏。若转弯时误差变大，先核对逐点偏移与外参，再查IMU量纲、覆盖和量程配置。可以增加一个独立对照，仅降低回放速率，比较header时间、接收墙钟和订阅覆盖各自如何变化。
