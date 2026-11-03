# 附加专项50 · 第一代 FAST-LIO

移动中的雷达扫过一面墙，为什么原始点云里可能看见两种墙面位置？这节课跟读第一代FAST-LIO，依次理解分时采样、IMU前向传播、扫描去畸变、几何特征和迭代误差状态更新。建议先修坐标变换、IMU、激光、传感器时钟和ROS记录回放。

LIO是LiDAR–Inertial Odometry，即激光惯性里程计。IMU用角速度和比力预测短时运动，激光扫描从环境几何提供校正。紧耦合指几何残差直接进入与IMU传播相连的状态更新。这个实验使用2021年6月的第一代历史树，其中独立的特征节点负责分类。后续专项的FAST-LIO2则从原始点出发直接做点到地图匹配。[第一代论文](https://arxiv.org/abs/2010.08196) · [固定历史源码](https://github.com/hku-mars/FAST_LIO/tree/6e1fa94351316eafac5fcf55c5fcd575f3e89829)

## 先把点时间说清楚

一条扫描消息是一段时间内采到的点。本文把首个采样时刻记为t₀，第j点采样时刻为tⱼ=t₀+Δtⱼ，扫描末时刻记为tₑ。应分别记录这三种时间和消息交付时间。回放速度改变的是消息到达的墙钟节奏，采样时间仍应来自原始记录。

本课使用实际运动中的Gazebo扫描器，在不同物理更新时刻进行射线查询。每点的坐标在采到它时的LiDAR坐标系中表达。IMU来自原生传感器，独立真值轨迹用于评估。普通整帧RaySensor更新提供单时刻几何，分时扫描需要在相应时刻实际做射线查询。给同一份快照按列补时间会改变观测语义。[Classic RaySensor更新实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/RaySensor.cc#L345)

固定历史版的HORIZON路径接收`livox_ros_driver/CustomMsg`。消息的`line`标识激光束，`tag`参与回波筛选，`offset_time`为相对于起点的纳秒偏移。前端将偏移除以10⁶，借用PCL点的`curvature`字段保存毫秒。后端再除以10³恢复秒。这里字段名来自点类型复用，其内容是时间。规则扫描束可以封装成该接口的消息，扫描形态仍由实际仿真传感器决定。[Livox消息定义](https://github.com/Livox-SDK/livox_ros_driver/blob/3d240d5666129e1a3052e78ee8487a04b08fdda3/livox_ros_driver/msg/CustomPoint.msg) · [历史前端转换](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/feature_extract.cpp#L183)

先手算一组时间。若首点在12.0s，某点偏移25000000ns，那么该点在12.025s被采到，进前端后`curvature`应为25。再检查IMU是否与扫描使用同一时钟、时间是否单调、扫描末端是否有足够IMU覆盖。原版同步路径由面特征消息末点的时间推算结束，故还需核对分线提取后的特征顺序和扫描跨度。课程构建时在独立源码副本中对面特征按时间稳定排序，再由末点确定后端参考时刻。原始扫描末列时间可能晚于最后一个面特征，两者在报告中分开检查。[历史同步代码](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/laserMapping.cpp#L583)

## IMU传播与预积分

状态包括姿态、位置、速度、陀螺零偏、加速度计零偏与重力。陀螺观测是机体系角速度。加速度计测的是比力，去偏后转到世界坐标再加重力，得到用于推进的世界加速度。静止时线速度保持零，但比力通常有约一个重力加速度的模长。这解释了为什么静止初始化应检查重力方向、量纲和角速度均值。

在一小段Δt内，可用以下近似理解传播。R把IMU向量转到世界坐标，b_g和b_a分别为两类零偏。

```text
R_next ≈ R · Exp((gyro − b_g) Δt)
a_W   ≈ R · (specific_force − b_a) + g_W
v_next ≈ v + a_W Δt
p_next ≈ p + v Δt + 0.5 a_W Δt²
P_next ≈ F P Fᵀ + Q_d
```

Exp把小旋转向量变成旋转矩阵。P是误差协方差，F描述误差随时间传播，Q_d描述离散过程噪声。固定代码使用相邻IMU读数求平均，考虑初始化加速度尺度，传播状态与协方差，并保存去畸变需要的轨迹信息。

这里的**逐采样传播**从当前全状态出发，连续推进滤波先验。**IMU预积分**通常把两个关键时刻之间的多条IMU观测整理成相对旋转、速度增量、位置增量及其协方差和偏置敏感度，可作为因子图约束。两者都需要积分，保存的量及使用方式不同。第一代固定实现的这一步对应滤波传播和去畸变，后续LIO-SAM专项再看预积分因子。[历史IMU实现](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/IMU_Processing.hpp#L175) · [Forster等人的预积分论文](https://arxiv.org/abs/1512.02363)

## 用同一时刻表达整帧点

记T_AB为从B系到A系的刚体变换。固定外参T_IL把LiDAR坐标变到IMU坐标，写成p_I=R_IL p_L+t_IL。t_IL是LiDAR原点在IMU坐标中的位置。将t_IL直接改负号只适合旋转为单位阵的逆变换。一般逆变换的平移为−R_ILᵀt_IL。

设两时刻的IMU预测位姿为T_WI(tⱼ)与T_WI(tₑ)。单个点的去畸变可沿着以下坐标链推导。

```text
p_Le = inv(T_WI(t_e) · T_IL) · T_WI(t_j) · T_IL · p_Lj
```

读式子时从右到左走。先将采样时的点变到IMU系和世界系，再变到扫描末的LiDAR系。若机器人在一帧内沿+X移动0.1m，前方墙在世界x=10m，首时刻LiDAR看到10m，末时刻看到9.9m。把首点表达在末时刻LiDAR系后，两者应指向同一面墙。若只旋转不补平移，或把逆变换用反，都会保留或放大墙面厚度。

固定历史实现对点时间排序，利用IMU传播保存的姿态、速度和加速度记录倒序访问点，把它们补偿到扫描末附近的参考坐标。该树中的LiDAR与IMU旋转按同轴处理，平移来自固定代码常量(0.04165, 0.02326, −0.0284)m。实验的SDF传感器安装使用同轴旋转和这组实际平移。实际安装、算法常量和可视化坐标都需要对应。对偏离质心的传感器，转动本身也会引起传感器原点移动，所以外参方向会影响去畸变。[去畸变代码](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/IMU_Processing.hpp#L280) · [固定外参](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/include/common_lib.h#L44)

## 面点、边点和实际进入滤波的残差

历史前端先按`line`分组，使用相邻点间距、候选段的局部几何和方向变化判别面段及边缘。平面候选及确认类别汇入surface，间距突跳及面段交界等边缘类别汇入corner。可以在可视化中比较墙面、柱边与遮挡处的分类。预处理阶段还会过滤回波，不能仅凭无点断定场景缺少特征。[历史特征判别与输出](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/feature_extract.cpp#L496)

论文给出面点到平面及边点到直线两类残差。所选历史树的地图更新主路径使用surface。它将去畸变、降采样后的面点投到世界坐标，寻找地图近邻并拟合局部平面，筛除不可靠对应，再形成点到面残差。corner话题用于观察前端分类，查看滤波器实际使用的量应跟到后端平面匹配。对于单位法向n与面上点q，平面残差可写成r=nᵀ(p_W−q)。沿法向的偏差直接进入r，平面内的平移一阶变化可能很小。只有一面墙时，一些方向就缺少几何约束。[论文第III-C节](https://arxiv.org/html/2010.08196) · [后端平面拟合与筛选](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/laserMapping.cpp#L842)

## 迭代误差状态更新

滤波器先把IMU传播得到的状态与协方差作为当前扫描的先验。用该状态变换去畸变后的点并建立地图对应，得到残差与雅可比。位置误差可用三维向量表示，姿态误差在旋转群的局部切空间用小旋转向量表示。修正时将小旋转映射回旋转矩阵并与当前姿态复合。虽然激光的残差主要直接约束姿态和位置，状态间的协方差关联也会影响速度与零偏等量的修正。

同一帧更新时，算法在新姿态附近重算残差、雅可比，并按实现条件刷新地图近邻。各轮保持同一帧传播先验。迭代改善非线性模型的线性化位置，收敛或到迭代上限后才完成该帧更新，再将更新后位姿下的点用于建图。读代码时先找`state_propagat`，再看`Hsub`、`solution`及结束条件，就能把预测、迭代和地图写入分开。[历史误差状态结构](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/include/common_lib.h#L58) · [历史迭代更新](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/laserMapping.cpp#L981)

## 采集、回放与复查

本课环境为Gazebo Classic 11.15.1和ROS Noetic，使用Gazebo原生IMU插件、C++编译器、catkin、PCL、Eigen和OpenCV。首次进入时，`prepare.py`准备传感器插件，`build_fast_lio1.py --download`下载并校验固定上游归档，再构建课程自己的catkin工作区。构建缓存默认位于`~/.cache/gazeboteach/a50-fast-lio1`，设置了`XDG_CACHE_HOME`时跟随该目录。已有固定归档时，可用`python3 build_fast_lio1.py --archives /实际归档目录`离线准备。归档名称和SHA256见脚本。

Livox依赖仅生成`CustomMsg`与`CustomPoint`的ROS消息绑定，运行时启动独立的历史特征与地图节点。构建脚本保存`build/fast-lio1.json`，可核对源码提交、归档与两个可执行文件的SHA256。构建补丁处理消息编译顺序、短线束和空帧、面特征时间排序，以及用处理后的扫描末时刻给输出加时间戳。滤波与建图使用固定历史树的核心流程。

从桌面启动实验后，让Gazebo继续运行，在课程终端执行以下命令。

```bash
python3 adapter.py run
```

插件按0.5ms物理周期进行一列真实ODE射线查询。每列含16个从−22°到+22°的垂直束，200列组成约0.1s的一圈。每列内的束共享本次查询的时刻。默认采集140圈，同时收集200Hz原生IMU及100Hz IMU连杆位姿真值。点云首时刻来自首列，点偏移来自实际查询时间，bag的扫描记录时刻则取最后一次原始射线采集时刻。

适配器把实采雷达和IMU写入`sensors.bag`，以0.8倍墙钟节奏给新启动的历史节点回放。bag话题为`/livox/lidar`和`/gt_lio/replay_imu`，后者通过launch重映射接入估计器。真值保留在原生采集记录中，用于离线对比。估计器订阅的两路消息保留原始仿真采样时间，bag无需发布`/clock`。回放中审计面特征`/laser_cloud_flat`、边特征`/laser_cloud_sharp`、地图`/Laser_map`和里程计`/aft_mapped_to_init`，同时核对发布节点和处理时间。

每次运行生成`results/run-<uuid>/`。先读最新的`build/verification.json`定位本次目录，再按需打开`report.json`、`native.jsonl`、`capture-done.json`、`estimator.log`、`replay.log`、各输出摘要和`trajectory-comparison.json`。可用以下命令核对bag中实际保存的话题。

```bash
bag=$(python3 -c 'import json; print(json.load(open("build/verification.json"))["bag"]["path"])')
rosbag info "$bag"
```

评估按里程计的处理时间匹配IMU连杆真值，位置线性插值、姿态用四元数球面插值，首个匹配位姿用于确定单个SE(3)对齐。对齐只变换参考坐标原点和方向，比例固定为1。默认至少匹配95%的里程计输出，位置RMSE门槛为0.65m，姿态RMSE门槛为0.35rad，并检查有效地图与实际运动。首次在ikaros验证得到140圈、447199个原始点、2860条IMU及131条里程计。匹配130个姿态，位置RMSE约0.02882m、姿态RMSE约0.00217rad。这组数对应本场景的一次运行，复现时以本次保存的报告为准。

## 两处数值练习

打开个人副本的`learner.py`。用两个整数纳秒量求点的绝对采样时间，再把点到面上参考点的差投影到单位法向。保留点到面残差的符号。运行`python3 student_check.py`核对大整数时间精度和几组法向算例。该命令只检查这两个函数。完整物理采集和估计器结果由桌面验收或`adapter.py run`生成。

## 单变量对照

先跑基准，沿同一帧追踪原始点、分类后的面点/边点、点时刻和IMU覆盖。保存输入、固定源码版本、配置及对齐规则。对照中可从同一墙块拟合平面并计算正交距离分布，用厚度指标辅助观察。进一步计算相邻或等间隔估计位姿的相对运动误差，再与本课的首位姿对齐指标并读。

保留同一段输入与初始化，在专门的对照副本中改变点时间处理，先预测墙在快速转动阶段怎样变厚。然后恢复正确时间，单独扰动一个外参分量，检查静止、平移和转动阶段的差异。保留错误注入方式、结果与恢复记录。若里程计迟迟没有输出，优先检查消息字段、线号、回波tag、点偏移换算和IMU末端覆盖。若直走正常而转弯误差明显，重点核对去畸变时间方向、LiDAR到IMU外参和绕轴符号。

独立挑战是在同一数据中分别选择单墙和有多方向结构的片段，比较点到面残差对各平移方向的灵敏度。保持相同评估方法，说明几何结构与运动如何共同影响可观测性。
