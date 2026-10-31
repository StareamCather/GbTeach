# 激光扫描：只读几何推导与实现边界

本页不覆盖个人world.sdf。以下数值是当前固定几何的预测，实际验收来自真实CPU/GPU消息；恢复起点请使用平台的备份并恢复操作。

## 从方向找索引，再回算方向

有效索引i满足0≤i<N，N为ranges实际长度。方向为`theta_i=angle_min+i×angle_increment`。找最接近theta的束之前，先确认目标方向在实际FOV内，且increment为有限正数，再计算四舍五入索引、检查边界并算回theta_i。这个索引对应最近已有方向，不是凭空产生一束恰好命中目标的射线。

| 输出数 | FOV | 角增量 | 0°索引 | ±30°索引 | ±45°索引 |
| --- | --- | --- | --- | --- | --- |
| 181 | [−90°,90°] | 1° | 90 | 60、120 | 45、135 |
| 361 | [−90°,90°] | 0.5° | 180 | 120、240 | 90、270 |

这里horizontal/resolution=1，因此本课samples与输出数一致。不要把scan/horizontal/resolution这个输出与射线数量相关的比例，和range/resolution长度字段混成同一个量；其它比例下CPU还可能做插值，数组长度增加不一定增加同等数量的独立物理射线。当前两条UpdateImpl路径不按range/resolution=0.01把读数统一量化到厘米网格，该字段也不证明真实测距精度。实际算法始终以消息元数据和真实长度为准。

点P=(x,y)的方向是atan2(y,x)，距离是sqrt(x²+y²)。r与theta转点为(r cos theta,r sin theta)；只有有限有效r才能参与这个转换。这些点在传感器坐标系中，移到world仍需实际位姿。两路使用laser_frame标签并不自动产生TF。

## 有限墙面的交点

墙中心X记为c，厚度0.2 m，朝向扫描器的表面为d=c−0.1 m。方向向量(cos theta,sin theta)的射线与X=d相交：

```text
t cos(theta) = d
t = d/cos(theta)
y = d tan(theta)
```

本课水平墙面有效Y范围[−4,4]。只有cos theta>0、t>0、|y|≤4，且交点高度在墙内、t处于量程内时，才是这个表面的候选命中；更近方块还可能遮挡它。不要把无限平面公式当作所有角度必然命中的保证。

| 墙中心c/m | 前表面d/m | 正前方/m | ±30°墙面距离/m | ±45°墙面距离/m |
| --- | --- | --- | --- | --- |
| 3.0 | 2.9 | 2.9 | 3.34863 | 4.10122，Y=±2.9仍在墙内 |
| 3.5 | 3.4 | 3.4 | 3.92598 | 4.80833，Y=±3.4仍在墙内 |
| 4.0 | 3.9 | 3.9 | 4.50333 | 5.51543，Y=±3.9仍在墙内 |
| 5.0 | 4.9 | 4.9 | 5.65803 | 平面交点Y=±4.9超出墙宽，实际墙不命中 |

最后一行可能出现在中心4 m基础上再向远处移动1 m的验收阶段，不是个人起点编辑允许范围的扩大。默认d=2.9时墙可见方向边界约±atan2(4,2.9)=±54.06°，所以±60°和±90°不命中墙。距离改变时应重新计算边界。

## 只有collision或只有visual的方块

两个方块边长0.4 m，中心X=1.5 m、Y=±1.5 m，Z=0.7 m。在对应±45°束上，进入靠近原点的角点X=1.3 m、|Y|=1.3 m，因此近距为`1.3/cos45°=1.3sqrt(2)≈1.83848 m`。

−45°方块只有collision，所以CPU读近距，GPU若没有其它遮挡便读远墙或无回波；+45°方块只有visual，所以两路角色对换。这不表示两路传感器位姿不同，而是几何输入不同。实际边缘采样、渲染精度和浮点运算允许小误差，不能要求CPU/GPU逐位相同。

CPU射线根据物理几何求交，GPU根据渲染路径生成距离，参见[Classic RaySensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/RaySensor.cc)与[GpuRaySensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/GpuRaySensor.cc)。材质颜色、可见形状与物理collision并非自动一致；这套理想模型不说明真实玻璃、黑色物体等材料一定具有哪种测距行为。

## GPU到ROS的兼容适配器

当前安装的CPU与GPU系统包装库同时导出gazebo::GazeboRosLaser类；同进程加载时可能混用实现。本设备实际启动出现GPU调用CPU RayPlugin的错误，上游[问题768](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/768)记录了相同类名冲突。课程保留系统libgazebo_ros_laser.so用于CPU，GPU改用独立命名的GazeboTeachL15GpuLaser类，不覆盖或重新安装系统库。

gpu_bridge.cc是SensorPlugin：检查父传感器确为GpuRaySensor，订阅它的实际Gazebo Transport LaserScanStamped，再把原生时间戳、角度、量程、ranges和intensities复制到ROS LaserScan。它不根据墙位置计算数组，不替换GpuRaySensor，不改变射线渲染或几何输入；验收中的解析几何仅用于预测和比较，不能成为消息来源。插件实现不是本课前置要求，第28课再展开Sensor Plugin。

prepare.py先验证个人world.sdf和开发依赖，用已安装Gazebo 11、roscpp与sensor_msgs的SDK参数编译libgazeboteach_l15_gpu_laser.so。缓存键包含适配器源码、Gazebo版本和编译链接参数，库位于用户缓存目录，本练习build/lib通过链接使用它；匹配缓存存在时直接复用。build/laser_lab.json记录world、CPU系统库、GPU适配器源码和生成库的SHA256及构建键，用于说明实际使用的输入与组件。这个指纹证明字节身份，不替代真实渲染和测距验收。

## 特殊值与时间字段

按[REP117](https://github.com/ros-infrastructure/rep/blob/master/rep-0117.rst)，+Inf表示超远/量程内无回波，−Inf表示过近，NaN表示无有效数值。程序应分类保留计数，不能将其替换成一个普通距离后无差别求平均。NaN与普通数的比较具有特殊行为，应显式使用isfinite/isnan。

当前Classic 11原生路径先把raw range≥max变为+Inf、≤min变为−Inf，再对中间值加噪声并夹到边界；GPU还有将NaN转为range_max的处理。因此有限max可来自边界处理，不总是证明刚好在max命中物体。近裁剪也可使盲区内几何被忽略，不能从REP语义倒推“所有过近物体必产生−Inf”。

[LaserScan规范](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/LaserScan.msg)定义header、束间时间与扫描间隔。本课实际加载的[系统CPU包装](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_laser.cpp)和gpu_bridge.cc均复制原生扫描时间，将time_increment与scan_time设为0。GPU适配器沿用[上游GPU包装](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_gpu_laser.cpp)的这项约定，该系统GPU库本课未加载。两条实际路径都不模拟机械转子逐束获取的时间序列，也不自动广播TF。用相邻不同header的跨度估计仿真频率，而不是1/scan_time。

N条不同时间戳消息的首末跨度为T时，平均频率估计为(N−1)/T；不要用N/T多算一个间隔。相同消息数的墙钟频率由实际接收时间估计，两者受实时因子影响，不应要求恒等。暂停意味着没有新测量；两路分别满足几何不等于已证明严格同步。

## 修改与验收的算法流程

config --distance修改保存场景中的墙中心，config --samples同步修改两传感器束数；保存后须重载，不通过悄悄移动运行世界掩盖旧输入。wall --distance则明确修改本次运行中target_wall的实际位置，保持原暂停状态，不冒充动力学运动。

完整验收先检查保存输入和当前实际状态，记录进入时的墙原始位姿、速度与暂停状态，订阅两路真实消息。先主动将墙放到保存配置的基线，采样后在阶段边界移动到基线+1 m，最后回到配置基线再采样；这些是明确的状态设置，不冒充物理连续运动。每阶段丢弃旧时间数据，经过至少0.2仿真秒传播保护，等到首个符合几何的新帧才建立测量窗口；此后收到的窗口帧均须符合要求。

每阶段每路至少12条唯一时间戳、首末跨度至少0.5 s，平均频率15到25 Hz。程序从实际消息重建九个固定测试方向的索引，逐帧验证有限预测距离误差≤0.08 m、预期无回波为+Inf，以及两种辅助方块和数组元数据；不是只对最后一帧求一次吻合。测量仍是离散传感器观察，不能证明两帧间全部物理时刻都已覆盖。

无论通过或失败，finally都尝试恢复**进入验收时**的墙位姿、速度和原暂停状态；若你此前手动移动过墙，最终应回到那个真实进入状态，而不是擅自留在保存配置的基线。报告保留恢复误差与前后状态。原始扫描和分项结果写入trace_path；JSON用+inf、-inf、nan字符串保存非有限类别，避免伪造数值或输出不标准的JSON。

物理、渲染和ROS传输有不同更新时间，验收必须取得新鲜稳定观察，不能只等SetModelState返回就宣布GPU画面已经更新。没有gzclient不等于GPU不需要渲染上下文；真实GPU数据和GUI画面各自说明相应路径的状态。

## 独立挑战的分层提示

目标：在正前方±20°给出最近距离与角度，并同时报告有效、无回波和其它无效数量。

1. 遍历i=0到N−1，用消息字段计算theta_i，只保留角度范围内的索引；不要假设永远181束。
2. 对相应r先浮点分类，再检查range_min≤r≤range_max；仅候选有效值参加最小值比较。
3. 保存最小r及其实际theta_i；没有候选时返回明确的空结果，不能用0 m假装有障碍，也不能直接保证无障碍。
4. 用181与361两种输出复查同一墙面；解释角度采样变化与距离精度的区别，说明一帧结果不能覆盖整段运动过程。

零均值随机噪声的多次平均与系统偏差、边界夹断不是同一问题；细物体落在两束之间是空间采样问题，快速目标在两帧间经过是时间采样问题。当前零噪声静态几何验收不代替这些拓展实验。
