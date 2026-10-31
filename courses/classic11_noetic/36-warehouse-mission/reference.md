# 参考：仓储任务的物理、时间与证据边界

本页对应Gazebo Classic 11.15.1、ROS 1 Noetic、gazebo_plugins 2.9.3的本课实现。先读[教学页](lesson.md)的操作顺序，再沿本页核对每一条数据从哪里来。公式用来预测与解释，运行结论只使用本轮实际采集的数据。

## 执行器的两种控制量

机器人使用`libgazebo_ros_diff_drive.so`。插件接收`geometry_msgs/Twist`的linear.x、angular.z，计算左右轮缘速度，再按轮半径变成关节角速度目标；轮电机带有有限力矩上限，车身依靠接触受力。它不是给world中的车身设置一条x(t)轨迹。[2.9.3原生实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_diff_drive.cpp)

门则是本课`door_plugin.cc`中的ModelPlugin，连接`slide_joint`。每个WorldUpdateBegin读取`Position(0)`和`GetVelocity(0)`，计算`120(q*−q)−20q̇`，夹在±force_cap范围内后调用`SetForce(0,F)`。这是滑动关节，q以m计，F以N计；若换成旋转关节，位置与输出单位会变成rad和N·m，不能只复制数值。

| 项目 | 机器人 | 滑门 |
| --- | --- | --- |
| 高层目标 | v、Ω | 闭合0 m或打开1.25 m |
| 原生驱动 | 轮关节速度电机，有限力矩 | 关节PD施力，有限力上限 |
| 动力学证据 | 轮角变化、车身世界位置与速度 | 实际q、q̇与命令确认中的施力样本 |
| 单凭什么仍不够 | 发布Twist、轮速目标为零 | 命令确认、open=true |

门插件的enabled=false使目标保持闭合；它不删除门碰撞，也不把门设成透明或移到另一位置。enabled与open控制的是目标选择，物理世界仍须推进，关节才会响应。插件Reset恢复默认启用并清除打开请求。目标确认中返回的q是最近一次物理更新测量；暂停时能确认命令，不意味着刚刚发生了新运动。

world.sdf保存默认force_cap=20 N。启动时mission.py从已准备并核对指纹的parameters.json读取door_force_n，通过door_client发送cap命令；原生插件只接受12、20、30 N并返回force_cap确认，不相符则启动失败。插件用原子变量在后续物理更新读取这一上限，世界Reset保留上限而清除open、恢复enabled。prepare不覆盖源world，也不通过改门位姿使参数“生效”。

SDF的prismatic定义、轴向、上下限与碰撞决定门的可动方向和范围。SDK同时提供施力和设置关节位置等不同API；本课门的任务运动只调用施力路径。初始装载、明确世界复位与连续任务控制必须在记录中区分。[SDF 1.6 joint](https://sdformat.org/spec/1.6/joint/)、[Classic 11.15.1 Joint.hh](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/Joint.hh)

## 不能把控制目标当测量

理想无滑关系为`v=(r/2)(ωR+ωL)`、`Ω=(r/b)(ωR−ωL)`；本课r=0.10 m、b=0.36 m。默认直行0.25 m/s得到两轮各2.5 rad/s；反馈应来自实际关节位置和实际模型运动，而非把2.5写回日志充当实测。

轮子的碰撞形状、轮轴方向、法向载荷、摩擦、车身惯性和ODE约束误差都会影响实际速度。轮角变化不能保证无滑位移，world位姿变化也可能来自外部碰撞。零目标时存在小量被动漂移的可能，停车判据采用有限速度/位置容差；不能要求每个浮点数永远不变，也不能因此把明显持续运动归为“正常误差”。

长WAIT_DOOR窗口会把毫米每秒量级的被动漂移累计为厘米级位移。独立验收在实际等待跨度T（模拟秒）内检查最大平面位移包络不超过`min(0.060, max(0.018, 0.006+0.008T))` m，同时要求朝门方向的最大推进不超过0.010 m；实际平面速度<0.028 m/s、|偏航角速度|<0.08 rad/s和零Twist要求保持。这里调整的是独立测量判据，控制器仍发零目标；它没有消除ODE接触漂移，也不允许借漂移逐渐接近关闭的门。

机器人外廓应包括轮子、支撑与所有collision，不能只量车身visual宽度。小车车身碰撞在局部x方向为[−0.31,0.11] m、宽0.24 m；两轮中心y=±0.18 m、厚0.01 m，因此横向碰撞总宽约0.37 m。固定门洞y范围为[−0.60,0.60] m；门板闭合时覆盖这一范围，开启后其下边缘为`q−0.60`。q>1.10 m给出边缘y>0.50 m，对居中小车留出几何余量。车有偏航或偏离中心时，投影外廓与余量会变化。

门的通行净宽随实际q变化，开度阈值要保证整车与门框间隙。前向扇区只观察某一高度与角度范围，不能覆盖任意三维障碍；本课默认通道和门几何是这些判据的一部分。[SDF collision](https://sdformat.org/spec/1.6/collision/)

## 扫描消息与几何输入

CPU `RaySensor`提供真实扫描，`libgazebo_ros_laser.so`将原生消息的stamp、角度元数据、量程和数组复制到ROS LaserScan。它不从任务脚本读门的位置来生成ranges。当前包装为按订阅需求工作的传感器链，订阅建立与真正出现新数据之间允许有启动延迟。[2.9.3 CPU ROS包装](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_laser.cpp)

对每个实际数组索引`0≤i<len(ranges)`，使用`θᵢ=angle_min+i·angle_increment`。默认361束覆盖±90°、间隔0.5°，量程0.06～8 m，目标更新频率10 Hz。前向判断取`|θᵢ|<0.08 rad`中的有限量程内距离最小值；默认约为±4.5°的19束。扇区没有有限有效值时front为缺失，不被当作清空。不要从某个期望束数硬编码前方索引；先读实际元数据。

CPU射线根据物理几何求交。visual只影响显示的物体不一定会被这条扫描看见；collision即使不显眼，也可能产生回波或阻挡小车。关闭门的近表面会遮挡后方物体，这正是本课门前扫描与开门后扫描不同的原因。[Classic RaySensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/RaySensor.cc)

只有有限、处于消息量程内的r才转成测量点。+Inf、−Inf和NaN分别归类保留，不能用range_max替换后冒充障碍命中。扇区全无效时，缺失有效近距不能直接当“无限通畅”；无回波的语义还须结合实际驱动和场景。[REP-117](https://github.com/ros-infrastructure/rep/blob/master/rep-0117.rst)

LaserScan的角度单位为rad、距离为m，零角沿frame的+X，正角绕+Z。消息定义允许逐束时间差，但本课2.9.3 CPU包装明确令time_increment=scan_time=0，整帧按stamp处理。用相邻不同stamp的跨度估计模拟频率，N帧覆盖T秒时使用(N−1)/T；不使用1/scan_time。[Noetic LaserScan.msg](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/LaserScan.msg)

## 以时间差受限的实际位姿投影

本课world与warehouse/odom对齐；warehouse/base_footprint是随车运动的frame，warehouse/laser是固定安装的激光frame。记扫描时刻机器人平面位姿为(x,y,ψ)，激光相对base平移为(tx,ty)=(0.15,0)。在本课轴向一致的安装下：

```text
θᵢ = angle_min + i·angle_increment
u = tx + rᵢcosθᵢ
v = ty + rᵢsinθᵢ
X = x + cosψ·u − sinψ·v
Y = y + sinψ·u + cosψ·v
```

z安装值0.2202 m是相对base的名义扫描高度；上述CSV/SVG使用平面x、y、yaw近似，适用于本课近水平地面，不声称完成任意侧倾/俯仰下的完整三维TF变换。若改变传感器朝向，必须把安装旋转也纳入变换，不能只修改frame字符串。

当前实现对一帧stamp为t的scan，从最多400条odom缓存中倒序找最近的`t₀≤t`。若没有前序样本，或`t−t₀>120 ms`，增加projection_rejected并拒绝该帧投影；否则使用该条真实odom的x、y、yaw。这里是保持近似，没有未来样本、插值或外推。CSV分别保存scan_stamp_ns与odom_stamp_ns，读者能独立核对选取过程。以0.25 m/s匀速直行为例，120 ms时差对应0.03 m位移量级；这不是对任意转向和加速的误差上界。

程序每隔6束保存一个有效点，以控制文件和SVG规模；原始ROS记录仍保留收到的完整扫描数组。点CSV列为scan_stamp_ns、beam_index、range_m、angle_rad、odom_stamp_ns、robot_x_m、robot_y_m、robot_yaw_rad、x_m、y_m。SVG使用(X,−Y)适配屏幕向下的纵轴，点的科学坐标仍以CSV为准。

若以后扩展为插值，应先缓存扫描，等到odom有`t₀≤t≤t₁`，用`α=(t−t₀)/(t₁−t₀)`插值平移，沿最短角差插值朝向，避免从+179°到−179°错误地绕过0°。同时设置等待墙钟上限、最大包围间隔及时间回退处理；没有后序样本不能声称已插值。这是学习挑战，不是当前投影算法。

ROS接收顺序不能代替采样时间顺序。单调墙钟接收时刻适合记录等待与进程超时；模拟stamp适合物理阶段时长和几何对齐；二者数值不能直接相减。暂停使物理时间不推进，真实消息可能停止产生，但工具仍需要有限墙钟等待来报告“目前没有新证据”。

`nav_msgs/Odometry`的pose相对于header.frame_id，twist相对于child_frame_id。因此world中的速度分量与转向后的车体linear.x不应逐项比较。[Noetic Odometry.msg](https://github.com/ros/common_msgs/blob/noetic-devel/nav_msgs/msg/Odometry.msg)

当前odometrySource=world，动态TF与odom来自同一差速插件对真实模型的读取；它们互相一致是接口一致性证据，不是两个独立定位器互相验证。独立Gazebo状态读回补充模型身份与真实运动检查，但同样属于同一仿真世界。门的q、rate由[GetJointProperties](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_msgs/srv/GetJointProperties.srv)返回，机器人world位姿与速度由[GetModelState](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_msgs/srv/GetModelState.srv)返回。这里没有评估真实传感器误差，也没有独立轮编码定位精度结论。[REP-105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)

## 投影图为什么不是SLAM地图

扫描点来自实测ranges，位姿来自本课已知的world来源odom。将二者相乘得到世界点，是坐标变换和数据可视化；SLAM还需要从观测中估计未知位姿与环境。本课没有扫描匹配、回环、地图优化或占据概率更新。

SDF用于定义要被测的物理场景；CSV/SVG生成时不得从SDF抽取墙线代替观测。未命中、未覆盖与遮挡处保持无点；多次看到移动门时可能留下重叠或展开的轨迹。这些点描述历史观测，不能直接作为门当前占用状态。对图的每一项解释都应能追溯到扫描身份、束索引、时间戳与所用位姿。

## 从平面误差产生有限Twist

APPROACH使用x目标−0.75 m，但会先被近距或门前位置保护截停。PASS和GOAL使用x目标2.20 m。drive(target)读取实际odom的x、y、yaw，先计算：

```text
e = target − x
ψ目标 = atan2(−y, max(0.35, e))
δ = wrap(ψ目标 − yaw)
Ω = clip(2.5δ, −0.60, +0.60)
v = clip(0.9e, −0.05, linear_speed_mps)
若 |δ|>0.50 rad，则本次v=0
```

横向偏差y改变期望朝向，角速度随朝向误差变化；x接近目标时前向速度按误差缩小，上限是个人车速参数。较大朝向误差时先转向，避免边大幅偏转边前冲。这是已知直通道上的小型反馈律，不搜索其它路径；障碍保护失败时停车，不自行绕行。WAIT_DOOR、STOPPING和终态发零目标，不调用这个前进反馈律。

## 状态机的测量护栏

状态机的顺序为`SCAN→APPROACH→WAIT_DOOR→PASS→GOAL→STOPPING→SUCCEEDED`。任一失败应保留原因，进入有限停止与结果记录，不能通过跳到下一状态“解除卡住”。

| 边界 | 必须能从记录解释的内容 |
| --- | --- |
| SCAN结束 | 本轮新数据与初始观察时间，不能沿用上轮缓存 |
| APPROACH结束 | front<0.65 m或x>−0.85 m；哪个保护先触发 |
| WAIT_DOOR结束 | q>1.10 m、front>1.20 m连续0.30模拟秒 |
| WAIT_DOOR失败 | 6模拟秒内清空条件未满足，零目标和门前真实状态 |
| PASS结束 | x>0.65 m后才请求关门；阈值与车体后缘间隙的关系 |
| GOAL结束 | odom中\|2.20−x\|<0.05 m，随后持续零目标 |
| STOPPING结束 | world实测\|x−2.20\|<0.06 m、\|y\|<0.10 m、平面速度<0.028 m/s、\|偏航角速度\|<0.08 rad/s，且门q<0.05 m，连续0.50模拟秒 |

连续条件采用稳定窗口，窗口内任一条件不满足就重新计时。这能够拒绝一帧瞬时清空。APPROACH与WAIT_DOOR使用不同距离阈值，对应不同阶段；不能把两值相减就称为全部安全余量。PASS还要求前向扇区有有限有效值且近距不小于0.30 m，失去条件就失败停车。

控制器仅在odom年龄小于0.15模拟秒、scan年龄小于0.30模拟秒且frame身份正确时推进对应状态；不新鲜时先归零。任务总时长上限40模拟秒和90墙钟秒，odom墙钟反馈超过3秒未更新也失败。接触传感器监测的是车身chassis_collision，出现接触事件会失败；它不等于已监测机器人所有collision的完整接触集合。

小车参考点、激光原点与车头外廓不是同一点。front是从激光出发测得的斜距或扇区近距，x是指定参考系的车体位置，q是门关节行程。它们单位可能都为m，仍不能互换。闭门朝来车方向的表面约为x=−0.10 m；居中、朝+X且忽略姿态微变时，正前方预测为`r≈−0.10−(x+0.15)`。初始x=−1.60 m得r≈1.35 m，r=0.65 m约对应x=−0.90 m。这说明近距保护预计会先于x>−0.85 m的位置保护触发；实际由离散采样与真实运动决定。

停车预测至少要考虑采样/控制延迟、减速过程、安装偏移及车体外廓。PASS结束的x>0.65 m使名义车身后缘x>0.34 m，已经越过门板x≈−0.10到−0.04 m所在区域；仍应核对真实侧向/朝向误差与接触，而非只凭参考点过了x=0就关门。

门关闭故障只禁用本课自动门控制，不破坏主机、系统插件或其它会话。有效失败应显示机器人接近后停止、门保持关闭、等待超时、状态FAILED；原始数据与失败结果留在原目录。恢复开启权限后，需要新任务实际通过并停住，旧报告不改写。

## 复位与时间回退

`lab.py reset`先确认暂停，再调用任务reset：清除状态、扫描/odom缓存与上一轮计时，恢复门控制默认权限并请求闭门，准备新的任务身份。它保留机器人位姿，下一次start重新建立稳定窗口。关闭门故障后的小车可从实际门前位置重新采样并恢复；成功后已在门后的机器人会被start拒绝，需要`lab.py reset --world`或重新加载本课，明确恢复世界初态。直接调用`/lesson36/reset`服务本身不负责暂停，命令行封装才执行暂停操作。

Gazebo世界reset或重载会改变物理实体与时钟。检测到模拟时间回退时，跨epoch缓存必须废弃，不能用回退前后的odom插值，也不能继续旧任务的超时和清空累计。TF旧动态数据、消息新鲜度和日志时间基准也应按新一轮解释。Noetic rospy的Rate/Timer对回退有明确处理语义，单纯使用ROS计时器不等于任务缓存自动正确复位。[Noetic timer.py](https://github.com/ros/ros_comm/blob/noetic-devel/clients/rospy/src/rospy/timer.py)

正常结束、失败、主动reset、进程异常收尾都应请求零目标。本课原生差速插件保存最后命令，不能期待“Python退出以后小车自然知道该停”。零目标的发送记录与实际停车反馈都必须保留。只请求暂停可能把运动中的状态冻结，不能把被冻结当作已完成物理减速。

## 来源、日志与结论层级

`build/warehouse.json`记录当前参数、源码、源world与构建产物摘要，以及实际SDK头文件、链接/运行库和系统ROS插件身份。应用加载现有world.sdf，力上限通过启动确认进入门插件。哈希证明字节对应关系，不能证明物理行为正确。修改文件后重新准备并重载，旧进程里加载的共享库不会因磁盘文件变动自动更新。

每次任务在`build/runs/<run_id>/`建立独立目录。阅读顺序建议为：result.json中的结果与失败原因→events.jsonl的状态与真实状态事件→raw_ros.jsonl中的ROS原始字节→points.csv→map.svg。build/latest_run.json指向最近已写完结果的目录。成功与失败都保留，不能让后一次运行覆盖前一次证据。

raw_ros.jsonl保留收到的ROS消息序列化字节（Base64）、消息源、可用的原stamp及单调墙钟接收时刻；Twist没有header，它的stamp字段为缺失，接收时刻不能改称消息自带时间。events.jsonl中的ground_truth是ROS状态服务实际读回的所选字段，含门q/rate与机器人位姿/速度。门命令确认还含最近物理更新的q、velocity、force；这些离散确认不是连续施力曲线，也不等于ROS服务返回了力。

| 数据 | 能支持的结论 | 单独不能支持的结论 |
| --- | --- | --- |
| 参数与输入摘要 | 当前任务采用哪份输入 | 已经真实执行 |
| Twist记录 | 当时请求了什么速度 | 当时实际车速 |
| 状态事件 | 控制器何时作出决策及理由 | 该决策的输入必然正确 |
| LaserScan与投影点 | 当时测到了哪些射线与位置 | 未观测处自由、完成SLAM |
| 门关节q、q̇与命令确认中的F | 实际行程、速度；确认时最近的施力样本 | 连续F(t)曲线、任意车宽都能通过 |
| 实际位姿与速度 | 真实过门、目标误差、有限阈值内停车 | 已验证任意仓库路线 |
| GUI人工观察 | 显示场景与运动的对应 | 精确末速与数据来源 |

课程verify.py复位世界，运行一次正常任务，检查实际终态、完整状态顺序、车身接触计数、实测点数量和CSV投影算式；它不自动执行关闭门失败及恢复两轮。后两轮使用教学页的明确命令保留独立结果，开发端独立验收另行检查这组行为。自动验收只对实际执行并记录的条件负责。静态检查能核对XML、接口和调用路径；真实Gazebo运行能核对物理结果；人工GUI观察能核对可见过程。三种证据分别注明，不把“源码里有判断”“接口有消息”或“世界能加载”当作整个任务通过。

## 自己修改后的算法说明

改车速时，解释Twist→左右轮目标→接触运动→到门时刻与停车距离这条链。改门上限时，解释目标误差→PD计算→限幅→实际q(t)→稳定清空时刻这条链。改投影时，解释原始束→有效值过滤→扫描时刻位姿→安装变换→世界点→CSV/SVG这条链。改FSM时，列出新增或改变的guard、超时、零目标路径与需重跑的失败案例。

保留预测和实际值的区别，说明哪些阈值依赖当前通道几何。一次成功不能扩大成“可用于任意仓库的导航”，一次失败也不能靠删除超时或忽略碰撞来解决。
