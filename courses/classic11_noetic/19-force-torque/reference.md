# 力与力矩：只读静力学和接口说明

此页不覆盖个人world.sdf。公式用于预测，实际wrench由动态刚体的重力和固定关节约束产生；本课不向传感器输出主题注入理想数据。

## 关于关节O的静态载荷

原生台锚点O=(0,0,1.2) m，load_link原点与O重合；系统台锚点O_stock=(0,1,1.2) m，stock_load_link原点与O_stock重合。两台相隔1 m，使用独立固定关节及相同物性，局部轴均与世界轴平行。以下公式分别关于各自锚点。边长0.2 m的均匀立方体质心相对O为r=(L,0,0)，世界重力加速度向量为g_W=(0,0,−9.81) m/s²；下式g=9.81表示其大小。该负载传给支座的等效作用为：

```text
F_child_to_parent = (0,0,−mg)
tau_O = r × F = (0,Lmg,0)
```

叉积按分量写为`r×F=(r_yF_z−r_zF_y, r_zF_x−r_xF_z, r_xF_y−r_yF_x)`。在本例中tau_y=−L×(−mg)=+Lmg，不能把“向下力”直接等同为“每个力矩分量都负”。交换叉积顺序会反号，r也必须从参考点指向力的作用位置。

| m/kg | L/m | Fz/N | tau_y/N·m |
| --- | --- | --- | --- |
| 1 | 0.3 | −9.81 | +2.943 |
| 2 | 0.3 | −19.62 | +5.886 |
| 1 | 0.5 | −9.81 | +4.905 |

支持方施加给负载的作用方向相反，平衡负载的重量与绕O的重力矩。负载静止，只能说明合外力和合外力矩满足相应平衡，不能由此说关节的某一侧wrench必须为零。运动时还存在惯性和其它载荷，本表的静力预测不直接套用全部动态样本。

## 惯性与质心的输入

均匀立方体边长a=0.2 m，关于质心、平行于边的惯性为`Ixx=Iyy=Izz=m a²/6`，非对角项为0。m=1 kg时每项约0.0066666667 kg·m²，m=2时加倍；重心偏置由inertial pose表示，不将mL²再次加进这组质心惯性。

只改变arm时，工具对两台成对联动质心、碰撞体、负载visual及显示用支臂；负载的米制尺寸和关于质心的惯性不因平移而改变。支臂为可视化，不再添加独立承重质量。保存输入须重载，程序核对实际质量、CoM和关节结构，不用颜色或显示位置代替物性。

## 同一点换轴

R把旧坐标表达转到新坐标表达，同一物理参考点上的wrench为：

```text
F_new = R F_old
tau_new = R tau_old
```

原生C的sensor相对child绕+X转90°，把child轴表达转换到sensor轴要用Rx(−90°)。因此默认：

```text
(0,0,−9.81) → (0,−9.81,0) N
(0,+2.943,0) → (0,0,−2.943) N·m
```

换轴不改变力和力矩各自的模长。原生B改变parent_to_child测量方向，六个分量整体反号；这与C的换轴是不同操作。比较作用反作用前要先统一坐标轴和力矩参考点，不能把不同局部轴下的原始分量直接相加。

## 换力矩参考点

在同一组轴中，若已知关于A的力矩，换到B：

```text
r_BA = p_A − p_B
F_B = F_A
tau_B = tau_A + r_BA × F_A
```

这里r_BA从B指向A。例如A=O、B=质心P，r_PO=(−L,0,0)，于是`tau_P=(0,+Lmg,0)+(−L,0,0)×(0,0,−mg)=0`。这表示同一等效负载关于重心的纯重力力矩为零，不表示关于支座O的力矩消失。

如果又要换轴，应把所有量表达在一致坐标系后执行对应的旋转与力臂换算，不能只旋转tau便声称已完成平移。不了解原理，可先看[力矩和力臂](https://openstax.org/books/university-physics-volume-1/pages/10-6-torque)及[叉积](https://openstax.org/books/calculus-volume-3/pages/2-4-the-cross-product)的图解。

## 三条实现边界

第一，系统[GazeboRosFT](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_ft_sensor.cpp)注册为ModelPlugin，在模型下按jointName读取GetForceTorque(0)的body2Force/body2Torque，固定child轴、child_to_parent方向，header使用child link名和可选TF前缀。它不读取一个任意frameName来旋转数据，也不是原生ForceTorqueSensor消息包装器。

第二，[原生ForceTorqueSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/ForceTorqueSensor.cc)挂在关节上，选择parent、child或sensor表达轴，以及parent_to_child或child_to_parent方向。frame=sensor使用sensor pose的旋转部分，平移不换力矩参考点；[官方教程](https://classic.gazebosim.org/tutorials?tut=force_torque_sensor&cat=sensors)明确力矩仍关于关节原点。本课的桥只把实际原生消息复制到ROS，不重新用mg或r×F生成值。

第三，[ODEJoint反馈处理](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODEJoint.cc)已经将质心处反馈换算到joint anchor，并按相应link轴表达。这一层的力矩参考点不同于一般情况下child link原点；本课故意令两者重合，不能推广为所有机器人都重合，也不能收到数据后再重复做一次同样的质心到关节补偿。

wrench_bridge.cc检查父对象确为原生ForceTorqueSensor，订阅它真实的Gazebo WrenchStamped，将原始仿真时间、force与torque复制为ROS WrenchStamped；C的轴旋转已经在原生传感器内部完成。桥的frameName只是相应表达轴的标签，不实施TF变换或力矩换点。prepare用已装SDK离线编译，缓存键包含源码、Gazebo版本和编译链接参数，build/wrench_lab.json保留源码、库及world指纹；字节身份记录不替代真实力学验收。

header提供时间和轴标签，不单独编码“关于哪个点”的全部语义，也不发布TF。本课三种原生配置共享原生台mount_joint；系统插件只读系统台stock_mount_joint，使用stock_load_link标签。两台反馈属于不同真实负载，必须先分别验证质量、CoM、惯性、锚点、轴与静止状态；比较是在相同静力条件下检查接口、方向和坐标约定，不是假设四路是同一条瞬时反馈。

## 反馈、采样与验收流程

实际关节需要provide_feedback，本课显式声明。原生传感器Init也会调用SetProvideFeedback(true)，所以删除SDF开关未必让反馈关闭；这样的运行不能证明“该开关无效”或“删掉必得零”。固定结构校验会拒绝错误配置，故障练习应区分输入拒绝与真正运行过的失效。

原生always_on保证本课采样持续开启；若关掉且没有订阅，直接读取Force()/Torque()可能得到旧值。系统ROS插件由WorldUpdateBegin读取已完成物理步骤留下的反馈，header标当前仿真时间；原生传感器调度另行采样。稳定静态窗口适合比较均值与符号，不承诺任意动态工况下严格同步。

prepare校验真实SDF、参数和依赖并准备消息适配组件；config工具只明确修改质量或力臂，保存后重载。observe限时读取四路实际数据，不改变物理状态。完整验收逐台检查实际质量、质心、惯性、重力、原生碰撞几何、锚点、轴与负载静止状态，采集各路不同时间戳的真实反馈，再按当前输入计算独立的静力预测和坐标对照；不将预测写进消息。

从明确的观察边界避开前0.2仿真秒，每路至少30条唯一时间戳、首末跨度至少0.6 s，频率40到60 Hz。逐条有符号力分量误差≤0.05 N、力矩分量≤0.03 N·m，原生A与系统输出的窗口均值各对应分量之差须小于0.03（力用N、力矩用N·m）。每路分别统计最小、最大、中位与均值，不要求各条消息跨通道具有完全相同的时间戳。

四路观察必须有至少0.5 s公共区间，其中每台都要至少3次独立连杆状态查询证明仍静止。连杆原点距固定锚点不超过0.005 m，线速率与角速率分别不超过0.01 m/s和0.01 rad/s，四元数向量部分模不超过0.005、总模距1不超过1e−6；实际质量、CoM、重力模式与质心惯性另行核对。查询带前后仿真时间边界，不把一张截图或六个预置常量当作静力证据。

默认报告路径为本会话wrench-verify-唯一编号.json，也可用--report指定。trace保留各路实际消息，truth与stock_truth分别保留两台的独立查询及时间边界，rig_properties保留两台真实质量/质心/惯性，rig_geometry保留实际entity_info的原文、child局部锚点与碰撞几何；actual_properties兼容字段指原生台，stock_actual_properties指系统台，restore记录原暂停状态与恢复结果；源码及桥生成库指纹随报告保留，便于区分实际测量版本。

验收只恢复进入前的暂停状态，不靠重设负载来建立“静态”。测量时不要同时施加外力、拖动模型或复位，世界时间不会倒回。有限稳定窗口证明已实际观察到的样本，不代表每个未发布的物理步都被记录。

## 为什么六个合量不是压力分布

力单位N，压力单位N/m²；关节wrench记录合力与关于指定点的合力矩，不包含每个接触位置上的面积和压力。不同接触分布可产生同样的F与tau，因此一般不能由六个数唯一反推出接触面压力图。第14课的接触点列表提供另一层信息，也仍需说明接触面积与求解模型。

独立挑战先用实测F与tau_O计算tau_P，保留符号和参考点说明；再比较两个不同分布但相同合wrench的例子。不要把“关于质心力矩接近零”写成“传感器没有承受载荷”，也不要把静态mg关系当作动态惯性载荷下的通用答案。


## 同关节读反馈的并发边界

[11.15.1 ODEJoint.GetForceTorque](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODEJoint.cc#L615)把原始ODE反馈写入共享this->wrench，累加质心到锚点的力臂项，再原地转换力/力矩的符号及坐标。该函数没有共同互斥，CacheForceTorque在此版本为空；[系统2.9.3插件](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_ft_sensor.cpp#L127)从WorldUpdateBegin调用它。[原生FT](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/ForceTorqueSensor.cc#L194)先按值取得JointWrench，随后才对自己的消息应用方向和表达轴，所以不是反向传感器直接改了另一个传感器的消息。

[SensorManager](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/SensorManager.cc#L787)对同一OTHER容器中的三原生FT顺序更新，容器运行在线程中；它的容器锁和每个FT的私有消息锁不覆盖系统插件的读取。lockstep会改变采样节流与等待条件，不能据此假定OTHER线程和物理更新回调取得共同的关节锁。

保留的首个失败记录中，m=1、L=.3且link真值仍静止，native_child和stock_child在6.74 s同时出现(+9.81,−5.886)，native_child在7.02 s再现。两个读者先各自重写共享原始值、再各自累加−2.943、随后各翻一次力和力矩的符号，可以得到这一对数值。该指令交错是说明观测与源码相容的推断；没有记录实际完整调度，也不声称纯算术演示就是Gazebo竞争的动态复现。

当前结构通过两个独立关节隔离这两条读取路径，系统台无原生FT，原生台无系统FT ModelPlugin。固定参数校验拒绝只改一台的质量/几何、改变系统台锚点、移除反馈或把读取者重新挂回同一关节。准备和参数修改流程不生成测量值；原生消息桥C++保持按原字节分量复制。8 s总预算内至多两次只读实体查询保存实际Joint.pose和child/collision信息，ROS另外读两台世界位姿与物性；Joint.pose是child局部锚点，实际零偏移与实际child世界pose共同确定锚点及表达轴。读取不是原子快照，稳定公共窗口仍必须同时覆盖两台真实静止查询，原先所有有符号逐样本判据继续有效。
