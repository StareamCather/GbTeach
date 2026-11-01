# 力与力矩：称出重量，也看见力臂

两套相同的称重台各有一块1 kg的立方体，通过各自的固定关节悬在支座一侧。它不动，力传感器为什么仍有读数？质量不变、把重心移远，竖直力几乎不变，力矩为什么增大？本课让真实关节约束承受重力，用六维wrench解释重量、力臂、坐标轴和测量方向。

模型中的负载是动态刚体，重力开启；固定关节约束它，不靠把整个模型设为static，也不靠程序持续写位置来伪装支撑。几何公式用于预测，输出来自ODE真正的关节反馈。本课先研究静态负载，不要求先学机器人动力学或编写传感器插件。

## Wrench里的六个分量

**wrench**将三维力和三维力矩放在一起；**力矩（torque）**描述力绕某个参考点产生的转动作用。它不是角度，也不是关节已经转了多少。ROS的[Wrench定义](https://github.com/ros/common_msgs/blob/noetic-devel/geometry_msgs/msg/Wrench.msg)包含force与torque，带时间和坐标标签的版本是[WrenchStamped](https://github.com/ros/common_msgs/blob/noetic-devel/geometry_msgs/msg/WrenchStamped.msg)。

| 数据 | 单位 | 读数时必须同时知道 |
| --- | --- | --- |
| force.x、force.y、force.z | N，牛顿 | 对哪一侧的作用、用哪组轴表达 |
| torque.x、torque.y、torque.z | N·m，牛顿米 | 同样的方向约定，以及关于哪个参考点 |
| header.stamp | 仿真时间 | 数据属于哪个观察时刻 |
| header.frame_id | 表达轴的标签 | 标签不自动完成TF，也不能省略力矩参考点说明 |

质量用kg，重量是重力作用的力，量级约mg，不能把1 kg直接填进“力=1 N”。静止时合外力近零，但传感器测量的某一侧载荷不必为零；支座还要传递平衡力矩，所以不转动也不表示力矩必定为零。

## 先预测符号，再看模长

原生称重台weighing_rig的锚点O位于世界(0,0,1.2) m，load_link原点也在O。系统插件称重台stock_weighing_rig的锚点O_stock位于(0,1,1.2) m，stock_load_link原点与它重合。两台的局部坐标轴都与世界轴对齐，质量、力臂和质心惯性相同，且分别固定于world。负载立方体边长0.2 m，默认质量m=1 kg，质心相对O在r=(L,0,0)，L=0.3 m。可视化支臂只帮助显示力臂，不给它额外添加一份未说明的质量。

本课默认读的是child_to_parent：负载通过关节施加给支座的作用。它的向下重量与绕O的等效力矩预测为：

```text
F = (0,0,−mg)
tau_O = r × F = (0,+Lmg,0)
```

默认Fz≈−9.81 N，tau_y≈+2.943 N·m。父侧作用到负载的支持力与支持力矩方向相反；不能不看测量方向，就要求每路力都向上。检查只取绝对值会掩盖这些错误，本课保留六个分量和符号。

**叉积（cross product）**产生一个同时垂直于r与F的向量，方向遵循右手定则；力臂的有效长度是到力作用线的垂直距离，不是任意一段模型尺寸。不了解叉积，可看[OpenStax叉积章节](https://openstax.org/books/calculus-volume-3/pages/2-4-the-cross-product)；力矩基础可看[OpenStax力矩图解](https://openstax.org/books/university-physics-volume-1/pages/10-6-torque)，不必先读完整大学力学教材。

## 四路输出，比较三种不同问题

原生台的mount_joint提供三种ForceTorqueSensor配置；相隔1 m的系统台由stock_mount_joint提供一条系统ROS输出。下面各力矩分别关于本台的锚点，不把两台的力矩移到同一个世界原点：

| 观测 | 表达轴与方向 | 默认力/N | 默认力矩/N·m |
| --- | --- | --- | --- |
| 系统ROS FT（独立系统台） | stock child轴、child_to_parent | (0,0,−9.81) | (0,+2.943,0) |
| 原生A | child轴、child_to_parent | 同上 | 同上 |
| 原生B | child轴、parent_to_child | (0,0,+9.81) | (0,−2.943,0) |
| 原生C | sensor轴，相对child绕+X转90°、child_to_parent | (0,−9.81,0) | (0,0,−2.943) |

A与系统ROS读相同方向，B只改变作用方向，C只改变表达轴。C中力和力矩的模长不因换轴改变，分量位置与符号却会变；这是坐标表达变化，不是换了一块负载或增加新的作用力。A、B、C共享原生台的关节反馈；系统输出来自另一台物性相同的真实负载。比较前要分别证明两台的物性一致、各自锚点与本台child原点重合、坐标轴平行且分别静止。两台世界锚点分别为(0,0,1.2) m和(0,1,1.2) m；比较的是各自锚点处的等效载荷，不能只凭同样的标签当作四台独立物理仪器互相标定。

```sh
rostopic echo -n 1 /lesson19/native_child
rostopic echo -n 1 /lesson19/stock_wrench
python3 lab.py observe --seconds 3
python3 verify.py
```

observe按墙钟限时，只读实际数据。verify分别检查两台的真实质量、质心、惯性、碰撞几何、固定关节锚点和稳定状态，再采集各路新鲜wrench，比较重量、力臂、符号和换轴结果；它不向消息主题发布计算好的力，也不反复设置负载位姿求一个“稳定读数”。暂停会停止新的物理反馈，不能凭等待超时便断言传感器损坏。

验收避开前0.2仿真秒，每路至少30条不同时间戳、跨度至少0.6 s，目标50 Hz，实际频率要求40到60 Hz。有效窗口内每个力分量距预测不超过0.05 N，每个力矩分量不超过0.03 N·m；四路还须覆盖至少0.5 s共同稳定区间，并为每台都取得至少3次独立连杆状态查询。它检查持续的真实反馈，不只挑一条接近mg的消息。

报告保存在本会话wrench-verify-唯一编号.json，终端打印trace_path；可用`python3 verify.py --report build/my-wrench-check.json`指定位置。原始各路时间戳、六分量、实际质量/质心、独立状态和前后暂停状态一起保存；验收结束恢复原暂停状态，不重设负载或倒回时间。

请使用平台打开的本课终端。原生A、B、C的主题分别为`/lesson19/native_child`、`/lesson19/native_parent_to_child`、`/lesson19/native_sensor_x90`；系统输出为`/lesson19/stock_wrench`。A、B标为load_link，系统输出标为stock_load_link，C标为sensor_x90。C的数值换轴发生在原生传感器内部，桥只复制已经换轴的结果并填写对应标签，不靠标签制造旋转，也不自动发布这两个frame之间的TF。

## 系统ROS插件与原生传感器不是同一种接口

```text
原生台：真实重力 + 动态负载 + mount_joint固定约束
  → ODE关节反馈 → 原生ForceTorqueSensor A/B/C
       → 原生WrenchStamped → 只作消息适配的桥 → ROS
系统台：相同物性、独立负载 + stock_mount_joint固定约束
  → 独立ODE关节反馈 → 系统GazeboRosFT ModelPlugin → ROS
  → 四路稳定窗口比较
```

系统libgazebo_ros_ft_sensor.so是**ModelPlugin**，直接读取joint.GetForceTorque的body2反馈，不是一个负责包装原生force_torque传感器的SensorPlugin。它固定使用child坐标轴与child_to_parent方向，header由child link名称产生，不能添加一个frameName或measure_direction就期待它自动改方向。

原生ForceTorqueSensor必须挂在joint下，可以选择parent、child或sensor表达轴，以及两个measure_direction。本课自带的桥只转发原生传感器实际消息，不生成理想wrench；插件实现留到第28课，不是本课前置要求。实际接口见[Noetic FT插件](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/2.9.3/gazebo_plugins/src/gazebo_ros_ft_sensor.cpp)、[Classic ForceTorqueSensor实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/ForceTorqueSensor.cc)与[官方力/力矩教程](https://classic.gazebosim.org/tutorials?tut=force_torque_sensor&cat=sensors)。

## 表达轴与力矩参考点要分开

如果只在同一个物理点换轴，旋转矩阵R把旧表达转成新表达，那么`F_new=R F_old`、`tau_new=R tau_old`。若参考点从A换到B，还要加力臂项：

```text
r_BA = position_A − position_B   （从B指向A，表达在同一组轴）
F_B = F_A
tau_B = tau_A + r_BA × F_A
```

只旋转分量不能完成参考点平移。比如重量作用在质心处，关于质心的这一重力力矩可为零；移到支座O描述同一负载，则出现r×F。本课load_link原点特意与joint anchor重合，header轴和关于O的说明能够对应，避免把不同原点悄悄混为一个。

在当前Classic原生ForceTorqueSensor中，sensor pose的旋转可改变表达轴，**平移不会将力矩参考点移到sensor pose位置**；力矩仍关于关节原点。底层ODE接口已将原始质心反馈换算到joint anchor，所以也不能对收到的数据重复加一次“质心到关节”的力臂补偿。模型放在哪里、用哪组轴表达、力矩关于哪里，是三个应分别核对的问题。

## 单变量：质量与力臂

world.sdf是真实个人输入。一次只修改一项，保存后重载，确认实际质量与质心已加载，再重新观察：

```sh
python3 lab.py config --mass 2
python3 lab.py config --arm 0.5
```

| 输入变化 | 力的预测 | 关于O的力矩预测 |
| --- | --- | --- |
| m=1→2 kg，L仍0.3 m | Fz从−9.81到−19.62 N | tau_y从+2.943到+5.886 N·m |
| L=0.3→0.5 m，m仍1 kg | Fz仍约−9.81 N | tau_y从+2.943到+4.905 N·m |

可调质量0.5到3 kg，力臂0.15到0.6 m。质量工具联动更新关于立方体质心的惯性；力臂工具联动质心和负载几何的位置，不把只移动一个visual当成实际载荷改变。立方体的惯性参数关于质心，不应再往里面随意加mL²来重复计算原点偏移。

两条示例命令应分别在相同起点做实验，不连续执行后还声称“只改变了一个变量”。重载前旧运行世界不会自动采用新质量；准备工具检查输入，不恢复模板覆盖个人修改。

## 反馈、时间与稳定窗口

关节反馈需要启用，本课显式设置provide_feedback，原生传感器初始化也会启用它。因此把SDF里的开关删掉后，即使仍看到数据，也不能据此推出该开关永远无用：原生传感器可能替你打开了反馈。固定课包的错误会先被检查拒绝，不伪造一个“删除开关必得零”的运行结论。

系统ROS插件在WorldUpdateBegin读取关节已有反馈，再以当前仿真时间标记；原生传感器有自己的采样调度。稳定负载适合比较均值和符号，但相近时间戳不意味着任意动态情况下都是严格同时的反馈。真实频率要按各路不同header时间估计，墙钟等待用于控制程序超时，不能代替物理时间推进。

当前实验用零噪声隔离力学关系，静态mg和Lmg不是任何运动过程都成立的恒等式。负载加速、旋转或碰撞时，反馈还包含惯性及其它外载；也不能把这六个合量当成接触压力分布。第14课接触点数据和本课关节wrench回答的是不同问题。

## 文件、故障与独立挑战

| 文件 | 职责 |
| --- | --- |
| world.sdf | 真实动态负载、固定关节、原生传感器与系统ROS FT插件 |
| prepare.py | 检查参数、依赖并准备本课消息适配组件 |
| wrench_bridge.cc | 只转发真实原生力矩消息，不设置载荷或生成理论值 |
| lab.py | 参数编辑、只读四路观察与测量公共逻辑 |
| verify.py | 真实静态反馈、符号/换轴与稳定窗口验收 |
| reference.md | 只读叉积、换参考点与示例推导 |

首次准备会用已安装的Gazebo 11和ROS开发包离线编译消息桥，后续复用匹配缓存；源码、生成库与个人world的指纹记录在build/wrench_lab.json。构建是平台初始化的一部分，不要求先写C++，也不修改系统FT插件。

| 现象 | 先检查什么 |
| --- | --- |
| 负载静止却有非零力 | 测的是一侧载荷，不是负载的合外力 |
| 力向下，不像向上支持力 | child_to_parent还是parent_to_child，别先取绝对值 |
| 力矩符号反了 | r的方向、叉积顺序、坐标轴与作用方向 |
| 改visual位置，力矩没变 | 实际质心是否改变，是否保存并重载 |
| 原生C的Z力消失、Y力出现 | sensor轴旋转90°的预期，不是重力关掉了 |
| 改frameName没有改变系统FT | 该ModelPlugin不靠这个字段选择表达轴 |
| sensor平移后力矩没变 | 当前原生接口仍关于joint anchor，需另做参考点换算 |
| 数据一直不更新 | 本课Master、暂停、订阅、反馈与加载日志 |

独立挑战：用实测child_to_parent的F与tau_O，将力矩换算到质心处。先明确从质心指向O的向量，再使用平移公式；理想静态纯重力情况下结果应接近零。解释为什么这不表示支座不承受力矩，再举两个不同受力分布可能产生同一wrench的例子，说明六个合量不能唯一恢复压力图。


## 为什么系统输出放在独立称重台

Classic 11.15.1的ODEJoint.GetForceTorque会改写关节对象中共享的中间wrench。系统插件在WorldUpdateBegin读取，原生FT在OTHER传感器线程读取，同一个关节被这两条路径同时读取时可能重入这段转换。一次保留的真实故障中，静止负载出现Fz=+9.81 N、tau_y=−5.886 N·m；源码的力臂累加与符号转换交错可以解释这一对值。这是源码与观测一致的因果推断，没有观测完整线程调度，不能写成已经捕获全部竞争指令。

本课把系统插件移到物性相同但关节独立的称重台，原生三传感器仍由同一个OTHER容器顺序更新。它消除两种读取路径对同一JointWrench中间对象的并发修改；不修改主机SDK、系统插件、输出符号、数值容差或原生桥。两台都由实际重力和关节约束产生反馈，关于各自锚点的静态预测相同。

config先验证两台质量、力臂、惯性和几何成对一致；修改mass或arm时原子更新两台，再重新校验。ready/verify通过ROS读回两台真实物性和世界位姿，并以有界只读entity_info核对实际关节、child局部锚点、轴向及碰撞尺寸；只读查询最多两次，合计预算8 s，错误明确失败。固定关节的实际局部锚点为零，配合实际child世界位姿确定各自世界锚点。这些读取来自不同时间，报告保留原始模型消息及时间边界，不声称原子快照。随后在同一个真实公共窗口逐条检查四路有符号数据，并分别核对两台持续静止；异常值不会取绝对值、放宽阈值或丢弃来凑通过。
