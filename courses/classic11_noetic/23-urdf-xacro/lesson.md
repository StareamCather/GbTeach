# URDF与Xacro：从描述文件得到真实机器人

第22课解决了“怎样把程序启动并连接起来”。本课解决另一个问题：机器人由哪些刚体组成，它们绕哪里转，质量分布在哪里，描述怎样进入Gazebo成为实际动力学模型。

URDF（Unified Robot Description Format，统一机器人描述格式）用XML描述机器人结构；Xacro（XML Macros，XML宏）负责生成这些XML。模型描述、生成文件、参数服务器上的字符串和Gazebo里的实体是不同阶段。完成本课需要读通这条链，并用实际几何、物性和关节运动核对它。

## 先分清四样东西

| 对象 | 解决的问题 | 检查内容 |
| --- | --- | --- |
| Xacro源文件 | 怎样复用、计算和选择描述片段 | property、macro、arg、条件与include |
| 展开的URDF | 最终有哪些link、joint与数值 | 树结构、坐标、几何、质量与惯性 |
| 转换后的SDF | Classic怎样表达这份机器人 | Gazebo扩展、固定关节合并与实际名称 |
| Gazebo运行实体 | 当前真实世界加载了什么 | 模型存在、link/joint、位姿、物性与运动 |

robot_description是常用的ROS参数名，通常保存展开后的URDF字符串。把字符串写入参数服务器只是存好描述；还需要调用生成实体的接口，才能在Gazebo里得到模型。只看到参数或XML解析成功，不能证明实际机器人已经加载。

本课继续复用平台提供的私有Master与Gazebo，不在课程launch里另开一套。一次性spawner（生成模型的程序）成功后可以正常退出；它的生命周期与需要持续运行的节点不同，不应照搬上一课持续节点的required=true。

## 本课的水平两关节桌面臂

模型名为xacro_arm。底座通过fixed关节连接world，安装高度0.7 m、偏航0.2 rad；肩关节再沿底座Z偏移0.08 m。两转动轴均为各自关节坐标系的+Z，第一段长度L1=0.6 m，第二段L2=0.4 m。肘关节放在第一段末端，并带0.15 rad零位偏航；第二段末端连接质量0.02 kg的球形工具。

这是水平运动的2R（两个revolute关节）结构。各动态link禁用重力，底座实际固定到world；模型整体保持可参与动力学的状态。教材中的球形工具不是只有外观：保留或合并时都要计入真实质量分布。

```text
world
 └─ lesson23_world_fixed → base_link
     └─ lesson23_shoulder → arm1
         └─ lesson23_elbow → arm2
             └─ lesson23_tool_fixed → tool_link
```

arm1质量1 kg、arm2质量0.5 kg，截面均为0.06×0.06 m；工具球半径0.025 m。肩关节角度范围±1.4 rad，肘关节±1.2 rad；两关节都声明0.03 N·m·s/rad的黏性阻尼。有限力矩结束后，仍由耦合惯性和阻尼继续决定运动，不照搬上一课无阻尼试块的匀速假设。

两个课程输入分别是L1取0.6/0.8 m，以及preserve_tool取true/false。后者只决定是否生成工具fixed关节的preserveFixedJoint扩展，**不删除URDF中的工具link或joint**。所以两份URDF拓扑相同，转换后的Gazebo独立刚体数量及第二段实际物性可能不同。

## link和joint组成有方向的树

**link（连杆）**是具有坐标系的刚体描述，**joint（关节）**说明父link与子link怎样连接和相对运动。URDF按父子关系形成树：一个根，每个非根link有一个父关节，可以分叉；闭环机构需要另行建模处理。

以两转动关节结构为例，沿树读就是“底座 → 肩关节 → 第一段连杆 → 肘关节 → 第二段连杆”。这里的树是拓扑关系，不是按XML书写顺序把所有坐标连续相加。节点名、link名、joint名和Gazebo模型名也分别属于不同对象。

| 类型 | 自由度与单位 | 需要注意 |
| --- | --- | --- |
| fixed | 无相对自由度 | 固定连接，不代表整个机器人静止 |
| revolute | 有限范围转动，rad | axis与lower/upper、effort、velocity |
| continuous | 连续转动，rad | 没有上下角度界，仍需合理动力学描述 |
| prismatic | 沿轴平移，m | 不是把角度字段改名，限位单位也变化 |

关节限位是约束/能力描述，写了effort或velocity并不会自动生成控制器。URDF里的mimic也不能无条件当作Gazebo物理引擎已建立齿轮约束；实际接口和动力学行为仍需检查。

不了解结构树，可先看[URDF模型解析器](https://github.com/ros/urdfdom/blob/1.0.4/urdf_parser/src/model.cpp)；它检查名称、父子关系与根。读源码是进一步理解接口的入口，不是开始实验的前置考试。

## origin到底相对谁

同样的origin字样，放在不同元素中有不同的参照关系：

| 所在位置 | xyz/rpy表达什么 |
| --- | --- |
| joint/origin | 父link到关节零位坐标系的变换 |
| visual/origin | 几何显示坐标系相对所属link的位置与朝向 |
| collision/origin | 碰撞几何相对所属link的位置与朝向 |
| inertial/origin | 质心及惯性坐标系相对所属link的位置与朝向 |

把link原点放在转轴处，通常比放在几何中心更方便：长条的visual、collision和质心都可以沿局部X偏移半个长度，而子关节放在末端。这样旋转的是整个子树，不需要每次手动移动几何。

joint的axis在**关节坐标系**表达。若joint/origin带旋转，axis也随该坐标系转到父系；不能直接把axis的数字当作world轴。省略axis还可能采用默认轴，所以课程显式写清单位方向。

URDF的rpy使用弧度，按固定轴roll/pitch/yaw约定解释。对列向量，旋转矩阵可写为Rz(yaw)Ry(pitch)Rx(roll)。例如关节零位绕父Z旋转π/2后，局部axis=(1,0,0)对应父系+Y；不是再绕world X转。

对于转动关节，子系随q变化可理解为“父到关节零位的固定变换 × 绕该joint轴的转动”。正角按右手定则；关节角不是任意两个物体world yaw直接相减就一定成立。

不了解坐标变换的复合，可看[Modern Robotics齐次变换讲解](https://modernrobotics.northwestern.edu/nu-gm-book-resource/3-3-1-homogeneous-transformation-matrices/)，重点理解变换相乘的顺序和各自参考系。

长度用m、质量kg、角度rad、力矩N·m、转动惯量kg·m²。[REP-103单位与坐标约定](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)给出基础规则；本课先沿实际模型核对，不要求先掌握整套变换树工具。

## 外观、碰撞、质量分布分别建模

visual决定看见什么，collision参与碰撞，inertial决定质量与转动响应。它们可以采用不同的简化程度，但位置和大小应相互一致。漂亮的网格不自动提供正确碰撞或惯性；给visual改颜色也不会改变质量。

**质心COM（center of mass）**是质量平均位置。惯性矩阵说明刚体绕不同方向转动的质量分布，需在明确的质心坐标系中给出。URDF用六个独立分量填写对称3×3矩阵，而不是只填一个“惯性大小”。

对质量m、尺寸L×W×H且与坐标轴对齐的均匀盒体，在质心处：

```text
Ixx = m(W² + H²)/12
Iyy = m(L² + H²)/12
Izz = m(L² + W²)/12
Ixy = Ixz = Iyz = 0
```

这组公式对应盒体质心，不是转轴端点。若link原点在长条一端，应把inertial/origin移到质心，同时填写质心惯性；不要先把I平移到端点，再又用COM偏移让引擎重复处理。绕另一点估算有效惯量时，才使用平行轴定理。

质量需为正、数值有限；对本课实体盒体，惯性矩阵应正定且满足物理主惯量的三角关系。只让三个对角数为正，并不能保证包含非零非对角项的矩阵合法。也不要用全零或随手填很小的惯性掩盖问题。

这里的正定，意味着任意非零角速度都对应正的旋转动能。不了解矩阵正定性，可看[MIT线性代数对应课程](https://ocw.mit.edu/courses/18-06-linear-algebra-spring-2010/resources/lecture-27-positive-definite-matrices-and-minima/)；不必先推导特征值，先用本课均匀几何的正确公式。

本课改变L1时保持各link质量不变，visual、collision、COM、下一关节位置和惯性一起按新尺寸计算。因此应在实际物性中看到质量保持、质心位置与惯性改变。固定密度的缩放规律另见参考拓展。

本机SDK链会把部分数值序列化为六位有效数字，例如arm2理论Iyy约0.0068166667，实际读回可为0.00681667。检查分三层：理论URDF到转换SDF按明确精度预算比较，实际物性再与对应SDF严格核对；报告保留每层原值。正确的单位与物理含义比要求所有小数位机械相等更重要。

## Xacro把重复描述变成有输入的模板

**property**保存可复用数值，**macro**定义可展开片段，**arg**接收命令行映射或默认输入，**if/unless**决定是否生成某段XML。这些操作在生成URDF时发生，不是Gazebo每个物理步重新执行。

个人源包中的关键片段如下，完整文件还包括底座、关节与惯性等描述：

```xml
<robot name="gazeboteach_2r" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="length1" default="0.6"/>
  <xacro:arg name="preserve_tool" default="true"/>
  <xacro:property name="l1" value="$(arg length1)"/>
  <xacro:include filename="$(find gazeboteach_l23)/urdf/beam_macros.xacro"/>
  <xacro:beam name="arm1" length="${l1}" mass="1.0" color="Blue"/>
  <xacro:if value="$(arg preserve_tool)">
    <gazebo reference="lesson23_tool_fixed">
      <preserveFixedJoint>true</preserveFixedJoint>
    </gazebo>
  </xacro:if>
</robot>
```

beam宏定义在beam_macros.xacro，输入为name、length、mass、color。它生成对应link，在visual/collision/inertial中共同使用length/2，并按该质量和尺寸计算惯性。include负责引入定义，调用宏才产生具体连杆。

`${...}`计算property/表达式，`$(arg ...)`读取xacro输入。它们与roslaunch的arg可以通过入口显式连接，但不是同一个自动共享的参数池；更不等于运行节点的私有ROS参数。展开后，宏调用应变成普通URDF元素，长度与惯性表达式也应成为明确数值。

条件决定是否生成相应XML，不是在运行中把已有模型临时隐藏。本课条件只增删保留扩展，工具结构始终存在；它让你观察同一URDF结构的不同转换结果。修改输入后必须重新展开，并按课程流程重新生成实际实体；只改生成物可能在下一次准备时被源文件重新生成。为避免旧文件掩盖问题，应追溯“源与配置 → 展开结果 → 转换结果 → 实际实体”的指纹和数值。

Noetic语法依据见[xacro官方项目](https://github.com/ros/xacro/tree/noetic-devel)与[该分支的展开实现](https://github.com/ros/xacro/blob/noetic-devel/src/xacro/__init__.py)。宏名、参数作用域或布尔条件报错，应在这一生成阶段处理，不等待Gazebo替你解释未展开的宏。

## 沿实际文件走一遍

| 个人练习中的文件 | 作用 |
| --- | --- |
| arm_settings.json | 保存长度与工具保留输入 |
| ros_ws/src/gazeboteach_l23/urdf/arm.urdf.xacro | 主描述、property、include、宏调用和工具保留条件 |
| 同包urdf/beam_macros.xacro | 盒体连杆的几何/COM/惯性宏 |
| build/arm.urdf | 实际xacro输出的展开结果 |
| build/arm.sdf | 实际gz sdf -p转换结果 |
| build/arm_lab.json | 输入、源、生成物指纹和转换物性 |
| 同包launch/arm.launch | 加载参数、一次性spawn与只读说明节点 |

准备阶段在本课私有Master设好模拟时钟、找到个人源包，再实际调用xacro生成URDF和gz sdf -p生成SDF。随后launch把同一URDF写入/lesson23/robot_description，spawn_model以-urdf模式创建xacro_arm；description_info节点持续报告描述来源，就绪检查再读取实际实体和物性。生成SDF用于核对转换；真正spawn仍走URDF路径。

在操作终端可先读取包与静态launch展开：

```sh
rospack find gazeboteach_l23
bash ros_lab.sh nodes
bash ros_lab.sh params
```

参数预览不创建第二份实体；实际加载来自应用管理的start。robot/@name为gazeboteach_2r，而本次Gazebo实例名由spawn参数指定为xacro_arm；理解各层名称，比只记一个“机器人名字”更可靠。

## Classic转换会改变哪些结构

Classic通过URDF到SDF转换加载机器人。Gazebo扩展使用gazebo元素补充仿真属性；reference指向相应link或joint，而不是任意ROS namespace。最终采用了哪些扩展，要查看实际转换与运行结果。

固定关节默认可能被**lumping（合并）**：把固定子link的质量、质心、惯性、visual和collision转换到父link中，再去掉独立固定关节/刚体。这是减少物理约束的处理，不是简单删除工具质量。因此URDF写了某个link，不保证Gazebo里仍有同名独立link可查询。

若必须保留固定joint，可在该joint的Gazebo扩展中使用preserveFixedJoint=true，并在本机转换输出与实际世界中确认。它与旧disableFixedJointLumping选项不应混称：当前转换实现中，前者保留fixed，旧选项可改成零限位revolute。需要保留的实体仍须有合法质量与惯性。

把底座固定到world，与把整个model设置static，是不同的物理选择。前者可以让内部关节继续运动；后者不能用来代替一台需要转动的动态机械臂。

本课world根link是Classic转换器识别的特殊锚定情况，不套用“所有fixed都与普通父link合并”的规则。普通工具fixed连接则正好用于比较保留与合并。

转换背景可看[Classic URDF教程](https://classic.gazebosim.org/tutorials?tut=ros_urdf)；本机行为还要结合[SDFormat URDF转换实现](https://github.com/gazebosim/sdformat/blob/sdf9/src/parser_urdf.cc)和真实查询。教程里的其他版本或额外控制插件不自动成为本课依赖。

## 先用FK作预测

先在纸上标出每个link原点、关节axis、几何中心和COM，再读展开数值。**前向运动学**就是由关节状态计算各link/末端位姿。本课令实际肩、肘角为q1、q2，工具原点位置满足：

```text
φ1 = 0.2 + q1
φ2 = 0.2 + q1 + 0.15 + q2
x = L1 cos(φ1) + L2 cos(φ2)
y = L1 sin(φ1) + L2 sin(φ2)
z = 0.7 + 0.08 = 0.78 m
```

零位q1=q2=0也不是两杆沿world X排齐；默认工具约在world(0.964,0.256,0.78) m。只把L1增至0.8 m，零位工具应多偏移约(0.196,0.040,0) m。必须把底座偏航与肘的零位偏航都计入，不能套成没有安装偏转的教科书平面图。

只改变一项输入时，先列出哪些派生值应变化，再用真实关节角和link位置比较。工具合并后即使无法再按独立tool link查询，它的几何与质量仍应留在转换后的父link中；也不能把link查询失败直接解释为工具消失。

本课有限关节力矩承担动力学验证：观察正、负肘力矩及接近upper limit时的相对角响应。两关节存在惯性耦合，施力肘关节可能使肩关节同时反动；分析时读取两个实际关节状态，而不是把每段分别当成独立τ/I。控制器集成在后续课程展开。

## 实际操作：先摆姿态，再看动力学

从应用开始实验后，先检查真实模型：

```sh
python3 lab.py ready
python3 lab.py pose --name positive_mixed
python3 lab.py observe --seconds 2
```

positive_mixed在阶段边界将肩、肘设为(π/6,−π/4) rad，negative_mixed为(−.4,.6) rad，zero为(0,0)。pose先暂停、清本课关节任务、设置关节角并清各link速度，再恢复进入时的pause。它用来建立明确初态；之后的运行由物理引擎推进。

用实际q代入上面的FK，检查arm1、arm2及工具的位置。preserve_tool=false时，工具是arm2里的几何与质量；端点可由实际arm2位姿加局部L2偏移重建，并明确这是几何重建，不是独立tool_link状态查询。

再分别从同一零初态施加正、负肘力矩：

```sh
python3 lab.py reset
python3 lab.py pulse --sign 1
python3 lab.py observe --seconds 2
```

reset建立零角度、零速度。pulse通过ApplyJointEffort安排±0.03 N·m，未来0.05模拟秒开始、持续0.2模拟秒。世界暂停时需点“继续”才执行；observe的0.5～5秒是墙钟采集时长，暂停时会读到保持的状态。看完正向实验，再reset后执行`python3 lab.py pulse --sign -1`比较方向，保持每次实验的初态和请求可解释。

effort是广义力，对本课revolute关节单位为N·m。请求返回与真实角运动分开观察；关节限位判断看相对q，不要求肩关节和整台机械臂同时停下。Gazebo中还应确认两段连杆及末端球的外观位置与配置一致。

## 两个重载单变量

**只改第一段长度。** 在默认preserve_tool=true下，先reset观察零位端点，预测L1变长后的几何与惯性，再保存并通过应用“重启实验”重载：

```sh
python3 lab.py config --length1 0.8
```

查看展开URDF和实际ready结果：arm1质量仍1 kg，COM x由.3变.4 m，Izz理论由.0303变为约.0536333 kg·m²，elbow零位位置跟随L1改变。重新做零位与混合角FK核对。最后保存`python3 lab.py config --length1 0.6`并重载恢复基线。

**只改工具保留。** 保持L1=.6，预测合并后的独立link数量、arm2质量与COM，再保存并重载：

```sh
python3 lab.py config --preserve-tool false
```

两种展开URDF仍为五个link和四个joint，包含world入口；实际普通物理link从四个变为三个，tool_link合并到arm2。球仍应可见，arm2质量变为.52 kg、COM x约.2076923 m，整个模型总质量保持2.52 kg。先比生成结构，再比实际物性与几何，最后再观察同样的有限力矩。

完成后保存`python3 lab.py config --preserve-tool true`并重载。配置保存后，准备指纹会要求重新生成与spawn，直接修改build生成物也会触发漂移诊断；每次只改变一个输入，便于解释结果。

## 运行完整验收并读报告

```sh
python3 verify.py
```

验收先核对源/输入/生成物、真实描述参数和实际link物性；保存进入时的关节、各物理link及pause快照后，完成三个静态姿态FK、正负肘力矩、接近上限的力矩响应，再恢复快照。

静态阶段在暂停边界初始化zero、positive_mixed、negative_mixed，读取实际关节角与link位姿，比较FK。动态阶段只在开始时设置角度与零速度，随后由原生有限effort推进，保留带世界/主机时间包夹的真实q/rate；近上限阶段从(0,1.12) rad出发，施加0.12 N·m、持续0.35模拟秒，观察肘相对角接近1.2 rad时的响应。

报告默认位于本次session的urdf-xacro-verify-UUID.json，也可执行`python3 verify.py --report /tmp/lesson23-verify.json`指定路径。重点读static_phases的实际姿态/FK误差、dynamic_phases的请求/样本/summary，以及restore前后状态。曲线或数字应来自实际样本，预测公式用于比较。

清理会暂停、按本课唯一关节名清任务、恢复两个关节角及全部实际link的位姿/线角速度，再恢复进入时的pause。继续运行后，快照状态按动力学演化；原先排队的力矩和已经流逝的模拟时间不重建。教材与图形观察帮助理解空间关系，实际运行与图形检查分别记入验证记录。

恢复还涉及速度的参考点。本课ODE路径的GetLinkState给出link原点速度，而SetLinkState底层设置质心速度；刚体旋转时两者相差ω×r。因此恢复程序读取实际COM偏移，用快照姿态将其转到world，再按v_COM=v_origin+ω×r换点后发送，原始快照保持原样。静止时这一项为零，有角速度且COM偏置时则需要显式处理。

## 诊断与独立挑战

| 现象 | 从哪里查 |
| --- | --- |
| Xacro展开失败 | XML、宏名/参数、property作用域、表达式与条件输入 |
| URDF树无法建立 | 重名、断开的link、找不到parent/child、重复父关系或闭环 |
| 参数存在但Gazebo没有模型 | spawner是否真的调用成功、目标Master/Gazebo与模型名 |
| 关节绕错轴或绕几何中心转 | joint/origin、axis所在坐标系、visual与link原点的区别 |
| 外观正确但碰撞错位 | collision几何、origin和单位；不能只查visual |
| 动力学不稳定 | 正质量、COM、质心惯性、单位和初始碰撞，再查求解设置 |
| 固定工具无法按原link名查到 | 转换后的lumping、Gazebo扩展reference及实际名称 |
| 改了输入但运行模型没变 | 是否重新展开/重载，生成物与实际实体是否来自同一次准备 |
| ROS joint属性说全是revolute、damping为空 | 当前Classic ROS API存在硬编码/未填字段，查转换结果与真实关节行为 |
| 清除关节任务调用了却仍有动作 | 本版clear_joint_forces按未scoped的joint名匹配，且清力不清速度 |

独立挑战：选择一段连杆，解释其joint零位、visual中心和COM的三个位置为何可以不同；再挑一个固定工具，预测合并前后独立link数量、父link总质量与COM怎样变化。用生成文件和实际物性各给一份证据，不用参数字符串存在替代加载与行为检查。

不了解SolidWorks导出URDF？查看[ROS社区导出器项目与教程入口](https://github.com/ros/solidworks_urdf_exporter)。CAD导出后仍应复核link分组、坐标轴、单位、COM和惯性；本课提供简单几何体，不要求先安装CAD。
