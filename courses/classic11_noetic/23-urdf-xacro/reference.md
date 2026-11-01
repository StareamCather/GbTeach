# 参考说明：URDF、Xacro与Classic转换

本课针对ROS Noetic/Gazebo Classic 11。模型加载应按生成链逐层检查：个人输入与Xacro → 展开的URDF → 本机转换出的SDF → spawn响应 → 实际实体、物性与关节行为。静态解析、参数服务器写入、运行行为各自证明不同事实。

实际转换采用已安装xacro生成URDF，再用gz sdf -p调用本机URDFdom/SDFormat路径。check_urdf、urdf_to_graphiz等可作为课外工具，但当前设备未安装，不能将其列为本课已运行检查或为此增加下载前提。

已核版本为xacro 1.14.20、ROS urdf包1.13.4、urdfdom 1.0.4和libsdformat9 9.10.1。缺少独立check_urdf工具，不表示缺少底层URDF解析能力；本课实际转换正使用已有解析库。

## URDF的坐标与单位

[urdfdom Joint结构](https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/joint.h)说明parent/child、parent_to_joint_origin_transform与axis；[URDF解析器](https://github.com/ros/urdfdom/blob/1.0.4/urdf_parser/src/joint.cpp)读取origin、axis与limits。axis在joint frame中；joint原点由父link给出，child link坐标系随关节运动。

用列向量表示，转动关节的关系可写为：

```text
ᴾTᶜ(q) = ᴾTᴶ(0) · Rot(axis, q)
R(r,p,y) = Rz(y) · Ry(p) · Rx(r)
```

左式的固定部分包含joint/origin的平移与旋转；axis在J系表达。移动关节改用沿axis的平移。零位表示q=0，并不说明所有link坐标系或几何中心都在world原点。

[Link结构](https://github.com/ros/urdfdom_headers/blob/master/include/urdf_model/link.h)分别保存visual、collision与inertial的origin。joint/origin定位关节；其余origin定位所属link内的几何或质心/惯性坐标。不能用视觉几何的位置偏移代替关节变换。

采用[REP-103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)的SI单位与右手规则：m、kg、s、rad、N、N·m和kg·m²。rpy不是degree；mesh的scale也不自动换算mass/inertia。若CAD输出为kg·mm²，换成kg·m²需乘10⁻⁶。

## 质量、质心和惯性

URDF inertial的origin指定COM位置与惯性坐标朝向，inertia是关于该COM、在该坐标轴表达的矩阵。写入六个分量形成对称矩阵：

```text
I = [ ixx  ixy  ixz
      ixy  iyy  iyz
      ixz  iyz  izz ]
```

本课实体应有正质量、有限COM、合法旋转与物理惯性。惯性正定不等于“对角元素各大于零”；非对角项也影响特征值。物理主惯量还满足任意两项之和不小于第三项。均匀盒体使用尺寸与质量公式，既能得到正定矩阵，也能核对单位量级。

若COM与link原点相差r，将质心惯性平移至另一参考点可用Iₒ=I꜀+m[(rᵀr)E−rrᵀ]，其中两矩阵应先在相同坐标轴表达。URDF已有inertial/origin时，不能把端点惯性当成COM惯性再交给引擎重复偏移。转动惯量随所选点和坐标轴变化，质量则不因换坐标改变。

惯性参数与外观网格独立。固定质量时，长度变化应重新计算COM、惯性与关节位置；固定密度时还要更新质量。完整均匀缩放s、密度不变时质量按s³、惯性按s⁵变化；这不是只在visual里改一个scale就自动发生的行为。

本课固定各link质量，均匀盒体截面0.06×0.06m。理论值示例如下，COM的x相对各自link原点，Izz关于COM：

| 配置/实体 | 质量/kg | COM x/m | 理论Izz/kg·m² |
| --- | --- | --- | --- |
| arm1，L1=.6 | 1 | .3 | .0303 |
| arm1，L1=.8 | 1 | .4 | .0536333333 |
| arm2，工具独立 | .5 | .2 | .0068166667 |
| tool_link，球r=.025 | .02 | 0 | .000005 |
| arm2，合并工具后 | .52 | .2076923077 | .0075908974 |

合并COM为(.5×.2+.02×.4)/.52；合并惯性将盒体与球体各自COM惯性平移至这个共同COM后相加。工具球各轴惯性为2mr²/5，合并后的Ixx为.000305，Iyy与Izz相同。总模型质量仍为2.52kg，变化的是独立刚体的分配。

这些是理论值，运行核对另保留转换物性与API物性。本机路径已观察到部分惯性参数六位有效数字表示，如base Ixx=.00285833、arm2 Iyy=.00681667。理论到SDF的预算需反映具体序列化；运行到实际转换值另作严格检查。原始URDF/SDF/API字段均应保留，不能把理论高精度数字补写成测量结果。

## Noetic Xacro展开

[xacro Noetic实现](https://github.com/ros/xacro/blob/noetic-devel/src/xacro/__init__.py)的eval_all分别处理property、macro、arg、include与if/unless。process_doc接收mappings，将它们作为arg上下文；未提供arg时使用声明默认值。property采用表达式作用域，macro调用展开片段，生成结束后不靠Gazebo解释这些宏。

| 写法 | 意义 |
| --- | --- |
| xacro:arg | 声明生成时输入与默认值 |
| $(arg name) | 读取生成时arg |
| xacro:property | 保存表达式可用的符号 |
| ${expression} | 求值表达式 |
| xacro:macro / xacro:名称 | 定义与调用XML模板 |
| xacro:include | 载入可复用描述 |
| xacro:if / xacro:unless | 条件生成XML结构 |

if/unless有自己的布尔解释规则；不能用普通Python中非空字符串的真假性来推断字符串false的Xacro条件结果。课程采用明确的true/false输入并在展开结果中检查结构是否出现。

ROS launch arg、Xacro arg、property和ROS Parameter Server是不同上下文；只有显式传递才能组成完整输入链。保存配置没有自动改写现存Gazebo实体。应保存源/输入与生成物指纹，验证当前运行对象的来源。

## URDF到SDF及固定关节合并

[Classic URDF说明](https://classic.gazebosim.org/tutorials?tut=ros_urdf)介绍自动转换与gazebo扩展。URDF本体不描述整座世界、灯光或地形，通常由外部场景/生成请求决定放置。本课使用Classic识别的world根与fixed连接接入实际世界，安装高度/偏航由这一锚定约定下的origin给出；不是任意根link名都自动成为全局world。扩展再补充相应仿真属性。

[SDFormat sdf9转换器](https://github.com/gazebosim/sdformat/blob/sdf9/src/parser_urdf.cc)的ReduceInertialToParent、ReduceVisualsToParent、ReduceCollisionsToParent等处理fixed joint reduction。它合并固定子link的内容与物性并做坐标转换；因此父link实际惯性不一定等于原URDF中父link自己的惯性。

preserveFixedJoint=true使对应固定joint保留fixed类型；旧disableFixedJointLumping=true可转为限位零的revolute。不同SDK版本对附加frame等转换细节可能不同，本课应以当前设备输出和实际link/joint名单为准，不跨版本承诺名字和数量。

本课preserve_tool条件仅决定工具fixed joint的preserveFixedJoint扩展是否出现。两种展开URDF均保留工具link/joint、0.02kg质量及同一几何位置；没有删除工具结构。普通fixed合并时，第二段的总质量、COM与惯性应加入工具贡献，visual/collision位置也需保持。world根link由Classic作特殊世界锚定处理，不按普通工具向父link合并的规则计算物性，也不要求给它添加常规动态质量。转换输出中即使保留了同名frame，它表示命名位姿，不自动等于独立物理刚体。

只要求工具与父link刚性连接时，合并通常符合物理意图；需要独立力反馈、传感器引用或名称查询时，则要核对保留策略与接口。保留不能用零质量或缺失inertial的实体硬凑。底座固定world不等于整个model static。

## 加载与行为证据边界

正常链路先在已有私有Master设置/use_sim_time=true，再启动Gazebo/API、加载课程源包与描述、执行一次性spawn，最后读取实际模型。不能把一次性spawner成功退出当作required持续节点故障；是否就绪应由实际实体与状态决定。

robot_description参数存在只证明XML已存储；spawn返回成功只证明该请求路径接纳/完成了相应创建阶段。还需查询实际link/joint、COM/惯性/重力模式、位姿与真实关节反馈。模拟步长与暂停状态也影响有限任务何时执行。

有限关节力矩用于检查动态响应。多刚体运动可用M(q)q̈+C(q,q̇)q̇+Dq̇+g(q)=τ表达惯性、速度耦合、阻尼和重力的关系；本课link禁重力，转动关节声明阻尼.03。几何用前向运动学核对，运动方向和载荷效果读取真实状态，不用每段独立τ/I替代测量。

固定关节本身没有转动自由度；当前API对其position可能返回NaN，本课只对两个moving joints读取有物理意义的q/rate。固定连接则通过转换类型、实际名单与几何/物性核对。

当前Noetic API的get_joint_properties将type硬编码为REVOLUTE，damping字段也未填，不能拿这两个字段验证真实关节类型或阻尼。ApplyJointEffort可接受model::joint的scoped名称，但clear_joint_forces实现按joint->GetName()匹配未scoped名称；本课应使用唯一课程前缀，并在调用处区分两种名字。实际限位响应需观察相对关节角，不把整条机械臂停止运动当作唯一判据。实现依据见[Gazebo ROS API](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp)。

spawn_model的robot_namespace用于相关插件的ROS命名空间，不负责改Gazebo link/joint名称，也不自动添加TF前缀。只读info节点可以持续required，正常一次性spawner不设置required；本课加载机器人描述与生成实体使用已有Master/Gazebo。

真实查询中的ROS header与接收时间仍保留各自来源，不改称native捕获stamp；对无header接口也不补造采样时刻。所有运行/图形结论应由本课实际验证记录给出，描述文件能读通不等于实验已通过。

## 公共操作与完整验收

| 命令 | 行为 |
| --- | --- |
| python3 lab.py ready | 核描述来源、生成物、实际模型与link物性，读两个moving joints |
| python3 lab.py pose --name positive_mixed | 在暂停边界初始化(π/6,−π/4)，清速度并恢复原pause |
| python3 lab.py pose --name negative_mixed | 同上，使用(−.4,.6) rad |
| python3 lab.py reset | 清本课关节任务、零角/零速，保留进入时pause |
| python3 lab.py pulse --sign 1 | 肘关节+.03N·m，50ms lead、200ms duration；−1使用负力矩 |
| python3 lab.py observe --seconds 2 | 只读实际q/rate，支持0.5～5墙钟秒 |
| python3 lab.py config --length1 0.8 | 保存长度，支持.6/.8，需重载 |
| python3 lab.py config --preserve-tool false | 保存工具保留布尔量，需重载 |
| python3 verify.py --report PATH | 保存三静态/三动态及恢复的完整原始报告 |

pose/reset/pulse在操作前检查本课两个joint basename未被另一模型复用，保证当前SDK按basename清任务时的归属。观察命令与ready可用于暂停状态；脉冲按模拟时间执行。课程每次从明确初态分次施力，CLI不提供跨进程的自动互斥承诺。

verify按以下真实流程工作：

1. 检查已加载输入/源与URDF/SDF指纹，读取/lesson23/robot_description的实际SHA及说明节点身份；查询非static模型、实际物理link名单、质量、COM、惯性和gravity_mode。
2. 保存进入时pause，暂停后拍下两个moving joints与全部物理link的真实快照。三个静态阶段分别初始化zero、positive_mixed、negative_mixed，实际q与请求误差小于10⁻⁶rad，FK位置与姿态角误差均小于10⁻⁵。
3. motion_positive与motion_negative分别从零角零速出发，有限肘力矩±.03N·m/0.2模拟秒。每段至少十次真实服务观测，并读取到end后0.1模拟秒；有符号位移至少.01rad、峰值有符号角速度至少.05rad/s，肩耦合运动保留原样。
4. upper_limit从q=(0,1.12) rad出发，+.12N·m/0.35模拟秒。实际最大肘角须在[1.17,1.22]rad；end之前最后60ms仍受力窗口内，至少三次样本的平均上限误差≤.02rad、最大|dq|≤.2rad/s。限位按相对角判断。
5. finally中逐项尝试暂停、清两关节任务、恢复关节角、按下述速度换点恢复每个实际link的pose/twist，并读回核对。关节q/rate误差小于10⁻⁵，link位置/线角速度分量与姿态角误差小于10⁻⁶；q与−q四元数按相同姿态处理。最后恢复原pause并记录每步清理错误。

本课采用world参考系时，[Noetic API的getLinkState/setLinkState](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp)分别调用WorldLinearVel与SetLinearVel。[Classic 11.15.1 ODELink](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODELink.cc)中，前者以link原点相对COM的偏移调用dBodyGetRelPointVel，后者直接调用dBodySetLinearVel，设置的是ODE质心速度。恢复不能简单将原点线速度原样回填。

```text
r_world = R_world_link · c_link
v_COM_world = v_origin_world + ω_world × r_world
```

c_link取实际GetLinkProperties读回的COM位置，R_world_link取待恢复快照的link姿态，ω_world也取该快照。程序仅将发往设置接口的线速度换到COM；保存的GetLinkState原始pose/twist不改，恢复后仍以原点速度读回比较。这样既处理非零角速度与质心偏置，也保留原恢复阈值及数据语义。

实际物性对转换SDF的比较使用mass10⁻⁸kg、COM10⁻⁸m、惯性分量10⁻⁹kg·m²等明确阈值；理论到转换的序列化预算单独处理。ready保留reported_type/reported_damping作为原API字段，不用它们证明真实类型或阻尼。

报告默认session/urdf-xacro-verify-UUID.json，--report覆盖位置；trace保存静态快照和带世界/主机包夹的动态joint_state_truth，static_phases/dynamic_phases保存阶段选择与统计，restore保存原/后快照和pause。GetJointProperties没有native header，包夹描述查询时段。恢复不重建旧力矩队列或回退时钟；恢复后继续运动属于真实物理演化。

### 真实观测的连接与时序

`Client.sample()`的顺序保持为GetWorldProperties、肩GetJointProperties、肘GetJointProperties、GetWorldProperties；两个关节读数位于首尾世界时间查询之间。报告保留整个查询时段，原有中点仅用于窗口选择，不是补造的关节捕获时间。四次服务调用、原有采集间隔、有限effort和全部验收阈值均保持不变。

高频只读路径按Client实例缓存这两个服务的`ServiceProxy(persistent=True)`。首次连接才调用wait_for_service；同一实例的只读调用由互斥锁串行执行，各次请求仍实际访问服务端。其余服务继续通过原common.call调用，包括ApplyJointEffort、清力、暂停和姿态/速度恢复；本次优化不给这些写入复用连接或增加自动重发。

单次只读调用的4墙钟秒预算包括等待互斥锁、首次服务发现和等待返回。读取报错、响应拒绝或超时会记录失败、移除并关闭全部读连接，此Client之后的读取直接报错；超时的后台调用即使稍后结束，也不会交出有效样本或重新放入缓存。Client.close会断开读连接并注销订阅。写入仍可逐项尝试恢复，但连接失败后的真实恢复读回可能无法完成，必须保留该诊断，不能宣称恢复已验证。

这一改动降低TCPROS建连开销，不提供固定采样频率承诺。最后60ms仍受力窗口中的三次样本必须真实取得；GUI负载、服务端处理或传输延迟仍可能使观测不足。发生失败应检查报告中的世界时间包夹和墙钟查询耗时，而不是用插值补齐、扩大窗口或改动动力学。

## CAD导出先修入口

[ROS社区SolidWorks URDF Exporter](https://github.com/ros/solidworks_urdf_exporter)提供项目与教程入口。CAD装配关系不会自动保证导出的关节轴、树、COM、单位和惯性正确；复核这些字段后再进入仿真。本课直接使用基本几何，CAD导出与大网格处理不属于运行前提。
