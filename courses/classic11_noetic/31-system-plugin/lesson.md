# SystemPlugin：在进程启动与真实物理更新之间建立边界

本课给 `gzserver` 加载一个 C++ `SystemPlugin`。它先在进程初始化阶段读配置、登记世界创建事件；等本课世界实际出现后再取得对象；到第一次真实物理更新开始时设置重力。方块随后怎样下落，由 Gazebo 的 ODE 物理引擎计算。

你将用相同方块比较两种重力，并只改变记录间隔观察另一种结果：重力会改变运动，记录间隔改变的是保存数据的节奏。插件内部状态、原生世界重力和原生方块轨迹分别提供证据。本课不需要 ROS。

## 六类插件放在哪一层

前五课已见过模型、世界、传感器、渲染和 Qt 界面的插件。本课补上进程这一层。名称首先说明由谁加载、入口收到什么对象；它不是禁止插件访问其他对象的安全沙箱。

| 类型 | 本课程中的运行位置与加载入口 | Load得到什么 | 后续工作的常见边界 |
| --- | --- | --- | --- |
| ModelPlugin | gzserver；model内的plugin | ModelPtr与插件SDF | 模型初始化、模型复位、世界更新 |
| WorldPlugin | gzserver；world内的plugin | WorldPtr与插件SDF | 世界初始化、完整世界复位、世界更新 |
| SensorPlugin | gzserver；sensor内的plugin | SensorPtr与插件SDF | 传感器实际产生新测量后的更新事件 |
| VisualPlugin | gzclient；visual内的plugin | VisualPtr与插件SDF | 渲染线程的PreRender及Visual生命期 |
| GUIPlugin | gzclient；gui内的plugin | 插件SDF，可为空 | QWidget、Qt信号槽和GUI线程 |
| SystemPlugin | 本课gzserver；命令行-s | argc与argv | 进程初始化、世界创建及显式连接的事件 |

这张表中的“更新事件”不是每个基类自动提供的统一 `Update` 虚函数。各课在 `Load` 等阶段显式保存事件连接，再由相应调度路径调用自己的回调。

ModelPlugin、WorldPlugin、SensorPlugin、VisualPlugin和SystemPlugin接口均声明Init/Reset，但实际调度不同。第28课的传感器软件状态、第29课的视觉命令复位与第27课的WorldPlugin.Reset不能互相套用。GUIPlugin则没有同样的Init/Reset虚接口。VisualPlugin的Init在实际加载路径中还可能重复调用，应按第29课解释。存在一个名为Reset的虚函数，不足以证明世界复位会调用它。

SystemPlugin也可用于其他进程的启动扩展；官方[SystemPlugin教程](https://github.com/osrf/gazebo_tutorials/blob/master/system_plugin/tutorial.md)使用客户端相机说明早期初始化与延后取得对象的思路。该旧教程的加载选项和相机示例不作为本课启动命令。本课只通过gzserver加载，因而直接操作服务端物理。把同一库加载到客户端，不会自动得到服务端的WorldPtr。可先回看第12课的进程与Transport、第26课的共享库，以及第27课的世界对象；接口原文见[Classic 11插件基类](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/common/Plugin.hh)。

## 为什么不用world中的plugin声明

本课的启动命令包含：

```sh
gzserver -u -s libgazeboteach_l31_system.so world.sdf
```

`-u`使世界初始暂停，`-s`要求服务端加载系统插件。平台为本次实验设置私有通信地址及插件搜索路径，实际工作目录是个人练习副本；上面的片段用于理解加载方式，不必另起一个重复服务端。

`world.sdf`故意没有任何plugin声明。将SystemPlugin写进world的plugin位置，会走WorldPlugin的加载入口；即使库路径正确，也不能把两种注册类型互换。准备程序会拒绝这种修改。

Linux的 `.so` 是共享库，`-fPIC`生成适合共享库加载的位置无关代码，`-shared`产生库目标；`GZ_REGISTER_SYSTEM_PLUGIN`导出Gazebo创建这类对象的入口。C++类继承、虚函数签名、注册宏和实际加载器必须匹配。文件名相同不代表库的插件类型或ABI相同。ABI是已编译代码之间的二进制约定，包含符号和对象布局，实际Gazebo SDK及依赖库须兼容。

不了解动态库，可查看[CMake的库目标说明](https://cmake.org/cmake/help/v3.16/command/add_library.html)；本课准备程序实际使用g++与`pkg-config gazebo`构建，并不要求先建立额外CMake项目。官方[插件入门](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)有基类与注册背景，旧教程的软件包名以本机Classic 11 SDK为准。

## Load、Init与世界创建的真实顺序

```text
个人parameters.json、源码与world.sdf
  → 准备配置文件、摘要及真正的.so
  → gzserver解析-s，创建SystemPlugin对象
  → Load(argc, argv)：读取个人配置，连接worldCreated
  → Gazebo初始化并启动Transport
  → SystemPlugin.Init：记录进程初始化；此时本课world仍未创建
  → 解析/加载lesson31世界，产生worldCreated(worldName)
  → 检查名字，取得真实World和falling_cube，建立通信与更新连接
  → 世界其余初始化完成；初始暂停，尚未发生正常物理更新
  → 第一次WorldUpdateBegin：核对实体身份，设置真实重力，读取积分前状态
  → 模型更新、碰撞检测、ODE积分、实际位姿更新
  → 后续每步读取真实状态，按record_every_steps保存记录
```

`Load(int argc, char **argv)`收到的是启动程序的参数数量和字符串数组，不是SDF元素。参数数组中包含命令行选项；不能把它直接当作`gravity_m_s2`的数值。本课只保存数量用于观察，重力和记录间隔来自准备程序生成的 `build/system_plugin.cfg`。若扩展为解析自定义选项，应先定义格式，并考虑宿主命令行解析器是否接受该选项；需要在Load以后使用的字符串应保存自有副本。

官方11.15.1的共享启动代码先调用所有SystemPlugin.Load，再初始化并运行Transport，随后调用Init。因此Load适合检查本地配置、连接早期事件，不适合依赖尚未启动的Transport进行阻塞请求。Init时Transport已经启动，也不等于本课World已经存在。在Load或Init里循环等待世界，可能阻止后续创建世界的代码继续执行。

世界创建事件提供世界名。本课只接受`lesson31`，再查找`falling_cube`并保存实际实体ID；其他世界不会被本课设置重力。更新时还核对相同名字处仍是原ID，目标被删除或替换时报告错误，不把替换对象当作原实验继续操作。进程级能力越早、可访问范围越广，越需要主动限定自己拥有的对象。

`worldCreated`发生在World.Load的末段，仍早于World.Init。它表示世界对象和本课实体已加载，不能自动理解为整个物理系统已初始化完毕。本课把改变重力的边界明确放在第一次真实WorldUpdateBegin；这里只建立连接和状态。以上顺序可对照[共享启动实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/gazebo_shared.cc)及[World实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/World.cc)。

## 暂停就绪不等于已经应用配置

世界`lesson31`有地面`floor`和自由方块`falling_cube`。方块边长0.4 m、质量1 kg，初始质心z=2 m，初速度为0；地面顶面z=0。均匀立方体名义惯量为`ma²/6≈0.0266667 kg·m²`。没有碰到地面时可以使用自由落体模型；平放接触后的质心约为0.2 m。

默认参数为：

```json
{"gravity_m_s2": 9.8, "record_every_steps": 10}
```

`gravity_m_s2`是向下重力加速度的正大小，单位m/s²，接受[1,20]内的有限数值。代码设置的实际向量为`(0,0,-gravity_m_s2)`；质量1 kg不表示其他质量方块会在无阻力自由落体中得到不同加速度。`record_every_steps`接受1..100的整数，是每经过多少次实际更新保存一条本课记录。

`world.sdf`的原始重力固定为`0 0 -9.8`，即使个人参数改为4.9也不改这一项。这样可以检验变化确实由SystemPlugin施加。刚启动且暂停时，预期Load和Init已经发生、世界也已创建，但`updates=0`、`gravity_applied=false`，`last_prephysics`和`last_record`尚为空；此时实际重力仍可能是SDF的-9.8。只有首个真实更新把配置应用后，才应要求实际重力等于个人值。

基准配置的两种重力恰好相同，单看-9.8不能证明插件已执行设置；4.9变式更能区分“世界文件默认值”和“插件应用后的实际值”。不要在ready里偷偷推进世界，只为提前把标志变成true。

## 两种时间与积分前采样

世界固定物理步长`h=0.001`模拟秒。模拟时间描述物理进展；墙钟描述命令等待和机器实际耗时。实时因子小于1时，0.1模拟秒可以用掉更长墙钟，不能用本地等待时长替代原生世界时钟。

World.Step先把模拟时钟与迭代计数推进，再发WorldUpdateBegin，随后进行本步ODE积分。所以从新世界开始，第n次正常更新的回调标签是`n*h`，但回调中读到的位置与速度还只经过前n−1次积分。第一条回调可以是`time=0.001`、`z=2`、`vz=0`，这符合积分前采样语义。

本课的`last_prephysics`每步保留最近实际读数；`last_record`只在本轮更新数能被记录间隔整除时更新。记录包含`phase=world_update_begin_prephysics`、原模拟时间/纳秒、世界迭代数、实体ID、实际z与vz、实际重力。查询时不把旧采样换上新的查询时刻。

原生`~/pose/info`和查询到的实体状态可能处于积分后或另一发布阶段。即使时间标签相同，也不能盲目要求它与积分前z逐位相等。先确定采样阶段，再决定应比较n次还是n−1次积分后的预测。重复时间、接收延迟和丢失消息需要保留，不能排序、改标签或用公式填补缺失测量。

## 先预测，再运行

从z₀=2 m、v₀=0的新世界开始，在首次接触前，连续理想模型为：

```text
v(t) = -g*t
z(t) = 2 - 0.5*g*t²
```

用半隐式欧拉的理想离散式理解本课步长：先更新速度，再以新速度更新位置。

```text
v_k = v_(k-1) - g*h
z_k = z_(k-1) + v_k*h
z_N = 2 - g*h²*N*(N+1)/2
```

| 新世界完成的积分数 | 模拟进展 | g=9.8的离散预测z | g=4.9的离散预测z |
| --- | --- | --- | --- |
| 100 | 0.1 s | 1.95051 m | 1.975255 m |
| 200 | 0.2 s | 1.80302 m | 1.90151 m |
| 500 | 0.5 s | 0.77255 m | 1.386275 m |

这些是预测值，不是预填的测量结果，也不是要求ODE在每位小数上完全相同。连续模型在0.1 s分别预测1.951 m与1.9755 m；离散差异来自积分方法。理想首次触地时间由`2−0.5*g*t²=0.2`估计，9.8时约0.606 s，4.9时约0.857 s；碰撞接触后不能继续外推自由落体式。

特别核对：第100次WorldUpdateBegin保存的是99次积分后的状态，基准理想值z≈1.95149 m、vz≈−0.9702 m/s；完成第100步后，原生实体理想z≈1.95051 m。两者的差异不该靠把某条真实数据改成另一个预测值消除。

第二段步进开始时方块已有速度，不能再次套用“这一段从静止开始”的公式。若重建初态，应执行真实复位或重载，并检查实际初始位姿与速度。

## 开始实验与只读检查

在平台选择本课并开始实验，等待个人副本、真正的SDK构建和服务端就绪。打开Gazebo可以看到橙色方块悬在地面上方，初始暂停。观察画面帮助确认对象；重力、时间和运动的数值仍需原生数据。

在本课终端执行：

```sh
python3 lab.py ready
python3 lab.py predict
```

ready确认个人准备摘要、插件身份、生命周期和原生世界。它不为了得到新记录而解除暂停。predict只输出明确标为预测的数值，不把它们写进Gazebo。确认生命周期历史中load/init的world_available为false、world_created为true，再解释为何暂停时尚无积分前样本。world_available表示本插件是否已绑定World指针；named_world_exists是实际按名字查询世界是否存在的另一项检查，不能只凭自己的空指针证明进程中没有世界。新启动的本课应对照两项及实际顺序。

继续做一次真实100步实验：

```sh
python3 lab.py pause
python3 lab.py step --steps 100
python3 lab.py ready
```

步进工具只发送一次真实请求，等待原生时间和迭代达到目标，再读取实际重力、实体与插件记录。100步应由正常更新计数及0.1模拟秒共同确认，不能把收到100条消息当作100次积分。若已经运行过，这仍是再推进100步，不能当成从z=2重新开始。

查看当前真实消息、运行有限演示以及最终验收：

```sh
python3 lab.py watch --seconds 2
python3 lab.py run --seconds 0.3
python3 verify.py
```

watch按墙钟观察，默认2秒、允许0.1..12秒，不解除暂停；可在另一个终端发有限步进，核对观察文件中的真实pose、world_stats及插件记录。暂停时world_stats仍可能发布，不能称“没有任何新消息”。

run默认推进0.3模拟秒，允许0.001..1秒且须为1 ms的整数倍；它先确认就绪、暂停，再发一次对应步数的请求，完成后保持暂停。step默认100步，允许1..1000步，要求进入时已暂停；1000步在本场景可能已触地，不能再用自由落体公式解释整段。

verify保存进入时的暂停状态，实际控制并复位世界：必要时先核验首次积分，再复位，从初态累计推进200步，验证1/10/100/199/200步边界、积分前采样、保存节奏、暂停冻结和再次完整复位；结束尝试恢复进入时的暂停状态。原来正在运行时，恢复后世界又会继续，最终画面可能已经发生新运动。它不恢复任意入场位姿、速度或旧记录快照。手动恢复连续运动用`python3 lab.py resume`，暂停用`python3 lab.py pause`；`status`是与ready相同的只读状态检查。

## 记录间隔怎样影响观察

在h=1 ms且每次正常物理更新都进入本课回调时，间隔10对应每0.01模拟秒保留一条，名义为每模拟秒100条；间隔20对应0.02模拟秒、每模拟秒50条。这不是每墙钟秒的保证接收条数。

更新仍每步执行：身份核对、必要的重力设置和真实积分前读数不会因间隔20而每20步才执行。只有“把本次读数加入记录序列”的条件变了。首次更新还会额外发布状态，生命周期也可发布状态，所以Transport消息总数不等于record_count。

`updates`、`record_count`累计本插件实例的更新与保存次数；`epoch_updates`、`epoch_record_count`用于当前世界时间轮次。当前轮次从1计数，间隔10在10/20/30等更新保存，不把第1次回调伪装成第10步记录。`recent_records`最多保留128条；`record_count`可能更大，不能把这个有界窗口当作完整长时日志。

记录变稀意味着更难看清短时变化。减少记录开销可能改变墙钟耗时或实时因子，但不会把本课的物理步长或配置重力减半。数据接收又可能有丢包或排队，推断采样率应读取源模拟时间与序号。

## 完整世界复位与SystemPlugin.Reset

```sh
python3 lab.py pause
python3 lab.py reset
python3 lab.py ready
```

本课reset先确认就绪、暂停，再发送一次完整世界复位请求，等待真实模拟时间为0、原生方块质心回到z=2 m，并保持暂停。下一次真实更新的积分前速度读数可继续核对重新释放的初态；reset的完成条件本身不是一次独立速度测量。它不直接调用C++的SystemPlugin.Reset。

在官方11.15.1中，SystemPlugin由进程的`g_plugins`持有；World.Reset遍历的是该世界保存的WorldPlugin集合。本课SystemPlugin虽重写了Reset作诚实计数，完整世界复位不会沿这条路径调用它，因此`reset_count`不应被工具手动加一。

另一条独立连接监听`worldReset`事件。该事件不携带世界名，本课只有确认自己世界的时钟确实从已观察的正时间回退后，才记录`world_reset_event_count`、增加epoch、清理本轮记录并将gravity_applied标为false。若尚未运行或没有可观察的时间回退，不能仅凭全局事件推断发生了本课的一轮重置。下一次真实更新重新确认并应用重力，累计updates与record_count保留。

仅时间复位走另一条原生路径，可能没有worldReset事件。本课还在更新中检测源模拟时间回退来开启新epoch；它不修改原时间戳。这是观察时间连续性的保护，不是默认实验要求你发送的另一种reset。

重载项目会结束本平台的旧服务端并创建新进程，产生新的插件对象，重新经历Load/Init和世界创建。关闭或重开GUI只影响客户端，不会让本课进程级插件重新Load，也不会把服务端世界倒回。生命周期计数、事件计数、世界时钟与GUI开关要分别解释。

## 个人文件与真实构建

| 文件 | 用途 |
| --- | --- |
| parameters.json | 只改重力大小或记录间隔进行对照 |
| system_plugin.cc | 进程入口、事件连接、重力应用和真实采样 |
| world.sdf | 固定原始重力、ODE步长、方块和地面；没有plugin声明 |
| observer.cc | SDK辅助观察工具，读取原生统计、实体、物理配置与自定义记录 |
| prepare.py / config.py | 输入验证、实际编译、摘要与未重载检查 |
| lab.py / verify.py | 有限真实操作与验收 |
| build/system_plugin.cfg | 准备程序生成、供Load读取的个人配置 |
| build/system_lab.json | 源码、配置、世界、SDK、编译器和产物摘要 |
| build/lib/libgazeboteach_l31_system.so | 当前个人实验真正加载的系统插件库入口 |
| build/system_observer | 当前个人实验的原生辅助工具入口 |

修改个人参数或源码后保存并重载项目。磁盘文件、已编译库和当前进程里已加载的代码是三个阶段；保存.cc不会自动替换运行中的对象。摘要不相符时工具拒绝操作，先重载，不删除检查或伪造准备记录。

准备阶段读取实际`pkg-config gazebo`版本与编译/链接参数，使用g++编译；源码、SDK及编译器进入缓存身份，锁与临时产物的原子放置协调同键构建。库不覆盖正在被其他个人进程映射的ELF文件；常规参数来自独立配置，改参数不必假装每次重编同一份C++。源码和运行报告中的摘要可以核对究竟加载了哪一份。

辅助observer沿用第28课的Transport/异步IO安全退出兼容层，只在实际全版本和所需私有ABI符号符合时使用，已验证边界为Classic11.15.1。它属于课程工具维护，不属于SystemPlugin的重力算法，也不要求初学者修改系统库。工具实际消息读写失败仍要报错，不能以强退或吞异常伪装验收通过。

## 两个单变量实验

第一次只把`gravity_m_s2`从9.8改为4.9，保留间隔10，保存并重载。先确认world.sdf仍写-9.8；暂停ready时允许gravity_applied=false；首次步进后再用原生物理重力确认变为-4.9，比较相同实际模拟时刻的真实下落。理想自由落体位移减半，触地时间增为约√2倍；已经接触时不能继续用位移比例判断。

第二次恢复基准重力9.8，只把`record_every_steps`从10改为20，再保存并重载。从新暂停世界执行同样100步，比较原生时间、迭代、方块z、updates与record_count。预期物理仍走100步、重力和轨迹保持同一规律；保存条数由10变为5。这个预期对应没有跨epoch的新初态边界，不应把接收端漏掉的消息数直接当作保存条数。

每次只改一个参数。把重力和间隔同时减半，再凭画面判断“记录稀疏使物体落慢”，无法建立因果。建议保留三组原始输出，并分列个人配置、原生重力、实际时间/迭代、原生位姿、插件保存计数与预测误差。

## 沿源码读懂算法

先定位继承`SystemPlugin`和`GZ_REGISTER_SYSTEM_PLUGIN`，再依次读Load、Init、WorldCreated、Update、WorldReset、Reset与析构。

1. Load检查本地配置，只连接世界创建事件；Init仅记录真实进程初始化。
2. WorldCreated筛选世界名，保存实际方块ID，建立通信和更新连接，不推进物理。
3. Update先筛选世界并核对实体身份，识别时间回退；当前轮次尚未应用时，调用World.SetGravity设置向下向量。
4. 每次真实回调增加更新计数，读取WorldPose、WorldLinearVel和原模拟时间，标记积分前阶段。
5. `epoch_updates % record_every_steps == 0`时保存该条真实样本，再按规则发布状态；物理引擎随后积分。
6. 全局复位事件必须结合本世界时间回退判断；Reset虚函数仅记录SDK是否真调用，析构释放各连接与通信对象。

关键区别是`SetGravity`设置物理作用条件，`WorldPose`读取物理状态。本课没有每步用`2−0.5*g*t²`写回方块z，也没有让记录计数代替真正积分。事件连接存为成员，使连接寿命跟随插件；Transport接收与更新共享的快照用互斥锁保护，查询只拷贝缓存，不在接收线程中抢着推进世界。

## 故障诊断与挑战

| 现象 | 检查顺序与含义 |
| --- | --- |
| 有.so却找不到系统插件 | 当前实验GAZEBO_PLUGIN_PATH、-s参数、实际库依赖与SDK ABI |
| 报插件类型不对 | SystemPlugin基类、注册宏与命令行加载入口是否一致 |
| Load里等待World导致启动卡住 | 后续创建阶段还未执行；改为连接worldCreated，不能阻塞等待自己尚未允许发生的阶段 |
| 初始ready里重力还是-9.8 | 是否尚未首个真实更新；配置4.9并不表示Load已施加重力 |
| 计数增长而方块轨迹错误 | 独立读取原生重力和实体，查初态、实际模拟时间、采样阶段与是否接触 |
| 100步只保存10条或5条 | 正常更新数与记录stride不同；另查是否跨epoch和原消息丢失 |
| 插件z与相同标签原生z有一小步差 | 先看prephysics阶段与ODE积分顺序，不修改原时间戳 |
| 世界复位后reset_count仍为0 | 本课SystemPlugin.Reset未被World.Reset调度；查独立事件和真实时钟回退 |
| 保存新参数却还在用旧效果 | 查看配置/源码摘要，重新加载整个个人服务端实例 |
| 关闭Gazebo窗口后还在下落 | GUI与gzserver是不同进程；读取实际服务端状态 |

挑战一：不改代码，设计一份能区分“只收到一次查询回执”“插件真正应用重力”“ODE实际积分”的三层证据，说明每层应读取什么。

挑战二：在新世界暂停时先步进1次，再步进9次。预测last_prephysics、last_record、gravity_applied和原生方块状态各怎样变化，实际读取后解释“第10步标签”和“已完成10次积分”的差异。

挑战三：若要增加独立的落地判据，先定义质心支撑高度、速度阈值、连续模拟时间窗口、有效样本数和数据缺失处理，再决定算法。只看一帧z接近0.2不能证明已经落稳。扩展实现仍需新的真实验收，默认课程通过不证明扩展通过。

本课验收应区分自动数值检查、真实图形观察和仍未实现的扩展。截图能确认方块、地面与窗口；原生物理重力及带源时间的轨迹才用于验证作用和运动。官方依据另见[服务端参数与初始化](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/Server.cc)、[进程插件持有与退出](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/gazebo.cc)、[ODE重力设置](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODEPhysics.cc)。
