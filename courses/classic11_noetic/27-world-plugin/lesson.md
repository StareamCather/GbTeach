# WorldPlugin：按节拍投料，管理真正出现的模型

本课把 Gazebo 世界变成一个小型投料台：世界插件按模拟时间排队创建几只方块，方块靠真实重力落到台面，最后由插件回收自己登记的实体。绿色圆柱 `external_sentinel` 是外部哨兵，用来检查回收是否误伤了其他对象。

主要问题是：**调用了创建模型的函数，为什么还不能马上宣称模型已经存在？** 你会沿着请求、实际出现、下落、落地和移除五个阶段观察同一批实体。第26课的 ModelPlugin 取得一个模型，本课的 WorldPlugin 取得整个世界，因而还要主动限定自己的管理范围。本课使用 Classic 11 C++ 与原生 Transport，不要求 ROS。

## 世界插件接到什么对象

`WorldPlugin::Load(physics::WorldPtr, sdf::ElementPtr)` 取得世界接口和自己的插件配置。模型查询、模拟时钟和模型创建等操作都从这个世界入口组织。注册宏使用 `GZ_REGISTER_WORLD_PLUGIN`，声明放在 SDF 的 `world` 下面，而不是 `model` 里面。

```xml
<world name="lesson27">
  <!-- 地板、外部哨兵、物理与灯光 -->
  <plugin name="batch_world" filename="libgazeboteach_l27_world.so">
    <count>3</count>
    <drop_height>1.2</drop_height>
    <!-- 准备阶段另外生成本实例前缀与配置摘要 -->
  </plugin>
</world>
```

`batch_world` 是 SDF 中的插件实例名，`BatchWorldPlugin` 是 C++ 类名，`libgazeboteach_l27_world.so` 是共享库文件，`lesson27` 是世界名。它们承担不同职责，不需要写成同一个字符串。声明位置和基类/注册宏应相配；仅把上一课的库换个文件名，不会自动变成世界插件。

| 名称 | 在本课中的作用 |
| --- | --- |
| WorldPtr | 当前实验世界的 C++ 接口 |
| Load / Init | 读取配置、建立通信与更新连接，随后完成初始化 |
| WorldUpdateBegin | 按实际更新处理意图、观察实体、安排插入或回收 |
| InsertModelString / InsertModelSDF | 分别用SDF文本或SDF对象排入世界工厂队列；本课实际使用文本入口 |
| ModelByName / GetId | 在模型实际出现后取得模型与实体身份 |
| Reset | 完整世界复位路径中调用的世界插件回调 |
| ConnectionPtr | 保存更新事件连接，跟随插件实例管理寿命 |

不了解共享库、插件注册或 C++ 成员，可先回看第26课，再看[官方插件入门](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)。世界接口的完整清单见 [Classic 11 World API](https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html)。这里的 WorldPlugin 是 Classic 接口，后续现代 Gazebo 的 ECS 系统插件需要对应课程包，不能直接混用。

## 离线投料台与坐标

世界名为 `lesson27`，重力为 `(0,0,-9.8) m/s²`，物理步长为0.001 s。地板顶面高度为0.1 m；绿色静态哨兵位于 `(1.5,1,0.3)`，不会被投料插件主动移动或回收。

每只动态方块边长0.16 m、质量0.5 kg，初始姿态没有旋转，link 与模型原点重合，因此本例的模型 z 就是方块质心高度。插件用内存中的完整 SDF 字符串定义方块的惯量、碰撞与外观，不下载在线模型。

```json
{"count": 3, "drop_height": 1.2}
```

`count` 接受1至5的整数，`drop_height` 接受0.6至2.0之间的有限数值。**drop_height 是质心在世界坐标中的绝对 z**，并非方块底部离地板的距离。

一批方块沿 X 轴居中排列，Y为0，按下式计算第 i 只的位置，i从0开始：

```text
x_i = [i - (count-1)/2] × 0.35 m
```

默认三只对应 x 为 -0.35、0、0.35 m。位置间隔大于方块边长，便于分别观察落体。数量改变时队列仍居中，因此位置范围也随之改变；落高、单块质量、几何和投料节拍保持原设定。

## 从命令到实际实体的信息流

```text
个人parameters.json与C++源文件
  → 准备/编译真实.so及SDK观察工具，生成本实例的SDF
  → gzserver加载WorldPlugin，执行Load与Init
  → batch请求携带唯一nonce，回调检查是否接受这项意图
  → WorldUpdateBegin按模拟时间生成本地SDF文本，排队InsertModelString
  → Gazebo处理factory队列，加载真实模型
  → 后续更新查到模型，登记名字与实际实体ID
  → 物理引擎积分重力/接触，原生姿态显示下落和支撑
  → clear意图等待在途模型出现，只回收已登记的相同ID
  → 实际查询确认实体消失，队列返回idle
```

这里的factory是Gazebo处理模型创建请求的机制；nonce是一次请求的随机关联标记，用来区分响应。官方 [World.cc 实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc)中，`InsertModelString`和`InsertModelSDF`都将工厂消息加入队列；调用返回不等于模型已加载。本课因此保存 `insert_requested_time_s`，等实际查到模型后才保存 `first_observed_time_s` 与 ID。两者都是仪表记录，第一次观察到的时刻也不必等于模型诞生的精确时刻。

这两个入口都要注意数据精度：`InsertModelSDF`会先把SDF对象重新序列化；当前使用`InsertModelString(text.str())`，只避免这一次额外转换，**不能绕过工厂后续的SDK9 Clone量化**。实机微探针确认，直接解析保留惯量`.0021333333333333334`，Root/模型Clone后则为`.00213333`；高精度质量也有同类现象。[SDFormat9.10的Param实现](https://github.com/gazebosim/sdformat/blob/sdformat9_9.10.0/src/Param.cc)通过默认6有效位的GetAsString重建Clone参数，[Element实现](https://github.com/gazebosim/sdformat/blob/sdformat9_9.10.0/src/Element.cc)又逐项复制，实际工厂流程会经过这一步。

本课均匀方块的数学惯量为`ma²/6=.0021333333333333334 kg·m²`，SDK实际加载值为`.00213333 kg·m²`，绝对差约`3.333×10⁻⁹ kg·m²`，相对差约1.5625 ppm。源码保留数学值，原生查询保留实际值；严格物性检查按本机可传递预期`float(format(theory, '.6g'))`比较，容差仍为`1e-10`，真实落体与稳定窗口阈值不变。本地17位文本不表示端到端17位，课程没有修改全局SDK、全局输出精度或实际消息，也不把序列化后的数值冒充数学原值。

本课名义请求间隔为0.35个模拟秒，每个实际更新至多排一只。工厂处理与消息调度还受主机执行影响，所以实际出现间隔可能不同于0.35 s；世界很慢时，墙钟耗时也会更长。代码不会把丢掉的时间段补写为“每0.35秒已经出现”。

| 状态或记录 | 能说明什么 |
| --- | --- |
| accepted=true | 命令回调接受了意图，尚不等于物理工作完成 |
| requested / pending_names | 已排队的请求与尚未观察到的名字 |
| appeared=true、非零id | 插件实际查到这个模型并登记身份 |
| phase=complete | 本批目标模型均已出现，尚不保证都落稳 |
| 原生pose连续下降 | Gazebo实际发布的对应实体在下落 |
| 原生entity接近支撑高度 | 该时刻的位置符合台面支撑几何 |
| clear后原生scene中消失 | 实体已从实际世界移除 |

`~/world_plugin/state` 的 `Any.STRING` JSON 是本课内部仪表；这里的 `~` 按 Classic Transport 的世界命名空间解析，不是 ROS 私有名称。SDK 工具另外读取 Gazebo 原生 `scene_info`、`entity_info`、`~/pose/info` 和 `~/world_stats`，提供实际实体、姿态与时钟证据。不要只让自己的插件说“我创建了三只”，便把它当作独立验收。

## 先预测自由落体

方块平放并由地板支撑时，质心高度约为：

```text
z_support = 0.1 + 0.16/2 = 0.18 m
Δh = drop_height - 0.18
t_contact ≈ sqrt(2Δh / 9.8)
```

默认质心从1.2 m静止释放，Δh为1.02 m，理想首次接触时间约0.456 s。这个模型描述接触前的自由落体；接触后，离散碰撞和约束求解决定支撑过程。它不承诺每一帧都恰好停在0.180000 m，也不把碰撞后的微小修正解释成持续自由落体。

先运行预测：

```bash
python3 lab.py predict
```

高精度个人落高也会经过SDK9工厂的6有效位转换。例如JSON请求`1.23456789` m时，插件保留这个请求值，但模型预期以`1.23457` m初始化。predict因此分列`requested_drop_height_m`与`sdk9_factory_expected_drop_height_m`，并分别给出`ideal_requested_free_fall_seconds`与按SDK可传递高度计算的`free_fall_seconds`。准备记录保留`factory_expected_drop_height`；不会改写你的JSON以掩盖转换。默认1.2和变式1.8在此转换下数值不变。

预测里不能把 `insert_requested_time_s` 直接当作释放时刻：方块还可能在工厂队列里。用原生实际样本比较时，应保留第一次看到的 z 和源时间；它可能已经下落了若干毫秒。不要修改样本时刻来让曲线对上预期。不了解自由落体，可参考 [OpenStax 大学物理：自由落体](https://openstax.org/books/university-physics-volume-1/pages/3-5-free-fall)，仍按本课的 g 和实际坐标计算。

## 从桌面完成第一批投料

点击“开始实验”，等待平台完成依赖检查、个人副本准备、SDK编译和世界就绪。先确认投料台与绿色哨兵，再运行：

```bash
python3 lab.py ready
python3 lab.py status
python3 lab.py batch
```

`ready` 核对真实世界、Load/Init、实际配置和本实例身份。`batch` 要求队列为空，先准备原生观察，再发送一次新批次请求；它继续真实模拟，直到所有模型实际出现并满足连续原生姿态的支撑稳定判据。轨迹还应显示明显下降，而不是只在最终位置凭空出现。

本课不会仅凭一帧z接近0.18 m就称落稳：对每块的末0.25个模拟秒窗口，要求至少8条样本、实际源时间跨度不少于0.2 s、z范围不超过0.003 m、所有相邻有限差分速度绝对值不超过0.03 m/s，且每条z距0.18 m不超过0.008 m。通过后再暂停并查询原生实体。`native_stability`保留这项证据；它是本课的有限稳定判据，并非任意机构的接触传感器或永远静止的证明。重复消息保留，不排序或改值来拼出稳定窗口。

批次结束会**保留落块供你观察**，并恢复进入该命令时的暂停状态。原先运行则继续运行，原先暂停则停在完成后的实际场景；模拟时间已经前进，不是恢复进入前快照。

若已有一批，再运行batch会拒绝，避免悄悄叠加。先运行：

```bash
python3 lab.py clear
python3 lab.py batch
```

要单独观察消息，可用 `python3 lab.py watch --seconds 2`。这里的seconds是观察进程的墙钟时长；分析落体仍看消息的模拟时间。暂停时可能没有新的运动姿态，不应为拿到新消息而暗中解除暂停。

## 为什么回收要同时记名字和ID

准备程序为本实例生成高熵 `session_prefix`，模型名还包含批次号和索引。插入前若同名模型已存在，插件拒绝占用它；第一次实际观察到模型后，登记对应实体 ID。随机命名减少正常实验中与其他对象冲突，但并不把名字本身变成永久身份。

回收时，名字仍存在而ID不同，说明原对象已消失、当前位置出现了另一个实例。本课保留替换者，并报告身份变化；不因为它叫同一个名字就把它删除。此时应检查实际世界并重新加载干净实验，不把错误抹掉后继续声称原批次全部正确。

`clear` 还要处理在途插入：先停止继续排队，已经送入工厂的请求不能靠清空自己的数组撤回。工具等待这些模型实际出现，再按登记ID回收，直到实际查询确认消失。否则很容易出现“刚清理完，一只迟到方块又冒出来”。

为完成这些更新，`clear` 即使从暂停进入，也会短暂继续真实模拟，结束后恢复原暂停。绿色静态哨兵在本课回收前后应保持同一实体记录。一般世界中的外部动态模型在这段真实时间也会运动，本课不会回滚它们或还原世界时钟；需要原始场景时使用平台“重载项目”。

## 暂停、步进与两种复位

先清楚地暂停和推进：

```bash
python3 lab.py pause
python3 lab.py step --steps 100
python3 lab.py status
```

步进只发送一次真实多步请求，再读回原生世界时间和插件更新数。本课步长0.001 s，100个正常更新对应0.1 s。低频JSON主题不必产生100条消息；记录发布率和物理更新率是不同概念。

已有落块时，比较：

```bash
python3 lab.py model-reset
python3 lab.py status
python3 lab.py reset
python3 lab.py status
```

| 操作 | 世界时钟 | 世界插件 | 本课动态方块 |
| --- | --- | --- | --- |
| model-reset | 沿用当前时间，可能有1ms管理刻度 | 不调用WorldPlugin::Reset | 模型实体复位，简单自由方块回到各自初始高度；ID仍保留 |
| reset | 完整世界复位使时间回到起点，清理阶段随后真实前进 | 同一对象进入Reset，Load/Init不重跑 | 本课Reset提出回收意图，后续更新完成清理 |
| 平台重载项目 | 新服务端建立新时钟 | 新实例重新Load/Init | 重新载入模板世界和实例身份 |

这里的 model-reset 命令保持暂停，按记录中的`factory_expected_drop_height`核对SDK实际可传递初始高度，并同时保留请求落高。方块回到这个高度后若继续运行，会再次真实下落；它们没有被删除。第26课带内部关节的模型有不同的内部姿态行为，不能把那个例子或“复位”一词推广成所有结构完全重建。

Classic 的完整世界复位路径会重置时间和实体，再调用世界插件的Reset；model-only路径只处理模型实体，不调用WorldPlugin::Reset。这个区别来自 [Classic World源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc)。本课的“回收方块”是自己在Reset里规定的策略，并非所有WorldPlugin或Gazebo完整复位默认都会删除动态模型。

本课Reset只增加复位计数、重设当前轮次计数并标记待回收；它不在复位回调里重入模型删除。随后reset工具继续真实更新，等清理结束，再恢复命令进入时的暂停状态。因此最后显示的世界时间可能已经大于零，应分别看 `world_after_reset` 和 `final_world`。

## 文件、构建与修改后的运行流程

| 文件 | 用途 |
| --- | --- |
| parameters.json | 数量与绝对质心落高的常规练习 |
| world.sdf | 地板、外部哨兵、光照、物理与世界插件声明 |
| world_plugin.cc | 生命周期、批次状态机、异步实体管理 |
| observer.cc | 原生Transport线读与控制工具 |
| prepare.py / config.py | 实际SDK构建、输入与来源一致性检查 |
| lab.py / verify.py | 有限实验、观察、对照复位和验收 |
| build/lib/libgazeboteach_l27_world.so | 当前个人副本实际使用的插件入口 |
| build/world_observer / world_lab.json | 观察工具与配置/源/SDK/产物摘要 |

准备阶段把个人JSON写入个人SDF，使用 `g++` 与 `pkg-config gazebo` 的实际Classic11 SDK参数构建。缓存键包含两份C++源码、SDK版本/参数和编译器；每个键用文件锁协调编译，再原子放置产物，避免并行准备替换已经被加载的同一缓存库。

保存参数、保存C++、生成库与进程加载代码是不同阶段。修改后点击“重载项目”，让新实例真正加载；当前运行实例的已加载配置不会因磁盘文本变化而自动更新。运行前会拒绝保存但未重新准备的输入与源码，避免用旧进程解释新文件。

沿 `world_plugin.cc` 阅读算法：

1. Load取得WorldPtr和参数，准备通信并保存事件连接；Init保存实际初始化状态。
2. Transport请求回调验证命令，记录批次或回收意图；返回accepted单独说明是否接受。
3. 更新回调在自己的短状态处理中观察实体、登记ID，按模拟时钟决定是否排下一只；用本地高精度SDF文本经InsertModelString交给工厂。
4. 每个插入请求留下待观察条目；查到实际实体后才能推进出现计数。
5. 回收阶段只处理本批记录；实际对象消失后才标记移除并返回idle。
6. Reset只标记后续工作；析构时解除连接、释放本实例通信资源。

请求接收与世界更新可能处于不同执行上下文，本课用互斥锁保护共享的意图、条目和缓存。只读快照返回最近实际回调的缓存及其原模拟时刻，不把查询时刻写成采样时刻。耗时编译、终端等待和无界网络操作不放进物理回调。

原生辅助工具还有一段设备兼容收尾：主线程在初始化前持有未连接的Connection以保护共享IOManager引用，校验私有符号及其真实运行库全版本与`GAZEBO_VERSION_FULL`相同，再进行正常wire通信；结束时先`transport::fini()`停并join运行线程、清注册表，再由主线程调用公开的`IOManager::Stop()`停并join异步IO，最后释放node、pub/sub与保护对象。它不修改主机SDK，不用强制退出、吞掉非零或睡眠代替join。私有ABI只用于观察工具的兼容层，世界插件教学本体不使用；目前仅Classic11.15.1实机验证，符号/版本不符会拒绝，不能视为其他SDK通用方案。该路径已通过140次正常查询/控制与3次dead-master错误退出检查；此前实际abort的本机调用栈没有被抓到，不能把未复现的GDB/trace运行写成已定位本机栈。

## 两个单变量实验

先恢复默认配置，只把 `count` 改为5，保留drop_height=1.2。保存并重载，预测排队总时长、实际实体数和居中位置。名义首末请求相隔 `(5-1)×.35=1.4` 个模拟秒；这不保证最后一只在1.4 s内就加载、落地。核对原生实体数量与逐只下降轨迹。

再恢复默认，只把 `drop_height` 改为1.8，保留count=3。理想首次接触约0.575 s，支撑高度仍约0.18 m，名义请求节拍不变。用每只实际原生轨迹和源时间比较；不要把从batch开始到返回的墙钟时长直接拿来替代落体时间。

## 错误诊断与独立挑战

| 现象 | 先查什么 |
| --- | --- |
| 编译失败或加载不了.so | 实际Classic11开发SDK、pkg-config输出、完整编译日志和GAZEBO_PLUGIN_PATH |
| accepted=true但没有方块 | 暂停/更新、pending_names、实际原生scene；accepted不保证factory完成 |
| complete后还有方块下降 | complete表示全出现；再看原生z与连续姿态判断支撑 |
| 出现节拍不是每.35墙钟秒 | 本课按模拟时间排请求，实际factory处理是异步的 |
| clear后迟到模型出现 | 是否把在途请求当作已经撤销，是否等实际消失 |
| 同名模型没有被删除 | 对照实体ID；替换者不是原登记对象，按错误提示检查 |
| 模型复位后reset_count不变 | 这是WorldPlugin；model-only不进入其Reset |
| 完整复位后时间已非零 | 分开读刚复位原生时钟和随后清理结束的时钟 |
| 修改文件没有改变运行行为 | 确认个人副本、重载步骤、准备摘要与实际加载参数 |

完成 `python3 verify.py` 或点击“验收实验”，会检查真实投料、下落、暂停/100步和两种复位，最后只清理本课登记实体并恢复进入时暂停。它会真实推进与复位世界，不恢复外部动态历史；不要在需要保留其他动态过程的世界里把它当作无副作用查询。纯视觉目标另看Gazebo：不同颜色的块、先后出现和真实落地、回收时绿色哨兵仍在。

独立挑战：保持默认实验可重现，设计“落稳后才投下一只”的生产线。先定义落稳判据与持续时间，再修改状态机，区分请求、出现、支撑和完成。可用连续原生高度/速度容差配合真实接触证据；若引入接触传感器，回看第14课。写出新判据后再调整测试，不伪造计数或把默认固定节拍的验收结果移植成新算法的证明。

进一步阅读：[官方创建模型教程](https://classic.gazebosim.org/tutorials?tut=plugins_world&cat=write_plugin)、[SDFormat世界结构](https://sdformat.org/spec?ver=1.6&elem=world)、[World API](https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html)。教程中在线model URI与旧构建示例帮助理解接口；本课素材全部在本地，实际构建以当前运行日志为准。
