# 参考推理：进程生命周期、重力变更与采样相位

本参考解释本课SystemPlugin的默认算法与两项单变量实验。下面的数值属于预测和应检查的条件；实际通过与否须读取这次设备运行的原始结果，不能从公式或编译成功推导出来。

## 从进程到世界的阶段

SystemPlugin的Load签名是`Load(int argc, char **argv)`，不是`Load(WorldPtr, sdf::ElementPtr)`。本课通过`gzserver -u -s libgazeboteach_l31_system.so world.sdf`加载；world.sdf没有plugin声明。argc/argv来自宿主程序启动参数，本课保存argument_count，个人数值由准备程序写入build/system_plugin.cfg。

官方11.15.1的Server.ParseArgs保存参数，-s进入SystemPlugin创建路径；gazebo_shared.setup依次调用Load、初始化Transport、启动Transport线程，再调用Init。Server随后加载世界。因而本课Load/Init历史中的world_available=false是正常阶段，不是漏载世界。该字段只表示本插件尚未绑定指针；named_world_exists通过实际按名查询独立说明世界是否存在，两者不能混成一项空指针推断。

World.Load先创建实体，再发worldCreated；World.Init随后初始化实体和物理引擎。回调拿到WorldPtr不表示世界已完成所有初始化。本课世界创建回调只筛选lesson31、取得falling_cube和真实ID、建立通信/更新连接；把物理变更放到首个正常WorldUpdateBegin。

这里不应声称“ODEPhysics.Init必会覆盖早期重力”作为理由：本目标11.15.1的ODEPhysics.Init是空函数。选择首个真实更新是本课明确的应用时机，能与暂停就绪及实际积分建立可检验的边界；不要用未经源码支持的初始化故事解释正确设计。

## 六类插件不能共用一套生命周期假设

ModelPlugin入口得到Model，WorldPlugin入口得到World，SensorPlugin入口得到Sensor，VisualPlugin入口得到Visual，GUIPlugin创建QWidget；SystemPlugin入口只收到启动参数。前四类相应SDF位置、GUI的位置与本课-s加载也不同。

SystemPlugin由进程持有，WorldPlugin由World保存。SensorPlugin的Reset虚函数存在，但SensorManager的时钟复位不等于该插件的软件历史必被清除；第28课依赖真实测量时间回退。VisualPlugin的Init可能在实际加载路径中重复；第29课课程reset又是恢复视觉参数的自定义命令。GUIPlugin没有可照抄的Init/Reset override。接口声明、宿主是否调用、课程自定义操作必须分别查证。

这些类型限定入口与通常职责，不构成C++访问隔离。即使SystemPlugin能查找多个World，本课只修改lesson31；实体同名但ID变化时拒绝继续操作替换者。析构释放事件连接和通信对象，连接不能仅靠短命局部变量维持。

## 应用重力与记录是两条逻辑

第一次真实更新执行`world->SetGravity(Vector3d(0,0,-gravity_m_s2))`，然后读取实际物理状态。后续每步仍读取真实状态，按本轮更新数对stride取模，满足条件才保存。

```text
校验本次worldName与目标实体ID
→ 如源时间回退则开启新epoch
→ 如本epoch尚未应用重力则设置实际World重力
→ 累加updates与epoch_updates
→ 读取实际WorldPose、WorldLinearVel、模拟时间及实际重力
→ 保存last_prephysics
→ epoch_updates可被stride整除时保存last_record并增加record_count
→ 更新缓存/必要发布
→ 返回Gazebo，让物理引擎完成本步
```

改变gravity_m_s2会改变实际物理条件；改变record_every_steps只改变保存节奏。配置g使用正大小，实际gravity_z应为−g。SDF重力原文一直是−9.8，4.9变式在首更新以后才可用真实物理读回证明应用。

初始暂停ready不推进世界，故load/init/world_created可已完成，updates仍为0，gravity_applied为false，last_prephysics和last_record为空。基准g=9.8时SDF值与目标相同，单看实际−9.8不足以证明SetGravity调用已经发生；必须结合应用标志、更新边界及4.9对照。

## 时间标签为何领先积分前状态一小步

World.Step在进入World.Update前做`simTime += h`和`iterations++`；World.Update先发WorldUpdateBegin，再更新模型、检测碰撞并调用ODE积分。因此标签与状态阶段应同时保留：

| 时刻 | 新世界的理想含义 |
| --- | --- |
| 初始暂停 | simTime=0；z=2；v=0；无正常回调 |
| 第1个WorldUpdateBegin | 标签h；状态尚未积分；应用重力并读取z=2、v=0 |
| 第1次积分完成 | 标签h；状态已含1步积分 |
| 第n个WorldUpdateBegin | 标签n*h；读取前n−1步积分后的状态 |
| 第n次积分完成 | 标签n*h；状态已含n步积分 |

用`phase=world_update_begin_prephysics`明确这个区别。不能把原始sim_time_s减去h后覆写为另一时间；分析时另计算用于模型比较的已积分时长。原生pose、实体查询与world_stats也需要按各自采样/发布阶段解释，相同标签不是同一阶段的保证。

源时间不单调时保留epoch和回退原值；重复消息也保留，不通过排序或补值制造连续。接收单调时间描述主机收到数据的时刻，不能代替物理模拟时间。

## 自由落体的预测答案

质心z0=2 m，方块边长0.4 m，地面顶0。无转动平放的支撑质心约0.2 m，接触前的下降距离上限为1.8 m。

连续模型：`v(t)=−g*t`，`z(t)=2−0.5*g*t²`。半隐式欧拉理想模型：先`v_k=v_(k−1)−g*h`，再`z_k=z_(k−1)+v_k*h`，从而`z_N=2−g*h²*N*(N+1)/2`。h=0.001 s时：

| 完成积分数N | g=9.8：z / vz | g=4.9：z / vz |
| --- | --- | --- |
| 100 | 1.95051 m / −0.98 m/s | 1.975255 m / −0.49 m/s |
| 200 | 1.80302 m / −1.96 m/s | 1.90151 m / −0.98 m/s |
| 500 | 0.77255 m / −4.9 m/s | 1.386275 m / −2.45 m/s |

基准第100次积分前回调的理想值则取N=99：z=1.95149 m，vz=−0.9702 m/s。间隔10时首条last_record在第10次回调，标签0.01 s，对应前9次积分，理想z=1.999559 m、vz=−0.0882 m/s；它不是第10次积分完成后的z=1.999461 m。

只改g为4.9时，同一未接触模拟时刻的理想下降距离与速度大小均减半。接触前时间由sqrt(2*1.8/g)估计，9.8时约0.606 s，4.9时约0.857 s，比例√2。这里给的是连续接触时刻估计，真实接触还受离散步长与接触求解影响。

位姿不能从这些公式生成后假冒Gazebo测量。真实ODE离散误差、接收与采样相位需要单独检查，触地后自由落体式失效；只有一帧在0.2 m附近不证明静止。原始测量、预测与残差要分列保存。

## 间隔10与20的答案

从没有正常更新的新epoch开始，实际走100次正常更新：stride10应保存第10/20/…/100次，共10条；stride20保存第20/40/…/100次，共5条。两者updates都增加100，实际物理时间都前进0.1 s。

名义记录周期分别是0.01与0.02模拟秒，不能直接叫作每墙钟秒100/50条。首次更新还会发布状态，生命周期和查询也有通信，所以收到的JSON数量不等于保存次数；网络观察又可能丢掉部分发布。

updates/record_count累计插件生命期，epoch_updates/epoch_record_count用于当前时间轮次。last_prephysics保存每步最近样本，last_record保持最近一次按stride保存的样本；查询的query_nonce只关联请求，不产生新物理测量。recent_records最多128条，长期运行的累计保存次数可大于窗口长度。

改变stride不改变h、g或积分次数；减少日志开销可能影响墙钟实时因子，却不能据此推导物理下落变慢。对照应比较同一实际模拟时间及相位的原生状态。

## 世界复位不会自动调用本课SystemPlugin.Reset

官方gazebo.cc使用进程级`std::vector<SystemPluginPtr> g_plugins`持有系统插件，退出时清理。World.Reset遍历本World的插件集合，集合中的对象是WorldPlugin；它没有遍历g_plugins。因此本课完整世界复位后SystemPlugin.reset_count应保持真实值0，不能通过手调Reset让它看起来符合别类插件的流程。

World.Reset会重置世界时间和实体，重置物理引擎，再发worldReset事件。事件本身没有world参数，本课回调还确认自己保存的世界时钟发生了回退，再增加world_reset_event_count、开启epoch、清理当前轮次记录并将gravity_applied设为false。事件计数与Reset虚函数计数是两类证据。

若从未观察到正的源时间，或者事件没有伴随自己World时间回退，该回调不会把全局事件强行归属于本课。因此不要要求在任意t=0操作后事件计数都必须增加。默认复位实验应先推进真实物理，再检查原生时间回退、方块初态、事件与Reset计数。

ResetTime路径本身产生timeReset事件，不等于worldReset；更新中的源时间回退保护可以开启新epoch。复位以后下一正常更新再应用配置，不修改旧记录时间以维持全局单调；累计updates/record_count保留。

关闭GUI不重建SystemPlugin；重载项目创建新gzserver及新对象，才重新Load/Init。完整复位也不重新编译库。复位、重载、保存源文件、重开GUI是四个不同操作。

## 原生证据应该怎样组织

| 要回答的问题 | 关键证据 |
| --- | --- |
| 加载的是SystemPlugin吗 | -s启动参数、注册类型、真实进程身份与Load/Init历史 |
| 当前个人配置是否已加载 | config/source摘要、准备记录与插件读回 |
| 是否真正应用4.9 | 首更新前后标志、独立原生physics重力读回、SDF仍为−9.8 |
| 方块是否真的下落 | 对应实体ID的原生位姿/速度与源时间，结合未接触区间 |
| 是否真推进100步 | 原生world_stats时间/迭代与正常更新计数差 |
| stride是否只改变记录 | 两组相同物理进展与原生轨迹，保存次数/源时间间隔不同 |
| 世界复位是否影响软件 | 实际时钟回退、原生初态、epoch、worldReset事件与Reset虚函数计数 |

插件JSON说明它读取了什么和做了什么，不能独立证明其作用；原生世界物理配置与实体数据提供另一条路径。截图另用于检查真实窗口、对象、暂停和下落的可见现象，不替代数值验收。命令成功退出、编译成功与实验目标达成也各不相同。

工具只发送一次非幂等步进请求，再读取实际边界。超时可能意味着反馈晚到，不能通过重发补成期望步数。run先确认就绪并暂停，按1ms整数步推进指定0.001..1模拟秒，结束保持暂停；step要求已暂停，允许1..1000次更新。watch按0.1..12墙钟秒旁观，不推进世界。reset先暂停，发一次完整复位，等原生时间0和z=2，保持暂停；其完成判据不包括独立速度读数，下一次积分前样本才用于核对速度初态。

verify实际复位并分段走1/9/90/99/1步，累计200；199步完成后的原生实体位姿还需与第200回调的积分前z交叉相等核对。随后检查暂停冻结并再复位，最后尝试恢复入场暂停状态。原来运行时会继续新一轮物理，不保证最终截图静止在z=2。已经发生的物理进展和记录历史都不回填，命令不是任意入场快照的还原器。

## 构建与辅助SDK工具边界

准备阶段校验有限参数、world身份及无SDF plugin，生成个人cfg，用实际g++和Gazebo Classic 11 pkg-config参数编译.so与observer。缓存键包含实际C++源码、SDK版本/参数与编译器，文件锁协调同键构建，以临时文件原子放置产物，个人build入口指向缓存。

准备记录保存参数、配置/世界/源码/产物摘要；插件Load记录源与配置摘要。保存文件而未重载时，prepared检查应拒绝用新文件解释旧实例。不能仅看build目录存在.so便说当前进程已经使用这份源码，也不覆盖运行进程正在映射的ELF产物。

本课observer沿用第28课的辅助Transport退出兼容：主线程保护未连接Connection的IO引用，验证动态私有入口所属库的真实全版本；真实消息读写结束后先transport.fini停/join通信线程，调用公开IOManager.Stop结束异步IO，再释放通信及保护对象。实际错误仍非零返回，不使用_exit或吞异常伪装正常完成。

这条私有ABI维护路径仅限已验证Classic11.15.1，版本/符号不符要拒绝。SystemPlugin教学算法不依赖初学者修改这层，也没有修改主机SDK。它解决辅助进程的资源寿命问题，不生成落体数据，不替物理验收。

## 独立挑战的思路

“只读查询成功”说明通道有回应；“重力已应用”需首更新与实际physics读回；“实际积分”还需原生时间、迭代及实体状态。三者不能用同一内部布尔字段互相证明。

先1步再9步的挑战中，首步后gravity_applied应为true、last_prephysics非空，stride10的last_record仍为空；累计10个正常更新后出现首条记录。记录仍是第10回调的积分前状态，真实步进完成后的实体已多完成本步积分。

落地判据至少需要接触高度附近的连续窗口、小速度、足够有效样本以及明确超时规则；如果只有模型位姿而无速度，可用源时间有限差分，但要说明噪声与缺测。默认课程没有实现你的扩展规则，需独立真实验证。

官方源码依据：[Plugin.hh](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/common/Plugin.hh)、[Server.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/Server.cc)、[gazebo_shared.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/gazebo_shared.cc)、[gazebo.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/gazebo.cc)、[World.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/World.cc)、[ODEPhysics.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODEPhysics.cc)。所有接口按本课Classic 11目标理解，不混用现代Gazebo的ECS系统接口。
