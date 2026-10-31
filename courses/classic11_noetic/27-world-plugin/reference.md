# 参考：世界级投料的状态机、物理证据与复位

本参考解释默认WorldPlugin实现与两项变式，不覆盖你的个人练习。本课以Classic11原生C++接口管理离线方块；内部记录帮助解释操作，原生实体/姿态用于核对实际结果。

## 预测量与实际测量

方块边长a=.16 m、质量m=.5 kg，模型与link原点重合，源码惯量采用均匀立方体公式 `Ixx=Iyy=Izz=ma²/6=.0021333333333333334 kg·m²`。本机SDK9工厂Clone后实际加载为`.00213333 kg·m²`，两者需要分别记录，不能把几何公式值直接标成实测惯量。地板顶面为.1 m，因此无转动平放时质心z约.18 m。

drop_height是绝对质心z，故接触前的理想下降距离为 `h=drop_height-.18`，从静止释放有 `t≈sqrt(2h/9.8)`。默认height1.2时约.456 s，只改成1.8时约.575 s。落稳高度不随初始落高改变，名义投料请求间隔仍为.35模拟秒。

高精度height须分清请求值与工厂可传递值：原JSON和插件配置保留请求，例如1.23456789；metadata的factory_expected_drop_height按`float(format(requested, '.6g'))`得到1.23457。predict分列requested_drop_height_m、sdk9_factory_expected_drop_height_m，以及前者的ideal_requested_free_fall_seconds和后者的free_fall_seconds。model_reset同时报告requested_drop_height/factory_expected_drop_height，按后者核对实际原生z，容差仍为1e-7；不改学生JSON来伪装二者相同。

默认count3时x为-.35、0、.35；只改count5后x为-.7、-.35、0、.35、.7，首末名义请求相隔1.4模拟秒。数量改变也通过居中公式改变位置范围，不能声称所有出生点都保持不变。台面均匀且相邻间隔大于边长，方便分开观察各自落体。

这些数值是预测。请求时模型尚未实际存在，不能把请求时刻直接当作物理释放时刻。原生第一次样本可能已发生下降，应保留原z、源时间和实体ID。接触后的离散约束响应也不再属于自由落体公式。

## 默认状态机

`idle → spawning → complete`描述批次的创建；`clearing → idle`描述回收。complete仅表示目标模型全被实际观察到，实验工具另外读取原生实体位置及连续下降轨迹。单次位置接近支撑高度本身不能证明静止。

当前batch使用每块末.25个模拟秒的原生pose窗口，要求至少8条样本、源时间跨度>=.2 s、z范围<=.003 m、每个相邻有限差分速度绝对值<=.03 m/s、每条z距支撑中心.18 m<=.008 m。通过后暂停再核对实体，结果存于native_stability；全部原始记录仍保留，重复或不递增时间不通过排序、改值来掩盖。该判据证明所观察窗口内的位置稳定，不等于接触力测量或无限时间静止保证。

1. batch命令在空闲且无旧条目时被接受，记录batchRequested；回包的accepted不代替更新完成。
2. 更新处理意图，增加批次号，以当前模拟时间安排第一只；以后达到下一排队时刻才送下一只，每次更新至多一只。
3. 每项先记录请求名字和源时刻。当前`InsertModelString`只提交本地SDF文本到队列，之后查到实际模型才记录ID与首次观察时刻。
4. 模型由Gazebo加载，重力与碰撞交给物理引擎；插件没有逐帧写入预测的z来制造落体。
5. clear取消尚未排出的批次意图，保留已进入工厂的在途条目，等它们实际出现后再回收。
6. 查询不到原对象后才记作移除；所有本批条目完成，才清记录返回idle。

本例在下一次排队时使用实际当前时间+.35，不按过去的理想时间表一次补投多只。实际出现时刻还受到Gazebo工厂处理调度影响，因而排队时间、出现时间与首次观察时间应分别保存。

`InsertModelSDF`接收SDF对象，再序列化为工厂消息；`InsertModelString`直接接收SDF文本，两者都走真实异步工厂。当前本地ostringstream/setprecision(17)仅保留到String入口，避免入口的额外对象序列化；后续World工厂共同执行的Root.Clone仍不能绕过。

实机微探针分别读取直接解析值、GetAsString、Root.Clone和模型Clone：直接解析I=.0021333333333333334，GetAsString为.00213333，Clone实际double为.0021333300000000001；额外高精度质量.5123456789012346也复制成.512346。因此第一次改String入口并未修复SDK精度，不能宣称端到端17位。[SDFormat9.10 Param::Clone/GetAsString](https://github.com/gazebosim/sdformat/blob/sdformat9_9.10.0/src/Param.cc)及[Element::Clone](https://github.com/gazebosim/sdformat/blob/sdformat9_9.10.0/src/Element.cc)解释了默认6有效位文本重建这条路径。

正式检查同时保留数学I、SDK可传递预期`float(format(theory, '.6g'))`与原生三轴实际I，用原`1e-10`容差严格核对可传递值。数学与加载值绝对差约3.333e-9 kg·m²，相对差1.5625e-6，即1.5625 ppm。源码质量.5、边长.16和数学惯量不改，真实落体、支撑稳定及步进阈值不变；未修改全局SDK/精度或原生消息。这个处理记录真实输入转换，仍须读回原生惯量，不能仅凭源字符串正确便宣称实际物性完全相同。

## 身份、作用域与异步边界

准备阶段生成本实例高熵前缀；模型名拼接前缀、批次与索引。插入前冲突会被拒绝，第一次实际查到模型后登记ID。清理只针对登记条目中名字和ID均匹配的对象；若同名处已是另一个ID，保留替换者并报告错误。这里依赖本课独立实例和私有命名约定，不把名字当作永久身份或面对任意恶意并发者的认证机制。

绿色静态external_sentinel是本课之外的对象，清理前后读取其原生实体记录确认未变。通用世界中的其他动态对象会在短暂继续期间自然运动；保持“只删除自有实体”不等于冻结外部物理历史。

只清空C++数组无法撤回已经送出的工厂请求。若在请求仍在途中就报告清理完成，后续可能冒出迟到模型。本课保留待观察条目，直到实际出现并回收，或者明确报错超时；不会把超时伪装成已经完成。

Transport接收与世界更新以互斥锁保护共享状态。接收回调只记录短意图，查询读取最近实际回调的缓存；世界操作在更新中处理。命令nonce与请求ID用于关联响应，有限去重集合减少重复意图；集合有上限且不跨进程重启，不能当作永久事务日志。客户端对非幂等命令发一次后观察结果，不因还没看到结果就盲目重播。

## 生命周期与两种Reset

Load取得当前WorldPtr、插件SDF配置、通信与更新连接，Init记录初始化；它们不因同一实例收到一次完整复位而再次执行。实例中的updates保留生命期计数，epoch_updates在本课Reset里归零，reset_count增加。

原生model-only复位处理模型实体，不进入WorldPlugin::Reset。本课model-reset明确设置all=false，只发一次并保持暂停；检查原生管理刻度完成、插件reset_count不变、方块ID保持和简单自由方块回到SDF初始高度。它们仍然存在，继续运行会再次下落。第26课内部铰链角保留的结论只对应那种结构，不应替代本课实际读数。

完整世界复位使模拟时间回到起点，并调用WorldPlugin::Reset。本课在此只标记待清理，避免在复位持有世界内部锁时重入删除；随后通过真实更新等待回收完成。Gazebo完整复位本身不意味着所有动态创建模型都被删除，这项回收来自本课的自定义策略。

因此reset返回的world_after_reset对应刚发生的原生时间回退，final_world则可能已经因清理而大于零。工具恢复进入该命令时的暂停状态，不倒回清理期间的时间与外部动态状态。平台重载会建立新的服务端与插件实例，Load/Init及实例前缀也重新建立。

官方依据：[World Reset及工厂实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc)、[World接口](https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1World.html)。

## 构建与实际来源

个人JSON经准备程序写入个人world.sdf，同时记录配置摘要和实例前缀。准备程序用本设备Classic11 SDK编译world_plugin.cc及observer.cc，生成共享库和原生观察可执行文件。缓存身份含实际C++、SDK版本/参数与编译器，键内文件锁和原子放置协调并行构建。

`build/world_lab.json`保留源/生成世界/配置/产物摘要；实际Load记录的源码和配置摘要也需相符。保存文件后重新准备和加载，才会使用新配置或代码；磁盘新文本、缓存.so和gzserver内已加载库是三个不同状态。不可仅因某路径有.so就宣称它已经在当前服务端生效。

辅助SDK观察工具的收尾顺序是：主线程在init前持有未连接Connection，保护共享IOManager引用；动态查找私有入口并确认导出库的真实全版本与GAZEBO_VERSION_FULL一致；完成真实消息读写后，transport::fini停/join运行线程并清注册表，主线程再显式调用公开IOManager::Stop停/join异步IO，随后释放node、发布/订阅和保护对象。原始错误仍返回非零，不以_exit、吞异常或延时假装线程已经结束。

这段私有ABI兼容仅限辅助工具，目前在Classic11.15.1验证，符号或运行库版本不匹配则拒绝；WorldPlugin教学类没有使用它。未修改主机SDK，亦不承诺其他版本可直接照搬。兼容路径140次正常查询/控制、3次dead-master错误退出通过。此前有两个真实abort，但GDB100次和原版trace200次未复现；其收尾分析依据上游相同问题的维护者栈和源码，不能声称抓到了本机故障栈。

## 验收应回答的具体问题

| 要回答的问题 | 应看证据 |
| --- | --- |
| 插件是否实际加载这一份代码 | Load/Init、配置/源码摘要及实际库映射 |
| 是否真的出现count只模型 | 原生scene/entity与实际ID，而非仅requested计数 |
| 模型是否真实下落与受支撑 | 对应ID的连续原生pose与实际最终高度 |
| 是否尊重暂停与100步 | 原生world_stats及正常更新计数，100步=.1 s |
| 两种复位是否不同 | 原生时钟/模型实体和WorldPlugin.Reset计数 |
| 是否完成作用域清理 | 原生自有实体消失、外部哨兵记录保持 |

默认batch结束保留方块，方便视觉观察；clear结束回收自有ID。verify会执行真实实验、步进和两种复位，最后清理并恢复进入时的暂停，不还原世界快照。GUI中的颜色、出现顺序、下降和绿色哨兵另作视觉检查，自动数值检查不替代这些学习目标。

若扩展为“上一只落稳才投下一只”，应增加等待支撑的状态与连续时间判据，说明高度/速度容差、采样不足及失败超时，再用真实原生数据验收新流程。默认固定节拍和默认验收不能直接证明这个新算法已经正确。

先修资料：[官方创建模型教程](https://classic.gazebosim.org/tutorials?tut=plugins_world&cat=write_plugin)、[插件类型与注册](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)、[SDFormat世界结构](https://sdformat.org/spec?ver=1.6&elem=world)。
