# 参考推理：原生记录、不可改写的日志与隔离回放

本参考解释本课的录制与验收算法。数值预测和检查规则不等于本次运行结果；真实通过与否以实际文件、原生消息和真实回放服务端的读回为准。

## 三条证据怎样连接

原始服务端、记录文件和回放服务端提供三个不同观察面：

```text
原始 gzserver：真正执行ODE积分
  ├─ 原生WorldStatistics + 原生实体查询 → 原始检查点
  └─ 原生LogRecord → 原始state.log
                         ├─ XML完整性、字节数、SHA-256
                         └─ 原生LogPlay::Step → raw_sdf → 日志状态
                                                        ↓
独立私有master：真正启动gzserver -u -p state.log
  → 原生LogPlaybackControl定位
  → 原生WorldStatistics + 实体查询 → 回放检查点
```

第一条比较确认日志保存了对应原始运动；第二条比较确认真实回放世界应用了日志状态。只把解码结果与自身复制品比较，不能证明回放服务端存在。只查到某个同名方块，也不能证明它来自这份日志。

课程没有位姿写入回放器，没有把预测公式写回Gazebo；`gzserver -p`是实际运行的原生可执行程序。回放报告应保留进程命令、实际可执行文件身份、独立master、原生回放统计和实体读回。GUI截图提供另一种观察，但不能取代数值比较。

## 原始物理实验的参考值

方块z0=2m，边长0.4m，地面顶z=0；固定步长h=0.001s，向下重力大小g=9.8m/s²。接触前连续模型为`z(t)=2−0.5*g*t²`、`v(t)=−g*t`；半隐式欧拉理想离散模型为：

```text
v_k = v_(k-1) - g*h
z_k = z_(k-1) + v_k*h
z_N = 2 - g*h²*N*(N+1)/2
```

默认record_steps=300时，工具在总步数的四分之一、二分之一、四分之三及终点保存暂停检查点，即累计75、150、225、300步。存在不可整除情况时按工具中的整数计算取目标，不能自己补成小数步。

| 默认累计积分数 | 源模拟时间 | 理想离散z预测 | 理想vz预测 |
| --- | --- | --- | --- |
| 75 | 0.075s | 1.97207m | −0.735m/s |
| 150 | 0.150s | 1.889015m | −1.47m/s |
| 225 | 0.225s | 1.750835m | −2.205m/s |
| 300 | 0.300s | 1.55753m | −2.94m/s |

这些值用于判断量级与运动阶段，不是日志中必须逐位出现的测量。回放默认也不验收速度；速度列只是物理预测，不能因为表中有它就宣称日志实际记录了速度。

本课原始检查点在世界已暂停且步进完成后查询。它不是第31课WorldUpdateBegin回调中的积分前样本。若把第N次积分前状态同第N次积分完成后的实体比较，会差一次积分；分析时区分阶段，不能修改原时间戳来掩盖差异。

## 有限录制算法

沿`Lab.record_native`读源码，可以按以下顺序理解：

1. 校验当前准备摘要，查询真实世界名与本课实体，确认没有其他仍活跃的原生录制路径。
2. 暂停原始世界，发送一次完整复位，等原生时间为0且方块z=2m；保存实际初态检查点。
3. 新建本次UUID目录，向正确master中的原生记录器发送一次start，附绝对base_path与encoding。
4. 等待实际LogStatus给出本次非空full_path；不靠猜测时间戳目录来寻找任意旧文件。
5. 分段发送正常物理步进，每段等原生模拟时间达到目标，再在暂停状态读取实体和统计。
6. 给记录线程有限收尾机会，再发送一次stop；等待后续状态、闭合XML和稳定字节共同满足条件。
7. 先独立解析完整外层XML，再交给原生LogPlay拆出实际SDF记录，保留源文件摘要。
8. 原始检查点T必须真实到达；选择实际日志中最后一个S≤T的状态，比较model/body两层序列化结果，分别保留原始时间与日志时间；确认原始终点达到record_steps对应的进展，最后一次匹配选中日志真实末状态。
9. 成功后写本次归档清单并原子更新最近清单；结束时原始世界保持暂停。

正常或异常收尾都会尝试停止本次已经发起的录制并暂停原始世界。停止反馈可能不确定，不能把该不确定性改写成成功；失败应保留文件与错误。该流程不会恢复进入record前的任意状态，也不把原来的物理运动撤销。

默认300步的总源时间为300×1,000,000ns=300,000,000ns。暂停检查点先读取统计、查询实体、再读取统计；前后均暂停、源模拟时间相同且iterations相同才接受中间实体结果。复位或seek在独立查询之间完成时，第一组读回可能跨过状态转换；这时重新采样只读结果，不能把一次不一致直接解释为原生回放位置错误。

检查点有6墙钟秒总读取预算，单次原生查询也受剩余预算约束。未满足前后统计一致时，在预算内重新做stats→entity→stats，记录`pending_transition`；满足时记录`stable`，返回实际状态。它不重发前面的reset、seek或step，也不改写源时间、迭代与位置；数值比较阈值及T/S保留状态规则不因这条读取流程改变。

每次检查点先创建`build/checkpoint-本次UUID.json`，保留context阶段和master地址，随后在每次原生查询后更新最后统计或实体结果，再保存各轮一致性摘要。context区分完整复位轮询、step前后、record初态/检查点与playback定位等路径。错误写入文件后才抛出，便于区分通信失败、转换未完成与稳定后的数值不符。成功检查点也返回`consistency_attempts`和`diagnostics_file`；它们是读取证据，不是新的物理样本或操作次数。

原生消息可能延迟、重复或缺失。非幂等操作是指重复执行会再改变世界，例如再发100步会多走100步；本课只发送一次，再等待和读取确认。超时不是重发start、reset或step的理由。

## stop完成为什么必须查文件

记录路径包含世界采集线程、编码缓冲和写盘线程。`LogStatus`报告的文件信息不是原始实体状态，`full_path`非空也不单独保证当前录制已经可播放。

本课要求停止后观察到记录路径清空、恰有一份目标state.log、文件以`</gazebo_log>`闭合且可完整解析，并确认实际字节数和SHA-256至少稳定0.2墙钟秒。稳定条件只是收尾证据的一部分；只有一个半截文件不再增长，仍不能通过XML检查。

官方11.15.1的C++ `LogRecord::Stop`通知清理线程并等待join；通过Transport发消息的观察者仍不能将“本地Publish返回”当作远端C++调用已完成，也不能将最先收到的旧LogStatus当成这次停止的最终反馈。材料不把“发送停止”与“完成落盘”混为一个瞬间。[记录器的停止与Cleanup实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/util/LogRecord.cc)是这一区分的依据。

`LogStatus`定义只含sim_time与log_file，log_file包含路径、大小、单位等；没有running或paused布尔字段。该版本PublishLogStatus把记录器RunTime写到sim_time，RunTime由记录器墙钟更新值相减得到。物理时长必须从WorldStatistics或日志state/sim_time读取，不凭字段名字猜测。

空闲状态没有周期性反馈，查询先订阅status并等待control连接，再发送仅有`start:false`的LogControl。OnLogControl的启动条件是`has_start() && start()`，显式false不进入启动分支，也没有stop、paused或资源记录字段，最后只触发实际状态发布。只读探测可以在有限等待内重复；报告保留query_raw_protobuf与query_count。开始/停止录制与有限步进不适用这个重试规则。

不能把该探测改成完全空的protobuf消息：远端消息经SerializeToString后为空，Classic的Connection.EnqueueMsg会直接丢弃空字节。bool字段显式存在且为false仍有protobuf字节，因此能通过Transport，又不执行状态变更。[Connection的空消息处理](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/transport/Connection.cc)与[LogControl处理器](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/util/LogRecord.cc)共同解释这条查询路径；无需先推进一个物理步来让状态查询成功。

`LogPlay::Open`对某些XML缺尾情形会尝试追加根结束标签。课程因此在任何原生读取或回放前先检查完整XML，并在读取前后核对日志哈希。若文件已经被别的工具修补，当前哈希只能识别修补后的字节，不能证明缺失内容已恢复；应保留原始失败证据并重新录制。

## SDF记录、压缩批次与差分的真实含义

原始文件外层为gazebo_log，header写出日志格式版本、Gazebo全版本和随机种子。原生LogPlay提供的Header字符串还可能包含根据状态推导的log_start/log_end；不能据此声称这些时间元素必定原样存在于录制文件的原始header中。inspect中的metadata与原始XML是不同层次的内容。

每个chunk拥有自己的encoding。txt将SDF文本直接放入CDATA；zlib或bz2先压缩文本，再以Base64放入CDATA。Base64是把二进制表示成文本的编码，不是第二次物理采样，也不是加密。改变这项压缩方式不会补回SDF序列化时已经舍去的小数精度。

一块chunk可以装有多条连续`<sdf>…</sdf>`记录。原生LogPlay.Step一次提取一条SDF；其中第一条初始化世界，后续记录提供state。file_bytes、chunk_count、frame_count与native_state_count计量对象不同，不能互相代用。

World.LogWorker确实计算WorldState差，目的是识别状态是否变化以及实体插入/删除。发现变化时，队列里保存的是当前完整的过滤后WorldState，源码还专门说明这样可以避免漏掉缓慢运动。不能把当前版本讲成“第一张关键帧加一串数值增量，需要累加位移才能恢复”。

“完整的过滤后状态”也不等于完整世界定义或全部程序内存。后续state仍依赖最初世界描述来建立实体；序列化可以省略属性。课程没有配置周期性关键帧，不能凭空给出每100帧关键帧或固定delta长度。相关实现见[World状态采集路径](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/World.cc)及[原生日志解码器](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/util/LogPlay.cc)。

## 样本间隔不能由总步数反推

本课没有向record_period设置固定正周期；原生默认period为−1，表示这里不另外以正间隔节流。是否新增状态仍取决于实际状态变化及线程运行边界，实体插入/删除另有处理。因此record_steps=300并不构成“必有300条state”的合同。

正确分析方法是先提取每条实际state/sim_time，保留原顺序和重复，再计算相邻源时间差。若要说明实际覆盖了哪些物理迭代，还应读取各条iterations；下落阶段也可能跳过某些迭代，不能只用“触地后静止”解释全部稀疏。浮点秒便于展示，匹配使用原始秒/纳秒换算的整数，避免显示舍入把两个不同时刻合并。

完整世界描述可以建立t=0的初态，但第一条保存的state可能晚于0。LogPlay的起止范围从实际包含sim_time的内容中读取，不能把原始世界300步的总进展、日志首末状态的时间差与GUI回放范围一概写成同一个0.3秒。源记录的状态数量及间隔均由本次日志确定。

时间线末端也有同样的区别。原始世界必须到达T_final=record_steps×h；如果落稳后没有新状态，日志末时间S_final可以小于T_final。本课不制造末帧，不延长LogPlay元数据范围，也不把保留状态换上新的时间戳。

对每个原始检查点T，算法仅选择日志中最新的S≤T，随后同时检查两条序列化预测：原始T位置的ModelState输出必须等于S记录的model位置，原始T位置的LinkState输出必须等于S记录的body世界位置。不会扫描更早候选去挑一个偶然接近的样本。S=T属于exact，S<T属于retained_state；后一种通过也只证明本次位置及保存精度下的等价，不证明S到T间每个时刻都静止，更不把缺失速度当成零速度测量。

`original_log_matches`中`original_sim_time_ns`为T，`recorded_sim_time_ns`与该条`sim_time_ns`为S，`original_checkpoint_index`指向原始检查点，`retained_interval_ns`为T−S，`match_kind`说明exact或retained_state。清单的`original_final_sim_time_ns`与`last_recorded_sim_time_ns`分别保留物理终点和文件末状态时间；后者不得超过前者。最后一次匹配还必须选中实际文件末状态，并通过两条位置序列化检查。

静止时日志大小增长慢或状态稀疏，不自动说明记录器故障；也不能反过来把每次字节增长说成又进行了一步物理。墙钟录制速度、物理步长、保存状态间隔、写盘批次与屏幕帧率分别测量。

官方旧[日志过滤教程](https://classic.gazebosim.org/tutorials?tut=log_filtering)介绍按属性和频率导出数据。过滤是另一次派生处理；它不恢复原来没保存的测量。本课不用过滤后的文件冒充原始state.log，也不需要运行旧教程中删除整个用户日志目录的清理命令。

## 保存字段与数值容差

11.15.1的ModelState输出先将位置分量按四位小数处理，再以默认浮点格式的`setprecision(3)`写出，即三位有效数字；LinkState先做同样的四位小数处理，再以四位有效数字写出。假设原值为1.23456，第一层得到1.2346，模型字段写成1.23，link字段写成1.235。这里是序列化算法示例，实际测量仍保留原始坐标。

record分别计算明确标为预测的`serialization_prediction_m`与`link_serialization_prediction_m`，各自复现目标SDK的三位/四位有效数字输出，再分别要求它们与实际日志对应位置分量误差不超过1e−9m。原始坐标与日志之间的残差也另外保留，不能将预测坐标冒充原始测量。

日志状态的`position_m`与`pose_xyz_rpy`保留model字段；`canonical_link_position_m`与`canonical_link_pose_xyz_rpy`保留body的世界状态；`effective_model_position_m`表示按本课canonical link语义恢复后的预期模型位置。回放与该有效位置直接比较采用1e−8m，同时与原始位置经过四位有效数字序列化的预测比较。这里没有扩大容差，也没有恢复已丢失的小数精度。

这里的最大位置误差为`max(|dx|,|dy|,|dz|)`，不是三维欧氏距离，也不是所有旋转、速度和传感器数据的综合误差。报告中的误差名必须按实际定义理解。本课是无转动落体场景，默认验收不宣称完整六自由度姿态或动力学状态都通过。

速度在API对象中存在，但默认LinkState.RecordVelocity为false；流式输出的acceleration、wrench和碰撞状态也被省略，ModelState采集中的关节状态路径不能泛化为所有关节均被完整记录。本课不以旧教程的一般能力概述代替当前版本的实际字段检查。[ModelState输出](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ModelState.hh)、[LinkState输出](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/LinkState.hh)与[LinkState默认配置](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/LinkState.cc)可交叉核对。

## 真实回放算法与隔离边界

回放首先验证最近录制清单和日志摘要，然后保存原始暂停世界的检查点。在新目录下创建独立HOME，配置新的私有GAZEBO_MASTER_URI与通信partition，关闭在线模型库入口，并清除继承来的模型/插件搜索路径。启动真实`gzserver --verbose -u -p <绝对路径>`；可选GUI是连接同一私有环境的真实gzclient。

等待服务端可查询以后，再向`~/playback_control`发送原生LogPlaybackControl，定位到各匹配中实际保存的S。原生seek选择已有记录并应用；本课检查实际回放时间严格等于S，然后读取实体。即使原始检查点是较晚的T，也不要求服务端凭空播放到T。不能只看发布请求成功，也不能把请求的target直接当作读回时间。

恢复过程还要沿实体层级读下去：World.SetState调用Model.SetState，后者先设置model世界位姿，再遍历LinkState；Link.SetState随后调用link.SetWorldPose。canonical link的setter会使用该link的初始相对变换反求父模型的世界位姿。因而model字段不是最后一次写入模型世界位置的唯一来源。

本课准备阶段限定只有body一个link，其初始相对位姿为单位变换：平移为零、旋转为零。body也是canonical link，所以恢复完成后的模型xyz等于日志中body的世界xyz。一般模型需要组合变换，不能直接照抄这个等式，也不能把body的世界位姿误当作相对模型位姿。官方[Model.SetState及canonical link选择](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/Model.cc)、[Link.SetState](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/Link.cc)及[Entity.SetWorldPoseCanonicalLink](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/Entity.cc)共同给出了这条路径。

对每个检查点，实际回放时间S、原始检查点时间T、日志中的model/body两层坐标及原始暂停坐标共同进入结果。原始WorldStatistics和日志iterations均保留，可用于独立复核；如果同一日志时间有多条记录，也保留实际候选和序号，不宣称时间唯一。选择最后一个S≤T后仍要求按canonical link语义恢复的位置匹配。原生读回与比较在断言前写入diagnostics.json，失败时也保留T/S、两层坐标与误差，不能只保留一个passed=false。

Classic的World.RunLoop在日志打开后关闭物理积分，进入LogStep。LogStep读取SDF状态、设置源时间和记录迭代，再应用模型状态。它不会再次发送原控制器命令，日志也没有自动恢复控制器历史。[原生回放控制消息](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/log_playback_control.proto)提供暂停、前后步进、seek和播放速度等能力；调整播放速度影响观看的墙钟节奏，不重算原始运动。

完成后再次查询原始世界，要求其源时间与位置保持原样；这说明本次隔离回放没有推进原始暂停世界。关闭回放子进程不结束平台已有的原始世界，也不关闭用户其他Gazebo或ROS进程。

## GUI演示与进程收尾

`playback --gui --seconds 6`在真实显示环境启动gzclient，先给启动有限时间，随后展示检查点并停留。seconds控制最后停留墙钟时间，范围0.5..20；它不是模拟时间，也不是命令总时限。没有DISPLAY时应报告不可用，不能改成无GUI运行后声称看过窗口。

GUI路径在3秒启动等待、各检查点0.5秒展示和最后停留中，以不超过0.1秒的轮询间隔检查所属gzserver与gzclient；每次seek前、原生检查点读回后及清理前也检查一次。检测到提前退出时，先把`gui_liveness`中的checks、last_stage及服务端/客户端returncode写入diagnostics.json，再报错；成功结果也保留检查摘要。这是进程存活证据，不是图像或控件像素验收。有限停留结束后才执行原有的所属进程清理，headless流程不增加GUI依赖。

自动检查使用暂停与定位；手工在它运行期间点击播放可能干扰这些前提。先让自动部分完成，再在最后停留窗口中观察时间线。实际操作了播放、暂停、倒回还是步进，应分别记录；headless verify只证明数值链路。

回放进程继承本次唯一token。工具扫描进程时同时核对当前用户、token和启动时间；发信号前再次核对，避免PID复用或旧PID误伤。退出先发送SIGTERM并有限等待，必要时仅对仍确认所属的进程发送SIGKILL，再检查是否仍有本次后代，写入ownership.json。不能用进程名批量清理，也不能把“父进程消失”当作子进程必定消失。

若命令被正常中断，工具会进入所属进程收尾；任意进程都可能遭遇无法执行清理的外部强制终止，不能宣称系统在任何崩溃情形都自动恢复。诊断时保留本次目录与身份信息，按所有权核验处理，不执行全机清理。

## 文件与来源怎样复核

准备记录build/log_lab.json保存当前输入及产物摘要，build/log_observer指向真实SDK编译产物。构建缓存键包含observer.cc内容、Gazebo版本及编译参数、编译器；个人参数与世界又由准备记录单独绑定。准备成功不是实验成功。

录制归档位于build/recordings/本次UUID下，最近成功入口为build/recording.json，实际state.log可能处于原生生成的嵌套目录，使用清单log_path。清单中的preparation保留录制时准备记录，回放要求它与当前准备记录相同；输入重载后应先重新录制。各检查点在build/checkpoint-本次UUID.json保留阶段与一致性读取过程；每次回放另建build/playback-本次token，保存ownership.json、diagnostics.json、服务端输出及成功时的result.json。可选verify.py --report把整次验收另存为指定JSON。

哈希的作用是回答“文件字节是否仍是同一份”，不是回答“这条物理轨迹是否正确”。源码摘要、SDK版本、世界摘要、参数、实际进程与日志摘要一起构成来源链；需要结合原生测量验证意义。旧目录存在不代表它就是本次运行，最新入口也只在成功录制后更新。

辅助SDK观察工具沿用课程已有的主线程Transport退出兼容封装，在全版本和所需私有ABI符号符合时才使用。它的职责是安全读写原生消息并停止、join异步通信线程；遇到实际错误仍报告失败，不以强退吞掉通信问题。它既不是Gazebo记录器实现，也不模拟物理。

## 单变量实验的参考答案

只把record_steps从300改为200：录制总物理进展变为0.2s，默认四分点为50/100/150/200步。连续预测终点z=1.804m，理想离散参照z=1.80302m；应以实际原生检查点为真值。较早相同时刻的运动规律应保持，文件字节和状态条数按实测说明。

只把encoding从zlib改为txt：同一世界的物理条件与总步数保持；CDATA内容变得可直接阅读，最终字节数通常变化。两次录制包含不同墙钟时间和可能不同chunk边界，因此无需整个文件哈希相同。zlib与bz2对已有文本是无损压缩，不能把压缩方式变化解释为落体轨迹精度变化。

只把record_steps改为700：0.7s可能已经跨过首次接触，约0.606s只是理想接触估计。接触后采用实际高度、连续样本和必要的速度/接触数据判断，不能把某一帧z≈0.2直接称为“已经静止”。默认日志不保存所需全部动力学量，扩展判据需要另行采集与真实验收。

## 复现结论的边界与挑战答案

能解码日志证明文件满足读取要求；真实-p进程及原生log_playback_stats证明服务端进入日志模式；源时间和实体与日志交叉相符才证明目标状态被恢复。这三层互不代替。

state.log可重现已保存的状态，但不是全部仿真内存的检查点。控制程序再运行还依赖完整初态、外部输入、插件状态、物理引擎和资源版本；随机种子不能包办这些条件。状态回放关闭积分，也不能用来检验“同一控制器再次运行能否得到同样轨迹”。

初始SDF中的sensor定义与实际传感器输出不同；RGB像素、点云、扫描、IMU消息和控制话题不随世界状态自动完整录包。若要复现这些数据，应明确保存消息内容、原始时间与坐标系、标定、输入调度和资源依赖，并验证它们与状态时间如何对齐。本课没有提前实现或验证这个扩展。

人工图形观察、自动数值验收与跨环境复现是三种范围。只有实际完成的范围才能写为通过；其余保留为独立练习或未验证项。官方[服务端参数实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/Server.cc)和[记录/回放入门](https://classic.gazebosim.org/tutorials?tut=logging_playback)可以继续对照，但旧教程中的删除命令、示例世界和旧版本字段不应未经核对直接照搬。
