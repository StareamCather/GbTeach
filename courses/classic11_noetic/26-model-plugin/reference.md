# 参考：ModelPlugin、力矩积分和生命周期记录

本参考解释默认线性转子项目。它不替换个人文件，也不把解析曲线作为测量输出。项目使用Gazebo Classic 11原生C++接口与Transport，不需要ROS节点或现代Gazebo的ECS接口。

## 从物理模型得到预测

圆柱转子绕对称轴的惯量为 `I=1/2×0.48×0.25²=.015 kg·m²`。默认附加关节阻尼、摩擦和角速度衰减均为零；本课插件明确提交驱动与线性阻力的合力矩。

从转动定律出发：

```text
I × dω/dt = τd - cω
ω∞ = τd/c
T = I/c
ω(t) = ω∞ + [ω(0)-ω∞] × exp(-t/T)
θ(t)-θ(0) = ω∞t + [ω(0)-ω∞]T × [1-exp(-t/T)]
```

基准τd=.03 N·m、c=.015 N·m·s/rad，故ω∞=2 rad/s、T=1 s。角速度从零启动3 s的理想ω约1.9004 rad/s，相对本轮起始角度的增量约4.0996 rad。若只把驱动改为-.03，增量符号反转、时间常数保持；只把c改为.03，ω∞=1、T=.5，3 s理想ω约.9975 rad/s、角增量约2.5012 rad。起始角度不是零时，同一公式仍用于相对位移，不能把预测值当作绝对yaw。

预测成立的前提包括同一初态、固定转轴、正确实际惯量、没有额外阻力或接触，以及合力矩未被接口截断。离散积分、控制回调与消息发布阶段也会造成可解释的差异。先保存真实数据，再用模型解释它，不按公式重写原生姿态或角速度。

## 一次更新中的代码责任

`Load`得到ModelPtr和该plugin的SDF子树，保存模型、查找关节、读取参数、准备只读状态查询及记录主题，连接WorldUpdateBegin。`Init`记录初始化；本课成员保存连接和状态，实例对应当前模型。

每次实际更新按如下顺序进行：

1. 读取本步回调可见的实际joint角速度。
2. 计算线性阻力项与合力矩 `τd-cω`，对目标转轴调用一次SetForce。
3. 更新插件生命期/轮次计数，保存这一阶段的实际读取与力矩。
4. 按记录节奏发布插件JSON，随后物理引擎完成该步的积分和约束处理。

`SetForce`接收力矩，不接收“已经乘过dt的角冲量”。同一步的多次施力可能累加，关节effort上限还会参与截断。本课一次回调只提交一次该轴的合力矩；10 N·m是该接口规格，默认驱动.03 N·m不是始终输出10 N·m。

插件持有的ConnectionPtr控制事件连接寿命。回调捕获对象时，要同时考虑对象销毁和连接解除；成员变量和资源管理不是只为“语法上能编译”。物理回调内也不应无界等待终端输入、网络下载或长时间任务。

## SDF参数、共享库与重载

SDF中的plugin声明位于 `rotor_lab` 模型内部，filename指向准备阶段生成的共享库，子元素提供关节和控制参数。插件名、C++类名、库文件名与模型名各有职责，不要求它们全部使用同一字符串。

注册宏给Gazebo提供ModelPlugin工厂入口。准备阶段使用安装设备的Classic SDK编译，与该SDK的头文件、链接依赖和运行库配套。平台把本课构建路径放入本会话的GAZEBO_PLUGIN_PATH，并保留系统资源路径。

本课直接调用g++及pkg-config参数，插件使用shared/PIC构建，观察工具编译为可执行文件；生成入口分别为build/lib/libgazeboteach_l26_model.so与build/model_observer，记录在build/model_lab.json。缓存身份包含实际C++源码、SDK版本/参数和编译器；prepare同时将JSON中的常规力矩/drag写入个人SDF的drive_torque/linear_drag。代码变更需要重建，普通参数变更则经新SDF在Load中读取。

磁盘C++、刚生成的.so与进程内已加载代码是三个状态。保存源文件后需要重新准备和加载，才能把新代码用于新实例；更改parameters.json也要确认生成SDF和实际加载值一致。当前进程不会因为文本文件变动就自动重新执行Load。

## 两条观测路径及其时间

插件记录是可解释的内部仪表：真实回调中读取的q/ω、计算并提交的力矩、加载参数和计数。只读状态查询返回最近保存的样本，保留它的原时刻与阶段；查询时间不应冒充采样时间。

last_hook/capture_hook及observation_phase区分初始化、原生关节复位后和积分前更新。sim_time_s是样本时刻，epoch_time_s保存本轮起点，已过时间为两者之差。applied_torque_nm保存传给SetForce的计算值，并非另一个F/T传感器；q_rad来自SDK关节位置接口，不应无条件当作已展开的累计圈数。

另一条路径来自Gazebo原生PosesStamped和world_stats。观察工具保留原生姿态/时间与接收信息，再从正确实体和参考系得到转角。当前固定模型的转轴/初态已知，link相对model的yaw可用于本课比较；有移动父级时，需要补上坐标变换。

四元数转yaw后，先按原消息顺序展开包裹角，再以实际时间差求有限差分。区间平均角速度适合对应区间中点，不应无条件标为末端的瞬时速度。若相邻有效采样间隔过大，以至转角可能跨过半圈，就应标明歧义，不能假装总能展开。

WorldUpdateBegin的读数位于物理积分前，原生pose发布可能来自后续阶段；同一时间标签不自动保证相同阶段。数据对齐应保留真实标记和离散误差预算。JSON发布率、GUI pose节奏、世界步长、主机接收频率和墙钟性能也不能合为一个Hz。

## 暂停、步进与复位

暂停通过真实世界控制和world_stats确认；正常物理更新停止后，新的运动记录可能不再产生。只读就绪检查允许显示缓存/延后的动态观察，不解除暂停来制造新消息。

100步操作应只提交一次真实步进请求，等世界时间推进相应步数后再判断完成。以本课实际物理步长核对时间差，同时保留插件update计数；记录主题不必每步都发一包。命令返回成功仍需实际状态确认。

本课使用model-only复位，经过MODEL类型实体的复位路径：模型层pose复位，物理速度和关节受力清理，然后进入ModelPlugin::Reset。它未对所有子link逐一重建相对姿态；在本课固定模型里，转子的内部角度保留，角速度清零。实机曾在约3.1328 rad处用原生pose比较确认该角度保留，应核对这两个读数而非把Reset名称当作零角保证。

插件Reset增加reset_count、重新设定epoch起点并清epoch_updates；生命期updates和Load/Init计数保留。模型复位继续当前世界模拟时钟；完整世界的时间重置属于另一条路径。

暂停状态下处理model-only控制可能消耗一个0.001 s调度迭代，再在正常OnUpdate与物理积分前返回。reset_clock_delta_s记录这个实际时间差，不把调度迭代等同于插件正常更新，也不把模型复位描述为世界时间倒回。单独reset保持暂停；demo和verify另恢复其进入时的暂停策略。

本课观察工具构造WorldReset时，明确设置all=false、time_only=false、model_only=true。Classic这份消息的all字段默认true，仅写model_only=true不足以说明没有请求完整重置；读取原生控制包时应同时检查这些字段。定义见[Classic WorldReset消息](https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/msgs/world_reset.proto)。

`demo`在运动前后都执行真实model-only复位。运动前以当前保留角度和零角速度开始新epoch，运动后保留到达角度、清零角速度，再恢复原暂停状态；原先运行时，真实力矩随后还会使它转动。原生姿态比较使用同一运动区间的角增量，初始角偏置因此消去。它不还原任意中途运动快照，已经推进的时间、累计记录与生命期计数也保留。

平台重载项目会停止旧服务端并创建新模型/插件，默认SDF在新实例中建立零角，Load和Init重新发生；原实例的计数不延续。这里没有添加SetPosition等设置器，也未用完整世界/时间重置替代model-only请求来凑零角结果。第24课的完整仿真时钟重置与当前操作需分别解释。

`torque_command_issued=true`仅表示这条记录对应实际更新中的一次SetForce调用；Load、Init、Reset记录为false，此时力矩字段的0是“未发出本次命令”的占位，并非力矩测量。

## 参考答案的组织方式

基准下初始合力矩为正.03 N·m，转子沿正轴加速；随着ω增大，阻力增大，合力矩减小。接近稳态时驱动力矩与阻力矩各接近.03 N·m、方向相反，净力矩接近零。

负驱动力矩使稳态方向反转；正阻力系数仍抵抗当前速度。增大c同时降低稳态速度和时间常数，因此“更快达到自己稳态的一定比例”与“转得更快”应分开描述。

改为平方阻力时，例如 `τdrag=-kω|ω|`，k的单位、稳态关系和响应曲线均改变。应重新建立预测和判据，再重建插件。默认线性公式与默认自动验收只针对本课基准控制律，不能通过伪造记录把另一种物理模型塞回同一结果。

判断实验完成时，应逐层核对构建和加载、生命周期、真实力矩/转动、参数变式以及暂停/复位行为；单看进程、共享库文件或内部“成功”字段不够。原生pose提供另一条物理观察，GUI方向标记提供实际视觉观察，二者各保留自己的证据。

官方参考：[ModelPlugin API](https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1ModelPlugin.html)、[Classic插件入门](https://classic.gazebosim.org/tutorials?tut=plugins_hello_world&cat=write_plugin)、[Joint API](https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1physics_1_1Joint.html)、[Classic Model源码](https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/Model.cc)、[Classic World源码](https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/World.cc)。
