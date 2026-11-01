# Transport滑门站：收到命令，是否已经执行

## 一条开门消息的三种证据

在终端发出“开到0.6 m”，门还没有动，可能是发错主题、消息被拒绝、物理暂停，也可能只是尚在运动。本课用一块沿轨道滑动的真实门板，追踪命令从客户端到服务端、再到物理世界的全过程。

| 阶段 | 说明什么 | 还不能说明什么 |
| --- | --- | --- |
| received | 服务端校验并接受了这条命令 | 控制目标已经进入物理更新 |
| applied | 某个真实物理更新已采用新目标 | 门板已经到达目标 |
| completed | 实际位置与速度连续满足到位条件 | 任意其它门或真实设备也会有相同性能 |

SDK通信与固定PD控制器由平台提供，不要求先写C++插件或ROS节点。门板质量2 kg，沿+X的prismatic关节运动；q用m、速度用m/s，初始q=0。它通过真实作用力运动，平台不把门板位置或速度写成预定轨迹。开始课程时门保持关闭，开关动作由学员明确发出。

## 消息、主题和序列化

**消息**是一次按约定字段组织的数据，例如命令编号、目标开度；**消息类型**规定这些字段的结构。**主题（topic）**是命名的消息通道，发布者向该主题发送，订阅者在消息到来时收到回调，这就是发布/订阅。**序列化**把结构化消息编码为可传输的字节；终端显示的可读文本是消息的一种表示，不等于网络上直接传输了这段排版文字。不了解这些术语，可看[Protobuf概览](https://protobuf.dev/overview/)和[Classic主题订阅教程](https://classic.gazebosim.org/tutorials?tut=topics_subscribed)。

本课使用Gazebo Classic Transport与gazebo.msgs消息。它与ROS通信、Ignition Transport各有自己的图和接口；本课不需要roscore。不能仅凭命令都叫gz，就照搬其它版本的Twist或服务命令。

## 先找到本课通信图

本课实验终端继承独立GAZEBO_MASTER_URI。Master负责主题名称与连接发现；另一个Master下即便主题名字相同，也属于另一张通信图。使用平台打开的终端检查本课，避免在另一实验环境里解释结果。[Classic架构说明](https://classic.gazebosim.org/tutorials?tut=architecture)介绍了这层关系。

Node使用world=default，因而`~/transport_lab/command`展开为`/gazebo/default/transport_lab/command`；这里的`~`是Gazebo节点命名空间缩写，不是Shell家目录。

```sh
gz topic -l
gz topic -w default -i /gazebo/default/transport_lab/command
gz topic -w default -i /gazebo/default/transport_lab/ack
gz topic -w default -e '~/transport_lab/state' -d 2
```

内省（introspection）是在运行时查看主题、类型与连接信息。应找到command的gazebo.msgs.Request，以及ack/state的gazebo.msgs.Any。command即使只有订阅者，也可被发现；看到主题不代表门已收到某条命令。state按仿真时间约20 Hz发布，暂停后没有新的物理状态样本，不应直接诊断成网络断线。

## 一条命令包含哪些信息

command主题的Request只是所用消息类型的名字；向自定义主题发布它，仍然是发布/订阅。它不会因为叫Request就自动成为一次RPC。

| 字段 | 本课含义 |
| --- | --- |
| id | 本客户端递增的正整数seq |
| request | 动作名set_target |
| dbl_data | 目标q，单位m，关闭为0，命令允许范围[0,0.7] |
| data | 以空格分隔的RUN、CLIENT、NONCE、EPOCH、ATTEMPT_NONCE |

RUN标识当前服务实例，CLIENT区分客户端，NONCE标识逻辑命令，EPOCH在模型复位后更新；ATTEMPT_NONCE则由工具为每次send新生成，用来分辨本次发布尝试的回包。课程工具自动管理这些标记，继续等待会沿用已接受票据而不重发；不要把教材里的字段示意当成可重复使用的真实身份。

Classic命令行发布语法是`-p 主题 -m 'Protobuf文本'`。例如消息正文的结构为：

```text
id: 本次SEQ
request: "set_target"
data: "当前RUN 本客户端 新NONCE 当前EPOCH 新ATTEMPT_NONCE"
dbl_data: 0.6
```

这段是字段示意，实际执行时使用课程工具生成、填好当前标识的完整命令行。`-m`在这里是消息文本，不是消息类型；先内省真实类型，不能往已存在的主题随意注入另一个Protobuf类型。

## 引导实验：查询、开门、关门

个人文件中，parameters.json是本课唯一修改入口，world.sdf保持固定机构；lab.py提供命令行工具，verify.py运行完整通信/物理验收。主“编辑练习文件”按钮默认打开world.sdf，要改参数请使用课程参数编辑命令或`gedit --new-window parameters.json`。

1. 开始课程，等待自动准备工具。运行`python3 lab.py check-config`检查个人输入；运行`python3 lab.py status`取得真正网络请求响应，确认当前RUN/world/epoch与加载指纹。再运行`python3 lab.py entity`，从原生entity_info核对真实关节与门板状态。
2. 在观察终端运行`python3 lab.py watch --seconds 2`，读取有界的真实网络状态。也可使用前面的gz topic命令先查看类型并监听ack；两种显示方式都要区分消息字段与实际物理快照时间。
3. 保持世界运行，在操作终端运行`python3 lab.py open`，查看同一事务的received、applied、completed及到位状态；再运行`python3 lab.py close`，确认真实返回q≈0。单独open若已经到位，可以合法地没有新位移；真实闭→开→闭过程由完整演示与验收检查，不能只看目标数值回显。
4. 运行`python3 lab.py cli-example`，它通过真实查询填入当前RUN/epoch、新nonce、attempt_nonce与seq，打印可复制的Classic gz发布命令。先监听ack，再把该完整命令执行一次，对照实际消息与三个阶段；每次新命令重新生成，旧命令行不能重复充作新事务。
5. `python3 lab.py demo`与“验收实验”都先发送目标0的真实消息建立闭门基线（prepare_closed），再open→close，共三笔事务；初始化不靠模型复位或写位姿。测量期间保持文件与控制输入一致；默认启动本身没有自动移动演示。

## 先订阅，再发命令

打开一个观察终端，先有界订阅ack/state；另一个操作终端发起开门。把三个阶段的同一组RUN/EPOCH/CLIENT/seq/NONCE/attempt_nonce与实际关节状态对应起来。连接成立只说明存在订阅者，发布函数返回也不等于业务接受或物理完成。

服务端收到完整、合法且当前可接受的命令后发received。它可能携带上一个物理步的缓存快照；应看快照的实际iteration和sim_time，不能把收到网络消息当成新推进了一次物理仿真。物理更新采用新目标、原生PD更新后，工具读取本步真实作用力并发applied；这是推进前的证据，尚不等于门已经移动。随后物理步末读取真实q和速度，连续满足条件后才有completed。

本课到位条件为`|q−target|≤0.005 m`且`|速度|≤0.01 m/s`，连续维持至少0.2仿真秒，并在applied之后3仿真秒内完成。仅从画面看门“差不多到了”，或只检查某一点位置误差，都不构成完整证据。到位后控制器继续保持目标。

## 暂停实验：收到与执行分开看

先用平台按钮确认世界暂停，再执行：

```sh
python3 lab.py send --target 0.6 --wait received --ticket ticket.json
```

命令只等接收确认，并把本次事务标识保存在ticket.json。预测是Transport仍可接收消息并返回received，而没有新的物理步时，不应出现本次applied/completed，q和仿真时钟也不会凭消息接收而变化。

随后单步观察applied和真实q变化，或继续让门运动到位；运行`python3 lab.py wait --ticket ticket.json --wait completed`继续等待同一事务，**不重新发布开门**。只等几次单步不一定积足连续0.2 s的到位样本。对照applied的iteration/time、真实q变化与completed的连续到位证据。网络等待按墙钟设截止时间，运动与到位按仿真时间判断；暂停不能靠等更长墙钟替代物理推进。

## 请求响应：问清当前状态

本课另外使用原生`~/request`、`~/response`实现只读快照查询，查询名为gazeboteach_transport_snapshot；课程query工具只允许该名称与固定的安全未知查询探针gazeboteach_transport_missing。响应须匹配请求编号与查询名，结果成功、类型正确、反序列化成功，内部查询nonce也须一致；它不是从本地status.json假装收到网络回包。

可先观察Gazebo已有的真实模型查询：

```sh
gz topic -w default -r entity_info -m transport_gate
```

它与本课自定义快照查询走同一套Classic请求响应机制，返回的数据类型与内容不同。原生CLI有自己的请求等待期限，退出码本身不能证明取得了有效响应。实际操作优先使用课程有界工具，结合回包字段和真实物理状态判断。

ACK/state的外层为gazebo.msgs.Any，type=STRING、string_value中是JSON；它是Gazebo自己的带类型容器，不是google.protobuf.Any。终端echo可能显示转义后的Protobuf字符串，课程读取工具会解开外层类型并解析JSON。不了解schema，可看[本版本Any定义](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/any.proto)与[proto2指南](https://protobuf.dev/programming-guides/proto2/)。

## 单变量挑战与故障诊断

个人parameters.json只有schema和open_target_m。本课仅将open_target_m从0.6改为0.4，其余保持原样；保存、check-config并重载，再用status查询当前实例与配置指纹，重新open、close及验收。预测新的开门终点接近0.4 m，关闭仍回到0；实际过程由力、惯量与阻尼决定，不要求完成时间按目标距离成比例。参考材料只读，不覆盖个人文件。

| 现象 | 先检查什么 |
| --- | --- |
| 开始课程后门不动 | 默认保持关闭，需明确发送开门命令 |
| 有主题却没有received | 本课Master/world/topic、真实订阅连接与消息字段 |
| 有received，没有applied | 世界是否暂停，事务是否仍是当前epoch，物理步是否推进 |
| 有applied，没有completed | 真实q/速度、持续到位时间与控制器日志 |
| 回包看起来正确却被客户端忽略 | RUN/EPOCH/CLIENT/seq/NONCE/attempt_nonce是否属于本次发布尝试 |
| Request能解析但命令被拒绝 | 必需业务字段、动作名、目标范围、旧seq或busy |
| 参数保存后仍是旧终点 | 显式重载并核对实际加载指纹 |
| CLI退出0但没有查询结果 | 是否有匹配且成功解码的真实Response，不能只看退出码 |

独立挑战：为一个超时案例画出“发现连接→收到命令→物理采用→真正到位”的证据链，指出目前只能确认哪一层，再用只读查询或继续等待补证。超时只说明截止时间内未拿到所需证据，不能直接断言从未执行，也不能盲目重复发送改变物理状态的命令。模型复位会取消旧事务、更新epoch并保持关闭目标；旧回包不能完成新事务，复位跳变不算运动轨迹。本协议不承诺服务崩溃后的全局exactly-once执行。

## 原生辅助工具的退出流程

本课通信工具先核对编译SDK与实际Transport动态库均为Classic 11.15.1，并检查兼容层需要的符号；不匹配时在Transport初始化前报错。主线程随后持有一个未连接的Connection，再初始化通信。这个持有者保证异步回调不会成为共享IOManager的最后一个所有者。通信结束或出现异常时，先执行Transport fini，再在主线程调用公开IOManager.Stop并等待IO线程结束，最后释放订阅、发布器、节点和持有者；回调使用的对象一直保留到这两个停止屏障之后。

这段流程只负责独立辅助进程的资源寿命，插件宿主中不执行全局Transport退出。原有命令参数、测量输出与每次控制只发布一次的规则保持；它不写入预测姿态、力或轨迹。编译缓存包含本课变更的C++源码与SDK/链接参数，保存源码后须重新准备或重载。设置`GAZEBOTEACH_IO_DIAGNOSTICS=1`可在标准错误流保留实际IO join证据，不混入实验数据输出。
