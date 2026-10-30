# 参考：Classic Transport事务与真实物理证据

本页只读，解释本课消息、状态与验收流程，不覆盖个人参数或场景。理论判断、通信证据和真实物理观测分别取得，发布成功不代替滑门到位。

## 固定机构与允许修改的输入

门板为居中均匀box，尺寸(.6,.06,.8) m、质量2 kg，初始世界位置(0,0,.6) m、无初始旋转。其中心主惯量Ixx=.10726666666666667、Iyy=.16666666666666667、Izz=.0606 kg·m²。world连接沿+X的prismatic slide_joint，q范围[0,.8] m；本固定初态下实际门板world x=q，y/z和轴向另独立核对，轨道约束支撑重力。

被动阻尼4 N·s/m，已给原生位置PD为P80 N/m、I0、D20 N·s/m，命令力范围±20 N。每步真实关节状态与门板WorldPose分开读取，运动过程中没有工具写入位姿、速度或预设轨迹。正常行程内门板不靠地面碰撞支撑或触发到位。控制器已提供，学员不必先编写插件。

parameters.json只含整数schema=1和有限数值open_target_m，允许[.2,.7] m，基准.6、单变量变式.4；关闭目标为0。参数合法不等于已经运动通过；保存后需显式重载。world的几何、惯量、拓扑、增益和工具源保持固定，prepare保留个人文件，运行前后核对实际加载与磁盘指纹。

## 通信图、名称与类型

Classic Master地址来自本会话GAZEBO_MASTER_URI。Node显式使用default世界，`~/transport_lab/command`展开为`/gazebo/default/transport_lab/command`，ack/state同理。~由Gazebo解析，不是Shell目录符号。相同完整主题名在不同私有Master下分属不同通信图；IGN_PARTITION不能替代Classic Master隔离，ROS_MASTER_URI也不是本课发现服务。

| 主题 | 真实线类型 | 用途 |
| --- | --- | --- |
| ~/transport_lab/command | gazebo.msgs.Request | 发布设置目标命令 |
| ~/transport_lab/ack | gazebo.msgs.Any | received/applied/completed或拒绝反馈 |
| ~/transport_lab/state | gazebo.msgs.Any | 按仿真时间20 Hz发布实际物理快照 |
| ~/request、~/response | gazebo.msgs.Request、gazebo.msgs.Response | 原生请求响应及本课只读查询 |

消息类型规定结构，字段的单位与业务约束仍由协议规定。一个Request发布到自定义command主题，仍然只是pub/sub；真正查询流程使用原生request/response主题及关联编号。可核对[Classic架构](https://classic.gazebosim.org/tutorials?tut=architecture)、[Node命名空间实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/transport/Node.cc)。

## 命令信封与拒绝条件

内置Request为proto2：id与request是必需字段，data与dbl_data在schema中可选。本课额外要求set_target必须提供dbl_data，值有限且在[0,.7] m；所以“Protobuf结构合法”与“业务接受”是两项检查。

| Request字段 | 约束 |
| --- | --- |
| id | 当前客户端递增的正整数seq |
| request | set_target |
| dbl_data | 目标q，单位m，必须存在且有限 |
| data | RUN_ID CLIENT_ID NONCE EPOCH ATTEMPT_NONCE五段，以空格分隔 |

RUN_ID绑定当前服务启动实例，CLIENT_ID是安全短ASCII标识，NONCE标识逻辑命令，EPOCH为复位代数，ATTEMPT_NONCE标识一次发布尝试。服务端核验信封、动作、目标、当前实例与epoch、seq/nonce，再接受至多一个新事务并发received；busy时拒绝，不悄悄覆盖正在运动的目标。拒绝命令不更新已接受seq，不改变待处理/已应用目标或力。

重复或旧seq、错误RUN/epoch、不支持动作、缺失或非法目标均应得到可区分的拒绝。只用正确线类型、完整初始化的Request测试这些业务错误；错误预期线类型在发布前按真实topic类型发现并拒绝，不向活跃TopicManager注入冲突类型。严格JSON中的非法数字为null或省略，不把NaN当有效JSON。

每次send自动产生新的attempt_nonce，ACK、票据和CSV保留它。重复逻辑命令可以保持seq/nonce不变，但新发布尝试使用新attempt_nonce并取得本次rejected；先前accepted尝试的迟到或latched ACK因attempt不同，不能满足新send。wait只沿已接受票据中的attempt继续订阅，不重新发布。

本协议在当前boot/epoch内防止同一已接受逻辑命令重复应用；所有字段完全相同的裸包仍是同一个attempt，不能声称识别出了没有新标识的另一次发布，也不承诺服务崩溃后的持久化exactly-once保证。这些关联字段均不代替真实执行证据。

## 三层ACK及物理时钟

1. Transport回调校验命令并加入受保护的待处理队列，返回received。回调只使用最后真实物理快照，不访问或修改physics对象；receipt中的iteration/sim_time可能早于回包到达时刻。
2. 真实物理Begin取出当前epoch的待处理命令，只设置一次原生PD目标。原生Model/JointController完成本步更新后，BeforePhysicsUpdate读取本步PD控制力输入再发applied，附实际iteration/time/目标及关联字段；这是pre-physics快照，不代表门板已经移动。之后控制器逐物理步施力，不用传输线程改位姿。
3. Post更新读取真实q、速度、link位姿和原生控制力输入，记录1 kHz样本。实际位置误差≤.005 m且|速度|≤.01 m/s连续≥.2仿真秒，即200个真实1 ms post步，且在applied后3仿真秒内，才发completed。控制器继续保持目标；单独open若已到位，可以没有新位移，完整验收另检查真实闭→开→闭。

力证据是原生JointController写入关节的控制力输入（GetCmd/GetForce(0)），不是力传感器读数或全部约束反力；ODE显式关节阻尼4 N·s/m另行作用，不计入这条保存的控制力账本。applied只证明本步PD输入已在pre-physics阶段写入，运动及到位仍由后续真实q/速度判定。

连续时长按真实物理区间累加：200步×.001 s=.2 s，首末两个post采样时间戳相差.199 s并不表示少了一个区间。每个区间及条件均须有原始记录，不能仅靠两个端点补出连续到位。

连接、发布完成和业务ACK是不同证据。[Publisher.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/transport/Publisher.cc)中的发送完成涉及订阅端传输入队，并不证明物理完成。工具先订阅，再Advertise和有界WaitForConnection，发布一次后按RUN/epoch/client/seq/nonce/attempt_nonce/stage匹配真实网络ACK；晚到或latched旧ACK不满足本次新发布尝试。

物理暂停不等于关闭Transport。暂停实验保留received事务后，只推进真实物理步并继续等它，不能再发送一遍命令冒充continuation。应用ACK用墙钟设置网络截止时间，到位用仿真时间和真实连续样本判断；客户端外层进程也有墙钟上限。若返回超时，未知的是截止前是否取得对应证据，不能凭超时推断动作绝未执行。

## Any容器与真正的请求响应

ACK/state外层gazebo.msgs.Any须检查type=STRING且string_value字段存在，随后解析其中JSON。这是本版本Gazebo proto2带标签容器，不是google.protobuf.Any，也不单凭存在字符串就忽略其type。[Any schema](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/any.proto)可核对字段。

只读查询名gazeboteach_transport_snapshot，通过原生~/request与~/response传输。查询data为新nonce，响应的Any.STRING JSON回显该nonce，含运行实例、世界、epoch、最后实际物理快照和源/配置指纹。客户端逐项检查Response.id/request/response/type、ParseFromString、Any标签及字段、内部nonce与预期boot/world；到此才确认得到本次查询的合格响应。

同一原生机制另用entity_info请求transport_gate，返回gazebo.msgs.Model，供独立核对实际joint与pose。自定义查询只回应自己的唯一查询名，不拦截entity_info或world_sdf。实现可看[TransportIface::request](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/transport/TransportIface.cc)、[Request](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/request.proto)和[Response](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/response.proto)。

Model消息内link.pose相对model，比较世界位姿须先组合model.pose。运动中RPC快照与entity_info可能取自不同物理时刻，不能要求其瞬时值完全相同；几何校核应在同一Model消息内部进行，或暂停后核对相同物理状态。

课程query助手与C++ query模式都只允许gazeboteach_transport_snapshot及固定安全未知探针gazeboteach_transport_missing；原生entity_info使用独立entity入口。原生gz的-r按具体请求语义工作，不能把任意请求都称为只读查询。

SDK请求显式有限超时，启动/发现和子进程也设外层期限；避免便利Publish或无参连接等待造成无限阻塞。必要的只读重查保持有界，并与改变物理目标的命令分开，不盲重发。

## Classic 11命令行语法对照

| 课程工具 | 实际用途 |
| --- | --- |
| python3 lab.py check-config | 只读检查个人输入，不能代替已加载实例核对 |
| python3 lab.py status / entity | 分别取得本课真实快照RPC、原生entity_info响应 |
| python3 lab.py watch --seconds 2 | 有界订阅真实网络状态 |
| python3 lab.py open / close / demo | 当前开度、关闭、有限开→关流程 |
| python3 lab.py cli-example | 按真实RUN/epoch生成新标识，打印可执行Classic发布命令 |
| python3 lab.py send --target .6 --wait received --ticket ticket.json | 发一次命令，只等received并保存事务票据 |
| python3 lab.py wait --ticket ticket.json --wait completed | 继续订阅匹配本事务后续ACK，绝不重发命令 |

表中的斜杠表示分别选择的动作，不是需要原样粘贴的Shell语法。暂停实验先确认pause，再send；随后实际步进或继续，再wait同一ticket。正常完整实验结束确认原pause，不恢复旧轨迹与时钟；receipt-only/continuation用于把通信和物理推进分开观察。

```sh
gz topic -l
gz topic -w default -i /gazebo/default/transport_lab/command
gz topic -w default -e '~/transport_lab/ack' -d 2
gz topic -w default -z '~/transport_lab/state' -d 2
gz topic -w default -r entity_info -m transport_gate
```

-i直接按完整主题名向Master取信息，不展开~，因此示例使用/gazebo/default/...；-e输出消息，-z观察频率，-d接整数秒期限。发布使用`-p TOPIC -m 'PROTO_TEXT'`；请求使用`-w WORLD -r REQUEST_NAME -m DATA`。本版本没有在这里用`-t`指定topic，也不以`-m TYPE`指定消息类型；消息文本用Protobuf文本格式，不是Python字典。

原生gz的请求等待为10 s，并可能忽略内部失败的返回码；不能只用进程退出0判请求成功。课程工具结合匹配响应与实际状态判断，另以外层期限约束原生工具。state的20 Hz以仿真时间计，暂停后频率降低或无新样本时，可用有界只读查询区分“Transport有响应、physics没推进”和真正失联。

本课command起初可只有订阅者，类型仍可由Master与gz topic -i发现；工具使用有界CLI完成这项内省。SDK的getTopicMsgType在此版本只查询发布者，不能把它的空结果直接解释成订阅主题不存在。

## 复位与旧事务

公开模型复位取消待处理/活动事务，增加epoch，清理接受记录与原生PID历史/目标，再安装hold0。门板复位位姿由Gazebo正常模型复位处理，helper不写pose/velocity；旧测量基线被丢弃，复位跳变不当成真实运动。旧epoch命令仍被拒绝，不自动开门或开始demo。q复位后的实际位置仍从SDK读取，不用状态文件写值作为证据。

因此复位前保存的ACK、continuation句柄或客户端缓存都必须与新epoch分辨。客户端重新查询当前实例/epoch，生成新的标识再发送新命令，不能拿上次completed匹配新开门。

## 挑战的判断路线

完整demo/verify先以目标0的真实set_target消息建立闭门基线，初始化阶段称prepare_closed(0)，再open(p)、close(0)，共三笔事务；prepare_closed不是新的消息动作名，也不靠Reset或写pose制造基线。基准p=.6、变式只改.4并显式重载，均保持结构/控制参数。合法低开度p=.2同样适用，通用真实跨度下限为p−.01 m，两端各在目标±.005 m内；不是无条件要求所有开度都移动至少.2 m。

验收结合真实开关运动、位置/速度连续到位、原生力范围与关节/门板几何一致；终点变化与持续到位分别检查，不把目标回显或最后一帧截图当成运动记录。完成时间可能不同，不预设与距离线性。

诊断时沿“Master/world/topic与线类型→连接→完整信封及接收ACK→真实物理applied→连续到位completed”逐层排查。错误topic没有订阅者、未知RPC、服务停止都应有界失败；不将原生CLI退出码或本地status文件代替真实回包。更深入的schema背景见[proto2语言指南](https://protobuf.dev/programming-guides/proto2/)，SDK与消息生成代码由平台准备。
