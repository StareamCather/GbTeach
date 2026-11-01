# 原生世界控制：持久连接、实际统计与有界退出

本文说明GazeboTeach框架中暂停、继续、单步、多步和模型复位的执行流程，对应Gazebo Classic 11.15.1。它是框架实现说明，不改变课程的物理模型或控制算法。实际GUI、压力和物理验收结果另列[验证记录](VALIDATION_LOG.md)；建立了以下接口，不等于已经完成这些运行验收。

## 为什么引入持久助手

开发中出现过两类不同的实际故障，需要分别定位。旧机械臂课程反复创建短命原生客户端时，GDB捕获过gzserver内`Master::RunOnce→RemoveConnection→RemovePublisher`路径上的空Connection指针异常。随后课程改为持久客户端，真实服务器在持续请求中可以保持运行；但框架继续调用官方`gz world -s`时，另观察到该CLI进程以−6退出并报告`Resource deadlock avoided`。后一个故障发生在客户端清理路径，不能把“服务器仍活着”当作单步命令已可靠完成，也不能把它写成前一个Master异常再次发生。

本次框架调整把原先反复运行短命`gz`控制命令的路径，接到一份由LessonSession持有的持久原生助手。算法仍发送Classic的WorldControl，并读取实际WorldStatistics；不是通过修改实体位姿、改关节速度或冻结显示来实现暂停和单步。没有修改主机SDK、预加载共享库、图形驱动或操作系统，也没有通过重试修改命令掩盖未知结果。[WorldControl定义](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/world_control.proto)、[WorldStatistics定义](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/world_stats.proto)

## 从能力选择到退出的完整流程

```text
LessonSession创建私有环境与会话身份
  → 核对实际SDK、编译器和11.15.1运行库身份
  ├→ 能力不具备：记录具体原因，选择原有CLI路径
  └→ 能力匹配：从源码和实际依赖指纹选择/构建缓存
       → 启动一份本会话拥有的native_control助手
       → 持有WorldControl发布连接与WorldStatistics订阅连接
       → stdin逐行JSON请求，每次新UUID
       → 请求串行执行，修改命令仅发布一次
       → 回读新统计，检查paused/iterations等实际条件
       → stdout返回本次UUID对应的结果，记录来源和进程身份
       → 关闭时stdin EOF
       → 原生主线程transport清理、IO线程停止和join
       → 确认助手退出，再清理本会话gzserver及其余进程
```

助手采用独立进程组/会话，继承本应用会话标记。Python持有具体Popen对象与启动身份，正常退出使用EOF，有限等待未完成时才对自己拥有的子进程使用有界终止后备路径。外层会话标记清理继续用于收尾，不使用killall或pkill处理用户的其它Gazebo、ROS或终端。

## 可选能力与构建缓存

入口为[try_create](../gazeboteach/native_control.py)的`try_create(root, env, world, session_dir)`，原生源码位于[resources/native_control.cc](../resources/native_control.cc)。[LessonSession](../gazeboteach/runtime.py)在已有服务端世界就绪后、课程启动后命令和辅助节点之前选择后端，不另启一份gzserver或Master。本功能只在能核对实际POSIX运行环境、g++、pkg-config、Gazebo 11.15.1 SDK与对应运行库，以及版本化gzserver身份时启用。仅有某个版本字符串、某份头文件或同名共享库不足以证明可用。

没有SDK或版本不在支持范围内时，明确记录诊断并保留原有CLI控制方式；不会因此把全部基础课程变成必须安装开发包。对于已识别为支持的环境，编译失败、运行库身份错误或握手失败需要显式报告，不静默换成另一条修改路径。已经向世界发送修改请求之后，也不因超时自动回落到CLI重发。

会话元数据把启用分支标为`classic11-sdk-persistent`；回落分支为`legacy-cli`，并保存supported=false和具体reason，版本不匹配时还保存sdk_version。CLI分支继续使用原有状态回读和命令处理，不继承原生助手新增的统计序号、精确迭代计数或SDK线程退出保护。后面的协议和精确单步说明针对持久助手分支。

课程子进程的工作目录属于个人练习区，PYTHONPATH也可能只含课程源目录。共享辅助模块使用平台原有的GAZEBOTEACH_SOURCE_ROOT定位应用，缺省时才按已安装_support文件的相对布局查找；先检查native_control.py及原生.cc资源确实存在，再仅向本子进程的sys.path加入应用根目录。这样导入不依赖个人练习区的当前目录，也不需要修改操作系统Python路径或增加运行器环境变量。

缓存位于`$XDG_CACHE_HOME/gazeboteach/native-control/<fingerprint>`，使用应用默认缓存位置时遵循相同目录结构，位于学生练习文件之外。指纹覆盖助手源码、编译器、实际传递头文件依赖、编译链接选项、链接库与运行依赖。加锁与原子产物替换避免并发构建使用半成品；复用前仍核对产物身份。重新编译不覆盖world.sdf、个人参数或其它课程源文件，也不替换系统Gazebo二进制。

## 每种操作究竟确认什么

Python接口为`NativeControl`，提供status、model_pose、pause、step、reset_models、metadata与close。控制请求共用顺序锁和有限墙钟等待，一次只处理一个请求。每个管道请求和回应使用新的UUID；WorldControl消息本身不含这个UUID，因此要区分“助手把结果匹配给哪次请求”与“世界实际出现了什么状态”。

原生进程启动后先返回ready=true及stats。后续每行请求都有id、action、timeout_ms；pause另有严格布尔类型paused，step另有整数count，model_pose另有字符串name，status与reset_models没有附加业务字段。回应含相同id与ok；成功时返回stats，单步还返回baseline_iterations和requested_steps，失败时保留error。stats包含seq、iterations、paused、sim_time、sim_sec和sim_nsec。Python默认等待为status 3 s、pause 4 s、step 5 s、reset_models 4 s，等待使用单调墙钟；这些是超时边界，不是模拟运行时长。

| 操作 | 原生动作 | 成功所需的实际证据 |
| --- | --- | --- |
| status | 不修改世界，等待新的统计序号 | 新到达的WorldStatistics中的paused、iterations、sim_time等字段 |
| model_pose(name) | 持久Request/Response通道上的只读entity_info查询 | 匹配正31bit请求ID、entity_info、成功状态、Model类型与名称，返回实际Model.pose及SDK转换的RPY；保留两侧新统计的独立接收记录 |
| pause(true/false) | WorldControl明确设置pause，只发布一次 | 发布后得到新统计，实际paused等于目标值 |
| step(N) | 先建立真实暂停状态与迭代基线I，再发布一次multi_step=N | 后续新统计仍暂停且iterations恰好为I+N；超出目标报告干扰或异常 |
| reset_models | 一次WorldReset：all=false、model_only=true、time_only=false | 保存发布与后续新统计；模型位姿是否复原仍需实际实体读回验证 |

model_pose不会发布WorldControl或Model修改。它首次使用时在同一个Node懒创建请求发布器和响应订阅器，此后持有到fini与主线程IO Stop/join之后才释放。[Classic 11.15.1 TransportIface实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/transport/TransportIface.cc)中的通用request函数每次创建并释放自己的节点和发布器，因此此处使用可复用通道。每次查询ID以PID、随机数与单调时钟播种，在正31bit范围内选取并在本辅助进程生命期内去重；返回时再核对ID、请求名、消息类型及实际模型名。请求连接建立只证明正向路径，若反向订阅尚未收到响应，最多重发一次只读entity_info并使用新ID，两次共享同一墙钟截止时间。这个有限重读仅适用于模型查询，原有修改操作没有新增重试。

model_pose成功回应增加model_name、request_id、response_id、response_type、pose、rpy、pose_stats_before及pose_stats_after。pose只包含position的x/y/z和orientation的x/y/z/w，Python拒绝非有限值或非单位四元数；rpy直接由已验证SDK的Quaternion.Euler生成。两侧统计分别来自查询前后新接收的真实WorldStatistics，最后一条也作为普通stats返回；它们不把异步模型响应包装成原子快照。课程01–07共同通过`_control_support/models.py`读取xyz与RPY，准确SDK缺失时明确诊断并保留运行时CLI路径；SDK已经可用后的失败直接报告。共享依赖参与控制桥缓存摘要，辅助源码则由原生控制构建的完整依赖指纹覆盖。 Python还检查原生ID落在1..0x7fffffff，左侧统计序号不早于该连接前次已确认序号，右侧严格更新且等于本次普通stats；外部完整复位可能使源模拟时钟倒回，因此不把模拟时间单调递增当作通用协议条件。

统计的“新”以接收序号推进判断，不用一条旧缓存立即满足所有请求。接收时刻、原生sim_time、迭代计数和GUI刷新时间分别保存或解释；它们不是同一个时钟。统计是实际世界输出，但并非一个携带请求UUID的原生操作确认服务。尤其复位后的新统计只能说明世界仍在提供反馈，单独不能证明每个模型已经恢复。

修改请求发出后若超时，结果可能已经发生而确认尚未到达。Python把该连接标为poisoned，保留错误和最后已观察统计，后续请求被拒绝；不重发pause、step或reset，也不自动重启助手。同一会话需要明确诊断或结束；不能把未知结果当作“没有执行”，否则单步可能变成多步、一次复位可能变成重复复位。

## 单步数量与时钟语义

N必须是1至1,000,000之间的整数，布尔值不作为步数。框架先确认世界实际处于暂停，再记录原生iterations基线I，一次发送N步，C++与Python都检查最终计数恰好为I+N。单看命令退出码、等待一段固定墙钟时间、看到画面变化或看到sim_time增加，都不能证明精确执行了N个物理迭代。其它GUI或终端在这段窗口内继续运行或额外单步，可能造成超出目标；程序应报告这一条件，而不重新取一个基线把多走的步数藏掉。[Classic处理WorldControl的实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/World.cc)

本接口的reset_models保留模拟时钟，不采用隐式默认all=true，也不设置time_only=true。[WorldReset默认值](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/world_reset.proto)使显式填写三个字段有意义。模型复位、时钟归零与重启会话是不同操作；原有课程涉及传感器时间缓存时，仍要按课程说明处理，不能由框架名称推断所有缓存都已清空。

## 主线程清理为什么是生命周期的一部分

Classic transport使用IO工作线程。该版本IOManager::Stop会停止io_service并join线程；若最后一个相关对象在IO线程自身析构，直接走到自我join便可能报告资源死锁。助手在初始化异步transport前建立由主线程持有的连接保护，核对精确SDK与运行DSO身份，退出时仍由主线程完成fini、停止与join。它不是任意SDK均适用的私有ABI约定，也不能用进程exit或固定sleep代替正确的线程退出。[Classic 11.15.1 IOManager实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/transport/IOManager.cc)

框架在停止gzserver之前先关闭持久助手，让EOF清理有机会在服务器仍存在时完成；随后再按现有拥有关系清理其它进程。正常路径和终止后备路径应在metadata中区分，记录EOF、退出状态、错误和是否需要额外终止。收到退出信号、成功写入EOF与已经完成IO线程join是不同阶段，不能只凭其中一个就宣布完整清理完成。

## 来源、诊断和验证边界

session.json的native_control保存backend、process、build、world、stats、poisoned、closed与cleanup。process记录pid、ppid、start_time、pgid、sid、uid、token、executable、executable_sha256和stderr_path；build记录源码与产物摘要、cache_key、record_path和sdk_version。cleanup区分eof_requested、forced、io_guard_joined、returncode和可用的error。助手stderr保存在本会话native-control.stderr.log，构建细节位于缓存build.json。

框架成功的暂停/继续、单步和模型复位记录包含native_result，对应本次UUID、实际统计与单步前后迭代值；错误保留原始原因。只读status返回新统计并更新助手的last_stats，不声称每次状态读取都另外写入持久请求日志。会话结束后保留最终元数据和已有操作记录，便于区分能力回落、构建失败、助手退出、控制结果未知和物理目标未满足。

静态检查覆盖协议、字段和退出顺序；真实框架检查需要覆盖暂停/继续、新统计、精确单步和多步、保留时钟的模型复位、无SDK时的显式回落，以及助手先于服务器退出。桌面操作应另外核对公开控制入口与GUI中的实际状态。压力测试只能说明已执行的请求和生命周期窗口，不证明任意频率、任意外部干扰或其它SDK版本均可靠。正式通过情况以[验证记录](VALIDATION_LOG.md)中的实际报告和原始证据为准。

修改本模块时，按“能力选择→构建身份→进程拥有关系→新请求→实际回读→记录→EOF/线程join→服务器清理”的顺序解释算法流程。改step就重查实际iterations差；改reset就写明模型和时钟语义；改超时不能引入隐式修改重试；改构建不能把可选SDK变成全部课程的启动前提。本文不修改课程37已经冻结的模型、力矩控制或教材。
