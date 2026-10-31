# 故障诊所：让一次修复有完整证据

同样是“方块没有按预期运动”，原因可能出现在不同层：SDF根本没有解析成功，模型出现了但插件没有加载，世界在推进却没有重力，或者图像里有方块而物理世界没有它的碰撞体。本课用四个受控故障练习定位、修复与回归。目标是拿出证据说明哪里错了、哪一处改动恢复了哪一种行为。

环境限定Ubuntu 20.04、Gazebo Classic 11.15.1、Python 3.8；平台界面使用Qt 5.12。本课不需要ROS。不要混用现代Gazebo的`gz sim`、ECS或ROS 2接口。默认预览是健康世界；所有注入、诊断和参考修复都使用个人`build/`中的副本及隔离的服务端，保留原来的学习文件和故障输入。

## 先建立诊断顺序

诊断从一个可以复述的现象开始，例如：“暂停初态z=2，推进到0.2模拟秒后，z仍为2，但x已经改变。”这比“物理坏了”更容易检验。

按**观察→复现→提出假设→最小修复→同一条件重跑→回归与留档**进行。先保存输入和日志，再用一次有限试验复现；为每个假设写出预期出现的证据，再改一个因素。修复后从相同初态重新运行，同样读取0、0.2和1.0模拟秒，最后确认原来的故障证据消失且健康行为恢复。

| 层次 | 这一层能回答什么 | 尚不能保证什么 |
| --- | --- | --- |
| XML / SDF解析 | 标签、层级和SDF转换是否被当前解析器接受？ | 质量、重力、碰撞和控制是否符合实验意图 |
| 世界与实体加载 | 目标世界、方块和物理参数是否真实存在？ | 插件共享库是否已经执行 |
| 插件加载 | 文件、依赖、注册入口和所属模型是否正确？ | 输出力的方向、大小、持续时间是否正确 |
| 物理行为 | 相同有限步数下，实际运动和接触是否符合预测？ | 其他输入、机器或GUI显示是否也已经验证 |

XML可解析、SDF可接受、插件可加载和行为正确是逐层增加的证据。单独看到一个成功退出码、一个模型图像或一个`HEALTHY`字符串，都不能替代整条链。

本课实际SDK还展示了一个容易漏掉的情况：`/usr/bin/gz-11.15.1 sdf -k`检查损坏的world闭合标签时，退出码仍为0、stdout为空，而stderr明确报告XML解析错误及`SDF parsing the xml failed`；健康输入退出码同为0，stdout为`Check complete`，stderr为空。因此工具保留原始退出码0，并结合原生成功标记、原生错误输出及独立XML结构检查判定解析有效性。不能把这个0改写成非零来让报告“看起来一致”。

## 最少背景与前置知识

建议回看第2课SDF、第6课质量惯量、第7课碰撞、第12课Transport、第26课ModelPlugin及第34课有限步进与独立试验。不会C++也可以先完成本课的文件诊断；代码阅读的重点是输入到真实物理结果的关系。

| 名词 | 本课中的含义 | 官方入口 |
| --- | --- | --- |
| model / link | model组织实体，link承载质量、惯量、碰撞和外观 | [SDF 1.6 link规范](https://sdformat.org/spec/1.6/link/) |
| visual / collision | visual参与显示，collision参与物理碰撞；二者可以使用不同几何 | [SDF 1.6 collision规范](https://sdformat.org/spec/1.6/collision/) |
| ModelPlugin | 作为共享库加载到所属模型，接入真实更新回调 | [Classic ModelPlugin教程](https://classic.gazebosim.org/tutorials?cat=write_plugin&tut=plugins_model) |
| ELF / ABI | ELF是Linux可执行与共享库格式；ABI是二进制接口约定，包括架构、符号和依赖 | [Classic 11.15.1插件加载源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/common/Plugin.hh) |
| 仿真时间 / iterations | 物理系统推进的时间及更新次数 | [Classic 11.15.1 World实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/World.cc) |

SDF里的`version="1.6"`是描述格式版本，Gazebo的11.15.1是应用版本，`libsdformat`还有自己的库版本。三个数字不能互相替代。先记录本机实际解析器与SDK，再决定使用哪份文档。[SDFormat版本说明](https://sdformat.org/spec/1.6/world/)

## 健康场景是共同参照

世界名为`lesson35`，模型为`cube_000`和静态`floor`。方块边长0.2 m、质量1 kg，从世界坐标(0,0,2)和零速度开始。世界重力为(0,0,−9.8) m/s²，ODE步长h=0.001 s。地面顶面为z=0；方块平放时，其质心高度约为0.1 m。

`force_plugin.cc`编译为`libfault_force.so`。它在每次物理更新开始时向方块link施加世界+X方向的1 N力，让水平方向成为插件是否有效的行为证据。它没有用指定轨迹、设置位姿或设置速度制造运动。健康方块会一边向+X运动，一边下落，再被地面承托；接触后的水平运动受实际接触与摩擦影响。

```text
个人SDF与当前插件源码
  → prepare.py构建并记录来源
  → 健康预览世界（供人工观察）
  → clinic.py创建保留原件的故障副本
  → 当前Gazebo解析器检查
  → 私有gzserver加载副本
  → 原生统计/物理参数/实体与日志读回
  → 有限步进到0.2 s、1.0 s
  → 依据证据诊断
  → 输出新的修复文件
  → 新服务端按相同步数重跑并比较
```

预览与私有试验都使用`real_time_update_rate=1000`作为目标墙钟更新频率，h=0.001 s；本课不测性能。1000个0.001 s物理步对应1模拟秒，实际等待还含通信、查询与启动开销。每次步进结束保持暂停，便于取得一致的检查点。

| 参数 | 健康参照 | 本课怎样使用 |
| --- | --- | --- |
| world gravity | 0 0 −9.8 m/s² | 零重力故障仅将它改为0 0 0 |
| max_step_size / real_time_update_rate | 0.001 s / 1000 Hz | 固定步数与模拟时刻的对应，不用于性能比较 |
| cube pose / mass / edge | (0,0,2) / 1 kg / 0.2 m | 修复前后保持一致 |
| force_x | 1 N | 插件读取的X方向力；源码只接受有限且绝对值不超过10 N的值 |
| step --steps | 1..1000的整数 | 本课先200步、再800步，合计1模拟秒 |
| Gazebo seed | 3501 | 私有服务端使用固定原生种子；不是Python生成轨迹的种子 |

健康自动判据针对上述参照，特别是1 N、1 kg和1 ms。探索其它力或质量需要先保存副本、重新准备并说明新预测；当前`HEALTHY`阈值不会自动按新参数重写。不要为了让任意参数通过而删除默认判据。

## 先写预测，再看诊断名称

接触前的连续近似是`ax=F/m=1 m/s²`、`az=−9.8 m/s²`，因此`x(t)=0.5t²`、`z(t)=2−4.9t²`。用先更新速度、后更新位置的离散式估计：

```text
vx_k = vx_(k−1) + (F/m)h
x_k  = x_(k−1) + vx_k h
vz_k = vz_(k−1) − gh
z_k  = z_(k−1) + vz_k h
```

在h=0.001 s、200步时，名义离散预测为x=0.0201 m、z=1.80302 m。无碰撞且持续自由落体至1000步时，z预测为−2.9049 m。健康方块约0.623 s首次接触地面，1.0 s时应靠近地面；触地后不能继续套自由落体公式。下表都是预测，真实数字应从你这次的原生记录填写。

| 待验证现象 | 0.2 s应看到什么 | 1.0 s应看到什么 | 能排除哪个常见误判 |
| --- | --- | --- | --- |
| 健康 | x增加，z下降 | 方块被地面承托，z约0.1 | 仅摆在地面不能证明做过下落 |
| 插件未加载 | x约0，z仍下降 | 仍能落地 | 服务端能运行不等于ModelPlugin有效 |
| 零重力 | x增加，z约2 | z仍约2，模拟时间已推进 | 悬空不一定是暂停 |
| 方块缺碰撞体 | 接触前的x、z近似健康 | 质心穿到地面下方 | 看得见方块不等于存在物理碰撞 |
| SDF标签损坏 | 解析阶段已有错误证据 | 不应捏造轨迹或补上测量 | 解析失败要先修语法，不能先调物理 |

本课只注入一个因素。真实工程里，零重力可能与缺碰撞同时出现，插件也可能加载了却作用于错误link。超出四种受控模式时应保留`UNKNOWN`和不确定性，不能靠文件名把未知故障强行归类。

## 开始与健康预览

在平台选择本课，等待个人准备、真实SDK编译和预览启动完成。在本课终端执行：

```sh
python3 lab.py ready
python3 lab.py status
```

ready核对个人输入、构建及预览的世界名、方块身份与步长，并返回原生物理参数和统计；冷启动就绪不证明1 N已经产生真实位移，也不等于四个故障已经修好。需要人工看运动时，从初态开始：

```sh
python3 lab.py reset
python3 lab.py step --steps 200
```

200步对应0.2模拟秒。打开Gazebo观察方块稍向+X运动并下落，结合status读回数值；再推进800步观察触地后的高度。也可用`resume`连续播放，用`pause`暂停。预览与诊断试验属于不同服务端，预览里的操作不能充当某次`before`或`after`的测量。

## 一次完成四组故障与参考修复

```sh
python3 clinic.py run
```

默认逐组运行`malformed-sdf`、`missing-plugin`、`zero-gravity`、`missing-collision`。也可只做一组：

```sh
python3 clinic.py run --case zero-gravity
```

每次运行生成新的`build/clinic-<UUID>/`归档。每组保留`bad.sdf`及新建的`repaired.sdf`，`before/`记录故障输入的实际诊断，`after/`记录修复输入的新进程重跑。正常的故障被识别是实验所需结果，不要为了让日志全无错误而删掉故障证据。

这条命令提供的是**自动生成的参考修复对照**。学生完成任务还应能解释判据、指出改动节点，并亲手对一份副本执行最小修复与复测。若只是从参考文件复制完整世界，应明确写“参考重建”，不能记作自己已定位并修好任意损坏文件。

## 亲手做一次可审查的修复

先只注入并保留输入，再诊断，输出目录和修复文件都使用尚不存在的新路径。下面选择缺碰撞体，路径仅是本次学习副本：

```sh
python3 clinic.py inject --case missing-collision --out build/my-bad.sdf
python3 clinic.py diagnose build/my-bad.sdf --out build/my-before
```

打开诊断目录中的JSON、解析器输出、原生事件和服务端日志。先不看场景名，说明三个证据：解析是否通过，0.2 s是否下落，1.0 s是否越过地面；再检查原生实体中方块collision是否存在。只看负高度还不足以证明一定是缺碰撞，还要排除地面缺失、几何位置错误等其他解释。

用编辑器把`build/my-bad.sdf`复制为`build/my-fixed.sdf`，只在`cube_000`的`<link name="body">`内恢复下面的collision，与inertial、visual同级，保留原来的visual、质量、惯量、重力与插件。不要覆盖`my-bad.sdf`。

```xml
<collision name="body">
  <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
  <surface><bounce><restitution_coefficient>0</restitution_coefficient></bounce></surface>
</collision>
```

再运行：

```sh
python3 clinic.py diagnose build/my-fixed.sdf --out build/my-after
```

用相同初态、200步与累计1000步比较修复前后。你应确认接触前下落仍成立，修复后1.0 s方块已被地面承托，水平推力的行为也仍有效。只修复穿透，却把方块改成static并放在地面上，不能满足这组回归。

工具也提供依据诊断报告输出新参考修复文件的入口：

```sh
python3 clinic.py repair build/my-bad.sdf --out build/reference-fixed.sdf --report build/my-before/diagnosis.json
```

请以当前命令返回的实际诊断报告路径为准。输出文件新建后，仍需`diagnose`重跑。repair生成SDF文件这件事本身不证明Gazebo已经加载它，更不证明正确运动。

## 文件与来源怎么核对

| 文件或目录 | 阅读目的 |
| --- | --- |
| world.sdf | 个人健康预览和故障注入的共同原型 |
| force_plugin.cc | +X恒力是怎样进入真实物理更新的 |
| observer.cc | 原生统计、物理参数、实体查询及有限步进 |
| config.py / prepare.py | 输入检查、SDK构建和来源摘要 |
| lab.py | 健康预览的就绪、状态、暂停、继续、复位及步进 |
| clinic.py | 注入、按证据诊断、输出新修复文件及独立重跑 |
| verify.py | 对当前个人输入执行有限自动验收 |
| build/clinic-<UUID>/report.json | 本次四组故障与修复的汇总入口 |
| 每组bad.sdf / repaired.sdf | 保留的原输入与实际重跑的新输入 |
| before/、after/中的parser.json / parser.log | 当前`gz sdf -k`的原始退出码、stdout/stderr、原生成功/错误标记、XML结构结果及联合判定 |
| native-events.jsonl / gzserver.out | 实际通信读回和服务端加载信息 |
| ownership.json / trial.json | 本次私有地址、进程归属、输入摘要与检查点 |
| diagnosis.json | diagnosis.code、diagnosis.evidence及完整observation；repair读取这一文件 |
| gzserver-proc-maps.txt | 服务端实际映射库；结合同一文件及字节摘要核对预期ELF已加载 |

输入SHA256用于核对字节身份；它不证明模型物理正确。当前源码、编译产物与实际Gazebo SDK也要有可追溯对应。保存个人学习源码后先重载项目，重新准备并运行；磁盘新源码不会自动替换已加载到服务端内存的插件。

每个试验都使用新的Classic master地址、独立partition和所属进程记录；收尾只处理本次创建的进程。不要用`killall`、`pkill`或删除系统插件制造故障，也不要修改用户的全局`~/.bashrc`来完成本课。一个同名`cube_000`不足以证明连接到目标实例，要结合私有地址、输入摘要和进程身份。

## 用日志把假设缩小

| 现象 | 先查证据 | 最小动作与复测 |
| --- | --- | --- |
| 标签错误、模型未出现 | parser原始输出、行列位置、实际输入字节 | 修一个闭合标签，重新解析，再开始物理验收 |
| 正常下落但x没有增加 | 插件filename、共享库与依赖、加载日志、0.2 s原生x | 只修插件路径或构建对应，重跑相同200步 |
| 方块悬空 | 原生sim_time/iterations是否推进、world gravity、link gravity与static | 先区分暂停与零重力，再改已证实的因素 |
| 穿过可见地面 | 方块和地面的collision、geometry、pose及实体读回 | 恢复缺失碰撞后重做下落与着地 |
| 文件已改却行为没变 | 实际运行SDF路径、摘要、插件构建来源、进程身份 | 保存并重载，开启新试验，保留旧记录 |
| 检查超时或证据不足 | parser/server输出、native-events、所属进程是否退出 | 保留失败目录，解决原因后开启新试验 |

插件查找首先区分“文件找不到”“依赖库或符号失败”“插件类型/注册入口错误”和“已加载但行为错误”。`GAZEBO_PLUGIN_PATH`为Classic提供插件搜索路径；绝对filename则直接指定文件。文件存在不能证明架构、依赖和ABI匹配。必要时对**本课自己构建的库**只读使用`file`、`readelf -h`、`ldd`，并保留完整服务端错误。[Classic插件配置](https://classic.gazebosim.org/tutorials?cat=write_plugin&tut=plugins_hello_world)与[11.15.1动态加载实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/common/Plugin.hh)给出了搜索、dlopen及注册符号阶段。

本课故意指向一份不存在的私有插件文件，不删除已安装库，也不把系统`GAZEBO_PLUGIN_PATH`改坏。修复目标是恢复本课已构建的库引用，而非盲目往全局路径加入所有目录。

## 把方法迁移到相邻问题

以下是排查路线，不是本课已经注入或自动验收的额外故障：

| 问题 | 应补的观察 | 先回看与官方资料 |
| --- | --- | --- |
| 方向或量级异常 | world/link坐标、力的分量、m与mm、rad与deg、质量单位 | 第3、6、26课；[Link力接口](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/Link.hh) |
| ROS命令无响应或串到另一台机器人 | 发布/订阅的完整名称、namespace、实际模型；不要用节点同名下结论 | 第22、33课；[Classic ROS通信](https://classic.gazebosim.org/tutorials?cat=connect_ros&tut=ros_comm) |
| ROS时间停住 | 原生世界是否推进、gazebo_ros是否发布/clock、/use_sim_time与当前ROS master | 第24课；[gazebo_ros_api_plugin Noetic实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp) |
| GUI黑屏、窗口失败或渲染报错 | 服务端物理是否仍可读、客户端日志、渲染驱动/显示环境、是否有服务端渲染传感器 | 第17、18、29课；[Classic 11.15.1渲染初始化与错误路径](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/RenderEngine.cc) |
| 运动爆炸或接触抖动 | 质量惯量、重叠初态、步长、求解器与接触参数 | 第6—8课；[Classic物理参数](https://classic.gazebosim.org/tutorials?cat=physics&tut=physics_params) |

这一课的headless落体能通过，不代表摄像机或GPU链路也通过；GUI出现模型同样不能代替物理读回。不要为排查路线自行卸载驱动、修改系统库或破坏其他正在运行的实验。

## 验收与挑战

```sh
python3 lab.py pause
python3 verify.py
```

自动验收实际启动隔离服务端，检查故障证据、诊断结果、修复后原生运动及来源关系。它的适用范围是本课受控场景；不是任意Gazebo世界的通用故障专家。阅读当前运行报告，逐条确认哪些检查通过、哪些失败；没有采集到的数据保持缺失，不能用公式预测填入。

人工观察单独记录：是否真正打开了Gazebo，是否看过健康方块下落与触地，是否看到暂停状态。未打开窗口时写“完成无界面原生数值验收，GUI观察未做”。静态源码检查、真实Gazebo运行、人工GUI观察是三种证据，不得合并成一句“全部已验证”。

挑战一：把一份故障SDF复制成无提示的文件名，再运行diagnose。只依据解析器、参数、collision、插件证据和两个物理检查点写出诊断；最后再核对故障来源。若改文件名就改变诊断，应继续查工具，而不是接受结果。

挑战二：选择零重力，列出“暂停”“static”“world gravity=0”“link gravity=false”四种假设各自需要的证据。实际修复本课world gravity后，验证z恢复下落且x推力仍有效。不要在同一次修复里同时改质量、步长和初始高度。

挑战三：做一张修复记录表，包含原输入摘要、现象、假设、支持/反驳证据、改动节点、新输入摘要、同条件结果、未覆盖范围。指出为什么修复后出现方块还不够，为什么缺碰撞组必须观察到越过地面的旧行为及被地面承托的新行为。

本课实现流程为：**健康SDF与真实SDK来源核对→保留原件并生成单故障副本→调用当前解析器→必要时新建私有Gazebo服务端→读回初态并推进两个检查点→用原生参数、实体、日志与真实运动共同判定→在新文件实施最小参考修复→新服务端同条件回归→保存全部输入、原始证据和所属进程收尾结果。** 插件每次更新只施加真实力，预测只用于比较；诊断和修复都不能补造缺失的物理测量。
