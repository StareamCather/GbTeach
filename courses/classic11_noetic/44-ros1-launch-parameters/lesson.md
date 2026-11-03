# 附加专项 44：launch、YAML与参数，配置究竟什么时候生效

这一课继续使用真实滑轨，集中解决工程里常见的一件事：**参数服务器上已经改成2，为什么机器仍按1执行？** 你会把包、launch替换、命名空间、私有参数、YAML类型和运行期读参串成一条可观察的信息流，再用真实位移验证它。

先修第22和42专项。本课使用已有ROS消息与服务，不需要生成自定义消息；源包通过本课`ROS_PACKAGE_PATH`发现，启动前不会要求新建全局catkin overlay。

## 先区分四层配置

| 层次 | 本课例子 | 何时决定 |
|---|---|---|
| launch arg | `namespace`、`pulse_scale` | 启动文件解析与替换时 |
| YAML与param服务器值 | `pulse_force`、`scope_example` | 加载或主动修改时 |
| 节点内部有效值 | 启动缓存的`pulse_scale` | 仪器启动读入后 |
| 本次物理指令 | 请求力×已缓存scale，有限持续时间 | 每次Trigger处理时 |

参数服务器保存配置数据。一个字典中的数值变化，不会自动证明节点重新读取了它，更不会证明新的力已作用在滑块上。是否重新读取、怎样检查、何时采用，由节点代码决定。

本课故意采用两种策略：`~pulse_scale`只在启动时读取并缓存；`~pulse_force`每次收到pulse服务请求时主动读取。这样可以在同一条滑轨上比较“服务器已变”“程序已采用”和“物理已响应”三种证据。

## 包与文件：源代码怎样找到自己的配置

| 文件或目录 | 作用 |
|---|---|
| `ros_ws/src/gazeboteach_workshop44/package.xml` | 声明本课ROS运行包 |
| `launch/lab.launch` | namespace分组、arg替换、typed参数与节点启动 |
| `config/controller.yaml` | 节点私有参数的默认值与类型示例 |
| `scripts/node.py` | 定位个人练习根目录，再进入仪器 |
| `instrument.py` | 本课独立参数逻辑、Trigger服务、真实关节观测与验收 |
| `lab.py` | ready、state、pulse、verify、student-check入口 |
| `student.py`、`reference.py` | 参数校验填空与独立完整参考 |
| `world.sdf` | 1 kg滑块、0.8 N·s/m阻尼与滑动关节 |
| `build/verification.json` | 实际参数、原生时间/关节读数与验证记录 |

本课保留个人`instrument.py`、`student.py`、`reference.py`，初始化不会拿公共脚本覆盖你的练习。`node.py`从自身包位置定位个人根目录并切换工作目录，避免roslaunch默认的ROS_HOME让程序误读另一个目录里的文件。

在本课终端运行 `rospack find gazeboteach_workshop44`，应找到当前个人副本。包能被rospack发现，不代表它一定经过编译；本课是源代码即可运行的Python包，不需要为了启动它额外修改全局`CMAKE_PREFIX_PATH`或`~/.bashrc`。

## 三种名称：同名键可以指向不同参数

默认节点是 `/workshop44/instrument`。ROS名称的解析方式不同：

| 写法 | 默认解析示例 |
|---|---|
| 全局名 `/scope_example` | `/scope_example` |
| 节点namespace下的相对名 `scope_example` | `/workshop44/scope_example` |
| 节点私有名 `~scope_example` | `/workshop44/instrument/scope_example` |

本课group下的相对`scope_example`值为`group_scope`，节点私有YAML中的同名键为`private_scope`。它们同时存在，不会互相覆盖。`pulse_force`位于节点私有位置，所以默认完整名是：

```text
/workshop44/instrument/pulse_force
```

先预测下面两条的结果，再运行：

```bash
rosparam get /workshop44/scope_example
rosparam get /workshop44/instrument/scope_example
python3 lab.py ready
python3 lab.py state
```

ready/state会给出实际节点与私有参数前缀。变体节点名字变化时，应以实际解析结果找参数，不能只把界面上看到的某个数字当作`~pulse_force`。

## launch arg与typed替换

`$(arg pulse_scale)`是launch解析期间的替换。命令行的 `pulse_scale:=0.5`先作为启动参数进入解析器，再由 `<param ... type="double">`明确转换为数值。

在本课终端可以静态展开一个变体：

```bash
roslaunch --nodes gazeboteach_workshop44 lab.launch namespace:=demo44 pulse_scale:=0.5
```

这条命令只显示准备启动的节点，不证明节点已运行。实际ROS图要用`rosnode list`和对应接口验证。

真正运行非默认变体时，课包的启动命令、启动后ready检查、验收和操作CLI必须使用同一namespace。例如，若仪器已经以`namespace:=demo44 pulse_scale:=0.5`启动，则操作使用：

```bash
python3 lab.py ready __ns:=/demo44
python3 lab.py state __ns:=/demo44
python3 lab.py pulse __ns:=/demo44
```

`__ns`是ROS名称重映射参数，由rospy处理；它不会把一个已经在默认namespace运行的仪器搬过去。仅改launch默认值而保留旧的ready/verify命令，会让启动器找错接口；课程变体配置要同步这些入口。默认课程直接使用`workshop44`，不需要做这项部署改动。

## YAML类型不是显示格式

这些值看起来接近，但类型不同：

```yaml
pulse_force: 1.0       # 浮点数
# pulse_force: 1       # 整数，本课也允许
# pulse_force: "1.0"   # 字符串，本课拒绝
# pulse_force: true    # 布尔值，本课拒绝
```

本课请求力必须是有限数值，范围为0.2～3 N；启动scale允许0.5～1.5，并另行限制实际施力不超过5 N。不要为了让输入通过，先把任意字符串或布尔值强制转换成浮点数。Python的`bool`还是`int`的子类，因此只写`isinstance(value, int)`不够表达这里的类型要求。

YAML文件加载到参数服务器后只是一个快照。运行中用rosparam修改，不会自动写回YAML；修改个人YAML后也需要重新加载或重启，节点才有机会读到新默认值。

## 第一组实验：运行期请求力与真实响应

开始实验后，保持默认scale=1。先看真实关节与参数，再改请求力：

```bash
python3 lab.py state
rosparam get /workshop44/instrument/pulse_force
rosparam set /workshop44/instrument/pulse_force 2.0
python3 lab.py pulse
```

pulse响应会记录这次实际读取的请求值、启动缓存的scale、实际使用的力和计划开始时间。它只确认一次有限施力已调度，不表示滑块已完成运动。

本课把脉冲安排在当时原生仿真时间之后约0.08 s开始，持续0.2**仿真秒**。如果暂停仿真，这些时长不会按墙钟流逝。参数有效值要用这次服务的实际使用记录与后续真实关节变化共同验证。

运行 `python3 verify.py` 会从相同初态分别施加F与2F。默认请求1 N与2 N，对应同一scale下的实际力；每次先清除施力并用本课模型复位归零位置与速度，保留仿真时钟。随后在脉冲开始后约0.45仿真秒读取原生状态，保留观测时间括区。

预测：在线性质量/阻尼模型、未碰到关节限位且没有力饱和时，2F的位移与速度应约为F的两倍。验收同时保留两次真实原始读数，并考虑查询时刻的微小差异。原始样本不是同时发生，不能只睡0.45墙钟秒就宣称时间对齐。

## 第二组实验：服务器scale改了，缓存scale没变

只修改参数服务器：

```bash
rosparam set /workshop44/instrument/pulse_scale 0.5
python3 lab.py state
python3 lab.py pulse
```

预测：state中服务器当前scale与启动缓存scale可能不同。若本次仪器启动时缓存的是1，实际力仍由请求力乘1得到；写入服务器的0.5不会自动改掉这个缓存。这里要检查本次实际使用的值，而不是只查`rosparam get`。

要让新的启动scale生效，需要按一致的launch配置重启仪器。重新启动时launch/YAML可能覆盖临时rosparam值，所以应区分“修改了运行期字典”与“保存了下一次启动的配置”。做完本组实验，恢复原来的服务器值，便于后续对照。

这是一种明确的启动缓存与主动读参设计。本课没有实现dynamic_reconfigure回调，也不把参数服务器当作会主动驱动所有节点更新的控制器。

## 第三组实验：非法参数应拒绝，节点仍可恢复

依次只测试一个非法输入，查看pulse失败原因；不要同时改其它参数。

```bash
rosparam set /workshop44/instrument/pulse_force true
python3 lab.py pulse
rosparam set /workshop44/instrument/pulse_force 6.0
python3 lab.py pulse
rosparam set /workshop44/instrument/pulse_force 1.0
python3 lab.py pulse
```

布尔值和越界值应在调度施力前被拒绝。恢复合法1.0后，节点应能再次处理请求，不应要求每次参数错误都重启整个应用。验收也会检查字符串、非有限值等边界，并通过真实关节状态确认拒绝时没有产生新的运动。

state用类型和可打印表示描述请求值，避免把NaN直接写成非法JSON数值。看到服务器保存了非法值并不代表节点已接受该值；同样，看到请求被拒绝也不代表服务器里的值已经自动修正。

## 两行填空：表达清楚类型与范围

`student.validate_force(value)`保留两处判断：

1. 它是否是允许的数值类型，并明确排除布尔值？
2. 在类型有效的前提下，是否有限且位于0.2～3 N？

第二处要注意短路求值：先排除字符串等类型，再调用有限性检查。填好后运行：

```bash
python3 lab.py student-check
```

完整答案在独立`reference.py`。本课仪器的可运行校验逻辑与这份练习分开；`student-check`只检查你的函数，默认演示和参考验收均不替你填写作业，报告的`assignment_pass=false`不应被误读为应用运行失败。参数校验练习使用上述独立函数入口。

## 调用预算与故障诊断

仪器与验收中的参数访问、服务发现和原生服务调用都使用调用者墙钟预算。超时表示没有在预算内拿到确定结果，不会自动撤销已经送达的远端请求；不确定的施力请求不自动重放。

读取和施力使用本课作用域关节名 `workshop::slide`；Noetic清队列接口按短名比较，因此清除使用 `slide`。独立世界只有这一个同名关节。清队列不清零已有速度；暂停期间的有限施力任务，也不会靠等几秒墙钟自然消耗完。

| 现象 | 定位方向 |
|---|---|
| 参数服务器已改，实际力没变 | 是启动缓存还是每次读入？检查本次服务采用的值 |
| group与私有值不同 | 检查完整参数路径，而不是只比较末尾键名 |
| 非默认节点有名字却ready超时 | 启动、ready、verify、CLI的namespace是否一致 |
| “1.0”看着合法却被拒绝 | 查它是数字还是字符串，是否被错误加了引号 |
| 非法值导致整个节点退出 | 应在请求边界返回失败，并保留恢复合法值的路径 |
| 比例不接近2 | 比较同一仿真时刻、相同初态、实际力与关节限位；不要只查字典值 |
| roslaunch找不到包 | rospack是否指向个人源包？检查本课ROS_PACKAGE_PATH与入口权限 |

## 独立挑战与验证范围

为一组小车设计统一的launch：每辆车有自己的namespace、group配置和节点私有配置，明确哪些参数只在启动读、哪些允许运行期更新。再设计一项真实运动实验，能区分“服务器值已改”和“控制器确实采用”。命名空间隔离不能替代不同机器人模型与执行器的隔离。

本课自动验收围绕真实参数解析、有限施力与独立关节反馈。关于默认与非默认namespace、scale变体、非法输入恢复、F/2F响应的实际验证状态，以运行报告为准。个人代码和YAML保留；实验记录与参数快照应保存到个人数据目录，不改全局shell。

不熟悉ROS名字解析？[官方名称解析实现](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosgraph/src/rosgraph/names.py)。要看typed参数怎样转换？[roslaunch官方加载器](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/loader.py)。需要真正的动态配置回调时，可进一步学习[dynamic_reconfigure](https://github.com/ros/dynamic_reconfigure)，与本课主动读参策略对照。
