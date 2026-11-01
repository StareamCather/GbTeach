# VisualPlugin：改变看见的外观，核对碰撞是否改变

本课给固定标识块编写 Gazebo Classic 的 `VisualPlugin`。它在真实 Gazebo 客户端中改变一个 visual 的颜色和缩放；服务端模型的位置和一米立方体碰撞几何保持不变。你将同时核对真实渲染画面、客户端接口读回和服务端原生实体，分清三类证据分别说明什么。

这次操作的对象是 `rendering::VisualPtr`，不是第26课的模型、第27课的世界或第28课的传感器。项目不依赖 ROS。只有 `gzserver` 启动时，视觉插件的实验还没有完成：它需要真实 `gzclient`、可用渲染环境和后续的渲染帧。

## 场景与先做的预测

世界名为 `lesson29`。静态模型 `beacon` 位于 `(0,0,0.8)`，link 名为 `body`，目标 visual 名为 `indicator`。它的服务端碰撞是一米立方体。另一只亮黄色参考块与目标分开摆放，用来观察相机或整体画面变化，避免把整幅画面放大误认为只有目标放大。

个人参数的默认值是 `color="red"`、`visual_scale=1.0`。开始前写下三个预测：只改成蓝色是否会改变碰撞；只把 visual 缩放改为1.4是否会改变物理尺寸；旋转或拉近观察相机是否能改变渲染中的像素面积。

最后一个问题说明了为什么不能只凭“看起来大了”验收缩放。比较前后画面时，应使用同一相机、窗口和参考物，并另外读回服务端真实碰撞参数。

## 一份模型，两个作用域

SDF 中的 `visual` 描述渲染外观，`collision` 描述物理接触所用几何。两者常有相同尺寸，但不是同一个运行时对象。模型、link、visual 的层级还会共同影响局部与世界坐标。

| 对象或信息 | 本课的作用域 |
| --- | --- |
| 服务端模型与 link pose | 决定物理实体所在位置 |
| collision 的 box size | 服务端实际碰撞形状 |
| rendering::Visual | 当前客户端的渲染对象 |
| visual 的颜色与局部 scale | 本课插件修改的显示属性 |
| 用户相机 | 决定同一对象如何投影到画面 |
| 截图中的像素 | 证明实际渲染结果，而非物理尺寸本身 |

本例对 `indicator` 设置的是局部绝对缩放，不是每帧再乘一次比例。若原盒子的显示尺寸是一米、父层级缩放为1，那么局部比例1.4会把这一 visual 的显示边长变为约1.4米。父层级存在其他缩放时，还应区分局部 scale 与累积后的 world/derived scale。

不了解 visual 与 collision，可先回看第02、07课；不了解坐标层级，可先回看第03、23课。关于具体渲染对象，可查阅 [Classic Visual API](https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1rendering_1_1Visual.html)。

## 插件应放在哪一层

视觉插件应声明在目标 `visual` 内部，注册类型应为 `GZ_REGISTER_VISUAL_PLUGIN`。下面省略了几何和准备阶段写入的摘要：

```xml
<model name="beacon">
  <link name="body">
    <visual name="indicator">
      <!-- geometry / material -->
      <plugin name="appearance"
              filename="libgazeboteach_l29_visual.so">
        <color>red</color>
        <visual_scale>1.0</visual_scale>
      </plugin>
    </visual>
    <!-- collision 独立声明 -->
  </link>
</model>
```

库文件名、SDF 插件实例名、C++ 类名、目标 visual 的作用域名称各有用途。把同一个库移到 model 下，不会把 VisualPlugin 自动转换为 ModelPlugin；把库编译成功，也不证明目标 visual 已经创建或成功加载它。

初始化分为两段：`after_start` 只核对服务端与个人准备结果，`after_gui` 再等待真实客户端视觉插件就绪。本课自动视觉验收需要 GUI；没有客户端时，服务端诊断仍有价值，但不能标为视觉实验通过。

## Load、Init 与实际渲染回调

`VisualPlugin::Load` 接到 visual 与插件配置，用来保存对象、读取个人初始值、建立消息接口以及保留渲染事件连接。它不应阻塞等待一个永远不会出现的外部交互。

本课实际改变渲染对象的工作放在 `PreRender` 事件中。事件连接保存在成员中；如果只创建局部连接，作用域结束后连接可能解除，表现为插件加载成功但画面不再更新。

`Init` 不能直接套用前面课程的固定计数。Classic 的 `Visual::LoadPlugin` 在 visual 已初始化时会调用插件 Init，`LoadPlugins` 还会遍历当前插件调用 Init。应依据本次实例的生命周期记录解释次数，并让初始化逻辑可重复执行；不能单靠“Init等于1”判断视觉是否正常。

本课命令 `python3 lab.py reset` 表示请求恢复个人模板的颜色和 visual_scale。它是课程自定义操作，不等于顶部“重置模型状态”的服务端模型复位，也不等于让世界模拟时钟归零。基类具有 Reset 方法，同样不代表世界的每种复位路径都会自动调用当前客户端插件的 Reset。

## 为什么消息回调不能直接修改 Ogre 对象

Transport 的接收回调与渲染循环不应被当成同一线程。本课消息回调只检查命令并排队；真正的材质、颜色、缩放和相机操作由后续渲染帧完成。请求被接受与画面已经应用是两个时刻。

运行流程是：

1. 在个人练习中校验参数，生成带配置摘要的 SDF，离线编译插件与原生工具。
2. 启动本课私有服务端，读取真实模型和碰撞；此时不声称视觉插件已经运行。
3. 打开真实客户端，Load取得目标visual；建立PreRender连接与原生消息接口。
4. Transport回调登记期望命令和序号，返回接收结果，不在该线程改Ogre对象。
5. 渲染线程取出待执行命令，为目标创建独立材质，设置颜色、照明模式及局部缩放。
6. 从真实Visual接口读取已应用属性，更新应用序号和渲染状态；工具等待对应应用结果。
7. 用真实截图核对像素变化，同时独立查询服务端原生实体确认物理几何未改。

命令序号用于关联请求与应用，不是画面真值。接口读回证明属性进入了客户端对象；真正截图才能进一步确认画面渲染结果。

## 独立材质与照明

多个 visual 可以引用同一份 Ogre 材质。直接修改共享材质可能让别的物体也变色，造成难以定位的副作用。本课为目标使用唯一材质副本，再设置 ambient、diffuse 和局部 scale；不要通过修改整个场景的共用材质模拟“只改变目标”。

本课目标和黄色参考物开启材质照明，同时设置ambient、diffuse、emissive以使颜色清楚。emissive让该表面显示自身颜色，不等于创建能照亮邻物的世界光源。整套操作是渲染显示策略，不改变碰撞、质量或重力；最终像素仍受到具体渲染路径影响，不能把每个像素都要求为输入RGBA的精确字节。

本次实机初稿曾出现一个有意义的反例：actual_diffuse已经读回红色，相机和初始化也已完成，但实际Gazebo窗口中的目标与参考物仍为白色。关闭lighting没有在该渲染路径中呈现预期颜色。配置据此改为开启lighting并保留对应颜色，像素验收标准不变；是否真正显示正确仍须看新配置的实际截图，不能依据一次API赋值成功作结论。

不了解材质，可先回看第13课，并查看 [Classic Visual 实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/rendering/Visual.cc) 中 SetMaterial、SetLighting 与 SetScale 的不同职责。

## 本课文件

| 文件 | 用途 |
| --- | --- |
| world.sdf | 离线场景、目标visual和独立collision |
| parameters.json | 个人初始颜色与视觉缩放 |
| visual_plugin.cc | VisualPlugin、命令排队和渲染更新 |
| observer.cc | 原生消息、实体查询与关联请求响应工具 |
| prepare.py / config.py | 输入检查、源摘要、个人SDF与编译缓存 |
| lab.py | 有限实验、视觉命令与状态观察 |
| verify.py | 课程验收入口 |
| reference.md | 只读推理和挑战提示 |

本课构建需要Gazebo Classic 11、OGRE和Qt5Widgets开发接口，以及g++与pkg-config。能打开Qt桌面应用不等于已经安装对应的C++头文件；缺少开发接口时应查看实际pkg-config或编译错误。

指针类型也以当前SDK定义为准：本次VisualPtr使用std共享指针，UserCameraPtr则使用boost共享指针，不能因名称相似便混用对应的弱指针类型。

构建缓存依据源码、SDK和编译器等输入建立，按键锁定并原子安装。修改个人参数或C++源后应重载项目，实际重新准备；保存文件不等于已有客户端自动加载新库。工具会核对准备摘要和实际实例，避免把旧画面当作新配置的结果。

## 实验一：同一画面中只改变颜色

先点击“开始实验”。等待服务端、Gazebo窗口、终端和客户端就绪，再读取实际视觉状态。记录个人初始颜色、visual scale、客户端身份和当前已应用命令序号。

在本课实验终端中运行：

```sh
python3 lab.py ready
python3 lab.py pause
python3 lab.py color --color blue
python3 lab.py color --color red
python3 lab.py watch --seconds 2
```

世界暂停时，客户端仍可处理渲染帧和颜色命令。暂停的是物理世界推进，不是把所有客户端渲染工作一起停止。分别记录世界时间与render_count，不要求两种计数同步增长。

| 实际状态字段 | 解读 |
| --- | --- |
| configured_color / configured_visual_scale | 本次个人模板初始值 |
| color / visual_scale | 当前命令所用外观值 |
| actual_diffuse_rgba / actual_scale_xyz | 真实Visual接口读回 |
| queued_command_seq / applied_command_seq | 已入队与渲染线程已处理的序号 |
| captured_command_seq | 真实相机帧保存完成的关联序号 |
| capture_armed / capture_image_events | 是否等待新图像、实际新图像事件计数 |
| capture_width / capture_height | 新图像事件提供的实际尺寸 |
| client_id / render_count | 当前客户端实例及渲染回调计数 |
| camera_initialized / camera_pose | 相机初始化状态及真实姿态读回 |

当前命令值和接口回读仍不能代替真实像素检查。例如改了一个不可见的visual，其属性可能正确，画面中的目标却不变。

保持相机不动，把颜色切换为蓝色，再恢复红色。预测黄色参考物是否应随之改变。对照以下三层信息：请求是否被接受、实际客户端属性是否已应用、真实画面的目标像素是否变化。

若命令成功返回但颜色未变，不要立刻反复发送命令。先检查对应应用序号是否已出现、渲染帧是否继续、是否看错了其他Gazebo窗口、是否编辑参数后忘了重载。

## 实验二：外观变大，碰撞是否变大

恢复固定相机，记录基准截图和服务端目标实体。只把 visual scale改为1.4，等待实际应用，再记录第二幅截图。

```sh
python3 lab.py camera
python3 lab.py scale --scale 1.0
python3 lab.py capture
python3 lab.py scale --scale 1.4
python3 lab.py capture
python3 lab.py reset
```

capture默认使用个人build目录中的新文件名；也可通过 `--path` 指定新的PNG路径，已有文件不会被覆盖。它保存真实UserCamera取得的像素，不按预测颜色生成图片。接收请求后，PreRender保存原CaptureData状态、保留ConnectNewImageFrame连接并开启像素读回；Camera完成PostRender读回后，新图像事件提供真实buffer、尺寸、深度与格式，工具才用Camera的静态SaveFrame入口写PNG，并恢复原CaptureData状态。

这里不靠“又渲染了若干帧”猜测图像已就绪。用户相机默认可能没有保留ImageData；早期直接调用保存接口曾得到空图像缓冲，即使画面本身能正常显示。因此要分别记录接受请求、读到非空新图像事件、实际保存完成三个阶段。独立维护验收还会从本次Gazebo窗口另抓像素，不把插件提供的图片作为唯一真值。

目标的像素范围应增大，参考物和相机应保持一致；服务器端的一米box collision及模型pose应保持原值。像素面积通常不会精确等于比例平方：透视、物体深度、可见面、离散像素边界与遮挡都影响面积，因此应结合已知相机、可见范围和原生几何判断。

可以在GUI中打开碰撞几何显示，人工观察放大的外观与原始碰撞轮廓是否重合。这个操作是额外图形练习，不能用服务端启动成功代替人工观察，也不能把本课的纯视觉缩放当作一种物理缩放命令。

## 实验三：相机也是实验变量

用鼠标围绕目标旋转并拉近视角。颜色或局部scale并未因此改变，但目标的像素位置和大小可能明显变化。再使用课程的camera命令回到固定视角。

课程在客户端初次就绪时初始化一次相机，随后允许你自由观察；不会每帧把相机强制拉回。做像素对照前主动回固定视角，避免把相机变化混进颜色或缩放的单变量实验。

```sh
python3 lab.py camera
python3 lab.py demo
python3 verify.py
python3 lab.py resume
```

demo与verify执行同一有限对照：暂停物理世界、回固定相机、在scale1依次显示红/蓝/绿并保存真实帧，再把绿色目标放大到1.4，与黄色参考物比较像素面积并查询原生服务端实体。正常或异常结束时会尝试恢复入场颜色、scale和暂停状态；相机留在课程固定视角，不恢复你此前手动设置的视角。模拟时钟、渲染帧和命令计数不作为快照回填。

## 两个个人单变量练习

练习A：保留visual_scale=1.0，只把初始color改为blue并重载。对照红色模板和蓝色个人配置，确认只有目标外观的颜色改变。

练习B：恢复color=red，只把初始visual_scale改为1.4并重载。保持窗口和相机相同，比较目标与参考物的画面范围，另外确认服务端碰撞仍为一米立方体。

不要同时改变颜色、scale和相机。把“输入参数”“实际客户端属性”“真实截图”“服务端物理状态”四项分别记下，再判断差异来自哪一层。

## 验收应该证明什么

加载成功、接口读回与实际渲染是不同检查。课程工具会执行有限的外观变化和状态观察；独立维护验收还应读取本次真实Gazebo窗口的像素，并分别核对服务端实体及插件实际进程映射。

VisualPlugin应加载在本次客户端中，不能只看到某个同名.so文件便宣称加载。客户端关闭后，应明确报告视觉通道不可用；重新打开GUI会建立新的客户端视觉实例，不能沿用旧实例的身份和应用序号解释新窗口。

课程辅助SDK工具沿用当前Classic 11.15.1验证过的主线程IO停止与join流程。它对共享IOManager的兼容封装有明确版本和符号检查，符号或动态库全版本不符时拒绝。插件教学本体使用正常Visual接口，不操作辅助工具的私有ABI。

## 常见故障与独立挑战

- 只有服务端：先打开本课Gazebo窗口；没有真实客户端不能通过视觉验收。
- 找不到库：检查个人构建输出、插件搜索路径、SDF filename和实际编译错误。
- 父对象不对：检查plugin是否放在indicator这个visual内，以及注册宏类型。
- 有请求没有应用：检查渲染连接是否仍保留，客户端是否存活，队列是否等待后续帧。
- 其他物体也变色：检查材质是否仍共享，或是否把修改级联到了不应修改的对象。
- 看起来变大但原因不明：回固定相机，核对窗口、参考物、实际scale与服务端collision。
- 修改源后仍是旧结果：重载项目，核对新编译摘要与新客户端实例。

独立挑战：在保留默认实验可复现的前提下，设计一个真实状态驱动的视觉指示器。例如订阅第28课的near/clear/unknown，把状态映射为不同颜色。先定义消息缺失、未知状态、旧时间和客户端重开后的处理规则，再在渲染线程应用；不要直接在接收线程修改Ogre对象，也不要把默认纯色实验通过当作扩展通信链已经通过。

进一步可设计按模拟时间或墙钟闪烁的指示器。先说明暂停世界时它是否继续闪烁，以及为什么；随后用实际时间与渲染帧验证。默认课程不提前替这个扩展选定时间语义。

辅助SDK进程在初始化Transport之前还会核对编译SDK完整版本必须为11.15.1；运行库DSO必须与该完整版本相同。其他Classic 11小版本不能沿用这段私有ABI兼容逻辑。退出时仍先fini，再由持有Connection的主线程Stop并join IO，最后释放通信对象；这项版本拒绝不改变插件或实验的物理更新流程。
