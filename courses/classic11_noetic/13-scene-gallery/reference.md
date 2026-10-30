# 场景展厅：只读参考与测量边界

本课仅在新建或重新打开 gzclient 后激活本会话窗口、做一次可恢复的尺寸刷新并执行总览初始化，最终仍通过实际相机输出确认；已打开的窗口和后续手动视角不会重复初始化，headless 运行跳过该步骤。

此页给出推导、示例答案与信息流说明。查看它不会改写个人world.sdf；恢复起点请使用平台“备份并恢复起点”，它会先备份当前练习。以下计算标为几何或脚本预测，实际观测以本轮原始数据和真实GUI画面为准。

## 文件与数据如何走到画面和物理

| 输入或输出 | 职责 |
| --- | --- |
| world.sdf | 个人真实场景输入，含visual、light、collision heightmap和Actor脚本 |
| assets/heightmap.png | 固定65×65数值高度图；尺寸、灰度、CRC均检查 |
| 系统Gazebo资源 | Gazebo/Wood脚本与纹理、walk.dae骨骼网格和动画，离线读取 |
| dirt_diffusespecular.png | 系统地表diffuse纹理，决定地形的渲染颜色 |
| flat_normal.png | 系统平坦法线贴图，参与光照而不改碰撞 |
| lab.py | 检查、原生状态观察、显式局部修改、真实GUI相机请求 |
| verify.py | 平台调用的完整实验验收入口 |
| control/gallery-frames.jsonl | 约50 Hz的Actor、射线与场景概览，须按真实时间与动画更新次数解读 |
| control/probe-tail.jsonl | 3到3.5仿真秒内，每真实1 ms物理步末的六球快照 |
| control/status.json、verification.json | 状态与本轮验收结果，分别位于control目录 |

流程是：保存的SDF与资源先经过合法性检查；Gazebo加载同一份个人输入；服务器物理回调读取真实高度场、射线、球与Actor；GUI独立加载视觉资源并渲染。相机工具向Classic `~/gui`发布gazebo.msgs.GUI，再从`~/user_camera/pose`读取真实相机姿态。后者是输出，不应向它发布“移动相机”命令。观察器读取物理数据，网络回调不直接冒充物理更新。

set-height、set-cycle、set-daylight先验证再只改对应SDF字段，提示reload_required；不会同时改其它允许的材质设置，也不会在运行时偷偷应用。check-config回答“保存文件是否可用”，inspect回答“当前加载了什么、实际状态如何”。保存输入与加载指纹不一致时，应重载而不是继续解释旧画面。

## 材质：渲染参数与物性分开

ambient、diffuse、specular、emissive都是渲染材质参数；摩擦与接触参数属于collision/surface，质量惯量属于inertial。相同collision上的两个球，在相同物理条件下不应因为木纹或RGB颜色不同而获得不同支撑高度。

matte_support与emissive_support两个立方体边长0.6 m，中心z=0.3 m，顶面z=0.6 m。球半径r=0.1 m，水平顶面静止接触的几何预期为：

```text
z_ball = z_top + r = 0.6 + 0.1 = 0.7 m
```

有限步长与接触求解允许小误差，验收还检查水平偏移和速度；不能只找到某一时刻z≈0.7就说已落稳。改材质后仍采用同一判据，不另给不同颜色设置不同物理容差。

ambient是环境光近似的材质响应，diffuse描述漫反射，高光取决于specular以及灯光和观察方向，emissive是自发亮外观。光泽球固定shininess=32，这是经典反射模型的高光指数，越大通常越集中。Classic并不因为emissive非零就进行相邻表面的全局光照求解。SDF字段存在或资源元数据正确，也不能证明实际像素已经呈现了期望效果；高光与纹理须在真实GUI检查。[Classic颜色与纹理](https://classic.gazebosim.org/tutorials?tut=color_model)提供基础介绍，[11.15.1 Visual实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/Visual.cc)可供深入核对实际材质路径。

脚本材质分为资源URI与材质name：前者定位脚本/纹理目录，后者选择脚本中的声明，例如Gazebo/Wood。GAZEBO_MODEL_PATH查模型集合，GAZEBO_RESOURCE_PATH查世界、媒体等资源；它们不是同一个变量。平台先加载系统Gazebo环境再追加本课路径，保留系统shader和纹理资源。不要为了修一个纹理把整个资源路径覆盖成个人目录。详见[官方组件与路径说明](https://classic.gazebosim.org/tutorials?tut=components)。

## 灯光：比较条件与单位

directional主要由方向决定；point和spot还随位置、范围与距离衰减。常见衰减形状为`1/(c+l·d+q·d²)`，d是到灯的距离；range限制有效范围，spot再加入锥形分布。不能把这组渲染系数直接称为经过标定的照度或流明。[SDF灯光字段](https://sdformat.org/spec/1.7/light/)与[OGRE Light接口](https://ogrecave.github.io/ogre/api/1.11/class_ogre_1_1_light.html)说明了这些概念。

本课的constant、linear保持在[0,1]，因为Classic运行时会夹取这两项；不要把文件中一个越界值当作一定被原样采用。聚光的inner/outer angle使用rad，inner≤outer；不同渲染路径的光锥过渡和阴影细节应实际看图，不由字段存在推导像素边界。

日光diffuse从0.8变0.2时，其它场景光、环境光和材质保持不变。合理预测是日光贡献减小，不是全图每个RGB值严格缩成1/4。emissive区可能保持较亮，阴影、其它灯和材质响应使不同区域变化不同。比较应使用同一相机、同一物体区域，避开移动Actor；跨GPU不要求整张截图hash相同。

阴影涉及场景全局阴影、灯cast_shadows和visual投影设置，也受具体渲染配置影响。碰撞由物理几何决定；有阴影不能证明有碰撞，无阴影也不能证明没有碰撞。Classic的directional、point、spot都有相应投影实现路径，实际可见效果须看本设备GUI，而不是沿用旧版本的一条限制。[11.15.1 Light实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/Light.cc)是进一步阅读入口。

## 高度场：像素、顶点、射线和落体

本课只使用一张真正的collision heightmap。PNG为65×65、8位灰度，包含0和255，四个灰度平台以行/列索引32为分界。visual与collision均为sampling=1，XY尺寸4×4 m，模型/连杆/碰撞原点与heightmap pos均为零。由此顶点间距为0.0625 m；图像像素(u,v)对应：

```text
x = -2 + 0.0625u
y = +2 - 0.0625v
h = (pixel/255) × sizeZ
```

这里的Y符号与PNG行从上向下增加有关。四个平台查询顶点依次为(u,v)=(16,16)、(48,16)、(16,48)、(48,48)，对应下表的位置。h公式依赖本课最大灰度255与固定原点；原生加载器按图像实际最大值缩放。任意图像的“灰度128”并不天然等于sizeZ的一半。GetHeight读取索引处的局部顶点高度，不是任意世界XY查询；ODE在三角网格内的插值也不能被笼统称为任意双线性碰撞插值。参见[HeightmapShape](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/HeightmapShape.cc)和[ImageHeightmap](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/common/ImageHeightmap.cc)。

地形的外观由visual/geometry/heightmap下专用的texture层提供。这里的diffuse是地表颜色图像，normal是法线贴图，编码用于光照计算的局部表面方向；两者都不会改写高度顶点或collision。normal贴图可以改变明暗细节而不产生物理凹凸，单层平坦法线贴图则不额外添加凹凸细节。OGRE地形路径不应靠普通visual外层的material颜色替代这套纹理配置。

本课直接引用`/usr/share/gazebo-11/media/materials/textures/dirt_diffusespecular.png`与同目录的`flat_normal.png`，不复制到个人模型包；单层texture的size=1 m控制纹理铺设尺度，不是高度图分辨率或地形高度。两文件来自已安装gazebo11-common，安装包声明Apache-2.0，保留系统引用及上游出处：[地表颜色图](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/media/materials/textures/dirt_diffusespecular.png)、[平坦法线图](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/media/materials/textures/flat_normal.png)。资源路径、hash与用途随本课资源记录保存。

四个球位于宽平台内部，避开过渡斜面；真实射线在每个球旁X偏移0.35 m处探测，避开球本身及其它展品。下表是三组输入的几何预测，均包含球半径0.1 m：

| 平台XY/m | 灰度 | size.z=.6 m时h/m | .6 m时球中心z/m | .9 m时球中心z/m | .75 m时球中心z/m |
| --- | --- | --- | --- | --- | --- |
| (−1,+1) | 85 | .2 | .3 | .4 | .35 |
| (+1,+1) | 170 | .4 | .5 | .7 | .6 |
| (−1,−1) | 0 | 0 | .1 | .1 | .1 |
| (+1,−1) | 255 | .6 | .7 | 1.0 | .85 |

set-height改变的是同一高度图在visual与collision中的size.z，两者必须一致，允许范围0.3到1.0 m。XY、URI、sampling和固定探针拓扑不随这个挑战改变；不会通过替换一张未说明的图来凑结果。65=2^6+1满足图像边长要求，检查还覆盖PNG尺寸、位深、类型与CRC；黑色平台也必须由真正的地形接住，不能让额外floor碰撞掩盖错误。

2^n+1是数据格式条件，还须满足当前渲染器的LOD批次约束；LOD指按观察距离选择细节层次。本课所用[Classic 11地形渲染实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/Heightmap.cc)固定minBatchSize=17、maxBatchSize=65，实际绘制dataSize至少为65；更小的合法高度图仍可能触发TerrainQuadTreeNode断言。这是地形数据与渲染批次的兼容要求，应检查图像分辨率和采样设置，不能仅凭服务器解析成功判定GUI可用。

每个球在测量窗口须连续至少0.2仿真秒满足：中心高度距几何预期≤0.01 m，水平偏移≤0.03 m，速率≤0.03 m/s。稳定证据使用probe-tail中至少201个连续真实1 ms步末快照，首末跨度至少0.2 s；不能把约50 Hz概览里的少数点当作逐步检查。射线须命中实际地形实体，表面高度误差≤1e−4 m。原生URI/Size/Pos/采样数/顶点与高度数据说明加载了什么，射线说明该处物理表面，球说明动力学落稳；三者互补，不用同一条回显重复充作三个独立证明。

## Actor：零切线样条并非匀速

人物和标记沿两个固定通道往返，端点x为−1、+1 m。两者共有的关键时刻是当前周期T乘以`0,5/12,1/2,11/12,1`；x依次为`−1,+1,+1,−1,−1`。人物在端点转身，标记始终保持单位朝向。周期4到12 s可调，但本课只改全部waypoint时间，不改路径点、通道或脚本开关。

| 周期T/s | 五个关键时刻/s | 第一个移动段的中点时刻/s |
| --- | --- | --- |
| 6 | 0，2.5，3，5.5，6 | 1.25 |
| 8 | 0，10/3，4，22/3，8 | 5/3 |
| 9 | 0，3.75，4.5，8.25，9 | 1.875 |

位置样条的tension=1使关键点切线为零。对于标记的一段移动，令u=(当前ScriptTime−段起点时间)/段时长，在0≤u≤1内：

```text
s(u) = 3u² - 2u³
x(u) = x0 + (x1-x0)s(u)
dx/dt = (x1-x0)(6u-6u²)/段时长
```

第一段从−1到+1 m，u=.25时x=−.6875 m，u=.5时x=0，u=.75时x=.6875 m；并不是匀速插值的−.5、0、+.5。T=6 s时移动段长2.5 s，最大速度在中点，为1.2 m/s；T=8时为0.9 m/s，T=9时为0.8 m/s。这里是固定标记路径的数学预测，不是对人物脚速或真实机器人行走速度的测量。实现链可看[PoseAnimation](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/common/Animation.cc)与[Spline](https://github.com/gazebosim/gz-math/blob/ign-math6/src/Spline.cc)。

验收用标记的实际ScriptTime核对路径，而不是把1 ms世界时钟直接当作每次动画更新时间。原生Actor更新最多约30 Hz，观察行重复姿态是可能的，报告应区分总行数与不同动画更新数。标记可与上述路径严格对照；人物的原始WorldPose还可能包含皮肤骨骼根的起伏或旋转，不应要求它的六维姿态逐分量等于waypoint。

人物骨骼数据另看LeftFoot/RightFoot相对Hips的变化；网格中存在节点和动画定义还不够，必须看到真实相对姿态改变，并在GUI两个时刻看到腿部变化。只显示一个沿路移动的方盒不能替代骨骼动画目标。路径移动与骨骼动作的组合、动画名匹配、interpolate_x等背景见[官方Actor教程](https://classic.gazebosim.org/tutorials?tut=actor)和[SDF 1.7 Actor](https://sdformat.org/spec/1.7/actor/)。

本课固定loop=true、auto_start=true、delay_start=0，采用世界暂停/继续；Play重设动画开始计时，不等价于恢复世界时钟。Actor通过动画更新姿态，不由本课PD、关节力矩或接触反馈生成步态，不带通用避障能力。运行时骨骼可能带碰撞几何，static、kinematic、gravity和collision计数按实际报告；不能从“动画人物”推导所有引擎和所有Actor都具有相同接触行为。

## 三个挑战的示例答案

1. 高度0.75 m：四个平台为0.25、0.5、0、0.75 m，球中心为0.35、0.6、0.1、0.85 m。双胞胎的碰撞顶面仍是0.6 m，两个球中心预期仍为0.7 m。
2. 周期9 s：关键时刻0、3.75、4.5、8.25、9 s；第一移动段中点1.875 s，x=0。速度先增后减，中点达到最大值0.8 m/s。周期改变的是脚本时间表，不是物理步长或必然相同的骨骼步频。
3. 只改visual材质：颜色、高光或自发亮外观可变；质量、惯量、碰撞形状、接触摩擦及水平支撑高度不因该修改自动改变。选择相同视角与日光，避免把另一盏灯、相机变化或Actor移动混入比较。

先预测的意义是为观察指定问题：若预期几何高度正确而GUI山形错误，查渲染资源；若画面像山而射线/球不符，查collision；若文件正确而运行指纹旧，重载；若标记路径正确而人物肢体没变，查骨骼动画而非把全局位移再测一遍。

## 完整验收流程与限制

1. 校验保存输入、固定结构、资源文件与格式；不合法输入在开始控制/测量前拒绝，不自动修复个人world。
2. 核对实际加载指纹，通过正常模型复位建立新轮次，读取实际暂停/运行状态并推进仿真；动态球不由helper写位置或速度。
3. 在物理所属更新回调读取原生高度场、物理射线、六个球及Actor，持续覆盖至少2.1个当前周期；保留时间、原始姿态与不同动画更新计数。
4. 检查地形与双胞胎球的稳定支撑、标记脚本路径、人物根移动和真实骨骼变化，写入原始行及分项报告；确认或恢复原暂停状态，不还原旧运动或动画相位。
5. 另在真实GUI检查总览、材质、第二高光视角、灯光/阴影、地形与探针、两个时刻的人物；日光0.8/0.2使用同相机对照。原生配置字段和headless成功只作佐证，不能代替这一步。

相机readback只证明GUI采用了该相机位姿；资源SHA只证明字节身份；原始骨骼数据只证明服务端动画状态。是否实际看见木纹、高光和肢体，还要看渲染画面。预设视角也不自动证明学员掌握了手动鼠标操作。

walk.dae来自已安装gazebo11-common资源`/usr/share/gazebo-11/media/models/walk.dae`，本课引用系统文件，不把它纳入本仓库MIT资产重新授权。它的网格与动画沿用上游Apache 2.0许可；木纹等系统资源同样应保留所属包与上游出处。本课无需联网下载模型库，也不要求学员制作或导出DAE。
