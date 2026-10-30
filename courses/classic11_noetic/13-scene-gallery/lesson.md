# 场景展厅：画面变化，物理世界改变了吗

每次新建或重新打开本课的 Gazebo 图形窗口时，平台会激活本会话窗口并做一次可恢复的尺寸刷新，再等待真实相机确认并切到展馆总览；已有窗口中随后由你调整的视角不会被反复覆盖，无 GUI 运行时不执行这一步。

## 用四个展台回答同一个问题

把木纹贴在立方体上，它就具有木材的摩擦和质量了吗？把地板画成山坡，球就会沿坡滚动吗？一个人物走过画面，是否意味着它正在靠关节力矩行走？本课用一个离线展厅，把**渲染外观、碰撞几何和动画姿态**逐项分开。

四个展台分别展示材质、灯光、真实高度场地形和Actor。工具与资源已经准备好，不要求先学ROS、制作骨骼网格、编写shader或C++插件。你需要做的是提出预测，修改个人world.sdf，保存并重载，再比较真实画面与实际物理数据。

| 展台 | 看什么 | 用什么证明 |
| --- | --- | --- |
| 材质 | 颜色、高光、自发亮、木纹 | 实际GUI画面；两个相同碰撞面的球是否仍停在同一高度 |
| 灯光 | 平行光、点光、聚光、衰减与阴影 | 固定相机的实际照明区域和阴影变化 |
| 地形 | 灰度高度图、表面纹理、碰撞高度场 | 原生高度数据、物理射线、四个真实落体球 |
| Actor | 整体路径与骨骼动画 | 实际根位姿、脚相对髋部的姿态，以及不同时刻的真实画面 |

本课使用Gazebo Classic 11的渲染与SDF 1.7语义。其它Gazebo版本的PBR、渲染引擎或消息接口不能仅凭相似名称直接套用。

## 先找到个人输入和真实视角

world.sdf是本课真正的个人输入。“编辑练习文件”打开它；保存后点“重载项目”才会重新加载资源与场景。准备工具检查资源和格式，不会把你改过的world.sdf悄悄恢复成模板。assets/heightmap.png是固定的离线数值资源；本课改变SDF中的高度缩放，不随意重画这张图。

开始课程后，在本课实验终端运行：

```sh
python3 lab.py check-config
python3 lab.py inspect
python3 lab.py view --station overview
```

check-config检查保存的输入；inspect读取当前已加载场景与实际状态。二者用途不同：文件检查通过，不代表运行中的场景已采用新文件。view向真正的GUI发送相机请求，并等候实际相机位姿读回；没有连接的GUI时应明确失败，不能把命令退出当成已经看到画面。

可用视角为overview、materials、specular、lights、terrain、actors。也可以在Gazebo的WORLD面板展开GUI调整相机，或直接用鼠标观察；不了解界面操作，可看[Classic GUI教程](https://classic.gazebosim.org/tutorials?tut=guided_b2)。世界暂停时，GUI仍能渲染和移动相机。

## 第一站：材质没有替你改物理属性

**材质（material）**规定表面如何参与渲染；**纹理（texture）**是映射到表面的图像。它们与collision中的碰撞形状、surface中的摩擦参数以及inertial中的质量惯量分属不同输入。给立方体换上木纹，不会自动赋予木材的密度、摩擦或弹性。

Classic常见的四项颜色如下，RGBA每项范围为0到1；RGB控制颜色分量，A是透明相关分量。本课先保持A=1，集中研究照明。

| 字段 | 一句话解释 | 观察要点 |
| --- | --- | --- |
| ambient | 表面对场景环境光近似的响应 | 不能把它当成独立灯源 |
| diffuse | 随表面法线与入射光方向变化的漫反射颜色 | 同色物体的不同面也可能明暗不同 |
| specular | 随观察和入射方向变化的镜面高光 | 移动相机比较，不能只看一个角度 |
| emissive | 表面自身的发亮外观 | 不会因此照亮旁边物体 |

先运行`python3 lab.py view --station materials`，辨认哑光、发亮与木纹样本；再用`python3 lab.py view --station specular`换一个角度看高光。光泽球采用经典高光指数shininess=32；在这类反射模型中，指数越大，高光通常越集中。场景光的颜色、相机方向和材质共同决定画面；颜色字段的数值不是测得的屏幕亮度。[Classic颜色与纹理教程](https://classic.gazebosim.org/tutorials?tut=color_model)可作为先修，实际版本效果以本课真实GUI为准。

找到matte_support与emissive_support两个材质不同、碰撞形状相同的立方体。它们的碰撞面都是边长0.6 m的箱体，顶面z=0.6 m；上方球的半径均为0.1 m。预测两个球落稳后的中心高度，再看实际数据。在world.sdf中查找模型名，沿link→visual name="surface"→material找到颜色字段，记录原值后只改diffuse或emissive，保持collision不变。保存、check-config、重载后，外观可以变化，球的支撑高度预期仍是0.7 m；比较完恢复原颜色，再进入下一项对照。

木纹使用离线OGRE材质脚本Gazebo/Wood。**材质脚本**是渲染器读取的一组材质设置，不是物理控制程序。script中的URI帮助查找脚本和纹理，name选择脚本里声明的材质；文件名正确而材质名错误，也可能显示成白色。不了解资源目录，可回看第04课，并参考[官方模型目录说明](https://classic.gazebosim.org/tutorials?tut=model_structure)与[SDF材质字段](https://sdformat.org/spec/1.7/material/)。

## 第二站：灯光影响画面，不产生碰撞面

**光照**描述灯光如何与表面材质共同形成画面；**阴影**是光线受到遮挡后的渲染结果，并不是碰撞体的显示方式。

| 灯类型 | 主要空间特征 | 有意义的单变量 |
| --- | --- | --- |
| directional，平行光 | 各处光线方向相同，不按灯到物体的距离衰减 | 方向、颜色、是否投射阴影 |
| point，点光 | 从一个位置向周围发光 | 位置、照射范围、距离衰减 |
| spot，聚光 | 沿给定方向照亮锥形区域 | 方向、内外锥角、距离衰减 |

运行`python3 lab.py view --station lights`。先辨认三种灯与接收光照的表面；场景日光也会照到局部灯区域，因此不要把亮度全部归因于最近的一盏灯。聚光锥角使用rad，outer_angle不得小于inner_angle；衰减的constant、linear与quadratic是三个系数，不是三种灯。参数说明见[SDF灯光规范](https://sdformat.org/spec/1.7/light/)。

做一次受控的日光对照：先在默认日光diffuse=0.8时，用materials视角观察并记录；然后运行：

```sh
python3 lab.py set-daylight --value 0.2
python3 lab.py check-config
```

这个命令只显式修改个人SDF中日光的中性diffuse字段，并提示需要重载。点“重载项目”，再次切到相同materials视角。预测漫反射区域会变暗，而emissive样本仍可显得亮；旁边物体不会因此得到一盏新灯。环境光和其它灯仍存在，所以不能要求每个像素恰好变成原来的四分之一。

观察阴影时同时考虑场景全局阴影、灯的cast_shadows以及投影物体visual的设置。不要因为某个阴影不明显就认定没有碰撞，也不要根据旧教程断言点光一定不能投影。完成对照后，用`python3 lab.py set-daylight --value 0.8`保存恢复，再重载。

## 第三站：高度图必须同时进入视觉与碰撞

**高度场（heightfield）**在规则XY网格的每个顶点记录一个高度；本课用65×65的8位灰度PNG作为高度图，覆盖4×4 m，visual与collision均为sampling=1。相邻顶点的水平间隔是4/(65−1)=0.0625 m。地形表面的彩色纹理只改变外观；heightmap几何才定义形状。

高度图、diffuse纹理与normal法线贴图分别承担不同任务：高度图决定地形几何；diffuse图像提供地表颜色；normal图像编码参与光照的表面方向，让光照响应呈现细节。法线是垂直于表面的方向，法线贴图不会把这些视觉细节变成碰撞凹凸。本课的地形使用heightmap自己的texture层，普通visual外层的material颜色不能替代这一层。地表离线引用系统gazebo11-common的dirt_diffusespecular.png与flat_normal.png，无需下载资源。

本课图像明确含灰度0与255，四个宽平台采用0、85、170、255。保持这张图、原点与XY尺寸不变时，高度可按`灰度/255 × size.z`计算。这个简式依赖本课的黑白端点；引擎会按图像实际最大值归一，换一张最大灰度不同的图不能盲用同式。不了解DEM，可看[Classic高程模型教程](https://classic.gazebosim.org/tutorials?tut=dem)；本课无需安装GIS工具或下载地图。

先运行`python3 lab.py view --station terrain`。若想完整看到下落，先暂停世界、点“重置模型状态”，确认视角后继续。球可能在GUI准备好之前就已落稳，不能把“没看到它下落”当成没有重力。

默认size.z=0.6 m，四个球的理想几何预期如下，均不是实测数值：

| 球的XY位置/m | 平台灰度 | 表面高度/m | 半径0.1 m球的落稳中心z/m |
| --- | --- | --- | --- |
| (−1,+1) | 85 | 0.2 | 0.3 |
| (+1,+1) | 170 | 0.4 | 0.5 |
| (−1,−1) | 0 | 0 | 0.1 |
| (+1,−1) | 255 | 0.6 | 0.7 |

画面中的大地板可以只有visual，不能替代真实地形碰撞。本课在地形区不另放会接球的碰撞平面。工具分别读取原生高度场数据、让物理射线命中地形、观察六个球靠物理接触落稳；这些证据各自回答不同问题。

只改地形高度，先手算size.z=0.9 m时四个球的位置，再运行：

```sh
python3 lab.py set-height --value 0.9
python3 lab.py check-config
```

重载后inspect，切terrain视角，再进行观察或验收。set-height同时修改visual和collision的heightmap尺寸，不改变其它展台；不能只把山画高而保留原来的碰撞山。此时四个平台预期为0.3、0.6、0、0.9 m，球中心再各加0.1 m，材质双胞胎的球仍预期在0.7 m。完成后用`python3 lab.py set-height --value 0.6`恢复并重载，使下一站只改变周期。

## 第四站：整体走动与骨骼走动是两件事

**Actor**是可按脚本更新姿态的特殊模型。**根位姿路径**控制整体在世界中的位置与朝向；**骨骼动画**改变网格内部骨骼的相对姿态。人物平移但腿不动，与人物原地摆腿，都是可能的组合，不能只凭人物x变了就说骨骼动画成功。

运行`python3 lab.py view --station actors`。人物walker采用系统离线walk.dae；旁边的marker是只有简单外观的小标记Actor，用来分离精确路径与人物骨骼引起的起伏。人物原始WorldPose可能含骨骼根的上下起伏或转动，不能把它的全部六维数值硬当成waypoint本身。工具另外观察脚相对髋部的变化，GUI中也应实际看到肢体变化。[Classic Actor教程](https://classic.gazebosim.org/tutorials?tut=actor)介绍了两类动画。

默认两条路径都是6 s循环：在脚本时刻0、2.5、3、5.5、6 s，x依次为−1、+1、+1、−1、−1 m；人物还在端点转身，标记朝向保持不变。tension=1使位置样条在关键点的切线为零，移动段会加速再减速，**不是匀速直线运动**。Actor原生更新最多约30 Hz仿真频率，连续两行观察数据可能属于同一次动画更新；应同时查看世界时钟和实际ScriptTime。

预测把周期变成8 s后的关键时刻：

```sh
python3 lab.py set-cycle --value 8
python3 lab.py check-config
```

重载后仍是同一路径，所有waypoint时间按8/6缩放；这不是把物理步长改成原来的8/6，也不能据此断言每个迈步周期必为8 s。分别观察人物根的位置、脚相对髋部的姿态、标记的路径，再暂停世界检查画面，继续后看动画恢复。完成后用`python3 lab.py set-cycle --value 6`保存恢复，并重载。

本课保留loop=true、auto_start=true、delay_start=0和既定路径拓扑，只用世界暂停/继续观察；Actor的Play会重新计时，不是世界暂停的替代品。脚本动画直接更新姿态，不是靠质量、关节力矩与接触求解出来的机器人步态，也不自动避障。运行时可能存在骨骼碰撞几何，不能由“看起来是人物”或碰撞数量推断通用接触行为。

## 完整观察、验收与实验记录

先恢复准备比较的单变量配置，保存、check-config并重载；不要在同一次测量中改SDF、拖动物体或发其它控制命令。

```sh
python3 lab.py inspect
python3 lab.py observe
```

observe与有限演示demo会通过正常模型复位建立新的采样轮次，运行至少2.1个当前Actor周期，再确认或恢复进入前的暂停状态。它们读取真实物理和动画数据，不给动态探针写预定位置或速度。结束不会还原原先的球运动、Actor相位或仿真时间；若原来在运行，世界会继续运行。

“验收实验”检查资源、已加载输入、高度场、落稳球与实际Actor采样。control/gallery-frames.jsonl约50 Hz记录Actor、射线与场景概览，Actor本身仍最多约30 Hz更新；落球在3到3.5仿真秒的稳定窗口另以每真实1 ms一步记录到control/probe-tail.jsonl，用至少201个连续步末快照、覆盖至少0.2 s证明稳定。状态在control/status.json，验收报告在control/verification.json。**物理与元数据检查通过，不等于视觉学习目标已经完成**：还要在真实Gazebo窗口查看材料纹理、两个高光角度、灯光与阴影、地形与探针，以及两个时刻的人物肢体。视角工具验证了相机读回，不替你证明已经学会鼠标操作。

## 独立挑战

每次只做一项，先写预测，再保存、检查、重载并验证：

1. 高度改为0.75 m，列出四个平台和球中心的预期高度。解释为什么双胞胎上的球不应跟着升高。
2. Actor周期改为9 s，列出五个waypoint时刻；判断移动段中点的位置和速度是否恒定。区分根路径周期与骨骼动画片段。
3. 保持碰撞几何不变，给一个样本改变diffuse或emissive。用相同视角和相同日光做对照，说明哪些画面改变、哪些物理量不应改变。

分层提示：先找数据属于visual、light、collision还是actor/script；再确认读到的是保存文件还是已加载状态；最后选择能回答问题的证据。参考答案单独查看，不会覆盖个人练习。

## 常见故障从哪一层查

| 现象 | 先查什么 |
| --- | --- |
| 木纹变白或消失 | script URI、材质name、纹理路径和大小写；看GUI资源错误，勿凭server存活判成功 |
| 改文件后画面没变 | 是否改个人world.sdf、是否保存并重载、加载指纹是否一致 |
| 镜面样本没有明显高光 | 灯的specular、观察角度、材质与实际渲染；换视角观察 |
| emissive亮但邻物不变亮 | 这是预期的自发亮外观；照亮邻物需要灯 |
| 地形像山，球却穿过或停错高度 | visual与collision高度图是否一致、有无额外碰撞平面、是否在平台内部采样 |
| 服务器能读地形，GUI却报TerrainQuadTreeNode/LOD断言 | 除2^n+1格式要求外，还须匹配当前渲染器的批次LOD限制；本课固定65×65、sampling=1 |
| Actor平移而肢体没变 | 皮肤/骨骼资源、动画名与trajectory类型、实际骨骼样本与GUI画面 |
| view等待超时 | gzclient是否真实连接本课会话；相机输入与输出主题不能混用 |

继续深入可查[SDF高度图几何](https://sdformat.org/spec/1.7/geometry/)、[SDF Actor](https://sdformat.org/spec/1.7/actor/)和[Classic资源路径说明](https://classic.gazebosim.org/tutorials?tut=components)。本课的资源格式、固定拓扑与可改范围是为了保持实验可比较，不是Gazebo所有建模能力的上限。
