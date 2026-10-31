# 深度与点云：像素里的米，怎样变成三维点

一块正对相机的平墙，在图像中央和偏右处深度几乎相同，为什么偏右那个点到相机的直线距离更长？一块绿色小物体明明能在RGB里看见，深度却是NaN，又是否表示那个方向安全畅通？本课用真实深度相机、前景盒与可移动背景墙，解释深度图和点云之间的关系。

**深度图（depth image）**为像素提供距离相关的标量；**点云（point cloud）**是一组按某个坐标系表达的三维点。这里的点云由同一批原生深度数据反投影产生，不能当成另一台独立传感器的重复测量。相机、图像布局和光学轴沿用第17课；不要求先学SLAM、三维重建或PCL。

## 先查看真实深度，保留RGB对照

相机位于世界(0,0,1) m，朝+X，默认320×240、水平视场60°、目标10 Hz。平台加载Classic原生depth相机与系统Noetic OpenNI Kinect观测插件，实际渲染输出深度、相机参数和PointCloud2；无需实体Kinect或联网下载驱动。点云每点有XYZ、颜色和填充，数据量明显大于单路RGB，本课固定10 Hz观察完整二维点云，不通过插值制造更多帧。

```sh
python3 lab.py observe --seconds 3
python3 lab.py view
```

observe读取有界深度与点云数据，view打开深度伪彩色查看器。深度图中的彩色是为了让人看清远近的**伪彩色**：近处偏红、远处偏蓝、无效区域为紫色，不是场景材质颜色，更不是原始米制数据；定量比较始终解码32FC1。这个查看器不是RGB和深度并排显示，RGB近远物体对照由验收另外记录。先核对时间戳，暂停时显示的最后一帧不代表又取得了新测量。

三路配对主题是`/lesson18/depth/image_raw`、`/lesson18/depth/camera_info`和`/lesson18/depth/points`，RGB辅助主题为`/lesson18/rgb/image_raw`。也可用`python3 lab.py view --seconds 3 --capture build/my-depth-view.png`限时查看并保存实际深度转换出的显示图；这张伪彩PNG不能替代原始float32用于测量。

本课必须有可用的图形渲染上下文。能启动gzserver或看到一个主题，不足以证明深度采样正常；实际图像、数值和三维点都要检查。

## 32FC1、米与无效值

本课深度消息是sensor_msgs/Image，encoding=32FC1：32位浮点、1通道，数值按[REP118深度约定](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst)表示沿光学Z轴的米制深度。不能将data当作RGB字节直接显示，也不能把每4个字节分别当作4个深度。

| 内容 | 本课解释 |
| --- | --- |
| width、height | 深度像素列数与行数 |
| encoding=32FC1 | 每像素一个32位浮点数，当前驱动输出单位m |
| step | 跨过整行的字节数，本课紧密排列为4×width |
| is_bigendian | 浮点数内部字节序；本设备输出0 |
| data | height×step个原始字节，须按类型和步幅解码 |
| 有限正深度 | 当前插件保留下来的有效测量 |
| NaN | 本课插件标记的无效深度，不能拿来求均值或当作0 m |

像素(u,v)的起始偏移为`v×step+4×u`。不同驱动也可能输出16UC1；常见OpenNI约定是毫米，0表示无效。但encoding本身描述存储类型，具体单位仍要查驱动约定。本课只有32FC1米输出，不会因你把变量类型换成整数就自动变成毫米；也不能把任意16位图像都除以1000。

NaN需要用isfinite/isnan等浮点分类判断；它不等于一个普通数字，也不能用“等于NaN”来可靠筛选。第15课射线的无回波可能是+Inf，本课包装把无效深度写成NaN；相似的传感器概念不保证相同接口表示。

## 光学Z与斜距是两种距离

光学坐标系+Z朝前、+X向图像右、+Y向图像下。深度d表示Z，不是点到原点的欧氏距离rho。给定有效深度和零畸变内参：

```text
Z = d
X = (u−cx) × d/fx
Y = (v−cy) × d/fy
rho = sqrt(X²+Y²+Z²)
```

这就是**反投影（back-projection）**：RGB像素加内参原本只能确定一条方向，深度再给出沿光学Z的尺度，得到一个点。点不在主轴上时rho>d，不能把LaserScan的径向range和深度图Z直接逐项比较。

本课背景墙中心X=3 m、厚0.1 m，正面深度2.95 m；在同一正面上的中央外侧像素，Z仍接近2.95，X、Y和rho则随方向改变。验收在墙内选取离开前景的像素，既检查平面Z，也检查三维斜距大于Z。深度来自真实渲染，几何公式只用于比较，原生路径见[Classic DepthCamera](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/DepthCamera.cc)。

## 渲染范围、软件过滤和显示颜色不要混用

本课相机的原生裁剪范围为0.1到8 m；ROS插件另用pointCloudCutoff=0.4和pointCloudCutoffMax=6过滤输出，只有严格满足`0.4<depth<max`才保留，否则32FC1和XYZ都写NaN，见[Noetic OpenNI Kinect插件](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp)。伪彩色显示还会把有效深度映射成颜色，这又是第三层。

本课读取的是这些ROS输出，不把Gazebo原生另一路深度消息可能使用的特殊值原样套用过来。固定的`depth_camera/output=depths`也须保留：它选择原生深度帧，ROS点云由包装器从该帧生成，改成native points并不是开启这里PointCloud2主题的方法。

| 场景区域 | 为什么值得观察 |
| --- | --- |
| 中央前景盒，正面1.9 m | 遮挡后方墙体，深度应先看到较近表面 |
| 背景墙，正面2.95 m | 同一平面多个像素的Z近似一致 |
| 很近的绿色小块，正面约0.24 m | 在渲染范围内、RGB可见，但被软件下限过滤成NaN |
| 墙上方的远蓝板，正面4.95 m | max=6时有效，max=4时被过滤，RGB仍可见 |
| 上方未命中区域 | 没有量程内有效表面，输出NaN |

近物体引起的NaN与远处空背景引起的NaN可能具有同一数值标记。因此“没有有效深度”不等于“没有障碍”，也不能把它统一替换成很远的安全距离。只改变显示颜色不会修改原始深度，只改变软件上限不会把蓝板从场景里删除。

## PointCloud2为什么需要字段表

PointCloud2把点存成二进制记录，记录内部结构由fields描述，不能无条件把每16字节当作XYZ和颜色。**有序点云（organized cloud）**保留二维行列组织，本课height与width对应深度图；无效点仍占原像素位置，不应先删掉NaN再假装索引仍与图像一致。

| 字段 | 用途 |
| --- | --- |
| fields中的name、offset、datatype、count | 各字段的名字、记录内字节偏移、类型与元素数 |
| point_step | 一条点记录占多少字节，可能含填充 |
| row_step | 一整行点占多少字节，可能有额外填充 |
| is_bigendian | 多字节字段的字节序 |
| data | 实际记录字节，长度应为height×row_step |
| is_dense | 是否宣告没有无效点，不是“画面看起来点很多”的意思 |

像素(u,v)对应点的字段起点为`v×row_step+u×point_step+field.offset`，再按datatype读取。字段枚举FLOAT32=7不表示7字节，见[PointCloud2](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/PointCloud2.msg)与[PointField](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/PointField.msg)。

本课插件生成XYZ和rgb字段，辅助函数可能加入填充，必须核对真实fields与步幅。rgb虽可能声明为FLOAT32，通常用于承载颜色位模式，不能像一个物理浮点量那样加减或乘1000。解释颜色前还要确认实际字节顺序；本版本包装使用缓存RGB，点云头中的深度时间戳不自动证明颜色也来自同一次曝光。

## 一个必须诚实保留的像素中心差异

第17课的CameraInfo主点默认是`((width+1)/2,(height+1)/2)`。当前OpenNI点云辅助函数反投影时却使用`((width−1)/2,(height−1)/2)`，相差各1像素。它们不能通过改写接收到的K或点云悄悄“统一”。

正式检查分开验证：相机元数据仍按本版本CameraInfo约定，实际点云则按辅助函数的真实约定与同帧深度比较。同时记录若用CameraInfo重建会有多大差异；相同d、f时，X与Y各相差d/f。默认320宽、墙深2.95 m时约0.01065 m，不应被误判成随机噪声或通过放宽阈值抹去。

这也是区分“读取现有插件”和“实现一个坐标一致的新驱动”的例子。智能使用现成接口，需要知道它实际做了什么，而不是为了让公式看起来漂亮就篡改数据。

## 一次只改一项，再看真正的深度变化

```sh
python3 lab.py config --width 640
python3 lab.py config --max-depth 4
```

每次只执行一项并重载。width=640同时保持4:3的480行，米制平面深度应不变，采样点更多；max-depth=4只改变软件有效上限，背景墙2.95及移远后的3.95仍有效，远蓝板4.95变NaN。其它固定结构保留，以便把变化归因到单个参数。

运行中的墙可以用`python3 lab.py wall --distance 4`明确移到中心X=4 m，正面变3.95 m；这不是保存起点编辑，也不是力控制的连续运动。原暂停状态会保留，继续后观察新帧。

```sh
python3 verify.py
```

完整验收记录进入时真实墙位姿、速度与暂停状态，依次建立配置基线、墙向远处移动1 m、回到基线三个阶段；实际前景盒保持不动。它从同一时间戳的Depth Image、深度CameraInfo和PointCloud2取样，检查墙深度增加约1 m、前景深度仍约1.9 m、近物体/背景的无效值以及实际XYZ。最后恢复进入时的真实墙状态和原暂停状态，不倒回世界时间。测量期间不要同时移墙、拖模型或重置。

每阶段至少12组不同时间戳的三路配对，跨度至少1仿真秒，实际配对频率须在8到12 Hz内。RGB使用自己的真实header单独观察，不因静止场景里看起来一致就宣称缓存颜色与每组三路数据是严格同曝光。分辨率与软件上限两个单变量实验采用相同10 Hz和同一套米制比较，不改变测量含义。

六个指定像素的已知表面深度容差为0.03 m，预期无效处须实际NaN；程序还逐像素比较深度和点云的有效掩码，检查Z一致及完整点云反投影。移墙后墙深度增量应为1±0.03 m，恢复后的墙深度距基线不超过0.01 m，不用一帧刚好接近来替代持续窗口。

原始报告保存在本会话depth-verify-唯一编号.json，终端打印报告和assets目录路径；也可用`python3 verify.py --report build/my-depth-check.json`指定位置。同级的报告名-assets目录按SHA去重保存原始.f32、.pc2与.rgb，另存三个阶段的深度伪彩PNG和实际RGB PNG。RGB原图和伪彩图分别标明来源，不能拿后者的紫色区域反推场景材质。

## 深度、RGB与真实RGB-D相机的边界

本课RGB和深度由同一个仿真相机模型提供，几何对齐更容易控制；这不等于已经完成实体RGB-D相机的内参、外参、时间同步和注册标定。点云的RGB还来自插件缓存，动态物体可能暴露时序差异。当前主验收依据真实深度和XYZ，RGB用于确认近远物体确实存在，不宣称已证明任意运动下的同步彩色点云。

CameraInfo沿用第17课相机有效FOV的SDK舍入与主点约定，投影需使用实际参数。光学frame标签不自动广播TF；要转世界坐标仍需真实安装变换。不了解点云，可看[Open3D官方RGB-D与点云入门](https://www.open3d.org/docs/release/tutorial/geometry/rgbd_image.html)，本课读取和验证不要求先安装Open3D。

## 文件、故障和独立挑战

| 文件 | 职责 |
| --- | --- |
| world.sdf | 原生深度相机、背景墙、前景及近远对照物 |
| prepare.py | 检查实际输入、插件和分析依赖 |
| lab.py | 正确解码深度/点字段、只读观察、显示与显式操作 |
| verify.py | 三阶段真实数据、反投影/几何比较与恢复 |
| reference.md | 只读公式、数据布局与示例答案 |

| 现象 | 先检查什么 |
| --- | --- |
| 读出巨大或极小的乱码距离 | 32FC1/16UC1、字节序、step与float32解码 |
| 绿色物体可见却深度NaN | 软件下限和近裁剪；NaN不能直接当作空闲 |
| 同一平墙边缘距离似乎更长 | 是否把欧氏斜距与光学Z混在一起 |
| 点云像被分层撕裂 | fields.offset、point_step、row_step和行列顺序 |
| is_dense=false但点很多 | 有无效点即可为false，不表示有效点总数少 |
| 用K反投影与已有点云有小偏差 | 当前辅助函数与CameraInfo的一像素主点差，分开核对 |
| 彩色点云颜色或动态边缘不吻合 | packed颜色字节顺序、RGB缓存与曝光时序，不先改XYZ补偿 |
| 改上限后RGB物体没消失 | 修改的是深度软件过滤，不是场景几何 |

独立挑战：在背景墙内部选择一个小ROI，分别报告有限深度的中位数、有效比例、对应点云Z中位数与欧氏斜距分布。提示一：ROI必须避开前景与轮廓；提示二：保留NaN计数而不是先用0填满；提示三：同一平面Z近似一致，不代表所有点的rho也一致。写明你采用的像素中心约定，并说明点云与深度来自同一测量，不能把二者吻合当成两台独立传感器互相证明。
