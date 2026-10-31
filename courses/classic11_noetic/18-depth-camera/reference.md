# 深度与点云：只读反投影与布局说明

此页不覆盖个人world.sdf。数学公式用于预测和比较，真正的深度来自原生DepthCamera渲染，PointCloud2由系统插件读取同一深度生成；不以解析数组替代测量。

## Z深度与米制反投影

[REP118](https://github.com/ros-infrastructure/rep/blob/master/rep-0118.rst)的32位浮点深度按光学Z轴用m表示；常见OpenNI的16UC1毫米与0无效是另一种明确约定，不是任意整数图像的自动单位。本课只提供32FC1米输出。

给定零畸变内参K和有效像素(u,v,d)：

```text
X = (u−cx)d/fx
Y = (v−cy)d/fy
Z = d
rho = d × sqrt(1 + ((u−cx)/fx)² + ((v−cy)/fy)²)
```

rho是光心到点的欧氏距离，离轴时大于Z。正对相机的平面可使一片像素Z接近常数，不能由此推出rho也相同。Classic的[depth_map顶点程序](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/media/materials/programs/depth_map.vert)使用透视裁剪w作为深度，[片元程序](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/media/materials/programs/depth_map.frag)直接输出它；结合实际平墙离轴样本，验证这是相机前向深度，不是归一化到0到1的显示颜色或射线总长度。

相机机体+X向前、+Y向左、+Z向上，光学轴+Z向前、+X向右、+Y向下。本课安装在(0,0,1)、无旋转，光学点(X,Y,Z)转世界的关系为`(Z,−X,1−Y)`；一般安装须使用完整外参，不通过修改frameName完成变换。

## 当前插件的两种主点约定

Noetic CameraUtils默认CameraInfo主点为`c_info=((W+1)/2,(H+1)/2)`，OpenNI点云辅助函数却用`c_pc=((W−1)/2,(H−1)/2)`，出处为[OpenNI Kinect源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp)。点云实际使用：

```text
X_pc = (u−(W−1)/2)d/f
Y_pc = (v−(H−1)/2)d/f
Z_pc = d
```

由于c_info比c_pc各大1像素，同深度下`X_pc−X_info=d/f`、`Y_pc−Y_info=d/f`。默认320宽f≈277.1273456，墙深2.95 m对应约0.010645 m，移远到3.95 m约0.014253 m。分辨率加倍而FOV不变，f约加倍，对应米制差减半。它是确定的现有接口差异，不是随机测量方差。

元数据检查仍保留原样K/P与主点，现有点云按其实际辅助函数验证，并另存CameraInfo反投影对照。HFOV沿用第17课当前SDK的6位有效数字写回：名义60°读回1.0472 rad；不能把相机实际K改成理想数值来求一致。这里也未实现一个替代原生点云发布器。

## 固定几何和有效范围的预测

| 对象或区域 | 正面Z/m | 默认max=6 | 改max=4后 |
| --- | --- | --- | --- |
| 近绿色小块 | 约0.24 | 低于min=0.4，NaN | 仍NaN |
| 中央前景盒 | 1.9 | 有效 | 有效 |
| 背景墙基线 | 2.95 | 有效 | 有效 |
| 背景墙移远1 m | 3.95 | 有效 | 有效 |
| 墙上方远蓝板 | 4.95 | 有效 | 超过软件max，NaN |
| 没有量程内表面的上角 | 无有效命中 | NaN | NaN |

原生camera clip为0.1到8，软件保留条件为严格`0.4<d<max`。32FC1无效写NaN，点云对应XYZ全部写NaN并标is_dense=false；该逻辑不保留“过近”和“超远”的不同数值标签。RGB里的近绿色或远蓝色仍可见，证明NaN不能被一概解释成那个方向没有实体。

固定原生`depth_camera/output=depths`触发浮点深度帧回调，ROS PointCloud2由包装器再转换；切到原生points会改变回调路径，不能作为开启本课ROS点云的设置。原生深度消息的特殊值与经过OpenNI过滤后的ROS NaN是不同接口，按实际读取来源解释。

前景会遮挡后面的墙，不能拿中央像素作为墙深度；墙ROI选择在约0.70W与0.80W的水平位置、半高处，避开轮廓。墙宽3 m、厚0.1 m，移远1 m后其正面从2.95到3.95，前景保持1.9。平面Z变化与独立真实墙位置互相对应；点云Z匹配同一深度只是转换一致性，不能当作独立传感器重复测量。

## 深度字节的解释

像素偏移为`v×step+4×u`，本设备32FC1小端，每个值4字节。默认320×240时step=1280、原始数据307200字节；640×480时step=2560、数据1228800字节。即使紧密排列也先检查消息字段，不能将第17课rgb8的3字节布局套用过来。

有效性先用isfinite判断，再按当前驱动范围解释。平均或中位数只用有效样本，同时单列无效数量和比例；不要用0填NaN后再统计。伪彩色映射只用于显示，可能剪裁、重标度或对NaN使用特殊颜色；PNG显示图不替代原始float32与元数据。

## PointCloud2记录、步幅与颜色

按照[PointCloud2消息](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/PointCloud2.msg)，fields描述记录中的字段，point_step是单点字节数，row_step是行字节数，data长度为height×row_step。[PointField](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/msg/PointField.msg)的datatype是类型枚举，FLOAT32=7而实际占4字节，offset是相对单点起始位置的字节偏移。

有序点云保留height×width布局，当前像素对应点的起点是`v×row_step+u×point_step`。在该处加各字段offset，按类型和字节序读取，不能固定假设XYZRGB总共16字节。ROS的[点云辅助实现](https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/include/sensor_msgs/impl/point_cloud2_iterator.h)可为xyz/rgb加入填充；当前真实布局以收到的fields和步幅为准。

is_dense=false表示包含无效点；有序结构仍保留这些点的索引，不表示点云已经变成无序或删除了全部数据。跨深度断层的相邻像素也可能对应相隔很远的表面，不应只凭二维相邻便连成实体三角网格。

rgb常被声明为FLOAT32来承载打包位模式，按常见packedRGB约定应按位重解释通道，而非把这个浮点数数值强转成颜色。本插件还逐字节复制缓存图像，通道顺序须实读核对；XYZ几何正确不自动证明颜色字节符合所有消费者的约定。缓存颜色未由本课证明与当前深度严格同曝光，不能因点云头与深度头相同就推广为动态RGB-D硬同步。

## 采样、单变量和恢复流程

prepare检查个人输入、系统插件和分析依赖，不从理论生成深度图。config --width修改保存分辨率并保持4:3，config --max-depth修改软件上限；都需重载。wall --distance明确重定位本次运行的背景墙，保持原暂停状态，不修改保存起点，也不冒充动力学运动。

view仅显示由真实float32深度映射得到的伪彩色，近红、远蓝、无效紫，并标明米制范围；它不是RGB显示器。RGB辅助数据由独立订阅与验收记录提供，保留自己的header。保存的伪彩PNG便于视觉检查，原始.f32、.pc2与.rgb字节及字段元数据才用于定量复核。

验收记录进入时墙位姿、速度与暂停状态，依次建立baseline、farther、restored。各阶段等待新鲜的深度、深度CameraInfo和PointCloud2，按精确相同header配对；另外观察真实RGB以确认近远物体可见，不要求缓存颜色具有严格同次曝光。它检查实际深度与已知表面、点云字段和XYZ、无效值，以及真实墙位置。

本课原生采样固定10 Hz，基线320宽、640宽和仅改软件上限的实验均保持这一频率。每阶段至少12组唯一时间戳三路配对，首末跨度至少1.0仿真秒，配对频率8到12 Hz；墙钟频率另受实时因子、渲染和消息传输影响。完整有序点云的数据量按point_step×width×height计算，不能因为只显示几个采样点就忽略实际传输和解码成本。

整幅图的finite(depth)须与finite(cloud.z)掩码相同，无效深度和三个XYZ分量都为NaN；有效点Z距同帧深度不超过1e−5 m，现有辅助函数主点约定下XY反投影误差不超过1e−4 m。六个固定像素的已知表面深度容差0.03 m，预期无效处必须NaN；离轴wall_b点的rho须大于Z+0.1 m。移墙后墙深度增加1 m的误差不超过0.03 m，恢复后距基线不超过0.01 m。这里的全图一致性属于同一深度到点云转换的检查，几何表面和独立模型状态另行作证。

每阶段另取控制边界后至少0.2仿真秒的新鲜RGB，验证近绿色与远蓝色像素存在，保存自己的时间戳与原字节，不将它冒充点云缓存颜色的同曝光证据。每个抽样点还保留CameraInfo反投影与实际点云的差，以及rgb字段4字节和常见packedRGB解码，供复核现有包装的实际布局。

报告默认本会话depth-verify-唯一编号.json，支持--report；同级报告名-assets目录以SHA去重保存.f32深度、.pc2点云和.rgb原字节及元数据。三个阶段各存-depth-pseudocolor.png与-actual-rgb.png，明确区分显示深度和实际RGB。伪彩使用`t=clamp((Z−0.4)/(max−0.4),0,1)`映射为`(255(1−t),255(1−|2t−1|),255t)`，无效设为(255,0,255)；这组显示通道不是原始米制值，NaN和浮点精度须从原始文件读取。

最终无论通过或失败，都尝试恢复进入时的墙位姿、速度和暂停状态，记录恢复结果。世界时间不倒回，原始数据、字段布局和显示PNG说明不同层面的证据；仅有可视化颜色或主题名称不能替代米制深度和点字段检查。

## ROI挑战的思路

在墙内选择避开前景、边界和无效区的小ROI，读取所有有限深度并求中位数；有效比例为有效数量除以该ROI像素总数，不能先删除无效点后总数也跟着变小。按同一像素索引解码点云Z，比较深度转换，再计算XYZ模观察离轴变化。

分别说明采用CameraInfo主点还是现有点云辅助函数主点，不把一像素差悄悄改掉。若ROI不含有效点，应返回“没有可用深度”，不能输出0米或保证无障碍。三维融合、滤波、网格与实体相机注册标定是进一步主题，当前场景没有用简单伪彩色显示代替这些算法。
