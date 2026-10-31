# 参考说明：五种Classic原生传感器的数据契约

本说明面向完成主教材后需要查字段、坐标变换和验收流程的学习者。原生接口依据Gazebo Classic 11.15.1，不以新版Gazebo Sim的同名系统替代。程序在实机运行五类传感器；SDK头文件存在、工厂名称能编译或GUI里有模型，都不能单独证明某站产生了有效测量。

## 共享的观测边界

```text
SDF真实参数/真实模型状态
  → Classic原生Sensor计算
  → Gazebo Transport原生protobuf
  → 本课适配桥逐字段复制为JSON
  → ROS String + 观察程序/验收程序
```

桥传递原生数据，不通过预期的经纬度、磁场公式、几何高度或模型位置合成输出。验收预测与实际测量分开；独立Gazebo状态查询用于证明受控模型真的到了指定状态，而不仅是某条观察主题改变了。

| 文件 | 作用 |
| --- | --- |
| world.sdf | 五种原生sensor、真实碰撞几何、世界地理/磁场参考 |
| config.py | 校验固定场景契约，局部修改磁场倍数或初始高度 |
| prepare.py | 用已安装SDK离线编译私有桥，记录输入/源码/库指纹 |
| sensor_bridge.cc | SensorPlugin适配实际原生protobuf，不生成测量 |
| lab.py | 只读观察、受控定位和参数保存 |
| verify.py | 分阶段采集真实消息、独立状态和验收结果 |
| build/stand_lab.json | 当前练习的准备记录；不把它当作原生测量 |

私有库为libgazeboteach_l20_sensor_bridge.so，构建缓存由源码、SDK版本与编译参数确定，并记录库SHA256；它不安装或替换系统传感器库。更改个人world后需重新准备/重载，才能保证报告对应真正运行的配置。

每类原生消息可通过官方定义查字段：[GPS](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/gps.proto)、[Magnetometer](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/magnetometer.proto)、[Altimeter](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/altimeter.proto)、[SonarStamped](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/sonar_stamped.proto)、[Sonar](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/sonar.proto)、[LogicalCameraImage](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/msgs/logical_camera_image.proto)。JSON是本课观察接口，不声称它就是通用ROS传感器消息标准。

| 观察字段 | 定义 |
| --- | --- |
| source_stamp_ns | 四类有time的原生消息所携带仿真时间；Logical为null |
| receipt_world_time_ns | 桥收到消息时读取的world仿真时间，不保证等于测量时刻 |
| bridge_sequence | 每站桥实例各自递增的序号，证明新回调，不伪称原生帧号或全局顺序 |

纳秒与序号采用十进制字符串保存，读入后明确转整数；Logical原生时间保持JSON null。另有source_kind、native_topic、native_type记录真实来源，native对象保留原生protobuf字段名。sensor_bridge.cc是每站挂载的SensorPlugin，只订阅相应真实原生主题；ROS发布类型为std_msgs/String，不是已经完成语义转换的NavSatFix/Range等标准传感器消息。

| ROS观察主题 | native_type |
| --- | --- |
| /lesson20/gps | gazebo.msgs.GPS |
| /lesson20/mag | gazebo.msgs.Magnetometer |
| /lesson20/alt | gazebo.msgs.Altimeter |
| /lesson20/sonar | gazebo.msgs.SonarStamped |
| /lesson20/logical | gazebo.msgs.LogicalCameraImage |

`rostopic echo -n 1 /lesson20/gps`会显示String.data中的JSON文本；使用本课工具可避免手动处理转义字符。不要把ROS主题名误当作原生Gazebo Transport路径，原生真实路径保存在native_topic。

不要把接收世界时间命名为“原生采样时间”，也不要为Logical填一个看似与其他传感器同步的假stamp。World时间暂停时不前进，wall time仍前进；native时间率与receipt时间率应明确区分。以后做同步/延迟分析时，这一区别比“主题都有10 Hz”更关键。

## GPS：从仿真局部坐标到WGS84

[GpsSensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/GpsSensor.cc)取得传感器世界位姿，对位置按配置加噪声，再调用世界SphericalCoords的SphericalFromLocal。输出latitude_deg、longitude_deg、altitude；速度取传感器安装点的WorldLinearVel，再转换到Global方向并加配置噪声，填velocity_east/north/up。当前课基准不加噪声、安装偏置为0，因此稳态速度约0。

这条路径直接使用仿真状态，不生成卫星轨道、伪距、可见卫星数或GNSS精度因子。消息time是world仿真时间，不是UTC。现实GPS链路的背景可看[GPS.gov定位系统介绍](https://www.gps.gov/gps)与[影响定位精度的因素](https://www.gps.gov/gps-accuracy-0)。

### 本版本heading为何显式取180°

[SphericalCoordinates.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/common/SphericalCoordinates.cc)的LOCAL输入路径含历史符号约定。令h为heading_offset、c=cos(−h)、s=sin(−h)，其水平变换为：

```text
E = −c·x + s·y
N = −s·x − c·y
U = z
```

同一约定用于位置与速度。h=0时E=−x、N=−y；本课h=180°时E≈x、N≈y。这里说明特定Classic版本的行为，不把它当成所有GIS、ROS或新版Gazebo的通用heading定义。磁力计读取另一个world向量，地理heading不会顺便旋转world magnetic_field。

课程采用EARTH_WGS84、lat0=lon0=0°、elevation=100 m。先把局部位置经heading变到ENU，再经地理参考变到ECEF，最后转换到椭球经纬度/高程。ECEF是地心地固坐标，ENU是参考点附近的切平面轴；两者关系见[ESA Navipedia](https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates)。

对小范围位移，可用下面的近似理解方向，不能替代完整转换做无限精度比较：

```text
delta_lat_rad ≈ delta_N / (M + h0)
delta_lon_rad ≈ delta_E / ((N_curvature + h0) cos(lat0))
```

M为子午圈曲率半径，N_curvature为卯酉圈曲率半径，均为米；h0为参考高度。转换后的椭球高度近似elevation+worldZ，水平位移还有曲率小项。它不等于最近地形表面距离，也没有自动转换成大地水准面上的正高/海拔。

## Magnetometer：保持磁场，旋转表达轴

[MagnetometerSensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/MagnetometerSensor.cc)读取world MagneticField，再用传感器世界姿态的逆旋转得到field_tesla，最后按配置给各轴加噪声并填写原生time。

这里B是磁感应强度，单位Tesla；不要混作单位A/m的H。若R_WS把传感器分量映到世界分量，则B_S=R_WSᵀB_W。默认B_W=(20,0,−40) μT，yaw=+90°时B_S=(0,−20,−40) μT；旋转保持模长√2000≈44.721 μT。

本课固定均匀磁场，未把现实材料的铁磁效应、设备硬铁/软铁误差或随地理位置变化的地磁模型自动加入传感器计算。世界B方向也不必指向地理北；真实磁偏角/磁倾角与时间、地点有关，可查[NOAA WMM说明](https://www.ncei.noaa.gov/products/world-magnetic-model)。该先修链接解释现实背景，不表示本课下载或调用了WMM数据库。

## Altimeter：参考值固定，竖直方向固定

[AltimeterSensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/AltimeterSensor.cc)在Load时将传感器世界Z保存为vertical_reference。Update时返回currentZ−reference与传感器安装点WorldLinearVel的Z分量，可分别加入位置/速度噪声；time取world仿真时间。

本课零偏置安装，reference=1 m；移动到z=2时vertical_position=+1 m、静止vertical_velocity≈0 m/s。绕yaw旋转不改变worldZ，也不会将“竖直”改成机体轴。一次性SetModelState改变位置并清速度，不建立连续爬升轨迹，不能要求速度输出等于位置跳变除以采样周期。

独立nav_velocity阶段一次初始化world vz=+0.25 m/s后不再写状态，让禁重力、无衰减的动态刚体自由上行。它应使真实z连续增加、GPS velocity_up与Altimeter vertical_velocity都接近+0.25 m/s；独立状态查询与时间窗口用于验证这段运动，而不只验某条消息中的常数。结束恢复进入时状态，不能把这类初始化自由运动称作持续电机控制或闭环飞行。

初始高度是另一个单变量：从SDF初始z=1改为2并重载，reference随加载改成2 m、Altimeter基线仍0；GPS基线从约101变为102 m。这与运行中升高1 m产生Altimeter +1读数的实验应分开。

它不读取地形高度，不射线探地，不计算气压；vertical_reference不是WGS84 elevation。本接口另有SetReferenceAltitude C++方法，但本课定位命令不调用它，不能假设每移动一次就重新归零。

## Sonar：原生碰撞体与有限range

[SonarSensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/SonarSensor.cc)建立cone或sphere感知碰撞体并开启collideWithoutContact，通过接触过滤器接收物理引擎记录。cone沿局部−Z，几何长度使用rangeMax−rangeMin。本课min=0、max=3 m、radius=0.2 m；不能将该配置的行为直接推广成带同样min/max语义的RaySensor。

实现对接触位置计算相对传感器的距离并减去接触穿透深度，取收到记录中的较小结果；它限制接触点数量，不保证复杂多物体场景中的精确全局最近点。本课的平面/平顶试块用于检查约1.5 m与0.9 m的实际变化。它不因visual颜色或透明度改变声学反射，也没有声速、频率、飞行时间和多径模型。

SonarStamped含原生time；Sonar内有frame、world_pose、range_min/max、radius、range、geometry及可选contact。world_pose是传感器世界位姿；contact按传感器局部坐标表达。frame是原生标签，不能拿它当作已经发布的ROS TF。

contact取ODE原始接触位置减去sensor世界原点，再逆旋转到sensor轴；它没有做穿透深度修正。range则使用原始距离减contact.depth，深度没有被放进Sonar消息。因而contact不应被解释成精确声学反射点，也不必等于(0,0,−range)；它的范数可能明显大于range，不能只据此断言桥损坏或用它改写原始range。

空接触期间实现有计数迟滞，旧range会短时保留，随后回到有限rangeMax；contact可保留上次值。有效解释需要联合看新消息、当前range、场景和过渡时间，不能把contact存在等同于当前命中。range=rangeMax不证明该距离恰好有反射面；本课将朝下探测的试架升到z=4 m、地面超出量程后用真实消息验证该行为。

本课让声纳试架为gravity=false动态刚体，不采用两个static碰撞体之间必然产生接触这一假设。定位后清零速度，保持测量台停在指定状态，不靠持续给range赋值。地面和试块有真实collision；感知碰撞体不承担推开障碍的机械执行器作用。

## Logical Camera：AABB相交与相对位姿

[LogicalCameraSensor.cc](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/LogicalCameraSensor.cc)设置near/far/HFOV/aspect构成frustum，遍历世界model及nested model，把与视锥相交的模型AABB对应的scoped name与相对pose加入消息；自身所属model被排除。零姿态沿+X看，详见[Classic官方教程](https://classic.gazebosim.org/tutorials?tut=logical_camera_sensor)。

AABB源于模型collision几何，不是RGB投影轮廓。visual独有的模型、透明外观、collision和visual大小不一致、遮挡以及旋转后AABB变大，都会使名单与RGB可见内容不同。地面等大包围盒可能长期被列出，本课只验证明确探针的进出，不把完整名单硬编码成“恰好一个目标”。

本场景在相机(0,−4,1)与探针(2,−4,1)之间放置遮挡件，中心(1,−4,1)、尺寸0.25×0.6×0.6 m。探针仍被上报是接口预期，不表示它透视或识别出了遮挡后的RGB轮廓。验收同时核查遮挡件与探针的实际位置，避免只有文字声称存在遮挡。

设T_WC为相机世界位姿，T_WM为模型世界位姿，则相对位姿T_CM=T_WC⁻¹T_WM。其平移为p_CM=R_WCᵀ(p_WM−p_WC)，姿态也要相乘转换。当前相机无旋转，世界(2,−4,1)对应相对(2,0,0)；转动相机后，直接相减不能替代逆旋转。

LogicalCameraImage顶层pose为相机world pose，model[i].pose为相对camera pose。其proto没有time字段；必须保留source_stamp_ns=null，并将观察者接收时间单列。不能用接收时间将它伪装成与GPS同一个native测量时刻。

## 九个状态的实验逻辑

| 状态 | 单项主要变化 | 主要观测 |
| --- | --- | --- |
| baseline | 标准布局 | 五类原生输出、各零点和基线 |
| nav_up | nav_rig的z从1到2 | GPS高度增1、Altimeter位置增1 |
| nav_east_north | nav_rig到(10,5,1) | 经/纬方向与数量级，Altimeter回0 |
| nav_yaw90 | 试架yaw=+90° | 磁场换轴、模长保持 |
| nav_velocity | 一次初始化world vz=+0.25 m/s | 真正自由运动、GPS向上速度与高度计速度 |
| sonar_obstacle | 声纳试块进入正下方 | range从约1.5到约0.9 |
| sonar_out_of_range | 声纳升到z=4 | 过迟滞后range=max=3 |
| logical_outside | 探针移到相机后方 | 指定name退出名单 |
| logical_restored | 探针回到前方 | name回来且relative pose正确 |

每个阶段先按该阶段上下文初始化四个受控模型，不把前一阶段的其他改动悄悄带进新比较。随后等待传播/原生历史影响消退，采集五路新消息、独立查模型状态、验证数据契约；五类都要求真实运行，不以某一个传感器成功来替代其他四类。四类使用不同native时间计新样本；Logical只统计桥观察序号和receipt窗口，不宣称具有原生时间率。

普通阶段guard=0.3仿真秒，sonar_out_of_range为0.8秒。guard后先等各路首条符合新状态的数据，随后该路所有接收的新帧都须满足判据，不从完整窗口中只挑好看的样本；过渡、等待符合新状态和被接受的帧均保留在trace。每阶段每路至少12事件、跨度至少1 s，observed_rate=(条数−1)/跨度，范围8至12 Hz；Logical这一率按接收世界时间计算。共同观察窗口还须含至少3次独立nav状态查询，不代表五路native时间已严格同步。

| 实际检查 | 当前阈值 |
| --- | --- |
| GPS经/纬相对WGS84预测 | 每角不超过10⁻⁸度 |
| GPS椭球高度、Altimeter相对高度 | 0.002 m |
| GPS三向速度、Altimeter垂直速度 | 0.005 m/s |
| Altimeter初始化reference | 10⁻⁹ m |
| Magnetometer三分量 | 10⁻⁹ T |
| Sonar有接触的基准几何距离 | 0.06 m，保留接触体近似误差 |
| Sonar超量程输出max=3 m | 10⁻⁶ m |
| Logical目标/遮挡件相对位置 | 10⁻⁵ m，并核查探针进出名单 |
| 独立nav状态 | 位置0.005 m、线速度0.005 m/s、角速度0.005 rad/s、姿态角差0.005 rad |

nav_velocity的预测使用一次初始化后的真实时间间隔，z=z0+0.25Δt；原生GPS/Altimeter速度和独立状态查询都要支持这段运动。状态查询前后另取world时间，不把异步查询结果伪装成与每条传感器严格同刻。

完整验收入口为`python3 verify.py [--report PATH]`，默认报告在会话目录stand-verify-唯一编号.json；stdout给摘要及trace_path，文件保存完整原始wire_json/native、评估、阶段上下文和恢复信息。平台verify_timeout为180墙钟秒；每个阶段另有有限等待，超时即失败，不把收到过一条消息当作通过。

控制入口为`python3 lab.py move --station nav|sonar|obstacle|target --x X --y Y --z Z [--yaw DEG] [--vz M_S]`，只对指定模型做一次SetModelState，清roll/pitch和未指定速度；仅nav允许非零vz。它临时暂停定位后恢复进入时的pause；不会持续追踪目标轨迹。probe与observe只读，probe另可`--report PATH`保存诊断，不能代替九阶段验收。

验收会临时改变本课四个受控模型的状态：nav_rig、sonar_rig、sonar_obstacle与tracked_target；结束恢复进入时的位姿/速度和暂停状态。恢复是把模型状态写回，不是倒放时间，也不保证原生传感器内部接触迟滞等历史缓存被清空；需要等待新观测形成新的稳定窗口。

## 学习记录建议

本课保存输入只开放field-scale=1/2和initial-height=1/2 m，直接修改world.sdf；准备记录存于build/stand_lab.json，包含world配置指纹、桥源码/库SHA256和SDK版本。保存参数后需要重载，观察/控制会拒绝与准备记录不一致的输入。其余factory、topic、noise、rate、地理参考及几何由预检固定；这项源码契约检查不能冒充五站真实运行验收。

```sh
python3 lab.py config --field-scale 2
python3 lab.py config --initial-height 2
```

上面是两个分别执行的单变量练习入口；每次只选一个、保存后重载并观察，先恢复该参数再做另一项。field-scale放大worldB而不是改JSON，initial-height改变模型初始pose而不是改Altimeter已加载的参考值。

每次只改一个受支持参数或场景状态，保存：配置/实际位姿、原生字段、单位与参考、source/receipt时间、预测、稳定窗口结果和异常。不要只保留屏幕上最后一条四舍五入后的数字。噪声、倾斜或复杂障碍可以作为进一步实验设计问题，本课验收并未声称测试它们。
