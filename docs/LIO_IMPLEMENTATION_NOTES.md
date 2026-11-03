# A50—A53：LIO 附加专项的实现前核查

核查日期：2026-11-03。本文属于 G03 的实施设计，**A50 FAST-LIO、A51 FAST-LIO2、A52 Point-LIO、A53 LIO-SAM 全部仍待编写、构建和真实验收**。本次只读检索上游源码、GitHub API 与 Classic 11.15.1 实现，没有部署这些算法，没有将其列为已交付课程，也没有改动已有课程。

## 1. 固定可复取的来源，不能以名称猜版本

下表完整 SHA 均由 GitHub 分支/提交 API 和对应 raw 文件核对，不是推测的版本号。这里的“候选固定源”意味着可复取、可审阅，**不意味着已在 ikaros 编译或稳定运行**。

| 专项 | 候选固定源 | 身份与实现区别 |
|---|---|---|
| A50 FAST-LIO 第一代 | [`hku-mars/FAST_LIO@6e1fa94351316eafac5fcf55c5fcd575f3e89829`](https://github.com/hku-mars/FAST_LIO/tree/6e1fa94351316eafac5fcf55c5fcd575f3e89829) | 2021-06-08 历史提交；有独立 `feature_extract.cpp` 与 `loam_feat_extract`，不是当前 FAST-LIO2 |
| A51 FAST-LIO2 | [`hku-mars/FAST_LIO@7cc4175de6f8ba2edf34bab02a42195b141027e9`](https://github.com/hku-mars/FAST_LIO/tree/7cc4175de6f8ba2edf34bab02a42195b141027e9) | ROS 1 main，2024-07-23；直接点到地图、IKFoM、ikd-Tree；含 MARSIM 输入分支 |
| A52 Point-LIO | [`hku-mars/Point-LIO@1510c3bbf1743f254d83d0b22fbabb5b2d729f6b`](https://github.com/hku-mars/Point-LIO/tree/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b) | ROS 1 原 master，2024-05-20；选择逐点处理版本，避免悄悄变成当前默认 grid-map 分支 |
| A53 LIO-SAM | [`TixiaoShan/LIO-SAM@0be1fbe6275fb8366d5b800af4fc8c76a885c869`](https://github.com/TixiaoShan/LIO-SAM/tree/0be1fbe6275fb8366d5b800af4fc8c76a885c869) | ROS 1 master，2023-04-17；点云去畸变/特征、IMU 预积分、GTSAM 因子图，非迭代滤波器的改名 |

第一代与第二代的分界有直接证据：后继提交 [`1087a72497d19c3ac19a608b89e28b5e62b00e2a`](https://github.com/hku-mars/FAST_LIO/commit/1087a72497d19c3ac19a608b89e28b5e62b00e2a) 于2021-07-05明确发布 FAST-LIO2。不能把当前 main 的 `feature_extract_enable=true` 当成“第一代 FAST-LIO”。第一代应使用上述历史树，保留其滤波、特征与建图路径。

Point-LIO 当前默认分支为 `point-lio-with-grid-map`，核查到的头为 [`4b86a469eb5572e70ed575af25b5f15dd06e8e3c`](https://github.com/hku-mars/Point-LIO/tree/4b86a469eb5572e70ed575af25b5f15dd06e8e3c)，日期2026-06-13。将来可另做扩展比较，但 A52 的首个实现先固定原 master。

### ROS 2 来源的准确表述

- FAST-LIO 作者仓库确有 `ROS2` 分支，固定候选 [`cc3c9e8b1469b33b8072764d6db2348ed6dc597f`](https://github.com/hku-mars/FAST_LIO/tree/cc3c9e8b1469b33b8072764d6db2348ed6dc597f)，提交日期2026-10-25。[该 SHA 的 README](https://github.com/hku-mars/FAST_LIO/blob/cc3c9e8b1469b33b8072764d6db2348ed6dc597f/README.md) 明确署名 ROS 2 fork 的社区维护者 Ericsiii，构建说明也指向其仓库。应称“作者仓库收录的社区维护 ROS 2 移植”，不能称没有作者仓库分支，也不能推断为原作者对所有 ROS 2 版本的支持承诺。它是 FAST-LIO2 路线，不是第一代 ROS 2 实现。
- LIO-SAM 作者仓库的 `ros2` 分支可固定为 [`08af3f32f01725372d4269838dc44c19c6d9e76b`](https://github.com/TixiaoShan/LIO-SAM/tree/08af3f32f01725372d4269838dc44c19c6d9e76b)，提交日期2025-02-14。[该版 README](https://github.com/TixiaoShan/LIO-SAM/blob/08af3f32f01725372d4269838dc44c19c6d9e76b/README.md#notes-on-ros2-branch) 说明了移植缺项和测试设备；不能假定与 ROS 1 全功能相同。
- 本次 Point-LIO 作者仓库分支 API 只发现 `master` 和 `point-lio-with-grid-map`，没有据此确认作者维护的 ROS 2 分支。社区移植若后续采用，必须另锁来源、补丁与许可证，单独验收。
- 上述 ROS 2 文档的已测发行版不能外推成 Jazzy 已验证。课程实现须分别记录 Humble/Fortress、Jazzy/Harmonic 等实际组合；包能在 colcon 中找到，不等于算法/消息时间链已通过。

## 2. 数据契约比 launch 文件更先决

### FAST-LIO2 与 Point-LIO 的旋转雷达输入

[FAST-LIO2 `preprocess.h`](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/src/preprocess.h#L41) 的 Velodyne 点类型需要 `float32 x/y/z/intensity/time` 和 `uint16 ring`。`time` 是本扫描内相对采样时刻，不是消息发布时间；`timestamp_unit` 必须与实际数值匹配。内部会将其换算到毫秒。Point-LIO 的[对应类型与预处理](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/preprocess.cpp#L173) 也使用逐点时间；`ring` 应为实际垂直束编号，不能把数组索引任意取模后称真实硬件 ring。

两者均有在缺时间时根据方位角估算扫描时间的路径。**这不是把一次性整帧 Gazebo 快照变成真实滚动扫描的依据**。尤其最后一点的 `time<=0` 会触发 FAST-LIO2 的无逐点时间分支；给所有点填0虽如实表达瞬时快照，算法仍可能按机械旋转重新估时，数据语义就会不一致。首版运动去畸变实验不采用该捷径。

推荐适配器输出秒单位的 float32 `time`，相应 YAML 显式 `timestamp_unit: 0`；`header.stamp` 是第一列真实采样时间，点按采样时间排序，最后一点应保留真实末列时间。过滤无回波后也要检查尾点时间和帧覆盖范围，避免时间结束点意外丢失。配置中的 `scan_line/scan_rate/blind` 应按本课实际模拟束数、转速、场景距离重写，不能照抄上游32线/2m盲区样例。

### 第一代 FAST-LIO 的特殊约束

历史版 [HORIZON 输入路径](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/feature_extract.cpp#L166) 使用 `livox_ros_driver/CustomMsg` 的 `line/tag/offset_time`，将纳秒偏移换算为毫秒存入特征点 `curvature`。首版优先考虑这个明确保留逐点时间的路径。

该历史版 [Velodyne 处理函数](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/feature_extract.cpp#L238) 从仰角推导线束，复制坐标/强度，不能假定它像 FAST-LIO2 一样读取同名 `time/ring`。若用此路径，必须先验证时间如何抵达后端；不能仅凭订阅成功就上线。

历史版 [common_lib.h](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/include/common_lib.h#L44) 还硬编码 Avia 的 LiDAR→IMU 平移。本课要么按该值建立同轴外参的实际传感器装配，要么给独立上游副本做可追溯的参数化补丁。不能在平台 YAML 写另一组值，却以为原程序已经读取。

CustomMsg 包装只表达消息兼容。使用规则扫描束发布该消息时，必须说明是“具有 Livox 消息接口的教学扫描器”，不能宣称重现 Avia/Horizon 的非重复扫描光学模式。

### IMU、同步与外参

给估计器的数据应来自真实 Gazebo IMU 传感器的角速度与比力，使用同一个仿真时钟；不是把模型速度差分或 ModelStates 位姿加工成“传感器观测”。启动时保留静止初始化窗口，验证加速度模长、重力方向、角速度单位、时间单调性及点云末时刻的 IMU 覆盖。

FAST-LIO2/Point-LIO 的 `extrinsic_T/R` 表示 LiDAR 在 IMU 机体坐标中的位姿，可写为 `p_I = R_IL p_L + t_IL`。应由 SDF 的固定安装关系导出，关闭无必要的在线外参估计；TF、算法配置与实际安装须一致。Point-LIO 还需实际填写 IMU饱和范围、采样间隔和 `acc_norm`，不能将 `g` 与 `m/s²` 混用。参数依据见[固定版 Velodyne 配置](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/config/velody16.yaml)。

### LIO-SAM 额外需要解决的部分

[固定版 `imageProjection.cpp`](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp#L4) 需要 XYZ/intensity/ring/相对秒 time，并使用最后一点时间计算扫描结束。原版说明以带姿态输出的 IMU 为前提，建议高频输入；其姿态与原始加速度/角速度还可能具有不同外参。[数据准备依据](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/README.md#prepare-imu-data)。

A53 上线前须明确姿态来源：优先从模拟 IMU/磁力计实际消息实现并验证独立 AHRS/初始化链；或明确采用已审阅的六轴初始化改动并记录算法差异。不能直接把模型真值四元数塞入 IMU orientation 后隐去来源，更不能向估计器注入真值里程计来制造低误差结果。原版与 ROS 2 分支的 `extrinsicRot`、`extrinsicRPY` 必须分别核对。

## 3. 逐列实际采样的 Classic 方案

本节是基于上游接口的可行性设计，尚未运行验证。

Classic 普通 RaySensor 一次 `UpdateImpl` 调用整组 `laserShape->Update()`，随后赋一次 `lastMeasurementTime`。因此16×180束的单次扫描是当时几何的快照，不能事后按列号均匀捏造0—100ms时间。[RaySensor 实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/RaySensor.cc#L345)。

建议实现一个独立的本课扫描 ModelPlugin，使用真实 `MultiRayShape` 碰撞查询：

1. 通过 `Physics()->CreateCollision("multiray", lidar_link)` 创建绑定真实传感器 link 的碰撞体，加载小束数的 ray SDF 并 Init；保留碰撞体/shape 生命周期。
2. 在世界物理更新回调中，按本次真实仿真时间计算旋转方位；`SetRay` 改变16束的起止点。端点在 link 局部坐标中含固定传感器安装变换，**测得的输出点仍在当前 LiDAR 坐标中**，不预先用真值去畸变。
3. `MultiRayShape::Update()` 重置长度、更新各 ray，再做真实 ODE 场景碰撞。读取 `GetRange/GetRetro`；距离由该 API 包含最小量程，无需重复加 min。根据实际范围与配置的可控噪声生成点；无回波按明确规则处理。
4. 将本次采样的真实时间和16个 ring 一起缓存。扫描完成时组成相对第一列时间的 PointCloud2；A50 另编码相同采样记录为 CustomMsg。时间只来自实际进行射线检测的回调。
5. 回调中只做有界射线查询和缓存，消息发布/磁盘写入放到队列消费者；队列溢出明确报错/记录，不能静默造点补帧。世界重置或时间回退时清空未完成扫描和 IMU同步缓存。

此路径的接口与变换有源码支持：[MultiRayShape::SetRay/Update](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/MultiRayShape.cc#L168)，[ODEMultiRayShape 实际碰撞](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODEMultiRayShape.cc#L83)，[ODERayShape 随绑定 link 更新全局端点](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODERayShape.cc#L91)。后续实现必须验证回调时序、自体遮挡、最大量程和线程/锁，不能把接口存在写成实际通过。

**采样率要与物理步长匹配。** 16束×180列/100ms相当于1800列/s；在1ms物理步下无法声称每列相隔精确0.555…ms且都在独立更新检测。可选：

- 推荐先用0.5ms物理步、每步一列、每200列一圈，得到10Hz、16×200的真实分时扫描；列内16束共享实际采样时刻，明确是理想同步发射束。
- 若保留1ms物理步，可每步检测两个方位列、组成16×200扫描；同一步的两列必须保留**相同**时间，称为离散发射组，不伪装成连续均匀硬件发射。
- 若必须180列，则记录量化/抖动后的真实时间，或改物理步长；不能线性插值时间后宣称几何也在插值时刻测过。

先用静态墙面确认量程、ring与外参；再让平台有真实平移/旋转，对比不去畸变/正确去畸变结果。不能从已生成的理想世界点云按真值轨迹反变换，替代实际移动传感器在各时刻的射线检测。

## 4. 外部依赖的最小可执行结构

每种算法使用独立外部 catkin/colcon 工作区；第一代和第二代都叫 `fast_lio`，**不能放在同一个有效 overlay 中互相覆盖**。平台只保存锁文件、构建/启动适配与教材；上游源码和构建缓存放到单独用户依赖目录。

| 外部依赖 | 已核对的固定来源/需求 |
|---|---|
| FAST-LIO1 | C++14、catkin、PCL≥1.8、Eigen、OpenCV、Python开发链接、ROS消息/TF/PCL/eigen_conversions、Livox消息接口；[CMake依据](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/CMakeLists.txt) |
| FAST-LIO2 | 相近ROS/PCL/Eigen/Python依赖；gitlink必须另取 [`hku-mars/ikd-Tree@e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`](https://github.com/hku-mars/ikd-Tree/tree/e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4)，不能只下载主库压缩包而漏掉它；IKFoM在该树内 |
| Point-LIO master | C++14、catkin、PCL/Eigen、Python开发、Livox消息；所选树中 `include/ikd-Tree` 与 `include/IKFoM` 已是普通目录，不能仅因遗留 `.gitmodules` 再追随别的浮动分支；[CMake依据](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/CMakeLists.txt) |
| LIO-SAM | PCL/OpenCV、GTSAM、Boost timer、OpenMP、ROS TF及消息；GTSAM必须按发行版单独固定兼容构建，不能把当前系统任意版本当已兼容；[ROS1 CMake](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/CMakeLists.txt)、[ROS2 CMake](https://github.com/TixiaoShan/LIO-SAM/blob/08af3f32f01725372d4269838dc44c19c6d9e76b/CMakeLists.txt) |

### 只为消息生成而引入 Livox 接口

ROS 1驱动可锁为 [`Livox-SDK/livox_ros_driver@3d240d5666129e1a3052e78ee8487a04b08fdda3`](https://github.com/Livox-SDK/livox_ros_driver/tree/3d240d5666129e1a3052e78ee8487a04b08fdda3)。只需要 [`CustomMsg.msg`](https://github.com/Livox-SDK/livox_ros_driver/blob/3d240d5666129e1a3052e78ee8487a04b08fdda3/livox_ros_driver/msg/CustomMsg.msg) 与 [`CustomPoint.msg`](https://github.com/Livox-SDK/livox_ros_driver/blob/3d240d5666129e1a3052e78ee8487a04b08fdda3/livox_ros_driver/msg/CustomPoint.msg) 时，建立明确标识的 **message-only** catkin包，只做 message_generation/std_msgs/message_runtime，保留上游包/消息命名和定义。核对生成后的 datatype、MD5、序列化，再用于兼容编译；不能把消息包声称成完整硬件驱动。

不要直接运行完整驱动的原版 CMake 来生成两个消息：它[会尝试寻找/拉取并构建 Livox SDK](https://github.com/Livox-SDK/livox_ros_driver/blob/3d240d5666129e1a3052e78ee8487a04b08fdda3/livox_ros_driver/CMakeLists.txt#L85)。仿真实验不需要触发硬件SDK下载。消息包放在课程自己的隔离overlay，不覆盖已安装真实驱动；分两阶段先构建消息接口、再构建算法，避免历史源码目标依赖不完整带来的并行构建竞态。

FAST-LIO ROS 2分支依赖 `livox_ros_driver2`。可供继续审阅的消息来源为 [`21445540f0d100dc86a7e6df312dd70bbdb4afdf`](https://github.com/Livox-SDK/livox_ros_driver2/tree/21445540f0d100dc86a7e6df312dd70bbdb4afdf)，核查日期内提交于2026-09-21。ROS 2消息生成必须独立校验 rosidl类型，不能把ROS 1生成头拿来复用；其硬件SDK也不因此成为仿真必要依赖。

下载过程使用官方源、有限超时、固定提交和完整内容校验；网络不通时Windows中转，保持相同锁定内容。归档SHA256在实际下载时计算记录，不在规划阶段虚构。

### 许可证边界

FAST-LIO历史版、当前ROS1版和Point-LIO所选树的根 LICENSE 是GPLv2文本；部分遗留 package.xml 仍写BSD，不应据此把整个树重标MIT。保留完整上游许可证、版权和文件来源，补丁只放在独立上游副本并记录。算法作为独立进程通过ROS消息与MIT平台协作，不把GPL实现复制进平台模块。将来若分发其源码或二进制，独立给出对应源、补丁和许可证材料；不是把平台MIT许可覆盖到依赖上。[FAST-LIO LICENSE](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/LICENSE)、[Point-LIO LICENSE](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/LICENSE)。

LIO-SAM为BSD-3-Clause，仍保留其声明与第三方依赖许可。[LIO-SAM LICENSE](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/LICENSE)。ROS1 Livox消息文件在[其 LICENSE.txt 明确列出的MIT部分](https://github.com/Livox-SDK/livox_ros_driver/blob/3d240d5666129e1a3052e78ee8487a04b08fdda3/LICENSE.txt) 内，复制它们时连同版权声明保存。这是本项目依赖打包设计，不能替代对实际分发产物逐文件许可的核查。

## 5. 四课的验收不能只有节点在线

四课共同要求：固定源和构建记录完整；真实扫描/IMU输入持续、时间及外参契约成立；产生算法自己的里程计/地图；用独立评估进程比较实际运行轨迹的相对误差和合理的对齐后绝对误差；地图平面厚度/结构误差可测。记录有效点数、处理时延、丢帧、初始化状态、轨迹覆盖，不用RViz出现点云替代估计质量。

Gazebo真值只给独立评估器，不进入算法订阅链、初始化里程计或所谓“预测观测”；记录ROS图核对。开始只做足够约束的三维走廊/厂房，避免把纯平面、无转动、完美无噪声的退化演示当鲁棒性认证。

- **A50**：证明实际运行历史第一代二进制、独立特征提取与迭代滤波链；新增不同时间/外参故障作诊断，不能用FAST-LIO2结果代替。
- **A51**：在同一传感器记录上观察直接点到地图与参数变化，明确映射/滤波耗时和结构误差；与A50比较时统一输入、初始化和评价坐标。
- **A52**：逐点时间与IMU覆盖先验收，再验证原程序逐点处理产生的输出；测得多少输出频率就报告多少，不能把论文4–8kHz写成本机已达标。饱和试验必须对实际IMU消息施加已记录的受控饱和，不能修改真值轨迹来代替。
- **A53**：分别验证IMU预积分、点云去畸变、特征因子与地图优化；若开回环，单独验证回环约束/轨迹修正。GPS/姿态输入未实现就保持禁用或明确未验证，不能从真值制造完美传感器后隐去来源。

每课的ROS1和ROS2验收状态分别记录；某一ROS1课程通过不为ROS2课程背书。正式实现仍依照“单一活动目标→逐课编写→真实运行/交互检查→独立提交推送→核对远端SHA”的顺序执行。
