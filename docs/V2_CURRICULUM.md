# V2 课程地图：37 门主课与 2 项选做

V2 共39个独立项目，全部基于 Gazebo Classic 11 / ROS 1 Noetic；精确 SDK / ABI 条件及可运行性由每课声明和实际环境检测决定。每课提供目标、最少背景、离线素材、个人练习、参考说明和真实验收入口。课程文件存在与课程已经验收是不同结论，具体发行的实测范围以 `V2_VERIFICATION.md` / `.json` 为准。

## 从世界建模到反馈控制

| 编号与教材 | 主要学习对象与实际证据 |
| --- | --- |
| [L01 第一个落地方块](../courses/classic11_noetic/01-first-world/lesson.md) | 世界 / 模型 / link、重力下落、暂停与实际位姿 |
| [L02 积木房间](../courses/classic11_noetic/02-sdf-room/lesson.md) | SDF visual 与 collision，外观和接触行为 |
| [L03 坐标与姿态](../courses/classic11_noetic/03-pose-frames/lesson.md) | 局部 / 世界坐标、旋转和传感器安装 |
| [L04 可复用模型包](../courses/classic11_noetic/04-reusable-models/lesson.md) | 本地模型目录、资源解析与重复使用 |
| [L05 图形编辑器](../courses/classic11_noetic/05-gui-editors/lesson.md) | Model / Building Editor，保存并重新加载桌子和房间；需要实际GUI操作 |
| [L06 质量、质心与惯性](../courses/classic11_noetic/06-mass-inertia/lesson.md) | 刚体惯性、重心和实际动力学响应 |
| [L07 摩擦与碰撞](../courses/classic11_noetic/07-friction-collision/lesson.md) | 滑移、碰撞、反弹和接触参数 |
| [L08 求解步长](../courses/classic11_noetic/08-solver-stability/lesson.md) | 固定步长、数值稳定性与真实接触结果 |
| [L09 单摆与滑轨](../courses/classic11_noetic/09-pendulum-slider/lesson.md) | 关节、重力和被动运动，预测与测量分开 |
| [L091 单级倒立摆选做](../courses/classic11_noetic/09a-single-inverted-pendulum/lesson.md) | Python关键行填空、近直立反馈稳定；默认留空未评分 |
| [L092 双级倒立摆选做](../courses/classic11_noetic/09b-double-inverted-pendulum/lesson.md) | 双级耦合、状态反馈与局部稳定；默认留空未评分 |
| [L10 电机转台](../courses/classic11_noetic/10-motor-pid/lesson.md) | 力矩 / 速度 / 位置PID，目标、误差、限幅和实际关节 |
| [L11 差速小车](../courses/classic11_noetic/11-differential-drive/lesson.md) | 左右轮速、转弯与里程计，轮地接触产生车身运动 |
| [L12 Transport滑门](../courses/classic11_noetic/12-transport-gate/lesson.md) | Classic原生通信、请求与世界实际反馈 |
| [L13 场景展馆](../courses/classic11_noetic/13-scene-gallery/lesson.md) | 材质、灯光、地形与Actor；渲染和碰撞各自核对 |

## 从原始传感器到 ROS 系统

| 编号与教材 | 主要学习对象与实际证据 |
| --- | --- |
| [L14 接触保险杠](../courses/classic11_noetic/14-contact-bumper/lesson.md) | 实际碰撞、接触消息与所属实体 |
| [L15 激光扫描](../courses/classic11_noetic/15-laser-scanner/lesson.md) | LaserScan数组、角度索引、量程和无回波 |
| [L16 IMU转台](../courses/classic11_noetic/16-imu-turntable/lesson.md) | 姿态、角速度、比力和传感器坐标 |
| [L17 RGB相机](../courses/classic11_noetic/17-rgb-camera/lesson.md) | 真实图像、针孔投影和optical坐标 |
| [L18 深度相机](../courses/classic11_noetic/18-depth-camera/lesson.md) | 轴向深度、有序点云与无效样本 |
| [L19 六维力矩](../courses/classic11_noetic/19-force-torque/lesson.md) | wrench、称重、力臂和测量参考系 |
| [L20 五站传感器](../courses/classic11_noetic/20-sensor-stand/lesson.md) | GPS、Magnetometer、Altimeter、Sonar、Logical Camera的原生字段和模型边界 |
| [L21 时间账本](../courses/classic11_noetic/21-synchronization/lesson.md) | 仿真 / 墙钟、采样频率、有限单步与lockstep |
| [L22 ROS启动](../courses/classic11_noetic/22-ros-launch/lesson.md) | ROS节点、参数、launch与Gazebo连接 |
| [L23 URDF与Xacro](../courses/classic11_noetic/23-urdf-xacro/lesson.md) | 参数化机器人描述、关节和真实两关节臂 |
| [L24 RViz、TF与时钟](../courses/classic11_noetic/24-rviz-tf-clock/lesson.md) | 坐标树、robot_state_publisher、/clock与RViz |
| [L25 ros_control](../courses/classic11_noetic/25-ros-control/lesson.md) | Effort接口、PID、controller_manager生命周期、资源冲突与限位 |

L20 的 GPS 将仿真位置换算为地理坐标，不接收卫星；高度计使用加载时参考，不解算气压；声纳使用接触体近似，不传播声波；Logical Camera按视锥 / 包围盒给出模型，不做图像识别。它们可用来理解接口和建模假设，不能当作现实硬件的完整误差模型。

## Classic 插件、工程实验与综合任务

| 编号与教材 | 主要学习对象与实际证据 |
| --- | --- |
| [L26 ModelPlugin](../courses/classic11_noetic/26-model-plugin/lesson.md) | 更新回调中读取关节并施加真实转子力矩 |
| [L27 WorldPlugin](../courses/classic11_noetic/27-world-plugin/lesson.md) | 按模拟节拍投料、插入与管理真实实体 |
| [L28 SensorPlugin](../courses/classic11_noetic/28-sensor-plugin/lesson.md) | 原生测距、滤波、距离判断与传感器消息 |
| [L29 VisualPlugin](../courses/classic11_noetic/29-visual-plugin/lesson.md) | gzclient中的颜色 / visual缩放，对照服务端collision；需要真实渲染 |
| [L30 GUIPlugin](../courses/classic11_noetic/30-gui-plugin/lesson.md) | Qt / MOC / 信号槽、GUI线程、按钮与实际世界完成确认 |
| [L31 SystemPlugin](../courses/classic11_noetic/31-system-plugin/lesson.md) | 进程生命周期、真实重力配置和有限步采样 |
| [L32 记录与回放](../courses/classic11_noetic/32-record-replay/lesson.md) | 原生state.log录制、文件状态读取与隔离服务端回放 |
| [L33 多机器人](../courses/classic11_noetic/33-multi-robot/lesson.md) | 命名空间、独立控制和共同参考坐标，真实多机器人读回 |
| [L34 性能与批量实验](../courses/classic11_noetic/34-performance-batch/lesson.md) | 每行数据来自新服务端真实落体，区分端到端计时与纯物理性能 |
| [L35 故障诊所](../courses/classic11_noetic/35-fault-clinic/lesson.md) | 保存输入、复现、最小修复、同条件重跑与证据留档 |
| [L36 仓库综合任务](../courses/classic11_noetic/36-warehouse-mission/lesson.md) | 激光 / 里程计 / 接触、轮式运动、PD物理滑门、有限状态机及故障恢复 |
| [L37 机械臂推块](../courses/classic11_noetic/37-arm-push/lesson.md) | 真实2R关节、FK / IK、力矩驱动、腕部FT / 接触反馈、HOLD与失能恢复 |

L32 状态回放读取保存的世界状态，不等于重新执行控制算法，也不是逐条重发位姿的自定义动画。L34 的主要耗时包含进程启动、连接和读回，不以该值冒充纯ODE实时因子。L36 使用已知直通道，输出的真实扫描投影用于检查观测与轨迹；没有SLAM、全局路径规划或完整仓储调度。L37 的解析IK负责目标生成，执行依赖实际关节力矩与接触；不是通用操作规划、抓取或仅修改位姿的动画。

## 如何使用先修材料和验收

建议按主课顺序学习；091 / 092 在第9课之后选做，需先理解状态、反馈与基本线性控制。进入综合任务前，L36 回看07 / 10 / 15 / 22 / 24 / 26 / 33 / 35，L37 回看06 / 09 / 10 / 19 / 23 / 24 / 26 / 35。跨领域背景和官方资料在各课相应概念处提供；前置阅读入口保持只读，不改当前课程或重建世界。[环境与前置阅读流程](ENVIRONMENT_V2.md)

正常学习：预测 → 运行 → 观察真实测量 → 修改一个因素 → 重载 / 重跑 → 解释差异。验收记录实际输入、实体、时间、原始消息和清理结果。已加载、已通信、已执行和达到物理目标是不同阶段；自动判据、人工GUI观察、参考答案回归和未验证范围分开记录。[安装与验收入口](V2_INSTALL.md)
