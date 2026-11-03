# 参考：自定义MoveSlide协议与PD闭环

## 两行控制律

位置误差为 `target-position`。速度阻尼与当前运动方向相反，所以默认阻尼项是 `-8*velocity`；完整力为 `clip(18*error-8*velocity,-10,+10)` N。完整函数在独立的`reference.py`中；默认参考演示不会写入`student.py`，也不会自动授予作业通过。

对1 kg、物理阻尼0.8 N·s/m的理想滑轨，未触发限幅时，加入参考反馈后，可把局部线性误差运动近似看作 `e'' + 8.8*e' + 18*e = 0`。该估计忽略ROS取样、服务延迟和0.06仿真秒分段施力的影响；实际结果必须由Gazebo关节读取和反馈验证。

初始从0到0.35 m时，比例项为6.3 N，低于10 N限幅。若从0去0.8 m，初始未限幅力为14.4 N，将被截到10 N。理解这点后再比较不同目标距离的到位时间。

## 执行次序与结束语义

目标进入后先检查target和timeout的有限性、范围（位置±1 m，墙钟预算0.5～12 s），再开始单目标执行。每轮先检查取消/墙钟期限，再读取Gazebo实际位置/速度、发布这次观测反馈、检查取消与到位条件，未结束时才计算PD、清除旧有限力并调度新0.06仿真秒力。到位条件同时要求误差<0.025 m和速度<0.07 m/s；最终清除后还需读回确认，不能把旧反馈当作终止时刻的状态。

取消优先走任务结束路径：清除本课调度的关节力、返回取消终止状态与真实结束位置。`PREEMPTED`表示任务取消已被处理，不表示速度为0。理想无新力阶段的速度在物理阻尼下逐渐衰减；不要在取消实现中通过设置位姿/速度来伪造正常停止。

目标墙钟预算耗尽时也结束任务。Gazebo暂停会冻结物理和仿真施力时长，但不冻结墙钟预算。调用者的RPC等待预算只限制自己等待多久，不能保证远端请求被撤销；不确定请求不自动重放。

## 自定义Action构建信息流

`MoveSlide.action`三段定义→catkin的action/message generation→个人`ros_ws/devel`中的Python消息模块→本课`ros_python_paths`子进程配置→Action服务端和客户端。`goal/status/feedback/result/cancel`消息共用目标身份；业务goal、feedback、result字段与Action外层包装不是同一层。

需要生成的消息放在当前个人工作空间，不向系统Python目录拷贝，不改变全局shell配置。构建失败要先查prepare日志和依赖，不要靠随手添加任意工作空间到全局PYTHONPATH掩盖问题。

## 观察和验证

- 服务脉冲：请求获响应后，独立关节数据必须实际变化。
- 正常Action：真实位置接近0.35 m，反馈非空，速度小；同时检查结果与独立读回。
- 取消：先确认收到反馈并发生运动，再取消本客户端goal，检查PREEMPTED和`reached=false`；观察惯性延续。
- 非法目标：越界、NaN、Inf应报告失败，不向物理引擎传播。
- 暂停超时：不同位置目标不能在暂停中达成，墙钟到期仍应退出。

自动参考验收与学员作业分开。默认`reference_demo`的`assignment_pass=false`是预期状态；切换`student`前先补完两行并执行`student-check`，再做真实闭环实验。改变一个增益后应重启，保证新个人函数被加载。

[GoalStatus官方定义](https://github.com/ros/common_msgs/blob/noetic-devel/actionlib_msgs/msg/GoalStatus.msg)可用于对照ACTIVE、PREEMPTED、SUCCEEDED和ABORTED。消息定义中的时间单位与业务语义由本课明确约定；不要把字段名带`time`就默认当作同一种时钟。


## 接口发现与私人节点目录

生成Python消息类使用本课devel Python路径；`rosmsg`按ROS_PACKAGE_PATH查找schema。prepare将catkin实际生成的7个`.msg`镜像到个人源包msg目录，实跑原生`rosmsg show -r`及MD5校验，记录于`build/action-build.json`。该镜像是派生产物，不能用手写schema替代真实生成。

roslaunch的默认工作目录可能是ROS_HOME。本课包的node.py先从自身路径定位个人练习根目录并chdir，再运行instrument，保证读取个人reference.py/student.py和记录个人build产物。用户不需要改全局shell或CMAKE_PREFIX_PATH。

`student-check`和student模式完整验收的算式基准固定为Kp18、Kd8、±10N。探索其他增益后应恢复原题再作业验收；其他增益算式检查失败不直接说明物理闭环不稳定。单独算式检查始终assignment_pass=false；只有student模式真实完整闭环验收也通过时，才标记assignment_pass=true。


## Noetic施力任务队列的清除语义

读取/施力查找接受 `workshop::slide`，而Noetic的 `clearJointForces` 以任务所指关节的 `GetName()` 短名比较，应传 `slide`。没有匹配也会返回成功，不能据此证明删除了多少任务。[官方源实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L1599)

本课独立世界只有一个名为slide的关节，清除短名有确定范围。控制循环清掉旧任务后才调度新的0.06仿真秒施力；清除失败会留下多个可能重叠的任务。正确清除不改变已有速度。

可反证的操作是：从真实零位零速开始→暂停→安排0.2仿真秒脉冲→以短名slide清队列→恢复仿真→在时钟实际前进后读取位置和速度，预测仍接近零；另起干净对照省略清除，恢复后预测应运动。暂停不会消耗finite job的仿真持续时间，不能靠多等几秒墙钟让任务自然过期。这里只说明诊断原理和预期，实际结果由新增运行报告给出。
