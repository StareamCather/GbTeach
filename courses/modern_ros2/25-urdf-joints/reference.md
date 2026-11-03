# 参考 · URDF、原生反馈与正运动学

URDF给出两条消费路径。Gazebo经SDFormat建立物理模型与控制系统，robot_state_publisher读取同一份描述作为运动学树。固定关节可能在物理转换时合并，ROS运动学仍按URDF定义发布固定边。[SDFormat转换](https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/) · [Robot State Publisher](https://github.com/ros/robot_state_publisher/blob/ros2/README.md)

`JointState.name`与位置、速度等数组按索引对齐。根据`shoulder`查实际角度，再按源时间查询TF。对于肩关节绕Z轴，肩的固定安装变换左乘RotZ(q)。末端另有静态偏移时，再右乘工具的固定变换。

查不到结果时从创建、原生关节发布、桥接、ROS JointState、描述参数、TF frame和时间逐层检查。运行`python3 lab.py`对照+0.65、−0.45、0rad三组指令、实际关节、TF及原生模型运动。


本课`tool`固定偏移(0.82,0.10,0.12)m，肩安装高0.70m。若测得q=0.65rad，预期工具位置约(0.592,0.576,0.82)m、偏航0.90rad。q=−0.45rad时约(0.782,−0.267,0.82)m、偏航−0.20rad。q=0时为(0.82,0.10,0.82)m、偏航0.25rad。`preserveFixedJoint`保留了Gazebo中可观测的工具连杆。完整验收还对照`/gt/arm_link_poses`的独立原生位置。


检查`build/robot.converted.sdf`的固定锚、转动肩关节和保留的固定工具。报告的`joint_stamp_ns`用于查对应时刻TF。原生连杆位姿先从模型系变到base系再对照。三阶段中的中位保持额外验证关节在零目标处稳定。
