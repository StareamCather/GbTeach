# 坐标与姿态参考答案

本页只读展示推导与结果，不替换个人练习。基准值对应本课 world、命名 frame、静态 model 和 link；修改参数后需重新计算。

## 1. 变换的方向与组合
记 `T_A_B` 为“把 B 坐标中的点转换成 A 坐标”的变换，包含旋转 R 与平移 t：

`p_A = R_A_B p_B + t_A_B`

两段变换应这样组合：

`R_A_C = R_A_B R_B_C`

`t_A_C = t_A_B + R_A_B t_B_C`

采用列向量、右手正转，RPY 对应 `R=Rz(yaw)Ry(pitch)Rx(roll)`。本课只需计算绕 z 的矩阵：

```text
Rz(theta) = [ cos(theta) -sin(theta)  0 ]
            [ sin(theta)  cos(theta)  0 ]
            [     0           0      1 ]
```

theta=π/2 时，局部 (x,y,z) 转为参考坐标中的 (−y,x,z)。先旋转局部向量，再叠加原点平移；不要把原点平移也绕世界零点旋转一次。

## 2. 基准完整链路
四段局部定义为：

| 变换 | 平移m | RPY rad |
|---|---|---|
| world → rig | (2,1,0.2) | (0,0,π/2) |
| rig → mount_frame | (0,0,0.6) | (0,0,0) |
| mount_frame → sensor_marker | (1,0,0) | (0,0,0) |
| sensor_marker → body | (0.15,0,0.05) | (0,0,0.25) |

逐段计算：

`t_world_mount = (2,1,0.2) + Rz(π/2)(0,0,0.6) = (2,1,0.8)`

`t_world_marker = (2,1,0.8) + Rz(π/2)(1,0,0) = (2,2,0.8)`

`t_world_body = (2,2,0.8) + Rz(π/2)(0.15,0,0.05) = (2,2.15,0.85)`

| 对象 | 世界位置m | 世界yaw rad |
|---|---|---|
| rig 模型原点 | (2,1,0.2) | 1.5707963268 |
| mount_frame 的加载时位姿 | (2,1,0.8) | 1.5707963268 |
| sensor_marker 模型原点／白球 | (2,2,0.8) | 1.5707963268 |
| sensor_marker 的 body／红盒中心 | (2,2.15,0.85) | 1.8207963268 |

`gz model -m sensor_marker -p` 查询模型原点，不是红盒中心。详细消息中 body 的 pose 是模型相对位姿 (0.15,0,0.05,0,0,0.25)；与模型世界变换组合后才得到表中最后一行。不要误取 visual 内的零 pose。

本例 yaw 可相加，只因为旋转共轴；加入 roll 或 pitch 后，仍应按完整矩阵顺序组合。

## 3. 引导变式与独立挑战
每次从基准开始，只改列出的项。在个人 world.sdf 中修改并保存，然后点击“重载项目”。

| 变式 | 模型世界位置m | 红盒中心世界位置m | 模型yaw／body yaw rad |
|---|---|---|---|
| 仅 rig yaw 改为0 | (3,1,0.8) | (3.15,1,0.85) | 0／0.25 |
| 基准yaw；标记局部平移改为(0,1,0) | (1,1,0.8) | (1,1.15,0.85) | π/2／π/2+0.25 |
| 基准；仅 body 局部yaw改为−0.25 | (2,2,0.8) | (2,2.15,0.85) | π/2／π/2−0.25 |

侧向挑战中，`Rz(π/2)(0,1,0)=(−1,0,0)`，所以模型世界 x 从2减到1；body 的平移仍沿世界方向 (0,0.15,0.05)。

深入变式中，body 自身 yaw 不改变它的原点相对模型的位置，只改变几何朝向。新的世界 yaw 为1.3207963268 rad。本课保留 visual 在 body 内的 pose 为零；若进一步移动 visual，其几何中心还会改变。

## 4. 删除 relative_to 的答案
只把 sensor_marker 的：

```xml
<pose relative_to="mount_frame">1 0 0 0 0 0</pose>
```

改为：

```xml
<pose>1 0 0 0 0 0</pose>
```

这是 world 的直属 model，pose 默认相对 world。模型原点变为 (1,0,0)、yaw=0；body 原点为 (1.15,0,0.05)、yaw=0.25。mount_frame 和 mount_axes 的定义没有改动。

“没有 relative_to 就相对 world”不能推广到所有元素。本课 link 的默认参考是所属 model，visual 的默认参考是所属 link；默认规则取决于元素类型。

## 5. 参考、附着与机械连接

```xml
<frame name="mount_frame" attached_to="rig">
  <pose relative_to="rig">0 0 0.6 0 0 0</pose>
</frame>
```

两次出现 rig，回答的是不同问题：relative_to 说明初始数值相对谁表达；attached_to 说明命名 frame 附着到谁。二者可以不同，不能用“相对谁”替代“附着到谁”。

frame 不具有质量、碰撞或物理关节。另一个 world 直属 model 使用 `relative_to="mount_frame"`，只得到初始解算结果，不会被自动绑定。两个模型都静态、都不落下，也不能证明存在连接。实际动态多 link 机构还需要正确的惯性与 joint，后续课程会展开。

mount_axes 是独立静态模型，只按 mount_frame 的初始位姿画轴。运行中移动 rig 后，不能把这个独立代理停在原处解释为 attached_to 语义失效；本课没有把抽象 frame 实时渲染成实体。rig 自己 link 上的轴 visual 则属于 rig，会随 rig 移动。

验收实际改变 rig 的世界 x 分量0.35 m，检查独立标记保持原世界位姿，再恢复 rig。这个反例检查“没有机械绑定”，不验证一个不存在的 TF 发布器。

## 6. 四元数与等价姿态
本页采用 `(x,y,z,w)` 顺序。绕 z 旋转 theta 的单位四元数为：

`q = (0,0,sin(theta/2),cos(theta/2))`

| 对象 | 单位四元数(x,y,z,w)，近似值 |
|---|---|
| 基准标记模型，yaw=π/2 | (0,0,0.7071067812,0.7071067812) |
| 基准body，yaw=π/2+0.25 | (0,0,0.7897480482,0.6134313494) |
| 深入变式body，yaw=π/2−0.25 | (0,0,0.6134313494,0.7897480482) |

四个分量同时取反仍表示同一姿态。验收按单位四元数点积绝对值比较旋转，不会因整体符号相反或 Euler 角跨过±π而误判。本课 SDF 1.7 pose 使用六数 RPY；四元数用于理解和读取消息，不能直接改成七数 pose。

## 7. 核对范围与继续阅读
自动验收检查当前变换图、真实 model/link 位姿、姿态等价性和独立模型不绑定的反例，恢复临时移动的 rig；不改变原暂停状态。它不替你画坐标轴，也不会把红盒变成相机。

ROS 常用机体轴为 x前、y左、z上，optical frame 为 z前、x右、y下。本课 body 是自定义标记坐标，不自动等于相机光学坐标系。

不了解版本支持？[查看官方版本路线图](https://sdformat.org/tutorials/developers/roadmap/)。需要核对默认参考和附着关系？[查看 SDF 1.7 位姿语义](https://sdformat.org/tutorials/specification/pose_frame_semantics/1.7/)。需要核对单位和轴约定？[查看 REP103 原文](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)。
