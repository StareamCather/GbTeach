# 参考：组合变换与时间配对

## 默认几何的推导

TF链为world→rail→cart→probe。前两段旋转为单位阵，world→rail平移(0,0,.7)，rail→cart平移(q,0,0)，cart→probe平移(.1,.15,.05)、绕z旋转90°。

probe原点的world坐标是(q+.1,.15,.75)。对非零局部点(.2,0,0)，先乘安装旋转得到(0,.2,0)，再加平移，得到(q+.1,.35,.75)。仅检查原点不能发现漏旋转；非零局部点能。

完整算法是逐行计算 `sum(R[i][j]*p[j]) + t[i]`。时间差是 `abs(t1-t0)`。完整独立函数在reference.py，student文件保留两处填写；student-check与默认参考物理验证都不自动授予作业通过。

## 方向、最新共同时间与原生交叉检查

`lookup_transform('world','probe',Time(0))`返回把probe表达转换到world的变换。Time(0)代表路径上最新共同可用时刻；动态缓存与静态边共同决定可查询性，不是任意混用不同时间的最新数据。

TF结果来自广播/订阅链。独立真值来自GetLinkState的workshop::probe位姿，再用该位姿变换同一局部点。原生服务与TF查询有各自时间，应保留这些信息，不宣称原子同时采样。比较四元数还需考虑q与-q的等价，不能只逐项比较符号。

安装从个人SDF中probe的relative_to=body pose读取。z=.05→.10使world高度.75→.80；偏航pi/2→.3时，局部点旋转为(.2*cos(.3),.2*sin(.3),0)，再加当前q及安装平移。

## 同步实验的来源与预期

pose保存同一次真实joint观察中的q，companion保存同一次观察中的v，单位分别是m与m/s。frame_id均为rail。两路共享教学观察事务的stamp，第二路不是另一个独立传感器，更不是IMU。

companion延迟发布但保留stamp，Exact仍可匹配。影子companion_offset把stamp加.012s，Exact应拒绝；ATS slop=.02s应允许配对，但stamp差仍然存在。Noetic实现使用严格小于slop的跨度判断，恰好等于容差也不配。slop=.005s时应拒绝这对影子消息。

ATS按stamp选候选，不看seq。Noetic rospy用各Publisher自己的计数覆盖Header.seq，无订阅时不增长，因此不同通道raw seq不必相同。验收用独立source_observation_seq说明共同来源，再以publication_trace中的(topic kind,stamp,publisher_seq)映射接收包；不改写raw seq，边界未映射对单列计数，不猜来源。受控观察最小周期.06仿真秒，配合offset≤.03与slop≤.02，避免拿下一事务借配。队列不足仍会造成丢配。

stamp由节点clock回读标记观察事务，原始服务与clock非原子。接收时间和stamp属于不同证据：主机monotonic接收时间不能直接减ROS仿真stamp。所有raw seq/stamp/receipt应先保存，再解释延迟与人为偏置。

## 验证边界

改变实际SDF安装后，TF和原生probe应该一起改变；不改硬编码验收答案。延迟/offset/slop单变量实验应分别验证预期的配与不配，不通过重写stamp制造“同步成功”。真实相机/激光有自己的原生采样周期与时钟问题，应另行建模。

等待受墙钟预算限制，暂停不使等待无限延长。RPC超时不撤销远端执行，不重放不确定运动请求；Noetic清力队列使用短名slide。个人源与练习保留，实际通过状态以运行报告为准。

实际查看命令使用ROS 1安装中的 `rosrun tf tf_echo world probe`，程序仍用TF2 Buffer。延迟由仿真时间调度。至少8组完整源迹核对companion的publish调用仿真时刻≥query_stamp+delay；这不是线上发送时间。另按同一source_observation_seq关联的主机monotonic接收差，中位数>0验证正delay下整体较晚送达，不等于声明的仿真延迟数值，不能将两种时钟跨域相减。

`/workshop45/publication_trace`为String JSON，保存source_observation_seq、query_stamp_ns、三路实际发布后的Header.seq/stamp和publish调用点仿真时间；验收捕获的源迹、接收原始值和配对关联写入build/verification.json。它补充数据来源证据，不把Header.seq重新解释成跨话题统一编号。
