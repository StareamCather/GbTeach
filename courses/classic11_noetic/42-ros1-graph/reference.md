# A42 参考流程与独立填空

`instrument.py`只实现本课：Gazebo原生关节读回 → PoseStamped发布 → TCPROS接收 → Master/连接头核对 → 本课模型复位并确认时钟不回退 → 一次有限力 → 新状态消息和独立关节查询确认运动。没有隐藏调用后续Action、TF或bag课程。

ready使用一个锁存发布者，只发布一次。之后创建的新订阅器收到的最近状态来自ROS的锁存机制，不是仪器悄悄重发。验收报告中的订阅者是验收节点自身，不会被冒称为用户已经手写了subscriber。

queue_size限制发布队列，.04s是每轮额外墙钟休眠；服务耗时会延长周期。GetJointProperties没有原生采样Header，状态header.stamp只是仪器读完服务后的clock读回，不能视为原子物理快照。

清理流程按Noetic实际实现区分名称：原生查询和ApplyJointEffort用 `workshop::slide`；clear_joint_forces用 `slide`，因为其删除条件比较 `Joint::GetName()`。该服务无匹配也返回true，不能把响应当成清队列的行为证据。本课隔离世界仅有一个slide，裸名称不会误清其他模型同名关节。正确行为检验是暂停→调度有限力→清理→继续→读真实位移；清除力本身仍不清除已有速度。[源码依据](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L1742)。

student.py的答案位于独立reference.py：

```python
def velocity(x0,x1,t0,t1):
    if t1 <= t0:
        raise ValueError("Need increasing timestamps")
    return (x1-x0)/(t1-t0)
```

检查 `velocity(0,.2,1,1.1)` 应约为2m/s；反向位移给负速度；相同或逆序时间必须报错。实际订阅差分含异步读回和发送时延，需保留原始时间，不能事后修改让曲线好看。

prepare只检查/初始化本课，不覆盖个人instrument.py与student.py。修改仪器后重启本课；加载指纹不同会阻止把旧进程当新代码验收。验收会将本课模型/速度复位，不改变仿真时钟和个人文件。参考实验通过的报告保留 `assignment_pass=false`，不能替代你完成个人速度函数与订阅器。
