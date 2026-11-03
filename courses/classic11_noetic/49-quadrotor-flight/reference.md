# 参考 · 高度PID与十字布局混控

`altitude_acceleration`把三个加速度贡献相加。积分由外部控制周期更新并限幅。

```python
return 3.0 * error - 2.6 * vertical_speed + .15 * integral
```

令前后两路之和为A，左右两路之和为B。由A+B=T和A−B=tau_z/k得到两个合量。由前后之差tau_y/L、左右之差tau_x/L进一步求四个推力。函数保持前、左、后、右的编号顺序。

```python
L = .22
k = .018
return np.array([
    total/4 - pitch_torque/(2*L) + yaw_torque/(4*k),
    total/4 + roll_torque/(2*L)  - yaw_torque/(4*k),
    total/4 + pitch_torque/(2*L) + yaw_torque/(4*k),
    total/4 - roll_torque/(2*L)  - yaw_torque/(4*k),
])
```

代回T、tau_x、tau_y、tau_z四式检查交叉项。也可把限幅后的四个推力代回，比较实际合量与请求值。正横滚增加左路并减少右路，正俯仰增加后路并减少前路，正偏航增加前后两路并减少左右两路。

读真实状态，计算位置加速度与姿态目标，再把姿态误差和机体系角速度转换成力矩。把总推力和力矩混控、限幅，经插件施加四路力与反作用扭矩。每条控制动作有20个1ms原生步，回合起点额外推进一个实际重置步。记录保留真实反馈与扰动时间。每回合400条动作和一次辅助重置，两个回合共16002个原生物理步。阵风窗口[2,3.5)s对应75条动作，尾段统计窗口(6.5,8]s取75个反馈样本。逐回合的状态、控制与执行记录在`flight-trace.jsonl`，摘要和通过状态见同一运行目录的`report.json`。

`reference.py`保留独立参考。演示和代数检查的`assignment_pass`为false。个人闭环运行将实际调用这两个函数，再按两条飞行轨迹给出验收结果。
