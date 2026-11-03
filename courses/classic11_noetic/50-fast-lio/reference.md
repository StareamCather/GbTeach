# 参考 · 第一代 FAST-LIO 的时间、坐标与滤波

## 点偏移与去畸变

本课跟读第一代固定历史树。HORIZON输入的`offset_time`以纳秒计，历史前端写入`curvature`时除以10⁶，进入后端后再除以10³换算为秒。25ms等于25000000ns，进前端后的数值为25。若扫描首时刻12.0s，该点为12.025s。同步依赖的实际末时刻需检查特征点的时间顺序与末点字段。[前端转换](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/feature_extract.cpp#L183) · [后端同步](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/laserMapping.cpp#L583)

若首时刻p_Lj=(10,0,0)m，扫描期间设备沿+X移动0.1m且没有旋转，则末时刻LiDAR系下同一点为(9.9,0,0)m。完整公式为`inv(T_WI(t_e) T_IL) T_WI(t_j) T_IL p_Lj`。T_IL从LiDAR变到IMU，换向时旋转转置，平移乘逆旋转再取负号。历史树实际使用同轴旋转与固定平移。[补偿代码](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/IMU_Processing.hpp#L280)

## 几何如何约束状态

单位法向n=(1,0,0)、面上点q=(3,0,0)m、估计世界点p=(3.2,1,0)m时，r=nᵀ(p−q)=0.2m。把p沿Y平移1m，残差仍为0.2m。这说明单个面约束优先感知法向偏差，多方向结构能补充几何约束。论文给出面及边残差，固定树的前端输出surface与corner，后端地图匹配主路径取surface构造点到面观测。[第一代论文](https://arxiv.org/html/2010.08196) · [历史后端](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/laserMapping.cpp#L842)

一次扫描内的迭代以同一传播先验为基础。先推进IMU状态与协方差，按每点时间补偿到统一参考坐标，再用局部地图平面建立残差。小旋转通过Exp复合到当前姿态，随后在新状态重新线性化。帧末保存更新状态与协方差，再将相应坐标下的点加入地图。预积分因子把一段IMU样本压缩成关键时刻之间的相对增量约束，属于不同的组织方式。[历史IMU传播](https://github.com/hku-mars/FAST_LIO/blob/6e1fa94351316eafac5fcf55c5fcd575f3e89829/src/IMU_Processing.hpp#L196) · [预积分论文](https://arxiv.org/abs/1512.02363)

复查对照时，从输入时间、消息字段和外参开始，再看有效特征、观测方向、轨迹误差和地图厚度。用相同原始数据及对齐规则比较，记录实际改动的唯一变量。


## 填空与实机复查

点采样时间保留整数纳秒，直接相加即可。若先转浮点，较大的绝对时间可能吞掉低位纳秒。

```python
return scan_start_ns + offset_ns
```

点到面残差取有符号投影。单位法向令输出量纲为米。

```python
return sum((p - q) * n for p, q, n in
           zip(point_world, point_on_plane, unit_normal))
```

`python3 student_check.py`检查两处公式。`python3 adapter.py run`在运行的本课会话采集140圈原生分时雷达、200Hz IMU和独立真值，写两路传感器bag，再运行固定历史前端与后端。先从`build/verification.json`定位本次结果目录。对照`sensors.bag`中的首列采样时间和点偏移、`report.json`中的末点时间与来源，再看`trajectory-comparison.json`逐样本的估计、对齐真值和误差。首个匹配位姿确定一次SE(3)变换，后续各时刻保持相同变换和单位比例。
