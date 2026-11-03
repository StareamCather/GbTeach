# 参考 · LIO-SAM的两套图与回环证据

图优化的每个因子把局部观测变成误差约束。标量残差r以标准差σ白化为r/σ，平方目标因此按1/σ²加权。多维协方差可写作Σ=LLᵀ，白化时求L⁻¹r。[iSAM2原论文](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.html)

预积分图的变量是校正时刻的X、V、B。IMU因子连接相邻时刻的位姿和速度并用起端偏置，偏置之间另接随机游走因子。激光校正给位姿先验。iSAM2更新后，以新的偏置重置积分，并重新传播校正时刻后的IMU。高频惯性输出和地图关键帧输出有各自时间与用途。[固定预积分代码](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imuPreintegration.cpp#L370)

地图图维护关键帧位姿。先有首帧先验和相邻里程计约束。回环候选需做ICP验证，通过后按相对Pose3的BetweenFactor入图。复查时应将候选、通过ICP、队列消费、iSAM2更新及历史关键帧修正分开。上游GPS位置因子为可选路径。[回环及地图图代码](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/mapOptmization.cpp#L1477)

前端逐点时间来自扫描首时刻加偏移。固定代码对点做扫描内旋转补偿，平移项在`findPosition`中保持为零。用同一bag的两模式比较时，先核对点时间与IMU覆盖，再看关键帧与回环证据，最后按同一规则评价轨迹。[固定去畸变代码](https://github.com/TixiaoShan/LIO-SAM/blob/0be1fbe6275fb8366d5b800af4fc8c76a885c869/src/imageProjection.cpp#L473)


## 标量练习答案

```python
def whitened_residual(state, observation, sigma):
    return (state - observation) / sigma

def weighted_position(observations, sigmas):
    weights = [1.0 / (sigma * sigma) for sigma in sigmas]
    return sum(w * z for w, z in zip(weights, observations)) / sum(weights)
```

令代价对x的导数为零，得到x Σw_i = Σw_i z_i，其中w_i=1/σ_i²。两个标准差相同时得到算术平均。测量0和10m分别对应标准差1和3m时，最优解为1m。运行`python3 student_check.py`核对。


## 真实运行的复查路径

运行`python3 adapter.py run`后，从`build/verification.json`定位本次目录和两个模式。先核对同一份bag的SHA256，再读各模式`factor-events.json`中的投影、IMU预积分、关键帧图与已应用回环。用`historical-keyframe-revisions.json`查看开启组旧关键帧位置如何变化。`global-map.pcd`与`map-before-loop.pcd`是已发布地图快照。在线轨迹和最终关键帧轨迹分别按两组共有采样时刻比较。

默认输入含Gazebo IMU传感器给出的理想姿态，航向初始化关闭。用于评估的独立真值从IMU连杆坐标移到LiDAR原点后再做首位姿对齐。扫描时间、IMU采样时间及图优化发生时刻各有用途。
