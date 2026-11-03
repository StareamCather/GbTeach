# 参考 · FAST-LIO2 对照的核对路径

直接模式从基本筛选后的原始点开始。特征模式先按束编号与局部几何挑选面点。两组接入同一FAST-LIO2后端，用去畸变、降采样后的候选拟合地图平面并建立残差。用同一份采集和配置复查特征开关，估计过程各自重新初始化。[固定预处理源码](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/src/preprocess.cpp#L284)

点时间等于扫描首列时刻加相对偏移。`time=0.0275`s对应后端点时间27.5ms。`ring`表示垂直束编号。检查筛选后末点、扫描参考时刻与IMU覆盖，再理解输出的处理时间。

给定单位法向n=(1,0,0)、面上点q=(3,0,0)m、世界点p=(3.2,1,0)m，残差nᵀ(p−q)=0.2m。将p改为(3.2,2,0)m时值保持0.2m。平面残差沿法向提供约束，转动信息还依赖点的力臂。IKFoM在每帧内沿同一传播先验迭代，在旋转流形上应用小旋转误差。[FAST-LIO2状态估计](https://arxiv.org/html/2107.06829)

ikd-Tree用空间近邻支持局部平面拟合。新点以更新状态插入，移动窗口把离开区域的点按盒删除，达到条件时局部重建。查询、插入、删除与重建各有不同的计时边界。[ikd-Tree原论文](https://arxiv.org/abs/2102.10808)

比较时先确认原始输入相同，再读点数、轨迹覆盖、误差与已定义的耗时窗口。高分位耗时反映慢帧，回放与整次运行耗时还含消息调度和启动。若两个模式的输出时间不同，分别匹配各自时间上的真值。


## 填空

```python
import math

def voxel_key(point, leaf_m):
    return tuple(math.floor(coordinate / leaf_m) for coordinate in point)

def translation_jacobian(unit_normal_world):
    return tuple(unit_normal_world)
```

体素边界使用向下取整，所以−0.01/0.2的索引是−1。固定对应平面时，t沿世界坐标轴增加ε，残差增加该轴法向分量乘ε。三个分量组成雅可比行。运行`python3 student_check.py`检查局部公式。运行`python3 adapter.py run`采集一份实际传感器bag，再分别回放给直接与特征模式。


从最新`build/verification.json`定位运行目录。`paired-timing.json`按原始首点时间配对，分别读PRE、SCAN、两者之和及对应点数。SCAN覆盖t0到t5，其中包含里程计发布调用，后续点云发布和地图快照另在计时窗外。每种模式的`trajectory-comparison.json`按各自处理时刻评估，`laser-map.pcd`保存实际局部树快照。两个模式比较基于同一份bag的SHA256。
