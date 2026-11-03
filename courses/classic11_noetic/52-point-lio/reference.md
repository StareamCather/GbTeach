# 参考 · 等时分组、事件率与观测模型

相同采样时刻的点共享同一次状态时间。对排序后的整数纳秒，按连续相等值分组即可。

```python
from itertools import groupby

def group_sizes(sorted_stamps_ns):
    return [sum(1 for _ in items) for _, items in groupby(sorted_stamps_ns)]

def event_rate_hz(distinct_stamps_ns):
    if len(distinct_stamps_ns) < 2:
        return 0.0
    elapsed_ns = distinct_stamps_ns[-1] - distinct_stamps_ns[0]
    return (len(distinct_stamps_ns) - 1) / (elapsed_ns * 1e-9)
```

例如[0,0,500000,500000,1000000]ns的组长为[2,2,1]，不同采样时刻为三个，跨越1ms，所以事件率为2000Hz。运行`python3 student_check.py`核对局部公式。

Point-LIO的观测模式把ω和比力作为状态，IMU更新将实际读数同状态加零偏的预测比较。原代码逐轴标记近量程分量。相应激光时间组继续利用局部地图的点到面观测更新。[原始实现的IMU观测与饱和分支](https://github.com/hku-mars/Point-LIO/blob/1510c3bbf1743f254d83d0b22fbabb5b2d729f6b/src/Estimator.cpp#L372)

`python3 adapter.py run`将同一份雷达与IMU记录回放两次。扫描发布模式使用扫描结束时间，高频模式在成功点组更新处发布该组时刻。看报告时区分输入点数、唯一采样时间、原生更新计数、里程计消息数、header事件率及订阅墙钟到达率。若某次输出点数更多，先检查它统计的是回波、时间组还是发布消息。


最新`build/verification.json`定位运行目录。内部更新看各模式的`native-scan-diagnostics.json`，发布与接收间隔看`odometry.json`。扫描首时刻的两模式内部组数配对见`paired-filter-counts.json`，扫描末时刻的里程计配对见`scan-end-comparison.json`。后者保留未找到高频对应时刻的缺口。地图产物分别是初始化地图`initial-map.pcd`和最近配准扫描`registered.pcd`。
