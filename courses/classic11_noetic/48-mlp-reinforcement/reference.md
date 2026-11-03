# 参考 · REINFORCE的两个局部计算

在`reward_to_go`中替换倒序循环内的空行，保留写入结果的一行。

```python
accumulator = rewards[i] + gamma * accumulator
```

在`mean_score`中，固定采样到的raw，求高斯log概率对均值的导数。

```python
return (raw_actions - means) / (sigma * sigma)
```

若rewards=[1,2,3]且γ=0.5，G=[2.75,3.5,3]。若raw=[2,−1]、μ=[1,1]且σ=2，score=[0.25,−0.5]。算法使用`gamma**t * (G_t - baseline_t) * score_t`作为每条转移的均值梯度信号，再乘网络参数对均值的导数。外层γ^t把G_t换算到回合起点。第一批baseline全零，以后用上一批同t的G均值。全批次的均值梯度经过总范数裁剪，按梯度上升更新。

采样变量raw的高斯score与实际施力F=8tanh(raw/8)分开计算。由于变换固定，给定实际力时其逆变换及雅可比与网络参数无关，因此用记录下来的raw就能计算score。力平方代价使用F。

默认每轮4个40步的真实回合，6轮更新。前后各从−0.55m和+0.55m运行120步。每对使用相同的独立评价种子与ε序列，策略、状态和施力可不同。最终评价从保存的末轮权重重新加载。报告分开记录训练梯度、评价回报及回报差。默认1440次动作与28次重置合计28828个原生物理步，总数见`report.json`，每回合的原生重置与动作记录见`on-policy-trajectories.jsonl`。

正常动作在暂停世界推进20个1ms原生物理步，回合起点另有一个实际重置步。初态来自物理回读。每次正常结束确认恢复原暂停状态。请求结果未知时关闭连接，避免重放动作。参考演示和代数检查保持`assignment_pass=false`，个人实现须经过重新采集、更新和评价。

补充推导可阅读[策略梯度、reward-to-go和baseline](https://spinningup.openai.com/en/latest/spinningup/rl_intro3.html)与[Williams的REINFORCE原论文](https://link.springer.com/article/10.1007/BF00992696)。
