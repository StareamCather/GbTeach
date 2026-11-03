# 参考：行为克隆的计算图与真实闭环

## 两处局部答案

同样的代数答案独立保存在`reference.py`，不会自动填写个人`learner.py`。

`learner.forward`接收已经按固定物理尺度归一化的输入；不要在函数中重复缩放。

```python
hidden = np.tanh(np.atleast_2d(X) @ W1 + b1)
return (hidden @ W2 + b2).reshape(-1)
```

输出[N]归一化动作，单样本也是长度1数组；施力路径再乘8N并限幅。矩阵乘法使用`@`，逐元素非线性使用`np.tanh`，最后一层保持线性。

`mse_derivative`：

```python
return 2 * (prediction - target)
```

`network.backward`负责/N，不能重复平均。它用更新前的W2传播隐藏梯度，乘以tanh导数1−H²，生成四组梯度后统一按全局L2范数上限5缩放，最后执行学习率0.025的梯度下降。梯度裁剪限制参数更新，执行器±8N限幅限制物理动作，二者不能互相替代。

## 数据和证据边界

600次真实专家转移按整段episode划分：0—7训练480行，8—9留出120行。固定特征为[(target−x)/1m,v/(2m/s)]，标签为专家力/8N；尺度不是从数据拟合，报告保存这些尺度与结构先验。留出MSE换回N²，训练损失无量纲。若拓展z-score，才需要在划分后仅用训练集拟合并保存均值/标准差。未来状态不得进入当前动作输入。数据文件顶层的`rows`保存各行`target_m/x_m/v_m_s/features`、专家力/归一化标签、`before/after/transition`与另一次RPC读回；应按文件中的schema和实际字段解析。

默认训练目标均为0。特征已经编码平移结构先验；真实闭环从x=0、v=0分别跟踪target±0.3m，各170步；报告依据x−target评估末段误差。这是两个新目标测试。训练误差、整段留出动作MSE和真实闭环末段误差回答不同问题，不能用其中一个替代另外两个。

`python3 lab.py demo`使用完整参考实现并标记reference_demo/assignment_pass=false。`student-check`只检查个人局部代数，仍不授予物理通过；`student`使用个人前向与导数重新采集、训练、真实闭环，通过后才授予作业状态。默认2600次全批次更新不是2600次物理转移。

## 原生物理接口流程

保存原暂停状态→连接会话私有socket→在暂停模式读原生状态→计算动作→后台工作线程请求World::Step(20)→更新回调逐步施力→核对Iterations/施力回调恰好增加20、原生时间增加20×dt→另一条state RPC读取真实UpdateEnd快照缓存，确认动作后版本/数值→计算下一动作。该RPC不是另一套物理API；每次动作前还应与上次快照相同，从而发现额外自由滑行。reset另由物理线程设置请求位置/速度并真实推进1步，返回积分后初态；10个专家episode与2个闭环目标共12辅助步，默认940次标准转移×20+12=18812物理步，两类分开报告，`reset_observations`保存请求值、真实初态及native_reply，闭环trial另存`actual_initial_state/reset_native_reply`。Python不推进动力学；训练耗时不产生自由滑行。正常end确认恢复原暂停状态；结果未知时仅断开连接，报告恢复未确认，不再发可能与原生步进竞争的请求。

超时请求的远端结果可能未知，关闭连接并停止本次流程，不自动重发或用旧状态继续。不要与GUI单步/继续或第二控制客户端并发操作同一实验。每次`results/run-<uuid>/`保留专家数据、训练记录、冻结后重载的策略、真实闭环轨迹和报告，最近结果另写`results/imitation-report.json`与`build/verification.json`。参考或个人成功状态、暂停恢复和原始时间/步数证据均以本次报告为准。

## 为什么低MSE仍可能闭环失败

监督损失在专家访问的状态上比较动作；闭环由策略自己的动作决定后续状态。小偏差可能把系统带到训练覆盖较少的区域，产生更多偏差。增加训练轮次并不能保证消除这种分布问题。当前专家未饱和部分为线性PD，因此线性回归是必要的简单基线，MLP成功并不自动说明非线性结构有收益。

[NumPy矩阵乘法](https://numpy.org/doc/stable/reference/generated/numpy.matmul.html)、[tanh](https://numpy.org/doc/stable/reference/generated/numpy.tanh.html)与[标准差](https://numpy.org/doc/stable/reference/generated/numpy.std.html)可核对计算定义；[imitation官方BC说明](https://imitation.readthedocs.io/en/latest/algorithms/bc.html)说明监督示范学习；[DAgger原论文](https://proceedings.mlr.press/v15/ross11a.html)讨论策略诱导状态分布。后者是拓展阅读，本课并未实现该算法。
