# 参考：参数的来源、采用时机与真实响应

## 校验题的两处判断

允许数值类型可以写成 `type(value) in (int, float)`，明确排除bool。范围判断应先用该类型条件短路，再检查 `math.isfinite(value)` 与 `.2 <= value <= 3`，避免先对字符串调用有限性检查。

`reference.py`给出独立完整函数。学生函数检查与生产仪器分开；student-check通过也不自动把参考物理演示标记为学员作业通过，assignment_pass保持false。

## 参数生命周期

`namespace`与`pulse_scale`先作为launch arg进入XML替换；scale通过typed double转换后写为节点私有参数。仪器启动读入并缓存这个scale。YAML中~pulse_force则每次Trigger重新读、检查类型和范围，再计算 `effective_force = requested_force * cached_scale`，并检查实际施力上界5N。

group相对scope_example与节点私有~scope_example是两个不同完整名称，默认分别为/workshop44/scope_example和/workshop44/instrument/scope_example。读取服务器中的新pulse_scale不能证明缓存已改变；本次服务记录的实际采用值与真实关节响应才是证据。

source-only Python包通过当前个人ROS_PACKAGE_PATH发现；不生成自定义消息，也不要求全局catkin overlay。node.py先定位个人练习根，避免ROS_HOME工作目录误读；初始化保留个人instrument和校验函数。

## F与2F的比较为何要控制仿真时间

从同一零位零速开始，清队列后安排在原生仿真时间+0.08s开始的0.2s脉冲。verify在开始后约0.45仿真秒采样，以WorldProperties前后时间读数括住独立joint查询。墙钟sleep无法保证相同仿真时刻。

对质量m、线性阻尼c、恒定力F与脉冲时长T，脉冲结束时：

```text
vT = (F/c) * (1-exp(-c*T/m))
xT = (F/c) * (T-(m/c)*(1-exp(-c*T/m)))
```

脉冲结束后再经过d秒：

```text
v = vT * exp(-c*d/m)
x = xT + (m/c)*vT*(1-exp(-c*d/m))
```

本课m=1kg、c=.8N·s/m、T=.2s。共同比较时刻0.45s对应d=.25s。在未碰限位、未饱和且模型线性时，F加倍使位移和速度约加倍。

为控制服务查询时刻的小差异，可用同一线性阻尼响应核把真实样本换算到共同0.45s时刻。时间校正只改变时间标尺，不把位移直接指定为预期值；原始关节值、原生时间括区、校正依据与校正结果均应保留。不要只给校正后的比例而隐去原始数据。

## 非法输入、恢复与不确定调用

非法类型、NaN/Inf、越界请求应在安排新施力前失败；state用类型/可打印值表示非法内容。恢复合法参数后可继续工作，无需以节点崩溃表示参数拒绝。

RPC调用者预算限制等待，不能取消已在远端执行的请求。结果不确定时不重放施力请求。读取/施力关节使用workshop::slide；Noetic清任务队列使用短名slide。有限持续时间是仿真时间，暂停不会按墙钟耗尽，正确清队列也不会把已有速度设为零。

非默认namespace变体须同时对齐启动arg、ready、verify和lab.py的__ns重映射。静态`roslaunch --nodes`只能证明解析结果；真实接口与采纳参数仍需运行验证。默认与变体的实际通过情况见各自报告，本参考不提前代替实测。
