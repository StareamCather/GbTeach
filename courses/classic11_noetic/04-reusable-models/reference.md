# 可复用模型包参考答案

本页只读展示流程、计算与诊断方法，不覆盖个人文件。先独立完成实验，再核对结果。

## 1. 三个文件各负责什么

| 文件或对象 | 负责的内容 |
|---|---|
| models/teach_crate/model.config | 可读元数据、模型发布版本、入口及SDF格式版本 |
| models/teach_crate/model.sdf | box/link的几何、碰撞、质量、惯性与默认模型定义 |
| world.sdf 的 include | 使用哪个包、实例名称、初始位姿 |
| crate_left 等运行实体 | 各自的状态，不会自动回写上述文件 |

包目录名 teach_crate 与 URI 对应，model.config 的可读名称可以不同。model.sdf 内的 model 名称是定义名字；本课 include 用 name 给出各个实例名，查询时使用后者。

`<version>1.0</version>` 是模型发布版本，`<sdf version="1.6">model.sdf</sdf>` 才声明入口与格式。版本元数据不是缩放参数或在线自动更新命令。

## 2. 搜索路径和离线迁移
若个人副本位于 WORKSPACE，GAZEBO_MODEL_PATH 应包含 WORKSPACE/models。解析 model://teach_crate 时，在这个父目录下寻找 teach_crate/model.config，再按其入口读取 model.sdf。

```text
GAZEBO_MODEL_PATH 中的一项：WORKSPACE/models
                           └── teach_crate/model.config
                                              └── 入口model.sdf
```

GAZEBO_RESOURCE_PATH 用于世界文件和媒体等其他资源，不能与模型包搜索路径互换。框架先保留系统 setup 提供的路径，再添加本课资源。

迁移时同时携带 world.sdf 和完整 models/teach_crate。若以后使用 meshes、materials，也要包含对应文件并检查引用。本课仅用本地几何和材质数值，不要求下载模型库，也不需本地 database.config。

## 3. 基准布局与实例独立性

| 实例名 | include初始位置m | 初始yaw rad |
|---|---|---|
| crate_left | (−1.5,0,1.25) | 0 |
| crate_right | (1.5,0,1.25) | 0.35 |
| crate_back | (0,1.5,1.25) | 0.7 |
| crate_front | (0,−1.5,1.25) | −0.35 |

四个实例均为动态模型。本课模型原点、box中心和惯性中心对齐；边长0.5 m，所以地面支撑后的中心 z 约为0.25 m，不能拿它与初始1.25 m直接比较。

暂停后只移动右箱，其他箱子应不动，文件也未被回写。重载后，新服务器从文件重新建立四个实例，初始布局仍由 include 决定，然后继续进行物理仿真。

共享的是模型定义，不是四个实例的状态内存。修改 model.sdf 后，已创建的实例不会实时同步；保存并结束、重开服务器会话后，各 include 才使用新定义。重开客户端和重置状态均不能替代重新解析模型包。

## 4. 单变量实验：高度改为0.6 m
质量1 kg不变，将尺寸改为0.5×0.5×0.6 m，collision 和 visual 的 size 都填写0.5 0.5 0.6。

均匀、轴对齐长方体的质心惯性为：

`Ixx=m*(sy²+sz²)/12`

`Iyy=m*(sx²+sz²)/12`

`Izz=m*(sx²+sy²)/12`

因此 Ixx=Iyy≈0.0508333333、Izz≈0.0416666667 kg·m²，交叉项为0。独立设计变量是高度；同步修改外观、碰撞和惯性是保持实体描述一致。

四箱稳定中心高度都从0.25升为0.3 m，增加0.05 m，初始高度1.25 m不变。保存后“结束实验→开始实验”或“重载项目”，再比较运行消息和落点。

## 5. 独立挑战：0.8×0.5×0.6 m
质量仍为1 kg，两处 size 都改成0.8 0.5 0.6。计算为：

`Ixx=(0.5²+0.6²)/12=0.0508333333`

`Iyy=(0.8²+0.6²)/12=0.0833333333`

`Izz=(0.8²+0.5²)/12=0.0741666667`

惯性单位为kg·m²，交叉项为0。稳定中心高度仍为0.3 m；增加水平方向尺寸，不会再次抬高这个中心支撑高度。实例应足够分散，避免相互接触改变姿态或落点。

给实例改名，只改对应 include 的 name，并按新名字查询，保留2至4个不同名称的实例；本课自动验收限定该范围，更多实例留后续批量课。修改 URI 会改变资源来源。改变尺寸和质量应编辑实际模型定义；本课没有任意 include 参数覆盖。

## 6. 错误定位

| 现象 | 检查与修复 |
|---|---|
| 找不到模型 | 核对URI拼写、大小写、目录名及模型路径中的父目录 |
| 包存在仍加载失败 | 核对model.config入口名称、格式版本及实际入口文件 |
| 换设备后缺网格/材质 | 检查依赖文件是否完整携带，以及各自引用 |
| 改文件却没变化 | 核对个人副本已保存，结束并重开服务器会话 |
| 只有一个实例移动 | 若只发运行时位姿命令，这是预期的独立状态 |
| 加载意外同名素材 | 检查路径列表中其他同名包，确认实际来源 |

目录改名实验先结束当前实验，将个人 teach_crate 改为 teach_crate_missing，URI不改，再尝试运行。看完日志后结束会话，恢复目录名再开始。应先修复资源链，而非修改物理步长或尝试联网。

## 7. 验收范围
验收读取个人当前 world.sdf、model.config、model.sdf，核对实际实例的布局、几何和物理参数，再主动抬升并观察落地，最后恢复原位姿及暂停标志。文件存在本身不等于运行实体已正确加载；目录讲解和复用原理仍需自己解释。

不了解目录组织？[查看官方模型结构](https://classic.gazebosim.org/tutorials?tut=model_structure&cat=build_robot)。需要核对路径？[查看组件与环境变量](https://classic.gazebosim.org/tutorials?tut=components&cat=get_started)。需要查include字段？[查看SDF 1.6规范](https://sdformat.org/spec/1.6/world/)。
