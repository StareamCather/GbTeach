# 参考推理：视觉外观、渲染证据与物理边界

## 三个预测

只改color，预期目标的客户端材质颜色改变；模型pose和collision尺寸不改变。只改visual_scale，预期目标显示范围改变，而服务端box仍为一米。改变相机，像素投影会改变，即使材质和scale保持不变。

这三件事由不同对象负责。不要从一个截图直接推断物理尺寸，也不要从服务端未变推断客户端一定没有变化。

## 插件位置与生命周期

父对象应为 `beacon::body::indicator` 对应的真实Visual，SDF声明在visual内，注册类型为VisualPlugin。Load保存对象和配置、建立通信与PreRender连接，之后的渲染回调处理待执行命令。

Init计数应按实际Classic路径记录。Visual::LoadPlugin在对象已初始化时会调用Init，LoadPlugins还会遍历插件调用Init，因此不能机械要求“一次Load必须只有一次Init”。实现应让Init可重复，而不是依赖重复调用改变颜色或逐次放大对象。

`python3 lab.py reset`是排队恢复个人模板color与visual_scale的自定义命令。它不重载共享库、不重新创建世界、不使时钟归零；顶部“重置模型状态”和世界复位也不自动等同于课程reset。

## 正确的处理顺序

参数与源检查 → 个人SDF与实际编译 → 服务端实体就绪 → 真实客户端创建Visual并加载插件 → 接收命令时只排队 → PreRender取队列 → 设置独立材质、颜色与局部绝对scale → 实际属性读回 → 真实截图核对 → 原生服务端实体对照。

接受序号用于证明命令接收，已应用序号用于证明渲染线程处理过命令；两者不应被合并成一个“成功”标志。即使属性已经读回，画面仍应另外检查。

## 材质为什么需要唯一副本

材质可被多个visual共享。对一份公共材质改diffuse或lighting，可能改变所有引用对象。本课先为indicator取得唯一材质，再设置颜色，避免黄色参考物等无关对象跟着改变。

本课目标和黄色参考物开启lighting，同时设置ambient/diffuse/emissive。emissive使表面显示自身颜色，不创建照亮邻物的世界光源。最终像素仍由实际渲染路径决定，不能把RGBA输入与所有截图字节直接画等号。

实机初稿中，目标actual_diffuse虽为红色，真实窗口里的目标和参考物却都为白色。该反例说明关闭lighting并不保证当前路径中的预期颜色；后续配置改为lighting=true而保持像素阈值，仍需新实测判断结果，不能把API颜色读回当作画面通过。

## 缩放应怎样解释

本例scale=1.4是对一个visual的局部绝对比例，原始显示边长1米且父层级比例为1。代码不应每帧执行“当前scale乘1.4”，否则会持续增长，和设定比例的语义不同。

客户端Visual更新的图形几何与sceneNode不等于服务端的collision对象。服务端模型pose、collision box size及实际实体身份应另行读取。GUI里的碰撞显示可作为人工补充练习。

透视截图中的面积比不能简单强定为1.4²。固定相机下，物体向相机方向的深度范围也随显示缩放改变；离散像素和可见面边界又会产生额外差异。应使用有依据的像素范围与参考物对照，并保留真实原图。

## 单变量对照

| 场景 | 保持 | 改变 | 预期 |
| --- | --- | --- | --- |
| 默认 | 原始物理场景/固定相机 | 红色、scale1 | 作为比较基准 |
| 仅蓝色 | scale1/相机/物理几何 | color=blue | 目标颜色变化 |
| 仅放大 | 红色/相机/物理几何 | visual_scale=1.4 | 目标显示范围增大 |

所有场景的碰撞尺寸都应从服务端实际数据核对。若参考物像素范围也改变，应先检查相机或窗口条件，而不是立刻修改视觉缩放算法。

demo/verify会在同一个实际相机中依次捕获红、蓝、绿的scale1画面，再捕获绿色scale1.4画面。捕获流程为PreRender保留原CaptureData、连接ConnectNewImageFrame并启用读回；Camera完成实际像素读回后，新图像事件给出buffer/尺寸/深度/格式，再用静态Camera.SaveFrame保存，最后恢复原CaptureData。不能靠通用PostRender事件或猜测帧数断言ImageData非空。早期空缓冲失败和新图像事件是否完成应分别留证。

独立维护验收另从本次Gazebo窗口抓取像素，插件给出的相机PNG不是唯一真值，也没有合成预期图片。演示尝试恢复入场颜色、scale和暂停，相机则留在固定课程视角；帧计数与命令历史不倒回。

configured_color/configured_visual_scale描述个人模板；color/visual_scale描述当前命令值；actual_diffuse_rgba/actual_scale_xyz来自Visual接口。queued、applied与captured序号分别关联入队、渲染处理和实际保存帧，不可合并为一个抽象成功状态。

capture_armed表示等待真实新图像，capture_image_events计数来自实际图像事件，capture_width/capture_height为事件尺寸。当前SDK的VisualPtr属于std指针体系而UserCameraPtr属于boost指针体系，辅助连接和弱引用应分别匹配，不能把前一类对象的指针写法机械复制到后一类。

## 关闭与重新打开GUI

VisualPlugin属于客户端实例。关闭GUI后，原来的视觉命令通道应不可用；重开GUI产生新的实例，应重新初始化一次相机并读回身份。个人参数经重载准备后仍是该次实例的初始值，之前临时的颜色命令不应被误当成新模板。

该过程不应结束其他项目的Gazebo，也不应销毁本课服务端物理场景。平台只管理本次创建的进程和窗口。

## 进一步挑战

状态驱动颜色需要额外定义消息到达、未知状态、超时与客户端重开的行为，再将状态放入渲染队列。第28课的距离过滤消息可作为输入，但视觉标识是否正确，还需真实通信与真实画面验证。

闪烁指示器需要先选时间语义：模拟时间随世界暂停，墙钟通常继续前进。必须明确实验要表达哪一种行为，保留实际时间与帧数据，不能通过重写时间戳掩盖不连续。

相关依据：[VisualPlugin接口](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/common/Plugin.hh)、[Visual载入与材质/缩放实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/rendering/Visual.cc)、[Classic Visual API](https://osrf-distributions.s3.amazonaws.com/gazebo/api/11.0.0/classgazebo_1_1rendering_1_1Visual.html)。
