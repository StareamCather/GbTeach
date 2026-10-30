# 参考解法：图形建模与保存链

本页只读，不替你生成或覆盖 student_table、student_room。离线 reference_* 包提供可观察的目标，但不能代替创建、保存、接入和重载的图形操作。

## 桌子的几何关系

先把模型编辑时的地面作为 z=0，所有 box 零旋转，visual 与 collision 的几何局部 pose 均为零。尺寸依次为 x、y、z，位置是各 link 中心：

| 部件 | box 尺寸 / m | 中心位置 / m |
| --- | --- | --- |
| 桌面 | 1.2 0.8 0.1 | 0 0 0.8 |
| 左前腿 | 0.1 0.1 0.75 | -0.5 -0.3 0.375 |
| 右前腿 | 0.1 0.1 0.75 | 0.5 -0.3 0.375 |
| 左后腿 | 0.1 0.1 0.75 | -0.5 0.3 0.375 |
| 右后腿 | 0.1 0.1 0.75 | 0.5 0.3 0.375 |

桌面底部0.75 m、顶部0.85 m；腿底部0、顶部0.75 m。四腿外边界 x=±0.55、y=±0.35，均在桌面投影 x=±0.6、y=±0.4 内。这张桌子是 static 组合模型；模型内有多个 link 并不等于它们已具有真实机械关节。

操作顺序可为：放置桌面 → Inspector 设 visual/collision 尺寸和 link pose → 创建一腿 → 复制或新建另外三腿 → 设置位置 → Model 页勾 Static → 拖动一次并撤销/重做 → Inspector 复查 → Save As 到 student_table → Exit Model Editor。

点退出菜单后，出现 Exit/Cancel 确认时还要点击 Exit；Cancel 不会退出编辑器。后面的 URI 切换与重载必须在实际回到普通场景后进行。

Model Editor 的快捷键是 Ctrl+Z 撤销、Ctrl+Shift+Z 重做。对齐和吸附帮助放置，但不会创建 joint。给桌面改颜色只影响 visual 的材质；物理桌面仍来自 collision。

重新加载后可右键学生桌子实例选 Edit Model 修改，再 Save As 到自己的 student_table 目录。确认位置，保留旧包后再覆盖；退出编辑器、use-exports、重载，才能把保存结果重新用于本课 world。不要因此推断建筑编辑器也能重开原来的二维施工过程。

## 为什么导出高度可能改变

Classic 11 新模型的导出逻辑会以顶层 link 中心位置的算术平均重新选原点。本例的 z 均值为：

```text
(0.8 + 4 × 0.375) / 5 = 0.46 m
桌面导出局部 z = 0.8 − 0.46 = 0.34 m
腿导出局部 z = 0.375 − 0.46 = −0.085 m
碰撞最低点 = −0.085 − 0.75/2 = −0.46 m
```

模型内部几何没有因此变形。若 include 无旋转、模型 z 放到0.46 m，脚底才重新落在世界 z=0；若机械地把 include z 写0，则会埋入地面。本课工具根据实际导出碰撞边界接入，不能把0.46当成适合所有模型的固定补偿。原点也不是本例模型的物理质心；增加一个 link 就可能改变这个平均值。[实际导出算法见 Classic 11 的 ModelCreator::GenerateSDF](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/gui/model/ModelCreator.cc)。

## 房间与门洞

GUI 练习可取外包3×2.5 m、高2 m、墙厚0.12 m的矩形房间。Wall Inspector 的 Length 包含墙端厚度延伸：水平端点中心距离为 `3−0.12=2.88` m，竖直为 `2.5−0.12=2.38` m。若自行把二维矩形中心选作(0,0)，四角端点可取 `(±1.44,±1.19)`；对应 Length 分别为3和2.5 m，不能把这两个长度当成端点中心距离。离线参考包用于观察墙和门洞的构成，不要求其墙端搭接、原点或 link 分组与 GUI 导出逐项相同。前墙中央门洞宽0.8、高1.4 m，底部与墙底齐平。

在 Building Editor 中连画矩形墙 → 双击墙核准尺寸 → Door 吸附前墙并居中 → 调整门洞尺寸 → 看三维预览 → Save As 到 student_room → Exit Building Editor → 在 Exit/Cancel 确认中点 Exit。端点吸附有助于闭合；需要偏离网格时按 Shift，不把三维操作的 Ctrl 规则混进来。这里没有可动门板，也不应在模型树中寻找铰链控制器。

导出的开口可能由左墙块、右墙块和上方过梁围出；visual 和 collision 都须保留空隙。若前墙整体跨度为3 m、门洞居中且宽0.8 m，两边各剩 `(3−0.8)/2=1.1` m，过梁高度为 `2−1.4=0.6` m。实际文件还会包含墙端搭接和原点变换，不能依赖固定生成名称判断正误。

本课真机保存的房间为 SDF 1.7、static=1、4个墙 link，门墙 link 内同时包含两侧1.1 m墙块和0.8×0.12×0.6 m过梁，各有 visual/collision。其模型原点在一面墙中点，墙的局部 y 从0延伸到−2.38 m；原点并不位于房间中心。模型根 pose 表示保存时的放置，接入 world 时由 include 位姿指定模型原点位置，不能再把根 pose 当成额外平移叠加。

单变量实验把门洞宽度从0.8改为1.0 m：左右边缘各向外移0.1 m，两侧余宽各减0.1 m；门洞顶面仍是1.4 m，过梁底面不变。应在退出前完成对照并保存。建筑编辑器没有 Undo，退出后导出包也不能还原成原二维建筑编辑历史；需要第二版时重新画墙并保存旧版本。[官方建筑编辑器说明](https://classic.gazebosim.org/tutorials?tut=building_editor&cat=build_world)给出了这种限制。

## 保存、接入和重载的答案

```bash
python3 lab.py show-paths
# 在编辑器中完成两包的 Save As，并退出编辑器
python3 lab.py use-exports
# 回应用点击“重载项目”，新场景就绪后验收
```

Save As 的 Location 是最终模型文件夹。student_table 与 student_room 内应各有 model.config 及它声明的模型 SDF。接入工具校验两个包，备份 world 后切换 URI，仅根据碰撞最低面补偿 include.z，保留 include.x/y、yaw 和学生包内容；不会把偏置原点自动移到房间中心。重载后 world 的引用应为 model://student_table 和 model://student_room，运行实例仍名为 table_example、room_example。

普通场景 Save World 与编辑器 Save Model 保存不同对象；保存一个另外命名的 world 文件不会自动改变应用入口。只重开 Gazebo 窗口也不会重新解析模型。要回看样例，执行 `python3 lab.py use-reference` 后重载；再用 use-exports 切回学生成果。

文件链可逐层排查：**实际保存目录 → model.config 的入口 → 模型 visual/collision/pose → world 的 URI 与 include pose → 当前服务器中的实体 → 实际接触行为**。资源解析先于实例化，实例化先于物理接触；在前一层失败时，后续画面不具备证明力。

## 怎样判断自己完成了

自动验收检查当前引用、受支持的导出几何、静态属性、桌面支撑和门洞空隙。人工检查还包括：你能从空编辑器创建两包，演示模型编辑器的撤销/重做和对齐，区分两种吸附，指出实际保存路径，并在重载后证明加载的是学生 URI。手写或复制一份可验收 SDF 不证明掌握鼠标编辑技能。

保留个人过程截图和预测记录；“查看实验记录”可辅助核对开始、重载与验收事件，但这些事件同样不能代替编辑器操作证据。需要继续理解 link/visual/collision，可看[官方模型编辑器示例](https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b3)；保存包的目录规则见[官方模型结构](https://classic.gazebosim.org/tutorials?tut=model_structure&cat=build_robot)。
