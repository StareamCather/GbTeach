# 用图形编辑器制作桌子与门洞房间

## 本课做成什么

用 **Model Editor（模型编辑器）**组合一张静态工作台，再用 **Building Editor（建筑编辑器）**画一间带门洞的小房间。把它们各自保存成模型包，接入本课 world，结束旧会话并重新加载。你不需要 ROS、C++、CAD 软件或在线模型库。

初始场景中的桌子和房间来自独立的离线参考包，用于观察目标与检查环境。学生模型目录起初不存在；参考场景通过验收不代表已经完成图形编辑。完成本课需要亲自创建、检查、保存并重新加载两份成果。

## 先分清保存的对象

**link** 是模型内的实体单元；**visual** 决定外观，**collision** 决定接触。**static** 表示整个模型不参加自由运动：本课用它固定桌面、桌腿和墙体，不需要先学习关节。把动态模型的几个部件放在一起，并不会自动把它们连接起来。

| 操作 | 保存或改变什么 | 怎样使本课下次仍能加载 |
| --- | --- | --- |
| 编辑器中的 Save / Save As | 当前模型包的 model.config、model.sdf 等资源 | 保存到个人练习的 models 下，再让 world 引用它 |
| 普通场景中的 Save World | 当前世界的 SDF 场景 | 必须确认保存到哪个文件；另存的世界不会自动成为本课入口 |
| 拖动普通场景中的模型 | 当前运行实例的位置 | 不会自动改写本课 world.sdf 或模型包 |
| 应用的“重载项目” | 停止本课旧进程，并从个人文件重新建场景 | 先保存文件和退出编辑器，再重载 |

点击“开始实验”，从本课命令区或“打开实验终端”运行 `python3 lab.py show-paths`。记下它输出的**个人练习目录**以及 student_table、student_room 的绝对保存路径。后面的课程资源编辑与导出都写到这个副本，不修改安装目录中的课程模板。

```text
个人练习目录/
  world.sdf
  lab.py
  models/
    reference_table/    离线观察样例
    reference_room/     离线观察样例
    student_table/      由你在模型编辑器中保存
    student_room/       由你在建筑编辑器中保存
```

## 项目一：在 Model Editor 中组合静态桌子

1. 在 Gazebo 菜单选择 **Edit → Model Editor**，快捷键 Ctrl+M。编辑模式会暂停仿真；先确认左侧出现模型编辑器面板。用 Insert 页的 Box，在视图区点击放置一个方块。Esc 返回选择模式。
2. 双击该 link，或右键选择 **Open Link Inspector**。在 Visual 页把 box 尺寸设为 `1.2 0.8 0.1` m，再在 Collision 页设置相同尺寸。检查两边几何的局部 pose 保持零；在 Link 页设置桌面中心位置 `(0,0,0.8)`、零旋转。改完输入框先点击别处提交，再确认对话框。
3. 再创建一条腿，尺寸为 `0.1 0.1 0.75` m，visual/collision 一致，中心高度0.375 m。复制或重新插入另外三条腿，放到桌面四角下方。可使用参考答案中的坐标；先精确设置一个，再理解对称关系。桌腿顶面应为0.75 m，刚好接触桌面底面 `0.8−0.1/2=0.75` m。
4. 在 Model 页勾选 **Static**，核对当前模型由桌面和四腿组成。练习一次 Translate（平移）工具拖动，再用 Ctrl+Z 撤销、Ctrl+Shift+Z 重做，最后恢复正确位置。不要把 Ctrl+Y 当作本课的重做键。
5. 多选两条腿，试用顶部 **Align（对齐）**工具，使它们在同一高度；再用 Inspector 核对结果。对齐改变位姿，不创建连接。Translate 工具下按住 Ctrl 拖动可练习网格吸附；Rotate 为旋转、Scale 为缩放。吸附用于粗放置，最终尺寸和位姿仍以 Inspector 及保存的 SDF 为准；本课不靠拖拽估计数值。
6. 选择编辑器 **File → Save As**。Model Name 填 `student_table`，Location 填 show-paths 给出的完整 student_table 文件夹路径。Location 是**最终模型目录**，不是 model.sdf 文件名，也不是仅填上一级 models。保存后检查该目录内确实有 model.config 和模型 SDF，再选 **File → Exit Model Editor**。出现 Exit/Cancel 确认时，点击 **Exit** 才会真正返回普通场景；Cancel 会留在编辑器。

模型编辑器在首次导出新模型时可能重新选择模型原点。五个 link 的中心均值不是桌脚所在平面，也不是按质量计算的质心。因此保存后的 link 局部 z 可能与刚才输入的数值不同；判断桌高要组合模型、link 和几何的 pose，不能只看一个 z 字段。本课接入工具按导出碰撞最低面补偿 include.z，使模型接地，保留学生模型文件。

不了解 Inspector、visual 与 collision 的对应关系？[查看官方模型编辑器操作示例](https://classic.gazebosim.org/tutorials?cat=guided_b&tut=guided_b3)。

## 项目二：在 Building Editor 中画门洞房间

1. 在普通场景选择 **Edit → Building Editor**，快捷键 Ctrl+B。左侧是工具，二维平面用于画墙，三维预览用于观察结果。墙的尺寸在 Inspector 中以米表示，不用屏幕像素估计最终大小。
2. 选择 **Wall**：点击起点、点击下一端点可连续画墙，双击结束；Esc 或右键取消正在绘制的段。先画矩形，再双击墙打开 **Wall Inspector**，校准高度2 m、厚度0.12 m。以外包约3×2.5 m为目标时，注意 **Length 包含墙端的厚度延伸，不等于两端点中心的距离**：水平墙端点中心相距2.88 m时 Length 为3 m，竖直墙相距2.38 m时 Length 为2.5 m。用端点坐标、Length 和三维预览共同检查，不把3 m误填成中心线距离。默认绘制会按0.25 m和15°等规则吸附；按住 Shift 可临时解除建筑平面的吸附。这与前面三维平移的 Ctrl 吸附是两套操作。
3. 选择 **Door**，移到前墙上让它吸附到墙，再点击放置。双击门洞设置宽0.8 m、高1.4 m，居于前墙中央、底部落在墙底。用三维预览确认它穿透墙厚。这里的 Door 是**墙上的开口**，没有门板、铰链或开关控制；Window 同样表示墙洞。
4. 在退出前完成下面的“门洞单变量实验”，并再次检查四周墙、门洞与三维预览。建筑编辑器**没有 Undo**；删错墙时，其上附着的门窗也会被删除。按阶段 Save As 保留版本比依赖撤销可靠。
5. 用编辑器 **File → Save As**，名称 `student_room`，Location 填 show-paths 给出的 student_room 绝对路径。确认保存成功，再选 **File → Exit Building Editor**；在弹出的 Exit/Cancel 确认中选择 **Exit**，并确认已经返回普通场景。退出后不能把这个导出模型重新打开成原来的二维建筑编辑过程；再次编辑 SDF 或模型实体也不会恢复原来的画墙历史。需要重画时新建一版，并先保存旧包。

保存的房间通常把带开口的墙拆成若干实体块，为每块写出 visual 与 collision。文件中未必存在一个名叫 Door 的可动物体；应检查实际空隙，而不是依赖生成的 link 名字。[官方建筑编辑器说明](https://classic.gazebosim.org/tutorials?tut=building_editor&cat=build_world)同时列出了这些编辑限制。

建筑编辑器导出的模型原点也不保证是房间中心；本课真机样例的原点位于一面墙中点。include 的位置放置的是这个原点。接入工具只做 z 方向接地补偿，不把房间沿 x/y 自动居中，所以重载后的房间可能相对参考样例偏向一侧；应结合导出的 link pose 判断位置，需要时调整个人 world 的 include.x/y 后重载。

## 门洞单变量实验与独立挑战

在建筑编辑器尚未退出时，先记录0.8 m门洞的左右边界，再只把宽度改为1.0 m，保持中心、墙高和门洞高度不变。先预测：两侧边缘各向外移动多少？上方过梁的底面会不会变化？检查 Inspector 和三维预览后保存。已退出的同学需重新绘制第二版，不能用“重新进入建筑编辑器”假装恢复原草图。

独立挑战：不照抄参考文件，在 Model Editor 里完成桌子并演示一次撤销/重做；在 Building Editor 里完成门洞房间，说明两套吸附的不同。给桌面改一种颜色，核对 collision 没随材质消失，再保存两包。通过下一节的接入流程，让重载后的场景确实使用你的学生模型，而不是仍展示参考包。

## 从导出包到可重复运行的 world

退出两个编辑器后，在本课实验终端运行：

```bash
python3 lab.py use-exports
```

工具先检查两份学生包是否完整、几何是否属于本课支持范围，再备份个人 world.sdf，并把 table_example、room_example 的模型引用切换到学生包。它不会靠复制参考包填满学生目录，也不会替你完成鼠标建模。应用仍在显示旧服务器里的实例；此时点击**“重载项目”**，等新场景就绪后再观察。

从“编辑练习文件”查看个人 world.sdf，应看到 `model://student_table` 与 `model://student_room`。实例名仍为 table_example、room_example；学生包名与运行实例名不是一回事。参考包采用 SDF 1.6，编辑器实际输出的格式以文件为准；不要仅改根 version 数字就声称完成格式转换。

重载后要修改学生桌子，可右键 table_example 选择 **Edit Model**。修改后再次 Save As，显式核对 Location 仍是自己的 student_table 目录；覆盖前先保留旧包。随后退出编辑器、重新执行 use-exports 并重载，更新导出原点的摆放补偿。建筑模型没有恢复原二维画墙历史的对应流程。

需要对照时运行 `python3 lab.py use-reference`，再“重载项目”。这只切回离线样例的入口，学生导出包仍应保留。要重新使用成果，再执行 use-exports 并重载。单击“重开 Gazebo 窗口”或“重置模型状态”都不重新解析模型文件。

完整信息流是：**鼠标和 Inspector → 编辑器内部设计 → model.config 与模型 SDF → world 的 include URI/位姿 → 重建服务器 → visual/collision 运行实体 → 几何与接触验证。**模型包保存、世界引用与正在运行的状态是三个独立环节。

## 验收与排错

模型稳定后点击“验收实验”，验收期间停止编辑和拖拽。自动部分读取当前 world 的实际引用与导出几何，再检查真实场景的静态模型、桌面碰撞与支撑、房间门洞空隙等。本课支持 SDF 1.4–1.7 的静态 box 组合、默认父坐标系、零 roll/pitch 和水平 yaw，不扩展到任意网格、嵌套模型或关节。它不能证明你使用过 Align、按过快捷键或亲自画过墙；请在自己的笔记中保留两种编辑器的创建过程、导出路径、单变量预测和重载后的截图。“查看参考答案”只读展示解法，不覆盖个人副本。

| 现象 | 先检查什么 |
| --- | --- |
| 模型保存了，重载又变回样例 | use-exports 是否成功；当前 world 的 URI 是否仍指向 reference_* |
| use-exports 报缺包或入口 | Location 是否误填成上一级；model.config 声明的 SDF 是否存在；两包是否都已保存 |
| 看起来有桌面，物体却穿过 | Collision 页是否与 Visual 同尺寸同位姿；是否只改了颜色或外观 |
| 桌子埋地或悬空 | 导出是否重选了模型原点；接入时是否按碰撞最低点摆放；是否误把局部 pose 当 world pose |
| 门洞被墙堵住 | Door 是否真的附着到墙；三维预览与保存后的碰撞几何是否有开口 |
| 想撤销建筑改动或重开原草图 | 建筑编辑器没有该历史恢复能力；用保存的版本对照并新建修改 |
| 修改文件后画面不变 | 先保存并退出编辑器，再“重载项目”；仅重开客户端不会重建模型 |

不了解模型目录与入口？[查看官方模型包结构](https://classic.gazebosim.org/tutorials?tut=model_structure&cat=build_robot)。需要核对快捷键？[查看 Classic 快捷键表](https://classic.gazebosim.org/hotkeys)。
