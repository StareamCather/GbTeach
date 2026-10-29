# GazeboTeach 提前停止交接

2026-10-29，用户最后指令为“立即保存现有成果，然后终止，要提前终止了”。此指令覆盖之前“完成L05后停止”的要求。所有编写代理已中断；不自动续跑、不设自动化。等待用户在新对话明确发起继续。

## 已交付与当前边界

已完成G00中转、G01桌面框架和L01–L04。最后应用提交为 `afdd761e4ba8d6c496b8834d9ae2c1ecc0821088`，已在ikaros提交，经Windows推送GitHub并读回main哈希核对。后续交接文档提交仅保存状态，不新增课程。

- L01：落地方块，真实落地、尺寸变式及错误惯性拒绝。
- L02：外观与碰撞房间，实体/幽灵平台及修复实验。
- L03：坐标与位姿，SE(3)、真实model/link姿态、独立模型不随父参考移动。
- L04：离线模型包，2–4实例复用、公共尺寸变式、入口错误/修复、只读查询有限重试。

各课的真实行为验收、桌面闭环和截图记录见 VALIDATION_LOG.md。L04最后四场景与7项独立超时注入回归均通过；正式GUI检查约33.6秒通过。注入回归不是物理验收。测试使用独立临时练习，不伪造用户学习进度。

停止前只读检查：ikaros正式课包仅01–04，仓库已交付内容干净，没有带GazeboTeach实验令牌的残留进程。没有开始L06。L05没有可运行课包，也未部署、验收或提交为正式课程。

## 下次从L05继续

先读本文件、EXECUTION_PLAN.md、VALIDATION_LOG.md、framework-flow.md与relay-flow.md，核对两端Git状态。用户明确允许继续后才建立L05目标。每次仅当前目标，验收/提交/推送/远端哈希核对完成才进入下一课。

L05仅完成以下方案与接口调研，尚未落实为代码：

- 项目候选：静态组合桌与带门洞房间；离线参考包reference_table/reference_room；学生另存student_table/student_room，不预填或覆盖成果。
- 候选布局：table_example在(-2.3,0,0)，room_example在(2.3,0,0)。桌顶1.2×0.8×0.1m、中心z0.8、四腿；房间3×2.5×2m、厚0.12m，前墙门洞宽0.8高1.4m。待实现和真机验证，不作现成保证。
- 计划lab工具先校验学生导出包，备份world再切换include URI；切回参考也只调整入口。验收应读取实际保存模型、几何/碰撞、门洞空隙及真实支撑行为，明确不能证明学生鼠标技能。
- Classic实际入口：Edit→Model Editor(Ctrl+M)、Building Editor(Ctrl+B)，编辑器内Save/Save As与Exit。建筑编辑器无Undo，退出后不能恢复原2D编辑状态；其Door/Window是墙洞，不是可动门。模型编辑器有Undo/Redo，Redo为Shift+Ctrl+Z。
- 真机开发包只有model/ModelEditor.hh与ModelEditorEvents.hh，没有ModelCreator/ModelEditorPalette或building私有头。Qt5Test和libXtst已安装。拟通过仅测试加载的GUIPlugin、公开QAction/QObject/QtTest与实际保存对话框验证编辑器；尚未编写驱动。不要把仅启动Gazebo替代GUI交互验收。
- SaveEntityDialog的Location是最终模型文件夹。实际编辑器保存可能写gui.ini；测试前核对隔离配置，避免改用户全局模型路径。GAZEBO_LOG_PATH支持日志目录，但本次尚未确认其是否足以隔离GUI配置。

官方调研入口：[Model Editor](https://classic.gazebosim.org/tutorials?tut=model_editor&cat=build_robot)、[Building Editor](https://classic.gazebosim.org/tutorials?tut=building_editor&cat=build_world)、[快捷键](https://classic.gazebosim.org/hotkeys)。

## 位置与维护

- ikaros：`/home/ikaros/GazeboTeach`，在图形桌面运行 `./launch.sh`。
- Windows源仓库：`C:\Users\z\Documents\Codex\2026-10-29\new-chat\work\GazeboTeach`。
- 中转工具：该仓库上级的`manage_gazeboteach.py`，用deploy明确文件、run真机命令、publish真机提交并由Windows推送/核对。
- GitHub：`git@github.com:StareamCather/GazeboTeach.git`，main，保留MIT与原历史，禁止强推。
- SSH别名ikaros，经既有Windows SSH配置连接；私钥不进入仓库或设备。技能入口`C:/Users/z/.codex/skills/ssh-mydevices/SKILL.md`。
- 设备Ubuntu20.04/Python3.8/Qt5.12/PyQt5/Gazebo11.15.1/ROS Noetic。实际图形变量从活跃GNOME发现，不能写死DISPLAY。
- 维护接口：`python3 -m gazeboteach --diagnose / --list / --smoke ID --gui --terminals / --gui-check`。自动验收用新副本；仅清理本应用拥有的进程组。

未交付的14–16旧传感器草稿仅存在Windows未跟踪目录，本次未部署或验证。已连同中转工具打包到`checkpoints/unreleased-2026-10-29.zip`，保持原文件路径；不得当作正式课程直接上线。它们与L05的调研状态分开。

原完整范围仍是35小项目+2综合项目及最终回归，尚未完成；后续由用户重新发起。
