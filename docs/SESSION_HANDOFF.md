# GazeboTeach 会话交接

> 2026-10-30 最新停止指令：完成L13后整理、保存、清理并结束本次工作，不开始L14，不自动续跑、不设置自动化。L14–L37与G02等待用户明天明确发起继续。最终验收与发布状态以VALIDATION_LOG.md及核对后的main记录为准。

> 2026-10-30 当前交付范围：主课01–13共13门，以及L09选做A/B（091单级、092串联双级倒立摆控制），合计15个项目。L13最终物理、渲染与桌面证据引用VALIDATION_LOG.md；本次提交、推送及远端哈希核对完成后，L13目标完成并结束本次工作。下一目标L14仅在用户明天明确发起后开始。主课01–37保留编号，091/092为附加选做；正式交付状态以README、验证记录和main历史为准。

091与092均默认reference_demo，各自learner.py仍保留三个TODO；参考演示不计作业完成。学生模式须实际执行当前个人源码，通过函数行为、指纹、真实模型及三个实际回合验收：091每回合10仿真秒，092每回合12仿真秒。两课都仅施加小车水平力，100Hz反馈/1kHz保持；092把原生q2相对角/角速度转换为第二杆绝对状态。完整起摆不属于当前局部保护桥的可直接运行范围。双杆共用桥变更后，091完整4场景回归109.003s与43项单元检查通过，无归属进程残留。

L10为固定均匀转子与Classic 11原生JointController的六阶段对照，正常运动由1 kHz原生力矩/PID推进，只在阶段边界建立真实零位、零速、累计角参考和清零控制器历史。正式6场景146.379s、43项单元检查、桌面闭环43.510s及公开reset_models专门回归21.731s均通过，源模板未变且归属进程清理干净。只编辑个人parameters.json，保存后须重载；完整流程与旧多圈初始化故障的证据边界已记入验证日志。不要将理想关节转矩模型当作含电气动态的实体直流驱动器。

L11差速小车已完成7场景/3个真实会话正式验证107.253s，取得10个2 s测量窗、共20,000个真实1 ms测量点；升降速、停稳和公开复位另行检查，不混作测量样本。43项单元检查.136s及真实Qt/Gazebo/两PTY桌面闭环29.140s通过，源模板不变、归属进程清理干净。仅速度/轮面配置按教材修改并重载，stock插件理想轮速度通过接触推动车身；原生Pose旧转向符号、独立编码积分/世界真值、复位清命令和QuickStep速度读数边界均有记录。L11发布与远端哈希已核对完成，旧14–16草稿继续保持未部署状态。

L12消息滑门站当前交付五字段信封协议：逻辑nonce与每次send的新attempt_nonce分开关联，query在网络初始化前限制为只读快照/安全未知探针，entity_info走独立入口。v2正式8组/2个并发私有Master验证258.719s通过，14个实际应用事务共16,324个真实1 ms post样本（含2次prepare_closed共400点）；另1次received后复位取消、未应用，单列不混计。43项单元检查.131s、真实桌面13项检查26.829s通过，源模板不变、归属进程清理干净。旧四字段私有、审查/纯回归、首次正式测试归属误判与当前完整v2正式证据分开保存；源码以v2指纹为准。完成本次发布及哈希核对后才进入L13，不提前部署旧草稿。

L13场景展厅把材质/灯光、真实高度场和Actor分为四站：个人world.sdf是实际输入，显式修改高度、周期或日光后须重载；65×65灰度图在visual/collision均采用sampling=1，单层地表diffuse/normal纹理引用系统离线资源。六个落球的真实1 kHz尾窗、原生高度/射线以及50 Hz动画观察分别保留，人物根路径与脚相对髋部变化分开验证；图形效果须由真实GUI证明，不能用headless成功替代。

L13使用课程可选after_gui初始化：有界等待真实GUI与相机姿态后只发布一次总览请求并读回确认；已有活GUI直接返回，不抢学生视角。钩子失败只回收本次新建客户端，完整start失败仍统一清理；无GUI模式不运行该钩子。框架独立修复、各轮失败与修复边界、最终物理/渲染/桌面检查及归属进程清理证据见VALIDATION_LOG.md对应条目。

下一次继续入口是L14接触传感器：Windows未跟踪的14–16旧草稿尚未部署，不算已交付；用户明确发起后先审查L14草稿、合同与当前设备，再按原编号逐课实施。L13保存与远端核对后保持停止，不创建下一目标或定时自动续跑。上列各课记录中的当时“下一目标”表述不覆盖本页最新停止指令；下方10月29日快照全部保留为历史。

## 以下为2026-10-29历史停止快照

下文保留当时的路径与调研记录；其中交付清单、停止指令和“下次从L05继续”等表述均为历史状态，不能覆盖上面的当前进度与继续授权。

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
