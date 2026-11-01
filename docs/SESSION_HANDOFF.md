# GazeboTeach 会话交接

2026-11-01 V2交付快照：主课01–37与L09选做091/092共39项目均已逐课提交，G02已按当前课程完整输入摘要汇总39个新副本真实GUI验收，未遗留同源码失败。两端178项单元检查通过（Linux无跳过、Windows4项平台跳过）；17项桌面/存储/会话隔离与12项先修阅读检查通过。正常入口为ikaros的`~/GazeboTeach/launch.sh`。本版仅验证Ubuntu20.04 / Classic11.15.1 / Noetic，其他版本由环境窗口逐课说明，Docker保留接口并提供手动指南。

固定源码中的这份快照记录源码验收边界；随后按同一不可变提交构建的安装验证、GitHub标签、发行资产及认证下载摘要由[V2发行页](https://github.com/StareamCather/GazeboTeach/releases/tag/v2.0.0)中的`release-manifest.json`、`V2_VERIFICATION.md/json`与`SHA256SUMS`绑定。发行后查看该报告确认最终结果，不通过反写自身源码制造循环证明。此前各停止点与课程进度保留为历史；当前不再有待开发课程。


2026-11-01 当前L37 机械臂推块综合任务：真实双关节、力反馈与故障恢复：真实独立行为验收55.281s、桌面13项26.836s通过。本课独立提交、Windows中转推送和远端核对完成后进入G02最终回归与V2发布；当前37主课+091/092共39项目，尚余G02与V2发布。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L37验收期间发现的Gazebo Transport框架缺陷已修复并真实回归，待独立提交推送核对后继续L37。持久SDK控制10轮/100请求、精确步进与真实模型复位、GUI诊断及两端127项测试通过；L37尚未发布，已发布仍38项目。


2026-11-01 当前L36 仓库综合任务：激光、物理滑门与有限状态机：真实独立行为验收74.613s、桌面13项43.159s通过。本课独立提交、Windows中转推送和远端核对完成后进入L37；当前36主课+091/092共38项目，尚余L37–L37共1门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L35 故障诊所：从真实证据定位、修复并复验：真实独立行为验收115.403s、桌面13项78.38s通过。本课独立提交、Windows中转推送和远端核对完成后进入L36；当前35主课+091/092共37项目，尚余L36–L37共2门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L34 性能与批量实验：真实落体、计时边界与可复现输入：真实独立行为验收86.439s、桌面13项76.33s通过。本课独立提交、Windows中转推送和远端核对完成后进入L35；当前34主课+091/092共36项目，尚余L35–L37共3门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L33 多机器人：命名空间、独立控制与共同坐标系：真实独立行为验收50.746s、桌面13项30.906s通过。本课独立提交、Windows中转推送和远端核对完成后进入L34；当前33主课+091/092共35项目，尚余L34–L37共4门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L32 原生状态日志：录制、读取与隔离回放：真实独立行为验收111.541s、桌面13项48.009s通过。本课独立提交、Windows中转推送和远端核对完成后进入L33；当前32主课+091/092共34项目，尚余L33–L37共5门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L31 SystemPlugin：进程生命周期、真实重力与步采样：真实独立行为验收51.782s、桌面13项40.722s通过。本课独立提交、Windows中转推送和远端核对完成后进入L32；当前31主课+091/092共33项目，尚余L32–L37共6门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L30 GUIPlugin：在Gazebo中编写真实Qt控制面板：真实独立行为验收42.976s、桌面13项27.356s通过。本课独立提交、Windows中转推送和远端核对完成后进入L31；当前30主课+091/092共32项目，尚余L31–L37共7门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L29 VisualPlugin：外观大小为何不等于碰撞大小：真实独立行为验收49.098s、桌面13项31.409s通过。本课独立提交、Windows中转推送和远端核对完成后进入L30；当前29主课+091/092共31项目，尚余L30–L37共8门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L28 SensorPlugin：从原生测距到滤波与距离判断：真实独立行为验收215.076s、桌面13项52.434s通过。本课独立提交、Windows中转推送和远端核对完成后进入L29；当前28主课+091/092共30项目，尚余L29–L37共9门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-11-01 当前L27 WorldPlugin：按节拍投料并管理世界实体：真实独立行为验收126.227s、桌面13项66.586s通过。本课独立提交、Windows中转推送和远端核对完成后进入L28；当前27主课+091/092共29项目，尚余L28–L37共10门主课及G02/V2。V1保持原26项目发行快照；最新授权持续至全部39项目V2。以下旧进度保留为历史。


2026-10-31 当前L26 ModelPlugin：三组独立原生位姿物理验收68.277s、真实桌面13项35.770s通过。最大角增量残差0.001761rad，实际100步与模型复位/生命周期均留证。完成独立提交、Windows中转推送和远端核对后进入L27 WorldPlugin；当前26主课+091/092共28项目，尚余L27–L37共11门主课及G02/V2。V1已发布且保持原26项目快照；最新授权持续至全部39项目V2，以下旧进度保留为历史。


2026-10-31 当前L25 ros_control：3个全新私有场景/6组正式检查96.916s、真实桌面13项36.707s通过。真实控制器/物理响应、私有ELF映射、非零q/qd冷暂停恢复、生命周期和100步均留证。完成本课独立提交、Windows推送和远端核对后进入L26 ModelPlugin；当前25主课+091/092共27项目，尚余L26–L37共12门主课及G02/V2。V1已发布并核对8附件，tag源为384c74bf5d96e34f31d90cd823d42f2ffea1829e。最新授权持续至V2，以下旧进度保留为历史。


2026-10-31 V1恢复进展：ikaros SSH已恢复；候选的104项Linux检查及修复后的106项检查全部通过，真实环境报告显示26项目就绪。独立含空格目录安装/重复安装/完整文件与权限、安装入口诊断和课程列表通过；源码与安装目录L24桌面各13项、L01真实落块、L12运行与独立RViz重开已通过。新SDK空缓存检查和最终资产安装复核继续执行，完成V1发布核对后立即进入L25；以接下来的验证记录及远端核对为最终依据。以下旧阻塞/候选文字保留为历史。

## 2026-10-31 恢复执行至 V2（当前有效交接范围）

用户已明确要求“继续完成所有课程，然后完成v2”。此前V1发布后终止、L25等待恢复的停止点已解除，后续按原编号逐课推进到L37、G02及V2发布核对完成。

接续基线：正式main/ikaros为L24的048e4cedbb5edc78720a4c88e73252ab90675f1e；候选代码1f825db6705de0f168e0979f7be9e94c7504109f及交接88c16bd6b31a08f40f67862ea747f8a003f5b136保存在wip/v1-environment-release。维护端已重新确认以太网1Gbps、ikaros-wh SSH可达、Python3.8.10和远端干净L24基线。恢复连线只证明可继续验收，不代表候选框架已部署或安装包已通过Linux检查。

当前先做候选框架的真实Linux安装、启动环境弹窗、GUI/终端、L01/L24及首次SDK构建回归，保留后文候选来源与验收恢复清单；完成框架与V1收尾提交/推送/精确核对后进入L25。每次只保留一个活动目标，每课完成真实行为和桌面验收、算法流程记录、独立提交与GitHub远端核对后，再进入下一课。

剩余目标依次为：L25 ros_control；L26–L31六类Classic插件；L32记录回放；L33多机器人；L34性能与批量实验；L35故障诊所；L36仓库小车综合实验；L37机械臂综合实验；G02最终回归；V2完整安装发布。

V2交付应覆盖37门主课加091/092，共39项目。核对课程清单/应用列表/安装manifest一致；保留全部课程实际验收，区分自动、人工图形、跳过与未验证；从最终通过提交构建完整安装资源，在全新独立安装目录完成启动、构建和实验检查；发布V2安装运行指南、验证记录及Release，并核对正式main、标签、source_commit及下载资产SHA256。清理测试进程、保存交接后结束本轮。

兼容范围仍为已验证Classic11/ROS1Noetic课包；环境识别与Docker配置预留不等于已有ROS2/现代Gazebo课程或自动Docker执行。无需为V2自行扩大这些范围。GitHub私钥继续只保留在Windows；保持历史、用户练习和无关进程，不改宿主系统、GPU驱动、默认路由或SSH安全策略。

以下先前V1保存点、旧断线状态和停止要求保留作历史取证；不再据此暂停本轮V2工作。


V1 候选保存点：代码提交 1f825db6705de0f168e0979f7be9e94c7504109f 已推送并通过 ls-remote 核对到 wip/v1-environment-release；正式 GitHub main 仍为 048e4cedbb5edc78720a4c88e73252ab90675f1e。无 v1.0.0 标签/Release，未部署到 ikaros。本节后的文档保存提交仅记录交接，不代表真实安装检查已完成。

候选资产在 Windows 仓库 build/releases/v1-candidate：436 个载荷文件、26 项目，自解包安装器约21.1MB；source_commit=1f825db6705de0f168e0979f7be9e94c7504109f。已执行真实候选内嵌 Python --check 和三项外部 SHA256 校验，但未在 Linux 执行安装或包装器。安装器 SHA256=67072677729abe795a3db56a5e0557842d2e877bdccb4743844c39fbe86c48a3，tar.gz=dc9445381bf798a9bc184ad2223b9282ea1d5bbf0bd5d8a090d2a79a7a725ef6。它们是未发布候选，后续正式发布必须从最终验收提交重新构建。

恢复顺序（仅 V1，不进入 L25）：

1. 检查本机以太网与 ikaros SSH 恢复，严格核对既有主机身份；保留当前远端 main 和用户数据。不改默认路由、SSH安全策略或驱动。
2. 通过现有 Windows 中转将候选 Git 历史/文件传入 ikaros，在独立候选分支验证，不能使用只传 main 的旧 relay 子命令遗漏此分支。
3. ikaros 执行 python3 -m unittest discover -s tests -q，必须包含新 Linux 安装/包装器/修改保护/卸载测试；Windows 的3项跳过不得直接算Linux通过。
4. 构建候选安装器并安装到项目 .deploy 内的独立 prefix（含空格路径也要测），通过 PREFIX/bin/gazeboteach 运行 --diagnose、--list（26项目）、--gui-check（真实启动弹窗+主窗口）。运行 tests/environment_ui_probe.py 并保留截图/JSON；其现代版本切换部分为合成事实，要与实际设备探测区分。
5. 独立安装目录进行 L01 GUI/双终端真实运行、L24 的 tests/ui_probe.py 和 RViz窗口回归，以及 L12 全新工作区首次SDK编译/真实验收；验证退出只清本课进程。记录实际检查范围，不凭旧课程记录声称安装包全部重新测过。
6. 修复具体失败并说明运行算法；更新指南/README候选标记和验证记录。按既有 ikaros提交→Windows获取→GitHub main快进推送→远端hash核对流程发布。由最终通过提交重建正式资产，上传 v1.0.0 Release（安装器、tar、manifest、SHA256SUMS、指南、最终验证记录），核对标签、来源提交和下载后摘要。
7. 发布核对、清理测试进程并保存后暂停结束；L25目标保持paused，后续13门主课及G02等待用户另行发起。

本地证据：.deploy/v1-windows-tests.log / .json（104项，101通过/3跳过）、v1-environment-synthetic.png（Windows离屏，已查看中文可读）、v1-global-baseline.json（旧全局文档摘要）、v1-candidate-payload-check.json、v1-candidate-push.json。现有旧验证记录原153506字节前缀与Handoff历史尾均保留。Docker手册已写，但本次未启动本机DockerDesktop daemon、未下载镜像，容器端到端仍未验证。


2026-10-31 最新范围：L24 已完成独立提交、推送与远端核对（048e4cedbb5edc78720a4c88e73252ab90675f1e）。当前只准备 V1 环境检测弹窗、Docker 预留接口、安装包与指南；V1 发布核对后暂停，不开始 L25。此前“L24 后进入 L25”的文字保留为历史，已被本次停止点覆盖。当前本机以太网 Disconnected / 0 bps，ikaros SSH 握手连续超时，真实安装验收尚无法执行；V1 改动保存到独立候选分支 wip/v1-environment-release，不能视为正式部署或 Release 已发布。


> 2026-10-31 当前L24 RViz/TF/时钟已通过3个全新私有场景/6组正式验收59.643s，真实桌面13项24.076s、独立demo前缀RViz窗口/重开验收13.488071422s通过。真实关节→JointState→RSP→TF、暂停/物理单步、同listener倒时恢复与人工渲染检查分层留证。完成L24独立提交、推送与远端核对后进入L25 ros_control；主课01–24与091/092共26项目，尚余L25–L37共13门主课及G02最终回归。以下旧进度保留为历史。

L24流程：私有Master先设/use_sim_time→Xacro/URDF/SDF及SHA→现有Gazebo中一次性spawn tf_arm→系统Gazebo JointState插件直接读取真实Position/GetVelocity→RSP按同一URDF计算动态TF并latch固定边→tf2/RViz。JointState header取ROS时钟缓存，TF复制原stamp；世界服务读钟、接收monotonic与查询时刻另存。namespace不自动改变frame，Noetic使用tf_prefix；频率20/40Hz与prefix空/demo分别做单变量重载。原生joint名单、Gazebo实体和模型尺寸不随TF前缀变化。

验收先保存入口状态，再受控继续取流；暂停中做真实1/100步、late-static订阅，随后由同一listener/Buffer跨真实reset_simulation，记录回退与原clear调用，固定边继续可用。结束清本课任务，先在原q/pose零twist下真实推进150tick刷新TF，再暂停恢复原pose/twist及pause；速度setter按实际COM补偿v_COM=v_origin+ω×R·COM。世界时间、队列与TF历史不回填。新paused客户端ready只做authority/参数/模型检查并注明动态观察延后；手动clock-reset缺旧epoch时先实推150tick预热，再reset+150tick，以中间原始clock事件证明回退。

正式、UI与demo前缀RViz证据独立保留；渲染由root查看原图，自动窗口测试不冒充图像理解。失败操作者探针/EOL修复与冷paused CLI补验单列。20课源/两tests冻结，下一课25；具体原始报告和SHA见VALIDATION_LOG。

> 2026-10-31 当前L23 URDF/Xacro已通过3个真实全新场景/8组验收55.596s，实际桌面13项20.444s通过。Xacro→URDF→本机SDF→实际spawn、几何/合并物性、三静态FK与三动态响应均有原始证据；非零角速度/偏置COM的恢复已按真实API参考点差异修复，原阈值保持。完成L23独立提交、推送与远端核对后进入L24 RViz/TF/时钟；主课01–23与091/092共25项目，尚余L24–L37共14门主课与G02最终回归。以下旧进度保留为历史。

L23流程为私有Master先设/use_sim_time→源包overlay与Xacro输入→真实xacro生成URDF→gz sdf -p转换并保留原值/SHA→roslaunch写/lesson23/robot_description并一次性spawn_model(-urdf)→只读说明节点/实际link物性就绪→保存暂停快照→三次边界姿态初始化与真实FK核对→±.03N·m/200ms及upper+.12N·m/350ms有限肘力矩→逐次读取真实双关节响应→清唯一basename任务、恢复q与全link pose/twist及原pause。静态SetModelConfiguration用于初态；动态期间由ODE推进，读取世界/主机时间包夹，不强制轨迹，也不以单杆τ/I替代多体耦合。

L1 .6→.8保持质量，联动几何、COM、惯性与肘origin；preserve_tool仅切Gazebo保留扩展，不删除URDF工具。URDF始终5link/4joint，实际物理link从4变3，合并后的arm2质量.52kg、COM x=.207692m、Izz=.0075909kg·m²；总模型质量仍2.52kg。理论URDF→SDK六有效数字转换按最终量子预算，运行API→转换SDF另严格比较，所有三层原值保持。API reported_type/damping不作为物理类型/阻尼证据，固定joint的NaN不当合法角度；只读两moving joints。

本版ODE GetLinkState读取link原点世界速度，而SetLinkState调用SetLinearVel设置CoM速度。恢复读取实际GetLinkProperties.COM与快照四元数/ω，先算r_world=R_world_link·c_link，再发送v_COM=v_origin+ω_world×r_world；只转换setter请求，不改原始快照，仍按原点速度读回核对。恢复阈值、独立验收源码与其余算法保持；旧任务队列和已流逝时间不重建。

正式v1失败整包保留两端.deploy/lesson23-formal-v1，原点/COM诊断保留lesson23-origin-com-diagnosis.json；v2正式与UI证据为lesson23-formal.json/-artifacts及lesson23-ui*。临时Storage已删除，旧/tmp身份路径不宣称仍存在，实际URDF/SDF/准备记录、服务原值、原生碰撞查询与日志已归档。当前17课源与v2冻结SHA精确一致，下一课24，仍逐课验收与核对后前进。

> 2026-10-31 当前L22 ROS启动已通过3个正常全新场景、9组核心场景、3组静态解析、2组launch故障及2组取消检查，正式102.024s；真实桌面13项20.654s通过。实际roslaunch图、参数/缓存、有限施力/到期匀速和状态恢复均有原始证据，4个会话token无残留。完成L22独立提交、推送与远端核对后进入L23 URDF/Xacro；主课01–22与091/092共24项目，尚余L23–L37共15门主课与G02最终回归。以下旧进度保留为历史。

L22流程为私有Master→个人源包overlay/输入指纹与/use_sim_time=true→Gazebo/API初始化→world-ready→真实roslaunch解析arg/include/group/私有param/remap并启动两个节点→after_startup核实际图/PID/缓存/新反馈→Trigger提交未来0.1模拟秒开始、持续0.3模拟秒的2或4N world+X有限ApplyBodyWrench→ODE真实加速→到期匀速→清本课body队列并恢复进入pose/twist/pause。服务调度包含开始/结束端点，实际速度可多一tick冲量；试块1kg、零g、零速度衰减，取消是清施力任务而非刹车。原队列、driver历史与已经流逝的模拟时间不倒回。

Master负责名称发现与参数，不中转全部主题负载。ModelStates原生无header，source_stamp_ns=null，observer的50墙钟Hz是输出上限；其回调计数和接收ROS/monotonic时间不伪装成native采样。GetModelState header是API ros::Time::now读回，真实查询保留前后GetWorldProperties与主机时间包夹；coast反馈只按同主机接收邻近交叉检查，不称采样同步。scheduled、服务成功及空clear响应均不是物理完成证据。namespace只重命名ROS图，不复制Gazebo实体；arg须显式写入私有param，remap不改类型/实体名，driver只在启动时缓存参数，保存未重载与运行参数不一致都在物理控制前拒绝。

公共ros_package_paths与after_startup支持已独立发布核对为aa1b335628836395eae531fe4f1686ef6345817f，74项Linux检查、既有L21图形回归和L22最小启动属于该框架修复证据，保留在原历史前缀，不混入本课正式计数。L22最小v1曾因xmlrpc模块名被函数遮蔽而就绪失败；修为独立模块别名，同时将/use_sim_time初始化前置到私有Master、再启动ROS API/节点。失败与修复最小探针都标startup_only且未发脉冲；它们不代表完整课程通过。

两端.deploy/lesson22-formal.json、formal-artifacts、日志及lesson22-ui*保留正式证据。临时Storage已清理；原报告中的/tmp/workspace身份路径仅描述当时实际节点，不宣称仍存在。当前17课源与正式SHA一致；下一目标23，继续逐课验收、提交、中转推送和核对，不把未验证草稿计入交付。

> 2026-10-31 当前L21同步/频率/lockstep已通过3个全新私有场景、8组核心验收与2组真实取消检查，共124.404s；真实桌面13项34.906s通过。相机成功原生Update、接收标签覆盖密度及ROS交付分层验收，暂停/真正单步多步与隔离服务器重启均有原始证据。完成L21独立提交、推送与远端核对后进入L22 ROS启动；交付主课01–21及091/092共23项目，尚余L22–L37共16门主课与G02最终回归。下列旧进度保留为历史。

L21运行流程为实际SDF/prepare指纹检查→完整--lockstep主world→SDK读取原生ImageStamped.time、IMU.stamp、WorldStats与无原生stamp的Perf/metadata→像素原字节外存→Python实际ROS事件发布/订阅→连续2模拟秒窗口→暂停墙钟保持→真实1步/0.1模拟秒多步→主world保持暂停并串行启动两个私有Master/IGN partition的纯原生child→恢复原pause并回收本课PID。child使用同一临时benchmark.sdf，仅共同把RTU改为0，argv模式差仅--lockstep；不将主限速场景与无上限child之差全归因于lockstep。

相机ConnectUpdated只在UpdateImpl成功后计数，与LastMeasurementTime成对快照；窗口用Δcounter/ΔmeasurementTime，不是N−1，也不是硬件曝光或Image已送达。钩子可能晚接，只证明窗口增量。consumer原trace顺序、原标签/receipt不变；升序unique标签只建立密度/gap/离线nearest索引，原始接收数、unique/重复标签和回退count/max并列，不能据此定位网络层。主观测20/100Hz门限保持；ON子实验20或10Hz目标由真实成功Update计数率±2%/≥1模拟秒且same/regressed增量0验证。WorldStats.real_time扣暂停，与主机CLOCK_MONOTONIC不同；ROS服务读取world时间不是另订阅/clock。

正式原始报告与flat asset-reference-map保存在两端.deploy/lesson21-formal*及ui*；自动临时Storage已删除，报告的旧/tmp路径仅为原观察身份，现存原图经映射读取扁平assets/SHA.rgb，不宣称旧路径仍存活。探索v1/v2为失败证据，v3探索PASS另列，不混正式计数；有意保留的lesson21-probe-storage-*可在ikaros复查。下一课为22，本次仍保持每课独立验收、提交、推送、远端核对后才前进，不自动把未验收草稿计交付。

> 2026-10-31 当前进度为L20其它传感器展台：3个真实私有场景/8组验收141.204s、180个通道窗口/2160条验收事件、真实桌面13项36.057s通过。GPS/Mag/Alt/Sonar/Logical五个原生工厂均实测，不以头文件存在计上线；磁场与初始高度单变量、9阶段真实运动/几何变化、错误输入拒绝与修复再验均正确。输入及9个课文件不变，4模型位姿/速度与暂停恢复，归属进程清理干净。完成提交推送与远端哈希核对后进入L21同步与lockstep；主课01–20与091/092合计22项目，尚余17门主课及G02。以下保留为历史，不覆盖继续授权。

L20薄桥逐字段转发五类原生protobuf为JSON，不生成测量值或假设NavSatFix状态。GPS原点WGS84(0°,0°,100m椭球高)、显式heading180°以对齐当前Classic局部XY/ENU；磁场独立，Mag输出RᵀB。Altimeter保留加载时worldZ参考。九阶段每次仅设置一次初始pose/twist，再由ODE推进；真实vz=.25m/s同时用GPS、Alt与GetModelState轨迹验证。四原生消息用自带time，Logical原生无time，source_stamp_ns=null、独立bridge_sequence及receipt_world_time_ns只作接收时基，允许不同序号共享同一接收时刻。

Sonar是局部−Z接触体积，1.5m地板实际1.505327m，抬高障碍约.901981m，传感器z4m时达到max3m；max时仍保留旧contact，不能以has_contact判断当前回波。contact为sensor局部未经penetration修正的原始碰撞位置，不是保证真实表面点。Logical由碰撞AABB与视锥判断，实际遮挡件在目标前方仍同时上榜；地板大AABB也可入名单，不强求model列表精确两项。正式原始数据/源指纹/实际状态与恢复记录见两端.deploy/lesson20-formal*和ui*及VALIDATION_LOG.md。

> 2026-10-31 当前进度为L19力／力矩：3个真实私有场景/8组验收54.654s、16路时间窗/501条验收wrench、桌面13项18.371s通过。质量与力臂单变量、方向反号、Rx(−90°)换轴、native/stock稳态对照及反算质量/力臂均正确；输入未变，原暂停恢复，固定机构未被程序移动，归属进程清理干净。完成L19提交、推送及远端哈希核对后进入L20其他传感器展台；主课01–19与091/092合计21项目。以下进度为历史，不覆盖继续授权。

L19三路ForceTorqueSensor由唯一名桥仅转发原生WrenchStamped；系统GazeboRosFT则是ModelPlugin直接读body2，两种实现分开。固定关节原点与load_link原点一致、CoM偏置+.3m，child_to_parent下Fz−9.81N/Ty+2.943Nm；反向全反号，sensor绕X+90°后Fy−9.81/Tz−2.943。只改mass或arm联动惯性与可视/碰撞参数并重载。guard时刻读实际GetWorldProperties.sim_time，各路独立唯一窗口不强求跨实现同stamp，姿态与实际物性佐证静力平衡。Native会自动启用feedback，删字段只作为课程预检故障，不谎称必无运行数据；详见VALIDATION_LOG.md。

> 2026-10-31 当前进度为L18深度／点云：3个真实图形场景/8组验收86.299s、12窗/144个验收三路组合、Qt深度查看器30帧、桌面13项23.250s均通过。按原始32FC1/PointCloud2字节独立解码，验证NaN、整幅Z与有效掩膜、字段/步长、真实墙移1m和主点差异；输入未变，墙体/暂停恢复误差0，归属进程清理干净。完成L18提交、推送与远端哈希核对后进入L19力矩传感器；主课01–18与091/092合计20项目。以下进度保留为历史，不覆盖继续授权。

L18使用系统OpenNI Kinect观察插件与真实DepthCamera，10Hz控制组织点云开销，输出depths而非nativepoints。软件范围严格.4<Z<4或6，原生渲染clip.1..8分开；32FC1米、NaN不是自由空间。PointCloud2本机point_step32，x/y/z/rgb偏移0/4/8/16；helper用(W−1)/2，CameraInfo用(W+1)/2，均保留并分别验证。实际红目标cloud字段原字节[255,0,0,0]按常规packedRGB解为蓝，已记录当前插件颜色布局问题，不改原始数据或宣称缓存RGB同曝光。伪彩PNG是显示，原始浮点和点云另存，详见VALIDATION_LOG.md。

> 2026-10-31 当前进度为L17 RGB相机：修正Classic/SDFormat9的HFOV六位有效数字写回所引起的严格内参预期差异后，3个真实图形场景/8组验收81.328s、16窗/192对验收图像与内参、真实Qt查看器53帧、桌面13项22.337s均通过。图像/K/P/像素阈值未改；原始RGB字节、SHA与PNG保存，输入未变，目标状态/暂停恢复误差0，归属进程清理干净。完成本课提交、推送与远端哈希核对后进入L18深度相机；主课01–17与091/092合计19项目。以下进度为历史，不覆盖继续授权。

L17系统RGB相机真渲染，Image与CameraInfo按完全相同headerstamp配对；RGB8/step/字节长度、K/P/R/D与光学帧语义分开核对。四阶段基线→目标远移→横移→恢复，每窗≥12对/≥.5仿真秒；实际像素色块与0.04m厚板八角点投影比较。名义60°经当前SDK SetHFOV→sdf9 Param::Set后读回1.0472rad，故320像素焦距277.1273455955935，与未舍入理想值相差约.000784px；修复的是已确认实现的预期计算，实际内参/图片没有改写。首轮失败、源码链、实测原始内参与v2结果分别保留，详见VALIDATION_LOG.md。

> 2026-10-31 当前进度为L16 IMU：3个真实私有场景/8组验收61.467s、20个物理窗口/1122条验收IMU消息，实际桌面13项21.443s均通过。五阶段含真实自由落体与同时悬浮+g对照，20窗唯一频率均为100Hz。输入未改变，两模型状态/暂停恢复误差0，归属进程清理干净。完成L16独立提交、推送与远端哈希核对后进入L17 RGB相机；主课01–16与091/092合计18项目。14–16旧草稿均已逐课审查交付，下列进度与旧停止点保留为历史，不覆盖继续授权。

L16固定世界两个CoM零偏置IMU：绿色悬浮刚体关闭模型重力、实际a≈0而原生传感器仍减世界g；红色探针保留重力，真实自由落体时比力≈0。正立/倾斜静止及±yaw旋转只在阶段边界初始化，随后ODE自由推进；参数只改转速/倾角并重载。原生100Hz、ROS门限上界200Hz避免重复限速，不产生200Hz新采样。位姿四元数按q/−q等价比较，近时link查询有前后时钟界，不冒充严格同步；恢复状态不倒放时间或传感器历史。详见VALIDATION_LOG.md。

> 2026-10-31 当前进度为L15激光：修复系统CPU/GPU插件同名类冲突后，完成3个真实图形私有场景/7组验收67.893s、24个双路窗口/289条验收扫描，实际桌面13项21.589s通过。输入不变，墙体/速度/暂停恢复，归属进程清理干净。完成本课独立提交、推送和远端哈希核对后再进入L16 IMU；主课01–15与091/092共17个项目。旧16草稿未部署。以下L14进度和L13停止点均保留为历史，不覆盖当前继续授权。

L15使用系统CPU插件和唯一类名GPU薄适配器，后者只复制原生GpuRaySensor的LaserScanStamped，不生成距离；prepare用已安装SDK离线编译缓存并记录源码/库/构建指纹。三阶段基线→移墙1m→恢复配置位置分别取得新扫描，finally另恢复进入时的真实墙体及暂停。±45°碰撞/外观对照、±30°墙距与极端无回波独立核对；scan_time/time_increment当前为0，不用它们估频或推断实体滚动采样。首轮失败、官方issue768、nm同名符号与修复证据见VALIDATION_LOG.md。

> 2026-10-31 用户明确“继续”，已解除下列L13历史停止点，恢复从L14起的逐课实施。L14接触传感器已完成3个真实私有ROS/Gazebo场景、8组验收47.730s、桌面13项19.622s及64项框架检查.188s；实际图形已检查，输入未改变，归属进程清理干净。完成本课独立提交、推送和远端哈希核对后，下一目标为L15激光。当前课包01–14及091/092合计16个项目；旧15–16草稿仍未部署。每次仅一个活动目标，保留既有历史，不设置自动化。

L14实际流程为个人SDF与准备指纹检查→真实插件/质量就绪→台面外落地负对照→中心零速释放→空中空消息→触地→连续落稳后逐消息统计支持力→恢复原暂停状态。质量1→2kg测得9.7119→19.4238N，落高1.4→2.2m测得下落.483→.642s且支持力不变；接触消息可能批量保留不同时刻，不能跨states累加。记录及副作用边界见VALIDATION_LOG.md和本课只读参考。

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
