# 实际验证记录

初始状态：保留用户初始提交 `9e23caf` 与 MIT License。

## G00 环境与中转

- SSH 登录 ikaros 成功，远程目录 /home/ikaros/GazeboTeach 初始为空。
- 确认 Python 3.8.10、Qt 5.12.8、PyQt 5.14.1、Gazebo 11.15.1、ROS Noetic。
- 确认实际 X11 会话 DISPLAY=:1，图形连接成功。
- Windows 已认证 GitHub 账号 StareamCather；仓库 main 可读。
- 通过 Windows 生成 Git bundle，复制到 ikaros 并克隆，保留原提交历史。
- G00 提交 `cc7aad8` 已在 ikaros 生成，Windows fetch 后推送 GitHub；远端 main SHA 与本机一致。
- 中转部署先校验 SHA256，随后在项目边界内解包；没有复制 GitHub 私钥。

## G01 框架验证

- 维护命令通过当前 GNOME 会话发现图形环境，不在应用中硬编码显示编号。
- 真实 Qt OpenGL 上下文创建成功；GL_VENDOR=NVIDIA Corporation，GL_RENDERER=RTX 4090，GL_VERSION=4.6.0 NVIDIA 535.183.01。
- 修正 Classic 11 的版本命令实际返回 255 但输出有效版本的行为；仅此明确版本探测接受该退出码，普通失败探测仍拒绝。
- `/usr/bin/python3 -m unittest discover -s tests -q`：25 项全部通过，包括两项 Linux 实际进程组检查，无跳过。
- 真实双会话集成通过：Master 和 IGN_PARTITION 不同，暂停第一会话时其时间增量为 0，第二会话继续推进约 2.403 秒。
- 暂停后推进 25 步，实测仿真时间增量 0.025 秒；模型从 (4,0,3) 恢复至 (0,0,2.5)，时钟未归零。
- 真实落体验收：方块抬至 z=2.4，恢复物理后落地至 z=0.2。
- 真实 GUI 存活、关闭后重开、两个终端健康检查通过；结束后两个会话令牌下均无残留进程。
- 桌面按钮流程通过：环境识别、选择场景、开始、暂停、单步、继续、验收、查看记录、关闭清理；完整流程约 10.6 秒。
- 两个终端均有有效 PTY，PID=PGID=SID=前台进程组，且互相独立；交互终端 Bash `monitor` 开启。
- 已实际查看工作台截图和无遮挡的 Gazebo 窗口截图：中文正文/控件正常，真实诊断蓝色方块和地面正确显示。
- 图形资源先加载系统配置再追加课程路径，避免覆盖系统 shader/material；终端沿用 VTE 独立会话，避免额外 setsid 导致任务控制丢失。
- 维护 CLI 支持只读诊断、课程清单、全新副本 smoke 和图形检查；诊断 fixture 不计入正式课程。

## L01 第一个落地方块

- `gz sdf -k` 解析通过；正式目录经 `--list` 正确识别为 Classic 11 独立课程。
- 标准边长 0.5 m、质量 1 kg：预测中心高度 0.25 m，连续稳定实测 0.2500 m，容差 ±0.0200 m。
- 挑战边长 0.8 m、初始中心高度 4 m：预测中心高度 0.4 m，连续稳定实测 0.4000 m，容差 ±0.0320 m。
- 验收分别在原先暂停和运行的状态开始，结束后均恢复原状态。
- 0.8 m 方块故意保留 0.5 m 的旧惯性：被明确拒绝；Ixx 当前 0.0416666667，应为 0.106666667 kg·m²。
- 正式课程经实际 Qt 课程树启动，真实 GUI 和两个完整 PTY 终端就绪，暂停/单步/继续/验收/记录/关闭通过，约 17.1 秒，无残留进程。
- 独立参考答案只读显示，查看前后个人 world.sdf SHA256 相同；教材约 2412 个显示字符，参考页约 2010 个显示字符。
- 已查看正式课程工作台及 Gazebo 截图，方块、地面与阴影正确显示。
- 验收算法：读取实际练习配置并核对几何/均匀惯性 → 保存暂停状态 → 暂停确认与抬升 → 运行并读取连续真实落点 → 按几何预测及容差判断 → finally 恢复原暂停状态。

## L02 积木房间：外观与碰撞

- SDF 解析通过；正式课程树加载、专业教材、参考答案、真实 GUI/两终端及全部按钮流程通过，约 19.1 秒，无残留进程。
- 两个平台同形同色，只有碰撞定义不同；基准实测 solid_probe z=1.2500 m、ghost_probe z=0.1500 m，与几何预测一致。
- 在个人副本补上右平台 collision，同时将左平台中心抬至1.4 m：实测 solid_probe z=1.6500 m、ghost_probe z=1.2500 m，新预测被正确接受。
- 平台 collision 厚度故意设为 -0.2：明确拒绝；修复后同一运行会话重新通过。
- 验收保存并恢复原暂停状态，分别覆盖原暂停与原运行两种状态；最终两个测试会话均无令牌进程残留。
- 参考页约1468个显示字符，查看前后个人场景SHA256一致；已查看工作台及清楚呈现两种落点的Gazebo画面。
- 运行流程：读取当前副本的地面/平台碰撞几何 → 按探针足迹和半高预测支撑高度 → 暂停确认并抬升 → 观察真实稳定落点 → 对比预测和实测 → 恢复原暂停状态。
- GUI实体树、碰撞显示、另存世界仍明确属于人工学习项目；另存快照不会自动替换应用的world.sdf入口。

## L03 正确安装传感器：坐标与姿态

- 本机 Classic 11 的 SDF 1.7 场景解析通过；安装标记明确不输出传感器数据。
- 基准 rig=(2,1,0.2)、yaw=π/2；命名挂载帧及标记的真实世界位置=(2,2,0.8)，与 SE(3) 预测一致。
- body link 相对标记原点偏移(0.15,0,0.05)，真实组合世界位置=(2,2.15,0.85)，验证模型原点与几何中心不同。
- 组合变式实测 marker=(-1.94129,1.20342,1.3)，独立预测=(-1.94128714,1.20342096,1.3)，位置误差约3×10^-6 m；方向跨π后的四元数角距约4.7×10^-6 rad，q与-q等价。
- 未知参考系被明确拒绝；修复引用后重新通过。省略marker的relative_to后，真实世界位置=(1,0,0)，默认参考规则被正确接受。
- 验收实际将rig沿世界x移动0.35 m，确认独立marker不随动，finally恢复rig；暂停状态和工作副本SHA256保持不变。
- 正式Qt课树、真实GUI/两个PTY终端、控制/验收/记录/参考页/关闭清理全部通过，约12.5秒，无令牌残留；已查看课程正文与RGB坐标轴场景截图。
- 运行流程：读取当前SDF → 解析位姿表达图与attachment有效性 → 独立SE(3)组合 → 查询真实模型/link位姿 → 用位置误差及四元数角距比较 → 运行期移动反例 → 恢复原位姿。

## L04 可复用的障碍物模型包

- 四次include引用同一本地teach_crate；world、model.config、model.sdf资源解析及真实加载通过，未下载在线模型。
- 四箱基准实际尺寸0.5³ m，落地中心z=0.25 m；xy及yaw与各自include布局一致。
- 私有练习变式将公共模型改为0.8×0.6×0.7 m并更新均匀惯性，实例改名alpha/beta/gamma及新布局。每个真实模型的直属collision/visual几何、质量、惯性均匹配公共入口，落地中心约0.35 m。
- 运行后将model.config入口改为不存在的model_missing.sdf，验收明确拒绝；修复入口后重验通过，说明已运行的旧模型不能掩盖当前损坏的资源链。
- 基准与变式分别验证原暂停true/false均恢复，原位姿位置误差不超过9×10^-6 m，方向误差约1.5×10^-5 rad；所有资源文件SHA256不变，无令牌进程残留。
- 正式桌面课树启动、实际Gazebo及两个PTY终端、暂停/单步/继续/验收、只读参考、记录、退出清理通过。已查看四箱场景与课程正文截图。
- 复查曾出现已存在模型的只读CLI查询超过2.1秒；补测24次查询通常约0.4秒、最大0.598秒且服务端健康。修复为仅-p/-i在总预算内最多重试一次，变更命令不重试；7项注入回归与真实物理验收分开记录，未放宽物理误差条件。
- 算法流程：当前副本URI → 真实搜索父目录 → config入口及SDF格式/几何/惯性校验 → 查询各运行实例 → 抬升并证明实例独立 → 观察实际落地 → finally恢复原6pose与pause。课堂移动命令仅用于已落地的基准箱，显式保留其余位姿分量。

## L05 用图形编辑器制作桌子与房间（2026-10-30）

- 实际Classic Model Editor逐个添加5个Box，通过Link/Visual/Collision Inspector设置位姿及两套尺寸，勾选Static，原生Save As保存模型包；File→Exit与确认框退出。原生SDF1.7/static=1、5links，桌板局部z=.34、四腿-.085，最低面-.46m。
- 实际Building Editor绘制4墙，经Wall Inspector修端点/Height=2/Thickness=.12，Door Inspector设宽.8、高1.4、Elevation=0；原生保存4links，门墙导出左右1.1m墙块及.8×.12×.6m门楣。两编辑器脚本分别从新会话完整重放通过，未手写导出SDF替代GUI。
- 原生桌/房文件保留在tests/fixtures/editor_exports，独立于学生课包。真正创建、Inspector修改和保存的GUI证据与自动几何验收分开；学生人工技能不由参考场景通过代替。
- 9项完整验收通过：缺学生包安全拒绝、参考基准、桌高/门宽挑战、缺入口拒绝/修复、居中SDF1.4+link改名/墙分块、回切保留资源、两份原生GUI包真实重载及再次回切。
- 基准与原生重载：桌面世界z=.85m，预测探针中心=.9m，实测=.9m；挑战桌面1.2m、中心预测/实测1.25m，门洞宽1.2m。原生门宽约.79999968m、高1.4m；真实collision盒核对与全墙厚切片确认洞内无碰撞体。
- 原生导出将90°写为1.5708、180°写为3.14159，按实际序列化误差设1e-5rad轴方向分类容差，保留尺寸/门洞条件；额外.001rad斜墙及洞内阻挡仍拒绝。
- 初次真实Model消息包含geometry-free LINK显示节点，已按类型/层级严格排除Gazebo辅助节点；删除用户visual或geometry、额外用户visual仍拒绝。未通过放松用户visual集合绕过错误。
- lab流程：完整校验两模型包 → 备份world → 按collision最低点补include.z → 原子切URI → 重建服务器 → 核对真实Model几何/姿态与门洞 → 抬升探针并观察支撑 → finally恢复探针和原pause。学生资源SHA不变。
- Qt正式课树、GUI、两PTY终端、暂停/单步/继续/验收、只读参考、记录及退出清理通过，最终约20.3秒；相机初始化使参考桌和完整门洞房间入画。已查看工作台、实际编辑器及Gazebo画面。
- GUI测试使用公开Qt对象/动作和受归属校验的X11双击；保存对话框异步调度，已验证模态保存可继续控制。私有INI预载保存父路径，所有编辑器会话退出后用户gui.ini SHA256不变、无令牌进程残留。

## L06 质量、质心与惯性实验台（2026-10-30）

- 三个离线独立Gazebo实验站：同形不同质量自由落体；相同外形/总质量、不同配重位置的斜坡塔；同外径/质量的实心与环形飞轮。环形DAE由本地程序生成，网格参数、几何与中心惯性一并检查，无在线模型依赖。
- 4个正式场景通过：三站基准；g=6、落体质量.25/8kg、初高2.5m、飞轮质量3kg及匹配惯性的组合变式；负惯性在控制仿真前拒绝；修复后实际重验通过。变式另在暂停时预先排入旧扭矩，确认历史输入未污染正式测量。
- 基准步长.001s、落体200步：两种质量实测落差均.19718m，半隐式ODE预测.197181m。低重心塔相对坡面法线倾角约3.7×10^-7rad，高重心塔约1.5707968rad；支撑投影仅判断静态稳定性，不用于预测完成倾倒时间。
- 同为50N·m单步扭矩、角冲量.05N·m·s：实心/环形Izz分别.1225/.2249kg·m²，实测角速率.408164/.222322rad/s；Iω分别.05000009/.0500002178N·m·s。g=6与飞轮3kg变式落差.12060m，角速率.272110/.148214rad/s。
- 原生Link的gazebo.msgs.Wrench通过Classic Transport在一个物理更新内施加相对扭矩，无ROS/自定义插件。真机发现reset-all不清空排队的wrench：现流程为完整配置校验→保存pause→暂停→确认单步排空旧输入→reset-all→读取实际基线→核对运行Model的质量/COM/六项惯量/几何→定步观测三站→按实际仿真时间求角速率→finally恢复原pause。实验会保留结果位姿、速度和时钟，教材未声称恢复整个动力学状态。
- 配置单独回归通过且未启动Gazebo：短落体、短塔观察、短飞轮角差、弱角速率、跨π采样混叠均有明确拒绝和逐项修复；默认与g=6变式获接受，源资源SHA不变。量程要求落差≥.02m，飞轮.02≤|角差|<2.8rad及|ω|≥.012rad/s；塔观察窗由支点初始角加速度估计下限，默认最低约1.686s、g=6约2.100s，当前2.7s。以上为仪器适用范围，与本课正式场景分开记录。
- 最终真实桌面检查43.411s通过：正式课程树、实际Gazebo、两个PTY终端、暂停/单步/继续/验收、只读参考、学习记录与退出清理，令牌进程残留为空。已查看三站场景及中文课程截图。

## L07 摩擦滑移与碰撞反弹（2026-10-30）

- 两个受控实验站：同一20°斜面上的方向摩擦滑块；同落高/几何/质量、数值接触设置相同而恢复系数不同的落球。mu/mu2为两个正交切向系数，三表面共同collision局部fdir1可选X/Y；各向系数、恢复系数和反弹速度阈值按当前Classic/ODE配对规则核算。
- 5个正式场景217.155s通过：X基准；g=8、low.mu=.2、球e=.6变式；三表面共同Y方向；负摩擦/越界恢复系数在控制前拒绝；恢复原字节后同Y会话实际重验。独立核算原始pose投影及反弹采样，各会话pause恢复、源资源SHA不变且无令牌进程残留。
- X基准0.8s：low沿坡实移.779768317m（连续近似.778681326m），high仅.000400164m。Y方向两块均约.000400164m，证明低mu所在方向转离坡向后实际使用mu2=.8，未把mu2解释为动摩擦。
- 基准反弹真实三点(t,z)：(.889,1.18615)、(.989,1.22539)、(1.089,1.16654)；直接采样最大z=1.22539m，三点括峰拟合z=1.22588005m，升高.90588005m，对比理想e²H=.9472m。g=8/μ=.2/e=.6变式滑移.395156974m，反弹升高.505155922m（理想.5328m）；无恢复项的球未见明显反弹。教材区分采样最大、插值峰、理想值与软接触/离散近似偏差。
- 14项独立纯配置回归通过（未启动Gazebo）：方向冲突/非切轴、摩擦/恢复系数、观测信号、阈值关闭、跑道终点越界与初始坡外悬空、全局/两球速度上限截断等均明确拒绝，逐项原字节修复后通过；默认、g=8和Y配置仍接受。
- 首轮发现静态surface声明max_vel=.1、运行值0：定位11.15.1 ODECollision::Load的静态强制转换。现报告分别保留saved_surface、expected_runtime_surface、actual_surface，只映射已确认static的max_vel，动态与其它字段严格比较，未放宽容差。
- 第二轮真实轨迹显示e=.8球仍在z≈.32m微弹；定位QuickStep在添加bounce目标后仍按c_v_max夹紧。现全局及两球max_vel=10m/s，两球均min_depth=.001m且数值接触设置相同；配置前置要求全局/两球上限≥max(e×预计撞速)+.5m/s，避免截断恢复项。修复未改反弹验收条件/容差；未修改宿主ODE库。
- 运行流程：完整配置与采样适用范围检查→确认暂停→单步排空历史输入→reset ALL及实际stats基线→核对7个真实Model surface/几何/质量/惯量→排序合并7个事件并记录4动态pose与实际时钟→独立投影滑移、确认真实上升/下降后拟合峰→仅恢复原pause。失败也保留原始测量JSON和原异常。
- 正式桌面检查53.706s通过：课树进入L07、实际Gazebo与两PTY终端、暂停/单步/继续/验收、只读参考、学习记录和退出清理，令牌残留为空。已查看中文课页与完整两站场景截图。

## L08 求解步长与接触稳定性（2026-10-30）

- 单接触双球承载：同半径.15m、质量2kg，在各自静态平台比较soft/firm局部接触刚度；pair kp串联、kd相加，计算局部ERP/CFM与min_depth+Mg/k的理想几何重叠。明确此观测不是内部LCP残差或力测量，不把接触柔顺性全算成数值错误。
- 小型官方SDK客户端只读请求physics_info，显式1.5s超时并校验响应request/type/序列化；确实读到运行服务器的solver、dt、iters和global参数。prepare按源码/SDK版本/编译flags哈希构建到独立用户cache，个人副本保存metadata，界面后台准备；无需学生先掌握C++。未加载服务端插件或修改宿主库。
- SDF解析及11项纯配置回归通过。正式7场景338.228s通过：quick80/.001基准、仅quick20、仅dt=.002、仅soft两侧kp4000、仅world求解器、负kp在控制前拒绝、修复后world实际重验；原始文件SHA不变，所有会话无令牌进程残留。
- quick80基准两次均值：soft重叠.019529608347m（理想.01972）、firm .000294383333m（理想.0002962）；soft尾部5点std约3×10^-9m、range约7.7×10^-9m。所有尾部实际link.enabled=true，显式禁auto-disable和额外速度衰减；不把非常小的数值波动解释为纳米级物理准确度。
- 单变量实测：quick20结果几乎相同；dt=.002时soft .019535255646m、firm .000294642857m；soft两侧kp2000→4000（pair1000→2000）后重叠.009814849925m（理想.00991），firm不变；world时soft .019720001588m、firm .0002962m。没有要求误差、噪声或耗时随配置单调排序，也不据此声称world在任何场景都更好。
- 每配置2次reset测量、各1.0–1.4s五点尾窗，独立从真实pose/stats回算几何重叠、均值、population std、range、有符号理想偏差及重复差；本例重复均值差0。服务器显式seed42，reset ALL会重新设seed，但同机重复相同不保证跨硬件/线程/消息时序逐位一致。
- 端到端wall包含CLI、Transport、ACK和采样，未冒充纯求解CPU时间或自由运行RTF；physics_info.real_time_factor是目标配置，update_rate=0无节流时也不能把该字段当实际sim/wall比。接触计算使用配对kp/kd生成的局部ERP/CFM，不声称只改global CFM就改变这些显式局部接触。
- 算法流程：完整模型/信号/阻尼时标校验→暂停并核对实际physics_info→每repeat单步排空历史输入→reset ALL及stats基线→首次核对真实Model/surface/质量惯量→定步预热和活跃尾部快照→重算几何统计/重复差→仅恢复原pause，保留最后结果与时钟；失败保留原始JSON和异常。
- 正式桌面检查59.602s通过：课树进入L08、后台prepare、实际Gazebo与两PTY终端、暂停/单步/继续/验收、只读参考、学习记录与退出清理，令牌残留为空。已查看完整双球承载场景和中文教材截图。

## 共享控制修复：按实际状态确认暂停（L09 调试触发，2026-10-30）

- L09首次正式g=6场景已取得正确完整轨迹，但最后重复发送pause=true的gz工具进程报告boost线程自join异常，课程整体判失败；原始日志保留，没有将物理测量成功冒充整个流程成功。
- 共享LessonSession.pause现先读取真实world_stats；已匹配不发送控制，需要改变时仅发送一次，再有界只读确认。命令报错但实际目标已确认时保留原错误并明确记录warning；未生效、状态未知、取消或服务端退出均不记录成功，不盲重发。
- UI缓存、状态与学习事件只在实际确认后更新。单步每次重新确认暂停，避免Gazebo窗口改变状态后继续相信旧缓存；步进本身仍只发一次，错误不重放。
- 18项Popen级故障注入回归通过，覆盖真实管理路径的环境/归属、同值零写、延迟生效、退出报错已生效/未生效、未知输出、取消、停止、旧缓存和步进不重发；Linux完整43项单元测试通过。
- 实际Gazebo集成（GUI、终端、双会话隔离及清理）28.855s通过，正式桌面诊断流程17.013s通过；已交付L01桌面与真实验收重新验证22.765s通过，均清理干净。L09自身的初始化/收尾确认与后续重新验收另在该课记录。
- 算法流程：实际状态读回→同值跳过/异值单次设置→有界只读确认→检查会话仍有效→更新缓存/metadata/学习记录；测量脉冲和模型重置不套用自动重发。

## L09 基础：被动单摆与重力滑轨（2026-10-30）

- SDF1.7两机构均以world→fixed→动态frame/guide再连接revolute/prismatic child，避免以static整模型冻结关节。无PID、无持续力矩/推力控制。球COM距铰点.8m、半径.1m、质量1kg；杆只作无质量visual，I_COM=.004、I_pivot=.644kg·m²，以物理摆而非误用质点摆预测周期。
- SDK单连接仅请求实际Model/physics_info、发布有界world multi_step并等待真实stats ACK；每点Joint.angle直接来自运行Joint::Position，同时以child COM和完整朝向独立校核θ=θ₀+q。初始几何θ₀=.2rad而joint q₀=0；native joint.pose按child坐标序列化与源SDF表达不同，已按明确帧变换核对，未视为漏加载。
- 13项纯配置风险回归通过；独立3项清理协议注入回归通过并标not_physics。正式5场景重跑261.088s通过：基准、仅g=6、仅滑轨lower−.5、axis/topology/limits非法组在控制前拒绝、原字节修复后重验；源资源SHA不变、所有令牌进程清理干净。
- 基准110点/5.45sim秒，真实同向过零周期1.804358715s，对比含弱阻尼/有限幅近似1.804389167s。t=.2s滑轨q实测−.188854934m、解析−.187885317m；末5点均约−.600000000m。g=6为140点/6.95sim秒，实测周期2.307152843s、近似2.307220580s；早期q−.114752226m、解析−.114163066m。
- lower仅改−.5m后，摆周期及滑轨早期位移保持基准结果，末端停靠−.500000000m。限位是实际关节约束，不由地面碰撞或控制器制造；guide/frame的world位置/朝向、bob姿态与carriage轴外位移均真实核对。
- 初始化和收尾复用同一pause确认协议：先读实际状态，同值零写；需改变仅一次写，异常后只读确认并保留原CLI错误，不盲重发。measurement_passed与cleanup.confirmed都真才输出成功JSON，失败保留实际样本与原始异常。恢复原运行状态后仿真自然继续，不声称冻结在采样末帧。
- 算法流程：全部配置/拓扑/惯量/信号窗口校验→实际pause确认→排空单步的iterations+1/time+dt ACK→reset ALL→核对运行physics和joint/link配置→单连接定步采真实q/pose/时间→交叉几何和同向过零周期→滑轨早期响应/末端止挡分析→确认原pause后记录最终结果。
- 正式桌面流程69.042s通过：课树进入09、后台构建、实际Gazebo与两PTY终端、暂停/单步/继续/验收、参考只读、记录及退出清理，令牌残留为空；已查看完整单摆/滑轨与中文教材截图。这里只保存L09基础部分；用户追加的091/092倒立摆填空选做仍须依次真实交付，当前L09扩展目标继续，尚不进入L10。

## L09选做A（091）：单级倒立摆控制填空（2026-10-30）

- 真实小车沿X的prismatic与有质量均匀杆绕Y的revolute构成欠驱动系统，仅小车接受水平力。标称M=1kg、m=.2kg、杆全长1m/半径.02m，I_COM,yy=.0166866666667kg·m²；上垂线为零、+Y使杆顶向+X。红色杆顶球仅visual方向标记，不增加碰撞、质量或惯量。
- learner.py保留state_error、feedback_force、clip_force三个TODO，control流水线和K已给定。单位/形状/角度包裹、非零target_x、增益符号/顺序、双向与不同限值的clip、组合控制均按行为检查，不靠删除TODO文字判断。参考代码独立，默认reference_demo不能获得学生作业通过，不覆盖个人填写。
- 正式4场景108.788s通过：默认参考演示的真实三回合可稳定，但学生verify明确拒绝；独立临时副本填入参考函数后student三回合通过；运行中修改learner源码但未重载，在启动新trial前按实际SHA不一致拒绝；错误零控制被函数检查拒绝，其另行无学分observe保存273点，杆角峰值3.133512rad，真实呈现未稳定而非只作静态拒绝。发布模板的三个TODO未填。
- 两模式共6个真实回合，每回合10仿真秒、1001个实际控制样本。正负初值及(+.1m,−.06rad)加扰动分别运行，第三回合4–4.25s确实施加+2N。真实CSV校核模式、执行源码SHA、每10ms恰好一次Python调用、杆几何方向与SDK关节角，以及100Hz控制/1kHz保持的小车力；模型质量、惯量、阻尼、步长、重力与固定world指纹均检查。
- 正向初值回合小车位置峰值.2606878m、控制力峰值9.5240096N。扰动回合末8–10s RMS约为：x=.0001233699202m、xdot=.0001614060168m/s、theta=1.9487381326774×10^-5rad、角速度=7.81509735×10^-5rad/s，均在既定.02m/.035m/s/.01rad/.04rad/s范围内。学生函数限幅与桥接保护分别检查，不用后者掩盖遗漏的TODO。
- 单独的浮点端点修正与物理运行分开：原CSV时刻7.999999999999996s按数学闭窗8–10s应纳入，Python仅增加1e-7s时间端点容差，远小于10ms采样间隔；未改CPP、K、物理模型、轨迹或验收阈值。保留原6回合CSV/报告，离线重算末窗各201点，两模式同项RMS数值差小于8.54×10^-17；`.deploy/lesson091-endpoint-recheck.json`明确new_physics_run=false、physical_sessions_started=0，未将重算写成新物理验证。小RMS与舍入差仅描述无噪声理想仿真数值，不代表实物精度。
- 最新Python经真实桌面再次验证：demo流程17.060s，student流程48.933s（实际执行三个回合），诊断fixture回归16.937s，43项单元回归通过。界面检查覆盖模式区分、个人临时参考填空、重载/验收/只读参考/记录及退出清理；已查看[演示工作台](images/lesson091-demo-ui.png)、[学生工作台](images/lesson091-student-ui.png)和[真实小车杆场景](images/lesson091-student-gazebo.png)。
- 算法流程：准备控制桥→按模式载入实际Python源码并记录SHA→从真实关节读取四状态→100Hz调用误差/反馈/限幅流水线→1kHz保持小车力并独立叠加已记录扰动→保留原生状态/几何/调用证据→联合检查函数行为、模式/源/模型指纹、三回合峰值与闭窗RMS→读回确认原pause后保存成功结果。每回合只在开始时初始化关节位置/速度，后续不靠改写姿态稳定；不恢复旧轨迹/仿真时间。
- 该课仅交付近直立单杆局部稳定。本桥超出.65rad局部角保护边界时将控制力置零，完整起摆必须另建扩展项目。下一目标为092双级倒立摆选做，尚未开发或交付；L09扩展目标保持活动，未进入L10。

## L09选做B（092）：串联双级倒立摆控制填空（2026-10-30）

- 真实小车与两根均匀圆柱杆串联，仅小车有水平控制力，两铰链无主动电机。M=1kg，m1/m2=.2/.15kg，杆全长.6/.5m、半径均.015m，I_COM,yy分别.00601125/.0031334375kg·m²；给定K6按完整耦合标称模型设计，未为通过实测而调整增益、模型或阈值。单/双级表示杆数，状态分别为4/6维，不把杆数混同系统微分阶次。
- learner.py保留三个TODO：原生相对关节量转换为[x−target,xdot,wrap(q1),q1dot,wrap(q1+q2),q1dot+q2dot]，给定负状态反馈，双向20N限幅。完整参考独立查看，默认reference_demo不计学生完成；个人代码由真实Python回调执行，错误或未完成实现不回退为参考，测试只在独立临时副本中填参考，发布模板不自动填答案。
- 正式4场景119.267s通过，报告`.deploy/lesson092-scenarios.json`：默认demo真实三回合可稳但学生verify拒绝；临时完成student三回合通过；保存代码未重载按运行源码SHA不一致在新trial前拒绝；零控制被函数检查拒绝，另行无学分真实观测未稳定。3个实际会话令牌均清理干净，正式轮源模板SHA不变；纯数值解析与Python行为回归单独标明，未冒充物理运行。
- 两模式共6个12仿真秒回合，每回合1201个实际控制点、[10,12]s闭窗201点，时间端点容差1e-7s。初态绝对角同向(+.03,+.03)、反向(+.03,−.03)均实际核对；第三回合原生q2约−.06rad而第二杆绝对角约−.03rad，4–4.25s施加+2N水平扰动。模式/源码SHA、每10ms一次Python回调、1ms物理步保持、学生限幅与桥接保护分别核对。
- student扰动回合独立CSV重算末窗RMS：x=.000169564648386m、xdot=.000257522206928m/s、theta1=2.295362794514×10^-5rad、omega1=1.886176711803×10^-5rad/s、theta2=2.189222607045×10^-5rad、omega2=2.011431093421×10^-5rad/s；均通过既定.025m/.04m/s/.01rad/.05rad/s六状态条件。该回合控制力峰值11.1444297914N。demo另有独立轨迹与统计，不声称两模式RMS逐位相同；微小数值来自无噪声理想仿真，不代表实物计量精度。
- 角度证明来自原生串联joint q/qd及两根杆实际世界轴方向，不以同一个杆轴冒充第二杆。真实COM和杆轴重建第一杆顶端、第二杆底端，与SDK上铰链Anchor对照；第一杆端到铰点最大误差3.094943391×10^-7m，处于.002m既定容差内。零控制的完整保留CSV独立统计为276点、两杆绝对角最大值3.134985018978rad，未把observe较早快照的127点当作完整记录。
- 最新双杆GUI验证通过：demo流程17.274s、student流程54.638s，后者真实运行三个12s回合；messages为空、归属进程退出清理通过。演示不计作业、临时学生填空、重载、实际Gazebo/终端、参考只读与记录均经桌面流程检查，截图为[演示工作台](images/lesson092-demo-ui.png)、[学生工作台](images/lesson092-student-ui.png)与[真实双杆场景](images/lesson092-student-gazebo.png)。
- 共享桥新增双杆路径后，已交付091完整4场景再次实际回归109.003s通过，43项单元回归通过，所有归属令牌清理干净；单杆默认路径的作业/演示/SHA与错误控制检查保持有效。该回归独立于双杆正式轮，不把单杆成功代替双杆验收。
- 算法流程：载入固定双杆模型与当前Python源码/模式并记录SHA→读取真实q1/q2及关节速度→转换两杆绝对角/角速度并组成六维误差→100Hz执行给定K内积和学生clip→1kHz保持小车力、单独叠加记录扰动→保留原生CSV/轴向/COM/Anchor/调用证据→函数行为、模式/源/模型指纹、非零初态、几何与三回合边界/RMS联合判定→读回确认原pause后保存成功结果。每回合只在开始时初始化，之后不靠改写姿态稳定，也不恢复旧轨迹和时钟。
- 该课交付近直立双杆局部稳定，任一绝对杆角超出.65rad保护边界时控制力置零，完整起摆须另建扩展项目。L09基础、091和092均已验收，本次发布与远端核对完成后L09目标完成；下一独立目标为L10，原主课01–37编号不变。

后续每课验证记录在实际执行后追加。

## L10 关节电机与转速/位置PID（2026-10-30）

- 固定1 kg、半径.25 m、厚.06 m均匀转子，Izz=.03125 kg·m²，绕Z关节黏性阻尼.1 N·m·s/rad。原生JointController每1 ms更新，默认有限六阶段共25仿真秒：恒转矩、速度P/PI、位置P/PD、低输出上限PI；结束清目标后自然减速，不隐式循环或主动零速制动。指针和底座为visual参照；模型没有电压、电流、反电动势或齿轮电气动态。
- 正式6场景146.379s通过，报告`.deploy/lesson10-scenarios.json`，2个真实会话：运动多圈暂停切换至恒转矩；基准六阶段；仅speed_p_kp .4→.8；非法参数/world在物理控制前拒绝；修复后同变式实际重验；有效参数保存但未重载拒绝。三个完整六阶段周期各25,000行、共75,000行原生1 kHz记录，另有切换后的独立1,000行恒转矩响应；纯数学、配置和编码器证据回归与真实运行分开记录。Linux完整43项单元检查通过，正式轮源模板SHA不变，所有会话令牌进程清理干净。
- 恒转矩.4 N·m：t=.2 s实际ω=1.892993602 rad/s、q=.210332459 rad，连续解1.890830304/.209115530；t=.5 s实际3.194483114/1.004917984，连续解3.192413928/1.002370647。有限步长离散观测与连续模型预测分别保留，不将连续曲线填入真实状态字段。
- 负载.2 N·m后的末1 s速度P均值2.800000000 rad/s，仅P增益改.8后3.333333333；标准PI均值3.998899476、目标误差RMS .001233989 rad/s。位置P/PD目标方向峰值1.221075022/.801073930 rad，PD第一有效步实际原始请求281.6 N·m、限幅后2 N·m。低上限PI均值1.500039844 rad/s、命令均值.35 N·m、带符号原生I项均值−.8 N·m，真实支持输出能力不足而非积分未启用的判断。
- 每步原生PID的误差、积分输出夹紧、D差分和最终命令均按实际步前q/ω独立重放；输入账本为控制命令减独立负载，黏性阻尼另计，不称反作用力传感。基准低上限相位，独立整段冲量分别求和后的有符号残差约−1.426×10^-13 N·m·s，课程同相位逐步累计最大前缀残差约2.188×10^-16，基准各相位最大前缀约1.416×10^-14；统计量不同，未合并为一个求解器精度。小RMS和浮点残差只描述理想仿真及离散账本一致性，不代表实物精度。
- 旧初始化故障曾在第一项matched paused cut真实复现：SetPosition(0)/SetVelocity(0)后native q=12.566370614359172 rad（4π）、ω=0，而实际转子世界X轴近[1,0,0]；原始故障记录保留，其余原计划四项未开始，未算作覆盖。修复仅作用于阶段边界：完整JointController::Reset清空旧控制目标及PID历史→恢复Load捕获的实际绝对位姿、零速并确认几何→经公开ODELink::GetODEId取得body→枚举核验唯一world hinge、锚点/轴及SDK身份→公开OpenDE C API dJointSetHingeCumulativeAngle(...,0)一次→严格读回q/ω/位姿→配置新PID及目标。未调用private C++ helper、重建关节、修改宿主库或在运动中持续改姿态。
- 正式暂停多圈切换从实际q=9.027792964 rad、ω=2.800000040 rad/s进入新恒转矩阶段，真实q/ω初始化为0后取得完整1,000步响应；每次有界推进请求均核对实际iterations与sim_time ACK，不盲重放阶段请求。三个正式完整周期共18次同步调用，以及此切换案例，均实际报告counter_offset_removed=false：同步前计数已经为0，不能把“调用同步”写成每次都消除了4π。
- 正式Qt桌面流程43.510s通过，报告`.deploy/lesson10-ui.json`：真实Gazebo、两PTY及交互终端任务控制、暂停/单步/继续、完整六阶段验收、只读参考、记录与关闭清理；messages为空。root已查看[中文桌面](images/lesson10-ui.png)和[真实转盘](images/lesson10-gazebo.png)截图，确认界面与场景可见。
- 额外公开重置回归21.731s通过，报告`.deploy/lesson10-public-reset.json`：1个独立真实会话，从多圈速度相位经LessonSession.reset_models()重启默认恒转矩阶段，测试本身没有写位姿或计数。实际仅取得1,000条1 s恒转矩行，full_six_phase_cycles_executed=0，不冒称另跑六阶段；原pause=true保留，sim_time由4.672推进到4.673而未回退，最终5.674 s。阶段初始化读回q/ω=0、counter_offset_removed=false，个人文件及源SHA不变，所有归属进程清理干净。
- 算法流程：校验个人参数、固定world与工具指纹→原生模式请求并建立阶段真实初态/计数参考/PID历史→每1 ms原生Model更新控制器，按步前时间独立叠加负载→配对保存步前PID/命令与步后q/ω、dt及世界时钟→重新核对阶段末指纹→独立重放PID、角展开/几何和动量账本→按各阶段稳态、RMS、峰值与饱和条件联合判定→读回确认原pause后保存成功结果。失败保留原始记录，不恢复旧轨迹或世界时钟。本次提交、推送及远端核对完成后，L10目标完成，下一独立目标为L11，主课01–37编号不变。

## L11 差速小车、轮编码里程计与滑移（2026-10-30）

- 固定差速小车采用设备stock libDiffDrivePlugin.so：r=.1 m、b=.36 m、两轮各.2 kg/宽.01 m、轴均+Y，车体2 kg与低摩擦后球支撑.05 kg。通过Classic gazebo.msgs.Pose的position.x与旧yaw数值编码标准(v,Ω)，va=−Ω；插件直接设置轮关节速度，由真实接触推动车身，无车体轨迹写入、无本课转矩PID。几何轮距/半径按真实anchor与collision包围盒推取并缓存，编码积分由观察工具独立完成，不冒称stock发布了odom。
- 正式v2报告`.deploy/lesson11-scenarios-v2.json`：7场景107.253s通过，3个新建真实会话。场景覆盖正常直行/自转/圆弧；仅forward_speed_m_s .25→.35；非法命令及缺失stock关节在控制前拒绝；修复后同变式真实重验；有效个人文件修改但未重载拒绝；非零运动中的公开模型复位清指令；新会话低摩擦直行。基准3窗、速度变式3窗、修复重验3窗、低摩擦1窗，每窗2000个真实1 ms测量点，共20,000点；升速/就绪/降速/停稳与Reset样本不混充测量窗。正式源指纹另存`.deploy/lesson11-v2-source-fingerprints.json`，源模板不变、所有会话令牌残留为空；43项test_*.py单元检查.136s通过。
- 正常2 s基准：直行真值局部x=.493949011878 m，对比理想.5 m；自转真值yaw=1.166715789926 rad、编码yaw=1.199115308570 rad，对比理想1.2 rad，轮轴中点净平移.005228560 m。圆弧真值(.419131631463,.223568732777,.979952065559)，对比理想(.420735492404,.229848847066,1)；坐标前两项m、最后一项rad。正常场景理想—真值与编码—真值均按全窗最大误差判定，保留原.04 m/.08 rad门限；直行额外航向.05 rad和低摩擦航向.2 rad也均按全窗max|Δyaw(t)|，不是仅看最终转回原方向。
- 仅前向速度改.35后，直行真值x=.695120646354 m（理想.7），相较基准增加.201171634476 m；自转目标保持不变，实际yaw=1.166659079706 rad，未要求接触结果逐位相同。变式圆弧真值(.587893892894,.314921997149,.983524228756)，理想(.589029689366,.321788385892,1)。所有阶段使用各自真实测量窗起点转局部坐标，未把加减速路程或偏置质心轨迹作为理想轮轴中点位移。
- 新低摩擦会话只做直行，两轮mu/mu2共同.001、地面仍.8：真实平均轮程.495813447061 m，真值累计路径.013414294897 m、净位移.012701120456 m、全窗最大|转角|.059320124642 rad，末编码—真值距离.485457098495 m。路径长度与净位移分别核算，真实支持“轮子转而车身少走”，不把绕圈后回原位当少走，也不宣称此差距是真实车辆通用滑移率。
- 原生轮角前后状态、旧Pose编码、轮速映射、GT锚点/车身朝向与独立SE(2)编码积分逐步核对；全部阶段相邻CSV的轮q后值与下行前值连续差最大0，基准圆弧真实跨越一次±π航向分支并正确展开。b̂=.48 m仅重放同一真实CSV的编码估计，明确offline_replay_only=true、new_simulation_started=false，不改变stock实际轮距或另计新物理会话。初始/最终零速均以实际轮/body速度连续.05 s ACK确认，不强制q=0或以命令代替停稳证据。
- 公开CLI入口另经离线检查，报告`.deploy/lesson11-offline-cli.json`：set-surface的slip/normal均只修改临时个人SDF，check-config核对模式并要求重载；replay-calibration 0.48读取正式基准圆弧的原始CSV，与独立重放逐项一致。源模板和原始轨迹不变，physical_sessions_started=0，不计作新增仿真或轮面物理验证。
- 公开Reset在速度变式同一真实会话的非零运动中触发：stock本身无Reset清wheelSpeed，观察工具通过合法topic发送零命令，丢弃跨复位积分基线，实际62个1 ms步达到停稳条件，其中50步连续合格，在200个归属步预算内确认；随后stopped、不自动重播。此场景additional_verification_recorded=false、test_writes_body_pose_or_velocity=false，未额外跑验收周期，也未通过测试写位姿或速度制造停止。
- 保留宽轮预检失败证据：原.04 m轮宽时，2 s自转GT约1.07924 rad、理想1.2，超过既定.08 rad误差；轮速/编码符合、姿态高度正常，四个.5 s窗反推有效接触轮距约.39999 m与b+原轮宽=.4 m相近，但未测接触点，未声称证实承载边缘的因果。按点接触运动学近似目标仅将轮宽改.01 m，并同步径向惯量Ixx=Izz=.000501666666666667 kg·m²，轴向Iyy=.001不变；r/b/质量、摩擦、速度命令与物理误差门限未为过验收而调整。新窄轮仍有约.03328 rad自转残差，保留实际接触偏差。原始source/raw在`.deploy/lesson11-contact-evidence`及`.deploy/lesson11-preflight-evidence/wide-wheel`保存。
- 首次正式轮的三段canonical物理验收已过，但额外测试把两个GetVelocity端点积分误当Joint::Position角增量的硬上界，导致整体报告失败；原报告`.deploy/lesson11-scenarios.json`、原测试d2b82fe8…和原CSV均保留。核对本版本QuickStep公开源码：含ERP的临时速度用于姿态积分，之后恢复non-ERP报告速度，端点速度积分不提供该无依据上界。仅修正错误测试假设，增加实际q跨行连续性检查，并保留速度积分差作诊断；未改backend、物理模型或观测阈值。原CSV SHA ea93fbe7…经新测试35d30440…只读重分析通过，`.deploy/lesson11-preflight-evidence/encoder-integral-check/reanalysis-v2.json`明确new_physics_run=false、physical_sessions_started=0；之后才另行执行上述v2正式真实轮，未把重分析冒充重跑或声称量化全部ERP贡献。
- 正式桌面流程29.140s通过，报告`.deploy/lesson11-ui.json`，13项实际检查：中文13项目课程树、真实gzclient/两PTY（含交互终端任务控制）、暂停/单步/继续、完整三阶段真实验收、只读参考不改个人world、学习记录、归属截图与关闭清理。messages为空、close_window_cleanup令牌残留为空；已查看[中文工作台](images/lesson11-ui.png)与[真实蓝色薄轮小车](images/lesson11-gazebo.png)，分别343105/137307字节，车身、薄轮、金色标记及地面清楚可见。
- 算法流程：校验当前参数/轮面SDF并准备带源/SDK/stock指纹的观察工具→核对运行模型与配置→真实零命令停稳ACK→.5 s升速及≥.3 s就绪→取首measure步前真实q/GT基线→采2000个1 ms前后状态与标准/旧接口命令→独立用Position增量积分编码SE(2)，用两anchor中点/车身航向取得世界真值并展开yaw→按全窗误差或低摩擦轮程/路径/净位移/漂移联合判定→.5 s降速与实际停稳→确认原pause后保存成功结果。公开复位另清命令与基线，不把跳变计作行程；失败保留原始数据与原异常，不恢复旧轨迹或时钟。本次提交、推送及远端核对完成后L11目标完成，下一独立目标为L12，主课01–37编号不变，旧14–16未部署草稿不纳入发布。

## L12 Classic Transport消息滑门与三层执行证据（2026-10-30）

- 固定2 kg门板由world→+X prismatic约束支撑，原生位置PD为P80/I0/D20、输出±20 N，被动阻尼4 N·s/m，物理步长1 ms。通信回调只校验信封和访问受保护的缓存快照；received不充作新物理样本。Begin消费pending并设置目标，原生控制器本步更新后BeforePhysicsUpdate读取PD控制力输入发applied，End读取真实q/速度/位姿，连续200个1 ms区间满足.005 m/.01 m/s条件且在3仿真秒内才completed。GetCmd/GetForce(0)是控制输入账本，不是力传感器或全部约束反力，显式阻尼另行作用；无helper位姿/速度或预设轨迹写入，启动默认hold0。
- 早期四字段协议的私有三事务、暂停received→物理步→原票据续等、pending Reset及旧epoch拒绝已通过sanity，原source保存在`.deploy/lesson12-protocol-v1-source`，原始记录在`.deploy/lesson12-private-evidence`。这批证据属于旧协议私有调试，不混入当前五字段协议的正式会话、事务或样本总数。
- 审查发现通用query可转发原生写请求，以及并发复用逻辑身份可能误认另一发布ACK的静态可达边界；没有实际执行误删，也没有宣称旧竞态已在物理会话复现。修复后Python与C++ query白名单都在Transport初始化前执行，只允许gazeboteach_transport_snapshot与安全未知探针gazeboteach_transport_missing，entity_info走独立模式。纯Python4项检查native_calls=0；实际新SDK对entity_delete/world_sdf_save/world_sdf_generate/entity_info四项在假Master TCP sentinel下均返回拒绝，连接尝试0，未启动Gazebo Master或物理会话。证据在`.deploy/lesson12-protocol-evidence/native-pre-network-guard.json`，原生-r仍按具体请求语义理解，不泛称只读。
- 新协议Request.data为RUN CLIENT COMMAND_NONCE EPOCH ATTEMPT_NONCE，每次send自动生成独立attempt_nonce；ACK、票据、CSV及客户端匹配贯通，同逻辑seq/nonce的新尝试匹配本次rejected，旧accepted ACK因attempt不同不能满足它。成功ACK另核requested target，applied/completed再核实际applied target；wait沿原票据attempt仅订阅不重发。旧armed/counter交错仅作pure_legacy_schedule_analysis_not_observed_physics，new_physics_run=false；完全相同裸包仍属同attempt，不承诺识别没有新标识的另一次发布或跨崩溃exactly-once。
- 首次正式报告`.deploy/lesson12-scenarios.json`在69.899s判失败，仅前2组通过，后续未完整执行。原因是测试递归扫描continuation票据的received_network_events历史，把旧send的published_once当成本次wait新发布；原current/historical数据保存在`.deploy/lesson12-protocol-evidence/continuation-history-failure`。修正测试40d21…只检查本次事件及received计数不增，并在仍暂停、已推进20个归属步时实际watch取得applied；纯字典归属回归分别证明历史PUB可保留、本次PUB或新received必须拒绝。backend、模型、增益及物理门限没有为第二轮变动，旧失败报告保留，未把离线重分析冒充完整新物理验收。
- 当前完整正式v2报告`.deploy/lesson12-scenarios-v2.json`：8组258.719s全部通过，2个同时存在的全新私有Master会话。覆盖默认真实开关/隔离、唯一开度变式三事务、暂停接收/旧ACK/原票据续等、正确线类型安全拒绝及有界发现/RPC失败、公开Reset取消/旧epoch、新协议真实并发两attempt、非法与未重载个人输入拒绝、原字节修复后三事务重验。源指纹见`.deploy/lesson12-v2-source-fingerprints.json`，源模板不变、两个令牌残留均为空；43项test_*.py单元检查.131s通过。
- 正式事务按(run,epoch,client,seq,command nonce,attempt nonce)独立去重：A默认2个、B变式12个，共14个已实际应用事务，16,324个真实1 ms post样本。组成是4×1385个.6 m开关样本、8×1298个.4 m开关样本及2×200个prepare_closed样本；另1个received后复位取消、未applied的事务单列，不算真实应用/运动样本。完整verify通过真实target0消息prepare_closed→open(p)→close0建立基线，不借Reset初始化；所有已完成事务末尾均独立核实连续200个到位区间与控制力≤20 N，首末post时间戳差.199s不误作缺少一个区间。
- 默认A的完成采样端点q=.5998694972696924 m，关闭q=.000130502730307818 m，各1385步/1.385仿真秒；B只改open_target_m=.4后开门q=.3998259686394453 m，关闭约.00017403136 m，各1298步/1.298仿真秒。终点、持续到位与实际运动跨度分别核对，未要求用时随开度线性。原生entity_info的Model.pose与link.pose组合后，在同一Model响应内部对照joint q和世界几何；异步RPC与entity_info不同采样时刻不作无依据瞬时等值比较。
- 两会话使用相同/gazebo/default/transport_lab/{command,ack,state}完整主题，但不同私有Master。A真实打开并仍持开时，B确实保持暂停，q=0、控制力0、iteration=3267/sim_time=3.267均未变，received/applied/completed计数均0，未用“A先关回原位”伪装隔离。正式默认开关同时取得54个变化的真实网络state样本；通信证据来自SDK网络，不以status.json代替响应。
- 暂停组在iteration=15107/sim_time=15.107收到.4 m命令，received从3增4，applied/completed仍3，q与时钟不变。推进20个归属物理步后仍暂停，真实当前网络watch及RPC显示applied，q=.0020736377811 m、速度=.196245214866 m/s、PD输入20 N；随后继续并wait同一已接受ticket取得completed，continuation_published_new_command=false，当前事件无新PUB且received计数未因续等增加。票据中保留的历史send日志与本次网络事件分开解释。
- 新协议真实并发组让相同client/seq/逻辑nonce的两发布分别带独立attempt，目标为.4/.6 m：实际一项拒绝、一项接受，winner为.6，2个foreign-attempt ACK被忽略，只有一次真实物理应用，随后完成开关。legacy_race_physically_reproduced=false，证明的是新协议并发行为，未追认旧协议活体错误。安全负组另核busy不覆盖pending、重复/旧seq/错run/epoch/动作/缺失与非法目标拒绝、拒绝seq100不占用接受历史，以及错误线类型在发布前拒绝；未向活跃Master注入冲突线类型。
- 公开Reset取消一个已received但未应用命令，旧nonce只取消一次，epoch更新、旧票据/旧epoch请求拒绝，新epoch真实开关重新通过；helper不写body pose/velocity，Gazebo正常模型复位与新hold0分工明确，不自动demo，不把复位跳变当轨迹。RPC另逐项检查Response.id/request/success/type、Any.STRING字段、反序列化、query nonce与boot/world；未知安全查询和无订阅者主题有墙钟上限，CLI退出0不冒充收到或执行成功。
- 真实Qt桌面13项检查26.829s通过，报告`.deploy/lesson12-ui.json`：中文14项目课程树、真实gzclient与两PTY（交互终端含任务控制）、暂停/单步/继续、完整prepare/open/close验收、只读参考不改world、学习记录、归属截图与关闭清理。messages和关闭残留为空；已查看[中文三ACK工作台](images/lesson12-ui.png)及[真实滑门场景](images/lesson12-gazebo.png)，分别296134/123225字节，蓝门、灰轨、金把手与红绿标记可见。UI额外验证不混入上述正式2会话/14事务统计。
- 算法流程：校验个人参数/固定模型/SDK与源指纹→查询当前boot/epoch并有界发现完整topic/type→先订阅ACK→生成逻辑/发布双标识并只发布一次→Transport缓存校验/received→物理Begin设目标、原生PD更新、pre-physics控制输入/applied→逐1 ms保存真实q/qd/几何并累计到位/completed→同attempt真实网络匹配与独立CSV/Model校核→确认原pause后保存成功结果。query守卫在初始化前，Reset取消旧身份并清基线，失败保留原始数据与异常，不恢复旧轨迹/时钟。本次提交、推送与远端哈希核对完成后L12目标完成，下一独立目标为L13；主课01–37编号不变，旧14–16草稿仍未部署、不计交付。

## L13：场景展馆——材质、光照、地形与 Actor（2026-10-30至31日）

本课以四个离线展台回答“画面变化时，物理世界改变了什么”。个人 world.sdf 是真实输入；准备程序校验输入、资源与版本并构建缓存，不重写 world/PNG。set-height、set-cycle、set-daylight 只原子修改指定字段，核对修改后的值并提示重载，格式变体、无关材质修改和注释均有回归。系统 walk.dae、木纹及地形 diffuse/normal 资源读取实际路径、SHA与包版权声明，不复制到 MIT 仓库。

最终地形使用原始65×65灰度PNG、4×4m、视觉与碰撞 sampling=1，顶点间距0.0625m，索引16/48对应世界四个±1m平台点。原生高度按图像实际最大灰度归一；本图含0/255锚点，四平台灰度85/170/0/255，对应默认表面0.2/0.4/0/0.6m。SDK高度数据、命中实体的物理射线、四个真实落体球分别验证加载、碰撞和动力学；两个视觉材质不同而碰撞相同的支撑箱另接两个球。底板只有visual，零高度平台不会被隐藏碰撞平面代接。

只读ModelPlugin在物理线程读取真实状态，约50Hz记录场景、射线和Actor，另在3–3.5仿真秒每个真实1ms物理步记录六球尾窗。验收要求同一段至少201连续快照、首末跨度至少0.2s，六球同时满足高度误差≤0.01m、水平偏移≤0.03m、速率≤0.03m/s；逐步iteration和时钟另核，不以50Hz概览插值充数。世界/模型复位由原生接口执行，观察器只清缓存与epoch；新epoch不得使用旧snapshot，完整观察恢复进入前的暂停状态。

人物Actor用系统31骨节点walk网格，旁边无skin标记分离精确根路径与骨骼动作。用实际ScriptTime验证tension=1的零切线Hermite位置，记录真实更新与重复观察行；人物以脚相对Hips的SE(3)位姿变化验证骨骼运动，不能只看全局平移。Actor固定活动循环，仅用世界暂停/继续；static/kinematic/gravity/collision字段按实际报告，不作通用碰撞保证，也不把脚本姿态更新解释成受力步态或避障。

相机助手先在有限预算内确认GUI连接及有效实际相机输出，再单次向 ~/gui 发布gazebo.msgs.GUI，等待 ~/user_camera/pose 的真实读回。先核有限且近单位四元数，再归一化比较旋转，保留原位置1e-4m与归一化dot1e-8界限及原始回包。公共after_gui可选入口在新客户端创建后执行本课一次总览；已有GUI直接返回，不抢后续学员视角；无GUI跳过，失败只清本次新客户端，整课启动异常仍统一清理。最终增加21项生命周期与隔离模拟检查，ikaros全部64项单元检查通过（0.176s）。

最终正式物理验收一次完成6组、3个fresh私有会话、4次完整观察，耗时143.538s。约50Hz真实帧701/697/912/915，共3225；真实1kHz尾快照501×4，共2004，各六球共同稳定0.500s。基准、高度0.9、周期8及精确修复后观察均通过；实际循环间隔6/6/7.98/7.98s满足预先冻结的±0.08s界。射线最大误差2.3841858e-8m，标记路径最大误差8.8818e-16m。七项错误资源/PNG/非活动脚本/隐藏碰撞平面/stale输入在控制前拒绝；公开复位时钟38.057→38.058，再以实际确认的200步推进至38.258，无时间回退。三会话令牌均无残留、清理错误为空；18个冻结文件前后与两端逐项一致。

正式物理证据为 .deploy/lesson13-scenarios-final.json、同名log/artifacts及 lesson13-formal-compact-final.json；紧凑摘要SHA673ec4555538a62d8ac47ed0b1fdf4b7e660221c97fc7515f9408f8db12b8ecd。纯PNG/LOD/样条/错开静止窗拒绝、配置局部编辑、SDK非零旋转坐标变换和相机回调回归单独标为无仿真，不混计实际观察次数。

调试历史保留：17²版本物理通过，但真实GUI触发OGRE LOD断言；精确Classic固定minBatch17/maxBatch65与OGRE LOD公式要求本配置至少65²，故统一提升原图及两网格，不降低物理门限。65²首图修复崩溃后，实际float四元数norm=1.0000000206588051触发原始dot错误，按旋转表征归一化修正，原真实样本及错误旋转拒绝均保留；地形专用纹理补齐后平台清晰。一次GUI脚本把build及Storage元数据更新误判成输入修改，随后仅修测试为固定12个声明输入前后完整SHA比较、生成物另列。旧删除临时目录不能补取的hash不伪造，原失败报告不覆盖。正常工作台截图另发现远视图，源码存在heightmap在未采用SDF相机时拉至高度+200m的路径；最终after_gui的真实启动记录用于检验自动纠正，不根据截图反推不存在的旧pose记录。

v1/v2/v3源ZIP与原私有、正式和GUI失败记录均保存在Windows .deploy；原先的17²正式报告、65²中间报告与最终报告各用不同名称，不能混称同一次运行。未修改宿主系统、GPU驱动、默认路由、SSH策略或用户gui.ini。后续图形、桌面与发布收尾记录如下。


最终图形取景记录为 .deploy/lesson13-visual-final.json：3个fresh GUI串行，46.719s、9张原始PNG；每次start返回前已存在且仅一条自动overview真实相机记录，实际pre-PUB和post-PUB位置均为[8,-10,9]。12个输入前后逐项SHA相同，生成物另列，3个令牌无残留。先前v4九图95.164s亦完成人工逐图检查：相同相机下日光0.8→0.2使普通表面变暗，自发亮样本仍明亮；关闭三灯cast_shadows后投影消失而暖/蓝光斑保留；木纹、两角度高光、纹理四平台与两个人体动作阶段清晰。自动相机结果不充作像素评判，服务端Light字段不称renderer读回。原始截图保存在docs/images/lesson13-*.png。

正常工作台随后暴露了间歇GUI初始化未就绪：旧截图为远小场景，另有有界真实诊断25s无Pose；单纯私有GZLOG没有修复此问题。调试器改变时序时成功，正常失败现场主线程事件循环正常，'/gazebo'经源码核为Server控制命名空间，并非假world；曝光对照未复现，不能认定缓存损坏、遮挡或命名空间就是已证根因。官方旧[2429](https://github.com/gazebosim/gazebo-classic/issues/2429)与[2053](https://github.com/gazebosim/gazebo-classic/issues/2053)针对自定义world命名的说明也不能直接套用当前default。

最终采用应用可验证的启动恢复：每会话日志/地形缓存放在自己的gazebo-log，保留父环境和原全局文件；仅本课新GUI按token/PID/XID核实后激活，尝试一次尺寸刷新并恢复，窗口管理器忽略resize时如实记false而不把它当硬失败。之后仍必须原生单次相机PUB及真实PoseACK，激活请求本身不代表成功。课程gui_start_attempts=3，旧课默认1；未就绪时只回收该次客户端、保留本课server，取消或server退出即停止。上限耗尽保留日志并走原失败清理。这是有界自恢复，不声称修好了Gazebo上游竞态。

发布入口最后一次真实Qt工作台验收 .deploy/lesson13-ui-release.json **PASS39.119s/13项**：开始按钮6.007s完成GUI真实总览及两个PTY，交互Bash有作业控制；暂停、单步、继续和完整物理验收通过，参考视图不改个人world，学习记录可读，退出令牌[]。Root实际查看最终Gazebo截图，已是清楚总览并有实时统计（非旧远视图），工作台15项目中文内容正常。之前各UI失败报告和首次resize误判均单列保留。最终GUI分支修改后，world/PNG/observer、物理算法与验收门限保持同一配置，此次桌面完整verify再次真实通过；没有无依据地重复全课程物理运行。

公共框架修复已独立提交并推送：[e2471571261fc3f3798b0ebf50be333df6dcdc17](https://github.com/StareamCather/GazeboTeach/commit/e2471571261fc3f3798b0ebf50be333df6dcdc17)，远端哈希已核对。本课原生observer SHA66023a2ad0c0bcfc7cee46ea7f407c4a8f70cb24686747ecdbcfeb9485bda1d0、PNG53981aa62807a641f879c43b5da79549c3a0897d17297464f5e24ed16239497d、world37821455f1cf9f0779e06674a4a5cef2081abc0d2aa677114eadd96142f42f56；GUI helper6fb78481574102e5f8ca6597d472c4afba4a076491fba0670a2adc5fd9cd8f3d，最终instrumentc48c4d7b58d7c01029ceefc468dbb9d1c68833815ba6771cf53868106aa44fc5。完整最终指纹与原始日志保存在两端.deploy，不包含任何私钥。

本次在L13独立提交推送、远端核对和清理后停止，交付范围为13门主课与091/092选做，共15项目。L14不开始；Windows的14–16旧草稿仍未部署、不计交付，无自动续跑或定时任务。用户下次明确发起后从L14审查开始。


## 2026-10-31 L14：碰撞保险杠与真实接触消息

用户明确继续后恢复逐课实施；此前L13停止点为历史。本课沿用系统Noetic gazebo_ros_bumper及gazebo_ros_api_plugin，无额外下载、驱动或框架修改。world.sdf为个人真实输入；prepare只校验固定机构、质量/均匀盒体惯性、释放高度和依赖，再保存输入指纹。参数编辑仅替换质量/惯性或pose的指定数值，保留其余字节、注释和格式，验证临时文件后原子替换，须重载才生效。

运行流程：检查保存输入与准备指纹→连接实际ROS服务并核对物理质量、重力和仿真时钟→只在阶段边界摆放本课puck并清零速度→在台外真实落地后取得持续空接触负对照→中心重新释放→取得新的空中空消息和精确碰撞对→用真实位置/速度/姿态判断落稳，丢弃初期过渡→累计至少20条唯一消息且跨越至少.4仿真秒的支撑窗→finally恢复原暂停状态。方块保留本轮结束状态，不恢复旧运动或时间。ROS等待和单次调用均有墙钟上限，控制请求不重发。

接触语义按Classic11.15.1/Noetic源码审查：每条ContactState的total_wrench已逐分量加总同一接触记录的点反馈；50Hz传感器会在一包states中追加多个原生物理时刻，故先对每条记录的力取模，再按每条ROS消息求平均，不能跨states相加为同一时刻的合力。ODE先把body1反馈转到collision1连杆轴，bumper未执行TF转换，frameName只写消息头；本课固定轴对齐并检查实际姿态，才与mg比较。位置/法线与wrench不能笼统称为同一world坐标。未声称连续无遗漏1kHz轨迹，也未把已采样冲击最大值当实体材料峰值。

探索测试lesson14-trial的8组行为均达到预期，但运行期间操作端更新了参数编辑器/教材，source_templates_unchanged=false，因此总体明确记录失败，不计正式通过。随后冻结全部9个课文件重新部署并完整重跑，没有靠降低判据获取通过。

正式tests/lesson14_acceptance.py报告.deploy/lesson14-formal.json：PASS，47.730s，3个全新私有ROS/Gazebo会话、8组场景（默认、只改质量、只改高度、保存未重载拒绝、错误传感器绑定拒绝、错误frame标签拒绝、错误惯性拒绝、精确恢复后完整再验）。四次正向完整观测共425条真实唯一header消息；稳定窗共87条ROS消息、1739条原生接触记录，三者分开计数。独立验收从保存原始数据重算逐消息支持力，核对完整碰撞名、真实几何点、唯一递增时钟、空中负例、台外实际落地负例与稳定窗；不是只信退出码。

质量1kg、落高1.4m：平均支持力9.7119N，中心z=.299999999996m，下落.483s；质量2kg且其余不变：19.4238N、下落.484s，支持力比2.0。只把落高改为2.2m：下落.642s，静止支持力比约1.0。稳态消息频率50Hz。固定窗mean/median须在mg的15%内，每包均值在35%内；本机观测约为mg的99%，不承诺所有引擎/接触参数都精确相等。错误输入在任何操控之前拒绝，暂停世界的iterations不变；原文件精确恢复后重新完成整轮真实观测。所有正向/拒绝场景保留原暂停状态，个人输入字节未变。

真实桌面tests/ui_probe.py --lesson 14：PASS，19.622s、13项检查，应用选择/初始化、独立ROS/Gazebo、两真实PTY与Bash任务控制、暂停/单步/继续、完整课程验收、只读参考不改world、学习记录、应用/Gazebo截图及关闭清理通过。主代理检查docs/images/lesson14-ui.png和lesson14-gazebo.png：中文教材与已验收状态可见，蓝色台面和红色方块正确落稳，真实渲染有实时统计。此图形检查不替代学员手工echo/释放/事件计数能力验收。

实际ikaros全部64项框架单元检查.188s通过、无跳过；Windows64项1.255s通过（2项Linux专用跳过）。正式运行前后9个课文件SHA一致；3个物理会话与桌面会话令牌归属进程均清理，无cleanup错误。完整原始JSON/日志在两端.deploy/lesson14-formal-artifacts与对应根报告，发布只提交课程、验收程序、2张原始截图及文档，不提交运行缓存。原VALIDATION_LOG前60239字节保留不变。完成本课独立发布并核对GitHubmain后才进入L15；旧15–16草稿仍未部署。

官方接口依据：[Classic接触教程](https://classic.gazebosim.org/tutorials?tut=contact_sensor)、[Noetic bumper源码](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_bumper.cpp)、[Classic11.15.1 ContactSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/ContactSensor.cc)、[ODE反馈](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODEPhysics.cc)。


## 2026-10-31 L15：CPU/GPU激光扫描与真实几何对照

场景提供同位CPU ray与GPU ray，默认181束/180°/20Hz、range .1..10m；低置扫描器外壳避开自身光线，固定墙中心x3m、厚.2m，另设collision-only方块(1.5,−1.5,.7)和visual-only方块(1.5,+1.5,.7)。个人SDF真实参与仿真；只开放墙中心2.5..4m及两路同步181/361束，局部数值编辑保留其它字节/注释/换行，验证后原子替换并要求重载。带CRLF和伪模型注释的编辑回归通过。

首轮真实运行明确失败，不能算通过：两个系统Noetic激光插件一起加载时，GPU触发RayPlugin requires a Ray Sensor as its parent，ready超时退出；源文件不变、归属清理干净。完整报告.deploy/lesson15-formal.json与原始runtime保留；Windows另存v1-failed报告和9文件源码归档lesson15-v1-stock-conflict-source.zip（SHA256 b57217c170af063292562062ab005f468a6085f360202588e47a77d52b572dfa）。检查系统两库的nm输出均导出同名gazebo::GazeboRosLaser::Load和vtable；官方issue768描述相同CPU/GPU类名冲突与改名修复。没有把该异常当作显卡故障，不改系统库、驱动或路由。

修复流程：保留系统CPU插件；GPU改为课程独有GazeboTeachL15GpuLaser SensorPlugin，验证parent真实GpuRaySensor，按本世界命名空间订阅原生LaserScanStamped，仅复制原生时间、角度、量程、ranges和intensities到ROS，time_increment/scan_time置0沿用该接口约定、frame只标注不发布TF。桥没有几何预测、生成数据或测距替换。回调只捕获sharedState、不捕获this；析构以互斥inactive屏障等待正在发布的回调，再取消订阅/关闭本Node和Publisher，迟到回调可安全退出，不调用全局ROSshutdown。prepare使用本机Gazebo11/roscpp/sensor_msgs SDK离线编译，仅用户缓存与练习build/lib软链接；缓存键含源码/SDK版本/flags，并记录源码及实际库SHA。首次真实编译5.089s通过：source fd2bcb9da72b3c6289c6e39a516e63ba2f808f82bda0f6cc2a6edf0f13126aa3，library 0cb8783f77fe0712db0b336e7803873f4562aaa86f55a071b1d462d8d4dccca6。未修改框架。

完整验收先检查个人SDF与准备指纹、真实两路发布者和元数据；每阶段暂停并设置/读回真实墙体，清队列，恢复推进，先留.2仿真秒传播保护，再取得首个正确几何帧，随后窗口内数据均须正确。基线→移墙1m→恢复配置位置，每路每阶段至少12个唯一时间戳且跨越≥.5仿真秒、频率15..25Hz，检查−90/−60/−45/−30/0/+30/+45/+60/+90°的数组索引和真实距离；支持有限墙宽下的无命中+inf。finally再恢复进入时实际墙体位姿、速度与暂停状态，不能把配置位置误当用户进入时位置。ROS调用有墙钟上限且控制不重发。原始非有限数保留为明确字符串，JSON不混入非标准Infinity/NaN数字。

修复后冻结全部9个课文件，tests/lesson15_acceptance.py正式.deploy/lesson15-v2.json PASS：67.893s，3个全新私有ROS/Gazebo图形场景，7组（默认181束、只改361束、只改墙距3.5m、保存未重载拒绝、错误frame拒绝、非合同noise拒绝、原字节恢复完整再验）。四轮正向观测有24个双路窗口、362条原始扫描、289条验收扫描；逐路唯一header递增、跨越时间、九束几何均由根验收独立重算，不只信退出码。频率20..20.0364Hz，窗口最后帧九束最大误差.0177445m，原合同.08m未修改。

默认中心CPU2.9000001m、GPU2.9001961m，真实墙体平移1m后3.9000001/3.9002643m，恢复后回到原值。−45°CPU测到collision-only方块1.8384776m、GPU看到后墙4.1032848m；+45°CPU看到后墙4.1012192m、GPU测到visual-only方块1.8393991m，分别证明两类几何来源。361束只改变数组和角分辨率；墙中心3.5m时中心距3.4m→移动后4.4m，远侧±45°超过墙宽后按原生返回+inf。所有正向场景实际墙状态恢复误差0，暂停状态恢复；所有输入拒绝发生在操控前，暂停世界iterations未动；9个课文件SHA前后相同、归属进程无残留。

真实桌面tests/ui_probe.py --lesson15：PASS21.589s、13项，包含环境选择、初始化/SDK缓存、真实Gazebo和两PTY、暂停/单步/继续、完整扫描验收、只读参考、学习记录与关闭清理。根代理查看原始lesson15-ui.png及lesson15-gazebo.png：中文课件、验收状态、扫描器、可见方块、墙和射线叠加显示正确；此截图仅证明静态可视结果，不替代消息验收，也不声称测量窗口内逐GUI帧都经过人工检查。实际GPU数据在真实DISPLAY且每物理场景启用gzclient条件下验证。完整JSON/原始扫描/日志在两端.deploy/lesson15-v2*，v1失败与修复后证据分开。原VALIDATION_LOG前65157字节保留不变。

教材按当前实现明确：CPU看collision、GPU看render；frame标签不等于TF；scan_time/time_increment为0，频率从真实header差求；range/resolution=.01不保证厘米距离量化；GPU对NaN/量程端点和噪声clamp的处理不能简单套用所有后端。当前固定无噪声课程没有上线噪声统计/强度材质标定，相关先修和拓展不计本次已验证。发布及远端哈希核对完成后才进入L16，未提前部署旧IMU草稿。

依据：[原插件冲突issue768](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/768)、[Noetic CPU激光](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_laser.cpp)、[Noetic GPU激光](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_gpu_laser.cpp)、[Classic RaySensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/RaySensor.cc)、[Classic GpuRaySensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/GpuRaySensor.cc)。


## 2026-10-31 L16：IMU转台、坐标与真实自由落体

固定场景使用两套原生ImuSensor与系统Noetic gazebo_ros_imu_sensor：均匀盒体转台.6×.25×.12m/1kg，碰撞与外观一致、三主惯性按均匀盒体计算；另有.2m立方体freefall_probe/1kg。两IMU均在CoM且零偏置、原生100Hz、零噪声、initialOrientationAsReference=false，显式线/角阻尼0，动态非kinematic。转台关闭模型重力维持悬浮，探针保留重力真实下落；没有持续设置轨迹、添加假加速度或替换消息。

实际接口依据：原生gyro为R^-1ωworld，比力为R^-1(Δv/Δt−g)，即使link.gravity=false仍减世界重力；因此悬浮a≈0时正立f≈+g，自由落体a≈g时f≈0。Noetic的false参考标志显式设世界到参考旋转为identity，姿态来自仿真状态，不是陀螺积分/融合；frameName仅写header，xyzOffset在该插件未用，rpyOffset只调整姿态而不一起旋转gyro/acc。固定offset0避免混合轴。noise0导致三协方差数组全0，按Imu消息语义为unknown，不作确定性零误差承诺。

两级频率配置：原生100Hz，ROS wrapper updateRateHZ200仅为上限，不插值或生成额外样本。wrapper按Sensor.LastUpdateTime的整数common::Time差限速，200上限让约10ms原生间隔通过、dt0重复不通过；不宣称已发现通用浮点门限缺陷。header为原生Sensor.LastUpdateTime，底层LinkData测量可落后少量tick，独立查询姿态必须保留前后时钟界，不冒充严格同刻真值。

修改后的完整流程：校验固定SDF和仅speed_rad_s/tilt_deg参数→核对prepare指纹、实际质量/重力标志、发布者和仿真时钟→一次摆放转台/设置零初速或给定±ω→ODE演化→.2仿真秒保护后取得≥50唯一header且跨越≥.6秒→检查正立静止、roll倾斜静止、正转、反转；每阶段同时取得独立GetLinkState位置/朝向/速度和ROS时钟前后界。第五阶段把重力探针一次零速释放到z3m，.15秒保护后取得≥30唯一header/≥.3秒且在释放后.65秒前结束，验证f≈0、实际下降/负vz，并保留同期悬浮f≈+g消息。最后逐模型独立尝试恢复进入时实际pose/twist，汇总失败后统一报告，finally另恢复原暂停；q/−q按等价旋转比较。恢复瞬间状态不恢复仿真时间或传感器差分历史。等待和单次ROS调用均有限墙钟时间，控制不重发。

参数仅允许速度.2..1.5rad/s与倾角10..60°，分别由lab config局部替换数值，保留其它JSON字节/格式；保存后须重载。手工turntable命令可正立±转速或倾斜静止，不把倾斜+非主轴旋转当作当前常速场景。prepare只校验/记录，不重写个人输入或偷偷补参数。静态审查先发现并修复了首个模型恢复失败会跳过第二模型的错误路径，修复后冻结再开始实测，没有用运行时阈值放宽掩盖失败。

正式tests/lesson16_acceptance.py报告.deploy/lesson16-formal.json PASS：61.467s，3个全新私有ROS/Gazebo场景、8组（默认.6rad/s/30°、只改.9rad/s、只改45°、保存未重载拒绝、错误frame拒绝、错误noise拒绝、错误reference拒绝、精确恢复后完整再验）。四轮正向运行含20个五阶段物理窗口，2882条实际原始双路消息、其中1122条验收消息；保护段/并行对照与验收样本分开，不声称所有原始消息都应符合稳态判据。根验收独立从保存数据重算gyro、比力、四元数归一化/静态倾角/连续展开yaw、唯一递增时钟、100Hz频率和自由落体位置速度变化，不只检查进程退出。

20个窗口唯一频率均100Hz。默认倾斜30°比力[0,4.905,8.4957092]m/s²；只改45°得到[0,6.9367175,6.9367175]。正/反gyro实际±.6rad/s，.61秒yaw变化±.365999989rad对预测±.366；速度单变量.9rad/s时正向.61秒变化.548999963rad对预测.549。四次自由落体平均fZ约8.15..8.19e−14m/s²，独立查询速度变化重估世界加速度约−9.81..−9.832m/s²；同时悬浮体fZ9.81。实际返回的近时服务区间/姿态比较保留，未把服务往返时间视为精确采样时刻。

每浮台窗口生产判据gyro分量误差<.02rad/s、比力<.05m/s²、静止姿态角误差<.005rad、四元数范数误差≤1e−6、展开yaw误差<.02rad、名义频率80..120Hz；独立根检查另要求90..110Hz。自由落体真实位置/速度还需符合3−g*t²/2和−g*t的有界查询误差，必须在撞地前测量。所有输入拒绝均在控制前发生，暂停世界iterations不变；恢复后的精确字节重新完成五阶段。两模型恢复位置/速度最大分量误差及旋转角误差均0，原暂停恢复；10个课文件SHA前后相同，三个会话归属进程无残留。

真实桌面tests/ui_probe.py --lesson16 PASS21.443s、13项：环境/课程、真实Gazebo/两PTY、暂停/单步/继续、五阶段完整验收、只读参考、学习记录和关闭清理。根代理检查lesson16-ui.png与lesson16-gazebo.png，中文正文和已验收状态正确、绿色悬浮模型与落地红色探针清晰，真实图形会话显示正常。截图不替代学员手工读IMU、理解四元数或编写姿态估计；噪声统计/融合/实体标定为背景与后续拓展，不算本课已验证。完整报告、原始消息、查询记录和日志保存在两端.deploy/lesson16-formal*及lesson16-ui*。本课复用现有依赖，不修改框架/系统，原VALIDATION_LOG前71112字节保留不变。发布核对后才进入L17。

依据：[Classic11.15.1 ImuSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/ImuSensor.cc)、[Noetic IMU插件](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp)、[Imu消息定义](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)、[REP103](https://www.ros.org/reps/rep-0103.html)、[REP145](https://www.ros.org/reps/rep-0145.html)。


## 2026-10-31 L17：RGB相机、真实像素与针孔投影

系统Noetic gazebo_ros_camera与Classic原生相机在实际DISPLAY渲染：相机(0,0,1)朝世界+X，默认320×240/R8G8B8、名义HFOV60°/20Hz，wrapper updateRate0沿用原生帧。红色目标.04×.6×.6m在(3,0,1)，固定绿/蓝参照板使用纯色emissive，避免把照明变化误作成像几何；无外部模型/纹理下载。仅开放320×240/640×480与HFOV60/75的单变量，个人SDF为真实输入；局部数值编辑保留其它字节/注释，prepare只验证依赖、固定结构与指纹，保存后须重载。

初次运行明确失败，不计通过：ready严格K检查拒绝，4.721s后清理，无残留。原始报告.deploy/lesson17-formal.json保留；9文件v1源码归档SHA42a08f7ec6d5aab7d71e4b0bec97f4be22963d0414bcf8b70aedf3265e930726。只读诊断临时跳过课程ready以读取真实未验证消息（不是验收）：实际K fx=fy277.1273455955935、cx160.5、cy120.5，P同焦距，Image rgb8/step960/230400字节，Image与Info实际stamp相同。未先改相机或降低阈值。

根因已由源码链和实测精确确认：CameraSensor把cameraSdf交给rendering::Camera::Load，后者读取角度并调用SetHFOV；Camera.cc819–822写回sdf参数，sdformat9 Param::Set使用默认std::stringstream六位有效数字，HFOV()再从该参数读回。名义π/3变1.0472rad，所以实际f=320/(2tan(1.0472/2))=277.1273455955935；名义75°变1.309rad，对应208.51539887168917。v2只修正验证预期的已确认SDK读回流程，保留原rel1e−7/abs1e−6判据，主点/D/R/P结构不放宽；不改实际K/P、SDF名义值或图像，像素投影继续使用实际发布K。intrinsics_check保存理想/有效HFOV、理想/预期/实际焦距及误差，失败诊断输出actual/expected/maxerror。独立根验收按相同公开setter事实重建焦距，并额外要求矩阵各项有限。

实现流程：订阅实际Image与CameraInfo（仅订阅Info可能受图像连接门控无输出）→用完全相同header时间戳配对，独立于到达顺序→校验尺寸/RGB8/endian0/step3W/数据长度及K/P/R/D/帧名→一次移动并读回真实红色目标→.2仿真秒传播保护/首个正确成像→窗口后续每帧严格验证→基线(3,0)、远移(4,0)、横移(3,+.5)、恢复(3,0)各取得≥12对/≥.5仿真秒/15..25Hz。真实RGB按色彩掩膜统计bbox/centroid/count，与厚板全部八角点投影比较，不能把离轴侧面简化为无限薄正面。结束恢复进入时目标pose/twist和暂停，q/−q视为等价旋转；控制不重发、等待有墙钟上限。

原始帧按内容SHA去重落盘.rgb（字节不改），每pair仍有独立真实时间戳/元数据/测量；PNG直接用实际RGB字节解码，不按公式画目标或改色。自有原生Qt实时查看器只显示实际配对图像，可限时运行/保存原图，QImage在GUI线程复制原始数据、只操作本进程窗口；无需先安装视觉算法或打开网站。

修复后冻结全部8个课文件，tests/lesson17_acceptance.py正式.deploy/lesson17-v2.json PASS81.328s：3个全新私有ROS/Gazebo图形场景、8组（默认、只改640×480、只改75°、保存未重载拒绝、错误frame拒绝、错误format拒绝、错误distortion拒绝、精确恢复后完整再验）。四轮正向共16个成像窗口、244对原始配对消息，其中192对验收；频率19.9637..21.9561Hz，名义20Hz不等于每个有限窗口都恰20。根验收逐原始文件核对3WH与SHA，独立保守红色掩膜、八角点投影、内参和光学轴方向重新验证，不只信辅助程序输出。台上真实Qt查看器限时3秒显示53帧，并保存首个原生图像。

默认红目标实际bbox[132,92,187,147]、3136个强红像素；移远1m得到[139,99,180,140]、1764像素，面积比.5625。向世界+Y移动.5m，bbox左边132→86，对应光学+X向右的负方向；恢复后原像素数/位置恢复。640×480基线bbox宽112px（默认56px）、强红像素12246，接近原面积四倍但栅格/边缘不能强求精确4；75°基线1764像素，视场变宽时目标变小。生产像素bbox中心容差2.5px、宽高3px、色块质心3px，保留legacyCameraInfo主点与像素索引中心约1px约定差；没有改实际metadata迎合像素。目标恢复位置/速度误差及旋转角误差均0，原暂停恢复；所有错误输入操控前拒绝、暂停iterations未动；8课文件SHA前后一致、3场景归属进程清理干净。

实际桌面tests/ui_probe.py --lesson17 PASS22.337s/13项：课程/环境、真实Gazebo与两PTY、暂停/单步/继续、完整成像验收、只读参考、学习记录及关闭清理。根代理检查应用/Gazebo原始截图，以及相机基线、远移、横移原图，确认三色参照、远移缩小/向左平移与独立场景视图一致。docs/images/lesson17-camera.png是984字节真实PNG，非空图判断以实际像素/尺寸为准，不以压缩文件大小推断信息真实性。原始报告、所有RGB/PNG、真实元数据诊断及v1/v2证据在两端.deploy/lesson17-*，源码仅发布课程、测试、三张原始截图和文档。原VALIDATION_LOG前77147字节保留不变。

教材还明确：Gazebo机体+X前/+Y左/+Z上与光学+Z前/+X右/+Y下的变换；frame标签不自动发布TF；单目像素不提供无尺度深度；K/P/R/D、RGB/BGR、step与8位数据endian分别解释。当前固定零畸变/无噪声场景未验证畸变标定、曝光/真实光学噪声或目标检测算法，它们为带先修链接的拓展。现有NumPy1.24.4/Qt直接复用，无框架或系统修改，完成本课发布核对后进入L18。

依据：[Noetic CameraUtils](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_camera_utils.cpp)、[Classic Camera SetHFOV](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/rendering/Camera.cc#L819)、[sdformat9 Param Set](https://github.com/gazebosim/sdformat/blob/sdf9/include/sdf/Param.hh#L305)、[Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)、[CameraInfo](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html)、[REP103](https://www.ros.org/reps/rep-0103.html)。


## 2026-10-31 L18：真实深度、无效值与有序点云

实际DepthCamera与系统libgazebo_ros_openni_kinect.so输出32FC1米制深度、CameraInfo、PointCloud2及独立RGB。原生output固定depths（误改points会切到不同原生回调），HFOV60°沿L17已确认的六位有效数字写回与实际内参校验。设计阶段即选10Hz以控制640×480、32字节/点的约9.8MB/帧传输成本，不是测试失败后降频；wrapper不插值。世界原生clip.1..8m，软件严格.4<Z<max（4或6）两层分开。真实前景Z1.9m、墙面2.95m、绿色近探针.24m、蓝色远探针4.95m与上角无回波；所有资源离线。

算法流程：校验真实个人SDF和prepare指纹→同时订阅depth/DepthInfo/cloud及RGB并设置有限大消息缓冲→按完全相同header配对前三路，不依赖到达顺序→按encoding/endian/step/fields/point_step/row_step解码实际原始字节→一次移动并读回真实墙体→.2仿真秒传播保护/首个正确几何帧→基线、远移1m、恢复各取得≥12唯一三路组合、≥1仿真秒、8..12Hz。整图深度有效掩膜与cloudXYZ一致，Z逐值对应，helper按其实际主点反投影；六个固定采样点再检查几何、NaN和轴向/径向区别。每阶段另外保留保护期之后实际RGB帧，证明近绿/远蓝可见；该帧不要求与三路同header，也不承诺cloud颜色缓存属于同曝光。finally恢复进入时墙体pose/twist和暂停，q/−q等价，控制不重发、调用有墙钟上限。

有序点云的实际布局读取而非假定紧密排列：本机FLOAT32/count1字段x/y/z/rgb偏移0/4/8/16，point_step32，默认row_step10240、640宽时20480。32FC1有效值单位m，超软件范围或无回波置NaN，cloudXYZ同样NaN、is_dense=false；不能把NaN当0m或自由空间。cameraInfo主点(W+1)/2,(H+1)/2，而原生wrapper helper用(W−1)/2,(H−1)/2；分别重建后对照其XY差Z/f，保留实际K和原始cloud，不通过改字段制造一致。depth_map shader输出clip.w，实际正面墙两离轴点Z相等、cloud欧氏范数增长，直接检验轴向深度而非径向range。

正式tests/lesson18_acceptance.py .deploy/lesson18-formal.json PASS86.299s，3个全新私有ROS/Gazebo图形场景/8组（默认320max6、只改640×480、只改max4、保存未重载拒绝、错误frame拒绝、错误16bit输出标志拒绝、错误distortion拒绝、精确恢复完整再验）。四轮正向12个物理窗口、162个原始三路组合，其中144个验收组合；频率10..10.0091Hz。根验收独立读全幅原始.f32/.pc2，按实际字段步长解码、验证SHA与长度、全部有限值严格.4<Z<max、整图NaN掩膜/深度Z一致及独立XYZ公式，不把摘要当原始证据。原始RGB/cloud/depth按内容SHA去重，重复内容仍保留不同真实header；.rgb、.pc2与.f32文件均已Windows读回并校验文件名SHA。

实际测量前景Z1.90000081m，墙面两个离轴点均2.95000076m；墙体移1m后均3.95000100m，前景保持。右侧点cloud范数3.12373908m大于其Z2.95000076，不能把两种距离混称。绿色近探针RGB真实可见但软件depth/XYZ为NaN；蓝色远探针在max6时Z4.95000076m，在仅改max4后NaN；原生clip和真实几何均未改。上角无回波为NaN。独立helperXY全图最大误差1.1921e−7m，墙面Z2.95时CameraInfo与helper反投影XY差约.0106449m（一个像素对应的Z/f）；640宽约减半。墙体恢复位置/速度和旋转误差均0，原暂停恢复。全部错误输入在控制前拒绝，暂停iterations不动；8个课文件SHA前后一致，归属进程清理无残留。

本机构建还真实暴露stockOpenni颜色布局差异：红色前景RGB字段原字节[255,0,0,0]，按常规little-endian packedRGB解为[0,0,255]；蓝色探针[0,0,255,0]相反。源码helper直接从缓存RGB写iter_rgb[0..2]，与常规packed位解释并不一致。程序完整记录原字段字节与常规解码，未改系统插件或暗中交换数据；不把这个rgb FLOAT32位载体当数值强度，不承诺真彩/曝光同步。几何验收基于XYZ/depth与独立RGB可见性，教材已预先说明该实现边界，具体本机观测在此留证。

Qt真实深度查看器限时3秒显示30帧、保存伪彩图；颜色由真实标量归一化映射，NaN显示紫色，颜色不是原始测量、修改显示范围也会改颜色。真实桌面tests/ui_probe.py --lesson18 PASS23.250s/13项：环境/课程、实际Gazebo/两PTY、暂停/单步/继续、完整深度验收、只读参考、学习记录及关闭清理。根代理查看默认深度伪彩、同场景实际RGB、max4伪彩、应用和Gazebo原图，确认场景与无效区域变化。源码发布四张原始截图（深度图明确命名pseudocolor），完整原始浮点/点云/实际RGB/日志在两端.deploy/lesson18-formal*及ui*，不以PNG代替浮点数据。

当前未声称完成实体RGBD标定、硬件无效值/噪声建模、真实RGB与cloud硬同步、标准真彩修复或三维重建；这些作为专业背景/后续拓展。复用现有SDK/NumPy/Qt，不改框架或系统；原VALIDATION_LOG前83430字节保留不变，完成提交推送与远端核对后再进入L19。

依据：[Noetic OpenniKinect](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_openni_kinect.cpp)、[Classic depth vertex shader](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/media/materials/programs/depth_map.vert)、[depth fragment shader](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/media/materials/programs/depth_map.frag)、[PointCloud2定义](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)、[REP118](https://www.ros.org/reps/rep-0118.html)。

发布前格式检查发现lab.py末尾多一个空行，已仅清理末尾换行；正式运行版62a1cb2efbbb6b74e7962856ed39a9fdef4c4ee9dfbb4c079deb74281d64687f与发布版aa157b90629cf750f8e577ba1917a415aa4f515fa795f1309f249f633909f165的Python AST逐节点完全相同，解码、裁剪、反投影、显示及恢复算法流程不变。保留正式运行原文件与格式变更记录，重新语法检查和Git空白检查通过，未把无关格式变化重复算作新的物理观测。


## 2026-10-31 L19：六维力矩、重量与力臂

真实动态load_link由world固定关节支撑，link原点与jointanchor同在世界(0,0,1.2)，均匀.2m立方体1kg、CoM位于link +X .3m。惯性关于CoM按m*.2²/6给出，碰撞与载荷外观同CoM；细连接杆仅可视、不额外伪装质量。显式重力开启/阻尼0、非static非kinematic，不用持续设置位姿或施加“预测力”。关节physics/provide_feedback=true，真实ODE约束承担重力与偏心力矩。

同一关节四路观测：A原生ForceTorqueSensor framechild/child_to_parent；B同轴parent_to_child，仅反测量方向；C sensorframe绕X+90°/child_to_parent；D系统libgazebo_ros_ft_sensor.so，是ModelPlugin直接GetForceTorque(0).body2，不是A的ROS包装。A/B/C通过唯一名SensorPlugin仅复制原生WrenchStamped的6个分量与仿真时间，ROSframeName只标记轴、不发布TF；D自动用childlink帧，不读取其它插件的frameName/measure_direction参数。桥回调只capture共享state，mutex/inactive屏障、取消订阅和本Publisher关闭保证卸载，无全局ROSshutdown或计算/写入wrench。prepare以既有Gazebo11/roscpp/geometry_msgs SDK编译私有缓存，记录源码、库和构建指纹，不改系统库或框架。

力矩参考点按源码与真实结果区分：ODEJoint先把CG反馈平移到jointanchor再旋入link轴、取输出侧符号；所以child_to_parent是负载对支座的力Fz≈−mg、τy≈+m*g*arm，不能误称支座对负载的向上力。原生sensorpose的旋转只换轴，位置不把力矩测点平移离开jointanchor；改变表达轴与改变力矩参考点是两种运算。力矩单位N·m，不是角度，六维wrench也不是接触压力图。

运行流程：检查真实SDF和质量/CoM/均匀块惯性/碰撞/载荷外观/细杆长度的联动→核对prepare指纹、实际质量/重力/CoM/惯性→读取GetWorldProperties.sim_time作实际guard起点（不依赖刚启动且世界暂停时可能尚未收到的ROSclock）→必要时继续世界→.2仿真秒保护后，四路各取得≥30唯一header/≥.6秒/40..60Hz、共同覆盖≥.5秒→逐帧及均值检查6个分量、Native与stock稳态比较，并取得≥3个实际静止link状态→finally仅恢复原暂停。机构不被重新摆放、施加外力或强制轨迹；四路采样路径不同，不虚构严格同刻header。

只允许质量.5..3kg、力臂0.15..0.6m，经局部参数编辑联动惯性与物理/可视几何，保持其它字节/注释，保存须重载。feedback故障只用课程预检拒绝：原生ForceTorqueSensor.Init会自动SetProvideFeedback(true)，同joint存在native时不能把删该字段宣传成必然无信号；stockROS路径本身不会代为开启。

正式tests/lesson19_acceptance.py .deploy/lesson19-formal.json PASS54.654s，3个全新私有ROS/Gazebo场景/8组（默认m1/arm.3、只改m2、只改arm.5、保存未重载拒绝、缺feedback合同拒绝、错误方向拒绝、错误均匀惯性拒绝、精确恢复完整再验）。四轮正向16个通道窗口、661条原始消息，其中501条验收，实际唯一频率均50Hz；根验收从原始数据独立重算B=−A、C=Rx(−90°)A、D≈A、−Fz/g≈m与−Ty/Fz≈arm及跨配置比例，不只信expected字段或进程退出码。

实际基线A/D F=[0,0,−9.81]N、τ=[0,2.943,0]N·m；B全反号；C F≈[0,−9.81,0]、τ≈[0,0,−2.943]，零轴仅约1e−15浮点残差。只改m2得到Fz−19.62、τy5.886，二者加倍；只改arm.5仍Fz−9.81、τy4.905，力不变而力矩为5/3。独立反算质量1/2/1kg、力臂.3/.3/.5m正确；实际CoM/惯性/重力与每次输入一致，link位置(0,0,1.2)、姿态和速度保持静止。原暂停恢复，个人输入字节不变；错误源在任何继续世界前拒绝，暂停iterations不动；9个课文件SHA正式运行前后一致，归属进程清理无残留。

生产验收每帧F分量误差≤.05N、τ分量≤.03N·m、nativeA/stock均值差≤.03，另检查实际静止位姿/速度与归一四元数；没有把数学期望灌入传感器。真实桌面tests/ui_probe.py --lesson19 PASS18.371s/13项：环境/课程、SDK缓存启动、实际Gazebo/两PTY、暂停/单步/继续、完整四路验收、只读参考、学习记录与关闭清理。根代理查看原始应用/Gazebo图，支柱、偏心质量、细连接杆和轴标记清楚，中文教材及已验收状态正确。图形检查不代表学员掌握手工wrench坐标转换或测量校准。

完整原始报告/各路时间戳/实际物性与link状态/日志保存在两端.deploy/lesson19-formal*及ui*。当前固定无噪声静载不等于硬件应变、六轴标定、温漂或动态载荷辨识；相关先修/挑战仅作背景，未计实测功能。无新网络依赖，原VALIDATION_LOG前89727字节保留不变，完成本课提交推送与远端核对后进入L20。

依据：[Classic ForceTorqueSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/ForceTorqueSensor.cc)、[ODEJoint反馈](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODEJoint.cc)、[Noetic GazeboRosFT](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_ft_sensor.cpp)、[WrenchStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/WrenchStamped.html)。


## 2026-10-31 L20：五站原生传感器与测量语义

上线范围以真实运行确认：已安装Classic11.15.1的GpsSensor、MagnetometerSensor、AltimeterSensor、SonarSensor与LogicalCameraSensor全部在最小场景产生消息，再进入整课验收。最小探针14.856s通过，运行5秒每路50条，独立Master/partition与进程归属清理无残留；仅作接口就绪，不把它替代物理行为验收。使用现有SDK、ROS和Qt，无新网络依赖，不改框架或系统。

唯一名SensorPlugin仅订阅原生protobuf并逐字段转为std_msgs/String JSON，保留native_type/native_topic/native完整字段与原封包wire_json。GPS生产者没有Topic override，按实际parent/name/data后缀路由；Mag使用GetTopic，Alt实际生产者用Topic而非其旧GetTopic，个人SDF明确完整topic；Sonar与Logical用各自Topic。每个传感器实际工厂与protobuf类型在接收端核对，不靠头文件存在、类名或订阅成功宣称有数据。桥不生成位置/磁场/距离/模型名单，也不伪造GNSS fix状态或协方差。

世界配置WGS84原点纬经0°/0°、椭球高度100m、heading180°。当前Classic SphericalCoordinates在heading0的LOCAL→GLOBAL存在XY反向约定，显式180使世界+X/+Y对应east/north；磁场另由world磁矢量给出，并不随地理heading自动绑定。导航台初始worldZ1m，GPS初始altitude101m；Altimeter初始vertical_reference1m且vertical_position0。Mag世界场[20,0,−40]µT，消息单位T并按传感器姿态Rᵀ旋入本体轴。初始高度与磁场scale分别只允许1/2，局部参数编辑保留其余字节/注释并要求重载；前者改变GPS绝对高度和Alt加载参考，后者改变实际world磁场，不在桥中乘测量值。

代码流程：校验个人真实SDF/固定工厂、topic、噪声、几何和prepare指纹→连接五路真实原生发布者→暂停并保存四个模型原pose/twist→逐阶段一次初始化并读回nav/sonar/obstacle/target实际状态→读实际世界sim_time作epoch→继续真实ODE→传播保护后采集新事件和导航状态→最后逐模型独立尝试恢复、读回确认，再恢复进入时暂停。导航台dynamic、gravityfalse、阻尼0、关闭自动休眠；vz=.25阶段只设置一次初速，之后真实自由演化，不持续写位姿或伪造理想读数。恢复不倒放仿真时间或传感器内部历史。

九阶段是baseline、nav_up(+1m)、nav_east_north(10,5)、nav_yaw90、nav_velocity(vz+.25m/s)、sonar_obstacle、sonar_out_of_range、logical_outside与logical_restored，每阶段其余状态重建为基准。通常guard.3仿真秒，Sonar超量程guard.8秒以覆盖7次无接触更新后才回max的旧值保持；原始guard/acquiring事件保留，首次符合预期后窗口内每条必须继续符合。五路各≥12事件/≥1仿真秒/8..12Hz。GPS/Mag/Alt/Sonar原生time原样使用并按stamp去重；LogicalCameraImage原生无time，source_stamp_ns为null，只以严格递增bridge_sequence识别消息、以非递减receipt_world_time_ns统计接收跨度/到达率，允许不同事件同一接收tick，不宣称原生采样时刻或采样频率。各路不虚构硬同步。

正式tests/lesson20_acceptance.py .deploy/lesson20-formal.json PASS141.204s：3个全新私有ROS/Gazebo场景/8组（默认、只改磁场scale2、只改初始高度2、保存未重载拒绝、错误heading拒绝、错误GPS topic拒绝、错误Sonar geometry拒绝、精确恢复完整再验）。四轮正向共180个通道窗口、2800条保留原始事件，其中2160条验收事件；四原生唯一时间频率9.990917..10Hz，Logical桥接收率9.990917..10.009099Hz，后者不称原生频率。独立根验收重新解析wire，逐项对照桥metadata及native字段/时间、有限数值、真实4台初始化和导航状态；GPS经WGS84 ECEF→ENU独立反算回世界坐标，不仅信课程expected字段；另核对磁矢量逆yaw、Alt参考/位移/速度、Sonar几何/范围、Logical实际目标进出与相对位姿。

实际GPS/Alt同时测得vz=.25m/s，原始位置随仿真时间的斜率和实际GetModelState速度/轨迹均吻合；并非仅瞬间传送后声称速度。高度2变式GPS初始102m、Alt reference2m而相对高度仍0；上移1m时两种高度都增加1m，参考不被传送重置。Mag yaw+90°后约[0,−20,−40]µT，磁场scale2时各分量加倍。误差门限保留GPS角1e−8deg/高度.002m、Alt参考1e−9m/位置.002m、速度.005m/s、磁场1e−9T；根验收另用真实状态和独立反算，不修改原始测量。

Sonar局部−Z cone/min0/max3/radius.2m，真实floor读数1.505327420222566m；.6m高障碍移动到其下方后.9019807406425385m；传感器移到worldZ4后读数3m。固定近似几何容差.06m，并非通过把原始值改为理想1.5/.9来过验收。源码range为碰撞点相对sensor向量范数减penetration depth；消息contact是转入sensor局部、未经该depth修正的原始碰撞几何位置，不保证真实表面或声学命中点，且消息没有depth可供单独重算range。实际out_of_range仍带之前contact[.0393957,.0449137,−.9]，证明has_contact不能代表当前回波；范围max也不是物体恰在3m的保证。

LogicalCamera原生pose是相机世界位姿(0,−4,1)，model.pose是sensor相对位姿，+X前向。目标在前方相对(2,0,0)，独立collision遮挡件在相对(1,0,0)，两者实际同时入名单；目标移到世界(−2,−4,1)即不再被报告，恢复后重新出现。模型名单来自碰撞AABB与视锥，不做遮挡、RGB识别或语义学习；大地板AABB也实际入名单，因此不要求列表精确只有两项。Logical原生无time的边界完整保留。

所有正式输入在运行中保持字节不变，错误源在任何控制/继续世界前被拒绝，暂停iterations不动；四个模型位姿/四元数/线角速度和原暂停逐项恢复，9个课文件在正式前后以及Windows读回SHA完全一致，无清理错误及归属进程残留。真实桌面tests/ui_probe.py --lesson20 PASS36.057s/13项：环境/课包、真实Gazebo/两个PTY和Bash jobcontrol、暂停/单步/继续、完整九阶段验收、只读参考、学习记录及关闭清理。根代理查看原始Qt与Gazebo截图，确认中文教材/通过状态、导航块、目标/遮挡件与逻辑视锥真实显示；截图仅是当前视角，五种传感器上线依据仍为逐项原生数据与受控变化。UI轮完整verify另用15.105s，不混入上述正式180窗/2160条计数。

完整原始报告、wire消息/时间、状态查询/恢复、源与库指纹、运行日志保存在两端.deploy/lesson20-native-probe*、lesson20-formal*与lesson20-ui*，两张实际截图随源码发布。桥源SHA cb281087e08d9d17744daf4b69b71e56afa33a6df8e7dba9f39c37dd280e15d3，实际SDK库SHA a4a83aa7b6c7fa51550ed1fb3e348084ef5195fe8865781d50a709489b2d3c73。原VALIDATION_LOG前95135字节完整保留，既有2026-10-29交接快照保留；完成L20独立提交、中转推送与远端核对后进入L21。

依据：[GpsSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/GpsSensor.cc)、[SphericalCoordinates](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/common/SphericalCoordinates.cc)、[MagnetometerSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/MagnetometerSensor.cc)、[AltimeterSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/AltimeterSensor.cc)、[SonarSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/SonarSensor.cc)、[LogicalCameraSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/LogicalCameraSensor.cc)。


## 2026-10-31 L21：时间来源、成功Update与独立lockstep

tests/lesson21_acceptance.py正式PASS 124.404s：3个全新私有场景、8组核心检查（默认、仅相机10Hz、仅步长2ms、保存未重载、错误IMU频率、错误图像格式、错误主限速、精确恢复再验）及2组真实取消检查全部通过。4轮正向verify分别包含连续、暂停、单步、多步与off/on完整重启，共12个双通道观察窗口/24个通道窗口；下面数值只来自该正式报告，不含探索或UI轮。正式共保留183852条raw观察事件，其中183355条被接受的camera/IMU标签事件，raw/unique/回退分层计数，不把事件数称作物理采样完整数。10个课源正式前后不变，3个会话token均无残留，cleanup_errors为空。课程本地离线构建，无新增下载或宿主系统/驱动修改。

L21运行流程为实际SDF/prepare指纹检查→完整--lockstep主world→SDK读取原生ImageStamped.time、IMU.stamp、WorldStats与无原生stamp的Perf/metadata→像素原字节外存→Python实际ROS事件发布/订阅→连续2模拟秒窗口→暂停墙钟保持→真实1步/0.1模拟秒多步→主world保持暂停并串行启动两个私有Master/IGN partition的纯原生child→恢复原pause并回收本课PID。child使用同一临时benchmark.sdf，仅共同把RTU改为0，argv模式差仅--lockstep；不将主限速场景与无上限child之差全归因于lockstep。

相机ConnectUpdated只在UpdateImpl成功后计数，与LastMeasurementTime成对快照；窗口用Δcounter/ΔmeasurementTime，不是N−1，也不是硬件曝光或Image已送达。钩子可能晚接，只证明窗口增量。consumer原trace顺序、原标签/receipt不变；升序unique标签只建立密度/gap/离线nearest索引，原始接收数、unique/重复标签和回退count/max并列，不能据此定位网络层。主观测20/100Hz门限保持；ON子实验20或10Hz目标由真实成功Update计数率±2%/≥1模拟秒且same/regressed增量0验证。WorldStats.real_time扣暂停，与主机CLOCK_MONOTONIC不同；ROS服务读取world时间不是另订阅/clock。

首版v1探索的主连续/pause/single/multi已过，但无实时限速off实际出现标签相对观测序回退；曾有camera1660raw/1655unique/5重复标签/2回退，IMU26776事件/995回退。不能把回退直接归因于网络重排，也不能排序后将标签重配原receipt。v2只建立独立sorted-unique标签统计视图，保留raw顺序、重复/回退与首观察事件序号；随后ON探索仍出现consumer密度17.469Hz而非目标20Hz、回退569，camera/IMU覆盖跨度305/54.5模拟秒，说明接收标签密度不能证明全部生产端更新。两次失败与各自源/原始数据保存在.deploy/lesson21-native-v1/v2与lesson21-course-v1/v2，不记作整课通过。

v3新增只读成功Update钩子：Camera.UpdateImpl经rendered/PostRender成功返回true后，由Sensor.updated信号累计事件，并在短锁下保存count与Camera.LastMeasurementTime一致快照；卸载停用状态并断开信号，回调只持共享计数/弱sensor，不改调度或强制更新。native ImageStamped.time另在发布路径取scene时间，不能与该getter无条件混同。metadata中的hook_requested时刻只说明局部挂接上下文，counter不能冒充从world启动全量。Python按counter建独立快照索引，用首末Δcount/ΔLastMeasurementTime，不减1；同counter重复快照必须一致，same/regressed增量另列。成功Update不等于硬件曝光或消息交付。v3探索另为PASS32.388s，其中verify14.058s，曾观察off更新6.3168Hz、on更新20Hz而ON consumer约17.071Hz；这些仅为探索例子，不替代下面正式值、不混计正式耗时或窗口。

| 正式正向轮次 | 主相机标签密度/Hz | 主IMU标签密度/Hz | 单步Δsim/ns | 多步次数 | 多步Δsim/ns |
| --- | --- | --- | --- | --- | --- |
| baseline | 20 | 100 | 1000000 | 100 | 100000000 |
| camera_rate_only | 10 | 99.952176 | 1000000 | 100 | 100000000 |
| physics_step_only | 20 | 100 | 2000000 | 50 | 100000000 |
| exact_source_repaired | 20 | 100 | 2000000 | 50 | 100000000 |

正式主窗口首次观测的不同标签按原序严格增加，目标相机20或10Hz±2%、IMU100Hz±5%保持；native时间原字段与各层单调时间交叉核对，间隔落在实际1/2ms网格。各正向pause_hold实际等待约0.400097至0.401173墙钟秒，原生sim/iterations与ROS服务world时间不推进；真单步1迭代，多步目标0.1模拟秒，1ms/2ms分别100/50步，返回仍暂停，WorldStats与服务读回一致。nearest只查询已观察标签，保存原stamp和首观测序号，不重采样、不重写stamp、不声称在线硬同步或融合。

| 轮次/模式 | 成功Update/Hz | Δcount/测量跨度s | 相机consumer标签密度/Hz | 相机raw接收/墙钟Hz | 相机raw/unique/重复标签 | camera/IMU回退数 | WorldStats RTF |
| --- | --- | --- | --- | --- | --- | --- | --- |
| baseline/lockstep_off | 6.119126 | 1552/253.631 | 6.103909 | 817.905914 | 1640/1631/9 | 2/1068 | 133.295012 |
| baseline/lockstep_on | 20 | 6080/304 | 17.242503 | 2639.19795 | 5291/5291/0 | 487/406 | 160.270525 |
| camera_rate_only/lockstep_off | 5.575566 | 1517/272.08 | 5.53487 | 801.726381 | 1610/1602/8 | 2/702 | 145.675548 |
| camera_rate_only/lockstep_on | 10 | 2702/270.2 | 10 | 1383.337039 | 2776/2776/0 | 46/513 | 139.152295 |
| physics_step_only/lockstep_off | 2.888286 | 1654/572.658 | 2.884574 | 869.303487 | 1743/1743/0 | 5/881 | 302.121262 |
| physics_step_only/lockstep_on | 20 | 13039/651.95 | 8.045675 | 2605.506533 | 5215/5215/0 | 471/481 | 344.328693 |
| exact_source_repaired/lockstep_off | 2.829779 | 1628/575.31 | 2.826241 | 856.895973 | 1722/1722/0 | 11/805 | 303.861673 |
| exact_source_repaired/lockstep_on | 20 | 12786/639.3 | 8.377668 | 2706.822985 | 5418/5418/0 | 406/617 | 335.121786 |

上表生产端是成功Update事件率，consumer是标签覆盖密度，raw墙钟接收率是另一分母；不把三者互换。ON producer均按实际counter证据验收，consumer回退/重复原样记录，不拿它失败替代全部更新丢失的结论。WorldStats RTF单独取其sim_time/real_time窗，不能用sorted标签覆盖/receipt比伪装世界RTF；Perf无原生stamp，初期异常值保留诊断，不作成功数值gate。两child同benchmark SHA、私有Master与partition不同、argv剔除--lockstep后相同，实际metadata另核strict flag、sensor rate、dt和RTU0；主世界RTU随dt联动保持名义cap1。当前数据不建立off必漏、on必更快或某硬件固定RTF承诺。

真正清理深度取消PASS 7.245973s：对当前令牌、PID起始时间/argv/Master身份都匹配的活跃observer发SIGSTOP，确认实际stop_owned SIGINT已pending后再取消caller并恢复observer；失败取消报告只含off child，没有继续启动on，两owned PID均回收且主pause恢复。应用关闭取消PASS 1.694461s：在独立child实际存活时调用LessonSession.stop，caller实际返回-9，完整归属token进程扫描后无残留。stop不能因最先管理组已退出而跳过剩余组；两种真实取消与8个核心输入/物理检查分列。

UI tests/ui_probe.py --lesson21 PASS 34.906s / 13项，Qt5.12.8、平台xcb：课程/环境、真实Gazebo/两个PTY与Bash jobcontrol、暂停/单步/继续、完整验证按钮、只读参考、学习记录、实际截图与退出清理均完成。UI按钮内verify另耗15.007382s/保留37442事件，不混上表正式计数。根代理已逐张查看UI报告指向的真实Qt和Gazebo原图，核对课程、验证状态与场景；图形截图不替代原生数据与成功Update证据。

图像是真正原生RGB_INT8数值3、128×96、step384、36864字节；SDK先保留protobuf/base64，Python将原data外置到.rgb并记录binary_storage、SHA256/长度，ROS事件传字段与blob引用，不是标准ROS Image像素流。独立正式检查在临时Storage删除前读取并保留原字节，现存asset-reference-map共12个原路径映射、1个内容去重原图；中心[255,0,0]且实际像素不均匀。旧/tmp路径仍作为观察身份留在报告，但临时树已删除；通过formal-artifacts/asset-reference-map.json映射到扁平assets/SHA.rgb查找现存字节，child/observer日志也平拷，避免Windows长路径。探索的lesson21-probe-storage-*另有意保留。

| 正向轮次 | 实际输入world SHA256 |
| --- | --- |
| baseline | c1a48e2ec4620b871a1c3ba93b41ea22865f8064a7c2e4b65d337552d52f4f12 |
| camera_rate_only | 4b541749402fffb2567454397946ff1b296631a770a4f17e74067bc886a2ec4d |
| physics_step_only | 0d5913062c7c4ef01c57120abc90150d3729bdd1739a077396d3c32437612b7b |
| exact_source_repaired | 0d5913062c7c4ef01c57120abc90150d3729bdd1739a077396d3c32437612b7b |

| native_observer.cc源SHA | metadata_plugin.cc源SHA | 实际observer二进制SHA | 实际metadata库SHA |
| --- | --- | --- | --- |
| a014bfd5aaabe0d2be8b3b1ba0c1ae06d4d7b5d42e6b97fc2f715d28a634517b | 84cd26e3b9f70d846898b9b192edb67fccee804a54972a9ba1dfd1454c5a7b1c | a90e5ad4f6d9803333419597d5df308a85c860e44c322ffba7c2a050064866c8 | d03054174a438e48ab67115a379704d5725dfaa2463354f7d894b08e1ae176b5 |

| 冻结课文件 | SHA256 |
| --- | --- |
| config.py | 51b746605885683ad71923978e7463a811f119991f10ede7be7b2b9ab85bddd9 |
| lab.py | 89735c7204985acf73d2ecccda1260d8df5e9d13dbef450899fb2dbcc9ee4eef |
| lesson.json | 6d91b57188021b78edb3e99466c29246700ad5f968db76cc208499184ac81438 |
| lesson.md | 05c87da929aa35097cd81bcd1577377bca172df774814f6840e2e30c5f9410fe |
| metadata_plugin.cc | 84cd26e3b9f70d846898b9b192edb67fccee804a54972a9ba1dfd1454c5a7b1c |
| native_observer.cc | a014bfd5aaabe0d2be8b3b1ba0c1ae06d4d7b5d42e6b97fc2f715d28a634517b |
| prepare.py | 38e32f0b3a5cb1292b01ff2c689ee42d817096aed697381b6fe7068d7caa7221 |
| reference.md | 6163f57d73c8feaab327b93302a3680b7a002cae6eb54734a9714146a7fc52a8 |
| verify.py | d31d457357617d1d7ad5ab440e293deff200a1e2a0793450c6ff874320440a49 |
| world.sdf | c1a48e2ec4620b871a1c3ba93b41ea22865f8064a7c2e4b65d337552d52f4f12 |

报告SHA256：formal 39a6c4d7814d20ed151031e72b49bd0035a8b20599cca251bfe5a03ebf2b3ed3；UI 9ce3da26c528064f05e539e3f7188fe8fdf69802b159939857a2198fd1d20b95；flat asset-reference-map 00d03472e83a48b21ee3213ede3b72eac6bfe22e86f6faa222fa708e94ca8055。统一冻结的独立验收源码SHA 9c46b411ad009b3e4bacc806bdc38a84a6944368ffb0e088722e1839de243577；UI探针SHA 12cf64d65a476946a9b9bd907aed419c1c2ab0a5136a48371e66ef50171c89a0；实际运行框架runtime.py SHA d7fd217a10c9a7c352894755db91a10ed961cabedaadf743cedc998d7208ffa2。主课server_args显式传递完整--lockstep启动模式，实际producer/metadata确认而不以命令字符串代替；新字段与原课程默认兼容，App.stop真实取消覆盖全部归属组。

证据位于两端.deploy/lesson21-formal.json、lesson21-formal-artifacts、formal日志及lesson21-ui*；自动临时练习、个人参数、源码、构建指纹和退出结果分别记录。原VALIDATION_LOG前102967字节原封保留，SHA fa72eeff734608d2861b81db01979696b3ef06011f74e2e324f3639d6f93b2b8；前交接快照18305字节及2026-10-29尾5054字节均保留。完成L21独立提交、Windows中转推送与远端核对后进入L22；当前预交付主课01–21与091/092为23项目，L22–L37共16门主课与G02仍未完成。

依据：[Classic lockstep](https://classic.gazebosim.org/tutorials?tut=lockstep_physics_sensors)、[Performance metrics](https://classic.gazebosim.org/tutorials?tut=performance_metrics)、[Sensor.Update](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/Sensor.cc)、[CameraSensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/sensors/CameraSensor.cc)、[World](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/World.cc)。


## 2026-10-31 框架：ROS课包路径与辅助节点就绪

为L22真实roslaunch补充两个可选公共接口，独立于课内容提交。ros_package_paths仅允许ROS课程的相对根目录：catalog验证源码边界，session环境再次resolve个人副本，拒绝逃逸或符号链接越界；加载Noetic后前置课内路径并保留setup和用户原包路径，不改os.environ或系统profile。验证命令、辅助节点与真实终端继承同一会话路径。未声明的新旧课程保持原环境行为。

after_startup命令在所有startup辅助进程创建之后、GUI/终端打开之前顺序执行，每条after_startup_timeout默认90秒且必须正有限；耗时仍在后台，异常/超时/取消走既有start失败清理，只回收本课进程。整体顺序是prepare→服务端就绪→after_start→全部startup→after_startup→GUI/终端，解决在辅助节点尚未启动时等待其ready的问题。

新增10个必要单元检查覆盖声明类型、路径/真实symlink边界、会话环境与全局隔离、钩子顺序及失败/超时/取消清理。本机74项中72通过、2项Linux专属跳过；ikaros全74项PASS0.203s。新框架下原L21公开CLI --smoke21 --gui --terminals在全新目录完整PASS24.272s，实际GUI/两终端与课程verify均通过，最后运行组件全部结束；L21源和本课原有默认流程未改，不把此回归混入其此前正式计数。

真实L22最小启动仅作为框架验收，不计整课交付：PASS8.774871s，无脉冲请求。private源ROS包由rospack/roslaunch实际发现并启动driver/observer；after_startup核真实XMLRPC Master/节点PID、参数、remap与新反馈后，才出现Gazebo窗口和两个独立PTY。读取两个真实终端进程的环境并各自实际rospack find，均指向本次个人ros_ws/src/gazeboteach_l22；原父环境字典完全不变，退出token进程扫描为空。启动日志记录的顺序与新接口一致。

此前最小负例25.316517s：课内XMLRPC辅助函数与module同名遮蔽，真实启动后ready失败。框架正确阻止GUI/终端并清理roslaunch两节点、私有Master和gzserver；报告/准确源码包保存在.deploy/lesson22-framework-probe-v1。修复课内import alias后再得上述正例，没有通过放宽节点就绪门限解决。课内prepare另在私有Master已启动、gzserver之前显式设/use_sim_time=true；这属于L22课码，不修改旧课的时钟默认值。

证据为两端.deploy/lesson22-framework-unit.log、lesson22-framework-legacy21.json/.log与lesson22-framework-probe.json/-runtime.log；最小ROS课包尚未完成物理实验/整课验收与发布，当前交付仍主课01–21及091/092共23项目。当前L22目标继续执行，先独立提交推送本框架支持，再完成课程的独立物理、桌面验收与发布；不跳到L23。原VALIDATION_LOG前114892字节完整保留。


## 2026-10-31 L22：ROS启动、实际计算图与有限力任务

tests/lesson22_acceptance.py正式PASS 102.024s：3个正常全新场景、9组核心场景、3组静态解析、2组launch故障及2组取消检查，共14项scenarios及2项cancellation_checks。3个正常场景为2N/cart_a基线、只改4N及只改cart_b；另一个全新错误remap启动场景，合计4个实际session token均无残留，cleanup_errors为空。17课源正式前后字节/SHA不变。正向含修复再验共4轮，以下原始计数只来自正式轮，不混最小探针或UI。

L22流程为私有Master→个人源包overlay/输入指纹与/use_sim_time=true→Gazebo/API初始化→world-ready→真实roslaunch解析arg/include/group/私有param/remap并启动两个节点→after_startup核实际图/PID/缓存/新反馈→Trigger提交未来0.1模拟秒开始、持续0.3模拟秒的2或4N world+X有限ApplyBodyWrench→ODE真实加速→到期匀速→清本课body队列并恢复进入pose/twist/pause。服务调度包含开始/结束端点，实际速度可多一tick冲量；试块1kg、零g、零速度衰减，取消是清施力任务而非刹车。原队列、driver历史与已经流逝的模拟时间不倒回。

Master负责名称发现与参数，不中转全部主题负载。ModelStates原生无header，source_stamp_ns=null，observer的50墙钟Hz是输出上限；其回调计数和接收ROS/monotonic时间不伪装成native采样。GetModelState header是API ros::Time::now读回，真实查询保留前后GetWorldProperties与主机时间包夹；coast反馈只按同主机接收邻近交叉检查，不称采样同步。scheduled、服务成功及空clear响应均不是物理完成证据。namespace只重命名ROS图，不复制Gazebo实体；arg须显式写入私有param，remap不改类型/实体名，driver只在启动时缓存参数，保存未重载与运行参数不一致都在物理控制前拒绝。

公共ros_package_paths与after_startup支持已独立发布核对为aa1b335628836395eae531fe4f1686ef6345817f，74项Linux检查、既有L21图形回归和L22最小启动属于该框架修复证据，保留在原历史前缀，不混入本课正式计数。L22最小v1曾因xmlrpc模块名被函数遮蔽而就绪失败；修为独立模块别名，同时将/use_sim_time初始化前置到私有Master、再启动ROS API/节点。失败与修复最小探针都标startup_only且未发脉冲；它们不代表完整课程通过。

| 正式轮次 | Master PID | roslaunch PID | driver PID | observer PID |
| --- | --- | --- | --- | --- |
| baseline | 85875 | 86150 | 86242 | 86243 |
| force_only | 88124 | 88396 | 88487 | 88488 |
| namespace_only | 90353 | 90625 | 90716 | 90717 |
| exact_inputs_repaired | 90353 | 90625 | 90716 | 90717 |

每轮由Master/XML-RPC读回实际节点URI/PID，并检查它们属于本次token和个人包，两课程节点共用同一roslaunch父进程，父进程与已有rosmaster分开；参数、source/settings指纹、实际remap及/gazebo服务/发布者一致。3次--nodes/--dump-params覆盖不同arg预览，前后当前节点URI与私有参数不变，预览namespace没有新节点；不把静态解析输出当作实时图，也不把仅URI对比夸称额外PID测量。

| 正式轮次 | 力/N | namespace | active条数/跨度s | a拟合/m·s⁻² | coast vx/m·s⁻¹ | coast a/m·s⁻² | 最大位移误差/m | 反馈/邻近对 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| baseline | 2.0 | /lesson22/cart_a | 25/0.228 | 1.999432 | 0.602…0.602 | 0 | 0.002246 | 60/25 |
| force_only | 4.0 | /lesson22/cart_a | 25/0.229 | 4 | 1.204…1.204 | 0 | 0.004636 | 60/25 |
| namespace_only | 2.0 | /lesson22/cart_b | 25/0.232 | 1.998245 | 0.602…0.602 | 0 | 0.002264 | 59/24 |
| exact_inputs_repaired | 2.0 | /lesson22/cart_b | 25/0.2295 | 1.99855 | 0.602…0.602 | 0 | 0.002256 | 60/25 |

四轮共保留745条原始观测：506次独立状态服务查询、239条实际headerless反馈，形成99对coast接收邻近比较。每轮active剔两端30ms、至少10条/0.15模拟秒，vx最小二乘斜率与F/m相差≤5%；coast从end+50ms起至少10条/0.3秒，加速度绝对值≤0.05m/s²、速度误差≤0.02m/s、全周期位移误差≤0.01m。所有原始service_header、世界/主机包夹与反馈接收时间保留。API响应只证明接纳，轨迹证明先加速后匀速；inclusive端点导致本次2N/4N的终速见表，不改写成理想0.6/1.2。

| 受控负例 | 实际拒绝诊断 |
| --- | --- |
| saved_without_reload | RuntimeError: Saved inputs differ from the loaded lesson: reload/restart required |
| live_parameter_cache_mismatch | RuntimeError: Bounded launch readiness failed: RuntimeError: Actual private driver parameters differ from launch arguments |
| wrong_mass | ValueError: Fixed world mass/inertia/gravity/damping/physics/body geometry differs from template |
| wrong_damping | ValueError: Fixed world mass/inertia/gravity/damping/physics/body geometry differs from template |
| wrong_timestep | ValueError: Fixed world mass/inertia/gravity/damping/physics/body geometry differs from template |

五种负例均在物理控制前拒绝，trace为空且没有command。质量/阻尼/步长负例修改的是个人SDF输入并触发固定模板契约，不能描述成已加载错误物性后的运行对照；实际原世界保持原状。运行私有参数被改为与缓存不符时，也以有限就绪超时拒绝；精确恢复保存输入和参数后重新完整通过。namespace_only还预设非平凡位置、姿态与线/角速度，前后恢复检查按q/−q等价，不只验证零初态。

| 取消场景 | 保留速度/m·s⁻¹ | 后段条数/跨度s | 后段a/m·s⁻² |
| --- | --- | --- | --- |
| cancel_queued | 0 | 89/0.848 | 0 |
| cancel_active | 0.256 | 65/0.623 | 0 |

queued取消发生在未来start前，active取消发生在实际加速后；随后观察跨越原end并继续匀速，实际vx保持、加速度归零，才证明取消效果。空clear响应不作为删除数量或测量证据。required=true故障向已确认token/start_time的driver PID发SIGTERM，实际driver/observer均退出；已有Master PID仍为90353，Gazebo仍活且dt=.001。其有限任务仍可到期，之后43条/0.4035模拟秒coast拟合a=0m/s²，说明launch进程生命周期不自动撤销Gazebo内部任务。新建错误observer输入remap在25.165116s内因真实图/订阅不符失败并清净，不能用server启动成功代替节点就绪。

真实桌面tests/ui_probe.py另PASS 20.654s/13项，包括开始初始化、两个真实PTY、暂停/继续、模型复位、验收按钮、参考页保留个人world、学习记录、两张归属窗口截图与退出清理。UI内另一次verify为1.531277s，只列作界面证据，不加到正式四轮统计。根代理已查看报告指向的真实Qt与Gazebo原图，检查中文课程/验收状态及蓝色试块与黄色视觉尺；截图不替代真实状态数据。本课纯Python/标准消息源包不需catkin_make或新增下载；个人源包只在session环境overlay，不改父进程/全局ROS_PACKAGE_PATH。

| 正向轮次 | 实际world SHA256 | 实际settings SHA256 |
| --- | --- | --- |
| baseline | 2d6bf76dc7babc9e707ce16e17be1f0aadea6e5b444524e5d2bf74cc034c62ad | b6a98bdc88aa8d49e606bc7c3f89d6eec06e606a612e42f3995148e2f31f6ba0 |
| force_only | 2d6bf76dc7babc9e707ce16e17be1f0aadea6e5b444524e5d2bf74cc034c62ad | 8586498cce2a47caadf37eb0a154acb49a3d7577e8a7cf9a020876e015ed6186 |
| namespace_only | 2d6bf76dc7babc9e707ce16e17be1f0aadea6e5b444524e5d2bf74cc034c62ad | 5c8c7ff908795a500c0b678ceaaa9d6daebfacc4ac103c9a9656cb8663ecde24 |
| exact_inputs_repaired | 2d6bf76dc7babc9e707ce16e17be1f0aadea6e5b444524e5d2bf74cc034c62ad | 5c8c7ff908795a500c0b678ceaaa9d6daebfacc4ac103c9a9656cb8663ecde24 |

| 冻结课文件 | SHA256 |
| --- | --- |
| config.py | b055b685fd3c9c81b654f8dd8a442de8de2665284566361829c6e9798d778d1d |
| lab.py | 555fa106ad052bb66254d059c879dd7e8a799b64cae34553a4793b7d8a759549 |
| launch_settings.json | b6a98bdc88aa8d49e606bc7c3f89d6eec06e606a612e42f3995148e2f31f6ba0 |
| lesson.json | fb6ba17e1623a810f6cf6933422514232a81bb6cd08ee86db27d4119d2ac2b1a |
| lesson.md | 1626b543c70076da5cd4d9fd547df092b594bf9dad343645f54cde0cd5a4457e |
| prepare.py | 615055301cad651e9e9fd75f4e0d2871a61785e277779eb4c88051a878e99322 |
| reference.md | 8f0be670916e3d55e621f65619c9a620b834597c2eecf8fa87aff30b87a84332 |
| ros_entry.py | 52adc919b875f83e8c1e991c6d14c869a9d1c1f1655331c7814dafc5901c520b |
| ros_lab.sh | 2856b4ea8cdbb4362fc9c1ecab55dd80cd0a8c050a15d7189924d1fcec5ecd9b |
| ros_ws/src/gazeboteach_l22/launch/lab.launch | 6849df18f2139d72ae54c7419d599094bdc443d9bdd06b97f4455b649e5a2094 |
| ros_ws/src/gazeboteach_l22/launch/nodes.launch | 9bb1e87f45ae1e30a54b3ea835767918849120b938c44120eba750a6a9c09370 |
| ros_ws/src/gazeboteach_l22/package.xml | c68f834dc2035cd94860415ca3b9a90223da31de4fc6c09d64c78fb716854383 |
| ros_ws/src/gazeboteach_l22/scripts/common.py | abf09baa1dd573e6201939ce18c075fc35dfd5d66e4e2d247f31d1a0f768e9e4 |
| ros_ws/src/gazeboteach_l22/scripts/pulse_driver.py | dce27e7e8d97ff5b05946e574e8a7597f3ff991bbe95ec411b91cea9e2085b47 |
| ros_ws/src/gazeboteach_l22/scripts/state_observer.py | 4e58166df9cf2d239848db358db6a6973b40242116a2626d69effa06820c1d7a |
| verify.py | f0bc1a4b9d83f5198f71e3838d2308f52d49a44e1aef09e02ccac0a1dfe7019b |
| world.sdf | 2d6bf76dc7babc9e707ce16e17be1f0aadea6e5b444524e5d2bf74cc034c62ad |

| 正式原始报告 | 现存归档路径 | SHA256 |
| --- | --- | --- |
| baseline | .deploy/lesson22-formal-artifacts/baseline.json | 6a569ed1dc7aed78039ff098d5edd8dbefb6dd3cf6638a4b8b430fe822555ef0 |
| force_only | .deploy/lesson22-formal-artifacts/force_only.json | 2321993faed7f6a5ba3e6f5aa8aa214dc278fd4343cebff3911bf6008d7894be |
| namespace_only | .deploy/lesson22-formal-artifacts/namespace_only.json | f1171704e9d8caa2700f221fcee85a8481bedfe83b745df09b035d4d4836d8ec |
| exact_inputs_repaired | .deploy/lesson22-formal-artifacts/exact_inputs_repaired.json | 6b45665a773f48694e6570dd3d3f057db915f9fc580a81a21de43f85dfddfe74 |
| saved_without_reload | .deploy/lesson22-formal-artifacts/saved_without_reload.json | b45a287bd1f1b50ca7132ef2e085dd8bc0eedff6976a1bfbe091514ef30d4e26 |
| live_parameter_cache_mismatch | .deploy/lesson22-formal-artifacts/live_parameter_cache_mismatch.json | 05a34098b09b31d66b05b89cbe0583d92f940ac02fb5bffffbf0dc2f375b882c |
| wrong_mass | .deploy/lesson22-formal-artifacts/wrong_mass.json | 8e10dd9a4eb6ce4c4650a658dc1189dabb706e491d06b63b70768f2a540a7ae6 |
| wrong_damping | .deploy/lesson22-formal-artifacts/wrong_damping.json | b51ca909565d99e11fa1f0658c970edff1276b824af52fdb58d15c222dbc0ae5 |
| wrong_timestep | .deploy/lesson22-formal-artifacts/wrong_timestep.json | 1e7332fde019ce32b8e65fdf416b17aa84ac4e7efab252eaadbe435a6cc3891a |

正式报告SHA256 8013111928cc00996f1fd29c2d070dce9076753d61f3457879f3aa2b0696e9a5；UI报告SHA256 6d9e6ab4539140245dcffc61ecadd20ae8b4613b2c8b774d601db394c60e2ef6。独立验收源码SHA256 5732aa6be4878f1867f1d775ac110ffe33053cc8b65c96f0a038eb2d8b96101b；UI探针SHA256 12cf64d65a476946a9b9bd907aed419c1c2ab0a5136a48371e66ef50171c89a0；运行框架runtime.py SHA256 e25447b2b2b4f4aece99e25568ac553de20fc46255b687fec6f166cce33a143e。原始报告、worker、ROSlaunch日志与session元信息在formal-artifacts归档；自动临时Storage删除后，不把旧/tmp身份路径称作现存文件。

原VALIDATION_LOG前117801字节/SHA 7d4ba9db1b99ed0ac59ec16b583d97c8a264bf888384fe997dbb993637af2a3c原封保留，包含本课之前独立发布的框架验证；未使用旧114892字节l22-before快照覆盖它。SESSION_HANDOFF原20624字节正文及2026-10-29尾5054字节保持。完成本课独立提交、Windows中转推送与远端核对后进入L23；预交付01–22主22课与091/092共24项目，余L23–L37共15门主课及G02。

依据：[Noetic Master API](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosmaster/src/rosmaster/master_api.py)、[roslaunch XML解析](https://github.com/ros/ros_comm/blob/noetic-devel/tools/roslaunch/src/roslaunch/xmlloader.py)、[ROS时间初始化](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/init.cpp)、[Gazebo ROS API](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp)、[ApplyBodyWrench](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/srv/ApplyBodyWrench.srv)、[ModelStates](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_msgs/msg/ModelStates.msg)。


## 2026-10-31 L23：URDF/Xacro、真实转换与两关节动力学

tests/lesson23_acceptance.py正式v2 PASS 55.596s，3个全新私有场景/8组scenarios全部通过：baseline、length_only、lump_only、saved_without_reload、generated_artifact_drift、bad_mass、bad_gravity及exact_inputs_and_properties_repaired。17课文件正式前后SHA不变，3个session token均无残留、cleanup_errors为空。4轮正向含精确修复再验，下列统计只来自v2正式，不混启动探索、失败v1或UI。

L23流程为私有Master先设/use_sim_time→源包overlay与Xacro输入→真实xacro生成URDF→gz sdf -p转换并保留原值/SHA→roslaunch写/lesson23/robot_description并一次性spawn_model(-urdf)→只读说明节点/实际link物性就绪→保存暂停快照→三次边界姿态初始化与真实FK核对→±.03N·m/200ms及upper+.12N·m/350ms有限肘力矩→逐次读取真实双关节响应→清唯一basename任务、恢复q与全link pose/twist及原pause。静态SetModelConfiguration用于初态；动态期间由ODE推进，读取世界/主机时间包夹，不强制轨迹，也不以单杆τ/I替代多体耦合。

L1 .6→.8保持质量，联动几何、COM、惯性与肘origin；preserve_tool仅切Gazebo保留扩展，不删除URDF工具。URDF始终5link/4joint，实际物理link从4变3，合并后的arm2质量.52kg、COM x=.207692m、Izz=.0075909kg·m²；总模型质量仍2.52kg。理论URDF→SDK六有效数字转换按最终量子预算，运行API→转换SDF另严格比较，所有三层原值保持。API reported_type/damping不作为物理类型/阻尼证据，固定joint的NaN不当合法角度；只读两moving joints。

本版ODE GetLinkState读取link原点世界速度，而SetLinkState调用SetLinearVel设置CoM速度。恢复读取实际GetLinkProperties.COM与快照四元数/ω，先算r_world=R_world_link·c_link，再发送v_COM=v_origin+ω_world×r_world；只转换setter请求，不改原始快照，仍按原点速度读回核对。恢复阈值、独立验收源码与其余算法保持；旧任务队列和已流逝时间不重建。

早期启动探针曾把固定joint的position当普通角度序列化，触发Out of range float values are not JSON compliant；失败保存在.deploy/lesson23-start-v1。后来保留原异常语义、只对moving joints读取合法q/rate，默认与lump启动/物性探针均通过，但这些探索不证明完整课程。真实版本xacro1.14.20、urdf包1.13.4、urdfdom1.0.4、libsdformat9 9.10.1；check_urdf未安装，实际使用已有xacro与gz sdf解析库，没有新增下载或伪称运行了该工具。

正式v1仅baseline/length完成，lump非零速度快照恢复失败，整轮passed=false；完整输入、源、日志、原始报告保存在两端.deploy/lesson23-formal-v1。它不是整体通过，也不是允许放宽恢复阈值的理由。原始arm2原点速度before=(0.00320553529242, -0.00280494143476, 0.001)m/s，after=(0.00341107058484, -0.00360988286952, 0.001)m/s；实际COM=(0.207692, 0, 0)m、world ω=(0, 0, 0.004)rad/s。实测after−before=(0.000205535292422, -0.00080494143476, 0)m/s，恰为−ω×R·COM，残差1.62630325873e-19m/s。诊断原文及SHA另存，不修改失败数据。v2只改lab.py与两份教材，verify.py、独立test及阈值均保持；补偿setter速度后重新完整运行同一八场景。

| 正向轮次 | 物理link数 | 静态姿态 | 独立FK比较 | 最大位置误差/m | 最大姿态误差/rad |
| --- | --- | --- | --- | --- | --- |
| baseline | 4 | 3 | 12 | 8.40036148998e-16 | 0 |
| length_only | 4 | 3 | 12 | 1.14304456355e-15 | 0 |
| lump_only | 3 | 3 | 9 | 5.31176747282e-16 | 0 |
| exact_inputs_and_properties_repaired | 3 | 3 | 9 | 5.55284957564e-16 | 0 |

共12个静态快照、42次实际物理link坐标比较。独立oracle从生成URDF逐joint复合origin、RPY与axis，核对3个指定姿态，不直接调用课程闭式FK；实际world/主机包夹在暂停期间保持。3个fresh场景还经gz model -i核对原生Model碰撞名单、数量及box/sphere/cylinder尺寸，原生文本已存档；实际link位姿另由上述独立FK验证，不把它说成逐collision pose比较。

| 轮次/link | 质量：理论/SDF/API kg | COM x：理论/SDF/API m | Izz：理论/SDF/API kg·m² |
| --- | --- | --- | --- |
| baseline/arm1 | 1/1/1 | 0.3/0.3/0.3 | 0.0303/0.0303/0.0303 |
| baseline/arm2 | 0.5/0.5/0.5 | 0.2/0.2/0.2 | 0.00681666666667/0.00681667/0.00681667 |
| length_only/arm1 | 1/1/1 | 0.4/0.4/0.4 | 0.0536333333333/0.0536333/0.0536333 |
| length_only/arm2 | 0.5/0.5/0.5 | 0.2/0.2/0.2 | 0.00681666666667/0.00681667/0.00681667 |
| lump_only/arm1 | 1/1/1 | 0.3/0.3/0.3 | 0.0303/0.0303/0.0303 |
| lump_only/arm2 | 0.52/0.52/0.52 | 0.207692307692/0.207692/0.207692 | 0.0075908974359/0.0075909/0.0075909 |
| exact_inputs_and_properties_repaired/arm1 | 1/1/1 | 0.3/0.3/0.3 | 0.0303/0.0303/0.0303 |
| exact_inputs_and_properties_repaired/arm2 | 0.52/0.52/0.52 | 0.207692307692/0.207692/0.207692 | 0.0075908974359/0.0075909/0.0075909 |

工具保留和lump均保留质量贡献与几何。独立惯性oracle将每个URDF刚体的COM与旋转惯性转入共同坐标，再按平行轴合成；SDK六有效数字误差按半个末位量子（带明确小数容差）对理论核对，actual与实际SDF另在1e−8对照。课程ready对COM1e−8m、惯性分量1e−9kg·m²等阈值保持，不能把理论高精度值回填原SDK字段。

| 轮次/动态阶段 | 实际条数 | 沿力矩方向Δq或upper最大q/rad | 跨度s或尾条数/均值误差rad | 沿力矩方向峰值或尾最大|dq|/rad·s⁻¹ |
| --- | --- | --- | --- | --- |
| baseline/motion_positive | 35 | 0.0847715141741 | 0.355 | 0.410897870567 |
| baseline/motion_negative | 35 | 0.086013467018 | 0.3565 | 0.415795502304 |
| baseline/upper_limit | 49 | 1.20004824284 | 6/1.3327731622e-07 | 0.000171859146479 |
| length_only/motion_positive | 35 | 0.0787565670809 | 0.354 | 0.382693028272 |
| length_only/motion_negative | 35 | 0.0798738203186 | 0.3545 | 0.386928282687 |
| length_only/upper_limit | 50 | 1.20000388079 | 6/1.17947338733e-06 | 0.000972523696189 |
| lump_only/motion_positive | 35 | 0.0844678029332 | 0.354 | 0.412151968021 |
| lump_only/motion_negative | 35 | 0.0857065079176 | 0.3545 | 0.4158477096 |
| lump_only/upper_limit | 49 | 1.20001714777 | 6/2.96584996932e-07 | 0.000398105090576 |
| exact_inputs_and_properties_repaired/motion_positive | 35 | 0.084771814165 | 0.355 | 0.409792133322 |
| exact_inputs_and_properties_repaired/motion_negative | 34 | 0.0838440544049 | 0.3485 | 0.414271256146 |
| exact_inputs_and_properties_repaired/upper_limit | 49 | 1.20008179269 | 6/1.51876775082e-07 | 0.000203828766963 |

4轮正向共有12个真实动态阶段，476次双moving-joint状态观测；连同静态快照，trace共488条。正负阶段有符号Δq≥.01rad、峰有符号dq≥.05rad/s；每阶段至少10次真实查询并跨过任务到期后.1模拟秒。upper从q=(0,1.12)出发，最大q∈[1.17,1.22]，end之前最后60ms仍受力窗口≥3条、平均上限误差≤.02rad、最大|dq|≤.2rad/s，原阈值无降级。独立test在formal前已把upper选择明确为end前受力窗口；v1/v2采用同一SHA，不拿到期后coast窗口替代该目标。

| 负例 | 实际拒绝诊断 |
| --- | --- |
| saved_without_reload | RuntimeError: Saved model inputs differ from the loaded world: reload/restart required |
| generated_artifact_drift | RuntimeError: Generated URDF/SDF artifact drifted; reload/restart required |
| bad_mass | RuntimeError: Bounded URDF spawn/property readiness failed: RuntimeError: Actual mass differs on arm1 |
| bad_gravity | RuntimeError: Bounded URDF spawn/property readiness failed: RuntimeError: Actual gravity_mode differs on arm1 |

保存配置未重载、已生成URDF追加注释导致SHA漂移，均在控制前拒绝；bad_mass与bad_gravity则通过独立ROS客户端实际修改已加载arm1质量为2kg/启用重力，并读回证明，再要求课程拒绝。四负例trace均空、不进入静态/动态操纵，原物性/状态恢复后再次完整验收。lump场景先设置两个合法关节角，只给arm2小的非零线/角速度，其余links保存各自实际快照。v2恢复线速度最大分量误差0m/s、角速度最大分量误差0rad/s，pause恢复；各link位姿/q/rate仍按原阈值核对。

真实桌面UI另PASS 20.444s/13项，实际Qt/X11窗口、两PTY、pause/step/resume、验收、只读参考保留个人world、学习记录、归属截图与退出清理通过。UI内verify实际1.809361402s，仅属UI证据，不加入上述正式4轮计数。根代理已逐张查看报告指向的原始Qt/Gazebo截图：25项目、中文正文与通过状态正常，蓝橙两关节臂、球工具及工作台清晰可见。课堂GUI观察与自动物性/动力学结果分别记录。

| 正向轮次 | world SHA | settings SHA | 实际URDF SHA | 实际SDF SHA |
| --- | --- | --- | --- | --- |
| baseline | 082bf84f7902a0bf695a0e01bdfe36d3a6db0ca4d0a9c9f126e619a1c949b91a | d1189e72511bd52faa0d43b7e4a446dc7b71309e7f34f08583d3405e6e19aee8 | da9fa3e232a0dd5de435b3c6c45c739de49b7f92921b49666830d35703ef71f8 | f93f7ec3fc50c8bd2c54decb51012fcba2bd9693af8ad8751b0c70391234bf89 |
| length_only | 082bf84f7902a0bf695a0e01bdfe36d3a6db0ca4d0a9c9f126e619a1c949b91a | 59070d894e73d27fefe5ba8d88df750c5ec07e06c2a8c3efdb8fd3859efe993d | 7eab7353e1b0ad5ddb7b5ea7451ac1a1d1dab657ef81f379733be229dade3aa5 | 923097cbcc335afeda76ea694a30525e2487a0c65fb6f4e5c58fab337bd9f7b1 |
| lump_only | 082bf84f7902a0bf695a0e01bdfe36d3a6db0ca4d0a9c9f126e619a1c949b91a | 842f8771e5a8b52ba9060e985b4615c3fe410db4e57b04fd7e625095b229c204 | 30fb652ddca68f05266f9ca6dbde1805d11f064801586f2d9e3335b32f25e14c | f44fe1ffbbd6a8f1a3154c3d9b62d78955ecc587bc67a702a4f51b8771f70a65 |
| exact_inputs_and_properties_repaired | 082bf84f7902a0bf695a0e01bdfe36d3a6db0ca4d0a9c9f126e619a1c949b91a | 842f8771e5a8b52ba9060e985b4615c3fe410db4e57b04fd7e625095b229c204 | 30fb652ddca68f05266f9ca6dbde1805d11f064801586f2d9e3335b32f25e14c | f44fe1ffbbd6a8f1a3154c3d9b62d78955ecc587bc67a702a4f51b8771f70a65 |

| 冻结课文件 | SHA256 |
| --- | --- |
| arm_settings.json | d1189e72511bd52faa0d43b7e4a446dc7b71309e7f34f08583d3405e6e19aee8 |
| config.py | eb3738790f12846fc831653d29290f6e3af23d9cf5b3862aec9143b92127cd8f |
| lab.py | 032b10987791cc3f5c8b7874d483c9ae44f2d097a76c96907ba93ae81032b33a |
| lesson.json | bb35bd70248ab588f2a530bc51431000e5639f1e39ffc6d846e53a7ca16de707 |
| lesson.md | 9c9d8b7280f440c253774495ac71e7cf61e8ec81da53c2c1a7b23aff08fd6b08 |
| prepare.py | c915e927ad5129c3e32a486475bdd062296453fdda8664af1b0a0fd818c4a9a0 |
| reference.md | 6090a9ddf11fabb7888a03b39bca1c19d6105b0f8dabc5137b661f3d89fd0fcb |
| ros_entry.py | 4b94b0191dfcd42fd1199a1acfbe9c2271c84e3fc0296b102d2809d571bb4ecc |
| ros_lab.sh | 2856b4ea8cdbb4362fc9c1ecab55dd80cd0a8c050a15d7189924d1fcec5ecd9b |
| ros_ws/src/gazeboteach_l23/launch/arm.launch | 384c00ae62fe5927e42e7d3681858d59f2967661dda4a84ed8d43d8b61df633d |
| ros_ws/src/gazeboteach_l23/package.xml | 0e3072b2b36c60de09c41e1a49daf5708dead4ef47e1f43293a47ae9afd045b6 |
| ros_ws/src/gazeboteach_l23/scripts/common.py | abf09baa1dd573e6201939ce18c075fc35dfd5d66e4e2d247f31d1a0f768e9e4 |
| ros_ws/src/gazeboteach_l23/scripts/description_info.py | 24f60ebe3c321f2686a07c4218d383dc26dda1817310aacc683c7cba21cf9415 |
| ros_ws/src/gazeboteach_l23/urdf/arm.urdf.xacro | dfd6b3c7a2611296ddd67d83c52d1373ca26324af490499a34d2194509db1e86 |
| ros_ws/src/gazeboteach_l23/urdf/beam_macros.xacro | 754f9d0556190da974f3d26ca0d7373637a1c1629a81d2805618052a44cb9bac |
| verify.py | 2f4b1e6b051ef25080a7d94ddd1a28af5551c31bbade4e9c77adf0886ab3021d |
| world.sdf | 082bf84f7902a0bf695a0e01bdfe36d3a6db0ca4d0a9c9f126e619a1c949b91a |

| 正式原始报告 | 现存归档路径 | SHA256 |
| --- | --- | --- |
| baseline | .deploy/lesson23-formal-artifacts/baseline.json | e5e4a428a9b00b4820c05768bc21cd4a4f9140078766bfce51137424f5cb0cf2 |
| length_only | .deploy/lesson23-formal-artifacts/length_only.json | 03f690d05d7f9ffb0ef84629a7f67a69037e9115408ecb3d51cbe44e7ab70a4f |
| lump_only | .deploy/lesson23-formal-artifacts/lump_only.json | 21c9b028f3ade3ad56d11a50239849590ccd6ec84d84422cc52290e38e50879c |
| exact_inputs_and_properties_repaired | .deploy/lesson23-formal-artifacts/exact_inputs_and_properties_repaired.json | d47827f64ff9f8d5038e69e1de29552e25c3229db12cd45f413d81d2e6330701 |
| saved_without_reload | .deploy/lesson23-formal-artifacts/saved_without_reload.json | 065d5fed43a4445e3a78a282b3af2822eb54da7ff834671ba1d34bb4fefdf2f7 |
| generated_artifact_drift | .deploy/lesson23-formal-artifacts/generated_artifact_drift.json | bd9ecbd22123f7ea450ee2ed7bdcdea0b90261c477a8c4568d27080867813f6d |

报告SHA：v2formal a0fd4a79db4c52f6cd6af8765b656a8447d2a159f41b8adb7755bb103af296dd；UI dceda0e51e6cd4a9ca73a61faa76bdea3fa48bcee094066a26372868ebe453d0；失败v1 7be811dc6a08815522fb3c1e11d123050a479a98aec0382e97a311bcc9b0dcbe；原点/COM诊断 8401209b27976b3154d5bead7eac2851ac3662213abec17ffd8932d5ca448023。独立test SHA 66aa29aa407e72c7e9d8c4b459a76fc7ec9aa6311bfdb7fdd9e4be985ef7d0a6；UI源码SHA 12cf64d65a476946a9b9bd907aed419c1c2ab0a5136a48371e66ef50171c89a0；实际runtime.py SHA e25447b2b2b4f4aece99e25568ac553de20fc46255b687fec6f166cce33a143e。生成物原字节与SHA、准备记录、原生碰撞文本、服务查询和日志均有现存归档；报告里的旧/tmp源路径只作原运行身份，临时Storage已删除。

原VALIDATION_LOG前130445字节SHA aedbd83e2f8a2ff0fbec55f891484690fbc036522a552cbaefd367db474b1736完整保留；原Handoff23386字节正文与2026-10-29尾5054字节保留，全部LF。完成L23独立提交、Windows中转推送及远端核对后进入L24；当前预交付主23课+091/092为25项目，L24–L37共14门及G02尚未完成。

依据：[URDF与Classic](https://classic.gazebosim.org/tutorials?tut=ros_urdf)、[Noetic Xacro](https://github.com/ros/xacro/tree/noetic-devel)、[SDFormat转换](https://github.com/gazebosim/sdformat/blob/sdf9/src/parser_urdf.cc)、[Noetic ROS API](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp)、[Classic11.15.1 ODELink](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ode/ODELink.cc)。


## 2026-10-31 L24：真实关节状态、TF、RViz与模拟时钟

tests/lesson24_acceptance.py正式PASS 59.643s：3fresh、6scenarios，四正向baseline/rate_only/prefix_only/exact_inputs_and_parameter_repaired及两个真实负例全部通过。20课程源全程不变并与当前冻结SHA相同，3个token清理为空、cleanup_errors为空。正式运行未做图形渲染检查；下列数据只统计正式四正向，不混探索、UI或独立RViz。

L24流程：私有Master先设/use_sim_time→Xacro/URDF/SDF及SHA→现有Gazebo中一次性spawn tf_arm→系统Gazebo JointState插件直接读取真实Position/GetVelocity→RSP按同一URDF计算动态TF并latch固定边→tf2/RViz。JointState header取ROS时钟缓存，TF复制原stamp；世界服务读钟、接收monotonic与查询时刻另存。namespace不自动改变frame，Noetic使用tf_prefix；频率20/40Hz与prefix空/demo分别做单变量重载。原生joint名单、Gazebo实体和模型尺寸不随TF前缀变化。

| 正式正向 | tf_prefix | JS数 | header跨度s | header标签Hz | 原始事件 | 已匹配动态TF包 | 世界回退s | clear增量 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| baseline | (空) | 21 | 1 | 20 | 1973 | 37 | 4.742→0.15 | 1 |
| rate_only | (空) | 41 | 1 | 40 | 2073 | 76 | 5.691→0.15 | 1 |
| prefix_only | demo | 21 | 1 | 20 | 1962 | 37 | 4.73→0.15 | 1 |
| exact_inputs_and_parameter_repaired | demo | 21 | 1 | 20 | 1932 | 37 | 1.856→0.15 | 1 |

正式共保留7940条原始四路事件，独立oracle匹配187个动态TF消息包；静态消息8包，非测量边界未匹配TF为0。TF每条保留原stamp，包顶层stamp为null；JointState保留effort空、frame_id空与实际/gazebo authority。接收event_id/monotonic顺序未排序改写。频率来自原JointState header标签，不是额外获得的native采样戳；rostopic hz默认接收回调ROS读钟，--wall-time才墙钟，二者与header统计分开。

三姿态×四正向共12个稳定姿态、48次world-link独立FK/真实Gazebo几何比较，另各姿态核base_link→tool_link。课程位置/角误差门限1e−5，独立oracle2e−5；不拿相同公式的两次计算替代物理回读。每轮pause至少.15墙钟秒而世界/ROS时间不前进；真正1tick=1ms、100tick=.1s，单步是否有JS取决发布门限。late-static实连latching=1；错误lesson24/base_link查询留下真实LookupException。

验收先保存入口状态，再受控继续取流；暂停中做真实1/100步、late-static订阅，随后由同一listener/Buffer跨真实reset_simulation，记录回退与原clear调用，固定边继续可用。结束清本课任务，先在原q/pose零twist下真实推进150tick刷新TF，再暂停恢复原pose/twist及pause；速度setter按实际COM补偿v_COM=v_origin+ω×R·COM。世界时间、队列与TF历史不回填。新paused客户端ready只做authority/参数/模型检查并注明动态观察延后；手动clock-reset缺旧epoch时先实推150tick预热，再reset+150tick，以中间原始clock事件证明回退。

正式四轮均记录原listener/buffer身份不变、真实clear增加及新时间段lookup，未重新构造缓存伪称自动恢复。恢复回读最大误差：position_m=0、rotation_rad=0、linear_m_s=0、angular_rad_s=0。prefix场景只给arm2小的非零线/角速度并设置两个合法关节角，其余links各自保存快照；独立前后快照再次核对状态与pause。新RViz暂停时先收固定TF，动态链等实际推进；reopen不偷推进世界。

| 负例 | 真实拒绝原因 |
| --- | --- |
| saved_without_reload | RuntimeError: Saved model inputs differ from the loaded world: reload/restart required |
| live_rsp_parameter_mismatch | RuntimeError: Bounded URDF spawn/property readiness failed: RuntimeError: Actual RSP tf_prefix parameter differs; reload required |

保存40Hz未重载、live RSP tf_prefix改wrong_prefix均在实验控制前拒绝，报告trace为空且无stream/pose/step/reset；独立paused世界时钟保持。原输入/参数恢复后再次全链通过。

真实Qt 5.12.8/xcb桌面13项PASS 24.076s，含实际启动、两PTY、pause/step/continue、验收按钮、练习保留/参考、记录、截图和关闭清理。UI内验收4.923876202s只作UI证据，不计入上述正式四轮。独立tests/lesson24_rviz_acceptance.py --tf-prefix demo PASS 13.488071422s：旧PID175047退出，新PID175133不同，Gazebo及两终端仍存活，session.stop后无归属进程。render_check_automated=false；根代理已逐张查看原始Qt/Gazebo/RViz主窗与重开截图：26项目和中文正常；demo/world的Global Status Ok，蓝橙2R、球工具和TF轴真实可见。Gazebo截图含实际RViz浮窗叠加，不声称无遮挡。

最初RViz探针v1/v2/v3均失败并保留两端.deploy/lesson24-rviz-v1/v2/v3；它们是操作者PGID设置、窗口遮挡及选择splash而非主窗的问题，不计作课程通过。v1归属manager stop后清理为空的原始cleanup.json另存。库存RViz1.14.26另有真实Noetic EOL模态；课程manager仅在RViz子进程环境副本设官方DISABLE_ROS1_EOL_WARNINGS=1，避免阻塞课程窗口，未改宿主环境/驱动/渲染路径。最终正式/图形均使用同一20源SHA。冷paused新客户端clock-reset补验证9.722044306s另存lesson24-cold-cli.json，真实pre_reset_observation、clear_events及退出无残留；独立于六场景，不追加正式计数。

| 正向 | settings SHA | URDF SHA | SDF SHA |
| --- | --- | --- | --- |
| baseline | 2c4298e800db893aa87cdd870e2c475f6cb6ffdbe90715120a362f41e094470b | 2402e39181c778c1a8ca614f7db423a1279cb2cb46ed4c9ad6b2ca8e5760e1f6 | 115c659f7b070442404e1bcae478cb99ef99185b1379948faffe9671fde1a76d |
| rate_only | ad23d61b874d2b8ca748f08efbb5354ac744c16f64d8f36f79e26e9502ba072a | c8709477f2ff5d0e55998f3876af0a3859fe37279cffe0ea99cfe25891dd97b4 | dca8e33bc154797fb6d26d1844971b08979e8a6357d79e86c15f259cbeb04fd7 |
| prefix_only | 966005bcec2c5f0ff9572dbbe6f4bf2ee1113453ba5fb3cfee81c67bada81d7a | 962f3ceb54f268e6d5db1e84342d7e9bd6a5391a1f95605e30acaef4aa496f60 | 115c659f7b070442404e1bcae478cb99ef99185b1379948faffe9671fde1a76d |
| exact_inputs_and_parameter_repaired | 966005bcec2c5f0ff9572dbbe6f4bf2ee1113453ba5fb3cfee81c67bada81d7a | 962f3ceb54f268e6d5db1e84342d7e9bd6a5391a1f95605e30acaef4aa496f60 | 115c659f7b070442404e1bcae478cb99ef99185b1379948faffe9671fde1a76d |

原始证据：.deploy/lesson24-formal.json及-formal-artifacts保留六场景raw、生成URDF/SDF、输入准备记录、独立前后服务快照和日志；.deploy/lesson24-ui.json、.deploy/lesson24-rviz.json及lesson24-rviz-final.rviz/.log/-runtime.log保留GUI配置/生命周期。正式与主UI的临时Storage已清理；独立RViz的.deploy/lesson24-rviz-*及冷CLI补验的/tmp/l24-cold-cli-*调试数据保留。复核使用现存归档，报告路径还记录原运行身份，不保证旧临时路径长期有效。三个发布图为docs/images/lesson24-ui.png、lesson24-gazebo.png、lesson24-rviz.png，重开原图另存.deploy/lesson24-rviz-reopened.png。

报告SHA：formal 1ddcd31b71d5f2991ea3edc48cf40f27f3e115c747dbfaed73a29713c1dca3ff；UI 52d061ddedc34d00e11aff8c5b46fd44a9285d3a03c1c1e9e658cef58fdc997b；RViz d871c774fd8533123654b832180a9878d4d42fd3333d049368c1e69e2807daf5。测试SHA：lesson24_acceptance.py dc4036f46210ba783ac507863c623cf1894ea676dcb3733ef0dfecb695a09b85；lesson24_rviz_acceptance.py 6e6a12e71d8f8fd55a2959640071ded297fb74fb6e698e63d4e30ef859aa13d7。完整20源SHA在formal.source_sha256与.deploy/lesson24-frozen-source.json，updater同时核当前源码/两test/三reports/四原图与输入字节。

旧Validation前144813字节SHA 60fd8953ddfaa570f472fe2d7d60ef6e48e5f35ec71403df3fcca0085146b2c1及Handoff原25944字节完整保留，历史尾5054字节SHA 452200908928e3a6eb8753ba1665862b07d2e489ba45835af6bbfe222451728b不变，全局文档LF。完成本课独立提交、Windows中转推送和远端核对后进入L25；当前预交付24主课+2选作为26项目，后续13主课及G02尚待完成。

依据：[Noetic原生关节发布插件](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_joint_state_publisher.cpp)、[RSP时间门控](https://github.com/ros/robot_state_publisher/blob/noetic-devel/src/joint_state_listener.cpp)、[tf2倒时处理](https://github.com/ros/geometry2/blob/noetic-devel/tf2_ros/src/tf2_ros/transform_listener.py)、[StaticCache](https://github.com/ros/geometry2/blob/noetic-devel/tf2/src/static_cache.cpp)、[RViz EOL官方开关](https://github.com/ros-visualization/rviz/blob/1.14.26/src/rviz/noetic_eol_dialog.cpp)。


## 2026-10-31 V1 发布候选：本地检查完成，设备连线阻塞

- 正式已发布基线：L24，提交 048e4cedbb5edc78720a4c88e73252ab90675f1e；本次不修改课程源，也不推进 L25。
- 新增环境报告、启动逐课弹窗、首轮确认/自动启动门禁、Docker 配置校验与预留后端、固定提交自解包安装器及 V1 原生/Docker 指南。运行算法见 framework-flow.md 的 V1 章节。
- Windows 本地运行 python -m unittest discover -s tests -q：104 项，101 项通过，3 项平台条件跳过，退出码0。包括21项兼容/后端测试、6项 Qt 状态测试和2项实际执行的打包检查；Linux安装/包装器/卸载检查尚未运行。原始日志与 JSON：.deploy/v1-windows-tests.log / .json。
- Qt 合成场景分别模拟完整 Classic/Noetic 和仅 ROS2/现代Gazebo；检查26行、首次确认、实验禁用/正文可读、自动启动门禁、Docker非法配置不保存、合法配置保存但不执行、重复查看报告不重启实验。Qt为Windows离屏，截图 .deploy/v1-environment-synthetic.png 经查看可读；不能作为 ikaros 真实桌面验收。截图采集显式注册本机中文字体，不修改产品字体逻辑。
- 打包隔离 Git fixture 验证固定提交、26课程清单、逐字节可重复构建、权限、私密/生成文件排除、摘要和篡改拒绝。真实应用候选载荷的校验结果在后续候选归档中记录；尚未声称真实Linux安装成功。
- 本机实际以太网状态 Disconnected、LinkSpeed0bps；192.168.50.2 SSH两次握手/初始远程检查均超时，ping2次丢包。未更改路由、SSH策略、驱动或系统。设备恢复前，独立安装目录中的 --list、--diagnose、--gui-check、环境弹窗验收、L01/L24真实图形回归及首次SDK构建仍待完成。
- Windows Docker CLI存在但DockerDesktopLinuxEngine管道不存在，未启动daemon或下载镜像；文档Docker方案尚未端到端验证。
- 候选保存到 wip/v1-environment-release；不发布 v1.0.0 标签或安装资产、不把未验收候选合入正式 main。后续必须先恢复 ikaros连接并完成上述检查，再核对正式提交/标签和上传资产的 SHA256。V1发布后按用户指令暂停，L25–L37和G02留待后续。


候选保存补记：代码提交1f825db6705de0f168e0979f7be9e94c7504109f已通过Windows推送到wip/v1-environment-release，并以ls-remote精确核对；GitHub main仍为L24的048e4cedbb5edc78720a4c88e73252ab90675f1e。固定该候选提交构建436文件/26项目、安装器21116658字节和tar21103697字节；候选内嵌Python的--check以及SHA256SUMS全部通过，未执行Linux安装。安装器SHA256 67072677729abe795a3db56a5e0557842d2e877bdccb4743844c39fbe86c48a3；tar SHA256 dc9445381bf798a9bc184ad2223b9282ea1d5bbf0bd5d8a090d2a79a7a725ef6；manifest SHA256 07e2c7db3085d22de6cc5212927346ca5521c8676adaeadb3f59271014cc7c07。源与资产尚未作为正式V1发布；后续必须从最终实机验收提交重新构建。具体恢复顺序已写入SESSION_HANDOFF。


## 2026-10-31 V1 实机框架与独立安装验收

设备以太网恢复1Gbps后，严格SSH核对到ikaros-wh；原main为L24的048e4cedbb5edc78720a4c88e73252ab90675f1e，候选历史88c16bd完整中转，未覆盖原历史。最新授权继续全部课程和V2，V1后不再暂停。

106项Linux单元/Qt状态/安装检查全部通过、无跳过；最初104项同样全过。新8项Qt测试覆盖旧报告重开、延迟检测/旧回调、新结果确认及全局探测失败。发现并修复重新检测时旧报告可清门禁的竞态：active期间禁用旧结果，关闭旧弹窗后探测，新结果重新pending并须确认；异常为incomplete。实际桌面探测Classic11.15.1/Noetic的26项目均满足声明，合成ROS2/现代Gazebo则26不兼容但教材可读，Docker合法配置只保存不执行。源码--gui-check观察真实弹窗与主窗口，root查看中文原图，发布图docs/images/v1-environment.png。

候选88c16bd真实安装到含空格的独立前缀，全部436文件SHA/权限复核、--check、重复安装、实际bin/gazeboteach入口的--diagnose/--list26通过。此处不将旧课程记录声称为新版安装包全课程重测。代表性实际检查：

| 检查 | 实际结果 |
| --- | --- |
| 源码L24桌面 | 13项通过，24.226s，包含新启动报告确认 |
| 安装目录L01 | 实际Gazebo/双PTY终端/落块验收通过，14.327s |
| 安装目录L12 | 实际Transport受力滑门验收通过，19.322s |
| L12独立空SDK缓存 | 启动前确认XDG_CACHE_HOME不存在，真实g++生成407904字节client和484984字节plugin；GUI/双终端/行为验收通过，29.127s |
| 安装目录L24桌面 | 13项通过，24.079s |
| 安装目录demo前缀RViz | 新/旧PID、重开、服务端/终端存活和退出清理通过，13.516s；两张真实原图均由root查看，RobotModel/TF与Global Status OK可见 |

安装探针首次在无.deploy的发行目录暴露维护脚本路径假设（启动前FileNotFound）；第二次归档路径同样暴露在finally。两次失败日志分别保留v1-installed-rviz.log与-v2.log，应用源载荷未改。维护探针改为系统临时练习与显式report父目录归档后v3通过；这是测试便携性修复，课程源与物理阈值未改变。源码tests/lesson24_rviz_acceptance.py因此产生新SHA，旧L24原验证历史不回写。

汇总.deploy/v1-acceptance-summary.json含10份实际报告SHA；已检查4个报告可提取token均无残留。最终资产需从修复后的正式提交重建，并在新前缀再核对实际入口/GUI与清单；最终源提交、资产SHA和公共下载核对结果写发行附带V1_VERIFICATION.md。Docker指南未端到端实测，自动执行仍明确预留。新版本完整课程尚余13门主课及G02，不把V1称为37主课全量完成。


报告摘要：

- v1-environment-ui.json SHA256 15ffbda240c49a27ba6a24ffbddb6d6f407348cecdbc68bd9cc7b7807179d9ac
- v1-source-gui-check.json SHA256 55b19f4e3e76ec2231d69af90473abce67f3555907316f8e582477e5afeee1a6
- v1-source-l24-ui.json SHA256 05b8607596322832cbee9fb912b6f1b7f8d7004678a6950207ad6e5176cf03d4
- v1-install-report.json SHA256 a60161229a1dca8b49537807d88bb19110d97971110e2f94ed86bf881b3caf0e
- v1-installed-gui.json SHA256 2b4329641c1f587ef90ad7dadd684daa125278f55f578b3ca3b068227018e7b9
- v1-installed-smoke01.json SHA256 41afd2b3d0dcaaf331fab90ad3ed272377e3656f28f65c04ebc33aac3215f2d7
- v1-installed-smoke12.json SHA256 0e64764385df3e1f07d8c4bc066d1c613b8e52d2c57f047e542649d019c32cb5
- v1-installed-sdk-fresh12.json SHA256 f2b2eb588f314d9417aca7766231c396a563c6734e4a09645338e06b20ae98b7
- v1-installed-l24-ui.json SHA256 c5fd0d42686ab3c6e466450dd66883b83ce45318921d1f925149f37209e5b4df
- v1-installed-rviz.json SHA256 c38641a851653fef81a04b8f6b8cd81767b9fbfff9b46034125f8aed7f760489


## 2026-10-31 L25：ros_control真实力矩、生命周期与限位

正式3个全新私有场景/6组检查通过96.916s，真实桌面13项36.707s通过；root查看两幅原图，27项目中文教材及蓝座橙杆均可见。GUI、两个独立PTY终端、暂停/单步、验收、编辑/持久化/重开与退出清理均实际检查。

本机无effort_controllers且sudo不可免交互，未改系统包。Windows从官方ros-controls/ros_controllers标签0.22.0获取提交c2348e85abf35cf33cf654a84d61db206abe9a73，归档SHA256 71d2a7c15520960c4540573d762d719e39b5436af5fd1a52c48a19b1d3c83fd9；保留17个上游原文件摘要及BSD许可，省略上游测试、关闭CATKIN_ENABLE_TESTING。个人副本离线CMake构建，派生个人XML使用唯一libgazeboteach_l25_effort.so，避免主机同名库优先加载。三个新场景逐一核对自有gzserver的/proc/PID/maps、实际文件inode/device及ELF SHA，不以声明代替加载证据。

算法：校验输入/源SHA→个人overlay→官方C++编译及派生XML→私有ROS/Gazebo与/use_sim_time→加载URDF/transmission/gazebo_ros_control→manager启动joint_state与主servo、只加载备用servo→配置/实际反馈就绪→位置目标→官方PID原始力矩→DefaultRobotHWSim限幅/SetForce→ODE积分→JointState/ControllerState与独立Gazebo查询。转杆轴惯量.0303+1*.3²=.1203kg·m²，p4/d.35/i0，力矩2N·m、名义速度4rad/s、角度±.8、控制周期.002s。

先通过真实命令形成q=.24317734932503843rad、qd=1.8006897688207277rad/s并暂停。保存未重载p故障在控制前拒绝，世界时间/q/qd/控制器不变，精确恢复原字节后重新验收。冷暂停ready只作静态确认；实验快照为观测保持目标而进行的短时真实推进另存原始证据。最终物理q/qd恢复误差均0，原暂停及三个控制器状态相同。原initialized备用实例被使用后变stopped，工具实际卸载/重载并确认消失→initialized；恢复运行组合/目标后再暂停，以实际COM补偿恢复link pose/twist。实例/PID/订阅身份、时钟和消息历史不倒回。

真实100步仅发送一次gz脉冲，有界回读12.308→12.408s；100*.001=.1s，独立世界查询同为.1s且仍暂停。ROS服务路径参数与请求name字段分开；SwitchController.timeout按Noetic实际float64=2.0发送。不重放非幂等步进来凑结果。

| 真实场景 | 结果 |
| --- | --- |
| p4/d.35/目标.45基准 | 峰值速度1.996250386rad/s，终段均值.453522171rad |
| 仅p改2 | 峰值1.135341459rad/s，终段均值.450185747rad，其余输入不变 |
| 仅目标改-.3 | 正确负向响应，终段均值-.302035544rad |
| 超限目标±1.2 | 实际set_point为±.8，关节趋近真实硬限位 |
| 生命周期 | STRICT同资源冲突/未知控制器均拒绝；合法soft切换独占真实Effort资源并产生运动 |

9段独立运动保留所有原始消息/查询。JointState/GetForce执行回读峰值≤2N·m；限幅前ControllerState.command原记录可达约176N·m，不当作实际执行力矩。超限段速度峰值4.010868rad/s保持原值：名义速度约束经努力接口离散调整施力，并非每个物理瞬时值直接裁为4；没有修改消息或隐藏超调。

正式v1发现inactive initialized→stopped恢复遗漏，q/rate/pause本已恢复；补实际卸载/加载后v2发现维护包装器name重名，修复后v3通过。两端.deploy/lesson25-formal-v1与-v2保留完整报告/raw/log，独立测试及阈值未改。三个早期GUI原型依次定位roslaunch默认CWD=ROS_HOME、SwitchController.timeout类型，修复后v3全闭环28.559s。UIv1通过35.703s、v2在包装器故障时失败7项；最终v3才是13项36.707s证据。

36个课程/依赖源文件与独立测试冻结，模板/上游原文件保持，逐文件摘要见formal.source_sha256与official_controller_source_sha256。三个formal token和UI关闭均无残留。临时Storage已清理，原运行身份路径留在报告中，应使用现存raw归档复核，不宣称旧/tmp路径仍存在。

报告SHA256：formal b8e278d9596933d568fba09afaf98a3450f7eda0f2ec87911a06e6536d46d0c6；UI 772ad90e43c5f8802a56d4fa14c5668fb0d81a0b1f6dd9c0bd0902fdcbadac25；独立测试 0cb08c9bf75cfd729f8f30ad370a42ad591e132963a2dea268c3105798340dc4。

另以独立提交ddd1b917e8050b41a55be9249df2468463a0c521修正Qt测试固定26项目假设，按实际catalog数量与逐行ID核对；27项目下106项Linux检查全部通过、无跳过，日志两端.deploy/lesson25-dynamic-catalog-tests.log。

V1已发布：v1.0.0源384c74bf5d96e34f31d90cd823d42f2ffea1829e、437载荷文件，8附件按摘要下载核对，仓库保持私有。API令牌仅Windows内存、仅官方API，CDN不转发；最终新安装入口/真实GUI已复核。V1_VERIFICATION.md区分候选代表性回归、最终包检查及未验证Docker。操作报告SHA256 b6d59cb1888151127d985f5b3d92e54f39d03ba29c6c23a243b2d97ca60a2b06；当前继续全课程/V2，不在V1后停止。

依据：[官方0.22控制器](https://github.com/ros-controls/ros_controllers/tree/0.22.0/effort_controllers)、[Noetic硬件实现](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros_control/src/default_robot_hw_sim.cpp)。完成本课独立提交/推送/远端核对后进入L26。


## 2026-10-31 L26：真实ModelPlugin、原生姿态和生命周期

三组全新私有Gazebo场景正式验收68.277s通过，真实桌面13项35.770s通过。root查看两幅原图：28项目中文课程、蓝色固定台座、青色转子和橙色非对称方向标记可见。自动物理验收不宣称代替图形检查；独立原生姿态证明运动，单张截图不单独证明动画。

算法：校验个人参数与源摘要→根据Gazebo11 SDK/编译器建立本地C++缓存→生成个人SDF参数及配置摘要→私有gzserver加载ModelPlugin并依次Load/Init→WorldUpdateBegin读取真实关节速度→SetForce(drive−drag*omega)→ODE积分→原生pose/info及entity_info独立测量。插件不调用SetPosition/SetVelocity制造运动。Izz=.015kg·m²，基准drive=.03N·m、drag=.015N·m·s/rad，终速2rad/s、时间常数1s。

| 全新场景 | 角增量最大残差(rad) | 原生终段均速(rad/s) | 解析终速(rad/s) |
| --- | --- | --- | --- |
| 基准 | .001760225496 | 1.911671918 | 2 |
| 仅驱动力矩改为-.03 | .001760225496 | -1.911671918 | -2 |
| 仅阻尼改为.03 | .000958394617 | .998038562 | 1 |

独立测试逐条解析未经修改的原生protobuf文本，以实际entity id识别转子、读回质量与惯量、解包四元数并按源时间比较解析角增量及有限差分速度；角度阈值.025rad、差分速度阈值.06rad/s保持。插件速度只作回调仪表交叉检查，其最大偏差.014448rad/s，不作为独立物理真值。三组分别保留297/293/301条有效原生位姿及全部原始JSONL，实际私有ELF映射按inode/device/SHA验证。

基准另验证保存参数但未重载时拒绝执行且世界不推进、精确原字节修复后就绪；暂停不产生普通更新；只发送一次100步控制，实际推进.1s和100次更新，原生转角增量.198767872880rad、解析.198764180888rad。未反复发送非幂等步进凑结果。

实机发现模型复位保留内部关节角度，清零速度、增加Reset计数并清零epoch更新数，Load/Init计数保持。正式v1错误期待零转角而失败，原始数据保留两端.deploy/lesson26-formal-v1；根据原生实体与Classic源实现修正为严格的复位前后转角保持，正式v2偏差0、速度0。没有通过写位姿强制归零，也未放宽解析运动阈值。复位管理可推进一个1ms世界时钟刻度而不调用普通更新，本次两次复位均实测+1000000ns；使用实际复位epoch并保留原钟，不回填时间。

两个原型缺陷已修正：WorldReset protobuf的all默认true，显式设置all=false/time_only=false/model_only=true；SDK异步发布器局部对象过早析构导致自身线程join，改为主线程持有receiver、publisher、subscriber及node，先transport::fini停止并join，再释放对象，不以睡眠或强制退出遮盖。修正后原型39.438s通过，最终正式与GUI均无此错误。

生命周期缓存applied_torque_nm=0是占位而非力矩测量；新增torque_command_issued仅在实际update施力后为true。模型复位不恢复进入时的转角/动量快照，演示结束再次执行真实model-only复位并恢复原暂停状态，若原本运行则继续受力；完整重载建立新世界和初始角度。教材和只读答案均明确说明上述流程与边界。

11个课程文件与独立测试冻结；模板未改，三次formal token及桌面结束均无本应用残留。raw与运行证据保留在两端.deploy，旧/tmp身份路径只作当时身份，不宣称仍可访问。后续G02需完善SDK依赖识别并澄清通用“重置模型状态”按钮语义，不以全世界时钟倒回替换模型复位。

报告SHA256：formal 656e6947da86fa7cc156b649bd2800db05dd0928abc2aa03cf5780c9f037f838；UI 80a1f90fae6812b875e6fcd0bbfaa9bfa1f4ab037ed041a7f5cd16479762ba4b；独立测试 2d9c32a57a14cad2aaa5e083956b856683760466e6b49ff3507c6943482a23e0。

完成本课独立提交、Windows中转推送和精确远端核对后进入L27。V1发行快照不变；当前继续完成全部课程及V2。


## 2026-11-01 L27：WorldPlugin异步投料、实际实体和安全回收

四个全新私有场景正式验收126.227s通过，包含基准count3/height1.2、仅count5、仅height1.8，以及count1/height1.23456789的额外精度边界回归。后一场不是教学单变量对照。共12个真实方块，各自原生位姿的三点差分重力中位数均9.799999999999962m/s²；每个差分值9.8±1、中位数9.8±.35及持续支撑速度<.01m/s阈值保持。独立原生实体核对实际质量、惯量、ID、出生高度和支撑中心z≈.18。

真实桌面13项66.586s通过，包含GUI、两个独立PTY、暂停/单步、实际验收、练习持久化/重开/编辑和退出清理。额外图形探针20.311s通过，原生Scene确认实际3块ID16/22/28，root查看未合成的原图：橙蓝橙方块、绿色外部哨兵和台面可见；同时查看29项目中文工作台。默认verify会清空自有模型，所以另外保留投料后图，不把回收后的空场景当作渲染创建的证明。

算法：参数/源摘要检查→个人SDF及高熵实例前缀→按SDK/编译器/两CPP源构建键并fcntl锁定缓存→离线C++编译及原子安装→world下加载WorldPlugin→Load/Init与原生RPC就绪→transport回调只登记batch/clear→物理线程WorldUpdateBegin按模拟时间.35s提交离线SDF factory→实际ModelByName确认并登记ID→ODE真实重力积分→独立原生位姿验证下落及持续落稳→clear等待所有在途请求出现后仅删除匹配ID→实际消失后idle。complete只代表目标实体均出现；batch工具另以末.25s原生窗至少8样本/跨度≥.2s、z范围≤.003m、每段|vz|≤.03m/s及距支撑中心≤.008m确认落稳，最终暂停再读实体。全过程不写测量值或用插件计数代替真实物理。

Clear只管理本实例登记的实体，保留外部哨兵完整原生内容；已出现的同名替换若ID不同则拒绝删除并报错。首次名字登记依赖高熵前缀及插入前冲突检查，不承诺抵御恶意并发抢占同名。基准实测在暂停世界只发一次batch、推进首tick后requested1/observed0，clear随后等待真实实体并回收，再推进1模拟秒确认没有迟到模型。只发送一次非幂等控制，不重放来凑结果。

暂停前后时间7675000000ns、iterations7675均保持；真实100步后为7775000000ns、7775，恰好+.1s/+100次普通插件更新。model-only复位后7776000000ns，实际+1ms管理tick、实体ID保持、WorldPlugin.Reset不调用；全世界复位后实际clock0/iterations0、WorldPlugin.Reset增加1而Load/Init保持。全世界复位本身不会删除动态模型，本课Reset只标记延迟清理，恢复更新后才实际回收；模拟时间和外部动力学不作为快照回填。保存参数但未重载的拒绝及原字节修复另行验证。

SDK9.10精度：formal v1(16.496s)和v2(25.841s)都在原生I=.00213333与数学I=.0021333333333333334的1e-10比较失败。实机微探针证实直接readString保留double，GetAsString仅六位有效数字，Root/Model.Clone经Param.Clone将I变为.0021333300000000001，高精度质量.5123456789012346变为.512346。World.ProcessFactoryMsgs的共同Root()->Clone使String入口也无法绕开；保留17位原始文本仅避免入口额外序列化。最终测试分别记录数学/SDK可传递预期/原生实际值，按float(format(theory,'.6g'))保持原1e-10实际核对，另记绝对差3.333333333e-9和相对差1.5625ppm。没有修改SDK、实际消息或自由落体阈值。

高精度边界实机通过：JSON原始1.23456789及其原字节保持，prepare另记factory_expected_drop_height=1.23457；predict分列请求/SDK高度与两种理论落体时间。课内model-reset只执行一次后，独立原生实体核对实际高度1.23457、所有ID保持、暂停、单1ms管理tick、Reset/updates不变，原高度误差1e-7阈值保持。

formal v3基准全部通过，但count-only启动及同版UI的stats客户端在已输出有效世界统计后偶发abort，boost thread报自身join。源码与上游维护者相同故障栈表明最后Connection引用可能在IO回调释放，从而IOManager析构join自身。针对本机运行GDB100次70.309s及仅加terminate调用栈诊断的原版200次55.887s均未复现，不能宣称已取得本机故障栈。原始两个abort、三个失败formal及对应源/raw/log保留两端.deploy/lesson27-formal-v1/-v2/-v3。

客户端兼容流程：初始化前主线程持有未连接Connection以保护共享IOManager→检查导出数据符号及其真实DSO全版本等于编译SDK的GAZEBO_VERSION_FULL→正常原生通信→transport::fini停止并join其run线程/清注册表→主线程调用public IOManager.Stop明确停止并join异步IO线程→释放node/pub/sub/guard。WorldPlugin教学本体不使用私有ABI，不改主机库，不吞非零，不用强制退出或睡眠替代join。首次保护检查因短版宏11.15与全版11.15.1不同而正确拒绝，归档safe-transport-v1后改用FULL。修正变体140次正常查询/控制和3次无Master初始化失败均正确join，55.761s通过；无Master仍返回1。私有数据符号兼容层只在本次Classic11.15.1验证，符号/版本不符明确拒绝；后续G02评估旧课同类客户端，不宣称所有SDK均已认证。

最终11个课程文件与独立测试冻结，四个自有gzserver实际私有ELF按inode/device/SHA核对，模板未改；formal四token、桌面、额外图形及所有诊断探针均无残留。临时练习目录已清理，历史/tmp路径仅作运行身份；持续原始数据保留两端.deploy/lesson27-formal-artifacts和figure-artifacts。算法流程、精度与复位边界已同步教材及只读答案。

依据：[Classic世界实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc)、[SDK9参数复制](https://github.com/gazebosim/sdformat/blob/sdformat9_9.10.0/src/Param.cc)、[原生IO管理](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/transport/IOManager.cc)、[上游维护者故障栈](https://gist.github.com/scpeters/ef9741628dfe58f681f6e46e413dcc5d)。

证据SHA256 lesson27-figure.json：b9d993ef7a7ee1c779e01959014d93b6618d7a20c975ea4426cf9a2ef9ec7b06。

证据SHA256 lesson27-sdk-precision.log：c47052fa74609e67d87faa72f96be10917bd5dd88861f543a998cc0eabc56fa3。

证据SHA256 lesson27-safe-transport-report.json：6b8207c06ceba9d6a37114c7bf2a0eaea50c07a454a196c3f584b131213b1356。

证据SHA256 lesson27-transport-gdb-report.json：6a0975e848fbf4becf40863035511e26fffbf3ea7b63984fb44bf6ca66e98132。

证据SHA256 lesson27-transport-trace-report.json：d9e2cbe64dc4435d9bf5242c23b74d897dc6aec68a9d1cc49ad23b50496c87eb。

报告SHA256：formal 59c168474da293bfb79b9c125ce953e3c862563ff77be8aa78da24045d7273b0；UI 1a078e7695448badaea03a4d9f45c9fab4095aa983eaa8d218e0359c959bdecb；独立测试 cf2048a97f3f93a1cc07859b731314239fba2fc772accf1803fbc2f756f0d507。

独立提交、Windows中转推送并精确核对远端后进入L28；当前继续全部39项目V2。


## 2026-11-01 L28：SensorPlugin原生测距、EMA和无回波

最终三组全新私有场景正式验收215.076s通过：基准threshold_m=.9/alpha=.35，仅阈值1.3，仅alpha=.7。分别318/329/320对原生LaserScanStamped与Any.STRING输出按源ns精确配对，共967对；每组五阶段原生源频率均20Hz。独立测试从未经修改的protobuf文本和Any原始JSON解码测量，几何和EMA历史独立重算，不把插件的previous_filtered字段作为递推真值。

真实桌面13项52.434s通过。root查看两张原图，30项目中文教材、蓝色扫描器、橙色静态目标和三束可视化射线可见；GUI、两PTY、暂停/单步、实际验收、编辑/重开/持久化和退出清理均通过。图形仅证明实际场景可显示，原生消息另行证明测距与算法。

算法：个人JSON/SDF及源摘要检查→按两CPP/SDK/编译器锁定并原子构建→sensor下加载SensorPlugin→检查真实RaySensor、Load/Init、设置active→保留ConnectUpdated连接→每次成功测量后读取真实Ranges/LastMeasurementTime→按[min,max)筛有限值并求最小值→首次有效值直接种子、后续EMA=alpha*x+(1-alpha)*previous→按filtered<=threshold输出near/clear→无有效值输出unknown及null并清空滤波→实际采样时钟严格回退时开启新epoch。弱sensor和共享State避免传感器/插件循环所有权及回调析构悬空；transport RPC只复制已锁缓存，不在请求线程持本地锁再取传感器数据。C++不生成测距或修改原始扫描。

| 真实目标中心x(m) | 有效束 | 原生最近距离(m) | 基准输出 |
| --- | --- | --- | --- |
| 1.4 | 3 | 1.2 | clear |
| .8 | 3 | .6 | 首帧EMA .99仍clear，随后near |
| 3.2 | 1 | 3 | clear，斜束越过方块半宽而无回波 |
| 8 | 0 | 无有效回波 | nearest/previous/filtered全null，unknown |
| 恢复1.4 | 3 | 1.2 | 首帧直接种子1.2，clear |

三束±.1rad、范围.1..6m、目标边长.4m，逐束按真实前表面和横向半宽验收。仅阈值改1.3时原生数据及EMA不变，初始1.2由clear变near；仅alpha改.7时移近首帧EMA=.78已near，对照基准.99，说明响应延迟和阈值的不同作用。原始无限值保留在原生protobuf，JSON显示null的同时记录raw_kinds，不把无回波当成零米或已确认的安全距离。

暂停先等待已到期工作排空，然后世界时间/迭代/回调保持；一次100物理tick真实推进.1s，实际另观测到2次传感器回调，不把20Hz异步回调数强定为物理步数。保存参数但未重载时拒绝且不推进，原字节修复后恢复。全世界复位实测clock0，重新取得62对新epoch消息，第一帧原生1.2与新种子1.2一致；epoch/rewind_count各增加1，SensorPlugin.Reset仍0、Load/Init保持1。本SDK的SensorManager重置采样时钟而非自动调用SensorPlugin.Reset，插件按实际测量时刻回退识别新epoch。

demo正常结束恢复目标完整原生pose和入场暂停状态，不回填EMA、计数、时钟或消息历史。手动near/far/noecho/home移动保持暂停，需resume才观察新测量；全世界reset将目标恢复初始SDF位置，区别于恢复入场目标。RaySensor.IsActive还会受原生扫描订阅影响，教材说明SetActive(false)不保证有订阅者时停采样，未把源码推理冒充实际主动停采样验收。

首轮独立测试210.290s三场景均通过，但GUI原型48.944s的课内验收器把invalid首帧previous=null硬接上一valid.filtered，误报EMA不连续。只修lab.py验收流程：当前有效且连续/同epoch才按前帧是否有效继承或重种子；当前无效则三项距离均null、unknown，不跨失效段接旧历史。C++算法与数值阈值未改，流程同步两MD；原型source/raw/log保留两端.deploy/lesson28-prototype-v1，首轮独立报告及源保留formal-v1。

最终独立测试增加自有stop-file请求，让三次连续watch均正常返回0并经过SDK IO stop/join，超时才作失败清理；不再用正常路径SIGTERM结束观察。沿用L27验证的严格DSO全版本/私有数据符号保护及主线程IO屏障，插件教学本体不使用私有ABI，未修改主机库。11课程文件与独立测试冻结，三个自有gzserver的实际私有ELF分别按inode/device/SHA检查，模板未改；formal三token及GUI结束均无残留。原始JSONL、生成配置及运行记录保留两端.deploy/lesson28-formal-artifacts，旧/tmp路径仅作当时运行身份。

正常路径的恢复已实际验证；verify在demo后的暂停/步进检查若失败可能保留暂停供排查，G02记录进一步评估该异常路径，不声称失败后仍恢复全部状态。

依据：[SensorPlugin接口](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/common/Plugin.hh)、[传感器更新](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/sensors/Sensor.cc)、[实际RaySensor](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/sensors/RaySensor.cc)。

报告SHA256：formal 8f85de7fa3bf5c7c3d4ec042d4c7b5117b83c3bc151d86cc631c6997a753d925；UI 648eb3368720eaa8076563c71ed46d839713e75c6ffe5e73dc0a1db59d108332；独立测试 03ded27e318a4a0ef05d0175d5d11808be1118e8910f4041260d328674db86dd。

独立提交、Windows中转推送并精确核对远端后进入L29；当前继续全部39项目V2。


## 2026-11-01 L29：VisualPlugin真实像素与碰撞隔离

三组全新私有Gazebo GUI场景正式验收49.098s通过，真实桌面13项31.409s通过；root查看31项目中文工作台及独立QScreen原始蓝色/放大红色画面，黄色参考物分离可见。正式视觉真值来自按本次gzclient PID/token选择的X11窗口截图，不使用插件生成的图片、属性回读或预期图作为独立视觉证据。三个图形场景顺序运行，随后单独运行桌面检查，避免两套自有窗口互相遮挡。

| 全新场景 | 目标/黄色参考像素面积比 | 相对基准 | 服务端碰撞(m) |
| --- | --- | --- | --- |
| 红色、scale1 | 4.610200364 | 1 | 1×1×1 |
| 仅改蓝色 | 4.661384335 | 1.011102331 | 1×1×1 |
| 仅改visual scale1.4 | 9.055191257 | 1.964164362 | 1×1×1 |

面积从原RGB的饱和色最大连通区测量，以不变的黄色参考归一；三组参考占原图比例均.006414510581。保留原像素，未替换、生成或重绘图像。颜色变式几何比.85..1.15、缩放面积比1.5..3.3阈值未改；透视/栅格实际测量不被强行写成理想1.96。三个自有gzclient分别核对实际ELF的inode/device/SHA，确认gzserver未加载该VisualPlugin；服务端原生实体、位姿及碰撞全内容保持。

算法：校验个人color/visual_scale与源码摘要→合并Gazebo/OGRE各组件/Qt SDK并按键加锁离线构建→server创建原始1米碰撞模型→client加载visual下插件→Load/Init与保留渲染连接→transport回调只排队并回queued序号→PreRender在渲染线程建立本visual唯一材质，设置ambient/diffuse/emissive、启用lighting、按绝对值设置局部scale→读实际属性并更新applied序号→真正渲染产生可观测像素。after_start只检查服务端，after_gui才等真实客户端并初始化一次课程相机；已有窗口保留后续用户orbit。目标beacon与侧方黄色reference分离，未修改服务端几何来制造视觉变式。

暂停世界下依次改蓝/绿/恢复个人模板，分别截图核对；render_count继续增加，而世界clock始终6322000000ns、iterations6322且paused。只关闭本次GUI后服务端继续，重新打开建立新的client_id，初始外观恢复；三次fresh及重开均实际Load1/Init2。Init2对应本SDK Visual.LoadPlugin在initialized后立即Init及LoadPlugins末再次遍历Init，教材要求初始化可重复，不把其他插件的Init次数套用。课程reset是排队恢复个人模板外观，不等同于原生模型或全世界复位。

教材中的相机PNG也来自真实像素：PreRender保存原CaptureData并连接NewImageFrame、启用读回→Camera.PostRender完成GPU像素读回后发出带buffer/宽高/深度/格式的事件→事件中static Camera.SaveFrame保存实际数据→恢复原捕获flag。只有真实非空图像事件才确认captured序号，不按渲染计数或固定睡眠猜图像就绪。该辅助路径由prototype3及最终桌面实际运行，独立正式验收另外使用QScreen。

早期问题与证据：首次编译依次定位OGRE基础/跨组件include缺失和VisualPtr的std/boost类型不匹配；prepare增加OGRE-Paging/Terrain/RTShaderSystem及Qt5Widgets，链接gazebo_gui。prototype1已打开GUI但SaveFrame报空图像，按上述真实事件修复；prototype2捕获新增UserCamera弱引用编译失败，按实机typedef修正：VisualPtr为std、UserCameraPtr仍boost。prototype3全部通过33.889s。首次独立正式20.201s发现实际两个方块全白，即使API回读红色[1,0,0,1]、相机及Init都正确；目标和参考原lighting=false，在本机渲染路径不呈现预期颜色，改lighting=true并保留专有RGB/自发光后正式像素通过。像素阈值未放宽，未把属性正确冒充画面正确。失败源/raw/log/截图保留两端.deploy/lesson29-prototype-v1/-v2与formal-v1。

11个课程文件及独立测试冻结，模板未改；三次formal token、GUI重开和桌面关闭均无自有进程残留。GUI、两PTY、暂停/单步、实际验收、编辑/持久化/重开和清理都检查通过。demo尝试恢复入场颜色/scale/pause，明确保留课程相机视角，不承诺恢复学生相机或回填渲染历史。原始独立PNG、原生实体和映射证据保留两端.deploy/lesson29-formal-artifacts，运行临时路径仅作当时身份。SDK客户端继续使用L27已验证的全版本/符号保护和主线程IO join，不更改主机库。

依据：[Classic Visual实现](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/rendering/Visual.cc)、[实际相机读回及保存](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/rendering/Camera.cc)、[VisualPlugin接口](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/common/Plugin.hh)。

原型3报告SHA256：525ed70745c40e5e074634f80f39f83e68eb9e163c7272a5ade224c0648cd59f。

报告SHA256：formal 23a6b54a2606098c46d49467a9bd288f95e0c27b88ff8895a13d49d49008167a；UI 37b8fca8c7d701abd643a91eaa25742a973924c26cbc0355ce9a995c9d441d76；独立测试 68d1400a533aa8b6ef00a8f4e8ad993b48e8a661bb415e620f50cee453d12af9。

独立提交、Windows中转推送并精确核对远端后进入L30；当前继续全部39项目V2。


## 2026-11-01 L30：GUIPlugin真实Qt按钮与世界控制

三组全新私有GUI场景独立验收42.976s通过，真实桌面13项27.356s通过。独立测试使用现有libX11/libXtst注入真实Button1单次press/release，不调用插件模拟click RPC；先激活本次gzclient、检查窗口PID/token、按钮全局矩形位于该窗口、鼠标最上层窗口归属祖先链，拒绝已按住的修饰键/鼠标。每次步进只点击一次，再查独立原生WorldStatistics和falling_cube实体，未用面板字段代替物理真值。

| 全新场景 | 实际tick/模拟时间 | 方块实际z(m) | 连续自由落体预测z(m) | 实际QLabel文本 |
| --- | --- | --- | --- | --- |
| N100、显示3位 | 100/.100s | 1.9505100000000002 | 1.951 | t = 0.100 s |
| 仅N改200 | 200/.200s | 1.8030200000000005 | 1.804 | t = 0.200 s |
| 仅显示改1位 | 100/.100s | 1.9505100000000002 | 1.951 | t = 0.1 s |

世界以-u暂停启动，方块初始质心2m、边长.4m、重力9.8m/s²；离散ODE积分与连续解析差分别-.00049/-.00098m，保留原值，在既定.012m容差内。显示精度没有改变实际物理结果。每组保留真实原始窗口PNG与按钮区域像素，root查看32项目中文工作台、Gazebo内蓝色面板和橙色方块；文本按Qt线程缓存回读并与独立原生时间格式核对，未声称独立OCR识别了文字。

算法：参数/源摘要检查→Qt5Core pkg-config host_bins定位真实moc及版本→生成元对象CPP→离线编译GUIPlugin及安全SDK观察器，按SDK/编译器/flags/moc/源码键加锁原子缓存→server原生世界就绪→client将GUIPlugin作为QWidget加入GL界面并Load/show→Transport回调只锁定缓存真实统计和待演示点击→Qt主线程QTimer处理请求、更新QLabel和读取按钮geometry→实际QPushButton.clicked槽仅发布一次WorldControl→面板禁用重复控制并等新的原生time/iterations/pause反馈→匹配后完成，超时只报错不重放。所有QWidget读取与修改位于GUI线程，时间不在点击后自行加到预期值。

课程自动演示通过RPC请求在GUI线程调用同一个QPushButton.click并走同槽；独立验收另以真实XTest鼠标证明按钮接线。基准还实点Resume/Pause，检查真实时间推进、物体继续下落和暂停反馈；只关闭本次GUI，确认服务器与暂停时钟保持，再开新client_id/面板，实际ELF仍仅映射到gzclient。保存参数但未重载时拒绝执行，原JSON字节修复后恢复。三组SDK映射按inode/device/SHA留证，server不加载GUIPlugin。

实机预编译纠正了两项依赖/API误判：/usr/bin/moc存在但qtchooser没有默认Qt，真正Qt5Core host_bins/moc为5.12.8，可直接运行，未修改宿主QT_SELECT；GUIPlugin及PluginT没有Init/Reset虚接口，删除了错误override及函数，未假造SDK生命周期。JSON init_count/reset_count仅兼容常量0，init_hook_supported/reset_hook_supported均false。真正生命周期为构造、Load、QWidget显示/定时器、析构。编译记录在两端.deploy/l30-sdk-preflight，原型GUI+两PTY完整闭环31.477s通过。

桌面另外验证GUI、两独立PTY、平台暂停/单步、课程验收、练习编辑/持久化/重开与退出清理。课程demo仅恢复入场暂停状态，不回填已发生的物理运动、时钟、迭代与消息；复位原始落高需平台重载或明确模型复位，不把按钮演示说成物理快照恢复。12课程文件(含Qt头文件)和独立测试冻结，模板未改；三formal token、GUI重开及桌面关闭后均无自有进程残留，原始鼠标归属、实体、统计、截图与日志保留两端.deploy/lesson30-formal-artifacts。SDK客户端沿用全版本/符号保护及主线程IO stop/join，不改主机库。

依据：[GUIPlugin接口](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/gui/GuiPlugin.hh)、[实际Load/show路径](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/gui/RenderWidget.cc)、[Qt信号与槽](https://doc.qt.io/qt-5/signalsandslots.html)。

原型报告SHA256：0d70417c2bcdd63edd3cddc3914f8c38f965a00d9d6ebfe2253fd8d1c90db1d4。

报告SHA256：formal 9f3be67827677ece7012274441e59c7a1570c72b3cdaf9c2cf78b40812a26f36；UI 91d71a21e04fcd88b23718022e10bbe211816509f130edda23723fed5ed9d53c；独立测试 9985ee3b58f26c6682a55685cbfa6212b2a440b92c9defd3d4f82a3ef4e212ea。

独立提交、Windows中转推送并精确核对远端后进入L31；当前继续全部39项目V2。


## 2026-11-01 L31：SystemPlugin进程边界、实际重力与采样

三组全新私有场景独立验收51.782s通过，真实桌面13项40.722s通过，额外stride1回归38.685s通过。root查看33项目中文工作台与真实Gazebo橙色落块。两CPP首版实机编译4.941/4.053s通过，最新源再次由正式prepare编译；原型GUI/两PTY完整闭环47.981s通过。

| 全新场景 | 200tick后原生z(m) | 连续预测z(m) | 真实记录数 |
| --- | --- | --- | --- |
| g9.8、stride10 | 1.8030200000000005 | 1.804 | 20 |
| 仅g改4.9 | 1.9015099999999994 | 1.902 | 20 |
| 仅stride改20 | 1.8030200000000005 | 1.804 | 10 |

实际落距比1.999999999999982，保持原值不强制写成2；原生ODE离散积分与连续预测残差分别-.00098/-.00049m。初始Physics消息实际均为SDF重力-9.8，首个真实更新后g4.9变式实际Physics为-4.9。改变采样间隔没有改变物理结果。独立测试解析原始protobuf，并在1→stride-1→stride→199→200逐边界原生查询位姿，交叉核对插件prephysics记录，不把插件自身姿态当唯一真值。

算法：个人参数/源摘要检查→离线编译与按键flock原子缓存→生成个人system_plugin.cfg→gzserver -u -s加载进程插件(世界SDF不放plugin)→Load仅读配置并注册事件→Init→本课worldCreated绑定真实World并建原生观测→首WorldUpdateBegin前实际设置world重力→每个真实更新递增计数并在stride整倍数读取实际prephysics pose/time→ODE继续积分。worldAvailable表示本实例持有WorldPtr，named_world_exists额外实际调用physics.has_world确认全局是否已有该命名世界。

实测生命周期严格Load1→Init1→worldCreated1：前两阶段worldAvailable/namedWorldExists均false，第三阶段均true；Reset0。标签n ms对应第n回调、姿态包含此前n-1次积分，因此200回调prephysics预测z1.80498与200步postphysics原生z1.80302分列。标签不减一毫秒或改写消息来掩盖回调相位。Load/Init不阻塞等世界，暂停ready明确重力尚未在首次更新应用；ODE::Init在本SDK为空，不把本课选择首update归因于虚构的重力覆盖。

所有非幂等step/reset只发布一次后有界读取实际完成。基准保存未重载参数被拒绝且时间/状态不推进，原字节修复后恢复；完整world reset实际恢复原生clock/model并另触发worldReset事件，进程实例与Load/Init保持，SystemPlugin.Reset不自动调用。全局worldReset无world参数，本插件以本课实际时钟回退确认归属，不把其他世界事件直接当本课复位。

边界回归：合法stride1在200tick产生200条累计记录，但recent只保留128条。静态审查发现原课程验收把累计数等同窗口长度，已仅修lab.verify：独立核total=200//stride，recent长度min(total,128)，核最后实际步号及每条源时刻/姿态。真实回归先单pulse200并独立查询Physics/实体/统计，确认总200、保留73..200；再真实复位、执行原课程verify，旁路观察器保留中间200tick与末尾reset，全部通过，不重写或补造记录。相同源字节已保存为tests/lesson31_stride1_acceptance.py，便于重跑，不仅留操作脚本。

三个formal gzserver均核对实际私有ELF inode/device/SHA，GUI无该进程插件；原始JSONL/原生实体/Physics/生成配置/日志保留两端.deploy/lesson31-formal-artifacts与stride1-artifacts。观察器正常stop-file退出，经过已验证的主线程IO join；模板未改，三formal、边界、GUI各token结束均无残留。11课程文件及主独立测试冻结，边界测试另记摘要。桌面实际验收包含GUI、两个PTY、暂停/步进、编辑/持久化/重开与清理。verify会真实复位、200步、再复位，仅恢复入场暂停，不回填物理历史或进程对象；暂停恢复后若继续运行方块会再次落下。

依据：[SystemPlugin接口](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/common/Plugin.hh)、[进程setup](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/gazebo_shared.cc)、[世界Reset调度](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/World.cc)。

证据SHA256 .deploy/lesson31-stride1.json：e007c287edb48316ed586ae6388e13b2f2b1d9f5e2d6466c0c250ac0ad74d7e9。

证据SHA256 .deploy/lesson31-prototype.json：45ff5e5824d0932b3830e1478acb25f594c760420944aa7038bab0bd5c6eaac0。

证据SHA256 tests/lesson31_stride1_acceptance.py：008778648d76aa506b0a6ebc935b53c1b41e05dd697f7db25855a6d8d30bb039。

报告SHA256：formal de7081ea0e76e0ec747ec14053398d2ad733a4d73005072bccf1f5bf82edbaf0；UI 39f6c4599d1511b58101b617e35e6ffc213348cff464fef4d2fcbd6c648e0f3c；独立测试 4b5a388bf2d429303c0b3ce3a871f6dc4cb7b7c4c83f20f43e0e57ba84d08199。

独立提交、Windows中转推送并精确核对远端后进入L32；当前继续全部39项目V2。


## 2026-11-01 L32：原生状态记录、隔离回放与三方对照

最终独立验收111.541s，三组全新原世界与三台独立原生播放器全部通过；桌面13项48.009s通过；最终V9真实用户GUI回放与原世界窗口检查55.308s通过，原生时间轴光标出现后才保存图像；另以300step/bz2全新工作区实际运行完整课程验收31.970s通过。root已查看34项目中文工作台、暂停在300ms的原世界及原生播放器00:00.300时间轴截图。GUI图像来自UID/PID/token核实的实际窗口，不通过生成图替代实验。

| 独立场景 | 实际原世界终点 | 最后日志时刻 | 保存状态/总帧 | 原始文件字节 | 原世界最终z(m) |
| --- | --- | --- | --- | --- | --- |
| 300step/zlib | 300ms/300iter | 300ms | 234/235 | 7741 | 1.557530000000001 |
| 仅长度800step | 800ms/800iter | 607ms | 541/542 | 16117 | 0.19999999999612778 |
| 仅编码txt | 300ms/300iter | 300ms | 234/235 | 120154 | 1.557530000000001 |

三文件均实际2个压缩/写入容器chunk，首直接状态14ms；总帧另含首个完整world。压缩方式改变后，重叠日志状态的模型与主连杆姿态均相同；不把该量化后相同扩大为所有原始浮点轨迹逐位相同。独立播放器各5个实际帧检查点，15处原生模型xyz与有效日志姿态最大误差均0。源及播放器原生时间、迭代、暂停状态、实际ELF与-p实参、私有Master/partition和原始protobuf分别核对。

算法：校验独立Classic11 SDK及单body/单位初始相对位姿→离线编译观察器并按源/SDK/编译器键加锁缓存→核原生世界→暂停/完整复位→单次原生LogControl.start指定个人目录与编码→有界物理步及原生检查点→单次stop→实际空full_path、完整闭合XML和稳定SHA→离线LogPlay逐帧读取→独立端口/partition/HOME/token的实际gzserver -u -p→单次原生seek实际日志时刻→原生实体/时钟对照→核原世界未变→仅清理自身进程。

日志状态查询用proto2显式start:false，字节非空且OnLogControl不执行start；无字段消息为零字节，会被实际Classic transport丢弃。仅该只读查询允许有界重试；start/stop/reset/step/seek不会因等候完成而重复发送。曾用普通gzserver文件名做进程断言，实际设备为/usr/bin/gzserver-11.15.1，已改为与所解析安装入口samefile核对并校验ELF，而非宽松名称前缀。

原始模型位置先按SDK小数4位处理，再defaultfloat有效数字3位；body世界位置为有效数字4位，分别独立预测序列化并以1e-9核对两套字段，保留原始量化残差。Gazebo先Model.SetState再Link.SetState，canonical body会更新父模型；本课单位相对变换下，播放器模型xyz应等于所存body世界xyz，而非低精度model/pose。实测300ms：原生原世界z1.55753，model日志1.56、body日志1.558，原生播放器1.558；没有将两类误差混合放宽门限。回放lab门限1e-8、独立验收门限2e-6保持。

800step场景607ms已落稳，原生变化状态过滤随后不再写入状态。原世界实际时间T800ms与最后状态S607ms保留区分；只允许选最新S≤T且模型3位、body4位均与实际T处位置序列化结果相同。报告保留T/S、原始/记录迭代及193ms间隔；回放停在真实S，不补帧、改时间戳或宣称连续受力/速度已获证明。独立旁路WorldStatistics实际见证暂停800ms/800iter。

桌面先继续运行再验收，暴露原单次stats→entity→stats读取可横跨复位/seek过渡；V8改为6s有界只读重采样，要求前后均暂停且时间和迭代相同，动作仍只发一次。每次build/checkpoint-UUID.json在异常前保留阶段、Master、最后三份原生读取、stable/pending_transition摘要；成功亦返回诊断路径。最终正式验收保留42份课程检查点日志。V9对可选GUI在启动、seek、展示及清理前每≤100ms检查真实server/client存活，发现退出保存阶段和输出并失败；这项存活检查与独立窗口/图像检查分开记录。

真实调试历史保留：原型v1缺gazebo_util显式链接；v2/v3初始日志查询超时；v4文件名断言；v5 canonical-link判定；v6默认300完整GUI/双PTY通过39.119s，800缺末帧；v7保留状态修正后800通过35.668s。第一版正式三场景114.561s通过、GUI47.524s通过，但桌面首次38.635s失败，已保留实际失败后修复一致读取。第二次正式因复用04:01停止标记让04:12观察器提前结束，属于验收脚本隔离缺陷；已为每轮wire与stop使用新UUID并确认活跃握手，3采样/50ms及全部数值标准均未改变。GUI测试也改为有界只读等候原生统计及实际可见时间轴后抓图，仅抬升自身核实窗口。旧GUI中一次强制中止时client记录-11，尚无故障栈证明发生阶段，G02另留有序关闭审计，最终GUI运行及主动结束client/server均-15、无残留。

10个课程文件和最终独立测试冻结。formal-v1/v2-evidence.zip包含各版源、报告、原始日志、SDK头文件与数据，保留两端；原型source/data/native诊断亦保留。最终正式及GUI、bz2证据归档两端。原生日志原始SHA在读取、原生播放器及复制后保持；不接受LogPlay自动补尾修复来掩盖未关闭日志。所有正式token、桌面及单独GUI完成后均无自有实验进程，原模板未改。

范围：真实状态/时钟/xyz及图形窗口、个人练习闭环；不宣称录制了ROS指令、相机像素、完整六维姿态误差或连续静止轨迹。官方依据：[LogRecord](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/util/LogRecord.cc)、[ModelState](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/ModelState.hh)、[LinkState](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/LinkState.hh)、[Model恢复顺序](https://github.com/gazebosim/gazebo-classic/blob/gazebo11_11.15.1/gazebo/physics/Model.cc)。

证据SHA256 .deploy/lesson32-replay-gui-v4.json：21f0ee71cb7f0a1b428f4f9311086bc0808e7ac21ca375847e71ac57d9246031。

证据SHA256 .deploy/lesson32-bz2.json：76b810483c54763b6a9d42dbd36fd56a0c0aaed9a2ec704066e2162e89fadcaa。

证据SHA256 .deploy/lesson32_gui_probe.py：3a1253d068ad2e4d818392bdbb585d7cab5657fed84af4040a6d44c583e66b57。

证据SHA256 .deploy/lesson32-formal-v1-evidence.zip：bb9a438cd449a952cbab1e8fd51402f1e83e8e0990fa5b50e3bae24365d8740b。

证据SHA256 .deploy/lesson32-formal-v2-evidence.zip：a82573391ebac702fbcd949fb28f70a167c140df211018e6e7714e82a9575f38。

报告SHA256：formal 76202e041729ea1de57ccdb1823e97dee133bfff3ab0facea521559d8ef45d39；UI c0470a4ff8b6133102e8cecb0bbe7121c962bf7ba8e5842818c74895e47fd482；独立测试 2536371f80d47dc6cb7b98ac4655d199259add667a803b84d7a4cefaa32b9fe8。

独立提交、Windows中转推送并精确核对远端后进入L33；当前继续全部39项目V2。
