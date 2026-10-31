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
