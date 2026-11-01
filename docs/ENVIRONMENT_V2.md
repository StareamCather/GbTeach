# V2 环境检查、只读教材与模型重置

本文说明 V2 框架修改后的算法流程。已验证的实验基线是 Ubuntu 20.04、Python 3.8、Gazebo Classic 11.15.1、ROS 1 Noetic 与 Qt 5.12；平台单元测试和真实 Gazebo 实验验收分别记录。

## 一次探测，逐课判断

启动时，工作线程执行 `detect_environment()`，界面线程只接收结果。一次环境快照供全部 39 个项目共用；兼容性表和开始按钮读取相同快照，不会为每个项目重复启动编译器或调用 pkg-config。点击“重新检测”会生成新快照。

1. 分别记录当前 PATH 选中的 Classic 可执行文件、其实际版本、已确认的其他 Classic 安装，以及现代 Gazebo 的实际版本探测结果。`gz` 或 `ign` 命令存在本身不代表安装了某一种模拟器。Classic 11.15.1 的版本查询允许其已知的退出码 255，其他失败不会冒充成功。
2. 分别列出 `/opt/ros` 中带 setup 的已安装发行版和当前 shell 的 `ROS_DISTRO`、`ROS_VERSION`。没有激活 Noetic 的外层 shell，仍然可以运行已安装 Noetic 的课程。ROS 2 当前处于激活状态，也不等于 Noetic 不存在。
3. 检测开发包时先构造子进程环境。清除 ROS 2 安装及 AMENT/COLCON overlay 在 PATH、PYTHONPATH、LD_LIBRARY_PATH、CMAKE_PREFIX_PATH、PKG_CONFIG_PATH 和 Gazebo 插件/模型/资源路径中的条目与 ROS 2 选择变量，再加载 Noetic setup；其他个人工具、学习素材路径和 ROS 1 overlay 保留。真正运行 ROS 1 课程时采用同一清理流程。应用不写用户 shell 配置，也不修改操作系统或全局环境。
4. 对每种声明使用的 pkg-config 包查询版本及完整编译、链接参数。`pkg-config` 可执行文件存在，与 `gazebo.pc` 存在是两项事实。再让选中的 g++ 以 `-fsyntax-only` 实际解析 `gazebo/gazebo.hh`，不生成或启动课程程序。缺少 SDK 头文件也会提前显示。
5. 编译原生代码的课程声明精确 SDK 11.15.1，并检查当前 gzserver 与 `/usr/bin/gzserver-11.15.1` 是否解析到同一文件。课程自身的 prepare 仍负责源文件、头文件、库文件、编译缓存与实际运行时的进一步校验；启动环境检查不能代替实验验收。
6. 第 29、30 课逐项要求 OGRE、OGRE-Paging、OGRE-Terrain、OGRE-RTShaderSystem 和 Qt5Widgets 开发包及可链接的 `libgazebo_gui.so`。Qt 开发版本按本机已验证的 5.12.8 声明。第 30 课从 `Qt5Core` 的 `host_bins` 查找 moc，执行 `moc -v` 并核对版本；不会依赖 `/usr/bin/moc` 的 qtchooser 默认选择，也不设置全局 QT_SELECT。
7. 第 33、36、37 课按现有 prepare 的真实前置条件，要求 `rospack find gazebo_plugins` 实际选中包的 package.xml 版本为 2.9.3。第 15、19、20 课列出其原生桥使用的 ROS pkg-config 包；第 17、18 课检查 NumPy 模块。其他使用系统插件的课程保留各自插件、工具和 ROS 包要求。

兼容性判断按“已知不支持的版本 → 检查信息是否完整 → 具体依赖是否满足”区分状态。版本不匹配、缺少依赖和检测未完成分别显示明确原因；所有状态下教材均可阅读。多个教学包同时存在时，首次选择优先采用有满足依赖项目的教学包，其他包仍留在选择器中。

## 清单字段

| 字段 | 含义 |
| --- | --- |
| `requires_pkg_config` | 开发包名到精确版本的映射；空字符串表示只要求该包和依赖可用 |
| `requires_native_server` | 要求当前 gzserver 与精确 11.15.1 运行时文件一致 |
| `requires_gui` | 必须请求启动真实 gzclient；只有 DISPLAY 不够 |
| `requires_qt_moc` | 要求 Qt5Core host_bins/moc 可执行且版本匹配 |
| `requires_libraries` | 由编译器解析到的额外开发库，如 gazebo_gui、gazebo_ode、gazebo_util |
| `requires_ros_package_versions` | 由 Noetic 子环境实际解析到的 ROS 包精确版本 |
| `requires_python_modules` | prepare 所需的额外 Python 模块 |

这些字段只描述每课实际需求；未使用 SDK 的基础世界课程不会因为缺少开发包而被一并禁用。可选持久控制能力不足时，环境页明确显示 `legacy-cli` 回退及原因。环境页的控制能力是只读预测；会话实际采用的后端以该会话构建结果和运行日志为准。

第 29、30 课依赖客户端插件。CLI 执行 `--smoke 29` 或 `--smoke 30` 时若缺少 `--gui`，兼容性检查在创建练习目录、编译 SDK 程序或启动实验之前给出明确错误。开始实验后仍按既有协议检查真实客户端，进程存在不等于目标达成。

## 前置教材链接不切换实验

Markdown 渲染器生成相对 Markdown 链接、引用式链接和标题锚点。点击课程中的前置链接时，应用只打开独立的“前置知识 · 只读教材”窗口，不走课程树的选择回调。

流程为：当前教材绝对路径作为 base → 解析链接与 fragment → URL 解码并规范化真实路径 → 核对已登记课程 Markdown 白名单和仓库目录边界 → 只读加载 → 跳转标题锚点。每次点击重新解析符号链接，避免目录加载后替换链接造成越界。非 Markdown、未登记文件、越界路径、脚本协议及带查询的内部地址不能打开；HTTP/HTTPS 资料仍由 QDesktopServices 打开。原始 HTML 始终转义，不加载网站脚本或远程数学依赖。

阅读窗口保留“返回上一份教材”的路径与滚动位置链；参考页以自身位置解析下一层相对链接。当前课程、自动开始设置、运行中的 session、个人练习和实验记录均不因阅读跳转改变，也不自动启动前置实验。

## 模型重置的边界

“重置模型状态”继续调用既有 MODEL reset 路径，保留仿真时钟，不替换成全时钟重置。按钮提示明确说明：任意插件的私有状态、关节内部控制状态不保证复位。需要重新创建本课插件及世界时使用“重载项目”；需要恢复练习文件时使用“备份并恢复起点”。

## 检查方式与证据边界

`tests/test_environment_v2.py` 覆盖开发工具存在但缺 pc、SDK 版本不符、真实头文件缺失、运行时身份不符、Qt 开发包与 moc 分别缺失、ROS 2 混合前缀的子环境清理、各 prepare 实际 pkg-config 依赖与清单一致，以及无 GUI 时在课程启动前拒绝第 29、30 课。

`tests/test_environment_dialog.py` 使用 Qt 实际鼠标点击第 37 课的第 7 课链接，核对独立阅读器内容和返回链，并检查 session 对象、当前课、个人文件字节、阅读记录均保持不变；另测危险/越界链接和外部资料分发。以上是合成环境的行为测试，不替代真实 Linux 桌面截图、全部课程的 Gazebo 验收或操作系统安装验证。
