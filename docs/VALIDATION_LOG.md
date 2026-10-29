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

后续每课验证记录在实际执行后追加。
