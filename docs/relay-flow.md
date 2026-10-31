# Windows 中转与更新流程

## 当前托管开发

本机编写本次目标的源码 → 打包指定文件 → SCP 到 ikaros 部署缓存 → SHA256 校验 → 在项目边界内解包 → 真实 Gazebo/Qt 验证 → 记录证据 → 在 ikaros 按目标提交 → Windows fetch 该提交 → 更新本机提交索引而保留尚未交付的草稿 → 推送 GitHub → 核对 GitHub main 与已验证提交的哈希。

首次部署通过 Git bundle 传递已有历史；后续每轮由 Windows 从 ikaros 获取实际提交。SSH 对 GitHub 和 ikaros 使用独立密钥，GitHub 私钥不复制到 ikaros。所有连接启用已知主机校验和有限连接超时。SCP 传输二进制文件，不经过 PowerShell 文本管道。

部署前检查目录和当前仓库状态。解包仅允许写入 /home/ikaros/GazeboTeach 内的相对文件；不会删除目标中未列出的文件。练习工作区和运行日志使用独立目录。

SSH 维护命令从该用户正在运行的 GNOME 会话读取 DISPLAY、XAUTHORITY 和 D-Bus 环境，不把某次登录的显示编号写成应用默认值。窗口只出现在 ikaros 的实际桌面。

## 后续在 ikaros 直接开发

ikaros 完成修改、验证并提交 → Windows 中转仓库 fetch ikaros → 快进同步到该提交 → 推送 origin。出现分叉时先分析历史并合并，禁止强制覆盖远端。

## 下载兜底

优先复用系统已有软件和随课离线资源。确实需要外部依赖时，先做有限超时下载；远程失败则 Windows 下载官方来源文件、核验，再通过 SCP 中转。不会为解决下载而更换系统驱动、默认路由或 SSH 策略。

GitHub 网络握手偶发超时时，先有限次数重试相同的幂等推送/读取；仍失败时使用 GitHub 官方 ssh.github.com:443 入口。该回退仅对当前 Git 命令生效，保留既有 GitHub 主机密钥校验，不改全局 SSH 或路由。认证或主机身份错误立即停止对应传输，不作为普通网络错误绕过。每次推送成功后仍必须读回 main 哈希核对。


## GitHub Release资产

固定已验收提交构建载荷→校验manifest、SHA256SUMS、远端main/标签→创建或恢复草稿→按摘要跳过相同附件并续传缺项→发布→下载复核。私有仓库保持可见性，GCM令牌仅Windows内存且认证只到固定API；捕获CDN重定向后用无认证连接，不保存临时签名URL。上传中断先列远端结果，不盲目重放歧义POST或覆盖不同附件。最终下载失败则将本次发布恢复为草稿；全部验证完成才计交付。仓库外release_gazeboteach.py属于维护工具，不让教学应用处理GitHub凭据。
