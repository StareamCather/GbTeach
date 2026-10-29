# GazeboTeach

面向聪明初学者的 Gazebo 桌面教学工作台。使用真实 Gazebo 场景、终端和观测数据学习建模、物理、传感器、ROS 集成与插件开发。

首个课程包：**Ubuntu 20.04 / Gazebo Classic 11 / ROS Noetic**，验证设备为 ikaros。

当前按顺序目标交付。框架、启动入口和课程的完成状态以验证记录为准，尚未验收的本地草稿不计入发布内容。

## 启动

在 ikaros 图形桌面中运行：

```sh
cd /home/ikaros/GazeboTeach
./launch.sh
```

SSH 调试时应使用该用户的实际图形会话。常规桌面启动不需要手工设置 DISPLAY。

教材和模型资源本地保存；先修资料链接在系统浏览器打开。应用保留每课个人练习副本，课程原始模板与参考材料不会被练习覆盖。

## 工作流程

启动脚本定位项目 → 探测实际模拟器与 ROS 环境 → 匹配课程包 → 选择课程 → 准备练习副本 → 启动本课独立仿真 → 打开观察/操作终端 → 根据真实状态验收 → 保存记录并停止本课进程。

每课提供目标、必要背景、输入到输出的信息流、文件结构、单变量实验、结果解释、错误诊断、挑战和可观察验收。更改控制器或插件时，教材同步解释算法处理流程。

## 开发与部署

- [执行计划](docs/EXECUTION_PLAN.md)
- [框架运行流程](docs/framework-flow.md)
- [验证记录](docs/VALIDATION_LOG.md)
- [Windows 中转流程](docs/relay-flow.md)

代码在 ikaros 实际运行、调试，通过 Windows 本机推送 GitHub。GitHub 私钥仅保留在 Windows。部署和构建缓存、练习工作区以及实验日志不纳入源码。

## 许可

保留仓库原有 MIT License，详见 LICENSE。
