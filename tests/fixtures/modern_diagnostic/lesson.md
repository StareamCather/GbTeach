# 现代运行器独立诊断

本场景用于框架验收。Harmonic 和 Fortress 分别加载各版本的系统插件。方块在重力下落到地面后，验收程序从 Gazebo Transport 读取实际位姿，核对落点高度。运行器从同一分区读取 WorldStatistics，核对暂停、单步和继续后的时钟与迭代数。
