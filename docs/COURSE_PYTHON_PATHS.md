# 课程私有 ROS Python 消息路径

自定义 ROS 消息由课程在个人练习目录构建。课包可声明：

```json
{
  "requires_ros": true,
  "ros_package_paths": ["ros_ws/src"],
  "ros_python_paths": ["ros_ws/devel/lib/python3/dist-packages"]
}
```

启动算法先验证 manifest 中列表类型和课程内相对路径，准备个人副本，再加载选定 ROS 发行版的子进程环境。随后把 `ros_python_paths` 解析为**个人副本**中的绝对路径，复查符号链接没有越出副本，依声明顺序前置到子环境 `PYTHONPATH`，保留发行版已有路径。路径可以尚不存在：后续 `prepare` 完成 catkin 消息构建后，辅助节点、验收进程和真实操作终端继承同一环境即可导入生成的模块。

Classic/Noetic 和 Harmonic/Jazzy 使用相同路径规则；课程仍需声明实际构建步骤与工具依赖。这个字段不表示消息已经构建，也不绕过课包兼容性检查。未声明或空列表时保留既有环境行为。应用不修改 `os.environ`、用户 bashrc 或系统 Python；不执行个人工作区中的任意 setup 脚本。

`ROS_PACKAGE_PATH` 用于 ROS1 包发现，`PYTHONPATH` 用于 Python 类型导入，两者作用不同。课包须实际检查其 `rosmsg`、`rostopic` 示例能够解析自定义消息；不能以导入成功替代 ROS 图或行为验收。退出实验后无需撤销系统配置，因为路径仅属于本次实验的子进程。

环境快照同时增加 `rossrv`、`catkin_make` 的实际命令发现，沿用已安装但未激活 Noetic 时的 bin 路径探测；课程据快照检查构建和服务类型观察工具。

两个路径字段均拒绝冒号和分号，解析后的绝对目录也不能包含当前系统的路径列表分隔符，避免合法文件名在写入环境变量时被拆成额外搜索目录。

验证包括：两种运行配置的 setup 覆盖顺序、未构建目录、不同个人副本的路径隔离、父进程环境保持、非法 manifest、路径列表注入和个人符号链接越界拒绝。对应 `tests/test_ros_python_paths.py`；真实 catkin 构建与消息交互证据随 A43 课程记录。
