# ROS 2 项目：面试讲解与能力证据

## 60 秒介绍

> I built an independent AMR navigation lab using ROS 2 and Nav2. It runs without robot hardware by simulating differential-drive motion, laser scans, and depth images. My code connects the sensor data to obstacle perception and manages navigation goals, feedback, cancellation, and final results. I separated the reusable planning and mission rules from the ROS adapters so I could test them independently. One integration issue I fixed was submitting a goal before the Nav2 lifecycle node became active. The project includes repeatable checks and exported trajectories, rather than only a successful screenshot.

这是作品集介绍。它不能证明你在某家公司交付过同样的系统，也不证明你已在 Jetson 或 ZED 实机上验证。

## 简历能力怎样对应代码

| 能力 | 证据入口 | 应能解释的问题 |
| --- | --- | --- |
| ROS 2 节点和数据接口 | `sim_world.py`、`depth_projector.py` | 为什么传感器使用 sensor-data QoS？ |
| 地图、定位和 TF | `config/nav2.yaml`、`launch/demo.launch.py` | map、odom、base_link 各自负责什么？ |
| 深度感知 | `core.py: project_depth` | 深度单位、无效值、内参和外参分别是什么？ |
| Nav2 与代价地图 | `config/nav2.yaml` | 二值地图、膨胀代价、机器人外形有何区别？ |
| 任务 action 与生命周期 | `mission_node.py` | 已发现服务为何还可能不能导航？取消何时算完成？ |
| 排障和验证 | `tests/ros_smoke.py` | 如何证明最终状态，而不是只看请求成功？ |
| Docker 工程化 | `Dockerfile` | Ubuntu 主机版本和容器 ROS 发行版有什么关系？ |

## 五个常见追问

**你是否写了 Nav2 的规划器？**

没有。Nav2 提供生产级框架中的 planner、controller、AMCL 和 costmap。本项目写了独立的 A* 教学基线、运动与传感器模拟器、深度适配和任务管理。这两个规划执行路径需要分别解释。

**机器人卡住时先改什么参数？**

先确认定位和 TF、传感器更新时间、机器人外形与地图，再区分没有可行路线和控制器无法跟踪路线。不要把所有问题都当作控制器参数问题。

**为什么点云里不能保留所有深度点？**

无效值会污染数据；距离和高度过滤用于排除不适合作为障碍的点。真正现场还需要考虑地面、反光、遮挡和时间同步。此模拟器没有模拟这些光学误差。

**为什么取消后还等待？**

请求已发出、服务器同意取消、最终结果返回是不同阶段。期间不应随意发送第二个目标；服务器也可能在取消到达前已经成功完成。

**如何接入 VLA？**

当前没有 VLA。可研究的扩展是语言／视觉信息如何形成任务目标，以及学习型动作策略如何与现有导航基线比较。语言生成一个 Nav2 目标不等于已经实现了端到端 VLA。

## 下一次面试前你应补的个人证据

录一段 2 分钟演示：运行测试 → 展示路线 → 取消任务 → 展示真实终态 → 解释一段自己修改的代码。保留一个失败案例和修复过程。只背说明、不能运行或解释自己的修改，会很容易在追问中暴露。
