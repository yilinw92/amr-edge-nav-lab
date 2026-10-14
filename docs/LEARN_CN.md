# 从能运行到能解释：ROS 2 项目学习路线

当前主项目已升级。进阶设计、失败案例、性能对照和练习见 [ENGINEERING_CN.md](ENGINEERING_CN.md)。本页保留入门说明。

## 第一课：先看一次真正的输入、状态和输出

进入仓库根目录，在 Windows 用 `python`，Ubuntu 用 `python3`：

```bash
python -m amr_edge_lab.offline --scenario normal --out artifacts/normal
python -m amr_edge_lab.offline --scenario blocked --out artifacts/blocked
```

分别打开两个目录里的 `replay.html`，拖动时间条。观察橙色障碍出现的位置，以及青色路径怎样变化。

先回答三个问题：

1. `normal` 的 `replans` 为什么是 0，`blocked` 为什么是 1？
2. 第二条轨迹更长，是否代表算法更差？说明障碍改变了什么约束。
3. 这个动画是否运行了 ROS 2？答案是没有，这是独立于 ROS 的离线基线。

打开 `amr_edge_lab/core.py`，先只读 `Grid` 和 `astar`。格子 `(10,10)` 的中心对应 `(1.05,1.05)` 米，因为地图分辨率是 0.1 米。

**你的第一次改动：** 把 `offline.py` 中的目标从 `(68,48)` 改成 `(68,10)`，重新运行，记录路径长度和结果。然后改回原值。写一段自己的观察，比背整个项目介绍更有用。

## 第二课：为什么机器人不能当成一个点

`Grid.inflated(0.3)` 把障碍附近一定范围内的格子也标成不可通行。原始地图里的一条缝隙可能允许点通过，但不允许有外形的机器人通过。

练习：把离线膨胀半径分别改为 0.1、0.3、0.6，比较路径。这里的半径是教学规划余量，不能直接当作 Nav2 的 `inflation_radius` 和 `robot_radius` 的全部含义；Nav2 还有分级代价值，而离线基线是二值阻塞。

再读禁止对角穿角的判断。若相邻的水平／竖直格子被占据，为什么对角移动不能直接通过？

## 第三课：ROS 2 的 topic、service 和 action

启动 Docker 演示后，在另一个终端运行：

```bash
docker exec amr-lab /portfolio-entrypoint.sh ros2 node list
docker exec amr-lab /portfolio-entrypoint.sh ros2 topic list
docker exec amr-lab /portfolio-entrypoint.sh ros2 action list
docker exec amr-lab /portfolio-entrypoint.sh ros2 topic hz /scan
```

- Topic：连续数据，比如 `/scan`、`/odom`、`/perception/obstacles`。
- Service：短请求与响应，比如 `/demo/toggle_blocker`。
- Action：持续运行的导航目标，需要接受、反馈、取消和最终结果。

`/mission/start` service 的成功只表示任务已提交。必须看后续 action 结果才能知道是否到达。

## 第四课：先能画出 TF，再谈定位

画出 `map -> odom -> base_link`。本项目中：

- `sim_world.py` 发布 `odom -> base_link`，模拟轮式里程计。
- AMCL 根据静态地图和激光数据估计定位，发布 `map -> odom`。
- 深度数据投影函数输出 `base_link` 坐标中的点。

```bash
docker exec amr-lab /portfolio-entrypoint.sh ros2 run tf2_ros tf2_echo map base_link
```

解释：为什么局部里程计应该连续，但全局定位修正可能造成位置变化？本模拟器里程计是理想的，不要把它称为已经验证了真实轮滑或定位漂移。

## 第五课：深度像素如何变成障碍点

读 `project_depth`。相机光学坐标中：

```text
X = (u - cx) * depth / fx
Y = (v - cy) * depth / fy
Z = depth
```

本项目约定相机水平朝前，距底盘坐标原点高 0.35 米，因此转换到机器人坐标：`x = Z, y = -X, z = 0.35 - Y`。

练习：用中心像素、右侧像素、下方像素分别算一个 2 米深度点；解释左右与上下符号。查看单元测试如何覆盖 NaN、Inf、0 和错误形状。

固定变换是基础案例。当前 perception.py 已加入参数化 translation/RPY 外参，perception_numpy.py 提供向量化处理；真实安装仍要校准并确认 TF。

## 第六课：复现一个真实启动问题

本项目第一次 ROS 集成测试遇到：action server 可以被发现，但 Nav2 的 lifecycle 节点还没激活，过早提交目标会被拒绝。

修复在 `route_node.py`：查询 `/bt_navigator/get_state`，确认 active 后才允许提交。

练习：解释“接口已注册”和“组件已经准备好处理请求”的区别。然后读 `Mission` 的状态转换，说明为什么取消请求不能立即写成 `CANCELED`。

## 第七课：自己完成一项可展示改动

选择一项完成，并保存改动前后证据：

1. 增加一个地图货架，重新生成 PGM，运行完整路线并解释路径变化。
2. 在深度投影中增加参数化高度过滤，并补一个有意义的边界测试。
3. 给任务状态增加“取消原因”，区分人工取消和超时。
4. 增加连续多个目标的任务队列，明确前一个目标完成后何时发送下一个。

每完成一次真实改动，再提交 Git；不要预先生成虚假的学习提交历史。

## 这周学会的验收标准

你应能亲手运行测试、解释自己的一个改动、说明一个失败案例、指出 Nav2 与自己代码的边界，并说清哪些证据来自仿真。能做到这些，项目才开始成为你的能力证据。
