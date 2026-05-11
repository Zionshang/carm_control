# carm_control 客户接口说明

这个项目对客户开放一个 LCM 命令接口。客户通过发送 `WaypointCommand` 控制机械臂，支持四类操作：

- `TRACK_TCP`：周期发送 TCP 位姿增量，机械臂连续跟踪累加后的目标位姿。
- `SAVE_CURRENT`：记录当前实际关节状态到指定名称。
- `GOTO`：按名称运行到已保存的关节状态。
- `HOME`：让机械臂回到 home 关节位姿 `[0, 0, 0, 0, 0, 0]`。

## 安装

```bash
pip install -e .
```

## LCM 协议

协议文件：

- `lcm_types/carm_waypoint.lcm`

LCM channel：

- `tcp_cmd`

消息类型：`WaypointCommand`

字段说明：

- `op`：操作类型，支持 `TRACK_TCP`、`SAVE_CURRENT`、`GOTO`、`HOME`
- `name`：位置名称，`SAVE_CURRENT` 和 `GOTO` 时使用
- `tcp_cmd`：固定 6 维 TCP 位姿增量，格式为 `[dx, dy, dz, droll, dpitch, dyaw]`，仅 `TRACK_TCP` 时使用
  - 推荐单次增量：
  - `dx/dy/dz`：米，例如 `0.001` 到 `0.003`
  - `droll/dpitch/dyaw`：弧度，例如 `0.005` 到 `0.02`

说明：

- `tcp_cmd` 中位置增量单位是米，角度增量单位是弧度。
- 服务端第一次收到 `TRACK_TCP` 时，会以当前实际 TCP 位姿作为内部目标；之后每条 `TRACK_TCP` 都会把 `tcp_cmd` 累加到该内部目标，再调用 `track_tcp_pose()`。
- 服务端会限制内部目标姿态：`roll/yaw ∈ [-pi, pi]`，`pitch ∈ [-pi/2, pi/2]`。
- 当前接口是单向命令接口，不返回 LCM 状态消息。
- 服务端收到非法命令时，只会在服务端日志打印错误。

各操作的消息内容约定：

- `TRACK_TCP`
  - `op = "TRACK_TCP"`
  - `name = ""`
  - `tcp_cmd = [dx, dy, dz, droll, dpitch, dyaw]`
- `SAVE_CURRENT`
  - `op = "SAVE_CURRENT"`
  - `name = "<位置名称>"`
  - `tcp_cmd = [0, 0, 0, 0, 0, 0]`
- `GOTO`
  - `op = "GOTO"`
  - `name = "<位置名称>"`
  - `tcp_cmd = [0, 0, 0, 0, 0, 0]`
- `HOME`
  - `op = "HOME"`
  - `name = ""`
  - `tcp_cmd = [0, 0, 0, 0, 0, 0]`

## 启动服务

```bash
python examples/lcm_waypoint_server.py \
  --addr 10.42.0.101 \
  --lcm-url "udpm://239.255.76.67:7667?ttl=1"
```

参数说明：

- `--addr`：机械臂控制器 IP
- `--lcm-url`：LCM 总线地址

## 客户使用流程

这个接口的使用目标是：

1. 通过发送 TCP 末端指令，把机械臂带到目标位置。
2. 在当前位置记录当前实际关节坐标。
3. 后续通过简单命令，直接运行到这个已记录的关节状态。

推荐流程如下。

### 1. 周期发送 `TRACK_TCP`

客户端应周期性发送 TCP 位姿增量，而不是直接发送绝对 TCP 坐标。

示例：

- `op = "TRACK_TCP"`
- `name = ""`
- `tcp_cmd = [0.002, 0.0, 0.0, 0.0, 0.0, 0.0]`

服务端收到后会把这条增量累加到内部 TCP 目标位姿，然后调用 `track_tcp_pose()`，让机械臂末端持续跟踪累加后的目标。

### 2. 到位后发送 `SAVE_CURRENT`

当机械臂已经通过 TCP 控制到达你想要记住的位置后，再发送一条保存命令。

示例：

- `op = "SAVE_CURRENT"`
- `name = "pick_1"`
- `tcp_cmd = [0, 0, 0, 0, 0, 0]`

这条命令会读取当前实际关节角，并把它保存为 `pick_1`。

### 3. 后续发送 `GOTO`

后续如果希望机械臂直接回到这个状态，只需要按名称发送一条简单命令。

示例：

- `op = "GOTO"`
- `name = "pick_1"`
- `tcp_cmd = [0, 0, 0, 0, 0, 0]`

这条命令会读取已保存的关节角，并执行 `move_joint()` 到该状态。

### 4. 如需回到 home，发送 `HOME`

示例：

- `op = "HOME"`
- `name = ""`
- `tcp_cmd = [0, 0, 0, 0, 0, 0]`

这条命令会执行 `move_joint([0, 0, 0, 0, 0, 0])`，让机械臂回到 home 关节位姿。

## 位置文件说明

位置固定保存到 `data/waypoints.json`。

每个名称保存的是：

- 当前实际关节角 `joint_pos`
- 保存时间 `saved_at`

同名 `SAVE_CURRENT` 会直接覆盖旧记录。
