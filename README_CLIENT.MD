# carm_control 客户接口说明

这个项目对客户开放一个 LCM 命令接口。客户通过发送 `WaypointCommand` 控制机械臂，支持四类操作：

- `TRACK_TCP`：周期发送 TCP 目标位姿，机械臂连续跟踪。
- `SAVE_CURRENT`：记录当前实际关节状态到指定名称。
- `GOTO`：按名称运行到已保存的关节状态。
- `STOP`：停止当前 `GOTO` 动作。

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

- `op`：操作类型，支持 `TRACK_TCP`、`SAVE_CURRENT`、`GOTO`、`STOP`
- `name`：位置名称，`SAVE_CURRENT` 和 `GOTO` 时使用
- `tcp_cmd`：固定 6 维 TCP 位姿，格式为 `[x, y, z, roll, pitch, yaw]`，仅 `TRACK_TCP` 时使用

说明：

- `tcp_cmd` 中角度单位是弧度。
- 当前接口是单向命令接口，不返回 LCM 状态消息。
- 服务端收到非法命令时，只会在服务端日志打印错误。

各操作的消息内容约定：

- `TRACK_TCP`
  - `op = "TRACK_TCP"`
  - `name = ""`
  - `tcp_cmd = [x, y, z, roll, pitch, yaw]`
- `SAVE_CURRENT`
  - `op = "SAVE_CURRENT"`
  - `name = "<位置名称>"`
  - `tcp_cmd = [0, 0, 0, 0, 0, 0]`
- `GOTO`
  - `op = "GOTO"`
  - `name = "<位置名称>"`
  - `tcp_cmd = [0, 0, 0, 0, 0, 0]`
- `STOP`
  - `op = "STOP"`
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

客户端应周期性发送最新的 TCP 目标位姿，而不是只发送一次。

示例：

- `op = "TRACK_TCP"`
- `name = ""`
- `tcp_cmd = [0.32, 0.01, 0.35, 0.0, 1.57, 0.0]`

服务端收到后会调用 `track_tcp_pose()`，让机械臂末端持续跟踪该目标。

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

### 4. 如需中断，发送 `STOP`

示例：

- `op = "STOP"`
- `name = ""`
- `tcp_cmd = [0, 0, 0, 0, 0, 0]`

这条命令用于停止当前 `GOTO` 动作。

## 位置文件说明

位置固定保存到 `data/waypoints.json`。

每个名称保存的是：

- 当前实际关节角 `joint_pos`
- 保存时间 `saved_at`

同名 `SAVE_CURRENT` 会直接覆盖旧记录。
