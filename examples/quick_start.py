from carm import Carm

# 连接到机械臂（默认 IP: 10.42.0.101:8090）
robot = Carm()

# 等待连接成功，检查状态
if robot.is_connected():
    print("Connected!")

# 将机械臂设置为就绪状态（清除错误、上使能、位置模式）
robot.set_ready()

# 获取当前关节位置
print("Joint positions:", robot.joint_pos)

# 移动到目标关节位置（阻塞等待完成）
robot.move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], is_sync=True)
print("Finished moving to joint position.")
print("Current joint positions:", robot.joint_pos)

# 移动到目标笛卡尔位置（阻塞等待完成）
robot.move_pose([0.30669, 0.000073, 0.415677, 0.733466, 0.000187, 0.679726, 0.000029], is_sync=True)
print("Finished moving to Cartesian position.")
print("Current Cartesian pose:", robot.cart_pose)


# 移动到目标关节位置（阻塞等待完成）
robot.move_joint([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], is_sync=True)
print("Finished moving back to joint position.")
print("Current joint positions:", robot.joint_pos)

# 关闭连接
robot.disconnect()
