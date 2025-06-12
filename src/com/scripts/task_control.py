#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math, numpy as np
from std_msgs.msg     import Int8, Int32
from geometry_msgs.msg import Twist

# ———————— 参数区 ————————
camera_yaws = np.array([
    -math.pi/4,   # cam0
     math.pi/4,   # cam1
     3*math.pi/4, # cam2
    -3*math.pi/4  # cam3
])
V0  = 0.3   # 线速度 m/s

# ———————— 全局状态 ————————
current_command = None   # 'search' / 'return' / 'exit'
cmd_pub = None           # 控制命令发布器

# ———————— 辅助函数 ————————
def normalize_angle(a):
    """归一化到 [-π, π]"""
    return (a + math.pi) % (2*math.pi) - math.pi

def decode_task_command(val):
    """Int8 值 → 字符串指令"""
    if val == 81: return 'search'
    if val == 45: return 'return'
    if val == 91: return 'exit'
    return None

# ———————— 统一回调 ————————
def unified_cb(msg,src):
    src=src-1
    global current_command

    # 如果是来自 control_command 的 Int8 指令

    cmd = decode_task_command(msg.data)
    if not cmd:
        rospy.logwarn(f"未知指令编码：{msg.data}")
        return
    current_command = cmd
    rospy.loginfo(f"[TASK COMMAND] {current_command}")
    if current_command == 'exit':
        cmd_pub.publish(Twist())  # 停止移动


    # 摄像头触发信号处理
    if not msg.data or current_command not in ('search', 'return'):
        cmd_pub.publish(Twist())
        return

    # 获取相机朝向并修正方向
    yaw = camera_yaws[src]
    if current_command == 'search':
        yaw += math.pi
    yaw = normalize_angle(yaw)

    # 速度分解为 x 和 y 分量
    vx = V0 * math.cos(yaw)
    vy = V0 * math.sin(yaw)

    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    # twist.angular.z = 0  # 不再使用角速度控制

    cmd_pub.publish(twist)
    rospy.loginfo(f"[CAM{src}] {current_command} → vx={vx:.2f}, vy={vy:.2f}")

# ———————— 主入口 ————————
def main():
    global cmd_pub
    rospy.init_node('task_cam_unified_controller', anonymous=True)

    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # 摄像头信号监听
    for i in range(1,5):
        rospy.Subscriber(f'/decoded_{i}',Int32, unified_cb, callback_args=i)


    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
