#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
import csv
import os
from datetime import datetime

def init_command_log():
    """初始化命令日志"""
    # 创建数据存储目录
    log_dir = os.path.join(os.path.expanduser('~'), 'command_logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 创建日志文件名（使用时间戳）
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(log_dir, f'command_sequence_{timestamp}.csv')
    
    # 初始化CSV文件
    with open(log_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'command_name', 'command_value', 'round', 'sequence'])
    
    return log_file

def log_command(log_file, command_name, command_value, round_num, sequence_num):
    """记录命令发布"""
    timestamp = rospy.Time.now().to_sec()
    with open(log_file, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, command_name, command_value, round_num, sequence_num])

def publish_command_sequence(pub, rate, log_file, round_num, repeat=5):
    """
    发送一组预定义命令，每个命令重复若干次
    """
    command_sequence = [
        ("分散", 85),
        ("聚集", 170),
        ("对齐", 102),
        ("停止", 153)
    ]

    for name, cmd in command_sequence:
        rospy.loginfo(f"开始发送命令：{name}（值={cmd}），共 {repeat} 次")
        for i in range(repeat):
            msg = Int16()
            msg.data = cmd
            pub.publish(msg)
            # 记录命令发布
            log_command(log_file, name, cmd, round_num, i+1)
            rospy.loginfo(f"第 {i+1}/{repeat} 次 -> 发布命令: {cmd}")
            rate.sleep()
            rospy.sleep(4)
        rospy.loginfo(f"{name} 命令发送完毕\n")

def interactive_sender():
    """
    用户控制何时发送命令序列，每次发送5轮
    """
    rospy.init_node('command_sequence_interactive', anonymous=True)
    pub = rospy.Publisher('control_command', Int16, queue_size=10)
    rate = rospy.Rate(0.25)  # 0.5Hz，即每2秒一次

    # 初始化日志文件
    log_file = init_command_log()
    rospy.loginfo(f"命令日志文件已创建：{log_file}")

    print("命令交互程序已启动...")
    print("每次发送将包含5轮命令序列")
    print("每轮包含：分散 → 聚集 → 对齐 → 停止（每个命令重复5次）")
    print("按 Enter（或输入 y）发送5轮命令，输入 q 退出")

    while not rospy.is_shutdown():
        user_input = input("是否发送5轮命令？(y/Enter/q): ").strip().lower()
        if user_input in ['', 'y']:
            for round_num in range(3):
                print(f"\n开始执行第 {round_num + 1}/5 轮命令序列")
                publish_command_sequence(pub, rate, log_file, round_num + 1, repeat=5)
                if round_num < 4:
                    print("等待3秒后开始下一轮...")
                    rospy.sleep(5)
            print("\n5轮命令序列执行完毕")
        elif user_input == 'q':
            print("程序退出。")
            break
        else:
            print("无效输入，请输入 y、回车 或 q。")

if __name__ == '__main__':
    try:
        interactive_sender()
    except rospy.ROSInterruptException:
        pass
