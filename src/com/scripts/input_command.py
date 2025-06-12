#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8

def get_command_from_input():
    """
    从键盘输入获取指令，返回对应的命令值
    """
    print("\n请选择控制模式：")
    print("1 - 分散模式 (Separation)")
    print("2 - 聚集模式 (Cohesion)")
    print("3 - 对齐模式 (Alignment)")
    print("4 - 停止模式 (Stop)")
    print("5 - 退出程序 (Exit)")

    # 获取用户输入
    user_input = input("请输入模式数字 (1-5): ")

    # 返回指令对应的整数值
    if user_input == '1':
        return 85  # 分散命令
    elif user_input == '2':
        return 43  # 聚集命令
    elif user_input == '3':
        return 102  # 对齐命令
    elif user_input == '4':
        return 153  # 停止命令
    elif user_input == '5':
        return None  # 退出信号
    else:
        print("无效的输入，请输入 1-5 之间的数字。")
        return None

def publish_command():
    """
    初始化 ROS 节点并发布指令
    """
    rospy.init_node('command_interactive', anonymous=True)
    pub = rospy.Publisher('control_command', Int8, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz，避免发送太快

    print("命令交互程序已启动...")
    print("此程序将向 control_command 话题发布控制命令")

    while not rospy.is_shutdown():
        cmd = get_command_from_input()
        if cmd is None:
            print("程序退出...")
            break
        
        msg = Int8()
        msg.data = cmd
        pub.publish(msg)
        rospy.loginfo(f"发布命令: {cmd}")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_command()
    except rospy.ROSInterruptException:
        pass