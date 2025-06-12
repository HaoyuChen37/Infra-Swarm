#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8

def get_command_from_input():
    """
    从键盘输入获取指令，返回对应的 int 值（将八位二进制转为整数）
    """
    print("请选择指令：")
    print("1 - 搜索指令 (search)")
    print("2 - 收队指令 (return)")
    print("3 - 退出指令 (exit)")

    # 获取用户输入
    user_input = input("请输入指令数字 (1/2/3): ")

    # 返回指令对应的整数编码
    if user_input == '1':
        return int('00011011', 2)  # 27
    elif user_input == '2':
        return int('00101101', 2)  # 45
    elif user_input == '3':
        return int('01011011', 2)  # 91
    else:
        print("无效的输入，请输入 1、2 或 3。")
        return None

def publish_command():
    """
    初始化 ROS 节点并发布指令
    """
    rospy.init_node('task_command', anonymous=True)
    pub = rospy.Publisher('control_command', Int8, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz，避免发送太快

    while not rospy.is_shutdown():
        cmd = get_command_from_input()
        if cmd is not None:
            msg = Int8()
            msg.data = cmd
            pub.publish(msg)
            rospy.loginfo(f"发布 Int8 指令值: {cmd}（二进制: {cmd:08b}）")
        else:
            rospy.logwarn("指令无效，未发送。")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_command()
    except rospy.ROSInterruptException:
        pass
