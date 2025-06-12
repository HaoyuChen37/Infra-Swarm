#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8

def get_command_from_input():
    """
    从键盘输入获取方向和速度，返回对应的7位命令值
    前5位表示方向（0-31，对应0-360度）
    后2位表示速度（0-3，对应0-max_speed）
    """
    print("\n请输入控制命令：")
    print("方向参考值 (0-31):")
    print("0  - 正右方 (0°)")
    print("4  - 右前方 (45°)")
    print("8  - 正前方 (90°)")
    print("12 - 左前方 (135°)")
    print("16 - 正左方 (180°)")
    print("20 - 左后方 (225°)")
    print("24 - 正后方 (270°)")
    print("28 - 右后方 (315°)")
    print("...以此类推，每增加1个单位增加11.25°")
    print("\n速度等级:")
    print("0 - 停止")
    print("1 - 低速")
    print("2 - 中速")
    print("3 - 高速")
    
    # 获取方向输入
    while True:
        try:
            direction = int(input("\n请输入方向值 (0-31): "))
            if 0 <= direction <= 31:
                break
            print("方向值必须在0-31之间")
        except ValueError:
            print("请输入有效的数字")
    
    # 获取速度输入
    while True:
        try:
            speed = int(input("请输入速度等级 (0-3): "))
            if 0 <= speed <= 3:
                break
            print("速度等级必须在0-3之间")
        except ValueError:
            print("请输入有效的数字")
    
    # 组合命令：方向左移2位，加上速度
    command = (direction << 2) | speed
    return command

def publish_command():
    """
    初始化 ROS 节点并发布指令
    """
    rospy.init_node('command_publisher', anonymous=True)
    pub = rospy.Publisher('control_command', Int8, queue_size=10)
    rate = rospy.Rate(1)  # 1Hz，避免发送太快

    print("命令发布程序已启动...")
    print("此程序将向 control_command 话题发布控制命令")
    print("命令格式：7位二进制数")
    print("前5位：方向 (0-31)")
    print("后2位：速度 (0-3)")

    while not rospy.is_shutdown():
        try:
            cmd = get_command_from_input()
            msg = Int8()
            msg.data = cmd
            pub.publish(msg)
            
            # 解析并显示命令的详细信息
            direction = (cmd >> 2) & 0x1F
            speed = cmd & 0x03
            angle = (direction * 360.0) / 32.0
            print(f"\n发布命令: {cmd}")
            print(f"方向值: {direction} (角度: {angle:.1f}°)")
            print(f"速度等级: {speed}")
            
            rate.sleep()
            
        except KeyboardInterrupt:
            print("\n程序退出...")
            break
        except Exception as e:
            rospy.logerr(f"错误: {e}")
            continue

if __name__ == '__main__':
    try:
        publish_command()
    except rospy.ROSInterruptException:
        pass 