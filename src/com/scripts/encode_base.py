#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int8
import sys

def encode_message(binary_data):
    """
    根据接收到的八位二进制编码（整数形式）进行解码，并构建编码后的消息。

    参数：
        binary_data (int): 八位二进制编码的整数。

    返回：
        str: 编码后的二进制消息。
    """
    # 确保binary_data是八位二进制数（0到255之间的整数）
    if not (0 <= binary_data <= 255):
        raise ValueError("输入数据必须在0到255之间。")

    # 将binary_data转换为8位二进制字符串
    binary_str = f"{binary_data:07b}"

    rospy.loginfo(f"Decoded data - {binary_str}")
    data_bits = binary_str # 总共7位数据位

    # 计算奇偶校验位（奇校验）
    parity_bit = '1' if data_bits.count('1') % 2 == 0 else '0'

    # 组成完整的消息
    start_bit = '0'
    stop_bits = '11'
    encoded_message = start_bit + data_bits + parity_bit + stop_bits

    # 重复消息三次
    repeated_message = encoded_message * 3
    repeated_message='0'+repeated_message

    return repeated_message

def control_command_callback(data):
    """
    订阅 control_command 话题的回调函数。
    解析接收到的八位二进制编码并逐个发布编码后的二进制位。
    """
    # 从自定义消息中获取二进制编码数据（int类型）
    binary_data = data.data # 假设data.direction已经是8位二进制数据的int表示
    
    # 编码消息
    encoded_message = encode_message(binary_data)
    rospy.loginfo(f"Encoded message: {encoded_message}")
    
    # 标记已接收到消息
    global received_message
    received_message = True
    
    # 逐个发布二进制位
    for bit in encoded_message:
        led_cmd = Int8()
        led_cmd.data = int(bit)  # 将 '0' 或 '1' 转换为整数
        pub.publish(led_cmd)
        rospy.loginfo(f"Publishing bit: {bit}")
        rate.sleep()  # 控制发布频率

    received_message = False

def publish_led_command():
    # 初始化 ROS 节点
    rospy.init_node('led_publisher', anonymous=True)
    
    # 创建一个发布者，发布到 'robot/ledcmd' 主题
    global pub, rate, received_message
    pub = rospy.Publisher('robot/ledcmd', Int8, queue_size=10)
    
    # 设置发布频率（例如 10 Hz）
    rate = rospy.Rate(10)  # 10 Hz
    
    # 订阅 control_command 话题，使用自定义消息类型 ControlCommand
    rospy.Subscriber('control_command', Int8, control_command_callback)
    
    # 初始化消息接收标志
    received_message = False

    while not rospy.is_shutdown():
        # 如果没有接收到消息，默认发送 1
        if not received_message:
            led_cmd = Int8()
            led_cmd.data = 1
            pub.publish(led_cmd)
            rospy.loginfo("No message received, publishing default: 1")
            received_message = False
        
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_led_command()
    except rospy.ROSInterruptException:
        pass
