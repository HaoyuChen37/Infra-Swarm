#!/usr/bin/env python3
import rospy
from cam.msg import Decoded, LeaderInfo
import numpy as np
import math

class CommandProcessor:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('command_processor', anonymous=True)
        
        # 订阅解码后的命令
        self.decoded_sub = rospy.Subscriber('/decoded', Decoded, self.decoded_callback)
        
        # 发布Leader信息
        self.leader_pub = rospy.Publisher('/leader_info', LeaderInfo, queue_size=10)
        
        # 速度参数
        self.max_speed = 0.2   # 最大速度
        
        rospy.loginfo("Command Processor Started")

    def decode_command(self, command):
        """
        将7位数字解码为方向和速度
        前5位表示方向（0-31，对应0-360度，0度为正右方）
        后2位表示速度（0-3，对应0-max_speed）
        """
        # 提取方向和速度值
        direction = (command >> 2) & 0x1F  # 获取前5位
        speed_level = command & 0x03       # 获取后2位
        
        # 将方向值（0-31）转换为角度（0-360）
        # 0度对应正右方，逆时针旋转
        angle = (direction * 360.0) / 32.0
        
        # 将速度等级（0-3）映射到实际速度
        speed = (speed_level / 3.0) * self.max_speed
        
        # 计算x和y方向的速度分量
        # 注意：angle已经是正确的角度，不需要额外调整
        angle_rad = math.radians(angle)
        vel_x = speed * math.cos(angle_rad)
        vel_y = speed * math.sin(angle_rad)
        
        return vel_x, vel_y

    def decoded_callback(self, msg):
        """
        处理解码后的命令
        """
        try:
            # 获取命令值和位置信息
            command = msg.command
            x_pos = msg.x
            y_pos = msg.y
            
            # 解码方向和速度
            vel_x, vel_y = self.decode_command(command)
            
            # 创建并发布LeaderInfo消息
            leader_msg = LeaderInfo()
            leader_msg.rel_x = x_pos
            leader_msg.rel_y = y_pos
            leader_msg.leader_vx = vel_x
            leader_msg.leader_vy = vel_y
            self.leader_pub.publish(leader_msg)
            
            # 记录日志
            rospy.loginfo(f"Received command: {command}")
            rospy.loginfo(f"Decoded velocities - X: {vel_x:.3f}, Y: {vel_y:.3f}")
            rospy.loginfo(f"Command position - X: {x_pos:.3f}, Y: {y_pos:.3f}")
            
        except Exception as e:
            rospy.logerr(f"Error processing command: {e}")

def main():
    try:
        processor = CommandProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 