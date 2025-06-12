#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import Twist

class CommandParser:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('command_parser', anonymous=True)
        
        # 订阅解码后的消息
        self.decoded_sub = rospy.Subscriber('/decoded', Int32, self.decoded_callback)
        
        # 发布控制参数
        self.param_pub = rospy.Publisher('/flocking_params', Float32MultiArray, queue_size=10)
        
        # 定义命令映射
        self.commands = {
            81: self.publish_separation_params,    # 分散命令
            100: self.publish_cohesion_params,     # 聚集命令
            20: self.publish_align_params,         # 对齐命令
            49: self.publish_stop_params,          # 停止命令
            30: self.publish_direction_params,     # 方向移动命令
        }
        
        # 定义不同模式下的参数
        self.separation_params = {
            'SEPARATION_DISTANCE': 1.2,    # 增加分离距离
            'ALIGNMENT_DISTANCE': 1.0,     # 减小对齐距离
            'COHESION_DISTANCE': 1.5,      # 减小聚集距离
            'SEPARATION_WEIGHT': 2.0,      # 增加分离权重
            'ALIGNMENT_WEIGHT': 0.5,       # 减小对齐权重
            'COHESION_WEIGHT': 0.3,        # 减小聚集权重
            'MAX_SPEED': 0.1             # 增加最大速度
        }
        
        self.cohesion_params = {
            'SEPARATION_DISTANCE': 0.6,    # 减小分离距离
            'ALIGNMENT_DISTANCE': 1.5,     # 增加对齐距离
            'COHESION_DISTANCE': 2.5,      # 增加聚集距离
            'SEPARATION_WEIGHT': 1.0,      # 减小分离权重
            'ALIGNMENT_WEIGHT': 1.2,       # 增加对齐权重
            'COHESION_WEIGHT': 1.5,        # 增加聚集权重
            'MAX_SPEED': 0.1             # 减小最大速度
        }

        self.align_params = {
            'SEPARATION_DISTANCE': 0.8,    # 中等分离距离
            'ALIGNMENT_DISTANCE': 2.0,     # 增加对齐距离
            'COHESION_DISTANCE': 1.5,      # 中等聚集距离
            'SEPARATION_WEIGHT': 1.0,      # 中等分离权重
            'ALIGNMENT_WEIGHT': 2.0,       # 增加对齐权重
            'COHESION_WEIGHT': 0.8,        # 中等聚集权重
            'MAX_SPEED': 0.1               # 中等速度
        }

        self.stop_params = {
            'SEPARATION_DISTANCE': 0.8,    # 保持默认值
            'ALIGNMENT_DISTANCE': 1.3,     # 保持默认值
            'COHESION_DISTANCE': 2.0,      # 保持默认值
            'SEPARATION_WEIGHT': 0.0,      # 停止分离
            'ALIGNMENT_WEIGHT': 0.0,       # 停止对齐
            'COHESION_WEIGHT': 0.0,        # 停止聚集
            'MAX_SPEED': 0.0               # 停止移动
        }

        self.direction_params = {
            'SEPARATION_DISTANCE': 0.8,    # 保持默认值
            'ALIGNMENT_DISTANCE': 1.3,     # 保持默认值
            'COHESION_DISTANCE': 2.0,      # 保持默认值
            'SEPARATION_WEIGHT': 0.0,      # 禁用分离
            'ALIGNMENT_WEIGHT': 0.0,       # 禁用对齐
            'COHESION_WEIGHT': 0.0,        # 禁用聚集
            'MAX_SPEED': 0.15              # 设置移动速度
        }
        
        rospy.loginfo("Command Parser Started")
        rospy.loginfo("Available commands:")
        rospy.loginfo("81: Separation mode")
        rospy.loginfo("100: Cohesion mode")
        rospy.loginfo("20: Alignment mode")
        rospy.loginfo("49: Stop mode")
        rospy.loginfo("30: Direction movement mode")

    def decoded_callback(self, data):
        """
        处理解码后的消息
        """
        command = data.data
        rospy.loginfo(f"Received command: {command}")
        
        # 检查命令是否在映射中
        if command in self.commands:
            # 执行对应的命令处理函数
            self.commands[command]()
        else:
            rospy.logwarn(f"Unknown command: {command}")

    def publish_params(self, params):
        """
        发布控制参数
        """
        msg = Float32MultiArray()
        # 按照固定顺序发布参数
        msg.data = [
            params['SEPARATION_DISTANCE'],
            params['ALIGNMENT_DISTANCE'],
            params['COHESION_DISTANCE'],
            params['SEPARATION_WEIGHT'],
            params['ALIGNMENT_WEIGHT'],
            params['COHESION_WEIGHT'],
            params['MAX_SPEED']
        ]
        self.param_pub.publish(msg)
        rospy.loginfo(f"Published parameters: {params}")

    def publish_separation_params(self):
        """
        发布分散模式的参数
        """
        self.publish_params(self.separation_params)
        rospy.loginfo("Published separation parameters")

    def publish_cohesion_params(self):
        """
        发布聚集模式的参数
        """
        self.publish_params(self.cohesion_params)
        rospy.loginfo("Published cohesion parameters")

    def publish_align_params(self):
        """
        发布对齐模式的参数
        """
        self.publish_params(self.align_params)
        rospy.loginfo("Published alignment parameters")

    def publish_stop_params(self):
        """
        发布停止模式的参数
        """
        self.publish_params(self.stop_params)
        rospy.loginfo("Published stop parameters")

    def publish_direction_params(self):
        """
        发布方向移动模式的参数
        """
        self.publish_params(self.direction_params)
        rospy.loginfo("Published direction movement parameters")

def main():
    try:
        parser = CommandParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()