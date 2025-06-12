#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from cam.msg import Cam1, Cam2, Cam3, Cam4
from geometry_msgs.msg import Twist
import numpy as np

# Flocking参数
SEPARATION_DISTANCE = 0.8    # 分离距离
ALIGNMENT_DISTANCE = 1.3     # 对齐距离  
COHESION_DISTANCE = 2.0      # 聚集距离

SEPARATION_WEIGHT = 1.5     # 分离权重（最重要，避免碰撞）
ALIGNMENT_WEIGHT = 1.0       # 对齐权重
COHESION_WEIGHT = 0.8        # 聚集权重

MAX_SPEED = 0.1              # 最大速度

class FlockingController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('simple_flocking_controller', anonymous=True)
        
        # 初始化全局变量
        self.all_neighbors = []           # 存储所有邻居
        self.velocity_publisher = rospy.Publisher('/robot/velcmd', Twist, queue_size=10)
        
        # 订阅四个相机
        rospy.Subscriber('/Cam1', Cam1, lambda data: self.callback(data, 'Cam1'))
        rospy.Subscriber('/Cam2', Cam2, lambda data: self.callback(data, 'Cam2'))
        rospy.Subscriber('/Cam3', Cam3, lambda data: self.callback(data, 'Cam3'))
        rospy.Subscriber('/Cam4', Cam4, lambda data: self.callback(data, 'Cam4'))
        
        rospy.loginfo("Simple Flocking Controller Started")
        rospy.loginfo("Rules: Separation + Alignment + Cohesion")

    def update_neighbors(self, data, cam_id):
        """更新邻居信息"""
        # 清空当前相机的数据
        self.all_neighbors = [n for n in self.all_neighbors if n['cam'] != cam_id]
        
        # 添加新数据
        for light in data.lights:
            # 计算相对位置（相对于机器人）
            relative_x = light.x
            relative_y = light.y
            distance = np.sqrt(relative_x**2 + relative_y**2)
            
            neighbor = {
                'x': relative_x,
                'y': relative_y,
                'distance': distance,
                'vel_x': light.vx,
                'vel_y': light.vy,
                'cam': cam_id
            }
            self.all_neighbors.append(neighbor)

    def calculate_separation(self):
        """计算分离力 - 远离太近的邻居"""
        sep_x, sep_y = 0, 0
        count = 0
        
        for neighbor in self.all_neighbors:
            dist = neighbor['distance']
            if 0 < dist < SEPARATION_DISTANCE:
                # 计算远离方向（相反方向）
                sep_x -= neighbor['x'] / dist  # 归一化并反向
                sep_y -= neighbor['y'] / dist
                count += 1
        
        if count > 0:
            sep_x /= count
            sep_y /= count
            
        return sep_x * SEPARATION_WEIGHT, sep_y * SEPARATION_WEIGHT

    def calculate_alignment(self):
        """计算对齐力 - 与邻居速度保持一致"""
        align_x, align_y = 0, 0
        count = 0
        
        for neighbor in self.all_neighbors:
            if neighbor['distance'] < ALIGNMENT_DISTANCE:
                align_x += neighbor['vel_x']
                align_y += neighbor['vel_y']
                count += 1
        
        if count > 0:
            align_x /= count
            align_y /= count
            
        return align_x * ALIGNMENT_WEIGHT, align_y * ALIGNMENT_WEIGHT

    def calculate_cohesion(self):
        """计算聚集力 - 向邻居中心移动"""
        cohes_x, cohes_y = 0, 0
        count = 0
        
        for neighbor in self.all_neighbors:
            if neighbor['distance'] < COHESION_DISTANCE:
                cohes_x += neighbor['x']
                cohes_y += neighbor['y']
                count += 1
        
        if count > 0:
            cohes_x /= count
            cohes_y /= count
            
        return cohes_x * COHESION_WEIGHT, cohes_y * COHESION_WEIGHT

    def calculate_flocking_velocity(self):
        """计算最终的flocking速度"""
        if not self.all_neighbors:
            return 0, 0
        
        # 计算三个flocking力
        sep_x, sep_y = self.calculate_separation()
        align_x, align_y = self.calculate_alignment()
        cohes_x, cohes_y = self.calculate_cohesion()
        
        # 合并所有力
        total_x = sep_x + align_x + cohes_x
        total_y = sep_y + align_y + cohes_y
        
        # 限制最大速度
        speed = np.sqrt(total_x**2 + total_y**2)
        if speed > MAX_SPEED:
            total_x = total_x / speed * MAX_SPEED
            total_y = total_y / speed * MAX_SPEED
        
        return total_x, total_y

    def callback(self, data, cam_id):
        """相机数据回调"""
        self.update_neighbors(data, cam_id)
        
        # 计算并发布速度
        vel_x, vel_y = self.calculate_flocking_velocity()
        
        # 发布速度指令
        velocity_msg = Twist()
        velocity_msg.linear.x = vel_x
        velocity_msg.linear.y = vel_y
        self.velocity_publisher.publish(velocity_msg)
        
        # 简单的状态日志
        neighbor_count = len(self.all_neighbors)
        current_speed = np.sqrt(vel_x**2 + vel_y**2)
        rospy.loginfo(f"Neighbors: {neighbor_count}, Speed: {current_speed:.3f}")

def main():
    """主函数"""
    controller = FlockingController()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass