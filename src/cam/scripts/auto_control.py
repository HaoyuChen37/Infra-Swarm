#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from cam.msg import Cam1, Cam2, Cam3, Cam4
from geometry_msgs.msg import Twist  # 导入Twist消息类型用于速度控制
import logging

rospy.init_node('lights_info_subscriber', anonymous=True)

# 设置日志文件路径
log_file_path = "Data/logfile.log"
logging.basicConfig(filename=log_file_path, level=logging.INFO)


# 定义全局变量
lights_data = {'Cam1': [], 'Cam2': [], 'Cam3': [], 'Cam4': []}  # 存储四个相机的数据
MAX_SPEED = 0.5  # 最大速度限制
MIN_DISTANCE = 0.5  # 最小安全距离
CAMERA_COUNT = 4  # 相机数量

# 回调函数：处理相机数据
def callback(data, cam_id):
    # rospy.loginfo(f"Received data from {cam_id}:")
    lights_list = []

    for light in data.lights:
        light_dict = {
            'x': light.x,
            'y': light.y,
            'distance': light.distance
        }
        lights_list.append(light_dict)

    lights_data[cam_id] = lights_list
    # rospy.loginfo(f"Updated lights_data for {cam_id}: {lights_data[cam_id]}")

    # 每次接收到相机数据后计算并发布速度
    calculate_and_publish_velocity()

# 计算速度
def calculate_velocity():
    global lights_data
    vel = (0, 0)
    p = 1.0  # 比例系数
    dis = 1.0  # 目标距离

    for cam_id, lights in lights_data.items():
        for light in lights:
            distance = light['distance']
            if distance < MIN_DISTANCE:
                rospy.logwarn(f"Target too close in {cam_id}! Distance: {distance}")
                continue

            # 调整速度计算公式
            vel = (
                vel[0] + p * light['x'] * (distance - dis),  # 注意负号
                vel[1] + p * light['y'] * (distance - dis)
            )

    # 限制速度
    speed = (vel[0]**2 + vel[1]**2)**0.5
    if speed > MAX_SPEED:
        vel = (vel[0] / speed * MAX_SPEED, vel[1] / speed * MAX_SPEED)

    return vel

# 发布速度指令
def publish_velocity(velocity):
    velocity_msg = Twist()
    velocity_msg.linear.x = velocity[0]
    velocity_msg.linear.y = velocity[1]
    velocity_publisher.publish(velocity_msg)
    # rospy.loginfo(f"Published velocity command: Linear x={velocity[0]}, y={velocity[1]}")

# 计算并发布速度
def calculate_and_publish_velocity():
    vel = calculate_velocity()
    publish_velocity(vel)

# 初始化订阅者和发布者
def lights_info_subscriber():
    global velocity_publisher
    rospy.init_node('lights_info_subscriber', anonymous=True)
    velocity_publisher = rospy.Publisher('/robot/velcmd', Twist, queue_size=10)

    # 订阅四个相机的数据
    rospy.Subscriber('/Cam1', Cam1, lambda data: callback(data, 'Cam1'))
    rospy.Subscriber('/Cam2', Cam2, lambda data: callback(data, 'Cam2'))
    rospy.Subscriber('/Cam3', Cam3, lambda data: callback(data, 'Cam3'))
    rospy.Subscriber('/Cam4', Cam4, lambda data: callback(data, 'Cam4'))

    rospy.spin()

if __name__ == '__main__':
    try:
        lights_info_subscriber()
        # calculate_and_publish_velocity()
    except rospy.ROSInterruptException:
        pass