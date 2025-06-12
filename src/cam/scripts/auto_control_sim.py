#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from cam.msg import Cam1, Cam2, Cam3, Cam4  # 假设这些自定义消息已在你的工作区定义并编译
from geometry_msgs.msg import Twist  # 导入Twist消息类型用于速度控制
import logging
import os # 导入os模块来检查和创建目录

# rospy.init_node('lights_info_subscriber', anonymous=True) # 节点初始化移至主函数或lights_info_subscriber函数内更合适

# 设置日志文件路径
log_dir = "Data"
log_file_path = os.path.join(log_dir, "logfile.log")

# 确保日志目录存在
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
logging.basicConfig(filename=log_file_path, level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


# 定义全局变量
lights_data = {'Cam1': [], 'Cam2': [], 'Cam3': [], 'Cam4': []}  # 存储四个相机的数据
MAX_SPEED = 0.5  # 最大线速度限制 (turtlesim中x方向)
# MAX_ANGULAR_SPEED = 1.0 # 如果要控制转向，可以定义一个最大角速度
MIN_DISTANCE = 0.5  # 最小安全距离
CAMERA_COUNT = 4  # 相机数量 (当前未使用)

velocity_publisher = None # 将发布者声明为全局变量，以便在回调和发布函数中使用

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
    logging.info(f"Updated lights_data for {cam_id}: {lights_data[cam_id]}")

    # 每次接收到相机数据后计算并发布速度
    calculate_and_publish_velocity()

# 计算速度
def calculate_velocity():
    global lights_data
    # 对于turtlesim, 我们主要关心 linear.x (前进/后退) 和 angular.z (旋转)
    # 当前的 vel[0] 将用于 linear.x
    # 当前的 vel[1] (基于 light['y']) 如果想让乌龟转向，需要映射到 angular_velocity_z
    # 这里我们暂时保持原样，vel[0] -> linear.x, vel[1] -> linear.y (将被turtlesim忽略)
    # 如果希望 light['y'] 控制转向，需要修改此处的逻辑
    
    linear_velocity_x = 0.0
    # angular_velocity_z = 0.0 # 如果要控制转向，初始化角速度
    
    # 以下是一个示例，如何将light['y']的影响转为角速度 (需要调整比例系数p_angular)
    # p_linear = 1.0  # 线性速度比例系数
    # p_angular = 1.0 # 角速度比例系数 (示例)
    # target_distance = 1.0 # 目标距离

    p = 1.0  # 原始比例系数
    dis = 1.0  # 原始目标距离

    accumulated_vx = 0.0
    accumulated_vy_effect = 0.0 # 用来累积原本计算vy的效果

    for cam_id, lights in lights_data.items():
        for light in lights:
            distance = light['distance']
            if distance == 0: # 避免除以零或无效距离
                rospy.logwarn(f"Invalid distance 0 from {cam_id} for light at ({light['x']}, {light['y']})")
                continue
            if distance < MIN_DISTANCE:
                rospy.logwarn(f"Target too close in {cam_id}! Distance: {distance}")
                # 也许在这里应该产生一个后退或者停止的指令
                continue

            # 原始速度计算逻辑
            accumulated_vx += p * light['x'] * (distance - dis)
            accumulated_vy_effect += p * light['y'] * (distance - dis) # 这个值原本是给 linear.y 的

    linear_velocity_x = accumulated_vx
    
    # --- 如何处理 accumulated_vy_effect ---
    # 选项1: 直接忽略 (turtlesim 不使用 linear.y)
    linear_velocity_y_ignored = accumulated_vy_effect

    # 选项2: 将其转换为角速度 (示例逻辑，需要根据实际需求调整)
    # 例如，可以简单地让 y 方向的偏移控制角速度
    # angular_velocity_z = -p_angular * accumulated_vy_effect # 负号可以用来调整转向，较大的y值导致转向
    # 或者使用 light['y'] 的平均值或最显著值来决定转向
    # angular_velocity_z = some_function_of_vy_effect(accumulated_vy_effect)

    # 限制线速度 (只针对x方向，因为turtlesim的 "speed" 主要是指linear.x)
    if abs(linear_velocity_x) > MAX_SPEED:
        linear_velocity_x = MAX_SPEED if linear_velocity_x > 0 else -MAX_SPEED

    # 如果你实现了 angular_velocity_z, 也可能需要限制它
    # if abs(angular_velocity_z) > MAX_ANGULAR_SPEED:
    #     angular_velocity_z = MAX_ANGULAR_SPEED if angular_velocity_z > 0 else -MAX_ANGULAR_SPEED
        
    # 返回一个包含linear.x和(如果你决定实现)angular.z的元组或字典
    # 为了简单起见，我们先只传递 (linear_x, linear_y_ignored)
    # 如果要控制转向，应该是 (linear_x, angular_z)
    return (linear_velocity_x, linear_velocity_y_ignored) # linear_velocity_y_ignored 会被放入Twist的linear.y

# 发布速度指令
def publish_velocity(velocity_components):
    global velocity_publisher
    if velocity_publisher is None:
        rospy.logerr("Velocity publisher is not initialized!")
        return

    velocity_msg = Twist()
    velocity_msg.linear.x = velocity_components[0]
    velocity_msg.linear.y = velocity_components[1] # turtlesim 会忽略这个值
    velocity_msg.linear.z = 0.0
    velocity_msg.angular.x = 0.0
    velocity_msg.angular.y = 0.0
    velocity_msg.angular.z = 0.0 # 如果要让乌龟旋转，需要在此处赋值

    # 如果你决定将 velocity_components[1] (或其它计算) 用作角速度:
    # velocity_msg.angular.z = velocity_components[1] # 或者你计算出的 angular_velocity_z

    velocity_publisher.publish(velocity_msg)
    logging.info(f"Published velocity to /turtle1/cmd_vel: Linear x={velocity_msg.linear.x}, Linear y={velocity_msg.linear.y}, Angular z={velocity_msg.angular.z}")
    # rospy.loginfo(f"Published velocity command: Linear x={velocity_msg.linear.x}, y={velocity_msg.linear.y}")

# 计算并发布速度
def calculate_and_publish_velocity():
    vel_components = calculate_velocity()
    publish_velocity(vel_components)

# 初始化订阅者和发布者
def lights_info_subscriber_node(): # Renamed function for clarity
    global velocity_publisher
    rospy.init_node('lights_info_to_turtlesim', anonymous=True) # 节点名可以更具体
    
    # 将发布主题更改为 turtlesim 的默认主题
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Velocity publisher created for /turtle1/cmd_vel")

    # 订阅四个相机的数据
    # 确保 cam.msg 中的 Cam1, Cam2, Cam3, Cam4 消息类型已正确定义和编译
    # 并且有节点在发布这些主题
    rospy.Subscriber('/Cam1', Cam1, lambda data: callback(data, 'Cam1'))
    rospy.Subscriber('/Cam2', Cam2, lambda data: callback(data, 'Cam2'))
    rospy.Subscriber('/Cam3', Cam3, lambda data: callback(data, 'Cam3'))
    rospy.Subscriber('/Cam4', Cam4, lambda data: callback(data, 'Cam4'))
    rospy.loginfo("Subscribed to /Cam1, /Cam2, /Cam3, /Cam4")

    rospy.spin()

if __name__ == '__main__':
    try:
        lights_info_subscriber_node()
    except rospy.ROSInterruptException:
        logging.info("ROS Interrupt Exception. Shutting down.")
        pass
    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}", exc_info=True)