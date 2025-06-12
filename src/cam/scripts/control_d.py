#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from cam.msg import Tracker, LeaderInfo   # 假设 LeaderInfo 在 cam.msg 中定义
from geometry_msgs.msg import Twist
import numpy as np
import math

# -----------------------------
#  参数区
# -----------------------------

# Flocking 参数
SEPARATION_DISTANCE = 0.8    # 分离距离
ALIGNMENT_DISTANCE  = 1.3    # 对齐距离  
COHESION_DISTANCE   = 2.0    # 聚集距离

SEPARATION_WEIGHT   = 1.5    # 分离权重
ALIGNMENT_WEIGHT    = 1.0    # 对齐权重
COHESION_WEIGHT     = 0.8    # 聚集权重

MAX_SPEED = 0.1              # 最大速度

# Leader 模式相关
LEADER_TIMEOUT = 5.0         # （秒）如果超过此时长未收到 Leader 信息，就恢复 flocking 模式

# -----------------------------
#  主类定义
# -----------------------------
class FlockingController:
    def __init__(self):
        rospy.init_node('flocking_controller', anonymous=True)

        # 存储邻居信息
        self.all_neighbors = []

        # 当前控制模式：'flocking' 或 'leader_following'
        self.mode = 'flocking'

        # 存储上一次收到 Leader 信息的时间戳
        self.last_leader_time = rospy.Time.now()

        # 存储转换到自身坐标系下的 Leader 速度向量
        self.leader_vel_in_self = (0.0, 0.0)

        # Publisher：最终发布给底层驱动的速度指令
        self.velocity_publisher = rospy.Publisher('/robot/velcmd', Twist, queue_size=10)

        # Subscriber：订阅邻居（Tracker）信息，用于 Flocking
        rospy.Subscriber('/tracked_lights', Tracker, self.tracker_callback)

        # Subscriber：订阅 Leader 信息
        # 假设 LeaderInfo.msg 中包含字段：
        #   - leader_vx (float64)：Leader 在自己坐标系下的 x 速度分量
        #   - leader_vy (float64)：Leader 在自己坐标系下的 y 速度分量
        #   - rel_x      (float64)：Leader 相对你车自身坐标系下的 x 位置
        #   - rel_y      (float64)：Leader 相对你车自身坐标系下的 y 位置
        #
        # 如果你系统里实际的消息类型或字段名不同，请替换成对应的类型/字段。
        rospy.Subscriber('/leader_info', LeaderInfo, self.leader_callback)

        rospy.loginfo("Flocking Controller Started (with Leader Integration)")

    # -----------------------------
    #  节点回调与工具函数
    # -----------------------------
    def update_neighbors(self, data):
        """
        用 Tracker 话题的数据更新邻居列表
        data.lights 是一个列表，每个元素包含 x, y, vx, vy
        """
        self.all_neighbors = []
        for light in data.lights:
            relative_x = light.x
            relative_y = light.y
            distance   = math.hypot(relative_x, relative_y)

            neighbor = {
                'x':       relative_x,
                'y':       relative_y,
                'distance':distance,
                'vel_x':   light.vx,
                'vel_y':   light.vy
            }
            self.all_neighbors.append(neighbor)

    def calculate_separation(self):
        """计算分离力"""
        sep_x, sep_y = 0.0, 0.0
        count = 0
        for nb in self.all_neighbors:
            d = nb['distance']
            if 0 < d < SEPARATION_DISTANCE:
                sep_x -= nb['x'] / d
                sep_y -= nb['y'] / d
                count += 1
        if count > 0:
            sep_x /= count
            sep_y /= count
        return sep_x * SEPARATION_WEIGHT, sep_y * SEPARATION_WEIGHT

    def calculate_alignment(self):
        """计算对齐力"""
        align_x, align_y = 0.0, 0.0
        count = 0
        for nb in self.all_neighbors:
            if nb['distance'] < ALIGNMENT_DISTANCE:
                align_x += nb['vel_x']
                align_y += nb['vel_y']
                count += 1
        if count > 0:
            align_x /= count
            align_y /= count
        return align_x * ALIGNMENT_WEIGHT, align_y * ALIGNMENT_WEIGHT

    def calculate_cohesion(self):
        """计算聚集力"""
        cohes_x, cohes_y = 0.0, 0.0
        count = 0
        for nb in self.all_neighbors:
            if nb['distance'] < COHESION_DISTANCE:
                cohes_x += nb['x']
                cohes_y += nb['y']
                count += 1
        if count > 0:
            cohes_x /= count
            cohes_y /= count
        return cohes_x * COHESION_WEIGHT, cohes_y * COHESION_WEIGHT

    def calculate_flocking_velocity(self):
        """计算合并三条 Flocking 规则之后的速度"""
        if not self.all_neighbors:
            return 0.0, 0.0

        sep_x, sep_y   = self.calculate_separation()
        align_x, align_y = self.calculate_alignment()
        cohes_x, cohes_y = self.calculate_cohesion()

        total_x = sep_x + align_x + cohes_x
        total_y = sep_y + align_y + cohes_y

        speed = math.hypot(total_x, total_y)
        if speed > MAX_SPEED:
            total_x = total_x / speed * MAX_SPEED
            total_y = total_y / speed * MAX_SPEED

        return total_x, total_y

    # def transform_leader_velocity(self, v_leader, p_leader):
    #     """
    #     将 Leader 在自己坐标系下的速度指令（v_leader）转换到本车坐标系（self frame）
        # 将 v_leader 绕 z 轴旋转 θ，得到在本车坐标系下的速度分量。
        # """
        # vx_l, vy_l = v_leader
        # lx, ly = p_leader

        # # 估算 Leader 的朝向角（相对于本车坐标系）
        # theta = math.atan2(ly, lx)
        # cos_t = math.cos(theta)
        # sin_t = math.sin(theta)
        # v_leader: (vx, vy) —— Leader 坐标系下的速度向量
        # p_leader:  (x, y)  —— Leader 相对于本车坐标系的位置
        # 思路：假定 Leader x 轴指向 p_leader 的方向，则其朝向角 θ = atan2(y, x)。

        # # 旋转矩阵 R(θ)：从 Leader frame -> Self frame
        # R = np.array([[cos_t, -sin_t],
        #               [sin_t,  cos_t]])

        # v_vec = np.array([[vx_l],
        #                   [vy_l]])
        # v_in_self = R.dot(v_vec)

        # return float(v_in_self[0, 0]), float(v_in_self[1, 0])
    
    def transform_leader_velocity(self, v_leader, p_leader):
        # """
        # 将 Leader 在自身坐标系下的速度指令（v_leader）转换到本车坐标系（self frame）。
        # 假设 Leader 坐标系与本车坐标系一致且对齐。
        # 因此，速度分量直接转换。
        # v_leader: (vx, vy) —— Leader 坐标系下的速度向量
        # p_leader:  (x, y)  —— Leader 相对于本车坐标系的位置 (此参数在此假设下不再用于旋转计算)
        # """
        vx_l, vy_l = v_leader
        # 如果 Leader 坐标系与本车坐标系一致且对齐，直接返回速度分量
        return vx_l, vy_l


    def leader_callback(self, msg):
        """
        当收到 /leader_info 消息时触发，
        msg 中假设包含：
          - msg.leader_vx, msg.leader_vy : Leader 在自己坐标系下的速度
          - msg.rel_x, msg.rel_y         : Leader 在本车坐标系下的相对位置
        """
        # 从消息里提取 Leader 指令
        v_leader = (msg.leader_vx, msg.leader_vy)
        p_leader = (msg.rel_x, msg.rel_y)

        # 将 v_leader 转换到本车坐标系
        vx_self, vy_self = self.transform_leader_velocity(v_leader, p_leader)

        # 存储，并切换到 leader_following 模式
        self.leader_vel_in_self = (vx_self, vy_self)
        self.last_leader_time = rospy.Time.now()
        self.mode = 'leader_following'
        rospy.loginfo(f"[Leader Callback] mode -> leader_following, "
                      f"transformed vel: ({vx_self:.3f}, {vy_self:.3f})")

    def tracker_callback(self, data):
        """
        主回调：既用来更新邻居信息，也根据模式发布速度指令
        订阅 /tracked_lights (Tracker)
        """
        # 1) 更新邻居信息
        self.update_neighbors(data)

        # 2) 根据时间戳判断是否要恢复 flocking
        now = rospy.Time.now()
        elapsed = (now - self.last_leader_time).to_sec()
        if self.mode == 'leader_following' and elapsed > LEADER_TIMEOUT:
            self.mode = 'flocking'
            rospy.loginfo("[Tracker Callback] Leader 指令超时，切换回 flocking 模式")

        # 3) 选择速度来源
        if self.mode == 'leader_following':
            # 只跟随 Leader 指令，忽略 Flocking 计算
            vel_x, vel_y = self.leader_vel_in_self
        else:
            # 正常 Flocking 计算
            vel_x, vel_y = self.calculate_flocking_velocity()

        # # 4) 速度限幅（最多 MAX_SPEED）
        speed = math.hypot(vel_x, vel_y)
        if speed > MAX_SPEED:
            vel_x = vel_x / speed * MAX_SPEED
            vel_y = vel_y / speed * MAX_SPEED

        # 5) 发布最终速度
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.velocity_publisher.publish(twist)

        # 6) 打印日志
        rospy.loginfo(f"[{self.mode}] neighbors: {len(self.all_neighbors)}, speed: {speed:.3f}")

# -----------------------------
#  主函数
# -----------------------------
def main():
    controller = FlockingController()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
