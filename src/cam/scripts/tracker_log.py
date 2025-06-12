#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
import numpy as np
import threading
import sys
import csv
import queue
import time
from datetime import datetime
sys.path.append('/home/nvidia/swarm/devel/lib/python3/dist-packages')
from cam.msg import LightInfo, Cam1, Cam2, Cam3, Cam4, TrackStatus, g_r,Tracker
from ocsort import OCSort
from collections import deque
from std_msgs.msg import Header

class UnifiedTracker:
    def __init__(self, output_path, tracker_params):
        self.output_path = output_path
        self.tracker_params = tracker_params
        self.tracker = OCSort(**tracker_params)
        self.frame_id = 0
        self.lock = threading.Lock()

        # 使用队列存储每个相机的数据，设置最大长度为10
        self.camera_queues = {
            1: deque(maxlen=10),
            2: deque(maxlen=10),
            3: deque(maxlen=10),
            4: deque(maxlen=10),
        }

        # 初始化日志记录
        self.log_queue = queue.Queue(maxsize=1000)  # 设置队列大小限制
        self.init_tracking_log()
        self.start_log_thread()

        # ROS发布者
        self.tracking_pub = rospy.Publisher('/unified_tracker', TrackStatus, queue_size=10)
        self.tracked_lights_pub = rospy.Publisher('/tracked_lights', Tracker, queue_size=10)

        # 添加数据记录相关的属性
        self.tracker_data = {
            'KF_pose': [],  # 位置数据
            'KF_vel': []    # 速度数据
        }
        
        # 创建日志目录
        self.log_dir = os.path.expanduser('~/tracker_data')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # 创建日志文件
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_dir, f'tracker_data_{timestamp}.csv')
        
        # 初始化CSV文件
        with open(self.log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'frame_id', 'track_id', 'x', 'y', 'z', 'vx', 'vy', 'vz'])

    def init_tracking_log(self):
        """初始化跟踪日志"""
        # 创建日志目录
        log_dir = os.path.join(os.path.expanduser('~'), 'tracker_logs')
        os.makedirs(log_dir, exist_ok=True)
        
        # 创建日志文件名（使用时间戳）
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.tracking_log = os.path.join(log_dir, f'tracking_log_{timestamp}.csv')
        
        # 初始化CSV文件
        with open(self.tracking_log, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'frame_id', 'track_id', 'status', 'x', 'y', 'vx', 'vy'])

    def start_log_thread(self):
        """启动日志记录线程"""
        self.log_thread = threading.Thread(target=self._log_worker, daemon=True)
        self.log_thread.start()

    def _log_worker(self):
        """日志记录工作线程"""
        buffer = []
        last_write_time = time.time()
        buffer_size = 100  # 缓冲区大小
        write_interval = 1.0  # 写入间隔（秒）

        while not rospy.is_shutdown():
            try:
                # 非阻塞方式获取日志数据
                while len(buffer) < buffer_size:
                    try:
                        log_data = self.log_queue.get_nowait()
                        buffer.append(log_data)
                    except queue.Empty:
                        break

                current_time = time.time()
                # 如果缓冲区已满或达到写入时间间隔，执行写入
                if len(buffer) >= buffer_size or (buffer and current_time - last_write_time >= write_interval):
                    with open(self.tracking_log, 'a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerows(buffer)
                    buffer.clear()
                    last_write_time = current_time

                time.sleep(0.01)  # 短暂休眠，避免CPU占用过高
            except Exception as e:
                rospy.logerr(f"Error in log worker: {e}")

    def log_tracking(self, frame_id, track_id, status, x, y, vx=0.0, vy=0.0):
        """记录跟踪状态（非阻塞方式）"""
        timestamp = rospy.Time.now().to_sec()
        try:
            self.log_queue.put_nowait([timestamp, frame_id, track_id, status, x, y, vx, vy])
        except queue.Full:
            rospy.logwarn("Log queue is full, dropping log entry")

    def save_tracker_data(self, frame_id, track_id, x, y, z, vx, vy, vz):
        """保存跟踪器数据"""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        with open(self.log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, frame_id, track_id, x, y, z, vx, vy, vz])
        
        # 同时保存到内存中用于实时分析
        self.tracker_data['KF_pose'].append([x, y, z])
        self.tracker_data['KF_vel'].append([vx, vy, vz])

    def process_data(self):
        """使用所有相机当前最新的方向向量和距离数据进行处理"""
        with self.lock:
            # 检查所有相机队列是否都有数据
            if not all(len(queue) > 0 for queue in self.camera_queues.values()):
                return

            current_ids = set()
            tracked_lights = []
            all_dets = []

            # 从每个相机队列中取出最早的数据进行处理
            for cam_id in range(1, 5):
                g_r_data = self.camera_queues[cam_id].popleft()  # 取出并移除最早的数据

                if g_r_data:
                    dets = []
                    for light_data in g_r_data.lights:
                        g = np.array([light_data.x, light_data.y, 0])
                        g_norm = np.linalg.norm(g)
                        if g_norm > 0:
                            g = g / g_norm
                        else:
                            rospy.logwarn(f"Camera {cam_id} received zero vector for light data in frame {self.frame_id}.")
                            continue

                        r = light_data.distance
                        score = 1.0
                        if len(g) == 3 and not np.any(np.isnan(g)) and not np.isnan(r):
                            dets.append((g, r, score))
                        else:
                            rospy.logwarn(f"Camera {cam_id} received invalid data (NaN or wrong dim) in frame {self.frame_id}: g={g}, r={r}")

                    if dets:
                        all_dets.extend(dets)

            if all_dets:
                online_targets = self.tracker.update(all_dets)

                for t in online_targets:
                    if len(t) >= 4:
                        x, y, z, tid = t[:4]
                        current_ids.add(tid)

                        # 获取速度信息
                        vx = t[4] if len(t) >= 6 else 0.0
                        vy = t[5] if len(t) >= 6 else 0.0

                        # 记录跟踪状态（非阻塞方式）
                        self.log_tracking(self.frame_id, int(tid), 1, x, y, vx, vy)

                        # 保存跟踪器数据
                        self.save_tracker_data(self.frame_id, int(tid), x, y, 0, vx, vy, 0)

                        # 发布跟踪消息
                        track_msg = TrackStatus()
                        track_msg.track_id = int(tid)
                        track_msg.status = 1
                        track_msg.x = x
                        track_msg.y = y
                        self.tracking_pub.publish(track_msg)

                        light = LightInfo()
                        light.x = x
                        light.y = y
                        light.vx = vx
                        light.vy = vy
                        tracked_lights.append(light)

            # 获取当前所有跟踪器的状态
            tracked_states = self.tracker.get_state()
            
            # 处理持续跟踪的目标
            if tracked_states:
                for state in tracked_states:
                    if len(state) >= 3:  # 确保有完整的坐标[x,y,z]
                        x, y, z, tid, vx, vy, vz = state
                        if tid not in current_ids:
                            # 记录跟踪状态（非阻塞方式）
                            self.log_tracking(self.frame_id, int(tid), 0, x, y, vx, vy)
                            
                            # 保存跟踪器数据
                            self.save_tracker_data(self.frame_id, int(tid), x, y, 0, vx, vy, 0)
                            
                            # 发布跟踪消息
                            track_msg = TrackStatus()
                            track_msg.track_id = int(tid)
                            track_msg.status = 0
                            track_msg.x = x
                            track_msg.y = y
                            self.tracking_pub.publish(track_msg)

            if tracked_lights:
                self.tracked_lights_pub.publish(Tracker(lights=tracked_lights))
                rospy.loginfo(f"Published {len(tracked_lights)} tracked lights for frame {self.frame_id}.")
            else:
                rospy.loginfo(f"Processed frame {self.frame_id}, but no lights tracked.")

            self.frame_id += 1


class TrackerNode:
    def __init__(self):
        rospy.init_node('unified_tracker_node', anonymous=True)

        # Initialize unified tracker
        output_path = "/home/nvidia/tracking_results"
        tracker_params = {
            "det_thresh": 0.5,
            "iou_threshold": 0.3,
            "use_byte": False,
        }
        self.tracker = UnifiedTracker(output_path, tracker_params)

        # 初始化订阅者为None
        self.cam1_sub = None
        self.cam2_sub = None
        self.cam3_sub = None
        self.cam4_sub = None

        # 创建定时器，4秒后开始订阅
        rospy.loginfo("Waiting 4 seconds before subscribing to camera topics...")
        rospy.Timer(rospy.Duration(10.0), self.start_subscriptions, oneshot=True)

    def start_subscriptions(self, event):
        """4秒后开始订阅相机话题"""
        rospy.loginfo("Starting camera subscriptions...")
        
        # 等待并获取每个话题的最新消息
        try:
            rospy.loginfo("Waiting for initial messages from all cameras...")
            msg1 = rospy.wait_for_message('/Cam1', Cam1, timeout=0.1)
            msg2 = rospy.wait_for_message('/Cam2', Cam2, timeout=0.1)
            msg3 = rospy.wait_for_message('/Cam3', Cam3, timeout=0.1)
            msg4 = rospy.wait_for_message('/Cam4', Cam4, timeout=0.1)
            rospy.loginfo("Received initial messages from all cameras")
            
            # 清空所有相机的队列，确保从最新状态开始
            for cam_id in range(1, 5):
                self.tracker.camera_queues[cam_id].clear()
            
            # 创建正式的订阅者
            self.cam1_sub = rospy.Subscriber('/Cam1', Cam1, lambda msg: self.camera_callback(msg, 1), queue_size=100)
            self.cam2_sub = rospy.Subscriber('/Cam2', Cam2, lambda msg: self.camera_callback(msg, 2), queue_size=100)
            self.cam3_sub = rospy.Subscriber('/Cam3', Cam3, lambda msg: self.camera_callback(msg, 3), queue_size=100)
            self.cam4_sub = rospy.Subscriber('/Cam4', Cam4, lambda msg: self.camera_callback(msg, 4), queue_size=100)
            
            rospy.loginfo("All camera subscriptions started successfully")
            
        except rospy.ROSException as e:
            rospy.logerr(f"Error during initialization: {str(e)}")
            rospy.logwarn("Continuing with subscriptions despite timeout...")
            
            # 即使超时也创建订阅者
            self.cam1_sub = rospy.Subscriber('/Cam1', Cam1, lambda msg: self.camera_callback(msg, 1), queue_size=100)
            self.cam2_sub = rospy.Subscriber('/Cam2', Cam2, lambda msg: self.camera_callback(msg, 2), queue_size=100)
            self.cam3_sub = rospy.Subscriber('/Cam3', Cam3, lambda msg: self.camera_callback(msg, 3), queue_size=100)
            self.cam4_sub = rospy.Subscriber('/Cam4', Cam4, lambda msg: self.camera_callback(msg, 4), queue_size=100)

    def camera_callback(self, msg, camera_id):
        """处理单个相机的回调"""
        # 将新数据添加到对应相机的队列中
        if msg and hasattr(msg, 'lights'):
            self.tracker.camera_queues[camera_id].append(msg)
            # 尝试处理数据
            self.tracker.process_data()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tracker_node = TrackerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 