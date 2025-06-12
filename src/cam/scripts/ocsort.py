"""
    This script is adopted from the SORT script by Alex Bewley alex@bewley.ai
"""
from __future__ import print_function

import numpy as np
from association import *


def k_previous_obs(observations, cur_age, k):
    """
    获取k帧之前的观测值
    参数:
        observations: 观测值字典，键为帧号，值为观测值
        cur_age: 当前帧号
        k: 回溯的帧数
    返回:
        6维观测向量，如果没有观测值则返回全-1的向量
    """
    if len(observations) == 0:
        return np.array([-1, -1, -1])
    
    for i in range(k):
        dt = k - i
        if cur_age - dt in observations:
            obs = observations[cur_age-dt]
            # 将观测值转换为6维向量
            g, r = obs
            g = np.asarray(g).reshape(3)
            return convert_x_to_bbox((g, r))  # 返回6维向量
    
    # 如果没有找到合适的观测值，返回最新的观测值
    max_age = max(observations.keys())
    obs = observations[max_age]
    g, r = obs
    g = np.asarray(g).reshape(3)
    return convert_x_to_bbox((g, r))   # 返回6维向量


def convert_bbox_to_z(bbox):
    """
    将方向向量g和距离r转换为伪线性观测向量z
    参数:
        bbox: 格式为(hat_g, hat_r)的元组/列表，其中hat_g是3维方向向量，hat_r是标量距离
    返回:
        z: 6维伪线性观测向量，格式为[0_3x1; hat_r*hat_g]
    """
    if not (isinstance(bbox, (list, tuple)) and len(bbox) >= 2):
        raise ValueError("convert_bbox_to_z的输入必须是(hat_g, hat_r)格式的元组/列表")
    
    hat_g = np.asarray(bbox[0]).reshape((3, 1))  # 确保hat_g是3x1列向量
    hat_r = float(bbox[1])
    
    # 构建伪线性观测向量 z = [0_3x1; hat_r*hat_g]
    z = np.zeros((6, 1))
    z[3:6] = hat_r * hat_g
    
    return z


def convert_x_to_bbox(x, score=None):
    """
    将方向向量g和距离r转换为位置，用于匹配
    参数:
        x: 格式为[g[0], g[1], g[2], r]的数组
        score: 可选的置信度分数
    返回:
        如果score为None: 返回3维位置向量 [x, y, z]
        如果score不为None: 返回带置信度的位置向量 [x, y, z, score]
    """
    # 解析输入参数x，格式为[g[0], g[1], g[2], r]
    if not (isinstance(x, (list, tuple, np.ndarray)) and len(x) >= 2):
        raise ValueError("convert_x_to_bbox的输入必须是[g[0], g[1], g[2], r]格式的数组")
    
    hat_g = np.asarray(x[0]).reshape((3, 1))  # 确保hat_g是3x1列向量
    hat_r = float(x[1])
    
    # 计算位置: position = hat_r * hat_g
    position = hat_r * hat_g
    
    if score is None:
        return position.reshape(3)  # 返回3维位置向量
    else:
        return np.append(position.reshape(3), score)  # 返回带置信度的位置向量


def speed_direction(pos1, pos2):
    """
    计算3D空间中的速度方向
    参数:
        pos1: 第一个3D位置向量 [x1, y1, z1]
        pos2: 第二个3D位置向量 [x2, y2, z2]
    返回:
        归一化的3D速度方向向量 [vx, vy, vz]
    """
    # 计算位移向量
    velocity = np.array([
        pos2[0] - pos1[0],  # x方向位移
        pos2[1] - pos1[1],  # y方向位移
        pos2[2] - pos1[2]   # z方向位移
    ])
    
    # 计算位移向量的模长
    norm = np.sqrt(np.sum(velocity**2)) + 1e-6
    
    # 返回归一化的速度方向
    return velocity / norm


class KalmanBoxTracker(object):
    """
    这个类表示单个被跟踪对象的内部状态，使用卡尔曼滤波器进行状态估计
    状态向量包含位置和速度信息
    """
    count = 0

    def __init__(self, bbox, delta_t=3, orig=False):  # bbox现在是[g[0], g[1], g[2], r]格式
        """
        使用初始测量值[g[0], g[1], g[2], r]初始化跟踪器
        状态向量为 [dp_x, dp_y, dp_z, dv_x, dv_y, dv_z]^T
        """
        self.kf_dt = 0.03  # 卡尔曼滤波器内部动力学的时间步长

        # 解析输入参数bbox，格式为[g[0], g[1], g[2], r]
        if not (isinstance(bbox, (list, tuple, np.ndarray)) and len(bbox) >=2 ):
            raise ValueError("KalmanBoxTracker的`bbox`参数必须是[g[0], g[1], g[2], r]格式的数组")

        # 从输入数组中提取方向向量和距离
        initial_hat_g = np.asarray(bbox[0]).reshape((3, 1))  # 确保hat_g是3x1列向量
        initial_hat_r = float(bbox[1])
 

        dim_x = 6  # 状态维度：[dp_x, dp_y, dp_z, dv_x, dv_y, dv_z]
        dim_z = 6  # 伪观测维度：z = [0_3x1; hat_r*hat_g]

        # 初始化卡尔曼滤波器
        if not orig:
            from kalmanfilter import KalmanFilterNew as KalmanFilter
            self.kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z)
        else:
            from filterpy.kalman import KalmanFilter
            self.kf = KalmanFilter(dim_x=dim_x, dim_z=dim_z)

        # 1. 状态转移矩阵F (6x6)
        # F = [[I_3, dt*I_3], [0_3, I_3]]
        self.kf.F = np.eye(dim_x)
        self.kf.F[0:3, 3:6] = self.kf_dt * np.eye(3)

        # 2. 观测矩阵H (6x6)
        # H = [[P_hat_g, 0_3x3], [I_3, 0_3x3]]，其中P_hat_g = I_3 - hat_g * hat_g^T
        P_hat_g_initial = np.eye(3) - (initial_hat_g @ initial_hat_g.T)
        self.kf.H = np.zeros((dim_z, dim_x))
        self.kf.H[0:3, 0:3] = P_hat_g_initial  # 左上块用于P_hat_g * dp
        self.kf.H[3:6, 0:3] = np.eye(3)        # 左下块用于I_3 * dp

        # 3. 过程噪声协方差Q (6x6)
        # Q = B * Sigma_q_accel_diag * B.T
        sigma_q_val = 0.02  # 目标加速度噪声的标准差（可调）
        B_matrix = np.zeros((dim_x, 3))
        B_matrix[0:3, 0:3] = (0.5 * self.kf_dt**2) * np.eye(3)
        B_matrix[3:6, 0:3] = self.kf_dt * np.eye(3)
        Sigma_q_accel_diag = sigma_q_val**2 * np.eye(3)
        self.kf.Q = B_matrix @ Sigma_q_accel_diag @ B_matrix.T

        # 4. 测量噪声协方差R (6x6)
        sigma_mu_sq_val = 0.2 # 方向噪声的方差
        sigma_w_sq_val = 0.4    # 距离噪声的方差
        
        # 构建P_hat_g矩阵
        P_hat_g = np.eye(3) - (initial_hat_g @ initial_hat_g.T)
        
        # 构建R矩阵
        self.kf.R = np.zeros((dim_z, dim_z))
        # 左上块: r²σμ²P_hat_g
        self.kf.R[0:3, 0:3] = (initial_hat_r**2) * sigma_mu_sq_val * P_hat_g
        # 右下块: σw²hat_g hat_g^T
        self.kf.R[3:6, 3:6] = sigma_w_sq_val * (initial_hat_g @ initial_hat_g.T)
        # 右上和左下块保持为0矩阵

        # 5. 初始状态协方差P (6x6)
        self.kf.P = np.eye(dim_x) * 1.0  # 默认初始不确定性
        self.kf.P[0:3, 0:3] *= 10.0      # 初始相对位置的不确定性更高
        self.kf.P[3:6, 3:6] *= 100.0     # 初始相对速度的不确定性最高（未观测）

        # 6. 初始状态x (6x1)
        dp_initial = initial_hat_r * initial_hat_g  # 应该是(3,1)形状
        dv_initial = np.zeros((3, 1))
        self.kf.x = np.vstack((dp_initial, dv_initial))

        # OCSORT特定属性
        self.time_since_update = 0
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.history = [] 
        self.hits = 0
        self.hit_streak = 0
        self.age = 0
        
        # 存储观测值
        self.last_observation = list(bbox)  # 将tuple转换为list并存储
        self.observations = dict()
        self.history_observations = []  # [g[0], g[1], g[2], r]格式的列表

        self.velocity = None  # 原为2D图像平面速度，现在KF状态包含3D相对速度
        self.delta_t = delta_t  # OCSORT特定参数，用于历史观测匹配

    def update(self, bbox):
        """
        Updates the state vector with observed bbox.
        """
        if bbox is not None:
            if self.last_observation is not None:  # 检查是否有之前的观测值
                previous_box = None
                for i in range(self.delta_t):
                    dt = self.delta_t - i
                    if self.age - dt in self.observations:
                        previous_box = self.observations[self.age-dt]
                        break
                if previous_box is None:
                    previous_box = self.last_observation
                """
                  Estimate the track speed direction with observations \Delta t steps away
                """
                self.velocity = speed_direction(convert_x_to_bbox(previous_box), convert_x_to_bbox(bbox))
            
            # 获取方向向量和距离
            hat_g = np.asarray(bbox[0]).reshape((3, 1))
            hat_r = float(bbox[1])
            
            # 更新观测矩阵
            P_hat_g = np.eye(3) - (hat_g @ hat_g.T)
            H1 = np.hstack([P_hat_g, np.zeros([3, 3])])
            H2 = np.hstack([np.eye(3), np.zeros([3, 3])])
            self.kf.H = np.vstack([H1, H2])
            
            # 更新测量噪声协方差R
            sigma_mu_sq_val = 0.01  # 方向噪声的方差
            sigma_w_sq_val = 0.1    # 距离噪声的方差
            
            # 左上块: r²σμ²P_hat_g
            self.kf.R[0:3, 0:3] = (hat_r**2) * sigma_mu_sq_val * P_hat_g+ 1e-6 * np.eye(3)
            # 右下块: σw²hat_g hat_g^T
            self.kf.R[3:6, 3:6] = sigma_w_sq_val * (hat_g @ hat_g.T)+ 1e-6 * np.eye(3)
            # 右上和左下块保持为0矩阵
            
            """
              Insert new observations. This is a ugly way to maintain both self.observations
              and self.history_observations. Bear it for the moment.
            """
            self.last_observation = bbox  # 记得统一bbox
            self.observations[self.age] = bbox
            self.history_observations.append(bbox)

            self.time_since_update = 0
            self.history = []
            self.hits += 1
            self.hit_streak += 1
            self.kf.update(convert_bbox_to_z(bbox))
        else:
            self.kf.update(bbox)

    def predict(self):
        """
        预测下一时刻的状态向量
        返回:
            3维位置向量 [x, y, z]
        """
        # 预测状态
        self.kf.predict()
        
        # 更新跟踪器状态
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        
        # 从状态向量中提取位置信息（前3个元素）
        position = self.kf.x[0:3].reshape(3)
        self.history.append(position)
        
        return position

    def get_state(self):
        """
        Returns the current position estimate.
        """
        return self.kf.x[0:6].reshape(6)


"""
    We support multiple ways for association cost calculation, by default
    we use IoU. GIoU may have better performance in some situations. We note 
    that we hardly normalize the cost by all methods to (0,1) which may not be 
    the best practice.
"""
ASSO_FUNCS = {  "iou": iou_batch}


class OCSort(object):
    def __init__(self, det_thresh, max_age=40, min_hits=1, 
        iou_threshold=0.3, delta_t=3, asso_func="iou", inertia=0.2, use_byte=False):
        """
        Sets key parameters for SORT
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.trackers = []
        self.frame_count = 0
        self.det_thresh = det_thresh
        self.delta_t = delta_t
        self.asso_func = ASSO_FUNCS[asso_func]
        self.inertia = inertia
        self.use_byte = use_byte
        KalmanBoxTracker.count = 0

    def update(self, detections):
        """
        更新跟踪器状态
        参数:
            detections: 检测结果列表，每个元素为(hat_g, hat_r, score)格式
                       hat_g: 3维方向向量
                       hat_r: 标量距离
                       score: 检测置信度
        返回:
            跟踪结果列表，每个元素为[x, y, z, track_id]格式
        """
        if detections is None or len(detections) == 0:
            return np.empty((0, 4))

        self.frame_count += 1
        
        # 处理检测结果，确保数据格式正确
        valid_detections = []
        valid_scores = []
        for det in detections:
            g, r, score = det
            # 检查方向向量是否有效
            if isinstance(g, np.ndarray) and len(g) == 3 and not np.any(np.isnan(g)):
                # 确保方向向量是归一化的
                g = g / np.linalg.norm(g)
                # 检查距离是否有效
                if not np.isnan(r) and r > 0:
                    valid_detections.append((g, r))
                    valid_scores.append(score)
        
        if not valid_detections:
            return np.empty((0, 4))
            
        # 转换为numpy数组，确保形状一致
        scores = np.array(valid_scores)
        # 将valid_detections转换为位置坐标
        dets = []
        for g, r in valid_detections:
            pos = r * g  # 直接计算位置坐标
            dets.append(pos)
        dets = np.array(dets)
        
        # 分离高置信度和低置信度检测结果
        inds_low = scores > 0.1
        inds_high = scores < self.det_thresh
        inds_second = np.logical_and(inds_low, inds_high)  # self.det_thresh > score > 0.1
        dets_second = dets[inds_second]  # 低置信度检测结果
        remain_inds = scores > self.det_thresh
        dets = dets[remain_inds]  # 高置信度检测结果

        # 获取现有跟踪器的预测位置
        trks = np.zeros((len(self.trackers), 3))  # 3维位置
        to_del = []
        ret = []
        for t, trk in enumerate(self.trackers):
            pos = trk.predict()  # 获取预测的3D位置
            # 检查预测位置是否有效
            if not np.any(np.isnan(pos)) and np.all(np.isfinite(pos)):
                trks[t] = pos  # 直接存储3D位置
            else:
                to_del.append(t)
        
        # 删除无效的跟踪器
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        for t in reversed(to_del):
            self.trackers.pop(t)

        # 获取跟踪器的速度信息
        velocities = np.array([trk.velocity if trk.velocity is not None else np.zeros(3) for trk in self.trackers])
        
        # 获取历史观测值
        last_boxes = []
        for trk in self.trackers:
            if trk.last_observation is not None:
                # 确保格式一致：[x, y, z]
                pos = convert_x_to_bbox(trk.last_observation)
                last_boxes.append(pos)
            else:
                last_boxes.append(np.zeros(3))
        last_boxes = np.array(last_boxes)
        
        k_observations = np.array([k_previous_obs(trk.observations, trk.age, self.delta_t) for trk in self.trackers])

        # 第一次关联匹配
        matched, unmatched_dets, unmatched_trks = associate(
            dets, trks, self.iou_threshold, velocities, k_observations, self.inertia)
        
        # 更新已匹配的跟踪器
        for m in matched:
            # 从valid_detections中获取原始检测结果
            det_idx = np.where(remain_inds)[0][m[0]]  # 获取实际的索引
            g, r = valid_detections[det_idx]
            self.trackers[m[1]].update((g, r))

        # 第二次关联匹配（使用低置信度检测结果）
        if self.use_byte and len(dets_second) > 0 and unmatched_trks.shape[0] > 0:
            u_trks = trks[unmatched_trks]
            iou_left = self.asso_func(dets_second, u_trks)
            iou_left = np.array(iou_left)
            if iou_left.max() > self.iou_threshold:
                matched_indices = linear_assignment(-iou_left)
                to_remove_trk_indices = []
                for m in matched_indices:
                    det_ind, trk_ind = m[0], unmatched_trks[m[1]]
                    if iou_left[m[0], m[1]] < self.iou_threshold:
                        continue
                    # 从valid_detections中获取原始检测结果
                    det_idx = np.where(inds_second)[0][m[0]]  # 获取实际的索引
                    g, r = valid_detections[det_idx]
                    self.trackers[trk_ind].update((g,r))
                    to_remove_trk_indices.append(trk_ind)
                unmatched_trks = np.setdiff1d(unmatched_trks, np.array(to_remove_trk_indices))

        # 处理未匹配的跟踪器
        for m in unmatched_trks:
            self.trackers[m].update(None)

        # 为未匹配的检测结果创建新的跟踪器
        for i in unmatched_dets:
            # 使用正确的索引从valid_detections中获取数据
            g, r = valid_detections[i]
            trk = KalmanBoxTracker((g, r), delta_t=self.delta_t)
            self.trackers.append(trk)

        # 收集跟踪结果
        i = len(self.trackers)
        for trk in reversed(self.trackers):
            if trk.last_observation is None:
                d = trk.get_state()  # 获取3D位置
            else:
                d = convert_x_to_bbox(trk.last_observation)  # 将(hat_g, hat_r)转换为位置
            
            if (trk.time_since_update < 1) and (trk.hit_streak >= self.min_hits or self.frame_count <= self.min_hits):
                ret.append(np.concatenate((d, [trk.id+1])).reshape(1, -1))
            i -= 1
            # 移除过期的跟踪器
            if trk.time_since_update > self.max_age:
                self.trackers.pop(i)

        if len(ret) > 0:
            return np.concatenate(ret)
        return np.empty((0, 4))

    
    def get_state(self):
        """
        获取所有跟踪器的状态（3D位置）
        返回:
            list: 包含所有有效跟踪器的列表，每个元素为[x, y, z, track_id]格式
        """
        tracks = []
        for trk in self.trackers:
            # 获取跟踪器状态（3D位置）
            state = trk.get_state() if hasattr(trk, 'get_state') else None
            
            # 处理不同格式的状态数据
            if state is not None:
                # 确保状态是3D位置向量
                if isinstance(state, (np.ndarray, list)) and len(state) >= 3:
                    # 如果状态是二维数组，取第一个元素
                    if isinstance(state[0], (np.ndarray, list)):
                        state = state[0]
                    
                    # 创建跟踪结果 [x, y, z, track_id]
                    track = [
                        float(state[0]),  # x
                        float(state[1]),  # y
                        float(state[2]),  # z
                        trk.id + 1 ,      # track_id
                        float(state[3]),  # vx
                        float(state[4]),  # vy
                        float(state[5])  # vz
                        
                    ]
                    tracks.append(track)
                    
                    # 打印调试信息（可选）
                    tracker_id = getattr(trk, 'id', 'unknown')
                    print(f"Tracker {tracker_id}: Position = [{track[0]:.2f}, {track[1]:.2f}, {track[2]:.2f}], Velocity = [{track[4]:.2f}, {track[5]:.2f}, {track[6]:.2f}]")

        return tracks