import os
import numpy as np


def iou_batch(bboxes1, bboxes2):
    """
    From SORT: Computes IOU between two bboxes in the form [x1,y1,x2,y2]
    """
    bboxes2 = np.expand_dims(bboxes2, 0)
    bboxes1 = np.expand_dims(bboxes1, 1)
    
    xx1 = np.maximum(bboxes1[..., 0], bboxes2[..., 0])
    yy1 = np.maximum(bboxes1[..., 1], bboxes2[..., 1])
    xx2 = np.minimum(bboxes1[..., 2], bboxes2[..., 2])
    yy2 = np.minimum(bboxes1[..., 3], bboxes2[..., 3])
    w = np.maximum(0., xx2 - xx1)
    h = np.maximum(0., yy2 - yy1)
    wh = w * h
    o = wh / ((bboxes1[..., 2] - bboxes1[..., 0]) * (bboxes1[..., 3] - bboxes1[..., 1])                                      
        + (bboxes2[..., 2] - bboxes2[..., 0]) * (bboxes2[..., 3] - bboxes2[..., 1]) - wh)                                              
    return(o)  



def speed_direction_batch(dets, tracks):
    """
    计算3D空间中的速度方向
    参数:
        dets: 检测结果，格式为[x, y, z, ...]
        tracks: 跟踪结果，格式为[x, y, z, ...]
    返回:
        dy, dx: 归一化的速度方向向量
    """
    tracks = tracks[..., np.newaxis]
    
    # 获取3D位置
    X1, Y1, Z1 = dets[:,0], dets[:,1], dets[:,2]
    X2, Y2, Z2 = tracks[:,0], tracks[:,1], tracks[:,2]
    
    # 计算位移向量
    dx = X1 - X2
    dy = Y1 - Y2
    dz = Z1 - Z2
    
    # 计算位移向量的模长
    norm = np.sqrt(dx**2 + dy**2 + dz**2) + 1e-6
    
    # 归一化位移向量
    dx = dx / norm
    dy = dy / norm
    dz = dz / norm
    
    return dy, dx, dz  # size: num_track x num_det


def linear_assignment(cost_matrix):
    try:
        import lap
        _, x, y = lap.lapjv(cost_matrix, extend_cost=True)
        return np.array([[y[i],i] for i in x if i >= 0]) #
    except ImportError:
        from scipy.optimize import linear_sum_assignment
        x, y = linear_sum_assignment(cost_matrix)
        return np.array(list(zip(x, y)))


def associate_detections_to_trackers(detections,trackers,iou_threshold = 0.3):
    """
    Assigns detections to tracked object (both represented as bounding boxes)
    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if(len(trackers)==0):
        return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)

    iou_matrix = iou_batch(detections, trackers)

    if min(iou_matrix.shape) > 0:
        a = (iou_matrix > iou_threshold).astype(np.int32)
        if a.sum(1).max() == 1 and a.sum(0).max() == 1:
            matched_indices = np.stack(np.where(a), axis=1)
        else:
            matched_indices = linear_assignment(-iou_matrix)
    else:
        matched_indices = np.empty(shape=(0,2))

    unmatched_detections = []
    for d, det in enumerate(detections):
        if(d not in matched_indices[:,0]):
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if(t not in matched_indices[:,1]):
            unmatched_trackers.append(t)

    #filter out matched with low IOU
    matches = []
    for m in matched_indices:
        if(iou_matrix[m[0], m[1]]<iou_threshold):
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1,2))
    if(len(matches)==0):
        matches = np.empty((0,2),dtype=int)
    else:
        matches = np.concatenate(matches,axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


def associate(detections, trackers, iou_threshold, velocities, previous_obs, vdc_weight):    
    if(len(trackers)==0):
        return np.empty((0,2),dtype=int), np.arange(len(detections)), np.empty((0,5),dtype=int)
    
    # 计算检测和跟踪器之间的3D位置距离
    position_cost = np.zeros((len(detections), len(trackers)))
    for i, det in enumerate(detections):
        for j, trk in enumerate(trackers):
            # 计算3D位置之间的欧氏距离
            pos_diff = np.array(det[:3]) - np.array(trk[:3])
            dist = np.sqrt(np.sum(pos_diff**2))
            # 使用更合适的距离衰减函数
            # 当距离小于0.5米时，相似度接近1；当距离大于1米时，相似度快速降低
            position_cost[i, j] = np.exp(-dist / 0.9)  # 调整衰减系数为0.3
    
    # # 打印调试信息
    # print("\n=== Debug Info ===")
    # print("Detections shape:", detections.shape)
    # print("Trackers shape:", trackers.shape)
    # print("Sample detection:", detections[0] if len(detections) > 0 else "No detections")
    # print("Sample tracker:", trackers[0] if len(trackers) > 0 else "No trackers")
    # print("==================\n")
    
    # 使用speed_direction_batch计算速度方向代价
    if previous_obs is not None and len(previous_obs) > 0:
        Y, X, Z = speed_direction_batch(detections, previous_obs)
        inertia_Y, inertia_X, inertia_Z = velocities[:,0], velocities[:,1], velocities[:,2]
        inertia_Y = np.repeat(inertia_Y[:, np.newaxis], Y.shape[1], axis=1)
        inertia_X = np.repeat(inertia_X[:, np.newaxis], X.shape[1], axis=1)
        inertia_Z = np.repeat(inertia_Z[:, np.newaxis], Z.shape[1], axis=1)
        diff_angle_cos = inertia_X * X + inertia_Y * Y + inertia_Z * Z
        diff_angle_cos = np.clip(diff_angle_cos, a_min=-1, a_max=1)
        diff_angle = np.arccos(diff_angle_cos)
        diff_angle = (np.pi /2.0 - np.abs(diff_angle)) / np.pi
        
        # 检查previous_obs是否有效（不是全-1的向量）
        valid_mask = np.ones(previous_obs.shape[0])
        for i in range(len(previous_obs)):
            if np.all(previous_obs[i] == -1):
                valid_mask[i] = 0
        valid_mask = np.repeat(valid_mask[:, np.newaxis], X.shape[1], axis=1)
        
        velocity_cost = (valid_mask * diff_angle) * vdc_weight
        velocity_cost = velocity_cost.T
    else:
        velocity_cost = np.zeros((len(detections), len(trackers)))
    
    # 组合所有代价，调整权重
    total_cost = (position_cost * 0.9 +  # 增加位置权重，因为集群场景中位置信息更可靠
                 velocity_cost * 0.1)     # 降低速度权重，因为短距离内速度变化可能较大
    
    # # 打印代价矩阵
    # print("\n=== Cost Matrix ===")
    # print("Position Cost:")
    # print(position_cost)
    # print("\nVelocity Cost:")
    # print(velocity_cost)
    # print("\nTotal Cost:")
    # print(total_cost)
    # print("==================\n")
    
    # 使用匈牙利算法进行匹配
    if total_cost.size > 0:
        matched_indices = linear_assignment(-total_cost)
    else:
        matched_indices = np.empty(shape=(0,2))
    
    # 找出未匹配的检测和跟踪器
    unmatched_detections = []
    for d in range(len(detections)):
        if d not in matched_indices[:,0]:
            unmatched_detections.append(d)
    
    unmatched_trackers = []
    for t in range(len(trackers)):
        if t not in matched_indices[:,1]:
            unmatched_trackers.append(t)
    
    # 过滤低相似度的匹配，使用更合适的阈值
    matches = []
    for m in matched_indices:
        if total_cost[m[0], m[1]] < 0.1:  # 提高相似度阈值，因为集群场景中目标应该更接近
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1,2))
    
    if len(matches) == 0:
        matches = np.empty((0,2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)
    
    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)


