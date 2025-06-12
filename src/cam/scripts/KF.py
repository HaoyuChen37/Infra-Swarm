import numpy as np
# 卡尔曼滤波器
kf_state_target = np.zeros([6, 1])
# 目标估计
def estimation_kf(g, r, pos_cam, dt, kf_P, kf_Q, kf_R, kf_first):
    global kf_state_target
    F = np.array([[1, 0, 0, dt, 0, 0],
                  [0, 1, 0, 0, dt, 0],
                  [0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])
    H1 = np.hstack([np.eye(3)-np.dot(g, g.T), np.zeros([3, 3])])
    H2 = np.hstack([np.eye(3), np.zeros([3, 3])])
    H = np.vstack([H1, H2])
    E1 = np.hstack([r*(np.eye(3)-np.dot(g, g.T)), np.zeros([3, 1])])
    E2 = np.hstack([r*np.eye(3), g])
    E = np.vstack([E1, E2])
    kf_R_now = np.dot(np.dot(E, kf_R), E.T)

    if kf_first:
        kf_first = False
        kf_state_target[0:3, 0:1] = pos_cam + r * g
    else:
        kf_state_target = np.dot(F, kf_state_target)
        kf_P = np.dot(np.dot(F, kf_P), F.T) + kf_Q
        K = np.dot(np.dot(kf_P, H.T), np.linalg.pinv(np.dot(np.dot(H, kf_P), H.T) + kf_R_now))
        m = np.vstack([np.dot(np.eye(3)-np.dot(g, g.T), pos_cam), pos_cam + r * g])
        kf_state_target = kf_state_target + np.dot(K, (m - np.dot(H, kf_state_target)))
        size_K, _ = K.shape
        kf_P = np.dot((np.identity(size_K) - np.dot(K, H)), kf_P)

    target_pos_est = kf_state_target[0:3]
    target_vel_est = kf_state_target[3:6]
    return target_pos_est, target_vel_est, kf_P, kf_first


# 没有检测到目标，仅预测
def estimation_kf_predict(dt):
    global kf_state_target
    F = np.array([[1, 0, 0, dt, 0, 0],
                  [0, 1, 0, 0, dt, 0],
                  [0, 0, 1, 0, 0, dt],
                  [0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 1]])
    kf_state_target = np.dot(F, kf_state_target)
    target_pos_est = kf_state_target[0:3]
    target_vel_est = kf_state_target[3:6]
    return target_pos_est, target_vel_est


    