#!/usr/bin/env python3
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped
import rospy
import tf
import sys
sys.path.append('/home/nvidia/swarm/swarm/devel/lib/python3/dist-packages')
from cam.msg import LightInfo,Detection, Dects,g_r
import math as m

# 外参矩阵
M_cam2hand = [
    np.array(  [[ 0.6571239 ,  0.05203458 , 0.75198443  ,0.05763316],
                [-0.7523269 , -0.0166949  , 0.6585784   ,0.07617752],
                [ 0.04682316 ,-0.99850572 , 0.0281764  ,-0.05346858],
                [ 0     ,  0   ,      0     ,     1       ]]),
    np.array([
        [-0.72011037, -0.03127949,  0.69315413,  0.06132023],
        [-0.69241984, -0.03192476, -0.72078817, -0.03504754],
        [0.04467466, -0.9990007,   0.00133081, -0.01705665],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [-0.70108422, -0.01602186, -0.71289847, -0.0414114],
        [0.71296302,  0.00224053, -0.70119805, -0.06623567],
        [0.01283176, -0.99986913,  0.00985219, -0.02437643],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [0.72243153,  0.02749143, -0.69089572, -0.07127752],
        [0.68931443,  0.04969853,  0.72275561,  0.03802169],
        [0.05420608, -0.99838584,  0.01695355, -0.03157856],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [0.6891949,   0.00746342,  0.72453757,  0.04061265],
        [-0.7239904, -0.03310119,  0.68901539,  0.05888463],
        [0.02912546, -0.99942414, -0.01740973, -0.00541817],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [-0.73037417, -0.02360289,  0.68263934,  0.06172566],
        [-0.68184366, -0.03410973, -0.73070223, -0.03933543],
        [0.04053133, -0.99913934,  0.00881939, -0.01045818],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [-0.68541455, -0.02824068, -0.72760522, -0.04325892],
        [0.72786528,  0.00151883, -0.68571848, -0.06084607],
        [0.02047026, -0.9996,      0.01951439, -0.00739562],
        [0.,          0.,          0.,          1.        ]
    ]),
    np.array([
        [0.72994282,  0.00838439, -0.68345679, -0.0618053],
        [0.68330577,  0.015384,    0.72997025,  0.03678699],
        [0.01663465, -0.99984651,  0.00550034, -0.01089537],
        [0.,          0.,          0.,          1.        ]
    ])
]


# 相机到小车转换矩阵
a = 0.04054 * m.sin(m.pi/4)
'''
发现方向反了，所有角度加pi
'''
T_camera_to_car = [
    np.array([[m.cos(3*m.pi/4),  m.sin(3*m.pi/4), 0,      a], # yaw = 3pi/4
             [-m.sin(3*m.pi/4),  m.cos(3*m.pi/4), 0,     -a],
             [0            ,   0            , 1,     0],
             [0            ,   0            , 0,     1]]),

    np.array([[m.cos(5*m.pi/4),  m.sin(5*m.pi/4), 0,    a], # yaw = 5pi/4
             [-m.sin(5*m.pi/4),  m.cos(5*m.pi/4), 0,    a],
             [0             ,   0             , 1,    0],
             [0             ,   0             , 0,    1]]),

    np.array([[m.cos(7*m.pi/4), m.sin(7*m.pi/4), 0,   -a], # yaw = 7pi/4
             [-m.sin(7*m.pi/4), m.cos(7*m.pi/4), 0,    a],
             [0               ,   0               , 1,    0],
             [0               ,   0               , 0,    1]]),

    np.array([[m.cos(9*m.pi/4), -m.sin(9*m.pi/4), 0,    -a], # yaw = 9pi/4
             [-m.sin(9*m.pi/4),  m.cos(9*m.pi/4), 0,    -a],
             [0              ,   0              , 1,     0],
             [0              ,   0              , 0,     1]])
]


def get_homogenious(quaternion, position): 
        # 将四元数转换为欧拉角，返回matrix
        R = tf.transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        t = np.array([position.x, position.y, position.z])
        T = np.zeros([4, 4])
        T[0:3, 0:3] = R[0:3, 0:3]
        T[0,3] = t[0]
        T[1,3] = t[1]
        T[2,3] = t[2]
        T[3, 3] = 1
        return T

def record_distance(pose1, pose2):
    dis = ((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2 + (pose1[2]-pose2[2])**2) **0.5
    return dis

class LightLocalizer():
    def __init__(self):
        # 初始化5个小车的齐次坐标变换矩阵
        self.T_car1_to_vicon = np.eye(4)  # car1 到 Vicon 的变换矩阵
        self.T_car2_to_vicon = np.eye(4)  # car2 到 Vicon 的变换矩阵
        self.T_car3_to_vicon = np.eye(4)  # car3 到 Vicon 的变换矩阵
        self.T_car4_to_vicon = np.eye(4)  # car4 到 Vicon 的变换矩阵
        self.T_car5_to_vicon = np.eye(4)  # car5 到 Vicon 的变换矩阵
        # 相机参数配置
        self.camera_matrix = np.array([[833.33,0,640],
                                        [0,833.33,512],
                                        [0,0,1]])
        self.dist_coeffs = np.array( [[-0.097259155290161,0.099318601051863,-0.012917853790591,2.737719210785963e-04,-5.326157181954759e-04]])
        # 多相机外参矩阵字典：{ (小车ID, 相机ID): 外参矩阵 }
        self.cam_extrinsics = {
            (0, 0): M_cam2hand[0],
            (0, 1): M_cam2hand[1],
            (0, 2): M_cam2hand[2],
            (0, 3): M_cam2hand[3],
            # 添加其他小车和相机组合...
        }
        # 添加ROI半径参数
        self.roi_radius = 30  # 像素单位
        # 添加小车坐标接收端
        self.car1_sub = rospy.Subscriber("/vicon/IRSWARM1/IRSWARM1", TransformStamped, self.car1_callback)
        self.car2_sub = rospy.Subscriber("/vicon/IRSWARM2/IRSWARM2", TransformStamped, self.car2_callback)
        self.car3_sub = rospy.Subscriber("/vicon/IRSWARM3/IRSWARM3", TransformStamped, self.car3_callback)
        self.car4_sub = rospy.Subscriber("/vicon/IRSWARM4/IRSWARM4", TransformStamped, self.car4_callback)
        self.car5_sub = rospy.Subscriber("/vicon/IRSWARM5/IRSWARM5", TransformStamped, self.car5_callback)

        # self.k = 30887
        self.k = 15000

        self.dis1_2 = 0

        # 初始化字典，每个键对应一个空列表
        self.true_data = {
            'time': [],
            'car1_pose': [],
            'car2_pose': [],
            'car3_pose': [],
            'car4_pose': [],
            'car5_pose': [],
            'true_dis':[],
            'calculated_dis': [],
            'calculated_loc': [],
            'pixel_sum': [],
            'pixel_loc': [],
            'cam_exp': [],
            'env_value': []
        }

    def car1_callback(self, msg):
        # vicon消息
        self.T_car1_to_vicon = get_homogenious(msg.transform.rotation, msg.transform.translation)
        self.dis1_2 = record_distance(self.T_car1_to_vicon[0:3, 3], self.T_car5_to_vicon[0:3, 3])
        # print('recieved car1 pose')
        

    def car2_callback(self, msg):
        # vicon消息
        self.T_car2_to_vicon = get_homogenious(msg.transform.rotation, msg.transform.translation)
    
    def car3_callback(self, msg):
        # vicon消息
        self.T_car3_to_vicon = get_homogenious(msg.transform.rotation, msg.transform.translation)
        
    def car4_callback(self, msg):
        # vicon消息
        self.T_car4_to_vicon = get_homogenious(msg.transform.rotation, msg.transform.translation)
    
    def car5_callback(self, msg):
        # vicon消息
        self.T_car5_to_vicon = get_homogenious(msg.transform.rotation, msg.transform.translation)

    def create_full_image_mask(self, img_shape):
        """
        创建一个覆盖整个图像的蒙版
        :param img_shape: 图像的形状 (height, width)
        :return: 覆盖整个图像的蒙版
        """
        # 创建一个与图像大小相同的全白蒙版
        mask = np.ones(img_shape[:2], dtype=np.uint8) * 255
        return mask
    
    def create_roi_mask(self, projected_pt, img_shape):
        """根据投影点创建圆形ROI掩膜"""
        mask = np.zeros(img_shape[:2], dtype=np.uint8)
        if projected_pt is not None:
            # 如果 projected_pt 是一个字典，提取第一个投影点
            if isinstance(projected_pt, dict):
                # 假设我们只关心第一个投影点
                for target_car_id, pt in projected_pt.items():
                    if pt is not None:
                        u, v = pt
                        # 检查坐标是否在图像范围内
                        if 0 <= u < img_shape[1] and 0 <= v < img_shape[0]:
                            cv2.circle(mask, (u, v), self.roi_radius, 255, -1)
            else:
                # 如果 projected_pt 是一个元组，直接使用
                u, v = projected_pt
                # 检查坐标是否在图像范围内
                if 0 <= u < img_shape[1] and 0 <= v < img_shape[0]:
                    cv2.circle(mask, (u, v), self.roi_radius, 255, -1)
        return mask
    
    def project_all_lights_to_image(self, self_car_id, self_cam_id):
        projections = {}
        try:
            # 获取当前小车的VICON位姿 (T_self_car_to_vicon)
            T_self_car_to_vicon = {
                0: self.T_car1_to_vicon,
                1: self.T_car2_to_vicon,
                2: self.T_car3_to_vicon,
                3: self.T_car4_to_vicon,
                4: self.T_car5_to_vicon
            }[self_car_id]
            # print(self.T_car1_to_vicon)
            # 获取当前相机的位姿参数
            T_cam_to_self_car = self.cam_extrinsics[(self_car_id, self_cam_id)]
            T_self_car_to_cam = np.linalg.inv(T_cam_to_self_car)
            T_vicon_to_self_car = np.linalg.inv(T_self_car_to_vicon)
            T_vicon_to_cam = T_self_car_to_cam.dot(T_vicon_to_self_car)
            

            # 遍历所有其他小车
            for target_car_id in range(1, 6):
                if target_car_id == self_car_id:
                    continue
                    
                # 获取目标小车的VICON位姿 (T_target_car_to_vicon)
                T_target_car_to_vicon = {
                    0: self.T_car1_to_vicon,
                    1: self.T_car2_to_vicon,
                    2: self.T_car3_to_vicon,
                    3: self.T_car4_to_vicon,
                    4: self.T_car5_to_vicon
                }[target_car_id]
                
                pose = T_target_car_to_vicon[:3, 3]
                if not np.array_equal(pose, [0, 0, 0]):
                    # 投影到图像
                    T_target_car_to_cam = T_vicon_to_cam.dot(T_target_car_to_vicon)
                    marker_pos = T_target_car_to_cam[0:3, 3].reshape([3,1]) 
                    marker_proj = self.camera_matrix.dot(marker_pos)
                    marker_proj = marker_proj / marker_proj[2]
                    u, v = int(marker_proj[0]), int(marker_proj[1])
                    # print(u,v)
                    # 验证坐标有效性
                    if 0 <= u < 640 and 0 <= v < 480:  # 假设图像尺寸640x480
                        projections[target_car_id] = (u, v)
                    else:
                        projections[target_car_id] = None
                else:
                    projections[target_car_id] = None
            # print(projections)
        except KeyError:
            pass
        
        return projections

    def process_roi(self, frame, roi_mask):
        """在ROI区域内处理图像并返回光源信息"""
        # 应用ROI掩膜
        masked_frame = cv2.bitwise_and(frame, frame, mask=roi_mask)
        
        # 计算图像的平均像素值（仅在ROI区域内）
        frame_mean = cv2.mean(frame)[0]
        
        # 动态调整阈值
        threshold = 2 * frame_mean
        _, binary = cv2.threshold(masked_frame, threshold, 255, cv2.THRESH_BINARY)
        
        # 创建相反蒙版
        inverted_binary = cv2.bitwise_not(binary)
        
        # 计算相反蒙版区域的平均像素值
        # 使用掩膜计算平均值
        mean_outside_roi = cv2.mean(frame, mask=inverted_binary)[0]
        
        # 查找轮廓
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centers = []
        pixel_sums = []
        detections = []
        
        for contour in contours:
            # 轮廓面积过滤
            area = cv2.contourArea(contour)
            if 4 < area < 10000:
                # 计算质心
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # 计算每个亮点的像素值总和
                    mask = np.zeros_like(frame)
                    cv2.drawContours(mask, [contour], -1, 255, -1)
                    masked_image = cv2.bitwise_and(frame, frame, mask=mask)
                    pixel_sum = cv2.sumElems(masked_image)[0]

                    centers.append((cx, cy))
                    pixel_sums.append(pixel_sum)

                    # 计算边界框
                    x, y, w, h = cv2.boundingRect(contour)
                    padding = 20
                    
                    detections.append([x - padding, y - padding, x + w + padding, y + h + padding, pixel_sum, 1]) # Store as tuple: (centroid_tuple, intensity, bbox_list, id)
        
        return centers, pixel_sums, mean_outside_roi, detections
    
    def process_frame_with_vicon(self, frame, car_id, cam_id):
        """完整的定位流程"""
        if frame.max() > 250:
            print("frame max value is too high")
            return None, None, None, None
        # 获取投影点
        projected_pt = self.project_all_lights_to_image(car_id, cam_id)

        # 创建ROI掩膜
        roi_mask = self.create_roi_mask(projected_pt, frame.shape)

        # cv2.imshow('frame', frame)
        # cv2.imshow('mask', roi_mask)

        # 处理ROI区域
        centers, pixel_sums, env_value, detections = self.process_roi(frame, roi_mask)
        return centers, pixel_sums, env_value, detections
    
    def process_frame_without_vicon(self, frame):
        """完整的定位流程"""
        roi_mask = self.create_full_image_mask(frame.shape)

        # 处理ROI区域
        centers, pixel_sums, env_value, detections = self.process_roi(frame, roi_mask)
        return centers, pixel_sums, env_value, detections
    

    def pixel_sum_to_distance(self, pixel_sum, exposure, env_calue):
        """基于光强衰减模型的距离估计"""
        # 计算接收光强 (考虑相机响应模型)
        # 根据平方反比定律计算距离
        if pixel_sum - env_calue < 0:
             print("negative!!!")
        distance = np.sqrt(self.k / ((pixel_sum- env_calue) / (exposure/732) ) )  
        return distance
    
    def pixel_to_car_coordinates(self, u, v, distance, cam_id):
        """
        将2D像素坐标转换为3D相机坐标系坐标
        参数：
            u, v : 像素坐标
            depth_value : 对应像素点的深度值（单位：米）
            camera_matrix : 相机内参矩阵 [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            dist_coeffs : 畸变系数 [k1, k2, p1, p2, k3]
        返回：
            (x, y, z) : 相机坐标系下的3D坐标
        """
        # 从像素坐标系转换到图像物理坐标系
        p_pixel = np.array([u, v, 1])
        P_inv = np.linalg.inv(self.camera_matrix)
        p_image = P_inv.dot(p_pixel)
        p_image = p_image / np.linalg.norm(p_image) * distance

        # 从图像物理坐标系转换到相机坐标系
        R_i2c = np.array([[0, 0, 1],
                          [1, 0, 0],
                          [0, 1, 0]])
        p_cam = R_i2c.dot(p_image)

        # 从相机坐标系转换到小车坐标系
        p_cam = np.append(p_cam, 1)
        p_car = T_camera_to_car[cam_id].dot(p_cam)
        g_car = np.array([p_car[0], p_car[1]])
        g_car = g_car / np.linalg.norm(g_car)
        # print(type(g_car), g_car)

        return g_car
    
    def reproject(self, pixel_loc, pixel_sum, exposure, env_calue, cam_id, savedata):
        lights = []
        loc =[]
        dis = []
        for i, (u, v) in enumerate(pixel_loc):
            distance = self.pixel_sum_to_distance(pixel_sum[i], exposure, env_calue)
            p_car = self.pixel_to_car_coordinates(u, v, distance, cam_id)
            lights.append(g_r(x=p_car[0], y=p_car[1], distance=distance))
            print(f'cam_id={cam_id}.x={p_car[0]}, y={p_car[1]}, distance={distance}')
            print(self.dis1_2)
            loc.append([p_car[0], p_car[1]])
            dis.append(distance)

        if savedata:
            self.savedata(pixel_loc, pixel_sum, exposure, loc, dis, env_calue)
        return lights
    
    def re_det(self,detections):
        dets = Dects()
        dets.detections = []
        for det_item in enumerate(detections):
            det=Detection()
            det.x1 = det_item[1][0]
            det.y1 = det_item[1][1]
            det.x2 = det_item[1][2]
            det.y2 = det_item[1][3]
            det.intensity = det_item[1][4]
            det.id = det_item[1][5]
            dets.detections.append(det)
        return dets


    def savedata(self, pixel_loc, pixel_sum, exposure, location, calculated_dis, env_calue):
        # 保存数据到字典中
        t = int(str(rospy.Time.now().to_nsec()))
        self.true_data['time'].append(t)
        self.true_data['car1_pose'].append(self.T_car1_to_vicon)
        self.true_data['car2_pose'].append(self.T_car2_to_vicon)
        self.true_data['car3_pose'].append(self.T_car2_to_vicon)
        self.true_data['car4_pose'].append(self.T_car2_to_vicon)
        self.true_data['car5_pose'].append(self.T_car2_to_vicon)
        self.true_data['true_dis'] .append(self.dis1_2)
        self.true_data['calculated_dis'].append(calculated_dis)
        self.true_data['calculated_loc'].append(location)
        self.true_data['pixel_sum'].append(pixel_sum)
        self.true_data['pixel_loc'].append(pixel_loc)
        self.true_data['cam_exp'].append(exposure)
        self.true_data['env_value'].append(env_calue)
        

def main():
	lightlocalizer = LightLocalizer()
	try:
		lightlocalizer.pixel_to_car_coordinates(320, 240, m.sqrt(2), 0)
	finally:
		cv2.destroyAllWindows()

# main()