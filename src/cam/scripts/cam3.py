#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from datetime import datetime
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import mvsdk
import platform
from scipy.io import savemat
from datetime import datetime
from std_msgs.msg import Header
from cam.msg import LightInfo, Cam3, Detection, Dects
from light_processing import LightLocalizer
import pdb
import time
import sys
sys.path.append('/home/nvidia/swarm/devel/lib/python3/dist-packages')

Cam_ID = 7

class Camera(object):
    def __init__(self, Cam_ID):
        rospy.init_node('Cam3_node', anonymous=True)
        self.bridge = CvBridge()
        self.light_pub = rospy.Publisher('/Cam3', Cam3, queue_size=100)
        self.det_pub = rospy.Publisher('/Det3', Dects, queue_size=10)
        # initialize camera parameters
        self.DevList = []
        self.hCamera = 0
        self.FrameBufferSize = 0
        self.pFrameBuffer = None
        self.shutter = [10000, 5000, 2500, 1250, 625, 312, 156, 78, 39, 19, 9, 5, 2, 1]
        self.exposure = 732
        self.pixel_sum = []
        self.pixel_loc = []
        self.detections = []
        self.env_calue = 0
        self.frame_ave_value = 0
        self.frame_max_value = 0
        # Initialize car_id, cam_id
        self.friendly_name = str(Cam_ID)
        self.car_id = Cam_ID // 4
        self.cam_id = Cam_ID % 4 - 1
        
    def initialization(self):
        # 枚举相机
        self.DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(self.DevList)
        print('nDev = ', nDev)
        if nDev < 1:
            print("No camera was found!")
            return

        # 打印所有相机设备信息
        for i, DevInfo in enumerate(self.DevList):
            print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))

        # 根据 FriendlyName 查找目标相机
        target_camera_info = None
        for DevInfo in self.DevList:
            if DevInfo.GetFriendlyName() == self.friendly_name:
                target_camera_info = DevInfo
                break

        # 打开相机
        self.hCamera = 0
        try:
            self.hCamera = mvsdk.CameraInit(target_camera_info, -1, -1)
        except mvsdk.CameraException as e:
            print("CameraInit Failed({}): {}".format(e.error_code, e.message))
            return

        # 获取相机特性描述
        cap = mvsdk.CameraGetCapability(self.hCamera)

        # 判断是黑白相机还是彩色相机
        monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

        # 黑白相机让ISP直接输出MONO数据
        mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)

        # 相机模式切换成连续采集
        mvsdk.CameraSetTriggerMode(self.hCamera, 0)

        # 手动曝光，曝光时间自适应
        mvsdk.CameraSetAeState(self.hCamera, 0)

        # 让SDK内部取图线程开始工作
        mvsdk.CameraPlay(self.hCamera)

        # 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
        self.FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (
            1 if monoCamera else 3)

        # 分配RGB buffer，用来存放ISP输出的图像
        # 备注：从相机传输到PC端的是RAW数据，在PC端通过软件ISP转为RGB数据（如果是黑白相机就不需要转换格式，但是ISP还有其它处理，所以也需要分配这个buffer）
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(self.FrameBufferSize, 16)

        # initialize exposure time
        # self.exposure_adjustment()
        mvsdk.CameraSetExposureTime(self.hCamera, self.exposure)
        # mvsdk.CameraSetExposureTime(self.hCamera, self.exposure)

    def get_frame(self):
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
        mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

        # 此时图片已经存储在pFrameBuffer中，对于彩色相机pFrameBuffer=RGB数据，黑白相机pFrameBuffer=8位灰度数据
        # 把pFrameBuffer转换成opencv的图像格式以进行后续算法处理
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape(
            (FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

        frame = cv2.resize(frame, (1280, 1024), interpolation=cv2.INTER_LINEAR)
        # cv2.imshow("Press q to end", frame)

        return frame

    def exposure_adjustment(self, low=1, high=100000):
        target_pixel_value = 250

        exposure_time = (low + high) // 2
        mvsdk.CameraSetExposureTime(self.hCamera, exposure_time)
        frame = self.get_frame()
        max_pixel = np.max(frame)
        best_diff = abs(target_pixel_value - max_pixel)
        best_exposure_time = exposure_time
        best_pixel = max_pixel

        while low <= high:
            exposure_time = (low + high) // 2
            mvsdk.CameraSetExposureTime(self.hCamera, exposure_time)
            frame = self.get_frame()
            frame = self.get_frame()
            if frame is None:
                continue
            max_pixel = np.max(frame)
            diff = abs(max_pixel - target_pixel_value)

            # 更新最佳差异和最佳曝光时间
            if diff < best_diff:
                best_diff = diff
                best_exposure_time = exposure_time
                best_pixel = max_pixel

            # 如果中间元素的值小于目标值，则移动到后半部分
            if max_pixel < target_pixel_value:
                low = exposure_time + 1
            # 如果中间元素的值大于目标值，则移动到前半部分
            elif max_pixel > target_pixel_value:
                high = exposure_time - 1
            else:
                break

            print('exposure_time = ', exposure_time, 'max_pixel = ', max_pixel, 'diff = ', diff)
            print('best_exposure_time = ', best_exposure_time, 'best_pixel = ', best_pixel, 'best_diff = ', best_diff)

        print(
            f"Best exposure time: {best_exposure_time} us with max pixel value: {best_pixel} and min diff: {best_diff}")

        # 微调曝光时间
        self.exposure = best_exposure_time

    def mask(self, frame, localizer, with_vicon = 0, savedata = False):
        if with_vicon == 0:
            # 调用 process_frame_without_vicon 方法
            self.pixel_loc, self.pixel_sum, self.env_calue, self.detections = localizer.process_frame_without_vicon(frame)
        else:
            self.pixel_loc, self.pixel_sum, self.env_calue, self.detections = localizer.process_frame_with_vicon(frame, self.car_id, self.cam_id)

        # reproject method
        lights = localizer.reproject(self.pixel_loc, self.pixel_sum, self.exposure, self.env_calue, self.cam_id, savedata)

        # 发布消息
        lights_info = Cam3(lights=lights)
        self.light_pub.publish(lights_info)

        det_info = localizer.re_det(self.detections)
        self.det_pub.publish(det_info)

    def release(self):
        # 关闭相机
        mvsdk.CameraUnInit(self.hCamera)

        # 释放帧缓存
        mvsdk.CameraAlignFree(self.pFrameBuffer)

if __name__ == '__main__':
    time.sleep(3)
    cam = Camera(Cam_ID)
    # folder_name = input('input the folder name:')
    folder_name = datetime.now().strftime('%m%d%H%M%S%f')
    image_dir = f"Data/{folder_name}/Cam3/"
    pose_dir = f"Data/{folder_name}/Cam3/data.mat"
    if not os.path.exists(image_dir):
        # 在Linux中创建目录
        os.makedirs(image_dir)

    try:
        r = rospy.Rate(30)  # 100hz
        cam.initialization()
        # 创建 LightLocalizer 类的实例
        localizer = LightLocalizer()

        while not rospy.is_shutdown():
            try:
                current_time = datetime.now().strftime('%m%d%H%M%S%f')
                frame = cam.get_frame()
                # cv2.imshow('frame', frame)
                # cv2.imwrite(image_dir + current_time + ".png", frame)
                if frame is None:
                    continue
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

            cam.mask(frame, localizer, savedata = True)

            # if np.max(frame) >= 250 or np.max(frame) <= 20:
            #     cam.exposure_adjustment()

            if cv2.waitKey(1) & 0xFF in [ord('q'), 27]:
                break

            r.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        cam.release()
        print('camera3 has closed')
        for key, value in localizer.true_data.items():
            localizer.true_data[key] = np.array(value)
        savemat(pose_dir, localizer.true_data)
        print('save file3 successfully')