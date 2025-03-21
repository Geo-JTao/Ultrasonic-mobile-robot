#! /usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import cv_bridge
import rospy
from rm_moveit_scipts.srv import grasp_pose, grasp_poseRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Callback function for clicking on OpenCV window
click_point_pix = ()
depth_value = 0

def mouseclick_callback(event, x, y, flags, param):
    global click_point_pix, depth_value
    if event == cv2.EVENT_LBUTTONDOWN:
        click_point_pix = (x, y)
        print(f"Click position: {click_point_pix}")
        depth_value = depth_img_cv[y, x]  # 深度图的像素值
        if depth_value == 0 :
            print("error depth value")
            depth_value = 520
        print(f"Depth value at clicked point: {depth_value}")
        cam_intrinsics = np.array([895.2587280273438, 0.0, 667.689208984375, 0.0, 895.4281616210938, 364.93109130859375, 0.0, 0.0, 1.0]).reshape(3, 3)
        u = x
        v = y
        pos_z = depth_value *0.001  # 深度图通常以毫米为单位，这里转换为米
        # 计算3D坐标
        ppx = cam_intrinsics[0][2]
        ppy = cam_intrinsics[1][2]
        pos_x = (u - ppx) * (pos_z / cam_intrinsics[0][0])
        pos_y = (v - ppy) * (pos_z / cam_intrinsics[1][1])
        camera_point = np.array([pos_x, pos_y, pos_z])
        # camera_point = np.array([pos_z, -pos_x,-pos_y])
        print(f"相机坐标系下: {camera_point}")
        
        # 提供控制指令
        # rpy_action_id = int(input("Please input action_id (0=push, 1=grasp): "))
        # rpy_angle = float(input("Please input angle: "))
        rpy_action_id = 0
        rpy_angle = 90
        rpy = [rpy_angle, rpy_action_id, 0]
        # 请求抓取位姿
        req = grasp_poseRequest()
        req.grasppose_x, req.grasppose_y, req.grasppose_z = camera_point[0], camera_point[1], camera_point[2]
        req.grasppose_R, req.grasppose_P, req.grasppose_Y = rpy[0], rpy[1], rpy[2]
        result = client.call(req)
        print("result:",result)

while True:
    rospy.init_node("grasp_client")
    client = rospy.ServiceProxy("moveit_grasp", grasp_pose)
    rospy.wait_for_service("moveit_grasp")
    rgb_img_msg = rospy.wait_for_message('/camera/color/image_raw', Image) 
    depth_img_msg = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image) 

    # 将ROS图像消息转换为OpenCV格式
    bridge = CvBridge()
    rgb_img_cv = bridge.imgmsg_to_cv2(rgb_img_msg, desired_encoding="passthrough")
    rgb_img_cv = cv2.cvtColor(rgb_img_cv, cv2.COLOR_RGB2BGR)
    depth_img_cv = bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding="passthrough")
    
    if len(click_point_pix) != 0:
        rgb_img_cv = cv2.circle(rgb_img_cv, click_point_pix, 7, (0, 0, 255), 2)

    cv2.imshow('rgb_img', rgb_img_cv)
    # cv2.imshow('depth_img',depth_img_cv)
    cv2.namedWindow('rgb_img')
    # cv2.namedWindow('depth_img')
    cv2.setMouseCallback('rgb_img', mouseclick_callback)
    if cv2.waitKey(1) == ord('c'):
        break
cv2.destroyAllWindows()
