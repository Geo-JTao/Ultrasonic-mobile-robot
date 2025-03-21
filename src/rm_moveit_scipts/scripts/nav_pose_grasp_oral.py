#! /usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import rospy
import sys
from rm_moveit_scipts.srv import grasp_pose,grasp_poseRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from saveimg import ImageSaver

import numpy as np
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from tools import draw_landmarks_on_image,add_keypoint_neck,add_keypoint_belly

from robot_navigation import Robot_navigation

RED = '\033[91m'
PINK = '\033[95m'
BLUE = '\033[94m'
BLUE_BOLD = '\033[94;1m'
YELLOW  = '\033[33m'
YELLOW_BOLD   = '\033[33;1m'
GREEN = '\033[92m'
GREEN_BOLD = '\033[92;1m'
ENDC = '\033[0m'

if __name__ == "__main__":

    open_nav_flag = True
    # open_nav_flag = False
    move_keypoint_idx = 2

    if open_nav_flag:
        # Set the navigation goal for robot_agv
        print(YELLOW_BOLD + "---------- AGV底盘机器人导航程序 ----------" + ENDC)
        ip = "0.0.0.0"
        port = 22002
        timeout = 60
        robot_nav = Robot_navigation(ip,port,timeout)
        robot_nav.stop_navigation()
        robot_nav.get_robot_base_info()
        print("机器人当前状态: ", robot_nav.status) 
        start_index = int(input("请输入机器人初始位置(导航点索引):"))
        robot_nav.start_navigation(start_index=start_index)
        print("机器人是否开启导航:{}".format(robot_nav.start_navigation_flag))
        robot_nav.get_current_map_path()
        print("当前导航使用的地图名称: {map},当前导航使用的路径名称:{path}".format(
            map=robot_nav.map_path_name["map"],
            path=robot_nav.map_path_name["path"]
            ))
        nav_total_num = int(input("请输入机器人导航次数:"))
        for i in range(nav_total_num):
            print(YELLOW + "********** 第 {} 次导航开始 **********".format(i) + ENDC)
            idx = int(input("请输入导航目标点索引:"))
            robot_nav.move_to_index(idx = idx)
            robot_nav.wait_for_task_compete(task_info = robot_nav.task_info)
            print(YELLOW + "********** 第 {} 次导航结束 **********".format(i) + ENDC)
        robot_nav.stop_navigation()
        print("机器人是否停止导航:{}".format(robot_nav.stop_navigation_flag))
        print(YELLOW_BOLD + "---------- AGV底盘机器人导航程序 ----------" + ENDC)

    # Human pose detect 
    print(GREEN_BOLD + "---------- 人体姿态关键点检测程序 ----------" + ENDC)
    rospy.init_node("grasp_client")
    client = rospy.ServiceProxy("moveit_grasp", grasp_pose)
    print("正在等待 Moveit 服务端启动...")
    rospy.wait_for_service("moveit_grasp")
    camera = ImageSaver()
    camera.save_images(color_filename="save_picture/color{}.png",
                            depth_filename="save_picture/depth{}.png".format(camera.counter))
    print("正在保存第 {} 张图像".format(camera.counter))
    # Create an PoseLandmarker object.
    base_options = python.BaseOptions(model_asset_path='/home/user/robot/pose-detect/pose_landmarker_full.task')
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        output_segmentation_masks=True)
    detector = vision.PoseLandmarker.create_from_options(options)
    # while(True):
    with open('src/picture_counter.txt','r') as f:
        num = int(f.read())
    print("正在加载第 {} 张图像...".format(num+1))
    rgb_path = "save_picture/color{}.png".format(num)
    # rgb_path = "save_picture/test{}.png".format(num)
    # depth_path = "saved_picture/depth{}.png".format(num)
    bridge = CvBridge()
    depth_img_msg = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image) 
    depth_img_cv = bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding="passthrough")
    # rgb_img = cv2.imread(rgb_path)
    # depth_img = np.array(Image.open(depth_path)).astype(np.float32)
    image = mp.Image.create_from_file(rgb_path)
    # Detect pose landmarks from the input image.
    detection_result = detector.detect(image)
    pose_landmarks_list = detection_result.pose_landmarks
    # Process the detection result.
    annotated_image,idx_to_coordinates = draw_landmarks_on_image(image.numpy_view(), detection_result)
    # for idx, coordinates in idx_to_coordinates.items():
    #     print(GREEN + "检测到第 {} 个关键点的像素坐标为：{}".format(idx, coordinates))
    annotated_image,keypoint_dict = add_keypoint_neck(idx_to_coordinates,annotated_image)
    # annotated_image,keypoint_dict = add_keypoint_belly(idx_to_coordinates,annotated_image)
    for idx, coordinates in keypoint_dict.items():
        print(GREEN + "处理后第 {} 个关键点的像素坐标为：{}".format(idx, coordinates))
    # cv2.imshow("annotated_image",annotated_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    print(BLUE_BOLD + "---------- 机械臂客户端执行程序 ----------" + ENDC)
    # move_keypoint_idx = int(input("选择一个关键点作为机械臂操作对象( -1 退出系统):"))
    if move_keypoint_idx == -1:
        print(RED + "正在退出！"+ ENDC)
        sys.exit()
    else:
        print(YELLOW + "选择的关键点索引为 {} , 对应像素点为：{}".format(move_keypoint_idx,keypoint_dict[move_keypoint_idx]) + ENDC)
        pix_x,pix_y = keypoint_dict[move_keypoint_idx]
        depth_value = depth_img_cv[pix_y, pix_x] 
        depth_default_value = 900
        if depth_value == 0 :
            print("*"*50)
            print(RED + "深度值获取异常，正在使用默认值 {}".format(depth_default_value) + ENDC)
            print("*"*50)
            depth_value = depth_default_value
        # depth_value = 855
        print(f"该像素点处的深度值为: {depth_value}")
        cam_intrinsics = np.array([895.2587280273438, 0.0, 667.689208984375, 0.0, 895.4281616210938, 364.93109130859375, 0.0, 0.0, 1.0]).reshape(3, 3)
        # 深度图通常以毫米为单位，这里转换为米
        pos_z = depth_value * 0.001  
        ppx = cam_intrinsics[0][2]
        ppy = cam_intrinsics[1][2]
        pos_x = (pix_x - ppx) * (pos_z / cam_intrinsics[0][0])
        pos_y = (pix_y - ppy) * (pos_z / cam_intrinsics[1][1])
        camera_point = np.array([pos_x, pos_y, pos_z])
        print(BLUE + f"相机坐标系下三维坐标为: {camera_point}" + ENDC)
        ready_to_move_flag = int(input("ready_to_move_flag: (0/1):"))
        if ready_to_move_flag == 0:
            sys.exit()
        # 提供控制指令
        # rpy_action_id = int(input("Please input action_id (0=push, 1=grasp): "))
        # rpy_angle = float(input("Please input angle: "))
        print("机械臂服务端正在处理请求...")
        rpy_action_id = 0
        rpy_angle = -45
        rpy = [rpy_angle, rpy_action_id, 0]
        req = grasp_poseRequest()
        req.grasppose_x, req.grasppose_y, req.grasppose_z = camera_point[0], camera_point[1], camera_point[2]
        req.grasppose_R, req.grasppose_P, req.grasppose_Y = rpy[0], rpy[1], rpy[2]
        result = client.call(req)
            