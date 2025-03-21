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
from tools import draw_landmarks_on_image,add_keypoint_neck,add_keypoint_neck_1

from robot_navigation import Robot_navigation
from flask import Flask, request, jsonify
import multiprocessing

RED = '\033[91m'
PINK = '\033[95m'
BLUE = '\033[94m'
BLUE_BOLD = '\033[94;1m'
YELLOW  = '\033[33m'
YELLOW_BOLD   = '\033[33;1m'
GREEN = '\033[92m'
GREEN_BOLD = '\033[92;1m'
ENDC = '\033[0m'


def navigation(
                robot_nav : Robot_navigation,
                nav_total_num,
                nav_sequence_forward,
            ):
    # Set the navigation goal for robot_agv
    robot_nav.get_robot_base_info()
    print("机器人当前状态: ", robot_nav.status) 
    robot_nav.get_current_map_path()
    print("当前导航使用的地图名称: {map},当前导航使用的路径名称:{path}".format(
        map=robot_nav.map_path_name["map"],
        path=robot_nav.map_path_name["path"]
        ))
    for i in range(nav_total_num):

        print(YELLOW + "********** 第 {} 次导航开始 **********".format( i + 1 ) + ENDC)
        if nav_sequence_forward == True:
            robot_nav.move_to_index( idx = i + 1 )
        else:
            robot_nav.move_to_index( idx = nav_total_num - i -1  )
        robot_nav.wait_for_task_compete(task_info = robot_nav.task_info)
        print(YELLOW + "********** 第 {} 次导航结束 **********".format(i + 1) + ENDC)
    

def human_pose_detect(
                    detect_model_path='/home/robot/pose-detect/pose_landmarker_full.task',
                    ):
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
    base_options = python.BaseOptions(model_asset_path = detect_model_path)
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        output_segmentation_masks=True)
    detector = vision.PoseLandmarker.create_from_options(options)
    # while(True):
    with open('src/picture_counter.txt','r') as f:
        num = int(f.read())
    print("正在加载第 {} 张图像...".format(num+1))
    rgb_path = "save_picture/color{}.png".format(num)
    bridge = CvBridge()
    depth_img_msg = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image) 
    depth_img_cv = bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding="passthrough")
    image = mp.Image.create_from_file(rgb_path)
    # Detect pose landmarks from the input image.
    detection_result = detector.detect(image)
    pose_landmarks_list = detection_result.pose_landmarks
    # Process the detection result.
    annotated_image,idx_to_coordinates = draw_landmarks_on_image(image.numpy_view(), detection_result)
    # detect_num = int(input("请输入部位编号：(0:甲状腺, 1:心脏, 2:腹部)"))
    detect_num = 0
    annotated_image,keypoint_dict = add_keypoint_neck(num = detect_num, pred_dict = idx_to_coordinates,annotated_image = annotated_image)
    # annotated_image,keypoint_dict  = add_keypoint_neck_1(idx_to_coordinates, annotated_image)
    for idx, coordinates in keypoint_dict.items():
        print(GREEN + "处理后第 {} 个关键点的像素坐标为：{}".format(idx, coordinates))
    return  client, depth_img_cv, keypoint_dict

def robot_move(
                move_keypoint_idx = 0,
                keypoint_dict = None,
                depth_img_cv = None,
                client = None,
            ):
     # 机械臂控制程序
    print(BLUE_BOLD + "---------- 机械臂客户端执行程序 ----------" + ENDC)
    print(YELLOW + "选择的关键点索引为 {} , 对应像素点为：{}".format(move_keypoint_idx,keypoint_dict[move_keypoint_idx]) + ENDC)
    pix_x,pix_y = keypoint_dict[move_keypoint_idx]
    depth_value = depth_img_cv[pix_y, pix_x] 
    depth_default_value = 900
    if depth_value == 0 :
        print("*"*50)
        print(RED + "深度值获取异常，正在使用默认值 {}".format(depth_default_value) + ENDC)
        print("*"*50)
        depth_value = depth_default_value
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
    # 提供控制指令
    print("机械臂服务端正在处理请求...")
    rpy_action_id = 0
    rpy_angle = -45
    rpy = [rpy_angle, rpy_action_id, 0]
    req = grasp_poseRequest()
    req.grasppose_x, req.grasppose_y, req.grasppose_z = camera_point[0], camera_point[1], camera_point[2]
    req.grasppose_R, req.grasppose_P, req.grasppose_Y = rpy[0], rpy[1], rpy[2]
    result = client.call(req)
    rospy.sleep(3)
    return result

def nav_pose_grasp():

    # step 1
    print(YELLOW_BOLD + "---------- AGV底盘机器人导航程序 ----------" + ENDC)
    ip = "0.0.0.0"
    port = 22002
    timeout = 60
    robot_nav = Robot_navigation(
                                ip = ip,
                                port = port,
                                timeout = timeout
                                )

    nav_start_index = 0
    nav_total_num = 2 
    nav_sequence_forward = True
    # robot_nav.start_navigation(start_index=nav_start_index)
    print("机器人是否开启导航:{}".format(robot_nav.start_navigation_flag))
    navigation(
                robot_nav = robot_nav,
                nav_total_num = nav_total_num,
                nav_sequence_forward = nav_sequence_forward
                )
    rospy.sleep(2)
    # # step 2
    # client, depth_img_cv, keypoint_dict = human_pose_detect()
    # # step 3
    # move_keypoint_idx = 0
    # result = robot_move(
    #             move_keypoint_idx = move_keypoint_idx,
    #             keypoint_dict = keypoint_dict,
    #             depth_img_cv = depth_img_cv,
    #             client = client,
    #         )
    # rospy.sleep(1)

    # # 控制小车回到0号点
    # if result.success:
    nav_total_num = 2
    nav_sequence_forward = False
    navigation(
            robot_nav = robot_nav,
            nav_total_num = nav_total_num,
            nav_sequence_forward = nav_sequence_forward
            )
        
    # robot_nav.stop_navigation()
    # print("机器人是否停止导航:{}".format(robot_nav.stop_navigation_flag))
    print(YELLOW_BOLD + "---------- AGV底盘机器人导航程序 ----------" + ENDC)


app = Flask(__name__)
@app.route("/diagnose", methods = ['POST'])
def diagnose():
    p = multiprocessing.Process(target=nav_pose_grasp)
    p.start()
    p.join()
    return "OK", 200

if __name__ == "__main__":
    # app.run()
    for i in range(4):
        print("test;{}".format(i))
        nav_pose_grasp()
