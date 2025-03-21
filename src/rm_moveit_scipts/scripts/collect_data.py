#! /usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import cv_bridge
import rospy
from rm_moveit_scipts.srv import grasp_pose, grasp_poseRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from saveimg import ImageSaver
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from tools import draw_landmarks_on_image

click_point_pix = ()
depth_value = 0
left_click_points = []
first_click = True

def mouseclick_callback(event, x, y, flags, param):
    global click_point_pix, depth_value, left_click_points, first_click
    if event == cv2.EVENT_LBUTTONDOWN:
        click_point_pix = (x, y)
        left_click_points.append(click_point_pix)
        print(f"Click position: {click_point_pix}")
        depth_value = depth_img_cv[y, x]  # 深度图的像素值
        if depth_value == 0:
            print("error depth value")
            depth_value = 520
        if first_click:
            print(f"Depth value at clicked point: {depth_value}")
            cam_intrinsics = np.array([895.2587280273438, 0.0, 667.689208984375, 0.0, 895.4281616210938, 364.93109130859375, 0.0, 0.0, 1.0]).reshape(3, 3)
            u = x
            v = y
            pos_z = depth_value * 0.001  # 深度图通常以毫米为单位，这里转换为米
            # 计算3D坐标
            ppx = cam_intrinsics[0][2]
            ppy = cam_intrinsics[1][2]
            pos_x = (u - ppx) * (pos_z / cam_intrinsics[0][0])
            pos_y = (v - ppy) * (pos_z / cam_intrinsics[1][1])
            camera_point = np.array([pos_x, pos_y, pos_z])
            rpy_action_id = 0
            rpy_angle = 90
            rpy = [rpy_angle, rpy_action_id, 0]
            # 请求抓取位姿
            req = grasp_poseRequest()
            req.grasppose_x, req.grasppose_y, req.grasppose_z = camera_point[0], camera_point[1], camera_point[2]
            req.grasppose_R, req.grasppose_P, req.grasppose_Y = rpy[0], rpy[1], rpy[2]
            result = client.call(req)
            first_click = False
        else:
            print(f"后续点击像素位置: {click_point_pix}")
            # print(f"后续点击像素值对应的深度值: {depth_value}")

def get_image_counter():
    try:
        # with open('src/picture_counter.txt', 'r') as f:
        #     num =  int(f.read())
        num = int(input("当前是第几次收集数据:"))
        return num
    except FileNotFoundError:
        return 0

def save_image_counter(counter):
    with open('src/picture_counter.txt', 'w') as f:
        f.write(str(counter))

def save_images(rgb_img_cv, depth_img_msg, counter):
    label_img_path = f"src/rm_moveit_scipts/scripts/data_img/label_img{counter}.png"
    cv2.imwrite(label_img_path, rgb_img_cv)

    camera = ImageSaver()
    camera.save_images(
        color_filename=f"src/rm_moveit_scipts/scripts/data_img/color{counter}.png",
        depth_filename=f"src/rm_moveit_scipts/scripts/data_img/depth{counter}.png"
    )
    print(f"正在保存第 {counter} 张图像")

def detect_pose(counter):
    # Create an PoseLandmarker object.
    base_options = python.BaseOptions(model_asset_path='/home/user/robot/pose-detect/pose_landmarker_full.task')
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        output_segmentation_masks=True
    )
    detector = vision.PoseLandmarker.create_from_options(options)

    rgb_path = f"src/rm_moveit_scipts/scripts/data_img/color{counter}.png"
    print(f"正在从 {rgb_path} 加载图像...")
    image = mp.Image.create_from_file(rgb_path)

    # Detect pose landmarks from the input image.
    detection_result = detector.detect(image)
    pose_landmarks_list = detection_result.pose_landmarks

    # Process the detection result.
    annotated_image, idx_to_coordinates = draw_landmarks_on_image(image.numpy_view(), detection_result)
    annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
    # cv2.imshow("annotated_image", annotated_image)

    detect_img_path = f"src/rm_moveit_scipts/scripts/data_img/detect_img{counter}.png"
    cv2.imwrite(detect_img_path, annotated_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return idx_to_coordinates

def save_dicts_to_file(left_click_dict, detect_dict, counter):
    file_path = f"src/rm_moveit_scipts/scripts/data_img/test{counter}.txt"
    with open(file_path, 'w') as f:
        f.write(f"label_list: {left_click_dict}\n")
        f.write(f"detect_list: {detect_dict}\n")
    print(f"已将字典保存到 {file_path}")

if __name__ == "__main__":
    rospy.init_node("grasp_client")
    client = rospy.ServiceProxy("moveit_grasp", grasp_pose)
    rospy.wait_for_service("moveit_grasp")

    counter = get_image_counter()

    while True:
        rgb_img_msg = rospy.wait_for_message('/camera/color/image_raw', Image)
        depth_img_msg = rospy.wait_for_message('/camera/aligned_depth_to_color/image_raw', Image)

        # 将ROS图像消息转换为OpenCV格式
        bridge = CvBridge()
        rgb_img_cv = bridge.imgmsg_to_cv2(rgb_img_msg, desired_encoding="passthrough")
        rgb_img_cv = cv2.cvtColor(rgb_img_cv, cv2.COLOR_RGB2BGR)
        depth_img_cv = bridge.imgmsg_to_cv2(depth_img_msg, desired_encoding="passthrough")

        for idx, point in enumerate(left_click_points):
            if idx > 0:
                rgb_img_cv = cv2.circle(rgb_img_cv, point, 7, (0, 0, 255), 2)
            else:
                rgb_img_cv = cv2.circle(rgb_img_cv, point, 7, (255, 0, 255), 2)

        cv2.imshow('rgb_img', rgb_img_cv)
        cv2.namedWindow('rgb_img')
        cv2.setMouseCallback('rgb_img', mouseclick_callback)

        if cv2.waitKey(1) == ord('c'):
            save_images(rgb_img_cv, depth_img_msg, counter)
            detect_dict = detect_pose(counter)

            left_click_dict = {i: point for i, point in enumerate(left_click_points)}

            save_dicts_to_file(left_click_dict, detect_dict, counter)

            # print(f"label_list:{left_click_dict}")
            # print(f"detect_list:{detect_dict}")

            counter += 1
            save_image_counter(counter)
            break

    cv2.destroyAllWindows()
    