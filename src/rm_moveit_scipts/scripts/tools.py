from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import joblib
import cv2

def draw_landmarks_on_image(rgb_image, detection_result):
  idx_to_coordinates = {}
  pose_landmarks_list = detection_result.pose_landmarks
  annotated_image = np.copy(rgb_image)
  # Loop through the detected poses to visualize.
  for idx in range(len(pose_landmarks_list)):
    pose_landmarks = pose_landmarks_list[idx]
    # Draw the pose landmarks.
    pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    pose_landmarks_proto.landmark.extend([
      landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
    ])
    idx_to_coordinates = solutions.drawing_utils.draw_landmarks(
      annotated_image,
      pose_landmarks_proto,
      solutions.pose.POSE_CONNECTIONS,
      solutions.drawing_styles.get_default_pose_landmarks_style())
  return annotated_image, idx_to_coordinates


def add_keypoint_neck_1(idx_to_coordinates:dict,annotated_image):
  keypoint_dict = {}
  keypoint_dict[10] = idx_to_coordinates[10]
  keypoint_dict[12] = idx_to_coordinates[12]
  # 方式1：10号点和12号点加权
  new_x = int((keypoint_dict[10][0] + keypoint_dict[12][0]) * 0.52 )
  new_y = int((keypoint_dict[10][1] + keypoint_dict[12][1]) * 0.44 )
  robot_point_1 = (new_x,new_y)
  keypoint_dict[1] = robot_point_1
  radius = 5  # 圈的半径
  color = (0, 0, 255)
  thickness = 2  # 圈的线宽
  cv2.circle(annotated_image, robot_point_1, radius, color, thickness)
  cv2.putText(annotated_image, '1', robot_point_1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
  # 方式2：11号点和12号点中心点
  keypoint_dict[11] = idx_to_coordinates[11]
  new_x = int(keypoint_dict[11][0] * 0.5 + keypoint_dict[12][0] * 0.5)
  new_y = int(keypoint_dict[11][1] * 0.5 + keypoint_dict[12][1] * 0.5)
  robot_point_2 = (new_x - 100  ,new_y )
  keypoint_dict[2] = robot_point_2
  cv2.circle(annotated_image, robot_point_2, radius, (255,0,0), thickness)
  cv2.putText(annotated_image, '2', robot_point_2, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

  annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
  cv2.imwrite("/home/robot/rm_robot_ws/save_picture/pred_pose_neck.png",annotated_image)
  return annotated_image,keypoint_dict

def add_keypoint_neck(
                      num = 0,
                      pred_dict = None,
                      annotated_image = None):
    keypoint_dict = {}
   
    Thyroid_theta_best_path = "/home/robot/pose-detect/Thyroid_least_squares_model.pkl"
    Thyroid_lasso_path = "/home/robot/pose-detect/Thyroid_lasso_model.pkl"
    Heart_theta_best_path = "/home/robot/pose-detect/Heart_least_squares_model.pkl"
    Heart_lasso_path = "/home/robot/pose-detect/Heart_lasso_model.pkl"
    Belly_theta_best_path = "/home/robot/pose-detect/Belly_least_squares_model.pkl"
    Belly_lasso_path = "/home/robot/pose-detect/Belly_lasso_model.pkl"

    pix_10 = pred_dict[10]
    pix_11 = pred_dict[11]
    pix_12 = pred_dict[12]
    pix_23 = pred_dict[23]
    pix_24 = pred_dict[24]

    if num == 0:
      theta_best_path = Thyroid_theta_best_path
      lasso_path = Thyroid_lasso_path
      detect_points = [[pix_10,pix_11,pix_12]]

    elif num == 1:
      theta_best_path = Heart_theta_best_path
      lasso_path = Heart_lasso_path
      detect_points = [[pix_11,pix_12,pix_23,pix_24]]

    elif num == 2:
      theta_best_path = Belly_theta_best_path
      lasso_path = Belly_lasso_path
      detect_points = [[pix_11,pix_12,pix_23,pix_24]]

    else:
      raise("输入的部位编号有误！")

    theta_best = joblib.load(theta_best_path)
    lasso = joblib.load(lasso_path)
    # print("detect_points", detect_points)
    test_detect_array = np.array(detect_points, dtype=np.float32)
    # raise
    X_test_flat = test_detect_array.reshape(test_detect_array.shape[0], -1)
    X_test_b = np.c_[np.ones((X_test_flat.shape[0], 1)), X_test_flat]
    # 最小二乘法进行预测
    y_pred_ls = X_test_b.dot(theta_best)
    # Lasso 回归进行预测
    y_pred_lasso = lasso.predict(X_test_flat)
    # 定义混合权重
    weight_ls = 0.75  # 最小二乘法的权重
    weight_lasso =0.25 # Lasso 回归的权重
    # 混合预测结果
    y_pred_mixed = weight_ls * y_pred_ls + weight_lasso * y_pred_lasso
    x_offset = -15
    y_offset = 0
    # 应用超参数调整预测结果
    y_pred_mixed[:, 0] += x_offset
    y_pred_mixed[:, 1] += y_offset
    # 预测点的坐标列表
    pred_coords = [[round(x, 2), round(y, 2)] for x, y in y_pred_mixed]

    pred_x = int(pred_coords[0][0])
    pred_y = int(pred_coords[0][1])
    keypoint_dict[0] = (pred_x,pred_y)
    cv2.circle(annotated_image, keypoint_dict[0], 5, (255,0,0), 2)
    cv2.putText(annotated_image, '0', keypoint_dict[0], cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
    cv2.imwrite("/home/robot/rm_robot_ws/save_picture/pred_pose_neck.png",annotated_image)
    return annotated_image,keypoint_dict

def add_keypoint_belly(idx_to_coordinates:dict,annotated_image):
  keypoint_dict = {}
  keypoint_dict[11] = idx_to_coordinates[11]
  keypoint_dict[24] = idx_to_coordinates[24]
  keypoint_dict[23] = idx_to_coordinates[23]
  new_x = int((keypoint_dict[11][0] + keypoint_dict[24][0]) * 0.5 )
  new_y = int((keypoint_dict[11][1] + keypoint_dict[24][1]) * 0.5 )
  new_x = int((new_x + keypoint_dict[23][0]) * 0.5 )
  new_y = int((new_y + keypoint_dict[23][1]) * 0.5 )
  robot_point_1 = (new_x  ,new_y + 100 )
  keypoint_dict[1] = robot_point_1
  radius = 5  # 圈的半径
  color = (0, 0, 255)
  thickness = 2  # 圈的线宽
  cv2.circle(annotated_image, robot_point_1, radius, color, thickness)
  cv2.putText(annotated_image, '1', robot_point_1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
  annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
  cv2.imwrite("/home/robot/rm_robot_ws/save_picture/pred_pose_belly.png",annotated_image)
  return annotated_image,keypoint_dict