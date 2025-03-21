#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from move_robot import MoveIt_Control
from saveimg import ImageSaver
from rm_moveit_scipts.srv import grasp_pose,grasp_poseResponse
import numpy as np
import math
import tf.transformations as tft

class RM_Robot:
    def __init__(self):
        self.move = MoveIt_Control()
        self.camera = ImageSaver()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

    def get_base_link_pos(self,x,y,z):
        cam_pos = np.array([x,y,z,1])
        tran_R_1 = np.dot(self.move.cam_pose[0:3,0:3],cam_pos[0:3])
        tran_T_2 = tran_R_1 + self.move.cam_pose[0:3,3]
        base_link_pos = tran_T_2
        print('base_link_pos_by_matrix: {}'.format(base_link_pos))
        return base_link_pos

    def camera_to_base_link_pos(self,x,y,z):
        cam_position = [x,y,z]
        point_source = PointStamped()
        point_source.header.frame_id = "camera_link"
        point_source.header.stamp = rospy.Time.now()
        point_source.point.x = cam_position[0]
        point_source.point.y = cam_position[1]
        point_source.point.z = cam_position[2]
        point_target = self. buffer.transform(point_source,"base_link",timeout=rospy.Duration(0.1))
        base_link_pos = [point_target.point.x,point_target.point.y,point_target.point.z]
        print("base_link_pos_by_tf: {}".format(base_link_pos))
        return base_link_pos

    def grasp(self,base_link_pos,angle):
        pre_grasp_pos =  [base_link_pos[0],base_link_pos[1],base_link_pos[2]+0.1]
        grasp_pos =  [base_link_pos[0],base_link_pos[1],base_link_pos[2]]
        rpy = [-2.93, 0, 1.57]
        rpy = [round(num, 2) for num in rpy]
        try:
            self.move.move_p(pre_grasp_pos,rpy,a=0.1,v=0.1,time_sleep=0.05)
            self.move.move_p(grasp_pos,rpy,a=0.1,v=0.1,time_sleep=0.05)
            rospy.sleep(0.5)
            joint_value = self.move.arm.get_current_joint_values()
            if (abs(angle)>90):
                angle = -(180-abs(angle))
            angle =abs(90-abs(angle))
            tool_rotation_angle = (angle /360 *2* np.pi) 
            joint_value[5] = joint_value[5] +tool_rotation_angle
            self.move.move_j(joint_value,a=0.1,v=0.1,time_sleep=0.05)
            input("input any key to go home:")
            # you can add your control gripper code in here
            self.move.go_home(time_sleep = 0.05)
            rospy.sleep(0.5)
            grasp_success = True
        except:
            grasp_success = False
        return grasp_success

    def push_test(self, pos_base = None,
                        rpy_base = None,
                        rpy_offsets = None,
                        xyz_offsets = None,
                        a = 0.5,
                        v = 0.4):
        rpy_base = [-3.0356, -0.078, 2.75]
        # 不同的 rpy 偏移量列表
        # r前后摇； p左右摇； y逆顺时针；
        r_offset_1 = -0.2
        r_offset_2 = 0.38
        p_offset_1 = -0.169
        p_offset_2 = 0.395
        y_offset_1 = 1.57/2
        y_offset_2 = -1.57/2
        rpy_offsets = [
            [0, p_offset_1, 0],
            [0, p_offset_2 , 0],
            [0,0,0],
            [r_offset_1, 0, 0],
            [r_offset_2, 0, 0],
            [0,0,0],
            [0, 0 , y_offset_1],
            [0, 0 , y_offset_2],
            [0,0,0],
            [0, p_offset_1, 0],
            [0, p_offset_2 , 0],
            [0,0,0],
            [r_offset_1, 0, 0],
            [r_offset_2, 0, 0],
            [0,0,0],
            [0, 0 , y_offset_1],
            [0, 0 , y_offset_2],
            [0,0,0]
        ]
        
        # 不同的 xyz 偏移量列表
        # x_offset_1 = -0.045
        # x_offset_2 = 0.02
        # y_offset_1 = -0.03
        # y_offset_2 = 0.025
        # xyz_offsets = [
        #     [x_offset_1, 0, 0],
        #     [x_offset_2, 0, 0],
        #     [0,0,0],
        #     [0, y_offset_1, 0],
        #     [0, y_offset_2 , 0],
        #     [0,0,0],
        # ]
        try:
            pre_pose_1 = [-0.3487,-0.0055, 1.4984-0.85,-2.727, -0.0048, 1.58]
            pre_pose_2 = [-0.4530, -0.00672, 1.2344-0.85,-2.931, 0.0043, 1.58]
            pre_pose_3 = [ -0.674, 0.1157,1.129-0.85,-3.025, -0.144, 1.48]
            waypoints_list = [pre_pose_1,pre_pose_2,pre_pose_3]
            # self.move.move_l(waypoints_list=waypoints_list,maxtries=100,
            #                 time_sleep=0.5,a=a,v=v)
            # 首先移动到检测点上方一定高度（0.15m）
            pre_pos_list = [pos_base[0],pos_base[1],pos_base[2] + 0.15 , *rpy_base]
            waypoints_list.append(pre_pos_list)
            pose_base_list = [*pos_base, *rpy_base] 
            waypoints_list.append(pose_base_list)
            # 执行固定位置下不同rpy的姿态移动
            for idx,offset in enumerate(rpy_offsets):
                # rpy_new = [rpy_base[i] + offset[i] for i in range(len(rpy_base))]
                # rpy_pose_list_new = [*pos_base, *rpy_new] 
                # waypoints_list.append(rpy_pose_list_new)
                if offset[0] != 0:
                    # print("正在绕x轴旋转")
                    orientation_r_new = [rpy_base[i] + offset[i] for i in range(len(rpy_base))]
                    orientation_r_list= [*pos_base, *orientation_r_new]
                    waypoints_list.append(orientation_r_list)
                elif offset[1] != 0:
                    # print("正在绕y轴旋转")
                    orientation_p_new = [rpy_base[i] + offset[i] for i in range(len(rpy_base))]
                    orientation_p_list= [*pos_base, *orientation_p_new]
                    waypoints_list.append(orientation_p_list)
                elif offset[2] != 0:
                    # print("正在绕z轴旋转")
                    pos_base_down = [pos_base[0],pos_base[1],pos_base[2]-0.005]
                    orientation_y_new = [rpy_base[i] + offset[i] for i in range(len(rpy_base))]
                    orientation_y_list= [*pos_base_down, *orientation_y_new]
                    waypoints_list.append(orientation_y_list)
                else :
                    rpy_new = [rpy_base[i] + offset[i] for i in range(len(rpy_base))]
                    rpy_pose_list_new = [*pos_base, *rpy_new] 
                    waypoints_list.append(rpy_pose_list_new)

            # 防止旋转后末端位置发生变化，让末端移动到起始位置上一点点高度
            # pre_pos_list[2] -= 0.1
            # waypoints_list.append(pre_pos_list)
            # waypoints_list.append(pose_base_list)
            # # 执行固定姿态下不同xyz的位置移动
            # for idx,offset in enumerate(xyz_offsets):
            #     # xyz_new = [pos_base[i] + offset[i] for i in range(len(pos_base))]
            #     # xyz_pose_list_new = [*xyz_new, *rpy_base] 
            #     # waypoints_list.append(xyz_pose_list_new)
            #     if offset[0] != 0:
            #         # 考虑人体脖子坡度，稍微抬高/降低z轴
            #         offset[2] += offset[0] * -0.2
            #         x_pos_new = [pos_base[i] + offset[i] for i in range(len(pos_base))]
            #         x_pos_list= [*x_pos_new,*rpy_base]
            #         waypoints_list.append(x_pos_list)
            #     elif offset[1] != 0:
            #         # 考虑人体脖子坡度，稍微抬高/降低z轴
            #         offset[2] += offset[1] * -0.2
            #         y_pos_new = [pos_base[i] + offset[i] for i in range(len(pos_base))]
            #         y_pos_list= [*y_pos_new,*rpy_base]
            #         waypoints_list.append(y_pos_list)
            #     else:
            #         xyz_new = [pos_base[i] + offset[i] for i in range(len(pos_base))]
            #         xyz_pose_list_new = [*xyz_new, *rpy_base] 
            #         waypoints_list.append(xyz_pose_list_new)

            print("len of waypoints_list:{}".format(len(waypoints_list)))
            self.move.move_l(waypoints_list=waypoints_list,maxtries=100,
                            time_sleep=0.5,a=a,v=v)
            self.move.go_home()
            push_success = True
        except:
            push_success = False
        return push_success

    def add_vertical_move(self,pos_base = None , vertical_rpy_base = None, waypoints_list = None ):
        waypoints_list.append([pos_base[0], pos_base[1], pos_base[2] + 0.04, *vertical_rpy_base])
        waypoints_list.append([*pos_base,*vertical_rpy_base])
        vertical_pos_list_1 = [ pos_base[0], pos_base[1] - 0.02, pos_base[2] - 0.01 , *vertical_rpy_base ]
        waypoints_list.append(vertical_pos_list_1)
        vertical_pos_list_2 = [ pos_base[0], pos_base[1] + 0.04, pos_base[2] - 0.01  , *vertical_rpy_base ]
        waypoints_list.append(vertical_pos_list_2)
        waypoints_list.append(vertical_pos_list_1)
        waypoints_list.append(vertical_pos_list_2)
        waypoints_list.append(vertical_pos_list_2)
        waypoints_list.append(vertical_pos_list_1)
        waypoints_list.append(vertical_pos_list_2)
        waypoints_list.append(vertical_pos_list_1)
        waypoints_list.append(vertical_pos_list_2)
        return waypoints_list
       
def grasp_callback(req):
    rospy.loginfo("正在处理客户端请求...")
    cam_position = [req.grasppose_x,req.grasppose_y,req.grasppose_z]
    print("收到相机数据:{}".format(cam_position))
    baselink_pos = rm_robot.camera_to_base_link_pos(cam_position[0],cam_position[1],cam_position[2])
    baselink_pos = [round(num, 3) for num in baselink_pos]
    angle = req.grasppose_R
    action_id = req.grasppose_P
    if action_id == 0:
        # 超声仪需要稍微高度向下一些方便扫描
        baselink_pos[2] -= 0.018 
        rm_robot.push_test( pos_base = baselink_pos, a = 0.5, v = 0.3)
    elif action_id == 1:
        baselink_pos[1] += 0.02
        rm_robot.grasp(baselink_pos,angle)

    response = grasp_poseResponse()
    response.success = True
    return response

if __name__ == "__main__":
    rm_robot = RM_Robot()
    rm_robot.camera.save_images(color_filename="save_picture/color{}.png",
                            depth_filename="save_picture/depth{}.png".format(rm_robot.camera.counter))
    print("Saving {} image".format(rm_robot.camera.counter))
    server = rospy.Service("moveit_grasp",grasp_pose, grasp_callback )
    print("*********************************")
    print("Waiting for client request......")
    print("*********************************")
    rospy.spin()