#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys,tf
import moveit_commander
from moveit_commander import PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from copy import deepcopy

class MoveIt_Control:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_control_server', anonymous=False)
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.joint_tolerance = 0.01
        self.position_tolerance = 0.01
        self.orientation_tolerance = 0.02
        self.planning_time = 10
        self.arm.set_goal_joint_tolerance(self.joint_tolerance)
        self.arm.set_goal_position_tolerance(self.position_tolerance)
        self.arm.set_goal_orientation_tolerance(self.orientation_tolerance)

        self.end_effector_link = self.arm.get_end_effector_link()
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.set_planning_time(self.planning_time)
        self.arm.allow_replanning(True)
        self.arm.set_planner_id("TRRT")
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)
        self.go_home(a = 0.3,v = 0.3)
        self.set_scene(reference_frame="base_link",table_size=[0.95,2,0.5],table_pos=[-0.6,0.3,-0.35])  

    def move_j(self, joint_configuration = None,a = 1,v = 1,time_sleep = 0.5):
        if joint_configuration == None:
            joint_configuration = [0.22671039347648622, -0.24478859786987306, 1.8223558265686035, 
                                -0.7253092260360718, 0.14525380625724793, -0.8118437696456909]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        self.arm.set_joint_value_target(joint_configuration)
        self.arm.go()
        rospy.sleep(time_sleep)

    def move_p(self, position,RPY = None,a = 1,v = 1,time_sleep = 0.5):
        # target_pos_str = ", ".join("{:.4f}".format(p) for p in position)
        # target_rpy_str = ", ".join("{:.4f}".format(r) for r in RPY)
        # print("target: pos={} , rpy={}".format(target_pos_str,target_rpy_str))
        if position is None:
            raise("you need input the base_link position ")
        if RPY == None:
            RPY = [-2.97, -0.056, 1.55]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2] 
        q = quaternion_from_euler(RPY[0],RPY[1],RPY[2])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        # traj = self.arm.plan()  # maybe it only work in python2
        plan_success,traj,planning_time,error_code = self.arm.plan()
        try:
            self.arm.execute(traj)
        except Exception as e:
            print("Error:", str(e))
        # 返回末端工具坐标系相对于world的位姿，在urdf中设置了world与base_link的z轴相差0.85
        current_pose = self.arm.get_current_pose().pose
        current_pose_x = current_pose.position.x
        current_pose_y = current_pose.position.y
        current_pose_z = current_pose.position.z - 0.85
        current_position = [current_pose_x, current_pose_y, current_pose_z]
        quaternion = (current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w)
        current_rpy = tf.transformations.euler_from_quaternion(quaternion)
        # current_pos_str = ", ".join("{:.4f}".format(p) for p in current_position)
        # current_rpy_str = ", ".join("{:.4f}".format(r) for r in current_rpy)
        # print("current: pos={} , rpy={}".format(current_pos_str,current_rpy_str))
        for i in range(len(current_position)):
            pos_error = position[i] - current_position[i]
            rpy_error = RPY[i] - current_rpy[i]
            if pos_error > self.position_tolerance :
                print("current_pos not enough , error = {} ".format(pos_error))
            if rpy_error > self.orientation_tolerance:
                print("current_rpy not enough , error = {} ".format(rpy_error))
        rospy.sleep(time_sleep)

    # 机械臂按多个目标位姿[x, y, z, r, p, y]执行笛卡尔路径规划
    def move_l(self, waypoints_list, a = 0.5, v = 0.5, time_sleep=0.5,maxtries = 100):
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        waypoints = []
        for pose in waypoints_list:
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = pose[0]
            target_pose.pose.position.y = pose[1]
            target_pose.pose.position.z = pose[2]
            q = quaternion_from_euler(pose[3], pose[4], pose[5])
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]
            waypoints.append(deepcopy(target_pose.pose))
        self.arm.set_start_state_to_current_state()
        fraction = 0.0  
        maxtries = maxtries  
        attempts = 0  
        self.arm.set_start_state_to_current_state()
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                                waypoints = waypoints,
                                eef_step = 0.001,
                                avoid_collisions=True,
                                path_constraints=None)
            attempts += 1
        if fraction == 1.0:
            rospy.loginfo("路径规划成功,正在移动机械臂...")
            self.arm.execute(plan)
            rospy.loginfo("路径执行完成。")
        else:
            rospy.loginfo(f"路径规划失败，仅 {fraction:.2f} 覆盖率，尝试了 {maxtries} 次。")
        rospy.sleep(time_sleep)

    def go_home(self,a = 0.1,v = 0.3,time_sleep = 0.5,grasp_flag = False):
        rml63_grasp_home = [0, 0.21641490745544434, -1.5589654956817627, 
                          0, -1.5743913425445557, 0.0]
        bed_test_1_home = [0.003926,-0.0113774,-0.455218,-0.00022,-2.2540689,-0.01350629]
        home_position = rml63_grasp_home if grasp_flag == True else bed_test_1_home
        print("正在回到起始位置，速度：{},加速度:{}".format(a,v))
        self.move_j(home_position,a = a,v = v)
        rospy.sleep(time_sleep)

    def set_scene(self,table_id = 'table', reference_frame = "world", table_size = [1.5,1.5,0.1], table_pos = [0,0,0]):
        self.scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        self.scene.remove_world_object(table_id)
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = table_pos[0]
        table_pose.pose.position.y = table_pos[1]
        table_pose.pose.position.z = table_pos[2]
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box(table_id, table_pose, table_size)
        self.setColor(table_id, r = 0.5, g = 0.5, b = 0.5, a = 1.0)
        self.sendColors()

    def setColor(self, name, r, g, b, a):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

    def testRobot(self):
        try:
            print("Test for robot...")
            pos = [-0.559,  - 0.055, 1.105-0.85, ]
            rpy = [-3.0356, -0.078, 2.69,]
            self.move_p(pos,rpy)
            # test_joint_position= [-6.98000033153221e-05, 0.0019020499542355537, -1.4231696361541748,
            #                     1.7450000828830525e-05, -1.2554402446746826, -6.98000033153221e-05]
            # self.move_j(test_joint_position,a=0.3,v=0.3)
            # rospy.sleep(0.3)
            # self.go_home(grasp_flag=True)
            # base_pos = [-0.4593,0.0161,0.4091-0.2]
            # base_rpy = [-2.97, -0.056, 1.55]
            # self.move_p(position=base_pos,RPY=base_rpy,a=0.1,v=0.1)
            # rospy.sleep(1)
            # self.go_home()  
            # pose_1 = [-0.3487,-0.0055, 1.4984-0.85,-2.727, -0.0048, 1.58]
            # pose_2 = [-0.4530, -0.00672, 1.2344-0.85,-2.931, 0.0043, 1.58]
            # pose_3 = [-0.5782, -0.300, 1.118-0.85,3.125, 0.0763, 1.98]
            # self.move_l(waypoints_list=[pose_1,pose_2,pose_3],a=0.3,v=0.4,time_sleep=0.5)
            
        except Exception as e:
            print("Test fail! ")
            print("Error:", str(e))
     
    def test_push(self, pos_base = [-0.37588, 0.0061, 0.388],
                        rpy_base = [-2.93, 0, 1.57],
                        rpy_offsets = None,
                        xyz_offsets = None ):
        pre_pose_1 = [-0.3487,-0.0055, 1.4984-0.85,-2.727, -0.0048, 1.58]
        pre_pose_2 = [-0.4530, -0.00672, 1.2344-0.85,-2.931, 0.0043, 1.58]
        pre_pose_3 = [-0.5782, -0.300, 1.118-0.85,3.125, 0.0763, 1.98]
        waypoints_list = [pre_pose_1,pre_pose_2,pre_pose_3]
        base_pose_list = [*pos_base, *rpy_base] 
        waypoints_list.append(base_pose_list)
        # 执行固定位置下不同rpy的姿态移动
        # input("执行固定位置下不同rpy的姿态移动:")
        for offset in rpy_offsets:
            rpy_new = [rpy_base[i] + offset[i] for i in range(len(rpy_base))]
            rpy_pose_list_new = [*pos_base, *rpy_new] 
            waypoints_list.append(rpy_pose_list_new)
        # 执行固定姿态下不同xyz的位置移动
        # input("执行固定姿态下不同xyz的位置移动:")
        for offset in xyz_offsets:
            xyz_new = [pos_base[i] + offset[i] for i in range(len(pos_base))]
            xyz_pose_list_new = [*xyz_new, *rpy_base] 
            waypoints_list.append(xyz_pose_list_new)
        self.move_l(waypoints_list=waypoints_list,a=0.3,v=0.4)


if __name__ =="__main__":
    moveit_server = MoveIt_Control()
    # 不同的 rpy 偏移量列表
    r_offset_1 = 5.9
    r_offset_2 = 0.31
    p_offset_1 = 0.395
    p_offset_2 = -0.169
    y_offset_1 = -1.57/2
    y_offset_2 = 1.57/2
    rpy_offsets = [
        [r_offset_1, 0, 0],
        [r_offset_2, 0, 0],
        [0, p_offset_1, 0],
        [0, p_offset_2 , 0],
        [0, 0 , y_offset_1],
        [0, 0 , y_offset_2],
    ]
    # 不同的 xyz 偏移量列表
    x_offset_1 = 0.03
    x_offset_2 = -0.05
    y_offset_1 = 0.07
    y_offset_2 = -0.02
    z_offset_1 = 0.05
    z_offset_2 = -0.02
    xyz_offsets = [
        [x_offset_1, 0, 0],
        [x_offset_2, 0, 0],
        [0, y_offset_1, 0],
        [0, y_offset_2 , 0],
        [0, 0 , z_offset_1],
        [0, 0 , z_offset_2],
    ]
    moveit_server.test_push(rpy_offsets=rpy_offsets,xyz_offsets=xyz_offsets)
    # moveit_server.testRobot()
