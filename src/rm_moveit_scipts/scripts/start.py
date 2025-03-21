#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslaunch
import subprocess
import time
import rospy

# 启动roscore
def start_roscore():
    try:
        roscore_process = subprocess.Popen(['roscore'])
        rospy.loginfo("Started roscore.")
        time.sleep(1)  # 等待roscore启动
        return roscore_process
    except Exception as e:
        rospy.logfatal("Failed to start roscore: %s", str(e))
        exit(1)

# 启动launch节点
def start_roslaunch(launch_file):
    try:
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        launch.start()
        rospy.loginfo("Started ROS launch file: %s", launch_file)
        return launch
    except roslaunch.RLException as e:
        rospy.logfatal("ROS launch failed: %s", str(e))
        exit(1)

# 启动Python文件
def start_python_script(script_path):
    try:
        process = subprocess.Popen(["python", script_path])
        rospy.loginfo("Started Python script: %s", script_path)
        return process
    except Exception as e:
        rospy.logfatal("Failed to start Python script %s: %s", script_path, e)
        exit(1)

if __name__ == "__main__":

    # 启动roscore
    roscore_process = start_roscore()
    # 初始化ros节点
    rospy.init_node('launch_manager', anonymous=False)

    # 添加文件路径
    launch_file_path = "src/rm_robot/rm_bringup/launch/real_rml63_robot.launch"  # 替换为你的 launch 文件路径
    python_script_1 = "src/rm_moveit_scipts/scripts/moveitServer.py"  # 替换为你的Python脚本路径
    # python_script_2 = "src/rm_planning/scripts/another_script.py"  # 如果有第二个脚本

    # 启动ROS launch文件
    launch_process = start_roslaunch(launch_file_path)

    # 等待launch文件启动
    wait_time = 3
    rospy.loginfo(f"Waiting for {wait_time} seconds for ROS to start...")
    time.sleep(wait_time)
    # 启动Python脚本
    script1_process = start_python_script(python_script_1)
    # script2_process = start_python_script(python_script_2)  # 如果有第二个脚本

    try:
        # 保持程序运行
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")
        launch_process.shutdown()
        script1_process.terminate()
        # script2_process.terminate()
        roscore_process.terminate()
