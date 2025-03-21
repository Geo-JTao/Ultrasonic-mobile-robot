#!/usr/bin/env python
# -*- coding: utf-8 -*-
import requests
import json
from socket import *

class Robot_navigation:
    def __init__(self, ip = "0.0.0.0", port = 22002, timeout = 60 ):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.addr = None
        self.connect_init(
                        ip = self.ip,
                        port = self.port,
                        timeout = self.timeout)
        self.api_url = None
        self.token = None
        self.get_robot_token(self.addr)
        self.status = None
        self.start_navigation_flag = False
        self.stop_navigation_flag = False
        self.map_path_name = None
        self.task_info = None

    # 获取机器人UDP广播
    def connect_init(self,ip,port,timeout,byte = 1024):
        udp_server = socket(AF_INET,SOCK_DGRAM)
        udp_server.bind((ip, port))
        udp_server.settimeout(timeout)
        data, self.addr = udp_server.recvfrom(byte)
        data = json.loads(data.decode("utf-8"))
        print("*"*50)
        print("初始化成功")
        # print(json.dumps(data, indent=4))
        print("*"*50)

    # 获取机器人token，采用默认的用户名密码admin
    def get_robot_token(self,addr):
        self.api_url = "http://{ip}:3546/api/v1".format(ip=addr[0])
        res = requests.get("{api_url}/token?username=admin&password=admin".format(api_url=self.api_url))
        self.token = res.json()

    #获取机器人基本信息
    def get_robot_base_info(self,api_url = None ,token = None):
        if api_url == None and token == None:
            api_url = self.api_url
            token = self.token
        res = requests.get("{api_url}/system/status?token={token}".format(
            api_url=api_url,
            token=token["token"]
        ))
        self.status = str(res.json()["status"])

    #开启机器人导航
    def start_navigation(self,api_url = None ,token = None, start_index = 0):
        if api_url == None and token == None:
            api_url = self.api_url
            token = self.token
        print("开始导航")
        # res = requests.get("{api_url}/navigation/start?token={token}&start_index={start_index}".format(
        #     api_url=api_url,
        #     token=token["token"],
        #     start_index=start_index
        # ))
        res = requests.get("{api_url}/navigation/start?token={token}".format(
            api_url=api_url,
            token=token["token"],
        ))
        self.start_navigation_flag = res.json()["result"]
        self.status = self.get_robot_base_info(api_url,token)
        

    #停止机器人导航
    def stop_navigation(self,api_url = None ,token = None):
        if api_url == None and token == None:
            api_url = self.api_url
            token = self.token
        # print("停止导航")
        res = requests.get("{api_url}/navigation/stop?token={token}".format(
            api_url=api_url,
            token=token["token"],
        ))
        self.stop_navigation_flag = res.json()["result"]
        self.status = self.get_robot_base_info(api_url,token)


    # 获取当前地图和路径信息
    def get_current_map_path(self,api_url = None ,token = None):
        if api_url == None and token == None:
            api_url = self.api_url
            token = self.token
        # print("获取当前地图和路径信息")
        res = requests.get("{api_url}/navigation/current_path?token={token}".format(
            api_url=api_url,
            token=token["token"]
        ))
        self.map_path_name = res.json()

    #移动机器人到目标点
    def move_to_index(self,api_url = None ,token = None,idx = 0):
        if api_url == None and token == None:
            api_url = self.api_url
            token = self.token
        print("移动机器人到目标点:{}".format(idx))
        res = requests.get("{api_url}/navigation/move_to_index?token={token}&index={index}".format(
            api_url = api_url,
            token = token["token"],
            index = idx
        ))
        self.task_info = res.json()
        self.status = self.get_robot_base_info(api_url,token)

    # 通过轮询等待任务执行完成
    def wait_for_task_compete(self,api_url = None ,token = None,task_info = None):
        if api_url == None and token == None:
            api_url = self.api_url
            token = self.token
        if task_info == None:
            raise("please input task_info when use 'wait_for_task_compete'")
        
        print("Waitting for task complete...")
        while True:
            res = requests.get("{api_url}/task?token={token}&id={task_id}".format(
            api_url=api_url,
            token=token["token"],
            task_id=task_info["id"]))
            info = res.json()
            if info["state"] == "COMPLETE" or info["state"] == "CANCELLED" or info["state"] == "ERROR":
                break
           

if __name__ == '__main__':
    ip = "0.0.0.0"
    port = 22002
    timeout = 60
    robot_nav = Robot_navigation(ip,port,timeout)
    robot_nav.stop_navigation()
    robot_nav.get_robot_base_info()
    print("机器人当前状态: ", robot_nav.status) 
    input("press 'enter' to start_navigation:")
    start_index = int(input("请输入机器人初始位置(导航点索引):"))
    robot_nav.start_navigation(start_index=start_index)
    print("是否开启导航:{}".format(robot_nav.start_navigation_flag))
    robot_nav.get_current_map_path()
    print("当前导航使用的地图名称: {map},当前导航使用的路径名称:{path}".format(
        map=robot_nav.map_path_name["map"],
        path=robot_nav.map_path_name["path"]
        ))
    for i in range(3):
        print("********** 第 {} 次导航开始 **********".format(i))
        idx = int(input("press number to move_to_index:"))
        robot_nav.move_to_index(idx=idx)
        robot_nav.wait_for_task_compete(task_info=robot_nav.task_info)
    input("press 'enter' to stop_navigation:")
    robot_nav.stop_navigation()
    print("是否停止导航:{}".format(robot_nav.stop_navigation_flag))
