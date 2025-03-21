import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

import numpy as np

# 全局变量
bridge = CvBridge()
rgb_img = None
depth_img = None

# 鼠标回调函数
def mouse_callback(event, x, y, flags, param):
    global rgb_img, depth_img
    if event == cv2.EVENT_MOUSEMOVE:
        if rgb_img is not None and depth_img is not None:
            # 获取 RGB 像素值
            rgb_pixel = rgb_img[y, x]
            # 获取深度像素值
            depth_pixel = depth_img[y, x]
            print(f"Pixel coordinates: ({x}, {y})")
            print(f"RGB values: ({rgb_pixel[0]}, {rgb_pixel[1]}, {rgb_pixel[2]})")
            print(f"Depth value: {depth_pixel}")

# RGB 图像回调函数
def rgb_image_callback(msg):
    global rgb_img
    try:
        # 将 ROS 图像消息转换为 OpenCV 图像
        rgb_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        print(f"Error converting RGB image: {e}")

# 深度图像回调函数
def depth_image_callback(msg):
    global depth_img
    try:
        # 将 ROS 图像消息转换为 OpenCV 图像
        depth_img = bridge.imgmsg_to_cv2(msg, "16UC1")
    except Exception as e:
        print(f"Error converting depth image: {e}")

def main():
    global rgb_img, depth_img
    # 初始化 ROS 节点
    rospy.init_node("rgbd_show")

    # 订阅 RGB 图像话题
    rospy.Subscriber('/camera/color/image_raw', Image, rgb_image_callback)
    # 订阅深度图像话题
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, depth_image_callback)
    # rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_image_callback)

    # 创建窗口并设置鼠标回调函数
    cv2.namedWindow("RGB Image")
    cv2.setMouseCallback("RGB Image", mouse_callback)

    try:
        while not rospy.is_shutdown():
            if rgb_img is not None:
                # 显示 RGB 图像
                cv2.imshow("RGB Image", rgb_img)
            if depth_img is not None:
                # 显示深度图像
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("Depth Image", depth_colormap)

            # 按 'q' 键退出循环
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except rospy.ROSInterruptException:
        pass
    finally:
        # 关闭所有 OpenCV 窗口
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
