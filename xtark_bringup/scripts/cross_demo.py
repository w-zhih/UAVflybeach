#!/usr/bin/env python
# coding=utf-8

import rospy
import signal
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String, Bool

class CrossDemo:
    def __init__(self, robot_name):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.pose = Pose()

        self.bridge_ = CvBridge()
        self.image_detect = None

        self.cmd_pub = rospy.Publisher(self.robot_name+'/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.robot_name+'/odom', Odometry, self.odom_callback)
        self.image_sub_ = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imagesubCallback)
        self.detect_pub = rospy.Publisher('/detect_result', String, queue_size=10)

        self.color_range_red = [(0, 240, 240), (6, 255, 255)] # 红色的HSV范围 HSV颜色空间 H色调S饱和度V亮度 
        self.color_range_yellow =  [(26, 240, 102), (34, 255, 255)] # 黄色
        self.color_range_blue =  [(100, 200, 71), (116, 255, 255)] # 蓝色
        # ============ 订阅裁判机 =============
        self.imageSub_ = rospy.Subscriber('/tello/cmd_start', Bool, self.startcommandCallback)  # 接收开始的命令
        self.parkPub_ = rospy.Publisher('/AKM_1/parkstate',String,queue_size = 10)
        self.is_begin_ = False
    

        # ============ 预备开始运行 ============
        while not rospy.is_shutdown():
            if self.is_begin_:
                self.Main()
                rospy.spin()
        rospy.logwarn('Car Controller node shut down.')
    
    
    def Main(self):
        detect_2 = None
        detect_4 = None
        detect_5 = None
        result = String()   # detect_result

        # ------------------------------- 1 point
        while self.pose.position.y < 3.7:
            #print("                               Theta: "+str(self.GetTheta()))
            self.CarMove(5, 0)
            rospy.sleep(0.1)
            
        self.CarMove(0, 0)

        # ------------------------------- turn left
        self.Turn(1, 7)

        # ------------------------------- 2 point
        while (self.pose.position.y < 6 and self.pose.position.x > 1.8):
            self.CarMove(5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)

        # ------------------------------- turn right
        self.Turn(-1, 18)

        # ------------------------------- detect=====================
        rospy.sleep(0.3)

        print("Start detecting......point 2")
        detect_2 = '2'+self.detect_ball_wzh()        
        print("point 2:"+detect_2)
        result.data = detect_2
        self.detect_pub.publish(result)

        # ------------------------------- turn left
        self.Turn(1, 10)

        # ------------------------------- 3 point
        while self.pose.position.y < 12.3:
            self.CarMove(5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)
        
        # ------------------------------- turn right
        self.Turn(-1, 18)

        while self.pose.position.x < 3:
            self.CarMove(5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)

        self.Turn(-1, 6)

        while self.pose.position.y < 12.8:
            self.CarMove(-5, 0)
            rospy.sleep(0.1)
        self.CarMove(0, 0)
        # ------------------------------- detect=====================
        rospy.sleep(0.3)

        print("Start detecting......point 4")

        detect_4 = '4'+self.detect_ball_wzh()  
        print("point 4:"+detect_4)
        result.data = detect_4
        self.detect_pub.publish(result)

        # ------------------------------- 4 point
        self.TurnB(1, 3)

        while self.pose.position.y < 13.8:
            self.CarMove(-5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)

        # ------------------------------- turn right
        self.TurnB(1, 13)

        while self.pose.position.x > 3:
            self.CarMove(5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)

        # ------------------------------- detect=====================
        rospy.sleep(0.3)

        print("Start detecting......point 5")
        detect_5 = '5'+self.detect_ball_wzh()  
        print("point 5:"+detect_5)
        result.data = detect_5
        self.detect_pub.publish(result)

        # ------------------------------- land
        while self.pose.position.x < 3.5:
            self.CarMove(-5, 0)
            rospy.sleep(0.1)
        
        self .CarMove(0, 0)


        rospy.loginfo("Racecar reached")

        park = String()
        park.data = "PARKING"
        self.parkPub_.publish(park)
        print("停靠消息已发布。")



    def Turn(self, DIR, CNT):
        cnt = 0
        while cnt < CNT:
            self.CarMove(3, DIR)
            rospy.sleep(0.05)
            cnt = cnt + 1
        self.CarMove(0, 0)
    

    def TurnB(self, DIR, CNT):
        cnt = 0
        while cnt < CNT:
            self.CarMove(-3, DIR)
            rospy.sleep(0.05)
            cnt = cnt + 1
        self.CarMove(0, 0)


    def GetTheta(self):
        PI = math.pi
        c = self.pose.orientation.w
        s = self.pose.orientation.z
        theta1 = 2 * math.acos(c)
        theta2 = 2 * math.asin(s)
        print("("+str(c)+" , " +str(s)+")")
        theta = 0
        if(c >= 0 and s >= 0):
            theta = (theta1 + theta2)/2
        elif(c < 0 and s >=0):
            theta = (theta1 + PI - theta2)/2
        elif(c < 0 and s < 0):
            theta = (2 * PI - theta1 - theta2)/2
        else:
            theta = (4 * PI - theta1 + theta2)/2

        # print("1: "+str(theta1))
        # print("2: "+str(theta2))
        return theta


    def imagesubCallback(self, data):
        try:
            #将sensor_msgs/Image类型的消息转化为BGR格式图像
            self.image_detect = self.bridge_.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as err:
            print(err)

    def detect_ball_wzh(self):
        print("到达检测点1，开始检测")
        if self.image_detect is None:
            return 'n'
        image_copy = self.image_detect.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]
        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道


        print('开始检测是否为黄球')

        frame_yellow = cv2.inRange(frame, self.color_range_yellow[0], self.color_range_yellow[1])  # 对原图像和掩模进行位运算
        dilated_yellow = cv2.dilate(frame_yellow, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_yellow = cv2.HoughCircles(dilated_yellow, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=5, maxRadius=100)
        print(circles_yellow)
        if circles_yellow is not None:
            return 'y'


        print('开始检测是否为红球')

        frame_red = cv2.inRange(frame, self.color_range_red[0], self.color_range_red[1])  # 对原图像和掩模进行位运算
        dilated_red = cv2.dilate(frame_red, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_red = cv2.HoughCircles(dilated_red, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=5, maxRadius=100)
        print(circles_red)
        if circles_red is not None:
            return 'r'
        

        print('开始检测是否为蓝球')

        frame_blue = cv2.inRange(frame, self.color_range_blue[0], self.color_range_blue[1])  # 对原图像和掩模进行位运算
        dilated_blue = cv2.dilate(frame_blue, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2) # 膨胀
        circles_blue = cv2.HoughCircles(dilated_blue, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=5, maxRadius=100)
        print(circles_blue)
        if circles_blue is not None:

            return 'b'

        return 'e'

    def detect_ball(self):
        return (self.detect_ball_blue())or(self.detect_ball_red())or(self.detect_ball_())

    def detect_ball_yellow(self):
        color_range_ = [(26, 43, 46), (34, 255, 255)] # Yellow 的HSV范围 HSV颜色空间 H色调S饱和度V亮度 

        if self.image_detect is None:
            return False
        image_copy = self.image_detect.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        frame = cv2.inRange(frame, color_range_[0], color_range_[1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                area_max_contour = c

        if area_max_contour is not None:
            if contour_area_max > 50:
                return True
        return False

    def detect_ball_red(self):
        color_range_ = [(0, 250, 250), (2, 255, 255)] # Red 的HSV范围 HSV颜色空间 H色调S饱和度V亮度 

        if self.image_detect is None:
            return False
        image_copy = self.image_detect.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        frame = cv2.inRange(frame, color_range_[0], color_range_[1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                area_max_contour = c

        if area_max_contour is not None:
            if contour_area_max > 50:
                return True
        return False

    def detect_ball_blue(self):
        color_range_ = [(100, 43, 46), (124, 255, 255)] # Blue 的HSV范围 HSV颜色空间 H色调S饱和度V亮度 

        if self.image_detect is None:
            return False
        image_copy = self.image_detect.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        frame = cv2.inRange(frame, color_range_[0], color_range_[1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
        (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                area_max_contour = c

        if area_max_contour is not None:
            if contour_area_max > 50:
                return True
        return False
    #--------------------------------------

    def odom_callback(self, msg):
        self.pose = msg.pose.pose


    def CarMove(self, x, z):
        self.cmd_twist.linear.x = x
        self.cmd_twist.angular.z = z
        self.cmd_pub.publish(self.cmd_twist)


    def sigint_handler(self, signum, frame):
        self.CarMove(0, 0)
        rospy.logwarn("Catched interrupt signal! Stop and exit...")
        exit()

        # 接收开始信号
    def startcommandCallback(self, msg):
        self.is_begin_ = msg.data

if __name__ == '__main__':
    print("开始运行")
    rospy.init_node("cross_demo_node")
    robot_name = rospy.get_param('~robot_name', 'AKM_1')
    print(robot_name)
    try:
        CrossDemo(robot_name)
    except Exception as e:
        rospy.logerr(e)
    

