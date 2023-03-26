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

class Demo:
    def __init__(self):
        self.bridge_ = CvBridge()
        
        self.image_sub_ = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imagesubCallback)
        while not rospy.is_shutdown():
            rospy.spin()
        rospy.logwarn('Car Controller node shut down.')


    def imagesubCallback(self, data):
        try:
            #将sensor_msgs/Image类型的消息转化为BGR格式图像
            image_detect = self.bridge_.imgmsg_to_cv2(data, 'bgr8')
            # cv2.imwrite(image_detect,'pic.png')
            try:
                print(image_detect.shape)
            except:
                print("eee")
        except CvBridgeError as err:
            print(err)


if __name__ == '__main__':
    print("开始运行")
    rospy.init_node("cross_demo_node")
    try:
        Demo()
    except Exception as e:
        rospy.logerr(e)