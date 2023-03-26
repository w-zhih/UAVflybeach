#!/usr/bin/python3
from asyncore import read
from re import T
import serial
import time
import numpy as np
import rospy
from sensor_msgs.msg import Imu
import tf


class PublishWTData():
    def __init__(self, ser):
        self.ser = ser
        self.rate = rospy.Rate(500)
        self.pub = rospy.Publisher("imu_real", Imu, queue_size=1)
        self.imudata_dict = {"seq": 0, "frame_id": "imu_real_link", "ax": 0, "ay": 0,
                             "az": 0, "wx": 0, "wy": 0, "wz": 0, "roll": 0, "pitch": 0, "yaw": 0}
        while not rospy.is_shutdown():
            self.SerialRead()
            self.ImuDataLoad()
            self.rate.sleep()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
            np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
            np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return qx, qy, qz, qw

    def ImuDataLoad(self):
        imuMsg = Imu()
        stamp = rospy.get_rostime()
        imuMsg.header.stamp, imuMsg.header.frame_id = stamp, "imu_real_link"
        imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w = self.get_quaternion_from_euler(
            self.imudata_dict["roll"], self.imudata_dict["pitch"], self.imudata_dict["yaw"])

        #imuMsg.orientation_covariance = cov_orientation
        imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z = self.imudata_dict[
            "wx"], self.imudata_dict["wy"], self.imudata_dict["wz"]
        #imuMsg.angular_velocity_covariance = cov_angular_velocity
        imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z = self.imudata_dict[
            "ax"], self.imudata_dict["ay"], self.imudata_dict["az"]
        #imuMsg.linear_acceleration_covariance = cov_linear_acceleration
        self.pub.publish(imuMsg)

    def SerialRead(self):
        rawData = self.ser.read(size=2).hex()
        # print("get raw data:%s" % rawData)
        if rawData == '5561':
            # print("start")
            data = self.ser.read(size=18)
            axL = data[0]
            axH = data[1]
            ayL = data[2]
            ayH = data[3]
            azL = data[4]
            azH = data[5]
            wxL = data[6]
            wxH = data[7]
            wyL = data[8]
            wyH = data[9]
            wzL = data[10]
            wzH = data[11]
            RollL = data[12]
            RollH = data[13]
            PitchL = data[14]
            PitchH = data[15]
            YawL = data[16]
            YawH = data[17]
            # data_dict = {}
            self.imudata_dict["ax"] = ((axH << 8) | axL) / 32768.0 * 16.0
            self.imudata_dict["ay"] = ((ayH << 8) | ayL) / 32768.0 * 16.0
            self.imudata_dict["az"] = ((azH << 8) | azL) / 32768.0 * 16.0
            self.imudata_dict["wx"] = ((wxH << 8) | wxL) / 32768.0 * 2000.0
            self.imudata_dict["wy"] = ((wyH << 8) | wyL) / 32768.0 * 2000.0
            self.imudata_dict["wz"] = ((wzH << 8) | wzL) / 32768.0 * 2000.0
            self.imudata_dict["roll"] = (
                (RollH << 8) | RollL) / 32768.0 * 180.0
            self.imudata_dict["pitch"] = (
                (PitchH << 8) | PitchL) / 32768.0 * 180.0
            self.imudata_dict["yaw"] = ((YawH << 8) | YawL) / 32768.0 * 180.0
            print("Roll: %7.3f" % self.imudata_dict["roll"], "Pitch: %7.3f" %
                  self.imudata_dict["pitch"], "Yaw: %7.3f" % self.imudata_dict["yaw"])
            


if __name__ == '__main__':

    # Initial conditions
    ser = serial.Serial()
    # for linux. Also change the USB# to the correct # if necessary.
    ser.port = '/dev/ttyUSB0'
    ser.baudrate = 115200
    ser.parity = 'N'
    ser.bytesize = 8
    ser.timeout = 1
    ser.open()

    rospy.init_node("wt901_publisher")
    data_pub = PublishWTData(ser)
