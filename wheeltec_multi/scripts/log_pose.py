#!/usr/bin/env python
# coding=utf-8

import rospy
import tf
import csv
import os
import math


class PoseLogger:
    def __init__(self):
        rospy.init_node('pose_logger', anonymous=True)

        # 从参数服务器获取机器人自身的tf frame和要保存的文件名
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        # 文件名通过参数传入，确保每台车的文件名不同
        file_name = rospy.get_param('~filename', 'default_pose.csv')

        # CSV文件保存路径
        save_directory = "/home/wheeltec/wheeltec_robot/src/wheeltec_multi/param/"

        # 检查目录是否存在，如果不存在则创建
        if not os.path.exists(save_directory):
            os.makedirs(save_directory)

        # 将目录和文件名组合成完整的保存路径
        self.save_path = os.path.join(save_directory, file_name)
        # self.save_path = os.path.join(os.path.expanduser('~'), file_name)

        self.csv_file = open(self.save_path, 'w')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["timestamp", "x", "y", "yaw"])

        self.listener = tf.TransformListener()

        # 设置定时器，以10Hz频率记录
        rospy.Timer(rospy.Duration(0.1), self.log_pose_data)

        rospy.loginfo("位置记录节点启动. Frame: '%s'. 保存至: %s", self.robot_frame, self.save_path)

    def log_pose_data(self, event):
        try:
            # 获取map坐标系到机器人基座标系的变换
            (trans, rot) = self.listener.lookupTransform('map', self.robot_frame, rospy.Time(0))

            timestamp = rospy.Time.now().to_sec()
            x = trans[0]
            y = trans[1]

            # 将四元数转换为欧拉角，获取yaw
            euler = tf.transformations.euler_from_quaternion(rot)
            yaw = euler[2]

            self.writer.writerow([timestamp, x, y, yaw])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF变换获取失败: %s", e)

    def shutdown(self):
        self.csv_file.close()
        rospy.loginfo("位置数据文件已保存。")


if __name__ == '__main__':
    logger = PoseLogger()
    rospy.on_shutdown(logger.shutdown)
    rospy.spin()