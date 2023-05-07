#!/usr/bin/env python
# !/usr/bin/env python

import rospy
import geometry_msgs.msg
import std_msgs.msg


class Subscriber:
    def __init__(self):
        self.sub = rospy.Subscriber(
            '/course_agv/velocity',
            geometry_msgs.msg.Twist, self.callback)
        self.vx = 0.0
        self.vw = 0.0

    def callback(self, msg: geometry_msgs.msg.Twist):
        self.vx = msg.linear.x
        self.vw = msg.angular.z


class Publisher:
    def __init__(self):
        self.left_pub = rospy.Publisher(
            '/course_agv/left_wheel_velocity_controller/command',
            std_msgs.msg.Float64, queue_size=1)
        self.right_pub = rospy.Publisher(
            '/course_agv/right_wheel_velocity_controller/command',
            std_msgs.msg.Float64, queue_size=1)
        self.left_cmd = std_msgs.msg.Float64()
        self.right_cmd = std_msgs.msg.Float64()

    def pub(self, vx, vw):
        if rospy.is_shutdown():
            exit()
        self.left_cmd.data = vx - vw
        self.right_cmd.data = vx + vw
        self.left_pub.publish(self.left_cmd)
        self.right_pub.publish(self.right_cmd)


if __name__ == '__main__':
    rospy.init_node('velocity_publisher', anonymous=True)
    sub = Subscriber()
    pub = Publisher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.pub(sub.vx, sub.vw)
        rate.sleep()
