#!/usr/bin/env python
# import roslib
# roslib.load_manifest('course_agv_control')
# import rospy
#
# import tf
# import gazebo_msgs.msg
#
# def callback(msg):
#     if name not in msg.name:
#         return
#     arrayIndex = msg.name.index(name)
#     br = tf.TransformBroadcaster()
#     position = msg.pose[arrayIndex].position
#     orientation = msg.pose[arrayIndex].orientation
#     br.sendTransform((position.x, position.y, position.z),
#                      (orientation.x, orientation.y, orientation.z, orientation.w),
#                      rospy.Time.now(),
#                      "robot_base",
#                      "map")
#
# if __name__ == '__main__':
#     robot_name = rospy.get_param('~robot_name', 'course_agv')
#     link_name = rospy.get_param('~link_name', 'robot_base')
#     name = robot_name + "::" + link_name
#     rospy.init_node('robot_tf_broadcaster',anonymous=True)
#     rospy.Subscriber("/gazebo/link_states",
#                      gazebo_msgs.msg.LinkStates,
#                      callback)
#     rospy.spin()
#!/usr/bin/env python
import roslib
roslib.load_manifest('course_agv_control')
import rospy

import tf
import gazebo_msgs.msg
class GazeboLinkPose:
    def __init__(self, name):
        self.orientation = None
        self.position = None
        self.name = name
        # self.link_name_rectified = link_name.replace("::", "_")

        self.states_sub = rospy.Subscriber(
            "/gazebo/link_states", gazebo_msgs.msg.LinkStates, self.callback)
        self.tf_pub = tf.TransformBroadcaster()

    def callback(self, msg):
        try:
            if self.name not in msg.name:
                return
            arrayIndex = msg.name.index(self.name)
            self.position = msg.pose[arrayIndex].position
            self.orientation = msg.pose[arrayIndex].orientation
        except ValueError:
            pass

    def publish_tf(self):
        try:
            br = tf.TransformBroadcaster()
            br.sendTransform((self.position.x, self.position.y, self.position.z),
                         (self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w),
                         rospy.Time.now(),
                         "robot_base",
                         "map")
        except AttributeError:
            pass
if __name__ == '__main__':
    try:
        robot_name = rospy.get_param('~robot_name', 'course_agv')
        link_name = rospy.get_param('~link_name', 'robot_base')
        name = robot_name + "::" + link_name
        gp = GazeboLinkPose(name)
        rospy.init_node('robot_tf_broadcaster',anonymous=True)
        publish_rate = 50
        rate = rospy.Rate(publish_rate)
        while not rospy.is_shutdown():
            gp.publish_tf()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

