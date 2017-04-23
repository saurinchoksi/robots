#!/usr/bin/env python
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():

    object_tr = geometry_msgs.msg.TransformStamped()
    object_tr.header.stamp = rospy.Time.now()
    object_tr.header.frame_id = "base_frame"
    object_tr.child_frame_id = "object_frame"
    object_q = tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)
    object_tr.transform.rotation.x = object_q[0]
    object_tr.transform.rotation.y = object_q[1]
    object_tr.transform.rotation.z = object_q[2]
    object_tr.transform.rotation.w = object_q[3]
    object_tr.transform.translation.x = 0.0
    object_tr.transform.translation.y = 1.0
    object_tr.transform.translation.z = 1.0
    br.sendTransform(object_tr)

"""
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"

    br.sendTransform(object_transform)

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    br.sendTransform(robot_transform)

    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    br.sendTransform(camera_transform)

"""

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
