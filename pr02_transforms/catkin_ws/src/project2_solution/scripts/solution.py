#!/usr/bin/env python
import rospy

import numpy

import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():

    # base to object
    # The transform from the 'base' coordinate frame to the 'object' coordinate frame consists of:
    # (1) a rotation expressed as (roll, pitch, yaw) of (0.79, 0.0, 0.79)
    # (2) a translation of 1.0m along the resulting y-axis and 1.0m along the resulting z-axis
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"

    bRo = tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)

    object_transform.transform.rotation.x = bRo[0]
    object_transform.transform.rotation.y = bRo[1]
    object_transform.transform.rotation.z = bRo[2]
    object_transform.transform.rotation.w = bRo[3]

    bTo = numpy.dot(tf.transformations.quaternion_matrix(bRo),
                   tf.transformations.translation_matrix((0.0, 1.0, 1.0)))

    bTLo = tf.transformations.translation_from_matrix(bTo)

    object_transform.transform.translation.x = bTLo[0]
    object_transform.transform.translation.y = bTLo[1]
    object_transform.transform.translation.z = bTLo[2]

    br.sendTransform(object_transform)

    # base to robot
    # The transform from the 'base' coordinate frame to the 'robot' coordinate frame consists of:
    # (1) a rotation around the z-axis by 1.5 radians
    # (1) a translation along the resulting y-axis of -1.0m
    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"

    bRr = tf.transformations.quaternion_about_axis(1.5, (0.0, 0.0, 1.0))

    robot_transform.transform.rotation.x = bRr[0]
    robot_transform.transform.rotation.y = bRr[1]
    robot_transform.transform.rotation.z = bRr[2]
    robot_transform.transform.rotation.w = bRr[3]

    bTr = numpy.dot(tf.transformations.quaternion_matrix(bRr),
                   tf.transformations.translation_matrix((0.0, -1.0, 0.0)))

    bTLr = tf.transformations.translation_from_matrix(bTr)

    robot_transform.transform.translation.x = bTLr[0]
    robot_transform.transform.translation.y = bTLr[1]
    robot_transform.transform.translation.z = bTLr[2]

    br.sendTransform(robot_transform)

    # robot to camera
    # The transform from the 'robot' coordinate frame to the 'camera' coordinate frame must be defined as follows:
    # The translation component of this transform is (0.0, 0.1, 0.1)
    # The rotation component this transform must be set such that the camera is pointing directly at the object.
    # In other words, the x-axis of the 'camera' coordinate frame must be pointing directly at the origin of the 'object' coordinate frame.
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"

    camera_transform.transform.translation.x = 0.0
    camera_transform.transform.translation.y = 0.1
    camera_transform.transform.translation.z = 0.1

    rRc = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)

    rTc = numpy.dot(tf.transformations.translation_matrix((0.0,0.1,0.1)),
            tf.transformations.quaternion_matrix(rRc))


    # solve for missing rotation

    # find camera to object Transform Matrix
    bTc = numpy.dot(bTr, rTc)
    bTc_inverse = tf.transformations.inverse_matrix(bTc)
    cTo = numpy.dot(bTc_inverse, bTo)

    # get normalized camera to object translation vector
    # points from camera to object
    cTLo = tf.transformations.translation_from_matrix(cTo)
    v = tf.transformations.unit_vector(cTLo)

    # rotate camera around vector orthogonal to x axis and v
    x = [1.0, 0.0, 0.0]
    w = numpy.cross(x, v)

    # get angle of rotation between x and v
    alpha = numpy.arccos(numpy.dot(x,v))

    # rotare camera around w by alpha degrees
    cRo = tf.transformations.quaternion_about_axis(alpha, w)

    camera_transform.transform.rotation.x = cRo[0]
    camera_transform.transform.rotation.y = cRo[1]
    camera_transform.transform.rotation.z = cRo[2]
    camera_transform.transform.rotation.w = cRo[3]

    br.sendTransform(camera_transform)

if __name__ == '__main__':
    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
