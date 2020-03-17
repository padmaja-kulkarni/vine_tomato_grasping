#!/usr/bin/env python
import rospy
import tf
import tf2_ros

import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('camera_tf_broadcaster')

    # create broadcaster
    br = tf2_ros.StaticTransformBroadcaster()
    broadcasted = False

    # broadcast if not broadcasted yet
    if not broadcasted:

        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "camera"

        static_transformStamped.transform.translation.x = 0
        static_transformStamped.transform.translation.y = 0.6
        static_transformStamped.transform.translation.z = 1.2

        quat = tf.transformations.quaternion_from_euler(0, 1.57, -1.57)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        br.sendTransform(static_transformStamped)
        broadcasted = True

    rospy.spin()
