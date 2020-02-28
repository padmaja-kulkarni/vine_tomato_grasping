#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

  
def publish():

    pub = rospy.Publisher('endEffectorPose', PoseStamped, queue_size=5)
    
    rospy.init_node('pose_transform', anonymous=True)

    # topics
    # rospy.Subscriber('chatter', PoseStamped, callback)
    
    rate = rospy.Rate(10) # 10hz
    
    
    while not rospy.is_shutdown():
        
        endEffectorPose = get_pose_stamped()
        rospy.loginfo(endEffectorPose)
        
        # rospy.sleep(1)
        pub.publish(endEffectorPose)
        rate.sleep()

def get_pose_stamped():
    endEffectorPose = PoseStamped()
    endEffectorPose.header.frame_id = rospy.get_param('/planning_frame')
    endEffectorPose.header.stamp = rospy.Time.now()

        
    endEffectorPose.pose.orientation.x = -0.310
    endEffectorPose.pose.orientation.y = 0.000
    endEffectorPose.pose.orientation.z = 0.001
    endEffectorPose.pose.orientation.w = 0.951
    endEffectorPose.pose.position.x = -0.014
    endEffectorPose.pose.position.y = 0.262
    endEffectorPose.pose.position.z = 1.127

    return endEffectorPose

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass