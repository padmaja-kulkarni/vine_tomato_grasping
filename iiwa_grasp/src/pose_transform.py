#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

  
def publish():

    pub = rospy.Publisher('endEffectorPose', PoseStamped, queue_size=5, latch=True)
    
    rospy.init_node('pose_transform', anonymous=True)
    rospy.sleep(10)
    while True:
        
    
        # publish first pose        
        endEffectorPose = get_pose_stamped()
        rospy.loginfo(endEffectorPose) # 'Published pose: ', 
        pub.publish(endEffectorPose)
        
        # wait for 20 seconds
        rospy.sleep(20)
        
        # publish second pose
        endEffectorPose = get_pose_stamped(posX = 0.1)
        # rospy.loginfo(endEffectorPose)
        pub.publish(endEffectorPose)
        rospy.sleep(5)
    

def get_pose_stamped(rotX = -0.310, rotY = 0.000, rotZ = 0.001, rotW = 0.951, posX = -0.014, posY = 0.262, posZ = 1.127):
    endEffectorPose = PoseStamped()
    
    endEffectorPose.header.frame_id = rospy.get_param('planning_frame')
    endEffectorPose.header.stamp = rospy.Time.now()

        
    endEffectorPose.pose.orientation.x = rotX
    endEffectorPose.pose.orientation.y = rotY
    endEffectorPose.pose.orientation.z = rotZ
    endEffectorPose.pose.orientation.w = rotW
    endEffectorPose.pose.position.x = posX
    endEffectorPose.pose.position.y = posY
    endEffectorPose.pose.position.z = posZ

    return endEffectorPose

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
