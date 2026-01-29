#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def publish_3d_goal():
    rospy.init_node('goal_publisher')
    pub = rospy.Publisher('/planner/goal_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(1)
    
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 5.0
    goal.pose.position.y = 7.0
    goal.pose.position.z = 3.0  # 3D坐标
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    
    while not rospy.is_shutdown():
        goal.header.stamp = rospy.Time.now()
        pub.publish(goal)
        rospy.loginfo("Published 3D goal: (%.1f, %.1f, %.1f)", 
                     goal.pose.position.x, 
                     goal.pose.position.y, 
                     goal.pose.position.z)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_3d_goal()
    except rospy.ROSInterruptException:
        pass
