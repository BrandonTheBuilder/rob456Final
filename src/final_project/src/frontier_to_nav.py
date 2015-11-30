#!/usr/bin/env python

# import relevant libraries
import roslib
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion

# The velocity command message
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

def exploration_callback(msg):
  for goal in msg.poses:
    waypoint = Twist()
    pose = goal.pose
    p = pose.position
    waypoint.linear.x = p.x
    waypoint.linear.y = p.y
    waypoint.linear.z = p.z
    o = pose.orientation
    (waypoint.angular.x, waypoint.angular.y, waypoint.angular.z)  = euler_from_quaternion([o.x, o.y, o.z, o.w])
    pub.publish(waypoint)




if __name__ == "__main__":
  # Initialise the mode
  rospy.init_node('frontier_to_nav')
  
  # Publish waypoint data to robot
  pub = rospy.Publisher('/base_link_goal',Twist,queue_size=10)
  sub = rospy.Subscriber('/exploration_path', Path, exploration_callback)
  
  # Turn control over to ROS
  rospy.spin()