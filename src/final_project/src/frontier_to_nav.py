#!/usr/bin/env python

# import relevant libraries
import roslib
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from hector_nav_msgs.srv._GetRobotTrajectory import GetRobotTrajectory

# The velocity command message
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray

class FrontierToNav:
  def __init__(self):
    self.publishWaypoint = rospy.Publisher('/base_link_goal', Twist, queue_size=10)
    self.subscribeToMoveBase = rospy.Subscriber('/move_base/status' , 
                                                  GoalStatusArray, 
                                                  self.callback)
    self._getPath = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
    
    self.getPath()
    self.publishGoal()

  
  def publishGoal(self):
    self.publishWaypoint.publish(self.path.pop())


  def getPath(self):
    request = GetRobotTrajectory._request_class()
    path = self._getPath(request)
    self.path = self._pathToWaypoints(path.trajectory.poses)


  def _pathToWaypoints(self, poses):
    path = []
    for goal in poses:
      waypoint = Twist()
      pose = goal.pose
      p = pose.position
      waypoint.linear.x = p.x
      waypoint.linear.y = p.y
      waypoint.linear.z = p.z
      o = pose.orientation
      (waypoint.angular.x, waypoint.angular.y, waypoint.angular.z)  = euler_from_quaternion([o.x, o.y, o.z, o.w])
      path.append(waypoint)
    return path


  def callback(self, status):
    if len(status.status_list) > 0:
      if status.status_list.pop().status == 3:
        if len(self.path) > 0:
          self.publishGoal()
        else:
          self.getPath()
      else:
        pass
    else:
      pass
    

if __name__ == "__main__":
  # Initialise the mode
  rospy.init_node('frontier_to_nav')
  rospy.wait_for_service('get_exploration_path')
  ftv = FrontierToNav()
  rospy.spin()