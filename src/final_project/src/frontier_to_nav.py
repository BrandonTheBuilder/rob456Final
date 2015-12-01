#!/usr/bin/env python

# import relevant libraries
import roslib
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from hector_nav_msgs.srv._GetRobotTrajectory import GetRobotTrajectory
from move_base_msgs.msg import MoveBaseActionResult
from nav_msgs.msg import Odometry

# The velocity command message
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
NAV_TIMEOUT = 100


class FrontierToNav:
  def __init__(self):
    # Keep track of the waypoints that actually work.
    self.actionablePath = [] 
    # Keep track of waypoints that cause the robot to get stuck.
    self.failedPath = []
    self.cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1).publish
    self.publishWaypoint = rospy.Publisher('/base_link_goal', Twist, queue_size=1).publish
    # self.subscribeToMoveBase = rospy.Subscriber('/move_base/status' , 
    #                                               GoalStatusArray, 
    #                                               self.callback)
    self._getPath = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
    self.run()


  
  def publishGoal(self):
    rospy.loginfo('[frontier_to_nav] Publishing Goal')
    self.currentGoal = self.path.pop()
    self.publishWaypoint(self.currentGoal)


  def getPath(self):
    request = GetRobotTrajectory._request_class()
    path = self._getPath(request)
    self.path = self._pathToWaypoints(path.trajectory.poses)
    self.path.reverse()


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
      if waypoint not in self.failedPath:
        path.append(waypoint)
    return path

  def reset(self):
    self.cancel()
    self.getPath()
    self.publishGoal()


  def retreat(self):
    self.cancel()
    if len(self.actionablePath)>0:
      self.publishWaypoint(self.actionablePath.pop())
    else:
      self.publishWaypoint(self.start)


  def run(self):
    
    startOdom = rospy.wait_for_message("/odom", Odometry)
    self.start = startOdom.twist.twist
    self.getPath()
    self.publishGoal()
    while True:
      try:
        status = rospy.wait_for_message("/move_base/result", MoveBaseActionResult, timeout=100)
        if status.status.status == 3:
          self.actionablePath.append(self.currentGoal)
          if len(self.path) > 0:
            self.publishGoal()
          else:
            rospy.loginfo('Path is empty, reseting...')
            self.reset()
        elif status.status.status == 4:
          rospy.loginfo('Cannot reach goal, robot is retreating')
          self.failedPath.append(self.currentGoal)
          self.retreat()
        else:
          rospy.loginfo('Move Base returned status {}, {}'.format(status.status.status
                                                                  ,status.status.text))
          self.failedPath.append(self.currentGoal)
          self.reset()
      except:
        rospy.loginfo('Reseting frontier_to_nav MoveBaseActionResult timed out')
        self.failedPath.append(self.currentGoal)
        self.reset()
      

  def callback(self, status):
    if hasattr(self, 'path'):
      if len(status.status_list) > 0:
        stat = status.status_list.pop()
        time = status.header.stamp.secs - stat.goal_id.stamp.secs
        if stat.status == 3:
          rospy.loginfo('Goal Reached')
          if len(self.path) > 0:
            self.publishGoal()
          else:
            self.waiting = True
            self.getPath()
            self.publishGoal()
            self.waiting = False
     
        elif stat.status == 1:
          rospy.loginfo('moving')
          
        else:
          rospy.loginfo('Reseting nav goals')
          self.getPath()
          self.publishGoal()

      else:
        pass
    

if __name__ == "__main__":
  # Initialise the mode
  rospy.init_node('frontier_to_nav')
  rospy.wait_for_service('get_exploration_path')
  ftv = FrontierToNav()
  rospy.spin()