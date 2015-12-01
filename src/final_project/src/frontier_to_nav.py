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
from sensor_msgs.msg import LaserScan

# The velocity command message
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
import math
NAV_TIMEOUT = 30
OBSTACLE_RANGE = 2
ANGLE_LIMIT = 0.5
ESCAPE_SPEED = 1
ESCAPE_DISTANCE = 5
TURN_SPEED = 1
PATH_INDEX = 5 #Take every PATH_INDEX item from the path list


class FrontierToNav:
  def __init__(self):
    # Keep track of the waypoints that actually work.
    self.actionablePath = [] 
    # Keep track of waypoints that cause the robot to get stuck.
    self.failedPath = []
    self.cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1).publish
    self.publishLocalWaypoint = rospy.Publisher('/base_link_goal', Twist, queue_size=1).publish
    self.publishGlobalWaypoint = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10).publish
    self.publishVelocity = rospy.Publisher('cmd_vel',Twist, queue_size=5).publish
    # self.subscribeToMoveBase = rospy.Subscriber('/move_base/status' , 
    #                                               GoalStatusArray, 
    #                                               self.callback)
    self._getPath = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
    startOdom = rospy.wait_for_message("/odom", Odometry)
    self.start = startOdom.pose
    self.status = 0
    self.run()


  def run(self):
    self.getPath()
    self.publishGoal()
    while True:
      try:
        status = rospy.wait_for_message("/move_base/result", MoveBaseActionResult, timeout=NAV_TIMEOUT)
        if status.status.status == 3:
          self.status = 0
          self.actionablePath.append(self.currentGoal)
          if len(self.path) > 0:
            self.publishGoal()
          else:
            rospy.loginfo('[frontier_to_nav] Path is empty')
            self.getPath()
            self.publishGoal()
        elif status.status.status == 4:
          rospy.loginfo('[frontier_to_nav] Move Base returned status 4 no path found.')
          self.failedPath.append(self.currentGoal)
          self.somethingsWrong()
        elif status.status.status == 2:
          rospy.loginfo('[frontier_to_nav] Move Base interupted by new goal.')
        else:
          rospy.loginfo('Move Base returned status {}, {}'.format(status.status.status
                                                                  ,status.status.text))
          self.failedPath.append(self.currentGoal)
          self.somethingsWrong()
      except:
        rospy.loginfo('[frontier_to_nav] MoveBaseActionResult timed out')
        self.failedPath.append(self.currentGoal)
        self.somethingsWrong()


  def getPath(self):
    request = GetRobotTrajectory._request_class()
    response = self._getPath(request)
    path = response.trajectory.poses
    self.path = path[0::PATH_INDEX]
    if len(path) > 0:
      self.path.append(path[len(path)-1]) 
      self.path.reverse()


  def publishGoal(self):
    if len(self.path)>0:
      rospy.loginfo('[frontier_to_nav] Publishing Goal')
      self.currentGoal = self.path.pop()
      self.publishGlobalWaypoint(self.currentGoal)
      rospy.loginfo('[frontier_to_nav] Setting Goal: %s', self.currentGoal)
      # del self.path[:]
    else:
      rospy.logwarn('[frontier_to_nav] There is no path to follow')


  def somethingsWrong(self):
    if self.status == 0:
      rospy.logwarn('[frontier_to_nav] Trying a reset')
      self.status = 1
      self.reset()

    elif self.status == 1:
      rospy.logwarn('[frontier_to_nav] Reset failed, attempting to retreat')
      self.status = 2
      self.retreat()

    elif self.status == 2:
      rospy.logwarn('[frontier_to_nav] Retreat failed, attempting emergency escape')
      self.status = 3
      self.helpImStuck()

    elif self.status == 3:
      rospy.logerr('[frontier_to_nav] All contingency plans failed, manual intervention'
                    ' may be required')


  def reset(self):
    rospy.loginfo('[frontier_to_nav] Reseting path')
    self.cancel()
    self.getPath()
    self.publishGoal()


  def retreat(self):
    self.cancel()
    if len(self.actionablePath)>0:
      rospy.loginfo('[frontier_to_nav] Retreating to last waypoint')
      self.publishGlobalWaypoint(self.actionablePath.pop())
    else:
      rospy.loginfo('[frontier_to_nav] Nowhere to run')
      # self.publishGlobalWaypoint(self.start)


  def helpImStuck(self):
    self.cancel()
    rospy.logwarn('[frontier_to_nav] Trying to unstick robot')
    scan = rospy.wait_for_message("/base_scan", LaserScan)
    maxAngle = scan.angle_max
    minAngle = scan.angle_min
    distanceArray = scan.ranges
    numScans = len(distanceArray)
    angleIncrement = scan.angle_increment
    
    while min(distanceArray) < OBSTACLE_RANGE:
      escVel = Twist()
      mindex = distanceArray.index(min(distanceArray))
      minAngle = mindex*angleIncrement + minAngle
      while abs((abs(self.getYaw() - minAngle)-math.pi)) > ANGLE_LIMIT:
        escVel.angular.z = TURN_SPEED;
        rospy.loginfo('[frontier_to_nav] Turning to run!')
        self.publishVelocity(escVel)
      escVel.linear.x = ESCAPE_SPEED
      rospy.loginfo('[frontier_to_nav] Running Away!')
      self.publishVelocity(escVel)
      scan = rospy.wait_for_message("/base_scan", LaserScan)
      distanceArray = scan.ranges
    stop = Twist()
    self.publishVelocity(stop)
    forward = Twist()
    forward.linear.x += ESCAPE_DISTANCE
    self.publishLocalWaypoint(forward)


  def getYaw(self):
    odom = rospy.wait_for_message("/odom", Odometry)
    pose = odom.pose.pose
    o = pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return yaw
    

if __name__ == "__main__":
  # Initialise the mode
  rospy.init_node('frontier_to_nav')
  rospy.wait_for_service('get_exploration_path')
  ftv = FrontierToNav()
  rospy.spin()
  