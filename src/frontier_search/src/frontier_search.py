#!/usr/bin/env python

import rospy
import std_msgs.msg
import message_filters
import math

from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

#TODO Move all constants to config file
OBSTACLE_DISTANCE = 2 #How close is too close for a valid frontier
MIN_XY_GOAL_DIST = 10 #How far away does a goal have to be Set this lower for worlds with tight walls!
DIST_WEIGHT = 1 #THe weight distance from the robot has in calculating goal
FOCUS_WEIGHT = 1 #The weight the focus of frontiers has on calculating goal
CONTROLLER_TIMEOUT = 100 #How long to wait until we give up on a goal and try again

class FrontierSearch(object):
    """
    Initialize creates the subscriber to get the map and creates a publisher to 
    publish frontiers
    """
    def __init__(self):
        rospy.Subscriber("/map", OccupancyGrid, self.getMap, queue_size=1)
        self.publishFrontiers = rospy.Publisher('new_frontiers', PointCloud, queue_size=10)
        self.gmap = None
        #list of cells that are not frontiers
        self.notFrontier = []
        self.lookingForFrontiers = False


    """
    the map callback saves the map to a class variable
    TODO: I know there is a topic /map/updates it would probably be more efficient
    to use that but I am not sure how it works or what the messages are
    """
    def getMap(self, gmap):
        self.gmap = gmap
        self.mapHeight = gmap.info.height
        self.mapWidth = gmap.info.width
        self.mapResolution = gmap.info.resolution
        self.xOrigin = gmap.info.origin.position.x
        self.yOrigin = gmap.info.origin.position.y
        #TODO: save variables to work with maps that are not at 90 degrees


    """
    getFrontiers returns frontiers found on the map
    """
    

    def getFrontiers(self):
        if self.gmap is not None:
            self.lookingForFrontiers = True
            rospy.loginfo("[frontier_search] Looking for frontiers")
            #First check to see if the cells are free and if we already decided they are not a frontier
            cells = [x for x in range(0, len(self.gmap.data)) if self.gmap.data[x] == 0]
            # and x not in self.notFrontier removed from previous line waay too slow!
            #The check to see if the remaining are frontiers
            frontiers = [cell for cell in cells if -1 in [self.gmap.data[val] for val in self.getNeighbors(cell)]]
            #Add all cells that are not frontiers to our list of not frontiers waay too slow!
            #self.notFrontier.extend([cell for cell in cells if cell not in frontiers])
            #Check to see if the area around them is clear of obstacles
            freeFrontiers = [frontier for frontier in frontiers if self.validFrontier(frontier)]
            rospy.loginfo("[frontier_search] Found %s frontiers", len(freeFrontiers))
            points = self.getPoints(freeFrontiers)
            cloud = PointCloud()
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = 'map'
            cloud.header = h
            cloud.points = points
            rospy.loginfo("[frontier_search] Publishing frontiers")
            # Publish a point cloud for your viewing pleasure
            self.publishFrontiers.publish(cloud)
            self.lookingForFrontiers = False
            return cloud
        else:
            rospy.loginfo("[frontier_search] No map available")
            return None


    def getNeighbors(self, index):
        # List neighbors in the order [North East South West]
        neighbors = []
        #North
        if index > self.mapWidth:
            neighbors.append(index - self.mapWidth)
        else:
            neighbors.append(None)

        if (index+1)%self.mapWidth == 0:
            neighbors.append(None)
        else:
            neighbors.append(index+1)
        if index < ((self.mapWidth*self.mapWidth)-self.mapWidth):
            neighbors.append(index+self.mapWidth)
        else:
            index.append(None)
        if index%self.mapWidth == 0:
            neighbors.append(None)
        else:
            neighbors.append(index-1)
        return neighbors


    def validFrontier(self, index):
        grid = self.gmap.data
        obIndex = int(OBSTACLE_DISTANCE/self.mapResolution)
        if index - (obIndex*self.mapWidth) < 0:
            return False
        if index + (obIndex*self.mapWidth) > self.mapWidth*self.mapHeight:
            return False
        for row in range(-obIndex, obIndex):
            location = index + row*self.mapWidth
            for column in range(-obIndex, obIndex):
                if grid[location+column] > 0:
                    return False
        return True


    def getPoints(self, indices):
        points = []
        #Finding the point value assumes square map may need to do more math
        for index in indices:
            p = Point()
            xind = index%self.mapWidth
            p.x = (xind+1)*self.mapResolution + self.xOrigin
            yind = index/self.mapWidth
            p.y = (yind+1)*self.mapResolution + self.yOrigin
            p.z = 0 #This is only for 2D frontiers
            points.append(p)
        return points


class MotionController(object):
    def __init__(self):
        #Initialize the frontier search class
        self.enterprise = FrontierSearch()
        #Subscrive to odom and move base status
        sub2 = message_filters.Subscriber('odom', Odometry)
        sub3 = message_filters.Subscriber('/move_base/status', GoalStatusArray)
        ts = message_filters.TimeSynchronizer([sub2, sub3], 1)
        ts.registerCallback(self.callback)
        #set up publisher for sending waypoints
        self.publishWaypoint = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10).publish
        

    def callback(self, odom, status):
        rospy.loginfo('Checking status')
        if len(status.status_list) > 0:
            rospy.loginfo('status at time: %s', status.header.stamp.secs)
            status = status.status_list.pop()
            time = rospy.get_time()
            moving =  status.status == 1 or status.status == 2 
            setGoal = not moving or time-status.goal_id.stamp.secs>CONTROLLER_TIMEOUT
        else:
            rospy.loginfo('No Status')
            setGoal = True
        if setGoal and not self.enterprise.lookingForFrontiers:
            points = self.enterprise.getFrontiers()
            if points is None:
                while points is None:
                    rospy.loginfo("[frontier_search] No frontiers found yet waiting")
                    rospy.sleep(10)
                    points = self.enterprise.getFrontiers()

            rospy.loginfo('Setting Waypoint')
            x = odom.pose.pose.position.x
            y = odom.pose.pose.position.y
            xLocus = sum([point.x for point in points.points])/len(points.points)
            yLocus = sum([point.y for point in points.points])/len(points.points)
            goals = [self.getDistance(point, (x,y), (xLocus, yLocus)) for point in points.points]
            goals.sort(key=lambda x: x[1])
            goal = PoseStamped()
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = 'map'
            goal.header = h
            # Set angle of goal to be lookng at the frontier locus
            angleToFocus = math.atan((yLocus-goals[0][0].y)/(xLocus-goals[0][0].x))
            quaternion = quaternion_from_euler(0,0,angleToFocus)
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.w = quaternion[2]
            goal.pose.orientation.w = quaternion[3]
            goal.pose.position.x = goals[0][0].x
            goal.pose.position.y = goals[0][0].y
            self.publishWaypoint(goal)
        else:
            rospy.loginfo("[frontier_search controller] Exploring")


    def getDistance(self, point, location, locus):
        #The first heuristic is the manhattan distance to the robot
        distance = abs(location[0]-point.x)+abs(location[1]-point.y)
        if distance < MIN_XY_GOAL_DIST:
            #If the point is closer than the minimum distance set the distance to a 
            # ridiculously high price
            distance = 10000000
        #The second heuristic is the manhattan distance to the average of frontiers
        frontiers = abs(locus[0]-point.x)+abs(locus[1]-point.y)
        cost = distance*DIST_WEIGHT + frontiers*FOCUS_WEIGHT
        return(point, cost)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('frontier_search')
    controller = MotionController()
    rospy.spin()
    
