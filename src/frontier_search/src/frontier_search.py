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

MAP_HEIGHT = 4000
MAP_WIDTH = 4000
MAP_RESOLUTION = 0.05
X_ORIGIN = -100
Y_ORIGIN = -100
OBSTACLE_DISTANCE = 2
MIN_XY_GOAL_DIST = 1
DIST_WEIGHT = 1 #THe weight distance from the robot has in calculating goal
FOCUS_WEIGHT = 0 #The weight the focus of frontiers has on calculating goal

class FrontierSearch(object):
    def __init__(self):
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
        self.publishFrontiers = rospy.Publisher('new_frontiers', PointCloud, queue_size=10)


    def mapCallback(self, gmap):
        rospy.loginfo("[frontier_search] Looking for frontiers")
        cells = [x for x in range(0, len(gmap.data)) if gmap.data[x] == 0]
        frontiers = [cell for cell in cells if -1 in [gmap.data[val] for val in self.getNeighbors(cell)]]
        freeFrontiers = [frontier for frontier in frontiers if self.validFrontier(frontier, gmap.data)]
        rospy.loginfo("[frontier_search] Found %s frontiers", len(freeFrontiers))
        points = self.getPoints(freeFrontiers)
        cloud = PointCloud()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'map'
        cloud.header = h
        cloud.points = points
        rospy.loginfo("[frontier_search] Publishing frontiers")
        self.publishFrontiers.publish(cloud)


    def getNeighbors(self, index):
        # List neighbors in the order [North East South West]
        neighbors = []
        if index > MAP_WIDTH:
            neighbors.append(index - MAP_WIDTH)
        else:
            neighbors.append(None)
        if (index+1)%MAP_WIDTH == 0:
            neighbors.append(None)
        else:
            neighbors.append(index+1)
        if index < ((MAP_WIDTH*MAP_WIDTH)-MAP_WIDTH):
            neighbors.append(index+MAP_WIDTH)
        else:
            index.append(None)
        if index%MAP_WIDTH == 0:
            neighbors.append(None)
        else:
            neighbors.append(index-1)
        return neighbors


    def validFrontier(self, index, grid):
        obIndex = int(OBSTACLE_DISTANCE/MAP_RESOLUTION)
        if index - (obIndex*MAP_WIDTH) < 0:
            return False
        if index + (obIndex*MAP_WIDTH) > MAP_WIDTH*MAP_HEIGHT:
            return False
        for row in range(-obIndex, obIndex):
            location = index + row*MAP_WIDTH
            for column in range(-obIndex, obIndex):
                if grid[location+column] > 0:
                    return False
        return True


    def getPoints(self, indices):
        points = []
        #Finding the point value assumes square map may need to do more math
        for index in indices:
            p = Point()
            xind = index%MAP_WIDTH
            p.x = (xind+1)*MAP_RESOLUTION + X_ORIGIN
            yind = index/MAP_WIDTH
            p.y = (yind+1)*MAP_RESOLUTION + Y_ORIGIN
            p.z = 0 #This is only for 2D frontiers
            points.append(p)
        return points


class MotionController(object):
    def __init__(self):
        sub = message_filters.Subscriber("new_frontiers", PointCloud)
        sub2 = message_filters.Subscriber('odom', Odometry)
        ts = message_filters.TimeSynchronizer([sub, sub2], 10)
        ts.registerCallback(self.callback)
        self.publishWaypoint = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10).publish
        

    def callback(self, points, odom):
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
        # I am not sure how to find the best angle to look so I am defaulting to up
        angleToFocus = math.atan((yLocus-goals[0][0].y)/(xLocus-goals[0][0].x))
        quaternion = quaternion_from_euler(0,0,angleToFocus)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.w = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        goal.pose.position.x = goals[0][0].x
        goal.pose.position.y = goals[0][0].y
        self.publishWaypoint(goal)


    def getDistance(self, point, location, locus):
        #The first heuristic is the manhattan distance to the robot
        distance = abs(location[0]-point.x)+abs(location[1]-point.y)
        if distance < MIN_XY_GOAL_DIST:
            #If the point is closer than the minimum distance set the distance to a 
            # ridiculously high price
            distance = (MAP_WIDTH+MAP_HEIGHT)*MAP_RESOLUTION
        #The second heuristic is the manhattan distance to the average of frontiers
        frontiers = abs(locus[0]-point.x)+abs(locus[1]-point.y)
        cost = distance*DIST_WEIGHT + frontiers*FOCUS_WEIGHT
        return(point, cost)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('frontier_search')
    enterprise = FrontierSearch()
    controller = MotionController()
    rospy.spin()
    
