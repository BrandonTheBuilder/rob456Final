from hector_nav_msgs.srv._GetRobotTrajectory import GetRobotTrajectory
import rospy

if __name__ == '__main__':
    rospy.wait_for_service('get_exploration_path')
    getPath = rospy.ServiceProxy('get_exploration_path', GetRobotTrajectory)
    path = getPath(GetRobotTrajectory._request_class())
    import IPython; IPython.embed()
