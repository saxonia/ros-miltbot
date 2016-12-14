#!/usr/bin/env python

import rospy
from miltbot_map.srv import GetWaypoint
from miltbot_map.msg import Waypoint
from move_base_msgs.msg import MoveBaseGoal

def load_waypoint_file():
    print("load_waypoint_file")
    

def handle_get_waypoint(req):
    load_waypoint_file()
    res = Waypoint()
    res.goal = MoveBaseGoal()
    res.building = "4"
    res.floor = "20"
    return res


def get_waypoint_server():
    rospy.init_node('get_waypoint_server')
    s = rospy.Service('get_waypoint', GetWaypoint, handle_get_waypoint)
    rospy.spin()

if __name__ == "__main__" :
    get_waypoint_server()