#!/usr/bin/env python

import roslaunch
import rospy

from icreate_lift_navigation.srv import RunGmappingService
from miltbot_map.srv import SetMapServer

ROOT_PATH = "/home/sax/miltbot_catkin_ws/src/ros-miltbot/icreate-platform/"
file_path = ROOT_PATH + "icreate_lift_navigation/launch/modules/gmapping.launch"
set_map_service_name_ = ""
launch = None
flag = False

def start_set_map():
    print("X" + set_map_service_name_)
    rospy.wait_for_service(set_map_service_name_)
    print("X2")
    try:
        set_map = rospy.ServiceProxy(set_map_service_name_, SetMapServer)
        print("X3")
        resp = set_map("Lift")
        return resp.flag
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def start_gmapping():
    "Return the pathname of the KOS root directory."
    global flag
    flag = True

def shutdown():
    "Return the pathname of the KOS root directory."
    launch.shutdown()

def gmapping_callback(msg):
    "Return the pathname of the KOS root directory."
    task = msg.task
    if task == 'open':
        print(msg.task)
        start_set_map()
        start_gmapping()
        return True
    elif task == 'close':
        shutdown()
        return True
    else:
        print("Wrong Task Request")
        return False

if __name__ == '__main__':
    rospy.init_node('dynamic_map_launch', anonymous=True)
    rospy.on_shutdown(shutdown)
    set_map_service_name_ = "set_map"
    rospy.Service('run_gmapping', RunGmappingService, gmapping_callback)
    flag = False 
    while not rospy.is_shutdown():
        if flag:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, [file_path])
            launch.start()
            flag = False