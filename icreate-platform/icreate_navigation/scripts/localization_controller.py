#!/usr/bin/env python

import roslaunch
import rospy

from icreate_navigation.srv import RunGmappingService
from miltbot_map.srv import SetMap

ROOT_PATH = "/home/saxonia/miltbot_catkin_ws/src/ros-miltbot/icreate-platform/"
gmapping_file_path = ROOT_PATH + "icreate_lift_navigation/launch/modules/gmapping.launch"
# gmapping_file_path = ROOT_PATH + "icreate_lift_navigation/launch/modules/dynamic_navigation_gmapping.launch"
amcl_file_path = ROOT_PATH + "icreate_navigation/launch/modules/amcl.launch"
# amcl_file_path = ROOT_PATH + "icreate_navigation/launch/modules/icreate_navigation_stack.launch"
set_map_service_name_ = ""
gmapping_launch = None
amcl_launch = None
gmapping_flag = False
amcl_flag = False

def start_set_map():
    rospy.wait_for_service(set_map_service_name_, timeout=2)
    try:
        set_map = rospy.ServiceProxy(set_map_service_name_, SetMap)
        resp = set_map("Lift", -1)
        return resp.flag
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def gmapping_start():
    "Return the pathname of the KOS root directory."
    global gmapping_flag
    gmapping_flag = True

    #Set Parameter Server
    rospy.set_param('/move_base/NavfnROS/allow_unknown',True)
    rospy.set_param('/move_base/NavfnROS/default_tolerance',0.2)
    rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.8)
    rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',0.5)
    rospy.set_param('/move_base/DWAPlannerROS/acc_lim_x',1.2)
    rospy.set_param('/velocity_smoother/accel_lim_v',0.4)
    rospy.set_param('/move_base/global_costmap/global_frame',"odom")
    rospy.set_param('/move_base/global_costmap/inflation_layer/inflation_radius',0.1)
    rospy.set_param('/move_base/local_costmap/inflation_layer/inflation_radius',0.1)
    

def amcl_start():
    global amcl_flag
    amcl_flag = True

    #Set Parameter Server
    rospy.set_param('/move_base/NavfnROS/allow_unknown',False)
    rospy.set_param('/move_base/NavfnROS/default_tolerance',0.0)
    rospy.set_param('/move_base/DWAPlannerROS/max_vel_x',0.3)
    rospy.set_param('/move_base/DWAPlannerROS/min_vel_x',0.1)
    rospy.set_param('/move_base/DWAPlannerROS/acc_lim_x',1.0)
    rospy.set_param('/velocity_smoother/accel_lim_v',0.2)
    rospy.set_param('/move_base/global_costmap/global_frame',"map")
    rospy.set_param('/move_base/global_costmap/inflation_layer/inflation_radius',0.3)
    rospy.set_param('/move_base/local_costmap/inflation_layer/inflation_radius',0.3)


def gmapping_shutdown():
    "Return the pathname of the KOS root directory."
    gmapping_launch.shutdown()

def amcl_shutdown():
    amcl_launch.shutdown()

def gmapping_callback(msg):
    "Return the pathname of the KOS root directory."
    task = msg.task
    print(msg.task)
    if task == 'open':
        start_set_map()
        gmapping_start()
        amcl_shutdown()
        return True
    elif task == 'close':
        gmapping_shutdown()
        amcl_start()
        return True
    elif task == 'restart':
        gmapping_shutdown()
        gmapping_start()
        return True
    else:
        print("Wrong Task Request")
        return False

if __name__ == '__main__':
    rospy.init_node('dynamic_map_launch', anonymous=True)
    set_map_service_name_ = "set_map_service"
    rospy.Service('run_gmapping', RunGmappingService, gmapping_callback)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    amcl_launch = roslaunch.parent.ROSLaunchParent(uuid, [amcl_file_path])
    amcl_launch.start()
    gmapping_flag = False
    amcl_flag = False 
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        r.sleep()
        if gmapping_flag:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            gmapping_launch = roslaunch.parent.ROSLaunchParent(uuid, [gmapping_file_path])
            gmapping_launch.start()
            gmapping_flag = False
        elif amcl_flag:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            amcl_launch = roslaunch.parent.ROSLaunchParent(uuid, [amcl_file_path])
            amcl_launch.start()
            amcl_flag = False