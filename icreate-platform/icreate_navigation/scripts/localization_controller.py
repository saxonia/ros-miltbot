#!/usr/bin/env python

import roslaunch
import rospy
import rospkg

from icreate_navigation.srv import RunGmappingService
from miltbot_map.srv import SetMap

rospack = rospkg.RosPack()
ROOT_PATH = rospack.get_path('icreate_navigation')
# ROOT_PATH = "/~/miltbot_catkin_ws/src/ros-miltbot/icreate-platform/"
gmapping_file_path = ROOT_PATH + "/launch/modules/gmapping.launch"
amcl_file_path = ROOT_PATH + "/launch/modules/amcl.launch"
set_map_service_name_ = ""
gmapping_launch = None
amcl_launch = None
gmapping_flag = False
amcl_flag = False
namespace = ''

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
    global gmapping_launch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    gmapping_launch = roslaunch.parent.ROSLaunchParent(uuid, [gmapping_file_path])
    gmapping_launch.start()

    #Set Parameter Server
    rospy.set_param('slam_gmapping/odom_frame_id', namespace + '/odom')
    rospy.set_param('slam_gmapping/base_frame_id', namespace + '/base_footprint')
    rospy.set_param('slam_gmapping/map_frame_id', namespace + '/map')
    rospy.set_param('move_base/NavfnROS/allow_unknown',True)
    rospy.set_param('move_base/NavfnROS/default_tolerance',0.2)
    rospy.set_param('move_base/DWAPlannerROS/max_vel_x',0.8)
    rospy.set_param('move_base/DWAPlannerROS/min_vel_x',0.3)
    rospy.set_param('move_base/DWAPlannerROS/max_trans_vel',0.6)
    rospy.set_param('move_base/DWAPlannerROS/min_trans_vel',0.3)
    # rospy.set_param('move_base/DWAPlannerROS/acc_lim_x',1.5)
    rospy.set_param('velocity_smoother/accel_lim_v',0.4)
    rospy.set_param('velocity_smoother/speed_lim_v',0.6)
    rospy.set_param('move_base/global_costmap/global_frame',namespace + "/odom")
    rospy.set_param('move_base/global_costmap/inflation_layer/inflation_radius',0.1)
    rospy.set_param('move_base/local_costmap/inflation_layer/inflation_radius',0.1)

def amcl_start():
    global amcl_launch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    amcl_launch = roslaunch.parent.ROSLaunchParent(uuid, [amcl_file_path])
    amcl_launch.start()

    #Set Parameter Server
    rospy.set_param('amcl/odom_frame_id', namespace + '/odom')
    rospy.set_param('amcl/base_frame_id', namespace + '/base_footprint')
    rospy.set_param('amcl/global_frame_id', namespace + '/map')
    rospy.set_param('move_base/NavfnROS/allow_unknown',False)
    rospy.set_param('move_base/NavfnROS/default_tolerance',0.0)
    rospy.set_param('move_base/DWAPlannerROS/max_vel_x',0.5)
    rospy.set_param('move_base/DWAPlannerROS/min_vel_x',-0.15)
    rospy.set_param('move_base/DWAPlannerROS/max_trans_vel',0.4)
    rospy.set_param('move_base/DWAPlannerROS/min_trans_vel',0.1)
    # rospy.set_param('move_base/DWAPlannerROS/acc_lim_x',1.4)
    rospy.set_param('velocity_smoother/accel_lim_v',0.2)
    rospy.set_param('velocity_smoother/speed_lim_v',0.3)
    rospy.set_param('move_base/global_costmap/global_frame',namespace + "/map")
    rospy.set_param('move_base/global_costmap/inflation_layer/inflation_radius',0.3)
    rospy.set_param('move_base/local_costmap/inflation_layer/inflation_radius',0.3)


def gmapping_shutdown():
    "Return the pathname of the KOS root directory."
    global gmapping_launch
    gmapping_launch.shutdown()

def amcl_shutdown():
    global amcl_launch
    amcl_launch.shutdown()

def gmapping_callback(msg):
    "Return the pathname of the KOS root directory."
    task = msg.task
    print(msg.task)
    if task == 'open':
        gmapping_start()
        amcl_shutdown()
        start_set_map()
        return True
    elif task == 'close':
        amcl_start()
        gmapping_shutdown()
        return True
    elif task == 'restart':
        gmapping_shutdown()
        gmapping_start()
        return True
    else:
        print("Wrong Task Request")
        return False

if __name__ == '__main__':
    rospy.init_node('localization_controller', anonymous=True)
    namespace = rospy.get_param('localization_controller/namespace')
    print('AAAAAAAAAAAAAAAAAAAAAA ' + namespace)
    set_map_service_name_ = "set_map_service"
    rospy.Service('run_gmapping', RunGmappingService, gmapping_callback)
    # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    # roslaunch.configure_logging(uuid)
    # amcl_launch = roslaunch.parent.ROSLaunchParent(uuid, [amcl_file_path])
    # amcl_launch.start()
    amcl_start()
    rospy.spin()
        