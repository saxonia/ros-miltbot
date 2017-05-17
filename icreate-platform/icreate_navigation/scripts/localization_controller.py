#!/usr/bin/env python

import roslaunch
import rospy
import rospkg
import dynamic_reconfigure.client

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
    rospy.set_param('slam_gmapping/odom_frame', namespace + '/odom')
    rospy.set_param('slam_gmapping/base_frame', namespace + '/base_footprint')
    rospy.set_param('slam_gmapping/map_frame', namespace + '/map')
    # rospy.set_param('move_base/DWAPlannerROS/max_vel_x', 0.7)
    # rospy.set_param('move_base/DWAPlannerROS/min_vel_x', 0.4)
    # rospy.set_param('move_base/DWAPlannerROS/max_trans_vel', 1.0)
    # rospy.set_param('move_base/DWAPlannerROS/min_trans_vel', 0.1)
    # # rospy.set_param('move_base/DWAPlannerROS/acc_lim_x',1.5)
    rospy.set_param('move_base/global_costmap/global_frame', namespace + "/odom")
    # rospy.set_param('move_base/global_costmap/inflation_layer/inflation_radius', 0.1)
    # rospy.set_param('move_base/local_costmap/inflation_layer/inflation_radius', 0.1)
    rospy.set_param('move_base/NavfnROS/allow_unknown', True)
    rospy.set_param('move_base/NavfnROS/default_tolerance', 0.2)
    # rospy.set_param('velocity_smoother/accel_lim_v', 0.5)
    # rospy.set_param('velocity_smoother/speed_lim_v', 1.0)

    # #Set Dynamic Reconfigure for Slam Gmapping
    # client = dynamic_reconfigure.client.Client("slam_gmapping")
    # params = {'odom_frame' : namespace + '/odom',
    #           'base_frame' : namespace + '/base_footprint',
    #           'map_frame' : namespace + '/map',
    #          }
    # config = client.update_configuration(params)

    #Set Dynamic Reconfigure for Move Base
    client = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS")
    params = {'max_vel_x' : 0.7,
            #   'min_vel_x' : 0.4,
              'min_vel_x' : 0.15,
              'max_trans_vel' : 1.0,
              'min_trans_vel' : 0.1
             }
    config = client.update_configuration(params)

    client = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_layer")
    params = {#'global_frame' : namespace + "/map",
              'inflation_radius' : 0.1
             }
    config = client.update_configuration(params)

    client = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer")
    params = {'inflation_radius' : 0.1
             }
    config = client.update_configuration(params)

    # client = dynamic_reconfigure.client.Client("move_base/NavfnROS")
    # params = {'allow_unknown' : True,
    #           'default_tolerance' : 0.2
    #          }
    # config = client.update_configuration(params)

    #Set Dynamic Reconfigure for Velocity Smoother
    client = dynamic_reconfigure.client.Client("velocity_smoother")
    params = {'accel_lim_v' : 0.5,
              'speed_lim_v' : 1.0
             }
    config = client.update_configuration(params)
    

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
    # rospy.set_param('move_base/DWAPlannerROS/max_vel_x', 0.5)
    # rospy.set_param('move_base/DWAPlannerROS/min_vel_x', 0.15)
    # rospy.set_param('move_base/DWAPlannerROS/max_trans_vel', 0.4)
    # rospy.set_param('move_base/DWAPlannerROS/min_trans_vel', 0.1)
    # rospy.set_param('move_base/DWAPlannerROS/acc_lim_x',1.4)
    rospy.set_param('move_base/global_costmap/global_frame', namespace + "/map")
    # rospy.set_param('move_base/global_costmap/inflation_layer/inflation_radius', 0.3)
    # rospy.set_param('move_base/local_costmap/inflation_layer/inflation_radius', 0.3)
    rospy.set_param('move_base/NavfnROS/allow_unknown', False)
    rospy.set_param('move_base/NavfnROS/default_tolerance', 0.0)
    # rospy.set_param('velocity_smoother/accel_lim_v', 0.2)
    # rospy.set_param('velocity_smoother/speed_lim_v', 0.3)

    #Set Dynamic Reconfigure for AMCL
    # client = dynamic_reconfigure.client.Client("amcl")
    # params = {'odom_frame_id' : namespace + '/odom',
    #           'base_frame_id' : namespace + '/base_footprint',
    #           'global_frame_id' : namespace + '/map'
    #          }
    # config = client.update_configuration(params)

    #Set Dynamic Reconfigure for Move Base
    client = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS")
    params = {'max_vel_x' : 0.5,
              'min_vel_x' : 0.15,
              'max_trans_vel' : 0.4,
              'min_trans_vel' : 0.1
             }
    config = client.update_configuration(params)

    client = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_layer")
    params = {#'global_frame' : namespace + "/map",
              'inflation_radius' : 0.3
             }
    config = client.update_configuration(params)

    client = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer")
    params = {'inflation_radius' : 0.3
             }
    config = client.update_configuration(params)

    # client = dynamic_reconfigure.client.Client("move_base/NavfnROS")
    # params = {'allow_unknown' : False,
    #           'default_tolerance' : 0.0
    #          }
    # config = client.update_configuration(params)

    #Set Dynamic Reconfigure for Velocity Smoother
    client = dynamic_reconfigure.client.Client("velocity_smoother")
    params = {'accel_lim_v' : 0.2,
              'speed_lim_v' : 0.3
             }
    config = client.update_configuration(params)

    



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
        