#!/usr/bin/env python

import roslaunch
import rospy

def shutdown() :
    launch.shutdown()

rospy.init_node('roslaunch_test', anonymous=True)
rospy.on_shutdown(shutdown)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
# launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/sax/miltbot_catkin_ws/src/ros-miltbot/icreate-platform/icreate_lift_navigation/launch/dynamic_navigation_gmapping.launch"])
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/sax/miltbot_catkin_ws/src/ros-miltbot/icreate-platform/icreate_lift_navigation/launch/modules/gmapping.launch"])

launch.start()

launch.spin()

def gmapping_callback(msg):
    task = msg.task
    if task == 'open' :
        print()
    elif task == 'close' :
        print()
    else:
        print()


if __name__ == '__main__' :
    rospy.init_node('dynamic_map_launch')

    rospy.Service(,RunGmappingService,gmapping_callback)
    rospy.spin()
    


