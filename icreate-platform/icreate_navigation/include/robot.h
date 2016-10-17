#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf/transform_listener.h>
 
namespace icreate {

class Robot {
    public:
        Robot(void);

        ~Robot(void);

        bool setCurrentPosition();

        move_base_msgs::MoveBaseGoal getCurrentPosition();

        bool setEndPosition(move_base_msgs::MoveBaseGoal goal);

    private:

    public:
        move_base_msgs::MoveBaseGoal    startPosition;
        move_base_msgs::MoveBaseGoal    endPosition;
        move_base_msgs::MoveBaseGoal    currentPosition;
};

}