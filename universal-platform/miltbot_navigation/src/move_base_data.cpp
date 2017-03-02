#include "miltbot_navigation/move_base_data.h"

namespace miltbot {

MoveBaseGoalData::MoveBaseGoalData(void) {

}

MoveBaseGoalData::MoveBaseGoalData(std::string goal_name, move_base_msgs::MoveBaseGoal goal, 
                                    std::string building, std::string building_floor) {
    this->goal_name = goal_name;
    this->goal = goal;
    this->building = building;
    this->building_floor = building_floor;
    this->task = "";
}

MoveBaseGoalData::MoveBaseGoalData(std::string goal_name, move_base_msgs::MoveBaseGoal goal, 
                                    std::string building, std::string building_floor, std::string task) {
    this->goal_name = goal_name;
    this->goal = goal;
    this->building = building;
    this->building_floor = building_floor;
    this->task = task;
}

MoveBaseGoalData::~MoveBaseGoalData(void) {}

void MoveBaseGoalData::setGoal(move_base_msgs::MoveBaseGoal &goal) {
    this->goal = goal;
}

move_base_msgs::MoveBaseGoal MoveBaseGoalData::getGoal() {
    return this->goal;
}

void MoveBaseGoalData::setGoalName(std::string goal_name) {
    this->goal_name = goal_name;
}

std::string MoveBaseGoalData::getGoalName() {
    return this->goal_name;
}

void MoveBaseGoalData::setBuilding(std::string building) {
    this->building = building;
}

std::string MoveBaseGoalData::getBuilding() {
    return this->building;
}

void MoveBaseGoalData::setBuildingFloor(std::string building_floor) {
    this->building_floor = building_floor;
}

std::string MoveBaseGoalData::getBuildingFloor() {
    return this->building_floor;
}

void MoveBaseGoalData::setTask(std::string task) {
    this->task = task;
}

std::string MoveBaseGoalData::getTask() {
    return this->task;
}

void MoveBaseGoalData::setPriority(int priority) {
    this->priority = priority;
}

int MoveBaseGoalData::getPriority() {
    return this->priority;
}

}