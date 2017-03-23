#ifndef _ROS_miltbot_common_Waypoint_h
#define _ROS_miltbot_common_Waypoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "move_base_msgs/MoveBaseGoal.h"

namespace miltbot_common
{

  class Waypoint : public ros::Msg
  {
    public:
      typedef int64_t _id_type;
      _id_type id;
      typedef const char* _name_type;
      _name_type name;
      typedef const char* _building_type;
      _building_type building;
      typedef const char* _building_floor_type;
      _building_floor_type building_floor;
      typedef move_base_msgs::MoveBaseGoal _goal_type;
      _goal_type goal;
      typedef const char* _task_type;
      _task_type task;
      typedef int32_t _priority_type;
      _priority_type priority;

    Waypoint():
      id(0),
      name(""),
      building(""),
      building_floor(""),
      goal(),
      task(""),
      priority(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_id.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_id.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_id.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_id.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      uint32_t length_building = strlen(this->building);
      varToArr(outbuffer + offset, length_building);
      offset += 4;
      memcpy(outbuffer + offset, this->building, length_building);
      offset += length_building;
      uint32_t length_building_floor = strlen(this->building_floor);
      varToArr(outbuffer + offset, length_building_floor);
      offset += 4;
      memcpy(outbuffer + offset, this->building_floor, length_building_floor);
      offset += length_building_floor;
      offset += this->goal.serialize(outbuffer + offset);
      uint32_t length_task = strlen(this->task);
      varToArr(outbuffer + offset, length_task);
      offset += 4;
      memcpy(outbuffer + offset, this->task, length_task);
      offset += length_task;
      union {
        int32_t real;
        uint32_t base;
      } u_priority;
      u_priority.real = this->priority;
      *(outbuffer + offset + 0) = (u_priority.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_priority.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_priority.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_priority.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->priority);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int64_t real;
        uint64_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_id.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_id.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_id.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_id.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->id = u_id.real;
      offset += sizeof(this->id);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      uint32_t length_building;
      arrToVar(length_building, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_building; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_building-1]=0;
      this->building = (char *)(inbuffer + offset-1);
      offset += length_building;
      uint32_t length_building_floor;
      arrToVar(length_building_floor, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_building_floor; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_building_floor-1]=0;
      this->building_floor = (char *)(inbuffer + offset-1);
      offset += length_building_floor;
      offset += this->goal.deserialize(inbuffer + offset);
      uint32_t length_task;
      arrToVar(length_task, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_task; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_task-1]=0;
      this->task = (char *)(inbuffer + offset-1);
      offset += length_task;
      union {
        int32_t real;
        uint32_t base;
      } u_priority;
      u_priority.base = 0;
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_priority.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->priority = u_priority.real;
      offset += sizeof(this->priority);
     return offset;
    }

    const char * getType(){ return "miltbot_common/Waypoint"; };
    const char * getMD5(){ return "7e839900446e6306308bf4077399f6b0"; };

  };

}
#endif