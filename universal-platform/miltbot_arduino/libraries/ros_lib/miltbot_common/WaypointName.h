#ifndef _ROS_miltbot_common_WaypointName_h
#define _ROS_miltbot_common_WaypointName_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace miltbot_common
{

  class WaypointName : public ros::Msg
  {
    public:
      uint32_t waypoint_name_length;
      typedef char* _waypoint_name_type;
      _waypoint_name_type st_waypoint_name;
      _waypoint_name_type * waypoint_name;

    WaypointName():
      waypoint_name_length(0), waypoint_name(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->waypoint_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->waypoint_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->waypoint_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->waypoint_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->waypoint_name_length);
      for( uint32_t i = 0; i < waypoint_name_length; i++){
      uint32_t length_waypoint_namei = strlen(this->waypoint_name[i]);
      varToArr(outbuffer + offset, length_waypoint_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->waypoint_name[i], length_waypoint_namei);
      offset += length_waypoint_namei;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t waypoint_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      waypoint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      waypoint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      waypoint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->waypoint_name_length);
      if(waypoint_name_lengthT > waypoint_name_length)
        this->waypoint_name = (char**)realloc(this->waypoint_name, waypoint_name_lengthT * sizeof(char*));
      waypoint_name_length = waypoint_name_lengthT;
      for( uint32_t i = 0; i < waypoint_name_length; i++){
      uint32_t length_st_waypoint_name;
      arrToVar(length_st_waypoint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_waypoint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_waypoint_name-1]=0;
      this->st_waypoint_name = (char *)(inbuffer + offset-1);
      offset += length_st_waypoint_name;
        memcpy( &(this->waypoint_name[i]), &(this->st_waypoint_name), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "miltbot_common/WaypointName"; };
    const char * getMD5(){ return "0f055aabc4b1d0b96ca6d1ef78863490"; };

  };

}
#endif