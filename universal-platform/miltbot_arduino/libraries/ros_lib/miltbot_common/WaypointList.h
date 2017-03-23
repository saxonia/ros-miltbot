#ifndef _ROS_miltbot_common_WaypointList_h
#define _ROS_miltbot_common_WaypointList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "miltbot_common/Waypoint.h"

namespace miltbot_common
{

  class WaypointList : public ros::Msg
  {
    public:
      uint32_t waypoints_length;
      typedef miltbot_common::Waypoint _waypoints_type;
      _waypoints_type st_waypoints;
      _waypoints_type * waypoints;

    WaypointList():
      waypoints_length(0), waypoints(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->waypoints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->waypoints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->waypoints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->waypoints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->waypoints_length);
      for( uint32_t i = 0; i < waypoints_length; i++){
      offset += this->waypoints[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t waypoints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      waypoints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->waypoints_length);
      if(waypoints_lengthT > waypoints_length)
        this->waypoints = (miltbot_common::Waypoint*)realloc(this->waypoints, waypoints_lengthT * sizeof(miltbot_common::Waypoint));
      waypoints_length = waypoints_lengthT;
      for( uint32_t i = 0; i < waypoints_length; i++){
      offset += this->st_waypoints.deserialize(inbuffer + offset);
        memcpy( &(this->waypoints[i]), &(this->st_waypoints), sizeof(miltbot_common::Waypoint));
      }
     return offset;
    }

    const char * getType(){ return "miltbot_common/WaypointList"; };
    const char * getMD5(){ return "3c2936debb785d9bd668ee7fc284db7e"; };

  };

}
#endif