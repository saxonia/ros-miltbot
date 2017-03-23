#ifndef _ROS_SERVICE_GetWaypoint_h
#define _ROS_SERVICE_GetWaypoint_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "miltbot_common/Waypoint.h"

namespace miltbot_map
{

static const char GETWAYPOINT[] = "miltbot_map/GetWaypoint";

  class GetWaypointRequest : public ros::Msg
  {
    public:
      typedef const char* _waypoint_type;
      _waypoint_type waypoint;

    GetWaypointRequest():
      waypoint("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_waypoint = strlen(this->waypoint);
      varToArr(outbuffer + offset, length_waypoint);
      offset += 4;
      memcpy(outbuffer + offset, this->waypoint, length_waypoint);
      offset += length_waypoint;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_waypoint;
      arrToVar(length_waypoint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_waypoint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_waypoint-1]=0;
      this->waypoint = (char *)(inbuffer + offset-1);
      offset += length_waypoint;
     return offset;
    }

    const char * getType(){ return GETWAYPOINT; };
    const char * getMD5(){ return "8402364dfd05925d55e65de7c231a373"; };

  };

  class GetWaypointResponse : public ros::Msg
  {
    public:
      typedef miltbot_common::Waypoint _data_type;
      _data_type data;

    GetWaypointResponse():
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETWAYPOINT; };
    const char * getMD5(){ return "6e32ed6224155b9fe6e09efae156b561"; };

  };

  class GetWaypoint {
    public:
    typedef GetWaypointRequest Request;
    typedef GetWaypointResponse Response;
  };

}
#endif
