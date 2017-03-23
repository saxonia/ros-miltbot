#ifndef _ROS_SERVICE_AddTarget_h
#define _ROS_SERVICE_AddTarget_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "miltbot_common/Waypoint.h"

namespace miltbot_system
{

static const char ADDTARGET[] = "miltbot_system/AddTarget";

  class AddTargetRequest : public ros::Msg
  {
    public:
      typedef miltbot_common::Waypoint _waypoint_type;
      _waypoint_type waypoint;

    AddTargetRequest():
      waypoint()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->waypoint.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->waypoint.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return ADDTARGET; };
    const char * getMD5(){ return "14d7d9b9ed590db599b5345f31a3665c"; };

  };

  class AddTargetResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    AddTargetResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return ADDTARGET; };
    const char * getMD5(){ return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class AddTarget {
    public:
    typedef AddTargetRequest Request;
    typedef AddTargetResponse Response;
  };

}
#endif
